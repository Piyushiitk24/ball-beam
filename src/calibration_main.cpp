/*
 * ================================================================
 *  Ball-and-Beam AS5600 Beam-Angle Calibration Firmware
 * ================================================================
 *  Platform  : Arduino Nano (ATmega328P, 16 MHz)
 *  Actuator  : NEMA17 stepper + TMC2209   STEP->D2  DIR->D3  EN->D4
 *  Encoder   : AS5600 on I2C              SDA->A4  SCL->A5  0x36
 *
 *  Purpose
 *  -------
 *  Dedicated calibration firmware for building an empirical mapping
 *  from motor-side AS5600 angle to beam angle measured externally
 *  with an iPhone level app.
 *
 *  This firmware is intentionally separate from the controller in
 *  main.cpp so the working control loop stays untouched.
 *
 *  Serial command protocol (single-letter, case-insensitive)
 *  --------------------------------------------------------
 *    E  Enable driver
 *    D  Disable driver
 *    L  Capture lower endpoint at the current held position
 *    U  Capture upper endpoint, reset sweep-relative steps to zero,
 *       and auto-detect which microstep direction moves toward lower
 *    N  Move one fixed forward increment toward lower and capture
 *    B  Move one fixed reverse increment toward upper and capture
 *    C  Clear sweep state in firmware by returning to the stored
 *       upper sweep start and resetting the relative step counter
 *    S  Print status
 *
 *  Host tool notes
 *  ---------------
 *  The companion Python tool is the primary UI. It waits for REC
 *  records, prompts for the iPhone angle, and writes the CSV log.
 *
 *  Wire protocol
 *  -------------
 *  One parseable CSV line is emitted per command result:
 *
 *    READY,<firmware_id>
 *    OK,<message>
 *    ERR,<message>
 *    STAT,<driver>,<rel_steps>,<forward_sign>,<last_valid>,
 *         <lower_valid>,<upper_valid>,<last_unwrapped>,
 *         <lower_unwrapped>,<upper_unwrapped>
 *    REC,<tag>,<t_ms>,<rel_steps>,<raw_mean>,<raw_std>,
 *        <deg_wrapped>,<deg_unwrapped>,<driver>,<forward_sign>,
 *        <lower_valid>,<upper_valid>
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================================================================
//  SECTION 1 — PIN ASSIGNMENTS
// ================================================================
static const uint8_t PIN_STEP = 2;
static const uint8_t PIN_DIR  = 3;
static const uint8_t PIN_EN   = 4;   // TMC2209 ENN, active-LOW


// ================================================================
//  SECTION 2 — AS5600 PARAMETERS
// ================================================================
static const uint8_t  AS5600_ADDR             = 0x36;
static const uint8_t  AS5600_REG_RAW_ANGLE_HI = 0x0C;
static const uint16_t AS5600_SAMPLES          = 64;
static const float    AS5600_RAW_TO_DEG       = 360.0f / 4096.0f;


// ================================================================
//  SECTION 3 — STEPPER PARAMETERS
// ================================================================
static const int32_t  STEPS_PER_REV           = 3200;   // 200 * 16 microsteps
static const int32_t  SWEEP_STEP_MICROSTEPS   = 4;
static const uint32_t STEP_PERIOD_US          = 800;    // calibration speed is intentionally gentle
static const uint32_t STEP_PULSE_US           = 5;
static const uint32_t DIR_SETUP_US            = 4;
static const uint32_t DIR_REVERSE_SETTLE_US   = 1200;
static const uint32_t SWEEP_SETTLE_MS         = 800;
static const uint32_t PROBE_SETTLE_MS         = 250;
static const float    PROBE_MIN_DELTA_DEG     = 0.05f;
static const float    LOWER_STOP_MARGIN_DEG   = 0.20f;

// Keep the same logical direction convention as the controller.
static const int8_t   DIR_SIGN                = -1;


// ================================================================
//  SECTION 4 — SAMPLE RECORDS
// ================================================================
struct SampleRecord {
    bool     valid;
    char     tag;
    uint32_t t_ms;
    int32_t  rel_steps;
    float    raw_mean;
    float    raw_std;
    float    deg_wrapped;
    float    deg_unwrapped;
};


// ================================================================
//  SECTION 5 — GLOBAL STATE
// ================================================================
static bool         g_driver_enabled     = false;
static int32_t      g_rel_steps          = 0;
static int8_t       g_forward_step_sign  = 0;   // logical signed step direction for 'N'
static int8_t       g_prev_motion_sign   = 0;
static bool         g_unwrap_seeded      = false;
static float        g_prev_wrapped_deg   = 0.0f;
static float        g_prev_unwrapped_deg = 0.0f;
static SampleRecord g_last_sample        = {};
static SampleRecord g_lower_sample       = {};
static SampleRecord g_upper_sample       = {};


// ================================================================
//  SECTION 6 — FUNCTION PROTOTYPES
// ================================================================
void  set_driver_enabled(bool enabled);
bool  read_as5600_raw_once(uint16_t &raw);
bool  capture_sample(char tag, SampleRecord &sample);
bool  step_signed(int32_t delta_steps);
bool  resolve_forward_direction(void);
void  handle_enable(void);
void  handle_disable(void);
void  handle_lower_capture(void);
void  handle_upper_capture(void);
void  handle_forward_step(void);
void  handle_reverse_step(void);
void  handle_clear_sweep(void);
void  handle_status(void);
void  print_ready(void);
void  print_ok(const __FlashStringHelper *message);
void  print_error(const __FlashStringHelper *message);
void  print_status(void);
void  print_sample(const SampleRecord &sample);
float unwrap_degrees(float wrapped_deg);


// ================================================================
//  setup()
// ================================================================
void setup() {
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_EN, OUTPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_EN, HIGH);   // disabled by default

    Serial.begin(115200);
    Wire.begin();

    print_ready();
    print_status();
}


// ================================================================
//  loop()
// ================================================================
void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
            continue;
        }
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 'a' + 'A');
        }

        switch (c) {
            case 'E': handle_enable();        break;
            case 'D': handle_disable();       break;
            case 'L': handle_lower_capture(); break;
            case 'U': handle_upper_capture(); break;
            case 'N': handle_forward_step();  break;
            case 'B': handle_reverse_step();  break;
            case 'C': handle_clear_sweep();   break;
            case 'S': handle_status();        break;
            default:  print_error(F("unknown_command")); break;
        }
    }
}


// ================================================================
//  Driver helpers
// ================================================================
void set_driver_enabled(bool enabled) {
    g_driver_enabled = enabled;
    digitalWrite(PIN_EN, enabled ? LOW : HIGH);
}

void handle_enable() {
    set_driver_enabled(true);
    print_ok(F("driver_enabled"));
}

void handle_disable() {
    set_driver_enabled(false);
    print_ok(F("driver_disabled"));
}


// ================================================================
//  AS5600 helpers
// ================================================================
bool read_as5600_raw_once(uint16_t &raw) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_RAW_ANGLE_HI);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = Wire.requestFrom((int)AS5600_ADDR, 2);
    if (received != 2) {
        return false;
    }

    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw = (uint16_t)(((hi & 0x0F) << 8) | lo);
    return true;
}

float unwrap_degrees(float wrapped_deg) {
    if (!g_unwrap_seeded) {
        g_unwrap_seeded      = true;
        g_prev_wrapped_deg   = wrapped_deg;
        g_prev_unwrapped_deg = wrapped_deg;
        return wrapped_deg;
    }

    float delta = wrapped_deg - g_prev_wrapped_deg;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    g_prev_wrapped_deg   = wrapped_deg;
    g_prev_unwrapped_deg = g_prev_unwrapped_deg + delta;
    return g_prev_unwrapped_deg;
}

bool capture_sample(char tag, SampleRecord &sample) {
    float mean_raw = 0.0f;
    float m2       = 0.0f;

    for (uint16_t i = 0; i < AS5600_SAMPLES; i++) {
        uint16_t raw = 0;
        if (!read_as5600_raw_once(raw)) {
            return false;
        }

        const float raw_f   = (float)raw;
        const float delta   = raw_f - mean_raw;
        mean_raw += delta / (float)(i + 1);
        const float delta2  = raw_f - mean_raw;
        m2 += delta * delta2;
    }

    float variance = m2 / (float)AS5600_SAMPLES;
    if (variance < 0.0f) {
        variance = 0.0f;
    }

    sample.valid         = true;
    sample.tag           = tag;
    sample.t_ms          = millis();
    sample.rel_steps     = g_rel_steps;
    sample.raw_mean      = mean_raw;
    sample.raw_std       = sqrtf(variance);
    sample.deg_wrapped   = mean_raw * AS5600_RAW_TO_DEG;
    sample.deg_unwrapped = unwrap_degrees(sample.deg_wrapped);

    g_last_sample = sample;
    return true;
}


// ================================================================
//  Stepper helpers
// ================================================================
bool step_signed(int32_t delta_steps) {
    if (!g_driver_enabled) {
        print_error(F("driver_disabled"));
        return false;
    }

    if (delta_steps == 0) {
        return true;
    }

    int8_t motion_sign = (delta_steps > 0) ? 1 : -1;
    bool dir_high = (DIR_SIGN > 0) ? (delta_steps > 0) : (delta_steps < 0);
    digitalWrite(PIN_DIR, dir_high ? HIGH : LOW);

    if (g_prev_motion_sign != 0 && motion_sign != g_prev_motion_sign) {
        delayMicroseconds(DIR_REVERSE_SETTLE_US);
    }
    g_prev_motion_sign = motion_sign;

    delayMicroseconds(DIR_SETUP_US);

    int32_t n_steps = (delta_steps > 0) ? delta_steps : -delta_steps;
    for (int32_t i = 0; i < n_steps; i++) {
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(PIN_STEP, LOW);
        if (STEP_PERIOD_US > STEP_PULSE_US) {
            delayMicroseconds(STEP_PERIOD_US - STEP_PULSE_US);
        }
    }

    g_rel_steps += delta_steps;
    return true;
}

bool resolve_forward_direction() {
    if (!g_lower_sample.valid || !g_upper_sample.valid) {
        print_error(F("endpoints_not_ready"));
        return false;
    }

    const float target_delta = g_lower_sample.deg_unwrapped - g_upper_sample.deg_unwrapped;
    if (fabsf(target_delta) < PROBE_MIN_DELTA_DEG) {
        print_error(F("endpoint_delta_too_small"));
        return false;
    }

    const int8_t desired_angle_sign = (target_delta > 0.0f) ? 1 : -1;
    const int8_t probe_dirs[2] = { 1, -1 };
    const SampleRecord baseline = g_last_sample;

    for (uint8_t i = 0; i < 2; i++) {
        const int8_t probe_dir = probe_dirs[i];

        if (!step_signed(probe_dir * SWEEP_STEP_MICROSTEPS)) {
            return false;
        }
        delay(PROBE_SETTLE_MS);

        SampleRecord probe_sample = {};
        if (!capture_sample('P', probe_sample)) {
            print_error(F("probe_capture_failed"));
            return false;
        }

        const float measured_delta = probe_sample.deg_unwrapped - baseline.deg_unwrapped;

        if (!step_signed(-probe_dir * SWEEP_STEP_MICROSTEPS)) {
            return false;
        }
        delay(PROBE_SETTLE_MS);

        SampleRecord return_sample = {};
        if (!capture_sample('R', return_sample)) {
            print_error(F("probe_return_capture_failed"));
            return false;
        }

        if (fabsf(measured_delta) >= PROBE_MIN_DELTA_DEG) {
            const int8_t measured_sign = (measured_delta > 0.0f) ? 1 : -1;
            if (measured_sign == desired_angle_sign) {
                g_forward_step_sign = probe_dir;
                return true;
            }
        }
    }

    g_forward_step_sign = 0;
    print_error(F("forward_direction_detect_failed"));
    return false;
}


// ================================================================
//  Command handlers
// ================================================================
void handle_lower_capture() {
    if (!g_driver_enabled) {
        print_error(F("driver_disabled"));
        return;
    }

    g_rel_steps = 0;          // manual moves invalidate relative step history
    g_forward_step_sign = 0;  // direction must be re-established from endpoints

    SampleRecord lower = {};
    if (!capture_sample('L', lower)) {
        print_error(F("lower_capture_failed"));
        return;
    }

    g_lower_sample = lower;
    print_sample(g_lower_sample);
}

void handle_upper_capture() {
    if (!g_driver_enabled) {
        print_error(F("driver_disabled"));
        return;
    }

    g_rel_steps = 0;          // sweep origin is always the stored upper point
    g_forward_step_sign = 0;

    SampleRecord upper = {};
    if (!capture_sample('U', upper)) {
        print_error(F("upper_capture_failed"));
        return;
    }
    g_upper_sample = upper;

    if (g_lower_sample.valid) {
        if (!resolve_forward_direction()) {
            return;
        }
    }

    print_sample(g_upper_sample);
}

void handle_forward_step() {
    if (!g_driver_enabled) {
        print_error(F("driver_disabled"));
        return;
    }
    if (!g_lower_sample.valid || !g_upper_sample.valid || g_forward_step_sign == 0) {
        print_error(F("run_L_and_U_first"));
        return;
    }
    if (g_last_sample.valid) {
        const float target_delta = g_lower_sample.deg_unwrapped - g_upper_sample.deg_unwrapped;
        const int8_t travel_sign = (target_delta > 0.0f) ? 1 : -1;
        const float remaining = (g_lower_sample.deg_unwrapped - g_last_sample.deg_unwrapped)
                              * (float)travel_sign;
        if (remaining <= LOWER_STOP_MARGIN_DEG) {
            print_error(F("at_lower_limit"));
            return;
        }
    }

    if (!step_signed(g_forward_step_sign * SWEEP_STEP_MICROSTEPS)) {
        return;
    }
    delay(SWEEP_SETTLE_MS);

    SampleRecord sample = {};
    if (!capture_sample('N', sample)) {
        print_error(F("forward_capture_failed"));
        return;
    }

    print_sample(sample);
}

void handle_reverse_step() {
    if (!g_driver_enabled) {
        print_error(F("driver_disabled"));
        return;
    }
    if (!g_lower_sample.valid || !g_upper_sample.valid || g_forward_step_sign == 0) {
        print_error(F("run_L_and_U_first"));
        return;
    }
    if (g_rel_steps <= 0) {
        print_error(F("already_at_upper_start"));
        return;
    }

    if (!step_signed(-g_forward_step_sign * SWEEP_STEP_MICROSTEPS)) {
        return;
    }
    delay(SWEEP_SETTLE_MS);

    SampleRecord sample = {};
    if (!capture_sample('B', sample)) {
        print_error(F("reverse_capture_failed"));
        return;
    }

    print_sample(sample);
}

void handle_clear_sweep() {
    if (!g_upper_sample.valid) {
        print_error(F("upper_endpoint_missing"));
        return;
    }
    if (!g_driver_enabled) {
        print_error(F("driver_disabled"));
        return;
    }

    if (g_rel_steps != 0) {
        if (!step_signed(-g_rel_steps)) {
            return;
        }
        delay(SWEEP_SETTLE_MS);
    }

    SampleRecord sample = {};
    if (!capture_sample('C', sample)) {
        print_error(F("clear_return_capture_failed"));
        return;
    }

    print_ok(F("sweep_reset"));
}

void handle_status() {
    print_status();
}


// ================================================================
//  Serial output helpers
// ================================================================
void print_ready() {
    Serial.println(F("READY,ball_beam_as5600_cal_v1"));
}

void print_ok(const __FlashStringHelper *message) {
    Serial.print(F("OK,"));
    Serial.println(message);
}

void print_error(const __FlashStringHelper *message) {
    Serial.print(F("ERR,"));
    Serial.println(message);
}

static void print_optional_float(bool valid, float value, uint8_t digits) {
    if (!valid) {
        Serial.print(F("NA"));
        return;
    }
    Serial.print(value, digits);
}

void print_status() {
    Serial.print(F("STAT,"));
    Serial.print(g_driver_enabled ? 1 : 0);
    Serial.print(',');
    Serial.print(g_rel_steps);
    Serial.print(',');
    Serial.print(g_forward_step_sign);
    Serial.print(',');
    Serial.print(g_last_sample.valid ? 1 : 0);
    Serial.print(',');
    Serial.print(g_lower_sample.valid ? 1 : 0);
    Serial.print(',');
    Serial.print(g_upper_sample.valid ? 1 : 0);
    Serial.print(',');
    print_optional_float(g_last_sample.valid, g_last_sample.deg_unwrapped, 5);
    Serial.print(',');
    print_optional_float(g_lower_sample.valid, g_lower_sample.deg_unwrapped, 5);
    Serial.print(',');
    print_optional_float(g_upper_sample.valid, g_upper_sample.deg_unwrapped, 5);
    Serial.println();
}

void print_sample(const SampleRecord &sample) {
    Serial.print(F("REC,"));
    Serial.print(sample.tag);
    Serial.print(',');
    Serial.print(sample.t_ms);
    Serial.print(',');
    Serial.print(sample.rel_steps);
    Serial.print(',');
    Serial.print(sample.raw_mean, 3);
    Serial.print(',');
    Serial.print(sample.raw_std, 3);
    Serial.print(',');
    Serial.print(sample.deg_wrapped, 5);
    Serial.print(',');
    Serial.print(sample.deg_unwrapped, 5);
    Serial.print(',');
    Serial.print(g_driver_enabled ? 1 : 0);
    Serial.print(',');
    Serial.print(g_forward_step_sign);
    Serial.print(',');
    Serial.print(g_lower_sample.valid ? 1 : 0);
    Serial.print(',');
    Serial.print(g_upper_sample.valid ? 1 : 0);
    Serial.println();
}
