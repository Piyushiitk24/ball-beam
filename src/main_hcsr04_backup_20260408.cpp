/*
 * ================================================================
 *  Ball-and-Beam Staged Controller
 * ================================================================
 *  Platform  : Arduino Nano (ATmega328P, 16 MHz)
 *  Ball pos  : HC-SR04 ultrasonic sensor  TRIG->D8  ECHO->D9
 *  Beam angle: AS5600 on I2C              SDA->A4  SCL->A5  0x36
 *  Actuator  : NEMA17 stepper + TMC2209   STEP->D2  DIR->D3  EN->D4
 *
 *  This replaces the older Sharp-sensor direct PID with a staged
 *  cascaded design:
 *
 *    M0: sensor/state readout only, no control output
 *    M1: manual beam-angle hold using AS5600 inner loop only
 *    M2: ball-position cascade, gated by POSITION_CALIBRATED
 *
 *  The boot mode is always M0 and the motor driver remains disabled
 *  until the host sends G. M2 is intentionally disabled until the
 *  HC-SR04 geometry is measured and POSITION_CALIBRATED is set true.
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>

// ================================================================
//  SECTION 1 - PIN ASSIGNMENTS
// ================================================================
static const uint8_t PIN_STEP        = 2;
static const uint8_t PIN_DIR         = 3;
static const uint8_t PIN_EN          = 4;   // TMC2209 ENN, active-LOW
static const uint8_t PIN_HCSR04_TRIG = 8;
static const uint8_t PIN_HCSR04_ECHO = 9;


// ================================================================
//  SECTION 2 - HC-SR04 POSITION SENSOR
// ================================================================
static const uint32_t HCSR04_TIMEOUT_US       = 6000UL;
static const float    HCSR04_CM_PER_US        = 0.01715f;
static const float    HCSR04_MIN_VALID_CM     = 2.0f;
static const float    HCSR04_MAX_VALID_CM     = 80.0f;
static const float    HCSR04_CAL_MARGIN_CM    = 5.0f;
static const float    HCSR04_MAX_JUMP_CM      = 4.0f;
static const uint8_t  HCSR04_JUMP_ACCEPT_COUNT = 4;
static const uint8_t  HCSR04_INVALID_LIMIT    = 8;

// Sponge smiley ball + HC-SR04 geometry measured from calibration
// telemetry. Endpoint values are set from filtered valid readings:
// sensor side ~3.95 cm, desired center ~10.30 cm, motor side ~27.25 cm.
static const bool     POSITION_CALIBRATED     = true;
static const float    D_MIN_CM                = 3.95f;
static const float    D_MAX_CM                = 27.25f;
static const float    D_SETPOINT_DEFAULT_CM   = 10.30f;

static const float    DIST_EMA_ALPHA          = 0.25f;
static const float    X_DOT_EMA_ALPHA         = 0.20f;


// ================================================================
//  SECTION 3 - AS5600 BEAM ANGLE
// ================================================================
static const uint8_t  AS5600_ADDR             = 0x36;
static const uint8_t  AS5600_REG_RAW_ANGLE_HI = 0x0C;
static const float    AS5600_RAW_TO_DEG       = 360.0f / 4096.0f;

// From calibration_logs/20260407_122510/beam_angle_calibration_report.md
static const float    THETA_SLOPE_DEG_PER_AS_DEG = 0.07666806f;
static const float    THETA_OFFSET_DEG           = -23.28443907f;
static const float    THETA_CAL_MIN_DEG          = -0.70f;
static const float    THETA_CAL_MAX_DEG          =  3.10f;
static const float    THETA_CAL_MARGIN_DEG       =  0.10f;
static const float    THETA_CAL_EXTRAPOLATE_DEG  =  0.30f;
static const float    THETA_DOT_EMA_ALPHA        =  0.30f;


// ================================================================
//  SECTION 4 - STEPPER AND INNER LOOP
// ================================================================
static const int32_t  STEPS_PER_REV              = 3200;
static const float    STEPS_PER_BEAM_DEG         =
    ((float)STEPS_PER_REV / 360.0f) / THETA_SLOPE_DEG_PER_AS_DEG;

// Same DIR polarity convention as the calibration firmware.
static const int8_t   DIR_SIGN                   = -1;

// Calibration showed that increasing theta requires negative logical
// microsteps with the current wiring and DIR_SIGN.
static const int8_t   INNER_STEP_SIGN            = -1;

static const float    INNER_KP                   = 0.70f;
static const int32_t  INNER_MAX_STEP_DELTA       = 60;
static const uint32_t STEP_PERIOD_US             = 200;
static const uint32_t STEP_START_PERIOD_US       = 900;
static const uint16_t STEP_RAMP_STEPS            = 16;
static const uint32_t STEP_PULSE_US              = 5;
static const uint32_t DIR_SETUP_US               = 4;
static const uint32_t DIR_REVERSE_SETTLE_US      = 1200;


// ================================================================
//  SECTION 5 - OUTER LOOP AND TIMING
// ================================================================
// HC-SR04 modules are more reliable when retriggered slowly enough for
// late echoes to decay; 60 ms is intentionally more conservative than
// the earlier 40 ms loop.
static const uint32_t LOOP_MS                    = 60;
static const float    DT                         = 0.060f;
static const uint8_t  PRINT_EVERY                = 17;  // T stream: about one row per second
static const float    THETA_CMD_LIMIT_DEFAULT_DEG = 2.5f;

static float g_outer_kp                          = 0.10f;
static float g_outer_kv                          = 0.00f;
static float g_theta_cmd_limit_deg               = THETA_CMD_LIMIT_DEFAULT_DEG;

// Manual sign test: A+ rolls toward motor/far end, A- rolls toward
// sensor/near end. Far-side error therefore needs negative theta.
static int8_t g_outer_sign                       = -1;


// ================================================================
//  SECTION 6 - STATE
// ================================================================
enum ControllerMode : uint8_t {
    MODE_TELEMETRY = 0,
    MODE_MANUAL_ANGLE = 1,
    MODE_CASCADE = 2,
};

struct DistanceSample {
    bool valid;
    float cm;
    uint32_t echo_us;
};

static ControllerMode g_mode              = MODE_TELEMETRY;
static bool           g_driver_enabled    = false;
static bool           g_cal_mode          = false;
static bool           g_telemetry_stream  = false;
static bool           g_print_once        = false;
static uint32_t       g_last_ms           = 0;

static float          g_distance_window[3] = {D_SETPOINT_DEFAULT_CM,
                                              D_SETPOINT_DEFAULT_CM,
                                              D_SETPOINT_DEFAULT_CM};
static uint8_t        g_distance_index     = 0;
static uint8_t        g_distance_count     = 0;
static bool           g_distance_seeded    = false;
static float          g_d_raw_cm           = D_SETPOINT_DEFAULT_CM;
static float          g_d_filt_cm          = D_SETPOINT_DEFAULT_CM;
static bool           g_distance_accepted  = false;
static float          g_pending_distance_cm = D_SETPOINT_DEFAULT_CM;
static uint8_t        g_pending_distance_count = 0;
static float          g_setpoint_cm        = D_SETPOINT_DEFAULT_CM;
static float          g_x_cm               = 0.0f;
static float          g_prev_x_cm          = 0.0f;
static float          g_x_dot_cm_s         = 0.0f;
static uint8_t        g_invalid_count      = 0;

static bool           g_unwrap_seeded      = false;
static float          g_prev_wrapped_deg   = 0.0f;
static float          g_prev_unwrapped_deg = 0.0f;
static uint16_t       g_as5600_raw         = 0;
static float          g_as5600_wrapped_deg = 0.0f;
static float          g_as5600_unwrapped_deg = 0.0f;
static float          g_theta_cal_deg      = 0.0f;
static float          g_theta_balance_deg  = 0.0f;
static float          g_theta_rel_deg      = 0.0f;
static float          g_prev_theta_rel_deg = 0.0f;
static float          g_theta_dot_deg_s    = 0.0f;
static bool           g_as5600_valid       = false;

static float          g_manual_theta_cmd_rel_deg = 0.0f;
static float          g_theta_cmd_rel_deg        = 0.0f;
static int32_t        g_step_pos                 = 0;
static int32_t        g_last_step_delta          = 0;


// ================================================================
//  SECTION 7 - PROTOTYPES
// ================================================================
DistanceSample read_hcsr04_cm(void);
float update_distance_filter(const DistanceSample &sample);
void accept_distance_sample(float cm);
float median_distance_window(void);
bool read_as5600_raw_once(uint16_t &raw);
bool update_as5600(void);
float unwrap_degrees(float wrapped_deg);
void update_derived_states(bool distance_valid, bool theta_valid);
float clamp_float(float value, float lo, float hi);
int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi);
int32_t round_to_i32(float value);
float clamp_theta_rel_command(float theta_rel_deg);
int32_t compute_inner_step_delta(float theta_cmd_rel_deg);
void step_relative(int32_t delta_steps);
void set_driver_enabled(bool enabled);
void set_mode(ControllerMode mode);
void reset_dynamic_state(void);
void reset_distance_filter(void);
bool is_immediate_serial_command(char c);
void execute_serial_command(char *buf);
void parse_serial(void);
void print_header(void);
void print_config(void);
void print_telemetry(bool valid, float d_raw_print, uint32_t echo_us, int32_t step_delta);


// ================================================================
//  setup()
// ================================================================
void setup() {
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_HCSR04_TRIG, OUTPUT);
    pinMode(PIN_HCSR04_ECHO, INPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_EN, HIGH);
    digitalWrite(PIN_HCSR04_TRIG, LOW);

    Serial.begin(115200);
    Wire.begin();

    DistanceSample initial_distance = read_hcsr04_cm();
    update_distance_filter(initial_distance);
    update_as5600();
    reset_dynamic_state();

    g_last_ms = millis();
    print_config();
    print_header();
}


// ================================================================
//  loop()
// ================================================================
void loop() {
    parse_serial();

    uint32_t now = millis();
    if ((now - g_last_ms) < LOOP_MS) {
        return;
    }
    g_last_ms = now;

    DistanceSample distance = read_hcsr04_cm();
    float d_raw_print = distance.valid ? distance.cm : -1.0f;
    update_distance_filter(distance);
    bool distance_ok = distance.valid && g_distance_accepted;

    bool theta_valid = update_as5600();
    update_derived_states(distance_ok, theta_valid);

    g_theta_cmd_rel_deg = 0.0f;
    int32_t step_delta = 0;

    if (!g_cal_mode) {
        if (g_mode == MODE_MANUAL_ANGLE) {
            g_theta_cmd_rel_deg = clamp_theta_rel_command(g_manual_theta_cmd_rel_deg);
            step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
        } else if (g_mode == MODE_CASCADE) {
            if (POSITION_CALIBRATED && g_invalid_count < HCSR04_INVALID_LIMIT) {
                float raw_theta_cmd = (float)g_outer_sign *
                    ((g_outer_kp * g_x_cm) + (g_outer_kv * g_x_dot_cm_s));
                raw_theta_cmd = clamp_float(raw_theta_cmd,
                                            -g_theta_cmd_limit_deg,
                                             g_theta_cmd_limit_deg);
                g_theta_cmd_rel_deg = clamp_theta_rel_command(raw_theta_cmd);
            } else {
                g_theta_cmd_rel_deg = clamp_theta_rel_command(0.0f);
            }
            step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
        }
    }

    if (!g_driver_enabled || !theta_valid || g_mode == MODE_TELEMETRY || g_cal_mode) {
        step_delta = 0;
    }

    if (step_delta != 0) {
        step_relative(step_delta);
    }
    g_last_step_delta = step_delta;

    static uint8_t print_count = 0;
    bool valid = distance_ok && theta_valid;
    if (g_print_once || (g_telemetry_stream && (++print_count >= PRINT_EVERY))) {
        print_count = 0;
        g_print_once = false;
        print_telemetry(valid, d_raw_print, distance.echo_us, step_delta);
    }
}


// ================================================================
//  HC-SR04 helpers
// ================================================================
DistanceSample read_hcsr04_cm() {
    digitalWrite(PIN_HCSR04_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_HCSR04_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_HCSR04_TRIG, LOW);

    uint32_t echo_us = pulseIn(PIN_HCSR04_ECHO, HIGH, HCSR04_TIMEOUT_US);
    if (echo_us == 0) {
        DistanceSample sample = {false, 0.0f, 0};
        return sample;
    }

    float cm = (float)echo_us * HCSR04_CM_PER_US;
    bool valid = (cm >= HCSR04_MIN_VALID_CM) && (cm <= HCSR04_MAX_VALID_CM);
    if (POSITION_CALIBRATED) {
        valid = valid
             && (cm >= (D_MIN_CM - HCSR04_CAL_MARGIN_CM))
             && (cm <= (D_MAX_CM + HCSR04_CAL_MARGIN_CM));
    }
    DistanceSample sample = {valid, cm, echo_us};
    return sample;
}

float update_distance_filter(const DistanceSample &sample) {
    g_distance_accepted = false;

    if (sample.valid) {
        float jump_cm = sample.cm - g_d_filt_cm;
        if (jump_cm < 0.0f) {
            jump_cm = -jump_cm;
        }

        if (g_distance_seeded && jump_cm > HCSR04_MAX_JUMP_CM) {
            float pending_jump_cm = sample.cm - g_pending_distance_cm;
            if (pending_jump_cm < 0.0f) {
                pending_jump_cm = -pending_jump_cm;
            }

            if (g_pending_distance_count == 0
                    || pending_jump_cm > HCSR04_MAX_JUMP_CM) {
                g_pending_distance_cm = sample.cm;
                g_pending_distance_count = 1;
            } else if (g_pending_distance_count < 255) {
                g_pending_distance_cm =
                    (0.5f * g_pending_distance_cm) + (0.5f * sample.cm);
                g_pending_distance_count++;
            }

            if (g_pending_distance_count >= HCSR04_JUMP_ACCEPT_COUNT) {
                accept_distance_sample(g_pending_distance_cm);
                g_pending_distance_count = 0;
            } else if (g_invalid_count < 255) {
                g_invalid_count++;
            }
        } else {
            g_pending_distance_count = 0;
            accept_distance_sample(sample.cm);
        }
    } else if (g_invalid_count < 255) {
        g_invalid_count++;
    }

    return g_d_filt_cm;
}

void accept_distance_sample(float cm) {
    g_invalid_count = 0;
    g_distance_accepted = true;
    g_d_raw_cm = cm;
    g_distance_window[g_distance_index] = cm;
    g_distance_index = (uint8_t)((g_distance_index + 1) % 3);
    if (g_distance_count < 3) {
        g_distance_count++;
    }

    float median_cm = median_distance_window();
    if (!g_distance_seeded) {
        g_d_filt_cm = median_cm;
        g_distance_seeded = true;
    } else {
        g_d_filt_cm = (DIST_EMA_ALPHA * median_cm)
                    + ((1.0f - DIST_EMA_ALPHA) * g_d_filt_cm);
    }
}

float median_distance_window() {
    if (g_distance_count == 0) {
        return g_d_filt_cm;
    }
    if (g_distance_count == 1) {
        uint8_t idx = (g_distance_index == 0) ? 2 : (uint8_t)(g_distance_index - 1);
        return g_distance_window[idx];
    }
    if (g_distance_count == 2) {
        return 0.5f * (g_distance_window[0] + g_distance_window[1]);
    }

    float a = g_distance_window[0];
    float b = g_distance_window[1];
    float c = g_distance_window[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
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

bool update_as5600() {
    uint16_t raw = 0;
    if (!read_as5600_raw_once(raw)) {
        g_as5600_valid = false;
        return false;
    }

    g_as5600_raw = raw;
    g_as5600_wrapped_deg = (float)raw * AS5600_RAW_TO_DEG;
    g_as5600_unwrapped_deg = unwrap_degrees(g_as5600_wrapped_deg);
    g_theta_cal_deg = (THETA_SLOPE_DEG_PER_AS_DEG * g_as5600_unwrapped_deg)
                    + THETA_OFFSET_DEG;
    g_theta_rel_deg = g_theta_cal_deg - g_theta_balance_deg;
    g_as5600_valid = true;
    return true;
}

float unwrap_degrees(float wrapped_deg) {
    if (!g_unwrap_seeded) {
        g_unwrap_seeded = true;
        g_prev_wrapped_deg = wrapped_deg;
        g_prev_unwrapped_deg = wrapped_deg;
        return wrapped_deg;
    }

    float delta = wrapped_deg - g_prev_wrapped_deg;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    g_prev_wrapped_deg = wrapped_deg;
    g_prev_unwrapped_deg += delta;
    return g_prev_unwrapped_deg;
}


// ================================================================
//  State and control helpers
// ================================================================
void update_derived_states(bool distance_valid, bool theta_valid) {
    float x_now = g_d_filt_cm - g_setpoint_cm;
    if (distance_valid && g_distance_seeded) {
        float x_rate = (x_now - g_prev_x_cm) / DT;
        g_x_dot_cm_s = (X_DOT_EMA_ALPHA * x_rate)
                     + ((1.0f - X_DOT_EMA_ALPHA) * g_x_dot_cm_s);
        g_prev_x_cm = x_now;
    } else {
        g_x_dot_cm_s = 0.0f;
    }
    g_x_cm = x_now;

    if (theta_valid) {
        float theta_rate = (g_theta_rel_deg - g_prev_theta_rel_deg) / DT;
        g_theta_dot_deg_s = (THETA_DOT_EMA_ALPHA * theta_rate)
                          + ((1.0f - THETA_DOT_EMA_ALPHA) * g_theta_dot_deg_s);
        g_prev_theta_rel_deg = g_theta_rel_deg;
    } else {
        g_theta_dot_deg_s = 0.0f;
    }
}

float clamp_float(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

int32_t round_to_i32(float value) {
    return (int32_t)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

float clamp_theta_rel_command(float theta_rel_deg) {
    float theta_cal_min = THETA_CAL_MIN_DEG + THETA_CAL_MARGIN_DEG
                        - THETA_CAL_EXTRAPOLATE_DEG;
    float theta_cal_max = THETA_CAL_MAX_DEG - THETA_CAL_MARGIN_DEG
                        + THETA_CAL_EXTRAPOLATE_DEG;
    float theta_cal_cmd = g_theta_balance_deg + theta_rel_deg;
    theta_cal_cmd = clamp_float(theta_cal_cmd, theta_cal_min, theta_cal_max);
    return theta_cal_cmd - g_theta_balance_deg;
}

int32_t compute_inner_step_delta(float theta_cmd_rel_deg) {
    if (!g_as5600_valid) {
        return 0;
    }

    float theta_cmd_cal_deg = g_theta_balance_deg + theta_cmd_rel_deg;
    theta_cmd_cal_deg = clamp_float(theta_cmd_cal_deg,
                                    THETA_CAL_MIN_DEG + THETA_CAL_MARGIN_DEG
                                        - THETA_CAL_EXTRAPOLATE_DEG,
                                    THETA_CAL_MAX_DEG - THETA_CAL_MARGIN_DEG
                                        + THETA_CAL_EXTRAPOLATE_DEG);

    float theta_error_deg = theta_cmd_cal_deg - g_theta_cal_deg;
    float raw_steps = (float)INNER_STEP_SIGN * INNER_KP
                    * theta_error_deg * STEPS_PER_BEAM_DEG;
    return clamp_i32(round_to_i32(raw_steps),
                     -INNER_MAX_STEP_DELTA,
                      INNER_MAX_STEP_DELTA);
}


// ================================================================
//  Stepper helpers
// ================================================================
void step_relative(int32_t delta_steps) {
    if (!g_driver_enabled || delta_steps == 0) {
        return;
    }

    delta_steps = clamp_i32(delta_steps,
                            -INNER_MAX_STEP_DELTA,
                             INNER_MAX_STEP_DELTA);
    if (delta_steps == 0) {
        return;
    }

    static int8_t s_prev_motion_sign = 0;
    int8_t motion_sign = (delta_steps > 0) ? 1 : -1;
    bool dir_high = (DIR_SIGN > 0) ? (delta_steps > 0) : (delta_steps < 0);
    digitalWrite(PIN_DIR, dir_high ? HIGH : LOW);

    if (s_prev_motion_sign != 0 && motion_sign != s_prev_motion_sign) {
        delayMicroseconds(DIR_REVERSE_SETTLE_US);
    }
    s_prev_motion_sign = motion_sign;

    delayMicroseconds(DIR_SETUP_US);

    int32_t n_steps = (delta_steps > 0) ? delta_steps : -delta_steps;
    uint16_t ramp_steps = STEP_RAMP_STEPS;
    if (ramp_steps > (uint16_t)(n_steps / 2)) {
        ramp_steps = (uint16_t)(n_steps / 2);
    }

    for (int32_t i = 0; i < n_steps; i++) {
        uint32_t period_us = STEP_PERIOD_US;
        if (ramp_steps > 0) {
            if (i < ramp_steps) {
                period_us = STEP_START_PERIOD_US
                    - ((STEP_START_PERIOD_US - STEP_PERIOD_US)
                    * (uint32_t)(i + 1)) / ramp_steps;
            } else if (i >= (n_steps - ramp_steps)) {
                uint32_t j = (uint32_t)(n_steps - 1 - i);
                period_us = STEP_START_PERIOD_US
                    - ((STEP_START_PERIOD_US - STEP_PERIOD_US)
                    * (j + 1)) / ramp_steps;
            }
        }

        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(PIN_STEP, LOW);
        if (period_us > STEP_PULSE_US) {
            delayMicroseconds(period_us - STEP_PULSE_US);
        }
    }

    g_step_pos += delta_steps;
}

void set_driver_enabled(bool enabled) {
    g_driver_enabled = enabled;
    digitalWrite(PIN_EN, enabled ? LOW : HIGH);
}


// ================================================================
//  Serial command handling
// ================================================================
void set_mode(ControllerMode mode) {
    if (mode == MODE_CASCADE && !POSITION_CALIBRATED) {
        Serial.println(F("# ERR: M2 disabled until POSITION_CALIBRATED is true"));
        return;
    }

    g_mode = mode;
    g_cal_mode = false;
    if (g_mode == MODE_MANUAL_ANGLE) {
        g_manual_theta_cmd_rel_deg = clamp_theta_rel_command(g_theta_rel_deg);
    } else if (g_mode == MODE_CASCADE) {
        reset_distance_filter();
    }
    reset_dynamic_state();
    Serial.print(F("# Mode -> M"));
    Serial.println((int)g_mode);
}

void reset_dynamic_state() {
    g_x_cm = g_d_filt_cm - g_setpoint_cm;
    g_prev_x_cm = g_x_cm;
    g_x_dot_cm_s = 0.0f;
    g_theta_rel_deg = g_theta_cal_deg - g_theta_balance_deg;
    g_prev_theta_rel_deg = g_theta_rel_deg;
    g_theta_dot_deg_s = 0.0f;
    g_last_step_delta = 0;
}

void reset_distance_filter() {
    g_distance_index = 0;
    g_distance_count = 0;
    g_distance_seeded = false;
    g_distance_accepted = false;
    g_pending_distance_count = 0;
    g_invalid_count = 0;
    g_d_raw_cm = g_setpoint_cm;
    g_d_filt_cm = g_setpoint_cm;
    g_pending_distance_cm = g_setpoint_cm;
    for (uint8_t i = 0; i < 3; i++) {
        g_distance_window[i] = g_setpoint_cm;
    }
}

bool is_immediate_serial_command(char c) {
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }

    return c == 'G' || c == 'X' || c == 'Z' || c == 'Y'
        || c == 'R' || c == 'H' || c == 'O' || c == 'T'
        || c == 'C' || c == '?';
}

void execute_serial_command(char *buf) {
    char cmd = buf[0];
    if (cmd >= 'a' && cmd <= 'z') {
        cmd = (char)(cmd - 'a' + 'A');
    }

    if (cmd == 'G') {
        set_driver_enabled(true);
        Serial.println(F("# Driver ENABLED"));

    } else if (cmd == 'X') {
        set_driver_enabled(false);
        Serial.println(F("# Driver DISABLED"));

    } else if (cmd == 'M') {
        int mode = atoi(buf + 1);
        if (mode == 0) {
            set_mode(MODE_TELEMETRY);
        } else if (mode == 1) {
            set_mode(MODE_MANUAL_ANGLE);
        } else if (mode == 2) {
            set_mode(MODE_CASCADE);
        } else {
            Serial.println(F("# ERR: mode must be M0, M1, or M2"));
        }

    } else if (cmd == 'A') {
        float angle = atof(buf + 1);
        g_manual_theta_cmd_rel_deg = clamp_theta_rel_command(angle);
        Serial.print(F("# Manual theta command -> "));
        Serial.print(g_manual_theta_cmd_rel_deg, 3);
        Serial.println(F(" deg rel"));

    } else if (cmd == 'Z') {
        if (!g_as5600_valid) {
            Serial.println(F("# ERR: cannot zero theta, AS5600 invalid"));
        } else {
            g_theta_balance_deg = g_theta_cal_deg;
            g_manual_theta_cmd_rel_deg = 0.0f;
            g_theta_cmd_rel_deg = 0.0f;
            reset_dynamic_state();
            Serial.print(F("# theta_balance_deg -> "));
            Serial.println(g_theta_balance_deg, 5);
            Serial.println(F("# Manual theta command reset to A0.0"));
        }

    } else if (cmd == 'S') {
        float sp = atof(buf + 1);
        if (sp >= D_MIN_CM && sp <= D_MAX_CM) {
            g_setpoint_cm = sp;
            reset_dynamic_state();
            Serial.print(F("# Setpoint -> "));
            Serial.print(g_setpoint_cm, 2);
            Serial.println(POSITION_CALIBRATED
                ? F(" cm")
                : F(" cm (provisional, position calibration still false)"));
        } else {
            Serial.print(F("# ERR: setpoint must be in ["));
            Serial.print(D_MIN_CM, 1);
            Serial.print(F(", "));
            Serial.print(D_MAX_CM, 1);
            Serial.println(F("] cm"));
        }

    } else if (cmd == 'Y') {
        g_outer_sign = (int8_t)(-g_outer_sign);
        Serial.print(F("# outer_sign -> "));
        Serial.println(g_outer_sign);

    } else if (cmd == 'P') {
        g_outer_kp = atof(buf + 1);
        Serial.print(F("# outer Kp -> "));
        Serial.println(g_outer_kp, 5);

    } else if (cmd == 'V') {
        g_outer_kv = atof(buf + 1);
        Serial.print(F("# outer Kv -> "));
        Serial.println(g_outer_kv, 5);

    } else if (cmd == 'L') {
        float limit = atof(buf + 1);
        g_theta_cmd_limit_deg = clamp_float(limit, 0.10f, 3.00f);
        Serial.print(F("# theta command limit -> "));
        Serial.print(g_theta_cmd_limit_deg, 3);
        Serial.println(F(" deg rel"));

    } else if (cmd == 'R') {
        reset_dynamic_state();
        Serial.println(F("# Dynamic state reset"));

    } else if (cmd == 'H') {
        print_header();

    } else if (cmd == 'O') {
        g_print_once = true;

    } else if (cmd == 'T') {
        g_telemetry_stream = !g_telemetry_stream;
        Serial.print(F("# Telemetry stream "));
        Serial.println(g_telemetry_stream ? F("ON") : F("OFF"));

    } else if (cmd == 'C') {
        g_cal_mode = !g_cal_mode;
        if (g_cal_mode) {
            g_mode = MODE_TELEMETRY;
        }
        reset_dynamic_state();
        Serial.print(F("# HC-SR04 calibration telemetry "));
        Serial.println(g_cal_mode ? F("ON (forced M0, use O for samples)") : F("OFF"));
        g_print_once = true;

    } else if (cmd == '?') {
        print_config();

    } else {
        Serial.println(F("# ERR: unknown command"));
    }
}

void parse_serial() {
    static char buf[32];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';
            if (idx > 0) {
                execute_serial_command(buf);
            }
            idx = 0;

        } else if (idx == 0 && is_immediate_serial_command(c)) {
            buf[0] = c;
            buf[1] = '\0';
            execute_serial_command(buf);

        } else if (idx < (uint8_t)(sizeof(buf) - 1)) {
            buf[idx++] = c;
        }
    }
}


// ================================================================
//  Telemetry
// ================================================================
void print_header() {
    Serial.println(F("t_ms,mode,valid,echo_us,d_raw_cm,d_filt_cm,x_cm,x_dot_cm_s,"
                     "theta_cal_deg,theta_rel_deg,theta_dot_deg_s,"
                     "theta_cmd_rel_deg,outer_sign,driver_enabled,step_delta,"
                     "invalid_count"));
}

void print_config() {
    Serial.println(F("# -- Ball-and-Beam HC-SR04 + AS5600 Controller --"));
    Serial.print(F("# HC-SR04 TRIG/ECHO = D")); Serial.print(PIN_HCSR04_TRIG);
    Serial.print(F("/D")); Serial.println(PIN_HCSR04_ECHO);
    Serial.print(F("# AS5600 addr = 0x")); Serial.println(AS5600_ADDR, HEX);
    Serial.print(F("# POSITION_CALIBRATED = "));
    Serial.println(POSITION_CALIBRATED ? F("YES") : F("NO"));
    Serial.print(F("# D_MIN/D_MAX/setpoint = "));
    Serial.print(D_MIN_CM, 2); Serial.print(F(", "));
    Serial.print(D_MAX_CM, 2); Serial.print(F(", "));
    Serial.println(g_setpoint_cm, 2);
    Serial.print(F("# HC-SR04 accepted calibrated range = ["));
    Serial.print(D_MIN_CM - HCSR04_CAL_MARGIN_CM, 2);
    Serial.print(F(", "));
    Serial.print(D_MAX_CM + HCSR04_CAL_MARGIN_CM, 2);
    Serial.println(F("] cm"));
    Serial.print(F("# HC-SR04 max jump / repeat accept = "));
    Serial.print(HCSR04_MAX_JUMP_CM, 2);
    Serial.print(F(" cm, "));
    Serial.println(HCSR04_JUMP_ACCEPT_COUNT);
    Serial.print(F("# theta = "));
    Serial.print(THETA_SLOPE_DEG_PER_AS_DEG, 8);
    Serial.print(F(" * as5600 + "));
    Serial.println(THETA_OFFSET_DEG, 8);
    Serial.print(F("# theta_balance_deg = ")); Serial.println(g_theta_balance_deg, 5);
    Serial.print(F("# theta cal safe range = ["));
    Serial.print(THETA_CAL_MIN_DEG + THETA_CAL_MARGIN_DEG
                 - THETA_CAL_EXTRAPOLATE_DEG, 2);
    Serial.print(F(", "));
    Serial.print(THETA_CAL_MAX_DEG - THETA_CAL_MARGIN_DEG
                 + THETA_CAL_EXTRAPOLATE_DEG, 2);
    Serial.println(F("] deg (includes extrapolation)"));
    Serial.print(F("# outer Kp/Kv/sign = "));
    Serial.print(g_outer_kp, 4); Serial.print(F(", "));
    Serial.print(g_outer_kv, 4); Serial.print(F(", "));
    Serial.println(g_outer_sign);
    Serial.print(F("# theta command relative limit = "));
    Serial.print(g_theta_cmd_limit_deg, 3);
    Serial.println(F(" deg"));
    Serial.print(F("# inner Kp/steps_per_beam_deg/step_sign = "));
    Serial.print(INNER_KP, 3); Serial.print(F(", "));
    Serial.print(STEPS_PER_BEAM_DEG, 2); Serial.print(F(", "));
    Serial.println(INNER_STEP_SIGN);
    Serial.print(F("# mode = M")); Serial.print((int)g_mode);
    Serial.print(F("  driver = ")); Serial.println(g_driver_enabled ? F("ENABLED") : F("DISABLED"));
    Serial.print(F("# telemetry stream = "));
    Serial.println(g_telemetry_stream ? F("ON") : F("OFF"));
    Serial.println(F("# Commands: G X M0 M1 M2 A<deg> Z S<cm> Y P<float> V<float> L<deg> R H O T C ?"));
    Serial.println(F("# ------------------------------------------------"));
}

void print_telemetry(bool valid, float d_raw_print, uint32_t echo_us, int32_t step_delta) {
    Serial.print(millis()); Serial.print(',');
    Serial.print((int)g_mode); Serial.print(',');
    Serial.print(valid ? 1 : 0); Serial.print(',');
    Serial.print(echo_us); Serial.print(',');
    Serial.print(d_raw_print, 2); Serial.print(',');
    Serial.print(g_d_filt_cm, 2); Serial.print(',');
    Serial.print(g_x_cm, 3); Serial.print(',');
    Serial.print(g_x_dot_cm_s, 3); Serial.print(',');
    Serial.print(g_theta_cal_deg, 4); Serial.print(',');
    Serial.print(g_theta_rel_deg, 4); Serial.print(',');
    Serial.print(g_theta_dot_deg_s, 3); Serial.print(',');
    Serial.print(g_theta_cmd_rel_deg, 4); Serial.print(',');
    Serial.print(g_outer_sign); Serial.print(',');
    Serial.print(g_driver_enabled ? 1 : 0); Serial.print(',');
    Serial.print(step_delta); Serial.print(',');
    Serial.println(g_invalid_count);
}
