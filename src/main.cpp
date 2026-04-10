/*
 * ================================================================
 *  Ball-and-Beam Staged Controller
 * ================================================================
 *  Platform  : Arduino Nano (ATmega328P, 16 MHz)
 *  Ball pos  : Sharp GP2Y0A41SK0F IR sensor   OUT->A0
 *  Beam angle: AS5600 on I2C                  SDA->A4  SCL->A5  0x36
 *  Actuator  : NEMA17 stepper + TMC2209       STEP->D2  DIR->D3  EN->D4
 *
 *  This keeps the staged cascaded design:
 *
 *    M0: sensor/state readout only, no control output
 *    M1: manual beam-angle hold using AS5600 inner loop only
 *    M2: ball-position cascade, gated by POSITION_CALIBRATED
 *
 *  The boot mode is always M0 and the motor driver remains disabled
 *  until the host sends G. M2 is intentionally disabled until the
 *  Sharp geometry is measured and POSITION_CALIBRATED is set true.
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>

// ================================================================
//  SECTION 1 - PIN ASSIGNMENTS
// ================================================================
static const uint8_t PIN_STEP  = 2;
static const uint8_t PIN_DIR   = 3;
static const uint8_t PIN_EN    = 4;   // TMC2209 ENN, active-LOW
static const uint8_t PIN_SHARP = A0;


// ================================================================
//  SECTION 2 - SHARP POSITION SENSOR
// ================================================================
static const float    SHARP_VREF                = 5.0f;
static const float    SHARP_V_PER_COUNT         = SHARP_VREF / 1023.0f;
static const uint8_t  SHARP_ADC_SAMPLES         = 8;
static const float    SHARP_FIT_K_V_CM          = 12.25f;
static const float    SHARP_FIT_OFFSET_CM       = -0.62f;
static const float    SHARP_MIN_VALID_V         = 0.08f;
static const uint8_t  DIST_INVALID_LIMIT        = 8;

// The near-stop obstruction keeps the ball out of the sub-4.3 cm
// foldback region, so D_MIN_CM must stay below the chosen stable
// near-stop reading instead of the average of the near-stop samples.
static const bool     POSITION_CALIBRATED       = true;
static const float    D_MIN_CM                  = 4.40f;
static const float    D_MAX_CM                  = 12.66f;
static const float    D_SETPOINT_MIDPOINT_CM    = 0.5f * (D_MIN_CM + D_MAX_CM);
static const float    D_SETPOINT_TRIM_DEFAULT_CM = 0.00f;
static const float    D_SETPOINT_DEFAULT_CM     =
    D_SETPOINT_MIDPOINT_CM + D_SETPOINT_TRIM_DEFAULT_CM;

static const float    DIST_EMA_ALPHA            = 0.25f;
static const float    X_DOT_EMA_ALPHA           = 0.12f;


// ================================================================
//  SECTION 3 - AS5600 BEAM ANGLE
// ================================================================
static const uint8_t  AS5600_ADDR               = 0x36;
static const uint8_t  AS5600_REG_RAW_ANGLE_HI   = 0x0C;
static const float    AS5600_RAW_TO_DEG         = 360.0f / 4096.0f;

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

static const float    INNER_KP_THETA             = 0.70f;
static const float    INNER_KD_THETA             = 0.000f;
static const float    INNER_THETA_DEADBAND_DEG   = 0.040f;
static const float    INNER_THETA_RATE_DEADBAND_DEG_S = 0.60f;
static const int32_t  INNER_MAX_STEP_DELTA       = 40;
static const uint32_t STEP_PERIOD_US             = 200;
static const uint32_t STEP_START_PERIOD_US       = 900;
static const uint16_t STEP_RAMP_STEPS            = 16;
static const uint32_t STEP_PULSE_US              = 5;
static const uint32_t DIR_SETUP_US               = 4;
static const uint32_t DIR_REVERSE_SETTLE_US      = 1200;


// ================================================================
//  SECTION 5 - OUTER LOOP AND TIMING
// ================================================================
// The GP2Y0A41SK0F updates much faster than HC-SR04. Use the earlier
// 40 ms cadence so the staged controller stays responsive without
// changing the stepper timing budget.
static const uint32_t LOOP_MS                    = 40;
static const float    DT                         = 0.040f;
static const uint8_t  PRINT_EVERY_DEFAULT        = 5;   // T stream: about 5 rows per second
static const uint8_t  PRINT_EVERY_MIN            = 1;
static const uint8_t  PRINT_EVERY_MAX            = 50;
static const float    THETA_CMD_LIMIT_DEFAULT_DEG = 1.50f;

static const float    OUTER_KX_DEFAULT           = 0.34f;
static const float    OUTER_KV_DEFAULT           = 0.10f;
static const float    OUTER_KT_DEFAULT           = 0.00f;
static const float    OUTER_KW_DEFAULT           = 0.00f;
static const float    OUTER_KI_DEFAULT           = 0.030f;
static const float    OUTER_INTEGRAL_CLAMP_DEG   = 0.20f;
static const float    OUTER_INTEGRAL_CAPTURE_CM  = 2.20f;
static const float    OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S = 0.50f;
// These bleed factors are applied on every 40 ms control cycle.
static const float    OUTER_INTEGRAL_BLEED_OUTSIDE = 0.995f;
static const float    OUTER_INTEGRAL_BLEED_RECOVERY = 0.998f;
static const float    OUTER_INTEGRAL_BLEED_CENTER  = 0.992f;
static const float    OUTER_INTEGRAL_BLEED_WRONG_SIGN = 0.80f;
static const float    OUTER_CENTER_BAND_CM       = 0.18f;
static const float    OUTER_CENTER_BAND_X_DOT_CM_S = 0.35f;
static const float    OUTER_X_DOT_LIMIT_CM_S     = 3.0f;
static const float    OUTER_THETA_DOT_LIMIT_DEG_S = 4.0f;
static const float    OUTER_GAIN_SCALE_MIN       = 0.50f;
static const float    OUTER_GAIN_SCALE_MAX       = 1.50f;
static const float    RECOVERY_ENTER_X_CM        = 1.50f;
static const float    RECOVERY_ENTER_X_DOT_CM_S  = 0.60f;
static const uint8_t  RECOVERY_ENTER_COUNT       = 6;
static const float    RECOVERY_EXIT_X_CM         = 0.45f;
static const float    RECOVERY_EXIT_INWARD_X_DOT_CM_S = 0.90f;
static const float    RECOVERY_FLOOR_DEFAULT_DEG = 0.95f;
static const float    RECOVERY_FLOOR_MIN_DEG     = 0.00f;
static const float    RECOVERY_FLOOR_MAX_DEG     = 1.25f;
static const float    RECOVERY_FLOOR_GAIN_DEG_PER_CM = 0.16f;
static const float    ZERO_TRIM_EST_X_CM         = 0.25f;
static const float    ZERO_TRIM_EST_X_DOT_CM_S   = 0.25f;
static const float    ZERO_TRIM_EST_THETA_CMD_DEG = 0.12f;
static const float    ZERO_TRIM_EST_THETA_DOT_DEG_S = 0.40f;
static const float    ZERO_TRIM_EST_ALPHA        = 0.05f;

static float g_theta_cmd_limit_deg               = THETA_CMD_LIMIT_DEFAULT_DEG;
static float g_outer_gain_scale                  = 1.00f;

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
    uint16_t adc_mean;
    float volts;
};

static ControllerMode g_mode                = MODE_TELEMETRY;
static bool           g_driver_enabled      = false;
static bool           g_cal_mode            = false;
static bool           g_telemetry_stream    = false;
static bool           g_print_once          = false;
static uint8_t        g_print_every         = PRINT_EVERY_DEFAULT;
static uint32_t       g_last_ms             = 0;

static float          g_distance_window[3]  = {D_SETPOINT_DEFAULT_CM,
                                               D_SETPOINT_DEFAULT_CM,
                                               D_SETPOINT_DEFAULT_CM};
static uint8_t        g_distance_index      = 0;
static uint8_t        g_distance_count      = 0;
static bool           g_distance_seeded     = false;
static float          g_d_raw_cm            = D_SETPOINT_DEFAULT_CM;
static float          g_d_filt_cm           = D_SETPOINT_DEFAULT_CM;
static bool           g_distance_accepted   = false;
static float          g_setpoint_cm         = D_SETPOINT_DEFAULT_CM;
static float          g_x_cm                = 0.0f;
static float          g_prev_x_cm           = 0.0f;
static float          g_x_dot_cm_s          = 0.0f;
static uint8_t        g_invalid_count       = 0;

static bool           g_unwrap_seeded       = false;
static float          g_prev_wrapped_deg    = 0.0f;
static float          g_prev_unwrapped_deg  = 0.0f;
static uint16_t       g_as5600_raw          = 0;
static float          g_as5600_wrapped_deg  = 0.0f;
static float          g_as5600_unwrapped_deg = 0.0f;
static float          g_theta_cal_deg       = 0.0f;
static float          g_theta_balance_deg   = 0.0f;
static bool           g_theta_balance_set   = false;
static float          g_theta_rel_deg       = 0.0f;
static float          g_prev_theta_rel_deg  = 0.0f;
static float          g_theta_dot_deg_s     = 0.0f;
static bool           g_as5600_valid        = false;

static float          g_manual_theta_cmd_rel_deg = 0.0f;
static float          g_theta_fs_cmd_deg         = 0.0f;
static float          g_theta_cmd_rel_deg        = 0.0f;
static float          g_outer_xi_cm_s            = 0.0f;
static bool           g_recovery_active          = false;
static uint8_t        g_recovery_enter_counter   = 0;
static float          g_recovery_floor_deg       = RECOVERY_FLOOR_DEFAULT_DEG;
static float          g_recovery_floor_active_deg = RECOVERY_FLOOR_DEFAULT_DEG;
static bool           g_zero_trim_est_valid      = false;
static float          g_zero_trim_est_deg        = 0.0f;
static uint16_t       g_zero_trim_est_count      = 0;
static float          g_inner_step_residual      = 0.0f;
static int32_t        g_step_pos                 = 0;
static int32_t        g_last_step_delta          = 0;


// ================================================================
//  SECTION 7 - PROTOTYPES
// ================================================================
DistanceSample read_sharp_cm(void);
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
float compute_full_state_theta_command(bool distance_valid);
int32_t compute_inner_step_delta(float theta_cmd_rel_deg);
void reset_zero_trim_estimator(void);
void update_zero_trim_estimator(bool distance_valid, bool theta_valid);
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
void print_telemetry(bool valid,
                     uint16_t adc_mean,
                     float adc_volts,
                     float d_raw_print,
                     int32_t step_delta);


// ================================================================
//  setup()
// ================================================================
void setup() {
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_SHARP, INPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_EN, HIGH);

    analogReference(DEFAULT);
    for (uint8_t i = 0; i < 5; i++) {
        analogRead(PIN_SHARP);
    }

    Serial.begin(115200);
    Wire.begin();

    DistanceSample initial_distance = read_sharp_cm();
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

    DistanceSample distance = read_sharp_cm();
    float d_raw_print = distance.valid ? distance.cm : -1.0f;
    update_distance_filter(distance);
    bool distance_ok = distance.valid && g_distance_accepted;

    bool theta_valid = update_as5600();
    update_derived_states(distance_ok, theta_valid);

    bool cascade_sensor_fault =
        (g_mode == MODE_CASCADE) && (g_invalid_count >= DIST_INVALID_LIMIT);
    if (!theta_valid || cascade_sensor_fault) {
        reset_dynamic_state();
    }

    g_theta_fs_cmd_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    int32_t step_delta = 0;

    if (!g_cal_mode) {
        if (g_mode == MODE_MANUAL_ANGLE) {
            g_theta_fs_cmd_deg = g_manual_theta_cmd_rel_deg;
            g_theta_cmd_rel_deg = clamp_theta_rel_command(g_manual_theta_cmd_rel_deg);
            step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
        } else if (g_mode == MODE_CASCADE) {
            if (POSITION_CALIBRATED && g_theta_balance_set
                    && g_invalid_count < DIST_INVALID_LIMIT && theta_valid) {
                g_theta_cmd_rel_deg = compute_full_state_theta_command(distance_ok);
            } else {
                g_theta_fs_cmd_deg = 0.0f;
                g_theta_cmd_rel_deg = clamp_theta_rel_command(0.0f);
            }
            step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
        } else {
            g_theta_fs_cmd_deg = 0.0f;
        }
    }

    if (!g_driver_enabled || !theta_valid || g_mode == MODE_TELEMETRY || g_cal_mode
            || cascade_sensor_fault) {
        step_delta = 0;
    }

    update_zero_trim_estimator(distance_ok, theta_valid);

    if (step_delta != 0) {
        step_relative(step_delta);
    }
    g_last_step_delta = step_delta;

    static uint8_t print_count = 0;
    bool valid = distance_ok && theta_valid;
    if (g_print_once || (g_telemetry_stream && (++print_count >= g_print_every))) {
        print_count = 0;
        g_print_once = false;
        print_telemetry(valid,
                        distance.adc_mean,
                        distance.volts,
                        d_raw_print,
                        step_delta);
    }
}


// ================================================================
//  Sharp helpers
// ================================================================
DistanceSample read_sharp_cm() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SHARP_ADC_SAMPLES; i++) {
        sum += (uint32_t)analogRead(PIN_SHARP);
    }

    uint16_t adc_mean = (uint16_t)((sum + (SHARP_ADC_SAMPLES / 2))
                                 / SHARP_ADC_SAMPLES);
    float volts = (float)adc_mean * SHARP_V_PER_COUNT;

    if (volts < SHARP_MIN_VALID_V) {
        DistanceSample sample = {false, 0.0f, adc_mean, volts};
        return sample;
    }

    float cm = (SHARP_FIT_K_V_CM / volts) + SHARP_FIT_OFFSET_CM;
    bool valid = (cm >= D_MIN_CM) && (cm <= D_MAX_CM);

    DistanceSample sample = {valid, cm, adc_mean, volts};
    return sample;
}

float update_distance_filter(const DistanceSample &sample) {
    g_distance_accepted = false;

    if (sample.valid) {
        accept_distance_sample(sample.cm);
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
        uint8_t idx = (g_distance_index == 0)
                    ? 2
                    : (uint8_t)(g_distance_index - 1);
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

float compute_full_state_theta_command(bool distance_valid) {
    float gain_scale = g_outer_gain_scale;
    float kx = OUTER_KX_DEFAULT * gain_scale;
    float kv = OUTER_KV_DEFAULT * gain_scale;
    float kt = OUTER_KT_DEFAULT * gain_scale;
    float kw = OUTER_KW_DEFAULT * gain_scale;
    float ki = OUTER_KI_DEFAULT * gain_scale;
    float x_dot_ctrl = clamp_float(g_x_dot_cm_s,
                                   -OUTER_X_DOT_LIMIT_CM_S,
                                    OUTER_X_DOT_LIMIT_CM_S);
    float theta_dot_ctrl = clamp_float(g_theta_dot_deg_s,
                                       -OUTER_THETA_DOT_LIMIT_DEG_S,
                                        OUTER_THETA_DOT_LIMIT_DEG_S);
    float abs_x_cm = fabs(g_x_cm);
    float abs_x_dot_cm_s = fabs(x_dot_ctrl);
    bool moving_inward = distance_valid && ((g_x_cm * x_dot_ctrl) < 0.0f);
    bool in_capture_band = abs_x_cm <= OUTER_INTEGRAL_CAPTURE_CM
                        && abs_x_dot_cm_s <= OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S;
    bool in_center_band = abs_x_cm <= OUTER_CENTER_BAND_CM
                       && abs_x_dot_cm_s <= OUTER_CENTER_BAND_X_DOT_CM_S;
    bool stalled_off_center = distance_valid
                           && abs_x_cm >= RECOVERY_ENTER_X_CM
                           && abs_x_dot_cm_s <= RECOVERY_ENTER_X_DOT_CM_S;

    if (g_recovery_active) {
        if (!distance_valid
                || abs_x_cm <= RECOVERY_EXIT_X_CM
                || (moving_inward
                    && abs_x_dot_cm_s >= RECOVERY_EXIT_INWARD_X_DOT_CM_S)) {
            g_recovery_active = false;
            g_recovery_enter_counter = 0;
        }
    } else if (stalled_off_center) {
        if (g_recovery_enter_counter < 255) {
            g_recovery_enter_counter++;
        }
        if (g_recovery_enter_counter >= RECOVERY_ENTER_COUNT) {
            g_recovery_active = true;
        }
    } else {
        g_recovery_enter_counter = 0;
    }

    float xi_limit_cm_s = 0.0f;
    if (ki > 0.0001f) {
        xi_limit_cm_s = OUTER_INTEGRAL_CLAMP_DEG / ki;
    }
    if (!distance_valid || !in_capture_band) {
        g_outer_xi_cm_s *= OUTER_INTEGRAL_BLEED_OUTSIDE;
    } else if (g_recovery_active) {
        g_outer_xi_cm_s *= OUTER_INTEGRAL_BLEED_RECOVERY;
    }
    if (g_x_cm != 0.0f && g_outer_xi_cm_s != 0.0f
            && (g_x_cm * g_outer_xi_cm_s) < 0.0f) {
        g_outer_xi_cm_s *= OUTER_INTEGRAL_BLEED_WRONG_SIGN;
    }
    if (in_center_band) {
        g_outer_xi_cm_s *= OUTER_INTEGRAL_BLEED_CENTER;
    }
    if (xi_limit_cm_s > 0.0f) {
        g_outer_xi_cm_s = clamp_float(g_outer_xi_cm_s,
                                      -xi_limit_cm_s,
                                       xi_limit_cm_s);
    }

    float i_term_deg = clamp_float(ki * g_outer_xi_cm_s,
                                   -OUTER_INTEGRAL_CLAMP_DEG,
                                    OUTER_INTEGRAL_CLAMP_DEG);
    float state_sum = (kx * g_x_cm)
                    + (kv * x_dot_ctrl)
                    + (kt * g_theta_rel_deg)
                    + (kw * theta_dot_ctrl)
                    + i_term_deg;
    float fs_cmd_deg = (float)g_outer_sign * state_sum;
    g_theta_fs_cmd_deg = fs_cmd_deg;

    float limited_cmd_deg = clamp_float(fs_cmd_deg,
                                        -g_theta_cmd_limit_deg,
                                         g_theta_cmd_limit_deg);
    bool clamped = fabs(fs_cmd_deg) > g_theta_cmd_limit_deg;
    bool command_is_corrective = (g_x_cm != 0.0f)
                              && ((g_x_cm * limited_cmd_deg * (float)g_outer_sign) > 0.0f);
    bool can_integrate = distance_valid && in_capture_band
                      && !(clamped && command_is_corrective);

    if (can_integrate) {
        g_outer_xi_cm_s += g_x_cm * DT;
        if (xi_limit_cm_s > 0.0f) {
            g_outer_xi_cm_s = clamp_float(g_outer_xi_cm_s,
                                          -xi_limit_cm_s,
                                           xi_limit_cm_s);
        }
        if (in_center_band) {
            g_outer_xi_cm_s *= OUTER_INTEGRAL_BLEED_CENTER;
        }
        i_term_deg = clamp_float(ki * g_outer_xi_cm_s,
                                 -OUTER_INTEGRAL_CLAMP_DEG,
                                  OUTER_INTEGRAL_CLAMP_DEG);
        state_sum = (kx * g_x_cm)
                  + (kv * x_dot_ctrl)
                  + (kt * g_theta_rel_deg)
                  + (kw * theta_dot_ctrl)
                  + i_term_deg;
        fs_cmd_deg = (float)g_outer_sign * state_sum;
        g_theta_fs_cmd_deg = fs_cmd_deg;
        limited_cmd_deg = clamp_float(fs_cmd_deg,
                                      -g_theta_cmd_limit_deg,
                                       g_theta_cmd_limit_deg);
    }

    g_recovery_floor_active_deg = g_recovery_floor_deg;
    if (g_recovery_active) {
        float dynamic_floor_deg = g_recovery_floor_deg
                                + (RECOVERY_FLOOR_GAIN_DEG_PER_CM
                                   * clamp_float(abs_x_cm - RECOVERY_ENTER_X_CM,
                                                 0.0f,
                                                 10.0f));
        g_recovery_floor_active_deg = clamp_float(dynamic_floor_deg,
                                                  RECOVERY_FLOOR_MIN_DEG,
                                                  RECOVERY_FLOOR_MAX_DEG);
    }
    if (g_recovery_active && g_recovery_floor_deg > 0.0f && g_x_cm != 0.0f) {
        float recovery_cmd_deg = (g_x_cm > 0.0f)
                               ? -g_recovery_floor_active_deg
                               :  g_recovery_floor_active_deg;
        recovery_cmd_deg *= (float)(-g_outer_sign);
        if (fabs(limited_cmd_deg) < g_recovery_floor_active_deg) {
            limited_cmd_deg = recovery_cmd_deg;
        }
    }

    return clamp_theta_rel_command(limited_cmd_deg);
}

void reset_zero_trim_estimator() {
    g_zero_trim_est_valid = false;
    g_zero_trim_est_deg = 0.0f;
    g_zero_trim_est_count = 0;
}

void update_zero_trim_estimator(bool distance_valid, bool theta_valid) {
    bool can_sample = (g_mode == MODE_CASCADE)
                   && g_driver_enabled
                   && distance_valid
                   && theta_valid
                   && g_theta_balance_set
                   && !g_recovery_active
                   && (fabs(g_x_cm) <= ZERO_TRIM_EST_X_CM)
                   && (fabs(g_x_dot_cm_s) <= ZERO_TRIM_EST_X_DOT_CM_S)
                   && (fabs(g_theta_cmd_rel_deg) <= ZERO_TRIM_EST_THETA_CMD_DEG)
                   && (fabs(g_theta_dot_deg_s) <= ZERO_TRIM_EST_THETA_DOT_DEG_S);
    if (!can_sample) {
        return;
    }

    if (!g_zero_trim_est_valid) {
        g_zero_trim_est_valid = true;
        g_zero_trim_est_deg = g_theta_rel_deg;
        g_zero_trim_est_count = 1;
        return;
    }

    g_zero_trim_est_deg += ZERO_TRIM_EST_ALPHA
                        * (g_theta_rel_deg - g_zero_trim_est_deg);
    if (g_zero_trim_est_count < 65535) {
        g_zero_trim_est_count++;
    }
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
    if (fabs(theta_error_deg) <= INNER_THETA_DEADBAND_DEG
            && fabs(g_theta_dot_deg_s) <= INNER_THETA_RATE_DEADBAND_DEG_S) {
        g_inner_step_residual = 0.0f;
        return 0;
    }
    float raw_steps = ((float)INNER_STEP_SIGN
                    * ((INNER_KP_THETA * theta_error_deg)
                    - (INNER_KD_THETA * g_theta_dot_deg_s))
                    * STEPS_PER_BEAM_DEG)
                    + g_inner_step_residual;
    int32_t rounded_steps = round_to_i32(raw_steps);
    int32_t step_delta = clamp_i32(rounded_steps,
                                   -INNER_MAX_STEP_DELTA,
                                    INNER_MAX_STEP_DELTA);
    if (step_delta != rounded_steps) {
        g_inner_step_residual = 0.0f;
    } else {
        g_inner_step_residual = raw_steps - (float)step_delta;
    }
    return step_delta;
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
    if (!enabled) {
        reset_dynamic_state();
    }
}


// ================================================================
//  Serial command handling
// ================================================================
void set_mode(ControllerMode mode) {
    if (mode == MODE_CASCADE && !POSITION_CALIBRATED) {
        Serial.println(F("# ERR: M2 disabled until POSITION_CALIBRATED is true"));
        return;
    }
    if (mode == MODE_CASCADE && !g_theta_balance_set) {
        Serial.println(F("# ERR: M2 requires Z first to set theta balance"));
        return;
    }

    g_mode = mode;
    g_cal_mode = false;
    if (g_mode == MODE_MANUAL_ANGLE) {
        g_manual_theta_cmd_rel_deg = clamp_theta_rel_command(g_theta_rel_deg);
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
    g_theta_fs_cmd_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    g_outer_xi_cm_s = 0.0f;
    g_recovery_active = false;
    g_recovery_enter_counter = 0;
    g_recovery_floor_active_deg = g_recovery_floor_deg;
    g_inner_step_residual = 0.0f;
    g_last_step_delta = 0;
}

void reset_distance_filter() {
    g_distance_index = 0;
    g_distance_count = 0;
    g_distance_seeded = false;
    g_distance_accepted = false;
    g_invalid_count = 0;
    g_d_raw_cm = g_setpoint_cm;
    g_d_filt_cm = g_setpoint_cm;
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
            g_theta_balance_set = true;
            g_manual_theta_cmd_rel_deg = 0.0f;
            g_theta_cmd_rel_deg = 0.0f;
            reset_zero_trim_estimator();
            reset_dynamic_state();
            Serial.print(F("# theta_balance_deg -> "));
            Serial.println(g_theta_balance_deg, 5);
            Serial.println(F("# Manual theta command reset to A0.0"));
        }

    } else if (cmd == 'S') {
        float sp = atof(buf + 1);
        if (sp >= D_MIN_CM && sp <= D_MAX_CM) {
            g_setpoint_cm = sp;
            reset_zero_trim_estimator();
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
        Serial.println(F("# P<float> deprecated in M2; use K<float> for gain scale and J<deg> for recovery floor"));

    } else if (cmd == 'V') {
        Serial.println(F("# V<float> deprecated in M2; use K<float> for gain scale and J<deg> for recovery floor"));

    } else if (cmd == 'K') {
        float scale = atof(buf + 1);
        g_outer_gain_scale = clamp_float(scale,
                                         OUTER_GAIN_SCALE_MIN,
                                         OUTER_GAIN_SCALE_MAX);
        Serial.print(F("# full-state gain scale -> "));
        Serial.println(g_outer_gain_scale, 3);

    } else if (cmd == 'J') {
        float floor_deg = atof(buf + 1);
        g_recovery_floor_deg = clamp_float(floor_deg,
                                           RECOVERY_FLOOR_MIN_DEG,
                                           RECOVERY_FLOOR_MAX_DEG);
        Serial.print(F("# recovery floor -> "));
        Serial.print(g_recovery_floor_deg, 3);
        Serial.println(F(" deg"));

    } else if (cmd == 'L') {
        float limit = atof(buf + 1);
        g_theta_cmd_limit_deg = clamp_float(limit, 0.10f, 3.00f);
        Serial.print(F("# theta command limit -> "));
        Serial.print(g_theta_cmd_limit_deg, 3);
        Serial.println(F(" deg rel"));

    } else if (cmd == 'N') {
        int decimation = atoi(buf + 1);
        if (decimation >= (int)PRINT_EVERY_MIN
                && decimation <= (int)PRINT_EVERY_MAX) {
            g_print_every = (uint8_t)decimation;
            Serial.print(F("# telemetry decimation -> every "));
            Serial.print(g_print_every);
            Serial.print(F(" loops (~"));
            Serial.print(1000.0f / ((float)LOOP_MS * (float)g_print_every), 2);
            Serial.println(F(" Hz)"));
        } else {
            Serial.print(F("# ERR: telemetry decimation must be in ["));
            Serial.print(PRINT_EVERY_MIN);
            Serial.print(F(", "));
            Serial.print(PRINT_EVERY_MAX);
            Serial.println(F("] loops"));
        }

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
        Serial.print(F("# Sharp calibration telemetry "));
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
    Serial.println(F("t_ms,mode,valid,adc_mean,adc_volts,d_raw_cm,d_filt_cm,"
                     "x_cm,x_dot_cm_s,theta_cal_deg,theta_rel_deg,"
                     "theta_dot_deg_s,theta_fs_cmd_deg,theta_cmd_rel_deg,"
                     "xi_cm_s,recovery_active,recovery_floor_deg,gain_scale,"
                     "zero_trim_est_deg,zero_trim_est_count,"
                     "theta_balance_set,outer_sign,driver_enabled,step_delta,"
                     "invalid_count"));
}

void print_config() {
    Serial.println(F("# -- Ball-and-Beam Sharp GP2Y0A41SK0F + AS5600 Controller --"));
    Serial.print(F("# Sharp input = A0"));
    Serial.println();
    Serial.print(F("# Sharp ADC samples = "));
    Serial.println(SHARP_ADC_SAMPLES);
    Serial.print(F("# Sharp fit d_cm = "));
    Serial.print(SHARP_FIT_K_V_CM, 3);
    Serial.print(F(" / V + "));
    Serial.println(SHARP_FIT_OFFSET_CM, 3);
    Serial.print(F("# Sharp minimum valid voltage = "));
    Serial.print(SHARP_MIN_VALID_V, 3);
    Serial.println(F(" V"));
    Serial.print(F("# POSITION_CALIBRATED = "));
    Serial.println(POSITION_CALIBRATED ? F("YES") : F("NO"));
    Serial.print(F("# D_MIN/D_MAX/setpoint = "));
    Serial.print(D_MIN_CM, 2); Serial.print(F(", "));
    Serial.print(D_MAX_CM, 2); Serial.print(F(", "));
    Serial.println(g_setpoint_cm, 2);
    Serial.print(F("# Setpoint midpoint/trim = "));
    Serial.print(D_SETPOINT_MIDPOINT_CM, 2); Serial.print(F(", "));
    Serial.println(D_SETPOINT_TRIM_DEFAULT_CM, 2);
    Serial.print(F("# Loop period = "));
    Serial.print(LOOP_MS);
    Serial.println(F(" ms"));
    Serial.print(F("# AS5600 addr = 0x"));
    Serial.println(AS5600_ADDR, HEX);
    Serial.print(F("# theta = "));
    Serial.print(THETA_SLOPE_DEG_PER_AS_DEG, 8);
    Serial.print(F(" * as5600 + "));
    Serial.println(THETA_OFFSET_DEG, 8);
    Serial.print(F("# theta_balance_deg = "));
    Serial.println(g_theta_balance_deg, 5);
    Serial.print(F("# theta_balance_set = "));
    Serial.println(g_theta_balance_set ? F("YES") : F("NO"));
    Serial.print(F("# theta cal safe range = ["));
    Serial.print(THETA_CAL_MIN_DEG + THETA_CAL_MARGIN_DEG
                 - THETA_CAL_EXTRAPOLATE_DEG, 2);
    Serial.print(F(", "));
    Serial.print(THETA_CAL_MAX_DEG - THETA_CAL_MARGIN_DEG
                 + THETA_CAL_EXTRAPOLATE_DEG, 2);
    Serial.println(F("] deg (includes extrapolation)"));
    Serial.print(F("# outer Kx/Kv/Kt/Kw/Ki/sign = "));
    Serial.print(OUTER_KX_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KV_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KT_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KW_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KI_DEFAULT, 4); Serial.print(F(", "));
    Serial.println(g_outer_sign);
    Serial.print(F("# full-state gain scale = "));
    Serial.println(g_outer_gain_scale, 3);
    Serial.print(F("# integral output clamp = +/-"));
    Serial.print(OUTER_INTEGRAL_CLAMP_DEG, 3);
    Serial.println(F(" deg"));
    Serial.print(F("# integral capture |x|/|x_dot| <= "));
    Serial.print(OUTER_INTEGRAL_CAPTURE_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# recovery floor base/max/gain = "));
    Serial.print(g_recovery_floor_deg, 3);
    Serial.print(F(", "));
    Serial.print(RECOVERY_FLOOR_MAX_DEG, 3);
    Serial.print(F(", "));
    Serial.print(RECOVERY_FLOOR_GAIN_DEG_PER_CM, 3);
    Serial.println(F(" deg, deg, deg/cm"));
    Serial.print(F("# recovery enter |x|/|x_dot| >=/<="));
    Serial.print(RECOVERY_ENTER_X_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(RECOVERY_ENTER_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# recovery exit |x| or inward |x_dot| <=/>= "));
    Serial.print(RECOVERY_EXIT_X_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(RECOVERY_EXIT_INWARD_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# theta command relative limit = "));
    Serial.print(g_theta_cmd_limit_deg, 3);
    Serial.println(F(" deg"));
    Serial.print(F("# zero-trim estimate = "));
    if (g_zero_trim_est_valid) {
        Serial.print(g_zero_trim_est_deg, 4);
        Serial.print(F(" deg from "));
        Serial.print(g_zero_trim_est_count);
        Serial.println(F(" samples"));
    } else {
        Serial.println(F("N/A"));
    }
    Serial.print(F("# inner Kp/Kd/steps_per_beam_deg/step_sign = "));
    Serial.print(INNER_KP_THETA, 3); Serial.print(F(", "));
    Serial.print(INNER_KD_THETA, 3); Serial.print(F(", "));
    Serial.print(STEPS_PER_BEAM_DEG, 2); Serial.print(F(", "));
    Serial.println(INNER_STEP_SIGN);
    Serial.print(F("# mode = M"));
    Serial.print((int)g_mode);
    Serial.print(F("  driver = "));
    Serial.println(g_driver_enabled ? F("ENABLED") : F("DISABLED"));
    Serial.print(F("# telemetry stream = "));
    Serial.println(g_telemetry_stream ? F("ON") : F("OFF"));
    Serial.print(F("# telemetry decimation = every "));
    Serial.print(g_print_every);
    Serial.print(F(" loops (~"));
    Serial.print(1000.0f / ((float)LOOP_MS * (float)g_print_every), 2);
    Serial.println(F(" Hz)"));
    Serial.println(F("# Commands: G X M0 M1 M2 A<deg> Z S<cm> Y K<float> J<deg> L<deg> N<int> R H O T C ?"));
    Serial.println(F("# ------------------------------------------------"));
}

void print_telemetry(bool valid,
                     uint16_t adc_mean,
                     float adc_volts,
                     float d_raw_print,
                     int32_t step_delta) {
    Serial.print(millis()); Serial.print(',');
    Serial.print((int)g_mode); Serial.print(',');
    Serial.print(valid ? 1 : 0); Serial.print(',');
    Serial.print(adc_mean); Serial.print(',');
    Serial.print(adc_volts, 4); Serial.print(',');
    Serial.print(d_raw_print, 2); Serial.print(',');
    Serial.print(g_d_filt_cm, 2); Serial.print(',');
    Serial.print(g_x_cm, 3); Serial.print(',');
    Serial.print(g_x_dot_cm_s, 3); Serial.print(',');
    Serial.print(g_theta_cal_deg, 4); Serial.print(',');
    Serial.print(g_theta_rel_deg, 4); Serial.print(',');
    Serial.print(g_theta_dot_deg_s, 3); Serial.print(',');
    Serial.print(g_theta_fs_cmd_deg, 4); Serial.print(',');
    Serial.print(g_theta_cmd_rel_deg, 4); Serial.print(',');
    Serial.print(g_outer_xi_cm_s, 4); Serial.print(',');
    Serial.print(g_recovery_active ? 1 : 0); Serial.print(',');
    Serial.print(g_recovery_floor_active_deg, 3); Serial.print(',');
    Serial.print(g_outer_gain_scale, 3); Serial.print(',');
    Serial.print(g_zero_trim_est_valid ? g_zero_trim_est_deg : 0.0f, 4); Serial.print(',');
    Serial.print(g_zero_trim_est_count); Serial.print(',');
    Serial.print(g_theta_balance_set ? 1 : 0); Serial.print(',');
    Serial.print(g_outer_sign); Serial.print(',');
    Serial.print(g_driver_enabled ? 1 : 0); Serial.print(',');
    Serial.print(step_delta); Serial.print(',');
    Serial.println(g_invalid_count);
}
