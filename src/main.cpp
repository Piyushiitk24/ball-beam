/*
 * ================================================================
 *  Ball-and-Beam Staged Controller
 * ================================================================
 *  Platform  : Arduino Nano (ATmega328P, 16 MHz)
 *  Ball pos  : Sharp GP2Y0A41SK0F IR sensor   OUT->A0
 *  Beam angle: AS5600 on I2C                  SDA->A4  SCL->A5  0x36
 *  Actuator  : NEMA17 stepper + TMC2209       STEP->D2  DIR->D3  EN->D4
 *
 *  Controller revision 2 keeps the staged interface:
 *
 *    M0: sensor/state readout only
 *    M1: manual beam-angle hold using the inner angle servo
 *    M2: outer LQI position hold + inner angle servo
 *
 *  The earlier recovery, trim-learning, and staircase-specific shaping
 *  logic has been removed for a simpler first stable baseline.
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
static const float    SHARP_VREF                 = 5.0f;
static const float    SHARP_V_PER_COUNT          = SHARP_VREF / 1023.0f;
static const uint8_t  SHARP_ADC_SAMPLES          = 8;
static const float    SHARP_FIT_K_V_CM           = 12.25f;
static const float    SHARP_FIT_OFFSET_CM        = -0.62f;
static const float    SHARP_MIN_VALID_V          = 0.08f;
static const uint8_t  DIST_INVALID_LIMIT         = 8;

// 40 mm ball calibration, 2026-04-15.
static const bool     POSITION_CALIBRATED        = true;
static const float    D_MIN_CM                   = 4.60f;
static const float    D_MAX_CM                   = 13.60f;
static const float    D_SETPOINT_MIDPOINT_CM     = 0.5f * (D_MIN_CM + D_MAX_CM);
static const float    D_SETPOINT_TRIM_DEFAULT_CM = 0.90f;
static const float    D_SETPOINT_DEFAULT_CM      =
    D_SETPOINT_MIDPOINT_CM + D_SETPOINT_TRIM_DEFAULT_CM;
static const uint32_t STAIRCASE_STAGE_MS         = 20000UL;
static const uint32_t STAIRCASE_TOTAL_MS         = 3UL * STAIRCASE_STAGE_MS;
static const float    STAIRCASE_FAR_SETPOINT_CM  = 11.40f;
static const float    STAIRCASE_CENTER_SETPOINT_CM = D_SETPOINT_DEFAULT_CM;
static const float    STAIRCASE_NEAR_SETPOINT_CM = 6.40f;

static const float    DIST_EMA_ALPHA             = 0.25f;
static const float    DIST_VEL_EMA_ALPHA         = 0.78f;
static const float    DIST_MEDIAN_STEP_LIMIT_CM  = 0.00f;
static const float    DIST_CTRL_BLEND_BAND_CM    = 0.60f;
static const float    DIST_CTRL_SIGN_MISMATCH_X_CM = 0.35f;
static const float    X_DOT_EMA_ALPHA            = 0.20f;
static const float    X_DOT_EMA_ALPHA_NEAR       = 0.08f;
static const float    X_DOT_NEAR_TARGET_BAND_CM  = 1.20f;


// ================================================================
//  SECTION 3 - AS5600 BEAM ANGLE
// ================================================================
static const uint8_t  AS5600_ADDR                = 0x36;
static const uint8_t  AS5600_REG_RAW_ANGLE_HI    = 0x0C;
static const float    AS5600_RAW_TO_DEG          = 360.0f / 4096.0f;

// From calibration_logs/20260407_122510/beam_angle_calibration_report.md
static const float    THETA_SLOPE_DEG_PER_AS_DEG = 0.07666806f;
static const float    THETA_OFFSET_DEG           = -23.28443907f;
static const float    THETA_BALANCE_FIXED_DEG    = 0.94660f;
static const float    THETA_CAL_MIN_DEG          = -0.70f;
static const float    THETA_CAL_MAX_DEG          = 3.10f;
static const float    THETA_CAL_MARGIN_DEG       = 0.10f;
static const float    THETA_CAL_EXTRAPOLATE_DEG  = 0.30f;
static const float    THETA_DOT_EMA_ALPHA        = 0.30f;


// ================================================================
//  SECTION 4 - CONTROL AND TIMING
// ================================================================
static const uint8_t  CONTROLLER_REV             = 4;
static const uint32_t LOOP_MS                    = 40;
static const float    DT                         = 0.040f;
static const uint8_t  PRINT_EVERY_DEFAULT        = 5;
static const uint8_t  PRINT_EVERY_MIN            = 1;
static const uint8_t  PRINT_EVERY_MAX            = 50;

static const int32_t  STEPS_PER_REV              = 3200;
static const float    STEPS_PER_BEAM_DEG         =
    ((float)STEPS_PER_REV / 360.0f) / THETA_SLOPE_DEG_PER_AS_DEG;
static const int8_t   DIR_SIGN                   = -1;
static const int8_t   INNER_STEP_SIGN            = -1;

static const float    THETA_CMD_LIMIT_DEFAULT_DEG = 2.00f;
static const float    THETA_CMD_LIMIT_MIN_DEG    = 0.10f;
static const float    THETA_CMD_LIMIT_MAX_DEG    = 3.00f;

// Inner angle servo.
static const float    INNER_KP_THETA             = 0.95f;
static const float    INNER_KI_THETA             = 0.45f;
static const float    INNER_KD_THETA             = 0.06f;
static const float    INNER_INTEGRAL_CLAMP_DEG   = 0.20f;
static const float    INNER_REF_MAX_RATE_DEG_S   = 12.0f;
static const float    INNER_THETA_DEADBAND_DEG   = 0.030f;
static const float    INNER_THETA_RATE_DEADBAND_DEG_S = 0.45f;
static const int32_t  INNER_MAX_STEP_DELTA       = 45;
static const uint32_t STEP_PERIOD_US             = 200;
static const uint32_t STEP_START_PERIOD_US       = 900;
static const uint16_t STEP_RAMP_STEPS            = 16;
static const uint32_t STEP_PULSE_US              = 5;
static const uint32_t DIR_SETUP_US               = 4;
static const uint32_t DIR_REVERSE_SETTLE_US      = 1200;

// Outer LQI position loop with softer center behavior and stronger escape braking.
static const float    OUTER_LQI_KX_DEFAULT       = 0.56f;
static const float    OUTER_LQI_KV_DEFAULT       = 0.18f;
static const float    OUTER_LQI_KI_DEFAULT       = 0.12f;
static const float    OUTER_INTEGRAL_CLAMP_DEG   = 0.28f;
static const float    OUTER_INTEGRATE_X_CM       = 0.90f;
static const float    OUTER_INTEGRATE_X_DOT_CM_S = 1.30f;
static const float    OUTER_X_DOT_LIMIT_CM_S     = 5.00f;
static const float    OUTER_X_DEADBAND_CM        = 0.10f;
static const float    OUTER_X_DOT_DEADBAND_CM_S  = 0.25f;
static const float    OUTER_HOLD_X_CM            = 0.18f;
static const float    OUTER_HOLD_X_DOT_CM_S      = 0.28f;
static const float    OUTER_HOLD_XI_DECAY        = 0.95f;
static const float    OUTER_GAIN_BOOST_X_CM      = 0.80f;
static const float    OUTER_GAIN_BOOST_FULL_X_CM = 2.20f;
static const float    OUTER_GAIN_BOOST_X_DOT_CM_S = 1.40f;
static const float    OUTER_GAIN_BOOST_FULL_X_DOT_CM_S = 4.50f;
static const float    OUTER_GAIN_BOOST_KX_SCALE  = 1.15f;
static const float    OUTER_GAIN_BOOST_KV_SCALE  = 1.70f;
static const float    OUTER_GAIN_BOOST_KI_SCALE  = 1.10f;
static const float    OUTER_OUTWARD_BRAKE_X_CM   = 0.45f;
static const float    OUTER_OUTWARD_BRAKE_X_DOT_CM_S = 0.90f;
static const float    OUTER_OUTWARD_KV_EXTRA     = 0.18f;
static const uint8_t  DIST_CTRL_INVALID_HOLD_SAMPLES = 2;
static const float    DIST_CTRL_INVALID_PREDICT_STEP_CM = 0.35f;
static const float    DIST_CTRL_INVALID_X_DOT_DECAY = 0.92f;
static const float    OUTER_GAIN_SCALE_MIN       = 0.50f;
static const float    OUTER_GAIN_SCALE_MAX       = 1.50f;


// ================================================================
//  SECTION 5 - STATE
// ================================================================
enum ControllerMode : uint8_t {
    MODE_TELEMETRY = 0,
    MODE_MANUAL_ANGLE = 1,
    MODE_CASCADE = 2,
};

enum StaircasePhase : uint8_t {
    STAIRCASE_PHASE_FAR = 0,
    STAIRCASE_PHASE_CENTER = 1,
    STAIRCASE_PHASE_NEAR = 2,
};

struct DistanceSample {
    bool valid;
    float cm;
    uint16_t adc_mean;
    float volts;
};

static ControllerMode g_mode                 = MODE_TELEMETRY;
static bool           g_driver_enabled       = false;
static bool           g_cal_mode             = false;
static bool           g_telemetry_stream     = false;
static bool           g_print_once           = false;
static uint8_t        g_print_every          = PRINT_EVERY_DEFAULT;
static uint32_t       g_last_ms              = 0;
static float          g_loop_dt_s            = DT;
static bool           g_staircase_active     = false;
static uint32_t       g_staircase_start_ms   = 0;
static StaircasePhase g_staircase_phase      = STAIRCASE_PHASE_FAR;

static float          g_distance_window[3]   = {D_SETPOINT_DEFAULT_CM,
                                                D_SETPOINT_DEFAULT_CM,
                                                D_SETPOINT_DEFAULT_CM};
static uint8_t        g_distance_index       = 0;
static uint8_t        g_distance_count       = 0;
static bool           g_distance_seeded      = false;
static float          g_d_raw_cm             = D_SETPOINT_DEFAULT_CM;
static float          g_d_filt_cm            = D_SETPOINT_DEFAULT_CM;
static float          g_d_vel_cm             = D_SETPOINT_DEFAULT_CM;
static float          g_d_rate_cm_s          = 0.0f;
static bool           g_distance_accepted    = false;
static float          g_setpoint_cm          = D_SETPOINT_DEFAULT_CM;
static float          g_x_cm                 = 0.0f;
static float          g_prev_d_vel_cm        = D_SETPOINT_DEFAULT_CM;
static float          g_x_dot_cm_s           = 0.0f;
static uint8_t        g_invalid_count        = 0;

static bool           g_unwrap_seeded        = false;
static float          g_prev_wrapped_deg     = 0.0f;
static float          g_prev_unwrapped_deg   = 0.0f;
static uint16_t       g_as5600_raw           = 0;
static float          g_as5600_wrapped_deg   = 0.0f;
static float          g_as5600_unwrapped_deg = 0.0f;
static float          g_theta_cal_deg        = 0.0f;
static float          g_theta_balance_deg    = 0.0f;
static bool           g_theta_balance_set    = false;
static float          g_theta_rel_deg        = 0.0f;
static float          g_prev_theta_rel_deg   = 0.0f;
static float          g_theta_dot_deg_s      = 0.0f;
static bool           g_as5600_valid         = false;

static float          g_manual_theta_cmd_rel_deg = 0.0f;
static float          g_theta_fs_cmd_deg         = 0.0f;
static float          g_theta_trim_ff_deg        = 0.0f;
static float          g_theta_cmd_unsat_deg      = 0.0f;
static float          g_theta_cmd_prelimit_deg   = 0.0f;
static float          g_theta_cmd_rel_deg        = 0.0f;
static float          g_theta_err_deg            = 0.0f;
static bool           g_theta_cmd_saturated      = false;
static bool           g_outer_saturated          = false;
static bool           g_inner_saturated          = false;
static bool           g_outer_antiwindup_active  = false;
static bool           g_inner_antiwindup_active  = false;
static bool           g_antiwindup_active        = false;
static float          g_outer_xi_cm_s            = 0.0f;
static float          g_inner_theta_ref_rel_deg  = 0.0f;
static float          g_inner_theta_xi_deg_s     = 0.0f;
static float          g_inner_step_residual      = 0.0f;
static float          g_theta_cmd_limit_deg      = THETA_CMD_LIMIT_DEFAULT_DEG;
static float          g_outer_gain_scale         = 1.00f;
static int8_t         g_outer_sign               = -1;
static int32_t        g_step_pos                 = 0;
static int32_t        g_last_step_delta          = 0;


// ================================================================
//  SECTION 6 - PROTOTYPES
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
float apply_signed_deadband(float value, float deadband);
float ramp_weight(float magnitude, float start, float full);
float near_target_weight(float x_cm);
float clamp_theta_rel_command(float theta_rel_deg);
float compute_outer_theta_command(bool distance_valid);
int32_t compute_inner_step_delta(float theta_target_rel_deg);
void step_relative(int32_t delta_steps);
void set_driver_enabled(bool enabled);
void set_mode(ControllerMode mode);
void reset_dynamic_state(void);
void apply_theta_balance_deg(float theta_balance_deg);
void reset_distance_filter(void);
void apply_setpoint_cm(float sp);
void apply_staircase_setpoint_cm(float sp);
void print_setpoint_announcement(void);
void cancel_staircase(const __FlashStringHelper *reason);
const __FlashStringHelper *staircase_phase_label(StaircasePhase phase);
float staircase_phase_setpoint_cm(StaircasePhase phase);
void set_staircase_phase(StaircasePhase phase);
bool start_staircase(uint32_t now_ms);
void update_staircase(uint32_t now_ms);
bool is_immediate_serial_command(char c);
void execute_serial_command(char *buf);
void parse_serial(void);
void print_header(void);
void print_config(void);
void print_telemetry(bool valid,
                     bool theta_valid,
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
    uint32_t elapsed_ms = now - g_last_ms;
    if (elapsed_ms < LOOP_MS) {
        return;
    }
    g_last_ms = now;
    g_loop_dt_s = 0.001f * (float)elapsed_ms;
    if (g_loop_dt_s < DT) {
        g_loop_dt_s = DT;
    } else if (g_loop_dt_s > (3.0f * DT)) {
        g_loop_dt_s = 3.0f * DT;
    }

    update_staircase(now);

    DistanceSample distance = read_sharp_cm();
    float d_raw_print = distance.valid ? distance.cm : -1.0f;
    update_distance_filter(distance);
    bool distance_ok = distance.valid && g_distance_accepted;
    bool distance_ctrl_ok = distance_ok
                         || (g_distance_seeded
                             && (g_invalid_count > 0)
                             && (g_invalid_count <= DIST_CTRL_INVALID_HOLD_SAMPLES));

    bool theta_valid = update_as5600();
    update_derived_states(distance_ok, theta_valid);

    bool cascade_sensor_fault =
        (g_mode == MODE_CASCADE) && (g_invalid_count >= DIST_INVALID_LIMIT);
    if (!theta_valid || cascade_sensor_fault) {
        reset_dynamic_state();
    }

    g_theta_fs_cmd_deg = 0.0f;
    g_theta_trim_ff_deg = 0.0f;
    g_theta_cmd_unsat_deg = 0.0f;
    g_theta_cmd_prelimit_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    g_theta_err_deg = 0.0f;
    g_theta_cmd_saturated = false;
    g_outer_saturated = false;
    g_inner_saturated = false;
    g_outer_antiwindup_active = false;
    g_inner_antiwindup_active = false;
    g_antiwindup_active = false;
    int32_t step_delta = 0;

    if (!g_cal_mode) {
        if (g_mode == MODE_MANUAL_ANGLE) {
            g_theta_fs_cmd_deg = g_manual_theta_cmd_rel_deg;
            g_theta_cmd_unsat_deg = g_manual_theta_cmd_rel_deg;
            g_theta_cmd_prelimit_deg = g_manual_theta_cmd_rel_deg;
            g_theta_cmd_rel_deg = clamp_theta_rel_command(g_theta_cmd_prelimit_deg);
            g_outer_saturated =
                fabs(g_theta_cmd_rel_deg - g_theta_cmd_prelimit_deg) > 0.0005f;
            g_theta_cmd_saturated = g_outer_saturated;
            if (g_driver_enabled && theta_valid) {
                step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
            }

        } else if (g_mode == MODE_CASCADE) {
            bool cascade_ready = POSITION_CALIBRATED
                              && g_theta_balance_set
                              && g_driver_enabled
                              && theta_valid
                              && !cascade_sensor_fault;
            if (cascade_ready && distance_ctrl_ok) {
                g_theta_cmd_prelimit_deg = compute_outer_theta_command(distance_ok);
            } else {
                g_theta_fs_cmd_deg = 0.0f;
                g_theta_cmd_unsat_deg = 0.0f;
                g_theta_cmd_prelimit_deg = 0.0f;
            }

            g_theta_cmd_rel_deg = clamp_theta_rel_command(g_theta_cmd_prelimit_deg);
            if (fabs(g_theta_cmd_rel_deg - g_theta_cmd_prelimit_deg) > 0.0005f) {
                g_outer_saturated = true;
            }
            g_theta_cmd_saturated = g_outer_saturated;

            if (g_driver_enabled && theta_valid) {
                step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
            }
        }
    }

    if (!g_driver_enabled || !theta_valid || g_mode == MODE_TELEMETRY || g_cal_mode
            || cascade_sensor_fault) {
        step_delta = 0;
    }

    g_antiwindup_active = g_outer_antiwindup_active || g_inner_antiwindup_active;

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
                        theta_valid,
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
        g_d_vel_cm = median_cm;
        g_distance_seeded = true;
    } else {
        if (DIST_MEDIAN_STEP_LIMIT_CM > 0.0f) {
            float delta_cm = median_cm - g_d_vel_cm;
            median_cm = g_d_vel_cm
                      + clamp_float(delta_cm,
                                    -DIST_MEDIAN_STEP_LIMIT_CM,
                                     DIST_MEDIAN_STEP_LIMIT_CM);
        }
        g_d_filt_cm = (DIST_EMA_ALPHA * median_cm)
                    + ((1.0f - DIST_EMA_ALPHA) * g_d_filt_cm);
        g_d_vel_cm = (DIST_VEL_EMA_ALPHA * median_cm)
                   + ((1.0f - DIST_VEL_EMA_ALPHA) * g_d_vel_cm);
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
    float slow_x_now = g_d_filt_cm - g_setpoint_cm;
    float x_now = slow_x_now;
    if (distance_valid && g_distance_seeded) {
        float d_rate = (g_d_vel_cm - g_prev_d_vel_cm) / g_loop_dt_s;
        g_d_rate_cm_s = d_rate;
        float fast_x_now = g_d_vel_cm - g_setpoint_cm;
        if (DIST_CTRL_BLEND_BAND_CM > 0.0f) {
            float blend_weight =
                clamp_float(fabs(fast_x_now - slow_x_now) / DIST_CTRL_BLEND_BAND_CM,
                            0.0f,
                            1.0f);
            bool sign_mismatch =
                (fast_x_now * slow_x_now) < 0.0f
                && fabs(fast_x_now) >= DIST_CTRL_SIGN_MISMATCH_X_CM
                && fabs(slow_x_now) >= DIST_CTRL_SIGN_MISMATCH_X_CM;
            if (sign_mismatch) {
                blend_weight = 1.0f;
            }
            x_now = slow_x_now + (blend_weight * (fast_x_now - slow_x_now));
        }

        float target_weight = near_target_weight(x_now);
        float x_dot_alpha = X_DOT_EMA_ALPHA
                          + ((X_DOT_EMA_ALPHA_NEAR - X_DOT_EMA_ALPHA)
                             * target_weight);
        g_x_dot_cm_s = (x_dot_alpha * d_rate)
                     + ((1.0f - x_dot_alpha) * g_x_dot_cm_s);
        g_prev_d_vel_cm = g_d_vel_cm;
    } else if (g_distance_seeded
            && (g_invalid_count > 0)
            && (g_invalid_count <= DIST_CTRL_INVALID_HOLD_SAMPLES)) {
        float predicted_step_cm =
            clamp_float(g_x_dot_cm_s * g_loop_dt_s,
                        -DIST_CTRL_INVALID_PREDICT_STEP_CM,
                         DIST_CTRL_INVALID_PREDICT_STEP_CM);
        x_now = g_x_cm + predicted_step_cm;
        g_d_rate_cm_s = g_x_dot_cm_s;
        g_x_dot_cm_s *= DIST_CTRL_INVALID_X_DOT_DECAY;
        g_prev_d_vel_cm = g_d_vel_cm;
    } else {
        g_d_rate_cm_s = 0.0f;
        g_x_dot_cm_s = 0.0f;
        g_prev_d_vel_cm = g_d_vel_cm;
    }
    g_x_cm = x_now;

    if (theta_valid) {
        float theta_rate = (g_theta_rel_deg - g_prev_theta_rel_deg) / g_loop_dt_s;
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

float apply_signed_deadband(float value, float deadband) {
    if (deadband <= 0.0f) {
        return value;
    }

    float abs_value = fabs(value);
    if (abs_value <= deadband) {
        return 0.0f;
    }

    return copysignf(abs_value - deadband, value);
}

float ramp_weight(float magnitude, float start, float full) {
    if (magnitude <= start) {
        return 0.0f;
    }
    if (full <= start) {
        return 1.0f;
    }

    return clamp_float((magnitude - start) / (full - start), 0.0f, 1.0f);
}

float near_target_weight(float x_cm) {
    if (X_DOT_NEAR_TARGET_BAND_CM <= 0.0f) {
        return 0.0f;
    }

    return clamp_float((X_DOT_NEAR_TARGET_BAND_CM - fabs(x_cm))
                           / X_DOT_NEAR_TARGET_BAND_CM,
                       0.0f,
                       1.0f);
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

float compute_outer_theta_command(bool distance_valid) {
    float kx = OUTER_LQI_KX_DEFAULT * g_outer_gain_scale;
    float kv = OUTER_LQI_KV_DEFAULT * g_outer_gain_scale;
    float ki = OUTER_LQI_KI_DEFAULT * g_outer_gain_scale;
    float x_mag = fabs(g_x_cm);
    float x_dot_mag = fabs(g_x_dot_cm_s);
    float gain_boost = fmaxf(ramp_weight(x_mag,
                                         OUTER_GAIN_BOOST_X_CM,
                                         OUTER_GAIN_BOOST_FULL_X_CM),
                             ramp_weight(x_dot_mag,
                                         OUTER_GAIN_BOOST_X_DOT_CM_S,
                                         OUTER_GAIN_BOOST_FULL_X_DOT_CM_S));
    kx *= 1.0f + ((OUTER_GAIN_BOOST_KX_SCALE - 1.0f) * gain_boost);
    kv *= 1.0f + ((OUTER_GAIN_BOOST_KV_SCALE - 1.0f) * gain_boost);
    ki *= 1.0f + ((OUTER_GAIN_BOOST_KI_SCALE - 1.0f) * gain_boost);

    if ((g_x_cm * g_x_dot_cm_s) > 0.0f) {
        float outward_weight = fmaxf(ramp_weight(x_mag,
                                                 OUTER_OUTWARD_BRAKE_X_CM,
                                                 OUTER_GAIN_BOOST_FULL_X_CM),
                                     ramp_weight(x_dot_mag,
                                                 OUTER_OUTWARD_BRAKE_X_DOT_CM_S,
                                                 OUTER_GAIN_BOOST_FULL_X_DOT_CM_S));
        kv += OUTER_OUTWARD_KV_EXTRA * outward_weight * g_outer_gain_scale;
    }

    float x_ctrl = apply_signed_deadband(g_x_cm, OUTER_X_DEADBAND_CM);
    float x_dot_ctrl = apply_signed_deadband(g_x_dot_cm_s,
                                             OUTER_X_DOT_DEADBAND_CM_S);
    x_dot_ctrl = clamp_float(x_dot_ctrl,
                             -OUTER_X_DOT_LIMIT_CM_S,
                              OUTER_X_DOT_LIMIT_CM_S);
    bool in_hold_zone = fabs(g_x_cm) <= OUTER_HOLD_X_CM
                     && fabs(g_x_dot_cm_s) <= OUTER_HOLD_X_DOT_CM_S;

    float xi_limit_cm_s = 0.0f;
    if (ki > 0.0001f) {
        xi_limit_cm_s = OUTER_INTEGRAL_CLAMP_DEG / ki;
    }

    bool can_integrate = distance_valid
                      && !in_hold_zone
                      && fabs(g_x_cm) <= OUTER_INTEGRATE_X_CM
                      && fabs(g_x_dot_cm_s) <= OUTER_INTEGRATE_X_DOT_CM_S
                      && (ki > 0.0001f);

    if (can_integrate) {
        float xi_candidate = g_outer_xi_cm_s + (g_x_cm * g_loop_dt_s);
        if (xi_limit_cm_s > 0.0f) {
            xi_candidate = clamp_float(xi_candidate,
                                       -xi_limit_cm_s,
                                        xi_limit_cm_s);
        }

        float candidate_sum = (kx * x_ctrl)
                            + (kv * x_dot_ctrl)
                            + (ki * xi_candidate);
        float candidate_unsat = (float)g_outer_sign * candidate_sum;
        float candidate_sat = clamp_float(candidate_unsat,
                                          -g_theta_cmd_limit_deg,
                                           g_theta_cmd_limit_deg);
        if (fabs(candidate_unsat - candidate_sat) > 0.0005f && ki > 0.0001f) {
            float desired_sum = (float)g_outer_sign * candidate_sat;
            xi_candidate = (desired_sum - (kx * x_ctrl) - (kv * x_dot_ctrl)) / ki;
            if (xi_limit_cm_s > 0.0f) {
                xi_candidate = clamp_float(xi_candidate,
                                           -xi_limit_cm_s,
                                            xi_limit_cm_s);
            }
            g_outer_antiwindup_active = true;
        }
        g_outer_xi_cm_s = xi_candidate;
    } else if (in_hold_zone) {
        g_outer_xi_cm_s *= OUTER_HOLD_XI_DECAY;
    }

    float control_sum = (kx * x_ctrl)
                      + (kv * x_dot_ctrl)
                      + (ki * g_outer_xi_cm_s);
    float theta_unsat = (float)g_outer_sign * control_sum;
    float theta_sat = clamp_float(theta_unsat,
                                  -g_theta_cmd_limit_deg,
                                   g_theta_cmd_limit_deg);

    if (fabs(theta_unsat - theta_sat) > 0.0005f) {
        g_outer_saturated = true;
    }

    if (in_hold_zone) {
        theta_sat = 0.0f;
    }

    g_theta_fs_cmd_deg = theta_unsat;
    g_theta_cmd_unsat_deg = theta_unsat;
    return theta_sat;
}

int32_t compute_inner_step_delta(float theta_target_rel_deg) {
    if (!g_as5600_valid) {
        return 0;
    }

    float target_rel_deg = clamp_theta_rel_command(theta_target_rel_deg);
    float ref_delta_limit = INNER_REF_MAX_RATE_DEG_S * g_loop_dt_s;
    float ref_delta = clamp_float(target_rel_deg - g_inner_theta_ref_rel_deg,
                                  -ref_delta_limit,
                                   ref_delta_limit);
    g_inner_theta_ref_rel_deg = clamp_theta_rel_command(g_inner_theta_ref_rel_deg
                                                      + ref_delta);

    float theta_err_deg = g_inner_theta_ref_rel_deg - g_theta_rel_deg;
    g_theta_err_deg = theta_err_deg;

    bool in_deadband = fabs(theta_err_deg) <= INNER_THETA_DEADBAND_DEG
                    && fabs(g_theta_dot_deg_s) <= INNER_THETA_RATE_DEADBAND_DEG_S
                    && fabs(target_rel_deg - g_inner_theta_ref_rel_deg)
                       <= INNER_THETA_DEADBAND_DEG;
    if (in_deadband) {
        g_inner_step_residual = 0.0f;
        return 0;
    }

    float theta_ctrl_limit_deg = ((float)INNER_MAX_STEP_DELTA) / STEPS_PER_BEAM_DEG;
    float theta_pd_deg = (INNER_KP_THETA * theta_err_deg)
                       - (INNER_KD_THETA * g_theta_dot_deg_s);

    if (INNER_KI_THETA > 0.0001f) {
        float xi_limit_deg_s = INNER_INTEGRAL_CLAMP_DEG / INNER_KI_THETA;
        float theta_unsat_deg = theta_pd_deg + (INNER_KI_THETA * g_inner_theta_xi_deg_s);
        float theta_sat_deg = clamp_float(theta_unsat_deg,
                                          -theta_ctrl_limit_deg,
                                           theta_ctrl_limit_deg);
        bool can_integrate = fabs(theta_unsat_deg - theta_sat_deg) <= 0.0005f
                          || (theta_err_deg * theta_unsat_deg) <= 0.0f;
        if (can_integrate) {
            g_inner_theta_xi_deg_s += theta_err_deg * g_loop_dt_s;
            g_inner_theta_xi_deg_s = clamp_float(g_inner_theta_xi_deg_s,
                                                 -xi_limit_deg_s,
                                                  xi_limit_deg_s);
        } else {
            g_inner_theta_xi_deg_s = (theta_sat_deg - theta_pd_deg) / INNER_KI_THETA;
            g_inner_theta_xi_deg_s = clamp_float(g_inner_theta_xi_deg_s,
                                                 -xi_limit_deg_s,
                                                  xi_limit_deg_s);
            g_inner_antiwindup_active = true;
        }
    }

    float theta_ctrl_deg = theta_pd_deg + (INNER_KI_THETA * g_inner_theta_xi_deg_s);
    float theta_ctrl_sat_deg = clamp_float(theta_ctrl_deg,
                                           -theta_ctrl_limit_deg,
                                            theta_ctrl_limit_deg);
    if (fabs(theta_ctrl_deg - theta_ctrl_sat_deg) > 0.0005f) {
        g_inner_saturated = true;
    }

    float raw_steps = ((float)INNER_STEP_SIGN * theta_ctrl_sat_deg * STEPS_PER_BEAM_DEG)
                    + g_inner_step_residual;
    int32_t rounded_steps = round_to_i32(raw_steps);
    int32_t step_delta = clamp_i32(rounded_steps,
                                   -INNER_MAX_STEP_DELTA,
                                    INNER_MAX_STEP_DELTA);
    if (step_delta != rounded_steps) {
        g_inner_step_residual = 0.0f;
        g_inner_saturated = true;
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
//  Serial command and mode helpers
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

    cancel_staircase(F("mode change"));
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
    g_prev_d_vel_cm = g_d_vel_cm;
    g_x_dot_cm_s = 0.0f;
    g_d_rate_cm_s = 0.0f;
    g_theta_rel_deg = g_theta_cal_deg - g_theta_balance_deg;
    g_prev_theta_rel_deg = g_theta_rel_deg;
    g_theta_dot_deg_s = 0.0f;
    g_theta_fs_cmd_deg = 0.0f;
    g_theta_trim_ff_deg = 0.0f;
    g_theta_cmd_unsat_deg = 0.0f;
    g_theta_cmd_prelimit_deg = clamp_theta_rel_command(g_theta_rel_deg);
    g_theta_cmd_rel_deg = g_theta_cmd_prelimit_deg;
    g_theta_err_deg = 0.0f;
    g_theta_cmd_saturated = false;
    g_outer_saturated = false;
    g_inner_saturated = false;
    g_outer_antiwindup_active = false;
    g_inner_antiwindup_active = false;
    g_antiwindup_active = false;
    g_outer_xi_cm_s = 0.0f;
    g_inner_theta_ref_rel_deg = clamp_theta_rel_command(g_theta_rel_deg);
    g_inner_theta_xi_deg_s = 0.0f;
    g_inner_step_residual = 0.0f;
    g_last_step_delta = 0;
}

void apply_theta_balance_deg(float theta_balance_deg) {
    cancel_staircase(F("theta zero"));
    g_theta_balance_deg = theta_balance_deg;
    g_theta_balance_set = true;
    g_manual_theta_cmd_rel_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    reset_dynamic_state();
}

void reset_distance_filter() {
    g_distance_index = 0;
    g_distance_count = 0;
    g_distance_seeded = false;
    g_distance_accepted = false;
    g_invalid_count = 0;
    g_d_raw_cm = g_setpoint_cm;
    g_d_filt_cm = g_setpoint_cm;
    g_d_vel_cm = g_setpoint_cm;
    g_prev_d_vel_cm = g_setpoint_cm;
    g_d_rate_cm_s = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
        g_distance_window[i] = g_setpoint_cm;
    }
}

void apply_setpoint_cm(float sp) {
    g_setpoint_cm = sp;
    reset_dynamic_state();
}

void apply_staircase_setpoint_cm(float sp) {
    g_setpoint_cm = sp;
    reset_dynamic_state();
}

void print_setpoint_announcement() {
    Serial.print(F("# Setpoint -> "));
    Serial.print(g_setpoint_cm, 2);
    Serial.println(F(" cm"));
}

void cancel_staircase(const __FlashStringHelper *reason) {
    if (!g_staircase_active) {
        return;
    }

    g_staircase_active = false;
    Serial.print(F("# Staircase CANCELLED"));
    if (reason != NULL) {
        Serial.print(F(" -> "));
        Serial.println(reason);
    } else {
        Serial.println();
    }
}

const __FlashStringHelper *staircase_phase_label(StaircasePhase phase) {
    switch (phase) {
        case STAIRCASE_PHASE_FAR:
            return F("FAR");
        case STAIRCASE_PHASE_CENTER:
            return F("CENTER");
        case STAIRCASE_PHASE_NEAR:
            return F("NEAR");
        default:
            return F("UNKNOWN");
    }
}

float staircase_phase_setpoint_cm(StaircasePhase phase) {
    switch (phase) {
        case STAIRCASE_PHASE_FAR:
            return STAIRCASE_FAR_SETPOINT_CM;
        case STAIRCASE_PHASE_CENTER:
            return STAIRCASE_CENTER_SETPOINT_CM;
        case STAIRCASE_PHASE_NEAR:
        default:
            return STAIRCASE_NEAR_SETPOINT_CM;
    }
}

void set_staircase_phase(StaircasePhase phase) {
    g_staircase_phase = phase;
    apply_staircase_setpoint_cm(staircase_phase_setpoint_cm(phase));
    Serial.print(F("# Staircase phase -> "));
    Serial.print(staircase_phase_label(phase));
    Serial.print(F(" ("));
    Serial.print(g_setpoint_cm, 2);
    Serial.println(F(" cm)"));
}

bool start_staircase(uint32_t now_ms) {
    if (g_staircase_active) {
        Serial.println(F("# ERR: staircase already active"));
        return false;
    }

    g_staircase_active = true;
    g_staircase_start_ms = now_ms;
    Serial.println(F("# Staircase START"));
    set_staircase_phase(STAIRCASE_PHASE_FAR);
    return true;
}

void update_staircase(uint32_t now_ms) {
    if (!g_staircase_active) {
        return;
    }

    uint32_t elapsed_ms = now_ms - g_staircase_start_ms;
    if (elapsed_ms >= STAIRCASE_TOTAL_MS) {
        g_staircase_active = false;
        Serial.println(F("# Staircase COMPLETE -> holding final setpoint (send X to disable)"));
        return;
    }

    StaircasePhase next_phase = STAIRCASE_PHASE_FAR;
    if (elapsed_ms >= (2UL * STAIRCASE_STAGE_MS)) {
        next_phase = STAIRCASE_PHASE_NEAR;
    } else if (elapsed_ms >= STAIRCASE_STAGE_MS) {
        next_phase = STAIRCASE_PHASE_CENTER;
    }

    if (next_phase != g_staircase_phase) {
        set_staircase_phase(next_phase);
    }
}

bool is_immediate_serial_command(char c) {
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }

    return c == 'G' || c == 'X' || c == 'E' || c == 'Z' || c == 'Y'
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
        cancel_staircase(F("driver disable"));
        set_driver_enabled(false);
        Serial.println(F("# Driver DISABLED"));

    } else if (cmd == 'E') {
        if (g_cal_mode) {
            Serial.println(F("# ERR: staircase unavailable during calibration telemetry"));
        } else if (g_mode != MODE_CASCADE) {
            Serial.println(F("# ERR: staircase requires M2"));
        } else if (!g_driver_enabled) {
            Serial.println(F("# ERR: staircase requires driver enabled (send G first)"));
        } else if (!POSITION_CALIBRATED) {
            Serial.println(F("# ERR: staircase disabled until POSITION_CALIBRATED is true"));
        } else if (!g_theta_balance_set) {
            Serial.println(F("# ERR: staircase requires Z first to set theta balance"));
        } else {
            start_staircase(millis());
        }

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
            apply_theta_balance_deg(g_theta_cal_deg);
            Serial.print(F("# theta_balance_deg -> "));
            Serial.println(g_theta_balance_deg, 5);
        }

    } else if (cmd == 'B') {
        float balance_deg = (buf[1] == '\0')
                          ? THETA_BALANCE_FIXED_DEG
                          : atof(buf + 1);
        balance_deg = clamp_float(balance_deg,
                                  THETA_CAL_MIN_DEG - THETA_CAL_EXTRAPOLATE_DEG,
                                  THETA_CAL_MAX_DEG + THETA_CAL_EXTRAPOLATE_DEG);
        apply_theta_balance_deg(balance_deg);
        Serial.print(F("# theta_balance_deg -> "));
        Serial.println(g_theta_balance_deg, 5);

    } else if (cmd == 'S') {
        float sp = atof(buf + 1);
        if (sp >= D_MIN_CM && sp <= D_MAX_CM) {
            cancel_staircase(F("manual setpoint"));
            apply_setpoint_cm(sp);
            print_setpoint_announcement();
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

    } else if (cmd == 'P' || cmd == 'V') {
        Serial.println(F("# Deprecated; use K/J/L"));

    } else if (cmd == 'K') {
        float scale = atof(buf + 1);
        g_outer_gain_scale = clamp_float(scale,
                                         OUTER_GAIN_SCALE_MIN,
                                         OUTER_GAIN_SCALE_MAX);
        Serial.print(F("# outer gain scale -> "));
        Serial.println(g_outer_gain_scale, 3);

    } else if (cmd == 'J') {
        Serial.println(F("# Legacy no-op: recovery logic removed in controller_rev 4"));

    } else if (cmd == 'L') {
        float limit = atof(buf + 1);
        g_theta_cmd_limit_deg = clamp_float(limit,
                                            THETA_CMD_LIMIT_MIN_DEG,
                                            THETA_CMD_LIMIT_MAX_DEG);
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
        cancel_staircase(F("calibration toggle"));
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
    Serial.println(F("t_ms,mode,staircase_active,staircase_phase,valid,theta_valid,"
                     "adc_mean,adc_volts,d_raw_cm,d_filt_cm,setpoint_cm,"
                     "x_cm,d_rate_cm_s,x_dot_cm_s,theta_cal_deg,theta_rel_deg,"
                     "theta_dot_deg_s,theta_fs_cmd_deg,theta_trim_ff_deg,"
                     "theta_cmd_prelimit_deg,theta_cmd_rel_deg,theta_cmd_sat,"
                     "xi_cm_s,recovery_active,recovery_count,recovery_floor_deg,"
                     "gain_scale,zero_trim_est_deg,zero_trim_est_count,"
                     "theta_balance_set,outer_sign,driver_enabled,step_pos,"
                     "step_delta,invalid_count,theta_err_deg,theta_cmd_unsat_deg,"
                     "inner_sat,outer_sat,antiwindup_active,controller_rev"));
}

void print_config() {
    Serial.println(F("# -- Ball-and-Beam Sharp GP2Y0A41SK0F + AS5600 Controller --"));
    Serial.print(F("# controller_rev = "));
    Serial.println(CONTROLLER_REV);
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
    Serial.print(F("# D_MIN/D_MAX/default setpoint = "));
    Serial.print(D_MIN_CM, 2); Serial.print(F(", "));
    Serial.print(D_MAX_CM, 2); Serial.print(F(", "));
    Serial.println(g_setpoint_cm, 2);
    Serial.print(F("# staircase stage/targets = "));
    Serial.print(STAIRCASE_STAGE_MS / 1000UL);
    Serial.print(F(" s, "));
    Serial.print(STAIRCASE_FAR_SETPOINT_CM, 2);
    Serial.print(F(", "));
    Serial.print(STAIRCASE_CENTER_SETPOINT_CM, 2);
    Serial.print(F(", "));
    Serial.print(STAIRCASE_NEAR_SETPOINT_CM, 2);
    Serial.println(F(" cm (far, center, near)"));
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
    Serial.print(F("# outer LQI Kx/Kv/Ki/sign = "));
    Serial.print(OUTER_LQI_KX_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_LQI_KV_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_LQI_KI_DEFAULT, 4); Serial.print(F(", "));
    Serial.println(g_outer_sign);
    Serial.print(F("# outer integral clamp / gate = "));
    Serial.print(OUTER_INTEGRAL_CLAMP_DEG, 3); Serial.print(F(" deg, "));
    Serial.print(OUTER_INTEGRATE_X_CM, 2); Serial.print(F(" cm, "));
    Serial.print(OUTER_INTEGRATE_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# outer deadband x / xdot / hold / xi decay = "));
    Serial.print(OUTER_X_DEADBAND_CM, 2); Serial.print(F(" cm, "));
    Serial.print(OUTER_X_DOT_DEADBAND_CM_S, 2); Serial.print(F(" cm/s, "));
    Serial.print(OUTER_HOLD_X_CM, 2); Serial.print(F(" cm / "));
    Serial.print(OUTER_HOLD_X_DOT_CM_S, 2); Serial.print(F(" cm/s, "));
    Serial.println(OUTER_HOLD_XI_DECAY, 3);
    Serial.print(F("# outer boost x/full, xdot/full, scales Kx/Kv/Ki = "));
    Serial.print(OUTER_GAIN_BOOST_X_CM, 2); Serial.print(F(" / "));
    Serial.print(OUTER_GAIN_BOOST_FULL_X_CM, 2); Serial.print(F(" cm, "));
    Serial.print(OUTER_GAIN_BOOST_X_DOT_CM_S, 2); Serial.print(F(" / "));
    Serial.print(OUTER_GAIN_BOOST_FULL_X_DOT_CM_S, 2); Serial.print(F(" cm/s, "));
    Serial.print(OUTER_GAIN_BOOST_KX_SCALE, 2); Serial.print(F(", "));
    Serial.print(OUTER_GAIN_BOOST_KV_SCALE, 2); Serial.print(F(", "));
    Serial.println(OUTER_GAIN_BOOST_KI_SCALE, 2);
    Serial.print(F("# outward brake x / xdot / extra Kv = "));
    Serial.print(OUTER_OUTWARD_BRAKE_X_CM, 2); Serial.print(F(" cm, "));
    Serial.print(OUTER_OUTWARD_BRAKE_X_DOT_CM_S, 2); Serial.print(F(" cm/s, "));
    Serial.println(OUTER_OUTWARD_KV_EXTRA, 3);
    Serial.print(F("# invalid distance hold samples / predict / xdot decay = "));
    Serial.print(DIST_CTRL_INVALID_HOLD_SAMPLES); Serial.print(F(", "));
    Serial.print(DIST_CTRL_INVALID_PREDICT_STEP_CM, 2); Serial.print(F(" cm, "));
    Serial.println(DIST_CTRL_INVALID_X_DOT_DECAY, 3);
    Serial.print(F("# outer gain scale / theta limit = "));
    Serial.print(g_outer_gain_scale, 3); Serial.print(F(", "));
    Serial.print(g_theta_cmd_limit_deg, 3);
    Serial.println(F(" deg"));
    Serial.print(F("# inner Kp/Ki/Kd/int clamp/ref rate = "));
    Serial.print(INNER_KP_THETA, 3); Serial.print(F(", "));
    Serial.print(INNER_KI_THETA, 3); Serial.print(F(", "));
    Serial.print(INNER_KD_THETA, 3); Serial.print(F(", "));
    Serial.print(INNER_INTEGRAL_CLAMP_DEG, 3); Serial.print(F(" deg, "));
    Serial.print(INNER_REF_MAX_RATE_DEG_S, 2);
    Serial.println(F(" deg/s"));
    Serial.print(F("# inner step cap / steps_per_beam_deg / step sign = "));
    Serial.print(INNER_MAX_STEP_DELTA); Serial.print(F(", "));
    Serial.print(STEPS_PER_BEAM_DEG, 2); Serial.print(F(", "));
    Serial.println(INNER_STEP_SIGN);
    Serial.print(F("# distance/xdot filter alpha = "));
    Serial.print(DIST_EMA_ALPHA, 3); Serial.print(F(", "));
    Serial.print(DIST_VEL_EMA_ALPHA, 3); Serial.print(F(", "));
    Serial.print(X_DOT_EMA_ALPHA, 3); Serial.print(F(" -> "));
    Serial.print(X_DOT_EMA_ALPHA_NEAR, 3);
    Serial.print(F(" inside |x| <= "));
    Serial.print(X_DOT_NEAR_TARGET_BAND_CM, 2);
    Serial.println(F(" cm"));
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
    Serial.println(F("# Legacy trim/recovery logic removed in controller_rev 4"));
    Serial.println(F("# Offline tool: tools/design_controller.py --log <telemetry.csv>"));
    Serial.println(F("# Commands: G X E M0 M1 M2 A Z/B[deg] S Y K J L N R H O T C ?"));
    Serial.println(F("# ------------------------------------------------"));
}

void print_telemetry(bool valid,
                     bool theta_valid,
                     uint16_t adc_mean,
                     float adc_volts,
                     float d_raw_print,
                     int32_t step_delta) {
    Serial.print(millis()); Serial.print(',');
    Serial.print((int)g_mode); Serial.print(',');
    Serial.print(g_staircase_active ? 1 : 0); Serial.print(',');
    Serial.print(g_staircase_active ? (int)g_staircase_phase : -1); Serial.print(',');
    Serial.print(valid ? 1 : 0); Serial.print(',');
    Serial.print(theta_valid ? 1 : 0); Serial.print(',');
    Serial.print(adc_mean); Serial.print(',');
    Serial.print(adc_volts, 4); Serial.print(',');
    Serial.print(d_raw_print, 2); Serial.print(',');
    Serial.print(g_d_filt_cm, 2); Serial.print(',');
    Serial.print(g_setpoint_cm, 2); Serial.print(',');
    Serial.print(g_x_cm, 3); Serial.print(',');
    Serial.print(g_d_rate_cm_s, 3); Serial.print(',');
    Serial.print(g_x_dot_cm_s, 3); Serial.print(',');
    Serial.print(g_theta_cal_deg, 4); Serial.print(',');
    Serial.print(g_theta_rel_deg, 4); Serial.print(',');
    Serial.print(g_theta_dot_deg_s, 3); Serial.print(',');
    Serial.print(g_theta_fs_cmd_deg, 4); Serial.print(',');
    Serial.print(g_theta_trim_ff_deg, 4); Serial.print(',');
    Serial.print(g_theta_cmd_prelimit_deg, 4); Serial.print(',');
    Serial.print(g_theta_cmd_rel_deg, 4); Serial.print(',');
    Serial.print(g_theta_cmd_saturated ? 1 : 0); Serial.print(',');
    Serial.print(g_outer_xi_cm_s, 4); Serial.print(',');
    Serial.print(0); Serial.print(',');
    Serial.print(0); Serial.print(',');
    Serial.print(0.0f, 3); Serial.print(',');
    Serial.print(g_outer_gain_scale, 3); Serial.print(',');
    Serial.print(0.0f, 4); Serial.print(',');
    Serial.print(0); Serial.print(',');
    Serial.print(g_theta_balance_set ? 1 : 0); Serial.print(',');
    Serial.print(g_outer_sign); Serial.print(',');
    Serial.print(g_driver_enabled ? 1 : 0); Serial.print(',');
    Serial.print(g_step_pos); Serial.print(',');
    Serial.print(step_delta); Serial.print(',');
    Serial.print(g_invalid_count); Serial.print(',');
    Serial.print(g_theta_err_deg, 4); Serial.print(',');
    Serial.print(g_theta_cmd_unsat_deg, 4); Serial.print(',');
    Serial.print(g_inner_saturated ? 1 : 0); Serial.print(',');
    Serial.print(g_outer_saturated ? 1 : 0); Serial.print(',');
    Serial.print(g_antiwindup_active ? 1 : 0); Serial.print(',');
    Serial.println(CONTROLLER_REV);
}
