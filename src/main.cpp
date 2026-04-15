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
static const uint32_t STAIRCASE_STAGE_MS        = 20000UL;
static const uint32_t STAIRCASE_TOTAL_MS        = 3UL * STAIRCASE_STAGE_MS;
// Keep the far staircase target away from the Sharp ceiling. The April 13 and
// April 14 logs still show too many upper-edge invalid samples near 11.46 cm.
static const float    STAIRCASE_FAR_SETPOINT_CM = D_MAX_CM - 1.55f;
static const float    STAIRCASE_CENTER_SETPOINT_CM = D_SETPOINT_DEFAULT_CM;
static const float    STAIRCASE_NEAR_SETPOINT_CM = 4.90f;

static const float    DIST_EMA_ALPHA            = 0.25f;
// Keep a slow display-oriented distance filter for telemetry, but run the
// controller from a faster position estimate to avoid stale-state corrections.
static const float    DIST_VEL_EMA_ALPHA        = 0.78f;
// Blend back toward the slower distance estimate once the ball is close to the
// active target so hold behavior is not driven by fast-filter jitter.
static const float    DIST_CTRL_BLEND_BAND_CM   = 1.20f;
static const float    X_DOT_EMA_ALPHA           = 0.20f;
// Near the target, keep the velocity estimate noticeably smoother so the
// outer loop does not interpret sensor flutter as real ball motion.
static const float    X_DOT_EMA_ALPHA_NEAR      = 0.08f;
static const float    X_DOT_NEAR_TARGET_BAND_CM = 1.20f;


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
// Add a small rate term so the angle loop does not chase every sign flip with
// a full-strength reversal once the beam is already moving toward target.
static const float    INNER_KD_THETA             = 0.030f;
static const float    INNER_THETA_DEADBAND_DEG   = 0.040f;
static const float    INNER_THETA_RATE_DEADBAND_DEG_S = 0.60f;
// The latest logs spend too much time at the inner-loop step cap. Raise the
// per-cycle budget modestly; it still fits comfortably inside the 40 ms loop.
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
// The GP2Y0A41SK0F updates much faster than HC-SR04. Use the earlier
// 40 ms cadence so the staged controller stays responsive without
// changing the stepper timing budget.
static const uint32_t LOOP_MS                    = 40;
static const float    DT                         = 0.040f;
static const uint8_t  PRINT_EVERY_DEFAULT        = 5;   // T stream: about 5 rows per second
static const uint8_t  PRINT_EVERY_MIN            = 1;
static const uint8_t  PRINT_EVERY_MAX            = 50;
static const float    THETA_CMD_LIMIT_DEFAULT_DEG = 2.00f;

static const float    OUTER_KX_DEFAULT           = 0.31f;
static const float    OUTER_KV_DEFAULT           = 0.17f;
static const float    OUTER_KT_DEFAULT           = 0.00f;
static const float    OUTER_KW_DEFAULT           = 0.00f;
static const float    OUTER_KI_DEFAULT           = 0.030f;
static const float    OUTER_INTEGRAL_CLAMP_DEG   = 0.24f;
// Preserve learned bias over a moderate region, but only accumulate new bias
// close to the target so center capture does not wind up a large stored error.
static const float    OUTER_INTEGRAL_CAPTURE_CM  = 1.60f;
static const float    OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S = 0.65f;
static const float    OUTER_INTEGRAL_ACCUM_CM    = 1.10f;
static const float    OUTER_INTEGRAL_ACCUM_X_DOT_CM_S = 0.55f;
// Center hold needs a slightly wider accumulation gate after the staircase
// transition so it can rebuild low-frequency bias without reopening the large
// off-center windup path.
static const float    OUTER_CENTER_INTEGRAL_ACCUM_CM = 1.50f;
static const float    OUTER_CENTER_INTEGRAL_ACCUM_X_DOT_CM_S = 1.80f;
// These bleed factors are applied on every 40 ms control cycle.
static const float    OUTER_INTEGRAL_BLEED_OUTSIDE = 0.988f;
static const float    OUTER_INTEGRAL_BLEED_RECOVERY = 0.988f;
static const float    OUTER_INTEGRAL_BLEED_CENTER  = 0.996f;
static const float    OUTER_INTEGRAL_BLEED_WRONG_SIGN = 0.85f;
// Near center, small vibration-driven sign changes should not annihilate the
// learned hold bias in just a few samples.
static const float    OUTER_INTEGRAL_BLEED_WRONG_SIGN_CENTER = 0.96f;
static const float    OUTER_WRONG_SIGN_CENTER_X_CM = 0.80f;
static const float    OUTER_WRONG_SIGN_CENTER_X_DOT_CM_S = 1.80f;
// Retain a bounded amount of center-hold bias across staircase phase changes so
// the center trim fallback can reuse it without seeding the outer integrator.
static const float    CENTER_SETPOINT_TOL_CM    = 0.05f;
static const float    OUTER_CENTER_BIAS_MEMORY_ALPHA = 0.10f;
static const float    OUTER_CENTER_BIAS_MEMORY_MAX_DEG = 0.10f;
static const float    CENTER_HOLD_TRIM_MAX_ABS_DEG = 0.06f;
// Keep center-hold trim out of large transport motion; it should only assist
// the final hold region.
static const float    CENTER_HOLD_TRIM_APPLY_X_CM = 0.35f;
static const float    CENTER_HOLD_TRIM_APPLY_X_DOT_CM_S = 0.35f;
static const uint8_t  CENTER_HOLD_TRIM_MIN_COUNT = 16;
static const float    OUTER_CENTER_BAND_CM       = 0.18f;
static const float    OUTER_CENTER_BAND_X_DOT_CM_S = 0.35f;
static const float    OUTER_X_DOT_LIMIT_CM_S     = 4.50f;
static const float    OUTER_THETA_DOT_LIMIT_DEG_S = 4.0f;
static const float    OUTER_GAIN_SCALE_MIN       = 0.50f;
static const float    OUTER_GAIN_SCALE_MAX       = 1.50f;
// Shape the outer-loop command so center hold does not chatter and the inner
// loop is not forced into repeated ±step-cap reversals.
static const float    OUTER_CTRL_SOFTEN_X_CM     = 1.10f;
static const float    OUTER_CTRL_SOFTEN_X_DOT_CM_S = 1.60f;
static const float    OUTER_CTRL_SOFTEN_MAX_REDUCTION = 0.42f;
static const float    OUTER_CMD_SLEW_NEAR_DEG_S  = 3.00f;
static const float    OUTER_CMD_SLEW_FAR_DEG_S   = 9.00f;
// During large staircase-center transports, allow faster command ramping far
// from target so excursions are arrested earlier.
static const float    OUTER_CMD_SLEW_TRANSPORT_X_CM = 2.40f;
static const float    OUTER_CMD_SLEW_TRANSPORT_GAIN = 0.80f;
// Only zero tiny commands inside a genuinely tight hold region. The previous
// gate was silencing corrective action while 0.1-0.2 cm residual error
// remained, which shows up clearly in the April 14 logs.
static const float    OUTER_CTRL_HOLD_X_CM       = 0.08f;
static const float    OUTER_CTRL_HOLD_X_DOT_CM_S = 0.20f;
static const float    OUTER_CTRL_HOLD_DEADBAND_DEG = 0.025f;
// The April 14 logs show that the negative-command side is a bit weaker in
// hold and transport, so keep a modest positive-x authority boost.
static const float    OUTER_POS_X_KX_SCALE       = 1.12f;
static const float    OUTER_POS_X_KV_SCALE       = 1.08f;
// When the ball is already moving back toward the target, reduce damping far
// from the target and fade it back in near the hold region. This avoids
// flipping the beam sign early while the ball is still clearly off-center.
static const float    OUTER_INWARD_DAMP_BAND_CM  = 1.60f;
static const float    OUTER_INWARD_DAMP_MIN_SCALE = 0.70f;
// Add a separate near-center brake so large staircase transports do not carry
// excessive speed through the center capture region.
static const float    OUTER_CENTER_INWARD_BRAKE_BAND_CM = 1.50f;
static const float    OUTER_CENTER_INWARD_EXTRA_KV = 0.14f;
// High-energy center capture schedule: when center-target error or speed is
// large, temporarily increase damping more than stiffness for faster settle
// with fewer recrosses; near-equilibrium hold remains unchanged.
static const float    OUTER_CENTER_CAPTURE_X_ENTER_CM = 0.90f;
static const float    OUTER_CENTER_CAPTURE_X_FULL_CM = 2.40f;
static const float    OUTER_CENTER_CAPTURE_X_DOT_ENTER_CM_S = 1.40f;
static const float    OUTER_CENTER_CAPTURE_X_DOT_FULL_CM_S = 3.60f;
static const float    OUTER_CENTER_CAPTURE_KX_SCALE_MAX = 1.10f;
static const float    OUTER_CENTER_CAPTURE_KV_SCALE_MAX = 1.90f;
// While already moving inward, keep only a fraction of the extra capture
// damping and add a small stiffness bonus so the ball does not stop short.
static const float    OUTER_CENTER_CAPTURE_INWARD_KV_REDUCTION = 0.70f;
static const float    OUTER_CENTER_CAPTURE_INWARD_KX_BONUS = 0.02f;
// Add an explicit near-center speed brake to dissipate kinetic energy during
// fast crossings without adding damping in quiet hold.
static const float    OUTER_CENTER_SPEED_BRAKE_BAND_CM = 1.15f;
static const float    OUTER_CENTER_SPEED_BRAKE_X_DOT_ENTER_CM_S = 1.10f;
static const float    OUTER_CENTER_SPEED_BRAKE_X_DOT_FULL_CM_S = 3.20f;
static const float    OUTER_CENTER_SPEED_BRAKE_EXTRA_KV = 0.42f;
// Staircase center keeps a small additional transport boost on top of the
// generic center-capture schedule.
static const float    OUTER_STAIR_CENTER_TRANSPORT_KX_SCALE_MAX = 1.10f;
static const float    OUTER_STAIR_CENTER_TRANSPORT_KV_SCALE_MAX = 1.45f;
static const float    OUTER_STAIR_CENTER_TRANSPORT_X_CM = 2.40f;
static const float    OUTER_STAIR_CENTER_SETTLE_BRAKE_BAND_CM = 1.40f;
static const float    OUTER_STAIR_CENTER_SETTLE_EXTRA_KV = 0.18f;
static const float    OUTER_POS_X_INWARD_BRAKE_BAND_CM = 2.00f;
static const float    OUTER_POS_X_INWARD_EXTRA_KV = 0.00f;
static const float    OUTER_POS_X_RECOVERY_SCALE = 1.10f;
// Recovery is a minimum-corrective-angle floor used only for truly stalled,
// off-center motion. Keep it out of the near-center regime, where it injects
// energy and causes unnecessary crossovers.
static const float    RECOVERY_ENTER_X_CM        = 1.20f;
static const float    RECOVERY_ENTER_X_DOT_CM_S  = 0.35f;
// The April 14 center-return logs spend long stretches about 0.8-1.3 cm off
// target without ever meeting the generic stall gate. Give center hold a
// dedicated recovery entry band so the recovery floor can break that offset.
static const float    RECOVERY_CENTER_ENTER_X_CM = 0.80f;
static const float    RECOVERY_CENTER_ENTER_X_DOT_CM_S = 1.20f;
static const uint8_t  RECOVERY_ENTER_COUNT       = 8;
static const float    RECOVERY_EXIT_X_CM         = 0.25f;
// Once the ball is clearly moving back toward the new target, hand off to the
// nominal controller earlier to avoid carrying recovery too deep into center.
static const float    RECOVERY_EXIT_HANDOFF_X_CM = 0.45f;
static const float    RECOVERY_EXIT_INWARD_X_DOT_CM_S = 0.90f;
// While the ball is already moving inward, fade out the recovery floor as it
// enters the center approach region so the nominal controller can brake it.
static const float    RECOVERY_INWARD_FLOOR_TAPER_X_CM = 1.20f;
static const float    RECOVERY_FLOOR_DEFAULT_DEG = 0.70f;
static const float    RECOVERY_FLOOR_MIN_DEG     = 0.00f;
static const float    RECOVERY_FLOOR_MAX_DEG     = 1.10f;
static const float    RECOVERY_FLOOR_GAIN_DEG_PER_CM = 0.18f;
// Large staircase setpoint jumps should pre-arm corrective transport instead of
// waiting for the generic stall detector to rediscover the condition.
static const float    SETPOINT_STEP_PREARM_MIN_CM = 1.20f;
static const float    ZERO_TRIM_EST_X_CM         = 0.50f;
static const float    ZERO_TRIM_EST_X_DOT_CM_S   = 1.00f;
static const float    ZERO_TRIM_EST_THETA_TRACK_ERR_DEG = 0.20f;
static const float    ZERO_TRIM_EST_THETA_DOT_DEG_S = 1.20f;
static const float    ZERO_TRIM_EST_CENTER_X_CM  = 0.18f;
static const float    ZERO_TRIM_EST_CENTER_X_DOT_CM_S = 0.25f;
static const float    ZERO_TRIM_EST_CENTER_THETA_TRACK_ERR_DEG = 0.08f;
static const float    ZERO_TRIM_EST_CENTER_THETA_DOT_DEG_S = 0.60f;
static const float    ZERO_TRIM_EST_CENTER_FS_CMD_DEG = 0.08f;
static const float    ZERO_TRIM_EST_ALPHA        = 0.02f;
// Learn a small position-dependent trim so local beam bow/friction does not
// appear as a permanent control error near the sensor-side targets. Center
// hold now uses the dedicated zero-trim path instead so the two trim learners
// cannot stack and flip the small-error command.
static const uint8_t  POSITION_TRIM_BIN_COUNT    = 9;
static const float    POSITION_TRIM_CAPTURE_X_CM = 0.50f;
static const float    POSITION_TRIM_CAPTURE_X_DOT_CM_S = 1.00f;
static const float    POSITION_TRIM_CAPTURE_THETA_TRACK_ERR_DEG = 0.20f;
static const float    POSITION_TRIM_CAPTURE_THETA_DOT_DEG_S = 1.20f;
static const float    POSITION_TRIM_MAX_ABS_DEG  = 0.60f;
static const float    POSITION_TRIM_ALPHA        = 0.08f;
static const float    POSITION_TRIM_MAX_USE_DISTANCE_CM = 1.50f;
// Apply learned trim only inside the local hold region. Outside that region,
// the trim behaves like a transport bias and can even flip the command sign.
static const float    POSITION_TRIM_APPLY_X_CM   = 0.60f;
static const float    POSITION_TRIM_APPLY_X_DOT_CM_S = 0.60f;
// Keep the learned trim out of the near-sensor region. The latest logs show
// that crossings there can look momentarily settled and cause the trim map to
// latch a false bias, which then drives a limit cycle.
static const float    POSITION_TRIM_ENABLE_MIN_CM = D_MIN_CM + 1.20f;
static const uint8_t  POSITION_TRIM_MIN_COUNT     = 4;

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

static ControllerMode g_mode                = MODE_TELEMETRY;
static bool           g_driver_enabled      = false;
static bool           g_cal_mode            = false;
static bool           g_telemetry_stream    = false;
static bool           g_print_once          = false;
static uint8_t        g_print_every         = PRINT_EVERY_DEFAULT;
static uint32_t       g_last_ms             = 0;
static bool           g_staircase_active    = false;
static uint32_t       g_staircase_start_ms  = 0;
static StaircasePhase g_staircase_phase     = STAIRCASE_PHASE_FAR;

static float          g_distance_window[3]  = {D_SETPOINT_DEFAULT_CM,
                                               D_SETPOINT_DEFAULT_CM,
                                               D_SETPOINT_DEFAULT_CM};
static uint8_t        g_distance_index      = 0;
static uint8_t        g_distance_count      = 0;
static bool           g_distance_seeded     = false;
static float          g_d_raw_cm            = D_SETPOINT_DEFAULT_CM;
static float          g_d_filt_cm           = D_SETPOINT_DEFAULT_CM;
static float          g_d_vel_cm            = D_SETPOINT_DEFAULT_CM;
static float          g_d_rate_cm_s         = 0.0f;
static bool           g_distance_accepted   = false;
static float          g_setpoint_cm         = D_SETPOINT_DEFAULT_CM;
static float          g_x_cm                = 0.0f;
static float          g_prev_x_cm           = 0.0f;
static float          g_prev_d_vel_cm       = D_SETPOINT_DEFAULT_CM;
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
static float          g_theta_trim_ff_deg        = 0.0f;
static float          g_theta_cmd_prelimit_deg   = 0.0f;
static float          g_theta_cmd_rel_deg        = 0.0f;
static float          g_theta_ctrl_shaped_deg    = 0.0f;
static bool           g_theta_cmd_saturated      = false;
static float          g_outer_xi_cm_s            = 0.0f;
static bool           g_recovery_active          = false;
static uint8_t        g_recovery_enter_counter   = 0;
static float          g_recovery_floor_deg       = RECOVERY_FLOOR_DEFAULT_DEG;
static float          g_recovery_floor_active_deg = RECOVERY_FLOOR_DEFAULT_DEG;
static bool           g_zero_trim_est_valid      = false;
static float          g_zero_trim_est_deg        = 0.0f;
static uint16_t       g_zero_trim_est_count      = 0;
static bool           g_center_bias_xi_valid     = false;
static float          g_center_bias_xi_cm_s      = 0.0f;
static bool           g_position_trim_valid[POSITION_TRIM_BIN_COUNT] = {false};
static float          g_position_trim_deg[POSITION_TRIM_BIN_COUNT] = {0.0f};
static uint16_t       g_position_trim_count[POSITION_TRIM_BIN_COUNT] = {0};
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
float control_distance_cm(void);
float near_target_weight(float x_cm);
int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi);
int32_t round_to_i32(float value);
float clamp_theta_rel_command(float theta_rel_deg);
float shape_outer_control_command(float theta_ctrl_deg, bool distance_valid);
float compute_full_state_theta_command(bool distance_valid);
int32_t compute_inner_step_delta(float theta_cmd_rel_deg);
void reset_zero_trim_estimator(void);
void reset_center_bias_memory(void);
void update_zero_trim_estimator(bool distance_valid, bool theta_valid);
void reset_position_trim_map(void);
bool position_trim_enabled_for_setpoint_cm(float cm);
uint8_t position_trim_bin_from_cm(float cm);
float position_trim_bin_center_cm(uint8_t idx);
float lookup_position_trim_deg(float cm);
float compute_position_trim_ff_deg(bool distance_valid);
float compute_center_hold_trim_ff_deg(bool distance_valid);
void update_position_trim_map(bool distance_valid, bool theta_valid);
bool is_center_setpoint_cm(float cm);
void step_relative(int32_t delta_steps);
void set_driver_enabled(bool enabled);
void set_mode(ControllerMode mode);
void reset_dynamic_state(void);
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
    reset_position_trim_map();

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
    update_staircase(now);

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
    g_theta_trim_ff_deg = 0.0f;
    g_theta_cmd_prelimit_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    g_theta_cmd_saturated = false;
    int32_t step_delta = 0;

    if (!g_cal_mode) {
        if (g_mode == MODE_MANUAL_ANGLE) {
            g_theta_fs_cmd_deg = g_manual_theta_cmd_rel_deg;
            g_theta_cmd_prelimit_deg = g_manual_theta_cmd_rel_deg;
            g_theta_cmd_rel_deg = clamp_theta_rel_command(g_theta_cmd_prelimit_deg);
            g_theta_cmd_saturated =
                fabs(g_theta_cmd_rel_deg - g_theta_cmd_prelimit_deg) > 0.0005f;
            step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
        } else if (g_mode == MODE_CASCADE) {
            if (POSITION_CALIBRATED && g_theta_balance_set
                    && g_invalid_count < DIST_INVALID_LIMIT && theta_valid) {
                g_theta_trim_ff_deg = compute_position_trim_ff_deg(distance_ok)
                                    + compute_center_hold_trim_ff_deg(distance_ok);
                float theta_ctrl_deg = compute_full_state_theta_command(distance_ok);
                float theta_ctrl_shaped_deg =
                    shape_outer_control_command(theta_ctrl_deg, distance_ok);
                g_theta_cmd_prelimit_deg = theta_ctrl_shaped_deg + g_theta_trim_ff_deg;
                g_theta_cmd_rel_deg = clamp_theta_rel_command(g_theta_cmd_prelimit_deg);
                g_theta_cmd_saturated =
                    fabs(g_theta_cmd_rel_deg - g_theta_cmd_prelimit_deg) > 0.0005f;
            } else {
                g_theta_fs_cmd_deg = 0.0f;
                g_theta_trim_ff_deg = 0.0f;
                g_theta_cmd_prelimit_deg = 0.0f;
                g_theta_cmd_rel_deg = clamp_theta_rel_command(0.0f);
                g_theta_ctrl_shaped_deg = 0.0f;
                g_theta_cmd_saturated = false;
            }
            step_delta = compute_inner_step_delta(g_theta_cmd_rel_deg);
        } else {
            g_theta_fs_cmd_deg = 0.0f;
            g_theta_cmd_prelimit_deg = 0.0f;
            g_theta_cmd_saturated = false;
        }
    }

    if (!g_driver_enabled || !theta_valid || g_mode == MODE_TELEMETRY || g_cal_mode
            || cascade_sensor_fault) {
        step_delta = 0;
    }

    update_zero_trim_estimator(distance_ok, theta_valid);
    update_position_trim_map(distance_ok, theta_valid);

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
    float x_now = control_distance_cm() - g_setpoint_cm;
    if (distance_valid && g_distance_seeded) {
        float d_rate = (g_d_vel_cm - g_prev_d_vel_cm) / DT;
        g_d_rate_cm_s = d_rate;
        float x_rate = (x_now - g_prev_x_cm) / DT;
        float target_weight = near_target_weight(x_now);
        float x_dot_alpha = X_DOT_EMA_ALPHA
                          + ((X_DOT_EMA_ALPHA_NEAR - X_DOT_EMA_ALPHA)
                             * target_weight);
        g_x_dot_cm_s = (x_dot_alpha * x_rate)
                     + ((1.0f - x_dot_alpha) * g_x_dot_cm_s);
        g_prev_x_cm = x_now;
        g_prev_d_vel_cm = g_d_vel_cm;
    } else {
        g_d_rate_cm_s = 0.0f;
        g_x_dot_cm_s = 0.0f;
        g_prev_x_cm = x_now;
        g_prev_d_vel_cm = g_d_vel_cm;
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

float control_distance_cm() {
    if (DIST_CTRL_BLEND_BAND_CM <= 0.0f) {
        return g_d_vel_cm;
    }

    float abs_fast_x_cm = fabs(g_d_vel_cm - g_setpoint_cm);
    float fast_weight = clamp_float(abs_fast_x_cm / DIST_CTRL_BLEND_BAND_CM,
                                    0.0f,
                                    1.0f);
    return g_d_filt_cm + (fast_weight * (g_d_vel_cm - g_d_filt_cm));
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

float shape_outer_control_command(float theta_ctrl_deg, bool distance_valid) {
    float abs_x_cm = fabs(g_x_cm);
    float abs_x_dot_cm_s = fabs(g_x_dot_cm_s);
    bool staircase_center_active = g_staircase_active
                                && g_staircase_phase == STAIRCASE_PHASE_CENTER;
    float x_weight = clamp_float((OUTER_CTRL_SOFTEN_X_CM - abs_x_cm)
                                     / OUTER_CTRL_SOFTEN_X_CM,
                                 0.0f,
                                 1.0f);
    float x_dot_weight =
        clamp_float((OUTER_CTRL_SOFTEN_X_DOT_CM_S - abs_x_dot_cm_s)
                        / OUTER_CTRL_SOFTEN_X_DOT_CM_S,
                    0.0f,
                    1.0f);
    float near_weight = x_weight * x_dot_weight;

    float softened_cmd_deg = theta_ctrl_deg
                           * (1.0f - (OUTER_CTRL_SOFTEN_MAX_REDUCTION * near_weight));
    float slew_deg_s = OUTER_CMD_SLEW_FAR_DEG_S
                    - ((OUTER_CMD_SLEW_FAR_DEG_S - OUTER_CMD_SLEW_NEAR_DEG_S)
                       * near_weight);
    if (staircase_center_active && OUTER_CMD_SLEW_TRANSPORT_X_CM > 0.0f) {
        float transport_weight = clamp_float(abs_x_cm / OUTER_CMD_SLEW_TRANSPORT_X_CM,
                                             0.0f,
                                             1.0f);
        float transport_slew_scale = 1.0f
                                  + (OUTER_CMD_SLEW_TRANSPORT_GAIN
                                     * transport_weight);
        slew_deg_s *= transport_slew_scale;
    }
    float max_delta_deg = slew_deg_s * DT;
    float delta_deg = clamp_float(softened_cmd_deg - g_theta_ctrl_shaped_deg,
                                  -max_delta_deg,
                                   max_delta_deg);
    g_theta_ctrl_shaped_deg += delta_deg;

    bool in_hold_deadband_region =
        abs_x_cm <= OUTER_CTRL_HOLD_X_CM
        && abs_x_dot_cm_s <= OUTER_CTRL_HOLD_X_DOT_CM_S;
    if (distance_valid
            && in_hold_deadband_region
            && fabs(g_theta_ctrl_shaped_deg) <= OUTER_CTRL_HOLD_DEADBAND_DEG) {
        g_theta_ctrl_shaped_deg = 0.0f;
    }

    return g_theta_ctrl_shaped_deg;
}

float compute_full_state_theta_command(bool distance_valid) {
    float gain_scale = g_outer_gain_scale;
    float x_dot_ctrl = clamp_float(g_x_dot_cm_s,
                                   -OUTER_X_DOT_LIMIT_CM_S,
                                    OUTER_X_DOT_LIMIT_CM_S);
    float theta_dot_ctrl = clamp_float(g_theta_dot_deg_s,
                                       -OUTER_THETA_DOT_LIMIT_DEG_S,
                                        OUTER_THETA_DOT_LIMIT_DEG_S);
    float abs_x_cm = fabs(g_x_cm);
    float abs_x_dot_cm_s = fabs(x_dot_ctrl);
    bool positive_x_side = g_x_cm > 0.0f;
    bool moving_inward = (g_x_cm * x_dot_ctrl) < 0.0f;
    bool center_target_active = is_center_setpoint_cm(g_setpoint_cm);
    bool staircase_center_active = center_target_active
                                && g_staircase_active
                                && g_staircase_phase == STAIRCASE_PHASE_CENTER;
    float kx_scale = positive_x_side ? OUTER_POS_X_KX_SCALE : 1.0f;
    float kv_scale = positive_x_side ? OUTER_POS_X_KV_SCALE : 1.0f;
    float kx = OUTER_KX_DEFAULT * gain_scale * kx_scale;
    float kv = OUTER_KV_DEFAULT * gain_scale * kv_scale;
    if (center_target_active) {
        float center_capture_x_weight = 0.0f;
        if (OUTER_CENTER_CAPTURE_X_FULL_CM > OUTER_CENTER_CAPTURE_X_ENTER_CM) {
            center_capture_x_weight =
                clamp_float((abs_x_cm - OUTER_CENTER_CAPTURE_X_ENTER_CM)
                                / (OUTER_CENTER_CAPTURE_X_FULL_CM
                                   - OUTER_CENTER_CAPTURE_X_ENTER_CM),
                            0.0f,
                            1.0f);
        }
        float center_capture_x_dot_weight = 0.0f;
        if (OUTER_CENTER_CAPTURE_X_DOT_FULL_CM_S
                > OUTER_CENTER_CAPTURE_X_DOT_ENTER_CM_S) {
            center_capture_x_dot_weight =
                clamp_float((abs_x_dot_cm_s - OUTER_CENTER_CAPTURE_X_DOT_ENTER_CM_S)
                                / (OUTER_CENTER_CAPTURE_X_DOT_FULL_CM_S
                                   - OUTER_CENTER_CAPTURE_X_DOT_ENTER_CM_S),
                            0.0f,
                            1.0f);
        }
        float center_capture_weight =
            (center_capture_x_weight > center_capture_x_dot_weight)
                ? center_capture_x_weight
                : center_capture_x_dot_weight;
        float center_capture_kx_scale =
            1.0f
            + ((OUTER_CENTER_CAPTURE_KX_SCALE_MAX - 1.0f)
               * center_capture_weight);
        float center_capture_kv_scale =
            1.0f
            + ((OUTER_CENTER_CAPTURE_KV_SCALE_MAX - 1.0f)
               * center_capture_weight);
        if (moving_inward) {
            center_capture_kx_scale += OUTER_CENTER_CAPTURE_INWARD_KX_BONUS
                                    * center_capture_weight;
            center_capture_kv_scale = 1.0f
                                  + ((center_capture_kv_scale - 1.0f)
                                     * OUTER_CENTER_CAPTURE_INWARD_KV_REDUCTION);
        }
        kx *= center_capture_kx_scale;
        kv *= center_capture_kv_scale;
    }
    if (staircase_center_active && OUTER_STAIR_CENTER_TRANSPORT_X_CM > 0.0f) {
        float transport_weight = clamp_float(abs_x_cm / OUTER_STAIR_CENTER_TRANSPORT_X_CM,
                                             0.0f,
                                             1.0f);
        float transport_kx_scale =
            1.0f
            + ((OUTER_STAIR_CENTER_TRANSPORT_KX_SCALE_MAX - 1.0f)
               * transport_weight);
        float transport_kv_scale =
            1.0f
            + ((OUTER_STAIR_CENTER_TRANSPORT_KV_SCALE_MAX - 1.0f)
               * transport_weight);
        kx *= transport_kx_scale;
        kv *= transport_kv_scale;
    }
    if (moving_inward) {
        float inward_brake_weight =
            clamp_float((OUTER_INWARD_DAMP_BAND_CM - abs_x_cm)
                            / OUTER_INWARD_DAMP_BAND_CM,
                        0.0f,
                        1.0f);
        float inward_kv_scale =
            OUTER_INWARD_DAMP_MIN_SCALE
            + ((1.0f - OUTER_INWARD_DAMP_MIN_SCALE) * inward_brake_weight);
        kv *= inward_kv_scale;
    }
    if (positive_x_side && x_dot_ctrl < 0.0f) {
        float inward_brake_weight =
            clamp_float((OUTER_POS_X_INWARD_BRAKE_BAND_CM - abs_x_cm)
                            / OUTER_POS_X_INWARD_BRAKE_BAND_CM,
                        0.0f,
                        1.0f);
        kv += OUTER_POS_X_INWARD_EXTRA_KV * gain_scale * inward_brake_weight;
    }
    if (moving_inward) {
        float center_inward_brake_weight =
            clamp_float((OUTER_CENTER_INWARD_BRAKE_BAND_CM - abs_x_cm)
                            / OUTER_CENTER_INWARD_BRAKE_BAND_CM,
                        0.0f,
                        1.0f);
        kv += OUTER_CENTER_INWARD_EXTRA_KV
              * gain_scale
              * center_inward_brake_weight;
    }
    if (staircase_center_active
            && moving_inward
            && OUTER_STAIR_CENTER_SETTLE_BRAKE_BAND_CM > 0.0f) {
        float staircase_center_brake_weight =
            clamp_float((OUTER_STAIR_CENTER_SETTLE_BRAKE_BAND_CM - abs_x_cm)
                            / OUTER_STAIR_CENTER_SETTLE_BRAKE_BAND_CM,
                        0.0f,
                        1.0f);
        kv += OUTER_STAIR_CENTER_SETTLE_EXTRA_KV
              * gain_scale
              * staircase_center_brake_weight;
    }
    if (center_target_active
            && OUTER_CENTER_SPEED_BRAKE_BAND_CM > 0.0f
            && OUTER_CENTER_SPEED_BRAKE_X_DOT_FULL_CM_S
                > OUTER_CENTER_SPEED_BRAKE_X_DOT_ENTER_CM_S
            && abs_x_cm <= OUTER_CENTER_SPEED_BRAKE_BAND_CM
            && abs_x_dot_cm_s >= OUTER_CENTER_SPEED_BRAKE_X_DOT_ENTER_CM_S) {
        float center_speed_weight =
            clamp_float((OUTER_CENTER_SPEED_BRAKE_BAND_CM - abs_x_cm)
                            / OUTER_CENTER_SPEED_BRAKE_BAND_CM,
                        0.0f,
                        1.0f);
        float center_speed_x_dot_weight =
            clamp_float((abs_x_dot_cm_s - OUTER_CENTER_SPEED_BRAKE_X_DOT_ENTER_CM_S)
                            / (OUTER_CENTER_SPEED_BRAKE_X_DOT_FULL_CM_S
                               - OUTER_CENTER_SPEED_BRAKE_X_DOT_ENTER_CM_S),
                        0.0f,
                        1.0f);
        kv += OUTER_CENTER_SPEED_BRAKE_EXTRA_KV
              * gain_scale
              * center_speed_weight
              * center_speed_x_dot_weight;
    }
    float kt = OUTER_KT_DEFAULT * gain_scale;
    float kw = OUTER_KW_DEFAULT * gain_scale;
    float ki = OUTER_KI_DEFAULT * gain_scale;
    bool recovery_state_observable = g_invalid_count < DIST_INVALID_LIMIT;
    moving_inward = recovery_state_observable && moving_inward;
    bool in_capture_band = abs_x_cm <= OUTER_INTEGRAL_CAPTURE_CM
                        && abs_x_dot_cm_s <= OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S;
    float integrate_x_cm_limit = center_target_active
                              ? OUTER_CENTER_INTEGRAL_ACCUM_CM
                              : OUTER_INTEGRAL_ACCUM_CM;
    float integrate_x_dot_cm_s_limit = center_target_active
                                    ? OUTER_CENTER_INTEGRAL_ACCUM_X_DOT_CM_S
                                    : OUTER_INTEGRAL_ACCUM_X_DOT_CM_S;
    bool in_integrate_band = abs_x_cm <= integrate_x_cm_limit
                          && abs_x_dot_cm_s <= integrate_x_dot_cm_s_limit;
    bool in_center_band = abs_x_cm <= OUTER_CENTER_BAND_CM
                       && abs_x_dot_cm_s <= OUTER_CENTER_BAND_X_DOT_CM_S;
    float recovery_enter_x_cm = center_target_active
                              ? RECOVERY_CENTER_ENTER_X_CM
                              : RECOVERY_ENTER_X_CM;
    float recovery_enter_x_dot_cm_s = center_target_active
                                    ? RECOVERY_CENTER_ENTER_X_DOT_CM_S
                                    : RECOVERY_ENTER_X_DOT_CM_S;
    bool stalled_off_center = recovery_state_observable
                           && abs_x_cm >= recovery_enter_x_cm
                           && abs_x_dot_cm_s <= recovery_enter_x_dot_cm_s;

    bool recovery_handoff_ready = moving_inward
                               && abs_x_cm <= RECOVERY_EXIT_HANDOFF_X_CM
                               && abs_x_dot_cm_s >= RECOVERY_EXIT_INWARD_X_DOT_CM_S;

    if (g_recovery_active) {
        if (!recovery_state_observable
                || abs_x_cm <= RECOVERY_EXIT_X_CM
                || recovery_handoff_ready) {
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
        float wrong_sign_bleed = OUTER_INTEGRAL_BLEED_WRONG_SIGN;
        if (center_target_active
                && abs_x_cm <= OUTER_WRONG_SIGN_CENTER_X_CM
                && abs_x_dot_cm_s <= OUTER_WRONG_SIGN_CENTER_X_DOT_CM_S) {
            wrong_sign_bleed = OUTER_INTEGRAL_BLEED_WRONG_SIGN_CENTER;
        }
        g_outer_xi_cm_s *= wrong_sign_bleed;
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
    bool can_integrate = distance_valid
                      && !g_recovery_active
                      && in_integrate_band
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

    bool center_bias_update_ready = center_target_active
                                 && distance_valid
                                 && !g_recovery_active
                                 && (in_capture_band || in_integrate_band);
    if (center_bias_update_ready) {
        float center_bias_sample_cm_s = g_outer_xi_cm_s;
        if (g_zero_trim_est_valid && ki > 0.0001f) {
            center_bias_sample_cm_s = g_zero_trim_est_deg / ki;
        }
        if (ki > 0.0001f) {
            float center_bias_limit_cm_s = OUTER_CENTER_BIAS_MEMORY_MAX_DEG / ki;
            center_bias_sample_cm_s = clamp_float(center_bias_sample_cm_s,
                                                  -center_bias_limit_cm_s,
                                                   center_bias_limit_cm_s);
        }
        if (!g_center_bias_xi_valid) {
            g_center_bias_xi_valid = true;
            g_center_bias_xi_cm_s = center_bias_sample_cm_s;
        } else {
            g_center_bias_xi_cm_s += OUTER_CENTER_BIAS_MEMORY_ALPHA
                                  * (center_bias_sample_cm_s - g_center_bias_xi_cm_s);
        }
    }

    float recovery_floor_scale = positive_x_side
                               ? OUTER_POS_X_RECOVERY_SCALE
                               : 1.0f;
    g_recovery_floor_active_deg = g_recovery_floor_deg * recovery_floor_scale;
    if (g_recovery_active) {
        float dynamic_floor_deg = g_recovery_floor_deg
                                + (RECOVERY_FLOOR_GAIN_DEG_PER_CM
                                   * clamp_float(abs_x_cm - RECOVERY_ENTER_X_CM,
                                                 0.0f,
                                                 10.0f));
        if (moving_inward
                && RECOVERY_INWARD_FLOOR_TAPER_X_CM > RECOVERY_EXIT_X_CM) {
            float taper_weight =
                clamp_float((abs_x_cm - RECOVERY_EXIT_X_CM)
                                / (RECOVERY_INWARD_FLOOR_TAPER_X_CM
                                   - RECOVERY_EXIT_X_CM),
                            0.0f,
                            1.0f);
            dynamic_floor_deg *= taper_weight;
        }
        dynamic_floor_deg *= recovery_floor_scale;
        g_recovery_floor_active_deg = clamp_float(dynamic_floor_deg,
                                                  RECOVERY_FLOOR_MIN_DEG,
                                                  RECOVERY_FLOOR_MAX_DEG);
    }
    if (g_recovery_active && g_recovery_floor_active_deg > 0.0f && g_x_cm != 0.0f) {
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

void reset_center_bias_memory() {
    g_center_bias_xi_valid = false;
    g_center_bias_xi_cm_s = 0.0f;
}

void update_zero_trim_estimator(bool distance_valid, bool theta_valid) {
    float theta_track_err_deg = g_theta_cmd_rel_deg - g_theta_rel_deg;
    bool center_target_active = is_center_setpoint_cm(g_setpoint_cm);
    bool can_sample = (g_mode == MODE_CASCADE)
                   && center_target_active
                   && g_driver_enabled
                   && distance_valid
                   && theta_valid
                   && g_theta_balance_set
                   && !g_recovery_active
                   && (fabs(g_x_cm) <= ZERO_TRIM_EST_CENTER_X_CM)
                   && (fabs(g_x_dot_cm_s) <= ZERO_TRIM_EST_CENTER_X_DOT_CM_S)
                   && (fabs(theta_track_err_deg)
                       <= ZERO_TRIM_EST_CENTER_THETA_TRACK_ERR_DEG)
                   && (fabs(g_theta_dot_deg_s)
                       <= ZERO_TRIM_EST_CENTER_THETA_DOT_DEG_S)
                   && (fabs(g_theta_fs_cmd_deg) <= ZERO_TRIM_EST_CENTER_FS_CMD_DEG);
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

void reset_position_trim_map() {
    g_theta_trim_ff_deg = 0.0f;
    for (uint8_t i = 0; i < POSITION_TRIM_BIN_COUNT; i++) {
        g_position_trim_valid[i] = false;
        g_position_trim_deg[i] = 0.0f;
        g_position_trim_count[i] = 0;
    }
}

bool position_trim_enabled_for_setpoint_cm(float cm) {
    return cm >= POSITION_TRIM_ENABLE_MIN_CM && !is_center_setpoint_cm(cm);
}

uint8_t position_trim_bin_from_cm(float cm) {
    if (POSITION_TRIM_BIN_COUNT <= 1 || D_MAX_CM <= D_MIN_CM) {
        return 0;
    }

    float scaled = clamp_float((cm - D_MIN_CM) / (D_MAX_CM - D_MIN_CM),
                               0.0f,
                               1.0f);
    int32_t idx = round_to_i32(scaled * (float)(POSITION_TRIM_BIN_COUNT - 1));
    idx = clamp_i32(idx, 0, POSITION_TRIM_BIN_COUNT - 1);
    return (uint8_t)idx;
}

float position_trim_bin_center_cm(uint8_t idx) {
    if (POSITION_TRIM_BIN_COUNT <= 1) {
        return D_SETPOINT_DEFAULT_CM;
    }

    float alpha = (float)idx / (float)(POSITION_TRIM_BIN_COUNT - 1);
    return D_MIN_CM + (alpha * (D_MAX_CM - D_MIN_CM));
}

float lookup_position_trim_deg(float cm) {
    if (POSITION_TRIM_BIN_COUNT == 0
            || !position_trim_enabled_for_setpoint_cm(cm)) {
        return 0.0f;
    }

    uint8_t nearest = position_trim_bin_from_cm(cm);
    int16_t left = -1;
    for (int16_t i = (int16_t)nearest; i >= 0; i--) {
        if (g_position_trim_valid[i]
                && g_position_trim_count[i] >= POSITION_TRIM_MIN_COUNT) {
            left = i;
            break;
        }
    }

    int16_t right = -1;
    for (uint8_t i = nearest; i < POSITION_TRIM_BIN_COUNT; i++) {
        if (g_position_trim_valid[i]
                && g_position_trim_count[i] >= POSITION_TRIM_MIN_COUNT) {
            right = (int16_t)i;
            break;
        }
    }

    if (left < 0 && right < 0) {
        return 0.0f;
    }
    if (left >= 0 && right >= 0 && left != right) {
        float left_cm = position_trim_bin_center_cm((uint8_t)left);
        float right_cm = position_trim_bin_center_cm((uint8_t)right);
        if (right_cm > left_cm) {
            float alpha = clamp_float((cm - left_cm) / (right_cm - left_cm),
                                      0.0f,
                                      1.0f);
            return g_position_trim_deg[left]
                 + (alpha * (g_position_trim_deg[right] - g_position_trim_deg[left]));
        }
    }

    int16_t idx = (left >= 0) ? left : right;
    if (idx < 0) {
        return 0.0f;
    }

    float distance_cm = fabs(cm - position_trim_bin_center_cm((uint8_t)idx));
    if (distance_cm > POSITION_TRIM_MAX_USE_DISTANCE_CM) {
        return 0.0f;
    }
    return g_position_trim_deg[idx];
}

float compute_position_trim_ff_deg(bool distance_valid) {
    if (!distance_valid) {
        return 0.0f;
    }

    float trim_deg = lookup_position_trim_deg(g_setpoint_cm);
    if (trim_deg == 0.0f) {
        return 0.0f;
    }

    float x_weight = clamp_float((POSITION_TRIM_APPLY_X_CM - fabs(g_x_cm))
                                     / POSITION_TRIM_APPLY_X_CM,
                                 0.0f,
                                 1.0f);
    float x_dot_weight =
        clamp_float((POSITION_TRIM_APPLY_X_DOT_CM_S - fabs(g_x_dot_cm_s))
                        / POSITION_TRIM_APPLY_X_DOT_CM_S,
                    0.0f,
                    1.0f);
    float trim_weight = (x_weight < x_dot_weight) ? x_weight : x_dot_weight;
    return trim_deg * trim_weight;
}

float compute_center_hold_trim_ff_deg(bool distance_valid) {
    if (!distance_valid || !is_center_setpoint_cm(g_setpoint_cm)) {
        return 0.0f;
    }

    float trim_deg = 0.0f;
    if (g_zero_trim_est_valid
            && g_zero_trim_est_count >= CENTER_HOLD_TRIM_MIN_COUNT) {
        trim_deg = g_zero_trim_est_deg;
    } else if (g_center_bias_xi_valid) {
        trim_deg = OUTER_KI_DEFAULT * g_center_bias_xi_cm_s;
    }
    if (trim_deg == 0.0f) {
        return 0.0f;
    }

    trim_deg = clamp_float(trim_deg,
                           -CENTER_HOLD_TRIM_MAX_ABS_DEG,
                            CENTER_HOLD_TRIM_MAX_ABS_DEG);
    float x_weight = clamp_float((CENTER_HOLD_TRIM_APPLY_X_CM - fabs(g_x_cm))
                                     / CENTER_HOLD_TRIM_APPLY_X_CM,
                                 0.0f,
                                 1.0f);
    float x_dot_weight =
        clamp_float((CENTER_HOLD_TRIM_APPLY_X_DOT_CM_S - fabs(g_x_dot_cm_s))
                        / CENTER_HOLD_TRIM_APPLY_X_DOT_CM_S,
                    0.0f,
                    1.0f);
    float trim_weight = (x_weight < x_dot_weight) ? x_weight : x_dot_weight;
    return trim_deg * trim_weight;
}

bool is_center_setpoint_cm(float cm) {
    return fabs(cm - STAIRCASE_CENTER_SETPOINT_CM) <= CENTER_SETPOINT_TOL_CM;
}

void update_position_trim_map(bool distance_valid, bool theta_valid) {
    float theta_track_err_deg = g_theta_cmd_rel_deg - g_theta_rel_deg;
    bool can_sample = (g_mode == MODE_CASCADE)
                   && g_driver_enabled
                   && distance_valid
                   && theta_valid
                   && g_theta_balance_set
                   && position_trim_enabled_for_setpoint_cm(g_setpoint_cm)
                   && !g_recovery_active
                   && (fabs(g_x_cm) <= POSITION_TRIM_CAPTURE_X_CM)
                   && (fabs(g_x_dot_cm_s) <= POSITION_TRIM_CAPTURE_X_DOT_CM_S)
                   && (fabs(theta_track_err_deg)
                       <= POSITION_TRIM_CAPTURE_THETA_TRACK_ERR_DEG)
                   && (fabs(g_theta_dot_deg_s)
                       <= POSITION_TRIM_CAPTURE_THETA_DOT_DEG_S);
    if (!can_sample) {
        return;
    }

    uint8_t idx = position_trim_bin_from_cm(g_setpoint_cm);
    float sample_deg = clamp_float(g_theta_rel_deg,
                                   -POSITION_TRIM_MAX_ABS_DEG,
                                    POSITION_TRIM_MAX_ABS_DEG);
    if (!g_position_trim_valid[idx]) {
        g_position_trim_valid[idx] = true;
        g_position_trim_deg[idx] = sample_deg;
        g_position_trim_count[idx] = 1;
        return;
    }

    g_position_trim_deg[idx] += POSITION_TRIM_ALPHA
                             * (sample_deg - g_position_trim_deg[idx]);
    if (g_position_trim_count[idx] < 65535) {
        g_position_trim_count[idx]++;
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
void apply_setpoint_cm(float sp) {
    g_setpoint_cm = sp;
    reset_zero_trim_estimator();
    reset_center_bias_memory();
    reset_dynamic_state();
}

void apply_staircase_setpoint_cm(float sp) {
    float prev_setpoint_cm = g_setpoint_cm;

    g_setpoint_cm = sp;

    // Keep the physical motion estimate coherent across staircase jumps. Most
    // target-specific dynamic state is cleared, but large inward steps still
    // pre-arm recovery so center return does not wait for the generic stall
    // detector to rediscover a known transport condition.
    g_x_cm = control_distance_cm() - g_setpoint_cm;
    g_prev_x_cm = g_x_cm;
    g_prev_d_vel_cm = g_d_vel_cm;
    g_x_dot_cm_s = 0.0f;
    g_d_rate_cm_s = 0.0f;
    g_theta_rel_deg = g_theta_cal_deg - g_theta_balance_deg;
    g_prev_theta_rel_deg = g_theta_rel_deg;
    g_theta_dot_deg_s = 0.0f;
    g_theta_fs_cmd_deg = 0.0f;
    g_theta_trim_ff_deg = 0.0f;
    g_theta_cmd_prelimit_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    g_theta_ctrl_shaped_deg = 0.0f;
    g_theta_cmd_saturated = false;
    g_outer_xi_cm_s = 0.0f;
    g_inner_step_residual = 0.0f;
    g_last_step_delta = 0;

    float setpoint_step_cm = fabs(g_setpoint_cm - prev_setpoint_cm);
    bool inward_step = g_setpoint_cm < prev_setpoint_cm;
    bool large_step = setpoint_step_cm >= SETPOINT_STEP_PREARM_MIN_CM;
    bool far_from_new_target = fabs(g_x_cm) >= RECOVERY_ENTER_X_CM;
    g_recovery_active = inward_step
                     && large_step
                     && far_from_new_target;
    g_recovery_enter_counter = g_recovery_active ? RECOVERY_ENTER_COUNT : 0;
    g_recovery_floor_active_deg = g_recovery_floor_deg;
}

void print_setpoint_announcement() {
    Serial.print(F("# Setpoint -> "));
    Serial.print(g_setpoint_cm, 2);
    Serial.println(POSITION_CALIBRATED
        ? F(" cm")
        : F(" cm (provisional, position calibration still false)"));
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
    g_x_cm = control_distance_cm() - g_setpoint_cm;
    g_prev_x_cm = g_x_cm;
    g_prev_d_vel_cm = g_d_vel_cm;
    g_x_dot_cm_s = 0.0f;
    g_d_rate_cm_s = 0.0f;
    g_theta_rel_deg = g_theta_cal_deg - g_theta_balance_deg;
    g_prev_theta_rel_deg = g_theta_rel_deg;
    g_theta_dot_deg_s = 0.0f;
    g_theta_fs_cmd_deg = 0.0f;
    g_theta_trim_ff_deg = 0.0f;
    g_theta_cmd_prelimit_deg = 0.0f;
    g_theta_cmd_rel_deg = 0.0f;
    g_theta_ctrl_shaped_deg = 0.0f;
    g_theta_cmd_saturated = false;
    g_outer_xi_cm_s = 0.0f;
    reset_center_bias_memory();
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
    g_d_vel_cm = g_setpoint_cm;
    g_prev_d_vel_cm = g_setpoint_cm;
    g_d_rate_cm_s = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
        g_distance_window[i] = g_setpoint_cm;
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
            cancel_staircase(F("theta zero"));
            g_theta_balance_deg = g_theta_cal_deg;
            g_theta_balance_set = true;
            g_manual_theta_cmd_rel_deg = 0.0f;
            g_theta_cmd_rel_deg = 0.0f;
            reset_zero_trim_estimator();
            reset_position_trim_map();
            reset_dynamic_state();
            Serial.print(F("# theta_balance_deg -> "));
            Serial.println(g_theta_balance_deg, 5);
            Serial.println(F("# Position-trim map reset"));
            Serial.println(F("# Manual theta command reset to A0.0"));
        }

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
                     "step_delta,invalid_count"));
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
    Serial.print(F("# outer Kx/Kv/Kt/Kw/Ki/sign = "));
    Serial.print(OUTER_KX_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KV_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KT_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KW_DEFAULT, 4); Serial.print(F(", "));
    Serial.print(OUTER_KI_DEFAULT, 4); Serial.print(F(", "));
    Serial.println(g_outer_sign);
    Serial.print(F("# side/inward tuning kx_scale/kv_scale/inward_damp_band/inward_kv_min/center_inward_band/center_inward_extra_kv/pos_inward_band/inward_extra_kv/recovery_scale = "));
    Serial.print(OUTER_POS_X_KX_SCALE, 3); Serial.print(F(", "));
    Serial.print(OUTER_POS_X_KV_SCALE, 3); Serial.print(F(", "));
    Serial.print(OUTER_INWARD_DAMP_BAND_CM, 2); Serial.print(F(", "));
    Serial.print(OUTER_INWARD_DAMP_MIN_SCALE, 3); Serial.print(F(", "));
    Serial.print(OUTER_CENTER_INWARD_BRAKE_BAND_CM, 2); Serial.print(F(", "));
    Serial.print(OUTER_CENTER_INWARD_EXTRA_KV, 3); Serial.print(F(", "));
    Serial.print(OUTER_POS_X_INWARD_BRAKE_BAND_CM, 2); Serial.print(F(", "));
    Serial.print(OUTER_POS_X_INWARD_EXTRA_KV, 3); Serial.print(F(", "));
    Serial.println(OUTER_POS_X_RECOVERY_SCALE, 3);
    Serial.print(F("# distance/xdot filter alpha = "));
    Serial.print(DIST_EMA_ALPHA, 3); Serial.print(F(", "));
    Serial.print(DIST_VEL_EMA_ALPHA, 3); Serial.print(F(", "));
    Serial.print(X_DOT_EMA_ALPHA, 3); Serial.print(F(" -> "));
    Serial.print(X_DOT_EMA_ALPHA_NEAR, 3);
    Serial.print(F(" inside |x| <= "));
    Serial.print(X_DOT_NEAR_TARGET_BAND_CM, 2);
    Serial.println(F(" cm"));
    Serial.print(F("# position-trim bins/max/capture/alpha = "));
    Serial.print(POSITION_TRIM_BIN_COUNT); Serial.print(F(", "));
    Serial.print(POSITION_TRIM_MAX_ABS_DEG, 3); Serial.print(F(", "));
    Serial.print(POSITION_TRIM_CAPTURE_X_CM, 2); Serial.print(F(" cm, "));
    Serial.print(POSITION_TRIM_CAPTURE_X_DOT_CM_S, 2); Serial.print(F(" cm/s, "));
    Serial.println(POSITION_TRIM_ALPHA, 3);
    Serial.print(F("# position-trim enable min/min count = "));
    Serial.print(POSITION_TRIM_ENABLE_MIN_CM, 2); Serial.print(F(" cm, "));
    Serial.println(POSITION_TRIM_MIN_COUNT);
    Serial.print(F("# position-trim apply |x|/|x_dot| <= "));
    Serial.print(POSITION_TRIM_APPLY_X_CM, 2); Serial.print(F(" cm / "));
    Serial.print(POSITION_TRIM_APPLY_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# staircase inward-step recovery prearm >= "));
    Serial.print(SETPOINT_STEP_PREARM_MIN_CM, 2);
    Serial.println(F(" cm"));
    Serial.print(F("# full-state gain scale = "));
    Serial.println(g_outer_gain_scale, 3);
    Serial.print(F("# outer command shaping soften |x|/|x_dot| <= "));
    Serial.print(OUTER_CTRL_SOFTEN_X_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(OUTER_CTRL_SOFTEN_X_DOT_CM_S, 2);
    Serial.print(F(" cm/s, reduction = "));
    Serial.print(OUTER_CTRL_SOFTEN_MAX_REDUCTION, 3);
    Serial.print(F(", slew near/far = "));
    Serial.print(OUTER_CMD_SLEW_NEAR_DEG_S, 2);
    Serial.print(F(" / "));
    Serial.print(OUTER_CMD_SLEW_FAR_DEG_S, 2);
    Serial.print(F(" deg/s, transport x/gain = "));
    Serial.print(OUTER_CMD_SLEW_TRANSPORT_X_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(OUTER_CMD_SLEW_TRANSPORT_GAIN, 3);
    Serial.print(F(", hold deadband = "));
    Serial.print(OUTER_CTRL_HOLD_DEADBAND_DEG, 3);
    Serial.println(F(" deg"));
    Serial.print(F("# integral output clamp = +/-"));
    Serial.print(OUTER_INTEGRAL_CLAMP_DEG, 3);
    Serial.println(F(" deg"));
    Serial.print(F("# integral capture |x|/|x_dot| <= "));
    Serial.print(OUTER_INTEGRAL_CAPTURE_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# integral accumulate |x|/|x_dot| <= "));
    Serial.print(OUTER_INTEGRAL_ACCUM_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(OUTER_INTEGRAL_ACCUM_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# center integral accumulate |x|/|x_dot| <= "));
    Serial.print(OUTER_CENTER_INTEGRAL_ACCUM_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(OUTER_CENTER_INTEGRAL_ACCUM_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# center hold trim max/apply |x|/|x_dot| = "));
    Serial.print(CENTER_HOLD_TRIM_MAX_ABS_DEG, 3); Serial.print(F(" deg, "));
    Serial.print(CENTER_HOLD_TRIM_APPLY_X_CM, 2); Serial.print(F(" cm / "));
    Serial.print(CENTER_HOLD_TRIM_APPLY_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# zero-trim center gate = "));
    Serial.print(ZERO_TRIM_EST_CENTER_X_CM, 2); Serial.print(F(" cm, "));
    Serial.print(ZERO_TRIM_EST_CENTER_X_DOT_CM_S, 2); Serial.print(F(" cm/s, "));
    Serial.println(ZERO_TRIM_EST_CENTER_FS_CMD_DEG, 3);
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
    Serial.print(F("# recovery exit |x| <= "));
    Serial.print(RECOVERY_EXIT_X_CM, 2);
    Serial.println(F(" cm"));
    Serial.print(F("# recovery handoff |x|/|x_dot| <=/>= "));
    Serial.print(RECOVERY_EXIT_HANDOFF_X_CM, 2);
    Serial.print(F(" cm / "));
    Serial.print(RECOVERY_EXIT_INWARD_X_DOT_CM_S, 2);
    Serial.println(F(" cm/s"));
    Serial.print(F("# recovery inward floor taper |x| <= "));
    Serial.print(RECOVERY_INWARD_FLOOR_TAPER_X_CM, 2);
    Serial.println(F(" cm"));
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
    Serial.println(F("# Commands: G X E M0 M1 M2 A<deg> Z S<cm> Y K<float> J<deg> L<deg> N<int> R H O T C ?"));
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
    Serial.print(g_recovery_active ? 1 : 0); Serial.print(',');
    Serial.print(g_recovery_enter_counter); Serial.print(',');
    Serial.print(g_recovery_floor_active_deg, 3); Serial.print(',');
    Serial.print(g_outer_gain_scale, 3); Serial.print(',');
    Serial.print(g_zero_trim_est_valid ? g_zero_trim_est_deg : 0.0f, 4); Serial.print(',');
    Serial.print(g_zero_trim_est_count); Serial.print(',');
    Serial.print(g_theta_balance_set ? 1 : 0); Serial.print(',');
    Serial.print(g_outer_sign); Serial.print(',');
    Serial.print(g_driver_enabled ? 1 : 0); Serial.print(',');
    Serial.print(g_step_pos); Serial.print(',');
    Serial.print(step_delta); Serial.print(',');
    Serial.println(g_invalid_count);
}
