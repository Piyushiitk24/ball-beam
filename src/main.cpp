// E
=============================================================================
// Ball-and-Beam Controller - clean rebuild, rev 5
// -----------------------------------------------------------------------------
// Current control architecture:
//   Sharp IR distance -> light position smoothing -> alpha-beta observer.
//   The observer estimates:
//     x_hat_cm    = ball position error relative to setpoint
//     v_hat_cm_s  = ball velocity estimate
//
//   M2 uses observer position control with gated I-trim:
//     theta_cmd_rel =
//       outer_sign * (Kp*x_hat + Kd*v_hat + Ki*xi)   clamped
//
//   The integral term is deliberately small and gated.  It trims final static
//   offset only when the ball is near the setpoint and nearly stopped.
//
//   Stepper tracks theta_cmd_rel open-loop (step count <-> beam degrees).
//   AS5600 is used for telemetry, step-counter anchoring at boot/G, and
//   verification. It is not in the inner control loop.
// -----------------------------------------------------------------------------
// Operating modes / serial protocol:
//   M0 idle
//   M1 manual open-loop beam angle command
//   M2 observer position control with gated I-trim
//   G  enable driver and re-anchor step counter to AS5600
//   X  safe stop: enter M0, cancel motion, reset observer/integrator, disable
//      driver
//   E  run a 60 s setpoint staircase: far -> center -> near, 20 s each
// -----------------------------------------------------------------------------
// Hardware:
//   - Arduino Nano (ATmega328P @ 16 MHz)
//   - Sharp GP2Y0A41SK0F IR distance sensor on A0
//       d_cm = SHARP_A / V  -  SHARP_B        (V = ADC voltage)
//       valid range 4.60 .. 13.60 cm
//   - AS5600 magnetic encoder on I2C addr 0x36, raw angle at reg 0x0C
//   - NEMA17 stepper via TMC2209
//       STEP=D2, DIR=D3, EN=D4 (active-LOW enable)
//       1/16 microstepping, driven non-blocking via AccelStepper
// -----------------------------------------------------------------------------
// Sign conventions (established empirically in prior work - do not flip):
//   x_cm           = d_filt_cm  -  setpoint_cm
//                    (positive x => ball is FARTHER from sensor than setpoint)
//   theta_rel_deg  = beam angle relative to physical horizontal
//                    (positive theta_rel => ball accelerates in +x direction,
//                     xddot ~= +k * theta_rel  with k ~= 10 cm/s^2 / deg)
//   outer_sign     = -1  => positive x produces negative theta_rel command.
//   STEP_SIGN      = +1  IF positive step counter -> positive beam angle.
//                    If wiring/linkage inverts this, flip to -1.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

// ------------------- timing --------------------------------------------------
static const uint32_t LOOP_MS       = 40;
static const float    LOOP_DT       = LOOP_MS * 1e-3f;
static const uint8_t  TELEM_DIVIDER = 2;       // telemetry every 2 ticks (12.5 Hz)

// ------------------- Sharp IR (A0) -------------------------------------------
static const uint8_t  SHARP_PIN        = A0;
static const uint8_t  SHARP_SAMPLES    = 8;
static const float    SHARP_A_CM_V     = 12.25f;
static const float    SHARP_B_CM       = 0.62f;
static const float    SHARP_VREF       = 5.0f;
static const float    D_MIN_CM         = 4.60f;
static const float    D_MAX_CM         = 13.60f;
static const uint8_t  INVALID_SAFE_MAX = 10;   // 10 ticks (0.4 s) ⇒ safe mode

// ------------------- AS5600 (I2C) --------------------------------------------
static const uint8_t  AS5600_ADDR       = 0x36;
static const uint8_t  AS5600_REG_RAWANG = 0x0C;
static const float    AS_RAW_A_DEG = 293.443f, AS_CAL_A_DEG = -0.70f;
static const float    AS_RAW_B_DEG = 343.916f, AS_CAL_B_DEG =  3.10f;

// ------------------- balance offset ------------------------------------------
static const float    THETA_BALANCE_CAL_DEG = 1.27260f;

// ------------------- stepper -------------------------------------------------
static const uint8_t  STEP_PIN = 2, DIR_PIN = 3, EN_PIN = 4;

// *** STEPS_PER_BEAM_DEG must be calibrated in M1 (Phase 1 of test plan). ***
// Starting placeholder: 80 microsteps per beam-degree.
static const float    STEPS_PER_BEAM_DEG = 118.0f;

// *** STEP_SIGN: flip to -1.0f if Phase 1 shows commanded and measured   ***
// *** beam angle oppose each other (i.e., A +1 gives measured ≈ -1°).    ***
static const float    STEP_SIGN          = +1.0f;

static const float    STEPPER_MAX_SPEED  = 5000.0f;
static const float    STEPPER_ACCEL      = 20000.0f;
static AccelStepper   stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ------------------- controller ----------------------------------------------
// Rev 5 direction:
//   Observer-PD for main behavior, with a small integral trim only for final
//   static offset removal.
static float Kp_base        = 0.55f;    // deg / cm
static float Kd_base        = 0.33f;    // deg / (cm/s)
static float Ki_base        = 0.035f;   // deg / (cm*s)
static float K_scale        = 1.0f;
static float outer_sign     = -1.0f;
static float setpoint_cm    = 10.00f;

// Built-in setpoint staircase for repeatable response tests.
static const float    STAIR_FAR_CM     = 12.50f;
static const float    STAIR_CENTER_CM  = 10.00f;
static const float    STAIR_NEAR_CM    = 7.50f;
static const uint32_t STAIR_STAGE_MS   = 20000UL;

static const float THETA_MAX_DEG = 4.0f;

// Position smoothing before observer.
// Slightly lower than before to reduce Sharp IR jitter.
static const float ALPHA_D       = 0.25f;

// Alpha-beta observer for ball position and velocity.
// x_hat = estimated position error, v_hat = estimated velocity.
static const float OBS_ALPHA     = 0.35f;
static const float OBS_BETA      = 0.08f;

// Integral term is only for slow/static offset.
// Units: cm*s.  Ki * XI_LIMIT ≈ max integral contribution in degrees.
// Keep this trim small; the previous 0.9 deg limit produced bias.
static const float XI_LIMIT_CM_S = 14.0f;

// Only integrate near the target and almost stopped.
// This prevents integral wind-up during transients.
static const float I_ENABLE_X_CM   = 2.5f;
static const float I_ENABLE_V_CM_S = 1.2f;

// ------------------- runtime state -------------------------------------------
enum Mode : uint8_t { M0_IDLE = 0, M1_MANUAL = 1, M2_POSITION = 2 };
static Mode     mode              = M0_IDLE;
static bool     driver_enabled    = false;
static bool     telemetry_on      = true;
static uint8_t  telem_counter     = 0;

static bool     d_filt_init       = false;
static float    d_raw_cm          = 0.0f;
static float    adc_mean_last     = 0.0f;     // rev 3: for telemetry
static float    adc_volts_last    = 0.0f;     // rev 3: for telemetry
static float    d_filt_cm         = 0.0f;
static float    d_filt_prev       = 0.0f;

// Measured position error from filtered Sharp distance.
static float    x_cm              = 0.0f;

// Observer states.
// x_hat_cm      = estimated position error
// v_hat_cm_s    = estimated ball velocity
// x_dot_cm_s    = kept for telemetry compatibility; equals v_hat_cm_s
static bool     obs_init          = false;
static float    x_hat_cm          = 0.0f;
static float    v_hat_cm_s        = 0.0f;
static float    x_dot_cm_s        = 0.0f;

// Integral of position error, used slowly and clamped.
static float    xi_cm_s           = 0.0f;

static uint8_t  invalid_count     = 0;
static bool     d_valid_last      = false;

static float    theta_cal_deg     = 0.0f;
static float    theta_rel_meas_deg= 0.0f;
static bool     theta_valid       = false;

static float    theta_cmd_rel_deg = 0.0f;
static float    theta_m1_manual   = 0.0f;

static uint32_t next_tick_ms      = 0;
static const uint32_t CONTROLLER_REV = 5;

static bool     stair_active       = false;
static uint8_t  stair_stage        = 0;
static uint32_t stair_stage_start_ms = 0;

// ------------------- forward declarations ------------------------------------
static float clampf(float v, float lo, float hi);
static bool  read_sharp_cm(float *d_cm, float *adc_mean, float *volts);
static bool  read_as5600_raw_deg(float *raw_deg);
static float as5600_cal_deg_from_raw(float raw_deg);
static long  theta_rel_to_steps(float theta_rel_deg);
static float steps_to_theta_rel(long steps);
static void  anchor_step_counter_to_as5600();
static void  cancel_stepper_motion();
static void  reset_outer_state();
static void  cancel_staircase();
static void  start_staircase();
static void  update_staircase();
static float staircase_target_cm(uint8_t stage);
static void  apply_staircase_stage(uint8_t stage);
static void  control_tick();
static void  handle_serial();
static void  process_command(const char *s);
static void  print_header();
static void  print_telemetry();
static void  print_config();

// =============================================================================
//                                  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(50);

  Wire.begin();
  Wire.setClock(400000);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);
  stepper.disableOutputs();
  driver_enabled = false;

  // Anchor step counter to physical beam angle via AS5600.
  anchor_step_counter_to_as5600();

  // Seed the distance filter.
  float adc = 0, volts = 0, d = 0;
  if (read_sharp_cm(&d, &adc, &volts)) {
    d_raw_cm       = d;
    adc_mean_last  = adc;
    adc_volts_last = volts;
    d_filt_cm      = d;
    d_filt_prev    = d;
    d_filt_init    = true;
    x_cm           = d - setpoint_cm;
    x_hat_cm       = x_cm;
    v_hat_cm_s     = 0.0f;
    x_dot_cm_s     = 0.0f;
    xi_cm_s        = 0.0f;
    obs_init       = true;
  } else {
    x_dot_cm_s     = 0.0f;
    v_hat_cm_s     = 0.0f;
    xi_cm_s        = 0.0f;
    obs_init       = false;
  }

  next_tick_ms = millis() + LOOP_MS;

  Serial.println(F("# ball-and-beam controller  (clean rebuild, rev 5)"));
  Serial.println(F("# M0 idle | M1 manual open-loop angle | M2 observer control + gated I-trim"));
  Serial.println(F("# cmds: M0/M1/M2  G  X  T  O  E  S<cm>  A<deg>  K<scale>  Y  ?"));
  print_header();
}

// =============================================================================
//                                   LOOP
// =============================================================================
void loop() {
  uint32_t now = millis();
  if ((int32_t)(now - next_tick_ms) >= 0) {
    next_tick_ms += LOOP_MS;
    control_tick();
    if (telemetry_on) {
      if (++telem_counter >= TELEM_DIVIDER) {
        telem_counter = 0;
        print_telemetry();
      }
    }
  }
  handle_serial();
  stepper.run();
}

// =============================================================================
//                               CONTROL TICK
// =============================================================================
static void control_tick() {
  update_staircase();

  // ---- 1. Read distance -----------------------------------------------------
  float adc_mean = 0.0f, volts = 0.0f, d_new = 0.0f;
  bool  d_valid  = read_sharp_cm(&d_new, &adc_mean, &volts);
  d_raw_cm       = d_new;
  adc_mean_last  = adc_mean;   // rev 3
  adc_volts_last = volts;      // rev 3
  d_valid_last   = d_valid;

  // ---- 2. Read angle (telemetry + safety only) ------------------------------
  float raw = 0.0f;
  if (read_as5600_raw_deg(&raw)) {
    theta_cal_deg      = as5600_cal_deg_from_raw(raw);
    theta_rel_meas_deg = theta_cal_deg - THETA_BALANCE_CAL_DEG;
    theta_valid        = true;
  } else {
    theta_valid = false;
  }

  // ---- 3. Filter + alpha-beta observer --------------------------------------
  d_filt_prev = d_filt_cm;

  if (d_valid) {
    if (!d_filt_init) {
      d_filt_cm   = d_new;
      d_filt_prev = d_new;
      d_filt_init = true;

      x_cm        = d_filt_cm - setpoint_cm;
      x_hat_cm    = x_cm;
      v_hat_cm_s  = 0.0f;
      x_dot_cm_s  = 0.0f;
      xi_cm_s     = 0.0f;
      obs_init    = true;
    } else {
      // Position measurement smoothing.
      d_filt_cm = ALPHA_D * d_new + (1.0f - ALPHA_D) * d_filt_cm;
      x_cm      = d_filt_cm - setpoint_cm;

      // Alpha-beta observer.
      if (!obs_init) {
        x_hat_cm    = x_cm;
        v_hat_cm_s  = 0.0f;
        x_dot_cm_s  = 0.0f;
        xi_cm_s     = 0.0f;
        obs_init    = true;
      } else {
        float x_pred  = x_hat_cm + v_hat_cm_s * LOOP_DT;
        float resid   = x_cm - x_pred;

        // Limit only the velocity correction so Sharp IR jitter does not create
        // unrealistically large velocity kicks. Position correction remains responsive.
        float resid_v = clampf(resid, -0.55f, 0.55f);

        x_hat_cm   = x_pred + OBS_ALPHA * resid;
        v_hat_cm_s = v_hat_cm_s + (OBS_BETA / LOOP_DT) * resid_v;
        x_dot_cm_s = v_hat_cm_s;
      }
    }

    invalid_count = 0;
  } else {
    // Sensor invalid: hold position estimate and slowly decay velocity estimate.
    v_hat_cm_s *= 0.80f;
    x_dot_cm_s  = v_hat_cm_s;

    if (invalid_count < 255) invalid_count++;
  }

  // ---- 4. Controller --------------------------------------------------------
  float theta_cmd = theta_cmd_rel_deg;

  if (mode == M2_POSITION) {
    if (d_valid && d_filt_init && obs_init) {
      float Kp = Kp_base * K_scale;
      float Kd = Kd_base * K_scale;
      float Ki = Ki_base * K_scale;

      // Small integral trim only near the target and nearly stopped.  With
      // Ki = 0, keep xi at zero so telemetry reflects pure PD.
      if (Ki > 0.0f) {
        if (fabsf(x_hat_cm) < I_ENABLE_X_CM &&
            fabsf(v_hat_cm_s) < I_ENABLE_V_CM_S) {
          xi_cm_s += x_hat_cm * LOOP_DT;
          xi_cm_s = clampf(xi_cm_s, -XI_LIMIT_CM_S, XI_LIMIT_CM_S);
        }
      } else {
        xi_cm_s = 0.0f;
      }

      float theta_unsat =
        outer_sign * (Kp * x_hat_cm + Kd * v_hat_cm_s + Ki * xi_cm_s);

      theta_cmd = clampf(theta_unsat, -THETA_MAX_DEG, THETA_MAX_DEG);
      theta_cmd_rel_deg = theta_cmd;
    } else if (invalid_count >= INVALID_SAFE_MAX) {
      cancel_staircase();
      mode = M0_IDLE;
      cancel_stepper_motion();
      stepper.disableOutputs();
      driver_enabled = false;
      Serial.println(F("# SAFE: distance invalid >0.4 s  →  M0, driver OFF"));
    }
    // transient invalid: hold last theta_cmd_rel_deg
  } else if (mode == M1_MANUAL) {
    theta_cmd = clampf(theta_m1_manual, -THETA_MAX_DEG, THETA_MAX_DEG);
    theta_cmd_rel_deg = theta_cmd;
  }
  // M0: no command update; stepper target was cancelled on entry.

  // ---- 5. Actuate -----------------------------------------------------------
  if (driver_enabled && mode != M0_IDLE) {
    stepper.moveTo(theta_rel_to_steps(theta_cmd_rel_deg));
  }
}

// =============================================================================
//                        STEPPER ANCHORING / CANCELLATION
// =============================================================================
// Rev 2: AS5600 used here to anchor step counter to physical reality at boot
// and on G. Not used in the control path.
static void anchor_step_counter_to_as5600() {
  float raw = 0.0f;
  if (read_as5600_raw_deg(&raw)) {
    theta_cal_deg       = as5600_cal_deg_from_raw(raw);
    theta_rel_meas_deg  = theta_cal_deg - THETA_BALANCE_CAL_DEG;
    theta_valid         = true;
    long s = theta_rel_to_steps(theta_rel_meas_deg);
    stepper.setCurrentPosition(s);   // AccelStepper: also resets target to s
    theta_cmd_rel_deg   = theta_rel_meas_deg;
  } else {
    stepper.setCurrentPosition(0);
    theta_cmd_rel_deg   = 0.0f;
    theta_valid         = false;
  }
}

// Rev 2: cancel any pending stepper motion by setting target = current.
static void cancel_stepper_motion() {
  long cur = stepper.currentPosition();
  stepper.moveTo(cur);
  theta_cmd_rel_deg = steps_to_theta_rel(cur);
}

// Reset observer/integrator cleanly when entering/leaving control modes.
static void reset_outer_state() {
  if (d_filt_init) {
    x_cm        = d_filt_cm - setpoint_cm;
    x_hat_cm    = x_cm;
    v_hat_cm_s  = 0.0f;
    x_dot_cm_s  = 0.0f;
    xi_cm_s     = 0.0f;
    obs_init    = true;
    d_filt_prev = d_filt_cm;
  } else {
    x_hat_cm    = 0.0f;
    v_hat_cm_s  = 0.0f;
    x_dot_cm_s  = 0.0f;
    xi_cm_s     = 0.0f;
    obs_init    = false;
  }
}

// =============================================================================
//                              SETPOINT STAIRCASE
// =============================================================================
static void cancel_staircase() {
  stair_active = false;
}

static float staircase_target_cm(uint8_t stage) {
  if (stage == 0) return STAIR_FAR_CM;
  if (stage == 1) return STAIR_CENTER_CM;
  return STAIR_NEAR_CM;
}

static void apply_staircase_stage(uint8_t stage) {
  setpoint_cm = staircase_target_cm(stage);
  reset_outer_state();

  Serial.print(F("# staircase "));
  Serial.print(stage + 1);
  Serial.print(F("/3: "));
  if (stage == 0) {
    Serial.print(F("far"));
  } else if (stage == 1) {
    Serial.print(F("center"));
  } else {
    Serial.print(F("near"));
  }
  Serial.print(F(" setpoint = "));
  Serial.println(setpoint_cm, 3);
}

static void start_staircase() {
  stair_active = true;
  stair_stage = 0;
  stair_stage_start_ms = millis();
  apply_staircase_stage(stair_stage);

  if (mode != M2_POSITION) {
    Serial.println(F("# note: staircase is active; use M2 to run position control"));
  }
}

static void update_staircase() {
  if (!stair_active) return;

  uint32_t now = millis();
  while (stair_active &&
         (uint32_t)(now - stair_stage_start_ms) >= STAIR_STAGE_MS) {
    stair_stage_start_ms += STAIR_STAGE_MS;
    stair_stage++;

    if (stair_stage >= 3) {
      stair_active = false;
      Serial.println(F("# staircase complete (holding final setpoint)"));
    } else {
      apply_staircase_stage(stair_stage);
    }
  }
}

// =============================================================================
//                               SENSOR READERS
// =============================================================================
static bool read_sharp_cm(float *d_cm, float *adc_mean_out, float *volts_out) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < SHARP_SAMPLES; i++) sum += analogRead(SHARP_PIN);
  float adc_mean = (float)sum / (float)SHARP_SAMPLES;
  float v        = adc_mean * (SHARP_VREF / 1023.0f);
  *adc_mean_out  = adc_mean;
  *volts_out     = v;
  if (v < 0.05f) { *d_cm = 0.0f; return false; }
  float d = SHARP_A_CM_V / v - SHARP_B_CM;
  *d_cm = d;
  return (d >= D_MIN_CM && d <= D_MAX_CM);
}

static bool read_as5600_raw_deg(float *raw_deg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_RAWANG);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  if (n != 2) return false;
  uint16_t hi  = Wire.read();
  uint16_t lo  = Wire.read();
  uint16_t raw = ((hi << 8) | lo) & 0x0FFF;
  *raw_deg = (float)raw * (360.0f / 4096.0f);
  return true;
}

static float as5600_cal_deg_from_raw(float raw_deg) {
  if (raw_deg < 180.0f) raw_deg += 360.0f;
  float slope = (AS_CAL_B_DEG - AS_CAL_A_DEG) / (AS_RAW_B_DEG - AS_RAW_A_DEG);
  return AS_CAL_A_DEG + slope * (raw_deg - AS_RAW_A_DEG);
}

static long theta_rel_to_steps(float theta_rel_deg) {
  return (long)(STEP_SIGN * theta_rel_deg * STEPS_PER_BEAM_DEG);
}

static float steps_to_theta_rel(long steps) {
  return (float)steps / (STEP_SIGN * STEPS_PER_BEAM_DEG);
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// =============================================================================
//                              SERIAL INTERFACE
// =============================================================================
static char    cmd_buf[32];
static uint8_t cmd_len = 0;

static void handle_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf[cmd_len] = 0;
      if (cmd_len > 0) process_command(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }
}

static void process_command(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  char c0 = *s;
  if (c0 == 0) return;

  switch (c0) {
    case 'M': case 'm': {
      char d = *(s + 1);
      if (d == '0') {
        cancel_staircase();
        mode = M0_IDLE;
        cancel_stepper_motion();                           // [1] rev 2
        reset_outer_state();
        Serial.println(F("# mode = M0 (idle, motion cancelled)"));
      } else if (d == '1') {
        cancel_staircase();
        mode = M1_MANUAL;
        theta_m1_manual = 0.0f;
        reset_outer_state();
        Serial.println(F("# mode = M1 (manual open-loop angle, theta=0)"));
      } else if (d == '2') {
        reset_outer_state();
        mode = M2_POSITION;
        Serial.println(F("# mode = M2 (observer position control with gated I-trim)"));
      } else {
        Serial.println(F("# ? M: expected M0|M1|M2"));
      }
      break;
    }
    case 'G': case 'g': {
      // [1] rev 2: re-anchor step counter to physical beam angle before
      // enabling, in case beam drifted under gravity while driver was off.
      anchor_step_counter_to_as5600();
      stepper.enableOutputs();
      driver_enabled = true;
      Serial.print(F("# driver ENABLED  (re-anchored: theta_rel_meas = "));
      Serial.print(theta_rel_meas_deg, 3);
      Serial.println(F(")"));
      break;
    }
    case 'X': case 'x': {
      cancel_staircase();
      mode = M0_IDLE;
      cancel_stepper_motion();
      reset_outer_state();
      stepper.disableOutputs();
      driver_enabled = false;
      Serial.println(F("# driver DISABLED, mode = M0 (motion cancelled)"));
      break;
    }
    case 'T': case 't': {
      telemetry_on = !telemetry_on;
      Serial.print(F("# telemetry = "));
      Serial.println(telemetry_on ? F("on") : F("off"));
      if (telemetry_on) print_header();
      break;
    }
    case 'O': case 'o': {
      print_telemetry();
      break;
    }
    case 'E': case 'e': {
      start_staircase();
      break;
    }
    case 'S': case 's': {
      float v = atof(s + 1);
      if (v >= D_MIN_CM + 0.5f && v <= D_MAX_CM - 0.5f) {
        cancel_staircase();
        setpoint_cm = v;
        reset_outer_state();
        Serial.print(F("# setpoint = ")); Serial.println(setpoint_cm, 3);
      } else {
        Serial.println(F("# ? S: out of safe range"));
      }
      break;
    }
    case 'A': case 'a': {
      float v = atof(s + 1);
      v = clampf(v, -THETA_MAX_DEG, THETA_MAX_DEG);
      theta_m1_manual = v;
      Serial.print(F("# M1 theta_cmd = ")); Serial.println(theta_m1_manual, 3);
      break;
    }
    case 'K': case 'k': {
      float v = atof(s + 1);
      if (v > 0.0f && v < 10.0f) {
        K_scale = v;
        Serial.print(F("# K_scale = ")); Serial.println(K_scale, 3);
      } else {
        Serial.println(F("# ? K: expected 0 < scale < 10"));
      }
      break;
    }
    case 'Y': case 'y': {
      if (mode == M2_POSITION) {
        Serial.println(F("# REFUSED: cannot flip outer_sign in M2. Switch to M0 first."));
      } else {
        outer_sign = -outer_sign;
        Serial.print(F("# outer_sign = ")); Serial.println(outer_sign, 1);
      }
      break;
    }
    case '?': {
      print_config();
      break;
    }
    default:
      Serial.print(F("# ? unknown cmd: "));
      Serial.println(s);
  }
}

static void print_config() {
  Serial.println(F("# -- config --"));
  Serial.print(F("# mode             = ")); Serial.println((int)mode);
  Serial.print(F("# driver           = ")); Serial.println(driver_enabled ? F("on") : F("off"));
  Serial.print(F("# setpoint_cm      = ")); Serial.println(setpoint_cm, 3);
  Serial.print(F("# staircase        = ")); Serial.println(stair_active ? F("active") : F("off"));
  Serial.print(F("# staircase_stage  = ")); Serial.println((int)stair_stage);
  Serial.print(F("# stair_far_cm     = ")); Serial.println(STAIR_FAR_CM, 3);
  Serial.print(F("# stair_center_cm  = ")); Serial.println(STAIR_CENTER_CM, 3);
  Serial.print(F("# stair_near_cm    = ")); Serial.println(STAIR_NEAR_CM, 3);
  Serial.print(F("# stair_stage_ms   = ")); Serial.println(STAIR_STAGE_MS);
  Serial.print(F("# Kp_base          = ")); Serial.println(Kp_base, 4);
  Serial.print(F("# Kd_base          = ")); Serial.println(Kd_base, 4);
  Serial.print(F("# Ki_base          = ")); Serial.println(Ki_base, 4);
  Serial.print(F("# K_scale          = ")); Serial.println(K_scale, 4);
  Serial.print(F("# Kp_eff           = ")); Serial.println(Kp_base * K_scale, 4);
  Serial.print(F("# Kd_eff           = ")); Serial.println(Kd_base * K_scale, 4);
  Serial.print(F("# Ki_eff           = ")); Serial.println(Ki_base * K_scale, 4);
  Serial.print(F("# outer_sign       = ")); Serial.println(outer_sign, 1);
  Serial.print(F("# theta_max_deg    = ")); Serial.println(THETA_MAX_DEG, 3);
  Serial.print(F("# alpha_d          = ")); Serial.println(ALPHA_D, 3);
  Serial.print(F("# obs_alpha        = ")); Serial.println(OBS_ALPHA, 3);
  Serial.print(F("# obs_beta         = ")); Serial.println(OBS_BETA, 3);
  Serial.print(F("# xi_limit_cm_s    = ")); Serial.println(XI_LIMIT_CM_S, 3);
  Serial.print(F("# i_enable_x_cm    = ")); Serial.println(I_ENABLE_X_CM, 3);
  Serial.print(F("# i_enable_v_cm_s  = ")); Serial.println(I_ENABLE_V_CM_S, 3);
  Serial.print(F("# steps_per_beam_d = ")); Serial.println(STEPS_PER_BEAM_DEG, 3);
  Serial.print(F("# step_sign        = ")); Serial.println(STEP_SIGN, 1);
  Serial.print(F("# balance_cal_deg  = ")); Serial.println(THETA_BALANCE_CAL_DEG, 5);
  Serial.print(F("# controller_rev   = ")); Serial.println(CONTROLLER_REV);
}

// =============================================================================
//                                  TELEMETRY
// =============================================================================
// Rev 3: plain CSV header (no "# CSV: " prefix) so the Python monitor's
// `t_ms,` prefix detector matches. Columns:
//   t_ms,mode,d_valid,theta_valid,adc_mean,adc_volts,d_raw,d_filt,setp,x,
//   x_hat,v_hat,xi,theta_cal,theta_rel_meas,theta_cmd,theta_err,step_pos,
//   step_target,driver,invalid_n,rev
static void print_header() {
  Serial.println(F("t_ms,mode,d_valid,theta_valid,adc_mean,adc_volts,"
                   "d_raw,d_filt,setp,x,x_hat,v_hat,xi,"
                   "theta_cal,theta_rel_meas,theta_cmd,theta_err,"
                   "step_pos,step_target,driver,invalid_n,rev"));
}

static void print_telemetry() {
  float theta_err = theta_cmd_rel_deg - theta_rel_meas_deg;  // rev 3

  Serial.print(millis());                     Serial.print(',');
  Serial.print((int)mode);                    Serial.print(',');
  Serial.print(d_valid_last ? 1 : 0);         Serial.print(',');
  Serial.print(theta_valid ? 1 : 0);          Serial.print(',');
  Serial.print(adc_mean_last, 1);             Serial.print(',');  // rev 3
  Serial.print(adc_volts_last, 3);            Serial.print(',');  // rev 3
  Serial.print(d_raw_cm, 3);                  Serial.print(',');
  Serial.print(d_filt_cm, 3);                 Serial.print(',');
  Serial.print(setpoint_cm, 3);               Serial.print(',');
  Serial.print(x_cm, 3);                      Serial.print(',');
  Serial.print(x_hat_cm, 3);                  Serial.print(',');
  Serial.print(v_hat_cm_s, 3);                Serial.print(',');
  Serial.print(xi_cm_s, 3);                   Serial.print(',');
  Serial.print(theta_cal_deg, 4);             Serial.print(',');
  Serial.print(theta_rel_meas_deg, 4);        Serial.print(',');
  Serial.print(theta_cmd_rel_deg, 4);         Serial.print(',');
  Serial.print(theta_err, 4);                 Serial.print(',');  // rev 3
  Serial.print(stepper.currentPosition());    Serial.print(',');
  Serial.print(stepper.targetPosition());     Serial.print(',');  // rev 3
  Serial.print(driver_enabled ? 1 : 0);       Serial.print(',');
  Serial.print(invalid_count);                Serial.print(',');
  Serial.println(CONTROLLER_REV);
}
