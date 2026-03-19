/*
 * ================================================================
 *  Ball-and-Beam Closed-Loop PID Controller
 * ================================================================
 *  Platform  : Arduino Nano (ATmega328P, 16 MHz)
 *  Sensor    : Sharp GP2Y0A21YK0F IR distance sensor → A0
 *  Actuator  : NEMA17 stepper + TMC2209   STEP→D2  DIR→D3
 *  Loop rate : Fixed 40 ms  (25 Hz, matched to Sharp update rate)
 *
 * ----------------------------------------------------------------
 *  Coordinate convention  (see modeling.md §1.3)
 * ----------------------------------------------------------------
 *  d        Distance from Sharp face to ball near surface  [cm]
 *           Increases as ball moves toward motor end.
 *
 *  x_ctrl = C_D - d    [cm]
 *           Positive when ball is on the pivot/sensor side of centre.
 *           Used internally; not exposed as a variable — the PID
 *           error is expressed directly in d-space (see below).
 *
 *  N        Absolute step count from home position  [steps]
 *           Positive N  →  motor-side-up  →  beam tilts so gravity
 *           pulls the ball back toward the sensor.
 *
 * ----------------------------------------------------------------
 *  Error and PID sign chain
 * ----------------------------------------------------------------
 *  error  =  d  -  d_setpoint            [cm]
 *
 *  If the ball drifts toward the motor:
 *    d increases  →  error > 0
 *    PID output N > 0  →  motor-side-up  →  ball returns  ✓
 *
 *  Gain units (from modeling.md §9.5):
 *    Kp  [steps / cm]
 *    Ki  [steps / (cm·s)]   applied as  Ki × Σ(error × dt)
 *    Kd  [step·s / cm]      applied as  Kd × Δerror / dt
 *
 *  PID output N is the TARGET ABSOLUTE step position, not an
 *  increment.  This is consistent with the plant model G(s) in
 *  §9.2 where N is the absolute beam angle in step units.
 *
 * ----------------------------------------------------------------
 *  Sensor calibration model
 * ----------------------------------------------------------------
 *  The Sharp GP2Y0A21YK0F output follows:
 *    d_cm  =  SHARP_K / V_out  +  SHARP_OFFSET
 *  where V_out = ADC_raw × (5.0 / 1023.0)  [Nano AVCC = 5 V]
 *
 *  Datasheet initial values: SHARP_K = 27.86 V·cm, SHARP_OFFSET = 0.
 *  These must be tuned against a ruler after mounting the sensor,
 *  as the curve varies ±15 % unit-to-unit and with ball surface finish.
 *
 *  Calibration procedure:
 *    1. Hold ball at 3 known positions (e.g. 8, 14, 20 cm from face).
 *    2. Read d_cm from the serial monitor with CAL mode enabled.
 *    3. Fit SHARP_K and SHARP_OFFSET:
 *         SHARP_K = (d1 - d2) / (1/V1 - 1/V2)
 *         SHARP_OFFSET = d1 - SHARP_K/V1
 *    4. Update the two #defines below and re-flash.
 *
 * ----------------------------------------------------------------
 *  Serial commands (115200 baud, newline-terminated)
 * ----------------------------------------------------------------
 *  S<d>   Set new setpoint in sensor-cm, e.g. "S14.1"
 *         Range enforced: [8.0, 20.2] cm
 *         Resets integrator and derivative state.
 *  R      Reset integrator and derivative state (no setpoint change)
 *  H      Re-print CSV column header
 *  C      Toggle calibration mode (prints raw ADC + voltage + d_cm)
 *  ?      Print current configuration summary
 *
 *  CSV output (one row per loop):
 *    t_ms, d_raw_cm, d_filt_cm, error_cm, integral_cms,
 *    p_steps, i_steps, d_steps, n_target, step_pos
 *
 * ----------------------------------------------------------------
 *  Timing budget at 40 ms (verified by analysis):
 *    ADC oversampling (4×)          :  ~0.4 ms
 *    Float arithmetic               :  ~1.0 ms
 *    Stepper pulses (150 steps max) : ~30.0 ms  (worst case)
 *    Serial print @ 115200 baud     :  ~3.5 ms
 *    Total worst case               : ~34.9 ms  (within 40 ms ✓)
 * ================================================================
 */

#include <Arduino.h>

// ================================================================
//  SECTION 1 — PIN ASSIGNMENTS
// ================================================================
static const uint8_t PIN_SHARP  = A0;
static const uint8_t PIN_STEP   = 2;
static const uint8_t PIN_DIR    = 3;
static const uint8_t PIN_EN     = 4;   // TMC2209 ENN, active-LOW


// ================================================================
//  SECTION 2 — SHARP SENSOR PARAMETERS
// ================================================================

// ADC reference: Nano AVCC = 5.0 V (default).
// If you have added an external AREF, change VREF accordingly and
// call analogReference(EXTERNAL) in setup().
static const float VREF             = 5.0f;
static const float V_PER_COUNT      = VREF / 1023.0f;   // 0.004888 V/count

// Sensor model:  d_cm = SHARP_K / V_out  +  SHARP_OFFSET
// *** TUNE THESE TWO VALUES after hardware calibration ***
static const float SHARP_K          = 27.86f;   // V·cm
static const float SHARP_OFFSET     = 0.0f;     // cm

// Number of ADC samples to average per reading.
// 4 samples costs ~0.4 ms but halves quantisation noise.
static const uint8_t ADC_SAMPLES    = 4;


// ================================================================
//  SECTION 3 — GEOMETRY AND USABLE RANGE
// ================================================================
//  All values derived from modeling.md §1.3 and §2.1.
//
//  Physical layout (from pivot clevis):
//    Sensor face    : 25 mm
//    Ball radius    : 20 mm
//    d_min          : 80 mm from sensor face  (Sharp minimum range)
//    d_max          : 202 mm from sensor face (runner far end)
//    Usable span    : 122 mm
//    Centre (C_D)   : (80 + 202) / 2 = 141 mm = 14.1 cm

static const float D_MIN_CM         = 8.0f;    // Sharp hard minimum  [cm]
static const float D_MAX_CM         = 20.2f;   // Runner far-end limit [cm]
static const float C_D_CM           = 14.1f;   // Default setpoint     [cm]


// ================================================================
//  SECTION 4 — STEPPER PARAMETERS
// ================================================================
// NEMA17 + TMC2209 at 16-microstep: 200 × 16 = 3200 steps/rev.
// k_step_rad = 2π / 3200 = 1.9635×10⁻³ rad/step
// k_step_deg = 360  / 3200 = 0.1125 °/step

static const int32_t STEPS_PER_REV  = 3200;

// Hard limit on absolute step count from home.
// ±400 steps = ±45°  (software safety clamp; normal operation ≤ ±36 steps for ±4°)
static const int32_t STEP_LIMIT     = 400;

// Maximum steps issued in a single 40 ms loop.
// 150 steps × 200 µs = 30 ms — leaves 10 ms headroom.
static const int32_t MAX_STEPS_LOOP = 150;

// Step pulse period [µs].  5000 steps/s → 5000 × 1.9635e-3 = 9.8 rad/s peak beam rate.
// In normal regulation the actual rate is < 500 steps/s.
static const uint32_t STEP_PERIOD_US = 200;

// DIR polarity:  +1 means HIGH on DIR pin = motor-side-up (positive θ).
// Set to -1 if the motor is wired with reversed direction.
// Verify by issuing a small positive step command and checking beam tilts
// motor-side-up before closing the loop.
static const int8_t DIR_SIGN        = +1;


// ================================================================
//  SECTION 5 — CONTROL LOOP TIMING
// ================================================================
static const uint32_t LOOP_MS       = 40;    // 25 Hz
static const float    DT            = 0.040f; // [s]
static const uint8_t  PRINT_EVERY   = 5;       // print 1 in 5 loops = every 200 ms


// ================================================================
//  SECTION 6 — PID GAINS
// ================================================================
//
//  Error [cm] → PID output [absolute steps]
//
//  Seed gains from first-principles pole placement (modeling.md §9.5):
//    Kp = 3.836  steps/cm
//    Ki = 1.483  steps/(cm·s)
//    Kd = 1.401  step·s/cm
//
//  Runtime-tuned gains (update after hardware tuning, §9.7):
//    Kp = 10.0,  Ki = 0.6,  Kd = 4.5
//
//  To switch to seed gains, change the three lines below.
//  When tuning: start with Kp only (Ki=Kd=0), add Kd next,
//  then add Ki last to eliminate steady-state offset.

static const float KP               = 3.836f;  // steps/cm
static const float KI               = 1.483f;  // steps/(cm·s)
static const float KD               = 1.401f;  // step·s/cm

// EMA filter coefficient on the distance measurement.
// Applied before the derivative to suppress ADC noise.
// α = 1.0 = no filtering.  α = 0.4 → ~1.6 Hz effective BW at 25 Hz.
// Derivative noise with α=0.4: ~0.6 steps RMS — acceptable.
static const float EMA_ALPHA        = 0.4f;

// Integrator anti-windup: clamp the integral contribution to
// ±INTEG_CLAMP_STEPS steps.  Prevents runaway accumulation when
// ball is outside sensor range or held by hand.
static const float INTEG_CLAMP_STEPS = 80.0f;  // [steps]


// ================================================================
//  SECTION 7 — STATE VARIABLES
// ================================================================
static int32_t  g_step_pos    = 0;          // current absolute step count
static float    g_integral    = 0.0f;       // integral accumulator [cm·s]
static float    g_prev_d_filt = C_D_CM;     // previous filtered distance [cm]
static float    g_d_filt      = C_D_CM;     // EMA-filtered distance [cm]
static float    g_setpoint    = C_D_CM;     // current setpoint in d-space [cm]
static uint32_t g_last_ms     = 0;

// Calibration mode flag (toggle with 'C' command)
static bool     g_cal_mode    = false;

// Last PID terms for telemetry
static float    g_p_term      = 0.0f;
static float    g_i_term      = 0.0f;
static float    g_d_term      = 0.0f;


// ================================================================
//  SECTION 8 — FUNCTION PROTOTYPES
// ================================================================
float   read_sharp_cm(void);
int32_t pid_update(float d_filt_cm);
void    step_to_abs(int32_t target);
void    parse_serial(void);
void    print_telemetry(float d_raw, float d_filt, int32_t n_target);
void    print_config(void);
void    print_header(void);


// ================================================================
//  setup()
// ================================================================
void setup() {
    // Stepper pins
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR,  OUTPUT);
    pinMode(PIN_EN,   OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR,  LOW);
    digitalWrite(PIN_EN,   LOW);    // LOW = driver enabled (ENN is active-low)

    // ADC — use AVCC (5 V) as reference
    analogReference(DEFAULT);
    // Discard first few ADC readings (RC settling after reference switch)
    for (uint8_t i = 0; i < 5; i++) analogRead(PIN_SHARP);

    Serial.begin(115200);
    print_config();
    print_header();

    // Warm-up EMA with first real reading
    g_d_filt      = read_sharp_cm();
    g_prev_d_filt = g_d_filt;
    g_last_ms     = millis();
}


// ================================================================
//  loop()
// ================================================================
void loop() {
    // ── Fixed-period gate ──────────────────────────────────────────
    uint32_t now = millis();
    if ((now - g_last_ms) < LOOP_MS) {
        // While waiting, service the serial receive buffer
        parse_serial();
        return;
    }
    g_last_ms = now;

    // ── 1. Read and filter sensor ──────────────────────────────────
    float d_raw = read_sharp_cm();
    g_d_filt    = EMA_ALPHA * d_raw + (1.0f - EMA_ALPHA) * g_d_filt;

    // ── 2. Calibration mode: print raw data and skip control ───────
    if (g_cal_mode) {
        uint16_t raw_sum = 0;
        for (uint8_t i = 0; i < ADC_SAMPLES; i++) raw_sum += analogRead(PIN_SHARP);
        float voltage = (raw_sum / (float)ADC_SAMPLES) * V_PER_COUNT;
        Serial.print(F("CAL  raw=")); Serial.print(raw_sum / ADC_SAMPLES);
        Serial.print(F("  V="));      Serial.print(voltage, 4);
        Serial.print(F("  d_cm="));   Serial.println(d_raw, 2);
        return;
    }

    // ── 3. PID update ──────────────────────────────────────────────
    int32_t n_target = pid_update(g_d_filt);

    // ── 4. Drive stepper to target position ───────────────────────
    step_to_abs(n_target);

    // ── 5. Telemetry ───────────────────────────────────────────────
    static uint8_t print_count = 0;
    if (++print_count >= PRINT_EVERY) {
        print_count = 0;
        print_telemetry(d_raw, g_d_filt, n_target);
    }

    // ── 6. Serial commands ─────────────────────────────────────────
    parse_serial();
}


// ================================================================
//  read_sharp_cm()
// ================================================================
//  Returns distance from Sharp face to ball near-surface [cm].
//
//  Model:  d_cm = SHARP_K / V_out  +  SHARP_OFFSET
//          V_out = (Σ ADC_raw / N_samples) × V_PER_COUNT
//
//  Output clamped to [D_MIN_CM, D_MAX_CM].
//  Returns D_MAX_CM if the ball is absent or reading is saturated.
// ================================================================
float read_sharp_cm() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(PIN_SHARP);
    }
    float voltage = (sum / (float)ADC_SAMPLES) * V_PER_COUNT;

    // Guard against divide-by-zero and near-zero voltage (no reflection)
    if (voltage < 0.08f) return D_MAX_CM;

    float d = SHARP_K / voltage + SHARP_OFFSET;

    // Clamp to valid range
    if (d < D_MIN_CM) d = D_MIN_CM;
    if (d > D_MAX_CM) d = D_MAX_CM;
    return d;
}


// ================================================================
//  pid_update()
// ================================================================
//  Computes target absolute step position from filtered distance.
//
//  Error convention:
//    error = d_filt - g_setpoint                          [cm]
//    error > 0  →  ball too far from sensor (toward motor)
//    N > 0      →  motor-side-up, gravity returns ball    ✓
//
//  Integral:   g_integral += error × DT                  [cm·s]
//  Derivative: on measurement (d_filt) to avoid derivative kick
//              when setpoint is stepped.
//              rate = (d_filt - d_filt_prev) / DT         [cm/s]
//              Sign is the same as d(error)/dt when setpoint
//              is constant — no sign flip needed.
//
//  Output: target absolute step count, clamped to ±STEP_LIMIT.
// ================================================================
int32_t pid_update(float d_filt_cm) {
    float error = d_filt_cm - g_setpoint;

    // ── Proportional ───────────────────────────────────────────────
    g_p_term = KP * error;

    // ── Integral with anti-windup ──────────────────────────────────
    g_integral += error * DT;
    float integ_max = INTEG_CLAMP_STEPS / KI;
    if      (g_integral >  integ_max) g_integral =  integ_max;
    else if (g_integral < -integ_max) g_integral = -integ_max;
    g_i_term = KI * g_integral;

    // ── Derivative on measurement (avoids setpoint-step kick) ──────
    float d_rate = (d_filt_cm - g_prev_d_filt) / DT;   // [cm/s]
    g_prev_d_filt = d_filt_cm;
    g_d_term = KD * d_rate;

    // ── Sum ────────────────────────────────────────────────────────
    float n_float = g_p_term + g_i_term + g_d_term;

    // ── Hard clamp to mechanical limit ────────────────────────────
    if      (n_float >  (float)STEP_LIMIT) n_float =  (float)STEP_LIMIT;
    else if (n_float < -(float)STEP_LIMIT) n_float = -(float)STEP_LIMIT;

    return (int32_t)(n_float + (n_float >= 0.0f ? 0.5f : -0.5f));  // round
}


// ================================================================
//  step_to_abs()
// ================================================================
//  Drives stepper to target absolute step position.
//
//  Steps per call are clamped to MAX_STEPS_LOOP so the 40 ms
//  loop budget is never exceeded.  If the requested move is larger
//  than this limit, the remaining delta is completed over subsequent
//  loops (the PID will naturally keep commanding toward the target).
//
//  DIR_SIGN inverts physical direction without touching any other
//  sign convention — flip it if the beam tilts the wrong way.
// ================================================================
void step_to_abs(int32_t target) {
    int32_t delta = target - g_step_pos;

    // Clamp per-loop excursion
    if      (delta >  MAX_STEPS_LOOP) delta =  MAX_STEPS_LOOP;
    else if (delta < -MAX_STEPS_LOOP) delta = -MAX_STEPS_LOOP;

    if (delta == 0) return;

    // Set direction pin
    bool dir_high = (DIR_SIGN > 0) ? (delta > 0) : (delta < 0);
    digitalWrite(PIN_DIR, dir_high ? HIGH : LOW);
    delayMicroseconds(4);   // DIR setup time ≥ 20 ns (TMC2209 spec); 4 µs is safe

    // Issue step pulses
    int32_t n_steps = (delta > 0) ? delta : -delta;
    for (int32_t i = 0; i < n_steps; i++) {
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(5);               // pulse width ≥ 100 ns; 5 µs is safe
        digitalWrite(PIN_STEP, LOW);
        delayMicroseconds(STEP_PERIOD_US - 5);
    }

    g_step_pos += delta;
}


// ================================================================
//  parse_serial()
// ================================================================
//  Non-blocking: drains whatever bytes are in the RX buffer.
//  Commands are accumulated until a newline (or CR) is received.
//
//  S<float>   New setpoint [cm], validated against [D_MIN_CM, D_MAX_CM]
//  R          Reset integrator and derivative state
//  H          Re-print CSV header
//  C          Toggle calibration (raw ADC) mode
//  ?          Print configuration summary
// ================================================================
void parse_serial() {
    static char    buf[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';
            if (idx > 0) {
                char cmd = buf[0];

                if (cmd == 'S' || cmd == 's') {
                    // ── Set setpoint ──────────────────────────────────
                    float sp = atof(buf + 1);
                    if (sp >= D_MIN_CM && sp <= D_MAX_CM) {
                        g_setpoint    = sp;
                        g_integral    = 0.0f;
                        g_prev_d_filt = g_d_filt;
                        Serial.print(F("# Setpoint -> "));
                        Serial.print(g_setpoint, 2);
                        Serial.println(F(" cm (integrator reset)"));
                    } else {
                        Serial.print(F("# ERR: setpoint must be in ["));
                        Serial.print(D_MIN_CM, 1);
                        Serial.print(F(", "));
                        Serial.print(D_MAX_CM, 1);
                        Serial.println(F("] cm"));
                    }

                } else if (cmd == 'R' || cmd == 'r') {
                    // ── Reset integrator / derivative state ──────────
                    g_integral    = 0.0f;
                    g_prev_d_filt = g_d_filt;
                    Serial.println(F("# Integrator and derivative state reset"));

                } else if (cmd == 'H' || cmd == 'h') {
                    // ── Re-print header ───────────────────────────────
                    print_header();

                } else if (cmd == 'C' || cmd == 'c') {
                    // ── Toggle calibration mode ───────────────────────
                    g_cal_mode = !g_cal_mode;
                    Serial.print(F("# Calibration mode "));
                    Serial.println(g_cal_mode ? F("ON") : F("OFF"));

                } else if (cmd == '?') {
                    // ── Print config ──────────────────────────────────
                    print_config();
                }
            }
            idx = 0;

        } else if (idx < (uint8_t)(sizeof(buf) - 1)) {
            buf[idx++] = c;
        }
    }
}


// ================================================================
//  print_telemetry()
// ================================================================
//  One CSV row per control loop:
//    t_ms       Loop timestamp [ms]
//    d_raw      Raw Sharp reading [cm]
//    d_filt     EMA-filtered reading [cm]
//    error      d_filt - setpoint [cm]
//    integral   Integral accumulator [cm·s]
//    p_steps    Proportional term [steps]
//    i_steps    Integral term [steps]
//    d_steps    Derivative term [steps]
//    n_target   Commanded absolute step position
//    step_pos   Actual stepper position after this loop
// ================================================================
void print_telemetry(float d_raw, float d_filt, int32_t n_target) {
    Serial.print(millis());          Serial.print(',');
    Serial.print(d_raw,    2);       Serial.print(',');
    Serial.print(d_filt,   2);       Serial.print(',');
    Serial.print(d_filt - g_setpoint, 3); Serial.print(',');
    Serial.print(g_integral, 3);     Serial.print(',');
    Serial.print(g_p_term,   2);     Serial.print(',');
    Serial.print(g_i_term,   2);     Serial.print(',');
    Serial.print(g_d_term,   2);     Serial.print(',');
    Serial.print(n_target);          Serial.print(',');
    Serial.println(g_step_pos);
}


// ================================================================
//  print_header() / print_config()
// ================================================================
void print_header() {
    Serial.println(F("t_ms,d_raw_cm,d_filt_cm,error_cm,integral_cms,"
                     "p_steps,i_steps,d_steps,n_target,step_pos"));
}

void print_config() {
    Serial.println(F("# ── Ball-and-Beam Controller ──────────────────"));
    Serial.print  (F("# SHARP_K      = ")); Serial.print(SHARP_K);     Serial.println(F(" V·cm"));
    Serial.print  (F("# SHARP_OFFSET = ")); Serial.print(SHARP_OFFSET); Serial.println(F(" cm"));
    Serial.print  (F("# D_MIN        = ")); Serial.print(D_MIN_CM);     Serial.println(F(" cm"));
    Serial.print  (F("# D_MAX        = ")); Serial.print(D_MAX_CM);     Serial.println(F(" cm"));
    Serial.print  (F("# Setpoint     = ")); Serial.print(g_setpoint);   Serial.println(F(" cm"));
    Serial.print  (F("# Kp = ")); Serial.print(KP);
    Serial.print  (F("  Ki = ")); Serial.print(KI);
    Serial.print  (F("  Kd = ")); Serial.println(KD);
    Serial.print  (F("# DT = ")); Serial.print(DT * 1000.0f, 0); Serial.println(F(" ms"));
    Serial.print  (F("# EMA_ALPHA    = ")); Serial.println(EMA_ALPHA);
    Serial.print  (F("# STEP_LIMIT   = ±")); Serial.print(STEP_LIMIT);
    Serial.print  (F(" steps (±")); Serial.print(STEP_LIMIT * 360.0f / STEPS_PER_REV, 1);
    Serial.println(F("°)"));
    Serial.print  (F("# DIR_SIGN     = ")); Serial.println(DIR_SIGN);
    Serial.println(F("# Commands: S<d_cm>  R  H  C  ?"));
    Serial.println(F("# ────────────────────────────────────────────────"));
}
