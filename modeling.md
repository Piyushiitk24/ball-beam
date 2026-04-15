# Ball-and-Beam Mathematical Modeling and Control Design

This is the single canonical modeling document for the ball-and-beam control system.
All first-principles derivations, hardware parameters, and controller designs live here.

---

## 1. Definitions

### 1.1 Physical Setup

The system consists of a beam pivoted at one end and actuated at the other by a stepper
motor through a crank-rocker linkage. A hollow ping-pong ball rolls freely along a
V-groove runner mounted on top of the beam. A Sharp GP2Y0A41SK0F IR distance sensor
mounted at the pivot end of the beam measures the ball's distance from the sensor face.
Because the Sharp has a minimum usable detection range of about 4 cm, a tape stop
blocks the near end of the runner and shortens the usable ball-travel window.

```text
  Sharp IR
  sensor
  [====]
  [====]  pivot                              motor crank
  [====]==O====[ tape stop ][ runner ]========O--( )
            ^           ~~~ball~~~            ^
            |                                 |
          fulcrum                         motor-side
          (fixed)                         clevis
```

The pivot-side clevis hole is the coordinate origin for all length measurements along
the beam.

### 1.2 Generalized Coordinates

The system has two generalized coordinates:

| Symbol | Description | Positive direction |
| -------- | ------------- | ------------------- |
| r | Ball position along beam from pivot (m) | Away from pivot (toward motor) |
| θ | Beam angle from horizontal (rad) | Motor side up |

The state vector of the full system is **q** = [r, θ]ᵀ.

### 1.3 Firmware Sign Convention

Sections 3-8 use the physical small-angle convention from Table 1:

- Positive analytical angle `θ` means motor side up.
- Positive analytical angle therefore makes gravity pull the ball toward the pivot, so
  `r` decreases.

The firmware in `src/main.cpp` does not control this analytical angle directly. It uses
the calibrated runtime coordinates

```text
theta_cal_deg = 0.07666806 * as5600_unwrapped_deg - 23.28443907
theta_rel_deg = theta_cal_deg - theta_balance_deg
x_cm          = d_vel_cm - setpoint_cm
```

with the empirically verified sign audit:

- `x_cm > 0` means the ball is toward the motor/far end.
- Manual command `A+` in `M1` rolls the ball toward the motor/far end, and `A-` rolls
  it toward the sensor/near end.
- Therefore, near the operating point, the firmware angle has the opposite sign to the
  analytical angle used in Sections 3-8:

```text
theta_rel_deg ≈ -theta_phys_deg
```

- The implementation sign chain is `DIR_SIGN = -1`, `INNER_STEP_SIGN = -1`,
  `g_outer_sign = -1`.

This translation is kept explicit so the plant derivation stays clean while the
controller record remains faithful to the firmware.

---

## 2. Parameter Evaluation

### 2.1 Raw Measurements

All values were measured on the final assembled hardware.

| Quantity | Measured value | Notes |
| ---------- | --------------- | ------- |
| Ball mass | 2.8 g | Standard 40 mm ping-pong ball |
| Ball diameter | 40.0 mm | |
| Bare beam mass | 33.2 g | Aluminium extrusion, 263 mm span |
| Sharp IR sensor + wires mass | 5.3 g | Pivot-mounted |
| Total beam assembly mass | 38.5 g | Bare beam + sensor + wires |
| Beam length (clevis-to-clevis) | 263 mm | |
| Sharp body start from pivot-side clevis | 27.3 mm | Mount footprint start |
| Sharp body end from pivot-side clevis | 40.3 mm | Mount footprint end |
| Sharp sensing face from pivot-side clevis | 40.3 mm | Front face used for distance mapping |
| Sharp mass lump from pivot-side clevis | 33.8 mm | Body midpoint used for COM/inertia |
| Nearest ball–sensor clearance | 44.0 mm | Tape-limited near stop |
| Farthest ball–sensor clearance | 126.6 mm | Far usable stop |
| Usable sensing window | 82.6 mm | `126.6 − 44.0` mm |
| Nearest valid ball position from pivot-side clevis | 84.3 mm | `40.3 + 44.0` mm |
| Farthest valid ball position from pivot-side clevis | 166.9 mm | `40.3 + 126.6` mm |
| Effective runner end clearance to motor clevis | 96.1 mm | `263.0 − 166.9` mm |
| Stepper full-steps per revolution | 200 | NEMA 17 |
| Microstep divisor | 16 | TMC2209 driver |
| Rolling viscous damping | 0.0125 N·s/m | Pre-identification estimate |
| Pivot viscous damping | 0.006 N·m·s | Pre-identification estimate |

### 2.2 SI Symbol Table

The symbols used throughout the derivation are defined and evaluated here.

**Ball:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| m | — | 0.0028 | kg |
| R | — | 0.020 | m |
| I | (2/3) m R² | 7.467 × 10⁻⁷ | kg·m² |

The factor 2/3 comes from the thin-shell hollow-sphere model. A thin spherical shell of
mass m and radius R has moment of inertia I = (2/3) m R² about any diameter.

Numerical substitution:

```text
I = (2/3) × 0.0028 × (0.020)²
  = (2/3) × 0.0028 × 0.0004
  = 7.4667 × 10⁻⁷  kg·m²
```

**Beam assembly:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| M_b | m_beam + m_sensor | 0.0385 | kg |
| L | — | 0.263 | m |
| L_com | see §2.3 | 0.11805 | m |
| J | see §2.4 | 7.715 × 10⁻⁴ | kg·m² |

**Actuation and sensing:**

| Symbol | Formula | Value | Unit |
| -------- | --------- | ------- | ------ |
| N_steps | 200 × 16 | 3200 | microsteps/rev |
| k_AS | 360 / 4096 | 0.087890625 | AS5600-deg/count |
| k_θ | signed AS5600-to-beam affine fit slope | 0.07666806 | beam-deg/AS5600-deg |
| b_θ | signed AS5600-to-beam affine fit intercept | -23.28443907 | beam-deg |
| steps_per_beam_deg | (3200 / 360) / k_θ | 115.9399 | microsteps/beam-deg |
| center_d | (44.0 mm + 126.6 mm) / 2 | 0.0853 | m |
| r₀ | x_sensor_face + center_d | 0.1256 | m |

**Physical constants:**

| Symbol | Value | Unit |
| -------- | ------- | ------ |
| g | 9.81 | m/s² |
| b_x | 0.0125 | N·s/m |
| b_θ | 0.006 | N·m·s |

### 2.3 Beam + Sensor Combined Centre of Mass

The beam assembly is modelled as a uniform slender rod (the aluminium extrusion) plus a
lumped point mass (the Sharp sensor and wires) at the sensor body midpoint. The sensing
face location is kept separate for ball-position mapping.

```text
x_com = (m_beam × x_beam_com + m_sensor × x_sensor_com) / M_b
```

where `x_beam_com = L / 2 = 0.263 / 2 = 0.1315 m` (uniform rod) and
`x_sensor_com = (0.0273 + 0.0403)/2 = 0.0338 m` (sensor body midpoint). The sensing
face used for distance mapping remains `x_sensor_face = 0.0403 m`.

Substitution:

```text
x_com = (0.0332 × 0.1315 + 0.0053 × 0.0338) / 0.0385
      = (0.0043658 + 0.00017914) / 0.0385
      = 0.00454494 / 0.0385
      = 0.11805 m
```

### 2.4 Beam Moment of Inertia About the Pivot

The pivot is at the pivot-side clevis hole. The beam inertia is estimated as a uniform
rod about one end plus a point mass for the sensor.

Uniform rod about one end:

```text
J_beam = (1/3) m_beam L²
       = (1/3) × 0.0332 × (0.263)²
       = (1/3) × 0.0332 × 0.069169
       = 7.6547 × 10⁻⁴  kg·m²
```

Sensor lumped at x_sensor_com = 0.0338 m:

```text
J_sensor = m_sensor × x_sensor_com²
         = 0.0053 × (0.0338)²
         = 0.0053 × 0.00114244
         = 6.055 × 10⁻⁶  kg·m²
```

Total beam inertia about pivot:

```text
J = J_beam + J_sensor
  = 7.6547 × 10⁻⁴ + 6.055 × 10⁻⁶
  = 7.715 × 10⁻⁴  kg·m²
```

### 2.5 Nominal Ball-Position Centre

The Sharp sensing face is at 40.3 mm from the pivot clevis. The tape-limited usable
ball window is measured directly from the sensor face as 44.0 mm to 126.6 mm, so the
usable window and center reading are:

```text
usable_window = 126.6 mm − 44.0 mm = 82.6 mm = 0.0826 m

center_d = (44.0 mm + 126.6 mm) / 2
         = 85.3 mm
         = 0.0853 m
```

The corresponding usable near, far, and center ball positions from the pivot are:

```text
r_near = 40.3 mm + 44.0 mm
       = 84.3 mm
       = 0.0843 m

r_far  = 40.3 mm + 126.6 mm
       = 166.9 mm
       = 0.1669 m

r₀     = 40.3 mm + 85.3 mm
       = 125.6 mm
       = 0.1256 m
```

This is the desired ball setpoint (center of the usable Sharp window).

---

## 3. Kinematics

The ball slides and rolls along the beam. The beam can rotate about the fixed pivot.
We express the ball's position in the inertial (lab) frame, then differentiate to get
velocity.

### 3.1 Ball Position

With the pivot at the origin, the ball's Cartesian coordinates in the inertial frame
are:

```text
x_I = r cos θ
y_I = r sin θ
```

where r is the distance along the beam and θ is the beam angle from horizontal.

### 3.2 Velocity Components

Differentiate with respect to time using the product and chain rules:

```text
ẋ_I = d/dt (r cos θ)
     = ṙ cos θ − r θ̇ sin θ

ẏ_I = d/dt (r sin θ)
     = ṙ sin θ + r θ̇ cos θ
```

The first term in each line is the rate of change of position along the beam; the
second term is the tangential velocity due to beam rotation.

### 3.3 Speed Squared

The square of the ball's speed in the inertial frame:

```text
v² = ẋ_I² + ẏ_I²
   = (ṙ cos θ − r θ̇ sin θ)² + (ṙ sin θ + r θ̇ cos θ)²
```

Expand:

```text
= ṙ² cos²θ − 2rṙθ̇ cosθ sinθ + r²θ̇² sin²θ
+ ṙ² sin²θ + 2rṙθ̇ sinθ cosθ + r²θ̇² cos²θ
```

The cross terms cancel. Group by cos²+sin²=1:

```text
v² = ṙ²(cos²θ + sin²θ) + r²θ̇²(sin²θ + cos²θ)
   = ṙ² + r²θ̇²
```

This is the standard result for polar coordinates: radial speed squared plus tangential
speed squared.

### 3.4 Rolling Constraint

The ball rolls without slipping along the beam. The contact point velocity relative to
the beam is the rate of change of r, i.e. ṙ. Therefore the ball's angular velocity
about its own centre is:

```text
ω_ball = ṙ / R
```

This constraint eliminates spin as an independent degree of freedom — the ball's
rotation is fully determined by its translational motion along the beam.

---

## 4. Energies

### 4.1 Ball Translational Kinetic Energy

The ball (mass m) moves through the inertial frame with speed v:

```text
T_trans = ½ m v²
        = ½ m (ṙ² + r²θ̇²)
```

### 4.2 Ball Rotational Kinetic Energy

Using the rolling constraint ω_ball = ṙ/R and I = (2/3) m R²:

```text
T_rot = ½ I ω_ball²
      = ½ × (2/3 m R²) × (ṙ/R)²
      = ½ × (2/3 m R²) × ṙ²/R²
      = (1/3) m ṙ²
```

Note that only radial motion (ṙ) contributes to ball spin — beam rotation θ̇ does not
spin the ball because θ̇ corresponds to the whole beam rotating, not the ball sliding
along it.

### 4.3 Beam Kinetic Energy

The beam assembly (mass M_b, inertia J about pivot) rotates at rate θ̇:

```text
T_beam = ½ J θ̇²
```

### 4.4 Total Kinetic Energy

Sum all three contributions:

```text
T = T_trans + T_rot + T_beam
  = ½ m (ṙ² + r²θ̇²) + (1/3) m ṙ² + ½ J θ̇²
  = ½ m ṙ² + (1/3) m ṙ² + ½ m r²θ̇² + ½ J θ̇²
  = (3/6 + 2/6) m ṙ² + ½(mr² + J)θ̇²
  = (5/6) m ṙ² + ½(mr² + J)θ̇²
```

The factor 5/6 in front of ṙ² is the hallmark of ball-and-beam dynamics: five-sixths
of the ball mass participates in radial kinetic energy because the remaining one-sixth
is stored in spin.

### 4.5 Potential Energy

Taking the pivot as the reference height, the potential energy has two contributions.

Ball at height r sin θ:

```text
V_ball = m g r sin θ
```

Beam centre of mass at height L_com sin θ:

```text
V_beam = M_b g L_com sin θ
```

Total:

```text
V = (m r + M_b L_com) g sin θ
```

### 4.6 Lagrangian

```text
ℒ = T − V

ℒ = (5/6) m ṙ² + ½(mr² + J)θ̇² − (m r + M_b L_com) g sin θ
```

---

## 5. Euler–Lagrange Equations

The Euler–Lagrange equations for each generalised coordinate qᵢ are:

```text
d/dt (∂ℒ/∂q̇ᵢ) − ∂ℒ/∂qᵢ = Qᵢ
```

where Qᵢ is the generalised force for coordinate i. Viscous damping is included as
generalised forces: Q_r = −b_x ṙ (rolling friction opposing sliding) and
Q_θ = τ − b_θ θ̇ (actuator torque minus pivot friction).

### 5.1 Equation in r (Ball Along Beam)

Compute the partial derivatives of ℒ:

```text
∂ℒ/∂ṙ = (5/3) m ṙ

d/dt(∂ℒ/∂ṙ) = (5/3) m r̈

∂ℒ/∂r = m r θ̇² − m g sin θ
```

Applying the Euler–Lagrange equation with Q_r = −b_x ṙ:

```text
(5/3) m r̈ − m r θ̇² + m g sin θ = −b_x ṙ
```

Rearranged into standard form:

```text
(5/3) m r̈ + b_x ṙ − m r θ̇² + m g sin θ = 0          ... (EOM-r)
```

Physical interpretation:

- (5/3) m r̈ — effective inertia (translational + rolling)
- b_x ṙ — viscous rolling damping
- −m r θ̇² — centrifugal force due to beam rotation
- m g sin θ — gravitational component along the beam

### 5.2 Equation in θ (Beam Rotation)

Compute the partial derivatives of ℒ:

```text
∂ℒ/∂θ̇ = (mr² + J) θ̇

d/dt(∂ℒ/∂θ̇) = (mr² + J) θ̈ + 2mr ṙ θ̇

∂ℒ/∂θ = −(m r + M_b L_com) g cos θ
```

Applying the Euler–Lagrange equation with Q_θ = τ − b_θ θ̇:

```text
(mr² + J) θ̈ + 2mr ṙ θ̇ + (mr + M_b L_com) g cos θ + b_θ θ̇ = τ
```

Rearranged:

```text
(J + mr²) θ̈ + 2mr ṙ θ̇ + b_θ θ̇ + (mr + M_b L_com) g cos θ = τ   ... (EOM-θ)
```

Physical interpretation:

- (J + mr²) θ̈ — total rotational inertia (beam + ball)
- 2mr ṙ θ̇ — Coriolis coupling between ball sliding and beam rotating
- b_θ θ̇ — pivot viscous damping
- (mr + M_b L_com) g cos θ — gravitational restoring torque
- τ — motor torque applied at beam (through crank linkage)

---

## 6. Nonlinear Model

Equations (EOM-r) and (EOM-θ) form the complete nonlinear coupled ODE system. Written
explicitly in state-space form with state **z** = [r, ṙ, θ, θ̇]ᵀ:

**Ball equation** (solving EOM-r for r̈):

```text
r̈ = (m r θ̇²) / m_e  −  (m g / m_e) sin θ  −  (b_x / m_e) ṙ
```

where `m_e = (5/3) m` is the effective rolling mass.

**Beam equation** (solving EOM-θ for θ̈):

```text
θ̈ = [ τ − 2mr ṙ θ̇ − b_θ θ̇ − (mr + M_b L_com) g cos θ ]
     / (J + mr²)
```

These two second-order ODEs must be integrated simultaneously because r appears in the
beam equation and θ appears in the ball equation.

**Effective rolling mass:**

```text
m_e = (5/3) m = (5/3) × 0.0028 = 0.004667 kg
```

The factor 5/3 arises directly from the 2/3 inertia factor of a thin-shell sphere:

```text
m_e = m + I/R² = m + (2/3)mR²/R² = m(1 + 2/3) = (5/3) m
```

---

## 7. Linearization about Horizontal Equilibrium

The nonlinear model is linearized for control design. The equilibrium of interest is
the beam horizontal with the ball at rest at position r₀:

```text
Equilibrium:  r = r₀,  θ = 0,  ṙ = 0,  θ̇ = 0
```

For this to be an equilibrium the actuator must supply a torque that balances gravity:

```text
τ₀ = (m r₀ + M_b L_com) g
```

In the control architecture below, θ is commanded directly, so τ₀ is handled
implicitly by the actuator.

### 7.1 Perturbation Variables

Define small deviations from equilibrium:

```text
r  = r₀ + x      (x is the small displacement along beam)
θ  = 0 + δ       (δ is the small beam angle)
ṙ  = ẋ
θ̇  = δ̇
```

We substitute into (EOM-r) and (EOM-θ) and drop all second-order terms (products of
small quantities: xδ, δ², ẋδ, xδ̇, etc.).

### 7.2 Linearized Ball Equation

Substitute into EOM-r:

```text
(5/3) m ẍ + b_x ẋ − m(r₀ + x)(δ̇)² + m g sinδ = 0
```

Second-order terms: the centrifugal term m(r₀+x)δ̇² is second-order (product of δ̇²)
and drops. Apply sinδ ≈ δ for small δ:

```text
(5/3) m ẍ + b_x ẋ + m g δ = 0
```

Divide through by m_e = (5/3) m:

```text
ẍ + β ẋ = − α g δ      ... (LBALL)
```

where:

```text
α = m / m_e  =  m / (5/3 m)  =  3/5  = 0.6            (dimensionless)
β = b_x / m_e  =  0.0125 / 0.004667  =  2.6786  s⁻¹
```

**Sign note:** With the positive-θ convention (motor side up), a positive `δ` tilts the
beam so that gravity pulls the ball toward the pivot (decreasing `r`). Any control
coordinate that defines positive position toward the motor side therefore needs the
actuation mapping to preserve the intended logical meaning of "positive command =
motor side up" rather than silently flipping PID or sensor signs. The derivation here
tracks the physics; sign bookkeeping is done in the unit-conversion step (§9).

### 7.3 Linearized Beam Equation

Substitute into EOM-θ with cosδ ≈ 1 and ṙ δ̇ ≈ 0:

```text
(J + m r₀²) δ̈ + b_θ δ̇ + (m r₀ + M_b L_com) g δ = τ − τ₀
```

This beam equation motivates a cascaded architecture with a fast beam-angle loop and a
slower ball-position loop. The implemented firmware record in §9 does not use the
superseded second-order actuator model from the previous draft of this document.

---

## 8. Numerical Specialization

All hardware values are now substituted. Every step is shown explicitly.

### 8.1 Effective Rolling Mass and Gravity Coefficients

```text
m_e = (5/3) × 0.0028
    = 0.0046667 kg

α   = 0.0028 / 0.0046667
    = 0.600000                          (exact, = 3/5)

β   = 0.0125 / 0.0046667
    = 2.678571  s⁻¹

α g = 0.600000 × 9.81
    = 5.886000  m/s²
```

### 8.2 Linearized Ball Plant (SI units)

From (LBALL):

```text
ẍ + 2.6786 ẋ = −5.886 δ          [x in m, δ in rad]
```

Laplace transform (zero initial conditions):

```text
s² X(s) + 2.6786 s X(s) = −5.886 Θ(s)

X(s) / Θ(s) = −5.886 / (s² + 2.6786 s)
             = −5.886 / (s (s + 2.6786))   [m/rad]
```

This is an integrator cascaded with a first-order lag (pole at s = −β).

### 8.3 Beam Inertia at Nominal Operating Point

At the design equilibrium r₀ = 0.1256 m (centre of the usable Sharp window):

```text
J_total(r₀) = J + m r₀²
             = 7.715 × 10⁻⁴ + 0.0028 × (0.1256)²
             = 7.715 × 10⁻⁴ + 0.0028 × 0.015775
             = 7.715 × 10⁻⁴ + 4.417 × 10⁻⁵
             = 8.157 × 10⁻⁴  kg·m²
```

### 8.4 Gravitational Restoring Torque Coefficient (Linearized)

```text
(m r₀ + M_b L_com) g = (0.0028 × 0.1256 + 0.0385 × 0.11805) × 9.81
                      = (0.00035168 + 0.00454494) × 9.81
                      = 0.00489662 × 9.81
                      = 0.04804  N·m/rad
```

The linearized beam equation at r₀ (ideal actuator, no damping) has natural frequency:

```text
ω_beam = √(0.04804 / 8.157 × 10⁻⁴)
        = √(58.89)
        = 7.67  rad/s  (1.22 Hz)
```

This time-scale separation supports a cascaded design with a fast beam-angle loop and a
slower ball-position loop. In the implemented firmware, however, the beam is not
treated as a perfect actuator: deadband, step quantization, step-rate limits, recovery
logic, and command saturation are retained explicitly.

---

## 9. Implemented Calibration and Controller Record

Sections 3-8 are the first-principles plant model used to motivate the control
architecture. This section records what was actually implemented in
`src/main.cpp` on April 11, 2026, and how the beam-angle calibration was obtained from
`src/calibration_main.cpp` and the April 7, 2026 same-session calibration run. All
runtime quantities in this section use the firmware units `cm`, `deg`, and `s`.

### 9.1 Runtime State Definitions

The controller variables used online are

```text
d_raw_cm        = instantaneous Sharp distance from the inverse-voltage fit
d_filt_cm       = slow filtered distance retained for telemetry/monitoring
d_vel_cm        = faster filtered distance used by the controller state
setpoint_cm     = active target distance
x_cm            = d_vel_cm - setpoint_cm
d_rate_cm_s     = d/dt of the velocity-oriented distance filter state
x_dot_cm_s      = filtered estimate of x_dot
theta_cal_deg   = calibrated AS5600 beam angle before balance zeroing
theta_rel_deg   = theta_cal_deg - theta_balance_deg
theta_dot_deg_s = filtered estimate of d/dt(theta_rel_deg)
xi_cm_s         = outer-loop integral state
```

Important implementation meanings are:

- `x_cm > 0` means the ball is toward the motor/far end.
- `theta_rel_deg > 0` is the empirically calibrated runtime direction that makes
  manual command `A+` roll the ball toward the motor/far end.
- `valid = distance_ok && theta_valid` is the combined telemetry health flag used in
  the CSV logs.
- `theta_balance_deg` is not part of the April 7 calibration fit. It is captured later
  on the assembled system by the `Z` command so that `theta_rel_deg = 0` represents the
  chosen balance reference for control.

### 9.2 Sharp Distance Measurement and Filtering

At each 40 ms control cycle, the firmware reads 8 ADC samples from the
GP2Y0A41SK0F and forms

```text
adc_mean[k] = round((1/8) * sum_{i=1..8} adc_i[k])
V_sharp[k]  = adc_mean[k] * (5.0 / 1023.0)
```

The distance calibration used online is

```text
d_raw_cm[k] = 12.25 / V_sharp[k] - 0.62
```

with validity conditions

```text
V_sharp[k] >= 0.08 V
4.40 cm <= d_raw_cm[k] <= 12.66 cm
```

For accepted samples, the controller uses a 3-sample median followed by two EMAs:

```text
m[k]       = median(last 3 valid distance samples)
d_filt[k]  = 0.25 * m[k] + 0.75 * d_filt[k-1]
d_vel[k]   = 0.60 * m[k] + 0.40 * d_vel[k-1]
d_rate[k]  = (d_vel[k] - d_vel[k-1]) / 0.04
```

The position-velocity estimate is then

```text
x_cm[k] = d_filt[k] - setpoint_cm[k]
```

with an adaptive near-side smoothing factor

```text
w_near[k] = clamp((8.53 - min(d_filt[k], setpoint_cm[k])) / (8.53 - 4.40), 0, 1)
alpha_xdot[k] = 0.12 + (0.28 - 0.12) * w_near[k]
x_dot_cm_s[k] = alpha_xdot[k] * d_rate[k] + (1 - alpha_xdot[k]) * x_dot_cm_s[k-1]
```

If a sample is invalid, `invalid_count` increments. In `M2`, `invalid_count >= 8`
declares a cascade sensor fault, resets dynamic controller state, and forces zero
command until valid sensing resumes.

### 9.3 AS5600 Beam-Angle Calibration and Nonlinear Beam/Stepper Relationship

The beam-angle calibration used by `main.cpp` comes from the dedicated calibration
firmware in `src/calibration_main.cpp` and the same-session run stored in
`calibration_logs/20260407_122510`.

The calibration procedure was:

- Use the dedicated AS5600 calibration firmware, not the main controller firmware.
- Capture the lower endpoint `L` and upper endpoint `U`.
- Sweep forward with `56` `N` commands and reverse with `56` `B` commands.
- Move `8` microsteps per `N` or `B` command at `1/16` microstepping
  (`3200` microsteps/rev).
- At each capture point, average `64` AS5600 raw samples and record the external beam
  angle from an iPhone level app taped cross-sectionally to the beam.

The controller uses the signed same-session affine fit

```text
theta_cal_deg = 0.07666806 * as5600_unwrapped_deg - 23.28443907
theta_rel_deg = theta_cal_deg - theta_balance_deg
```

This affine fit was chosen for online control because it is simple and already accurate
enough over the tested range. The measured calibrated range used by `main.cpp` is

```text
THETA_CAL_MIN_DEG         = -0.70
THETA_CAL_MAX_DEG         =  3.10
THETA_CAL_MARGIN_DEG      =  0.10
THETA_CAL_EXTRAPOLATE_DEG =  0.30

theta_soft_min = -0.70 + 0.10 - 0.30 = -0.90 deg
theta_soft_max =  3.10 - 0.10 + 0.30 =  3.30 deg
```

The corresponding actuator conversion used online is empirical rather than geometric:

```text
STEPS_PER_BEAM_DEG = (3200 / 360) / 0.07666806 = 115.9399219 microsteps/deg
```

The same-session signed fit quality reported in
`calibration_logs/20260407_122510/beam_angle_calibration_report.md` is:

| Fit | RMS error (deg) | Max abs error (deg) |
| --- | ---: | ---: |
| Forward sweep | 0.040024 | 0.123804 |
| Reverse sweep | 0.041749 | 0.091776 |
| Combined signed affine fit | 0.045209 | 0.117525 |

The measured hysteresis was:

```text
mean forward/reverse hysteresis = 0.046621 deg
max forward/reverse hysteresis  = 0.117778 deg
zero shift                      = 0.79102 AS5600-deg ≈ 7.03 microsteps
```

For thesis plots and offline reconstruction, the measured beam/stepper relationship is
better represented as the direction-averaged nonlinear LUT

```text
theta_nl_deg(a) = piecewise-linear interpolation through the signed LUT knots
                  with endpoint clamp on [293.44348, 344.21817] AS5600-deg
```

Representative LUT knots from the April 7, 2026 report are:

| `a = as5600_deg_unwrapped` | `theta_nl_deg(a)` |
| ---: | ---: |
| 293.44348 | -0.700000 |
| 299.56835 | -0.350000 |
| 304.10156 | 0.044173 |
| 309.37500 | 0.400000 |
| 316.66992 | 1.000000 |
| 329.57608 | 2.000000 |
| 343.91601 | 3.100000 |
| 344.21817 | 3.100000 |

The firmware currently uses the affine fit online. The nonlinear LUT is the thesis
record of the measured nonlinearity and should be preferred for offline plots or for a
future controller revision that needs the most faithful static mapping.

### 9.4 Implemented Operating Modes

The controller in `main.cpp` has three staged modes:

- `M0`: telemetry only. No control output is applied.
- `M1`: manual beam-angle hold using the AS5600 inner loop only. Command `A<deg>` sets
  `g_manual_theta_cmd_rel_deg`.
- `M2`: cascaded ball-position control. This mode is allowed only when
  `POSITION_CALIBRATED = true` and `theta_balance_set = true`.

Additional runtime facts from the implementation are:

- Boot mode is always `M0`.
- The TMC2209 driver remains disabled until `G` is received.
- `X` disables the driver and resets dynamic controller state.
- `Z` sets `theta_balance_deg = theta_cal_deg`, marks `theta_balance_set = true`, and
  clears both trim estimators.
- `S<cm>` updates the setpoint if `4.40 <= S <= 12.66`.
- The `C` command in `main.cpp` only toggles Sharp calibration telemetry; it is not the
  dedicated beam-angle calibration workflow from `src/calibration_main.cpp`.

### 9.5 Inner Beam-Angle Servo Implemented in `M1` and `M2`

The implemented inner loop is a discrete P servo with deadband, quantization-aware
rounding, and a per-cycle step clamp.

The calibrated command angle first becomes

```text
theta_cmd_cal_deg = clamp(theta_balance_deg + theta_cmd_rel_deg, -0.90, 3.30)
e_theta_deg       = theta_cmd_cal_deg - theta_cal_deg
```

If both

```text
|e_theta_deg| <= 0.040 deg
|theta_dot_deg_s| <= 0.60 deg/s
```

then the controller outputs zero step command and clears the fractional residual.
Otherwise it computes

```text
raw_steps[k] =
    INNER_STEP_SIGN
    * (INNER_KP_THETA * e_theta_deg[k] - INNER_KD_THETA * theta_dot_deg_s[k])
    * STEPS_PER_BEAM_DEG
    + residual[k-1]
```

with the actual firmware constants

```text
INNER_STEP_SIGN    = -1
INNER_KP_THETA     = 0.70
INNER_KD_THETA     = 0.00
STEPS_PER_BEAM_DEG = 115.9399219
```

The final step command is

```text
delta_steps[k] = clamp(round(raw_steps[k]), -40, 40)
```

If `delta_steps[k]` is not saturated, the residual keeps the fractional part
`raw_steps[k] - delta_steps[k]`; otherwise the residual is reset to zero.

The actual stepper execution still retains hardware timing details:

- `200 us` nominal step period
- `900 us` start/end ramp period
- `16` ramp steps
- `5 us` step pulse
- `1200 us` settle delay after direction reversal

Finally, `step_delta` is forced to zero whenever the driver is disabled, the AS5600 is
invalid, the mode is `M0`, calibration telemetry is active, or the cascade distance
sensor fault is asserted.

### 9.6 Outer Ball-Position Law Implemented in `M2`

The outer loop in `main.cpp` is not the LQI controller from the superseded draft of
this document. It is an empirically tuned, saturated state-feedback law with
integration, recovery logic, and optional learned position trim.

The derivative states are first limited:

```text
x_dot_ctrl     = clamp(x_dot_cm_s,      -4.50, 4.50)
theta_dot_ctrl = clamp(theta_dot_deg_s, -4.00, 4.00)
```

With the latest April 13, 2026 firmware settings,

```text
kx = 0.31 * gain_scale
kv = 0.17 * gain_scale
kt = 0.00 * gain_scale
kw = 0.00 * gain_scale
ki = 0.030 * gain_scale

gain_scale   = 1.0 by default
g_outer_sign = -1 by default
```

The positive-side asymmetry hooks all default to neutral values
(`OUTER_POS_X_KX_SCALE = OUTER_POS_X_KV_SCALE = OUTER_POS_X_RECOVERY_SCALE = 1`,
`OUTER_POS_X_INWARD_EXTRA_KV = 0`), so the default controller is symmetric even though
the code keeps those hooks available. In addition, when the ball is already moving
back toward the target, the velocity gain is tapered by

```text
kv_eff =
    kv * (0.70 + 0.30 * clamp((1.60 - |x_cm|) / 1.60, 0, 1))
```

so damping is reduced while the ball is still far away and fades back to full
strength near the hold region. To brake the center staircase capture harder, the
firmware also adds

```text
kv_eff += 0.20 * clamp((1.50 - |x_cm|) / 1.50, 0, 1)
```

whenever the ball is already moving inward.

The outer integrator is constrained by

```text
OUTER_INTEGRAL_CLAMP_DEG = 0.24 deg
xi_limit_cm_s            = OUTER_INTEGRAL_CLAMP_DEG / ki = 8.0 cm*s   (at gain_scale = 1)
```

The firmware now uses a two-band integral policy. Existing bias is retained inside the
memory region

```text
|x_cm|       <= 1.60 cm
|x_dot_ctrl| <= 0.65 cm/s
```

but new bias is accumulated only inside the tighter center-approach region

```text
|x_cm|       <= 0.95 cm
|x_dot_ctrl| <= 0.45 cm/s
```

When the active setpoint is the staircase center target, that accumulation gate is
relaxed to

```text
|x_cm|       <= 1.20 cm
|x_dot_ctrl| <= 0.75 cm/s
```

with an additional center band

```text
|x_cm|     <= 0.18 cm
|x_dot_ctrl| <= 0.35 cm/s
```

The firmware then applies the actual bleed rules from `main.cpp`:

```text
outside capture band: xi <- 0.988 * xi
during recovery:      xi <- 0.988 * xi
wrong sign:           xi <- 0.85  * xi      if x_cm * xi < 0
inside center band:   xi <- 0.996 * xi
```

When the active setpoint is center and the state is still close to center, the
wrong-sign bleed is softened to preserve hold bias through small vibration-driven
crossings:

```text
wrong sign near center: xi <- 0.96 * xi
                        for |x_cm| <= 0.80 cm and |x_dot_ctrl| <= 1.80 cm/s
```

Integration is allowed only when the distance estimate is valid, recovery is inactive,
the state is inside the active accumulation band, and the command is not
simultaneously saturated and already corrective in sign.

When the active setpoint is center and the state is inside the memory region, the
firmware also updates a bounded center-bias seed. It tracks the current zero-trim
estimate when available, otherwise the current integral state, and clamps that stored
memory to `±0.10 deg` equivalent before reuse.

For hold-only center operation, the firmware now also applies a separate bounded
center-hold trim feedforward

```text
center_hold_trim_deg =
    clamp(zero_trim_est_deg or ki * center_bias_xi_cm_s, -0.12, 0.12)
    * min(clamp((2.00 - |x_cm|) / 2.00, 0, 1),
          clamp((2.50 - |x_dot_cm_s|) / 2.50, 0, 1))
```

It is used only after the zero-trim estimate has at least `8` accepted samples, and it
is added after the outer state-feedback command. When the active setpoint is center,
the position-trim map is disabled so this dedicated center trim is the only learned
bias feedforward path.

The core state-feedback law is therefore

```text
i_term_deg =
    clamp(ki * xi_cm_s, -0.24, 0.24)

theta_fs_cmd_deg =
    g_outer_sign
    * (kx * x_cm
     + kv_eff * x_dot_ctrl
     + kt * theta_rel_deg
     + kw * theta_dot_ctrl
     + i_term_deg)
```

which is then limited by the runtime command cap

```text
theta_limited_deg = clamp(theta_fs_cmd_deg, -g_theta_cmd_limit_deg, g_theta_cmd_limit_deg)
```

with `g_theta_cmd_limit_deg = 2.00 deg` by default and an allowed operator adjustment
range of `0.10` to `3.00 deg`.

### 9.7 Recovery Logic, Zero-Trim, and Position-Trim Feedforward

The outer loop contains three additional implementation features that were missing or
misstated in the previous draft.

**1. Recovery logic**

Recovery is a minimum-corrective-angle floor used only when the ball appears stalled
off-center. The observable stalled condition is

```text
invalid_count < 8
|x_cm| >= 1.20 cm
|x_dot_ctrl| <= 0.35 cm/s
```

and it must persist for

```text
RECOVERY_ENTER_COUNT = 8 loops = 0.32 s
```

before `g_recovery_active` is asserted.

Recovery exits when any of the following becomes true:

```text
invalid_count >= 8
|x_cm| <= 0.25 cm
or
moving inward, |x_cm| <= 0.45 cm, and |x_dot_ctrl| >= 0.90 cm/s
```

The active recovery floor is

```text
recovery_floor_active_deg =
    clamp(0.70 + 0.18 * max(|x_cm| - 1.20, 0), 0.00, 1.10)
```

When recovery is still active but the ball is already moving inward, that floor
is tapered toward zero inside the center approach region:

```text
recovery_floor_active_deg *= clamp((|x_cm| - 0.25) / (1.20 - 0.25), 0, 1)
```

and if the saturated state-feedback command is smaller in magnitude than this floor,
the firmware replaces it with the minimum corrective angle of the proper sign.

Large inward staircase setpoint jumps also pre-arm recovery whenever

```text
|setpoint_new - setpoint_old| >= 1.20 cm
```

and the ball is still at least `1.20 cm` away from the new target after the jump.
The staircase center transition is excluded from this pre-arm path so the center stage
enters on the same recovery logic as isolated `M2` center hold.

**2. Zero-trim estimator**

The firmware maintains a slow estimate of the local balance angle. It is still logged
for telemetry, and it now also seeds the bounded center-bias memory that is reused when
the staircase returns to center. It updates only under tight near-equilibrium
conditions:

```text
|x_cm| <= 0.50 cm
|x_dot_cm_s| <= 1.00 cm/s
|theta_cmd_rel_deg - theta_rel_deg| <= 0.20 deg
|theta_dot_deg_s| <= 1.20 deg/s
```

When the active setpoint is center, the estimator tightens further to avoid learning
from small residual oscillations:

```text
|x_cm| <= 0.35 cm
|x_dot_cm_s| <= 0.45 cm/s
|theta_cmd_rel_deg - theta_rel_deg| <= 0.12 deg
|theta_dot_deg_s| <= 0.80 deg/s
```

and uses the EMA

```text
zero_trim_est_deg[k] =
    zero_trim_est_deg[k-1]
    + 0.05 * (theta_rel_deg[k] - zero_trim_est_deg[k-1])
```

During staircase phase changes, this estimate is no longer reset, so a center-hold bias
learned before the far excursion can be reused when the setpoint comes back to center.
The staircase center transition no longer seeds the outer integrator, so that reuse
happens only through the same center-hold trim path used in isolated `M2`.

**3. Position-trim map and feedforward**

The controller can learn a local position-dependent hold bias across

```text
POSITION_TRIM_BIN_COUNT = 9
```

setpoint bins spanning `[4.40, 12.66] cm`, but it is enabled only for

```text
setpoint_cm >= D_MIN_CM + 1.20 = 5.60 cm
```

Each bin stores an EMA of the locally required holding angle, clipped to
`[-0.60, 0.60] deg`, and samples are accepted only when

```text
|x_cm| <= 0.50 cm
|x_dot_cm_s| <= 1.00 cm/s
|theta_cmd_rel_deg - theta_rel_deg| <= 0.20 deg
|theta_dot_deg_s| <= 1.20 deg/s
```

The center setpoint is excluded from this map. Center hold now uses only the dedicated
center-trim path above, so position trim cannot stack with center trim and flip the
small-error command sign.

The lookup interpolates between populated neighboring bins, requires at least
`4` samples per usable bin, and refuses to use a trim point farther than `1.50 cm`
from the requested setpoint.

When available, the feedforward trim added after the outer controller is

```text
theta_trim_ff_deg = trim_lookup(setpoint_cm) * min(x_weight, xdot_weight)
```

where the weighting fades to zero outside

```text
|x_cm| <= 0.60 cm
|x_dot_cm_s| <= 0.60 cm/s
```

The final command chain implemented in `main.cpp` is therefore

```text
theta_ctrl_deg         = clamp_theta_rel_command(theta_limited_or_recovery_deg)
theta_cmd_prelimit_deg = theta_ctrl_deg + theta_trim_ff_deg
theta_cmd_rel_deg      = clamp_theta_rel_command(theta_cmd_prelimit_deg)
```

### 9.8 Staircase Experiment Logic and Telemetry Fields

The April 11, 2026 experiment used the built-in staircase command `E`, which is allowed
only in `M2` with the driver enabled and `theta_balance_set = true`.

The staircase parameters hard-coded in `main.cpp` are

```text
STAIRCASE_STAGE_MS         = 20000 ms
STAIRCASE_TOTAL_MS         = 60000 ms
STAIRCASE_FAR_SETPOINT_CM  = 11.46
STAIRCASE_CENTER_SETPOINT_CM = 8.53
STAIRCASE_NEAR_SETPOINT_CM = 4.90
```

Every staircase setpoint change resets the dynamic controller states tied to the old
target: `x_cm`, `x_dot_cm_s`, `theta_dot_deg_s`, `theta_fs_cmd_deg`,
`theta_cmd_rel_deg`, `xi_cm_s`, and the inner residual. The distance filter states and
the zero-trim estimator are preserved so the physical motion estimate and learned
center-hold bias remain continuous across the phase change. When the staircase enters
the center phase, it does not seed the outer integrator or pre-arm recovery, so the
center entry uses the same controller path as isolated `M2` center hold.

The most important telemetry columns for later thesis analysis are:

- `theta_fs_cmd_deg`: outer-loop command before position-trim feedforward.
- `theta_trim_ff_deg`: learned local trim contribution.
- `theta_cmd_prelimit_deg`: `theta_ctrl_deg + theta_trim_ff_deg`.
- `theta_cmd_rel_deg`: final relative beam-angle command sent to the inner loop.
- `xi_cm_s`: outer integrator state.
- `recovery_active`, `recovery_count`, `recovery_floor_deg`: recovery status.
- `valid`, `theta_valid`, `invalid_count`: sensor-health status.

---

## 10. Implementation Constants and Experiment Record

### 10.1 Implementation-Sourced Runtime Constants (`src/main.cpp`)

| Group | Constants | Value | Meaning |
| --- | --- | --- | --- |
| Timing | `LOOP_MS`, `DT` | `40 ms`, `0.040 s` | Main control cadence |
| Sharp fit | `SHARP_ADC_SAMPLES`, `SHARP_FIT_K_V_CM`, `SHARP_FIT_OFFSET_CM`, `SHARP_MIN_VALID_V` | `8`, `12.25`, `-0.62`, `0.08 V` | Distance acquisition and inverse-voltage calibration |
| Distance window | `D_MIN_CM`, `D_MAX_CM`, `D_SETPOINT_DEFAULT_CM` | `4.40`, `12.66`, `8.53 cm` | Valid operating range and default center setpoint |
| Distance filtering | `DIST_EMA_ALPHA`, `DIST_VEL_EMA_ALPHA`, `X_DOT_EMA_ALPHA`, `X_DOT_EMA_ALPHA_NEAR`, `DIST_INVALID_LIMIT` | `0.25`, `0.78`, `0.20`, `0.40`, `8` | Slow telemetry distance filter, faster control distance filter, velocity filtering, and fault threshold |
| Angle calibration | `AS5600_RAW_TO_DEG`, `THETA_SLOPE_DEG_PER_AS_DEG`, `THETA_OFFSET_DEG`, `THETA_DOT_EMA_ALPHA` | `360/4096`, `0.07666806`, `-23.28443907`, `0.30` | AS5600 conversion and affine beam-angle fit |
| Angle safety window | `THETA_CAL_MIN_DEG`, `THETA_CAL_MAX_DEG`, `THETA_CAL_MARGIN_DEG`, `THETA_CAL_EXTRAPOLATE_DEG` | `-0.70`, `3.10`, `0.10`, `0.30 deg` | Soft commandable beam-angle envelope `[-0.90, 3.30] deg` |
| Stepper conversion | `STEPS_PER_REV`, `STEPS_PER_BEAM_DEG`, `DIR_SIGN`, `INNER_STEP_SIGN` | `3200`, `115.9399219`, `-1`, `-1` | Empirical stepper-to-beam conversion and sign chain |
| Inner servo | `INNER_KP_THETA`, `INNER_KD_THETA`, `INNER_THETA_DEADBAND_DEG`, `INNER_THETA_RATE_DEADBAND_DEG_S`, `INNER_MAX_STEP_DELTA` | `0.70`, `0.00`, `0.040`, `0.60`, `40` | Discrete P angle servo, deadband, and step clamp |
| Step timing | `STEP_PERIOD_US`, `STEP_START_PERIOD_US`, `STEP_RAMP_STEPS`, `STEP_PULSE_US`, `DIR_REVERSE_SETTLE_US` | `200`, `900`, `16`, `5`, `1200 us` | Step pulse generation and reversal settling |
| Command limits | `THETA_CMD_LIMIT_DEFAULT_DEG`, `OUTER_GAIN_SCALE_MIN/MAX` | `2.00 deg`, `0.50` to `1.50` | Default command cap and outer-loop gain-scaling range |
| Outer gains | `OUTER_KX_DEFAULT`, `OUTER_KV_DEFAULT`, `OUTER_KT_DEFAULT`, `OUTER_KW_DEFAULT`, `OUTER_KI_DEFAULT`, `g_outer_sign` | `0.31`, `0.17`, `0.00`, `0.00`, `0.030`, `-1` | Default saturated state-feedback law |
| Inward damping | `OUTER_INWARD_DAMP_BAND_CM`, `OUTER_INWARD_DAMP_MIN_SCALE`, `OUTER_CENTER_INWARD_BRAKE_BAND_CM`, `OUTER_CENTER_INWARD_EXTRA_KV`, `OUTER_POS_X_KX_SCALE`, `OUTER_POS_X_KV_SCALE`, `OUTER_POS_X_INWARD_BRAKE_BAND_CM`, `OUTER_POS_X_INWARD_EXTRA_KV`, `OUTER_POS_X_RECOVERY_SCALE` | `1.60 cm`, `0.70`, `1.50 cm`, `0.20`, `1.00`, `1.00`, `2.00 cm`, `0.00`, `1.00` | Symmetric inward-damping taper, added near-center inward brake, plus neutral positive-side asymmetry hooks |
| Integral management | `OUTER_INTEGRAL_CLAMP_DEG`, `OUTER_INTEGRAL_CAPTURE_CM`, `OUTER_INTEGRAL_CAPTURE_X_DOT_CM_S`, `OUTER_INTEGRAL_ACCUM_CM`, `OUTER_INTEGRAL_ACCUM_X_DOT_CM_S`, `OUTER_CENTER_INTEGRAL_ACCUM_CM`, `OUTER_CENTER_INTEGRAL_ACCUM_X_DOT_CM_S`, `OUTER_INTEGRAL_BLEED_OUTSIDE`, `OUTER_INTEGRAL_BLEED_RECOVERY`, `OUTER_INTEGRAL_BLEED_CENTER`, `OUTER_INTEGRAL_BLEED_WRONG_SIGN`, `OUTER_INTEGRAL_BLEED_WRONG_SIGN_CENTER`, `OUTER_WRONG_SIGN_CENTER_X_CM`, `OUTER_WRONG_SIGN_CENTER_X_DOT_CM_S` | `0.24 deg`, `1.60 cm`, `0.65 cm/s`, `0.95 cm`, `0.45 cm/s`, `1.20 cm`, `0.75 cm/s`, `0.988`, `0.988`, `0.996`, `0.85`, `0.96`, `0.80 cm`, `1.80 cm/s` | Integral memory region, default accumulation region, relaxed center accumulation region, clamp, and center-protected bleed factors |
| Center bias memory | `CENTER_SETPOINT_TOL_CM`, `OUTER_CENTER_BIAS_MEMORY_ALPHA`, `OUTER_CENTER_BIAS_MEMORY_MAX_DEG` | `0.05 cm`, `0.10`, `0.10 deg` | Center-only low-frequency bias memory retained across staircase phases and available to the center-hold trim fallback |
| Center hold trim | `CENTER_HOLD_TRIM_MAX_ABS_DEG`, `CENTER_HOLD_TRIM_APPLY_X_CM`, `CENTER_HOLD_TRIM_APPLY_X_DOT_CM_S`, `CENTER_HOLD_TRIM_MIN_COUNT` | `0.12 deg`, `2.00 cm`, `2.50 cm/s`, `8` | Bounded center-hold feedforward derived from zero trim or center-bias memory after a minimum settled-sample count |
| Center/rate limits | `OUTER_CENTER_BAND_CM`, `OUTER_CENTER_BAND_X_DOT_CM_S`, `OUTER_X_DOT_LIMIT_CM_S`, `OUTER_THETA_DOT_LIMIT_DEG_S` | `0.18 cm`, `0.35 cm/s`, `4.50 cm/s`, `4.0 deg/s` | Near-target band and derivative clipping |
| Recovery | `RECOVERY_ENTER_X_CM`, `RECOVERY_ENTER_X_DOT_CM_S`, `RECOVERY_ENTER_COUNT`, `RECOVERY_EXIT_X_CM`, `RECOVERY_EXIT_HANDOFF_X_CM`, `RECOVERY_EXIT_INWARD_X_DOT_CM_S`, `RECOVERY_INWARD_FLOOR_TAPER_X_CM`, `RECOVERY_FLOOR_DEFAULT_DEG`, `RECOVERY_FLOOR_MAX_DEG`, `RECOVERY_FLOOR_GAIN_DEG_PER_CM` | `1.20 cm`, `0.35 cm/s`, `8`, `0.25 cm`, `0.45 cm`, `0.90 cm/s`, `1.20 cm`, `0.70 deg`, `1.10 deg`, `0.18 deg/cm` | Stall recovery detection, handoff, and inward-tapered corrective floor |
| Zero trim | `ZERO_TRIM_EST_X_CM`, `ZERO_TRIM_EST_X_DOT_CM_S`, `ZERO_TRIM_EST_THETA_TRACK_ERR_DEG`, `ZERO_TRIM_EST_THETA_DOT_DEG_S`, `ZERO_TRIM_EST_CENTER_X_CM`, `ZERO_TRIM_EST_CENTER_X_DOT_CM_S`, `ZERO_TRIM_EST_CENTER_THETA_TRACK_ERR_DEG`, `ZERO_TRIM_EST_CENTER_THETA_DOT_DEG_S`, `ZERO_TRIM_EST_ALPHA` | `0.50 cm`, `1.00 cm/s`, `0.20 deg`, `1.20 deg/s`, `0.35 cm`, `0.45 cm/s`, `0.12 deg`, `0.80 deg/s`, `0.05` | Local balance-angle estimator with tighter center-only capture before center-hold trim learns from it |
| Position trim | `POSITION_TRIM_BIN_COUNT`, `POSITION_TRIM_CAPTURE_X_CM`, `POSITION_TRIM_CAPTURE_X_DOT_CM_S`, `POSITION_TRIM_CAPTURE_THETA_TRACK_ERR_DEG`, `POSITION_TRIM_CAPTURE_THETA_DOT_DEG_S`, `POSITION_TRIM_MAX_ABS_DEG`, `POSITION_TRIM_ALPHA`, `POSITION_TRIM_MAX_USE_DISTANCE_CM`, `POSITION_TRIM_APPLY_X_CM`, `POSITION_TRIM_APPLY_X_DOT_CM_S`, `POSITION_TRIM_ENABLE_MIN_CM`, `POSITION_TRIM_MIN_COUNT` | `9`, `0.50 cm`, `1.00 cm/s`, `0.20 deg`, `1.20 deg/s`, `0.60 deg`, `0.08`, `1.50 cm`, `0.60 cm`, `0.60 cm/s`, `5.60 cm`, `4` | Learned local setpoint bias map for non-center targets only, with relaxed capture thresholds for real beam vibration |
| Staircase | `STAIRCASE_STAGE_MS`, `STAIRCASE_FAR_SETPOINT_CM`, `STAIRCASE_CENTER_SETPOINT_CM`, `STAIRCASE_NEAR_SETPOINT_CM`, `SETPOINT_STEP_PREARM_MIN_CM` | `20000 ms`, `11.46`, `8.53`, `4.90 cm`, `1.20 cm` | Built-in thesis experiment sequence and inward-step recovery pre-arm threshold for non-center stages |

### 10.2 Experiment Record: April 11, 2026 Telemetry Run

The hardware run recorded in
`telemetry_logs/20260411_142117/telemetry.csv` reflects the April 11, 2026 cascade
tuning that preceded the April 13 retune documented above, not the older LQI design
from the previous version of this document.

The logged experiment record is:

- Total rows: `364`, spanning `t_ms = 6471` to `79072`.
- Initial telemetry-only phase: `7` rows in `M0`.
- Cascade phase: `357` rows in `M2`, beginning at `t_ms = 7879`.
- Staircase started at `t_ms = 14074`.
- Far phase: `100` rows at `12.16 cm`.
- Center phase: `100` rows at `8.53 cm`, beginning at `t_ms = 34073`.
- Near phase: `100` rows at `4.90 cm`, beginning at `t_ms = 54076`.
- After the near phase, the controller continued holding `4.90 cm` until the log ended.

The run-wide controller settings observed in the CSV were:

- `gain_scale = 1.000` for all rows.
- `outer_sign = -1` for all rows.
- `theta_balance_set = 1` for all rows.
- Base recovery floor `= 0.700 deg`.
- `theta_trim_ff_deg = 0.0000` for every logged row, so no learned position-trim
  feedforward was active during this experiment.

The logged nonidealities were:

- `43` rows with `valid = 0`, caused by intermittent invalid Sharp samples.
- `9` rows with `recovery_active = 1`.
- Dynamic recovery floor briefly rising to `1.10 deg` during the larger staircase
  transitions.

This is therefore the correct thesis-facing record of what was actually run on the
hardware on April 11, 2026:

- first-principles plant model from Sections 3-8,
- affine online AS5600 calibration with nonlinear LUT retained for offline reference,
- tuned cascaded controller from `src/main.cpp`,
- and the recorded staircase experiment from `telemetry_logs/20260411_142117`.
