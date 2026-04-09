# Beam Angle Calibration Report

Run directory: `calibration_logs/20260407_122510`

Raw data source: `calibration_logs/20260407_122510/calibration_data.csv`

Diagnosis source: `calibration_logs/20260407_122510/same_session_diagnosis.txt`

This report uses only the valid same-session full sweep from `20260407_122510`. It does not use the aborted `20260407_122246` run or the earlier split-session forward/reverse runs for the final relationship.

## Experiment Context

- Sensor: AS5600 on motor side.
- AS5600 I2C address: `0x36`.
- AS5600 wiring: `A4 -> SDA`, `A5 -> SCL`, `5V -> VCC/VIN`, `GND -> GND`.
- Stepper driver: TMC2209 using STEP/DIR/EN.
- Microstepping: `1/16`, so `3200` microsteps per motor revolution.
- Sweep increment: `8` microsteps per `N` or `B` command.
- External angle reference: iPhone level app taped cross-sectionally to the beam.
- Data collected in one continuous session: `L`, `U`, `56` forward `N` samples, and `56` reverse `B` samples.
- The iPhone values behaved like unsigned magnitude readings near the zero crossing, so signed beam angle was reconstructed from AS5600 position relative to the measured zero crossing.

## Sign Convention

The signed beam angle convention used here is:

- `theta < 0`: upper-side region before the iPhone-level zero crossing.
- `theta = 0`: iPhone-level zero from this experiment.
- `theta > 0`: lower-end direction after the zero crossing.

Important: `theta = 0` here is the iPhone-level zero, not necessarily the ball's true balanced beam angle. A separate `theta_balance` offset must still be measured before final controller tuning.

## Signed Mathematical Mapping

Use this affine mapping for first telemetry integration:

```text
theta_deg = 0.07666806 * as5600_deg_unwrapped - 23.28443907
```

Equivalent zero-centered form:

```text
theta_deg = 0.07666806 * (as5600_deg_unwrapped - 303.70455)
```

Convert to radians for controller state use:

```text
theta_rad = theta_deg * 0.01745329252
```

Do not use the unsigned affine fit from `fit_report.txt` for controller beam angle. That report was generated from app magnitude values and does not encode the signed angle crossing required for control.

## Fit Quality

Same-session signed fit results:

| Fit | RMS error (deg) | Max abs error (deg) |
|---|---:|---:|
| Forward sweep | 0.040024 | 0.123804 |
| Reverse sweep | 0.041749 | 0.091776 |
| Combined signed fit | 0.045209 | 0.117525 |

Hysteresis estimate:

| Metric | Value |
|---|---:|
| Mean forward/reverse hysteresis | 0.046621 deg |
| Max forward/reverse hysteresis | 0.117778 deg |
| Forward/reverse zero shift | 0.79102 deg AS5600 |
| Forward/reverse zero shift | about 7.03 microsteps |

The same-session hysteresis is small enough for telemetry and initial controller experiments. The earlier larger mismatch seen across split sessions was mostly setup/session drift, not severe mechanical backlash.

## Controller Inference

This calibration is good enough to proceed with a telemetry-only four-state build using:

```text
x          = filtered ball position
x_dot      = derivative of filtered ball position
theta      = signed beam angle from AS5600 using this mapping
theta_dot  = derivative of filtered theta
```

Do not close the loop with a new full-state controller until live telemetry confirms sign, scaling, zero offset, and derivative noise. The controller should first print and validate `x`, `x_dot`, `theta`, and `theta_dot`.

For first implementation, use the signed affine map because it is simple and the combined RMS error is low. Use the direction-averaged LUT below if the controller needs the most faithful calibrated curve. In both cases, measure and subtract a separate `theta_balance` offset before final controller tuning.

## Lookup Table Construction

The LUT below is a signed direction-averaged calibration table:

- Independent variable: `as5600_deg_unwrapped`.
- Output: `theta_lut_deg`.
- Forward values were signed using forward zero `304.10156 deg`.
- Reverse values were signed using reverse zero `303.31054 deg`.
- The forward sweep AS5600 sample positions, plus endpoints, were used as the grid.
- Reverse signed theta was linearly interpolated onto that grid.
- `theta_lut_deg` is the average of forward and reverse theta when both exist.
- At the upper edge where reverse interpolation is unavailable, the single available forward value is used.

Recommended LUT runtime behavior:

- Sort by `as5600_deg_unwrapped`.
- Clamp below `293.44348 deg` to the first row.
- Clamp above `344.21817 deg` to the last row.
- Linearly interpolate between neighboring rows.
- Convert `theta_lut_deg` to radians only after interpolation.

## Direction-Averaged Signed LUT

| as5600_deg_unwrapped | theta_lut_deg | theta_forward_deg | theta_reverse_interp_deg | reverse_minus_forward_deg |
|---:|---:|---:|---:|---:|
| 293.44348 | -0.700000 | -0.700000 | -0.700000 | 0.000000 |
| 294.16992 | -0.661941 | -0.700000 | -0.623881 | 0.076119 |
| 295.04882 | -0.650000 | -0.700000 | -0.600000 | 0.100000 |
| 296.01562 | -0.559091 | -0.600000 | -0.518182 | 0.081818 |
| 296.97830 | -0.459304 | -0.500000 | -0.418608 | 0.081392 |
| 297.77343 | -0.450000 | -0.500000 | -0.400000 | 0.100000 |
| 298.56860 | -0.361385 | -0.400000 | -0.322770 | 0.077230 |
| 299.56835 | -0.350000 | -0.400000 | -0.300000 | 0.100000 |
| 300.49804 | -0.260000 | -0.300000 | -0.220000 | 0.080000 |
| 301.29321 | -0.160849 | -0.200000 | -0.121697 | 0.078303 |
| 302.16796 | -0.150000 | -0.200000 | -0.100000 | 0.100000 |
| 303.13476 | -0.059052 | -0.100000 | -0.018104 | 0.081896 |
| 304.10156 | 0.044173 | 0.000000 | 0.088347 | 0.088347 |
| 304.89257 | 0.100000 | 0.100000 | 0.100000 | 0.000000 |
| 305.77148 | 0.141629 | 0.100000 | 0.183258 | 0.083258 |
| 306.73828 | 0.200000 | 0.200000 | 0.200000 | 0.000000 |
| 307.70507 | 0.244523 | 0.200000 | 0.289046 | 0.089046 |
| 308.56475 | 0.347680 | 0.300000 | 0.395361 | 0.095361 |
| 309.37500 | 0.400000 | 0.400000 | 0.400000 | 0.000000 |
| 310.42007 | 0.495949 | 0.500000 | 0.491899 | -0.008101 |
| 311.30999 | 0.591339 | 0.600000 | 0.582679 | -0.017321 |
| 312.18750 | 0.600000 | 0.600000 | 0.600000 | 0.000000 |
| 312.99496 | 0.690936 | 0.700000 | 0.681872 | -0.018128 |
| 314.03320 | 0.745455 | 0.700000 | 0.790909 | 0.090909 |
| 315.00000 | 0.845455 | 0.800000 | 0.890909 | 0.090909 |
| 315.79101 | 0.940763 | 0.900000 | 0.981526 | 0.081526 |
| 316.66992 | 1.000000 | 1.000000 | 1.000000 | 0.000000 |
| 317.66009 | 1.042775 | 1.000000 | 1.085549 | 0.085549 |
| 318.61041 | 1.100000 | 1.100000 | 1.100000 | 0.000000 |
| 319.48242 | 1.194368 | 1.200000 | 1.188735 | -0.011265 |
| 320.35580 | 1.250000 | 1.300000 | 1.200000 | -0.100000 |
| 321.32812 | 1.291667 | 1.300000 | 1.283333 | -0.016667 |
| 322.29492 | 1.395000 | 1.400000 | 1.390000 | -0.010000 |
| 323.08593 | 1.490000 | 1.500000 | 1.480000 | -0.020000 |
| 323.96484 | 1.550000 | 1.600000 | 1.500000 | -0.100000 |
| 324.93164 | 1.641111 | 1.700000 | 1.582222 | -0.117778 |
| 325.89843 | 1.690909 | 1.700000 | 1.681818 | -0.018182 |
| 326.69223 | 1.785270 | 1.800000 | 1.770540 | -0.029460 |
| 327.61096 | 1.887463 | 1.900000 | 1.874927 | -0.025073 |
| 328.55163 | 1.937216 | 1.900000 | 1.974432 | 0.074432 |
| 329.57608 | 2.000000 | 2.000000 | 2.000000 | 0.000000 |
| 330.38085 | 2.088888 | 2.100000 | 2.077777 | -0.022223 |
| 331.25976 | 2.137871 | 2.100000 | 2.175742 | 0.075742 |
| 332.22656 | 2.200000 | 2.200000 | 2.200000 | 0.000000 |
| 333.19335 | 2.286363 | 2.300000 | 2.272727 | -0.027273 |
| 334.07226 | 2.300000 | 2.300000 | 2.300000 | 0.000000 |
| 334.92370 | 2.384856 | 2.400000 | 2.369713 | -0.030287 |
| 335.91796 | 2.486363 | 2.500000 | 2.472727 | -0.027273 |
| 336.88476 | 2.586364 | 2.600000 | 2.572727 | -0.027273 |
| 337.67578 | 2.600000 | 2.600000 | 2.600000 | 0.000000 |
| 338.55468 | 2.682984 | 2.700000 | 2.665968 | -0.034032 |
| 339.60937 | 2.786885 | 2.800000 | 2.773770 | -0.026230 |
| 340.56655 | 2.800000 | 2.800000 | 2.800000 | 0.000000 |
| 341.36718 | 2.882487 | 2.900000 | 2.864975 | -0.035025 |
| 342.24609 | 2.984640 | 3.000000 | 2.969280 | -0.030720 |
| 343.23764 | 3.000000 | 3.000000 | 3.000000 | 0.000000 |
| 343.91601 | 3.100000 | 3.100000 | 3.100000 | 0.000000 |
| 344.21817 | 3.100000 | 3.100000 | NA | NA |
