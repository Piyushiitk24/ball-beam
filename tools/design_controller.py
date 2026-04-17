#!/usr/bin/env python3
"""
Fit simple inner/outer models from telemetry and emit suggested firmware gains.

The tool assumes the controller architecture used by controller_rev 2:

  - inner angle loop tracks `theta_cmd_rel_deg -> theta_rel_deg`
  - outer plant uses `theta_rel_deg -> x_cm`
  - outer controller is an LQI law on `[x, x_dot, integral(x)]`

It uses only the Python standard library so it can run in this repo without
NumPy or SciPy.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from typing import Iterable, List, Sequence


EPS = 1.0e-9


def coerce(value: str):
    if value is None:
        return value
    text = value.strip()
    if text == "":
        return text
    try:
        if any(ch in text for ch in ".eE"):
            return float(text)
        return int(text)
    except ValueError:
        return text


def load_rows(path: str) -> List[dict]:
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        return [{key: coerce(value) for key, value in row.items()} for row in reader]


def median_positive(values: Iterable[float]) -> float:
    positives = [value for value in values if value > 0.0]
    if not positives:
        raise ValueError("no positive values available")
    return statistics.median(positives)


def fit_pairs(rows: Sequence[dict], mode_values: Sequence[int]) -> List[tuple]:
    pairs = []
    for prev_row, row in zip(rows, rows[1:]):
        if prev_row.get("mode") not in mode_values:
            continue
        if row.get("mode") not in mode_values:
            continue
        if prev_row.get("driver_enabled") != 1 or row.get("driver_enabled") != 1:
            continue
        if prev_row.get("theta_valid") != 1 or row.get("theta_valid") != 1:
            continue
        dt_s = (float(row["t_ms"]) - float(prev_row["t_ms"])) * 0.001
        if dt_s <= 0.0:
            continue
        pairs.append((prev_row, row, dt_s))
    return pairs


def fit_outer_pairs(rows: Sequence[dict]) -> List[tuple]:
    pairs = []
    for prev_row, row in zip(rows, rows[1:]):
        if prev_row.get("mode") != 2 or row.get("mode") != 2:
            continue
        if prev_row.get("driver_enabled") != 1 or row.get("driver_enabled") != 1:
            continue
        if prev_row.get("valid") != 1 or row.get("valid") != 1:
            continue
        if prev_row.get("theta_valid") != 1 or row.get("theta_valid") != 1:
            continue
        if float(prev_row.get("setpoint_cm", 0.0)) != float(row.get("setpoint_cm", 0.0)):
            continue
        dt_s = (float(row["t_ms"]) - float(prev_row["t_ms"])) * 0.001
        if dt_s <= 0.0:
            continue
        pairs.append((prev_row, row, dt_s))
    return pairs


def solve_linear_system(matrix: List[List[float]], rhs: List[float]) -> List[float]:
    n = len(rhs)
    a = [row[:] + [rhs[idx]] for idx, row in enumerate(matrix)]

    for pivot_col in range(n):
        pivot_row = max(range(pivot_col, n), key=lambda row: abs(a[row][pivot_col]))
        if abs(a[pivot_row][pivot_col]) < EPS:
            raise ValueError("singular normal equation matrix")
        if pivot_row != pivot_col:
            a[pivot_col], a[pivot_row] = a[pivot_row], a[pivot_col]

        pivot = a[pivot_col][pivot_col]
        for col in range(pivot_col, n + 1):
            a[pivot_col][col] /= pivot

        for row in range(n):
            if row == pivot_col:
                continue
            factor = a[row][pivot_col]
            if abs(factor) < EPS:
                continue
            for col in range(pivot_col, n + 1):
                a[row][col] -= factor * a[pivot_col][col]

    return [a[row][n] for row in range(n)]


def ordinary_least_squares(features: Sequence[Sequence[float]],
                           targets: Sequence[float]) -> List[float]:
    cols = len(features[0])
    ata = [[0.0 for _ in range(cols)] for _ in range(cols)]
    aty = [0.0 for _ in range(cols)]

    for feat, target in zip(features, targets):
        for row in range(cols):
            aty[row] += feat[row] * target
            for col in range(cols):
                ata[row][col] += feat[row] * feat[col]

    return solve_linear_system(ata, aty)


def rmse(predictions: Sequence[float], targets: Sequence[float]) -> float:
    return math.sqrt(
        sum((pred - target) * (pred - target) for pred, target in zip(predictions, targets))
        / float(len(predictions))
    )


def fit_inner_model(rows: Sequence[dict]) -> dict:
    pairs = fit_pairs(rows, mode_values=(1, 2))
    if len(pairs) < 6:
        raise ValueError("not enough valid inner-loop samples")

    sample_dt_s = median_positive(dt_s for _, _, dt_s in pairs)
    max_gap_s = 3.0 * sample_dt_s

    features = []
    targets = []
    for prev_row, row, dt_s in pairs:
        if dt_s > max_gap_s:
            continue
        features.append([
            float(prev_row["theta_rel_deg"]),
            float(prev_row["theta_cmd_rel_deg"]),
            1.0,
        ])
        targets.append(float(row["theta_rel_deg"]))

    coeffs = ordinary_least_squares(features, targets)
    predictions = [
        sum(weight * value for weight, value in zip(coeffs, feat))
        for feat in features
    ]
    a_coeff, b_coeff, c_coeff = coeffs
    dc_gain = None
    tau_s = None
    if abs(1.0 - a_coeff) > EPS:
        dc_gain = b_coeff / (1.0 - a_coeff)
    if 0.0 < abs(a_coeff) < 1.0:
        tau_s = -sample_dt_s / math.log(abs(a_coeff))

    return {
        "sample_count": len(features),
        "sample_dt_s": sample_dt_s,
        "a": a_coeff,
        "b": b_coeff,
        "c": c_coeff,
        "dc_gain": dc_gain,
        "tau_s": tau_s,
        "rmse_deg": rmse(predictions, targets),
    }


def fit_outer_model(rows: Sequence[dict]) -> dict:
    pairs = fit_outer_pairs(rows)
    if len(pairs) < 6:
        raise ValueError("not enough valid outer-loop samples")

    sample_dt_s = median_positive(dt_s for _, _, dt_s in pairs)
    max_gap_s = 3.0 * sample_dt_s

    features = []
    targets = []
    for prev_row, row, dt_s in pairs:
        if dt_s > max_gap_s:
            continue
        x_ddot_cm_s2 = (float(row["x_dot_cm_s"]) - float(prev_row["x_dot_cm_s"])) / dt_s
        features.append([
            float(prev_row["x_dot_cm_s"]),
            float(prev_row["theta_rel_deg"]),
            1.0,
        ])
        targets.append(x_ddot_cm_s2)

    coeffs = ordinary_least_squares(features, targets)
    predictions = [
        sum(weight * value for weight, value in zip(coeffs, feat))
        for feat in features
    ]
    a_v, b_theta, bias = coeffs

    return {
        "sample_count": len(features),
        "sample_dt_s": sample_dt_s,
        "a_v": a_v,
        "b_theta": b_theta,
        "bias": bias,
        "rmse_cm_s2": rmse(predictions, targets),
    }


def mat_mul(left: Sequence[Sequence[float]],
            right: Sequence[Sequence[float]]) -> List[List[float]]:
    rows = len(left)
    cols = len(right[0])
    inner = len(right)
    return [
        [
            sum(left[row][idx] * right[idx][col] for idx in range(inner))
            for col in range(cols)
        ]
        for row in range(rows)
    ]


def mat_add(left: Sequence[Sequence[float]],
            right: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [left[row][col] + right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def mat_sub(left: Sequence[Sequence[float]],
            right: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [left[row][col] - right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def mat_transpose(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    return [list(col) for col in zip(*matrix)]


def max_abs_entry(matrix: Sequence[Sequence[float]]) -> float:
    return max(abs(value) for row in matrix for value in row)


def discretize_outer_model(a_v: float, b_theta: float, dt_s: float) -> tuple:
    if abs(a_v) < 1.0e-6:
        a_11 = 1.0
        a_12 = dt_s
        a_22 = 1.0
        b_1 = 0.5 * b_theta * dt_s * dt_s
        b_2 = b_theta * dt_s
    else:
        exp_term = math.exp(a_v * dt_s)
        a_11 = 1.0
        a_12 = (exp_term - 1.0) / a_v
        a_22 = exp_term
        b_1 = b_theta * ((exp_term - 1.0) - (a_v * dt_s)) / (a_v * a_v)
        b_2 = b_theta * (exp_term - 1.0) / a_v

    a_matrix = [
        [a_11, a_12],
        [0.0, a_22],
    ]
    b_matrix = [
        [b_1],
        [b_2],
    ]
    return a_matrix, b_matrix


def solve_dare_single_input(a_matrix: Sequence[Sequence[float]],
                            b_matrix: Sequence[Sequence[float]],
                            q_matrix: Sequence[Sequence[float]],
                            r_value: float,
                            max_iters: int = 4000) -> tuple:
    p_matrix = [row[:] for row in q_matrix]
    a_t = mat_transpose(a_matrix)
    b_t = mat_transpose(b_matrix)

    for _ in range(max_iters):
        btpb = mat_mul(b_t, mat_mul(p_matrix, b_matrix))[0][0]
        denom = r_value + btpb
        if abs(denom) < EPS:
            raise ValueError("singular Riccati denominator")
        k_matrix = [[value / denom for value in row]
                    for row in mat_mul(b_t, mat_mul(p_matrix, a_matrix))]
        s_matrix = [[denom]]
        p_next = mat_add(
            q_matrix,
            mat_sub(
                mat_mul(a_t, mat_mul(p_matrix, a_matrix)),
                mat_mul(mat_transpose(k_matrix), mat_mul(s_matrix, k_matrix)),
            ),
        )
        diff = max_abs_entry(mat_sub(p_next, p_matrix))
        p_matrix = p_next
        if diff < 1.0e-10:
            break

    btpb = mat_mul(b_t, mat_mul(p_matrix, b_matrix))[0][0]
    denom = r_value + btpb
    k_matrix = [[value / denom for value in row]
                for row in mat_mul(b_t, mat_mul(p_matrix, a_matrix))]
    return p_matrix, k_matrix


def design_lqi(outer_fit: dict,
               design_dt_s: float,
               qx: float,
               qv: float,
               qi: float,
               r_value: float) -> dict:
    a_matrix, b_matrix = discretize_outer_model(outer_fit["a_v"],
                                                outer_fit["b_theta"],
                                                design_dt_s)
    a_aug = [
        [a_matrix[0][0], a_matrix[0][1], 0.0],
        [a_matrix[1][0], a_matrix[1][1], 0.0],
        [design_dt_s, 0.0, 1.0],
    ]
    b_aug = [
        [b_matrix[0][0]],
        [b_matrix[1][0]],
        [0.0],
    ]
    q_matrix = [
        [qx, 0.0, 0.0],
        [0.0, qv, 0.0],
        [0.0, 0.0, qi],
    ]
    _, gain = solve_dare_single_input(a_aug, b_aug, q_matrix, r_value)
    return {
        "design_dt_s": design_dt_s,
        "kx": gain[0][0],
        "kv": gain[0][1],
        "ki": gain[0][2],
        "outer_sign": -1 if outer_fit["b_theta"] >= 0.0 else 1,
    }


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def suggest_inner_constants(inner_fit: dict) -> dict:
    tau_s = inner_fit.get("tau_s") or 0.145
    dc_gain = abs(inner_fit.get("dc_gain") or 0.85)

    kp = clamp(0.80 / max(dc_gain, 0.50), 0.70, 1.20)
    ki = clamp(0.065 / max(tau_s, 0.08), 0.30, 0.80)
    kd = clamp(0.40 * tau_s, 0.03, 0.08)
    ref_rate_deg_s = clamp(1.75 / max(tau_s, 0.08), 8.0, 18.0)

    return {
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "integral_clamp_deg": 0.20,
        "ref_rate_deg_s": ref_rate_deg_s,
    }


def print_summary(rows: Sequence[dict],
                  inner_fit: dict,
                  outer_fit: dict,
                  lqi: dict,
                  inner_constants: dict) -> None:
    print(f"Loaded rows: {len(rows)}")
    print()
    print("Inner fit")
    print("  theta[k+1] = a * theta[k] + b * cmd[k] + c")
    print(f"  samples: {inner_fit['sample_count']}")
    print(f"  sample_dt_s: {inner_fit['sample_dt_s']:.4f}")
    print(f"  a: {inner_fit['a']:.6f}")
    print(f"  b: {inner_fit['b']:.6f}")
    print(f"  c: {inner_fit['c']:.6f}")
    if inner_fit["dc_gain"] is not None:
        print(f"  dc_gain: {inner_fit['dc_gain']:.6f}")
    if inner_fit["tau_s"] is not None:
        print(f"  tau_s: {inner_fit['tau_s']:.4f}")
    print(f"  rmse_deg: {inner_fit['rmse_deg']:.4f}")
    print()

    print("Outer fit")
    print("  x_ddot = a_v * x_dot + b_theta * theta + bias")
    print(f"  samples: {outer_fit['sample_count']}")
    print(f"  sample_dt_s: {outer_fit['sample_dt_s']:.4f}")
    print(f"  a_v: {outer_fit['a_v']:.6f}")
    print(f"  b_theta: {outer_fit['b_theta']:.6f}")
    print(f"  bias: {outer_fit['bias']:.6f}")
    print(f"  rmse_cm_s2: {outer_fit['rmse_cm_s2']:.4f}")
    print()

    print("LQI design")
    print(f"  design_dt_s: {lqi['design_dt_s']:.4f}")
    print(f"  outer_sign: {lqi['outer_sign']}")
    print(f"  kx: {lqi['kx']:.6f}")
    print(f"  kv: {lqi['kv']:.6f}")
    print(f"  ki: {lqi['ki']:.6f}")
    print()

    print("Suggested firmware block")
    print(f"static const uint8_t CONTROLLER_REV = 2;")
    print(f"static const float OUTER_LQI_KX_DEFAULT = {lqi['kx']:.4f}f;")
    print(f"static const float OUTER_LQI_KV_DEFAULT = {lqi['kv']:.4f}f;")
    print(f"static const float OUTER_LQI_KI_DEFAULT = {lqi['ki']:.4f}f;")
    print(f"static const float INNER_KP_THETA = {inner_constants['kp']:.4f}f;")
    print(f"static const float INNER_KI_THETA = {inner_constants['ki']:.4f}f;")
    print(f"static const float INNER_KD_THETA = {inner_constants['kd']:.4f}f;")
    print(
        f"static const float INNER_INTEGRAL_CLAMP_DEG = "
        f"{inner_constants['integral_clamp_deg']:.4f}f;"
    )
    print(
        f"static const float INNER_REF_MAX_RATE_DEG_S = "
        f"{inner_constants['ref_rate_deg_s']:.4f}f;"
    )


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit controller_rev 2 model parameters from telemetry.",
    )
    parser.add_argument("--log", required=True, help="Path to telemetry CSV")
    parser.add_argument(
        "--design-dt",
        type=float,
        default=0.040,
        help="Controller design timestep in seconds (default: 0.040)",
    )
    parser.add_argument("--qx", type=float, default=4.0, help="LQI state weight on x")
    parser.add_argument("--qv", type=float, default=1.0, help="LQI state weight on x_dot")
    parser.add_argument("--qi", type=float, default=4.0, help="LQI state weight on integral(x)")
    parser.add_argument("--r", type=float, default=80.0, help="LQI input weight")
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    rows = load_rows(args.log)
    if len(rows) < 8:
        print("telemetry log is too short", file=sys.stderr)
        return 1

    try:
        inner_fit = fit_inner_model(rows)
        outer_fit = fit_outer_model(rows)
        lqi = design_lqi(outer_fit,
                         design_dt_s=args.design_dt,
                         qx=args.qx,
                         qv=args.qv,
                         qi=args.qi,
                         r_value=args.r)
        inner_constants = suggest_inner_constants(inner_fit)
    except ValueError as exc:
        print(f"fit failed: {exc}", file=sys.stderr)
        return 1

    print_summary(rows, inner_fit, outer_fit, lqi, inner_constants)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
