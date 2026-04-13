#!/usr/bin/env python3
"""Generate plots and LaTeX-ready tables for the ball and beam report."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

REPORT_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = REPORT_DIR.parent
FIGURES_DIR = REPORT_DIR / "figures"
TABLES_DIR = REPORT_DIR / "tables"
MPLCONFIG_DIR = REPORT_DIR / "build" / "matplotlib"
MPLCONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIG_DIR))

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
except ImportError as exc:  # pragma: no cover - environment dependent
    raise SystemExit(
        "Missing Python dependencies for the report.\n"
        "Install them with:\n"
        "  ./.venv/bin/python -m pip install -r report/requirements.txt"
    ) from exc

TELEMETRY_CSV = PROJECT_DIR / "telemetry_logs" / "20260411_142117" / "telemetry.csv"
CALIBRATION_CSV = (
    PROJECT_DIR
    / "calibration_logs"
    / "combined_20260407_forward_reverse"
    / "combined_cleaned_calibration.csv"
)

RUNTIME_THETA_SLOPE = 0.07666806
RUNTIME_THETA_OFFSET = -23.28443907
DISPLAY_INTERPOLATION_LIMIT = 2
DISPLAY_SMOOTH_WINDOW = 5
TRANSIENT_WINDOW_S = 8.0
HOLD_DETAIL_WINDOW_S = 6.0


@dataclass
class StepMetrics:
    name: str
    transition: str
    step_size_cm: float
    valid_count: int
    total_count: int
    peak_abs_error_cm: float | None
    rmse_cm: float | None
    mae_cm: float | None
    steady_state_error_cm: float | None
    overshoot_cm: float | None
    wrong_way_excursion_cm: float | None
    rise_time_s: float | None
    settling_time_s: float | None
    settling_band_cm: float


@dataclass
class SegmentView:
    index: int
    name: str
    setpoint_cm: float
    transition_label: str
    start_s: float
    end_s: float
    df: pd.DataFrame
    previous_setpoint_cm: float | None = None


def latex_escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    escaped = value
    for src, dst in replacements.items():
        escaped = escaped.replace(src, dst)
    return escaped


def fmt_float(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return "N/A"
    return f"{value:.{digits}f}"


def fmt_int(value: int) -> str:
    return f"{value:d}"


def require_columns(frame: pd.DataFrame, columns: list[str], source: Path) -> None:
    missing = [col for col in columns if col not in frame.columns]
    if missing:
        raise SystemExit(
            f"Missing expected columns in {source}: {', '.join(missing)}"
        )


def spans_from_mask(times: np.ndarray, mask: np.ndarray) -> list[tuple[float, float]]:
    spans: list[tuple[float, float]] = []
    if len(times) == 0:
        return spans
    start: float | None = None
    previous_time = float(times[0])
    for time_value, is_active in zip(times, mask, strict=False):
        current_time = float(time_value)
        if bool(is_active) and start is None:
            start = current_time
        if not bool(is_active) and start is not None:
            spans.append((start, previous_time))
            start = None
        previous_time = current_time
    if start is not None:
        spans.append((start, previous_time))
    return spans


def add_step_change_lines(ax: plt.Axes, times: list[float]) -> None:
    for step_time in times:
        ax.axvline(step_time, color="0.7", linestyle="--", linewidth=0.9, zorder=0)


def crossing_time(times: np.ndarray, values: np.ndarray, threshold: float, rising: bool) -> float | None:
    if rising:
        indices = np.flatnonzero(values >= threshold)
    else:
        indices = np.flatnonzero(values <= threshold)
    if indices.size == 0:
        return None
    return float(times[int(indices[0])])


def infer_step_name(index: int, target: float, unique_levels: np.ndarray) -> str:
    sorted_levels = np.sort(unique_levels)
    if len(sorted_levels) >= 3:
        low = float(sorted_levels[0])
        mid = float(sorted_levels[1])
        high = float(sorted_levels[-1])
        if math.isclose(target, high, abs_tol=1e-6):
            return "Far step"
        if math.isclose(target, mid, abs_tol=1e-6) and index > 0:
            return "Center return"
        if math.isclose(target, low, abs_tol=1e-6):
            return "Near step"
    return f"Step {index + 1}"


def make_display_trace(series: pd.Series, valid_mask: pd.Series) -> pd.Series:
    masked = series.where(valid_mask)
    interpolated = masked.interpolate(
        limit=DISPLAY_INTERPOLATION_LIMIT,
        limit_direction="both",
        limit_area="inside",
    )
    return interpolated.rolling(
        window=DISPLAY_SMOOTH_WINDOW,
        center=True,
        min_periods=1,
    ).mean()


def transient_window_end(frame: pd.DataFrame) -> float:
    return min(TRANSIENT_WINDOW_S, float(frame["t_rel_s"].iloc[-1]))


def hold_detail_start(frame: pd.DataFrame) -> float:
    end_time = float(frame["t_rel_s"].iloc[-1])
    if end_time <= HOLD_DETAIL_WINDOW_S:
        return 0.0
    return end_time - HOLD_DETAIL_WINDOW_S


def padded_limits(values: list[float], min_margin: float = 0.2, frac: float = 0.12) -> tuple[float, float]:
    finite_values = [float(value) for value in values if math.isfinite(float(value))]
    if not finite_values:
        return 0.0, 1.0

    lower = min(finite_values)
    upper = max(finite_values)
    span = upper - lower
    margin = max(min_margin, frac * max(span, 1e-9))
    if span < 1e-9:
        margin = max(min_margin, 0.15 * max(abs(lower), 1.0))
    return lower - margin, upper + margin


def build_segment_views(closed_loop: pd.DataFrame) -> list[SegmentView]:
    raw_segments = [segment.copy() for _, segment in closed_loop.groupby("segment_id", sort=True)]
    unique_levels = closed_loop["setpoint_cm"].dropna().unique()
    segment_views: list[SegmentView] = []

    for index, segment in enumerate(raw_segments):
        segment = segment.copy()
        segment["t_rel_s"] = segment["t_s"] - float(segment["t_s"].iloc[0])
        segment["display_position_cm"] = make_display_trace(
            segment["d_filt_cm"],
            segment["valid_bool"],
        )
        segment["display_error_cm"] = segment["display_position_cm"] - segment["setpoint_cm"]
        setpoint_cm = float(segment["setpoint_cm"].iloc[0])
        previous_setpoint_cm = None
        if index == 0:
            name = "Initial center hold"
            transition_label = f"Hold at {setpoint_cm:.2f} cm"
        else:
            previous_setpoint_cm = float(raw_segments[index - 1]["setpoint_cm"].iloc[-1])
            name = infer_step_name(index - 1, setpoint_cm, unique_levels)
            transition_label = f"{previous_setpoint_cm:.2f} -> {setpoint_cm:.2f} cm"

        segment_views.append(
            SegmentView(
                index=index,
                name=name,
                setpoint_cm=setpoint_cm,
                transition_label=transition_label,
                start_s=float(segment["t_s"].iloc[0]),
                end_s=float(segment["t_s"].iloc[-1]),
                df=segment,
                previous_setpoint_cm=previous_setpoint_cm,
            )
        )

    return segment_views


def compute_step_metrics(segment_views: list[SegmentView]) -> list[StepMetrics]:
    metrics: list[StepMetrics] = []

    for index in range(1, len(segment_views)):
        previous_segment = segment_views[index - 1]
        current_segment = segment_views[index]
        prev_setpoint = float(previous_segment.setpoint_cm)
        target_setpoint = float(current_segment.setpoint_cm)
        step_size_cm = target_setpoint - prev_setpoint
        step_sign = 1.0 if step_size_cm >= 0.0 else -1.0
        step_name = current_segment.name

        valid_segment = current_segment.df.loc[current_segment.df["valid_bool"]].copy()
        metric = StepMetrics(
            name=step_name,
            transition=f"{prev_setpoint:.2f} -> {target_setpoint:.2f}",
            step_size_cm=step_size_cm,
            valid_count=int(valid_segment.shape[0]),
            total_count=int(current_segment.df.shape[0]),
            peak_abs_error_cm=None,
            rmse_cm=None,
            mae_cm=None,
            steady_state_error_cm=None,
            overshoot_cm=None,
            wrong_way_excursion_cm=None,
            rise_time_s=None,
            settling_time_s=None,
            settling_band_cm=max(0.2, 0.05 * abs(step_size_cm)),
        )

        if valid_segment.empty:
            metrics.append(metric)
            continue

        time_rel = valid_segment["t_rel_s"].to_numpy()
        measurement = valid_segment["d_filt_cm"].to_numpy()
        error = valid_segment["x_cm"].to_numpy()

        metric.peak_abs_error_cm = float(np.max(np.abs(error)))
        metric.rmse_cm = float(np.sqrt(np.mean(np.square(error))))
        metric.mae_cm = float(np.mean(np.abs(error)))
        window = min(10, len(error))
        metric.steady_state_error_cm = float(np.mean(error[-window:]))

        directional_error = step_sign * error
        metric.overshoot_cm = float(max(0.0, float(np.max(directional_error))))

        start_measurement = float(measurement[0])
        movement_toward_target = step_sign * (measurement - start_measurement)
        metric.wrong_way_excursion_cm = float(max(0.0, -float(np.min(movement_toward_target))))

        if not math.isclose(target_setpoint, start_measurement, abs_tol=1e-9):
            threshold_10 = start_measurement + 0.10 * (target_setpoint - start_measurement)
            threshold_90 = start_measurement + 0.90 * (target_setpoint - start_measurement)
            rising = target_setpoint > start_measurement
            t10 = crossing_time(time_rel, measurement, threshold_10, rising=rising)
            t90 = crossing_time(time_rel, measurement, threshold_90, rising=rising)
            if t10 is not None and t90 is not None and t90 >= t10:
                metric.rise_time_s = float(t90 - t10)

        within_band = np.abs(error) <= metric.settling_band_cm
        for settle_index, settled in enumerate(within_band):
            if settled and bool(np.all(within_band[settle_index:])):
                metric.settling_time_s = float(time_rel[settle_index])
                break

        metrics.append(metric)

    return metrics


def write_text(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def make_signal_mapping_table() -> str:
    rows = [
        ("t_ms", "Elapsed experiment time", "Converted to seconds from the first logged sample."),
        ("setpoint_cm", "Reference ball position", "Controller target in Sharp distance coordinates."),
        ("d_filt_cm", "Measured ball position", "Filtered distance used for control and plotting."),
        ("x_cm", "Tracking error", "Defined in firmware as d_filt_cm - setpoint_cm."),
        ("x_dot_cm_s", "Ball velocity estimate", "Filtered derivative used by the outer loop."),
        ("theta_rel_deg", "Measured beam angle", "Runtime beam-angle coordinate after balance zeroing."),
        ("theta_dot_deg_s", "Beam-angle rate", "Filtered derivative of theta_rel_deg."),
        ("theta_cmd_rel_deg", "Final control input", "Relative beam-angle command sent to the inner loop."),
        ("valid", "Measurement validity", "Position-derived metrics only use rows where valid = 1."),
        ("mode and driver_enabled", "Closed-loop filter", "Analysis keeps rows with mode = 2 and driver_enabled = 1."),
    ]
    lines = [
        r"\small",
        r"\begin{tabularx}{\linewidth}{@{}l l X@{}}",
        r"\toprule",
        r"Telemetry column & Mapped quantity & Note \\",
        r"\midrule",
    ]
    for column, quantity, note in rows:
        lines.append(
            f"{latex_escape(column)} & {latex_escape(quantity)} & {latex_escape(note)} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabularx}"])
    return "\n".join(lines) + "\n"


def make_run_summary_table(stats: dict[str, float | int | str]) -> str:
    rows = [
        ("Telemetry rows", fmt_int(int(stats["telemetry_rows"]))),
        ("Closed-loop rows", fmt_int(int(stats["closed_loop_rows"]))),
        ("Duration [s]", fmt_float(float(stats["duration_s"]))),
        ("Invalid rows", fmt_int(int(stats["invalid_rows"]))),
        ("Recovery-active rows", fmt_int(int(stats["recovery_rows"]))),
        ("Command-saturated rows", fmt_int(int(stats["saturated_rows"]))),
        ("Max abs tracking error [cm]", fmt_float(float(stats["max_abs_error_cm"]))),
        ("Max abs beam angle [deg]", fmt_float(float(stats["max_abs_beam_angle_deg"]))),
        ("Max abs command [deg]", fmt_float(float(stats["max_abs_command_deg"]))),
        ("Closed-loop setpoints [cm]", latex_escape(str(stats["setpoint_levels"]))),
    ]
    lines = [
        r"\small",
        r"\begin{tabularx}{0.78\linewidth}{@{}l X@{}}",
        r"\toprule",
        r"Metric & Value \\",
        r"\midrule",
    ]
    for metric, value in rows:
        lines.append(f"{latex_escape(metric)} & {value} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabularx}"])
    return "\n".join(lines) + "\n"


def make_calibration_summary_table(stats: dict[str, float | int]) -> str:
    rows = [
        ("Grid points", fmt_int(int(stats["point_count"]))),
        ("AS5600 range [deg]", f"{fmt_float(float(stats['as_min']))} to {fmt_float(float(stats['as_max']))}"),
        ("Forward theta range [deg]", f"{fmt_float(float(stats['forward_min']))} to {fmt_float(float(stats['forward_max']))}"),
        ("Reverse theta range [deg]", f"{fmt_float(float(stats['reverse_min']))} to {fmt_float(float(stats['reverse_max']))}"),
        ("Mean hysteresis [deg]", fmt_float(float(stats["mean_hysteresis"]))),
        ("RMS hysteresis [deg]", fmt_float(float(stats["rms_hysteresis"]))),
        ("Max hysteresis [deg]", fmt_float(float(stats["max_hysteresis"]))),
        ("Forward affine fit", f"{fmt_float(float(stats['forward_slope']), 5)} x + {fmt_float(float(stats['forward_intercept']), 5)}"),
        ("Reverse affine fit", f"{fmt_float(float(stats['reverse_slope']), 5)} x + {fmt_float(float(stats['reverse_intercept']), 5)}"),
    ]
    lines = [
        r"\small",
        r"\begin{tabularx}{0.82\linewidth}{@{}l X@{}}",
        r"\toprule",
        r"Metric & Value \\",
        r"\midrule",
    ]
    for metric, value in rows:
        lines.append(f"{latex_escape(metric)} & {latex_escape(value)} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabularx}"])
    return "\n".join(lines) + "\n"


def make_step_metrics_table(metrics: list[StepMetrics]) -> str:
    lines = [
        r"\scriptsize",
        r"\setlength{\tabcolsep}{4pt}",
        r"\begin{tabularx}{\linewidth}{@{}l l r r r r r r r r r@{}}",
        r"\toprule",
        r"Step & Transition & $\Delta$ [cm] & Valid & Peak $|e|$ & RMSE & MAE & SSE & Ov. & Wrong-way & Rise / settle [s] \\",
        r"\midrule",
    ]
    for metric in metrics:
        rise_and_settle = f"{fmt_float(metric.rise_time_s)} / {fmt_float(metric.settling_time_s)}"
        valid_ratio = f"{metric.valid_count}/{metric.total_count}"
        lines.append(
            " & ".join(
                [
                    latex_escape(metric.name),
                    latex_escape(metric.transition),
                    fmt_float(metric.step_size_cm, 2),
                    latex_escape(valid_ratio),
                    fmt_float(metric.peak_abs_error_cm),
                    fmt_float(metric.rmse_cm),
                    fmt_float(metric.mae_cm),
                    fmt_float(metric.steady_state_error_cm),
                    fmt_float(metric.overshoot_cm),
                    fmt_float(metric.wrong_way_excursion_cm),
                    latex_escape(rise_and_settle),
                ]
            )
            + r" \\"
        )
    lines.extend([r"\bottomrule", r"\end{tabularx}"])
    return "\n".join(lines) + "\n"


def write_generated_macros(stats: dict[str, float | int]) -> None:
    lines = [
        rf"\renewcommand{{\TelemetryRowCount}}{{{int(stats['telemetry_rows'])}}}",
        rf"\renewcommand{{\TelemetryClosedLoopRowCount}}{{{int(stats['closed_loop_rows'])}}}",
        rf"\renewcommand{{\TelemetryDurationSeconds}}{{{stats['duration_s']:.3f}}}",
        rf"\renewcommand{{\TelemetryInvalidRowCount}}{{{int(stats['invalid_rows'])}}}",
        rf"\renewcommand{{\TelemetryRecoveryRowCount}}{{{int(stats['recovery_rows'])}}}",
        rf"\renewcommand{{\TelemetryFarValidCount}}{{{int(stats['far_valid_count'])}}}",
        rf"\renewcommand{{\TelemetryFarTotalCount}}{{{int(stats['far_total_count'])}}}",
        rf"\renewcommand{{\TelemetryCenterValidCount}}{{{int(stats['center_valid_count'])}}}",
        rf"\renewcommand{{\TelemetryCenterTotalCount}}{{{int(stats['center_total_count'])}}}",
        rf"\renewcommand{{\TelemetryNearValidCount}}{{{int(stats['near_valid_count'])}}}",
        rf"\renewcommand{{\TelemetryNearTotalCount}}{{{int(stats['near_total_count'])}}}",
        rf"\renewcommand{{\CalibrationPointCount}}{{{int(stats['calibration_point_count'])}}}",
        rf"\renewcommand{{\CalibrationMeanHysteresisDeg}}{{{stats['calibration_mean_hysteresis']:.3f}}}",
        rf"\renewcommand{{\CalibrationMaxHysteresisDeg}}{{{stats['calibration_max_hysteresis']:.3f}}}",
        rf"\renewcommand{{\CalibrationRmsHysteresisDeg}}{{{stats['calibration_rms_hysteresis']:.3f}}}",
        rf"\renewcommand{{\TelemetryMaxAbsErrorCm}}{{{stats['max_abs_error_cm']:.3f}}}",
        rf"\renewcommand{{\TelemetryMaxBeamAngleDeg}}{{{stats['max_abs_beam_angle_deg']:.3f}}}",
        rf"\renewcommand{{\TelemetryMaxCommandDeg}}{{{stats['max_abs_command_deg']:.3f}}}",
        "",
    ]
    write_text(TABLES_DIR / "generated_macros.tex", "\n".join(lines))


def prepare_telemetry() -> tuple[pd.DataFrame, pd.DataFrame]:
    telemetry = pd.read_csv(TELEMETRY_CSV)
    require_columns(
        telemetry,
        [
            "t_ms",
            "mode",
            "driver_enabled",
            "valid",
            "recovery_active",
            "staircase_active",
            "staircase_phase",
            "d_filt_cm",
            "setpoint_cm",
            "x_cm",
            "x_dot_cm_s",
            "theta_rel_deg",
            "theta_dot_deg_s",
            "theta_cmd_rel_deg",
            "theta_cmd_sat",
            "step_pos",
            "step_delta",
        ],
        TELEMETRY_CSV,
    )

    for column in telemetry.columns:
        telemetry[column] = pd.to_numeric(telemetry[column], errors="coerce")
    telemetry["t_ms"] = pd.to_numeric(telemetry["t_ms"], errors="coerce")
    telemetry["t_s"] = (telemetry["t_ms"] - float(telemetry["t_ms"].iloc[0])) / 1000.0
    telemetry["valid_bool"] = telemetry["valid"].fillna(0).astype(int).eq(1)
    telemetry["closed_loop_bool"] = (
        telemetry["mode"].fillna(-1).astype(int).eq(2)
        & telemetry["driver_enabled"].fillna(0).astype(int).eq(1)
    )

    closed_loop = telemetry.loc[telemetry["closed_loop_bool"]].copy()
    if closed_loop.empty:
        raise SystemExit(f"No closed-loop rows found in {TELEMETRY_CSV}")

    segment_change = closed_loop["setpoint_cm"].diff().abs().fillna(0.0) > 1e-9
    closed_loop["segment_id"] = segment_change.cumsum().astype(int)
    return telemetry, closed_loop


def prepare_calibration() -> pd.DataFrame:
    calibration = pd.read_csv(CALIBRATION_CSV)
    require_columns(
        calibration,
        [
            "as5600_deg_unwrapped",
            "forward_theta_deg",
            "reverse_theta_deg",
            "reverse_minus_forward_deg",
        ],
        CALIBRATION_CSV,
    )
    for column in calibration.columns:
        calibration[column] = pd.to_numeric(calibration[column], errors="coerce")
    calibration = calibration.dropna().copy()
    if calibration.empty:
        raise SystemExit(f"No usable calibration rows found in {CALIBRATION_CSV}")
    return calibration


def configure_plot_style() -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
        }
    )


def plot_telemetry_overview(segment_views: list[SegmentView]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(10.8, 7.4), sharey=False)
    axes = axes.flatten()

    for axis, segment in zip(axes, segment_views, strict=False):
        frame = segment.df
        valid_frame = frame.loc[frame["valid_bool"]]
        invalid_frame = frame.loc[~frame["valid_bool"]]
        display_frame = frame.loc[frame["display_position_cm"].notna()]

        axis.plot(
            frame["t_rel_s"],
            frame["setpoint_cm"],
            color="#1f77b4",
            linewidth=2.0,
            label="setpoint" if segment.index == 0 else None,
        )
        axis.plot(
            valid_frame["t_rel_s"],
            valid_frame["d_filt_cm"],
            color="#d62728",
            linewidth=0.9,
            alpha=0.28,
            label="raw valid" if segment.index == 0 else None,
        )
        axis.plot(
            display_frame["t_rel_s"],
            display_frame["display_position_cm"],
            color="#d62728",
            linewidth=2.1,
            label="display-smoothed" if segment.index == 0 else None,
        )
        if not invalid_frame.empty:
            axis.scatter(
                invalid_frame["t_rel_s"],
                invalid_frame["d_filt_cm"],
                color="#9467bd",
                marker="x",
                s=18,
                alpha=0.35,
                label="invalid sample" if segment.index == 0 else None,
                zorder=3,
            )

        y_values = valid_frame["d_filt_cm"].tolist() + display_frame["display_position_cm"].tolist()
        y_values.append(segment.setpoint_cm)
        if segment.previous_setpoint_cm is not None:
            y_values.append(segment.previous_setpoint_cm)
        axis.set_xlim(0.0, math.ceil(float(frame["t_rel_s"].iloc[-1]) * 2.0) / 2.0)
        axis.set_ylim(*padded_limits(y_values, min_margin=0.28, frac=0.08))
        axis.set_title(segment.name)
        axis.text(
            0.02,
            0.04,
            f"{segment.transition_label}\nvalid {int(frame['valid_bool'].sum())}/{len(frame)}",
            transform=axis.transAxes,
            ha="left",
            va="bottom",
            fontsize=8.5,
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.85, "edgecolor": "0.8"},
        )
        axis.set_xlabel("local time [s]")

    axes[0].set_ylabel("position [cm]")
    axes[2].set_ylabel("position [cm]")
    axes[0].legend(loc="best")
    fig.suptitle("Ball and beam telemetry overview by staircase phase", y=0.98, fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.965))
    fig.savefig(FIGURES_DIR / "telemetry_overview.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_position_tracking(segment_views: list[SegmentView]) -> None:
    step_segments = segment_views[1:]
    fig, axes = plt.subplots(
        len(step_segments),
        2,
        figsize=(11.2, 8.6),
        sharex=False,
        gridspec_kw={"width_ratios": [1.8, 1.0]},
    )

    for row_axes, segment in zip(axes, step_segments, strict=False):
        transition_axis, hold_axis = row_axes
        frame = segment.df
        valid_frame = frame.loc[frame["valid_bool"]]
        display_frame = frame.loc[frame["display_position_cm"].notna()]
        transition_end = transient_window_end(frame)
        hold_start = hold_detail_start(frame)
        hold_frame = frame.loc[frame["t_rel_s"] >= hold_start]
        hold_valid = hold_frame.loc[hold_frame["valid_bool"]]
        hold_display = hold_frame.loc[hold_frame["display_position_cm"].notna()]

        for axis in (transition_axis, hold_axis):
            axis.plot(
                frame["t_rel_s"],
                frame["setpoint_cm"],
                color="#1f77b4",
                linewidth=2.0,
                label="setpoint" if segment.index == 1 and axis is transition_axis else None,
            )
            axis.plot(
                valid_frame["t_rel_s"],
                valid_frame["d_filt_cm"],
                color="#d62728",
                linewidth=0.9,
                alpha=0.25,
                label="raw valid" if segment.index == 1 and axis is transition_axis else None,
            )
            axis.plot(
                display_frame["t_rel_s"],
                display_frame["display_position_cm"],
                color="#d62728",
                linewidth=2.1,
                label="display-smoothed" if segment.index == 1 and axis is transition_axis else None,
            )

        transition_values = frame.loc[
            frame["t_rel_s"] <= transition_end,
            ["display_position_cm", "d_filt_cm"],
        ].to_numpy().ravel()
        transition_y_values = [
            float(value)
            for value in transition_values
            if math.isfinite(float(value))
        ]
        transition_y_values.append(segment.setpoint_cm)
        if segment.previous_setpoint_cm is not None:
            transition_y_values.append(segment.previous_setpoint_cm)
        transition_axis.set_xlim(0.0, transition_end)
        transition_axis.set_ylim(*padded_limits(transition_y_values, min_margin=0.25, frac=0.10))
        transition_axis.set_ylabel("position [cm]")
        transition_axis.set_title(f"{segment.name}: transition detail")
        transition_axis.grid(True, alpha=0.35)
        transition_axis.text(
            0.02,
            0.05,
            f"{segment.transition_label}\nfirst {transition_end:.1f} s after the setpoint change",
            transform=transition_axis.transAxes,
            ha="left",
            va="bottom",
            fontsize=8.3,
            bbox={"boxstyle": "round,pad=0.20", "facecolor": "white", "alpha": 0.82, "edgecolor": "0.85"},
        )

        hold_y_values = hold_valid["d_filt_cm"].tolist() + hold_display["display_position_cm"].tolist()
        hold_y_values.append(segment.setpoint_cm)
        hold_axis.set_xlim(hold_start, float(frame["t_rel_s"].iloc[-1]))
        hold_axis.set_ylim(*padded_limits(hold_y_values, min_margin=0.18, frac=0.16))
        hold_axis.set_title("Late hold detail")
        hold_axis.grid(True, alpha=0.35)
        hold_axis.text(
            0.03,
            0.05,
            f"last {float(frame['t_rel_s'].iloc[-1]) - hold_start:.1f} s\nvalid {int(frame['valid_bool'].sum())}/{len(frame)}",
            transform=hold_axis.transAxes,
            ha="left",
            va="bottom",
            fontsize=8.3,
            bbox={"boxstyle": "round,pad=0.20", "facecolor": "white", "alpha": 0.82, "edgecolor": "0.85"},
        )

    axes[-1, 0].set_xlabel("local time from setpoint change [s]")
    axes[-1, 1].set_xlabel("local time from setpoint change [s]")
    axes[0, 0].legend(loc="best")
    fig.suptitle("Reference tracking with transition and late-hold detail", y=0.995, fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))
    fig.savefig(FIGURES_DIR / "telemetry_position_tracking.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_tracking_error(segment_views: list[SegmentView], step_metrics: list[StepMetrics]) -> None:
    step_segments = segment_views[1:]
    fig, axes = plt.subplots(len(step_segments), 1, figsize=(10.0, 7.4), sharex=False, sharey=False)

    for axis, segment, metric in zip(axes, step_segments, step_metrics, strict=False):
        frame = segment.df
        hold_start = hold_detail_start(frame)
        detail_frame = frame.loc[frame["t_rel_s"] >= hold_start].copy()
        detail_frame["t_detail_s"] = detail_frame["t_rel_s"] - hold_start
        valid_detail = detail_frame.loc[detail_frame["valid_bool"]]
        display_detail = detail_frame.loc[detail_frame["display_error_cm"].notna()]

        axis.plot(
            valid_detail["t_detail_s"],
            valid_detail["x_cm"],
            color="#2ca02c",
            linewidth=0.9,
            alpha=0.28,
            label="raw valid error" if segment.index == 1 else None,
        )
        axis.plot(
            display_detail["t_detail_s"],
            display_detail["display_error_cm"],
            color="#2ca02c",
            linewidth=2.0,
            label="display-smoothed error" if segment.index == 1 else None,
        )
        axis.axhline(0.0, color="0.25", linewidth=1.0)
        axis.axhline(metric.settling_band_cm, color="#1f77b4", linestyle="--", linewidth=1.0, alpha=0.8)
        axis.axhline(-metric.settling_band_cm, color="#1f77b4", linestyle="--", linewidth=1.0, alpha=0.8)

        y_values = valid_detail["x_cm"].tolist() + display_detail["display_error_cm"].tolist()
        y_values.extend([metric.settling_band_cm, -metric.settling_band_cm, 0.0])
        axis.set_xlim(0.0, float(detail_frame["t_detail_s"].iloc[-1]))
        axis.set_ylim(*padded_limits(y_values, min_margin=0.12, frac=0.18))
        axis.set_ylabel("error [cm]")
        axis.set_title(f"{segment.name}: late-hold tracking error")
        axis.text(
            0.02,
            0.05,
            f"last {float(detail_frame['t_detail_s'].iloc[-1]):.1f} s, band ±{metric.settling_band_cm:.2f} cm",
            transform=axis.transAxes,
            ha="left",
            va="bottom",
            fontsize=8.4,
            bbox={"boxstyle": "round,pad=0.20", "facecolor": "white", "alpha": 0.82, "edgecolor": "0.85"},
        )
        axis.grid(True, alpha=0.35)

    axes[-1].set_xlabel("late-hold local time [s]")
    axes[0].legend(loc="best")
    fig.suptitle("Late-hold tracking error by commanded transition", y=0.995, fontsize=14)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))
    fig.savefig(FIGURES_DIR / "telemetry_tracking_error.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_beam_angle(closed_loop: pd.DataFrame) -> None:
    change_times = closed_loop.loc[
        closed_loop["setpoint_cm"].diff().abs().fillna(0.0) > 1e-9, "t_s"
    ].tolist()

    fig, ax = plt.subplots(figsize=(10, 4.2))
    ax.plot(closed_loop["t_s"], closed_loop["theta_rel_deg"], color="#2ca02c", linewidth=1.7)
    add_step_change_lines(ax, change_times)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("beam angle [deg]")
    ax.set_title("Measured beam angle")
    fig.tight_layout()
    fig.savefig(FIGURES_DIR / "telemetry_beam_angle.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_control_input(closed_loop: pd.DataFrame) -> None:
    change_times = closed_loop.loc[
        closed_loop["setpoint_cm"].diff().abs().fillna(0.0) > 1e-9, "t_s"
    ].tolist()

    fig, ax = plt.subplots(figsize=(10, 4.2))
    ax.plot(closed_loop["t_s"], closed_loop["theta_cmd_rel_deg"], color="#ff7f0e", linewidth=1.7, label="theta_cmd_rel_deg")
    if "theta_fs_cmd_deg" in closed_loop.columns:
        ax.plot(
            closed_loop["t_s"],
            closed_loop["theta_fs_cmd_deg"],
            color="#1f77b4",
            linewidth=1.2,
            alpha=0.75,
            label="theta_fs_cmd_deg",
        )
    add_step_change_lines(ax, change_times)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("command [deg]")
    ax.set_title("Outer-loop beam-angle command")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(FIGURES_DIR / "telemetry_control_input.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_velocity_and_rate(closed_loop: pd.DataFrame) -> None:
    change_times = closed_loop.loc[
        closed_loop["setpoint_cm"].diff().abs().fillna(0.0) > 1e-9, "t_s"
    ].tolist()

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True)
    axes[0].plot(closed_loop["t_s"], closed_loop["x_dot_cm_s"], color="#9467bd", linewidth=1.6)
    axes[0].set_ylabel("ball vel. [cm/s]")
    axes[0].set_title("Velocity and rate estimates")
    axes[1].plot(closed_loop["t_s"], closed_loop["theta_dot_deg_s"], color="#8c564b", linewidth=1.6)
    axes[1].set_ylabel("beam rate [deg/s]")
    axes[1].set_xlabel("time [s]")
    for axis in axes:
        add_step_change_lines(axis, change_times)
    fig.tight_layout()
    fig.savefig(FIGURES_DIR / "telemetry_velocity_rate.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_calibration_curve(calibration: pd.DataFrame) -> None:
    angle_grid = calibration["as5600_deg_unwrapped"].to_numpy()
    runtime_fit = RUNTIME_THETA_SLOPE * angle_grid + RUNTIME_THETA_OFFSET
    averaged_curve = 0.5 * (
        calibration["forward_theta_deg"].to_numpy() + calibration["reverse_theta_deg"].to_numpy()
    )

    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    ax.plot(angle_grid, calibration["forward_theta_deg"], color="#1f77b4", linewidth=1.5, label="forward")
    ax.plot(angle_grid, calibration["reverse_theta_deg"], color="#d62728", linewidth=1.5, label="reverse")
    ax.plot(angle_grid, averaged_curve, color="#2ca02c", linewidth=1.6, linestyle="--", label="forward/reverse mean")
    ax.plot(angle_grid, runtime_fit, color="#ff7f0e", linewidth=1.4, linestyle=":", label="runtime affine fit")
    ax.set_xlabel("AS5600 unwrapped angle [deg]")
    ax.set_ylabel("beam angle [deg]")
    ax.set_title("Beam-angle calibration curves")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(FIGURES_DIR / "calibration_curve.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_calibration_hysteresis(calibration: pd.DataFrame) -> None:
    fig, ax = plt.subplots(figsize=(9.5, 4.4))
    ax.plot(
        calibration["as5600_deg_unwrapped"],
        calibration["reverse_minus_forward_deg"],
        color="#9467bd",
        linewidth=1.6,
    )
    ax.axhline(0.0, color="0.3", linewidth=1.0)
    ax.set_xlabel("AS5600 unwrapped angle [deg]")
    ax.set_ylabel("reverse - forward [deg]")
    ax.set_title("Calibration hysteresis")
    fig.tight_layout()
    fig.savefig(FIGURES_DIR / "calibration_hysteresis.pdf", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    configure_plot_style()

    telemetry, closed_loop = prepare_telemetry()
    calibration = prepare_calibration()
    segment_views = build_segment_views(closed_loop)
    step_metrics = compute_step_metrics(segment_views)

    plot_telemetry_overview(segment_views)
    plot_position_tracking(segment_views)
    plot_tracking_error(segment_views, step_metrics)
    plot_beam_angle(closed_loop)
    plot_control_input(closed_loop)
    plot_velocity_and_rate(closed_loop)
    plot_calibration_curve(calibration)
    plot_calibration_hysteresis(calibration)

    segment_valid_counts: dict[int, tuple[int, int]] = {}
    for segment_id, segment in closed_loop.groupby("segment_id", sort=True):
        segment_valid_counts[int(segment_id)] = (
            int(segment["valid_bool"].sum()),
            int(segment.shape[0]),
        )

    setpoint_levels = sorted({float(level) for level in closed_loop["setpoint_cm"].dropna().tolist()})
    telemetry_stats = {
        "telemetry_rows": int(telemetry.shape[0]),
        "closed_loop_rows": int(closed_loop.shape[0]),
        "duration_s": float(telemetry["t_s"].iloc[-1]),
        "invalid_rows": int((~telemetry["valid_bool"]).sum()),
        "recovery_rows": int(telemetry["recovery_active"].fillna(0).astype(int).sum()),
        "saturated_rows": int((telemetry["theta_cmd_sat"].fillna(0).abs() > 0.0).sum()),
        "max_abs_error_cm": float(closed_loop.loc[closed_loop["valid_bool"], "x_cm"].abs().max()),
        "max_abs_beam_angle_deg": float(closed_loop["theta_rel_deg"].abs().max()),
        "max_abs_command_deg": float(closed_loop["theta_cmd_rel_deg"].abs().max()),
        "setpoint_levels": ", ".join(f"{level:.2f}" for level in setpoint_levels),
        "far_valid_count": segment_valid_counts.get(1, (0, 0))[0],
        "far_total_count": segment_valid_counts.get(1, (0, 0))[1],
        "center_valid_count": segment_valid_counts.get(2, (0, 0))[0],
        "center_total_count": segment_valid_counts.get(2, (0, 0))[1],
        "near_valid_count": segment_valid_counts.get(3, (0, 0))[0],
        "near_total_count": segment_valid_counts.get(3, (0, 0))[1],
    }

    forward_fit = np.polyfit(
        calibration["as5600_deg_unwrapped"].to_numpy(),
        calibration["forward_theta_deg"].to_numpy(),
        1,
    )
    reverse_fit = np.polyfit(
        calibration["as5600_deg_unwrapped"].to_numpy(),
        calibration["reverse_theta_deg"].to_numpy(),
        1,
    )
    hysteresis = calibration["reverse_minus_forward_deg"].to_numpy()
    calibration_stats = {
        "point_count": int(calibration.shape[0]),
        "as_min": float(calibration["as5600_deg_unwrapped"].min()),
        "as_max": float(calibration["as5600_deg_unwrapped"].max()),
        "forward_min": float(calibration["forward_theta_deg"].min()),
        "forward_max": float(calibration["forward_theta_deg"].max()),
        "reverse_min": float(calibration["reverse_theta_deg"].min()),
        "reverse_max": float(calibration["reverse_theta_deg"].max()),
        "mean_hysteresis": float(np.mean(hysteresis)),
        "rms_hysteresis": float(np.sqrt(np.mean(np.square(hysteresis)))),
        "max_hysteresis": float(np.max(np.abs(hysteresis))),
        "forward_slope": float(forward_fit[0]),
        "forward_intercept": float(forward_fit[1]),
        "reverse_slope": float(reverse_fit[0]),
        "reverse_intercept": float(reverse_fit[1]),
    }

    write_text(TABLES_DIR / "signal_mapping.tex", make_signal_mapping_table())
    write_text(TABLES_DIR / "telemetry_run_summary.tex", make_run_summary_table(telemetry_stats))
    write_text(TABLES_DIR / "calibration_summary.tex", make_calibration_summary_table(calibration_stats))
    write_text(TABLES_DIR / "telemetry_step_metrics.tex", make_step_metrics_table(step_metrics))

    write_generated_macros(
        {
            **telemetry_stats,
            "calibration_point_count": calibration_stats["point_count"],
            "calibration_mean_hysteresis": calibration_stats["mean_hysteresis"],
            "calibration_max_hysteresis": calibration_stats["max_hysteresis"],
            "calibration_rms_hysteresis": calibration_stats["rms_hysteresis"],
        }
    )

    print(f"Wrote figures to {FIGURES_DIR}")
    print(f"Wrote tables to {TABLES_DIR}")


if __name__ == "__main__":
    main()
