#!/usr/bin/env python3
"""Interactive host tool for AS5600-to-beam-angle calibration."""

from __future__ import annotations

import argparse
import configparser
import csv
import json
import math
import sys
import time
from bisect import bisect_right
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


FIRMWARE_COMMANDS = {"E", "D", "L", "U", "N", "B", "C", "S"}
LOCAL_COMMANDS = {"H", "Q"}


@dataclass
class CaptureRecord:
    tag: str
    t_ms: int
    rel_steps: int
    as5600_raw_mean: float
    as5600_raw_std: float
    as5600_deg_wrapped: float
    as5600_deg_unwrapped: float
    driver_enabled: int
    forward_sign: int
    lower_valid: int
    upper_valid: int


@dataclass
class LoggedRow:
    phase: str
    command: str
    sweep_dir: str
    sweep_index: int
    t_ms: int
    rel_steps: int
    as5600_raw_mean: float
    as5600_raw_std: float
    as5600_deg_wrapped: float
    as5600_deg_unwrapped: float
    iphone_theta_deg: float
    driver_enabled: int
    forward_sign: int
    lower_valid: int
    upper_valid: int


class CalibrationSession:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / "calibration_data.csv"
        self.report_path = self.output_dir / "fit_report.txt"
        self.json_path = self.output_dir / "fit_report.json"

        self.lower: Optional[LoggedRow] = None
        self.upper: Optional[LoggedRow] = None
        self.sweep_rows: list[LoggedRow] = []
        self.forward_count = 0
        self.reverse_count = 0

        self.driver_enabled = False
        self.driver_enabled_at: Optional[float] = None

    def set_driver_enabled(self, enabled: bool) -> None:
        self.driver_enabled = enabled
        self.driver_enabled_at = time.monotonic() if enabled else None

    def hold_elapsed_s(self) -> float:
        if self.driver_enabled_at is None:
            return 0.0
        return time.monotonic() - self.driver_enabled_at

    def clear_sweep(self, reason: str) -> None:
        self.sweep_rows.clear()
        self.forward_count = 0
        self.reverse_count = 0
        self.rewrite_outputs()
        print(f"[host] cleared sweep data: {reason}")

    def record_capture(self, command: str, capture: CaptureRecord, iphone_theta_deg: float) -> LoggedRow:
        if command == "L":
            self.clear_sweep("lower endpoint recaptured")
            row = self._make_logged_row("endpoint", "endpoint", 0, command, capture, iphone_theta_deg)
            self.lower = row
        elif command == "U":
            self.clear_sweep("upper endpoint recaptured")
            row = self._make_logged_row("endpoint", "endpoint", 0, command, capture, iphone_theta_deg)
            self.upper = row
        elif command == "N":
            row = self._make_logged_row("sweep", "forward", self.forward_count, command, capture, iphone_theta_deg)
            self.forward_count += 1
            self.sweep_rows.append(row)
        elif command == "B":
            row = self._make_logged_row("sweep", "reverse", self.reverse_count, command, capture, iphone_theta_deg)
            self.reverse_count += 1
            self.sweep_rows.append(row)
        else:
            raise ValueError(f"unexpected capture command: {command}")

        self.rewrite_outputs()
        return row

    def all_rows(self) -> list[LoggedRow]:
        rows: list[LoggedRow] = []
        if self.lower is not None:
            rows.append(self.lower)
        if self.upper is not None:
            rows.append(self.upper)
        rows.extend(self.sweep_rows)
        return rows

    def rewrite_outputs(self) -> None:
        rows = self.all_rows()
        fieldnames = list(asdict(rows[0]).keys()) if rows else list(LoggedRow.__dataclass_fields__.keys())

        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(asdict(row))

        report = build_report(rows)
        self.report_path.write_text(report["text"], encoding="utf-8")
        self.json_path.write_text(json.dumps(report["json"], indent=2), encoding="utf-8")

    def _make_logged_row(
        self,
        phase: str,
        sweep_dir: str,
        sweep_index: int,
        command: str,
        capture: CaptureRecord,
        iphone_theta_deg: float,
    ) -> LoggedRow:
        return LoggedRow(
            phase=phase,
            command=command,
            sweep_dir=sweep_dir,
            sweep_index=sweep_index,
            t_ms=capture.t_ms,
            rel_steps=capture.rel_steps,
            as5600_raw_mean=capture.as5600_raw_mean,
            as5600_raw_std=capture.as5600_raw_std,
            as5600_deg_wrapped=capture.as5600_deg_wrapped,
            as5600_deg_unwrapped=capture.as5600_deg_unwrapped,
            iphone_theta_deg=iphone_theta_deg,
            driver_enabled=capture.driver_enabled,
            forward_sign=capture.forward_sign,
            lower_valid=capture.lower_valid,
            upper_valid=capture.upper_valid,
        )


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    defaults = load_platformio_defaults(project_root)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=defaults.get("port"), help="Serial port for the calibration firmware")
    parser.add_argument("--baud", type=int, default=defaults.get("baud", 115200), help="Serial baud rate")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=project_root / "calibration_logs" / timestamp,
        help="Directory for CSV and fit reports",
    )
    parser.add_argument("--timeout", type=float, default=15.0, help="Command timeout in seconds")
    return parser.parse_args()


def load_platformio_defaults(project_root: Path) -> dict[str, object]:
    config_path = project_root / "platformio.ini"
    if not config_path.exists():
        return {}

    config = configparser.ConfigParser(interpolation=None)
    config.read(config_path)

    for section_name in ("env:nanoatmega328new_calibration", "env:nanoatmega328new"):
        if config.has_section(section_name):
            section = config[section_name]
            result: dict[str, object] = {}
            if "upload_port" in section:
                result["port"] = section.get("upload_port")
            if "monitor_speed" in section:
                result["baud"] = section.getint("monitor_speed")
            return result

    return {}


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required for the host calibration tool.\n"
            "Install it with: python3 -m pip install pyserial"
        ) from exc

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.25
    ser.write_timeout = 1.0
    ser.rts = False
    ser.dtr = False
    ser.open()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def print_help() -> None:
    print("Commands:")
    print("  E  enable driver")
    print("  D  disable driver")
    print("  L  capture lower endpoint, then enter the iPhone angle")
    print("  U  capture upper endpoint, then enter the iPhone angle")
    print("  N  move one forward sweep increment and log a point")
    print("  B  move one reverse sweep increment and log a point")
    print("  C  clear sweep data only and return to the stored upper sweep start")
    print("  S  print firmware status")
    print("  H  show this help")
    print("  Q  quit the host tool")


def read_startup_lines(ser, timeout_s: float = 1.5) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"<< {line}")


def parse_capture_line(line: str) -> CaptureRecord:
    parts = line.split(",")
    if len(parts) != 12 or parts[0] != "REC":
        raise ValueError(f"unexpected REC line: {line}")

    return CaptureRecord(
        tag=parts[1],
        t_ms=int(parts[2]),
        rel_steps=int(parts[3]),
        as5600_raw_mean=float(parts[4]),
        as5600_raw_std=float(parts[5]),
        as5600_deg_wrapped=float(parts[6]),
        as5600_deg_unwrapped=float(parts[7]),
        driver_enabled=int(parts[8]),
        forward_sign=int(parts[9]),
        lower_valid=int(parts[10]),
        upper_valid=int(parts[11]),
    )


def wait_for_response(ser, expect_capture: bool, timeout_s: float):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(f"<< {line}")
        prefix = line.split(",", 1)[0]

        if prefix == "REC":
            return ("REC", parse_capture_line(line))
        if prefix == "ERR":
            return ("ERR", line)
        if not expect_capture and prefix in {"OK", "STAT"}:
            return (prefix, line)

    raise TimeoutError("timed out waiting for firmware response")


def prompt_phone_angle() -> float:
    while True:
        raw = input("iphone theta deg> ").strip()
        try:
            return float(raw)
        except ValueError:
            print("Enter a signed numeric angle like -1.2 or 0.0")


def send_command(ser, command: str) -> None:
    ser.write((command + "\n").encode("ascii"))
    ser.flush()


def sign_of(value: float) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


def maybe_print_endpoint_progress(session: CalibrationSession, row: LoggedRow) -> None:
    if row.command not in {"N", "B"}:
        return
    if session.lower is None or session.upper is None:
        return

    travel_sign = sign_of(session.lower.as5600_deg_unwrapped - session.upper.as5600_deg_unwrapped)
    if travel_sign == 0:
        return

    if row.command == "N":
        remaining = (session.lower.as5600_deg_unwrapped - row.as5600_deg_unwrapped) * travel_sign
        if remaining <= 0.0:
            print("[host] reached or passed the stored lower-end AS5600 region")
        else:
            print(f"[host] remaining to lower endpoint: {remaining:.4f} deg (AS5600)")
    else:
        remaining = (row.as5600_deg_unwrapped - session.upper.as5600_deg_unwrapped) * travel_sign
        if remaining <= 0.0:
            print("[host] reached or passed the stored upper-end AS5600 region")
        else:
            print(f"[host] remaining to upper endpoint: {remaining:.4f} deg (AS5600)")


def prompt_string(session: CalibrationSession) -> str:
    driver_text = "OFF"
    if session.driver_enabled:
        driver_text = f"ON {session.hold_elapsed_s():.1f}s"

    return (
        f"[driver {driver_text} | "
        f"L {'Y' if session.lower else 'N'} | "
        f"U {'Y' if session.upper else 'N'} | "
        f"F {session.forward_count} | "
        f"R {session.reverse_count}] cmd> "
    )


def build_report(rows: list[LoggedRow]) -> dict[str, object]:
    points = [(row.as5600_deg_unwrapped, row.iphone_theta_deg) for row in rows]
    affine = linear_fit(points)
    lut = monotone_lut_fit(points)
    hysteresis = hysteresis_metrics(rows)
    recommendation = choose_model(affine, lut, hysteresis)

    lines = [
        "AS5600-to-beam-angle calibration report",
        f"generated_at: {datetime.now().isoformat(timespec='seconds')}",
        f"rows_total: {len(rows)}",
        f"rows_forward: {sum(1 for row in rows if row.command == 'N')}",
        f"rows_reverse: {sum(1 for row in rows if row.command == 'B')}",
        f"rows_endpoints: {sum(1 for row in rows if row.command in {'L', 'U'})}",
        "",
        "affine_fit:",
    ]

    if affine is None:
        lines.append("  unavailable")
    else:
        lines.extend(
            [
                f"  slope_deg_per_deg: {affine['slope']:.8f}",
                f"  intercept_deg: {affine['intercept']:.8f}",
                f"  rms_error_deg: {affine['rms_error_deg']:.6f}",
                f"  max_abs_error_deg: {affine['max_abs_error_deg']:.6f}",
            ]
        )

    lines.extend(["", "monotone_lut_fit:"])
    if lut is None:
        lines.append("  unavailable")
    else:
        lines.extend(
            [
                f"  direction: {lut['direction']}",
                f"  rms_error_deg: {lut['rms_error_deg']:.6f}",
                f"  max_abs_error_deg: {lut['max_abs_error_deg']:.6f}",
                f"  lut_points: {len(lut['lut_points'])}",
            ]
        )

    lines.extend(["", "hysteresis:"])
    if hysteresis is None:
        lines.append("  unavailable")
    else:
        lines.extend(
            [
                f"  overlap_samples: {hysteresis['samples']}",
                f"  mean_abs_deg: {hysteresis['mean_abs_deg']:.6f}",
                f"  max_abs_deg: {hysteresis['max_abs_deg']:.6f}",
            ]
        )

    lines.extend(["", f"recommended_model: {recommendation}"])

    return {
        "text": "\n".join(lines) + "\n",
        "json": {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "rows_total": len(rows),
            "rows_forward": sum(1 for row in rows if row.command == "N"),
            "rows_reverse": sum(1 for row in rows if row.command == "B"),
            "rows_endpoints": sum(1 for row in rows if row.command in {"L", "U"}),
            "affine_fit": affine,
            "monotone_lut_fit": lut,
            "hysteresis": hysteresis,
            "recommended_model": recommendation,
        },
    }


def linear_fit(points: list[tuple[float, float]]) -> Optional[dict[str, float]]:
    if len(points) < 2:
        return None

    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)

    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom <= 0.0:
        return None

    slope = sum((x - mean_x) * (y - mean_y) for x, y in points) / denom
    intercept = mean_y - slope * mean_x

    residuals = [y - (slope * x + intercept) for x, y in points]
    rms_error = math.sqrt(sum(res * res for res in residuals) / len(residuals))
    max_abs_error = max(abs(res) for res in residuals)

    return {
        "slope": slope,
        "intercept": intercept,
        "rms_error_deg": rms_error,
        "max_abs_error_deg": max_abs_error,
    }


def monotone_lut_fit(points: list[tuple[float, float]]) -> Optional[dict[str, object]]:
    if len(points) < 2:
        return None

    sorted_points = sorted(points, key=lambda pair: pair[0])
    xs = [point[0] for point in sorted_points]
    ys = [point[1] for point in sorted_points]

    affine = linear_fit(sorted_points)
    if affine is not None and abs(affine["slope"]) > 1e-9:
        direction = "increasing" if affine["slope"] > 0.0 else "decreasing"
    else:
        direction = "increasing" if ys[-1] >= ys[0] else "decreasing"

    work_ys = ys[:] if direction == "increasing" else [-value for value in ys]
    fitted_work = isotonic_increasing(work_ys)
    fitted_ys = fitted_work[:] if direction == "increasing" else [-value for value in fitted_work]

    residuals = [orig - fit for orig, fit in zip(ys, fitted_ys)]
    rms_error = math.sqrt(sum(res * res for res in residuals) / len(residuals))
    max_abs_error = max(abs(res) for res in residuals)

    lut_points = [{"as5600_deg_unwrapped": x, "beam_theta_deg": y} for x, y in zip(xs, fitted_ys)]
    return {
        "direction": direction,
        "rms_error_deg": rms_error,
        "max_abs_error_deg": max_abs_error,
        "lut_points": lut_points,
    }


def isotonic_increasing(values: list[float]) -> list[float]:
    blocks: list[dict[str, object]] = []

    for index, value in enumerate(values):
        blocks.append({"start": index, "end": index, "weight": 1.0, "value": value})
        while len(blocks) >= 2 and float(blocks[-2]["value"]) > float(blocks[-1]["value"]):
            right = blocks.pop()
            left = blocks.pop()
            weight = float(left["weight"]) + float(right["weight"])
            avg = (
                float(left["value"]) * float(left["weight"])
                + float(right["value"]) * float(right["weight"])
            ) / weight
            blocks.append(
                {
                    "start": int(left["start"]),
                    "end": int(right["end"]),
                    "weight": weight,
                    "value": avg,
                }
            )

    fitted = [0.0] * len(values)
    for block in blocks:
        for index in range(int(block["start"]), int(block["end"]) + 1):
            fitted[index] = float(block["value"])
    return fitted


def hysteresis_metrics(rows: list[LoggedRow]) -> Optional[dict[str, float]]:
    forward = sorted(
        [(row.as5600_deg_unwrapped, row.iphone_theta_deg) for row in rows if row.command == "N"],
        key=lambda pair: pair[0],
    )
    reverse = sorted(
        [(row.as5600_deg_unwrapped, row.iphone_theta_deg) for row in rows if row.command == "B"],
        key=lambda pair: pair[0],
    )

    if len(forward) < 2 or len(reverse) < 2:
        return None

    rev_xs = [x for x, _ in reverse]
    rev_ys = [y for _, y in reverse]
    overlap_min = max(forward[0][0], reverse[0][0])
    overlap_max = min(forward[-1][0], reverse[-1][0])
    if overlap_max <= overlap_min:
        return None

    deltas: list[float] = []
    for x_fwd, y_fwd in forward:
        if x_fwd < overlap_min or x_fwd > overlap_max:
            continue
        y_rev = linear_interpolate(rev_xs, rev_ys, x_fwd)
        if y_rev is None:
            continue
        deltas.append(abs(y_fwd - y_rev))

    if not deltas:
        return None

    return {
        "samples": float(len(deltas)),
        "mean_abs_deg": sum(deltas) / len(deltas),
        "max_abs_deg": max(deltas),
    }


def linear_interpolate(xs: list[float], ys: list[float], x: float) -> Optional[float]:
    if len(xs) != len(ys) or len(xs) < 2:
        return None
    if x < xs[0] or x > xs[-1]:
        return None

    index = bisect_right(xs, x)
    if index == 0:
        return ys[0]
    if index >= len(xs):
        return ys[-1]

    x0 = xs[index - 1]
    x1 = xs[index]
    y0 = ys[index - 1]
    y1 = ys[index]
    if x1 == x0:
        return y0

    alpha = (x - x0) / (x1 - x0)
    return y0 + alpha * (y1 - y0)


def choose_model(
    affine: Optional[dict[str, float]],
    lut: Optional[dict[str, object]],
    hysteresis: Optional[dict[str, float]],
) -> str:
    if affine is None and lut is None:
        return "insufficient_data"
    if affine is None:
        return "monotone_lut"
    if lut is None:
        return "affine"

    hyst_ok = hysteresis is None or hysteresis["max_abs_deg"] <= 0.15
    if (
        affine["rms_error_deg"] <= 0.10
        and affine["max_abs_error_deg"] <= 0.20
        and hyst_ok
    ):
        return "affine"
    return "monotone_lut"


def main() -> int:
    args = parse_args()
    if not args.port:
        print("No serial port configured. Pass --port explicitly.", file=sys.stderr)
        return 2

    session = CalibrationSession(args.output_dir)
    print(f"[host] writing logs to {session.output_dir}")

    try:
        ser = open_serial(args.port, args.baud)
    except Exception as exc:  # pragma: no cover - user-facing serial error path
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 2

    try:
        read_startup_lines(ser)
        print_help()

        while True:
            raw = input(prompt_string(session)).strip().upper()
            if not raw:
                continue
            if len(raw) != 1 or not raw.isalpha():
                print("Enter a single-letter command.")
                continue

            command = raw
            if command in LOCAL_COMMANDS:
                if command == "H":
                    print_help()
                    continue
                if command == "Q":
                    break

            if command not in FIRMWARE_COMMANDS:
                print("Unknown command. Use H for help.")
                continue

            send_command(ser, command)

            expect_capture = command in {"L", "U", "N", "B"}
            try:
                kind, payload = wait_for_response(ser, expect_capture=expect_capture, timeout_s=args.timeout)
            except TimeoutError as exc:
                print(f"[host] {exc}")
                continue

            if kind == "ERR":
                continue

            if command == "E":
                session.set_driver_enabled(True)
            elif command == "D":
                session.set_driver_enabled(False)
            elif command == "C":
                session.clear_sweep("user command C")
                continue
            elif command == "S":
                continue

            if kind != "REC":
                continue

            capture = payload
            if capture.driver_enabled:
                session.set_driver_enabled(True)

            iphone_theta = prompt_phone_angle()
            row = session.record_capture(command, capture, iphone_theta)
            print(
                "[host] recorded "
                f"{command}: theta={row.iphone_theta_deg:.4f} deg, "
                f"as5600={row.as5600_deg_unwrapped:.5f} deg, "
                f"rel_steps={row.rel_steps}"
            )
            maybe_print_endpoint_progress(session, row)
            print(f"[host] csv: {session.csv_path}")
            print(f"[host] report: {session.report_path}")

    except KeyboardInterrupt:
        print("\n[host] interrupted")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
