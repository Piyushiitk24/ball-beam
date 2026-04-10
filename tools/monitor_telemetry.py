#!/usr/bin/env python3
"""Interactive serial monitor that saves timestamped telemetry CSV sessions."""

from __future__ import annotations

import argparse
import configparser
import csv
import json
import sys
import threading
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


TELEMETRY_HEADER_PREFIX = "t_ms,"


@dataclass
class SessionMetadata:
    started_at: str
    port: str
    baud: int
    output_dir: str
    csv_path: str
    log_path: str


class TelemetrySession:
    def __init__(self, output_dir: Path, port: str, baud: int) -> None:
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.output_dir / "telemetry.csv"
        self.log_path = self.output_dir / "session.log"
        self.meta_path = self.output_dir / "session_meta.json"

        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.log_file = self.log_path.open("w", encoding="utf-8")
        self.csv_writer: Optional[csv.writer] = None
        self.header_row: Optional[list[str]] = None
        self.row_count = 0

        metadata = SessionMetadata(
            started_at=datetime.now().isoformat(timespec="seconds"),
            port=port,
            baud=baud,
            output_dir=str(self.output_dir),
            csv_path=str(self.csv_path),
            log_path=str(self.log_path),
        )
        self.meta_path.write_text(json.dumps(asdict(metadata), indent=2), encoding="utf-8")

    def close(self) -> None:
        self.csv_file.close()
        self.log_file.close()

    def log_host_line(self, line: str) -> None:
        stamped = f"# [host {datetime.now().strftime('%H:%M:%S')}] {line}"
        self.log_file.write(stamped + "\n")
        self.log_file.flush()

    def handle_serial_line(self, line: str) -> None:
        print(line, flush=True)
        self.log_file.write(line + "\n")
        self.log_file.flush()

        if line.startswith(TELEMETRY_HEADER_PREFIX):
            row = next(csv.reader([line]))
            if self.header_row is None:
                self.header_row = row
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(row)
                self.csv_file.flush()
            return

        if not line or line.startswith("#") or self.header_row is None or self.csv_writer is None:
            return

        try:
            row = next(csv.reader([line]))
        except csv.Error:
            return

        if len(row) != len(self.header_row):
            return
        if not row[0].isdigit():
            return

        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.row_count += 1


def load_platformio_defaults(project_root: Path) -> dict[str, object]:
    config_path = project_root / "platformio.ini"
    if not config_path.exists():
        return {}

    config = configparser.ConfigParser(interpolation=None)
    config.read(config_path)

    for section_name in ("env:nanoatmega328new", "env:nanoatmega328new_calibration"):
        if not config.has_section(section_name):
            continue
        section = config[section_name]
        result: dict[str, object] = {}
        if "upload_port" in section:
            result["port"] = section.get("upload_port")
        if "monitor_speed" in section:
            result["baud"] = section.getint("monitor_speed")
        return result

    return {}


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    defaults = load_platformio_defaults(project_root)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=defaults.get("port"),
        help="Serial port. Defaults to upload_port from platformio.ini.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=defaults.get("baud", 115200),
        help="Serial baud rate. Defaults to monitor_speed from platformio.ini.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=project_root / "telemetry_logs" / timestamp,
        help="Directory where telemetry.csv and session.log will be written.",
    )
    return parser.parse_args()


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required for the telemetry monitor.\n"
            "Use the project virtualenv or install it with: python -m pip install pyserial"
        ) from exc

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.25
    ser.write_timeout = 1.0
    ser.rts = False
    ser.dtr = False
    ser.open()
    return ser


def stdin_pump(ser, session: TelemetrySession, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
        except KeyboardInterrupt:
            stop_event.set()
            return

        if line == "":
            stop_event.set()
            return

        command = line.rstrip("\r\n")
        if not command:
            continue

        session.log_host_line(command)
        try:
            ser.write((command + "\n").encode("utf-8"))
            ser.flush()
        except Exception as exc:  # pragma: no cover - hardware dependent
            session.log_host_line(f"serial write failed: {exc}")
            stop_event.set()
            return


def main() -> int:
    args = parse_args()
    if not args.port:
        raise SystemExit("No serial port specified. Pass --port or set upload_port in platformio.ini.")

    session = TelemetrySession(args.output_dir, args.port, args.baud)
    ser = open_serial(args.port, args.baud)
    stop_event = threading.Event()
    input_thread = threading.Thread(
        target=stdin_pump,
        args=(ser, session, stop_event),
        daemon=True,
    )
    input_thread.start()

    print(f"# Telemetry CSV: {session.csv_path}", flush=True)
    print(f"# Session log   : {session.log_path}", flush=True)
    print("# Type firmware commands and press Enter. Ctrl+C exits.", flush=True)

    try:
        while not stop_event.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            session.handle_serial_line(line)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        session.close()

    print(f"# Saved {session.row_count} telemetry rows to {session.csv_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
