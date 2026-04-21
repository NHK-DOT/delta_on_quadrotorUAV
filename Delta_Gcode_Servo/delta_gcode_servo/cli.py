from __future__ import annotations

import argparse
from pathlib import Path

from .servo import export_servo_commands_json, run_gcode_file


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Delta robot G-code to LX-225 servo pipeline")
    subparsers = parser.add_subparsers(dest="command", required=True)

    export_parser = subparsers.add_parser("export-servo-commands", help="Export G-code as servo JSON")
    export_parser.add_argument("input", help="Input G-code file")
    export_parser.add_argument("-o", "--output", help="Output JSON file")
    export_parser.add_argument("--time-ms", type=int, default=100, help="Fixed move time per segment")
    export_parser.set_defaults(func=cmd_export_servo_commands)

    run_parser = subparsers.add_parser("run-gcode", help="Execute G-code over serial bus")
    run_parser.add_argument("input", help="Input G-code file")
    run_parser.add_argument("--port", required=True, help="Serial port name, for example COM9")
    run_parser.add_argument("--baudrate", type=int, default=9600, help="Serial baudrate")
    run_parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout in seconds")
    run_parser.add_argument(
        "--connect-delay",
        type=float,
        default=0.2,
        help="Delay after opening the serial port before sending commands",
    )
    run_parser.add_argument("--time-ms", type=int, default=100, help="Fixed move time per segment")
    run_parser.add_argument("--settle-time", type=float, default=0.0, help="Delay after each command in seconds")
    run_parser.set_defaults(func=cmd_run_gcode)

    return parser


def cmd_export_servo_commands(args: argparse.Namespace) -> None:
    input_path = Path(args.input)
    output_path = Path(args.output) if args.output else input_path.with_suffix(".servo.json")
    export_servo_commands_json(input_path, output_path, time_ms=args.time_ms)
    print(f"Exported servo JSON: {output_path}")


def cmd_run_gcode(args: argparse.Namespace) -> None:
    commands = run_gcode_file(
        args.input,
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        connect_delay=args.connect_delay,
        time_ms=args.time_ms,
        settle_time=args.settle_time,
    )
    print(f"Executed {len(commands)} servo commands on {args.port}")


def main(argv: list[str] | None = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    args.func(args)
