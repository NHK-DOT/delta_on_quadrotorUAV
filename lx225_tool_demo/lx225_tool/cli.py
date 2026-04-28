from __future__ import annotations

import argparse
from pathlib import Path

from .config import AppConfig, ServoProfile, load_config
from .service import LX225Service, ServoSnapshot


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "lx225_tool.demo.toml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LX225 read/set tool without motion commands")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"Config file path, default: {DEFAULT_CONFIG_PATH}",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    discover = subparsers.add_parser("discover-id", help="Broadcast-read a single servo ID")
    discover.set_defaults(func=cmd_discover_id)

    gui = subparsers.add_parser("gui", help="Launch desktop GUI")
    gui.set_defaults(func=cmd_gui)

    read_id = subparsers.add_parser("read-id", help="Read ID from a known servo address")
    read_id.add_argument("--id", type=int, required=True, help="Current servo ID to query")
    read_id.set_defaults(func=cmd_read_id)

    read = subparsers.add_parser("read", help="Read position, offset, and limits")
    add_servo_selector(read)
    read.set_defaults(func=cmd_read)

    read_limit = subparsers.add_parser("read-limit", help="Read internal angle limits")
    add_servo_selector(read_limit)
    read_limit.set_defaults(func=cmd_read_limit)

    set_limit = subparsers.add_parser("set-limit", help="Write internal angle limits")
    add_servo_selector(set_limit)
    set_limit.add_argument("--raw-min", type=int, required=True, help="Internal minimum raw limit")
    set_limit.add_argument("--raw-max", type=int, required=True, help="Internal maximum raw limit")
    set_limit.set_defaults(func=cmd_set_limit)

    capture_limit = subparsers.add_parser("capture-limit", help="Capture current pose as one side of the limit")
    capture_limit.add_argument("--servo", required=True, help="Configured servo name")
    capture_limit.add_argument("--side", choices=("min", "max"), required=True, help="Which side to overwrite")
    capture_limit.set_defaults(func=cmd_capture_limit)

    read_offset = subparsers.add_parser("read-offset", help="Read current offset")
    add_servo_selector(read_offset)
    read_offset.set_defaults(func=cmd_read_offset)

    set_offset = subparsers.add_parser("set-offset", help="Adjust servo offset")
    add_servo_selector(set_offset)
    set_offset.add_argument("--offset", type=int, required=True, help="Signed offset value")
    set_offset.add_argument("--save", action="store_true", help="Persist the adjusted offset")
    set_offset.set_defaults(func=cmd_set_offset)

    write_id = subparsers.add_parser("write-id", help="Write servo ID")
    write_id.add_argument("--old-id", type=int, required=True, help="Current servo ID")
    write_id.add_argument("--new-id", type=int, required=True, help="Target servo ID")
    write_id.set_defaults(func=cmd_write_id)

    show_map = subparsers.add_parser("show-map", help="Show the custom raw-angle mapping")
    show_map.add_argument("--servo", required=True, help="Configured servo name")
    show_map.set_defaults(func=cmd_show_map)

    angle_to_raw = subparsers.add_parser("angle-to-raw", help="Convert custom angle to raw position")
    angle_to_raw.add_argument("--servo", required=True, help="Configured servo name")
    angle_to_raw.add_argument("--angle", type=float, required=True, help="Custom angle value")
    angle_to_raw.set_defaults(func=cmd_angle_to_raw)

    raw_to_angle = subparsers.add_parser("raw-to-angle", help="Convert raw position to custom angle")
    raw_to_angle.add_argument("--servo", required=True, help="Configured servo name")
    raw_to_angle.add_argument("--raw", type=int, required=True, help="Raw position value")
    raw_to_angle.set_defaults(func=cmd_raw_to_angle)

    return parser


def add_servo_selector(parser: argparse.ArgumentParser) -> None:
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--servo", help="Configured servo name")
    group.add_argument("--id", type=int, help="Direct servo ID")


def get_profile(cfg: AppConfig, servo_name: str) -> ServoProfile:
    return cfg.resolve_servo(servo_name)


def print_snapshot(snapshot: ServoSnapshot) -> None:
    if snapshot.servo_name:
        print(f"servo_name   : {snapshot.servo_name}")
    print(f"servo_id     : {snapshot.servo_id}")
    print(f"position_raw : {snapshot.position_raw}")
    if snapshot.mapped_angle is not None:
        print(f"mapped_angle : {snapshot.mapped_angle:.3f}")
    else:
        print("mapped_angle : <no configured mapping>")
    print(f"offset       : {snapshot.offset}")
    print(f"limit_min    : {snapshot.limit_min_raw}")
    print(f"limit_max    : {snapshot.limit_max_raw}")


def cmd_discover_id(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        servo_id = service.discover_single_id()
    print(f"discovered_id: {servo_id}")


def cmd_gui(args: argparse.Namespace, cfg: AppConfig) -> None:
    from .gui import launch_gui

    launch_gui(cfg.source_path)


def cmd_read_id(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        servo_id = service.read_id(args.id)
    print(f"queried_id: {args.id}")
    print(f"reported_id: {servo_id}")


def cmd_read(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        snapshot = service.read_snapshot(servo_name=args.servo, servo_id=args.id, best_effort=True)
    print_snapshot(snapshot)


def cmd_read_limit(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        low, high = service.read_angle_limit(servo_name=args.servo, servo_id=args.id)
    print(f"limit_min_raw: {low}")
    print(f"limit_max_raw: {high}")


def cmd_set_limit(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        service.write_angle_limit(
            servo_name=args.servo,
            servo_id=args.id,
            raw_min=args.raw_min,
            raw_max=args.raw_max,
        )
    target = args.servo if args.servo is not None else f"id={args.id}"
    print(f"updated_limit: {target}")
    print(f"limit_min_raw: {args.raw_min}")
    print(f"limit_max_raw: {args.raw_max}")


def cmd_capture_limit(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        current_raw, low, high = service.capture_limit(servo_name=args.servo, side=args.side)
    print(f"captured_side : {args.side}")
    print(f"captured_raw  : {current_raw}")
    print(f"new_limit_min : {low}")
    print(f"new_limit_max : {high}")


def cmd_read_offset(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        offset = service.read_offset(servo_name=args.servo, servo_id=args.id)
    print(f"offset: {offset}")


def cmd_set_offset(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        service.set_offset(servo_name=args.servo, servo_id=args.id, offset=args.offset, save=args.save)
    target = args.servo if args.servo is not None else f"id={args.id}"
    print(f"updated_offset: {target}")
    print(f"offset        : {args.offset}")
    print(f"saved         : {args.save}")


def cmd_write_id(args: argparse.Namespace, cfg: AppConfig) -> None:
    with LX225Service(cfg) as service:
        service.write_id(args.old_id, args.new_id)
    print(f"old_id: {args.old_id}")
    print(f"new_id: {args.new_id}")


def cmd_show_map(args: argparse.Namespace, cfg: AppConfig) -> None:
    profile = get_profile(cfg, args.servo)
    mapping = profile.mapping
    print(f"servo_name                 : {profile.name}")
    print(f"servo_id                   : {profile.id}")
    print(f"raw_min                    : {mapping.raw_min}")
    print(f"raw_max                    : {mapping.raw_max}")
    print(f"mapped_angle_at_raw_min    : {mapping.mapped_angle_at_raw_min:.6f}")
    print(f"mapped_angle_at_raw_max    : {mapping.mapped_angle_at_raw_max:.6f}")
    print(f"raw_span                   : {mapping.raw_span}")
    print(f"angle_span                 : {mapping.angle_span:.6f}")
    print(f"angle_per_raw_step         : {mapping.angle_per_raw:.6f}")
    print(f"position_step              : {mapping.position_step}")
    print(f"angle_per_quantized_step   : {mapping.angle_per_raw * mapping.position_step:.6f}")


def cmd_angle_to_raw(args: argparse.Namespace, cfg: AppConfig) -> None:
    profile = get_profile(cfg, args.servo)
    raw_value = profile.mapping.angle_to_raw(args.angle, quantize=True)
    print(f"servo_name: {profile.name}")
    print(f"input_angle: {args.angle}")
    print(f"raw_value: {raw_value}")


def cmd_raw_to_angle(args: argparse.Namespace, cfg: AppConfig) -> None:
    profile = get_profile(cfg, args.servo)
    angle = profile.mapping.raw_to_angle(args.raw)
    print(f"servo_name: {profile.name}")
    print(f"input_raw: {args.raw}")
    print(f"mapped_angle: {angle:.6f}")


def main(argv: list[str] | None = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    args.func(args, cfg)
