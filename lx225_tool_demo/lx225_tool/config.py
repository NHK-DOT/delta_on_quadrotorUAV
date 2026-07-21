from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil

from .mapping import ServoMapping

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    tomllib = None


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baudrate: int = 115200
    timeout: float = 0.2
    connect_delay: float = 0.1


@dataclass(frozen=True)
class LandingGearConfig:
    servo_ids: tuple[int, ...] = (4, 5, 6)
    down_raw: int = 500
    up_raw: int = 1000
    move_time_ms: int = 180


@dataclass(frozen=True)
class ServoProfile:
    name: str
    id: int
    mapping: ServoMapping
    home_raw: int | None = None
    startup_check_raw: int | None = None


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig
    defaults_position_step: int
    landing_gear: LandingGearConfig
    servos: dict[str, ServoProfile]
    source_path: Path

    def resolve_servo(self, name: str) -> ServoProfile:
        try:
            return self.servos[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.servos))
            raise KeyError(f"Unknown servo '{name}'. Known servos: {known}") from exc


def _require_tomllib() -> None:
    if tomllib is None:
        raise RuntimeError("Python 3.11+ is required because this tool reads TOML via tomllib")


def load_config(path: str | Path) -> AppConfig:
    _require_tomllib()
    source_path = Path(path).resolve()
    data = tomllib.loads(source_path.read_text(encoding="utf-8"))

    serial_data = data.get("serial", {})
    defaults_data = data.get("defaults", {})

    serial_cfg = SerialConfig(
        port=str(serial_data.get("port", "COM3")),
        baudrate=int(serial_data.get("baudrate", 115200)),
        timeout=float(serial_data.get("timeout", 0.2)),
        connect_delay=float(serial_data.get("connect_delay", 0.1)),
    )

    default_step = int(defaults_data.get("position_step", 1))
    landing_gear_data = data.get("landing_gear", {})
    if not isinstance(landing_gear_data, dict):
        landing_gear_data = {}
    landing_gear_ids_raw = landing_gear_data.get("servo_ids", [4, 5, 6])
    if not isinstance(landing_gear_ids_raw, list):
        landing_gear_ids_raw = [4, 5, 6]
    landing_gear_cfg = LandingGearConfig(
        servo_ids=tuple(int(value) for value in landing_gear_ids_raw),
        down_raw=int(landing_gear_data.get("down_raw", 500)),
        up_raw=int(landing_gear_data.get("up_raw", 1000)),
        move_time_ms=int(landing_gear_data.get("move_time_ms", 180)),
    )
    servo_section = data.get("servos", {})
    if not servo_section:
        raise ValueError("No [servos.*] entries found in config")

    servos: dict[str, ServoProfile] = {}
    for name, item in servo_section.items():
        mapping = ServoMapping(
            raw_min=int(item.get("raw_min", 0)),
            raw_max=int(item.get("raw_max", 1000)),
            mapped_angle_at_raw_min=float(item.get("mapped_angle_at_raw_min", 0.0)),
            mapped_angle_at_raw_max=float(item.get("mapped_angle_at_raw_max", 240.0)),
            position_step=int(item.get("position_step", default_step)),
        )
        servos[name] = ServoProfile(
            name=name,
            id=int(item["id"]),
            mapping=mapping,
            home_raw=int(item["home_raw"]) if "home_raw" in item else None,
            startup_check_raw=int(item["startup_check_raw"]) if "startup_check_raw" in item else None,
        )

    return AppConfig(
        serial=serial_cfg,
        defaults_position_step=default_step,
        landing_gear=landing_gear_cfg,
        servos=servos,
        source_path=source_path,
    )


def format_config_text(config: AppConfig) -> str:
    lines = [
        "# LX225 工具示例配置",
        "# 目标：",
        "# 1. 把常用参数尽量放到配置文件里，不要每次在命令行写一长串参数。",
        "# 2. 把“硬件读写目标”和“本地映射显示规则”分开。",
        "# 3. 本地映射只影响显示和换算，不会让舵机运动。",
        "#",
        "# 约定：",
        "# 1. `raw_*` 是舵机协议里的原始位置值，通常可理解为官方软件看到的 0..1000。",
        "# 2. `mapped_angle_*` 是你自己定义的角度坐标系，不必等于官方 0..240°。",
        "# 3. `position_step` 是你希望量化到的原始步长。",
        "#    - 1  表示最细",
        "#    - 5  表示每 5 个 raw 量化一步",
        "#    - 10 表示接近官方 UI 的常见步进",
        "",
        "[serial]",
        "# 实际连接舵机驱动板的串口。",
        "# 注意不要填成 IMU 的串口。",
        f'port = "{config.serial.port}"',
        "",
        "# 波特率。",
        f"baudrate = {config.serial.baudrate}",
        "",
        "# 串口读超时，单位秒。",
        f"timeout = {config.serial.timeout:.2f}",
        "",
        "# 打开串口后的额外等待时间，单位秒。",
        f"connect_delay = {config.serial.connect_delay:.2f}",
        "",
        "[defaults]",
        "# 全局默认量化步长。",
        f"position_step = {config.defaults_position_step}",
        "",
        "# Current real-machine role split:",
        "# - servos 1/2/3 are the arm actuators controlled by IK/FK.",
        "# - servos 4/5/6 are a simple two-position retractable landing gear.",
        "[landing_gear]",
        f"servo_ids = [{', '.join(str(value) for value in config.landing_gear.servo_ids)}]",
        f"down_raw = {config.landing_gear.down_raw}",
        f"up_raw = {config.landing_gear.up_raw}",
        f"move_time_ms = {config.landing_gear.move_time_ms}",
        "",
    ]

    for index, profile_name in enumerate(sorted(config.servos)):
        profile = config.servos[profile_name]
        mapping = profile.mapping
        if index > 0:
            lines.append("")
        lines.extend(
            [
                f"[servos.{profile.name}]",
                f"id = {profile.id}",
                f"position_step = {mapping.position_step}",
                f"raw_min = {mapping.raw_min}",
                f"raw_max = {mapping.raw_max}",
            ]
        )
        if profile.home_raw is not None:
            lines.append(f"home_raw = {profile.home_raw}")
        if profile.startup_check_raw is not None:
            lines.append(f"startup_check_raw = {profile.startup_check_raw}")
        lines.extend(
            [
                f"mapped_angle_at_raw_min = {mapping.mapped_angle_at_raw_min}",
                f"mapped_angle_at_raw_max = {mapping.mapped_angle_at_raw_max}",
            ]
        )

    lines.append("")
    return "\n".join(lines)


def save_config(config: AppConfig, path: str | Path | None = None) -> Path:
    target = Path(path).resolve() if path is not None else config.source_path
    target.write_text(format_config_text(config), encoding="utf-8")
    return target


def build_updated_config(config: AppConfig, profile: ServoProfile) -> AppConfig:
    servos = dict(config.servos)
    servos[profile.name] = profile
    return AppConfig(
        serial=config.serial,
        defaults_position_step=config.defaults_position_step,
        landing_gear=config.landing_gear,
        servos=servos,
        source_path=config.source_path,
    )


def initial_snapshot_path(path: str | Path) -> Path:
    source = Path(path).resolve()
    return source.with_name(f"{source.stem}.initial{source.suffix}")


def ensure_initial_snapshot(path: str | Path) -> Path:
    source = Path(path).resolve()
    snapshot = initial_snapshot_path(source)
    if not snapshot.exists():
        shutil.copyfile(source, snapshot)
    return snapshot
