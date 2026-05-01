from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    tomllib = None


def _linear_map(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    if in_min == in_max:
        raise ValueError("input range cannot be zero")
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)


@dataclass(frozen=True)
class ServoAxisMapping:
    name: str
    servo_id: int
    raw_min: int
    raw_max: int
    logical_min: float
    logical_max: float
    position_step: int = 1

    def __post_init__(self) -> None:
        if self.raw_min == self.raw_max:
            raise ValueError(f"{self.name}: raw_min and raw_max cannot be equal")
        if self.logical_min == self.logical_max:
            raise ValueError(f"{self.name}: logical_min and logical_max cannot be equal")
        if self.position_step <= 0:
            raise ValueError(f"{self.name}: position_step must be positive")

    @property
    def raw_low(self) -> int:
        return min(self.raw_min, self.raw_max)

    @property
    def raw_high(self) -> int:
        return max(self.raw_min, self.raw_max)

    @property
    def raw_span(self) -> int:
        return self.raw_max - self.raw_min

    @property
    def logical_span(self) -> float:
        return self.logical_max - self.logical_min

    def clamp_raw(self, raw_value: int | float) -> int:
        return max(self.raw_low, min(self.raw_high, int(round(float(raw_value)))))

    def quantize_raw(self, raw_value: int | float) -> int:
        quantized = int(round(float(raw_value) / self.position_step) * self.position_step)
        return self.clamp_raw(quantized)

    def raw_to_logical(self, raw_value: int | float) -> float:
        return _linear_map(
            float(raw_value),
            float(self.raw_min),
            float(self.raw_max),
            self.logical_min,
            self.logical_max,
        )

    def logical_to_raw(self, logical_value: float, *, quantize: bool = True) -> int:
        raw_value = _linear_map(
            float(logical_value),
            self.logical_min,
            self.logical_max,
            float(self.raw_min),
            float(self.raw_max),
        )
        return self.quantize_raw(raw_value) if quantize else self.clamp_raw(raw_value)

    def logical_units_per_degree(
        self,
        *,
        physical_min_deg: float = 0.0,
        physical_max_deg: float = 240.0,
    ) -> float:
        physical_span = abs(float(physical_max_deg) - float(physical_min_deg))
        if physical_span <= 0.0:
            raise ValueError("physical angle span must be positive")
        return abs(self.logical_span) / physical_span

    def physical_deg_to_logical(
        self,
        angle_deg: float,
        *,
        physical_min_deg: float = 0.0,
        physical_max_deg: float = 240.0,
    ) -> float:
        clamped_angle = max(
            min(float(angle_deg), max(physical_min_deg, physical_max_deg)),
            min(physical_min_deg, physical_max_deg),
        )
        return _linear_map(
            clamped_angle,
            float(physical_min_deg),
            float(physical_max_deg),
            self.logical_min,
            self.logical_max,
        )

    def logical_to_physical_deg(
        self,
        logical_value: float,
        *,
        physical_min_deg: float = 0.0,
        physical_max_deg: float = 240.0,
    ) -> float:
        physical_angle = _linear_map(
            float(logical_value),
            self.logical_min,
            self.logical_max,
            float(physical_min_deg),
            float(physical_max_deg),
        )
        return max(
            min(physical_angle, max(physical_min_deg, physical_max_deg)),
            min(physical_min_deg, physical_max_deg),
        )

    def physical_deg_to_raw(
        self,
        angle_deg: float,
        *,
        physical_min_deg: float = 0.0,
        physical_max_deg: float = 240.0,
    ) -> int:
        logical_value = self.physical_deg_to_logical(
            angle_deg,
            physical_min_deg=physical_min_deg,
            physical_max_deg=physical_max_deg,
        )
        return self.logical_to_raw(logical_value)

    def raw_to_physical_deg(
        self,
        raw_value: int | float,
        *,
        physical_min_deg: float = 0.0,
        physical_max_deg: float = 240.0,
    ) -> float:
        logical_value = self.raw_to_logical(raw_value)
        return self.logical_to_physical_deg(
            logical_value,
            physical_min_deg=physical_min_deg,
            physical_max_deg=physical_max_deg,
        )


def default_mapping_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "lx225_tool_demo" / "config" / "lx225_tool.demo.toml"


def _require_tomllib() -> None:
    if tomllib is None:
        raise RuntimeError("Python 3.11+ is required to load TOML servo mappings")


def load_servo_mappings(config_path: str | Path | None = None) -> dict[int, ServoAxisMapping]:
    _require_tomllib()
    source_path = Path(config_path).resolve() if config_path is not None else default_mapping_config_path()
    data = tomllib.loads(source_path.read_text(encoding="utf-8"))
    default_step = int(data.get("defaults", {}).get("position_step", 1))
    servo_section = data.get("servos", {})
    if not isinstance(servo_section, dict) or not servo_section:
        raise ValueError(f"No [servos.*] entries found in {source_path}")

    mappings: dict[int, ServoAxisMapping] = {}
    for name, item in servo_section.items():
        if not isinstance(item, dict):
            raise ValueError(f"Invalid servo config for {name!r}")
        mapping = ServoAxisMapping(
            name=str(name),
            servo_id=int(item["id"]),
            raw_min=int(item.get("raw_min", 0)),
            raw_max=int(item.get("raw_max", 1000)),
            logical_min=float(item.get("mapped_angle_at_raw_min", 0.0)),
            logical_max=float(item.get("mapped_angle_at_raw_max", 240.0)),
            position_step=int(item.get("position_step", default_step)),
        )
        mappings[mapping.servo_id] = mapping
    return mappings


def load_servo_mappings_for_ids(
    servo_ids: list[int],
    *,
    config_path: str | Path | None = None,
    strict: bool = True,
) -> dict[int, ServoAxisMapping]:
    mappings = load_servo_mappings(config_path=config_path)
    resolved: dict[int, ServoAxisMapping] = {}
    missing: list[int] = []
    for servo_id in servo_ids:
        mapping = mappings.get(int(servo_id))
        if mapping is None:
            missing.append(int(servo_id))
            continue
        resolved[int(servo_id)] = mapping
    if strict and missing:
        raise KeyError(f"Servo mapping config missing IDs: {missing}")
    return resolved
