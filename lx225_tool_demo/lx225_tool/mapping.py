from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoMapping:
    raw_min: int
    raw_max: int
    mapped_angle_at_raw_min: float
    mapped_angle_at_raw_max: float
    position_step: int = 1

    def __post_init__(self) -> None:
        if self.raw_min == self.raw_max:
            raise ValueError("raw_min and raw_max cannot be equal")
        if self.position_step <= 0:
            raise ValueError("position_step must be positive")

    @property
    def raw_span(self) -> int:
        return self.raw_max - self.raw_min

    @property
    def angle_span(self) -> float:
        return self.mapped_angle_at_raw_max - self.mapped_angle_at_raw_min

    @property
    def angle_per_raw(self) -> float:
        return self.angle_span / float(self.raw_span)

    def clamp_raw(self, raw_value: int) -> int:
        low = min(self.raw_min, self.raw_max)
        high = max(self.raw_min, self.raw_max)
        return max(low, min(high, raw_value))

    def quantize_raw(self, raw_value: int) -> int:
        stepped = int(round(raw_value / self.position_step) * self.position_step)
        return self.clamp_raw(stepped)

    def raw_to_angle(self, raw_value: int) -> float:
        ratio = (raw_value - self.raw_min) / float(self.raw_max - self.raw_min)
        return self.mapped_angle_at_raw_min + ratio * self.angle_span

    def angle_to_raw(self, angle_value: float, *, quantize: bool = True) -> int:
        if self.angle_span == 0:
            raise ValueError("angle span cannot be zero")
        ratio = (angle_value - self.mapped_angle_at_raw_min) / self.angle_span
        raw_value = int(round(self.raw_min + ratio * self.raw_span))
        raw_value = self.clamp_raw(raw_value)
        return self.quantize_raw(raw_value) if quantize else raw_value
