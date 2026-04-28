from __future__ import annotations

from dataclasses import dataclass

from .config import AppConfig, ServoProfile
from .driver import LX225Driver


@dataclass(frozen=True)
class ServoSnapshot:
    servo_name: str | None
    servo_id: int
    position_raw: int | None
    mapped_angle: float | None
    offset: int | None
    limit_min_raw: int | None
    limit_max_raw: int | None
    errors: tuple[str, ...] = ()


class LX225Service:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.driver = LX225Driver(
            port=config.serial.port,
            baudrate=config.serial.baudrate,
            timeout=config.serial.timeout,
            connect_delay=config.serial.connect_delay,
        )

    def _simple_timeout(self) -> float:
        return max(0.6, float(self.config.serial.timeout))

    def __enter__(self) -> "LX225Service":
        self.driver.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.driver.close()

    def connect(self) -> None:
        self.driver.connect()

    def close(self) -> None:
        self.driver.close()

    def resolve_profile(self, *, servo_name: str | None = None, servo_id: int | None = None) -> ServoProfile | None:
        if servo_name is not None:
            return self.config.resolve_servo(servo_name)
        if servo_id is not None:
            for profile in self.config.servos.values():
                if profile.id == servo_id:
                    return profile
        return None

    def read_snapshot(
        self,
        *,
        servo_name: str | None = None,
        servo_id: int | None = None,
        best_effort: bool = False,
    ) -> ServoSnapshot:
        profile = self.resolve_profile(servo_name=servo_name, servo_id=servo_id)
        actual_id = profile.id if profile is not None else int(servo_id)
        if actual_id is None:
            raise ValueError("servo_name or servo_id is required")

        if not best_effort:
            try:
                position_raw = self.driver.read_position(actual_id)
            except Exception:
                position_raw = self.driver.read_position_simple(actual_id, timeout=self._simple_timeout())
                if position_raw is None:
                    raise
            offset = self.driver.read_offset(actual_id)
            limit_min_raw, limit_max_raw = self.driver.read_angle_limit(actual_id)
            mapped_angle = profile.mapping.raw_to_angle(position_raw) if profile is not None else None
            return ServoSnapshot(
                servo_name=profile.name if profile is not None else None,
                servo_id=actual_id,
                position_raw=position_raw,
                mapped_angle=mapped_angle,
                offset=offset,
                limit_min_raw=limit_min_raw,
                limit_max_raw=limit_max_raw,
            )

        position_raw: int | None = None
        offset: int | None = None
        limit_min_raw: int | None = None
        limit_max_raw: int | None = None
        errors: list[str] = []

        try:
            position_raw = self.driver.read_position(actual_id)
        except Exception as exc:
            errors.append(f"position_raw: {exc}")
            try:
                position_raw = self.driver.read_position_simple(actual_id, timeout=self._simple_timeout())
                if position_raw is not None:
                    errors.append("position_raw fallback: simple controller read protocol")
            except Exception as exc_simple:
                errors.append(f"position_raw simple_fallback: {exc_simple}")

        try:
            offset = self.driver.read_offset(actual_id)
        except Exception as exc:
            errors.append(f"offset: {exc}")

        try:
            limit_min_raw, limit_max_raw = self.driver.read_angle_limit(actual_id)
        except Exception as exc:
            errors.append(f"angle_limit: {exc}")

        mapped_angle = None
        if profile is not None and position_raw is not None:
            mapped_angle = profile.mapping.raw_to_angle(position_raw)
        return ServoSnapshot(
            servo_name=profile.name if profile is not None else None,
            servo_id=actual_id,
            position_raw=position_raw,
            mapped_angle=mapped_angle,
            offset=offset,
            limit_min_raw=limit_min_raw,
            limit_max_raw=limit_max_raw,
            errors=tuple(errors),
        )

    def discover_single_id(self) -> int:
        try:
            return self.driver.discover_single_id()
        except Exception:
            found = self.driver.scan_simple_positions(timeout=self._simple_timeout())
            if not found:
                raise
            if len(found) != 1:
                raise RuntimeError(f"Simple scan found multiple servo IDs: {sorted(found)}")
            return next(iter(found))

    def read_id(self, servo_id: int) -> int:
        try:
            return self.driver.read_id(servo_id)
        except Exception:
            position = self.driver.read_position_simple(servo_id, timeout=self._simple_timeout())
            if position is None:
                raise
            return servo_id

    def write_id(self, old_id: int, new_id: int) -> None:
        self.driver.write_id(old_id, new_id)

    def read_angle_limit(self, *, servo_name: str | None = None, servo_id: int | None = None) -> tuple[int, int]:
        profile = self.resolve_profile(servo_name=servo_name, servo_id=servo_id)
        actual_id = profile.id if profile is not None else int(servo_id)
        return self.driver.read_angle_limit(actual_id)

    def write_angle_limit(
        self,
        *,
        servo_name: str | None = None,
        servo_id: int | None = None,
        raw_min: int,
        raw_max: int,
    ) -> None:
        if raw_min > raw_max:
            raise ValueError("raw_min must be <= raw_max")
        profile = self.resolve_profile(servo_name=servo_name, servo_id=servo_id)
        actual_id = profile.id if profile is not None else int(servo_id)
        self.driver.write_angle_limit(actual_id, raw_min, raw_max)

    def capture_limit(self, *, servo_name: str, side: str) -> tuple[int, int, int]:
        profile = self.config.resolve_servo(servo_name)
        current_raw = self.driver.read_position(profile.id)
        old_min, old_max = self.driver.read_angle_limit(profile.id)
        if side == "min":
            new_min, new_max = current_raw, old_max
        elif side == "max":
            new_min, new_max = old_min, current_raw
        else:
            raise ValueError("side must be 'min' or 'max'")
        if new_min > new_max:
            raise ValueError(
                f"Captured {side} limit would invert the range: {new_min} > {new_max}. Check servo direction first."
            )
        self.driver.write_angle_limit(profile.id, new_min, new_max)
        return current_raw, new_min, new_max

    def read_offset(self, *, servo_name: str | None = None, servo_id: int | None = None) -> int:
        profile = self.resolve_profile(servo_name=servo_name, servo_id=servo_id)
        actual_id = profile.id if profile is not None else int(servo_id)
        return self.driver.read_offset(actual_id)

    def set_offset(self, *, servo_name: str | None = None, servo_id: int | None = None, offset: int, save: bool) -> None:
        profile = self.resolve_profile(servo_name=servo_name, servo_id=servo_id)
        actual_id = profile.id if profile is not None else int(servo_id)
        self.driver.adjust_offset(actual_id, offset)
        if save:
            self.driver.save_offset(actual_id)
