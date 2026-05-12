from config import SERVO_MAPPINGS


def _linear_map(x, in_min, in_max, out_min, out_max):
    if in_min == in_max:
        raise ValueError("input range cannot be zero")
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)


class ServoAxisMapping(object):
    def __init__(self, name, servo_id, raw_min, raw_max, logical_min, logical_max, position_step=1):
        if raw_min == raw_max:
            raise ValueError("%s: raw_min and raw_max cannot be equal" % name)
        if logical_min == logical_max:
            raise ValueError("%s: logical_min and logical_max cannot be equal" % name)
        if position_step <= 0:
            raise ValueError("%s: position_step must be positive" % name)
        self.name = name
        self.servo_id = int(servo_id)
        self.raw_min = int(raw_min)
        self.raw_max = int(raw_max)
        self.logical_min = float(logical_min)
        self.logical_max = float(logical_max)
        self.position_step = int(position_step)

    @property
    def raw_low(self):
        return min(self.raw_min, self.raw_max)

    @property
    def raw_high(self):
        return max(self.raw_min, self.raw_max)

    @property
    def logical_span(self):
        return self.logical_max - self.logical_min

    def clamp_raw(self, raw_value):
        value = int(round(float(raw_value)))
        return max(self.raw_low, min(self.raw_high, value))

    def quantize_raw(self, raw_value):
        value = int(round(float(raw_value) / self.position_step) * self.position_step)
        return self.clamp_raw(value)

    def raw_to_logical(self, raw_value):
        return _linear_map(
            float(raw_value),
            float(self.raw_min),
            float(self.raw_max),
            self.logical_min,
            self.logical_max,
        )

    def logical_to_raw(self, logical_value):
        raw_value = _linear_map(
            float(logical_value),
            self.logical_min,
            self.logical_max,
            float(self.raw_min),
            float(self.raw_max),
        )
        return self.quantize_raw(raw_value)

    def logical_units_per_degree(self, physical_min_deg=0.0, physical_max_deg=240.0):
        span = abs(float(physical_max_deg) - float(physical_min_deg))
        if span <= 0.0:
            raise ValueError("physical angle span must be positive")
        return abs(self.logical_span) / span


def load_servo_mappings_for_ids(servo_ids):
    mappings = {}
    for servo_id in servo_ids:
        item = SERVO_MAPPINGS[int(servo_id)]
        mappings[int(servo_id)] = ServoAxisMapping(
            name=item["name"],
            servo_id=int(servo_id),
            raw_min=item["raw_min"],
            raw_max=item["raw_max"],
            logical_min=item["logical_min"],
            logical_max=item["logical_max"],
            position_step=item["position_step"],
        )
    return mappings
