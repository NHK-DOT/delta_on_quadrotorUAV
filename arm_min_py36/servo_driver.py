import time

import serial


class Packet(object):
    HEADER = b"\x55\x55"

    @staticmethod
    def pack(cmd, params):
        length = 2 + len(params)
        packet = bytearray(Packet.HEADER)
        packet.append(length & 0xFF)
        packet.append(cmd & 0xFF)
        for item in params:
            packet.append(int(item) & 0xFF)
        return bytes(packet)


class BusServoDriver(object):
    CMD_SERVO_MOVE = 0x03
    CMD_GET_BATTERY_VOLTAGE = 0x0F
    CMD_MULT_SERVO_UNLOAD = 0x14
    CMD_MULT_SERVO_POS_READ = 0x15

    def __init__(self, port, baudrate=9600, timeout=1.0, connect_delay=0.2, trace_hook=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_delay = connect_delay
        self.trace_hook = trace_hook
        self.ser = None

    def _trace(self, direction, packet, note=""):
        if self.trace_hook is None:
            return
        try:
            self.trace_hook(direction, packet, note)
        except Exception:
            pass

    def is_open(self):
        return self.ser is not None and bool(getattr(self.ser, "is_open", False))

    def connect(self):
        if self.is_open():
            return
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )
        if self.connect_delay > 0:
            time.sleep(self.connect_delay)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.is_open():
            self.ser.close()

    def write_packet(self, cmd, params=None):
        if not self.is_open():
            raise RuntimeError("serial port not open")
        packet = Packet.pack(cmd, params or [])
        self._trace("TX", packet, "cmd=0x%02X" % cmd)
        self.ser.write(packet)
        self.ser.flush()

    def read_packet(self, expected_cmd=None, timeout=None):
        if not self.is_open():
            raise RuntimeError("serial port not open")
        deadline = time.time() + (self.timeout if timeout is None else float(timeout))
        while time.time() < deadline:
            first = self.ser.read(1)
            if first != b"\x55":
                continue
            second = self.ser.read(1)
            if second != b"\x55":
                continue
            length_raw = self.ser.read(1)
            cmd_raw = self.ser.read(1)
            if len(length_raw) != 1 or len(cmd_raw) != 1:
                continue
            length = length_raw[0]
            cmd = cmd_raw[0]
            payload_len = max(0, length - 2)
            payload = self.ser.read(payload_len)
            if len(payload) != payload_len:
                continue
            packet = Packet.HEADER + length_raw + cmd_raw + payload
            self._trace("RX", packet, "cmd=0x%02X" % cmd)
            if expected_cmd is not None and cmd != expected_cmd:
                self._trace("RX_SKIP", packet, "expected=0x%02X actual=0x%02X" % (expected_cmd, cmd))
                continue
            return cmd, list(payload)
        self._trace("RX_TIMEOUT", b"", "expected=%r" % (expected_cmd,))
        raise RuntimeError("timeout waiting for controller response")

    def set_servo_positions(self, targets, time_ms):
        params = [len(targets), int(time_ms) & 0xFF, (int(time_ms) >> 8) & 0xFF]
        for servo_id, position in targets:
            position = int(position)
            params.extend([int(servo_id), position & 0xFF, (position >> 8) & 0xFF])
        self.write_packet(self.CMD_SERVO_MOVE, params)

    def unload_servos(self, servo_ids):
        ids = list(dict.fromkeys(int(servo_id) for servo_id in servo_ids))
        if ids:
            self.write_packet(self.CMD_MULT_SERVO_UNLOAD, [len(ids)] + ids)

    def read_servo_positions(self, servo_ids, timeout=None):
        ids = list(dict.fromkeys(int(servo_id) for servo_id in servo_ids))
        if not ids:
            return {}
        if not self.is_open():
            raise RuntimeError("serial port not open")
        self.ser.reset_input_buffer()
        self.write_packet(self.CMD_MULT_SERVO_POS_READ, [len(ids)] + ids)
        _, payload = self.read_packet(expected_cmd=self.CMD_MULT_SERVO_POS_READ, timeout=timeout)
        if not payload:
            raise RuntimeError("empty servo position payload")
        count = payload[0]
        expected_len = 1 + count * 3
        if len(payload) != expected_len:
            raise RuntimeError("bad servo position payload length: %d != %d" % (len(payload), expected_len))
        positions = {}
        offset = 1
        for _ in range(count):
            servo_id = payload[offset]
            position = payload[offset + 1] | (payload[offset + 2] << 8)
            positions[servo_id] = position
            offset += 3
        missing = [servo_id for servo_id in ids if servo_id not in positions]
        if missing:
            raise RuntimeError("controller response missing servo IDs: %s" % missing)
        return positions

    def get_battery_voltage_mv(self, timeout=None):
        if not self.is_open():
            raise RuntimeError("serial port not open")
        self.ser.reset_input_buffer()
        self.write_packet(self.CMD_GET_BATTERY_VOLTAGE, [])
        _, payload = self.read_packet(expected_cmd=self.CMD_GET_BATTERY_VOLTAGE, timeout=timeout)
        if len(payload) != 2:
            raise RuntimeError("bad voltage payload length: %d" % len(payload))
        return payload[0] | (payload[1] << 8)


def serial_permission_hint(port):
    return (
        "If %s fails with Permission denied, add the user to dialout/tty or run:\n"
        "  sudo chmod a+rw %s\n"
        "Then unplug/replug the USB serial adapter or log out and log in again."
    ) % (port, port)
