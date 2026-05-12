#!/usr/bin/env python3
import argparse
import sys
import time

from config import BAUDRATE, DEFAULT_PORT
from servo_driver import BusServoDriver, serial_permission_hint
from servo_mapping import load_servo_mappings_for_ids


SERVO_IDS = [1, 2, 3]


class ServoCalibrator(object):
    def __init__(self, port):
        self.port = port
        self.driver = BusServoDriver(port=port, baudrate=BAUDRATE, timeout=1.0, connect_delay=0.2)
        self.mappings = load_servo_mappings_for_ids(SERVO_IDS)
        self.positions = {}
        for servo_id in SERVO_IDS:
            self.positions[servo_id] = self.mappings[servo_id].quantize_raw(
                self.mappings[servo_id].raw_max
            )

    def connect(self):
        try:
            print("Opening serial %s @ %d..." % (self.port, BAUDRATE))
            self.driver.connect()
            print("Serial opened.")
            return True
        except Exception as exc:
            print("Serial open failed: %s" % exc)
            print(serial_permission_hint(self.port))
            return False

    def read_positions(self):
        try:
            feedback = self.driver.read_servo_positions(SERVO_IDS, timeout=0.25)
            for servo_id in SERVO_IDS:
                self.positions[servo_id] = self.mappings[servo_id].quantize_raw(feedback[servo_id])
            self.print_status()
        except Exception as exc:
            print("Read failed: %s" % exc)

    def print_status(self):
        print("")
        for servo_id in SERVO_IDS:
            mapping = self.mappings[servo_id]
            raw = self.positions[servo_id]
            print(
                "servo%d raw=%d coord=%.2f range=%d..%d step=%d"
                % (servo_id, raw, mapping.raw_to_logical(raw), mapping.raw_low, mapping.raw_high, mapping.position_step)
            )

    def send(self, servo_id, raw, time_ms=500):
        mapping = self.mappings[servo_id]
        raw = mapping.quantize_raw(raw)
        answer = input("Move servo%d to raw %d? type YES: " % (servo_id, raw)).strip()
        if answer != "YES":
            print("Cancelled.")
            return
        self.driver.set_servo_positions([(servo_id, raw)], time_ms)
        self.positions[servo_id] = raw
        time.sleep(time_ms / 1000.0 + 0.05)
        print("servo%d raw=%d coord=%.2f" % (servo_id, raw, mapping.raw_to_logical(raw)))

    def send_all_reference(self):
        targets = []
        for servo_id in SERVO_IDS:
            mapping = self.mappings[servo_id]
            raw = mapping.quantize_raw(mapping.raw_max)
            targets.append((servo_id, raw))
        answer = input("Move all servos to reference %s? type YES: " % (targets,)).strip()
        if answer != "YES":
            print("Cancelled.")
            return
        self.driver.set_servo_positions(targets, 800)
        for servo_id, raw in targets:
            self.positions[servo_id] = raw
        time.sleep(0.9)
        self.print_status()

    def run_servo_menu(self, servo_id):
        mapping = self.mappings[servo_id]
        print("")
        print("servo%d menu: + - ++ -- number b" % servo_id)
        print("range=%d..%d step=%d" % (mapping.raw_low, mapping.raw_high, mapping.position_step))
        while True:
            raw = self.positions[servo_id]
            cmd = input("servo%d raw=%d > " % (servo_id, raw)).strip().lower()
            if cmd == "b":
                return
            if cmd == "+":
                self.send(servo_id, raw + mapping.position_step, 300)
            elif cmd == "-":
                self.send(servo_id, raw - mapping.position_step, 300)
            elif cmd == "++":
                self.send(servo_id, raw + mapping.position_step * 5, 500)
            elif cmd == "--":
                self.send(servo_id, raw - mapping.position_step * 5, 500)
            else:
                try:
                    value = int(cmd)
                except ValueError:
                    print("Unknown command.")
                    continue
                self.send(servo_id, value, 500)

    def run(self):
        if not self.connect():
            return
        self.read_positions()
        print("")
        print("Commands:")
        print("  1/2/3: jog one servo")
        print("  s: read and show current raw positions")
        print("  r: move all servos to controller reference pose")
        print("  q: quit")
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                self.read_positions()
            elif cmd == "r":
                self.send_all_reference()
            elif cmd in ("1", "2", "3"):
                self.run_servo_menu(int(cmd))
            else:
                print("Unknown command.")
        self.close()

    def close(self):
        try:
            self.driver.close()
        except Exception:
            pass


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Minimal LX225 servo calibration helper.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device, default: %(default)s")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    calibrator = ServoCalibrator(args.port)
    try:
        calibrator.run()
    finally:
        calibrator.close()


if __name__ == "__main__":
    main()
