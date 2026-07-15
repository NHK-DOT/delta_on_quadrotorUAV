#!/usr/bin/env python3
"""Feedback-limited 8BitDo teleoperation between manually sampled raw poses."""

from __future__ import print_function

import argparse
import json
import os
import sys
import time

from jetson_workspace_common import DEFAULT_GAMEPAD_CONFIG, open_gamepad, open_servo_driver


def clamp(value, low, high):
    return max(low, min(high, value))


class SemanticJog(object):
    def __init__(self, args):
        self.args = args
        with open(args.workspace, "r") as fh:
            self.workspace = json.load(fh)
        self.servo_ids = tuple(int(item) for item in self.workspace["servo_ids"])
        if len(self.servo_ids) != 3:
            raise ValueError("workspace must contain exactly three servo IDs")
        self.labels = self.workspace["labels"]
        self.bounds = {
            servo_id: tuple(int(value) for value in self.workspace["raw_bounds"][str(servo_id)])
            for servo_id in self.servo_ids
        }
        self.driver = None
        self.gamepad = None
        self.feedback = {}
        self.command = {}
        self.target = {}
        self.running = True
        self.requested_label = "hold"
        self.last_feedback = 0.0
        self.last_status = 0.0
        self.output_dir = os.path.abspath(args.output_dir)
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.status_path = os.path.join(self.output_dir, "runtime_status.log")

    def connect(self):
        self.driver = open_servo_driver(self.args.port, self.args.baudrate)
        self.gamepad = open_gamepad(self.args.gamepad_config, self.args.gamepad_device)
        self.read_feedback(force=True)
        self.command = dict(self.feedback)
        self.target = dict(self.feedback)
        self.write_status()

    def close(self):
        if self.gamepad is not None:
            self.gamepad.close()
        if self.driver is not None:
            self.driver.close()

    def read_feedback(self, force=False):
        now = time.time()
        if not force and now - self.last_feedback < float(self.args.feedback_interval_sec):
            return True
        self.last_feedback = now
        last_error = None
        for _attempt in range(max(1, int(self.args.feedback_read_retries))):
            try:
                raw = self.driver.read_servo_positions(self.servo_ids, timeout=self.args.servo_timeout)
                self.feedback = {servo_id: int(raw[servo_id]) for servo_id in self.servo_ids}
                return True
            except Exception as exc:
                last_error = exc
                time.sleep(float(self.args.feedback_retry_delay_sec))
        print("feedback failed: %s" % last_error)
        return False

    def requested_endpoint(self):
        dpad_x, dpad_y, right_y, buttons = self.gamepad.read()
        if buttons.get("y", False):
            self.running = False
            return "emergency_stop"
        if buttons.get("a", False) or buttons.get("start", False):
            self.running = False
            return "quit"
        if right_y <= -0.25:
            return "top_home"
        if right_y >= 0.25:
            return "bottom_safe"
        if dpad_x >= 0.5:
            return "right_mid"
        if dpad_x <= -0.5:
            return "left_mid"
        if dpad_y >= 0.5:
            return "front_mid"
        if dpad_y <= -0.5:
            return "back_mid"
        return "hold"

    def set_target(self, label):
        self.requested_label = label
        if label == "hold":
            self.target = dict(self.feedback)
            return
        endpoint = self.labels[label]
        self.target = {
            servo_id: int(clamp(endpoint[index], self.bounds[servo_id][0], self.bounds[servo_id][1]))
            for index, servo_id in enumerate(self.servo_ids)
        }

    def step_and_send(self):
        step = max(1, int(round(float(self.args.raw_per_sec) / float(self.args.command_rate_hz))))
        lead = max(0, int(self.args.max_feedback_lead_ticks))
        next_command = {}
        changed = False
        for servo_id in self.servo_ids:
            current = int(self.command[servo_id])
            target = int(self.target[servo_id])
            value = current + int(clamp(target - current, -step, step))
            low, high = self.bounds[servo_id]
            if lead:
                value = int(clamp(value, self.feedback[servo_id] - lead, self.feedback[servo_id] + lead))
            value = int(clamp(value, low, high))
            next_command[servo_id] = value
            changed = changed or value != current
        if not changed:
            return True
        try:
            self.driver.set_servo_positions(
                [(servo_id, next_command[servo_id]) for servo_id in self.servo_ids],
                int(round(1000.0 / float(self.args.command_rate_hz))),
            )
            self.command = next_command
            return True
        except Exception as exc:
            print("send failed: %s" % exc)
            return False

    def write_status(self):
        with open(self.status_path, "w") as fh:
            fh.write("semantic raw jog\n")
            fh.write("requested: %s\n" % self.requested_label)
            fh.write("feedback: %s\n" % self.feedback)
            fh.write("target: %s\n" % self.target)
            fh.write("command: %s\n" % self.command)

    def run(self):
        self.connect()
        print("Semantic raw jog enabled. D-pad: left/right/front/back. Right stick: up/down. A: quit. Y: stop.")
        interval = 1.0 / float(self.args.command_rate_hz)
        while self.running:
            start = time.time()
            if not self.read_feedback():
                return 2
            self.set_target(self.requested_endpoint())
            if not self.running or not self.step_and_send():
                break
            if start - self.last_status >= 0.1:
                self.last_status = start
                self.write_status()
            time.sleep(max(0.0, interval - (time.time() - start)))
        self.write_status()
        return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--servo-timeout", type=float, default=0.5)
    parser.add_argument("--gamepad-config", default=DEFAULT_GAMEPAD_CONFIG)
    parser.add_argument("--gamepad-device", default="")
    parser.add_argument("--raw-per-sec", type=float, default=120.0)
    parser.add_argument("--command-rate-hz", type=float, default=20.0)
    parser.add_argument("--feedback-interval-sec", type=float, default=0.20)
    parser.add_argument("--feedback-read-retries", type=int, default=3)
    parser.add_argument("--feedback-retry-delay-sec", type=float, default=0.03)
    parser.add_argument("--max-feedback-lead-ticks", type=int, default=12)
    parser.add_argument("--output-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples", "semantic_jog"))
    return parser.parse_args()


def main():
    controller = SemanticJog(parse_args())
    try:
        return controller.run()
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
