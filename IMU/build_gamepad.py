code = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import sys, time
from pathlib import Path
from typing import Tuple
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))
from delta_gcode_servo.servo import BusServoDriver
from delta_gcode_servo.config import robot_params
from delta_gcode_servo.kinematics import inverse_kinematics, forward_kinematics
from delta_gcode_servo.robot import DeltaRobot

class GamepadReader:
    def __init__(self):
        try:
            import pygame
            pygame.init()
            if pygame.joystick.get_count() == 0:
                self.joystick = None
                return
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        except:
            self.joystick = None
    def is_available(self):
        return self.joystick is not None
    def read(self):
        if not self.joystick:
            return 0.0, 0.0, 0.0, False
        try:
            lx, ly = self.joystick.get_axis(0), self.joystick.get_axis(1)
            ry = self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else 0.0
            lx = 0 if abs(lx) < 0.1 else lx
            ly = 0 if abs(ly) < 0.1 else ly
            ry = 0 if abs(ry) < 0.1 else ry
            return lx, ly, ry, self.joystick.get_button(0)
        except:
            return 0.0, 0.0, 0.0, False

class RealTimeArmController:
    def __init__(self, port="COM9", baudrate=9600):
        self.driver = BusServoDriver(port=port, baudrate=baudrate, connect_delay=0.2)
        self.robot = DeltaRobot()
        self.params = robot_params()
        self.current_servo_positions = {1: 1000, 2: 1000, 3: 1000}
        self.servo_limits = {1: (500, 1000), 2: (500, 920), 3: (500, 1000)}
        self.current_position = None
        self.current_angles_rad = None
        self.gamepad = GamepadReader()
        self.is_ready = False
    def connect(self):
        try:
            print(f"Connecting to {self.port}...")
            self.driver.connect()
            if not self.gamepad.is_available():
                return False
            return True
        except:
            return False
    def angle_to_position(self, angle_deg):
        pos = int((angle_deg / 240.0) * 1000)
        return max(0, min(1000, pos))
    def position_to_angle(self, position):
        return (position / 1000.0) * 240.0
    def read_actual_servo_positions(self):
        print("Confirm servo positions")
        resp = input("All servos at 1000 (servo2 at 920)? (y/n): ").strip().lower()
        return resp == 'y'
    def init_from_servo_positions(self):
        try:
            angles_deg = np.array([self.position_to_angle(self.current_servo_positions[i]) for i in [1,2,3]], dtype=float)
            angles_rad = np.radians(angles_deg)
            position, success = forward_kinematics(angles_rad[0], angles_rad[1], angles_rad[2], self.params)
            if success:
                self.current_position = position
            else:
                self.current_position = np.array([0.0, 0.0, 240.0], dtype=float)
            self.current_angles_rad = angles_rad
            return True
        except:
            return False
    def confirm_and_init(self):
        if not self.read_actual_servo_positions():
            return False
        if not self.init_from_servo_positions():
            return False
        self.is_ready = True
        print("System ready!")
        return True
    def send_servo_positions(self, time_ms=50):
        if not self.is_ready or not self.driver.ser:
            return False
        try:
            targets = []
            for i in [1,2,3]:
                angle = np.degrees(self.current_angles_rad[i-1])
                pos = self.angle_to_position(angle)
                pos = max(self.servo_limits[i][0], min(self.servo_limits[i][1], pos))
                targets.append((i, pos))
            self.driver.set_servo_positions(targets, time_ms)
            return True
        except:
            return False
    def update_from_gamepad(self):
        if not self.gamepad.is_available():
            return True, False
        lx, ly, ry, q = self.gamepad.read()
        if q:
            return False, False
        if abs(lx) < 0.01 and abs(ly) < 0.01 and abs(ry) < 0.01:
            return True, False
        new_pos = self.current_position.copy()
        new_pos[0] += lx * 2.0
        new_pos[1] -= ly * 2.0
        new_pos[2] -= ry * 1.5
        angles_rad, success = inverse_kinematics(new_pos[0], new_pos[1], new_pos[2], self.params)
        if not success:
            return True, False
        self.current_position = new_pos
        self.current_angles_rad = angles_rad
        return True, True
    def run(self):
        if not self.connect():
            return
        if not self.confirm_and_init():
            return
        print("Starting control...")
        try:
            while True:
                cont, has_input = self.update_from_gamepad()
                if not cont:
                    break
                if has_input:
                    self.send_servo_positions()
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
    def cleanup(self):
        try:
            self.driver.close()
        except:
            pass

def main():
    port = input("Port (COM9): ").strip() or "COM9"
    controller = RealTimeArmController(port=port)
    controller.run()

if __name__ == "__main__":
    main()
'''

with open("C:\\Users\\hanjuncheng\\Desktop\\nodejs\\Delta_Gcode_Servo\\real_machine_test\\gamepad_controller.py", "w", encoding="utf-8") as f:
    f.write(code)

print("Done!")
