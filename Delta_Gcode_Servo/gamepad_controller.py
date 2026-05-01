#!/usr/bin/env python3
"""
Xbox 手柄实时控制 Delta 机械臂
十字键: X/Y 平面移动 (上=后, 下=前)
右摇杆: Z 垂直移动 (上下)
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Tuple

import numpy as np

# 导入舵机通信和运动学
sys.path.insert(0, str(Path(__file__).parent))
from delta_gcode_servo.servo import BusServoDriver
from delta_gcode_servo.config import robot_params
from delta_gcode_servo.kinematics import inverse_kinematics
from delta_gcode_servo.robot import DeltaRobot
from delta_gcode_servo.servo_mapping import load_servo_mappings_for_ids


class GamepadReader:
    """读取 Xbox 手柄输入"""
    
    def __init__(self):
        try:
            import pygame
            pygame.init()
            self.pygame = pygame
            
            # 初始化游戏杆
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                print("❌ 未检测到游戏杆/手柄")
                self.joystick = None
                return
            
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"✓ 检测到游戏杆: {self.joystick.get_name()}")
            print(f"  轴数: {self.joystick.get_numaxes()}")
            print(f"  十字键数: {self.joystick.get_numhats()}")
            print(f"  按钮数: {self.joystick.get_numbuttons()}")
            
        except ImportError:
            print("❌ pygame 未安装，请运行: pip install pygame")
            self.joystick = None
    
    def is_available(self) -> bool:
        return self.joystick is not None
    
    def read(self) -> Tuple[float, float, float, bool]:
        """
        读取手柄输入
        返回: (dpad_x, dpad_y, right_y, button_a)
        十字键只输出四向，不输出左前/右前/左后/右后。
        """
        if not self.joystick:
            return 0.0, 0.0, 0.0, False
        
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                return 0.0, 0.0, 0.0, True
        
        try:
            # Xbox 十字键/hat 映射: X 左-1 右+1, Y 下-1 上+1。
            # 机构约定: 十字键上=后, 十字键下=前。
            if self.joystick.get_numhats() > 0:
                dpad_x, dpad_y = self.joystick.get_hat(0)
                if dpad_x != 0 and dpad_y != 0:
                    dpad_x, dpad_y = 0, 0
            else:
                dpad_x, dpad_y = 0, 0

            # 轴 4: 右摇杆 Y (上-1, 下+1)
            right_y = self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else 0.0
            
            # 死区处理 (防止漂移)
            deadzone = 0.1
            right_y = 0.0 if abs(right_y) < deadzone else right_y
            
            # 检查按钮 A (按下返回 True)
            button_a = self.joystick.get_button(0)
            
            return float(dpad_x), float(dpad_y), right_y, button_a
            
        except Exception as e:
            print(f"⚠️  读取手柄出错: {e}")
            return 0.0, 0.0, 0.0, False


class RealTimeArmController:
    """实时机械臂控制器"""
    
    def __init__(self, port: str = "COM9", baudrate: int = 9600):
        self.driver = BusServoDriver(port=port, baudrate=baudrate, connect_delay=0.2)
        self.robot = DeltaRobot()
        self.params = robot_params()
        self.port = port
        
        # 当前状态
        self.servo_ids = [1, 2, 3]
        self.physical_angle_min_deg = float(self.params.servo_physical_angle_min_deg)
        self.physical_angle_max_deg = float(self.params.servo_physical_angle_max_deg)
        self.servo_mappings = load_servo_mappings_for_ids(self.servo_ids)
        self.servo_raw_directions = {1: -1, 2: -1, 3: -1}
        self.servo_logical_directions = {
            servo_id: self.servo_raw_directions[servo_id]
            * (1 if self.servo_mappings[servo_id].logical_span >= 0.0 else -1)
            for servo_id in self.servo_ids
        }
        self.servo_units_per_degree = {
            servo_id: self.servo_mappings[servo_id].logical_units_per_degree(
                physical_min_deg=self.physical_angle_min_deg,
                physical_max_deg=self.physical_angle_max_deg,
            )
            for servo_id in self.servo_ids
        }
        self.reference_servo_positions = {
            servo_id: self.servo_mappings[servo_id].quantize_raw(self.servo_mappings[servo_id].raw_max)
            for servo_id in self.servo_ids
        }
        self.reference_servo_coords = {
            servo_id: self.servo_mappings[servo_id].raw_to_logical(self.reference_servo_positions[servo_id])
            for servo_id in self.servo_ids
        }
        self.servo_limits = {
            servo_id: (
                self.servo_mappings[servo_id].raw_low,
                self.servo_mappings[servo_id].raw_high,
            )
            for servo_id in self.servo_ids
        }

        self.current_position = np.array(self.robot.current_position, dtype=float)
        self.current_angles, success = inverse_kinematics(
            self.current_position[0],
            self.current_position[1],
            self.current_position[2],
            self.params,
        )
        if not success:
            raise RuntimeError("Home position inverse kinematics failed")
        self.reference_angles = self.current_angles.copy()
        self.current_servo_positions = self.reference_servo_positions.copy()
        
        # 手柄读取器
        self.gamepad = GamepadReader()
        
        # 控制参数
        self.speed_xy = 1.2  # XY 平面移动速度 (mm/帧)
        self.speed_z = 0.9   # Z 垂直移动速度 (mm/帧)
        self.update_rate = 50  # 更新频率 (Hz)
        self.update_interval = 1.0 / self.update_rate
        
    def connect(self) -> bool:
        """连接到舵机和手柄"""
        try:
            print(f"\n正在连接到 {self.port}...")
            self.driver.connect()
            print(f"✓ 成功连接到舵机")
            
            if not self.gamepad.is_available():
                print("⚠️  警告: 手柄未能初始化")
                return False
            
            print("✓ 手柄已就绪")
            return True
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def servo_angles_to_positions(self, angles_rad: np.ndarray) -> dict[int, int]:
        positions: dict[int, int] = {}
        for index, servo_id in enumerate(self.servo_ids):
            delta_deg = float(np.degrees(angles_rad[index] - self.reference_angles[index]))
            target_coord = (
                self.reference_servo_coords[servo_id]
                + self.servo_logical_directions[servo_id] * delta_deg * self.servo_units_per_degree[servo_id]
            )
            positions[servo_id] = self.servo_mappings[servo_id].logical_to_raw(target_coord)
        return positions

    def servo_position_to_coord(self, servo_id: int, position: int) -> float:
        return self.servo_mappings[servo_id].raw_to_logical(position)
    
    def send_servo_positions(self, time_ms: int = 50) -> bool:
        """发送当前舵机位置到硬件"""
        if not self.driver.ser or not self.driver.ser.is_open:
            return False
        
        try:
            targets = []
            positions = self.servo_angles_to_positions(self.current_angles)
            for servo_id in self.servo_ids:
                min_pos, max_pos = self.servo_limits[servo_id]
                position = max(min_pos, min(max_pos, positions[servo_id]))
                targets.append((servo_id, position))
                self.current_servo_positions[servo_id] = position
            
            self.driver.set_servo_positions(targets, time_ms)
            return True
        except Exception as e:
            print(f"⚠️  发送失败: {e}")
            return False
    
    def update_from_gamepad(self) -> bool:
        """从手柄更新位置"""
        if not self.gamepad.is_available():
            return True  # 继续运行
        
        dpad_x, dpad_y, right_y, quit_btn = self.gamepad.read()
        
        if quit_btn:
            return False  # 退出
        
        # 计算目标位置增量
        # 十字键: X (左-1, 右+1), Y (上=后, 下=前)
        # 右摇杆: Z (上-1, 下+1)
        delta_x = dpad_x * self.speed_xy
        delta_y = -dpad_y * self.speed_xy
        delta_z = -right_y * self.speed_z   # 反转 Z 轴
        
        # 更新目标位置
        new_position = self.current_position.copy()
        new_position[0] += delta_x
        new_position[1] += delta_y
        new_position[2] += delta_z
        
        # 验证是否在工作空间内
        bounds = self.robot.get_workspace_bounds()
        new_position[0] = np.clip(new_position[0], bounds["x_min"], bounds["x_max"])
        new_position[1] = np.clip(new_position[1], bounds["y_min"], bounds["y_max"])
        new_position[2] = np.clip(new_position[2], bounds["z_min"], bounds["z_max"])
        
        # 计算逆运动学
        angles, success = inverse_kinematics(
            new_position[0],
            new_position[1],
            new_position[2],
            self.params
        )
        
        if not success:
            # 运动学失败，保持当前位置
            return True
        
        # 更新当前状态
        self.current_position = new_position
        self.current_angles = angles
        
        return True
    
    def print_status(self):
        """打印当前状态"""
        # 清屏 (可选)
        # print("\033[2J\033[H", end="")
        
        print("\n" + "="*70)
        print("Delta 机械臂 - 手柄实时控制")
        print("="*70)
        
        # 当前末端执行器位置
        print(f"\n末端执行器位置 (XYZ):")
        print(f"  X: {self.current_position[0]:7.2f} mm")
        print(f"  Y: {self.current_position[1]:7.2f} mm")
        print(f"  Z: {self.current_position[2]:7.2f} mm")
        
        # 关节角度
        print(f"\n关节角度 (度):")
        for i in range(3):
            angle_deg = np.degrees(self.current_angles[i])
            print(f"  θ{i+1}: {angle_deg:7.2f}°")
        
        # 舵机位置
        print(f"\n舵机 raw / mapped coord:")
        for servo_id in self.servo_ids:
            pos = self.current_servo_positions[servo_id]
            coord = self.servo_position_to_coord(servo_id, pos)
            min_pos, max_pos = self.servo_limits[servo_id]
            print(f"  舵机 {servo_id}: raw {pos:4d} (coord {coord:7.2f}, 范围 {min_pos}-{max_pos})")
        
        # 工作空间检查
        bounds = self.robot.get_workspace_bounds()
        in_bounds = (
            bounds["x_min"] <= self.current_position[0] <= bounds["x_max"]
            and bounds["y_min"] <= self.current_position[1] <= bounds["y_max"]
            and bounds["z_min"] <= self.current_position[2] <= bounds["z_max"]
        )
        status = "✓ 在工作空间内" if in_bounds else "⚠️  超出工作空间"
        print(f"\n工作空间检查: {status}")
        
        # 手柄提示
        print(f"\n手柄控制:")
        print(f"  十字键: X/Y 平面四向移动 (速度 {self.speed_xy} mm/帧)")
        print(f"  右摇杆: Z 垂直移动 (速度 {self.speed_z} mm/帧)")
        print(f"  按钮 A: 退出 (按下按钮 A 或 Ctrl+C)")
    
    def run(self):
        """主控制循环"""
        if not self.connect():
            return
        
        print("\n✓ 系统就绪，开始实时控制...")
        print("⚠️  请确保机械臂周围空间足够，避免碰撞！\n")
        
        try:
            last_print = 0
            while True:
                # 从手柄读取并更新
                if not self.update_from_gamepad():
                    print("\n⚠️  退出指令收到")
                    break
                
                # 发送舵机指令
                self.send_servo_positions(time_ms=int(self.update_interval * 1000))
                
                # 定期打印状态 (每秒一次)
                now = time.time()
                if now - last_print >= 1.0:
                    self.print_status()
                    last_print = now
                
                # 控制更新频率
                time.sleep(self.update_interval)
        
        except KeyboardInterrupt:
            print("\n✓ 程序被中断")
        except Exception as e:
            print(f"\n❌ 出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        try:
            self.driver.close()
            print("\n✓ 已安全断开连接")
        except:
            pass


def main():
    try:
        port = input("输入串口 (默认 COM9): ").strip() or "COM9"
        
        controller = RealTimeArmController(port=port)
        controller.run()
        
    except Exception as e:
        print(f"❌ 出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
