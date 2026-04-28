#!/usr/bin/env python3
"""
安全的 LX-225 舵机校准工具 - 实机测试版本
支持逐个舵机的增量移动和反向测试
"""

from __future__ import annotations

import time
import sys
from pathlib import Path

# 添加包到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from delta_gcode_servo.servo import BusServoDriver
from delta_gcode_servo.config import robot_params
from delta_gcode_servo.servo_mapping import load_servo_mappings_for_ids


class ServoCalibrator:
    def __init__(self, port: str = "COM9", baudrate: int = 9600):
        self.driver = BusServoDriver(port=port, baudrate=baudrate, connect_delay=0.2)
        self.params = robot_params()
        self.port = port
        self.servo_ids = [1, 2, 3]
        self.servo_mappings = load_servo_mappings_for_ids(self.servo_ids)
        self.current_positions = {
            servo_id: self.servo_mappings[servo_id].quantize_raw(self.servo_mappings[servo_id].raw_max)
            for servo_id in self.servo_ids
        }
        self.servo_limits = {
            servo_id: (
                self.servo_mappings[servo_id].raw_low,
                self.servo_mappings[servo_id].raw_high,
            )
            for servo_id in self.servo_ids
        }

    def quantize_position(self, servo_id: int, position: int | float) -> int:
        return self.servo_mappings[servo_id].quantize_raw(position)

    def position_to_coord(self, servo_id: int, position: int | float) -> float:
        return self.servo_mappings[servo_id].raw_to_logical(position)

    def connect(self) -> bool:
        """连接到舵机"""
        try:
            print(f"\n正在连接到 {self.port}...")
            self.driver.connect()
            print(f"✓ 成功连接到 {self.port}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False

    def send_position(self, servo_id: int, position: int, time_ms: int = 500) -> bool:
        """安全地发送舵机位置指令"""
        if not self.driver.ser or not self.driver.ser.is_open:
            print("错误: 串口未打开")
            return False

        min_pos, max_pos = self.servo_limits[servo_id]
        position = self.quantize_position(servo_id, position)
        position = max(min_pos, min(max_pos, position))

        try:
            self.driver.set_servo_positions([(servo_id, position)], time_ms)
            self.current_positions[servo_id] = position
            coord = self.position_to_coord(servo_id, position)
            print(f"  舵机 {servo_id}: raw {position} (coord {coord:.2f})")
            time.sleep(time_ms / 1000.0 + 0.1)
            return True
        except Exception as e:
            print(f"✗ 发送命令失败: {e}")
            return False

    def test_single_servo(self, servo_id: int):
        """逐个测试舵机的增量移动和方向验证"""
        mapping = self.servo_mappings[servo_id]
        min_pos, max_pos = self.servo_limits[servo_id]
        small_step = mapping.position_step
        large_step = mapping.position_step * 5

        print(f"\n{'='*60}")
        print(f"舵机 {servo_id} 调试模式")
        print(f"{'='*60}")
        print("说明:")
        print(f"  - raw 范围: {min_pos}-{max_pos}")
        print(f"  - 映射值范围: {mapping.logical_min:.2f}-{mapping.logical_max:.2f}")
        print(f"  - 量化步长: {mapping.position_step}")
        print("\n命令:")
        print(f"  +: 增加 {small_step} raw")
        print(f"  -: 减少 {small_step} raw")
        print(f"  ++: 增加 {large_step} raw（谨慎使用）")
        print(f"  --: 减少 {large_step} raw（谨慎使用）")
        print("  c: 自定义 raw 位置（谨慎使用）")
        print("  q: 回到主菜单")
        print("  x: 退出程序")

        while True:
            try:
                pos = self.current_positions[servo_id]
                coord = self.position_to_coord(servo_id, pos)
                print(f"\n舵机 {servo_id} | 当前: raw={pos}, coord={coord:.2f}")
                cmd = input("输入命令 > ").strip().lower()

                if cmd == 'q':
                    break
                if cmd == 'x':
                    return False
                if cmd == '+':
                    new_pos = min(max_pos, pos + small_step)
                    if new_pos == pos:
                        print(f"  已到达最大值 {max_pos}")
                    else:
                        self.send_position(servo_id, new_pos, 300)
                elif cmd == '-':
                    new_pos = max(min_pos, pos - small_step)
                    if new_pos == pos:
                        print(f"  已到达最小值 {min_pos}")
                    else:
                        self.send_position(servo_id, new_pos, 300)
                elif cmd == '++':
                    print(f"  ⚠️  大幅增加 {large_step} raw，请确保安全！")
                    if input("  继续? (y/n): ").lower() == 'y':
                        self.send_position(servo_id, min(max_pos, pos + large_step), 500)
                elif cmd == '--':
                    print(f"  ⚠️  大幅减少 {large_step} raw，请确保安全！")
                    if input("  继续? (y/n): ").lower() == 'y':
                        self.send_position(servo_id, max(min_pos, pos - large_step), 500)
                elif cmd == 'c':
                    try:
                        custom_pos = int(input(f"输入 raw 位置 ({min_pos}-{max_pos}): "))
                        if min_pos <= custom_pos <= max_pos:
                            print("  ⚠️  自定义位置，请确保安全！")
                            if input("  继续? (y/n): ").lower() == 'y':
                                self.send_position(servo_id, custom_pos, 500)
                        else:
                            print(f"错误: 输入范围外 ({min_pos}-{max_pos})")
                    except ValueError:
                        print("错误: 请输入有效的数字")
                else:
                    print("未知命令")
            except KeyboardInterrupt:
                print("\n中断")
                break
            except Exception as e:
                print(f"错误: {e}")

        return True

    def main_menu(self):
        """主菜单"""
        while True:
            print(f"\n{'='*60}")
            print("Delta 机械臂 - LX-225 舵机调试工具 [实机测试版]")
            print(f"{'='*60}")
            print("选项:")
            for servo_id in self.servo_ids:
                min_pos, max_pos = self.servo_limits[servo_id]
                print(f"  {servo_id}: 调试舵机 {servo_id} (raw {min_pos}-{max_pos})")
            print("  s: 显示当前舵机状态")
            print("  d: 断开连接")
            print("  x: 退出")

            cmd = input("\n输入选项 > ").strip().lower()

            if cmd in {"1", "2", "3"}:
                if not self.test_single_servo(int(cmd)):
                    break
            elif cmd == 's':
                self.show_status()
            elif cmd == 'd':
                self.driver.close()
                print("✓ 已断开连接")
                if input("重新连接? (y/n): ").lower() == 'y':
                    if not self.connect():
                        break
            elif cmd == 'x':
                break
            else:
                print("未知选项")

        self.cleanup()

    def show_status(self):
        """显示当前舵机状态"""
        print(f"\n{'舵机ID':<10} {'raw':<10} {'coord':<12}")
        print("-" * 36)
        for servo_id in self.servo_ids:
            pos = self.current_positions[servo_id]
            coord = self.position_to_coord(servo_id, pos)
            print(f"{servo_id:<10} {pos:<10} {coord:<12.2f}")

    def cleanup(self):
        """清理资源"""
        try:
            self.driver.close()
            print("\n✓ 已安全断开连接")
        except Exception:
            pass


def main():
    try:
        port = input("输入串口 (默认 COM9): ").strip() or "COM9"
        
        calibrator = ServoCalibrator(port=port)
        
        if not calibrator.connect():
            return
        
        print("\n✓ 已连接，建议先把 3 个舵机放在各自 raw_max 对应的安全准备位")
        print("  下面的加减步进会按配置文件里的 position_step 量化")
        time.sleep(0.5)
        
        calibrator.main_menu()
        
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
