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


class ServoCalibrator:
    def __init__(self, port: str = "COM9", baudrate: int = 9600):
        self.driver = BusServoDriver(port=port, baudrate=baudrate, connect_delay=0.2)
        self.params = robot_params()
        self.port = port
        self.current_positions = {1: 1000, 2: 1000, 3: 1000}  # 从当前位置 1000 开始
        # 舵机的位置限制
        self.servo_limits = {
            1: (500, 1000),
            2: (500, 920),   # 舵机 2 的上限是 920
            3: (500, 1000)
        }
        
    def angle_to_position(self, angle_deg: float) -> int:
        """
        将角度转换为舵机位置值
        0° → 0, 240° → 1000
        """
        position = int((angle_deg / 240.0) * 1000)
        return max(0, min(1000, position))
    
    def position_to_angle(self, position: int) -> float:
        """
        将舵机位置值转换为角度
        """
        return (position / 1000.0) * 240.0
    
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
        """
        安全地发送舵机位置指令
        """
        if not self.driver.ser or not self.driver.ser.is_open:
            print("错误: 串口未打开")
            return False
        
        # 根据舵机 ID 应用特定的范围限制
        min_pos, max_pos = self.servo_limits[servo_id]
        position = max(min_pos, min(max_pos, position))
        
        try:
            self.driver.set_servo_positions([(servo_id, position)], time_ms)
            self.current_positions[servo_id] = position
            angle = self.position_to_angle(position)
            print(f"  舵机 {servo_id}: 位置 {position} (角度 {angle:.1f}°)")
            time.sleep(time_ms / 1000.0 + 0.1)  # 等待舵机到位
            return True
        except Exception as e:
            print(f"✗ 发送命令失败: {e}")
            return False
    
    def test_single_servo(self, servo_id: int):
        """
        逐个测试舵机的增量移动和方向验证
        """
        print(f"\n{'='*60}")
        print(f"舵机 {servo_id} 调试模式")
        print(f"{'='*60}")
        min_pos, max_pos = self.servo_limits[servo_id]
        print("说明:")
        print(f"  - 范围: {min_pos}-{max_pos}")
        print("  - 当前从最大值开始，仅做小增量移动")
        print("  - 角度映射: 0° 对应位置 0，240° 对应位置 1000")
        print("\n命令:")
        print("  +: 增加 10 位置")
        print("  -: 减少 10 位置")
        print("  ++: 增加 50 位置（谨慎使用）")
        print("  --: 减少 50 位置（谨慎使用）")
        print("  c: 自定义位置（谨慎使用）")
        print("  q: 回到主菜单")
        print("  x: 退出程序")
        
        while True:
            try:
                pos = self.current_positions[servo_id]
                angle = self.position_to_angle(pos)
                print(f"\n舵机 {servo_id} | 当前: 位置={pos}, 角度={angle:.1f}°")
                cmd = input("输入命令 > ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'x':
                    return False
                elif cmd == '+':
                    new_pos = min(max_pos, pos + 10)
                    if new_pos == pos:
                        print(f"  已到达最大值 {max_pos}")
                    else:
                        self.send_position(servo_id, new_pos, 300)
                elif cmd == '-':
                    new_pos = max(min_pos, pos - 10)
                    if new_pos == pos:
                        print(f"  已到达最小值 {min_pos}")
                    else:
                        self.send_position(servo_id, new_pos, 300)
                elif cmd == '++':
                    print("  ⚠️  大幅增加 50 位置，请确保安全！")
                    if input("  继续? (y/n): ").lower() == 'y':
                        new_pos = min(max_pos, pos + 50)
                        self.send_position(servo_id, new_pos, 500)
                elif cmd == '--':
                    print("  ⚠️  大幅减少 50 位置，请确保安全！")
                    if input("  继续? (y/n): ").lower() == 'y':
                        new_pos = max(min_pos, pos - 50)
                        self.send_position(servo_id, new_pos, 500)
                elif cmd == 'c':
                    try:
                        custom_pos = int(input(f"输入位置 ({min_pos}-{max_pos}): "))
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
            print(f"Delta 机械臂 - LX-225 舵机调试工具 [实机测试版]")
            print(f"{'='*60}")
            print("选项:")
            print("  1: 调试舵机 1 (范围 500-1000)")
            print("  2: 调试舵机 2 (范围 500-920)")
            print("  3: 调试舵机 3 (范围 500-1000)")
            print("  s: 显示当前舵机状态")
            print("  d: 断开连接")
            print("  x: 退出")
            
            cmd = input("\n输入选项 > ").strip().lower()
            
            if cmd == '1':
                if not self.test_single_servo(1):
                    break
            elif cmd == '2':
                if not self.test_single_servo(2):
                    break
            elif cmd == '3':
                if not self.test_single_servo(3):
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
        print(f"\n{'舵机ID':<10} {'位置':<10} {'角度':<10}")
        print("-" * 30)
        for servo_id in [1, 2, 3]:
            pos = self.current_positions[servo_id]
            angle = self.position_to_angle(pos)
            print(f"{servo_id:<10} {pos:<10} {angle:.1f}°")
    
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
        
        calibrator = ServoCalibrator(port=port)
        
        if not calibrator.connect():
            return
        
        print("\n✓ 已连接，舵机当前位置应为 1000")
        print("  开始小增量测试，请谨慎操作避免机械结构损伤")
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
