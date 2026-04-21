#!/usr/bin/env python3
"""
Delta 机械臂实机测试工具启动器
"""

import sys
import subprocess
from pathlib import Path


def main():
    print("\n" + "="*70)
    print("Delta 机械臂 - 实机测试工具包")
    print("="*70)
    print("\n请选择要运行的工具:\n")
    print("  1: 舵机校准工具 (单个舵机+/-增量测试)")
    print("  2: 手柄控制工具 (实时Xbox手柄控制)")
    print("  q: 退出")
    
    choice = input("\n选择 (1/2/q): ").strip().lower()
    
    current_dir = Path(__file__).parent
    
    if choice == '1':
        print("\n启动舵机校准工具...\n")
        subprocess.run([sys.executable, str(current_dir / "servo_calibration.py")])
    elif choice == '2':
        print("\n启动手柄控制工具...")
        print("⚠️  请确保 pygame 已安装: pip install pygame\n")
        subprocess.run([sys.executable, str(current_dir / "gamepad_controller.py")])
    elif choice == 'q':
        print("\n退出")
    else:
        print("\n❌ 无效选择")
        main()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as e:
        print(f"\n❌ 出错: {e}")
        import traceback
        traceback.print_exc()
