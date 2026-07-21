#!/usr/bin/env python3
"""
Delta 机械臂实机测试工具启动器
"""

import sys
import subprocess
from pathlib import Path


def safe_gamepad_command(current_dir: Path) -> list[str]:
    project_root = current_dir.parents[1]
    venv_python = project_root / ".venv" / "Scripts" / "python.exe"
    python_exe = str(venv_python if venv_python.exists() else Path(sys.executable))
    return [
        python_exe,
        str(current_dir / "gamepad_controller.py"),
        "--port",
        "COM19",
        "--start-from-current",
        "--slow-start",
    ]


def main():
    print("\n" + "="*70)
    print("Delta 机械臂 - 实机测试工具包")
    print("="*70)
    print(f"Python: {sys.executable}")
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
        print("Using the project .venv and safe startup flags.\n")
        subprocess.run(safe_gamepad_command(current_dir))
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
