#!/usr/bin/env python3
"""
quick_start.py - 快速启动实机测试工具
"""

import sys
import subprocess
from pathlib import Path

# 添加上级目录到 Python 路径
sys.path.insert(0, str(Path(__file__).parent.parent))

if __name__ == "__main__":
    print("\n" + "="*70)
    print("Delta 机械臂实机测试 - 快速启动")
    print("="*70 + "\n")
    print(f"Python: {sys.executable}\n")
    
    tools = {
        "1": ("servo_calibration", "舵机校准工具 (单舵机小增量测试)"),
        "2": ("gamepad_controller", "手柄控制工具 (实时Xbox手柄控制)"),
    }
    
    for key, (module, desc) in tools.items():
        print(f"  {key}: {desc}")
    print("  q: 退出\n")
    
    choice = input("选择工具 (1/2/q): ").strip().lower()
    
    if choice in tools:
        module, _ = tools[choice]
        try:
            if module == "servo_calibration":
                from servo_calibration import main
                main()
            else:  # gamepad_controller
                project_root = Path(__file__).resolve().parents[2]
                venv_python = project_root / ".venv" / "Scripts" / "python.exe"
                python_exe = str(venv_python if venv_python.exists() else Path(sys.executable))
                subprocess.run(
                    [
                        python_exe,
                        str(Path(__file__).with_name("gamepad_controller.py")),
                        "--port",
                        "COM19",
                        "--start-from-current",
                        "--slow-start",
                    ]
                )
        except ImportError as e:
            print(f"\n❌ 导入失败: {e}")
            print("请确保在 real_machine_test 目录下运行此脚本")
        except Exception as e:
            print(f"\n❌ 出错: {e}")
    elif choice != 'q':
        print("\n❌ 无效选择")
