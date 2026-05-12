# 8BitDo Bluetooth Gamepad Minimal Pack

这是给 `8BitDo Ultimate 2 Wireless` 蓝牙手柄准备的最小包，面向 Ubuntu
18.04。它不依赖 pygame，直接读 Linux `/dev/input/event*`。

目录结构：

- `src/`：功能代码，包含 evdev 读入、状态查看、一次性日志测试、旧控制器蓝牙启动器
- `config/`：映射配置和蓝牙 MAC 配置
- `logs/`：日志输出，测试脚本默认覆盖写
- `deploy/`：部署和启动脚本
- `docs/`：使用说明

先从这里开始：

```bash
cd Delta_Gcode_Servo/bt_8bitdo_min
bash deploy/install_ubuntu18.sh
bash deploy/run_log_test.sh 30
```

如果要用这个蓝牙手柄跑旧的实机控制器：

```bash
bash deploy/run_control_bt.sh
```

日志测试会覆盖写：

- `logs/gamepad_once.log`
- `logs/gamepad_once.json`

完整流程见 `docs/USAGE.md`。
