# 使用说明

## 目录结构

- `src/`：功能代码，包含 evdev 读手柄、实时状态显示、一次性日志测试、旧控制器蓝牙启动器
- `config/`：配置，包括手柄映射和蓝牙 MAC
- `logs/`：日志，测试脚本默认覆盖写
- `deploy/`：部署和启动脚本
- `docs/`：说明文档

## 1. 配对手柄

先编辑 `config/bluetooth_mac.conf`，填入手柄蓝牙 MAC：

```bash
GAMEPAD_MAC=AA:BB:CC:DD:EE:FF
```

然后运行：

```bash
bash deploy/install_ubuntu18.sh
```

如果暂时不知道 MAC，可以手动用 `bluetoothctl`：

```bash
bluetoothctl
power on
agent on
default-agent
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

## 2. 查看实时输入

```bash
python3 src/show_control_state.py
```

它会显示归一化后的摇杆、扳机和动作按钮状态。

## 3. 一次性覆写日志测试

```bash
bash deploy/run_log_test.sh 30
```

这个脚本每次都会覆盖写：

- `logs/gamepad_once.log`
- `logs/gamepad_once.json`

测试时把两个摇杆都推到最大行程，LT/RT 按到底，每个按键都按一次。日志会记录：

- 每个轴的观测最小值和最大值
- LT/RT 的实际模拟量范围
- 每个键的实际 Linux code、按下次数、释放次数
- 原始事件流

## 4. 跑旧实机控制器

这个最小包没有改旧的大文件，而是用一个启动器替换旧控制器里的 `GamepadReader`：

```bash
bash deploy/run_control_bt.sh
```

它会继续使用 `real_machine_test/gamepad_controller.py` 的机械臂控制逻辑，但手柄输入改成当前 8BitDo 蓝牙 evdev 映射。

## 5. 当前映射

- 左摇杆：`ABS_X`, `ABS_Y`
- 右摇杆：`ABS_Z`, `ABS_RZ`
- LT/RT 模拟量：`ABS_BRAKE`, `ABS_GAS`
- 面按键：`BTN_SOUTH`, `BTN_EAST`, `BTN_WEST`, `BTN_NORTH`
- 肩键：`BTN_TL`, `BTN_TR`

旧控制器兼容动作：

- `A / BTN_SOUTH`：退出
- `B / BTN_EAST`：记录点
- `X / BTN_WEST`：切换 safe scan
- `Y / BTN_NORTH`：切换传感器坐标模式
- `LB / RB`：工具舵机反向/正向

## 6. 注意

- 这个包只按蓝牙模式做。
- 不要写死 `event8`；脚本会按名称、bus、vendor、product 自动找设备。
- 如果没有权限读 `/dev/input/event*`，先运行部署脚本，然后注销再登录。
- 日志测试就是覆盖写设计，不会追加旧日志。
