# LX225 Tool Demo

`lx225_tool_demo` 是一个给 `LX225` 总线舵机做安全读取、基础设置和本地映射换算的小工具。

它当前的主目标很明确：

- 先稳定读出当前单舵机的 `ID` 和 `raw` 位置
- 再把这个 `raw` 位置定义成你自己的坐标系锚点
- 整个 GUI 不发送任何运动命令

## 先看这里：怎么用

### 1. 当前默认连接

按现在这台机器上已经验证过的情况：

- `COM4` 是 `IMU`
- `COM19` 是舵机驱动板
- 波特率是 `9600`

对应配置文件：

- [config/lx225_tool.demo.toml](C:/Users/hanjuncheng/Desktop/78arm/lx225_tool_demo/config/lx225_tool.demo.toml)

### 2. 启动 GUI

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\lx225_tool_demo
python -m lx225_tool gui
```

### 3. GUI 推荐操作顺序

打开后按这个顺序用：

1. 点 `扫描单舵机 raw`
   适合“驱动板上当前只接了一个舵机”的场景。
   如果扫描成功，界面会自动回填当前硬件 `ID` 和 `raw`。

2. 点 `读取当前 raw`
   这一步会刷新当前舵机的实际 `raw` 位置。
   如果标准读链路失败，程序会尽力走已经验证过的 `simple 0x15` 回退读法。

3. 去中间的 `自定义坐标映射`
   先点 `用当前 raw`，把当前位置复制成锚点。

4. 选择锚点角色
   你可以把这个位置定义成：
   - `锚点设为新最小端`
   - `锚点设为新中点`
   - `锚点设为新最大端`

5. 用换算按钮核对
   - `坐标 -> raw`
   - `raw -> 坐标`

这一步只做本地换算，不写串口，不会让舵机突然跳动。

## 这版 GUI 现在到底做什么

主界面现在已经收口，重点放在这几件事：

- 读取单舵机 `raw`
- 区分“硬件目标 ID”和“映射模板”
- 让你把“当前位置 = 你的新坐标值”这件事快速做出来

主界面没有放进主流程的内容：

- 运动控制
- 位置运行命令
- 未验证的复杂 EEPROM 流程

也就是说，当前蓝黑 GUI 的中心思想不是“驱动舵机运动”，而是“先把读数和映射做对”。

## 界面里几个关键概念

### `当前舵机 ID`

这是串口真正要读写的硬件目标。

### `映射模板`

这是本地换算模板，只影响：

- 当前 `raw` 在模板坐标系里的值
- `坐标 <-> raw` 的换算结果
- 配置片段生成

它不会给舵机发位置命令。

### `raw`

这里的 `raw` 是底层位置值。

当前已知官方上位机常见显示是：

- `0 .. 1000` 对应 `0 .. 240°`

但这只是官方显示映射，不是你必须采用的逻辑。
现在这套工具允许你自己定义：

- `raw_min`
- `raw_max`
- `coord@raw_min`
- `coord@raw_max`
- `position_step`

例如你完全可以定义：

- “当前位置对应 1000”
- “总跨度 2000”
- “每 5 个 raw 量化一步”

## 配置文件怎么改

示例配置已经带注释，重点看两块：

### 串口配置

```toml
[serial]
port = "COM19"
baudrate = 9600
timeout = 0.60
connect_delay = 1.00
```

### 映射配置

```toml
[landing_gear]
servo_ids = [4, 5, 6]
down_raw = 500
up_raw = 1000
move_time_ms = 180

[servos.servo4]
id = 4
position_step = 5
raw_min = 0
raw_max = 1000
mapped_angle_at_raw_min = 0.0
mapped_angle_at_raw_max = 1000.0
```

当前总线职责约定：`1/2/3` 是真实控制机械臂的三个主舵机，`4/5/6` 是简单二值起落架，默认 `500/1000` 后续再校准。GUI 会自动把配置里的舵机纳入：

- 舵机快捷读取
- 读取配置内多舵机 raw
- 多舵机实时读数

注意：

- `id` 是硬件舵机 ID
- `position_step` 是你想采用的量化步长
- `raw_min/raw_max` 是你的映射参考区间，不一定等于舵机内部硬限位
- `mapped_angle_*` 虽然字段名还叫 `angle`，但你完全可以把它当成“你自己的坐标值”

## 命令行

如果只想看帮助：

```powershell
python -m lx225_tool --help
```

如果只想读当前舵机：

```powershell
python -m lx225_tool read --servo servo1
```

如果只想扫单舵机：

```powershell
python -m lx225_tool discover-id
```

## 安全边界

这套工具的安全边界是：

- 允许读取
- 允许做 `ID / OFFSET / LIMIT` 这类设置
- 不允许发送运动命令

当前 GUI 主流程已经按这个边界收口；即使你在中间区域改映射，也只是本地换算，不会触发舵机运动。
