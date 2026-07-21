# 2026-07-08 机械臂重标定状态

当前结论：

- `1/2/3` 是机械臂三轴，`4/5/6` 仅作为起落架。
- 旧的手柄/IK 主线暂时不能恢复使用，因为机械结构已改，旧模型与真实机械臂不一致。
- 2026-06-26 那批 Jetson 3K AprilTag 拟合结果残差过大，`fit_report.json` 明确不适合直接用于控制。
- 2026-07-08 已经采到了新的 raw-only 标签包络，但还缺对应的 Jetson 3K `tool XYZ` 数据集。

## 今日已有原始结论

来自 `calibration_snapshots/20260708_135833_labeled_workspace_raw_summary.json`：

- `top_home` 中位 raw: `1=459 2=48 3=1021`
- `bottom_safe` 中位 raw: `1=-14 2=1120 3=622`
- `center_mid` 中位 raw: `1=370 2=-46 3=976`
- `left_mid` 中位 raw: `1=306 2=1116 3=803`
- `right_mid` 中位 raw: `1=33 2=-97 3=672`
- `front_mid` 中位 raw: `1=80 2=1165 3=953`
- `back_mid_redo` 中位 raw: `1=235 2=-193 3=652`

建议的 raw 安全包络：

- servo1: `[-34, 479]`
- servo2: `[-213, 1185]`
- servo3: `[602, 1041]`

说明：

- 这只是 raw 包络，不是新的 IK/FK。
- `bottom_safe` 的 servo3 组内抖动较大，后续最好补采。

## 新的 Jetson 只读采样入口

已新增：

- `jetson_py36/jetson_structure_calibration_sampler_py36.py`
- `jetson_py36/run_structure_calibration_sampler_py36_jetson.sh`

特性：

- 只读，不发任何运动命令。
- 直接在 Jetson 本机读取：
  - `/dev/ttyUSB0` 的 `1/2/3` raw
  - Jetson 3K 鱼眼 AprilTag JSON 的 `tool_position_mm`
- 一次采一个标签，适合人工把机械臂摆到位后再执行。

## 下一步推荐流程

1. 先只采 `1/2/3`，完全忽略起落架。
2. 用 Jetson 只读采样补齐以下标签的 `raw + XYZ`：
   - `top_home`
   - `bottom_safe`
   - `center_mid`
   - `left_mid`
   - `right_mid`
   - `front_mid`
   - `back_mid`
   - `left_front_mid`
   - `right_front_mid`
   - `left_back_mid`
   - `right_back_mid`
3. 每个标签至少采 `3` 次，确认重复性。
4. 采样后再用 `workspace_model_tools.py fit` 离线拟合。
5. 残差收敛后，才恢复 Jetson 手柄笛卡尔控制。

## 当前边界

- 在完成新结构拟合之前，不再启用旧的 IK/FK 手柄主线。
- 若只想验证总线和手柄连通性，也必须保持“控制程序不发运动”的边界，直到新模型通过拟合检查。
