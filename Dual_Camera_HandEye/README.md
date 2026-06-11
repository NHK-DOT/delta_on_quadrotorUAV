# Dual Camera Hand-Eye / Visual Follow Demo

这个目录现在只做一件事：把 78arm 已有的相机、IMU、FK/IK 和实机控制入口之间的
坐标关系讲清楚，并提供离线可跑的小 demo。它不打开舵机串口，不直接控制实机。

## 明确答复

你现在要做的不是传统工业机器人那种“拿探针、千分表、笔尖去触碰一个固定锚点”的
手眼标定。你的 12V 平面电磁铁不会提供一个稳定尖点，所以不适合作为传统 TCP
接触锚定工具。

你这套系统更接近“大疆云台/火控雷达的目标跟随”：

1. 相机识别到目标。
2. 目标在画面里偏离期望位置。
3. 控制器把画面误差转换成末端小步位移。
4. 末端跟着目标移动，让目标回到画面中心或预设抓取区域。
5. 目标进入容差后，再执行下降/吸附/抬升。

所以这里推荐先做 **image-follow 图像跟随闭环**，再做完整三维抓取外参。

## 当前硬件策略

- 底座相机 `base_camera`：看末端执行器上表面的 AprilTag，用来估计/核验实际
  `base_T_tool`。
- 末端上表面：AprilTag + IMU，二者和末端工具坐标系刚性固定。
- 末端下表面：12V 平面电磁铁，是最终吸附执行器。
- 侧面相机 `object_camera`：安装在执行机构旁边，斜向下看电磁铁下方附近区域。
  它可以照不到电磁铁本体；只要它能稳定看到待抓取物体，就可以先做图像跟随。

这个相机位置后续还可以改。当前建议是：

- 先保证目标能在工作高度附近长期出现在画面里；
- 把期望位置设成画面中心或略偏向电磁铁投影落点的位置；
- 先锁 Z，只做 XY 小步跟随；
- 等 XY 跟随稳定后，再加入下降、吸附、抬升。

## 你现在已经有的条件

- `Delta_Gcode_Servo/real_machine_test/gamepad_controller.py`
  - 已有 FK/IK。
  - 已有手柄实时控制。
  - 已有 `B` 记录点，`BACK` 切换 `LINE` / `PICK_PLACE`，`START` 二次确认自动运动。
  - 已经读取 `IMU/wt61c_latest.json` 和
    `AprilTag_Vision/myAprilTag/output/apriltag_latest.json`。
- `AprilTag_Vision/myAprilTag`
  - 已有相机打开、AprilTag 检测、位姿估计、JSON 快照输出。
- `IMU`
  - 已有 WT61C 快照输出。
- `part_model_rev/999.STL`
  - 已经作为 IMU + 上表面 AprilTag 固定件纳入仓库。
- `Dual_Camera_HandEye`
  - 已有底座相机外参估计。
  - 已有侧面相机固定外参 `tool_T_object_camera` 的数据接口。
  - 已有把物体检测投到 `base` 坐标系的离线命令。

还没完成的是：**把视觉误差自动写进实机控制循环**。目前这个目录只输出下一步目标，
没有直接驱动舵机。

## 两条跟随路线

### 1. image-follow：先做画面跟随

这是当前最适合你的路线。它不要求侧面相机看到电磁铁，也不要求你马上完成完整
三维手眼标定。

输入：

- 当前 `base_T_tool`
- 侧面相机安装外参 `tool_T_object_camera`
- 目标在画面中的 `normalized_xy` 或 `center_px`

输出：

- 图像误差
- 建议的末端小步位移
- 下一步 `next_base_T_tool`
- 可选 Delta IK 可达性检查

命令：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm
python Dual_Camera_HandEye\demo.py plan-image-follow-step `
  --calibration Dual_Camera_HandEye\output\calibration_result.json `
  --base-tool-rpy 0.0 0.0 -0.28 0.0 0.0 0.0 `
  --object-snapshot AprilTag_Vision\myAprilTag\output\apriltag_latest.json `
  --object-id 5 `
  --gain-mm-per-norm 15 `
  --max-step-mm 3 `
  --tolerance-norm 0.04 `
  --output Dual_Camera_HandEye\output\image_follow_step.json
```

注意：这里的 `base-tool-rpy` 示例仍是 hand-eye demo 的视觉坐标，不等于实机控制器
的最终模型坐标。接入实机前要统一 `base` 坐标和 `Delta_Gcode_Servo` 的 XYZ 符号。

### 2. metric-follow：后续做精准抓取

这条路线用完整外参计算：

```text
base_T_object = base_T_tool * tool_T_object_camera * object_camera_T_object
```

再让电磁铁中心 `tool_T_pickup` 对齐目标：

```powershell
python Dual_Camera_HandEye\demo.py plan-follow-step `
  --calibration Dual_Camera_HandEye\output\calibration_result.json `
  --base-tool-rpy 0.0 0.0 -0.28 0.0 0.0 0.0 `
  --object-snapshot AprilTag_Vision\myAprilTag\output\apriltag_latest.json `
  --object-id 5 `
  --pickup-offset-mm 0 0 -35 `
  --track-axes xy `
  --max-step-mm 5 `
  --output Dual_Camera_HandEye\output\follow_step.json
```

这条路线更适合真正“抓取点对齐”，但前提是侧面相机的物体检测能给出稳定的
尺度/深度/位姿，或者你能用固定高度假设把像素坐标落到桌面平面。

## 底座相机核验末端位置

底座相机看到上表面 AprilTag 后，可以反推当前末端：

```text
base_T_tool_from_camera =
    base_T_base_camera * base_camera_T_hand_tag * inverse(tool_T_hand_tag)
```

命令：

```powershell
python Dual_Camera_HandEye\demo.py estimate-tool `
  --calibration Dual_Camera_HandEye\output\calibration_result.json `
  --base-camera-snapshot AprilTag_Vision\myAprilTag\output\apriltag_latest.json `
  --hand-tag-id 5 `
  --output Dual_Camera_HandEye\output\base_tool_from_camera.json
```

真实使用时，底座相机和侧面相机建议写到不同快照文件，避免两个相机抢同一个
`apriltag_latest.json`。

## 下一步怎么干

1. 先固定侧面相机，让目标在电磁铁工作高度附近能稳定进入画面。
2. 用你现有识别逻辑输出和 `apriltag_latest.json` 类似的快照，至少包含：
   - `timestamp_unix`
   - `detections[0].center_px` 或 `detections[0].normalized_xy`
   - 可选 `detections[0].position_m`
3. 跑 `plan-image-follow-step`，看 `command_step_base_mm` 方向是否符合直觉。
4. 只在桌面/固定底座上做 XY 小步闭环，不下降、不吸电磁铁。
5. 确认“目标往右，末端也往正确方向追”后，再把这段输出接进
   `gamepad_controller.py` 的受限自动模式。
6. 最后加状态机：

```text
巡航/UWB 到大致区域
  -> 识别目标
  -> 盘旋窗口内 image-follow 对中
  -> 低速下降
  -> 电磁铁上电吸附
  -> 抬升
  -> 飞走
```

## 必须补齐的缺口

- 视觉 `base` 坐标和 `Delta_Gcode_Servo` 实机 XYZ 的单位/符号映射。
- 侧面相机和电磁铁中心的相对偏移 `tool_T_pickup`。
- 目标检测快照格式。你说第二个相机识别逻辑在别处，后面换新机子时再适配；这里
  只要求它最终输出同样的 `center_px` / `normalized_xy`。
- 电磁铁控制的电源/继电器/MOS 管接口目前不在这个 demo 里。
