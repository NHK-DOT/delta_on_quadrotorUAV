# Dual Camera Hand-Eye Demo

这个目录给当前 78arm 的双相机手眼协同方案提供一个可验证 demo。它只复用现有
`AprilTag_Vision/myAprilTag`、`IMU` 和实机控制代码的输出文件，不另写相机检测包，
也不发送机械臂运动命令。

## 当前硬件布局

- **底座相机 `base_camera`**：固定在底座/机架上，观察末端执行器上表面的
  AprilTag。它用于反推末端当前位置，并核验机器人 FK 的 `base_T_tool`。
- **末端上表面 AprilTag `hand_tag`**：固定在新机械件上，和末端工具坐标系形成
  刚性关系 `tool_T_hand_tag`。
- **末端下表面抓取机构**：和末端工具坐标系同体安装，后续抓取点最终要落回
  `base` 坐标系。
- **侧面物体相机 `object_camera`**：安装在执行机构侧面，观察待抓取物体。它不再
  假设必须看底座/工作台 AprilTag；它的安装外参来自 CAD/卡尺/装配测量：
  `tool_T_object_camera`。
- **`part_model_rev/999.STL`**：新加入的 IMU + AprilTag 固定件模型，用来约束
  `tool_T_hand_tag`、IMU 和末端工具之间的机械关系。

核心目标是把所有视觉结果统一到机械臂基座坐标系 `base`：

```text
base_T_object = base_T_tool * tool_T_object_camera * object_camera_T_object
```

底座相机同时提供一条独立核验链：

```text
base_T_tool_from_camera =
    base_T_base_camera * base_camera_T_hand_tag * inverse(tool_T_hand_tag)
```

如果 `base_T_tool_from_camera` 和机器人 FK 的 `base_T_tool` 差很多，优先检查
AprilTag 尺寸、相机内参、装配尺寸、末端零点和时间同步。

## 坐标系命名

```text
base                 机械臂基座坐标系
tool                 末端执行器/法兰坐标系，机器人 FK 输出 base_T_tool
base_camera          底座相机坐标系
object_camera        执行机构侧面物体识别相机坐标系
hand_tag             末端上表面 AprilTag 坐标系
object               待抓取物体坐标系或物体检测输出的目标点坐标系
```

变换名采用 `A_T_B`：把 `B` 坐标系下的点转换到 `A` 坐标系。

## 依赖

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\Dual_Camera_HandEye
python -m pip install -r requirements.txt
```

`numpy` 是数学链路必需依赖。`opencv-contrib-python` 只用于兼容旧版
`--wrist-method handeye` 数据，不是当前推荐路线。

## 运行合成数据 demo

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm
python Dual_Camera_HandEye\demo.py generate --output Dual_Camera_HandEye\output\synthetic_samples.json --samples 24
python Dual_Camera_HandEye\demo.py calibrate --samples Dual_Camera_HandEye\output\synthetic_samples.json --output Dual_Camera_HandEye\output\calibration_result.json
```

输出包含：

- `base_T_base_camera`：底座相机相对机械臂基座的外参，由底座相机看末端
  AprilTag 的多帧样本估计。
- `tool_T_object_camera`：侧面物体相机相对末端工具坐标的外参，当前推荐从
  CAD/卡尺/装配测量写入 `known_transforms`。
- 残差统计：用于判断底座相机链路是否稳定。

## 真实采样数据格式

真实机器采样时，保存 JSON：

```json
{
  "units": "m",
  "known_transforms": {
    "tool_T_hand_tag": {
      "translation": [0.0, 0.0, -0.035],
      "rotation_rpy_deg": [0.0, 0.0, 0.0]
    },
    "tool_T_object_camera": {
      "translation": [0.045, -0.012, 0.032],
      "rotation_rpy_deg": [0.0, -58.0, 3.0]
    }
  },
  "samples": [
    {
      "name": "p001",
      "base_T_tool": {
        "translation": [0.0, 0.0, -0.28],
        "rotation_rpy_deg": [0.0, 0.0, 0.0]
      },
      "base_camera": {
        "camera_T_hand_tag": {
          "translation": [0.1, -0.02, 0.45],
          "rotation_rpy_deg": [10.0, 0.0, 2.0]
        }
      }
    }
  ]
}
```

说明：

- `base_T_tool` 来自当前机械臂 FK，单位统一为米。
- `base_camera.camera_T_hand_tag` 来自现有
  `AprilTag_Vision/myAprilTag/src/apriltag_usb_detector.py` 的 AprilTag 位姿估计。
- `tool_T_hand_tag` 和 `tool_T_object_camera` 建议从 `999.STL` 对应装配尺寸、
  CAD 或卡尺测量得到。
- 侧面相机看物体，不需要在每个标定点看底座 tag。

## 复用现有 AprilTag 输出

现有检测程序会写：

```text
AprilTag_Vision/myAprilTag/output/apriltag_latest.json
```

把某个检测结果转换成 `camera_T_target`：

```powershell
python Dual_Camera_HandEye\demo.py snapshot-transform `
  --snapshot AprilTag_Vision\myAprilTag\output\apriltag_latest.json `
  --tag-id 5 `
  --transform-name base_camera_T_hand_tag `
  --output Dual_Camera_HandEye\output\snapshot_transform.json
```

侧面相机识别物体后，结合当前 `base_T_tool` 和标定结果投影到机械臂基座：

```powershell
python Dual_Camera_HandEye\demo.py project-object `
  --calibration Dual_Camera_HandEye\output\calibration_result.json `
  --base-tool-rpy 0.0 0.0 -0.28 0.0 0.0 0.0 `
  --object-snapshot AprilTag_Vision\myAprilTag\output\apriltag_latest.json `
  --object-id 5 `
  --output Dual_Camera_HandEye\output\object_in_base.json
```

如果已有 `base_T_tool` JSON，也可以用 `--base-tool path\to\base_T_tool.json`。

## 和现有代码的边界

- `AprilTag_Vision/myAprilTag` 负责相机打开、像素格式、AprilTag 检测、JSON 快照。
- `IMU/wt61c_latest.json` 继续作为姿态参考快照；本 demo 不直接读串口。
- `Delta_Gcode_Servo/real_machine_test/gamepad_controller.py` 已经读取 IMU 和
  AprilTag 快照；本 demo 只定义坐标链路和离线核验方法。
- 本目录不打开舵机串口、不执行运动、不替代现有实机控制入口。

## 采样建议

- 底座相机外参建议采 20 到 40 个点，覆盖工作空间的左/右/前/后/高/低。
- 每个点静止 0.2 到 0.5 秒后再采样，避免运动模糊和时间不同步。
- AprilTag 尺寸、相机内参、分辨率必须和检测脚本一致。
- 末端上表面 AprilTag、IMU、侧面相机和抓取机构必须刚性固定。
- Delta 末端姿态自由度不足时，不要依赖经典手眼算法自动求完整旋转外参；
  当前路线是“底座相机多帧估计 + 侧面相机安装外参”。
