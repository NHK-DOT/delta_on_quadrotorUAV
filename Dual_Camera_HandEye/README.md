# Dual Camera Hand-Eye Calibration Demo

这个目录给当前 78arm 的双相机方案做一个可落地的手眼标定 demo：

- **底座相机 base camera**：固定在机械臂底座/机架上，看末端执行器上的 AprilTag，用来估计“手在哪里”。
- **末端相机 wrist camera**：固定在末端执行器上，看底座/工作台上的 AprilTag，也用于识别抓取物和确认实际抓取情况。

核心目标不是先写复杂控制，而是先把所有视觉结果统一到机械臂基座坐标系 `base`，这样后面抓取点、AprilTag、末端位姿、相机测量都能在同一个坐标系里做判断。

## 坐标系

本 demo 使用下面的坐标命名：

```text
base                 机械臂基座坐标系，机器人正运动学/FK 输出的参考系
tool                 末端执行器/法兰坐标系，机器人 FK 输出 base_T_tool
base_camera          固定在底座上的相机坐标系
wrist_camera         固定在末端执行器上的相机坐标系
hand_tag             固定在末端执行器上的 AprilTag
base_tag             固定在底座或工作台上的 AprilTag
```

变换名采用 `A_T_B`，意思是把 `B` 坐标系下的点变换到 `A` 坐标系。

## 推荐硬件布置

1. 在末端执行器上固定一个 AprilTag，形成稳定的 `tool_T_hand_tag`。
2. 在底座或工作台上固定一个 AprilTag，形成稳定的 `base_T_base_tag`。
3. 底座相机拍 `hand_tag`，输出 `base_camera_T_hand_tag`。
4. 末端相机拍 `base_tag`，输出 `wrist_camera_T_base_tag`。
5. 每次采样同步记录机器人 FK：`base_T_tool`。

如果你的 Delta 机械臂末端没有姿态自由度，只能做 XYZ 平移，那么经典手眼标定里的旋转观测是不充分的。这个 demo 因此默认使用“已知 AprilTag 刚性安装位置 + 多帧平均”的直接外参链，而不是强行依赖末端旋转运动。

## 标定链路

### 1. 底座相机外参

已知：

```text
base_T_tool                每个采样点的机器人末端位姿
tool_T_hand_tag            末端 AprilTag 相对末端工具坐标的安装位姿
base_camera_T_hand_tag     底座相机检测到的 AprilTag 位姿
```

每一帧都能得到：

```text
base_T_base_camera =
    base_T_tool * tool_T_hand_tag * inverse(base_camera_T_hand_tag)
```

多帧求平均后得到稳定的 `base_T_base_camera`。

### 2. 末端相机外参

已知：

```text
base_T_tool                每个采样点的机器人末端位姿
base_T_base_tag            底座 AprilTag 相对机械臂基座的安装位姿
wrist_camera_T_base_tag    末端相机检测到的底座 AprilTag 位姿
```

每一帧都能得到：

```text
tool_T_wrist_camera =
    inverse(base_T_tool) * base_T_base_tag * inverse(wrist_camera_T_base_tag)
```

多帧求平均后得到稳定的 `tool_T_wrist_camera`。

### 3. 抓取时怎么用

末端相机看到物体后，如果视觉输出 `wrist_camera_T_object`，物体在机械臂基座下的位置就是：

```text
base_T_object =
    base_T_tool * tool_T_wrist_camera * wrist_camera_T_object
```

底座相机看到末端 AprilTag 后，也可以反推当前末端：

```text
base_T_tool =
    base_T_base_camera * base_camera_T_hand_tag * inverse(tool_T_hand_tag)
```

这两条链可以互相验算：机器人 FK 算出来的 `base_T_tool` 和底座相机反推出来的 `base_T_tool` 如果差很多，说明相机、AprilTag、机械臂零点或时间同步有问题。

## 目录结构

```text
Dual_Camera_HandEye/
  README.md
  requirements.txt
  demo.py
  src/dual_handeye/
    __init__.py
    calibration.py
    cli.py
    geometry.py
    synthetic.py
```

## 安装依赖

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\Dual_Camera_HandEye
python -m pip install -r requirements.txt
```

`numpy` 是必需的。`opencv-contrib-python` 用于后续接 AprilTag/ArUco 检测，也提供可选的经典 `calibrateHandEye` 路线。

## 运行合成数据 demo

先生成一份带噪声的模拟采样数据：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\Dual_Camera_HandEye
python demo.py generate --output output\synthetic_samples.json --samples 24
```

再跑标定：

```powershell
python demo.py calibrate --samples output\synthetic_samples.json --output output\calibration_result.json
```

输出里会包含：

- `base_T_base_camera`：底座相机相对机械臂基座的外参
- `tool_T_wrist_camera`：末端相机相对末端工具坐标的外参
- 每条链的平移/旋转残差

## 真实机器采样格式

真实机器接入时，把采样保存成 JSON，结构如下：

```json
{
  "units": "m",
  "known_transforms": {
    "tool_T_hand_tag": {
      "translation": [0.0, 0.0, -0.035],
      "rotation_rpy_deg": [0.0, 0.0, 0.0]
    },
    "base_T_base_tag": {
      "translation": [0.12, 0.05, -0.02],
      "rotation_rpy_deg": [0.0, 0.0, 0.0]
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
      },
      "wrist_camera": {
        "camera_T_base_tag": {
          "translation": [0.04, 0.03, 0.32],
          "rotation_rpy_deg": [-5.0, 12.0, 1.0]
        }
      }
    }
  ]
}
```

### 数据来源

- `base_T_tool`：来自机械臂 FK，单位建议统一为米。
- `camera_T_hand_tag` / `camera_T_base_tag`：来自 AprilTag 位姿估计，必须使用已经标定过内参的相机。
- `tool_T_hand_tag`：末端 AprilTag 的安装尺寸，建议用 CAD 或卡尺测量。
- `base_T_base_tag`：底座 AprilTag 的安装尺寸，建议把 tag 贴在一个可测量的基准板上。

## 采样建议

- 采 20 到 40 个点，覆盖工作空间：左/右/前/后/高/低都要有。
- 每个点停稳 0.2 到 0.5 秒后再采样，避免运动模糊和时间不同步。
- AprilTag 尺寸、相机内参、图像分辨率必须和检测脚本一致。
- 两个 tag 必须刚性固定，不能用手临时扶。
- 如果末端没有旋转自由度，不要期待经典手眼算法能自动解出所有旋转量，优先使用本 demo 的直接链路。

## 和现有 AprilTag 工具对接

现有 `AprilTag_Vision/myAprilTag/src/apriltag_usb_detector.py` 已经能做相机标定、AprilTag 检测和 JSON 输出。后续可以写一个采样脚本，把它的检测结果和机器人当前 `base_T_tool` 合并成这里的 `samples.json`。

当前 demo 暂时不直接打开相机、不发机械臂运动命令，目的是先把数学链路和数据接口固定下来。等外参结果稳定后，再把 `base_T_object = base_T_tool * tool_T_wrist_camera * wrist_camera_T_object` 接到抓取规划里。
