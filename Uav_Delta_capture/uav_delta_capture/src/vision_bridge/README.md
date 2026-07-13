# vision_bridge - 视觉桥接包

视觉检测结果的坐标变换与桥接。

## 功能

- 视觉目标坐标系变换（相机 → 机械臂）
- TF2 坐标变换管理
- Jetson 视觉处理桥接
- 摄像头感知节点

## 节点

### vision_transform_node

视觉坐标变换节点。

**订阅：**
- `vision/target_offset`（`geometry_msgs/PointStamped`）- 相机坐标系目标

**发布：**
- `target_point`（`geometry_msgs/PointStamped`）- 机械臂坐标系目标

**TF：**
- `camera_optical_frame` → `delta_base_link`

**参数：**
- `target_topic_sub`（string）- 输入话题，默认 `vision/target_offset`
- `target_topic_pub`（string）- 输出话题，默认 `target_point`
- `target_frame`（string）- 目标坐标系，默认 `delta_base_link`
- `source_frame`（string）- 源坐标系，默认 `camera_optical_frame`
- `transform_timeout`（double）- TF 超时，默认 `0.1` s

### perception_node

摄像头感知节点（Python）。

**发布：**
- `vision/target_offset`（`geometry_msgs/PointStamped`）- 检测到的目标偏移

### jetson_bridge_node

Jetson 视觉处理桥接节点。

**发布：**
- `vision/detections` - 检测结果
- `vision/target_offset`（`geometry_msgs/PointStamped`）- 目标偏移

## 使用

```bash
# 启动坐标变换
ros2 launch vision_bridge vision_transform.launch.py

# 启动感知节点
ros2 run vision_bridge perception_node.py

# 启动 Jetson 桥接
ros2 run vision_bridge jetson_bridge_node
```

## 配置

配置文件：`config/vision_transform.yaml`

## TF 树

```
map
 └── base_link
      └── delta_base_link
           └── camera_optical_frame
```
