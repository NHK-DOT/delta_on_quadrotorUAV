# NX–STM32MP257 有线局域网与 ROS 2 通信验证

**验证日期：2026 年 7 月 14 日。** 本文记录 Jetson Xavier NX 与
STM32MP257 之间的专用有线网络、257 上已有的 ROS 2 Docker 环境，以及
跨设备 ROS 2 通信验证结果。本文是集成与恢复记录，不构成飞行授权。

English version:
[`NX_257_WIRED_LAN_AND_ROS2.md`](NX_257_WIRED_LAN_AND_ROS2.md)

## 1. 网络职责划分

STM32MP257 同时使用两条彼此独立的有线网络：

```text
Jetson Xavier NX eth0  10.42.0.1/24
            | 专用网线
STM32MP257 end0        10.42.0.2/24

STM32MP257 USB 有线网卡（本次观察到的名称：enu1u2u4）
            | 外部路由器 / 公网
            `-- DHCP 获取 192.168.1.0/24 地址（本次为 192.168.1.41）
```

`end0` 是 STM32MP257 板载实体有线网口。它专用于 NX–MP257 通信；如需
将其接到外部路由器，必须另行调整配置。USB 有线网卡负责接入路由器和
公网。

这里使用的是独立三层 IP 子网，而不是 Linux 二层 bridge。这样可隔离
控制/数据流量，同时满足 ROS 2 DDS 节点发现和话题通信需求。

## 2. 固定地址配置

### NX

NX 上的 NetworkManager 连接名称为 `nx-257-direct`，绑定 `eth0`：

```text
eth0 = 10.42.0.1/24
```

NX 的 Wi-Fi 与该控制网独立；连接现场路由器时仍为
`192.168.1.174/24`。

NX 检查命令：

```bash
ip -4 -br addr show eth0
ip route get 10.42.0.2
ping -c 4 10.42.0.2
```

### STM32MP257

257 使用 OpenSTLinux 和 `systemd-networkd`。持久化本地覆盖配置为：

```text
/etc/systemd/network/10-nx-257-end0-static.network
```

```ini
[Match]
Name=end0

[Network]
Address=10.42.0.2/24
DHCP=no
IPv6AcceptRA=no
```

该文件只匹配 `end0`，不会修改 `wlan0`、USB 有线网卡或 `tailscale0`。
其中没有 `Gateway=` 和 `DNS=`，因此 NX 控制链路不会抢占 257 的公网
默认路由。

257 检查命令：

```bash
ip -4 -br addr
ip -4 route
networkctl status end0 --no-pager
ping -I end0 -c 4 10.42.0.1
```

## 3. 外网与内网隔离

连接路由器后，本次识别到的 USB 有线网卡为 `enu1u2u4`。它通过 DHCP
获得 `192.168.1.41/24`，并持有默认路由：

```text
default via 192.168.1.1 dev enu1u2u4
10.42.0.0/24 dev end0 src 10.42.0.2
192.168.1.0/24 dev enu1u2u4 src 192.168.1.41
```

不同 USB 网卡或不同 USB 端口可能产生不同接口名，不能在飞行软件中硬
编码 `enu1u2u4`。应通过 `ip -4 -br addr` 或默认路由动态识别外网接口。

2026 年 7 月 14 日验证结果：

| 链路 | 结果 |
| --- | --- |
| STM32MP257 `end0` → NX `10.42.0.1` | 正常，约 0.49 ms |
| STM32MP257 USB 有线网 → 路由器 `192.168.1.1` | 正常 |
| STM32MP257 USB 有线网 → `1.1.1.1` | 正常，约 3.2 ms |

## 4. 257 上已有的 ROS 2 Docker 环境

OpenSTLinux 宿主机没有原生 `/opt/ros`；257 上的 ROS 2 运行环境位于
Docker 内。

已确认的原有资源：

| 项目 | 内容 |
| --- | --- |
| 自定义镜像 | `my_ros2_humble:latest` |
| 架构 | `arm64` |
| 镜像内容 | ROS 2 Humble、MAVROS、MAVROS extras、GeographicLib 数据集、colcon 工具链 |
| 主容器 | `ros2humble` |
| 网络模式 | `host` |
| 工作区挂载 | `/usr/local/Uav_Delta_capture` → `/workspace` |
| 容器工作目录 | `/workspace/uav_delta_capture` |
| 板端启动入口 | `/usr/local/Uav_Delta_capture/start_ready.sh` |

挂载工作区已完成构建，包含 `delta_kinematics`、`uav_delta_msgs`、
`fcu_bridge`、`vision_test`、`safety`、`uwb_driver`、`uwb_navigation`、
`vision_bridge` 和 `uav_bridge` 等 ROS 2 包。

原有容器停止的直接原因是 Docker 被要求映射 `/dev/ttyACM0` 与
`/dev/ttyUSB0`，但验证时 257 上没有这两个设备。该问题是串口硬件设备
可用性问题，不是 ROS 2 或 NX–257 网络故障。启动飞行相关节点前先检查：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttySTM* 2>/dev/null
docker ps -a
```

在确认飞控和外设的实际设备名之前，不要删除或重建原有 `ros2humble`
容器，也不要盲目修改其设备映射。

## 5. 跨设备 ROS 2 通信测试

NX 原有 `ros2_humble` Humble 容器采用 host 网络。为验证专用网线上的
DDS 通信，在 257 上临时运行 Humble 容器，并采用：

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=78
ROS_LOCALHOST_ONLY=0
```

测试期间临时关闭 257 的 USB 外网接口；测试后立即恢复并确认 DHCP 地址
和默认路由恢复。这样 DDS 发现和数据只能经过 `10.42.0.1 ↔ 10.42.0.2`，
不会误走路由器的 `192.168.1.x` 网络。

| 方向 | 话题 | 收到的消息 |
| --- | --- | --- |
| NX → STM32MP257 | `/nx_to_257_direct` | `nx-to-257-over-10.42` |
| STM32MP257 → NX | `/board_to_nx_direct` | `257-to-nx-over-10.42` |

该测试证明 Fast DDS 的节点发现及 ROS 2 双向话题传输可通过
`10.42.0.1 ↔ 10.42.0.2` 正常工作。它不等价于飞控授权、MAVROS 硬件
接入、UWB 测量、飞行安全或 Delta 机械臂动作验证。

## 6. 运行原则

1. `end0` 固定专用于 `10.42.0.0/24` 的 NX–MP257 通信。
2. 257 的公网与路由器访问使用 USB 有线网卡或 Wi-Fi。
3. 严格保持职责边界：257 负责 MAVROS、飞控权限、任务状态、UWB 与
   FCU 接口；NX 负责感知、机械臂执行和观测结果上报。
4. 正式系统应统一约定生产 `ROS_DOMAIN_ID`；`78` 仅用于 2026 年 7 月
   14 日的隔离通信测试。
5. 在串口设备、飞控状态和安全条件确认前，不要直接启动飞行任务节点。

## 7. 临时测试资源说明

ROS 2 通信测试使用的临时容器已删除。为完成测试曾导入一个 Humble 基础
镜像；原有无人机运行环境仍应以 `my_ros2_humble:latest` 为准。长期部署
前应检查未使用的 Docker 镜像，仅保留板端启动路径实际需要的资源。
