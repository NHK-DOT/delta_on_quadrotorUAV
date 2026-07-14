# NX–STM32MP257 Wired LAN and ROS 2 Verification

**Verified on July 14, 2026.** This note records the dedicated wired network
between the Jetson Xavier NX and STM32MP257 board, the existing board-side ROS
2 Docker environment, and the cross-machine ROS 2 validation. It is an
integration and recovery record, not flight authorization.

For a Chinese version, see
[`NX_257_WIRED_LAN_AND_ROS2.zh-CN.md`](NX_257_WIRED_LAN_AND_ROS2.zh-CN.md).

## 1. Network Roles

The system uses two separate wired networks on the STM32MP257 board:

```text
Jetson Xavier NX eth0  10.42.0.1/24
            | dedicated Ethernet cable
STM32MP257 end0        10.42.0.2/24

STM32MP257 USB Ethernet interface (name can vary, observed: enu1u2u4)
            | external router / Internet
            `-- DHCP address on 192.168.1.0/24 (observed: 192.168.1.41)
```

`end0` is the STM32MP257's physical onboard Ethernet port. It is dedicated to
NX–MP257 communication and must not be reused as the external-router port
without changing its configuration. The USB Ethernet adapter is the external
network path.

This is a routed IP subnet, not a Linux bridge. That is intentional: the
dedicated `10.42.0.0/24` link isolates control/data traffic while still
supporting normal ROS 2 DDS discovery and topic traffic.

## 2. Persistent Address Configuration

### NX

The NX NetworkManager connection is named `nx-257-direct` and binds to
`eth0`. Its address is `10.42.0.1/24`. The NX Wi-Fi connection remains
independent at `192.168.1.174/24` when connected to the field router.

Useful checks on NX:

```bash
ip -4 -br addr show eth0
ip route get 10.42.0.2
ping -c 4 10.42.0.2
```

### STM32MP257

The board runs OpenSTLinux with `systemd-networkd`. The persistent, local
override is:

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

The file only matches `end0`; it does **not** modify `wlan0`, the USB Ethernet
adapter, or `tailscale0`. It intentionally has no `Gateway=` or `DNS=` entry,
so the NX control link never replaces the board's external default route.

Useful checks on the board:

```bash
ip -4 -br addr
ip -4 route
networkctl status end0 --no-pager
ping -I end0 -c 4 10.42.0.1
```

## 3. External Network Separation

When connected to a router, the observed USB Ethernet interface was
`enu1u2u4`. It received `192.168.1.41/24` through DHCP and owned the default
route:

```text
default via 192.168.1.1 dev enu1u2u4
10.42.0.0/24 dev end0 src 10.42.0.2
192.168.1.0/24 dev enu1u2u4 src 192.168.1.41
```

The interface name may change with a different USB adapter or port. Do not
hard-code `enu1u2u4` into flight software; discover the active external
interface from `ip -4 -br addr` or the default route.

Validation completed on July 14, 2026:

| Path | Result |
| --- | --- |
| STM32MP257 `end0` to NX `10.42.0.1` | Reachable, approximately 0.49 ms |
| STM32MP257 USB Ethernet to router `192.168.1.1` | Reachable |
| STM32MP257 USB Ethernet to `1.1.1.1` | Reachable, approximately 3.2 ms |

## 4. Existing ROS 2 Docker Environment on STM32MP257

The OpenSTLinux host does not provide a native `/opt/ros` installation. ROS 2
is intentionally provided by Docker on this board.

Existing project assets found on the board:

| Item | Value |
| --- | --- |
| Custom image | `my_ros2_humble:latest` |
| Architecture | `arm64` |
| Image contents | ROS 2 Humble, MAVROS, MAVROS extras, GeographicLib data, colcon tooling |
| Primary container | `ros2humble` |
| Network mode | `host` |
| Workspace mount | `/usr/local/Uav_Delta_capture` → `/workspace` |
| Container working directory | `/workspace/uav_delta_capture` |
| Project startup entry | `/usr/local/Uav_Delta_capture/start_ready.sh` |

The mounted workspace has already been built. It includes `delta_kinematics`,
`uav_delta_msgs`, `fcu_bridge`, `vision_test`, `safety`, `uwb_driver`,
`uwb_navigation`, `vision_bridge`, and `uav_bridge`.

The original container was stopped because Docker was asked to map
`/dev/ttyACM0` and `/dev/ttyUSB0`, but neither device existed on the board at
the time of verification. This is a hardware-device availability problem, not
a ROS 2 or NX–MP257 network failure. Before starting flight-related software,
inspect the actual serial devices:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttySTM* 2>/dev/null
docker ps -a
```

Do not remove or recreate the original `ros2humble` container until its device
mapping is reconciled with the currently connected FCU and peripheral devices.

## 5. Cross-Machine ROS 2 Test

The NX already had a running `ros2_humble` Humble container in host-network
mode. A temporary Humble container was run on the STM32MP257 to validate DDS
across the dedicated cable.

Test environment:

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=78
ROS_LOCALHOST_ONLY=0
```

To prove that discovery and traffic used the dedicated cable rather than the
external router, the board's USB Ethernet interface was temporarily brought
down during the test, then restored and confirmed to reacquire its DHCP route.
The following messages were received:

| Direction | Topic | Received payload |
| --- | --- | --- |
| NX → STM32MP257 | `/nx_to_257_direct` | `nx-to-257-over-10.42` |
| STM32MP257 → NX | `/board_to_nx_direct` | `257-to-nx-over-10.42` |

This validates Fast DDS participant discovery and bidirectional ROS 2 topic
delivery over `10.42.0.1 ↔ 10.42.0.2`. It does not validate FCU authority,
MAVROS hardware access, UWB measurements, flight safety, or Delta-arm motion.

## 6. Operational Guidance

1. Keep `end0` dedicated to `10.42.0.0/24` NX–MP257 traffic.
2. Use USB Ethernet or Wi-Fi for the board's external network and Internet.
3. Keep the NX/MP257 responsibility boundary intact: the board owns MAVROS,
   flight authority, mission state, UWB and FCU interfaces; NX owns perception,
   arm execution, and observation reporting.
4. Use a deliberate shared `ROS_DOMAIN_ID` for production nodes. Domain `78`
   was selected only for the isolated July 14, 2026 communication test.
5. Run the existing preflight and MAVROS scripts only after serial devices,
   flight-controller state, and safety conditions have been checked.

## 7. Temporary Test Runtime Notes

The temporary ROS test container was removed after validation. A Humble base
image was imported on the board solely to run that test; the original UAV
runtime is `my_ros2_humble:latest`. Review unused Docker images before long
term deployment and retain only images required by the board's documented
startup path.
