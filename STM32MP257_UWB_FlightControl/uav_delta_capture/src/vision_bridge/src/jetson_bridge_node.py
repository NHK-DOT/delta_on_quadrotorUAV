#!/usr/bin/env python3
import json
import socket
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class JetsonBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('jetson_bridge_node')
        self.udp_port = self.declare_parameter('udp_port', 5005).value
        self.publish_topic = self.declare_parameter('publish_topic', 'vision/target_offset').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_optical_frame').value
        self.timeout_sec = self.declare_parameter('timeout_sec', 3.0).value

        self.offset_pub = self.create_publisher(PointStamped, self.publish_topic, 10)
        self.json_pub = self.create_publisher(String, 'vision/jetson_detections', 10)

        self._offset_msg = PointStamped()
        self._json_msg = String()
        self._last_recv_time = 0.0
        self._warned_timeout = False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('0.0.0.0', self.udp_port))
        self._sock.settimeout(1.0)

        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self._timer = self.create_timer(1.0, self._check_timeout)

        self.get_logger().info(
            f'Jetson bridge started: udp_port={self.udp_port}, publish_topic={self.publish_topic}'
        )

    def _recv_loop(self) -> None:
        while rclpy.ok():
            try:
                data, addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            self._last_recv_time = time.time()
            self._warned_timeout = False

            try:
                msg = json.loads(data.decode('utf-8').strip())
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                self.get_logger().warn(f'Invalid JSON from {addr}: {exc}')
                continue

            target = msg.get('target')
            if not target:
                continue

            offset = target.get('offset', {})
            dx = offset.get('dx', 0)
            dy = offset.get('dy', 0)
            conf = target.get('conf', 0.0)
            distance_m = target.get('distance_m') or msg.get('distance_m') or 0.0

            self._offset_msg.header.stamp = self.get_clock().now().to_msg()
            self._offset_msg.header.frame_id = self.camera_frame
            self._offset_msg.point.x = float(dx)
            self._offset_msg.point.y = float(dy)
            self._offset_msg.point.z = float(distance_m)
            self.offset_pub.publish(self._offset_msg)

            self._json_msg.data = data.decode('utf-8').strip()
            self.json_pub.publish(self._json_msg)

            self.get_logger().debug(
                f'From {addr}: dx={dx} dy={dy} conf={conf:.2f} dist={distance_m:.3f}m'
            )

    def _check_timeout(self) -> None:
        if self._last_recv_time == 0.0:
            return
        elapsed = time.time() - self._last_recv_time
        if elapsed > self.timeout_sec and not self._warned_timeout:
            self.get_logger().warn(
                f'No Jetson data for {elapsed:.1f}s (timeout={self.timeout_sec}s)'
            )
            self._warned_timeout = True

    def destroy_node(self) -> None:
        self._sock.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetsonBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
