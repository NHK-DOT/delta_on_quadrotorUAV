#!/usr/bin/env python3
"""Validate NX observation packets and publish non-authoritative ROS topics.

This bridge never exposes FCU, arming, mode, waypoint, or velocity controls.
Those decisions stay on the STM32MP257 mission stack.
"""

import json
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class JetsonBridgeNode(Node):
    def __init__(self):
        super().__init__('jetson_bridge_node')
        self.udp_port = self.declare_parameter('udp_port', 5005).value
        self.publish_topic = self.declare_parameter('publish_topic', 'vision/target_offset').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_optical_frame').value
        self.timeout_sec = self.declare_parameter('timeout_sec', 3.0).value
        self.max_packet_age_sec = self.declare_parameter('max_packet_age_sec', 2.0).value
        self.required_protocol = self.declare_parameter(
            'required_protocol', '78arm.nx-arm-bridge/v1'
        ).value

        self.offset_pub = self.create_publisher(PointStamped, self.publish_topic, 10)
        self.json_pub = self.create_publisher(String, 'vision/jetson_detections', 10)
        self.arm_status_pub = self.create_publisher(String, 'vision/arm_status', 10)
        self.grasp_done_pub = self.create_publisher(String, 'grasp_done', 10)
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
            'Jetson observation bridge started: udp_port=%s protocol=%s' %
            (self.udp_port, self.required_protocol)
        )

    def _recv_loop(self):
        while rclpy.ok():
            try:
                data, addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode('utf-8').strip())
            except (ValueError, UnicodeDecodeError) as exc:
                self.get_logger().warn('Invalid JSON from %s: %s' % (addr, exc))
                continue
            if not self._valid_packet(msg):
                self.get_logger().warn('Rejected NX observation from %s' % (addr,))
                continue
            self._last_recv_time = time.time()
            self._warned_timeout = False
            self._publish_raw(data)
            arm_status = msg['arm_status']
            state = str(arm_status.get('state', 'UNKNOWN')).upper()
            self.arm_status_pub.publish(String(data=json.dumps(arm_status, sort_keys=True)))
            if state == 'GRASPED':
                self.grasp_done_pub.publish(String(data='DONE'))
            elif state == 'FAILED':
                self.grasp_done_pub.publish(String(data='FAILED'))
            target = msg.get('target')
            if target is None:
                continue
            offset = target['offset']
            self._offset_msg.header.stamp = self.get_clock().now().to_msg()
            self._offset_msg.header.frame_id = str(target.get('frame_id', self.camera_frame))
            self._offset_msg.point.x = float(offset['dx'])
            self._offset_msg.point.y = float(offset['dy'])
            self._offset_msg.point.z = float(target.get('distance_m', 0.0))
            self.offset_pub.publish(self._offset_msg)

    def _publish_raw(self, data):
        self._json_msg.data = data.decode('utf-8').strip()
        self.json_pub.publish(self._json_msg)

    def _valid_packet(self, msg):
        if not isinstance(msg, dict) or msg.get('protocol') != self.required_protocol:
            return False
        try:
            timestamp = float(msg.get('timestamp_unix', 0.0))
        except (TypeError, ValueError):
            return False
        if abs(time.time() - timestamp) > float(self.max_packet_age_sec):
            return False
        arm_status = msg.get('arm_status')
        if not isinstance(arm_status, dict):
            return False
        if str(arm_status.get('state', 'UNKNOWN')).upper() not in {
            'IDLE', 'GRASPING', 'GRASPED', 'FAILED', 'UNKNOWN'
        }:
            return False
        target = msg.get('target')
        if target is None:
            return True
        if not isinstance(target, dict) or not isinstance(target.get('offset'), dict):
            return False
        try:
            dx = float(target['offset']['dx'])
            dy = float(target['offset']['dy'])
            distance = float(target.get('distance_m', 0.0))
            confidence = float(target.get('conf', 0.0))
        except (KeyError, TypeError, ValueError):
            return False
        return abs(dx) <= 10000.0 and abs(dy) <= 10000.0 and 0.0 <= distance <= 100.0 and 0.0 <= confidence <= 1.0

    def _check_timeout(self):
        if self._last_recv_time == 0.0:
            return
        elapsed = time.time() - self._last_recv_time
        if elapsed > self.timeout_sec and not self._warned_timeout:
            self.get_logger().warn(
                'No validated NX observation for %.1fs (timeout=%.1fs)' %
                (elapsed, self.timeout_sec)
            )
            self._warned_timeout = True

    def destroy_node(self):
        self._sock.close()
        super().destroy_node()


def main(args=None):
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
