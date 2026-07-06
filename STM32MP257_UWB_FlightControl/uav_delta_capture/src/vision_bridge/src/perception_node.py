#!/usr/bin/env python3
import sys
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class PerceptionLogicNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.publish_topic = self.declare_parameter('publish_topic', 'vision/target_offset').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_optical_frame').value
        self.model_path = self.declare_parameter('model_path', '').value
        self.input_size = self.declare_parameter('input_size', 320).value
        self.use_npu = self.declare_parameter('use_npu', True).value
        self.conf_thresh = self.declare_parameter('conf_thresh', 0.25).value

        self.offset_pub = self.create_publisher(PointStamped, self.publish_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self._offset_msg = PointStamped()
        self._engine = None

        self._init_engine()

        self.get_logger().info(
            f'Perception node started: image_topic={self.image_topic}, '
            f'publish_topic={self.publish_topic}, engine={type(self._engine).__name__ if self._engine else "None"}'
        )

    def _init_engine(self) -> None:
        try:
            sys.path.insert(0, '/usr/local/Uav_Delta_capture/uav_delta_capture/src/vision_test')
            from vision_test.inference_engine import InferenceEngine
            self._engine = InferenceEngine(
                model_path=self.model_path if self.model_path else None,
                input_size=self.input_size,
                use_npu=self.use_npu,
                conf_thresh=self.conf_thresh,
            )
            self.get_logger().info(f'InferenceEngine loaded: {self._engine.model_path}')
        except Exception as exc:
            self.get_logger().warn(f'Failed to load InferenceEngine: {exc}. Using stub.')
            self._engine = None

    def infer_npu(self, image: Image) -> Optional[Tuple[float, float, float]]:
        if image.width == 0 or image.height == 0:
            return None

        if self._engine is None:
            return float(image.width) * 0.5, float(image.height) * 0.5, 1.0

        encoding = image.encoding
        h, w = image.height, image.width
        data = np.frombuffer(image.data, dtype=np.uint8).reshape(h, w, -1)

        if encoding in ('rgb8', 'bgr8'):
            if encoding == 'bgr8':
                data = data[:, :, ::-1].copy()
        elif encoding == 'mono8':
            data = np.stack([data] * 3, axis=-1)
        else:
            self.get_logger().warn(f'Unsupported encoding: {encoding}')
            return None

        try:
            det, _ = self._engine.infer(data)
        except Exception as exc:
            self.get_logger().warn(f'Inference failed: {exc}')
            return None

        if det is None:
            return None

        cx = det['center'][0]
        cy = det['center'][1]
        conf = det['conf']
        return float(cx), float(cy), float(conf)

    def image_callback(self, msg: Image) -> None:
        result = self.infer_npu(msg)
        if result is None:
            self.get_logger().debug('infer_npu returned None.')
            return

        u_px, v_px, score = result
        if score < 0.1:
            self.get_logger().debug('Low confidence target skipped.')
            return

        cx = float(msg.width) * 0.5
        cy = float(msg.height) * 0.5

        self._offset_msg.header.stamp = msg.header.stamp
        self._offset_msg.header.frame_id = msg.header.frame_id or self.camera_frame
        self._offset_msg.point.x = u_px - cx
        self._offset_msg.point.y = v_px - cy
        self._offset_msg.point.z = 0.0

        self.offset_pub.publish(self._offset_msg)
        self.get_logger().debug(
            f'Published target offset px: ({self._offset_msg.point.x:.2f}, {self._offset_msg.point.y:.2f})'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
