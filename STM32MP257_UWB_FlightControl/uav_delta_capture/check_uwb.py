import rclpy
from rclpy.node import Node
from uav_delta_msgs.msg import UwbAoa

class TestSub(Node):
    def __init__(self):
        super().__init__("test_uwb_check")
        self.count = 0
        self.sub = self.create_subscription(UwbAoa, "uwb_aoa/data", self.cb, 10)

    def cb(self, msg):
        if self.count == 0:
            self.get_logger().info(
                f"GOT UWB: dist={msg.distance_m:.2f}m "
                f"az={msg.azimuth_deg:.1f} deg valid={msg.signal_valid}"
            )
        self.count += 1

rclpy.init()
n = TestSub()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
print(f"Total UWB messages received: {n.count}")
n.destroy_node()
rclpy.shutdown()
