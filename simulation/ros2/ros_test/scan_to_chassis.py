import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanToChassis(Node):
    def __init__(self):
        super().__init__("scan_to_chassis")
        self.declare_parameter("scan_frame_id", "lidar_link")
        self._publisher = self.create_publisher(LaserScan, "/scan", 10)
        self._logged_first_scan = False
        self.create_subscription(LaserScan, "/scan_raw", self._handle_scan, 10)

    def _handle_scan(self, msg):
        msg.header.frame_id = self.get_parameter("scan_frame_id").value
        self._publisher.publish(msg)

        if not self._logged_first_scan:
            self.get_logger().info(
                f"Republishing /scan_raw as /scan with frame_id {msg.header.frame_id!r}"
            )
            self._logged_first_scan = True


def main(args=None):
    rclpy.init(args=args)
    node = ScanToChassis()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
