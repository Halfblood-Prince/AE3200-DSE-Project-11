import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapMonitor(Node):
    def __init__(self):
        super().__init__("map_monitor")
        self._received_map = False
        self._seconds_waited = 0
        self.declare_parameter("map_topic", "/map_valid")
        map_topic = self.get_parameter("map_topic").value
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._handle_map, map_qos)
        self.create_timer(5.0, self._report_status)

    def _handle_map(self, msg):
        if self._received_map:
            return
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn("Ignoring empty map while waiting for lidar returns")
            return
        self._received_map = True
        map_topic = self.get_parameter("map_topic").value
        self.get_logger().info(
            f"Received {map_topic} "
            f"({msg.info.width}x{msg.info.height}, resolution {msg.info.resolution:.3f})"
        )

    def _report_status(self):
        if self._received_map:
            return
        self._seconds_waited += 5
        self.get_logger().warn(
            f"Still waiting for filtered map after {self._seconds_waited}s. "
            "Move the robot and confirm /scan uses frame lidar_link with base_link -> lidar_link TF."
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
