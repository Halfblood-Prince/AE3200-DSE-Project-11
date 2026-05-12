import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class AutoDrive(Node):
    def __init__(self):
        super().__init__("auto_drive")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._scan = None
        self._logged_first_scan = False
        self.create_subscription(LaserScan, "/scan", self._handle_scan, 10)
        self.create_timer(0.1, self._publish_cmd)

    def _handle_scan(self, msg):
        self._scan = msg
        if not self._logged_first_scan:
            self.get_logger().info("Auto drive received /scan and is publishing /cmd_vel")
            self._logged_first_scan = True

    def _publish_cmd(self):
        cmd = Twist()
        if self._scan is None:
            self._cmd_pub.publish(cmd)
            return

        front = self._sector_min(-0.35, 0.35)
        left = self._sector_min(0.35, 1.2)
        right = self._sector_min(-1.2, -0.35)

        if front < 0.75:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8 if left < right else 0.8
        else:
            cmd.linear.x = 0.25
            cmd.angular.z = 0.18

        self._cmd_pub.publish(cmd)

    def _sector_min(self, start_angle, end_angle):
        ranges = []
        angle = self._scan.angle_min
        for value in self._scan.ranges:
            if start_angle <= angle <= end_angle and math.isfinite(value):
                ranges.append(value)
            angle += self._scan.angle_increment
        return min(ranges) if ranges else float("inf")


def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
