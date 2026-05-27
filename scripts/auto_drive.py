#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class AutoDrive(Node):
    def __init__(self) -> None:
        super().__init__("auto_drive")
        self._cloud: Optional[PointCloud2] = None
        self._logged_first_cloud = False
        self._last_point_warning_ns = 0

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._points_sub = self.create_subscription(
            PointCloud2,
            "/points_raw",
            self._handle_cloud,
            10,
        )
        self._timer = self.create_timer(0.1, self._publish_cmd)

    def _handle_cloud(self, msg: PointCloud2) -> None:
        self._cloud = msg
        if not self._logged_first_cloud:
            self.get_logger().info(
                "Auto drive received /points_raw and is publishing /cmd_vel"
            )
            self._logged_first_cloud = True

    def _publish_cmd(self) -> None:
        cmd = Twist()
        if self._cloud is None:
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

    def _sector_min(self, start_angle: float, end_angle: float) -> float:
        if self._cloud is None:
            return math.inf

        best = math.inf
        try:
            points = point_cloud2.read_points(
                self._cloud,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
            for point in points:
                x = float(point[0])
                y = float(point[1])
                z = float(point[2])
                if (
                    not math.isfinite(x)
                    or not math.isfinite(y)
                    or not math.isfinite(z)
                    or x <= 0.0
                ):
                    continue
                if z < -0.35 or z > 1.2:
                    continue

                angle = math.atan2(y, x)
                if start_angle <= angle <= end_angle:
                    best = min(best, math.hypot(x, y))
        except (KeyError, IndexError, ValueError, TypeError) as error:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_point_warning_ns >= 5_000_000_000:
                self.get_logger().warning(
                    f"Unable to read XYZ fields from /points_raw: {error}"
                )
                self._last_point_warning_ns = now_ns

        return best


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoDrive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
