#!/usr/bin/env python3
"""Simple lidar-reactive driver for keeping the robot moving in simulation."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class AutoDrive(Node):
    """Read the 3D lidar cloud and publish a small obstacle-avoidance twist."""

    def __init__(self) -> None:
        """Create publishers, subscribers, and the command timer."""
        super().__init__("auto_drive")

        # The latest point cloud is cached because the timer publishes commands
        # at a fixed rate even when lidar messages arrive at a different rate.
        self._cloud: Optional[PointCloud2] = None
        self._logged_first_cloud = False
        self._last_point_warning_ns = 0

        # Commands go to /cmd_vel, which Gazebo teleop and VelocityControl use.
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._points_sub = self.create_subscription(
            PointCloud2,
            "/points_raw",
            self._handle_cloud,
            10,
        )
        self._timer = self.create_timer(0.1, self._publish_cmd)

    def _handle_cloud(self, msg: PointCloud2) -> None:
        """Store the newest lidar scan and log once when data starts flowing."""
        self._cloud = msg
        if not self._logged_first_cloud:
            self.get_logger().info(
                "Auto drive received /points_raw and is publishing /cmd_vel"
            )
            self._logged_first_cloud = True

    def _publish_cmd(self) -> None:
        """Publish forward motion unless the front sector is blocked."""
        cmd = Twist()
        if self._cloud is None:
            # Publishing zero while waiting prevents stale motion commands.
            self._cmd_pub.publish(cmd)
            return

        # Split the forward half-space into right, front, and left sectors.
        front = self._sector_min(-0.35, 0.35)
        left = self._sector_min(0.35, 1.2)
        right = self._sector_min(-1.2, -0.35)

        if front < 0.75:
            # Turn toward the side with more clearance.
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8 if left < right else 0.8
        else:
            # Cruise slowly with a slight turn so the map grows even in open space.
            cmd.linear.x = 0.25
            cmd.angular.z = 0.18

        self._cmd_pub.publish(cmd)

    def _sector_min(self, start_angle: float, end_angle: float) -> float:
        """Return the closest valid point distance inside an angular sector."""
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
                # Ignore floor noise and high returns that are not immediate obstacles.
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
    """Start the ROS node and keep it alive until shutdown."""
    rclpy.init(args=args)
    node = AutoDrive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
