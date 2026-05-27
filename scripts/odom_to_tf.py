#!/usr/bin/env python3
"""Convert odometry messages into the odom -> base_link TF transform."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    """Bridge /odom pose data into TF for RViz and OctoMap."""

    def __init__(self) -> None:
        """Create the TF broadcaster and /odom subscription."""
        super().__init__("odom_to_tf")

        # TF is generated from odometry because the Gazebo bridge only forwards
        # the nav_msgs/Odometry message, not the corresponding TF tree.
        self._broadcaster = TransformBroadcaster(self)
        self._logged_first_odom = False
        self._subscription = self.create_subscription(
            Odometry,
            "/odom",
            self._handle_odom,
            10,
        )

    def _handle_odom(self, msg: Odometry) -> None:
        """Publish the pose from /odom as odom -> base_link."""
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)

        if not self._logged_first_odom:
            self.get_logger().info(
                "Publishing TF odom -> base_link from /odom "
                f"(source frames: '{msg.header.frame_id}' -> '{msg.child_frame_id}')"
            )
            self._logged_first_odom = True


def main(args: list[str] | None = None) -> None:
    """Run the odometry-to-TF node."""
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
