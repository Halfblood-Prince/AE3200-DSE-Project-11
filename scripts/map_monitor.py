#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def transient_map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class MapMonitor(Node):
    def __init__(self) -> None:
        super().__init__("map_monitor")
        self.declare_parameter("map_topic", "/map_valid")

        self._received_map = False
        self._seconds_waited = 0

        map_topic = self.get_parameter("map_topic").value
        self._subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._handle_map,
            transient_map_qos(),
        )
        self._timer = self.create_timer(5.0, self._report_status)

    def _handle_map(self, msg: OccupancyGrid) -> None:
        if self._received_map:
            return
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warning("Ignoring empty map while waiting for lidar returns")
            return

        self._received_map = True
        map_topic = self.get_parameter("map_topic").value
        self.get_logger().info(
            f"Received {map_topic} "
            f"({msg.info.width}x{msg.info.height}, resolution {msg.info.resolution:.3f})"
        )

    def _report_status(self) -> None:
        if self._received_map:
            return

        self._seconds_waited += 5
        self.get_logger().warning(
            "Still waiting for filtered map after "
            f"{self._seconds_waited}s. Move the robot and confirm /points_raw "
            "uses frame lidar_link with map -> odom -> base_link -> lidar_link TF."
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
