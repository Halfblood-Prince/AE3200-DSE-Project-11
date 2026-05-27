#!/usr/bin/env python3
"""Republish only valid OccupancyGrid messages from the mapper."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def transient_map_qos() -> QoSProfile:
    """Use latched map QoS so late subscribers immediately receive the map."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class MapFilter(Node):
    """Drop empty maps that can confuse downstream navigation tools."""

    def __init__(self) -> None:
        """Create the input subscription and filtered map publisher."""
        super().__init__("map_filter")

        # Topics are parameters so the node can be reused with other mappers.
        self.declare_parameter("input_topic", "/map")
        self.declare_parameter("output_topic", "/map_valid")

        # These flags keep startup logs useful without repeating every message.
        self._dropped_empty = 0
        self._published_first = False

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        qos = transient_map_qos()

        self._publisher = self.create_publisher(OccupancyGrid, output_topic, qos)
        self._subscription = self.create_subscription(
            OccupancyGrid,
            input_topic,
            self._handle_map,
            qos,
        )

    def _handle_map(self, msg: OccupancyGrid) -> None:
        """Forward non-empty occupancy grids and suppress empty placeholders."""
        if msg.info.width == 0 or msg.info.height == 0:
            self._dropped_empty += 1
            if self._dropped_empty == 1:
                self.get_logger().warning("Dropping empty /map from the mapper")
            return

        self._publisher.publish(msg)
        if not self._published_first:
            self.get_logger().info(
                "Publishing filtered map on /map_valid "
                f"({msg.info.width}x{msg.info.height}, resolution {msg.info.resolution:.3f})"
            )
            self._published_first = True


def main(args: list[str] | None = None) -> None:
    """Run the map filter node."""
    rclpy.init(args=args)
    node = MapFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
