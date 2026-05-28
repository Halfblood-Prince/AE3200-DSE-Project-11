#!/usr/bin/env python3
"""Filter lidar self-hits before RViz and OctoMap consume the cloud."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


@dataclass(frozen=True)
class CloudFilterConfig:
    """Sensor-frame crop settings for removing robot and floor returns."""

    min_sensor_z: float = -0.55
    self_min_x: float = -1.10
    self_max_x: float = 0.20
    self_min_y: float = -0.45
    self_max_y: float = 0.45
    self_min_z: float = -0.45
    self_max_z: float = -0.15

    def accepts(self, x: float, y: float, z: float) -> bool:
        """Return true when a point should be kept."""
        if not all(math.isfinite(value) for value in (x, y, z)):
            return False
        if z < self.min_sensor_z:
            return False
        return not (
            self.self_min_x <= x <= self.self_max_x
            and self.self_min_y <= y <= self.self_max_y
            and self.self_min_z <= z <= self.self_max_z
        )


def filtered_cloud_qos() -> QoSProfile:
    """Use reliable volatile QoS for filtered clouds consumed by OctoMap/RViz."""
    return QoSProfile(
        depth=5,
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def _xyz_offsets(fields: list[PointField]) -> tuple[int, int, int]:
    """Return x/y/z float offsets from a PointCloud2 field list."""
    by_name = {field.name: field for field in fields}
    missing = [name for name in ("x", "y", "z") if name not in by_name]
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(missing)}")

    xyz_fields = [by_name[name] for name in ("x", "y", "z")]
    unsupported = [field.name for field in xyz_fields if field.datatype != PointField.FLOAT32]
    if unsupported:
        raise ValueError(f"PointCloud2 fields must be FLOAT32: {', '.join(unsupported)}")

    return tuple(field.offset for field in xyz_fields)


def _iter_point_records(msg: PointCloud2):
    """Yield point record offsets while respecting organized-cloud row padding."""
    raw = bytes(msg.data)
    if msg.point_step <= 0:
        return

    for row in range(msg.height):
        row_start = row * msg.row_step
        for column in range(msg.width):
            start = row_start + column * msg.point_step
            end = start + msg.point_step
            if end <= len(raw):
                yield start, raw[start:end]


def filter_cloud_message(msg: PointCloud2, config: CloudFilterConfig) -> PointCloud2:
    """Return a new cloud containing only points accepted by the filter config."""
    x_offset, y_offset, z_offset = _xyz_offsets(msg.fields)
    unpack_float = struct.Struct(">f" if msg.is_bigendian else "<f").unpack_from
    raw = bytes(msg.data)
    kept = bytearray()

    for start, record in _iter_point_records(msg):
        x = unpack_float(raw, start + x_offset)[0]
        y = unpack_float(raw, start + y_offset)[0]
        z = unpack_float(raw, start + z_offset)[0]
        if config.accepts(x, y, z):
            kept.extend(record)

    filtered = PointCloud2()
    filtered.header = msg.header
    filtered.height = 1
    filtered.width = len(kept) // msg.point_step if msg.point_step else 0
    filtered.fields = msg.fields
    filtered.is_bigendian = msg.is_bigendian
    filtered.point_step = msg.point_step
    filtered.row_step = len(kept)
    filtered.data = bytes(kept)
    filtered.is_dense = True
    return filtered


class CloudFilter(Node):
    """Republish lidar points after removing self and near-floor returns."""

    def __init__(self) -> None:
        """Create the filtered cloud publisher and raw cloud subscription."""
        super().__init__("cloud_filter")

        self.declare_parameter("input_topic", "/points_raw")
        self.declare_parameter("output_topic", "/points_filtered")
        self.declare_parameter("min_sensor_z", CloudFilterConfig.min_sensor_z)
        self.declare_parameter("self_min_x", CloudFilterConfig.self_min_x)
        self.declare_parameter("self_max_x", CloudFilterConfig.self_max_x)
        self.declare_parameter("self_min_y", CloudFilterConfig.self_min_y)
        self.declare_parameter("self_max_y", CloudFilterConfig.self_max_y)
        self.declare_parameter("self_min_z", CloudFilterConfig.self_min_z)
        self.declare_parameter("self_max_z", CloudFilterConfig.self_max_z)

        self._config = self._read_config()
        self._logged_first_cloud = False
        self._logged_bad_cloud = False

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        publisher_qos = filtered_cloud_qos()

        self._publisher = self.create_publisher(PointCloud2, output_topic, publisher_qos)
        self._subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._handle_cloud,
            qos_profile_sensor_data,
        )

    def _read_config(self) -> CloudFilterConfig:
        """Build a filter config from node parameters."""
        return CloudFilterConfig(
            min_sensor_z=float(self.get_parameter("min_sensor_z").value),
            self_min_x=float(self.get_parameter("self_min_x").value),
            self_max_x=float(self.get_parameter("self_max_x").value),
            self_min_y=float(self.get_parameter("self_min_y").value),
            self_max_y=float(self.get_parameter("self_max_y").value),
            self_min_z=float(self.get_parameter("self_min_z").value),
            self_max_z=float(self.get_parameter("self_max_z").value),
        )

    def _handle_cloud(self, msg: PointCloud2) -> None:
        """Filter one raw cloud and publish the result."""
        try:
            filtered = filter_cloud_message(msg, self._config)
        except ValueError as exc:
            if not self._logged_bad_cloud:
                self.get_logger().warning(f"Cannot filter point cloud: {exc}")
                self._logged_bad_cloud = True
            return

        self._publisher.publish(filtered)
        if not self._logged_first_cloud:
            input_count = msg.width * msg.height
            removed = input_count - filtered.width
            output_topic = self.get_parameter("output_topic").value
            self.get_logger().info(
                f"Publishing filtered lidar cloud on {output_topic}; "
                f"removed {removed}/{input_count} points from the first cloud"
            )
            self._logged_first_cloud = True


def main(args: list[str] | None = None) -> None:
    """Run the point-cloud filter node."""
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
