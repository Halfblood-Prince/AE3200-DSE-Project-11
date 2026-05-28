"""Edge-case tests for pathfinding and point-cloud filtering behavior."""

import struct

import numpy as np
import pytest

from pathfinding import astar as astar_module


def test_astar_start_equal_goal_returns_single_node_path():
    """A start position that is already the goal should produce a one-node path."""
    grid = np.zeros((1, 2, 2), dtype=np.uint8)

    assert astar_module.astar(grid, (1, 1, 0), (1, 1, 0)) == [(1, 1, 0)]


def test_astar_occupied_goal_is_unreachable():
    """A blocked goal cell should not be returned as a valid path endpoint."""
    grid = np.zeros((1, 2, 2), dtype=np.uint8)
    grid[0, 1, 1] = 1

    assert astar_module.astar(grid, (0, 0, 0), (1, 1, 0)) is None


def test_neighbors_at_origin_do_not_wrap_negative_indices():
    """Corner neighbor lookup should never wrap around NumPy negative indices."""
    grid = np.zeros((2, 2, 2), dtype=np.uint8)

    neighbors = astar_module.get_neighbors(grid, (0, 0, 0))

    assert set(neighbors) == {(1, 0, 0), (0, 1, 0), (0, 0, 1)}
    assert all(min(position) >= 0 for position in neighbors)


def test_reconstruct_path_without_parents_returns_current_only():
    """Reconstructing from a node with no parent should return that node."""
    assert astar_module.reconstruct_path({}, (2, 3, 1)) == [(2, 3, 1)]


def ros_cloud_modules():
    """Import ROS cloud modules, skipping cloud-filter edge tests outside ROS."""
    pytest.importorskip("rclpy")
    pytest.importorskip("sensor_msgs.msg")
    from sensor_msgs.msg import PointCloud2, PointField
    from ros_test import cloud_filter

    return cloud_filter, PointCloud2, PointField


def make_cloud(
    points,
    *,
    datatype=None,
    is_bigendian=False,
    height=1,
    row_padding=0,
):
    """Build a compact or padded PointCloud2 for cloud-filter edge tests."""
    _cloud_filter, point_cloud_type, point_field_type = ros_cloud_modules()
    datatype = point_field_type.FLOAT32 if datatype is None else datatype
    endian = ">" if is_bigendian else "<"
    point_step = 12
    row_step = point_step * len(points) + row_padding

    msg = point_cloud_type()
    msg.height = height
    msg.width = len(points) // height if height else 0
    msg.fields = [
        point_field_type(name="x", offset=0, datatype=datatype, count=1),
        point_field_type(name="y", offset=4, datatype=datatype, count=1),
        point_field_type(name="z", offset=8, datatype=datatype, count=1),
    ]
    msg.is_bigendian = is_bigendian
    msg.point_step = point_step
    msg.row_step = row_step
    msg.is_dense = True

    data = bytearray()
    for index, point in enumerate(points):
        data.extend(struct.pack(f"{endian}fff", *point))
        end_of_row = (index + 1) % msg.width == 0 if msg.width else True
        if end_of_row:
            data.extend(b"\x00" * row_padding)
    msg.data = bytes(data)
    return msg


def unpack_cloud_points(msg, *, is_bigendian=False):
    """Return xyz tuples from a filtered PointCloud2."""
    endian = ">" if is_bigendian else "<"
    raw = bytes(msg.data)
    return [
        struct.unpack_from(f"{endian}fff", raw, offset)
        for offset in range(0, len(raw), msg.point_step)
    ]


def test_cloud_filter_rejects_clouds_missing_xyz_fields():
    """Clouds without all xyz fields should fail clearly instead of publishing garbage."""
    cloud_filter, point_cloud_type, point_field_type = ros_cloud_modules()
    msg = point_cloud_type()
    msg.fields = [
        point_field_type(name="x", offset=0, datatype=point_field_type.FLOAT32, count=1),
        point_field_type(name="y", offset=4, datatype=point_field_type.FLOAT32, count=1),
    ]

    with pytest.raises(ValueError, match="missing fields: z"):
        cloud_filter.filter_cloud_message(msg, cloud_filter.CloudFilterConfig())


def test_cloud_filter_rejects_non_float_xyz_fields():
    """Cloud filtering expects Gazebo-style FLOAT32 xyz fields."""
    cloud_filter, _point_cloud_type, point_field_type = ros_cloud_modules()
    msg = make_cloud([(1.0, 0.0, 0.0)], datatype=point_field_type.INT32)

    with pytest.raises(ValueError, match="must be FLOAT32"):
        cloud_filter.filter_cloud_message(msg, cloud_filter.CloudFilterConfig())


def test_cloud_filter_handles_big_endian_records():
    """Endian handling should not accidentally keep or drop the wrong point."""
    cloud_filter, _point_cloud_type, _point_field_type = ros_cloud_modules()
    msg = make_cloud([(2.0, 0.0, 0.0), (0.0, 0.0, -0.3)], is_bigendian=True)

    filtered = cloud_filter.filter_cloud_message(msg, cloud_filter.CloudFilterConfig())

    assert filtered.is_bigendian is True
    assert filtered.width == 1
    assert unpack_cloud_points(filtered, is_bigendian=True) == [(2.0, 0.0, 0.0)]


def test_cloud_filter_respects_organized_row_padding():
    """Organized clouds may have row padding that must not be interpreted as points."""
    cloud_filter, _point_cloud_type, _point_field_type = ros_cloud_modules()
    msg = make_cloud([(2.0, 0.0, 0.0), (3.0, 0.0, 0.0)], height=2, row_padding=4)

    filtered = cloud_filter.filter_cloud_message(msg, cloud_filter.CloudFilterConfig())

    assert filtered.width == 2
    assert filtered.row_step == filtered.point_step * filtered.width
    assert unpack_cloud_points(filtered) == [(2.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
