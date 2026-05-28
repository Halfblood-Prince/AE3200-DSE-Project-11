"""Unit tests for ROS helper logic that can run in a ROS environment."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs.msg")

from nav_msgs.msg import OccupancyGrid

from ros_test import auto_drive, map_filter, map_monitor


def make_grid(width, height, resolution=0.05):
    """Build a minimal OccupancyGrid-like message for map tests."""
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    return msg


def test_transient_map_qos_helpers_match():
    """Map helper nodes should use the same latched QoS settings."""
    filter_qos = map_filter.transient_map_qos()
    monitor_qos = map_monitor.transient_map_qos()

    assert filter_qos.depth == 1
    assert monitor_qos.depth == 1
    assert filter_qos.durability == monitor_qos.durability
    assert filter_qos.reliability == monitor_qos.reliability


def test_map_filter_drops_empty_maps():
    """Empty maps should be counted but not published."""
    node = map_filter.MapFilter.__new__(map_filter.MapFilter)
    node._dropped_empty = 0
    node._published_first = False
    node.get_logger = lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None)
    node._publisher = SimpleNamespace(published=[], publish=lambda msg: node._publisher.published.append(msg))

    node._handle_map(make_grid(0, 0))

    assert node._dropped_empty == 1
    assert node._publisher.published == []


def test_map_filter_publishes_non_empty_maps():
    """Valid maps should be forwarded exactly once per input message."""
    node = map_filter.MapFilter.__new__(map_filter.MapFilter)
    node._dropped_empty = 0
    node._published_first = False
    node.get_logger = lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None)
    node._publisher = SimpleNamespace(published=[], publish=lambda msg: node._publisher.published.append(msg))
    msg = make_grid(4, 3)

    node._handle_map(msg)

    assert node._publisher.published == [msg]
    assert node._published_first is True


def test_auto_drive_sector_min_filters_points(monkeypatch):
    """The sector scan should ignore invalid/high/behind points and keep the nearest obstacle."""
    points = [
        (2.0, 0.0, 0.0),
        (0.5, 0.1, 0.0),
        (0.2, 0.0, 1.5),
        (-0.1, 0.0, 0.0),
    ]
    monkeypatch.setattr(auto_drive.point_cloud2, "read_points", lambda *_, **__: iter(points))

    node = auto_drive.AutoDrive.__new__(auto_drive.AutoDrive)
    node._cloud = object()
    node._last_point_warning_ns = 0

    assert node._sector_min(-0.35, 0.35) == pytest.approx(0.5099019514)
