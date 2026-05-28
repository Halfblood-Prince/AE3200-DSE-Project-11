"""Unit tests for ROS helper logic that can run in a ROS environment."""

from types import SimpleNamespace
import struct

import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("builtin_interfaces.msg")
pytest.importorskip("nav_msgs.msg")
pytest.importorskip("sensor_msgs.msg")

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from ros_test import astar_path_publisher, cloud_filter, map_filter, map_monitor, odom_to_tf


class RecordingLogger:
    """Capture node log messages for assertions."""

    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


class RecordingPublisher:
    """Capture messages published by helper-node methods."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


def patch_node_methods(monkeypatch):
    """Replace rclpy Node methods with lightweight recorders."""

    def node_init(self, name):
        self._node_name = name
        self._declared_parameters = {}
        self._publishers = []
        self._subscriptions = []
        self._timers = []
        self._logger = RecordingLogger()

    def declare_parameter(self, name, default):
        self._declared_parameters[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self._declared_parameters[name])

    def create_publisher(self, msg_type, topic, qos):
        publisher = RecordingPublisher()
        self._publishers.append((msg_type, topic, qos, publisher))
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        subscription = SimpleNamespace(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self._subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self._timers.append(timer)
        return timer

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=0, nanosec=0))
        )

    for module in (astar_path_publisher, cloud_filter, map_filter, map_monitor, odom_to_tf):
        monkeypatch.setattr(module.Node, "__init__", node_init)
        monkeypatch.setattr(module.Node, "declare_parameter", declare_parameter)
        monkeypatch.setattr(module.Node, "get_parameter", get_parameter)
        monkeypatch.setattr(module.Node, "create_publisher", create_publisher)
        monkeypatch.setattr(module.Node, "create_subscription", create_subscription)
        monkeypatch.setattr(module.Node, "create_timer", create_timer)
        monkeypatch.setattr(module.Node, "get_clock", get_clock)
        monkeypatch.setattr(module.Node, "get_logger", lambda self: self._logger)


def make_grid(width, height, resolution=0.05):
    """Build a minimal OccupancyGrid-like message for map tests."""
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    return msg


def make_cloud(points):
    """Build a PointCloud2 with FLOAT32 x/y/z fields."""
    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    msg.is_dense = True
    return msg


def unpack_cloud_points(msg):
    """Return xyz tuples from a compact FLOAT32 test cloud."""
    data = bytes(msg.data)
    return [
        struct.unpack_from("<fff", data, offset)
        for offset in range(0, len(data), msg.point_step)
    ]


def test_transient_map_qos_helpers_match():
    """Map helper nodes should use the same latched QoS settings."""
    filter_qos = map_filter.transient_map_qos()
    monitor_qos = map_monitor.transient_map_qos()
    path_qos = astar_path_publisher.transient_path_qos()

    assert filter_qos.depth == 1
    assert monitor_qos.depth == 1
    assert path_qos.depth == 1
    assert filter_qos.durability == monitor_qos.durability
    assert filter_qos.reliability == monitor_qos.reliability
    assert path_qos.durability == filter_qos.durability
    assert path_qos.reliability == filter_qos.reliability


def test_astar_path_publisher_initializes_configured_topic(monkeypatch):
    """The saved-map planner should publish a latched nav_msgs/Path for RViz."""
    patch_node_methods(monkeypatch)

    node = astar_path_publisher.AstarPathPublisher()

    assert node._node_name == "astar_path_publisher"
    assert node._publishers[0][1] == "/astar_path"
    assert node._timers[0].period == 1.0
    node._publish_path()
    assert node._publisher.published == []


def test_astar_path_message_uses_voxel_centers():
    """A* grid indices should be converted back into map-frame voxel centers."""
    grid = astar_path_publisher.OctomapGrid(
        grid=np.zeros((1, 1, 2), dtype=np.uint8),
        origin=(1.0, 2.0, 3.0),
        resolution=0.5,
        occupied_leaves=0,
        free_leaves=2,
    )
    stamp = Time(sec=7, nanosec=5)

    msg = astar_path_publisher.path_to_message(grid, [(0, 0, 0), (1, 0, 0)], "map", stamp)

    assert msg.header.frame_id == "map"
    assert msg.header.stamp == stamp
    assert len(msg.poses) == 2
    assert msg.poses[0].pose.position.x == 1.25
    assert msg.poses[0].pose.position.y == 2.25
    assert msg.poses[0].pose.position.z == 3.25
    assert msg.poses[1].pose.position.x == 1.75
    assert all(pose.pose.orientation.w == 1.0 for pose in msg.poses)


def test_astar_path_message_allows_empty_paths():
    """No-route results should still publish an empty Path in the right frame."""
    grid = astar_path_publisher.OctomapGrid(
        grid=np.zeros((1, 1, 1), dtype=np.uint8),
        origin=(0.0, 0.0, 0.0),
        resolution=1.0,
        occupied_leaves=0,
        free_leaves=1,
    )
    stamp = Time(sec=1, nanosec=0)

    msg = astar_path_publisher.path_to_message(grid, None, "map", stamp)

    assert msg.header.frame_id == "map"
    assert msg.header.stamp == stamp
    assert msg.poses == []


def test_prepare_planning_grid_frees_start_and_goal_only():
    """Start and goal should be usable even if the saved map marks them occupied."""
    grid = astar_path_publisher.OctomapGrid(
        grid=np.ones((1, 1, 3), dtype=np.uint8),
        origin=(0.0, 0.0, 0.0),
        resolution=1.0,
        occupied_leaves=3,
        free_leaves=0,
    )

    planning_grid = astar_path_publisher.prepare_planning_grid(grid, (0, 0, 0), (2, 0, 0))

    np.testing.assert_array_equal(planning_grid, np.array([[[0, 1, 0]]], dtype=np.uint8))
    assert grid.grid[0, 0, 0] == 1


def test_prepare_planning_grid_rejects_out_of_bounds_goal():
    """Bad world-to-grid conversions should fail before A* starts."""
    grid = astar_path_publisher.OctomapGrid(
        grid=np.zeros((1, 1, 1), dtype=np.uint8),
        origin=(0.0, 0.0, 0.0),
        resolution=1.0,
        occupied_leaves=0,
        free_leaves=1,
    )

    with pytest.raises(ValueError, match="goal index"):
        astar_path_publisher.prepare_planning_grid(grid, (0, 0, 0), (1, 0, 0))


def test_parse_xyz_requires_three_values():
    """List parameters for start and goal should be exactly x/y/z."""
    assert astar_path_publisher.parse_xyz([1, 2, 3], "start_xyz") == (1.0, 2.0, 3.0)
    with pytest.raises(ValueError, match="exactly three"):
        astar_path_publisher.parse_xyz([1, 2], "goal_xyz")


def test_plan_path_from_bt_delegates_to_converter_and_astar(monkeypatch):
    """The planning helper should convert world points to A* grid indices."""
    octomap_grid = astar_path_publisher.OctomapGrid(
        grid=np.ones((1, 1, 3), dtype=np.uint8),
        origin=(0.0, 0.0, 0.0),
        resolution=1.0,
        occupied_leaves=3,
        free_leaves=0,
    )
    calls = {}

    def fake_converter(bt_path, **kwargs):
        calls["converter"] = (bt_path, kwargs)
        return octomap_grid

    def fake_astar(grid, start, goal):
        calls["astar"] = (grid.copy(), start, goal)
        return [start, goal]

    monkeypatch.setattr(astar_path_publisher, "bt_file_to_numpy_grid", fake_converter)
    monkeypatch.setattr(astar_path_publisher, "astar", fake_astar)

    result = astar_path_publisher.plan_path_from_bt(
        "map.bt",
        (0.5, 0.5, 0.5),
        (2.5, 0.5, 0.5),
        planning_resolution=0.5,
        padding=2.0,
        unknown_is_occupied=False,
    )

    assert result[0] is octomap_grid
    assert result[1] == [(0, 0, 0), (2, 0, 0)]
    assert result[2:] == ((0, 0, 0), (2, 0, 0))
    assert calls["converter"][0] == "map.bt"
    assert calls["converter"][1]["include_points"] == ((0.5, 0.5, 0.5), (2.5, 0.5, 0.5))
    np.testing.assert_array_equal(calls["astar"][0], np.array([[[0, 1, 0]]], dtype=np.uint8))


def test_astar_path_publisher_plans_and_republishes(monkeypatch):
    """A successful plan should be cached and republished for late RViz subscribers."""
    patch_node_methods(monkeypatch)
    octomap_grid = astar_path_publisher.OctomapGrid(
        grid=np.zeros((1, 1, 2), dtype=np.uint8),
        origin=(0.0, 0.0, 0.0),
        resolution=1.0,
        occupied_leaves=0,
        free_leaves=2,
    )

    def fake_plan(*_args, **_kwargs):
        return octomap_grid, [(0, 0, 0), (1, 0, 0)], (0, 0, 0), (1, 0, 0)

    monkeypatch.setattr(astar_path_publisher, "plan_path_from_bt", fake_plan)
    node = astar_path_publisher.AstarPathPublisher()
    node._declared_parameters["bt_path"] = "map.bt"

    node._plan_once()
    node._plan_once()
    node._publish_path()

    assert node._planned is True
    assert len(node._path_message.poses) == 2
    assert len(node._publisher.published) == 1
    assert len(node._logger.infos) == 1
    assert "Publishing A* path" in node._logger.infos[0]


def test_astar_path_publisher_publishes_empty_path_when_unreachable(monkeypatch):
    """A failed A* search should still publish an empty path message."""
    patch_node_methods(monkeypatch)
    octomap_grid = astar_path_publisher.OctomapGrid(
        grid=np.zeros((1, 1, 2), dtype=np.uint8),
        origin=(0.0, 0.0, 0.0),
        resolution=1.0,
        occupied_leaves=0,
        free_leaves=2,
    )

    def fake_plan(*_args, **_kwargs):
        return octomap_grid, None, (0, 0, 0), (1, 0, 0)

    monkeypatch.setattr(astar_path_publisher, "plan_path_from_bt", fake_plan)
    node = astar_path_publisher.AstarPathPublisher()
    node._declared_parameters["bt_path"] = "map.bt"

    node._plan_once()
    node._publish_path()

    assert node._planned is True
    assert node._publisher.published[0].poses == []
    assert "No A* path" in node._logger.warnings[-1]


def test_cloud_filter_initializes_configured_topics(monkeypatch):
    """CloudFilter should republish /points_raw as /points_filtered."""
    patch_node_methods(monkeypatch)

    node = cloud_filter.CloudFilter()

    assert node._node_name == "cloud_filter"
    assert node._publishers[0][1] == "/points_filtered"
    assert node._subscriptions[0].topic == "/points_raw"
    assert node._subscriptions[0].callback == node._handle_cloud


def test_cloud_filter_removes_self_floor_and_invalid_points():
    """The cloud filter should keep real obstacle returns and drop self hits."""
    msg = make_cloud(
        [
            (2.0, 0.0, 0.0),
            (0.0, 0.0, -0.3),
            (1.0, 0.0, -0.8),
            (float("nan"), 0.0, 0.0),
        ]
    )

    filtered = cloud_filter.filter_cloud_message(msg, cloud_filter.CloudFilterConfig())

    assert filtered.height == 1
    assert filtered.width == 1
    assert filtered.row_step == filtered.point_step
    assert unpack_cloud_points(filtered) == [(2.0, 0.0, 0.0)]


def test_map_filter_initializes_configured_topics(monkeypatch):
    """MapFilter should wire its default topics through rclpy publishers."""
    patch_node_methods(monkeypatch)

    node = map_filter.MapFilter()

    assert node._node_name == "map_filter"
    assert node._publishers[0][1] == "/map_valid"
    assert node._subscriptions[0].topic == "/map"
    assert node._subscriptions[0].callback == node._handle_map


def test_map_filter_drops_empty_maps():
    """Empty maps should be counted but not published."""
    node = map_filter.MapFilter.__new__(map_filter.MapFilter)
    node._dropped_empty = 0
    node._published_first = False
    logger = RecordingLogger()
    node.get_logger = lambda: logger
    node._publisher = SimpleNamespace(published=[], publish=lambda msg: node._publisher.published.append(msg))

    node._handle_map(make_grid(0, 0))

    assert node._dropped_empty == 1
    assert node._publisher.published == []

    node._handle_map(make_grid(0, 3))
    node._handle_map(make_grid(3, 0))

    assert node._dropped_empty == 3
    assert len(logger.warnings) == 1


def test_map_filter_publishes_non_empty_maps():
    """Valid maps should be forwarded exactly once per input message."""
    node = map_filter.MapFilter.__new__(map_filter.MapFilter)
    node._dropped_empty = 0
    node._published_first = False
    logger = RecordingLogger()
    node.get_logger = lambda: logger
    node._publisher = SimpleNamespace(published=[], publish=lambda msg: node._publisher.published.append(msg))
    msg = make_grid(4, 3)

    node._handle_map(msg)

    assert node._publisher.published == [msg]
    assert node._published_first is True

    second_msg = make_grid(5, 2)
    node._handle_map(second_msg)

    assert node._publisher.published == [msg, second_msg]
    assert len(logger.infos) == 1


def test_map_monitor_initializes_subscription_and_timer(monkeypatch):
    """MapMonitor should subscribe to the filtered map and create a status timer."""
    patch_node_methods(monkeypatch)

    node = map_monitor.MapMonitor()

    assert node._node_name == "map_monitor"
    assert node._subscriptions[0].topic == "/map_valid"
    assert node._subscriptions[0].callback == node._handle_map
    assert node._timers[0].period == 5.0


def test_map_monitor_tracks_empty_valid_and_repeated_maps():
    """MapMonitor should warn on empty maps and log only the first valid map."""
    node = map_monitor.MapMonitor.__new__(map_monitor.MapMonitor)
    node._received_map = False
    node._seconds_waited = 0
    node._logger = RecordingLogger()
    node.get_logger = lambda: node._logger
    node.get_parameter = lambda name: SimpleNamespace(value="/map_valid")

    node._handle_map(make_grid(0, 0))
    node._handle_map(make_grid(2, 0))

    assert node._received_map is False
    assert len(node._logger.warnings) == 2

    node._handle_map(make_grid(3, 2))
    node._handle_map(make_grid(4, 4))

    assert node._received_map is True
    assert len(node._logger.infos) == 1
    assert "Received /map_valid" in node._logger.infos[0]


def test_map_monitor_status_warning_stops_after_valid_map():
    """Status reports should repeat until a usable map arrives."""
    node = map_monitor.MapMonitor.__new__(map_monitor.MapMonitor)
    node._received_map = False
    node._seconds_waited = 0
    node._logger = RecordingLogger()
    node.get_logger = lambda: node._logger

    node._report_status()
    node._report_status()

    assert node._seconds_waited == 10
    assert len(node._logger.warnings) == 2

    node._received_map = True
    node._report_status()

    assert node._seconds_waited == 10
    assert len(node._logger.warnings) == 2


def test_odom_to_tf_initializes_subscription(monkeypatch):
    """OdomToTf should create a broadcaster and subscribe to /odom."""
    patch_node_methods(monkeypatch)

    class FakeBroadcaster:
        def __init__(self, node):
            self.node = node

        def sendTransform(self, transform):
            self.transform = transform

    monkeypatch.setattr(odom_to_tf, "TransformBroadcaster", FakeBroadcaster)

    node = odom_to_tf.OdomToTf()

    assert node._node_name == "odom_to_tf"
    assert isinstance(node._broadcaster, FakeBroadcaster)
    assert node._subscriptions[0].topic == "/odom"
    assert node._subscriptions[0].callback == node._handle_odom


def test_odom_to_tf_publishes_transform_and_logs_once():
    """OdomToTf should translate odometry pose fields into a TF transform."""
    from nav_msgs.msg import Odometry

    node = odom_to_tf.OdomToTf.__new__(odom_to_tf.OdomToTf)
    node._sent_transforms = []
    node._broadcaster = SimpleNamespace(sendTransform=node._sent_transforms.append)
    node._logged_first_odom = False
    node._logger = RecordingLogger()
    node.get_logger = lambda: node._logger
    msg = Odometry()
    msg.header.frame_id = "odom_source"
    msg.child_frame_id = "base_source"
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 3.0
    msg.pose.pose.orientation.w = 1.0

    node._handle_odom(msg)
    node._handle_odom(msg)

    transform = node._sent_transforms[0]
    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_link"
    assert transform.transform.translation.x == 1.0
    assert transform.transform.translation.y == 2.0
    assert transform.transform.translation.z == 3.0
    assert transform.transform.rotation.w == 1.0
    assert len(node._sent_transforms) == 2
    assert len(node._logger.infos) == 1


def test_main_functions_spin_and_shutdown(monkeypatch):
    """Each executable main should initialize, spin, destroy, and shut down."""
    for module, class_name in (
        (astar_path_publisher, "AstarPathPublisher"),
        (cloud_filter, "CloudFilter"),
        (map_filter, "MapFilter"),
        (map_monitor, "MapMonitor"),
        (odom_to_tf, "OdomToTf"),
    ):
        events = []
        fake_node = SimpleNamespace(destroy_node=lambda: events.append("destroy"))
        monkeypatch.setattr(module.rclpy, "init", lambda args=None: events.append(("init", args)))
        monkeypatch.setattr(module.rclpy, "spin", lambda node: events.append(("spin", node)))
        monkeypatch.setattr(module.rclpy, "shutdown", lambda: events.append("shutdown"))
        monkeypatch.setattr(module, class_name, lambda: fake_node)

        module.main(["--ros-args"])

        assert events == [
            ("init", ["--ros-args"]),
            ("spin", fake_node),
            "destroy",
            "shutdown",
        ]
