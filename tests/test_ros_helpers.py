"""Unit tests for ROS helper logic that can run in a ROS environment."""

from types import SimpleNamespace
import struct

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs.msg")
pytest.importorskip("sensor_msgs.msg")

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from ros_test import cloud_filter, map_filter, map_monitor, odom_to_tf


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

    for module in (cloud_filter, map_filter, map_monitor, odom_to_tf):
        monkeypatch.setattr(module.Node, "__init__", node_init)
        monkeypatch.setattr(module.Node, "declare_parameter", declare_parameter)
        monkeypatch.setattr(module.Node, "get_parameter", get_parameter)
        monkeypatch.setattr(module.Node, "create_publisher", create_publisher)
        monkeypatch.setattr(module.Node, "create_subscription", create_subscription)
        monkeypatch.setattr(module.Node, "create_timer", create_timer)
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

    assert filter_qos.depth == 1
    assert monitor_qos.depth == 1
    assert filter_qos.durability == monitor_qos.durability
    assert filter_qos.reliability == monitor_qos.reliability


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
