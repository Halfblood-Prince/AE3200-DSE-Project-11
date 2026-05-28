"""System-level static tests for the Gazebo, bridge, mapping, and RViz stack."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read_asset(relative_path):
    """Read a repository asset as UTF-8 text."""
    return (ROOT / relative_path).read_text(encoding="utf-8")


def launch_argument_default(launch_text, name):
    """Return the string default value for a DeclareLaunchArgument block."""
    pattern = re.compile(
        r"DeclareLaunchArgument\(\s*"
        rf'"{re.escape(name)}",\s*'
        r'default_value="([^"]*)"',
        re.MULTILINE,
    )
    match = pattern.search(launch_text)
    assert match is not None, f"Missing launch argument {name}"
    return match.group(1)


def test_world_loads_required_gazebo_systems_and_robot_model():
    """The world should load the robot and Gazebo systems required by the stack."""
    world = ET.parse(ROOT / "robot" / "environment.world").getroot()
    plugin_filenames = {plugin.attrib.get("filename") for plugin in world.findall(".//plugin")}
    include_uris = [element.text for element in world.findall(".//include/uri")]

    assert "model://robot" in include_uris
    assert {
        "gz-sim-physics-system",
        "gz-sim-user-commands-system",
        "gz-sim-scene-broadcaster-system",
        "gz-sim-sensors-system",
        "gz-sim-imu-system",
    }.issubset(plugin_filenames)


def test_launch_defaults_keep_real_mode_safe_and_sim_opt_in():
    """Default launch arguments should avoid starting simulation-only pieces unexpectedly."""
    launch_text = read_asset("launch/gazebo_slam.launch.py")

    assert launch_argument_default(launch_text, "run") == "real"
    assert launch_argument_default(launch_text, "rum") == ""
    assert launch_argument_default(launch_text, "gazebo_gui") == "true"
    assert launch_argument_default(launch_text, "camera") == "false"
    assert launch_argument_default(launch_text, "odom_tf") == "true"
    assert launch_argument_default(launch_text, "lidar_tf") == "true"
    assert launch_argument_default(launch_text, "mapper") == "true"
    assert launch_argument_default(launch_text, "map_odom_tf") == "true"
    assert launch_argument_default(launch_text, "rviz") == "true"


def test_launch_orders_transport_tf_mapping_and_visualization():
    """Startup order should bring up bridges/TF before mapping and RViz."""
    launch_text = read_asset("launch/gazebo_slam.launch.py")
    launch_actions = launch_text.split("return LaunchDescription(", maxsplit=1)[1]

    points_bridge_index = launch_actions.index("points_bridge,")
    cloud_filter_index = launch_actions.index("cloud_filter,")
    odom_tf_index = launch_actions.index("odom_to_tf,")
    map_tf_index = launch_actions.index("map_to_odom_static_tf,")
    mapper_timer_index = launch_actions.index("actions=[octomap_server, map_filter, map_monitor]")
    rviz_timer_index = launch_actions.index("actions=[rviz]")

    assert points_bridge_index < cloud_filter_index < odom_tf_index < map_tf_index
    assert map_tf_index < mapper_timer_index < rviz_timer_index
    assert "TimerAction(\n                period=2.0,\n                actions=[octomap_server, map_filter, map_monitor]" in launch_text
    assert "TimerAction(period=8.0, actions=[rviz])" in launch_text


def test_system_topic_inventory_is_documented_and_backed_by_sources():
    """README expected topics should match topics produced by launch/config/assets."""
    readme_text = read_asset("README.md")
    launch_text = read_asset("launch/gazebo_slam.launch.py")
    robot_text = read_asset("robot/robot.sdf")
    octomap_text = read_asset("config/octomap_server.yaml")

    expected_topics = [
        "/points_raw",
        "/points_filtered",
        "/front_camera/image",
        "/imu",
        "/odom",
        "/occupied_cells_vis_array",
        "/octomap_binary",
        "/octomap_full",
        "/map",
        "/map_valid",
        "/cmd_vel",
    ]
    for topic in expected_topics:
        assert topic in readme_text

    for source, topics in (
        (launch_text, ["/points_filtered", "/map_valid", "/cmd_vel"]),
        (robot_text, ["/points_raw", "/front_camera/image", "/imu", "/odom", "/cmd_vel"]),
        (octomap_text, ["frame_id: map", "base_frame_id: base_link"]),
    ):
        for topic in topics:
            assert topic in source
