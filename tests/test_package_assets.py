"""Tests for package metadata and Gazebo asset consistency."""

import xml.etree.ElementTree as ET
from pathlib import Path


def test_robot_world_includes_split_robot_model():
    """The world file should include the separate robot model URI."""
    world = ET.parse("robot/environment.world")
    uri_values = [element.text for element in world.findall(".//include/uri")]

    assert "model://robot" in uri_values


def test_robot_model_config_points_to_robot_sdf():
    """The Gazebo model manifest should resolve model://robot to robot.sdf."""
    model_config = ET.parse("robot/model.config")

    assert model_config.findtext("name") == "robot"
    assert model_config.findtext("sdf") == "robot.sdf"


def test_launch_adds_package_share_to_gazebo_resource_path():
    """Gazebo should be able to resolve model://robot after installation."""
    text = Path("launch/gazebo_slam.launch.py").read_text()

    assert "GZ_SIM_RESOURCE_PATH" in text
    assert "IGN_GAZEBO_RESOURCE_PATH" in text


def test_robot_uses_sliding_velocity_control_and_no_diffdrive():
    """The robot should move with VelocityControl instead of wheel-based DiffDrive."""
    text = Path("robot/robot.sdf").read_text()

    assert "gz-sim-velocity-control-system" in text
    assert "gz-sim-odometry-publisher-system" in text
    assert "DiffDrive" not in text
    assert "wheel" not in text.lower()


def test_setup_entrypoints_match_nodes():
    """setup.py should expose all installed ROS helper node entrypoints."""
    text = Path("setup.py").read_text()

    for entrypoint in ("map_filter", "map_monitor", "odom_to_tf"):
        assert f"{entrypoint} = ros_test.{entrypoint}:main" in text
