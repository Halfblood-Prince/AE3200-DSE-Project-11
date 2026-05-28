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


def test_robot_uses_hover_velocity_control_and_no_diffdrive():
    """The robot should move in 3D with VelocityControl instead of wheel drive."""
    text = Path("robot/robot.sdf").read_text()

    assert "gz-sim-velocity-control-system" in text
    assert "gz-sim-odometry-publisher-system" in text
    assert "<gravity>false</gravity>" in text
    assert "<dimensions>3</dimensions>" in text
    assert "DiffDrive" not in text
    assert "wheel" not in text.lower()


def test_launch_and_rviz_use_filtered_point_cloud():
    """OctoMap and RViz should consume the self-filtered cloud."""
    launch_text = Path("launch/gazebo_slam.launch.py").read_text()
    rviz_text = Path("rviz/slam.rviz").read_text()

    assert "cloud_filter" in launch_text
    assert '("cloud_in", "/points_filtered")' in launch_text
    assert "Value: /points_filtered" in rviz_text


def test_lidar_uses_required_scan_resolution():
    """The simulated lidar should keep the expected 1024 x 16 scan pattern."""
    text = Path("robot/robot.sdf").read_text()

    assert "<samples>1024</samples>" in text
    assert "<samples>16</samples>" in text


def test_octomap_disables_redundant_ground_segmentation():
    """The filtered cloud should avoid noisy PCL ground-plane warnings."""
    text = Path("config/octomap_server.yaml").read_text()

    assert "filter_ground_plane: false" in text
    assert "ground_filter:" not in text


def test_camera_bridge_is_opt_in_to_limit_transport_load():
    """High-bandwidth camera images should not be bridged by default."""
    launch_text = Path("launch/gazebo_slam.launch.py").read_text()
    robot_text = Path("robot/robot.sdf").read_text()

    assert 'LaunchConfiguration("camera")' in launch_text
    assert '"camera",' in launch_text
    assert 'default_value="false"' in launch_text
    assert 'name="ros_gz_camera_bridge"' in launch_text
    assert "<update_rate>30</update_rate>" in robot_text
    assert "<always_on>0</always_on>" in robot_text


def test_setup_entrypoints_match_nodes():
    """setup.py should expose all installed ROS helper node entrypoints."""
    text = Path("setup.py").read_text()

    for entrypoint in ("cloud_filter", "map_filter", "map_monitor", "odom_to_tf"):
        assert f"{entrypoint} = ros_test.{entrypoint}:main" in text
