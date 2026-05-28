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
    text = Path("launch/slam.launch.py").read_text()

    assert "GZ_SIM_RESOURCE_PATH" in text
    assert "IGN_GAZEBO_RESOURCE_PATH" in text


def test_launch_file_uses_public_slam_name():
    """The installed launch API should be slam.launch.py."""
    assert Path("launch/slam.launch.py").is_file()
    assert sorted(path.name for path in Path("launch").glob("*.launch.py")) == ["slam.launch.py"]


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
    launch_text = Path("launch/slam.launch.py").read_text()
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


def test_camera_bridge_runs_in_sim_at_reduced_resolution():
    """Simulation should start the front camera bridge at 960 x 540."""
    launch_text = Path("launch/slam.launch.py").read_text()
    robot_text = Path("robot/robot.sdf").read_text()

    assert 'LaunchConfiguration("camera")' in launch_text
    assert '"camera",' in launch_text
    assert 'default_value="true"' in launch_text
    assert 'name="ros_gz_camera_bridge"' in launch_text
    assert "condition=IfCondition(camera_in_sim)" in launch_text
    assert "<update_rate>30</update_rate>" in robot_text
    assert "<width>960</width>" in robot_text
    assert "<height>540</height>" in robot_text
    assert "<always_on>1</always_on>" in robot_text


def test_setup_entrypoints_match_nodes():
    """setup.py should expose all installed ROS helper node entrypoints."""
    text = Path("setup.py").read_text()

    for entrypoint in ("astar_path_publisher", "cloud_filter", "map_filter", "map_monitor", "odom_to_tf"):
        assert f"{entrypoint} = ros_test.{entrypoint}:main" in text


def test_saved_octomap_planning_assets_are_documented_for_rviz():
    """The .bt-to-A* helper should be installable and visible in RViz docs/assets."""
    readme_text = Path("README.md").read_text()
    rviz_text = Path("rviz/slam.rviz").read_text()
    package_text = Path("package.xml").read_text()

    assert "bt_file_to_numpy_grid" in Path("pathfinding/octomap_grid.py").read_text()
    assert "astar_path_publisher" in readme_text
    assert ".bt -> NumPy grid -> A* -> nav_msgs/Path" in readme_text
    assert "Value: /astar_path" in rviz_text
    assert "<exec_depend>python3-numpy</exec_depend>" in package_text
