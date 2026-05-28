"""Subsystem contract tests for mapping, motion, and optional camera plumbing."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def read_asset(relative_path):
    """Read a repository asset as UTF-8 text."""
    return (ROOT / relative_path).read_text(encoding="utf-8")


def robot_model():
    """Parse the robot SDF model."""
    return ET.parse(ROOT / "robot" / "robot.sdf").getroot()


def plugin_by_filename(model, filename):
    """Return the first plugin with the requested filename."""
    for plugin in model.findall(".//plugin"):
        if plugin.attrib.get("filename") == filename:
            return plugin
    raise AssertionError(f"Missing plugin {filename}")


def test_lidar_filter_mapper_topic_contract():
    """The lidar, filter, mapper, and RViz should agree on cloud topics."""
    model = robot_model()
    launch_text = read_asset("launch/slam.launch.py")
    octomap_text = read_asset("config/octomap_server.yaml")
    rviz_text = read_asset("rviz/slam.rviz")

    lidar = model.find(".//sensor[@name='gpu_3d_lidar']")
    assert lidar is not None
    assert lidar.findtext("topic") == "/points_raw"

    assert "/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" in launch_text
    assert '("/points_raw/points", "/points_raw")' in launch_text
    assert '"input_topic": "/points_raw"' in launch_text
    assert '"output_topic": "/points_filtered"' in launch_text
    assert '("cloud_in", "/points_filtered")' in launch_text
    assert "filter_ground_plane: false" in octomap_text
    assert "Value: /points_filtered" in rviz_text


def test_motion_subsystem_uses_3d_velocity_and_odometry():
    """Teleop, bridge, VelocityControl, and odometry should form one 3D motion path."""
    model = robot_model()
    launch_text = read_asset("launch/slam.launch.py")
    teleop_text = read_asset("config/gazebo_teleop.config")

    velocity_control = plugin_by_filename(model, "gz-sim-velocity-control-system")
    odometry = plugin_by_filename(model, "gz-sim-odometry-publisher-system")

    assert model.findtext(".//link[@name='base_link']/gravity") == "false"
    assert velocity_control.findtext("topic") == "/cmd_vel"
    assert "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" in launch_text
    assert "<topic>/cmd_vel</topic>" in teleop_text
    assert odometry.findtext("odom_topic") == "/odom"
    assert odometry.findtext("dimensions") == "3"


def test_camera_subsystem_is_optional_and_isolated():
    """The high-bandwidth camera bridge should stay out of the default bridge."""
    model = robot_model()
    launch_text = read_asset("launch/slam.launch.py")

    base_bridge_block = launch_text.split("# Camera images are high-bandwidth", maxsplit=1)[0]
    camera = model.find(".//sensor[@name='front_camera']")

    assert camera is not None
    assert camera.findtext("topic") == "/front_camera/image"
    assert camera.findtext("always_on") == "0"
    assert "/front_camera/image" not in base_bridge_block
    assert 'LaunchConfiguration("camera")' in launch_text
    assert 'name="ros_gz_camera_bridge"' in launch_text
    assert "condition=IfCondition(camera_enabled)" in launch_text


def test_helper_nodes_are_installed_and_launched_once():
    """Every Python helper executable should be exposed and launched by name."""
    setup_text = read_asset("setup.py")
    launch_text = read_asset("launch/slam.launch.py")

    for executable in ("cloud_filter", "odom_to_tf", "map_filter", "map_monitor"):
        assert f"{executable} = ros_test.{executable}:main" in setup_text
        assert f'executable="{executable}"' in launch_text
        assert f'name="{executable}"' in launch_text
