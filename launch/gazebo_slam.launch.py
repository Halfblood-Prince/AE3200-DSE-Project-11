"""Launch Gazebo, ROS bridges, OctoMap mapping, RViz, and helper nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_bool(value):
    """Interpret common launch argument strings as booleans."""
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


class SimulationMode(Substitution):
    """Return true when either run:=sim or the typo-compatible rum:=sim is set."""

    def __init__(self, run_mode, rum_mode):
        """Store launch configurations for both supported mode arguments."""
        super().__init__()
        self._run_mode = run_mode
        self._rum_mode = rum_mode

    def describe(self):
        """Describe this substitution in launch diagnostics."""
        return "run mode is simulation"

    def perform(self, context):
        """Evaluate run/rum values inside the current launch context."""
        run_value = self._run_mode.perform(context)
        rum_value = self._rum_mode.perform(context)
        is_sim = str(run_value or "").strip().lower() == "sim"
        is_sim = is_sim or str(rum_value or "").strip().lower() == "sim"
        return "true" if is_sim else "false"


class GazeboArgs(Substitution):
    """Build gz sim arguments with optional GUI disabled."""

    def __init__(self, gz_args, world, gui_config, gazebo_gui):
        """Store the arguments needed to compose the gz sim command line."""
        super().__init__()
        self._gz_args = gz_args
        self._world = world
        self._gui_config = gui_config
        self._gazebo_gui = gazebo_gui

    def describe(self):
        """Describe this substitution in launch diagnostics."""
        return "Gazebo command-line arguments"

    def perform(self, context):
        """Return the final gz_args string for GUI or server-only mode."""
        gz_args = str(self._gz_args.perform(context) or "").strip()
        world = str(self._world.perform(context) or "").strip()
        gui_enabled = _launch_bool(self._gazebo_gui.perform(context))

        if gui_enabled:
            gui_config = str(self._gui_config.perform(context) or "").strip()
            return f"{gz_args} {world} --gui-config {gui_config}".strip()

        return f"{gz_args} -s {world}".strip()


def generate_launch_description():
    """Create the full launch graph for real robot and simulation modes."""
    # Launch configurations keep every user-facing option in one place.
    pkg_share = FindPackageShare("ros_test")
    run_mode = LaunchConfiguration("run")
    rum_mode = LaunchConfiguration("rum")
    world = LaunchConfiguration("world")
    gui_config = LaunchConfiguration("gui_config")
    gz_args = LaunchConfiguration("gz_args")
    gazebo_gui_enabled = LaunchConfiguration("gazebo_gui")
    odom_tf_enabled = LaunchConfiguration("odom_tf")
    lidar_tf_enabled = LaunchConfiguration("lidar_tf")
    mapper = LaunchConfiguration("mapper")
    map_odom_tf_enabled = LaunchConfiguration("map_odom_tf")
    rviz_enabled = LaunchConfiguration("rviz")

    # Derived substitutions combine simple launch flags into reusable conditions.
    is_sim = SimulationMode(run_mode, rum_mode)
    use_sim_time = ParameterValue(is_sim, value_type=bool)

    # Package-relative assets are resolved after install by FindPackageShare.
    default_world = PathJoinSubstitution([pkg_share, "robot", "environment.world"])
    default_gui_config = PathJoinSubstitution([pkg_share, "config", "gazebo_teleop.config"])
    octomap_params = PathJoinSubstitution([pkg_share, "config", "octomap_server.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "slam.rviz"])
    gazebo_args = GazeboArgs(gz_args, world, gui_config, gazebo_gui_enabled)

    # Gazebo runs only in simulation mode and uses the Teleop GUI config.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gazebo_args}.items(),
        condition=IfCondition(is_sim),
    )

    # One clear log line makes it obvious which mode was selected.
    sim_mode_log = LogInfo(
        msg="ros_test launch mode: simulation (Gazebo, ROS-Gazebo bridge, simulated clock)",
        condition=IfCondition(is_sim),
    )
    real_mode_log = LogInfo(
        msg="ros_test launch mode: real robot (hardware topics, wall clock, no Gazebo bridge)",
        condition=UnlessCondition(is_sim),
    )

    # Bridge clock, camera, IMU, odom, and velocity commands between Gazebo and ROS.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        condition=IfCondition(is_sim),
    )

    # The 3D lidar needs a PointCloudPacked bridge plus a stable lidar frame id.
    points_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_points_bridge",
        output="screen",
        arguments=[
            "/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        parameters=[{"override_frame_id": "lidar_link"}],
        remappings=[("/points_raw/points", "/points_raw")],
        condition=IfCondition(is_sim),
    )

    # Convert /odom into TF so the map, robot base, and sensors share one tree.
    odom_to_tf = Node(
        package="ros_test",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(odom_tf_enabled),
    )

    # The SDF lidar pose is mirrored here as base_link -> lidar_link.
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_tf",
        output="screen",
        arguments=[
            "--x",
            "0.45",
            "--y",
            "0.0",
            "--z",
            "0.32",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(lidar_tf_enabled),
    )

    # OctoMap needs a map frame; this identity transform keeps the demo simple.
    map_to_odom_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(map_odom_tf_enabled),
    )

    # OctoMap consumes the 3D point cloud and publishes both 3D and 2D map outputs.
    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[octomap_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cloud_in", "/points_raw"),
            ("projected_map", "/map"),
        ],
        condition=IfCondition(mapper),
    )

    # Filter avoids sending empty projected maps to downstream tools.
    map_filter = Node(
        package="ros_test",
        executable="map_filter",
        name="map_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/map",
                "output_topic": "/map_valid",
            }
        ],
        condition=IfCondition(mapper),
    )

    # RViz visualizes TF, the raw point cloud, and OctoMap voxels.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_enabled),
    )

    # Monitor prints useful hints when mapping has not produced a valid map yet.
    map_monitor = Node(
        package="ros_test",
        executable="map_monitor",
        name="map_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "map_topic": "/map_valid"}],
        condition=IfCondition(mapper),
    )

    # Timers stagger startup so bridges and transforms exist before consumers start.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run",
                default_value="real",
                description="Launch mode: use run:=sim for Gazebo simulation, otherwise real robot mode.",
            ),
            DeclareLaunchArgument(
                "rum",
                default_value="",
                description="Typo-compatible alias for run; prefer run:=sim.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world file to load.",
            ),
            DeclareLaunchArgument(
                "gui_config",
                default_value=default_gui_config,
                description="Gazebo GUI config with the Teleop panel.",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r",
                description="Arguments passed to gz sim before the world path.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Set false to run Gazebo server-only without the GUI.",
            ),
            DeclareLaunchArgument(
                "odom_tf",
                default_value="true",
                description="Set false if the real robot already publishes odom -> base_link TF.",
            ),
            DeclareLaunchArgument(
                "lidar_tf",
                default_value="true",
                description="Set false if robot_state_publisher already publishes base_link -> lidar_link TF.",
            ),
            DeclareLaunchArgument(
                "mapper",
                default_value="true",
                description="Set false to disable OctoMap mapping.",
            ),
            DeclareLaunchArgument(
                "map_odom_tf",
                default_value="true",
                description="Set false if another mapper/localizer already publishes map -> odom TF.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Set false to disable RViz.",
            ),
            sim_mode_log,
            real_mode_log,
            gazebo,
            bridge,
            points_bridge,
            odom_to_tf,
            lidar_static_tf,
            map_to_odom_static_tf,
            TimerAction(
                period=2.0,
                actions=[octomap_server, map_filter, map_monitor],
            ),
            TimerAction(period=8.0, actions=[rviz]),
        ]
    )
