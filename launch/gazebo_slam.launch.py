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
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


class SimulationMode(Substitution):
    """Return true when either run:=sim or the typo-compatible rum:=sim is set."""

    def __init__(self, run_mode, rum_mode):
        super().__init__()
        self._run_mode = run_mode
        self._rum_mode = rum_mode

    def describe(self):
        return "run mode is simulation"

    def perform(self, context):
        run_value = self._run_mode.perform(context)
        rum_value = self._rum_mode.perform(context)
        is_sim = str(run_value or "").strip().lower() == "sim"
        is_sim = is_sim or str(rum_value or "").strip().lower() == "sim"
        return "true" if is_sim else "false"


class AllTrue(Substitution):
    """Return true only when all launch configurations are truthy strings."""

    def __init__(self, *launch_configurations):
        super().__init__()
        self._launch_configurations = launch_configurations

    def describe(self):
        return "all launch configurations are true"

    def perform(self, context):
        for launch_configuration in self._launch_configurations:
            if not _launch_bool(launch_configuration.perform(context)):
                return "false"
        return "true"


class GazeboArgs(Substitution):
    """Build gz sim arguments with optional GUI disabled."""

    def __init__(self, gz_args, world, gui_config, gazebo_gui):
        super().__init__()
        self._gz_args = gz_args
        self._world = world
        self._gui_config = gui_config
        self._gazebo_gui = gazebo_gui

    def describe(self):
        return "Gazebo command-line arguments"

    def perform(self, context):
        gz_args = str(self._gz_args.perform(context) or "").strip()
        world = str(self._world.perform(context) or "").strip()
        gui_enabled = _launch_bool(self._gazebo_gui.perform(context))

        if gui_enabled:
            gui_config = str(self._gui_config.perform(context) or "").strip()
            return f"{gz_args} {world} --gui-config {gui_config}".strip()

        return f"{gz_args} -s {world}".strip()


def generate_launch_description():
    pkg_share = FindPackageShare("ros_test")
    run_mode = LaunchConfiguration("run")
    rum_mode = LaunchConfiguration("rum")
    world = LaunchConfiguration("world")
    gui_config = LaunchConfiguration("gui_config")
    gz_args = LaunchConfiguration("gz_args")
    gazebo_gui_enabled = LaunchConfiguration("gazebo_gui")
    auto_drive_enabled = LaunchConfiguration("auto_drive")
    odom_tf_enabled = LaunchConfiguration("odom_tf")
    lidar_tf_enabled = LaunchConfiguration("lidar_tf")
    mapper = LaunchConfiguration("mapper")
    map_odom_tf_enabled = LaunchConfiguration("map_odom_tf")
    nav2_enabled = LaunchConfiguration("nav2")
    explore_enabled = LaunchConfiguration("explore")
    rviz_enabled = LaunchConfiguration("rviz")
    web_enabled = LaunchConfiguration("web")
    web_port = LaunchConfiguration("web_port")
    web_bind_address = LaunchConfiguration("web_bind_address")
    web_user = LaunchConfiguration("web_user")
    web_password = LaunchConfiguration("web_password")

    is_sim = SimulationMode(run_mode, rum_mode)
    mapper_and_map_odom_tf = AllTrue(mapper, map_odom_tf_enabled)
    nav2_and_explore = AllTrue(nav2_enabled, explore_enabled)
    use_sim_time = ParameterValue(is_sim, value_type=bool)

    default_world = PathJoinSubstitution([pkg_share, "robot.sdf"])
    default_gui_config = PathJoinSubstitution([pkg_share, "config", "gazebo_teleop.config"])
    octomap_params = PathJoinSubstitution([pkg_share, "config", "octomap_server.yaml"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "slam.rviz"])
    gazebo_args = GazeboArgs(gz_args, world, gui_config, gazebo_gui_enabled)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": gazebo_args,
        }.items(),
        condition=IfCondition(is_sim),
    )

    sim_mode_log = LogInfo(
        msg="ros_test launch mode: simulation (Gazebo, ROS-Gazebo bridge, simulated clock)",
        condition=IfCondition(is_sim),
    )

    real_mode_log = LogInfo(
        msg="ros_test launch mode: real robot (hardware topics, wall clock, no Gazebo bridge)",
        condition=UnlessCondition(is_sim),
    )

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

    odom_to_tf = Node(
        package="ros_test",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(odom_tf_enabled),
    )

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
        condition=IfCondition(mapper_and_map_odom_tf),
    )

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_enabled),
    )

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_map_server",
            executable="map_saver_server",
            name="map_saver",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "map_saver",
                    ],
                }
            ],
            condition=IfCondition(nav2_enabled),
        ),
    ]

    nav2_explorer = Node(
        package="ros_test",
        executable="nav2_waypoint_explorer",
        name="nav2_waypoint_explorer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_topic": "/map_valid",
                "map_save_path": "maps/complete_environment",
                "min_exploration_goals": 10,
                "frontier_timeout_sec": 45.0,
                "initial_scan_sec": 10.0,
                "frontier_sample_step_m": 0.35,
                "frontier_clearance_m": 0.45,
                "frontier_min_distance_m": 3.0,
                "frontier_max_distance_m": 18.0,
                "return_to_start": True,
            }
        ],
        condition=IfCondition(nav2_and_explore),
    )

    map_monitor = Node(
        package="ros_test",
        executable="map_monitor",
        name="map_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "map_topic": "/map_valid"}],
        condition=IfCondition(mapper),
    )

    auto_drive = Node(
        package="ros_test",
        executable="auto_drive",
        name="auto_drive",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(auto_drive_enabled),
    )

    web_server = Node(
        package="ros_test",
        executable="web_server",
        name="aerosentinel_web",
        output="screen",
        additional_env={
            "PORT": web_port,
            "AEROSENTINEL_BIND_ADDRESS": web_bind_address,
            "AEROSENTINEL_USER": web_user,
            "AEROSENTINEL_PASSWORD": web_password,
            "AEROSENTINEL_PUBLIC_DIR": PathJoinSubstitution([pkg_share, "website", "public"]),
        },
        condition=IfCondition(web_enabled),
    )

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
                description="Gazebo SDF world to load.",
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
                "auto_drive",
                default_value="false",
                description="Set true to make the robot drive itself.",
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
                "nav2",
                default_value="false",
                description="Set false to disable the Nav2 navigation stack.",
            ),
            DeclareLaunchArgument(
                "explore",
                default_value="false",
                description="Set false to keep Nav2 ready but disable autonomous exploration.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Set false to disable RViz.",
            ),
            DeclareLaunchArgument(
                "web",
                default_value="true",
                description="Set false to disable the AeroSentinel C++ web dashboard.",
            ),
            DeclareLaunchArgument(
                "web_port",
                default_value="8080",
                description="Port for the AeroSentinel C++ web dashboard.",
            ),
            DeclareLaunchArgument(
                "web_bind_address",
                default_value="0.0.0.0",
                description="Bind address for the AeroSentinel C++ web dashboard.",
            ),
            DeclareLaunchArgument(
                "web_user",
                default_value="admin",
                description="Username for the AeroSentinel C++ web dashboard.",
            ),
            DeclareLaunchArgument(
                "web_password",
                default_value="admin",
                description="Password for the AeroSentinel C++ web dashboard.",
            ),
            sim_mode_log,
            real_mode_log,
            web_server,
            gazebo,
            bridge,
            points_bridge,
            odom_to_tf,
            lidar_static_tf,
            map_to_odom_static_tf,
            TimerAction(
                period=2.0,
                actions=[octomap_server, map_filter, map_monitor, auto_drive],
            ),
            TimerAction(period=8.0, actions=[rviz]),
            TimerAction(period=12.0, actions=nav2_nodes),
            TimerAction(period=25.0, actions=[nav2_explorer]),
        ]
    )
