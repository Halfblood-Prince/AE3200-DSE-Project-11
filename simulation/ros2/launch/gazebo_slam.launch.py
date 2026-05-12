from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ros_test")
    world = LaunchConfiguration("world")
    gui_config = LaunchConfiguration("gui_config")
    gz_args = LaunchConfiguration("gz_args")
    auto_drive_enabled = LaunchConfiguration("auto_drive")
    mapper = LaunchConfiguration("mapper")
    nav2_enabled = LaunchConfiguration("nav2")
    explore_enabled = LaunchConfiguration("explore")
    web_enabled = LaunchConfiguration("web")
    web_port = LaunchConfiguration("web_port")
    web_bind_address = LaunchConfiguration("web_bind_address")

    default_world = PathJoinSubstitution([pkg_share, "robot.sdf"])
    default_gui_config = PathJoinSubstitution([pkg_share, "config", "gazebo_teleop.config"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "slam.rviz"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [gz_args, " ", world, " --gui-config ", gui_config],
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
    )

    odom_to_tf = Node(
        package="ros_test",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    scan_to_chassis = Node(
        package="ros_test",
        executable="scan_to_chassis",
        name="scan_to_chassis",
        output="screen",
        parameters=[{"use_sim_time": True}],
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
        parameters=[{"use_sim_time": True}],
    )

    simple_mapper = Node(
        package="ros_test",
        executable="simple_mapper",
        name="simple_mapper",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(mapper),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "true",
        }.items(),
        condition=UnlessCondition(mapper),
    )

    map_filter = Node(
        package="ros_test",
        executable="map_filter",
        name="map_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/map",
                "output_topic": "/map_valid",
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_map_server",
            executable="map_saver_server",
            name="map_saver",
            output="screen",
            parameters=[nav2_params],
            condition=IfCondition(nav2_enabled),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
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
                "use_sim_time": True,
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
        condition=IfCondition(
            PythonExpression(["'", nav2_enabled, "' == 'true' and '", explore_enabled, "' == 'true'"])
        ),
    )

    map_monitor = Node(
        package="ros_test",
        executable="map_monitor",
        name="map_monitor",
        output="screen",
        parameters=[{"use_sim_time": True, "map_topic": "/map_valid"}],
    )

    auto_drive = Node(
        package="ros_test",
        executable="auto_drive",
        name="auto_drive",
        output="screen",
        parameters=[{"use_sim_time": True}],
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
        },
        condition=IfCondition(web_enabled),
    )

    return LaunchDescription(
        [
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
                "auto_drive",
                default_value="false",
                description="Set true to make the robot drive itself.",
            ),
            DeclareLaunchArgument(
                "mapper",
                default_value="false",
                description="Set true to use the simple odom mapper instead of slam_toolbox.",
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
                "web",
                default_value="true",
                description="Set false to disable the AeroSentinel Flask web dashboard.",
            ),
            DeclareLaunchArgument(
                "web_port",
                default_value="8080",
                description="Port for the AeroSentinel Flask web dashboard.",
            ),
            DeclareLaunchArgument(
                "web_bind_address",
                default_value="0.0.0.0",
                description="Bind address for the AeroSentinel Flask web dashboard.",
            ),
            web_server,
            gazebo,
            bridge,
            odom_to_tf,
            scan_to_chassis,
            lidar_static_tf,
            TimerAction(
                period=2.0,
                actions=[slam_toolbox, simple_mapper, map_filter, map_monitor, auto_drive],
            ),
            TimerAction(period=8.0, actions=[rviz]),
            TimerAction(period=12.0, actions=nav2_nodes),
            TimerAction(period=25.0, actions=[nav2_explorer]),
        ]
    )
