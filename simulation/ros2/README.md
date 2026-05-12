# ROS 2 Jazzy Gazebo Lidar SLAM Simulation

This package launches a Gazebo Harmonic simulation of a differential-drive lidar robot inside a large perimeter-bounded space with sparse SLAM landmarks, bridges the simulated sensor data into ROS 2, runs `slam_toolbox`, and opens RViz.

## Install dependencies

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-flask python3-opencv ros-jazzy-nav2-bringup ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-rviz2 ros-jazzy-slam-toolbox ros-jazzy-tf2-ros
```

## Build

From the root of your ROS 2 workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch ros_test gazebo_slam.launch.py
```

Gazebo opens with a floating Teleop panel, RViz opens with `/scan`, TF, and `/map` displays, and the web dashboard opens manual control over `/cmd_vel`. Nav2 autonomous exploration is detached by default for now.

The launch also starts the AeroSentinel Flask dashboard at `http://127.0.0.1:8080/mission/alpha-0426` and binds it to `0.0.0.0` by default. The default development login is `admin` / `admin`.

The launch starts:

- Gazebo Harmonic world: `robot.sdf` with an outer perimeter and sparse non-wall landmarks
- Gazebo Teleop GUI panel
- ROS-Gazebo bridge for `/clock`, `/scan_raw`, `/front_camera/image`, `/imu`, `/odom`, and ROS `/cmd_vel`
- 1920x1080 front camera mounted on the robot chassis at 60 FPS
- scan republisher from `/scan_raw` to `/scan` with frame `lidar_link`
- static TF from `base_link` to `lidar_link`, matching the lidar pose in `robot.sdf`
- `odom_to_tf`, publishing `odom -> base_link`
- `slam_toolbox`, publishing `/map` from `/scan` and TF
- `map_filter`, republishing only non-empty SLAM maps as `/map_valid`
- RViz
- `map_monitor`, which reports when `/map` is received
- AeroSentinel Flask dashboard on port `8080`, displaying the front camera feed as an OpenCV-encoded MJPEG stream and publishing manual keyboard commands to `/cmd_vel`
- Nav2 navigation servers, costmaps, behavior tree navigator, waypoint follower, and map saver only when `nav2:=true`
- `nav2_waypoint_explorer` only when both `nav2:=true` and `explore:=true`

## Manual and Fallback Modes

Nav2 is disabled by default. To reattach the navigation stack:

```bash
ros2 launch ros_test gazebo_slam.launch.py nav2:=true
```

To re-enable autonomous exploration:

```bash
ros2 launch ros_test gazebo_slam.launch.py nav2:=true explore:=true
```

For the simple wall-following driver instead of Nav2:

```bash
ros2 launch ros_test gazebo_slam.launch.py nav2:=false auto_drive:=true
```

For the older odom-only fallback mapper:

```bash
ros2 launch ros_test gazebo_slam.launch.py nav2:=false mapper:=true
```

To disable the Flask dashboard or run it on another port:

```bash
ros2 launch ros_test gazebo_slam.launch.py web:=false
ros2 launch ros_test gazebo_slam.launch.py web_port:=8081
```

The dashboard publishes manual drive commands with keyboard input and the on-screen D-pad at about 25 Hz while keys are held. The default speed limits are `AEROSENTINEL_MAX_LINEAR=2.5` and `AEROSENTINEL_MAX_ANGULAR=1.8`.

To keep the dashboard bound to localhost only:

```bash
ros2 launch ros_test gazebo_slam.launch.py web_bind_address:=127.0.0.1
```

Set `AEROSENTINEL_PASSWORD` and `AEROSENTINEL_SECRET_KEY` before exposing the dashboard on a network.

If the RViz map appears to slide with the robot, make sure RViz Global Options uses fixed frame `map`, not `odom`. The robot should move in the map; the map should not move with the robot.

## Expected topics

```text
/scan_raw sensor_msgs/msg/LaserScan from Gazebo
/scan     sensor_msgs/msg/LaserScan with frame lidar_link
/front_camera/image sensor_msgs/msg/Image 1920x1080 RGB camera feed at 60 FPS
/imu     sensor_msgs/msg/Imu
/odom    nav_msgs/msg/Odometry
/map     nav_msgs/msg/OccupancyGrid
/map_valid nav_msgs/msg/OccupancyGrid with empty startup maps filtered out
/cmd_vel geometry_msgs/msg/Twist from the web controls, Gazebo Teleop panel, or other manual publishers
```

Move the robot for a few seconds before expecting a useful map. The simulated lidar range is 5 m, so the map expands as the robot explores nearby rooms and corridors.

During live SLAM, RViz may briefly show pose corrections because `slam_toolbox` updates the `map -> odom` transform. Loop closure is enabled, and the explorer deliberately returns near the start pose before saving so the final map is optimized before pathfinding use.
