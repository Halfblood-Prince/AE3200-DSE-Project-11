# ROS 2 Lyrical Gazebo Lidar SLAM Simulation

This package launches a Gazebo Jetty simulation of a differential-drive lidar robot inside a large perimeter-bounded space with sparse SLAM landmarks, bridges the simulated sensor data into ROS 2, runs the built-in mapper by default, and opens RViz.

## Install dependencies

On Ubuntu 26.04, enable the ROS 2 Lyrical apt source first if it is not already configured:

```bash
sudo apt update
sudo apt install -y curl software-properties-common
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-testing-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-testing-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-testing-apt-source.deb
```

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions ros-lyrical-ament-cmake ros-lyrical-rclcpp ros-lyrical-geometry-msgs ros-lyrical-sensor-msgs ros-lyrical-nav-msgs ros-lyrical-ros-gz ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-sim ros-lyrical-rviz2 ros-lyrical-tf2-ros
```

As of May 20, 2026, the Lyrical apt repositories do not provide the Nav2 and
SLAM Toolbox binary packages this project used on Jazzy/Kilted, including
`ros-lyrical-nav2-msgs`, `ros-lyrical-nav2-bringup`, and
`ros-lyrical-slam-toolbox`. The launch defaults therefore use the built-in
simple mapper and keep Nav2 disabled. Once those packages are released for
Lyrical, install them separately and use `mapper:=false`, `nav2:=true`, and
`explore:=true` as needed.

The default build installs shipped binaries from `bin/<arch>` when the bundle
metadata confirms it was built for ROS 2 Lyrical on Ubuntu 26.04, including the
AeroSentinel web server and its bundled Drogon/FFmpeg/libdatachannel/OpenCV
runtime libraries. You do not need to install those web development packages for
a normal `colcon build` once the Lyrical binary workflow has republished the
bundle.

## Build

Clone into a ROS 2 workspace, source Lyrical, then build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --branch ak/ros2 https://github.com/halfblood-prince/ae3200-dse-project-11.git
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If `source /opt/ros/lyrical/setup.bash` fails, install ROS 2 Lyrical and the
dependencies listed above first. The prebuilt binaries in `bin/` avoid compiling
the ROS node executables and the web server, but `colcon build` still needs the
ROS 2 `ament_cmake` environment to install the package.

The ROS nodes and the AeroSentinel web server are C++ executables. By default,
CMake installs the complete prebuilt bundle for the current CPU in `bin/amd_x64`
or `bin/arm_x64`. If that bundle is incomplete, configuration fails with a
missing-binary or stale-binary message instead of installing executables built
for another ROS/Ubuntu target. Maintainers can explicitly rebuild from source with
`colcon build --cmake-args -DROS_TEST_USE_PREBUILT_BINARIES=OFF` after installing
the native development dependencies used by the binary workflow.

## Prebuilt Linux binaries

The GitHub workflow `AeroSentinel Linux Binaries` builds the package for:

- `amd_x64`
- `arm_x64`

On successful non-PR runs, the workflow publishes the generated executables back
to the `ak/ros2` branch under:

```text
bin/amd_x64/
bin/arm_x64/
```

After cloning `ak/ros2`, `colcon build` automatically installs the complete
prebuilt binary set for the current CPU architecture when it is present. The
workflow artifacts are still uploaded to each Actions run for download/debugging,
but a normal clone does not require manually extracting them.

Each folder must contain the complete prebuilt executable set:
`auto_drive`, `map_filter`, `map_monitor`, `odom_to_tf`, `scan_to_chassis`,
`simple_mapper`, and `web_server`, plus a `lib/` directory containing the shared
libraries needed by `web_server`, and a `build-info.env` file with
`ROS_DISTRO=lyrical` and `UBUNTU_CODENAME=resolute`. The
`nav2_waypoint_explorer` executable is optional and is built only when Nav2
packages are available and CMake is configured with
`-DROS_TEST_BUILD_NAV2_EXPLORER=ON`.

## Launch

```bash
ros2 launch ros_test gazebo_slam.launch.py
```

Gazebo opens with a floating Teleop panel, RViz opens with `/scan`, TF, and `/map` displays, and the web dashboard opens manual control over `/cmd_vel`. Nav2 autonomous exploration is detached by default for now.

The launch also starts the AeroSentinel C++ dashboard at `http://127.0.0.1:8080/mission/alpha-0426` and binds it to `0.0.0.0` by default. The default development login is `admin` / `admin`.

The launch starts:

- Gazebo Jetty world: `robot.sdf` with an outer perimeter and sparse non-wall landmarks
- Gazebo Teleop GUI panel
- ROS-Gazebo bridge for `/clock`, `/scan_raw`, `/front_camera/image`, `/imu`, `/odom`, and ROS `/cmd_vel`
- 1920x1080 front camera mounted on the robot chassis at 60 FPS
- scan republisher from `/scan_raw` to `/scan` with frame `lidar_link`
- static TF from `base_link` to `lidar_link`, matching the lidar pose in `robot.sdf`
- `odom_to_tf`, publishing `odom -> base_link`
- built-in `simple_mapper`, publishing `/map` from odometry and scan data
- `map_filter`, republishing only non-empty SLAM maps as `/map_valid`
- RViz
- `map_monitor`, which reports when `/map` is received
- AeroSentinel C++ dashboard on port `8080`, displaying the front camera feed as a source-encoded H.264 WebRTC video track and publishing manual keyboard commands to `/cmd_vel`
- Nav2 navigation servers, costmaps, behavior tree navigator, waypoint follower, and map saver only when `nav2:=true`
- optional `nav2_waypoint_explorer` only when it has been built and both `nav2:=true` and `explore:=true`

## Manual and Fallback Modes

Nav2 and SLAM Toolbox are disabled by default on Lyrical until their binary
packages are available. To use SLAM Toolbox after installing it:

```bash
ros2 launch ros_test gazebo_slam.launch.py mapper:=false
```

To reattach the navigation stack after installing Nav2:

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

To keep using the built-in odom/scan mapper explicitly:

```bash
ros2 launch ros_test gazebo_slam.launch.py nav2:=false mapper:=true
```

To disable the C++ dashboard or run it on another port:

```bash
ros2 launch ros_test gazebo_slam.launch.py web:=false
ros2 launch ros_test gazebo_slam.launch.py web_port:=8081
```

The dashboard publishes manual drive commands with keyboard input and the on-screen D-pad at about 25 Hz while keys are held. The default speed limits are `AEROSENTINEL_MAX_LINEAR=2.5` and `AEROSENTINEL_MAX_ANGULAR=1.8`.

To keep the dashboard bound to localhost only:

```bash
ros2 launch ros_test gazebo_slam.launch.py web_bind_address:=127.0.0.1
```

The launch file sets dashboard credentials explicitly. The defaults are
`admin` / `admin`. Choose different credentials before exposing the dashboard on
a network:

```bash
ros2 launch ros_test gazebo_slam.launch.py web_user:=operator web_password:=change-me
```

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

When SLAM Toolbox is installed and enabled with `mapper:=false`, RViz may briefly show pose corrections because `slam_toolbox` updates the `map -> odom` transform. Loop closure is enabled, and the optional explorer deliberately returns near the start pose before saving so the final map is optimized before pathfinding use.
