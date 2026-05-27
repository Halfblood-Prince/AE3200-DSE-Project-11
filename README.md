# AeroSentinel ROS 2 Lyrical 3D Mapping

This package launches the AeroSentinel robot stack for ROS 2 Lyrical Luth on
Ubuntu 26.04. The ROS helper nodes are Python executables; the web dashboard
remains a Drogon C++ server. The stack uses a 3D lidar point cloud and OctoMap
for mapping. Real robot mode is the default; Gazebo Jetty simulation is enabled
explicitly with `run:=sim`.

The simulation path starts Gazebo, bridges sensor data into ROS 2, builds an
OctoMap from `/points_raw`, and opens RViz with the 3D point cloud and OctoMap
voxel displays. The projected 2D occupancy grid is still published for Nav2 and
other navigation internals, but it is not shown in RViz by default.

## Requirements

- Ubuntu 26.04, codename `resolute`
- ROS 2 Lyrical Luth
- Gazebo Jetty packages from the ROS 2 Lyrical apt repository
- A supported CPU architecture:
  - `amd_x64`
  - `arm_x64`

## Install Dependencies

Enable the ROS 2 Lyrical apt source if it is not already configured:

```bash
sudo apt update
sudo apt install -y curl software-properties-common
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-testing-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-testing-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-testing-apt-source.deb
```

Install the runtime and build dependencies used by the default Python-node and
prebuilt-server package build:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  ros-lyrical-action-msgs \
  ros-lyrical-ament-cmake \
  ros-lyrical-geometry-msgs \
  ros-lyrical-nav-msgs \
  ros-lyrical-nav2-msgs \
  ros-lyrical-octomap-server \
  ros-lyrical-rclcpp \
  ros-lyrical-rclpy \
  ros-lyrical-ros-gz \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim \
  ros-lyrical-rviz2 \
  ros-lyrical-sensor-msgs \
  ros-lyrical-sensor-msgs-py \
  ros-lyrical-tf2-ros
```

Nav2 is optional. The launch file keeps Nav2 disabled by default. Install the
Lyrical Nav2 packages separately before using `nav2:=true` or `explore:=true`.

## Build

Clone the `ros_simulation` branch into a ROS 2 workspace, source Lyrical, and
build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --branch ros_simulation https://github.com/halfblood-prince/ae3200-dse-project-11.git
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

By default, CMake installs the Python ROS nodes from `scripts/` and installs the
shipped Drogon `web_server` executable from `bin/<arch>` instead of compiling
the server locally. Each prebuilt bundle must match:

```text
ROS_DISTRO=lyrical
UBUNTU_CODENAME=resolute
```

If the binary bundle is missing or stale, configuration stops with a clear
missing-binary or stale-binary message.

## Maintainer Source Builds

Use a source build only when editing the Drogon web server or regenerating the
shipped `web_server` binary bundle. Python ROS node changes do not need native
compilation. Install the native development dependencies first:

```bash
sudo apt update
sudo apt install \
  build-essential \
  cmake \
  git \
  libc-ares-dev \
  libavcodec-dev \
  libavutil-dev \
  libbrotli-dev \
  libdrogon-dev \
  libhiredis-dev \
  libjsoncpp-dev \
  default-libmysqlclient-dev \
  libopencv-dev \
  libpq-dev \
  libsqlite3-dev \
  libssl-dev \
  libswscale-dev \
  libx264-dev \
  libyaml-cpp-dev \
  pkg-config \
  uuid-dev \
  zlib1g-dev
```

`web_server` also requires `LibDataChannel` to be findable by CMake. The GitHub
workflow builds `libdatachannel` from source before compiling the package.
Local maintainers can install it from their distro, vcpkg, or source.

Build the package from source with:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DROS_TEST_USE_PREBUILT_BINARIES=OFF
```

After changing `website/src/main.cc`, push to `ros_simulation` and let the
`AeroSentinel Linux Binaries` workflow publish refreshed `bin/amd_x64` and
`bin/arm_x64` bundles back to the branch. Client machines should pull that
binary commit before running the default `colcon build`.

The required prebuilt executable is:

```text
web_server
```

The Python ROS executables installed from source are:

```text
auto_drive
map_filter
map_monitor
nav2_waypoint_explorer
odom_to_tf
```

`nav2_waypoint_explorer` is installed with the package, but it still requires
Nav2 runtime packages and is only launched when both `nav2:=true` and
`explore:=true`.

## Launch

Real robot mode is the default:

```bash
ros2 launch ros_test gazebo_slam.launch.py
```

Real robot mode does not start Gazebo or the ROS-Gazebo bridge. It uses wall
clock time and expects the robot stack to publish:

```text
/odom               nav_msgs/msg/Odometry
/points_raw         sensor_msgs/msg/PointCloud2 with frame lidar_link
/front_camera/image sensor_msgs/msg/Image, when the web camera feed is used
```

It also expects a base controller subscribed to `/cmd_vel`.

If the robot already publishes its own transforms, disable the helper TF nodes:

```bash
ros2 launch ros_test gazebo_slam.launch.py odom_tf:=false lidar_tf:=false map_odom_tf:=false
```

Simulation mode is explicit:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim
```

`rum:=sim` is accepted as a compatibility alias, but `run:=sim` is preferred.

The launch starts:

- Gazebo Jetty with `robot.sdf` in `run:=sim`
- Gazebo Teleop GUI in `run:=sim`, unless `gazebo_gui:=false`
- ROS-Gazebo bridges for `/clock`, `/points_raw`, `/front_camera/image`,
  `/imu`, `/odom`, and `/cmd_vel` in `run:=sim`
- A 1920x1080 front camera at 60 FPS in `run:=sim`
- A 16-channel 3D lidar bridged from Gazebo `/points_raw/points` to ROS
  `/points_raw`
- `odom_to_tf`, publishing `odom -> base_link`, unless `odom_tf:=false`
- Static `base_link -> lidar_link` TF unless `lidar_tf:=false`
- Static identity `map -> odom` TF while OctoMap is active, unless
  `map_odom_tf:=false`
- `octomap_server`, consuming `/points_raw`
- `map_filter`, republishing non-empty projected maps as `/map_valid`
- RViz with 3D lidar points and OctoMap voxel displays
- AeroSentinel Drogon C++ dashboard on port `8080`
- Nav2 servers only when `nav2:=true`
- Optional Python `nav2_waypoint_explorer` when both `nav2:=true` and
  `explore:=true`

## RViz 3D Map

RViz uses [rviz/slam.rviz](rviz/slam.rviz). The default display set is:

```text
Fixed Frame: map
3D Lidar Points: /points_raw
OctoMap Voxels:  /occupied_cells_vis_array
TF:              enabled
```

The 2D projected map topics `/map` and `/map_valid` are intentionally not shown
in RViz by default. They remain available for Nav2, map monitoring, and map
saving.

If the 3D map is blank:

```bash
ros2 topic echo /occupied_cells_vis_array --once
ros2 topic echo /points_raw --once
ros2 run tf2_ros tf2_echo map lidar_link
```

Move the robot for a few seconds before expecting useful voxels. OctoMap only
marks space observed by the 3D lidar, and the simulation lidar range is 12 m.

If the point cloud or voxels appear to move with the robot, check that RViz
Global Options uses fixed frame `map` and that the TF chain exists:

```text
map -> odom -> base_link -> lidar_link
```

## Common Options

Disable OctoMap mapping:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim mapper:=false
```

Enable Nav2 after installing the Nav2 packages:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim nav2:=true
```

Enable autonomous waypoint exploration:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim nav2:=true explore:=true
```

Use the simple point-cloud obstacle-avoidance driver:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim nav2:=false auto_drive:=true
```

Disable RViz:

```bash
ros2 launch ros_test gazebo_slam.launch.py rviz:=false
```

Run Gazebo headless when the GUI cannot create an OpenGL window:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim gazebo_gui:=false
```

On WSL, remote desktops, and machines without a working OpenGL display, run the
simulator headless and disable RViz until the display stack is fixed:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim gazebo_gui:=false rviz:=false
```

Disable the web dashboard or change its port:

```bash
ros2 launch ros_test gazebo_slam.launch.py web:=false
ros2 launch ros_test gazebo_slam.launch.py web_port:=8081
```

Use `web_port:=8081` if another process is already using port `8080`.

Bind the dashboard to localhost only:

```bash
ros2 launch ros_test gazebo_slam.launch.py web_bind_address:=127.0.0.1
```

Set dashboard credentials:

```bash
ros2 launch ros_test gazebo_slam.launch.py web_user:=operator web_password:=change-me
```

## Web Dashboard

The dashboard opens at:

```text
http://127.0.0.1:8080/mission/alpha-0426
```

Default development credentials are `admin` / `admin`. Change them before
exposing the dashboard on a network.

The dashboard displays the front camera feed as a source-encoded H.264 WebRTC
video track. It publishes manual `/cmd_vel` commands with keyboard input and
the on-screen D-pad at about 25 Hz while controls are held.

Default speed limits:

```text
AEROSENTINEL_MAX_LINEAR=2.5
AEROSENTINEL_MAX_ANGULAR=1.8
```

## Expected Topics

```text
/points_raw               sensor_msgs/msg/PointCloud2 from the 3D lidar
/front_camera/image       sensor_msgs/msg/Image camera feed
/imu                      sensor_msgs/msg/Imu from Gazebo or a real IMU
/odom                     nav_msgs/msg/Odometry
/occupied_cells_vis_array visualization_msgs/msg/MarkerArray from OctoMap
/octomap_binary           octomap_msgs/msg/Octomap
/octomap_full             octomap_msgs/msg/Octomap
/map                      nav_msgs/msg/OccupancyGrid projected from OctoMap
/map_valid                nav_msgs/msg/OccupancyGrid after empty-map filtering
/cmd_vel                  geometry_msgs/msg/Twist
```

## Mapping Notes

OctoMap integration is odometry-fixed in this setup, so it does not perform loop
closure. Drive smoothly for the cleanest 3D map.

The projected 2D map is derived from OctoMap using the height and projection
settings in [config/octomap_server.yaml](config/octomap_server.yaml). It is kept
for navigation compatibility, not as the primary RViz visualization.

## Troubleshooting

If `web_server` reports `Address already in use` on `0.0.0.0:8080`, either stop
the old process or launch on another port:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim web_port:=8081
```

If Gazebo or RViz reports GLX, EGL, Ogre, `currentGLContext`, or
`Invalid parentWindowHandle` errors, the ROS graph is not the problem. The
machine cannot create a working OpenGL render window. Start with:

```bash
ros2 launch ros_test gazebo_slam.launch.py run:=sim gazebo_gui:=false rviz:=false web_port:=8081
```

Then fix the display stack before re-enabling RViz. On WSL, make sure WSLg and
GPU drivers are working. For a quick software-rendering attempt, try:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_XCB_GL_INTEGRATION=none
ros2 launch ros_test gazebo_slam.launch.py run:=sim gazebo_gui:=false web_port:=8081
```
