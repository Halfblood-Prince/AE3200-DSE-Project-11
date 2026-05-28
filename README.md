<!-- Overview and operator notes for the ROS 2 Jazzy Gazebo/OctoMap package. -->
# ROS 2 Jazzy 3D Mapping

[![Unit Tests](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/tests.yml/badge.svg?branch=ros_simulation)](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/tests.yml)
[![Subsystem Tests](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/subsystem-tests.yml/badge.svg?branch=ros_simulation)](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/subsystem-tests.yml)
[![System Tests](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/system-tests.yml/badge.svg?branch=ros_simulation)](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/system-tests.yml)
[![Edge Case Tests](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/edge-case-tests.yml/badge.svg?branch=ros_simulation)](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/edge-case-tests.yml)
[![Coverage](https://img.shields.io/endpoint?url=https%3A%2F%2Fraw.githubusercontent.com%2Fhalfblood-prince%2Fae3200-dse-project-11%2Fros_simulation%2F.github%2Fbadges%2Fcoverage.json)](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/coverage.yml)
[![CodeQL](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/codeql.yml/badge.svg?branch=ros_simulation)](https://github.com/halfblood-prince/ae3200-dse-project-11/actions/workflows/codeql.yml)

## Project Structure

- `.github/`: GitHub Actions workflows for CodeQL, unit, subsystem, system, edge-case, and coverage checks.
- `config/`: Gazebo GUI and OctoMap parameter files.
- `launch/`: ROS 2 launch files that start Gazebo, bridges, mapping, RViz and helper nodes.
- `pathfinding/`: Standalone Python A* example code.
- `resource/`: ROS package index marker used by `ament_python`.
- `robot/`: Gazebo simulation assets. `environment.world` defines the world and includes `model://robot`; `model.config` resolves that model to `robot.sdf`, which defines the hover-capable cuboid robot, 3D lidar, IMU, camera, velocity control, and odometry publisher.
- `ros_test/`: Python package containing the ROS helper node implementations installed by `setup.py`.
- `rviz/`: RViz layout for lidar, TF, and OctoMap visualization.
- `tests/`: Pytest tests for the pathfinding helper, package assets, and ROS helper-node behavior.

This package launches the AeroSentinel robot stack for ROS 2 Jazzy Jalisco on
Ubuntu 24.04 with Gazebo Harmonic simulation support. The simulated robot is a
flat cuboid that moves in 3D through Gazebo `VelocityControl`, publishes odometry
through Gazebo `OdometryPublisher`, and keeps the existing 3D lidar, IMU, and
camera topics.

Real robot mode is the default. Gazebo simulation is enabled explicitly with
`run:=sim`.

## Requirements

- Ubuntu 24.04, codename `noble`
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic packages from the ROS 2 Jazzy apt repository

## Install Dependencies

Install the runtime and build dependencies:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-octomap-server \
  ros-jazzy-rclpy \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-rviz2 \
  ros-jazzy-sensor-msgs \
  ros-jazzy-tf2-ros
```

## Build

Clone the `ros_simulation` branch into a ROS 2 workspace, source Jazzy, and
build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --branch ros_simulation https://github.com/halfblood-prince/ae3200-dse-project-11.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The installed Python ROS executables are:

```text
cloud_filter
map_filter
map_monitor
odom_to_tf
```

## Tests and Coverage

GitHub Actions runs the unit, subsystem, system, edge-case, and coverage pytest
suites inside the `ros:jazzy-ros-base` container so ROS message packages are
available. The coverage job enforces at least 90% total coverage, and the
coverage badge is updated from `coverage.json` after pushes to `ros_simulation`.
For local checks, install the test tools, source Jazzy, and run pytest from the
repository root:

```bash
sudo apt install python3-pytest python3-pytest-cov python3-numpy
source /opt/ros/jazzy/setup.bash
pytest
```

Targeted suites can be run directly:

```bash
pytest -o addopts="" tests/test_subsystems.py tests/test_ros_helpers.py
pytest -o addopts="" tests/test_system_stack.py
pytest -o addopts="" tests/test_edge_cases.py
```

## Launch

Real robot mode is the default:

```bash
ros2 launch ros_test slam.launch.py
```

Real robot mode does not start Gazebo or the ROS-Gazebo bridge. It uses wall
clock time and expects the robot stack to publish:

```text
/odom               nav_msgs/msg/Odometry
/points_raw         sensor_msgs/msg/PointCloud2 with frame lidar_link
/front_camera/image sensor_msgs/msg/Image, if camera data is needed
```

It also expects a base controller subscribed to `/cmd_vel`.

Simulation mode is explicit:

```bash
ros2 launch ros_test slam.launch.py run:=sim
```

`rum:=sim` is accepted as a compatibility alias, but `run:=sim` is preferred.

The launch starts:

- Gazebo Harmonic with `robot/environment.world` in `run:=sim`
- Gazebo Teleop GUI in `run:=sim`, unless `gazebo_gui:=false`
- The hover-capable cuboid robot from `robot/robot.sdf`
- ROS-Gazebo bridges for `/clock`, `/points_raw`, `/imu`, `/odom`, and `/cmd_vel`
- An optional 1920x1080 front camera at 30 FPS, bridged with `camera:=true`
- A 1024 x 16 3D lidar bridged from Gazebo `/points_raw/points` to ROS `/points_raw`
- `cloud_filter`, republishing self-filtered lidar points as `/points_filtered`
- `odom_to_tf`, publishing `odom -> base_link`, unless `odom_tf:=false`
- Static `base_link -> lidar_link` TF unless `lidar_tf:=false`
- Static identity `map -> odom` TF while OctoMap is active, unless `map_odom_tf:=false`
- `octomap_server`, consuming `/points_filtered`
- `map_filter`, republishing non-empty projected maps as `/map_valid`
- `map_monitor`, reporting whether a valid projected map has appeared
- RViz with 3D lidar points and OctoMap voxel displays

## RViz 3D Map

RViz uses [rviz/slam.rviz](rviz/slam.rviz). The default display set is:

```text
Fixed Frame: map
3D Lidar Points: /points_filtered
OctoMap Voxels:  /occupied_cells_vis_array
TF:              enabled
```

If raw `/points_raw` shows circles around the robot, those are usually lidar
returns from the ground plane or from the robot's own body. RViz and OctoMap use
`/points_filtered` by default so those self-hits do not become a trail of
obstacles in the global map.

OctoMap's PCL ground-plane segmentation is disabled because the filtered,
hover-capable scan often does not contain a stable ground plane to fit. Floor
suppression is handled by `/points_filtered` and the OctoMap height limits
instead.

The 2D projected map topics `/map` and `/map_valid` are intentionally not shown
in RViz by default. They remain available for map monitoring and other custom
consumers.

If the 3D map is blank:

```bash
ros2 topic echo /occupied_cells_vis_array --once
ros2 topic echo /points_filtered --once
ros2 run tf2_ros tf2_echo map lidar_link
```

Move the robot for a few seconds before expecting useful voxels. OctoMap only
marks space observed by the 3D lidar, and the simulation lidar range is 12 m.

## Common Options

Move the robot with the Gazebo Teleop panel. Use `q` and `e` for vertical motion
and the usual Gazebo teleop keys for planar motion and yaw.

Disable OctoMap mapping:

```bash
ros2 launch ros_test slam.launch.py run:=sim mapper:=false
```

Disable RViz:

```bash
ros2 launch ros_test slam.launch.py rviz:=false
```

Bridge the front camera image topic:

```bash
ros2 launch ros_test slam.launch.py run:=sim camera:=true
```

Run Gazebo headless when the GUI cannot create an OpenGL window:

```bash
ros2 launch ros_test slam.launch.py run:=sim gazebo_gui:=false
```

On WSL, remote desktops, and machines without a working OpenGL display, run the
simulator headless and disable RViz until the display stack is fixed:

```bash
ros2 launch ros_test slam.launch.py run:=sim gazebo_gui:=false rviz:=false
```

## Expected Topics

```text
/points_raw               sensor_msgs/msg/PointCloud2 from the 3D lidar
/points_filtered          sensor_msgs/msg/PointCloud2 after self/floor filtering
/front_camera/image       sensor_msgs/msg/Image camera feed, if camera:=true
/imu                      sensor_msgs/msg/Imu from Gazebo
/odom                     nav_msgs/msg/Odometry from Gazebo
/occupied_cells_vis_array visualization_msgs/msg/MarkerArray from OctoMap
/octomap_binary           octomap_msgs/msg/Octomap
/octomap_full             octomap_msgs/msg/Octomap
/map                      nav_msgs/msg/OccupancyGrid projected from OctoMap
/map_valid                nav_msgs/msg/OccupancyGrid after empty-map filtering
/cmd_vel                  geometry_msgs/msg/Twist
```

## Mapping Notes

OctoMap integration is odometry-fixed in this setup, so it does not perform loop
closure. Move the cuboid smoothly for the cleanest 3D map.

The projected 2D map is derived from OctoMap using the height and projection
settings in [config/octomap_server.yaml](config/octomap_server.yaml). It is kept
for compatibility with simple map consumers, not as the primary RViz
visualization.

For full 3D consumers, use `/octomap_full` or `/octomap_binary`. The RViz voxel
display uses `/occupied_cells_vis_array`, which is only the visualization view of
the OctoMap.
