# AeroSentinel Control Center

A Drogon C++ web app that serves the AeroSentinel drone mission dashboard.

The dashboard UI lives in `public/`. The C++ backend is `src/main.cc` and provides login, logout, protected dashboard routes, static assets, the mission JSON API, an OpenCV-encoded MJPEG camera stream from ROS image messages, and manual `/cmd_vel` controls.

## Build

Install ROS 2, Drogon, and OpenCV development dependencies. On Ubuntu 24.04 with ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install -y cmake g++ libdrogon-dev libopencv-dev ros-jazzy-ament-cmake ros-jazzy-rclcpp ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs
```

From this `website` directory:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --config RelWithDebInfo
```

On Windows with vcpkg for Drogon/OpenCV, source a ROS 2 environment first and pass your vcpkg toolchain:

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

## Run

Linux/macOS:

```bash
AEROSENTINEL_USER=admin \
AEROSENTINEL_PASSWORD=admin \
AEROSENTINEL_BIND_ADDRESS=127.0.0.1 \
PORT=8080 \
./build/aerosentinel-control
```

Windows PowerShell:

```powershell
$env:AEROSENTINEL_USER="admin"
$env:AEROSENTINEL_PASSWORD="admin"
$env:AEROSENTINEL_BIND_ADDRESS="127.0.0.1"
$env:PORT="8080"
.\build\Release\aerosentinel-control.exe
```

Open:

```text
http://127.0.0.1:8080/mission/alpha-0426
```

Default development credentials are `admin` / `admin` when `AEROSENTINEL_PASSWORD` is not set.

## Environment

- `PORT`: server port, default `8080`
- `AEROSENTINEL_BIND_ADDRESS`: bind address, default `0.0.0.0`
- `AEROSENTINEL_USER`: login username, default `admin`
- `AEROSENTINEL_PASSWORD`: login password, default `admin`
- `AEROSENTINEL_SECURE_COOKIES`: set to `true` behind HTTPS
- `AEROSENTINEL_CAMERA_TOPIC`: ROS image topic for the live feed, default `/front_camera/image`
- `AEROSENTINEL_JPEG_QUALITY`: OpenCV JPEG stream quality from `1` to `100`, default `85`
- `AEROSENTINEL_MAX_LINEAR`: maximum manual linear speed in m/s, default `2.5`
- `AEROSENTINEL_MAX_ANGULAR`: maximum manual turn speed in rad/s, default `1.8`

## Smoke Test

Start the app in one terminal, then run:

```powershell
python smoke_test.py --base-url http://127.0.0.1:8080
```

The smoke test uses Python's standard library only.

## CI

The app builds with CMake and runs the standard-library smoke test against the Drogon server.
