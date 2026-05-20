# AeroSentinel Control Center

A Drogon C++ web app that serves the AeroSentinel drone mission dashboard.

The dashboard UI lives in `public/`. The C++ backend is `src/main.cc` and provides login, logout, protected dashboard routes, static assets, the mission JSON API, a WebRTC H.264 video track encoded at the ROS image source, and manual `/cmd_vel` controls.

## Build From Source

The normal ROS package build does not compile this app; it installs the shipped
`web_server` binary and bundled shared libraries from `bin/<arch>`. Use these
steps only when regenerating the shipped binaries or developing the web server
itself.

Install ROS 2, Drogon, and OpenCV development dependencies. On Ubuntu 26.04 with ROS 2 Lyrical:

```bash
sudo apt update
sudo apt install -y cmake g++ libavcodec-dev libavutil-dev libdrogon-dev libopencv-dev libswscale-dev libx264-dev ros-lyrical-ament-cmake ros-lyrical-rclcpp ros-lyrical-geometry-msgs ros-lyrical-sensor-msgs
```

Install libdatachannel from your distro, vcpkg, or source package as well; CMake expects the `LibDataChannel` package to be findable.

From this `website` directory:

```bash
source /opt/ros/lyrical/setup.bash
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
- `AEROSENTINEL_JPEG_QUALITY`: OpenCV JPEG snapshot quality from `1` to `100`, default `85`
- `AEROSENTINEL_H264_ENCODER`: FFmpeg H.264 encoder name, default `libx264`
- `AEROSENTINEL_H264_BITRATE`: H.264 target bitrate in bits per second, default `6000000`
- `AEROSENTINEL_H264_GOP_FRAMES`: H.264 keyframe interval in frames, default `60`
- `AEROSENTINEL_H264_PROFILE`: encoder profile option, default `baseline`
- `AEROSENTINEL_H264_PROFILE_ID`: WebRTC H.264 SDP profile-level-id, default `42e01f`
- `AEROSENTINEL_H264_FMTP`: full WebRTC H.264 SDP FMTP string, default `profile-level-id=42e01f;packetization-mode=1;level-asymmetry-allowed=1`
- `AEROSENTINEL_X264_PARAMS`: x264 low-level params, default `keyint=60:min-keyint=60:scenecut=0:repeat-headers=1:annexb=1`
- `AEROSENTINEL_WEBRTC_ICE_SERVERS`: optional comma-separated STUN/TURN server URLs for WebRTC ICE
- `AEROSENTINEL_WEBRTC_MAX_BUFFERED_BYTES`: max queued WebRTC media bytes before dropping camera frames, default `262144`
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
