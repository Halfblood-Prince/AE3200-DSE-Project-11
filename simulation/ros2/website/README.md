# AeroSentinel Control Center

A Python/Flask app that serves an AeroSentinel drone mission dashboard.

The dashboard UI lives in `public/`. The Flask backend is `app.py` and provides login, logout, the mission page, static assets, the mission JSON API, an OpenCV-encoded MJPEG camera stream, and manual `/cmd_vel` controls when ROS is available.

## Run

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
$env:AEROSENTINEL_USER="admin"
$env:AEROSENTINEL_PASSWORD="admin"
$env:AEROSENTINEL_SECRET_KEY="change-this-secret"
python app.py
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
- `AEROSENTINEL_SECRET_KEY`: Flask session signing key
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

## CI

The GitHub Actions CI installs Flask, compiles the Python files, starts the app, and runs the smoke test.
