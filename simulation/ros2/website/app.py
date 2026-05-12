from __future__ import annotations

import hmac
import os
import secrets
import threading
import time
from pathlib import Path

from flask import (
    Flask,
    Response,
    jsonify,
    make_response,
    redirect,
    request,
    send_from_directory,
    session,
    stream_with_context,
)


ROOT_DIR = Path(__file__).resolve().parent
PUBLIC_DIR = ROOT_DIR / "public"
CAMERA_TOPIC_ENV = "AEROSENTINEL_CAMERA_TOPIC"
DEFAULT_CAMERA_TOPIC = "/front_camera/image"
JPEG_QUALITY_ENV = "AEROSENTINEL_JPEG_QUALITY"
DEFAULT_JPEG_QUALITY = 85
_cv2 = None
_np = None


class CameraFrameStore:
    def __init__(self):
        self._condition = threading.Condition()
        self._jpeg = None
        self._timestamp = 0.0
        self._width = 0
        self._height = 0
        self._encoding = ""
        self._sequence = 0
        self._topic = DEFAULT_CAMERA_TOPIC

    def update_from_ros_image(self, msg, topic: str) -> None:
        jpeg = ros_image_to_jpeg(msg, camera_jpeg_quality())
        with self._condition:
            self._jpeg = jpeg
            self._timestamp = time.time()
            self._width = int(msg.width)
            self._height = int(msg.height)
            self._encoding = str(msg.encoding)
            self._sequence += 1
            self._topic = topic
            self._condition.notify_all()

    def get(self):
        with self._condition:
            return self._snapshot_locked()

    def wait_for_frame(self, last_sequence: int, timeout: float = 2.0):
        with self._condition:
            self._condition.wait_for(
                lambda: self._jpeg is not None and self._sequence != last_sequence,
                timeout=timeout,
            )
            return self._snapshot_locked()

    def _snapshot_locked(self):
        if self._jpeg is None:
            return None
        return {
            "jpeg": self._jpeg,
            "timestamp": self._timestamp,
            "width": self._width,
            "height": self._height,
            "encoding": self._encoding,
            "sequence": self._sequence,
            "topic": self._topic,
        }


def load_opencv():
    global _cv2, _np

    if _cv2 is not None and _np is not None:
        return _cv2, _np

    try:
        import cv2
        import numpy as np
    except Exception as exc:
        raise RuntimeError(
            "OpenCV camera streaming requires python3-opencv or opencv-python."
        ) from exc

    _cv2 = cv2
    _np = np
    return _cv2, _np


def ros_image_to_jpeg(msg, quality: int) -> bytes:
    return raw_image_to_jpeg(
        int(msg.width),
        int(msg.height),
        str(msg.encoding),
        int(msg.step),
        bytes(msg.data),
        quality,
    )


def raw_image_to_jpeg(
    width: int,
    height: int,
    encoding: str,
    step: int,
    raw: bytes,
    quality: int,
) -> bytes:
    cv2, _ = load_opencv()
    image = raw_image_to_cv_image(width, height, encoding, step, raw)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("OpenCV failed to encode camera frame as JPEG.")
    return encoded.tobytes()


def raw_image_to_cv_image(width: int, height: int, encoding: str, step: int, raw: bytes):
    cv2, np = load_opencv()
    encoding = encoding.lower()

    if width <= 0 or height <= 0:
        raise ValueError("Camera image has invalid dimensions.")

    source_format, channels = image_encoding_format(encoding)
    row_size = width * channels
    step = step if step > 0 else row_size
    if len(raw) < step * height:
        raise ValueError("Camera image payload is smaller than expected.")

    if step != row_size:
        raw = b"".join(raw[y * step : y * step + row_size] for y in range(height))
    else:
        raw = raw[: height * row_size]

    if channels == 1:
        image = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
    else:
        image = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, channels))

    if source_format == "rgb":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if source_format == "rgba":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if source_format == "bgra":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def image_encoding_format(encoding: str):
    if encoding in {"rgb8", "r8g8b8", "8uc3"}:
        return "rgb", 3
    if encoding in {"bgr8", "b8g8r8"}:
        return "bgr", 3
    if encoding in {"rgba8", "r8g8b8a8", "8uc4"}:
        return "rgba", 4
    if encoding in {"bgra8", "b8g8r8a8"}:
        return "bgra", 4
    if encoding in {"mono8", "8uc1"}:
        return "mono", 1
    raise ValueError(f"Unsupported camera image encoding: {encoding}")


camera_frames = CameraFrameStore()
ros_bridge_lock = threading.Lock()
ros_bridge_started = False


class RosControlBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self._publisher = None
        self._twist_type = None
        self._ready = False
        self._last_error = "ROS bridge has not started."

    def configure(self, publisher, twist_type) -> None:
        with self._lock:
            self._publisher = publisher
            self._twist_type = twist_type
            self._ready = True
            self._last_error = ""

    def set_error(self, message: str) -> None:
        with self._lock:
            self._ready = False
            self._last_error = message

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        with self._lock:
            publisher = self._publisher
            twist_type = self._twist_type
            ready = self._ready
            error = self._last_error

        if not ready or publisher is None or twist_type is None:
            return False, error

        msg = twist_type()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        publisher.publish(msg)
        return True, ""


ros_control = RosControlBridge()


def env_or_default(name: str, fallback: str) -> str:
    value = os.environ.get(name)
    return value if value else fallback


def env_flag(name: str) -> bool:
    return env_or_default(name, "").lower() in {"1", "true", "yes", "on"}


def camera_jpeg_quality() -> int:
    try:
        quality = int(env_or_default(JPEG_QUALITY_ENV, str(DEFAULT_JPEG_QUALITY)))
    except ValueError:
        return DEFAULT_JPEG_QUALITY
    return max(1, min(100, quality))


def env_float(name: str, fallback: float) -> float:
    try:
        return float(env_or_default(name, str(fallback)))
    except ValueError:
        return fallback


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def no_store(response):
    response.headers["Cache-Control"] = "no-store"
    return response


def ensure_ros_bridge(app: Flask) -> None:
    global ros_bridge_started

    with ros_bridge_lock:
        if ros_bridge_started:
            return
        ros_bridge_started = True

    topic = env_or_default(CAMERA_TOPIC_ENV, DEFAULT_CAMERA_TOPIC)
    camera_enabled = not env_flag("AEROSENTINEL_DISABLE_CAMERA")
    try:
        if camera_enabled:
            load_opencv()
    except RuntimeError as exc:
        camera_enabled = False
        app.logger.warning("ROS camera feed disabled; %s", exc)

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except Exception as exc:
        message = f"ROS control bridge disabled; could not import ROS support: {exc}"
        ros_control.set_error(message)
        app.logger.warning(message)
        return

    def spin_ros_node():
        node = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("aerosentinel_web")
            cmd_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
            ros_control.configure(cmd_publisher, Twist)

            if camera_enabled:
                def on_image(msg):
                    try:
                        camera_frames.update_from_ros_image(msg, topic)
                    except Exception as exc:
                        node.get_logger().warn(str(exc))

                node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
                app.logger.info("Subscribed AeroSentinel web feed to ROS topic %s", topic)
            app.logger.info("AeroSentinel web controls publishing to /cmd_vel")
            rclpy.spin(node)
        except Exception:
            ros_control.set_error("ROS control bridge stopped unexpectedly.")
            app.logger.exception("ROS web bridge stopped unexpectedly.")
        finally:
            if node is not None:
                node.destroy_node()

    thread = threading.Thread(target=spin_ros_node, name="aerosentinel-ros", daemon=True)
    thread.start()


def create_app() -> Flask:
    app = Flask(__name__, static_folder=None)
    app.secret_key = env_or_default("AEROSENTINEL_SECRET_KEY", secrets.token_hex(32))
    app.config.update(
        SESSION_COOKIE_HTTPONLY=True,
        SESSION_COOKIE_SAMESITE="Strict",
        SESSION_COOKIE_SECURE=env_flag("AEROSENTINEL_SECURE_COOKIES"),
    )

    username = env_or_default("AEROSENTINEL_USER", "admin")
    password = env_or_default("AEROSENTINEL_PASSWORD", "admin")

    if "AEROSENTINEL_PASSWORD" not in os.environ:
        app.logger.warning(
            "AEROSENTINEL_PASSWORD is not set; using development credentials "
            "username='%s', password='admin'.",
            username,
        )
    if "AEROSENTINEL_SECRET_KEY" not in os.environ:
        app.logger.warning(
            "AEROSENTINEL_SECRET_KEY is not set; generated sessions will reset on restart."
        )
    ensure_ros_bridge(app)

    def is_authenticated() -> bool:
        return session.get("authenticated") is True

    def require_auth_page():
        if not is_authenticated():
            return redirect("/login")
        return None

    def login_page(show_error: bool = False):
        error = (
            '<p class="auth-error">Invalid username or password.</p>'
            if show_error
            else ""
        )
        html = f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>AeroSentinel Login</title>
    <link rel="stylesheet" href="/styles.css">
  </head>
  <body class="auth-body">
    <main class="auth-shell">
      <section class="auth-panel">
        <div class="auth-brand">
          <div class="brand-mark" aria-hidden="true"><span></span></div>
          <div><strong>AeroSentinel</strong><small>CONTROL CENTER</small></div>
        </div>
        <p class="auth-kicker">Secure mission access</p>
        <h1>Operator Sign In</h1>
        <form class="auth-form" method="post" action="/login">
          <label>Username<input type="text" name="username" autocomplete="username" required autofocus></label>
          <label>Password<input type="password" name="password" autocomplete="current-password" required></label>
          {error}
          <button type="submit">Unlock Dashboard</button>
        </form>
      </section>
    </main>
  </body>
</html>"""
        return no_store(make_response(html))

    @app.get("/")
    def root():
        auth_redirect = require_auth_page()
        if auth_redirect:
            return auth_redirect
        return send_from_directory(PUBLIC_DIR, "index.html")

    @app.get("/mission/alpha-0426")
    def mission_page():
        auth_redirect = require_auth_page()
        if auth_redirect:
            return auth_redirect
        return send_from_directory(PUBLIC_DIR, "index.html")

    @app.get("/index.html")
    def index_page():
        auth_redirect = require_auth_page()
        if auth_redirect:
            return auth_redirect
        return send_from_directory(PUBLIC_DIR, "index.html")

    @app.get("/login")
    def login_get():
        if is_authenticated():
            return redirect("/mission/alpha-0426")
        return login_page(request.args.get("error") == "1")

    @app.post("/login")
    def login_post():
        submitted_username = request.form.get("username", "")
        submitted_password = request.form.get("password", "")

        if hmac.compare_digest(submitted_username, username) and hmac.compare_digest(
            submitted_password, password
        ):
            session.clear()
            session["authenticated"] = True
            session["operator"] = submitted_username
            return no_store(redirect("/mission/alpha-0426", code=303))

        app.logger.warning("Failed login attempt for user '%s'", submitted_username)
        return redirect("/login?error=1", code=303)

    @app.route("/logout", methods=["GET", "POST"])
    def logout():
        session.clear()
        return redirect("/login")

    @app.get("/api/mission/alpha-0426")
    def mission_api():
        if not is_authenticated():
            return jsonify({"error": "authentication_required"}), 401

        return jsonify(
            {
                "id": "ALPHA-0426",
                "status": "ACTIVE",
                "profile": "Search & Inspect",
                "survey": "Ridge Line Survey",
                "drone": "Sentinel-7B",
                "battery": 78,
                "altitude_m": 512,
                "speed_ms": 15.2,
                "distance_km": 1.2,
                "signal_percent": 94,
                "version": "python-flask-1.0.0",
            }
        )

    @app.post("/api/control/cmd_vel")
    def control_cmd_vel():
        if not is_authenticated():
            return jsonify({"error": "authentication_required"}), 401

        data = request.get_json(silent=True) or {}
        if not isinstance(data, dict):
            return no_store(jsonify({"error": "invalid_velocity"})), 400
        max_linear = abs(env_float("AEROSENTINEL_MAX_LINEAR", 2.5))
        max_angular = abs(env_float("AEROSENTINEL_MAX_ANGULAR", 1.8))
        try:
            linear_x = clamp(float(data.get("linear_x", 0.0)), -max_linear, max_linear)
            angular_z = clamp(float(data.get("angular_z", 0.0)), -max_angular, max_angular)
        except (TypeError, ValueError):
            return no_store(jsonify({"error": "invalid_velocity"})), 400

        ok, error = ros_control.publish_cmd_vel(linear_x, angular_z)
        if not ok:
            return no_store(jsonify({"error": "ros_control_unavailable", "detail": error})), 503

        return no_store(
            jsonify(
                {
                    "linear_x": linear_x,
                    "angular_z": angular_z,
                    "max_linear": max_linear,
                    "max_angular": max_angular,
                }
            )
        )

    def add_camera_headers(response, frame):
        response.headers["X-Camera-Topic"] = frame["topic"]
        response.headers["X-Camera-Width"] = str(frame["width"])
        response.headers["X-Camera-Height"] = str(frame["height"])
        response.headers["X-Camera-Encoding"] = frame["encoding"]
        response.headers["X-Camera-Age"] = f"{time.time() - frame['timestamp']:.3f}"
        return no_store(response)

    @app.get("/api/camera/frame.jpg")
    def camera_frame():
        if not is_authenticated():
            return jsonify({"error": "authentication_required"}), 401

        frame = camera_frames.get()
        if frame is None:
            return no_store(jsonify({"error": "camera_frame_unavailable"})), 503

        return add_camera_headers(Response(frame["jpeg"], mimetype="image/jpeg"), frame)

    @app.get("/api/camera/stream.mjpg")
    def camera_stream():
        if not is_authenticated():
            return jsonify({"error": "authentication_required"}), 401

        def generate():
            last_sequence = 0
            while True:
                frame = camera_frames.wait_for_frame(last_sequence)
                if frame is None or frame["sequence"] == last_sequence:
                    continue
                last_sequence = frame["sequence"]
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-store\r\n\r\n"
                    + frame["jpeg"]
                    + b"\r\n"
                )

        response = Response(
            stream_with_context(generate()),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Accel-Buffering"] = "no"
        return response

    @app.get("/styles.css")
    def styles():
        return send_from_directory(PUBLIC_DIR, "styles.css")

    @app.get("/app.js")
    def app_js():
        return send_from_directory(PUBLIC_DIR, "app.js")

    @app.get("/assets/<path:filename>")
    def assets(filename: str):
        return send_from_directory(PUBLIC_DIR / "assets", filename)

    return app


app = create_app()


if __name__ == "__main__":
    port = int(env_or_default("PORT", "8080"))
    bind_address = env_or_default("AEROSENTINEL_BIND_ADDRESS", "0.0.0.0")
    app.run(host=bind_address, port=port)
