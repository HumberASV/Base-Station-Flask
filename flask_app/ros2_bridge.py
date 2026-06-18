"""
ROS2 bridge — subscribes to ASV topics and keeps shared telemetry state current.

Topics consumed
---------------
Phone  (std_msgs/Float32MultiArray)  [latitude, longitude, speed, heading]
Task   (std_msgs/Float32MultiArray)  [action, target_heading, target_speed]
zed    ()[odom, obj_det/objects, rgb/color/rect/image]
Factory fallback
----------------
Subscriptions are created unconditionally.  If a topic hasn't delivered a
message within STALE_THRESHOLD_S the bridge logs a warning and zeroes the
signal-strength field, so the web client knows the ASV is unreachable.
The factory default state (from telemetry_factory.make_default_state) remains
in effect for every field that has no live ROS2 data.

If rclpy is not importable (ROS2 not installed) the bridge simply does not
start and the app keeps serving factory defaults.
"""

import logging
import math
import os
import threading
import time

try:
    import cv2
    import numpy as np
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    from cv_bridge import CvBridge
    _CVBRIDGE_OK = True
except (ImportError, AttributeError):
    _CVBRIDGE_OK = False

try:
    import io as _io
    from PIL import Image as _PILImage
    _PIL_OK = True
except ImportError:
    _PIL_OK = False

log = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from rcl_interfaces.msg import Log as RosoutLog
    from std_msgs.msg import Float32MultiArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    log.warning("rclpy not found — ROS2 bridge disabled. Factory defaults will stream to clients.")

    class Node:  # type: ignore[no-redef]
        def __init__(self, *a, **kw): pass
        def get_logger(self): return log
        def create_subscription(self, *a, **kw): pass
        def destroy_node(self): pass

try:
    from zed_msgs.msg import ObjectsStamped
    _ZED_MSGS_AVAILABLE = True
except ImportError:
    _ZED_MSGS_AVAILABLE = False
    log.debug("zed_msgs not found — ZED object detection topic will not be subscribed.")

# ---------------------------------------------------------------------------
# Module-level JPEG frame buffer — written by the ROS2 bridge (raw frames)
# and overwritten by annotated_udp_stream when it is also running.
# Flask's /video_feed route reads from here.
# ---------------------------------------------------------------------------
_SAD_FRAME: bytes | None = None
_sad_path = os.path.join(os.path.dirname(__file__), "images", "sad.jpg")
try:
    with open(_sad_path, "rb") as _f:
        _SAD_FRAME = _f.read()
except OSError:
    log.warning("Could not load fallback image: %s", _sad_path)

_latest_frame: bytes | None = _SAD_FRAME
_frame_lock = threading.Lock()
_frames_received: int = 0  # incremented each time _on_zed_image fires

log.info("ros2_bridge ready  cv2=%s  cv_bridge=%s  pil=%s  ros2=%s  sad_frame=%s",
         _CV2_OK, _CVBRIDGE_OK, _PIL_OK, _ROS2_AVAILABLE, _SAD_FRAME is not None)


def set_latest_frame(jpeg: bytes) -> None:
    global _latest_frame
    with _frame_lock:
        _latest_frame = jpeg


def get_latest_frame() -> bytes | None:
    with _frame_lock:
        return _latest_frame


def get_status() -> dict:
    with _frame_lock:
        frame = _latest_frame
    return {
        "cv2": _CV2_OK,
        "cv_bridge": _CVBRIDGE_OK,
        "pil": _PIL_OK,
        "ros2": _ROS2_AVAILABLE,
        "sad_frame_loaded": _SAD_FRAME is not None,
        "frames_received": _frames_received,
        "showing_sad": frame == _SAD_FRAME,
        "frame_bytes": len(frame) if frame else 0,
    }


PHONE_TOPIC = "Phone"
TASK_TOPIC = "Task"
ZED_ODOM_TOPIC = "zed/zed_node/odom"
ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
ZED_OBJECTS_TOPIC = "zed/obj_det/objects"
ROSOUT_TOPIC = "/rosout"

_STALE_THRESHOLD_S = 5.0
_LOG_MAX = 50

# Maps the action float published by Task to a TaskStatus string
_ACTION_TO_STATUS = {
    0.0: "standby",
    1.0: "autonomous",
    2.0: "remote",
}


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert quaternion to (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _append_log(log_list: list, entry: str) -> None:
    """Append to the rolling log, avoiding duplicate consecutive entries."""
    if not log_list or log_list[-1] != entry:
        log_list.append(entry)
    if len(log_list) > _LOG_MAX:
        del log_list[:-_LOG_MAX]

def _to_bgr_cv2(frame: "np.ndarray", enc: str) -> "np.ndarray":
    if enc in ('bgra8', 'bgra'):
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if enc in ('rgba8', 'rgba'):
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if enc in ('rgb8', 'rgb'):
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame  # already bgr8


def _draw_ros_objects(frame: "np.ndarray", objects: list) -> None:
    """Draw 2D bounding boxes from raw ZED ROS2 objects onto frame in-place."""
    if not (_CV2_OK and objects):
        return
    for obj in objects:
        try:
            corners = obj.bounding_box_2d.corners
        except AttributeError:
            continue
        if not corners:
            continue
        pts = np.array([[int(c.kp[0]), int(c.kp[1])] for c in corners], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        label = f"{obj.label} {obj.confidence:.0f}%"
        x0, y0 = pts[0]
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x0, y0 - th - 6), (x0 + tw + 4, y0), (0, 255, 0), cv2.FILLED)
        cv2.putText(frame, label, (x0 + 2, y0 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


class _BaseStationNode(Node):
    """
    Minimal ROS2 node that mirrors ASV topics into the shared telemetry dict.
    Both subscriptions are always created — if the ASV is offline they simply
    never fire and the staleness ticker logs a warning after STALE_THRESHOLD_S.
    """

    def __init__(self, state: dict, lock: threading.Lock):
        super().__init__("base_station_receiver")
        self._state = state
        self._lock = lock
        self._last_phone = 0.0
        self._last_task = 0.0
        self._last_zed_odom = 0.0
        self._last_zed_image = 0.0
        self._last_zed_objects = 0.0
        self._bridge = CvBridge() if (_CV2_OK and _CVBRIDGE_OK) else None
        log.info("[ZED] cv2=%s cv_bridge=%s", _CV2_OK, _CVBRIDGE_OK)
        # Raw ROS2 ZED objects kept for annotation overlay (not serialized into state)
        self._raw_objects: list = []
        self._raw_objects_lock = threading.Lock()

        reliable_qos = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        video_qos = best_effort_qos

        self.create_subscription(Float32MultiArray, PHONE_TOPIC, self._on_phone, best_effort_qos)
        self.create_subscription(Float32MultiArray, TASK_TOPIC, self._on_task, best_effort_qos)
        self.create_subscription(Odometry, ZED_ODOM_TOPIC, self._on_zed_odom, best_effort_qos)
        self.create_subscription(Image, ZED_IMAGE_TOPIC, self._on_zed_image, video_qos)
        self.create_subscription(RosoutLog, ROSOUT_TOPIC, self._on_rosout, reliable_qos)
        if _ZED_MSGS_AVAILABLE:
            self.create_subscription(
                ObjectsStamped, ZED_OBJECTS_TOPIC, self._on_zed_objects, reliable_qos
            )

        self.get_logger().info(
            f"Subscribed to '{PHONE_TOPIC}', '{TASK_TOPIC}', '{ZED_ODOM_TOPIC}', "
            f"'{ZED_IMAGE_TOPIC}'"
            + (f", '{ZED_OBJECTS_TOPIC}'" if _ZED_MSGS_AVAILABLE else " (ZED objects skipped — zed_msgs unavailable)")
            + ". Waiting for publishers…"
        )

    # ------------------------------------------------------------------
    # Phone: [latitude, longitude, speed, heading]
    # ------------------------------------------------------------------
    def _on_phone(self, msg):
        d = msg.data
        if len(d) < 4:
            return
        lat, lon, speed, heading = float(d[0]), float(d[1]), float(d[2]), float(d[3])
        log.debug("[Phone] lat=%.6f lon=%.6f speed=%.3f heading=%.2f", lat, lon, speed, heading)
        with self._lock:
            s = self._state
            s["asv"]["latitude"] = lat
            s["asv"]["longitude"] = lon
            s["asv"]["speed"] = speed
            s["asv"]["heading"] = heading
            s["task"]["location"]["latitude"] = lat
            s["task"]["location"]["longitude"] = lon
            s["signal"]["strength"] = 100.0
        self._last_phone = time.monotonic()

    # ------------------------------------------------------------------
    # Task: [action, target_heading, target_speed]
    # ------------------------------------------------------------------
    def _on_task(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        action = float(d[0])
        target_heading = float(d[1])
        target_speed = float(d[2])
        status = _ACTION_TO_STATUS.get(action, "standby")
        log.debug("[Task] action=%.0f status=%s heading=%.1f speed=%.3f", action, status, target_heading, target_speed)
        entry = (
            f"Task — action={action:.0f} "
            f"heading={target_heading:.1f}° "
            f"speed={target_speed:.2f} m/s"
        )
        with self._lock:
            s = self._state
            s["planning"]["status"] = status
            s["task"]["data"]["status"] = status
            _append_log(s["task"]["log"], entry)
        self._last_task = time.monotonic()

    # ------------------------------------------------------------------
    # ZED: odom  (nav_msgs/Odometry)
    # ------------------------------------------------------------------
    def _on_zed_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        roll, pitch, yaw = _quat_to_rpy(o.x, o.y, o.z, o.w)
        log.debug("[ZED/odom] pos=(%.3f, %.3f, %.3f) rpy=(%.1f°, %.1f°, %.1f°)",
                  p.x, p.y, p.z, math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        with self._lock:
            zed = self._state["zed"]["odom"]
            zed["position"] = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}
            zed["orientation"] = {
                "roll": math.degrees(roll),
                "pitch": math.degrees(pitch),
                "yaw": math.degrees(yaw),
            }
        self._last_zed_odom = time.monotonic()

    # ------------------------------------------------------------------
    # ZED: RGB image  (sensor_msgs/Image) — stores metadata + produces JPEG
    # ------------------------------------------------------------------
    def _on_zed_image(self, msg):
        global _frames_received
        _frames_received += 1
        log.info("[ZED/image] #%d  %dx%d  enc=%s", _frames_received, msg.width, msg.height, msg.encoding)

        with self._lock:
            cam = self._state["zed"]["camera"]
            cam["active"] = True
            cam["width"] = int(msg.width)
            cam["height"] = int(msg.height)
            cam["encoding"] = msg.encoding
        self._last_zed_image = time.monotonic()

        jpeg = self._encode_jpeg(msg)
        if jpeg is not None:
            set_latest_frame(jpeg)
            log.debug("[ZED/image] buffered %d bytes", len(jpeg))
        else:
            log.warning("[ZED/image] all encode paths failed — frame dropped")

    def _encode_jpeg(self, msg) -> "bytes | None":
        enc = msg.encoding.lower()

        # ── Path 1: cv_bridge + cv2 ──────────────────────────────────────
        if _CV2_OK and self._bridge:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                frame = _to_bgr_cv2(frame, enc)
                with self._raw_objects_lock:
                    _draw_ros_objects(frame, list(self._raw_objects))
                ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    return buf.tobytes()
                log.warning("[ZED/image] cv_bridge path: imencode failed")
            except Exception as exc:
                log.warning("[ZED/image] cv_bridge path failed: %s", exc, exc_info=True)

        # ── Path 2: numpy + cv2 (no cv_bridge) ───────────────────────────
        if _CV2_OK:
            try:
                raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                n_ch = msg.step // msg.width
                frame = raw.reshape((msg.height, msg.width, n_ch))
                frame = _to_bgr_cv2(frame, enc)
                ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    return buf.tobytes()
                log.warning("[ZED/image] numpy path: imencode failed")
            except Exception as exc:
                log.warning("[ZED/image] numpy path failed: %s", exc, exc_info=True)

        # ── Path 3: PIL (no cv2 at all) ───────────────────────────────────
        if _PIL_OK:
            try:
                raw = bytes(msg.data)
                if enc in ('bgra8', 'bgra'):
                    img = _PILImage.frombuffer('RGBA', (msg.width, msg.height), raw, 'raw', 'BGRA', 0, 1).convert('RGB')
                elif enc in ('rgba8', 'rgba'):
                    img = _PILImage.frombuffer('RGBA', (msg.width, msg.height), raw, 'raw', 'RGBA', 0, 1).convert('RGB')
                elif enc in ('bgr8', 'bgr'):
                    img = _PILImage.frombuffer('RGB', (msg.width, msg.height), raw, 'raw', 'BGR', 0, 1)
                elif enc in ('rgb8', 'rgb'):
                    img = _PILImage.frombuffer('RGB', (msg.width, msg.height), raw, 'raw', 'RGB', 0, 1)
                else:
                    log.warning("[ZED/image] PIL: unsupported encoding %s", msg.encoding)
                    return None
                buf = _io.BytesIO()
                img.save(buf, format='JPEG', quality=80)
                return buf.getvalue()
            except Exception as exc:
                log.warning("[ZED/image] PIL path failed: %s", exc, exc_info=True)

        log.warning("[ZED/image] no encoder available  cv2=%s  cv_bridge=%s  pil=%s",
                    _CV2_OK, _CVBRIDGE_OK, _PIL_OK)
        return None

    # ------------------------------------------------------------------
    # ZED: object detection  (zed_msgs/ObjectsStamped)
    # ------------------------------------------------------------------
    def _on_zed_objects(self, msg):
        log.debug("[ZED/objects] count=%d  %s", len(msg.objects),
                  [(o.label, f"{o.confidence:.0f}%") for o in msg.objects])
        objects = [
            {
                "label": obj.label,
                "label_id": int(obj.label_id),
                "confidence": float(obj.confidence),
                "position": {
                    "x": float(obj.position.x),
                    "y": float(obj.position.y),
                    "z": float(obj.position.z),
                },
            }
            for obj in msg.objects
        ]
        with self._lock:
            self._state["zed"]["objects"] = objects
        with self._raw_objects_lock:
            self._raw_objects = list(msg.objects)
        self._last_zed_objects = time.monotonic()

    # ------------------------------------------------------------------
    # /rosout  (rcl_interfaces/Log) — system-wide ROS2 log stream
    # ------------------------------------------------------------------
    _ROSOUT_LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

    def _on_rosout(self, msg):
        level = self._ROSOUT_LEVEL.get(msg.level, str(msg.level))
        if msg.level < 20:  # skip DEBUG — too noisy
            return
        entry = f"[{level}] [{msg.name}]: {msg.msg}"
        log.debug("[rosout] %s", entry)
        with self._lock:
            _append_log(self._state["task"]["log"], entry)

    # ------------------------------------------------------------------
    # Staleness check — called once per spin iteration
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        with self._lock:
            log_list = self._state["task"]["log"]

            if self._last_phone > 0 and (now - self._last_phone) > _STALE_THRESHOLD_S:
                self._state["signal"]["strength"] = 0.0
                _append_log(log_list, "WARNING: ASV telemetry (Phone) lost.")
                self._last_phone = now  # reset to suppress repeat spam

            if self._last_task > 0 and (now - self._last_task) > _STALE_THRESHOLD_S:
                _append_log(log_list, "WARNING: Task data lost.")
                self._last_task = now

            if self._last_zed_odom > 0 and (now - self._last_zed_odom) > _STALE_THRESHOLD_S:
                _append_log(log_list, "WARNING: ZED odometry lost.")
                self._last_zed_odom = now

            if self._last_zed_image > 0 and (now - self._last_zed_image) > _STALE_THRESHOLD_S:
                self._state["zed"]["camera"]["active"] = False
                _append_log(log_list, "WARNING: ZED camera feed lost.")
                self._last_zed_image = now
                if _SAD_FRAME is not None:
                    set_latest_frame(_SAD_FRAME)


# ----------------------------------------------------------------------
# Thread entry point
# ----------------------------------------------------------------------

def _spin_loop(state: dict, lock: threading.Lock) -> None:
    try:
        rclpy.init()
    except Exception as exc:
        log.error("rclpy.init() failed (%s) — ROS2 bridge disabled.", exc)
        return

    node = _BaseStationNode(state, lock)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            node.tick()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        log.error("ROS2 spin error: %s: %s", type(exc).__name__, exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def start(state: dict, lock: threading.Lock) -> "threading.Thread | None":
    """
    Start the ROS2 bridge in a daemon thread.
    Returns None (and does nothing) when ROS2 is not installed.
    """
    if not _ROS2_AVAILABLE:
        return None
    t = threading.Thread(
        target=_spin_loop,
        args=(state, lock),
        daemon=True,
        name="ros2-bridge",
    )
    t.start()
    log.info("ROS2 bridge thread started.")
    return t
