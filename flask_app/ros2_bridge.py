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
import threading
import time

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    _CV2_OK = True
except (ImportError, AttributeError):
    _CV2_OK = False

log = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
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
_latest_frame: bytes | None = None
_frame_lock = threading.Lock()


def set_latest_frame(jpeg: bytes) -> None:
    global _latest_frame
    with _frame_lock:
        _latest_frame = jpeg


def get_latest_frame() -> bytes | None:
    with _frame_lock:
        return _latest_frame


PHONE_TOPIC = "Phone"
TASK_TOPIC = "Task"
ZED_ODOM_TOPIC = "zed/odom"
ZED_IMAGE_TOPIC = "zed/rgb/color/rect/image"
ZED_OBJECTS_TOPIC = "zed/obj_det/objects"

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
        self._bridge = CvBridge() if _CV2_OK else None
        # Raw ROS2 ZED objects kept for annotation overlay (not serialized into state)
        self._raw_objects: list = []
        self._raw_objects_lock = threading.Lock()

        reliable_qos = QoSProfile(depth=10)
        video_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Float32MultiArray, PHONE_TOPIC, self._on_phone, 10)
        self.create_subscription(Float32MultiArray, TASK_TOPIC, self._on_task, 10)
        self.create_subscription(Odometry, ZED_ODOM_TOPIC, self._on_zed_odom, reliable_qos)
        self.create_subscription(Image, ZED_IMAGE_TOPIC, self._on_zed_image, video_qos)
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
        with self._lock:
            cam = self._state["zed"]["camera"]
            cam["active"] = True
            cam["width"] = int(msg.width)
            cam["height"] = int(msg.height)
            cam["encoding"] = msg.encoding
        self._last_zed_image = time.monotonic()

        if not (_CV2_OK and self._bridge):
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._raw_objects_lock:
                raw_objs = list(self._raw_objects)
            _draw_ros_objects(frame, raw_objs)
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                set_latest_frame(buf.tobytes())
        except Exception as exc:
            log.debug("Frame encode error: %s", exc)

    # ------------------------------------------------------------------
    # ZED: object detection  (zed_msgs/ObjectsStamped)
    # ------------------------------------------------------------------
    def _on_zed_objects(self, msg):
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
