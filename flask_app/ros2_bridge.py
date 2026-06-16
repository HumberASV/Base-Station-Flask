"""
ROS2 bridge — subscribes to ASV topics and keeps shared telemetry state current.

Topics consumed
---------------
Phone  (std_msgs/Float32MultiArray)  [latitude, longitude, speed, heading]
Task   (std_msgs/Float32MultiArray)  [action, target_heading, target_speed]

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
import threading
import time

log = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    log.warning("rclpy not found — ROS2 bridge disabled. Factory defaults will stream to clients.")

    # Stub so the class body below can be parsed when ROS2 is absent.
    class Node:  # type: ignore[no-redef]
        def __init__(self, *a, **kw): pass
        def get_logger(self): return log
        def create_subscription(self, *a, **kw): pass
        def destroy_node(self): pass

PHONE_TOPIC = "Phone"
TASK_TOPIC = "Task"

_STALE_THRESHOLD_S = 5.0
_LOG_MAX = 50

# Maps the action float published by Task to a TaskStatus string
_ACTION_TO_STATUS = {
    0.0: "standby",
    1.0: "autonomous",
    2.0: "remote",
}


def _append_log(log_list: list, entry: str) -> None:
    """Append to the rolling log, avoiding duplicate consecutive entries."""
    if not log_list or log_list[-1] != entry:
        log_list.append(entry)
    if len(log_list) > _LOG_MAX:
        del log_list[:-_LOG_MAX]


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

        self.create_subscription(Float32MultiArray, PHONE_TOPIC, self._on_phone, 10)
        self.create_subscription(Float32MultiArray, TASK_TOPIC, self._on_task, 10)
        self.get_logger().info(
            f"Subscribed to '{PHONE_TOPIC}' and '{TASK_TOPIC}'. "
            "Waiting for ASV publishers…"
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
    except Exception as exc:
        log.error("ROS2 spin error: %s", exc)
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
