"""
ROS2 bridge — subscribes to ASV topics and keeps shared telemetry state current.

Topics consumed
---------------
phone              (std_msgs/Float32MultiArray)  [latitude, longitude, speed, heading]
task               (std_msgs/Float32MultiArray)  [action, target_heading, target_speed]
/asv/joint_states  (sensor_msgs/JointState)  prop_l_joint/prop_r_joint/rudder_joint fractions
battery_status/*   (sensor_msgs/BatteryState)  prop_l, prop_r, main
zed                ()[odom, obj_det/objects, rgb/color/rect/image]
/map               (nav_msgs/OccupancyGrid)  SLAM Toolbox global map -> map.occupancyGrid
/scan              (sensor_msgs/LaserScan)   lidar liveness only (not forwarded)
/tf, /tf_static    (via tf2_ros)  map -> base_link vessel pose -> map.vesselPose

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
import zlib

## Module-level logger
log = logging.getLogger(__name__)

## numpy is guarded on its own, separately from the cv2/cv_bridge block below, because
## the /map conversion needs it and the ZED image path does not. Without this the
## occupancy-grid vectorisation would be disabled by a missing cv_bridge, which has
## nothing to do with it.
try:
    import numpy as np
    _NUMPY_OK = True
except ImportError:
    _NUMPY_OK = False
    log.warning("numpy not found — /map conversion falls back to the pure-Python path.")

## Verify that the cv2, numpy, and bridge packages are available for image processing. If not, the ZED image subscription will be disabled.
try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    log.debug("cv2, numpy, and cv_bridge are available — ZED image subscription enabled.")
    _CV2_OK = True
    _CVBRIDGE_OK = True
except ImportError:
    log.warning("cv2, numpy, or cv_bridge not found — ZED image subscription disabled.")
    _CV2_OK = False
    _CVBRIDGE_OK = False

## Verify that the zed_interfaces package is available for ZED object detection. If not, the ZED object detection subscription will be disabled.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from rcl_interfaces.msg import Log as RosoutLog
    from std_msgs.msg import Float32MultiArray
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import CompressedImage, BatteryState, JointState, LaserScan
    log.debug("rclpy and ROS2 message types are available — ROS2 bridge enabled.")
    _ROS2_AVAILABLE = True
except ImportError:
    # ROS2 not installed — disable the bridge and keep serving factory defaults.
    _ROS2_AVAILABLE = False
    log.warning("rclpy not found — ROS2 bridge disabled. Factory defaults will stream to clients.")

    # Fallback Node class for non-ROS2 environments, so this script can be run
    # without ROS2 installed (e.g., for testing the UDP streamer).
    class Node:  # type: ignore[no-redef]
        def __init__(self, *a, **kw): pass
        def get_logger(self): return log
        def create_subscription(self, *a, **kw): pass
        def create_timer(self, *a, **kw): pass
        def destroy_node(self): pass

try:
    from zed_msgs.msg import ObjectsStamped
    log.debug("zed_msgs is available — ZED object detection subscription enabled.")
    _ZED_MSGS_AVAILABLE = True
except ImportError:
    _ZED_MSGS_AVAILABLE = False
    log.warning("zed_msgs not found — ZED object detection topic will not be subscribed.")

## tf2_ros supplies the map-frame vessel pose. Guarded separately from rclpy so a missing
## tf2 degrades to "no pose" (the client hides the vessel) rather than taking the whole
## bridge down — every other topic is still worth serving without it.
try:
    from tf2_ros import Buffer, TransformListener, TransformException
    log.debug("tf2_ros is available — map-frame vessel pose enabled.")
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False
    log.warning("tf2_ros not found — no map-frame vessel pose will be published.")

# ---------------------------------------------------------------------------
# Module-level JPEG frame buffer — written by the ROS2 bridge (raw frames)
# and overwritten by annotated_udp_stream when it is also running.
# Flask's /video_feed route reads from here.
# ---------------------------------------------------------------------------
_SAD_FRAME: bytes = None
_sad_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "images", "sad.jpg")
try:
    with open(_sad_path, "rb") as _f:
        _SAD_FRAME = _f.read()
except OSError:
    log.warning("Could not load fallback image: %s", _sad_path)

_latest_frame: bytes = _SAD_FRAME
_frame_lock = threading.Lock()
# Bumped on every set_latest_frame. /video_feed uses it to tell "a new frame arrived"
# from "the same frame is still the newest", so it can skip re-sending bytes the client
# already has — the MJPEG generator polls faster than the camera publishes.
_latest_frame_id: int = 0
_spin_running: bool = False
_spin_error: str = ""
_spin_started_at: float = 0.0

# not using pillow rn
_PIL_OK = False

log.info("ros2_bridge ready  cv2=%s  cv_bridge=%s  pil=%s  ros2=%s  sad_frame=%s",
         _CV2_OK, _CVBRIDGE_OK, _PIL_OK, _ROS2_AVAILABLE, _SAD_FRAME is not None)


def set_latest_frame(jpeg: bytes) -> None:
    """
    Update the latest JPEG frame buffer with a new frame.
    Arguments:
        jpeg: The JPEG-encoded image data to store as the latest frame.
    """
    global _latest_frame, _latest_frame_id
    with _frame_lock:
        _latest_frame = jpeg
        _latest_frame_id += 1


def get_latest_frame() -> bytes:
    """
    Retrieve the latest JPEG frame buffer.
    Returns:
        The latest JPEG-encoded image data, or None if no frame is available.
    """
    with _frame_lock:
        return _latest_frame


def get_latest_frame_with_id() -> tuple:
    """
    Retrieve the latest JPEG frame along with its monotonic id.

    Returns `(jpeg, frame_id)`. The id changes only when a genuinely new frame has been
    buffered, letting the MJPEG generator poll at a fine interval — so a fresh frame goes
    out promptly — without re-sending the same bytes on every poll.
    """
    with _frame_lock:
        return _latest_frame, _latest_frame_id

# Topic names for the ASV ROS2 topics consumed by the bridge.
# NOTE: these must match Loon-E's actual publisher topic names exactly (ROS2
# topic names are case-sensitive) — see src/loone/loone/phone.py and task.py.
PHONE_TOPIC = "phone"
TASK_TOPIC = "task"
JOINT_STATES_TOPIC = "/asv/joint_states"
ZED_ODOM_TOPIC = "zedx/zed_node/odom"
ZED_IMAGE_TOPIC = "/zedx/zed_node/rgb/color/rect/image/compressed"
ZED_OBJECTS_TOPIC = "zedx/obj_det/objects"
ROSOUT_TOPIC = "/rosout"
BATTERY_TOPIC = ""
# SLAM Toolbox's global map, and the raw lidar it is built from. The ASV moved off
# nav2/ros2_control onto SLAM Toolbox, which publishes /map as an OccupancyGrid.
MAP_TOPIC = "/map"
SCAN_TOPIC = "/scan"

# TF frames the vessel pose is looked up between. Loon-E's chain is
# `map -> odom -> zedx_camera_link -> base_link`: SLAM Toolbox tracks the ZED frame
# (mapper_params_online_async.yaml sets base_frame: zedx_camera_link, because the ZED X
# publishes `odom -> <camera_name>_camera_link` and no separate base_link exists), and
# bringup.launch.py adds a static `zedx_camera_link -> base_link` for the boat centre.
#
# MAP_FRAME must be SLAM Toolbox's "map" and NOT loone's "gps_map" — config.yaml keeps
# those deliberately distinct, and only "map" is the frame /map's OccupancyGrid lives in.
#
# Both are env-overridable because the static camera->base_link offsets are still an
# unmeasured TODO in bringup.launch.py (currently all zeros); until they are measured,
# pointing BASE_FRAME at zedx_camera_link gives the same answer with one less hop.
MAP_FRAME = os.getenv("MAP_FRAME", "map")
BASE_FRAME = os.getenv("BASE_FRAME", "base_link")

# ---------------------------------------------------------------------------
# OccupancyGrid -> web-client Grid conversion
# ---------------------------------------------------------------------------
# The web client renders an arbitrary cols x rows board of CellTypes
# (react-isometric-engine), so /map keeps its real aspect ratio and is only
# downsampled enough to bound the payload. These numeric values must match the
# engine's CellTypes — same figures the factory mirrors.
CELL_EMPTY = 0
CELL_OCCUPIED = 1

# Longest-axis cap for the board sent to the client, aspect preserved. 0 disables the
# cap and sends the map at its real resolution.
#
# History: this defaulted to 48 because app.py re-serialised the ENTIRE telemetry state
# on every websocket tick (~7 Hz), so grid size multiplied straight into *continuous*
# bandwidth. app.py then started sending the grid only when it actually changes, and the
# default was relaxed to 0 on the reasoning that the remaining cost is paid only per map
# change.
#
# That reasoning accounted for bytes but not for CPU, and CPU is what actually hurt.
# _occupancy_grid_to_cells allocates one dict per output cell, and until the conversion
# moved to its own thread it ran on the same single-threaded rclpy executor that services
# the ZED camera callback. SLAM Toolbox republishes /map every 5 s (map_update_interval)
# at 0.05 m/cell, so a 40 m x 40 m survey is an 800x800 board = 640,000 dicts every five
# seconds — which stalled the video feed for seconds at a time, on a Wi-Fi link that had
# no headroom to spare.
#
# 128 is therefore the default: ~16k cells, roughly 290 KB on the wire, and a conversion
# that finishes in milliseconds. Raise it (or set 0) when the link and the CPU can take
# it — the work is off the callback thread now, so an uncapped map no longer starves the
# camera, it just costs bandwidth and client-side render time. 48 is ~40 KB.
MAP_MAX_AXIS = int(os.getenv("MAP_MAX_AXIS", "128"))

# nav_msgs/OccupancyGrid data is int8: -1 unknown, else 0-100 occupancy probability.
# Anything at or above this is treated as an obstacle. 65 is the usual SLAM Toolbox
# "occupied" threshold; unknown (-1) is deliberately NOT treated as free, it is left
# empty so the client's fog logic can distinguish "known clear" from "never seen".
OCCUPANCY_THRESHOLD = 65


def _board_dims(width: int, height: int) -> tuple:
    """
    Chooses the output board size for a source map, preserving aspect ratio and capping
    the longest axis at MAP_MAX_AXIS. Never upsamples, and never downsamples at all when
    MAP_MAX_AXIS is 0 or negative (the default) — the cap is opt-in.
    """
    longest = max(width, height)
    if MAP_MAX_AXIS <= 0 or longest <= MAP_MAX_AXIS:
        return width, height
    scale = MAP_MAX_AXIS / longest
    return max(1, int(round(width * scale))), max(1, int(round(height * scale)))


def _reduce_occupancy_np(data, width: int, height: int, rows: int, cols: int) -> tuple:
    """
    Block-reduces an OccupancyGrid's cells down to a `rows` x `cols` board with numpy.

    Returns `(occupied, known)`, each a `rows`-long list of `cols`-long bool lists,
    already flipped north-up. A coarse cell is occupied if ANY source cell in its
    footprint is at or above OCCUPANCY_THRESHOLD, and known if ANY is non-negative —
    the same rule as _reduce_occupancy_py, which this must stay bit-identical to.
    """
    src = np.asarray(data, dtype=np.int8)
    if src.size < width * height:
        raise ValueError(f"OccupancyGrid data too short: {src.size} < {width * height}")
    src = src[: width * height].reshape(height, width)

    # Block boundaries, using the same integer arithmetic as the scalar path so the two
    # agree cell for cell. _board_dims never upsamples, so rows <= height and cols <=
    # width, which makes these strictly increasing — what reduceat needs to reduce a real
    # span per output rather than passing a lone element through.
    row_starts = np.arange(rows, dtype=np.intp) * height // rows
    col_starts = np.arange(cols, dtype=np.intp) * width // cols

    def _blocks(mask):
        # max over a boolean block is logical-or, i.e. "any source cell qualifies".
        return np.maximum.reduceat(
            np.maximum.reduceat(mask, row_starts, axis=0), col_starts, axis=1
        )

    # Source row 0 is the SOUTHERN edge and the board is emitted north-up, so reverse.
    # See the axis convention in _occupancy_grid_to_cells' docstring — this flip is
    # required, not cosmetic.
    return _blocks(src >= OCCUPANCY_THRESHOLD)[::-1].tolist(), _blocks(src >= 0)[::-1].tolist()


def _reduce_occupancy_py(data, width: int, height: int, rows: int, cols: int) -> tuple:
    """
    Pure-Python fallback for _reduce_occupancy_np, used when numpy is unavailable.

    Same contract and same output. Kept because numpy is an optional dependency here —
    it arrives with the cv2/cv_bridge stack, which a ROS2-less install does not have.
    """
    occupied_out, known_out = [], []
    for row in range(rows):
        # Source rows covered by this output row, counted from the NORTH edge.
        flipped = rows - 1 - row
        src_r0 = (flipped * height) // rows
        src_r1 = max(src_r0 + 1, ((flipped + 1) * height) // rows)
        occ_row, known_row = [], []
        for col in range(cols):
            src_c0 = (col * width) // cols
            src_c1 = max(src_c0 + 1, ((col + 1) * width) // cols)

            occupied = False
            known = False
            for sr in range(src_r0, min(src_r1, height)):
                base = sr * width
                for sc in range(src_c0, min(src_c1, width)):
                    v = data[base + sc]
                    # -1 is "never observed". Any non-negative reading means SLAM has
                    # actually seen this patch, whether or not something is in it.
                    if v >= 0:
                        known = True
                    if v >= OCCUPANCY_THRESHOLD:
                        occupied = True
                        break
                if occupied:
                    break

            occ_row.append(occupied)
            known_row.append(known)
        occupied_out.append(occ_row)
        known_out.append(known_row)
    return occupied_out, known_out


def _occupancy_grid_to_cells(msg) -> tuple:
    """
    Downsamples a nav_msgs/OccupancyGrid into the board of {type,x,y,z} cells the web
    client renders, and returns `(grid, info)` where `info` carries the real-world
    meaning of that board.

    A coarse cell is occupied if ANY source cell inside it is at or above
    OCCUPANCY_THRESHOLD — matching the factory's `_derive_coarse_grid`, and erring
    toward showing an obstacle rather than hiding one.

    The board keeps the map's aspect ratio rather than being squashed into a square,
    and `info.resolution` is scaled to metres-per-*output*-cell so the client can
    convert real speeds and positions into grid units.

    Axis convention
    ---------------
    The board is emitted NORTH-UP: output row 0 comes from the *northernmost* source
    rows, so board `+col` is map `+x` (east) and board `+row` is map `-y` (south).

    This flip is required, not cosmetic. nav_msgs/OccupancyGrid is row-major from its
    origin corner, so source row 0 is the SOUTHERN edge. The web client's isometric
    engine draws increasing row toward screen +y, and its +y is south (its physics
    integrates heading 0 as dy = -v, i.e. north is -y). Passing the rows through
    unflipped therefore rendered the live map upside-down — harmless only for as long
    as nothing real was ever positioned on it, which stopped being true the moment the
    vessel got a genuine map-frame pose.

    `info.origin` accordingly describes the BOARD's own (0,0) corner — the north-west
    corner in map frame — not the source message's south-west origin.

    NOTE: still unexercised against a real /map publisher — there is no bag file or
    live SLAM Toolbox here. Shape, thresholds, the resolution scaling and the row flip
    all follow the message spec, but nothing below has met real data.
    """
    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0:
        return [], None

    cols, rows = _board_dims(width, height)

    reduce_fn = _reduce_occupancy_np if _NUMPY_OK else _reduce_occupancy_py
    occupied, known = reduce_fn(msg.data, width, height, rows, cols)

    grid = [
        [
            {
                "type": CELL_OCCUPIED if occ else CELL_EMPTY,
                "x": col,
                "y": row,
                "z": 0.0,
            }
            for col, occ in enumerate(occ_row)
        ]
        for row, occ_row in enumerate(occupied)
    ]

    # Flat row-major "has SLAM actually observed this cell" mask. Needed because the
    # downsample collapses OccupancyGrid's -1 (unknown) and 0 (known-clear) into the same
    # CELL_EMPTY, leaving the client unable to tell unsurveyed water from open water —
    # without it, fog-of-war either hides known-clear cells or reveals the entire map.
    # Built from the same north-up arrays as the cells, so it inherits the row flip.
    known_mask: list = [
        k or o
        for known_row, occ_row in zip(known, occupied)
        for k, o in zip(known_row, occ_row)
    ]

    src_res = float(msg.info.resolution) if msg.info.resolution else 0.0
    info = {
        "cols": cols,
        "rows": rows,
        # Metres per OUTPUT cell — the source resolution stretched by the downsample
        # factor. Sending the source value would understate each rendered cell by the
        # decimation ratio and put the vessel's speed out by the same factor.
        "resolution": (src_res * width / cols) if (src_res > 0 and cols > 0) else 0.0,
        # The board's own (0,0) corner: north-west, because the rows were flipped above.
        # msg.info.origin is the source map's south-west corner, so walk north by the
        # map's full height to reach it.
        "origin": {
            "x": float(msg.info.origin.position.x),
            "y": float(msg.info.origin.position.y) + height * src_res,
        },
        "known": known_mask,
    }
    return grid, info

# Servo-fraction -> degrees mapping for the rudder (sensor_msgs/JointState.position
# on JOINT_STATES_TOPIC is a fraction in [0, 1], not a real angle — see busio_node.py /
# thrust_mixer.py). RUDDER_CENTER_FRACTION mirrors thrust_mixer's `rudder_center`
# parameter default (0.55 = straight ahead). RUDDER_MAX_DEG mirrors the maxRudderDeg=90
# prop the website passes to ShipRudderVisualizerPanel. This is an approximation —
# there is no real servo-fraction-to-degree calibration anywhere in the codebase yet;
# it needs tuning against the actual rudder throw once on the water.
RUDDER_CENTER_FRACTION = 0.55
RUDDER_MAX_DEG = 90.0

# Threshold in seconds for staleness detection of ASV topics. If a topic hasn't
# delivered a message within this time, the bridge logs a warning and zeroes the
# signal-strength field, so the web client knows the ASV is unreachable.
_STALE_THRESHOLD_S = 5.0
# /map is event-driven, not a steady stream — SLAM Toolbox may publish nothing for
# a long time in a static scene, so 5s would produce constant false alarms.
_MAP_STALE_THRESHOLD_S = 60.0
# TF, by contrast, is continuous while SLAM is alive. Past this the pose is cleared to
# None rather than left to go quietly stale: the client hides the vessel on a null pose,
# and a frozen boat sitting on live obstacle data is exactly the misrepresentation the
# map-frame pose exists to remove.
_POSE_STALE_THRESHOLD_S = 3.0
# Seconds between repeats of the "no TF yet" warning. A missing map->base_link is the
# normal state before SLAM converges, so this must not spam the log at spin rate.
_TF_WARN_INTERVAL_S = 30.0
# How often tick() polls TF and re-checks staleness. This used to run once per
# spin_once(), i.e. after every message however cheap, which meant a TF lookup and a lock
# acquisition per callback at whatever rate the busiest topic happened to publish. On a
# timer the cost is fixed and predictable instead. Must stay well under the tightest
# threshold above (_POSE_STALE_THRESHOLD_S, 3 s) so staleness is still caught promptly.
_TICK_PERIOD_S = 0.5

# Maximum number of log entries to keep in the rolling log for each topic.
_LOG_MAX = 50

# Maps the action float published by Task to a TaskStatus string
_ACTION_TO_STATUS = {
    0.0: "standby",
    1.0: "autonomous",
    2.0: "remote",
}


def _quat_to_rpy(x: float, y: float, z: float, w: float): #-> tuple[float, float, float]:
    """
    Convert quaternion to (roll, pitch, yaw) in radians.
    Args:
        x, y, z, w: Quaternion components.
    Returns:
        A tuple containing the roll, pitch, and yaw angles in radians.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _rudder_fraction_to_deg(fraction: float) -> float:
    """
    Map a rudder servo fraction in [0, 1] (RUDDER_CENTER_FRACTION = straight) to an
    approximate angle in degrees, clamped to +/-RUDDER_MAX_DEG. See the module-level
    comment on RUDDER_CENTER_FRACTION/RUDDER_MAX_DEG — this scaling is a placeholder
    pending real on-water calibration.
    """
    span = max(RUDDER_CENTER_FRACTION, 1.0 - RUDDER_CENTER_FRACTION)
    deg = (fraction - RUDDER_CENTER_FRACTION) / span * RUDDER_MAX_DEG
    return max(-RUDDER_MAX_DEG, min(RUDDER_MAX_DEG, deg))


def _append_log(log_list: list, entry: str) -> None:
    """
    Append to the rolling log, avoiding duplicate consecutive entries.
    Args:
        log_list: The list of log entries to append to.
        entry: The new log entry to add.
    """
    if not log_list or log_list[-1] != entry:
        log_list.append(entry)
    if len(log_list) > _LOG_MAX:
        del log_list[:-_LOG_MAX]

def _jpeg_dimensions(data: bytes):
    """
    Reads (width, height) out of a JPEG's SOF segment without decoding the image.

    _on_zed_image's passthrough path forwards the camera's JPEG untouched, so it never
    builds a pixel buffer to measure — but the client still wants zed.camera.width/height.
    Walking the segment headers costs microseconds against the ~10-30 ms a full decode
    would cost purely to read two numbers.

    Returns None when the data is not a JPEG this can parse, in which case the caller
    leaves the last known dimensions in place rather than publishing a guess.
    """
    n = len(data)
    if n < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return None
    i = 2
    while i + 3 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        # Padding, SOI and the standalone RSTn markers carry no length field.
        if marker in (0xFF, 0x01, 0xD8) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        # Start of scan / end of image — past here is entropy-coded data, no SOF to find.
        if marker in (0xDA, 0xD9):
            return None
        seg_len = (data[i + 2] << 8) | data[i + 3]
        # SOF0..SOF15 carry the frame dimensions. C4/C8/CC share the range but are
        # DHT/JPG/DAC, which do not.
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            if i + 9 > n:
                return None
            return ((data[i + 7] << 8) | data[i + 8],   # width
                    (data[i + 5] << 8) | data[i + 6])   # height
        if seg_len < 2:
            return None
        i += 2 + seg_len
    return None


def _to_bgr_cv2(frame: "np.ndarray", enc: str) -> "np.ndarray":
    if enc in ('bgra8', 'bgra'):
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if enc in ('rgba8', 'rgba'):
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if enc in ('rgb8', 'rgb'):
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame  # already bgr8


def _draw_ros_objects(frame: "np.ndarray", objects: list) -> None:
    """
    Draw 2D bounding boxes from raw ZED ROS2 objects onto frame in-place.
    Args:
        frame: The image frame to draw on.
        objects: A list of ZED ROS2 object messages.
    """
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

def is_ros2_available() -> bool:
    """
    Check if ROS2 is available in the current environment.
    Returns:
        True if ROS2 is available, False otherwise.
    """
    return _ROS2_AVAILABLE

def is_zed_available() -> bool:
    """
    Check if ZED message types are available in the current environment.
    Returns:
        True if ZED message types are available, False otherwise.
    """
    return _ZED_MSGS_AVAILABLE

def get_available_topics(): #-> list[str]:
    """
    Get a list of available ROS2 topics that the bridge can subscribe to.
    Returns:
        A list of topic names that are available for subscription.
    """
    topics = [
        PHONE_TOPIC, TASK_TOPIC, JOINT_STATES_TOPIC, ZED_ODOM_TOPIC, ZED_IMAGE_TOPIC,
        MAP_TOPIC, SCAN_TOPIC,
        "battery_status/prop_l", "battery_status/prop_r", "battery_status/main",
    ]
    if _ZED_MSGS_AVAILABLE:
        topics.append(ZED_OBJECTS_TOPIC)
    if _TF2_AVAILABLE:
        # Subscribed by tf2_ros' TransformListener rather than by us directly, but the
        # vessel pose depends on them, so they belong in the client's topic list.
        topics.extend(["/tf", "/tf_static"])
    return topics


class _BaseStationNode(Node):
    """
    Minimal ROS2 node that mirrors ASV topics into the shared telemetry dict.
    Both subscriptions are always created — if the ASV is offline they simply
    never fire and the staleness ticker logs a warning after STALE_THRESHOLD_S.
    """

    def __init__(self, state: dict, lock: threading.Lock):
        """
        Initialize the base station ROS2 node.
        Arguments:
            state: Shared telemetry state dictionary to update.
            lock: Threading lock for synchronizing access to the state.
        """
        super().__init__("base_station_receiver")
        self._state = state
        self._lock = lock
        self._last_phone = 0.0
        self._last_task = 0.0
        self._last_zed_odom = 0.0
        self._last_zed_image = 0.0
        self._last_zed_objects = 0.0
        self._last_joint_states = 0.0
        self._last_map = 0.0
        self._last_scan = 0.0
        self._last_pose = 0.0
        self._last_tf_warn = 0.0
        self._last_tf_stamp = None
        self._pose_seen = False
        # Fingerprint of the last /map message accepted, so an unchanged republish is
        # dropped before any conversion work happens — see _map_key / _on_map.
        self._last_map_key = None
        # Single-slot handoff to the map worker thread. Single-slot on purpose: a burst
        # of republishes collapses to the newest, which is the only one worth drawing.
        self._map_pending = None
        self._map_pending_lock = threading.Lock()
        self._map_wakeup = threading.Event()
        self._map_worker_stop = False
        self._scan_seen = False
        # Latest propulsion-battery percentages, averaged into state["battery"]["motors"]
        # as each one reports (prop_l/prop_r are two independent BatteryState topics).
        self._last_prop_l_pct = None
        self._last_prop_r_pct = None
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
        # BEST_EFFORT, even though the ZED image publisher offers RELIABLE. That is not a
        # mismatch: DDS pairs a reader and writer when the OFFERED reliability is at least
        # the REQUESTED one, and BEST_EFFORT < RELIABLE, so a best-effort subscription
        # matches a reliable publisher. Only the reverse — reliable reader, best-effort
        # writer — fails to match.
        #
        # It must be BEST_EFFORT because these frames cross the Wi-Fi hotspot link from
        # the ASV, and cyclonedds.xml caps MaxMessageSize at 65500B so every frame is
        # fragmented into several datagrams. Under RELIABLE a single lost fragment holds
        # the whole sample back until it has been NACKed and retransmitted; on a marginal
        # link that backlog compounds and the feed ends up permanently seconds behind.
        # BEST_EFFORT discards the incomplete frame and delivers the next fresh one, which
        # is what live video actually wants. annotated_udp_stream.py made the same choice.
        video_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # SLAM Toolbox latches /map as RELIABLE + TRANSIENT_LOCAL. A VOLATILE
        # subscriber will not match that publisher at all under DDS QoS rules, so
        # this profile is required, not merely preferable — get it wrong and /map
        # silently delivers nothing rather than erroring.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Float32MultiArray, PHONE_TOPIC, self._on_phone, best_effort_qos)
        self.create_subscription(Float32MultiArray, TASK_TOPIC, self._on_task, best_effort_qos)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_states, best_effort_qos)
        self.create_subscription(Odometry, ZED_ODOM_TOPIC, self._on_zed_odom, best_effort_qos)
        self.create_subscription(CompressedImage, ZED_IMAGE_TOPIC, self._on_zed_image, video_qos)
        self.create_subscription(RosoutLog, ROSOUT_TOPIC, self._on_rosout, reliable_qos)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, map_qos)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, best_effort_qos)
        self.create_subscription(BatteryState, "battery_status/prop_l", self._on_battery_prop_l, reliable_qos)
        self.create_subscription(BatteryState, "battery_status/prop_r", self._on_battery_prop_r, reliable_qos)
        self.create_subscription(BatteryState, "battery_status/main", self._on_battery_main, reliable_qos)
        if _ZED_MSGS_AVAILABLE:
            self.create_subscription(
                ObjectsStamped, ZED_OBJECTS_TOPIC, self._on_zed_objects, reliable_qos
            )

        # TF listener for the map-frame vessel pose. It creates its own /tf and
        # /tf_static subscriptions on this node (with the TRANSIENT_LOCAL profile
        # /tf_static needs), so the existing rclpy.spin_once loop services it — no
        # spin_thread required. Both must be held as attributes: dropping the listener
        # tears down those subscriptions and the buffer silently stops filling.
        self._tf_buffer = Buffer() if _TF2_AVAILABLE else None
        self._tf_listener = TransformListener(self._tf_buffer, self) if _TF2_AVAILABLE else None

        # Staleness/TF polling on a fixed timer rather than after every spin_once —
        # see _TICK_PERIOD_S.
        self.create_timer(_TICK_PERIOD_S, self.tick)

        # /map conversion runs here, off the executor thread. Even vectorised it allocates
        # one dict per board cell, and the executor is single-threaded: anything slow on
        # it stops servicing the ZED camera callback, which is exactly how a five-second
        # SLAM republish turned into a five-second video freeze.
        self._map_worker = threading.Thread(
            target=self._map_worker_loop, daemon=True, name="map-convert"
        )
        self._map_worker.start()

        self.get_logger().info(
            f"Subscribed to '{PHONE_TOPIC}', '{TASK_TOPIC}', '{JOINT_STATES_TOPIC}', "
            f"'{ZED_ODOM_TOPIC}', '{ZED_IMAGE_TOPIC}', '{MAP_TOPIC}', '{SCAN_TOPIC}', "
            f"battery_status/{{prop_l,prop_r,main}}"
            + (f", '{ZED_OBJECTS_TOPIC}'" if _ZED_MSGS_AVAILABLE else " (ZED objects skipped — zed_msgs unavailable)")
            + (f". Vessel pose from TF '{MAP_FRAME}' -> '{BASE_FRAME}'" if _TF2_AVAILABLE else ". No vessel pose — tf2_ros unavailable")
            + ". Waiting for publishers…"
        )


    def _update_motors_battery(self) -> None:
        """Recompute state['battery']['motors'] as the average of whichever of the two
        propulsion batteries (prop_l/prop_r) have reported so far."""
        pcts = [p for p in (self._last_prop_l_pct, self._last_prop_r_pct) if p is not None]
        if pcts:
            self._state["battery"]["motors"] = sum(pcts) / len(pcts)

    def _on_battery_prop_l(self, msg):
        """Callback for the left propulsion (Dewalt) battery."""
        pct = msg.percentage * 100.0
        log.debug("[Battery/prop_l] voltage=%.2fV percentage=%.1f%%", msg.voltage, pct)
        with self._lock:
            self._last_prop_l_pct = pct
            self._update_motors_battery()

    def _on_battery_prop_r(self, msg):
        """Callback for the right propulsion (Dewalt) battery."""
        pct = msg.percentage * 100.0
        log.debug("[Battery/prop_r] voltage=%.2fV percentage=%.1f%%", msg.voltage, pct)
        with self._lock:
            self._last_prop_r_pct = pct
            self._update_motors_battery()

    def _on_battery_main(self, msg):
        """Callback for the primary (Blue Robotics) battery."""
        pct = msg.percentage * 100.0
        log.debug("[Battery/main] voltage=%.2fV percentage=%.1f%%", msg.voltage, pct)
        with self._lock:
            self._state["battery"]["primary"] = pct

    # ------------------------------------------------------------------
    # /asv/joint_states: prop_l_joint/prop_r_joint/rudder_joint fractions
    # ------------------------------------------------------------------
    def _on_joint_states(self, msg):
        """
        Callback for handling incoming actuator JointState echoes from busio_node.
        """
        with self._lock:
            s = self._state
            for name, position in zip(msg.name, msg.position):
                fraction = float(position)
                if name == "prop_l_joint":
                    s["motors"]["left"] = fraction * 100.0
                elif name == "prop_r_joint":
                    s["motors"]["right"] = fraction * 100.0
                elif name == "rudder_joint":
                    s["rudder"]["angle"] = _rudder_fraction_to_deg(fraction)
        log.debug("[JointStates] %s", dict(zip(msg.name, msg.position)))
        self._last_joint_states = time.monotonic()

    # ------------------------------------------------------------------
    # Phone: [latitude, longitude, speed, heading]
    # ------------------------------------------------------------------
    def _on_phone(self, msg):
        """
        Callback for handling incoming phone location messages.
        """
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
        """
        Callback for handling incoming task messages.
        """
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
        """
        Callback for handling incoming ZED odometry messages.
        """
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
        """
        Callback for handling incoming ZED RGB image messages.

        The topic is already JPEG-compressed (sensor_msgs/CompressedImage), so
        msg carries only header/format/data — no width, height or encoding.

        Decoding is therefore only worth doing when there is an object-detection overlay
        to draw. With no boxes pending the payload is forwarded byte for byte: a decode
        plus a re-encode costs ~10-30 ms on the executor thread and re-compresses an
        already-lossy image for a result the client cannot tell apart. Dimensions come
        from the JPEG header instead — see _jpeg_dimensions.
        """
        payload = bytes(msg.data)
        self._last_zed_image = time.monotonic()

        with self._raw_objects_lock:
            objects = list(self._raw_objects)

        if not objects:
            dims = _jpeg_dimensions(payload)
            with self._lock:
                cam = self._state["zed"]["camera"]
                cam["active"] = True
                cam["encoding"] = msg.format
                if dims is not None:
                    cam["width"], cam["height"] = int(dims[0]), int(dims[1])
            set_latest_frame(payload)
            log.debug("[ZED/image] passthrough %d bytes", len(payload))
            return

        if not _CV2_OK:
            # Nothing can be drawn without cv2, but the frame itself is still good —
            # forward it rather than dropping it and stalling the feed.
            with self._lock:
                self._state["zed"]["camera"]["active"] = True
            set_latest_frame(payload)
            log.warning("[ZED/image] cv2 unavailable — forwarding frame without overlay")
            return

        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            log.warning("[ZED/image] imdecode failed (format=%s) — frame dropped", msg.format)
            return

        height, width = frame.shape[:2]
        with self._lock:
            cam = self._state["zed"]["camera"]
            cam["active"] = True
            cam["width"] = int(width)
            cam["height"] = int(height)
            cam["encoding"] = msg.format

        _draw_ros_objects(frame, objects)

        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            set_latest_frame(buf.tobytes())
            log.debug("[ZED/image] annotated %d bytes", buf.size)
        else:
            log.warning("[ZED/image] imencode failed — frame dropped")

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
        """
        Callback for handling incoming ZED object detection messages.
        """
        objects = [
            {
                "label": obj.label,
                "label_id": int(obj.label_id),
                "confidence": float(obj.confidence),
                "position": {
                    # obj.position is zed_msgs/Object's fixed-size `float32[3]` field,
                    # which rosidl generates as a bare numpy.ndarray (no .x/.y/.z) —
                    # index it instead of attribute-accessing it.
                    "x": float(obj.position[0]),
                    "y": float(obj.position[1]),
                    "z": float(obj.position[2]),
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
    # /map  (nav_msgs/OccupancyGrid) — SLAM Toolbox's global map
    # ------------------------------------------------------------------
    @staticmethod
    def _map_key(msg) -> tuple:
        """
        Cheap fingerprint of an OccupancyGrid, used to drop unchanged republishes.

        A CRC over the raw int8 cells plus the geometry that gives them meaning. This
        replaced comparing the *converted* board dict by dict, which had the fast path
        backwards: it did the expensive conversion first and only then discovered nothing
        had changed. SLAM Toolbox republishes /map on a timer whether or not anything
        moved, so that is the common case, and it now costs one C-speed pass.
        """
        info = msg.info
        return (
            int(info.width),
            int(info.height),
            round(float(info.resolution), 6),
            round(float(info.origin.position.x), 6),
            round(float(info.origin.position.y), 6),
            zlib.crc32(bytes(msg.data)),
        )

    def _on_map(self, msg):
        """
        Hands SLAM Toolbox's global map to the worker thread that converts it.

        Deliberately does no conversion work itself: this runs on the single-threaded
        executor shared with the ZED camera callback, and the conversion allocates a dict
        per board cell. Doing it inline is what made the video freeze for seconds every
        time SLAM republished.
        """
        try:
            key = self._map_key(msg)
        except Exception:
            log.exception("[/map] failed to fingerprint OccupancyGrid")
            return

        self._last_map = time.monotonic()
        if key == self._last_map_key:
            return
        self._last_map_key = key

        with self._map_pending_lock:
            self._map_pending = msg
        self._map_wakeup.set()

    def _map_worker_loop(self) -> None:
        """
        Converts whatever /map message is pending, forever, off the executor thread.

        Takes the newest pending message and discards any it superseded — an older map is
        never worth drawing once a newer one exists. The 1 s wait timeout is what lets the
        thread notice _map_worker_stop during shutdown.
        """
        while not self._map_worker_stop:
            if not self._map_wakeup.wait(timeout=1.0):
                continue
            self._map_wakeup.clear()

            with self._map_pending_lock:
                msg, self._map_pending = self._map_pending, None
            if msg is None:
                continue

            started = time.monotonic()
            try:
                grid, info = _occupancy_grid_to_cells(msg)
            except Exception:
                log.exception("[/map] failed to convert OccupancyGrid")
                continue
            if not grid:
                continue

            log.info("[/map] %dx%d @ %.3f m/cell -> %dx%d board @ %.3f m/cell (%.1f ms)",
                     msg.info.width, msg.info.height, msg.info.resolution,
                     info["cols"], info["rows"], info["resolution"],
                     (time.monotonic() - started) * 1000.0)
            # Assign rather than mutate: app.py's websocket loop uses the grid's object
            # identity to decide when to re-send the heavy map keys.
            with self._lock:
                self._state["map"]["occupancyGrid"] = grid
                self._state["map"]["info"] = info

    def shutdown(self) -> None:
        """Stops the map worker. Called from _spin_loop's teardown."""
        self._map_worker_stop = True
        self._map_wakeup.set()

    # ------------------------------------------------------------------
    # /scan  (sensor_msgs/LaserScan) — raw lidar
    # ------------------------------------------------------------------
    def _on_scan(self, msg):
        """
        Records lidar liveness only.

        The Status contract has nowhere to put a raw scan — the client's maps draw
        the SLAM-derived occupancyGrid and the ZED's detected objects, not raw
        ranges — so forwarding ~360 floats at scan rate would add bandwidth to an
        already 35 KB/frame payload for nothing to render. Subscribing still earns
        its keep: it makes lidar failure visible in the log instead of silently
        degrading the map, and /scan appears in the client's topic list.
        """
        self._last_scan = time.monotonic()
        if not self._scan_seen:
            self._scan_seen = True
            ranges = [r for r in msg.ranges if r == r]  # drop NaNs
            log.info("[/scan] first scan: %d rays, range %.2f-%.2f m",
                     len(msg.ranges), msg.range_min, msg.range_max)
            with self._lock:
                _append_log(self._state["task"]["log"], f"Lidar online ({len(ranges)} rays).")

    # ------------------------------------------------------------------
    # TF: map -> base_link — the vessel's true pose on the SLAM map
    # ------------------------------------------------------------------
    def _poll_vessel_pose(self) -> None:
        """
        Looks up the vessel's pose in the map frame and mirrors it into
        `state["map"]["vesselPose"]` as `{x, y, heading}`.

        This is the only honest source of the boat's position on the SLAM grid. The
        alternatives all fail: `phone`'s GPS is in a different frame entirely (loone
        keeps "gps_map" deliberately distinct from SLAM's "map"), and `zed/odom` is the
        ZED's own drifting visual-odometry frame with no loop closure, so neither can be
        placed on /map's cells. Before this the web client dead-reckoned the vessel from
        speed and heading starting at the board's centre, which is pure fiction drawn on
        top of real obstacle data.

        `x`/`y` are metres in the same frame as `info.origin`. `heading` is converted
        here, not on the client: the map frame is ENU (yaw measured counter-clockwise
        from +x/east) while every other heading on the wire is a compass bearing
        (clockwise from north), and the bridge is the side that knows which is which.
        """
        if self._tf_buffer is None:
            return

        try:
            # A zero Time means "latest available" rather than "at time zero" — asking
            # for `now()` would race the transform's own timestamp and raise
            # ExtrapolationException on nearly every call.
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except TransformException as exc:
            # Absent TF is the normal state until SLAM converges, so warn at most once
            # per _TF_WARN_INTERVAL_S. Staleness (below) is what actually clears the pose.
            now = time.monotonic()
            if now - self._last_tf_warn > _TF_WARN_INTERVAL_S:
                self._last_tf_warn = now
                log.warning("[TF] no %s -> %s transform: %s", MAP_FRAME, BASE_FRAME, exc)
            return

        # Freshness is decided by the transform's own timestamp ADVANCING, not by the
        # lookup succeeding. tf2's Buffer caches transforms for ~10s, so asking for the
        # latest one keeps succeeding long after the publisher has died — which silently
        # refreshed the staleness timer forever and left a frozen boat drawn as current,
        # the exact failure the staleness check exists to prevent.
        #
        # Comparing stamps rather than ages also sidesteps clock skew: the ASV and the
        # basestation are different machines and need not be NTP-synced, so an absolute
        # "now - stamp" age would be meaningless. This only ever compares the ASV's clock
        # to itself, and uses the local monotonic clock purely to measure elapsed time
        # here. /tf is published continuously whether or not the vessel moves, so a stamp
        # that stops advancing genuinely means the data stopped.
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp == self._last_tf_stamp:
            return
        self._last_tf_stamp = stamp

        t = tf.transform.translation
        r = tf.transform.rotation
        _roll, _pitch, yaw = _quat_to_rpy(r.x, r.y, r.z, r.w)
        # Map frame is ENU: yaw is counter-clockwise from +x (east). Every heading on the
        # wire is a compass bearing: clockwise from north.
        heading = (90.0 - math.degrees(yaw)) % 360.0

        with self._lock:
            self._state["map"]["vesselPose"] = {
                "x": float(t.x),
                "y": float(t.y),
                "heading": heading,
            }
        self._last_pose = time.monotonic()

        if not self._pose_seen:
            self._pose_seen = True
            log.info("[TF] first %s -> %s pose: (%.2f, %.2f) m heading %.1f°",
                     MAP_FRAME, BASE_FRAME, t.x, t.y, heading)
            with self._lock:
                _append_log(self._state["task"]["log"], "Map-frame vessel pose acquired.")

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
        """
        Poll TF for the vessel pose, then check the staleness of every data source.
        """
        self._poll_vessel_pose()

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

            # /map legitimately goes quiet when SLAM Toolbox has nothing new to
            # publish, so a long gap is not itself a fault — use a much longer
            # window than the other topics and never clear the last known map.
            if self._last_map > 0 and (now - self._last_map) > _MAP_STALE_THRESHOLD_S:
                _append_log(log_list, "WARNING: SLAM map updates stalled.")
                self._last_map = now

            if self._last_scan > 0 and (now - self._last_scan) > _STALE_THRESHOLD_S:
                _append_log(log_list, "WARNING: Lidar (/scan) lost.")
                self._last_scan = now

            # Unlike /map, the pose is NOT preserved when it goes stale. A boat frozen at
            # its last position but still drawn over live obstacles reads as current, so
            # null it and let the client hide the vessel and say so.
            if self._last_pose > 0 and (now - self._last_pose) > _POSE_STALE_THRESHOLD_S:
                self._state["map"]["vesselPose"] = None
                self._pose_seen = False
                _append_log(log_list, "WARNING: Map-frame vessel pose (TF) lost.")
                self._last_pose = now

            if self._last_zed_image > 0 and (now - self._last_zed_image) > _STALE_THRESHOLD_S:
                self._state["zed"]["camera"]["active"] = False
                _append_log(log_list, "WARNING: ZED camera feed lost.")
                self._last_zed_image = now
                # Deliberately leave _latest_frame alone -- keep showing the last frame
                # actually received rather than replacing it with a placeholder. sad.jpg
                # is only ever the *initial* value, before any real frame has arrived.


# ----------------------------------------------------------------------
# Thread entry point
# ----------------------------------------------------------------------

def _spin_loop(state: dict, lock: threading.Lock) -> None:
    """
    Spin the ROS2 node in a loop, handling incoming messages and updating the shared state.
    Arguments:
        state: Shared telemetry state dictionary to update.
        lock: Threading lock for synchronizing access to the state.
    """
    global _spin_running, _spin_error, _spin_started_at

    try:
        rclpy.init()
    except Exception as exc:
        _spin_error = f"rclpy.init() failed: {exc}"
        log.error(_spin_error)
        return

    _spin_started_at = time.monotonic()
    node = _BaseStationNode(state, lock)
    _spin_running = True
    log.info("ROS2 spin loop running (topic=%s)", ZED_IMAGE_TOPIC)
    try:
        # Plain spin: staleness/TF polling is a node timer now (see _TICK_PERIOD_S), so
        # there is no longer per-iteration work to interleave by hand.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        _spin_error = f"{type(exc).__name__}: {exc}"
        log.error("ROS2 spin error: %s", _spin_error)
    finally:
        _spin_running = False
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

def start(state: dict, lock: threading.Lock):# -> threading.Thread | None:
    """
    Start the ROS2 bridge in a daemon thread.
    Returns None (and does nothing) when ROS2 is not installed.
    Arguments:
        state: Shared telemetry state dictionary to update.
        lock: Threading lock for synchronizing access to the state.
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
