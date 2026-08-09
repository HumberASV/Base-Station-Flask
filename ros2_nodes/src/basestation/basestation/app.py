import json
import logging
import os
import threading
import time
import argparse
import re


# ---------------------------------------------------------------------------
# Expressions
# ---------------------------------------------------------------------------

port = r'^\b(6553[0-5]|655[0-2]\d|65[0-4]\d{2}|6[0-4]\d{3}|[1-5]\d{4}|[1-9]\d{0,3}|0)\b$'
host = r'^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?|localhost)$'

def validate_host_port(host: str, port: str) -> bool:
    """
    Validates the host and port values using regex patterns.
    Returns True if both are valid, False otherwise.
    """
    if not re.match(host, host):
        logging.error(f"Invalid host: {host}")
        return False
    if not re.match(port, port):
        logging.error(f"Invalid port: {port}")
        return False
    return True


def _truthy(value) -> bool:
    """Parses CLI/env/launch-substitution strings ("true", "1", "yes", ...) as bool."""
    return str(value).strip().lower() in ("1", "true", "yes", "on")


# ---------------------------------------------------------------------------
# Parsing command line arguments
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="HumberASV Base Station Web Client")
parser.add_argument("--host", type=str, default=os.getenv("HOST", "0.0.0.0"), help="Host to run the Flask app on (default: 0.0.0.0)")
parser.add_argument("--port", type=int, default=int(os.getenv("PORT", 8080)), help="Port to run the Flask app on (default: 8080)")
parser.add_argument("--dashboard", action="store_true", help="Enable dashboard mode")
parser.add_argument(
    "--factory",
    type=str,
    nargs="?",
    const="true",
    default=os.getenv("FACTORY_MODE", "false"),
    help="Stream factory (fake) telemetry instead of the ROS2 bridge, so the web client has "
         "something dynamic to render without a real ASV or ROS2 install. Pass alone as a bare "
         "flag on the CLI, or with an explicit true/false value (used by the launch file).",
)

args, _ = parser.parse_known_args()  # ignore --ros-args etc. injected by `ros2 launch`

HOST = args.host
PORT = args.port
DASHBOARD_MODE = args.dashboard
FACTORY_MODE = _truthy(args.factory)

# Check if the provided host and port are valid
if not validate_host_port(HOST, str(PORT)):
    logging.error("Invalid host or port. Exiting.")
    exit(1)

# ---------------------------------------------------------------------------
# Start the Flask app
# ---------------------------------------------------------------------------
import cv2
from flask import Flask, render_template, request, jsonify, Response, session
from flask_sock import Sock
from basestation.factory import telemetry_factory
from basestation.modules import ros2_bridge

_debug = os.getenv("FLASK_DEBUG", "0") == "1"
logging.basicConfig(level=logging.DEBUG if _debug else logging.INFO)

app = Flask(__name__)
app.secret_key = os.getenv("SECRET_KEY", "change-me-in-production")

sock = Sock(app)

STREAM_URL = "/video_feed"
FALLBACK_VIDEO_PATH = os.path.join(os.path.dirname(__file__), "assets", "video", "technical-difficulty.mp4")

# ---------------------------------------------------------------------------
# Shared telemetry state — written by the ROS2 bridge, read by WebSocket clients
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()
_telemetry_state: dict = telemetry_factory.make_default_state()

if FACTORY_MODE:
    logging.info("Factory mode enabled (--factory) — streaming fake telemetry, ROS2 bridge not started.")
    telemetry_factory.start(_telemetry_state, _state_lock)
else:
    ros2_bridge.start(_telemetry_state, _state_lock)

# Inject the video stream path so the WebSocket payload always includes it.
_telemetry_state["video"] = {"streamUrl": STREAM_URL}

# Inject the list of topics the bridge subscribes to, so clients (e.g. the
# visualizer's TaskData panel) can display which services/topics are active.
# Static for the life of the process, same pattern as the video injection above.
_telemetry_state["system"] = {"topics": ros2_bridge.get_available_topics()}


@app.context_processor
def inject_globals():
    """Makes dashboard_enabled available to every template (used by the nav drawer)."""
    return {"dashboard_enabled": DASHBOARD_MODE}


# ---------------------------------------------------------------------------
# index route
# ---------------------------------------------------------------------------

@app.route("/")
def index():
    """Main page of the base station web client."""
    return render_template("index.html")


# ---------------------------------------------------------------------------
# index route
# ---------------------------------------------------------------------------
if DASHBOARD_MODE:
    @app.route("/dashboard")
    def dashboard():
        """Dashboard page of the base station web client."""
        return render_template("dashboard.html",
        ros2_available=ros2_bridge.is_ros2_available(),
        zed_available=ros2_bridge.is_zed_available(),
        video_stream_url=STREAM_URL,
        topics=ros2_bridge.get_available_topics()
        )

# ---------------------------------------------------------------------------
# Health check — polled by the visualizer's isBasestationOnline() before it
# will attempt the /telemetry WebSocket connection.
# ---------------------------------------------------------------------------

@app.route("/health")
def health():
    r = jsonify({"status": "ok"})
    r.headers["Access-Control-Allow-Origin"] = "*"
    return r


# ---------------------------------------------------------------------------
# WebSocket — telemetry stream
# ---------------------------------------------------------------------------

# Keys under `map` that are large and change rarely, so they are sent only when the map
# actually changes rather than ~7x/second with everything else. `info` rides along because
# `info.known` is a cols x rows boolean mask that changes with the grid.
_HEAVY_MAP_KEYS = ("occupancyGrid", "navigationGrid", "info")


def _serialise_state(state: dict, include_map: bool) -> str:
    """
    Serialises the telemetry state, optionally omitting the heavy map payload.

    Omitting a key is safe because the web client merges leaf-wise over what it already
    holds (see telemetrySlice's mergeChangedLeaves) — a key that isn't present simply
    leaves the stored value untouched. The copies below are shallow and touch only the
    `map` sub-dict, so this costs nothing next to the json.dumps it shrinks.

    `separators` drops json.dumps' default `", "` / `": "` padding. On a payload sent ~10
    times a second that whitespace is pure overhead — nothing downstream reads the JSON
    by eye, and JSON.parse does not care.
    """
    if include_map:
        return json.dumps(state, separators=(",", ":"))
    trimmed = dict(state)
    trimmed["map"] = {k: v for k, v in state["map"].items() if k not in _HEAVY_MAP_KEYS}
    return json.dumps(trimmed, separators=(",", ":"))


@sock.route("/telemetry")
def telemetry_ws(ws):
    """
    Streams telemetry at ~10 Hz, re-sending the occupancy map only when it changes.

    This loop used to re-serialise the ENTIRE state on every tick, so the grid's size
    multiplied straight into continuous bandwidth — which is what forced ros2_bridge to
    downsample every map to a 48-cell board. Both producers *replace* the grid list when
    the map changes (ros2_bridge._on_map assigns only after its own diff; the factory's
    _apply_map assigns a fresh one) and neither ever mutates it in place, so object
    identity is an exact and free change signal.

    The previous list is held by reference rather than compared by `id()`: an id is just
    an address, and a freed list's address can be reused by a new one, which would read
    as "unchanged" and strand the client on a stale map. Holding the reference also keeps
    that from happening in the first place.
    """
    last_grid = None
    first = True
    try:
        while True:
            with _state_lock:
                grid = _telemetry_state["map"].get("occupancyGrid")
                # `first` covers a client connecting mid-session: it must receive the map
                # in its opening frame even though nothing has changed since boot.
                include_map = first or grid is not last_grid
                payload = _serialise_state(_telemetry_state, include_map)
            ws.send(payload)
            if include_map:
                last_grid = grid
                first = False
            time.sleep(0.1)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Video routes
# ---------------------------------------------------------------------------

def _fallback_video_frames():
    """Loops the local fallback clip forever, yielding MJPEG frames (used when ROS2 is unavailable)."""
    while True:
        cap = cv2.VideoCapture(FALLBACK_VIDEO_PATH)
        try:
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            delay = 1.0 / fps
            while True:
                ok, frame = cap.read()
                if not ok:
                    break  # end of clip — reopen to loop
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
                    )
                time.sleep(delay)
        finally:
            cap.release()


# How often the MJPEG generator checks for a new frame. Much finer than any camera's
# frame interval on purpose: the loop only *sends* when the frame id changes, so polling
# often just means a fresh frame goes out promptly instead of waiting out a fixed sleep.
_VIDEO_POLL_S = 0.005


def _live_video_frames():
    """
    Streams the latest JPEG frame from the ROS2 bridge / annotated_udp_stream buffer.

    Sends each frame exactly once. This used to yield on a fixed 0.033 s tick regardless
    of whether the buffer had changed, so a 15 fps camera still pushed 30 fps of bytes —
    half of them a byte-for-byte repeat of what the client had already decoded.
    """
    last_id = None
    while True:
        frame, frame_id = ros2_bridge.get_latest_frame_with_id()
        if frame is not None and frame_id != last_id:
            last_id = frame_id
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            continue
        time.sleep(_VIDEO_POLL_S)


@app.route("/video_feed")
def video_feed():
    # In factory mode there's no real camera behind ros2_bridge (its thread was never
    # started), so always use the looped fallback clip even if rclpy itself is importable.
    use_live = ros2_bridge.is_ros2_available() and not FACTORY_MODE
    frames = _live_video_frames() if use_live else _fallback_video_frames()
    r = Response(frames, mimetype="multipart/x-mixed-replace; boundary=frame")
    r.headers["Access-Control-Allow-Origin"] = "*"
    return r

# ---------------------------------------------------------------------------
# error route
# ---------------------------------------------------------------------------

@app.route("/uh_oh")
def uh_oh():
    message = "No data is being received from the ASV."
    return render_template("uh_oh.html", message=message)


def main():
    app.run(host=HOST, port=PORT, threaded=True)


if __name__ == "__main__":
    main()
