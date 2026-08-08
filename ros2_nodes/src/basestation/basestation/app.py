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


# ---------------------------------------------------------------------------
# Parsing command line arguments
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="HumberASV Base Station Web Client")
parser.add_argument("--host", type=str, default=os.getenv("HOST", "0.0.0.0"), help="Host to run the Flask app on (default: 0.0.0.0)")
parser.add_argument("--port", type=int, default=int(os.getenv("PORT", 8080)), help="Port to run the Flask app on (default: 8080)")
parser.add_argument("--dashboard", action="store_true", help="Enable dashboard mode")

args, _ = parser.parse_known_args()  # ignore --ros-args etc. injected by `ros2 launch`

HOST = args.host
PORT = args.port
DASHBOARD_MODE = args.dashboard

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

ros2_bridge.start(_telemetry_state, _state_lock)

# Inject the video stream path so the WebSocket payload always includes it.
_telemetry_state["video"] = {"streamUrl": STREAM_URL}


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

@sock.route("/telemetry")
def telemetry_ws(ws):
    try:
        while True:
            with _state_lock:
                payload = json.dumps(_telemetry_state)
            ws.send(payload)
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


def _live_video_frames():
    """Streams the latest JPEG frame from the ROS2 bridge / annotated_udp_stream buffer."""
    while True:
        frame = ros2_bridge.get_latest_frame()
        if frame is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
        time.sleep(0.033)  # cap at ~30 fps


@app.route("/video_feed")
def video_feed():
    frames = _live_video_frames() if ros2_bridge.is_ros2_available() else _fallback_video_frames()
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
