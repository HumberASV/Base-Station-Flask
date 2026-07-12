import copy
import json
import logging
import os
import threading
import time

from flask import Flask, render_template, request, jsonify, Response
from flask_sock import Sock

from factory import telemetry_factory
import ros2_bridge

logging.basicConfig(level=logging.INFO)

app = Flask(__name__)
app.secret_key = os.getenv("SECRET_KEY", "change-me-in-production")

sock = Sock(app)
# ---------------------------------------------------------------------------
# Shared telemetry state — written by the ROS2 bridge, read by WebSocket clients
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()
_telemetry_state: dict = telemetry_factory.make_default_state()

# Start the ROS2 bridge (no-op when ROS2 is not installed)
ros2_bridge.start(_telemetry_state, _state_lock)

# Inject the video stream path so the WebSocket payload always includes it.
_telemetry_state["video"] = {"streamUrl": "/video_feed"}


# ---------------------------------------------------------------------------
# WebSocket — telemetry stream (plain WebSocket, consumed by the web client)
# ---------------------------------------------------------------------------

@sock.route("/telemetry")
def telemetry_ws(ws):
    """
    Streams the current telemetry state to connected web clients at 10 Hz.
    Each frame is a JSON-serialised Status object matching the web client type.
    The connection is plain WebSocket (not socket.io) so `new WebSocket(url)`
    on the client side works directly.
    """
    try:
        while True:
            with _state_lock:
                payload = json.dumps(_telemetry_state)
            ws.send(payload)
            time.sleep(0.1)
    except Exception:
        pass  # client disconnected


# ---------------------------------------------------------------------------
# MJPEG video feed — served from the frame buffer populated by ros2_bridge
# (raw frames) or annotated_udp_stream (annotated frames, takes priority).
# ---------------------------------------------------------------------------

@app.route("/video_feed")
def video_feed():
    def generate():
        while True:
            frame = ros2_bridge.get_latest_frame()
            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(0.033)  # cap at ~30 fps

    r = Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
    r.headers["Access-Control-Allow-Origin"] = "*"
    return r

# ---------------------------------------------------------------------------
# Client / error routes
# ---------------------------------------------------------------------------

@app.route("/client")
def client():
    """Landing page for users with a valid access token."""
    return render_template("client.html")


@app.route("/uh_oh")
def uh_oh():
    """Displayed when a user presents an invalid or expired token."""
    return render_template("uh_oh.html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
