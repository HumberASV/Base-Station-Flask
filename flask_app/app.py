import json
import logging
import os
import threading
import time

from flask import Flask, render_template, request, jsonify, Response
from flask_sqlalchemy import SQLAlchemy
from flask_sock import Sock

from factory import telemetry_factory
import ros2_bridge

_debug = os.getenv("FLASK_DEBUG", "0") == "1"
logging.basicConfig(level=logging.DEBUG if _debug else logging.INFO)

app = Flask(__name__)
app.config["SQLALCHEMY_DATABASE_URI"] = os.getenv("DATABASE_URL", "sqlite:///app.db")
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
app.config["DEBUG"] = _debug

# Set SKIP_STREAM_CHECK=1 in dev when no ASV is connected
_SKIP_STREAM_CHECK = os.getenv("SKIP_STREAM_CHECK", "0") == "1"

db = SQLAlchemy(app)
sock = Sock(app)

# ---------------------------------------------------------------------------
# Shared telemetry state — written by the ROS2 bridge, read by WebSocket clients
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()
_telemetry_state: dict = telemetry_factory.make_default_state()

ros2_bridge.start(_telemetry_state, _state_lock)
_telemetry_state["video"] = {"streamUrl": "/video_feed"}


with app.app_context():
    db.create_all()


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

@app.route("/health")
def health():
    r = jsonify({"status": "ok"})
    r.headers["Access-Control-Allow-Origin"] = "*"
    return r


@app.route("/video_status")
def video_status():
    return jsonify(ros2_bridge.get_status())


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
            time.sleep(0.033)

    r = Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
    r.headers["Access-Control-Allow-Origin"] = "*"
    return r


# ---------------------------------------------------------------------------
# Client / error routes
# ---------------------------------------------------------------------------

@app.route("/client")
def client():
    if not _SKIP_STREAM_CHECK:
        live = ros2_bridge.streams_live()
        if not live["telemetry"] and not live["video"]:
            return render_template(
                "uh_oh.html",
                message="No data is being received from the ASV. "
                        "Please try again when the vehicle is online.",
            )
    return render_template("client.html")


@app.route("/uh_oh")
def uh_oh():
    message = "No data is being received from the ASV."
    return render_template("uh_oh.html", message=message)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True, debug=_debug, use_reloader=False)
