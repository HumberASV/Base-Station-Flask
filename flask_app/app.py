import copy
import json
import logging
import os
import threading
import time

from flask import Flask, render_template, request, jsonify
from flask_login import LoginManager, UserMixin, login_required, login_user, logout_user
from flask_sqlalchemy import SQLAlchemy
from flask_sock import Sock

from factory import telemetry_factory
import ros2_bridge

logging.basicConfig(level=logging.INFO)

app = Flask(__name__)
app.secret_key = os.getenv("SECRET_KEY", "change-me-in-production")
app.config["SQLALCHEMY_DATABASE_URI"] = os.getenv("DATABASE_URL", "sqlite:///app.db")
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False

db = SQLAlchemy(app)
sock = Sock(app)

login_manager = LoginManager(app)
login_manager.login_view = "admin_login"

# ---------------------------------------------------------------------------
# Shared telemetry state — written by the ROS2 bridge, read by WebSocket clients
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()
_telemetry_state: dict = telemetry_factory.make_default_state()

# Start the ROS2 bridge (no-op when ROS2 is not installed)
ros2_bridge.start(_telemetry_state, _state_lock)


# ---------------------------------------------------------------------------
# Database models
# ---------------------------------------------------------------------------

class User(UserMixin, db.Model):
    __tablename__ = "users"
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(80), unique=True, nullable=False)
    password_hash = db.Column(db.String(128), nullable=False)


class Token(db.Model):
    __tablename__ = "tokens"
    id = db.Column(db.Integer, primary_key=True)
    token = db.Column(db.String(128), unique=True, nullable=False)
    user_id = db.Column(db.Integer, db.ForeignKey("users.id"), nullable=False)
    expires_at = db.Column(db.DateTime, nullable=False)


with app.app_context():
    db.create_all()


@login_manager.user_loader
def load_user(user_id):
    return db.session.get(User, int(user_id))


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
# Admin routes
# ---------------------------------------------------------------------------

@app.route("/admin")
@login_required
def admin():
    """Admin dashboard for managing access tokens."""
    return render_template("admin.html")


@app.route("/admin/login", methods=["GET", "POST"])
def admin_login():
    return render_template("login.html")


@app.route("/admin/create_token", methods=["POST"])
@login_required
def create_token():
    pass


@app.route("/admin/revoke_token", methods=["POST"])
@login_required
def revoke_token():
    pass


@app.route("/admin/expire_token", methods=["POST"])
@login_required
def expire_token():
    pass


@app.route("/admin/tokens")
@login_required
def tokens():
    pass


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
