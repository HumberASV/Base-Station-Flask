import json
import logging
import os
import secrets
import threading
import time
from datetime import datetime, timedelta, timezone

import click
from flask import Flask, render_template, request, jsonify, Response, redirect, url_for, session, flash
from flask_login import LoginManager, UserMixin, login_required, login_user, logout_user, current_user
from flask_sqlalchemy import SQLAlchemy
from flask_sock import Sock
from werkzeug.security import generate_password_hash, check_password_hash

from factory import telemetry_factory
import ros2_bridge

_debug = os.getenv("FLASK_DEBUG", "0") == "1"
logging.basicConfig(level=logging.DEBUG if _debug else logging.INFO)

app = Flask(__name__)
app.secret_key = os.getenv("SECRET_KEY", "change-me-in-production")
app.config["SQLALCHEMY_DATABASE_URI"] = os.getenv("DATABASE_URL", "sqlite:///app.db")
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
app.config["DEBUG"] = _debug

# Set SKIP_STREAM_CHECK=1 in dev when no ASV is connected
_SKIP_STREAM_CHECK = os.getenv("SKIP_STREAM_CHECK", "0") == "1"

db = SQLAlchemy(app)
sock = Sock(app)

login_manager = LoginManager(app)
login_manager.login_view = "admin_login"

# ---------------------------------------------------------------------------
# Shared telemetry state — written by the ROS2 bridge, read by WebSocket clients
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()
_telemetry_state: dict = telemetry_factory.make_default_state()

ros2_bridge.start(_telemetry_state, _state_lock)
_telemetry_state["video"] = {"streamUrl": "/video_feed"}


# ---------------------------------------------------------------------------
# Database models
# ---------------------------------------------------------------------------

class User(UserMixin, db.Model):
    __tablename__ = "users"
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(80), unique=True, nullable=False)
    password_hash = db.Column(db.String(256), nullable=False)
    tokens = db.relationship("Token", backref="creator", lazy=True)


class Token(db.Model):
    __tablename__ = "tokens"
    id = db.Column(db.Integer, primary_key=True)
    token = db.Column(db.String(256), unique=True, nullable=False)  # pbkdf2 hash of plaintext
    label = db.Column(db.String(80), nullable=True, default="")
    user_id = db.Column(db.Integer, db.ForeignKey("users.id"), nullable=False)
    expires_at = db.Column(db.DateTime, nullable=False)


with app.app_context():
    db.create_all()
    # Add label column if it's missing (SQLite schema evolution without Alembic)
    try:
        from sqlalchemy import text as _text
        with db.engine.connect() as _conn:
            _cols = {row[1] for row in _conn.execute(_text("PRAGMA table_info(tokens)")).fetchall()}
            if "label" not in _cols:
                _conn.execute(_text("ALTER TABLE tokens ADD COLUMN label VARCHAR(80) DEFAULT ''"))
                _conn.commit()
    except Exception:
        pass


@login_manager.user_loader
def load_user(user_id):
    return db.session.get(User, int(user_id))


# ---------------------------------------------------------------------------
# CLI — create admin user (FR-35)
# ---------------------------------------------------------------------------

@app.cli.command("create-admin")
@click.argument("username")
@click.password_option()
def create_admin_cmd(username, password):
    """Create an admin user account."""
    if User.query.filter_by(username=username).first():
        click.echo(f"Error: user '{username}' already exists.")
        return
    user = User(
        username=username,
        password_hash=generate_password_hash(password, method="pbkdf2:sha256"),
    )
    db.session.add(user)
    db.session.commit()
    click.echo(f"Admin user '{username}' created.")


# ---------------------------------------------------------------------------
# WebSocket — telemetry stream (FR-24)
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
# Admin routes (FR-28 through FR-32, FR-35, FR-36)
# ---------------------------------------------------------------------------

@app.route("/admin")
@login_required
def admin():
    return render_template("admin.html")


@app.route("/admin/login", methods=["GET", "POST"])
def admin_login():
    if current_user.is_authenticated:
        return redirect(url_for("admin"))
    if request.method == "POST":
        username = request.form.get("username", "").strip()
        password = request.form.get("password", "")
        user = User.query.filter_by(username=username).first()
        if user and check_password_hash(user.password_hash, password):
            login_user(user)
            return redirect(url_for("admin"))
        flash("Invalid username or password.", "error")
    return render_template("login.html")


@app.route("/admin/logout")
@login_required
def admin_logout():
    logout_user()
    return redirect(url_for("admin_login"))


@app.route("/admin/create_token", methods=["POST"])
@login_required
def create_token():
    label = request.form.get("label", "").strip()[:80]
    try:
        days = max(1, min(int(request.form.get("days", 1)), 365))
    except (ValueError, TypeError):
        days = 1

    plaintext = secrets.token_urlsafe(32)
    token_hash = generate_password_hash(plaintext, method="pbkdf2:sha256")
    expires = datetime.now(timezone.utc) + timedelta(days=days)

    tok = Token(
        token=token_hash,
        label=label,
        user_id=current_user.id,
        expires_at=expires,
    )
    db.session.add(tok)
    db.session.commit()

    return jsonify({
        "id": tok.id,
        "token": plaintext,
        "label": label,
        "expires_at": expires.isoformat(),
    })


@app.route("/admin/revoke_token", methods=["POST"])
@login_required
def revoke_token():
    token_id = request.form.get("token_id", type=int)
    tok = db.session.get(Token, token_id)
    if not tok:
        return jsonify({"error": "Token not found"}), 404
    db.session.delete(tok)
    db.session.commit()
    return jsonify({"status": "revoked", "id": token_id})


@app.route("/admin/expire_token", methods=["POST"])
@login_required
def expire_token():
    token_id = request.form.get("token_id", type=int)
    tok = db.session.get(Token, token_id)
    if not tok:
        return jsonify({"error": "Token not found"}), 404
    tok.expires_at = datetime.now(timezone.utc) - timedelta(seconds=1)
    db.session.commit()
    return jsonify({"status": "expired", "id": token_id})


@app.route("/admin/tokens")
@login_required
def tokens():
    now = datetime.now(timezone.utc)
    result = []
    for t in Token.query.all():
        expires = t.expires_at
        if expires.tzinfo is None:
            expires = expires.replace(tzinfo=timezone.utc)
        result.append({
            "id": t.id,
            "label": t.label or "",
            "expires_at": expires.isoformat(),
            "active": expires > now,
            "created_by": t.creator.username,
        })
    return jsonify(result)


# ---------------------------------------------------------------------------
# Client / error routes (FR-33, FR-34)
# ---------------------------------------------------------------------------

def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _token_expires(tok: Token) -> datetime:
    """Return expires_at as an aware UTC datetime."""
    exp = tok.expires_at
    return exp if exp.tzinfo else exp.replace(tzinfo=timezone.utc)


@app.route("/client", methods=["GET", "POST"])
def client():
    # FR-33: verify streams before granting access
    if not _SKIP_STREAM_CHECK:
        live = ros2_bridge.streams_live()
        if not live["telemetry"] and not live["video"]:
            return render_template(
                "uh_oh.html",
                message="No data is being received from the ASV. "
                        "Please try again when the vehicle is online.",
            )

    # Resume existing session if the token is still valid
    token_id = session.get("client_token_id")
    if token_id:
        tok = db.session.get(Token, token_id)
        if tok and _token_expires(tok) > _now_utc():
            return render_template("client.html", authorized=True)
        session.pop("client_token_id", None)

    if request.method == "POST":
        raw = request.form.get("token", "").strip()
        # Sanitize: printable ASCII only, bounded length
        raw = "".join(c for c in raw if c.isprintable() and ord(c) < 128)[:128]

        now = _now_utc()
        for tok in Token.query.all():
            if _token_expires(tok) > now and check_password_hash(tok.token, raw):
                session["client_token_id"] = tok.id
                return redirect(url_for("client"))

        return redirect(url_for("uh_oh", reason="invalid_token"))

    return render_template("client.html", authorized=False)


@app.route("/uh_oh")
def uh_oh():
    _messages = {
        "invalid_token": "The token you entered is invalid or has expired.",
        "streams_down": "No data is being received from the ASV.",
    }
    message = _messages.get(request.args.get("reason", ""),
                             "You do not have permission to access this page.")
    return render_template("uh_oh.html", message=message)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True, debug=_debug, use_reloader=False)
