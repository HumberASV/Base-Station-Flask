
"""
"map": {
            "occupancyGrid": [],
            "navigationGrid": [],
        },
        "planning": {
            "status": "standby",
            "course": [],
            "plan": [],
        },
        "task": {
            "log": random.choice(logs),
            "location": {"latitude": 0.0, "longitude": 0.0},
            "data": {
                "id": 0,
                "name": "No active task",
                "status": "standby",
                "latitude": 0.0,
                "longitude": 0.0,
            },
        },
        "rudder": {"angle": 0.0},
        "motors": {"left": 0.0, "right": 0.0},
        "power": {"motors": 0.0, "primary": 0.0},
        "asv": {
            "speed": 0.0,
            "heading": 0.0,
            "longitude": 0.0,
            "latitude": 0.0,
        },
        "signal": {
            "strength": 0.0,
        },
"""

class TelemetryState:
    """
    A class to hold the shared telemetry state for the Flask application.
    This state is updated by the ROS2 bridge and read by WebSocket clients.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "timestamp": None,
            "position": None,
            "orientation": None,
            "velocity": None,
            "battery": None,
            "sensors": {},
        }

    def update(self, new_data):
        with self.lock:
            self.data.update(new_data)