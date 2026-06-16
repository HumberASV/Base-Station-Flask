"""
Default telemetry state factory.

Provides the initial/fallback Status payload that matches the web client's
Status type (statusTypes.ts). Used when ROS2 topics are unavailable.
"""


def make_default_state() -> dict:
    """
    Returns an idle Status dict with zero/empty values.
    Fields are updated in-place by ros2_bridge as real data arrives.
    """
    return {
        "map": {
            "occupancyGrid": [],
            "navigationGrid": [],
            "fineGrid": [],
            "courseTrail": [],
        },
        "planning": {
            "status": "standby",
            "course": [],
            "plan": [],
        },
        "task": {
            "log": ["Base station started — awaiting ASV connection."],
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
        "signal": {"strength": 0.0},
        "video": {"streamUrl": ""},
    }
