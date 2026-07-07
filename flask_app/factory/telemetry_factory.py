"""
Default telemetry state factory.

Provides the initial/fallback Status payload that matches the web client's
Status type (statusTypes.ts). Used when ROS2 topics are unavailable.
"""
import random

logs = [
    "Canadians call their 2 dollar coin a toonie",
    "Terry Fox is a hero",
    "Maple Syrup is great",
    "Eh?",
    "Canadians do not like to be called Americans",
    "Yes, we measure height in feet and drive km/h",
    "Canada manufactors lots of cars",
    "Canada is a great trade partner",
    "Canada has the largest undefended land border with the USA",
    "Canada shares a land border with Denmark",
    "We even sell maple syrup at the dollar store",
    "We have two seasons: winter and construction",
    "Canadians sometimes say sorry when they are not at fault"
]

def make_default_state() -> dict:
    """
    Returns an idle Status dict with zero/empty values.
    Fields are updated in-place by ros2_bridge as real data arrives.
    """
    return {
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
            "log": [random.choice(logs)],
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
        "zed": {
            "odom": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            },
            "objects": [],
            "camera": {
                "active": False,
                "width": 0,
                "height": 0,
                "encoding": "",
            },
        },
    }
