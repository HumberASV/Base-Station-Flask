"""
Default telemetry state factory.

Provides the initial/fallback Status payload that matches the web client's
Status type (statusTypes.ts). Used when ROS2 topics are unavailable.
"""


def make_default_state() -> dict:
    """
    Returns an idle Status dict with zero/empty values, mirroring
    visualizer/src/types/statusTypes.ts's InitialStatus.
    Fields are updated in-place by ros2_bridge as real data arrives.
    """
    return {
        "map": {
            "occupancyGrid": [],
            "navigationGrid": [],
            "courseTrail": [],
        },
        "planning": {
            "status": "idle",
            "course": [],
            "plan": [],
        },
        "task": {
            "log": [],
            "location": {"latitude": 0.0, "longitude": 0.0},
            "data": {"id": 0, "name": "", "status": "standby", "latitude": 0.0, "longitude": 0.0},
        },
        "rudder": {
            "angle": 0.0,
        },
        "battery": {
            "motors": 0.0,
            "primary": 0.0,
        },
        "motors": {
            "left": 0.0,
            "right": 0.0,
        },
        "asv": {
            "speed": 0.0,
            "heading": 0.0,
            "longitude": 0.0,
            "latitude": 0.0,
        },
        "signal": {
            "strength": 100.0,
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
