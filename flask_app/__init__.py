import pyzed.sl as sl
import rospy
from time import time

def generate_data():
    """
    A background task to generate streaming data and emit it to connected clients.
    """
    while True:
        # Your data generation logic here
        data = "Your streaming data here"
        socketio.emit('data', {'data': data})
        time.sleep(1)  # Adjust the sleep time as needed

def generate_frames():
    """
    A background task to generate streaming frames (e.g., video frames) and emit them to connected clients.
    """
    while True:
        # Your frame generation logic here
        frame = "Your streaming frame here"
        socketio.emit('frame', {'frame': frame})
        time.sleep(1)  # Adjust the sleep time as needed