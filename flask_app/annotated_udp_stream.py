#!/usr/bin/env python3
"""
annotated_udp_stream.py — ZED annotated MJPEG stream over UDP.

Subscribes to ZED camera image and object detection topics, draws 2D
bounding box tags on each frame, and broadcasts JPEG frames via UDP
to any registered client.

Usage
-----
    python annotated_udp_stream.py [--port 9999]
                                   [--image-topic zed/rgb/color/rect/image]
                                   [--objects-topic zed/obj_det/objects]
                                   [--quality 80]

Client registration
-------------------
A client registers by sending any UDP datagram to the server port.
The server records the sender (IP, port) and streams frames back to it.
Clients that stop receiving (OSError on sendto) are evicted automatically.

Frame protocol
--------------
Each UDP datagram: MAGIC(4) | FRAME_ID(4) | CHUNK_IDX(2) | NUM_CHUNKS(2) | JPEG_DATA

magic = b'ZFRM'

Large JPEG frames are split into <=60 KB chunks so they stay under the UDP
practical MTU. The client reassembles by collecting all chunks with the same
FRAME_ID before decoding.
"""

import argparse
import socket
import struct
import threading

try:
    import ros2_bridge as _ros2_bridge
except ImportError:
    _ros2_bridge = None  # type: ignore[assignment]

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _ROS2_OK = True
except ImportError:
    _ROS2_OK = False

    class Node:  # type: ignore[no-redef]
        def __init__(self, *a, **kw): pass
        def get_logger(self): return _FallbackLog()
        def create_subscription(self, *a, **kw): pass
        def destroy_node(self): pass

    class _FallbackLog:
        def info(self, m): print(f'[INFO] {m}')
        def warn(self, m): print(f'[WARN] {m}')
        def error(self, m): print(f'[ERR]  {m}')

try:
    from zed_interfaces.msg import ObjectsStamped
    _ZED_OK = True
except ImportError:
    _ZED_OK = False

_MAGIC = b'ZFRM'
_HDR_FMT = '>4sIHH'                    # magic(4) frame_id(4) chunk_idx(2) num_chunks(2)
_HDR_SIZE = struct.calcsize(_HDR_FMT)  # 12 bytes
_CHUNK_BYTES = 60_000                   # JPEG payload bytes per UDP datagram


class _UDPStreamer:
    """Registers clients (any incoming datagram) and broadcasts chunked JPEG frames."""

    def __init__(self, port: int):
        self._clients: set[tuple[str, int]] = set()
        self._lock = threading.Lock()
        self._frame_id = 0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self._sock.bind(('', port))
        self._sock.settimeout(1.0)
        self._running = True

        threading.Thread(target=self._accept_loop, daemon=True, name='udp-accept').start()
        print(f'[UDP] Listening for clients on :{port}')

    def _accept_loop(self):
        while self._running:
            try:
                _, addr = self._sock.recvfrom(64)
                with self._lock:
                    if addr not in self._clients:
                        self._clients.add(addr)
                        print(f'[UDP] Client registered {addr[0]}:{addr[1]}')
            except socket.timeout:
                continue
            except OSError:
                break

    def send_frame(self, jpeg: bytes):
        with self._lock:
            if not self._clients:
                return
            clients = list(self._clients)

        self._frame_id = (self._frame_id + 1) & 0xFFFF_FFFF
        chunks = [jpeg[i:i + _CHUNK_BYTES] for i in range(0, len(jpeg), _CHUNK_BYTES)]
        n = len(chunks)
        dead: set[tuple[str, int]] = set()

        for idx, chunk in enumerate(chunks):
            pkt = struct.pack(_HDR_FMT, _MAGIC, self._frame_id, idx, n) + chunk
            for addr in clients:
                try:
                    self._sock.sendto(pkt, addr)
                except OSError:
                    dead.add(addr)

        if dead:
            with self._lock:
                self._clients -= dead
            for addr in dead:
                print(f'[UDP] Client dropped {addr[0]}:{addr[1]}')

    def stop(self):
        self._running = False
        self._sock.close()


def _draw_objects(frame: np.ndarray, objects: list) -> None:
    """Draw 2D bounding boxes and labels onto frame in-place."""
    for obj in objects:
        corners_msg = obj.bounding_box_2d.corners
        if not corners_msg:
            continue

        # corners are Keypoint2Di with kp uint32[2] = [x, y] in pixels
        pts = np.array([[int(c.kp[0]), int(c.kp[1])] for c in corners_msg], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        label = f'{obj.label} {obj.confidence:.0f}%'
        x0, y0 = pts[0]
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x0, y0 - th - 6), (x0 + tw + 4, y0), (0, 255, 0), cv2.FILLED)
        cv2.putText(frame, label, (x0 + 2, y0 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


class _AnnotatedStreamNode(Node):

    def __init__(self, streamer: _UDPStreamer, image_topic: str, objects_topic: str, quality: int):
        if _ROS2_OK:
            super().__init__('annotated_udp_stream')

        self._streamer = streamer
        self._quality = quality
        self._bridge = CvBridge() if _ROS2_OK else None
        self._objects: list = []
        self._obj_lock = threading.Lock()

        if not _ROS2_OK:
            return

        reliable_qos = QoSProfile(depth=10)
        video_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, image_topic, self._on_image, video_qos)

        if _ZED_OK:
            self.create_subscription(ObjectsStamped, objects_topic, self._on_objects, reliable_qos)
            self.get_logger().info(f'Subscribed to {image_topic} and {objects_topic}')
        else:
            self.get_logger().warn(
                f'zed_interfaces unavailable — subscribed to {image_topic} only (no overlay)'
            )

    def _on_objects(self, msg: 'ObjectsStamped'):
        with self._obj_lock:
            self._objects = list(msg.objects)

    def _on_image(self, msg: 'Image'):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        with self._obj_lock:
            objects = list(self._objects)

        _draw_objects(frame, objects)

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            jpeg = buf.tobytes()
            self._streamer.send_frame(jpeg)
            # Share annotated frame with ros2_bridge so Flask can serve it via HTTP.
            if _ros2_bridge is not None:
                _ros2_bridge.set_latest_frame(jpeg)


def main():
    ap = argparse.ArgumentParser(description='ZED annotated UDP stream')
    ap.add_argument('--port', type=int, default=9999)
    ap.add_argument('--image-topic', default='zed/rgb/color/rect/image')
    ap.add_argument('--objects-topic', default='zed/obj_det/objects')
    ap.add_argument('--quality', type=int, default=80, metavar='1-100')
    args = ap.parse_args()

    streamer = _UDPStreamer(args.port)

    if not _ROS2_OK:
        print('[ERR] rclpy not available — cannot subscribe to ROS2 topics. Exiting.')
        streamer.stop()
        return

    rclpy.init()
    node = _AnnotatedStreamNode(streamer, args.image_topic, args.objects_topic, args.quality)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        streamer.stop()


if __name__ == '__main__':
    main()
