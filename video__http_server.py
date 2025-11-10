#! /usr/bin/env python3

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclply
from rxlpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np

class FrameBuffer:
    """Thread-safe buffer for latest JPEG frame."""
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg = None

    def update_from_bgr(self, frame_bgr):
        ok, buf = cv2.imencode('.jpg', frame_bgr)
        if ok:
            with self._lock:
                self._jpg = buf.tobytes()

    def get(self):
        with self._lock:
            return self._jpg
        
frame_buffer = FrameBuffer()

class ImageSubscriber(Node):
    """Subscribes to a ROS2 image topic and keeps the lastest frame in encoded as JPEG in frame_buffer."""
    def __init__(self, topic_name: str):
        super().__init__('video_http_server_node')
        self.topic_name = topic_name
        self.get_logger().info(f'Subscribing to image topic: {self.topic_name}')
        self.sub = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        # convert sensor_msgs/Image to OpenCV BGR image
        h = msg.height
        w = msg.width
        c = msg.step // w

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, c))

        if msg.encoding.lower().startswith('rgb'):
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        frame_buffer.update_from_bgr(frame_bgr)

class MJPEGRequestHandler(BaseHTTPRequestHandler):
    """Serves multipart/x-mixed-replace MJPEG stream at /stream."""
    def do_GET(self):
        if self.path != '/stream':
            self.send_error(404)
            self.end_headers()
            self.wfile.write(b'Not Found')
            return

        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        try:
            while True:
                jpg_bytes = frame_buffer.get()
                if jpg_bytes is None:
                    time.sleep(0.03)
                    continue
                
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpg_bytes)))
                self.end_headers()
                self.wfile.write(jpg_bytes)
                self.wfile.write(b'\r\n')

                time.sleep(0.03)

        except (BrokenPipeError, ConnectionResetError):
            pass

def start_http_server(port: int):
    """Start HTTP server in a background thread."""
    server =HTTPServer(('', port), MJPEGRequestHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server

def main():
    parser = argparse.ArgumentParser(description='ROS2 -> MJPEG HTTP bridge')
    parser.add_argument(
        '--topic',
        default='/camera/image_raw',
        help='ROS2 image topic to stream (sensor_msgs/msg/Image)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8080,
        help='TCP port for the HTTP MJPEG server'
    )
    args = parser.parse_args()

    rclply.init()

    node = ImageSubscriber(args.topic)

    server = start_http_server(args.port)
    node.get_logger().info(f'Serving MJPEG stream at http://0.0.0.0:{args.port}/stream')

    try:
        rclply.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclply.shutdown()

if __name__ == '__main__':
    main()