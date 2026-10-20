"""Run on the QDrone compute module and stream its downward camera.

The process owns OV7251 camera device 2. It sends framed, raw 320x240
grayscale images to one ground-computer client. It does not access HIL and it
does not command the aircraft.
"""

import argparse
import socket
import struct
import time

import numpy as np
from quanser.multimedia import ImageDataType, ImageFormat, VideoCapture


MAGIC = b"QDRF"
VERSION = 1
HEADER = struct.Struct("!4sHHIIQ")


def arguments():
    parser = argparse.ArgumentParser(description="QDrone downward-camera TCP streamer")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18910)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    return parser.parse_args()


def send_session(connection, capture, image, output_period):
    frame_id = 0
    next_send = time.monotonic()
    while True:
        if not capture.read(image):
            time.sleep(0.001)
            continue
        now = time.monotonic()
        if now < next_send:
            continue
        # Pure NumPy decimation: 640x480 -> 320x240. No OpenCV is required on
        # the flight computer.
        small = np.ascontiguousarray(image[::2, ::2])
        payload = small.tobytes(order="C")
        header = HEADER.pack(
            MAGIC, VERSION, small.shape[1], small.shape[0], len(payload),
            time.time_ns())
        connection.sendall(header + payload)
        frame_id += 1
        next_send = now + output_period


def main():
    args = arguments()
    if args.fps <= 0 or args.camera_fps <= 0:
        raise ValueError("Frame rates must be positive.")

    image = np.zeros((480, 640), dtype=np.uint8)
    capture = VideoCapture(
        "video://localhost:2", args.camera_fps, 640, 480,
        ImageFormat.ROW_MAJOR_GREYSCALE, ImageDataType.UINT8, None, 0)
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.bind, args.port))
    listener.listen(1)

    try:
        capture.start()
        print("Downward camera active: device 2, 640x480 grayscale")
        print("Streaming 320x240 grayscale on tcp://{}:{}".format(args.bind, args.port))
        while True:
            connection, address = listener.accept()
            print("Ground computer connected: {}:{}".format(*address))
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                send_session(connection, capture, image, 1.0 / args.fps)
            except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError) as error:
                print("Ground computer disconnected: {}".format(error))
            finally:
                connection.close()
    except KeyboardInterrupt:
        print("Stopping camera streamer.")
    finally:
        capture.stop()
        capture.close()
        listener.close()


if __name__ == "__main__":
    main()
