"""Ground YOLO receiving raw grayscale frames from a QUARC Commander."""

from __future__ import annotations

import argparse
import csv
import os
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from yolo_pad_writer_passive import (
    HEADERS, LatestObservationServer, atomic_latest, best_detection, row_values,
)


WIDTH = 640
HEIGHT = 480
FRAME_BYTES = WIDTH * HEIGHT


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Direct-stream QDrone YOLO")
    parser.add_argument("--project-root", type=Path, default=root)
    parser.add_argument("--listen-address", default="0.0.0.0")
    parser.add_argument("--camera-port", type=int, default=18910)
    parser.add_argument("--detection-port", type=int, default=18901)
    parser.add_argument("--confidence", type=float, default=0.60)
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def recv_exact(connection: socket.socket, count: int) -> bytes:
    chunks = bytearray(count)
    view = memoryview(chunks)
    received = 0
    while received < count:
        size = connection.recv_into(view[received:])
        if size == 0:
            raise ConnectionError("QDrone camera stream closed")
        received += size
    return bytes(chunks)


def receive_frame(connection: socket.socket):
    pixels = recv_exact(connection, FRAME_BYTES)
    # Simulink matrices use column-major storage. Reconstruct the HxW image
    # with Fortran ordering so the ground preview is not transposed.
    return np.frombuffer(pixels, dtype=np.uint8).reshape(
        (HEIGHT, WIDTH), order="F")


def make_camera_listener(address: str, port: int) -> socket.socket:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((address, port))
    listener.listen(1)
    listener.settimeout(1.0)
    print(f"Waiting for QUARC camera stream on tcp://{address}:{port}")
    return listener


def main() -> None:
    args = parse_args()
    root = args.project_root.resolve()
    weights = root / "models" / "qdrone_landing_pad_yolo11n_v3_best.pt"
    data_dir = root / "data"
    log_dir = root / "logs"
    latest_file = data_dir / "pad_detection.csv"
    session_file = log_dir / f"yolo_direct_{datetime.now():%Y%m%d_%H%M%S}.csv"
    if not weights.is_file():
        raise FileNotFoundError(f"YOLO model not found: {weights}")
    data_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(weights))
    publisher = LatestObservationServer(args.detection_port)
    print(f"Session log: {session_file}")
    print(f"Detection output: tcpip://localhost:{args.detection_port}")
    print("DIRECT MODE: no MATLAB window, screen capture, or ROI is used.")

    frame_index = 0
    listener = make_camera_listener(args.listen_address, args.camera_port)
    connection = None
    with session_file.open("w", newline="", encoding="utf-8", buffering=1) as history:
        writer = csv.writer(history)
        writer.writerow(HEADERS)
        try:
            while True:
                if connection is None:
                    try:
                        connection, address = listener.accept()
                    except socket.timeout:
                        continue
                    connection.settimeout(2.0)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print(f"QUARC camera connected from {address[0]}:{address[1]}")
                loop_start = time.perf_counter()
                try:
                    gray = receive_frame(connection)
                except (ConnectionError, OSError, socket.timeout, RuntimeError) as error:
                    print(f"Camera stream lost: {error}")
                    connection.close()
                    connection = None
                    continue

                observation_timestamp = time.time()
                receive_ms = (time.perf_counter() - loop_start) * 1000.0
                gray = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_LINEAR)
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                inference_start = time.perf_counter()
                results = model(frame, conf=args.confidence, verbose=False)
                inference_ms = (time.perf_counter() - inference_start) * 1000.0
                detected, center_x, center_y, confidence, bounds = best_detection(results)
                publisher.publish(detected, center_x, center_y, confidence,
                                  frame_index, observation_timestamp)
                loop_ms = (time.perf_counter() - loop_start) * 1000.0
                age_ms = max(0.0, (time.time() - observation_timestamp) * 1000.0)
                row = row_values(observation_timestamp, detected, center_x, center_y,
                                 confidence, frame_index, receive_ms, inference_ms,
                                 loop_ms, age_ms)
                writer.writerow(row)
                atomic_latest(latest_file, row)

                if detected:
                    x1, y1, x2, y2 = bounds
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 255, 0), 2)
                    cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    status = f"PAD {confidence:.3f} ({center_x:.0f},{center_y:.0f})"
                else:
                    status = "NO PAD"
                cv2.circle(frame, (320, 240), 5, (255, 0, 0), -1)
                cv2.putText(frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0) if detected else (0, 0, 255), 2)
                print(f"frame={frame_index} detected={int(detected)} "
                      f"confidence={confidence if detected else float('nan'):.3f} "
                      f"age_ms={age_ms:.1f}")
                frame_index += 1
                if not args.no_preview:
                    cv2.imshow("DIRECT QDrone YOLO - q to stop", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        finally:
            if connection is not None:
                connection.close()
            listener.close()
            publisher.close()
            cv2.destroyAllWindows()
    print(f"Saved {frame_index} observations to {session_file}")


if __name__ == "__main__":
    main()
