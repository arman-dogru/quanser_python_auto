"""Direct QDrone YOLO with fail-closed raw-image freshness checks.

Run this INSTEAD OF yolo_pad_direct_stream.py, never alongside it. The wire
format sent to the Mission Server is unchanged. Rejected frames are not sent,
so the Mission Server's existing vision-staleness timer can expire.
"""

import csv
import socket
import time
from datetime import datetime

import cv2
from ultralytics import YOLO

from camera_freshness import CameraFreshness
from yolo_pad_direct_stream import make_camera_listener, parse_args, receive_frame
from yolo_pad_writer_passive import (
    HEADERS, LatestObservationServer, atomic_latest, best_detection, row_values,
)


HEALTH_HEADERS = (
    "timestamp", "raw_frame", "image_crc32", "status",
    "raw_duplicate_count", "same_detection_count",
)


def show_stale_preview(gray, reason):
    frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.putText(frame, "CAMERA/DETECTION STALE - NO OBSERVATION SENT", (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 255), 2)
    cv2.putText(frame, reason, (8, 55), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2)
    cv2.imshow("DIRECT QDrone YOLO - q to stop", frame)
    return cv2.waitKey(1) & 0xFF == ord("q")


def main():
    args = parse_args()
    root = args.project_root.resolve()
    weights = root / "models" / "qdrone_landing_pad_yolo11n_v3_best.pt"
    latest_file = root / "data" / "pad_detection.csv"
    log_dir = root / "logs"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_file = log_dir / f"yolo_direct_fresh_{stamp}.csv"
    health_file = log_dir / f"camera_freshness_{stamp}.csv"
    if not weights.is_file():
        raise FileNotFoundError(f"YOLO model not found: {weights}")
    latest_file.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(weights))
    guard = CameraFreshness(max_same_detection=5)
    publisher = LatestObservationServer(args.detection_port)
    listener = make_camera_listener(args.listen_address, args.camera_port)
    connection = None
    raw_index = 0
    accepted = 0
    print(f"Detection log: {session_file}")
    print(f"Camera freshness log: {health_file}")
    print("FRESH MODE: repeated images/results are withheld from Mission Server.")
    print("Run this script instead of yolo_pad_direct_stream.py, not together.")

    with session_file.open("w", newline="", encoding="utf-8", buffering=1) as history, \
            health_file.open("w", newline="", encoding="utf-8", buffering=1) as health:
        writer = csv.writer(history)
        health_writer = csv.writer(health)
        writer.writerow(HEADERS)
        health_writer.writerow(HEALTH_HEADERS)
        try:
            while True:
                if connection is None:
                    try:
                        connection, address = listener.accept()
                    except socket.timeout:
                        continue
                    connection.settimeout(2.0)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    guard.reset_camera_connection()
                    print(f"QUARC camera connected from {address[0]}:{address[1]}")

                loop_start = time.perf_counter()
                try:
                    gray = receive_frame(connection)
                except (ConnectionError, OSError, socket.timeout, RuntimeError) as error:
                    print(f"Camera stream lost; observations withheld: {error}")
                    connection.close()
                    connection = None
                    guard.reset_camera_connection()
                    continue

                raw_index += 1
                now = time.time()
                fresh, crc = guard.check_image(gray)
                if not fresh:
                    health_writer.writerow((f"{now:.6f}", raw_index, f"{crc:08x}",
                                            "raw_duplicate", guard.raw_duplicate_count,
                                            guard.same_detection_count))
                    if guard.raw_duplicate_count == 1 or guard.raw_duplicate_count % 10 == 0:
                        print(f"CAMERA STALE: raw frame repeated {guard.raw_duplicate_count} times; no detection sent")
                    if not args.no_preview and show_stale_preview(gray, "Repeated raw camera image"):
                        break
                    continue

                receive_ms = (time.perf_counter() - loop_start) * 1000.0
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                inference_start = time.perf_counter()
                results = model(frame, conf=args.confidence, verbose=False)
                inference_ms = (time.perf_counter() - inference_start) * 1000.0
                detected, center_x, center_y, confidence, bounds = best_detection(results)
                if not guard.check_detection(detected, center_x, center_y, confidence):
                    health_writer.writerow((f"{now:.6f}", raw_index, f"{crc:08x}",
                                            "detection_unchanged", 0,
                                            guard.same_detection_count))
                    if guard.same_detection_count == guard.max_same_detection or guard.same_detection_count % 10 == 0:
                        print(f"DETECTION STALE: identical result {guard.same_detection_count} times; no detection sent")
                    if not args.no_preview and show_stale_preview(gray, "Repeated YOLO result"):
                        break
                    continue

                # Only a changed image and non-stale result receives a new
                # frame number and timestamp on the control path.
                observation_timestamp = time.time()
                publisher.publish(detected, center_x, center_y, confidence,
                                  raw_index, observation_timestamp)
                accepted += 1
                health_writer.writerow((f"{now:.6f}", raw_index, f"{crc:08x}",
                                        "accepted_for_publish", 0,
                                        guard.same_detection_count))
                loop_ms = (time.perf_counter() - loop_start) * 1000.0
                age_ms = max(0.0, (time.time() - observation_timestamp) * 1000.0)
                row = row_values(observation_timestamp, detected, center_x, center_y,
                                 confidence, raw_index, receive_ms, inference_ms,
                                 loop_ms, age_ms)
                writer.writerow(row)
                atomic_latest(latest_file, row)

                if detected:
                    x1, y1, x2, y2 = bounds
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 255, 0), 2)
                    cv2.circle(frame, (int(center_x), int(center_y)), 5,
                               (0, 0, 255), -1)
                    status = f"PAD {confidence:.3f} ({center_x:.0f},{center_y:.0f})"
                else:
                    status = "NO PAD"
                cv2.circle(frame, (320, 240), 5, (255, 0, 0), -1)
                cv2.putText(frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0) if detected else (0, 0, 255), 2)
                print(f"frame={raw_index} detected={int(detected)} "
                      f"confidence={confidence if detected else float('nan'):.3f} "
                      f"age_ms={age_ms:.1f}")
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
    print(f"Published {accepted} observations from {raw_index} raw frames")
    print(f"Detection log: {session_file}")
    print(f"Camera freshness log: {health_file}")


if __name__ == "__main__":
    main()
