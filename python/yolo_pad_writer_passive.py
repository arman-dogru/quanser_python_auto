"""Phase 5 passive QDrone landing-pad monitor.

This process owns screen capture and YOLO inference. It writes observations
only; it has no connection to the Mission Server DesiredPose signal.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import os
import socket
import struct
import time
from datetime import datetime
from pathlib import Path
from ctypes import wintypes

import cv2
import numpy as np
from PIL import ImageGrab
from ultralytics import YOLO


HEADERS = [
    "timestamp", "detected", "centerX", "centerY", "confidence", "frame",
    "capture_ms", "inference_ms", "loop_ms", "observation_age_ms",
]


def parse_args() -> argparse.Namespace:
    default_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Passive QDrone YOLO monitor")
    parser.add_argument("--project-root", type=Path, default=default_root)
    parser.add_argument("--confidence", type=float, default=0.60)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--window-title", default="Optical Flow Camera feed",
                        help="Unique text in the QUARC video-window title")
    parser.add_argument("--list-windows", action="store_true",
                        help="List visible window titles and exit")
    parser.add_argument("--manual-roi", action="store_true",
                        help="Use manual full-screen selection as a fallback")
    parser.add_argument("--reselect-roi", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument("--tcp-port", type=int, default=0,
                        help="Serve each observation as 6 little-endian doubles; "
                             "0 keeps the original file-only passive mode")
    return parser.parse_args()


def select_roi(roi_file: Path) -> tuple[int, int, int, int]:
    full = cv2.cvtColor(np.array(ImageGrab.grab()), cv2.COLOR_RGB2BGR)
    title = "Select complete 640x480 GRAYSCALE DOWNWARD camera image, then press ENTER"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    x, y, width, height = cv2.selectROI(title, full, False, True)
    cv2.destroyWindow(title)
    if width <= 0 or height <= 0:
        raise RuntimeError("No camera ROI selected.")
    roi_file.parent.mkdir(parents=True, exist_ok=True)
    roi_file.write_text(f"{x},{y},{width},{height}", encoding="utf-8")
    return int(x), int(y), int(width), int(height)


def get_roi(roi_file: Path, force: bool) -> tuple[int, int, int, int]:
    if roi_file.exists() and not force:
        values = tuple(int(value) for value in roi_file.read_text().strip().split(","))
        if len(values) == 4 and values[2] > 0 and values[3] > 0:
            return values
    return select_roi(roi_file)


def visible_windows() -> list[tuple[int, str]]:
    """Return visible top-level Windows windows as (handle, title)."""
    user32 = ctypes.windll.user32
    try:
        user32.SetProcessDPIAware()
    except AttributeError:
        pass
    windows: list[tuple[int, str]] = []
    callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)

    def callback(handle, _):
        if user32.IsWindowVisible(handle):
            length = user32.GetWindowTextLengthW(handle)
            if length:
                buffer = ctypes.create_unicode_buffer(length + 1)
                user32.GetWindowTextW(handle, buffer, length + 1)
                title = buffer.value.strip()
                if title:
                    windows.append((int(handle), title))
        return True

    user32.EnumWindows(callback_type(callback), 0)
    return windows


def client_bbox(handle: int) -> tuple[int, int, int, int]:
    """Get a top-level window's client rectangle in physical screen pixels."""
    user32 = ctypes.windll.user32
    rect = wintypes.RECT()
    origin = wintypes.POINT(0, 0)
    if not user32.GetClientRect(wintypes.HWND(handle), ctypes.byref(rect)):
        raise RuntimeError("Could not read the camera-window client rectangle.")
    if not user32.ClientToScreen(wintypes.HWND(handle), ctypes.byref(origin)):
        raise RuntimeError("Could not locate the camera window on the screen.")
    width = int(rect.right - rect.left)
    height = int(rect.bottom - rect.top)
    if width < 320 or height < 240:
        raise RuntimeError(f"Camera client area is too small: {width}x{height}.")
    return int(origin.x), int(origin.y), width, height


def centered_four_three(x: int, y: int, width: int, height: int):
    """Take the largest centered 4:3 image inside a letter/pillar-boxed client."""
    target_ratio = 4.0 / 3.0
    if width / height > target_ratio:
        crop_height = height
        crop_width = int(round(height * target_ratio))
    else:
        crop_width = width
        crop_height = int(round(width / target_ratio))
    crop_x = x + (width - crop_width) // 2
    crop_y = y + (height - crop_height) // 2
    ratio = crop_width / crop_height
    if abs(ratio - target_ratio) > 0.01:
        raise RuntimeError(f"Automatic camera crop is not 4:3: {crop_width}x{crop_height}.")
    return crop_x, crop_y, crop_width, crop_height


def automatic_camera_bbox(title_fragment: str):
    windows = visible_windows()
    matches = [(handle, title) for handle, title in windows
               if title_fragment.casefold() in title.casefold()]
    if not matches:
        likely = [title for _, title in windows
                  if any(word in title.casefold() for word in ("video", "camera", "optical"))]
        details = "\n  ".join(likely) if likely else "(none)"
        raise RuntimeError(
            f'No visible window title contains "{title_fragment}".\n'
            f"Likely camera windows:\n  {details}\n"
            "Use --list-windows, then pass --window-title with unique title text."
        )
    if len(matches) > 1:
        exact = [item for item in matches if item[1].casefold() == title_fragment.casefold()]
        if len(exact) == 1:
            matches = exact
        else:
            titles = "\n  ".join(title for _, title in matches)
            raise RuntimeError(
                f'Window title text "{title_fragment}" is ambiguous:\n  {titles}\n'
                "Pass a more specific --window-title value."
            )
    handle, title = matches[0]
    x, y, width, height = client_bbox(handle)
    bbox = centered_four_three(x, y, width, height)
    return title, (x, y, width, height), bbox


def best_detection(results, class_id: int = 0):
    best = None
    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            if int(box.cls[0].item()) != class_id:
                continue
            confidence = float(box.conf[0].item())
            if best is None or confidence > best[0]:
                best = confidence, box.xyxy[0].cpu().numpy()
    if best is None:
        return False, float("nan"), float("nan"), float("nan"), None
    confidence, bounds = best
    x1, y1, x2, y2 = bounds
    return True, float((x1 + x2) / 2), float((y1 + y2) / 2), confidence, bounds


def row_values(timestamp, detected, center_x, center_y, confidence, frame,
               capture_ms, inference_ms, loop_ms, age_ms):
    return [
        f"{timestamp:.6f}", int(detected),
        f"{center_x:.2f}" if detected else "NaN",
        f"{center_y:.2f}" if detected else "NaN",
        f"{confidence:.4f}" if detected else "NaN", frame,
        f"{capture_ms:.3f}", f"{inference_ms:.3f}", f"{loop_ms:.3f}",
        f"{age_ms:.3f}",
    ]


def atomic_latest(path: Path, row, retries: int = 20) -> bool:
    """Replace the latest-value CSV without terminating on a transient lock."""
    temporary = path.with_suffix(".tmp")
    for attempt in range(retries):
        try:
            with temporary.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(HEADERS)
                writer.writerow(row)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            return True
        except (PermissionError, OSError):
            if attempt + 1 < retries:
                time.sleep(0.02)
    return False


class LatestObservationServer:
    """Non-blocking, single-client TCP publisher for the host Mission Server."""

    def __init__(self, port: int):
        self.port = port
        self.listener = None
        self.client = None
        if port:
            self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind(("127.0.0.1", port))
            self.listener.listen(1)
            self.listener.setblocking(False)

    def publish(self, detected, center_x, center_y, confidence, frame, timestamp):
        if not self.listener:
            return
        if self.client is None:
            try:
                self.client, address = self.listener.accept()
                self.client.setblocking(False)
                print(f"Mission Server connected from {address[0]}:{address[1]}")
            except BlockingIOError:
                return
        values = (
            float(bool(detected)),
            float(center_x) if detected else 0.0,
            float(center_y) if detected else 0.0,
            float(confidence) if detected else 0.0,
            float(frame),
            float(timestamp),
        )
        try:
            self.client.sendall(struct.pack("<6d", *values))
        except (BlockingIOError, BrokenPipeError, ConnectionResetError, OSError):
            try:
                self.client.close()
            finally:
                self.client = None

    def close(self):
        if self.client:
            self.client.close()
        if self.listener:
            self.listener.close()


def main() -> None:
    args = parse_args()
    if args.list_windows:
        for _, title in visible_windows():
            print(title)
        return
    root = args.project_root.resolve()
    model_path = root / "models" / "qdrone_landing_pad_yolo11n_v3_best.pt"
    data_dir = root / "data"
    log_dir = root / "logs"
    roi_file = data_dir / "qdrone_camera_screen_roi.txt"
    latest_file = data_dir / "pad_detection.csv"
    session_file = log_dir / f"yolo_passive_{datetime.now():%Y%m%d_%H%M%S}.csv"
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")
    data_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    if args.manual_roi or args.reselect_roi:
        x, y, width, height = get_roi(roi_file, True)
        print(f"Manual camera ROI: {x},{y},{width},{height}")
    else:
        window_title, client, bbox = automatic_camera_bbox(args.window_title)
        x, y, width, height = bbox
        print(f'Camera window: "{window_title}"')
        print(f"Window client area: {client[2]}x{client[3]} at ({client[0]},{client[1]})")
        print(f"Automatic centered 4:3 capture: {width}x{height} at ({x},{y})")
    model = YOLO(str(model_path))
    print(f"Session log: {session_file}")
    publisher = LatestObservationServer(args.tcp_port)
    if args.tcp_port:
        print(f"YOLO observation server: tcpip://localhost:{args.tcp_port}")
        print("INTERRUPT-ONLY MODE: confirmed detections may freeze the grid at a hover;")
        print("camera data cannot command XY motion or descent.")
    else:
        print("PASSIVE MODE: detections cannot modify DesiredPose.")

    frame_index = 0
    latest_write_failures = 0
    period = 1.0 / max(args.fps, 0.1)
    with session_file.open("w", newline="", encoding="utf-8", buffering=1) as history:
        history_writer = csv.writer(history)
        history_writer.writerow(HEADERS)
        while True:
            loop_start = time.perf_counter()
            observation_timestamp = time.time()
            capture_start = time.perf_counter()
            image = ImageGrab.grab(bbox=(x, y, x + width, y + height))
            capture_ms = (time.perf_counter() - capture_start) * 1000
            frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
            frame = cv2.resize(frame, (640, 480))
            # The Phase 5 source is exclusively the downward OV7251 grayscale
            # feed. Replicate that single channel for YOLO's 3-channel input.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            inference_start = time.perf_counter()
            results = model(frame, conf=args.confidence, verbose=False)
            inference_ms = (time.perf_counter() - inference_start) * 1000
            detected, center_x, center_y, confidence, bounds = best_detection(results)
            publisher.publish(detected, center_x, center_y, confidence,
                              frame_index, observation_timestamp)
            loop_ms = (time.perf_counter() - loop_start) * 1000
            age_ms = max(0.0, (time.time() - observation_timestamp) * 1000)
            row = row_values(observation_timestamp, detected, center_x, center_y,
                             confidence, frame_index, capture_ms, inference_ms,
                             loop_ms, age_ms)
            history_writer.writerow(row)
            if atomic_latest(latest_file, row):
                latest_write_failures = 0
            else:
                latest_write_failures += 1
                if latest_write_failures == 1 or latest_write_failures % 25 == 0:
                    print("WARNING: pad_detection.csv is locked; the complete "
                          "session log is still recording. Close any program "
                          "that has the latest-value CSV open.")

            if detected:
                x1, y1, x2, y2 = bounds
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                status = f"PAD {confidence:.3f}  ({center_x:.0f},{center_y:.0f})"
            else:
                status = "NO PAD"
            cv2.circle(frame, (320, 240), 5, (255, 0, 0), -1)
            cv2.putText(frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if detected else (0, 0, 255), 2)
            print(f"frame={frame_index} detected={int(detected)} "
                  f"confidence={confidence if detected else float('nan'):.3f} "
                  f"age_ms={age_ms:.1f}")

            if not args.no_preview:
                cv2.imshow("PASSIVE QDrone YOLO - q to stop", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            frame_index += 1
            time.sleep(max(0.0, period - (time.perf_counter() - loop_start)))
    cv2.destroyAllWindows()
    publisher.close()
    print(f"Saved {frame_index + 1} observations to {session_file}")


if __name__ == "__main__":
    main()
