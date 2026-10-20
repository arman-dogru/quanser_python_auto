"""Fail-closed freshness checks for the QDrone's raw downward-camera feed.

Only a changed camera image may become a new vision observation. A long run
of bit-for-bit identical YOLO results is an additional conservative guard.
"""

import zlib

import numpy as np


class CameraFreshness:
    def __init__(self, max_same_detection=5):
        if max_same_detection < 2:
            raise ValueError("max_same_detection must be at least 2")
        self.max_same_detection = max_same_detection
        self.previous_image = None
        self.previous_crc = None
        self.raw_duplicate_count = 0
        self.previous_detection = None
        self.same_detection_count = 0

    def reset_camera_connection(self):
        # A reconnect is not proof that the camera resumed acquiring images.
        # Keep the detection guard, but establish a new raw-image baseline.
        self.previous_image = None
        self.previous_crc = None
        self.raw_duplicate_count = 0

    def check_image(self, gray):
        if gray.ndim != 2 or gray.dtype != np.uint8:
            raise ValueError("expected an 8-bit grayscale camera image")
        crc = zlib.crc32(gray.tobytes())
        if (crc == self.previous_crc and self.previous_image is not None
                and np.array_equal(gray, self.previous_image)):
            self.raw_duplicate_count += 1
            return False, crc
        self.previous_image = gray.copy()
        self.previous_crc = crc
        self.raw_duplicate_count = 0
        return True, crc

    def check_detection(self, detected, center_x, center_y, confidence):
        if not detected:
            self.previous_detection = None
            self.same_detection_count = 0
            return True
        signature = (round(float(center_x), 2), round(float(center_y), 2),
                     round(float(confidence), 4))
        if signature == self.previous_detection:
            self.same_detection_count += 1
        else:
            self.previous_detection = signature
            self.same_detection_count = 1
        return self.same_detection_count < self.max_same_detection
