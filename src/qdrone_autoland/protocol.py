"""Wire formats, taken from the three primary sources (see docs/INTERFACES.md).

All QUARC Stream blocks in both models are configured with
``byte_order = little endian``, ``send_dimensions = off`` and
``receive_dimensions = off``: a sample is the raw bytes of the signal, no
header, no length prefix. Element types are back-propagated in the models;
every signal involved resolves to ``double`` (see the interface doc for the
per-signal evidence), and the Python vision publisher packs ``<6d``.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

import numpy as np

VISION_PORT = 18901    # mission server "YOLO Observation TCP Client" uri; publisher --detection-port default
COMMAND_PORT = 18001   # mission server URI_Follower; Commander Stream Client URI_Host
FRAME_PORT = 18910     # Commander camera Stream Client; publisher --camera-port default

# Vision publisher -> mission server, TCP 18901 (python/yolo_pad_writer_passive.py
# LatestObservationServer.publish: struct.pack("<6d", ...)).
VISION = struct.Struct("<6d")
VISION_DEFAULT = (0.0, 0.0, 0.0, 0.0, -1.0, 0.0)   # Stream Client default_value [0 0 0 0 -1 0]

# Mission server -> Commander, TCP 18001: root Mux [Triggers(4), isTracking(1),
# Measured Pose(4), Desired Pose(4), Timestamp(1)] = 14 doubles.
COMMAND = struct.Struct("<14d")
# Commander -> mission server: element 14 echoed back through a Memory block.
LOOPBACK = struct.Struct("<d")

# Commander -> ground vision, TCP 18910: one 640x480 uint8 frame per 0.2 s,
# MATLAB greyscale (HxW) column-major, 307200 bytes, no header.
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT


@dataclass(frozen=True)
class VisionPacket:
    detected: float
    center_x: float
    center_y: float
    confidence: float
    frame: float
    timestamp: float

    def as_vector(self) -> tuple[float, ...]:
        return (self.detected, self.center_x, self.center_y, self.confidence, self.frame, self.timestamp)

    @classmethod
    def unpack(cls, data: bytes) -> VisionPacket:
        return cls(*VISION.unpack(data))

    def pack(self) -> bytes:
        return VISION.pack(*self.as_vector())


@dataclass(frozen=True)
class CommandPacket:
    triggers: tuple[float, float, float, float]
    is_tracking: float
    measured_pose: tuple[float, float, float, float]
    desired_pose: tuple[float, float, float, float]
    timestamp: float

    def as_vector(self) -> tuple[float, ...]:
        return (*self.triggers, self.is_tracking, *self.measured_pose, *self.desired_pose, self.timestamp)

    def pack(self) -> bytes:
        return COMMAND.pack(*self.as_vector())

    @classmethod
    def unpack(cls, data: bytes) -> CommandPacket:
        v = COMMAND.unpack(data)
        return cls(tuple(v[0:4]), v[4], tuple(v[5:9]), tuple(v[9:13]), v[13])

    def is_finite(self) -> bool:
        return all(math.isfinite(x) for x in self.as_vector())


def encode_frame(gray: np.ndarray) -> bytes:
    """What the Commander's Stream Client puts on the wire for one frame."""
    if gray.shape != (FRAME_HEIGHT, FRAME_WIDTH) or gray.dtype != np.uint8:
        raise ValueError("expected a 480x640 uint8 image")
    return gray.tobytes(order="F")


def decode_frame(payload: bytes) -> np.ndarray:
    """Inverse of :func:`encode_frame`; identical to the publisher's receive_frame."""
    if len(payload) != FRAME_BYTES:
        raise ValueError(f"expected {FRAME_BYTES} bytes, got {len(payload)}")
    return np.frombuffer(payload, dtype=np.uint8).reshape((FRAME_HEIGHT, FRAME_WIDTH), order="F")


class RecordBuffer:
    """Accumulate a byte stream and return only the newest complete record.

    This is the "Receive most recent data" behaviour of the QUARC Stream
    blocks, and it drains any backlog so stale queued records are never used.
    """

    def __init__(self, size: int, max_buffer: int = 1 << 20) -> None:
        self.size = size
        self.max_buffer = max_buffer
        self.buf = bytearray()
        self.dropped = 0

    def feed(self, data: bytes) -> bytes | None:
        self.buf += data
        n = len(self.buf) // self.size
        if n == 0:
            if len(self.buf) > self.max_buffer:  # cannot happen with size < max_buffer
                self.buf.clear()
            return None
        end = n * self.size
        newest = bytes(self.buf[end - self.size:end])
        self.dropped += n - 1
        del self.buf[:end]
        return newest
