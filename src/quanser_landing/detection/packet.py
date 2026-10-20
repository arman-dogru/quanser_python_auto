"""Wire format of the Stage 6 detection stream (``udp://localhost:5010``).

What the model states (``MISSION SERVER .../Stream Server`` block):

* QUARC Stream Server, URI ``udp://localhost:5010?peer=any``, receive only
  ("Do not send data"), "Receive most recent data", sample time 0.033 s;
* output data type ``double``, default value ``[0 0 0]`` (so three doubles,
  24 bytes per sample), little-endian byte order, receive dimensions off;
* the ``rcv`` output is demuxed into ``detected``, ``error_x``, ``error_y``
  (in that order) and fed to ``YOLOPoseCommandGenerator``.

What it does not state: anything about the sender (it was not supplied),
its send rate, timestamps or confidence threshold. QUARC's Stream API
writes the raw bytes of the signal with no framing when dimensions are not
sent, so a datagram carrying exactly ``struct.pack('<3d', ...)`` matches the
block's configuration; this has not been tested against a running QUARC
model.
"""
from __future__ import annotations

import struct

PACKET_FORMAT = "<3d"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 24 bytes
DEFAULT_VALUE = (0.0, 0.0, 0.0)  # Stream Server "Default output value"


def encode(detected: float, error_x: float, error_y: float) -> bytes:
    return struct.pack(PACKET_FORMAT, float(detected), float(error_x), float(error_y))


def decode(payload: bytes) -> tuple[float, float, float]:
    """Decode one sample. A datagram holding several whole samples (QUARC can
    coalesce unflushed data) yields the most recent one, matching the block's
    "Receive most recent data" option. Partial samples raise ValueError."""
    if len(payload) < PACKET_SIZE or len(payload) % PACKET_SIZE:
        raise ValueError(f"expected a multiple of {PACKET_SIZE} bytes, got {len(payload)}")
    return struct.unpack(PACKET_FORMAT, payload[-PACKET_SIZE:])
