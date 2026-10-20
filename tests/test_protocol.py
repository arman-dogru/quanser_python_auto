from __future__ import annotations

import struct

import numpy as np
import pytest

from qdrone_autoland import protocol as p


def test_sizes():
    assert p.VISION.size == 48 and p.COMMAND.size == 112 and p.LOOPBACK.size == 8
    assert p.FRAME_BYTES == 307200


def test_vision_layout_matches_publisher():
    # LatestObservationServer.publish: struct.pack("<6d", detected, cx, cy, conf, frame, timestamp)
    raw = struct.pack("<6d", 1.0, 320.5, 240.25, 0.9, 17.0, 1.7e9)
    v = p.VisionPacket.unpack(raw)
    assert v.as_vector() == (1.0, 320.5, 240.25, 0.9, 17.0, 1.7e9)
    assert v.pack() == raw


def test_command_layout_is_root_mux_order():
    pkt = p.CommandPacket((1, 1, 0, 0), 1, (0.1, 0.2, 0.3, 0.4), (0.5, 0.6, 0.7, 0.8), 123.0)
    assert struct.unpack("<14d", pkt.pack()) == (1, 1, 0, 0, 1, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 123.0)
    assert p.CommandPacket.unpack(pkt.pack()) == pkt
    assert pkt.is_finite()
    assert not p.CommandPacket((1, 1, 0, 0), 1, (float("nan"), 0, 0, 0), (0, 0, 0, 0), 0).is_finite()


def test_frame_is_column_major():
    img = np.arange(p.FRAME_BYTES, dtype=np.uint32).astype(np.uint8).reshape(480, 640)
    wire = p.encode_frame(img)
    assert wire[:2] == bytes([img[0, 0], img[1, 0]])     # down the first column first
    np.testing.assert_array_equal(p.decode_frame(wire), img)
    with pytest.raises(ValueError):
        p.encode_frame(img.T.copy())
    with pytest.raises(ValueError):
        p.decode_frame(wire[:-1])


def test_record_buffer_keeps_newest_and_drains():
    b = p.RecordBuffer(p.VISION.size)
    recs = [p.VISION.pack(1, i, i, 0.9, i, 0) for i in range(5)]
    assert b.feed(b"".join(recs) + recs[0][:10]) == recs[4]
    assert b.dropped == 4
    assert b.feed(b"") is None
    assert b.feed(recs[0][10:]) == recs[0]
