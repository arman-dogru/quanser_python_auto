"""Interop with python/yolo_pad_direct_stream_fresh.py, run unmodified.

The publisher runs as a subprocess with a temporary project root (the YOLO
weights are symlinked in, so it writes its logs there). This test plays the
Commander's camera Stream Client: it connects to the camera port and sends
raw 640x480 column-major frames encoded with ``protocol.encode_frame``. The
port's own VisionClient + VisionGuard read the observations.

Synthetic frames do not contain a landing pad, so detection quality is not
tested here (it needs recorded imagery); the wire format, frame numbering,
timestamps and the publisher's freshness gating are.
"""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from qdrone_autoland import protocol
from qdrone_autoland.io.sockets import VisionClient
from qdrone_autoland.protocol import VISION
from qdrone_autoland.safety import VisionGuard

pytestmark = pytest.mark.vision


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_publisher_defaults_match_protocol(root, monkeypatch):
    pytest.importorskip("cv2")
    pytest.importorskip("ultralytics")
    monkeypatch.syspath_prepend(str(root / "python"))
    import yolo_pad_direct_stream as d
    monkeypatch.setattr(sys, "argv", ["x"])
    a = d.parse_args()
    assert (a.camera_port, a.detection_port) == (protocol.FRAME_PORT, protocol.VISION_PORT)
    assert (d.WIDTH, d.HEIGHT, d.FRAME_BYTES) == (protocol.FRAME_WIDTH, protocol.FRAME_HEIGHT, protocol.FRAME_BYTES)
    a_srv, b_cli = socket.socketpair()
    img = np.random.default_rng(0).integers(0, 255, (480, 640), dtype=np.uint8)
    sender = threading.Thread(target=b_cli.sendall, args=(protocol.encode_frame(img),), daemon=True)
    sender.start()                      # 300 kB exceeds the socketpair buffer
    np.testing.assert_array_equal(d.receive_frame(a_srv), img)
    sender.join(5)
    a_srv.close(); b_cli.close()


def test_fresh_publisher_end_to_end(root, tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("ultralytics")
    (tmp_path / "models").mkdir()
    for f in (root / "models").iterdir():
        (tmp_path / "models" / f.name).symlink_to(f)
    cam_port, det_port = _free_port(), _free_port()
    env = dict(os.environ, PYTHONUNBUFFERED="1", YOLO_OFFLINE="1")
    proc = subprocess.Popen(
        [sys.executable, str(root / "python" / "yolo_pad_direct_stream_fresh.py"), "--no-preview",
         "--project-root", str(tmp_path), "--listen-address", "127.0.0.1",
         "--camera-port", str(cam_port), "--detection-port", str(det_port)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    out = ""
    try:
        cam = None
        end = time.monotonic() + 120
        while cam is None and time.monotonic() < end:
            try:
                cam = socket.create_connection(("127.0.0.1", cam_port), timeout=1)
            except OSError:
                assert proc.poll() is None, proc.stdout.read()
                time.sleep(0.2)
        assert cam is not None
        vc = VisionClient(port=det_port, reconnect_s=0.0)
        guard = VisionGuard()
        recs: list[tuple] = []
        accepted: list[tuple] = []

        buf = bytearray()

        def pump(seconds, until=None):
            stop = time.monotonic() + seconds
            while time.monotonic() < stop:
                data = vc.read(time.time())
                if data:
                    buf.extend(data)
                    v, new = guard.step(time.time(), data)
                    if new:
                        accepted.append(v)
                    del recs[:]
                    recs.extend(VISION.unpack(bytes(buf[i:i + 48])) for i in range(0, len(buf) - len(buf) % 48, 48))
                    if until is not None and len(recs) >= until:
                        return
                time.sleep(0.005)

        rng = np.random.default_rng(1)
        vc.read(time.time())
        # Distinct frames -> one observation each (non-detections always pass). Wait for each one:
        # the first YOLO inference (warm-up) can take many seconds on a CI runner.
        for k in range(8):
            cam.sendall(protocol.encode_frame(rng.integers(0, 255, (480, 640), dtype=np.uint8)))
            pump(60 if k == 0 else 20, until=k + 1)
        n_distinct = len(recs)
        # The camera "freezes": the first frozen image is new, its repeats must be withheld.
        frozen = protocol.encode_frame(rng.integers(0, 255, (480, 640), dtype=np.uint8))
        cam.sendall(frozen)
        pump(20, until=n_distinct + 1)
        for _ in range(5):
            cam.sendall(frozen)
            pump(0.2)
        pump(3.0)
        n_after_freeze = len(recs)
        cam.close()
        vc.close()
    except Exception as exc:
        proc.terminate()
        raise AssertionError(f"publisher output:\n{proc.communicate(timeout=10)[0]}") from exc
    finally:
        if proc.poll() is None:
            proc.terminate()
        try:
            out = proc.communicate(timeout=10)[0]
        except subprocess.TimeoutExpired:
            proc.kill()
            out = proc.communicate()[0]
        except ValueError:                    # already drained by the error path above
            pass
    assert n_distinct == 8, out
    assert n_after_freeze - n_distinct == 1, out           # first frozen frame is new; repeats withheld
    frames = [r[4] for r in recs]
    assert frames == sorted(frames) and len(set(frames)) == len(frames)
    assert all(abs(time.time() - r[5]) < 120 for r in recs)
    assert all(r[0] in (0.0, 1.0) for r in recs)
    assert len(accepted) + guard.buffer.dropped == len(recs) and not guard.refused   # all fresh, all well-formed
    assert "CAMERA STALE" in out
