"""Socket adapters and the real-time loop on 127.0.0.1."""
from __future__ import annotations

import socket
import struct
import sys
import time

import pytest

from qdrone_autoland.blocks import R_RSF_TO_MF, JoystickSample
from qdrone_autoland.config import DEFAULT
from qdrone_autoland.io.loopback import FakeCommander, FakePublisher
from qdrone_autoland.io.serve import MissionServer
from qdrone_autoland.io.sockets import CommanderServer, VisionClient
from qdrone_autoland.io.sources import ScriptedJoystick, StaticPose
from qdrone_autoland.mission_server import OptiTrackSample
from qdrone_autoland.protocol import COMMAND, LOOPBACK, VISION
from qdrone_autoland.safety import Mode


def _wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


def test_vision_client_drains_backlog_and_reconnects():
    pub = FakePublisher(0)
    vc = VisionClient(port=pub.port, reconnect_s=0.0)
    vc.read(time.time())
    assert _wait(lambda: pub.publish(1, 1, 1, 0.9, 1, time.time()))
    for k in range(2, 50):
        pub.publish(1, k, k, 0.9, k, time.time())
    buf = bytearray()
    assert _wait(lambda: (buf.extend(vc.read(time.time())), len(buf) >= 49 * 48)[1])
    data = bytes(buf)
    assert len(data) == 49 * 48 and VISION.unpack(data[-48:])[4] == 49
    pub.client.close(); pub.client = None
    assert _wait(lambda: (vc.read(time.time()), vc.sock is None)[1])
    vc.read(time.time())
    assert _wait(lambda: pub.publish(1, 1, 1, 0.9, 99, time.time()))
    assert _wait(lambda: len(vc.read(time.time())) == 48)
    vc.close(); pub.close()


def test_commander_server_sends_14_doubles_and_reads_loopback():
    srv = CommanderServer("127.0.0.1", 0)
    c = socket.create_connection(("127.0.0.1", srv.port))
    assert _wait(lambda: (srv.poll(), srv.state == 2.0)[1])
    payload = COMMAND.pack(*range(14))
    assert srv.send(payload)
    got = b""
    while len(got) < 112:
        got += c.recv(112)
    assert struct.unpack("<14d", got) == tuple(float(v) for v in range(14))
    c.sendall(LOOPBACK.pack(13.0) + LOOPBACK.pack(42.0))
    assert _wait(lambda: (srv.poll(), srv.loopback_timestamp == 42.0)[1])
    with pytest.raises(ValueError):
        srv.send(b"x" * 10)
    c.close()
    assert _wait(lambda: (srv.poll(), srv.state == 0.0)[1])
    srv.close()


def test_commander_server_never_desynchronises_under_backpressure():
    srv = CommanderServer("127.0.0.1", 0)
    c = socket.create_connection(("127.0.0.1", srv.port))
    c.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    assert _wait(lambda: (srv.poll(), srv.state == 2.0)[1])
    srv.client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
    for k in range(20000):
        srv.send(COMMAND.pack(*([float(k)] * 14)))
    assert srv.dropped > 0
    c.settimeout(0.5)
    data = b""
    extra = 0
    while True:
        try:
            chunk = c.recv(1 << 16)
        except TimeoutError:
            break
        if not chunk:
            break
        data += chunk
        if extra < 50:              # keep writing while the reader drains: tails must complete first
            srv.send(COMMAND.pack(*([-1.0] * 14)))
            extra += 1
    n = len(data) // 112
    assert n > 0
    for i in range(n):
        rec = struct.unpack("<14d", data[i * 112:(i + 1) * 112])
        assert len(set(rec)) == 1, f"record {i} is torn"
    c.close(); srv.close()


def _server(mode):
    pub = FakePublisher(0)
    cmd = CommanderServer("127.0.0.1", 0)
    fake = FakeCommander(cmd.port)
    fake.start()
    rsf = (0.0, -0.9, DEFAULT.model.rigid_body_height_offset)
    pose = StaticPose(OptiTrackSample(tuple(R_RSF_TO_MF @ rsf), (0.0, 0.0, 0.0), 1.0))

    def joy(t):
        b = [0.0] * 32
        b[0] = 1.0 if t >= 0.6 else 0.0
        b[1] = 1.0
        return JoystickSample(buttons=tuple(b), x=-1.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=1.0)
    srv = MissionServer(DEFAULT, mode, VisionClient(port=pub.port, reconnect_s=0.0), cmd, pose,
                        ScriptedJoystick(joy))
    return pub, fake, srv


@pytest.mark.parametrize("mode", [Mode.MONITOR, Mode.AUTOLAND])
def test_serve_loop_end_to_end(mode):
    pub, fake, srv = _server(mode)
    assert fake.connected.wait(3)
    frame = 0
    for chunk in range(10):
        srv.run(duration=(chunk + 1) * 0.1)
        frame += 1
        pub.publish(1, 300.0, 200.0, 0.9, frame, time.time())
    time.sleep(0.1)
    fake.stop()
    st = srv.stats
    assert st.stopped_reason is None and st.ticks == 1000 and st.sent >= 990
    assert fake.packets and fake.packets[-1].triggers[2] == 1.0            # E-stop reached the Commander side
    assert all(p.is_finite() for p in fake.packets)
    assert srv.runtime.vision_guard.accepted >= 5
    loop = srv.last.outputs.loopback_time
    assert 0 <= loop < 0.5                                                 # loopback echo is flowing
    srv.close(); pub.close()


def test_serve_dry_run_sends_nothing():
    pub, fake, srv = _server(Mode.DRY_RUN)
    assert fake.connected.wait(3)
    srv.run(duration=0.3)
    time.sleep(0.1)
    fake.stop()
    assert srv.stats.sent == 0 and not fake.packets
    srv.close(); pub.close()


def test_serve_fail_stop_disconnects_commander(monkeypatch):
    pub, fake, srv = _server(Mode.MONITOR)
    assert fake.connected.wait(3)
    srv.run(duration=0.1)
    monkeypatch.setattr(srv.runtime.core, "step", lambda *_: (_ for _ in ()).throw(RuntimeError("boom")))
    srv.run(duration=0.2)
    assert "boom" in srv.stats.stopped_reason and srv.commander.state == 0.0
    fake.stop(); srv.close(); pub.close()


def test_serve_overrun_fail_stops():
    pub, fake, srv = _server(Mode.MONITOR)
    clock = iter([0.0] + [0.2] * 10)
    st = srv.run(duration=1.0, clock=lambda: next(clock), sleep=lambda s: None)
    assert "overrun" in st.stopped_reason
    fake.stop(); srv.close(); pub.close()


@pytest.mark.vision
def test_vision_client_against_real_latest_observation_server(root):
    sys.path.insert(0, str(root / "python"))
    pytest.importorskip("cv2"); pytest.importorskip("ultralytics")
    from yolo_pad_writer_passive import LatestObservationServer
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    pub = LatestObservationServer(port)
    vc = VisionClient(port=port, reconnect_s=0.0)
    vc.read(time.time())
    time.sleep(0.05)
    for k in range(1, 6):
        pub.publish(k % 2, 320.0, 240.0, 0.8, k, time.time())
    time.sleep(0.05)
    data = vc.read(time.time())
    recs = [VISION.unpack(data[i:i + 48]) for i in range(0, len(data), 48)]
    assert len(data) % 48 == 0 and [r[4] for r in recs] == [1, 2, 3, 4, 5]
    assert recs[1][:4] == (0.0, 0.0, 0.0, 0.0)          # not detected -> zeros, as published
    vc.close(); pub.close()
