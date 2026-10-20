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


class _ChunkySocket:
    """Stand-in socket: returns queued chunks, then would-block."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.sent = bytearray()
        self.accept_sizes = []

    def recv(self, n):
        if not self.chunks:
            raise BlockingIOError
        return self.chunks.pop(0)

    def send(self, data):
        n = self.accept_sizes.pop(0) if self.accept_sizes else len(data)
        if n == 0:
            raise BlockingIOError
        self.sent += data[:n]
        return n

    def close(self):
        pass


def test_vision_client_reads_every_chunk():
    vc = VisionClient()
    recs = [VISION.pack(1, k, k, 0.9, k, 0) for k in range(5)]
    vc.sock = _ChunkySocket(recs)
    assert vc.read(0.0) == b"".join(recs)


def test_commander_partial_write_completed_before_next_packet():
    srv = CommanderServer("127.0.0.1", 0)
    fake = _ChunkySocket([])
    srv.client = fake
    fake.accept_sizes = [50, 62, 112]
    assert srv.send(COMMAND.pack(*([1.0] * 14)))      # 50 bytes out, 62 pending
    assert srv.send(COMMAND.pack(*([2.0] * 14)))      # pending tail first, then the new packet
    assert bytes(fake.sent) == COMMAND.pack(*([1.0] * 14)) + COMMAND.pack(*([2.0] * 14))
    fake.accept_sizes = [0]
    assert not srv.send(COMMAND.pack(*([3.0] * 14))) and srv.dropped == 1
    # A tail that itself only partly drains must stay pending (never be dropped).
    fake.sent.clear()
    fake.accept_sizes = [50, 30, 32, 112]
    p4, p5, p6 = (COMMAND.pack(*([v] * 14)) for v in (4.0, 5.0, 6.0))
    assert srv.send(p4)
    assert not srv.send(p5)                           # 30 of 62 tail bytes out; p5 dropped
    assert srv.send(p6)                               # remaining 32 tail bytes, then p6
    assert bytes(fake.sent) == p4 + p6
    srv.client = None
    srv.close()


def test_serve_reconnects_vision_after_framing_loss():
    pub, fake, srv = _server(Mode.MONITOR)
    assert fake.connected.wait(3)
    closes = []
    real_close = srv.vision.close
    srv.vision.close = lambda: (closes.append(1), real_close())[1]
    srv.run(duration=0.05)
    assert _wait(lambda: pub.publish(1, 300.0, 200.0, 0.9, 1, time.time()))
    srv.run(duration=0.1)
    pub.send_raw(b"\x00\x01\x02")
    for k in range(3):
        pub.publish(1, 300.0, 200.0, 0.9, 2 + k, time.time())
        srv.run(duration=0.15 + 0.05 * k)
    assert srv.runtime.vision_guard.resyncs == 1 and closes == [1]     # reconnect requested
    before = srv.runtime.vision_guard.accepted
    for k in range(20):
        pub.publish(1, 300.0, 200.0, 0.9, 10 + k, time.time())
        srv.run(duration=0.35 + 0.02 * k)
    assert srv.runtime.vision_guard.accepted > before        # fresh connection, framing restored
    fake.stop(); srv.close(); pub.close()


class _Raising:
    def __init__(self, after: float, what: str):
        self.after, self.what = after, what

    def poll(self, now):
        if now > self.after:
            raise RuntimeError(f"{self.what} decode error")
        return None

    def read(self, now):
        if self.what == "vision":
            raise OSError("vision socket exploded")
        return b""

    def close(self):
        pass


@pytest.mark.parametrize("which", ["pose", "joystick", "vision"])
def test_serve_input_exception_is_a_fail_stop(which):
    """An exception in any input (a future NatNet/joystick reader, the vision
    socket) must stop the loop with a reason and disconnect the Commander,
    never escape run()."""
    pub, fake, srv = _server(Mode.MONITOR)
    assert fake.connected.wait(3)
    if which == "pose":
        srv.pose = _Raising(0.05, "natnet")
    elif which == "joystick":
        srv.joystick = _Raising(0.05, "joystick")
    st = srv.run(duration=0.1)
    if which == "vision":
        srv.vision = _Raising(0.0, "vision")
        st = srv.run(duration=0.2)
    assert st.stopped_reason is not None and st.stopped_reason.startswith("input: ")
    assert {"pose": "natnet", "joystick": "joystick", "vision": "vision"}[which] in st.stopped_reason
    assert srv.commander.state == 0.0 and srv.commander.client is None
    assert srv.run(duration=1.0).ticks == st.ticks                     # latched: does not resume
    fake.stop(); srv.close(); pub.close()


def test_serve_does_not_busy_spin():
    """Hybrid sleep: well under one core while keeping 1 kHz model time."""
    srv = MissionServer(DEFAULT, Mode.DRY_RUN, VisionClient(port=1), CommanderServer("127.0.0.1", 0))
    st = srv.run(duration=1.0)
    srv.close()
    assert st.ticks == 1000 and st.stopped_reason is None
    assert st.cpu_s < 0.6 * st.wall_s, (st.cpu_s, st.wall_s)
    assert st.lag_p50_s < 0.001
