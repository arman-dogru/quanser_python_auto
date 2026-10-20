from __future__ import annotations

import socket
import time

import pytest

from quanser_landing.detection import BoundingBox, Detection, decode, encode, read_jsonl, write_jsonl
from quanser_landing.io import CommandGate, VehicleStatus, qdrone_packet
from quanser_landing.io.loopback import LoopbackDetectionSource, RecordingCommandSink
from quanser_landing.io.udp_detection import UdpDetectionReceiver, UdpDetectionSender

OK = VehicleStatus(armed=True, takeoff=True, emergency_abort=False, joystick_issue=False,
                   comm_issue=False, is_tracking=True)


def _det(**kw):
    base = dict(timestamp_s=1.0, image_width=640, image_height=480, detected=True,
                box=BoundingBox(400, 200, 440, 240), class_id=0, class_name="landing_pad", confidence=0.9)
    base.update(kw)
    return Detection(**base)


def test_contract_pixel_error_and_inputs():
    d = _det()
    assert d.pixel_error == (100.0, -20.0)
    assert d.controller_inputs() == (1.0, 100.0, -20.0)
    assert _det(detected=False, box=None).controller_inputs() == (0.0, 0.0, 0.0)


def test_contract_refuses_other_image_sizes_unless_rescaled():
    d = _det(image_width=1280, image_height=960, box=BoundingBox(800, 400, 880, 480))
    with pytest.raises(ValueError):
        d.controller_inputs()
    assert d.controller_inputs(rescale=True) == (1.0, 100.0, -20.0)


def test_contract_validation():
    with pytest.raises(ValueError):
        _det(box=None)
    with pytest.raises(ValueError):
        _det(timestamp_s=float("nan"))


def test_jsonl_roundtrip(tmp_path):
    p = tmp_path / "d.jsonl"
    dets = [_det(), _det(detected=False, box=None, timestamp_s=2.0)]
    assert write_jsonl(dets, p) == 2
    assert list(read_jsonl(p)) == dets


def test_packet_codec():
    b = encode(1, -12.5, 3.0)
    assert len(b) == 24
    assert decode(b) == (1.0, -12.5, 3.0)
    assert decode(b + encode(0, 1, 2)) == (0.0, 1.0, 2.0)  # most recent
    with pytest.raises(ValueError):
        decode(b[:-1])


def test_packet_little_endian():
    import struct
    assert encode(1, 2, 3) == struct.pack("<ddd", 1, 2, 3)


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_udp_roundtrip_most_recent_and_default():
    port = _free_port()
    rx = UdpDetectionReceiver("127.0.0.1", port)
    tx = UdpDetectionSender("127.0.0.1", port)
    try:
        first = rx.poll()
        assert first.values == (0.0, 0.0, 0.0) and not first.new
        tx.send(1, 10, 20)
        tx.send(1, 30, 40)
        rx.sock.sendto(b"bad", ("127.0.0.1", port))
        time.sleep(0.05)
        s = rx.poll()
        assert s.new and s.values == (1.0, 30.0, 40.0)
        s2 = rx.poll()
        assert not s2.new and s2.values == (1.0, 30.0, 40.0)  # held, like the block
        assert rx.bad_datagrams == 1
    finally:
        rx.close()
        tx.close()


def test_loopback_source():
    src = LoopbackDetectionSource()
    assert src.poll().values == (0.0, 0.0, 0.0)
    src.push((1, 2, 3))
    assert src.poll().new and not src.poll().new


def test_gate_monitor_only_by_default():
    gate = CommandGate()
    d = gate.check((0.01, 0.0, 1.25, 0.0), (0.0, 0.0, 1.25, 0.0), OK, detection_age_s=0.01)
    assert not d.allowed and d.reasons == ("monitor-only",)


@pytest.mark.parametrize("field, reason", [("armed", "not armed"), ("takeoff", "takeoff toggle off"),
                                           ("is_tracking", "OptiTrack not tracking")])
def test_gate_interlocks(field, reason):
    gate = CommandGate(monitor_only=False)
    status = VehicleStatus(**{**OK.__dict__, field: False})
    d = gate.check((0.0, 0.0, 1.25, 0.0), (0.0, 0.0, 1.25, 0.0), status, 0.0)
    assert reason in d.reasons


def test_gate_refuses_descent_abort_comm_and_stale():
    gate = CommandGate(monitor_only=False)
    status = VehicleStatus(**{**OK.__dict__, "emergency_abort": True, "comm_issue": True})
    d = gate.check((0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 1.25, 0.0), status, 1.0)
    assert {"descent disabled", "emergency abort", "QDrone communication issue", "stale detection"} <= set(d.reasons)


def test_gate_refuses_stage6_scalar_pose():
    gate = CommandGate(monitor_only=False)
    d = gate.check((1.0, 1.0, 1.0, 1.0), (0.2, -0.3, 1.25, 0.0), OK, 0.0)
    assert not d.allowed


def test_gate_allows_small_step_when_everything_ok():
    gate = CommandGate(monitor_only=False)
    d = gate.check((0.01, -0.01, 1.25, 0.0), (0.0, 0.0, 1.25, 0.0), OK, 0.01)
    assert d.allowed


def test_qdrone_packet_layout():
    pkt = qdrone_packet((1, 1, 0, 0), 1, (0, 0, 1, 0), (0.1, 0, 1, 0), 12.5)
    assert len(pkt) == 14 and pkt[-1] == 12.5 and pkt[9] == 0.1
    sink = RecordingCommandSink()
    assert sink.send(pkt) and sink.sent == [pkt]
