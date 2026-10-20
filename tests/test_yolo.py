"""YOLO wrapper plumbing. The supplied checkpoint is COCO YOLO11n, so these
tests use a COCO class on Ultralytics' bundled sample image; they check the
contract, not landing-pad detection."""
from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from conftest import ROOT

ultralytics = pytest.importorskip("ultralytics")

from quanser_landing.detection.yolo import YoloPadDetector, frame_timestamps, resolve_classes  # noqa: E402

WEIGHTS = ROOT / "models" / "yolo11n.pt"
OFFICIAL_YOLO11N_SHA256 = "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"
pytestmark = pytest.mark.yolo


def test_checkpoint_is_stock_coco_yolo11n():
    assert hashlib.sha256(WEIGHTS.read_bytes()).hexdigest() == OFFICIAL_YOLO11N_SHA256


def test_class_filter_is_mandatory_and_checked():
    names = {0: "person", 1: "bicycle"}
    assert resolve_classes(names, ["bicycle", 0]) == [1, 0]
    with pytest.raises(ValueError):
        resolve_classes(names, [])
    with pytest.raises(ValueError):
        resolve_classes(names, ["landing_pad"])


def test_checkpoint_has_no_landing_pad_class():
    with pytest.raises(ValueError):
        YoloPadDetector(WEIGHTS, classes=["landing_pad"], conf_threshold=0.5)


@pytest.fixture(scope="module")
def sample_image() -> Path:
    p = Path(ultralytics.__file__).parent / "assets" / "bus.jpg"
    if not p.exists():
        pytest.skip("ultralytics sample image missing")
    return p


def test_detect_contract_on_sample_frame(sample_image):
    det = YoloPadDetector(WEIGHTS, classes=["bus"], conf_threshold=0.5)
    d = det.detect(str(sample_image), timestamp_s=12.5, frame_id="bus.jpg")
    assert d.timestamp_s == 12.5 and (d.image_width, d.image_height) == (810, 1080)
    assert d.detected and d.class_name == "bus" and d.confidence >= 0.5
    flag, ex, ey = d.controller_inputs(rescale=True)
    assert flag == 1.0 and abs(ex) <= 320 and abs(ey) <= 240
    none = YoloPadDetector(WEIGHTS, classes=["airplane"], conf_threshold=0.5).detect(str(sample_image), 0.0)
    assert not none.detected and none.controller_inputs(rescale=True) == (0.0, 0.0, 0.0)


def test_frame_timestamps(tmp_path):
    frames = [tmp_path / "a.jpg", tmp_path / "b.jpg"]
    assert frame_timestamps(frames, fps=10) == [0.0, 0.1]
    csvp = tmp_path / "t.csv"
    csvp.write_text("frame,timestamp_s\na.jpg,1.5\nb.jpg,1.6\n")
    assert frame_timestamps(frames, timestamps_csv=csvp) == [1.5, 1.6]
    with pytest.raises(ValueError):
        frame_timestamps(frames)
