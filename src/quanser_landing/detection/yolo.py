"""YOLO inference on recorded frames (ultralytics, CPU).

The supplied ``models/yolo11n.pt`` is byte-identical to Ultralytics' public
COCO-pretrained YOLO11n release (80 COCO classes, no landing-pad class); see
``docs/PORT_NOTES.md``. It can exercise this pipeline but cannot detect a
landing pad. Use a landing-pad checkpoint and name its class explicitly.

Class filtering is explicit and mandatory: :class:`YoloPadDetector` refuses
to run without a class list, and refuses class names the checkpoint does not
have. Among the boxes that pass the class and confidence filters the most
confident one is reported (the Stage 6 packet has room for one target).
"""
from __future__ import annotations

import time
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path

from .contract import BoundingBox, Detection

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def list_frames(folder: Path) -> list[Path]:
    return sorted(p for p in Path(folder).iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)


def resolve_classes(names: dict[int, str], wanted: Sequence[str | int]) -> list[int]:
    """Map class names or ids to ids, refusing anything the model lacks."""
    if not wanted:
        raise ValueError("an explicit class filter is required (e.g. --classes landing_pad)")
    by_name = {v: k for k, v in names.items()}
    ids = []
    for w in wanted:
        if isinstance(w, int) or (isinstance(w, str) and w.isdigit()):
            i = int(w)
            if i not in names:
                raise ValueError(f"class id {i} not in checkpoint (has {len(names)} classes)")
            ids.append(i)
        elif w in by_name:
            ids.append(by_name[w])
        else:
            raise ValueError(f"class {w!r} not in checkpoint classes {sorted(by_name)[:10]}...")
    return ids


@dataclass
class YoloPadDetector:
    weights: Path
    classes: Sequence[str | int]
    conf_threshold: float
    imgsz: int = 640
    device: str = "cpu"

    def __post_init__(self):
        from ultralytics import YOLO  # optional dependency

        if not 0.0 < self.conf_threshold < 1.0:
            raise ValueError("conf_threshold must be in (0, 1)")
        self.model = YOLO(str(self.weights))
        self.names: dict[int, str] = dict(self.model.names)
        self.class_ids = resolve_classes(self.names, self.classes)

    def detect(self, image, timestamp_s: float, frame_id: str = "") -> Detection:
        """Run on one image (path or HxWx3 array)."""
        res = self.model.predict(image, imgsz=self.imgsz, conf=self.conf_threshold,
                                 classes=self.class_ids, device=self.device, verbose=False)[0]
        h, w = res.orig_shape
        best = None
        for b in res.boxes:
            cid, conf = int(b.cls.item()), float(b.conf.item())
            if cid not in self.class_ids or conf < self.conf_threshold:
                continue
            if best is None or conf > best[1]:
                best = (cid, conf, [float(v) for v in b.xyxy[0].tolist()])
        common = dict(timestamp_s=timestamp_s, image_width=int(w), image_height=int(h),
                      frame_id=frame_id, source=str(self.weights.name),
                      extra={"conf_threshold": self.conf_threshold, "class_filter": list(self.class_ids)})
        if best is None:
            return Detection(detected=False, **common)
        cid, conf, xyxy = best
        return Detection(detected=True, class_id=cid, class_name=self.names[cid], confidence=conf,
                         box=BoundingBox(*xyxy), **common)

    def detect_frames(self, frames: Iterable[Path], timestamps: Iterable[float]) -> Iterator[Detection]:
        for path, ts in zip(frames, timestamps, strict=True):
            yield self.detect(str(path), ts, frame_id=Path(path).name)


def frame_timestamps(frames: Sequence[Path], fps: float | None = None,
                     timestamps_csv: Path | None = None) -> list[float]:
    """Timestamps for recorded frames: from a ``frame,timestamp_s`` CSV, or
    ``index / fps``. One of the two is required; file times are not used
    because copies do not preserve them."""
    if timestamps_csv is not None:
        table = {}
        for line in Path(timestamps_csv).read_text().splitlines()[1:]:
            if line.strip():
                name, ts = line.split(",")[:2]
                table[name.strip()] = float(ts)
        missing = [f.name for f in frames if f.name not in table]
        if missing:
            raise ValueError(f"no timestamp for {missing[:3]}...")
        return [table[f.name] for f in frames]
    if fps is None or fps <= 0:
        raise ValueError("give --fps or --timestamps")
    return [i / fps for i in range(len(frames))]


def wall_clock() -> float:
    return time.time()
