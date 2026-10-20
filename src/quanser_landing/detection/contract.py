"""Detection contract between a detector and the landing controller.

The Stage 6 model receives only three numbers per packet,
``[detected, error_x, error_y]`` (see :mod:`.packet`). It carries no
timestamp, image size, class or confidence. This contract keeps those
fields so logs can be replayed and audited; :meth:`Detection.controller_inputs`
reduces a detection to the three values the chart expects.

Pixel convention (from the chart comments): ``error = pad_centre -
image_centre`` with ``x`` along image columns and ``y`` along image rows,
for a 640 x 480 image (centre 320, 240). Whether that matches the real camera
mounting is unverified; the chart only says "calibration-confirmed".
"""
from __future__ import annotations

import json
import math
from collections.abc import Iterable, Iterator
from dataclasses import asdict, dataclass, field

CONTROLLER_IMAGE_SIZE = (640, 480)


@dataclass(frozen=True)
class BoundingBox:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


@dataclass(frozen=True)
class Detection:
    """One processed frame.

    ``timestamp_s`` is the frame's capture time on the source's clock
    (seconds). ``image_width``/``image_height`` are the size of the frame the
    detector saw. ``detected`` is False when no box of an allowed class
    passed the confidence threshold; the box fields are then ``None``.
    """

    timestamp_s: float
    image_width: int
    image_height: int
    detected: bool
    frame_id: str = ""
    class_id: int | None = None
    class_name: str | None = None
    confidence: float | None = None
    box: BoundingBox | None = None
    source: str = ""
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image size must be positive")
        if self.detected and self.box is None:
            raise ValueError("a positive detection needs a box")
        if not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp must be finite")

    @property
    def pixel_error(self) -> tuple[float, float] | None:
        """``(error_x, error_y)`` in this frame's pixels, or None."""
        if not self.detected or self.box is None:
            return None
        cx, cy = self.box.center
        return (cx - self.image_width / 2.0, cy - self.image_height / 2.0)

    def controller_inputs(self, rescale: bool = False) -> tuple[float, float, float]:
        """``(detected, error_x, error_y)`` for the Stage 6 chart.

        The chart hard-codes a 640 x 480 image. Frames of another size raise
        unless ``rescale`` is True, in which case errors are scaled to
        640 x 480 pixel units. No detection gives ``(0, 0, 0)``, the Stream
        Server's default value in the model.
        """
        if (self.image_width, self.image_height) != CONTROLLER_IMAGE_SIZE and not rescale:
            raise ValueError(
                f"controller assumes 640x480 frames, got {self.image_width}x{self.image_height}; "
                "pass rescale=True to scale the pixel errors")
        err = self.pixel_error
        if err is None:
            return (0.0, 0.0, 0.0)
        sx = CONTROLLER_IMAGE_SIZE[0] / self.image_width
        sy = CONTROLLER_IMAGE_SIZE[1] / self.image_height
        return (1.0, err[0] * sx, err[1] * sy)

    def to_json(self) -> str:
        d = asdict(self)
        return json.dumps(d, sort_keys=True)

    @classmethod
    def from_dict(cls, d: dict) -> Detection:
        d = dict(d)
        if d.get("box") is not None:
            d["box"] = BoundingBox(**d["box"])
        return cls(**d)


def write_jsonl(detections: Iterable[Detection], path) -> int:
    n = 0
    with open(path, "w", encoding="utf-8") as fh:
        for det in detections:
            fh.write(det.to_json() + "\n")
            n += 1
    return n


def read_jsonl(path) -> Iterator[Detection]:
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                yield Detection.from_dict(json.loads(line))
