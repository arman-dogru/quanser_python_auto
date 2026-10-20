"""In-memory implementations of the adapter interfaces, for tests and replay."""
from __future__ import annotations

from collections import deque
from collections.abc import Iterable

from .interfaces import DetectionSample, PoseSample


class ListPoseSource:
    def __init__(self, samples: Iterable[PoseSample]):
        self._q = deque(samples)

    def read(self) -> PoseSample | None:
        return self._q.popleft() if self._q else None

    def close(self) -> None:
        self._q.clear()


class LoopbackDetectionSource:
    """Stream-Server-like holder: ``push`` values, ``poll`` returns the most
    recent one (``[0 0 0]`` before any) and whether it is new."""

    def __init__(self):
        self._last = (0.0, 0.0, 0.0)
        self._pending = False
        self._t = None

    def push(self, values, t: float | None = None) -> None:
        self._last = tuple(float(v) for v in values)
        self._pending = True
        self._t = t

    def poll(self) -> DetectionSample:
        new, self._pending = self._pending, False
        return DetectionSample(self._last, new, 2, 0, self._t)

    def close(self) -> None:
        pass


class RecordingCommandSink:
    def __init__(self):
        self.sent: list[tuple[float, ...]] = []

    def send(self, packet: tuple[float, ...]) -> bool:
        self.sent.append(tuple(packet))
        return True

    def close(self) -> None:
        pass
