"""Adapter interfaces for the three live inputs/outputs of Stage 6.

The model reads localization from the QUARC *OptiTrack Trackables* block
(Motive API, Windows, rigid bodies ``[1, 2]``, 100 Hz), detections from a UDP
stream on port 5010, and talks to the QDrone through a QUARC Stream Server on
``tcpip://localhost:18002``. Only the detection stream's wire format is fully
specified by the model; see ``docs/SIMULINK_SPEC.md`` for the others.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class PoseSample:
    """One rigid body in the Research Studio frame, as ``Extract Vehicle
    Data`` produces it: ``[position(3), orientation(3), tracking]``."""

    timestamp_s: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float]
    tracking: float

    def as_vector(self) -> tuple[float, ...]:
        return (*self.position, *self.orientation, self.tracking)


@dataclass(frozen=True)
class DetectionSample:
    """What the Stage 6 Stream Server exposes each 0.033 s sample."""

    values: tuple[float, float, float]  # rcv: detected, error_x, error_y
    new: bool  # the block's ``new`` output
    state: int  # QUARC stream state (2 = connected)
    error: int  # QUARC error code (0 = none)
    received_at: float | None  # local monotonic time of the last datagram


@runtime_checkable
class CameraSource(Protocol):
    def read(self) -> tuple[object, float] | None:
        """Return ``(frame, timestamp_s)`` or None when exhausted."""

    def close(self) -> None: ...


@runtime_checkable
class PoseSource(Protocol):
    def read(self) -> PoseSample | None: ...

    def close(self) -> None: ...


@runtime_checkable
class DetectionSource(Protocol):
    def poll(self) -> DetectionSample: ...

    def close(self) -> None: ...


@runtime_checkable
class CommandSink(Protocol):
    def send(self, packet: tuple[float, ...]) -> bool: ...

    def close(self) -> None: ...
