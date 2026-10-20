"""Command gate: the only path from a controller output to a command sink.

Stage 6 is monitor-only: ``use_yolo = 0`` means the YOLO ``cmd_pose`` never
reaches the ``Desired Pose`` port, and ``enable_descent = 0`` inside the
chart. This gate keeps both defaults and adds the interlocks the model
exposes to the vehicle:

* ``monitor_only`` (default True): nothing is ever sent;
* the joystick triggers: armed, takeoff, no emergency abort, no joystick
  issue (``JOYSTICK`` subsystem);
* no communication issue on the QDrone stream (``Communication - QDrone``
  plus the falling-edge latch);
* ``isTracking`` from OptiTrack must be 1;
* descent is not supported: a command whose z is below the measured z is
  refused, whatever the chart says.

Two further checks are Python additions, not in the model, and are listed
as such in ``docs/PORT_NOTES.md``: detection staleness (the model keeps
re-using the last UDP sample forever) and a cap on the horizontal step.

In the model the mission server only *reports* communication issues (a
warning on the host); enforcement happens in the vehicle-side model, which
was not supplied. Here the gate enforces them itself.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field


@dataclass(frozen=True)
class VehicleStatus:
    armed: bool
    takeoff: bool
    emergency_abort: bool
    joystick_issue: bool
    comm_issue: bool
    is_tracking: bool


@dataclass(frozen=True)
class GateDecision:
    allowed: bool
    reasons: tuple[str, ...]


@dataclass
class CommandGate:
    monitor_only: bool = True
    max_detection_age_s: float | None = 0.2  # Python addition
    max_horizontal_step_m: float = 0.05 * math.sqrt(2) + 1e-9  # chart can't exceed this
    log: list[GateDecision] = field(default_factory=list)

    def check(self, desired_pose: Sequence[float], measured_pose: Sequence[float],
              status: VehicleStatus, detection_age_s: float | None) -> GateDecision:
        reasons = []
        if self.monitor_only:
            reasons.append("monitor-only")
        if not status.armed:
            reasons.append("not armed")
        if not status.takeoff:
            reasons.append("takeoff toggle off")
        if status.emergency_abort:
            reasons.append("emergency abort")
        if status.joystick_issue:
            reasons.append("joystick issue")
        if status.comm_issue:
            reasons.append("QDrone communication issue")
        if not status.is_tracking:
            reasons.append("OptiTrack not tracking")
        if any(not math.isfinite(v) for v in (*desired_pose, *measured_pose)):
            reasons.append("non-finite pose")
        else:
            if desired_pose[2] < measured_pose[2]:
                reasons.append("descent disabled")
            dx = desired_pose[0] - measured_pose[0]
            dy = desired_pose[1] - measured_pose[1]
            if math.hypot(dx, dy) > self.max_horizontal_step_m:
                reasons.append("horizontal step too large")
            if desired_pose[3] != measured_pose[3]:
                reasons.append("yaw change not expected")
        if self.max_detection_age_s is not None and (
                detection_age_s is None or detection_age_s > self.max_detection_age_s):
            reasons.append("stale detection")
        d = GateDecision(allowed=not reasons, reasons=tuple(reasons))
        self.log.append(d)
        return d


# Order of the root Mux feeding "Communication - QDrone" (Stage 6 model):
# MISSION SERVER outputs 1..5 -> Mux inputs 1..5.
QDRONE_PACKET_FIELDS = (
    "trigger_arm", "trigger_takeoff", "trigger_abort", "trigger_joystick_issue",  # Triggers [4]
    "is_tracking",  # isTracking (0,1)
    "measured_x", "measured_y", "measured_z", "measured_yaw",  # Measured Pose [4]
    "desired_x", "desired_y", "desired_z", "desired_yaw",  # Desired Pose [4]
    "timestamp_s",  # Timestamp to send (s), echoed back by the vehicle
)


def qdrone_packet(triggers: Sequence[float], is_tracking: float, measured_pose: Sequence[float],
                  desired_pose: Sequence[float], timestamp_s: float) -> tuple[float, ...]:
    """Assemble the 14-element vector the mission server sends to the QDrone.

    Element order is read from the model. Element data types are not: the
    Stream Server's data type is "Inherited via back propagation" and the
    vehicle-side model that defines what it expects was not supplied, so no
    byte encoder is provided for this packet.
    """
    pkt = (*map(float, triggers), float(is_tracking), *map(float, measured_pose),
           *map(float, desired_pose), float(timestamp_s))
    if len(pkt) != len(QDRONE_PACKET_FIELDS):
        raise ValueError(f"expected {len(QDRONE_PACKET_FIELDS)} elements, got {len(pkt)}")
    return pkt
