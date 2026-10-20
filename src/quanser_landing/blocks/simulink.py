"""Python equivalents of the Simulink/QUARC blocks around the Stage 6 charts.

Unlike :mod:`.charts`, these blocks have no MATLAB source to run, so they are
**not** parity-checked against an oracle. Each class states which part of its
behaviour comes straight from the model XML and which part comes from the
block's documentation (QUARC or Simulink). Points the documentation leaves
open are marked ``ASSUMPTION`` and listed in ``docs/PORT_NOTES.md``.
"""
from __future__ import annotations

import math
import operator
from dataclasses import dataclass, field
from typing import Callable, Sequence

_OPS: dict[str, Callable[[float, float], bool]] = {
    ">": operator.gt, ">=": operator.ge, "<": operator.lt,
    "<=": operator.le, "==": operator.eq, "~=": operator.ne,
}


@dataclass
class QuarcThreshold:
    """QUARC ``Threshold`` block (Discontinuities library).

    From the model: operator, threshold and duration (``>=``, 0.5, 0.1 s in
    the communication-loss path; ``>=``, 0.5, 0.2 s for the joystick paths).
    From the QUARC docs: ``elapsed`` is 1 when the comparison has been true
    for longer than ``duration``; the timer resets when it becomes false.

    ASSUMPTION: "longer than" is implemented as ``held > duration`` with the
    hold time measured from the first sample at which the comparison became
    true. The docs do not pin down the boundary sample.
    """

    op: str = ">="
    threshold: float = 0.5
    duration: float = 0.1
    _since: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self._since = None

    def step(self, t: float, u: float) -> tuple[int, float]:
        if _OPS[self.op](u, self.threshold):
            if self._since is None:
                self._since = t
            held = t - self._since
        else:
            self._since = None
            held = 0.0
        return int(held > self.duration), held


@dataclass
class FallingEdgeLatch:
    """``Triggered Subsystem`` in the mission server (falling-edge trigger).

    From the model: a Memory + Add(+1) counter that runs on each falling
    edge of the communication-issue signal, followed by ``>= 1``; initial
    output 0, states held. The mission server multiplies the raw issue flag
    by this output, so a communication issue is only reported after the
    link has recovered from an issue once.
    """

    count: int = 0
    _prev: float | None = field(default=None, init=False)

    def reset(self) -> None:
        self.count = 0
        self._prev = None

    def step(self, u: float) -> int:
        # Simulink "falling": positive -> zero/negative (also zero -> negative).
        if self._prev is not None and ((self._prev > 0 and u <= 0) or (self._prev == 0 and u < 0)):
            self.count += 1
        self._prev = u
        return int(self.count >= 1)


@dataclass
class StreamCommMonitor:
    """``Communication - QDrone`` / ``QBot 2e`` loss detection, end to end.

    chart_33 -> Threshold(>=0.5 held 0.1 s) -> floor conversion gives the
    subsystem's ``Communication Issue (0,1)``. The mission server then gates
    it with :class:`FallingEdgeLatch`. ``step`` returns both values.
    """

    threshold: QuarcThreshold = field(default_factory=QuarcThreshold)
    latch: FallingEdgeLatch = field(default_factory=FallingEdgeLatch)

    def reset(self) -> None:
        self.threshold.reset()
        self.latch.reset()

    def step(self, t: float, state: float, error: float) -> tuple[int, int]:
        from .charts import detect_stream_connection_loss

        raw = detect_stream_connection_loss(t, state, error)
        issue, _ = self.threshold.step(t, raw)
        issue = int(math.floor(issue))
        reported = issue * self.latch.step(issue)
        return issue, reported


@dataclass
class InverseModulus:
    """QUARC ``Inverse Modulus`` (unwrap) with modulus ``2*pi`` in the model.

    ASSUMPTION: a jump of more than half the modulus between consecutive
    samples is treated as a wrap; the first sample passes through unchanged.
    This matches the documented Nyquist constraint but the exact boundary
    (exactly half a modulus) is not documented.
    """

    modulus: float = 2 * math.pi
    _prev_in: float | None = field(default=None, init=False)
    _offset: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self._prev_in = None
        self._offset = 0.0

    def step(self, u: float) -> float:
        if self._prev_in is not None:
            d = u - self._prev_in
            if d > self.modulus / 2:
                self._offset -= self.modulus
            elif d < -self.modulus / 2:
                self._offset += self.modulus
        self._prev_in = u
        return u + self._offset


# "Rotate Basis from RSF to MF" constant in both Convention Correction blocks.
R_RSF_TO_MF = ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0))


def _matvec_transpose(m, v):
    return tuple(sum(m[r][c] * v[r] for r in range(3)) for c in range(3))


def motive_to_rsf_position(xyz_mf: Sequence[float]) -> tuple[float, float, float]:
    """Position half of ``Convention Correction (Motive Frame to Research
    Studio Frame)``: ``xyz_RSF = R' * xyz_MF`` with ``R`` the constant above.

    So ``x_RSF = -z_MF``, ``y_RSF = -x_MF``, ``z_RSF = y_MF``.
    """
    return _matvec_transpose(R_RSF_TO_MF, xyz_mf)


def motive_to_rsf_rotation_pre_gimbal(rot_mf: Sequence[float]) -> tuple[float, float, float]:
    """``R' * rot_MF``: the model multiplies the Euler-angle *vector* by the
    basis matrix before its ``Prevent Gimbal Lock`` subsystem. That
    subsystem (QUARC Euler-angle blocks, relative YZX then ZYX) is not ported:
    its angle conventions could not be confirmed without QUARC. See
    ``docs/PORT_NOTES.md``.
    """
    return _matvec_transpose(R_RSF_TO_MF, rot_mf)


@dataclass
class MeasurementConditioning:
    """``Measurement Conditioning`` subsystem (stored parameters only).

    Input: the 7-vector ``[position(3), orientation(3), tracking]`` produced by
    ``Extract Vehicle Data - QDrone``. Steps, all from the model XML:

    1. position + bias ``[0 0 -0.07]`` ("subtract rigid body reference
       offset (initial height)");
    2. saturate to ``[-2 -2 0] .. [2 2 5]`` m;
    3. yaw = orientation(3), unwrapped by Inverse Modulus ``2*pi``.

    Outputs: ``measured_pose [x y z yaw]``, ``is_tracking`` and the corrected
    7-vector ``[pos(3), roll, pitch, yaw_unwrapped, tracking]``.
    """

    bias: tuple[float, float, float] = (0.0, 0.0, -0.07)
    lower: tuple[float, float, float] = (-2.0, -2.0, 0.0)
    upper: tuple[float, float, float] = (2.0, 2.0, 5.0)
    unwrap: InverseModulus = field(default_factory=InverseModulus)

    def reset(self) -> None:
        self.unwrap.reset()

    def step(self, data7: Sequence[float]):
        if len(data7) != 7:
            raise ValueError("expected [position(3), orientation(3), tracking]")
        pos = [data7[i] + self.bias[i] for i in range(3)]
        # Simulink Saturation: min(max(u, lower), upper)
        pos = [min(max(pos[i], self.lower[i]), self.upper[i]) for i in range(3)]
        yaw = self.unwrap.step(float(data7[5]))
        measured = (pos[0], pos[1], pos[2], yaw)
        corrected = (pos[0], pos[1], pos[2], float(data7[3]), float(data7[4]), yaw, float(data7[6]))
        return measured, float(data7[6]), corrected


def stage6_desired_pose(use_yolo: float, yolo_cmd_pose: Sequence[float],
                        mode_switch_output: float) -> tuple[float, float, float, float]:
    """``Switch1`` feeding ``Desired Pose (m, rad) [4]`` in Stage 6, verbatim.

    ``u2 > 0.5`` (``use_yolo``) selects the YOLO ``cmd_pose``; otherwise the
    third input is selected. In Stage 6 that third input is wired to the
    *Mode switch* output (a scalar, 0 or 1), which Simulink scalar-expands to
    ``[m, m, m, m]``. In the preceding integration model it was the
    follower/home pose switch. This function reproduces the Stage 6 wiring
    so replay matches the model; :mod:`quanser_landing.io.safety` refuses
    to forward such a pose. See ``docs/SIMULINK_SPEC.md`` (finding F1).
    """
    if use_yolo > 0.5:
        return tuple(float(v) for v in yolo_cmd_pose)  # type: ignore[return-value]
    m = float(mode_switch_output)
    return (m, m, m, m)
