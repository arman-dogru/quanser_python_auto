"""The two non-controller MATLAB Function charts embedded in Stage 6.

* ``chart_13_fcn.m`` / ``chart_23_fcn.m`` (identical): ``Extract Object Data``,
  which slices one rigid body out of the OptiTrack data vector.
* ``chart_33_fcn.m`` / ``chart_42_fcn.m`` (identical): ``Detect
  Communication loss`` for the QDrone and QBot 2e stream servers.

Both are stateless. Parity with the ``.m`` files is checked against Octave.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


class MatlabIndexError(ValueError):
    """Raised where MATLAB would error on a non-integer or out-of-range index."""


@dataclass(frozen=True)
class ObjectData:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float]
    tracking: float


def _at(data: Sequence[float], one_based: float) -> float:
    if (not math.isfinite(one_based) or one_based != int(one_based)
            or one_based < 1 or one_based > len(data)):
        raise MatlabIndexError(f"index {one_based!r} is not valid for length {len(data)}")
    return float(data[int(one_based) - 1])


def extract_object_data(data: Sequence[float], index: float) -> ObjectData:
    """``[position, orientation, tracking] = fcn(data, index)``.

    ``data`` is laid out as ``[p_1..p_n (3 each), r_1..r_n (3 each),
    t_1..t_n]`` with ``n = length(data) / 7``. Out-of-range indices (``<= 0``
    or ``> n``) return zeros, as in the source. Indices that MATLAB cannot
    use (non-integer, or a length that is not a multiple of 7) raise
    :class:`MatlabIndexError` where MATLAB would raise.
    """
    n = len(data) / 7
    if index <= 0 or index > n:
        return ObjectData((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    position = (_at(data, index * 3 - 2), _at(data, index * 3 - 1), _at(data, index * 3))
    orientation = (_at(data, (index + n) * 3 - 2), _at(data, (index + n) * 3 - 1),
                   _at(data, (index + n) * 3))
    tracking = _at(data, index + n * 6)
    return ObjectData(position, orientation, tracking)


def detect_stream_connection_loss(time: float, state: float, error: float) -> int:
    """``stream_connection_loss = fcn(time, state, error)``.

    After the first second, anything other than ``state == 2`` (QUARC
    ``PSTREAM_STATE_CONNECTED``) with ``error >= 0`` is a loss. NaN inputs
    therefore count as a loss after one second.
    """
    loss = 0
    if time > 1:
        if not (state == 2 and error >= 0):
            loss = 1
    return loss
