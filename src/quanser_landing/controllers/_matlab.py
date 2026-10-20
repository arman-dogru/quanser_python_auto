"""Small helpers that reproduce MATLAB scalar semantics exactly.

MATLAB's ``max``/``min`` ignore NaN when the other operand is a number:
``max(NaN, -1) == -1``. Python's built-ins do not (``max(nan, -1)`` returns
``nan`` or ``-1`` depending on argument order), so the controllers use these
helpers wherever the MATLAB source uses ``min``/``max``.
"""
from __future__ import annotations

import math


def mmax(a: float, b: float) -> float:
    """MATLAB ``max(a, b)`` for real scalars (NaN-ignoring)."""
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return a if a >= b else b


def mmin(a: float, b: float) -> float:
    """MATLAB ``min(a, b)`` for real scalars (NaN-ignoring)."""
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return a if a <= b else b


def clamp(value: float, low: float, high: float) -> float:
    """``min(max(value, low), high)`` with MATLAB NaN semantics.

    A NaN input therefore becomes ``low``, exactly as in the source charts.
    """
    return mmin(mmax(value, low), high)
