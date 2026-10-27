"""MATLAB semantics that differ from Python's builtins."""
from __future__ import annotations

import math


def mmax(a: float, b: float) -> float:
    """MATLAB ``max(a, b)`` for scalars: NaN operands are ignored."""
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return a if a >= b else b


def mmin(a: float, b: float) -> float:
    """MATLAB ``min(a, b)`` for scalars: NaN operands are ignored."""
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return a if a <= b else b


def clamp(x: float, lo: float, hi: float) -> float:
    """``min(max(x, lo), hi)`` exactly as written in the MATLAB source."""
    return mmin(mmax(x, lo), hi)


def all_finite(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def norm2(x: float, y: float) -> float:
    """2-norm of a 2-vector (MATLAB ``norm`` uses a scaled, overflow-safe sum)."""
    return math.hypot(x, y)


def norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)
