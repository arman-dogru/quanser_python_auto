"""Port of "Extract Vehicle Data - QDrone/Extract Object Data" chart."""
from __future__ import annotations


def extract_object_data(data, index):
    """Return ``(position[3], orientation[3], tracking)`` for rigid body ``index``.

    ``data`` is the Data Server vector: positions of all n bodies, then their
    orientations, then n tracking flags (n = len(data)/7). MATLAB's ``n`` is a
    double, so a length that is not a multiple of 7 gives a fractional n, which
    this port reproduces.
    """
    d = [float(v) for v in data]
    n = len(d) / 7
    if index <= 0 or index > n:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0

    def at(i):  # MATLAB 1-based indexing; fractional indices are an error there too
        if i != int(i):
            raise IndexError("non-integer index")
        return d[int(i) - 1]
    position = (at(index * 3 - 2), at(index * 3 - 1), at(index * 3))
    orientation = (at((index + n) * 3 - 2), at((index + n) * 3 - 1), at((index + n) * 3))
    tracking = at(index + n * 6)
    return position, orientation, tracking
