"""Qt point lists straight from numpy.

Building a ``QPolygonF`` one ``QPointF`` at a time costs a Python call per
point; filling its buffer from numpy costs one copy.
"""

from __future__ import annotations

import numpy as np
from PyQt5 import QtGui


def polygon(xs: np.ndarray, ys: np.ndarray) -> QtGui.QPolygonF:
    """Points ``(xs[i], ys[i])`` as one ``QPolygonF``."""
    poly = QtGui.QPolygonF(len(xs))
    if len(xs):
        buffer = poly.data()
        buffer.setsize(16 * len(xs))
        view = np.frombuffer(buffer, np.float64).reshape(-1, 2)
        view[:, 0], view[:, 1] = xs, ys
    return poly
