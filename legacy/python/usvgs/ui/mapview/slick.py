"""The oil slick as one image.

The simulator moves the oil as a cloud of parcels.  Drawing each one as a soft
round blob costs a stroke per parcel per frame; instead the parcels are binned
onto a small grid in metres, smoothed, and turned into film thickness, then
coloured by how oil looks at that thickness (Bonn Agreement: rainbow-grey
sheen, then brown, then black) into one image.  It is rebuilt only when the
slick has moved, and blitted each frame like the chart.  It also reads as a
slick rather than as dots.
"""

from __future__ import annotations

import time

import numpy as np
from PyQt5 import QtGui
from PyQt5.QtCore import QRectF, Qt

from ...core import geo
from .polygons import polygon

#: Spread of each parcel: about the size of the old blobs.
SIGMA_M = 2.5
#: The grid is at most this many cells along its longer side.
MAX_CELLS = 220
#: Rebuilt at most this often (s); the slick drifts slowly.
REBUILD_S = 0.15

#: log10(film thickness in µm) → premultiplied-later RGBA.
STOPS = np.array([
    (-1.6, 150, 140, 190, 0),      # barely there
    (-0.6, 150, 140, 190, 70),     # sheen
    (0.3, 120, 100, 120, 105),     # rainbow / metallic
    (1.0, 90, 60, 35, 150),        # brown
    (1.8, 25, 15, 8, 210),         # black, thick
])


def _blur(grid: np.ndarray, sigma_cells: float) -> np.ndarray:
    """Gaussian blur, separable, as a few whole-array shifted adds per axis."""
    radius = max(1, int(3 * sigma_cells))
    x = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * (x / sigma_cells) ** 2)
    kernel /= kernel.sum()
    for axis in (0, 1):
        n = grid.shape[axis]
        padded = np.pad(grid, [(radius, radius) if a == axis else (0, 0) for a in (0, 1)])
        out = np.zeros_like(grid)
        for k, weight in enumerate(kernel):
            out += weight * (padded[k:k + n] if axis == 0 else padded[:, k:k + n])
        grid = out
    return grid


def colour_film(film_um: np.ndarray) -> np.ndarray:
    """RGBA (uint8, premultiplied) for an array of film thicknesses."""
    t = np.log10(np.maximum(film_um, 1e-3))
    rgba = np.stack([np.interp(t, STOPS[:, 0], STOPS[:, i]) for i in range(1, 5)], axis=-1)
    rgba[..., :3] *= rgba[..., 3:4] / 255.0
    return rgba.astype(np.uint8)


class SlickImage:
    def __init__(self) -> None:
        self._source = None
        self._built_at = 0.0
        self._image: QtGui.QImage | None = None
        self._bounds: tuple[float, float, float, float] | None = None   # north, west, south, east
        self._beached: tuple[np.ndarray, np.ndarray] | None = None

    def _build(self, oil) -> None:
        lat, lon, volume, beached = oil
        live = volume > 1e-4
        floating = live & ~beached
        self._beached = (lat[live & beached], lon[live & beached])
        self._image = None
        if not floating.any():
            return
        lat, lon, volume = lat[floating], lon[floating], volume[floating]
        lat0 = float(lat.mean())
        m_lat, m_lon = geo.metres_per_degree(lat0)
        north = (lat - lat0) * m_lat
        east = (lon - float(lon.mean())) * m_lon
        margin = 4 * SIGMA_M
        n0, n1 = north.min() - margin, north.max() + margin
        e0, e1 = east.min() - margin, east.max() + margin
        cell = max(0.8, max(n1 - n0, e1 - e0) / MAX_CELLS)
        rows = max(2, int(np.ceil((n1 - n0) / cell)))
        cols = max(2, int(np.ceil((e1 - e0) / cell)))
        n1, e1 = n0 + rows * cell, e0 + cols * cell
        # Row 0 is the northern edge, like the chart.
        litres, _, _ = np.histogram2d(n1 - north, east - e0, bins=(rows, cols),
                                      range=((0, rows * cell), (0, cols * cell)), weights=volume)
        litres = _blur(litres, SIGMA_M / cell)
        film_um = litres / (cell * cell) * 1000.0     # 1 L/m² is a 1 mm film
        data = np.ascontiguousarray(colour_film(film_um))
        self._image = QtGui.QImage(data.data, cols, rows, 4 * cols,
                                   QtGui.QImage.Format_RGBA8888_Premultiplied).copy()
        lon0 = float(lon.mean())
        self._bounds = (lat0 + n1 / m_lat, lon0 + e0 / m_lon, lat0 + n0 / m_lat, lon0 + e1 / m_lon)

    def paint(self, painter: QtGui.QPainter, canvas, oil) -> None:
        if oil is None:
            return
        now = time.monotonic()
        if oil is not self._source and (now - self._built_at >= REBUILD_S or self._source is None):
            self._source, self._built_at = oil, now
            self._build(oil)
        if self._image is not None:
            north, west, south, east = self._bounds
            left, top = canvas.to_screen(north, west)
            right, bottom = canvas.to_screen(south, east)
            painter.save()
            painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
            painter.drawImage(QRectF(left, top, right - left, bottom - top), self._image)
            painter.restore()
        if self._beached is not None and len(self._beached[0]):
            x, y = canvas.to_screen_array(*self._beached)
            px = max(3.0, 4.0 / canvas.metres_per_pixel())
            painter.save()
            painter.setPen(QtGui.QPen(QtGui.QColor(20, 10, 4, 220), px, Qt.SolidLine, Qt.RoundCap))
            painter.drawPoints(polygon(x, y))
            painter.restore()
