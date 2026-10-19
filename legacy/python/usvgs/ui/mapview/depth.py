"""Chart layers: depth shading, isobaths, spot depths, and the survey.

The chart grid is coloured once into an image and drawn by its corners, so the
whole layer costs one scaled blit per repaint.  Isobaths are computed by
marching squares, once per level of detail: zoomed out, they come from a
coarser copy of the grid, which looks the same at that size and has a fraction
of the segments.  They are filtered to the view and handed to Qt from numpy in
one call per depth.  The survey is drawn
from per-chunk images that are rebuilt only when a chunk gets new soundings.
"""

from __future__ import annotations

import math

import numpy as np
from PyQt5 import QtGui
from PyQt5.QtCore import QPointF, QRectF, Qt

from ...core.bathymetry import CHUNK, ESTIMATED, DepthGrid, SoundingGrid, contour_segments
from .. import theme
from .polygons import polygon

ISOBATHS = (0.0, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0)
#: Isobaths come from a grid this many times coarser...
LOD_FACTORS = (1, 2, 4, 8)
#: ...the first whose cells are at least this many pixels across.
LOD_CELL_PX = 5.0
#: Over natural water the coastline is surf only this close in (metres per
#: pixel); further out it is a thin line.
SURF_MAX_M_PER_PX = 2.5


def _image_from_rgba(rgba: np.ndarray) -> QtGui.QImage:
    rgba = np.ascontiguousarray(rgba)
    height, width = rgba.shape[:2]
    image = QtGui.QImage(rgba.data, width, height, 4 * width, QtGui.QImage.Format_RGBA8888)
    return image.copy()  # own the pixels; the numpy buffer goes away


class ChartLayer:
    def __init__(self, grid: DepthGrid, shallow_alarm_m: float) -> None:
        self.grid = grid
        self.shallow_alarm_m = shallow_alarm_m
        self.image = self._render()
        self._land_window: tuple[int, int, int, int] | None = None
        self._land = True
        self._isobaths: dict[int, dict[float, np.ndarray]] = {}
        self.isobaths = self.isobaths_at(1)

    def isobaths_at(self, factor: int) -> dict[float, np.ndarray]:
        """The contours of the grid averaged over ``factor`` × ``factor`` cells."""
        if factor not in self._isobaths:
            grid = self.grid
            if factor > 1:
                rows, cols = (s // factor for s in grid.shape)
                z = np.nan_to_num(grid.depth[:rows * factor, :cols * factor], nan=0.0)
                z = z.reshape(rows, factor, cols, factor).mean(axis=(1, 3))
                # Same corner, bigger cells: the coarse cell centres land on
                # the middle of each block.
                grid = DepthGrid(z, grid.north, grid.west, grid.dlat * factor, grid.dlon * factor)
            self._isobaths[factor] = {level: contour_segments(grid, level) for level in ISOBATHS}
        return self._isobaths[factor]

    def set_shallow_alarm(self, metres: float) -> None:
        if abs(metres - self.shallow_alarm_m) > 1e-6:
            self.shallow_alarm_m = metres
            self.image = self._render()

    def _render(self) -> QtGui.QImage:
        depth = self.grid.depth
        water = np.isfinite(depth)
        d = np.nan_to_num(depth, nan=0.0)
        f = np.clip(d / 30.0, 0.0, 1.0) ** 0.55
        shallow = np.array(theme.WATER_SHALLOW, np.float32)
        deep = np.array(theme.WATER_DEEP, np.float32)
        rgb = shallow + (deep - shallow) * f[..., None]

        # Relief: light from the north-west, so banks and channels read as shapes.
        gy, gx = np.gradient(d)
        shade = np.clip(1.0 + 0.10 * (gx - gy), 0.75, 1.3)
        rgb *= shade[..., None]

        hazard = water & (d < self.shallow_alarm_m)
        rgb[hazard] = rgb[hazard] * 0.45 + np.array(theme.WATER_HAZARD, np.float32) * 0.55
        estimated = self.grid.quality == ESTIMATED
        rgb[estimated] = rgb[estimated] * 0.75 + np.array((90, 90, 120), np.float32) * 0.25

        land = QtGui.QColor(theme.LAND)
        rgb[~water] = (land.red(), land.green(), land.blue())
        rgba = np.empty(depth.shape + (4,), np.uint8)
        rgba[..., :3] = np.clip(rgb, 0, 255).astype(np.uint8)
        rgba[..., 3] = 255
        return _image_from_rgba(rgba)

    def _window(self, canvas) -> tuple[int, int, int, int]:
        """Rows and columns of the grid in view: r0, r1, c0, c1."""
        g = self.grid
        min_lat, min_lon, max_lat, max_lon = canvas.view_bounds()
        c0 = max(0, math.floor((min_lon - g.west) / g.dlon) - 1)
        c1 = min(g.shape[1], math.ceil((max_lon - g.west) / g.dlon) + 1)
        r0 = max(0, math.floor((g.north - max_lat) / g.dlat) - 1)
        r1 = min(g.shape[0], math.ceil((g.north - min_lat) / g.dlat) + 1)
        return r0, r1, c0, c1

    def land_in_view(self, canvas) -> bool:
        r0, r1, c0, c1 = self._window(canvas)
        if (r0, r1, c0, c1) != self._land_window:
            self._land_window = (r0, r1, c0, c1)
            self._land = bool(np.isnan(self.grid.depth[r0:r1, c0:c1]).any()) \
                if r1 > r0 and c1 > c0 else False
        return self._land

    def paint(self, painter: QtGui.QPainter, canvas, image: QtGui.QImage | None = None) -> None:
        """The chart shading, or another ``image`` of the same grid, over the view."""
        g = self.grid
        # Visible part of the grid, in image pixels, so a deep zoom does not
        # scale the whole gulf up to draw a few cells.
        r0, r1, c0, c1 = self._window(canvas)
        if c1 <= c0 or r1 <= r0:
            return
        left, top = canvas.to_screen(g.north - r0 * g.dlat, g.west + c0 * g.dlon)
        right, bottom = canvas.to_screen(g.north - r1 * g.dlat, g.west + c1 * g.dlon)
        painter.save()
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.drawImage(QRectF(left, top, right - left, bottom - top), image or self.image,
                          QRectF(c0, r0, c1 - c0, r1 - r0))
        painter.restore()

    def paint_isobaths(self, painter: QtGui.QPainter, canvas, natural: bool = False) -> None:
        """Depth contours.  Over natural water they are fainter, and the
        coastline is surf: a soft white band with a brighter edge."""
        min_lat, min_lon, max_lat, max_lon = canvas.view_bounds()
        cell_px = self.grid.dlat * canvas.scale
        factor = next((f for f in LOD_FACTORS if cell_px * f >= LOD_CELL_PX), LOD_FACTORS[-1])
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        for level, segments in self.isobaths_at(factor).items():
            if not len(segments):
                continue
            # Thin out the deep contours when zoomed far out.
            if cell_px < 3.0 and level not in (0.0, 5.0, 20.0):
                continue
            inside = (
                (segments[:, 0] >= min_lon) & (segments[:, 0] <= max_lon)
                & (segments[:, 1] >= min_lat) & (segments[:, 1] <= max_lat)
            )
            chosen = segments[inside]
            if not len(chosen):
                continue
            if level == 0.0 and natural:
                pen = None
            elif level == 0.0:
                pen = QtGui.QPen(theme.qcolor(theme.LAND_EDGE), 1.6)
            else:
                r, g, b = theme.CONTOUR
                alpha = 150 if level in (2.0, 5.0, 10.0, 20.0) else 90
                pen = QtGui.QPen(QtGui.QColor(r, g, b, alpha), 1.0)
                if level <= self.shallow_alarm_m + 1e-6:
                    pen.setColor(QtGui.QColor(*theme.WATER_HAZARD, 200))
                if natural:
                    colour = pen.color()
                    colour.setAlpha(colour.alpha() // 2)
                    pen.setColor(colour)
            # Point pairs, one line each: (start, end, start, end, ...).
            lon = chosen[:, [0, 2]].ravel()
            lat = chosen[:, [1, 3]].ravel()
            x, y = canvas.to_screen_array(lat, lon)
            lines = polygon(x, y)
            if pen is None:
                metres_px = canvas.metres_per_pixel()
                if metres_px > SURF_MAX_M_PER_PX:
                    # Too far out for surf to show: a thin line of sand.
                    painter.setPen(QtGui.QPen(QtGui.QColor(206, 196, 160, 150), 1.3))
                    painter.drawLines(lines)
                    continue
                # Surf: a soft band a few metres wide with a brighter edge.
                width = max(2.0, 5.0 / metres_px)
                for stroke, alpha in ((width, 45), (width * 0.4, 85), (1.0, 150)):
                    painter.setPen(QtGui.QPen(QtGui.QColor(235, 245, 250, alpha), stroke,
                                              Qt.SolidLine, Qt.RoundCap))
                    painter.drawLines(lines)
                continue
            painter.setPen(pen)
            painter.drawLines(lines)
        painter.restore()

    def paint_spot_depths(self, painter: QtGui.QPainter, canvas) -> None:
        """Depth figures in the cells, as on a paper chart, once they are legible."""
        g = self.grid
        cell_px = g.dlat * canvas.scale
        if cell_px < 46:
            return
        min_lat, min_lon, max_lat, max_lon = canvas.view_bounds()
        c0 = max(0, math.floor((min_lon - g.west) / g.dlon))
        c1 = min(g.shape[1], math.ceil((max_lon - g.west) / g.dlon))
        r0 = max(0, math.floor((g.north - max_lat) / g.dlat))
        r1 = min(g.shape[0], math.ceil((g.north - min_lat) / g.dlat))
        if (c1 - c0) * (r1 - r0) > 900:
            return
        painter.save()
        font = painter.font()
        font.setPointSize(8)
        font.setItalic(True)
        painter.setFont(font)
        painter.setPen(QtGui.QColor(200, 225, 245, 170))
        for r in range(r0, r1):
            for c in range(c0, c1):
                value = g.depth[r, c]
                if value != value:
                    continue
                x, y = canvas.to_screen(g.north - (r + 0.5) * g.dlat, g.west + (c + 0.5) * g.dlon)
                text = f"{value:.1f}" if value < 10 else f"{value:.0f}"
                if g.quality[r, c] == ESTIMATED:
                    text = f"~{text}"
                painter.drawText(QRectF(x - 24, y - 8, 48, 16), Qt.AlignCenter, text)
        painter.restore()


SURVEY_STOPS = ((0.0, (230, 40, 40)), (0.25, (250, 200, 40)), (0.5, (60, 210, 90)),
                (0.75, (40, 190, 230)), (1.0, (60, 70, 220)))


def survey_rgb_array(depth: np.ndarray, low: float, high: float) -> np.ndarray:
    """:func:`survey_rgb` for many depths at once, as uint8 RGB."""
    span = max(0.5, high - low)
    t = np.clip((depth - low) / span, 0.0, 1.0)
    at = [s for s, _ in SURVEY_STOPS]
    return np.stack([np.interp(t, at, [c[i] for _, c in SURVEY_STOPS]) for i in range(3)],
                    axis=-1).astype(np.uint8)


def survey_rgb(depth: float, low: float, high: float) -> tuple[int, int, int]:
    """The multibeam palette: red shallow, through yellow and green, to blue deep."""
    span = max(0.5, high - low)
    t = min(1.0, max(0.0, (depth - low) / span))
    stops = ((0.0, (230, 40, 40)), (0.25, (250, 200, 40)), (0.5, (60, 210, 90)),
             (0.75, (40, 190, 230)), (1.0, (60, 70, 220)))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:], strict=False):
        if t <= t1:
            k = (t - t0) / (t1 - t0)
            return tuple(int(a + (b - a) * k) for a, b in zip(c0, c1, strict=True))
    return stops[-1][1]


class SurveyLayer:
    def __init__(self, grid: SoundingGrid) -> None:
        self.grid = grid
        self._images: dict[tuple[int, int], QtGui.QImage] = {}
        self._range = (0.0, 10.0)
        self.opacity = 0.85

    def depth_range(self) -> tuple[float, float]:
        return self._range

    def refresh(self, lock) -> None:
        """Rebuild the chunks that changed.  Call on the interface thread."""
        with lock:
            grid = self.grid
            if not len(grid):
                self._images.clear()
                grid.dirty_chunks.clear()
                return
            low, high = grid.min_depth, grid.max_depth
            # A big change of range recolours everything.
            if abs(low - self._range[0]) > 0.5 or abs(high - self._range[1]) > 0.5:
                self._range = (low, max(low + 0.5, high))
                grid.dirty_chunks |= grid.chunks()
            dirty = list(grid.dirty_chunks)
            grid.dirty_chunks.clear()
            cells = {chunk: list(grid.chunk_cells(chunk)) for chunk in dirty}
        for chunk, entries in cells.items():
            if not entries:
                self._images.pop(chunk, None)
                continue
            dx, dy, depth = (np.array(column) for column in zip(*entries, strict=True))
            rgba = np.zeros((CHUNK, CHUNK, 4), np.uint8)
            rgba[CHUNK - 1 - dy, dx, :3] = survey_rgb_array(depth, *self._range)
            rgba[CHUNK - 1 - dy, dx, 3] = 255
            self._images[chunk] = _image_from_rgba(rgba)

    def paint(self, painter: QtGui.QPainter, canvas) -> None:
        if not self._images:
            return
        painter.save()
        painter.setOpacity(self.opacity)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        min_lat, min_lon, max_lat, max_lon = canvas.view_bounds()
        for chunk, image in self._images.items():
            sw, ne = self.grid.chunk_bounds(chunk)
            if ne.lat < min_lat or sw.lat > max_lat or ne.lon < min_lon or sw.lon > max_lon:
                continue
            left, top = canvas.to_screen(ne.lat, sw.lon)
            right, bottom = canvas.to_screen(sw.lat, ne.lon)
            painter.drawImage(QRectF(left, top, right - left, bottom - top), image)
        painter.restore()


def paint_legend(painter: QtGui.QPainter, rect: QRectF, low: float, high: float,
                 title: str) -> None:
    """A vertical colour bar for the survey palette."""
    painter.save()
    painter.setPen(Qt.NoPen)
    painter.setBrush(QtGui.QColor(8, 14, 20, 200))
    painter.drawRoundedRect(rect.adjusted(-6, -20, 40, 6), 6, 6)
    steps = 24
    for i in range(steps):
        t = i / (steps - 1)
        painter.setBrush(QtGui.QColor(*survey_rgb(low + t * (high - low), low, high)))
        h = rect.height() / steps
        painter.drawRect(QRectF(rect.left(), rect.top() + i * h, rect.width(), h + 0.5))
    painter.setPen(QtGui.QColor(theme.TEXT))
    font = painter.font()
    font.setPointSize(8)
    painter.setFont(font)
    painter.drawText(QPointF(rect.left() - 2, rect.top() - 7), title)
    painter.drawText(QPointF(rect.right() + 4, rect.top() + 9), f"{low:.1f}")
    painter.drawText(QPointF(rect.right() + 4, rect.bottom()), f"{high:.1f}")
    painter.restore()

