"""Natural water: the sea as it looks, not as a chart colours it.

Three parts, cheapest first:

* **colour** — one image of the chart grid, coloured the way water looks over
  a bottom at that depth: sand-turquoise in the shallows, through green-blue,
  to dark blue in deep water.  Land is left transparent.  Shallows under the
  depth alarm keep a warm tint, since this is still the chart you steer by.
* **waves** — a small tile of travelling ripples, shaded as if lit from the
  sky, in a loop of frames.  It is laid over the water as a brush, turned to
  run downwind and fixed to the ground so it does not slide when the chart
  pans.  Stronger wind makes it bigger and stronger.
* **foam** — the 0 m isobath is drawn as surf along the beach (in
  :mod:`.depth`).

The waves are drawn into a small off-screen layer, a quarter of the view's
pixels, and the land is cut out of it there, where it is cheap; the layer is
redrawn only when the waves or the view move enough to show.
"""

from __future__ import annotations

import math
import time

import numpy as np
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import QRectF

from .. import theme

#: Wave tile: pixels a side, frames in the loop, and the loop's length.
TILE = 256
FRAMES = 24
LOOP_S = 3.0
#: The waves are drawn at 1/WAVE_RES of the view's resolution each way.
WAVE_RES = 2

#: Colour of water over a bottom at depth d (m), from the beach outwards.
DEPTH_COLOURS = (
    (0.0, (88, 172, 160)),
    (1.5, (52, 146, 152)),
    (4.0, (28, 112, 138)),
    (10.0, (16, 74, 110)),
    (25.0, (10, 48, 82)),
    (50.0, (7, 32, 60)),
)


def water_rgb(depth: np.ndarray) -> np.ndarray:
    """Natural water colour for an array of depths, as float RGB."""
    stops = np.array([d for d, _ in DEPTH_COLOURS])
    colours = np.array([c for _, c in DEPTH_COLOURS], np.float32)
    d = np.clip(depth, stops[0], stops[-1])
    return np.stack([np.interp(d, stops, colours[:, i]) for i in range(3)], axis=-1)


def _wave_frames() -> list[QtGui.QImage]:
    """A loop of tileable ripple frames, as light and shadow over transparent.

    Each frame is a sum of travelling sine waves.  Every wave fits the tile a
    whole number of times and goes round a whole number of times in the loop,
    so the tile repeats in space and in time without a seam.  The waves run
    along +x, spread ±40°, and longer ones move faster, as on real water.
    """
    rng = np.random.default_rng(11)
    y, x = (np.mgrid[0:TILE, 0:TILE] / TILE).astype(np.float32)
    waves = []
    for _ in range(18):
        cycles = rng.uniform(3, 16)
        angle = rng.normal(0.0, 0.45)
        kx = round(cycles * math.cos(angle))
        ky = round(cycles * math.sin(angle))
        k = math.hypot(kx, ky)
        if k < 2:
            continue
        # Deep-water dispersion: speed goes as the square root of wavelength,
        # so the turns per loop go as the square root of the wavenumber.
        turns = max(1, round(1.6 * math.sqrt(k)))
        waves.append((kx, ky, turns, k ** -1.3, rng.uniform(0, 2 * math.pi)))

    light = np.array([-0.55, -0.85], np.float32)   # from the top of the tile: the sky ahead
    frames = []
    for f in range(FRAMES):
        t = f / FRAMES
        gx = np.zeros((TILE, TILE), np.float32)
        gy = np.zeros((TILE, TILE), np.float32)
        height = np.zeros((TILE, TILE), np.float32)
        for kx, ky, turns, amp, phase in waves:
            arg = 2 * math.pi * (kx * x + ky * y - turns * t) + phase
            c = np.cos(arg)
            height += amp * np.sin(arg)
            gx += amp * kx * c
            gy += amp * ky * c
        slope = gx * light[0] + gy * light[1]
        slope /= 2.2 * float(slope.std()) or 1.0
        bright = np.clip(slope, 0, 1) ** 1.4
        dark = np.clip(-slope, 0, 1)
        # Crests catch the sky; the steepest faces flash like glints.
        alpha = np.maximum(bright * 110, dark * 80)
        glint = (slope > 0.95) & (height > 0)
        alpha[glint] = 230
        rgba = np.zeros((TILE, TILE, 4), np.float32)
        lit = slope > 0
        rgba[lit, :3] = (215, 238, 255)
        rgba[~lit, :3] = (0, 12, 26)
        rgba[glint, :3] = 255
        rgba[..., 3] = alpha
        rgba[..., :3] *= alpha[..., None] / 255.0    # premultiplied
        data = np.ascontiguousarray(rgba.astype(np.uint8))
        image = QtGui.QImage(data.data, TILE, TILE, 4 * TILE, QtGui.QImage.Format_RGBA8888_Premultiplied)
        frames.append(image.copy())
    return frames


def _image(rgba: np.ndarray, fmt: QtGui.QImage.Format) -> QtGui.QImage:
    data = np.ascontiguousarray(rgba)
    height, width = data.shape[:2]
    return QtGui.QImage(data.data, width, height, 4 * width, fmt).copy()


class WaterSurface:
    """Paints natural water for a :class:`~.depth.ChartLayer`'s grid."""

    def __init__(self) -> None:
        self._frames: list[QtGui.QImage] | None = None
        self._images: tuple | None = None
        self._layer: QtGui.QImage | None = None
        self._layer_key: tuple | None = None
        self._frame = 0
        self.started = time.monotonic()

    # -- the parts ----------------------------------------------------------

    @property
    def frames(self) -> list[QtGui.QImage]:
        if self._frames is None:
            self._frames = _wave_frames()
        return self._frames

    def images(self, chart) -> tuple[QtGui.QImage, QtGui.QImage]:
        """Water colour (opaque, land in the land colour) and land alone (water
        clear) for ``chart``'s grid; rebuilt when its alarm changes."""
        if self._images is None or self._images[0] is not chart.grid \
                or self._images[1] != chart.shallow_alarm_m:
            depth = chart.grid.depth
            water = np.isfinite(depth)
            d = np.nan_to_num(depth, nan=0.0)
            rgb = water_rgb(d)
            hazard = water & (d < chart.shallow_alarm_m)
            rgb[hazard] = rgb[hazard] * 0.7 + np.array((200, 120, 80), np.float32) * 0.3
            land = QtGui.QColor(theme.LAND)
            land_rgb = (land.red(), land.green(), land.blue())
            rgb[~water] = land_rgb
            colour = np.full(depth.shape + (4,), 255, np.uint8)
            colour[..., :3] = np.clip(rgb, 0, 255).astype(np.uint8)
            ground = np.zeros(depth.shape + (4,), np.uint8)   # premultiplied: clear is all zero
            ground[~water] = land_rgb + (255,)
            self._images = (chart.grid, chart.shallow_alarm_m,
                            _image(colour, QtGui.QImage.Format_RGBA8888),
                            _image(ground, QtGui.QImage.Format_RGBA8888_Premultiplied))
        return self._images[2], self._images[3]

    @staticmethod
    def strength(wind_ms: float, overhead: bool = False) -> float:
        """How visible the waves are: a flat calm still shows a little texture.
        Looking straight down there is far less glare than at a low angle."""
        level = min(1.0, 0.45 + wind_ms / 9.0)
        return level * 0.38 if overhead else level

    def wave_brush(self, canvas, wind: tuple[float, float] | None,
                   now: float | None = None) -> QtGui.QBrush:
        """The ripple tile as a brush in map space: downwind, ground-fixed, animated."""
        now = time.monotonic() if now is None else now
        speed, wind_from = wind if wind else (2.0, 270.0)
        frame = self._frame = int((now - self.started) / LOOP_S * FRAMES) % FRAMES
        # The tile is this many metres across: a few wavelengths of chop,
        # bigger in more wind, and never smaller on screen than a texture.
        tile_m = 14.0 + 3.0 * min(speed, 12.0)
        tile_px = max(220.0, tile_m / canvas.metres_per_pixel())
        anchor_x, anchor_y = canvas.to_screen(57.0, 24.0)
        brush = QtGui.QBrush(self.frames[frame])
        transform = QtGui.QTransform()
        transform.translate(anchor_x, anchor_y)
        transform.rotate((wind_from + 180.0) - 90.0)   # tile +x runs downwind
        transform.scale(tile_px / TILE, tile_px / TILE)
        brush.setTransform(transform)
        return brush

    # -- painting -----------------------------------------------------------

    def paint_waves(self, painter: QtGui.QPainter, canvas,
                    wind: tuple[float, float] | None, land=None) -> None:
        """The waves over the view, cut away where ``land`` (a chart layer) is.

        Filling a turned, smoothed texture is the dear part, so it is done at
        a quarter of the pixels into a layer and scaled up: the waves are
        soft, and it does not show."""
        device = painter.device()
        w, h = device.width(), device.height()
        size = QtCore.QSize((w + 1) // WAVE_RES, (h + 1) // WAVE_RES)
        if self._layer is None or self._layer.size() != size:
            self._layer = QtGui.QImage(size, QtGui.QImage.Format_ARGB32_Premultiplied)
            self._layer_key = None
        layer = self._layer
        brush = self.wave_brush(canvas, wind)
        # Redrawn only when the frame turns over or the view really moves:
        # the screen repaints faster than the waves animate, and a turn of
        # under half a degree or a shift of a pixel or two does not show on
        # moving water.
        m = QtGui.QTransform(brush.transform()) * painter.transform()
        turn = round(math.degrees(math.atan2(m.m12(), m.m11())) * 2)
        key = (self._frame, turn, round(math.hypot(m.m11(), m.m12()), 3),
               round(m.dx() / (2 * WAVE_RES)), round(m.dy() / (2 * WAVE_RES)),
               land is not None and land.grid)
        if key != self._layer_key:
            self._layer_key = key
            layer.fill(0)
            p = QtGui.QPainter(layer)
            p.scale(1.0 / WAVE_RES, 1.0 / WAVE_RES)
            p.setTransform(painter.transform(), True)
            p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
            cw, ch = canvas._w(), canvas._h()
            p.fillRect(QRectF(-cw, -ch, 3 * cw, 3 * ch), brush)
            if land is not None:
                p.setCompositionMode(QtGui.QPainter.CompositionMode_DestinationOut)
                land.paint(p, canvas, self.images(land)[1])
            p.end()
        painter.save()
        painter.resetTransform()
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.setOpacity(self.strength(wind[0] if wind else 2.0, canvas.view_mode != "chase"))
        painter.drawImage(QRectF(0, 0, size.width() * WAVE_RES, size.height() * WAVE_RES), layer)
        painter.restore()
