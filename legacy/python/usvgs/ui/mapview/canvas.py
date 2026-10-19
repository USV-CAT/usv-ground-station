"""The chart.

Pan, zoom, the boat and its track, home, the route and the survey rectangle,
the oil slick and the vectors acting on the boat.  Four tools decide what a tap
on the water does:

* **pan** — nothing but moving the chart;
* **points** — add a waypoint; drag one to move it; right-click (or select and
  press Delete) to remove it;
* **rectangle** — drag out a survey area, then drag its corners to resize, its
  middle to move it and the round handle to rotate it;
* **go to** — send the boat to the tapped point (Point Nav).

Three views:

* **north up** — a chart;
* **heading up** — the chart turns so the bow points up the screen, the boat
  sits low in the view and there is more water ahead than astern;
* **chase** — a third-person camera behind the boat, the heading-up chart laid
  out as a plane in perspective under a sky, the way a driving game looks.

Two looks for the water: **natural** (the colour of water over that depth,
moving waves, surf on the beach, and in the chase view the sky reflected
towards the horizon and the sun's glitter) and **chart** (depth shading, as a
nautical chart colours it).  Isobaths, hazards and soundings show in both.

Coordinates come in three spaces.  *Map space* is north-up pixels
(:meth:`to_screen`).  *View space* is map space turned to the view's rotation
(:meth:`project`); markers are drawn there so their text stays upright.  In the
chase view, view space is an off-screen image which a perspective transform
then lays onto the screen.

Everything fixed (the water or depth colour, the vector map and the
isobaths) is drawn once into one opaque image a little bigger than the view,
and blitted until the view zooms, or moves past the spare edge.  While a
zoom is in progress the image is only stretched, so a held zoom button or a
pinch never waits on it, and it is redrawn sharp once the zoom stops.
"""

from __future__ import annotations

import collections
import contextlib
import datetime
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal

from ...core import geo
from ...core.geo import LatLon
from ...core.mission import Mission, Rectangle
from .. import theme
from .depth import ChartLayer, SurveyLayer, paint_legend
from .slick import SlickImage
from .tiles import SIMPLIFY_DEG, TiledMap
from .water import WaterSurface

MIN_SCALE = 2_000.0          # px per degree latitude: ~30 km across
#: The chase camera stops further in (~5 km across): its ground plane reaches
#: far past the screen, and beyond this it would be loading half the country
#: to show it as haze on the horizon.
CHASE_MIN_SCALE = 12_000.0
MAX_SCALE = 20_000_000.0     # ~0.5 cm per pixel
ZOOM_STEP = 1.3
HIT_PX = 22                  # generous: fingers, not mouse pointers
TRACK_DRAW_LIMIT = 2500

TOOLS = ("pan", "points", "rect", "goto")
VIEWS = ("north", "heading", "chase")
VIEW_LABELS = {"north": "N↑", "heading": "H↑", "chase": "3D"}

#: Chase camera: horizon height, and how much bigger than the screen the
#: ground plane image is (it is tall, to reach far ahead).
HORIZON = 0.20
CHASE_W, CHASE_H = 1.3, 2.2
#: The ground plane is rendered at this fraction of screen resolution: the
#: perspective magnifies it about twice around the boat, so this costs a
#: quarter of the pixels for much the same picture.
CHASE_RES = 0.5
#: How far ahead of the boat the ground image is centred, as a fraction of its
#: height; this puts the boat at about 60 % of the screen height.
CHASE_AHEAD = 0.17

#: Between position reports the boat is carried on by its motion for at most
#: this long; a correction is blended out over about this time; and the
#: heading-up views turn with this time constant.
PREDICT_MAX_S = 0.4
GLIDE_S = 0.12
TURN_EASE_S = 0.17

#: Spare round the view in the cached map image, as a fraction of its longer side.
STATIC_MARGIN = 0.25
#: A zoom counts as finished once it has been still this long.
ZOOM_SETTLE_MS = 160

WATER_STYLES = ("natural", "chart")

#: Daylight sky over natural water, top to horizon; the horizon colour is also
#: what the water reflects far off.
SKY_TOP = (46, 98, 158)
SKY_HORIZON = (172, 204, 226)

VECTOR_COLOURS = {
    "heading": theme.BOAT,
    "cog": theme.GOOD,
    "thrust": "#ff7a1a",
    "wind": "#7fe3ff",
    "current": "#a48bff",
}


class ChartCanvas(QtWidgets.QWidget):
    mission_edited = pyqtSignal()
    area_edited = pyqtSignal()
    goto_requested = pyqtSignal(float, float)
    cursor_moved = pyqtSignal(float, float)
    selection_changed = pyqtSignal(int)
    view_changed = pyqtSignal()

    def __init__(self, tiles: TiledMap, mission: Mission, lock=None) -> None:
        super().__init__()
        self.tiles = tiles
        self.mission = mission
        self.lock = lock or threading.RLock()
        self.setMouseTracking(True)
        self.setMinimumSize(320, 240)
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        self.setAttribute(Qt.WA_AcceptTouchEvents)
        self.setFocusPolicy(Qt.NoFocus)

        self.centre_lat, self.centre_lon = 57.048, 23.99
        self.scale = 180_000.0
        self.view_mode = "north"
        self.rotation = 0.0
        self._size_override: tuple[int, int] | None = None
        #: Drawing the chase camera's ground image: text is left off it, since
        #: perspective would smear it, and drawn upright on top instead.
        self._ground_pass = False
        self._perspective: QtGui.QTransform | None = None
        self._chase_image: QtGui.QImage | None = None
        #: The last position report, when it came, and how the boat was moving.
        self._fix: tuple[float, float, float] | None = None
        self._fix_time = 0.0
        self._fix_motion: tuple[float | None, float, float] = (None, 0.0, 0.0)
        self._fix_error = (0.0, 0.0, 0.0)
        self._advanced_at = time.monotonic()
        #: Simulated time runs this much faster than the clock.
        self.time_scale = 1.0
        self.show_fps = False
        self._frame_times: collections.deque[float] = collections.deque()
        self._paint_ms = 0.0

        self.chart: ChartLayer | None = None
        self.survey: SurveyLayer | None = None
        self.show_chart = True
        self.show_isobaths = True
        self.show_survey = True
        self.show_osm = True
        self.show_track = True
        self.show_leg_lengths = True
        self.show_vectors = True
        self.show_oil = True
        self.water_style = "natural"
        self.water = WaterSurface()
        self.slick = SlickImage()

        self.boat: tuple[float, float, float] | None = None   # lat, lon, heading
        self.boat_cog: tuple[float, float] | None = None      # cog deg, sog m/s
        self.boat_roll = 0.0
        self.boat_length_m = 1.6
        self.boat_beam_m = 0.9
        self.home: LatLon | None = None
        self.station: LatLon | None = None
        self.target: LatLon | None = None
        self.target_label = ""
        self.active_index = -1
        self.geofence_m: float | None = None
        self.track: list | None = None
        self.follow = True
        #: Vectors at the boat: dict of thrust (fraction), turn (fraction),
        #: wind (speed, from), current (drift, set).
        self.vectors: dict = {}
        #: Oil parcels: (lat, lon, volume litres, beached) arrays.
        self.oil: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
        self.skimmer_deployed = False
        self.skimmer_swath_m = 2.4

        self.tool = "pan"
        self.selected = -1
        self._press: QPointF | None = None
        self._press_view: tuple[float, float] | None = None
        self._moved = False
        self._drag_wp: int | None = None
        self._rect_anchor: LatLon | None = None
        self._rect_drag: str | None = None          # "corner0..3", "move", "rotate"
        self._rect_start: Rectangle | None = None
        self._rect_press_geo: LatLon | None = None
        self.rect_angle_deg = 90.0
        self._last_pinch = 1.0
        self._last_pinch_centre: QPointF | None = None
        self.grabGesture(Qt.PinchGesture)

        self._static: SimpleNamespace | None = None
        self._zooming = False
        self._zoom_settle = QtCore.QTimer(self)
        self._zoom_settle.setSingleShot(True)
        self._zoom_settle.setInterval(ZOOM_SETTLE_MS)
        self._zoom_settle.timeout.connect(self._zoom_finished)

    # ------------------------------------------------------------------
    # Projection
    # ------------------------------------------------------------------

    def _w(self) -> int:
        return self._size_override[0] if self._size_override else self.width()

    def _h(self) -> int:
        return self._size_override[1] if self._size_override else self.height()

    def _lon_scale(self) -> float:
        return max(0.05, math.cos(math.radians(self.centre_lat)))

    def to_screen(self, lat: float, lon: float) -> tuple[float, float]:
        """Map space: north-up pixels."""
        return (
            self._w() / 2.0 + (lon - self.centre_lon) * self.scale * self._lon_scale(),
            self._h() / 2.0 - (lat - self.centre_lat) * self.scale,
        )

    def to_geo(self, x: float, y: float) -> tuple[float, float]:
        return (
            self.centre_lat - (y - self._h() / 2.0) / self.scale,
            self.centre_lon + (x - self._w() / 2.0) / (self.scale * self._lon_scale()),
        )

    def to_screen_array(self, lat: np.ndarray, lon: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """:meth:`to_screen` for many points at once."""
        return (
            self._w() / 2.0 + (lon - self.centre_lon) * (self.scale * self._lon_scale()),
            self._h() / 2.0 - (lat - self.centre_lat) * self.scale,
        )

    def view_transform(self) -> QtGui.QTransform:
        """Map space to view space: a turn about the middle of the view."""
        t = QtGui.QTransform()
        if self.rotation:
            cx, cy = self._w() / 2.0, self._h() / 2.0
            t.translate(cx, cy)
            t.rotate(-self.rotation)
            t.translate(-cx, -cy)
        return t

    def project(self, lat: float, lon: float) -> QPointF:
        """View space, where markers are drawn."""
        return self.view_transform().map(QPointF(*self.to_screen(lat, lon)))

    def unproject(self, point: QPointF) -> tuple[float, float]:
        inverse, _ = self.view_transform().inverted()
        p = inverse.map(point)
        return self.to_geo(p.x(), p.y())

    def event_geo(self, pos: QPointF) -> tuple[float, float] | None:
        """The water under a point on the widget, in any view."""
        if self.view_mode != "chase":
            return self.unproject(pos)
        if self._perspective is None or pos.y() < self.height() * HORIZON + 2:
            return None
        inverse, ok = self._perspective.inverted()
        if not ok:
            return None
        with self._chase_space():
            return self.unproject(inverse.map(pos))

    @contextlib.contextmanager
    def _chase_space(self):
        """Work in the chase camera's ground image: its size and its scale."""
        saved = self.scale
        self._size_override = self._chase_size()
        self.scale = saved * CHASE_RES
        self._ground_pass = True
        try:
            yield
        finally:
            self.scale = saved
            self._size_override = None
            self._ground_pass = False

    def view_bounds(self) -> tuple[float, float, float, float]:
        """South, west, north, east of everything visible, whatever the rotation."""
        corners = [QPointF(0, 0), QPointF(self._w(), 0), QPointF(0, self._h()),
                   QPointF(self._w(), self._h())]
        points = [self.unproject(c) for c in corners]
        lats = [p[0] for p in points]
        lons = [p[1] for p in points]
        return min(lats), min(lons), max(lats), max(lons)

    def metres_per_pixel(self) -> float:
        return geo.metres_per_degree(self.centre_lat)[0] / self.scale

    def _direction(self, origin: tuple[float, float], bearing: float) -> QPointF:
        """A unit vector in view space for a compass bearing at ``origin``."""
        a = self.project(*origin)
        b = self.project(*geo.destination(origin, bearing, 10.0))
        d = b - a
        length = math.hypot(d.x(), d.y()) or 1.0
        return QPointF(d.x() / length, d.y() / length)

    # ------------------------------------------------------------------
    # View
    # ------------------------------------------------------------------

    def set_view_mode(self, mode: str) -> None:
        self.view_mode = mode if mode in VIEWS else "north"
        if self.view_mode != "north":
            self.follow = True
        else:
            self.rotation = 0.0
        if self.view_mode == "chase" and self.scale < CHASE_MIN_SCALE:
            self.scale = CHASE_MIN_SCALE
        self._update_follow()
        self.view_changed.emit()
        self.update()

    def cycle_view(self) -> str:
        self.set_view_mode(VIEWS[(VIEWS.index(self.view_mode) + 1) % len(VIEWS)])
        return self.view_mode

    def set_scale(self, scale: float, focus: QtCore.QPointF | None = None) -> None:
        lowest = CHASE_MIN_SCALE if self.view_mode == "chase" else MIN_SCALE
        scale = max(lowest, min(MAX_SCALE, scale))
        if focus is not None and not self.follow and self.view_mode != "chase":
            before = self.unproject(focus)
            self.scale = scale
            after = self.unproject(focus)
            self.centre_lat += before[0] - after[0]
            self.centre_lon += before[1] - after[1]
        else:
            self.scale = scale
            self._update_follow()
        self._zooming = True
        self._zoom_settle.start()
        self.view_changed.emit()
        self.update()

    def _zoom_finished(self) -> None:
        self._zooming = False
        self.update()

    def zoom(self, steps: float, focus: QtCore.QPointF | None = None) -> None:
        self.set_scale(self.scale * ZOOM_STEP**steps, focus)

    def zoom_to_metres(self, metres: float) -> None:
        """Show ``metres`` of water across the shorter side of the view."""
        lat_m = geo.metres_per_degree(self.centre_lat)[0]
        self.set_scale(min(self.width(), self.height()) / (metres / lat_m))

    @property
    def view_width_m(self) -> float:
        return min(self.width(), self.height()) * self.metres_per_pixel()

    def centre_on(self, lat: float, lon: float) -> None:
        self.centre_lat, self.centre_lon = lat, lon
        self.update()

    def pan_pixels(self, dx: float, dy: float) -> None:
        if self.view_mode == "chase":
            return
        self.follow = False
        inverse, _ = self.view_transform().inverted()
        d = inverse.map(QPointF(dx, dy)) - inverse.map(QPointF(0, 0))
        self.centre_lat += d.y() / self.scale
        self.centre_lon -= d.x() / (self.scale * self._lon_scale())
        self.update()

    def frame(self, points: list[LatLon], margin_m: float = 40.0) -> None:
        if not points:
            return
        if self.view_mode != "north":
            self.set_view_mode("north")
        box = geo.BBox.around(points, margin_m)
        self.follow = False
        self.centre_lat, self.centre_lon = box.centre.lat, box.centre.lon
        span_lat = max(1e-6, box.max_lat - box.min_lat)
        span_lon = max(1e-6, box.max_lon - box.min_lon) * self._lon_scale()
        self.set_scale(min(self.height() / span_lat, self.width() / span_lon) * 0.9)

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def set_boat(self, lat: float, lon: float, heading: float, cog: float | None,
                 sog: float | None, roll: float = 0.0, rot: float | None = None) -> None:
        """A position report.  Between reports the boat is drawn where its
        speed, course and rate of turn say it has got to (:meth:`advance`)."""
        now = time.monotonic()
        fix = (lat, lon, heading)
        if fix != self._fix:
            shown = self.boat
            self._fix, self._fix_time = fix, now
            self._fix_motion = (cog, sog or 0.0, rot or 0.0)
            if shown is None:
                self._fix_error = (0.0, 0.0, 0.0)
            else:
                # Glide from where the boat was drawn rather than jump.
                lat1, lon1, hdg1 = self._predict(now)
                self._fix_error = (shown[0] - lat1, shown[1] - lon1,
                                   geo.angle_difference(shown[2], hdg1))
        self.boat_cog = (cog, sog or 0.0) if cog is not None else None
        self.boat_roll = roll
        self.advance(now)

    def _predict(self, now: float) -> tuple[float, float, float]:
        lat, lon, heading = self._fix
        cog, sog, rot = self._fix_motion
        dt = min(now - self._fix_time, PREDICT_MAX_S) * self.time_scale
        if cog is not None and sog > 0.02:
            m_lat, m_lon = geo.metres_per_degree(lat)
            run = sog * dt
            lat += run * math.cos(math.radians(cog)) / m_lat
            lon += run * math.sin(math.radians(cog)) / m_lon
        return lat, lon, (heading + rot * dt) % 360.0

    def advance(self, now: float | None = None) -> None:
        """Move the drawn boat and the camera on to ``now``; call every frame."""
        if self._fix is None:
            return
        now = time.monotonic() if now is None else now
        lat, lon, heading = self._predict(now)
        fade = math.exp(-(now - self._fix_time) / GLIDE_S)
        e_lat, e_lon, e_hdg = self._fix_error
        self.boat = (lat + e_lat * fade, lon + e_lon * fade, (heading + e_hdg * fade) % 360.0)
        step = min(0.25, max(0.0, now - self._advanced_at))
        self._advanced_at = now
        if self.view_mode != "north":
            # Ease the turn rather than snapping, so the picture does not
            # shake with compass noise; the same at any frame rate.
            k = 1.0 - math.exp(-step / TURN_EASE_S)
            self.rotation += geo.angle_difference(self.boat[2], self.rotation) * k
            self.rotation %= 360.0
        self._update_follow()

    def frame_stats(self) -> tuple[float, float]:
        """Frames drawn in the last second, and the mean time to draw one (ms)."""
        now = time.monotonic()
        while self._frame_times and now - self._frame_times[0] > 1.0:
            self._frame_times.popleft()
        return float(len(self._frame_times)), self._paint_ms

    def _update_follow(self) -> None:
        if not (self.follow and self.boat):
            return
        lat, lon, _ = self.boat
        if self.view_mode == "north":
            self.centre_lat, self.centre_lon = lat, lon
            return
        # Look ahead: the boat sits low in the view, the water ahead fills it.
        ahead_px = self.height() * 0.25 if self.view_mode == "heading" else self._chase_size()[1] * CHASE_AHEAD
        ahead = geo.destination((lat, lon), self.rotation, ahead_px * self.metres_per_pixel())
        self.centre_lat, self.centre_lon = ahead.lat, ahead.lon

    def set_tool(self, tool: str) -> None:
        self.tool = tool if tool in TOOLS else "pan"
        self.setCursor(Qt.CrossCursor if self.tool in ("points", "rect", "goto") else Qt.ArrowCursor)
        self.update()

    def set_mission(self, mission: Mission) -> None:
        self.mission = mission
        self.selected = -1
        self.update()

    # ------------------------------------------------------------------
    # Painting
    # ------------------------------------------------------------------

    def _chase_size(self) -> tuple[int, int]:
        return int(self.width() * CHASE_W * CHASE_RES), int(self.height() * CHASE_H * CHASE_RES)

    def paintEvent(self, event) -> None:
        started = time.perf_counter()
        painter = QtGui.QPainter(self)
        if self.view_mode == "chase":
            self._paint_chase(painter)
        else:
            self._paint_scene(painter, overlays=True)
            self._paint_scale(painter)
            self._paint_legends(painter)
        if self.show_fps:
            self._paint_fps(painter)
        painter.end()
        self._frame_times.append(time.monotonic())
        self._paint_ms += ((time.perf_counter() - started) * 1000.0 - self._paint_ms) * 0.1

    def _paint_fps(self, painter: QtGui.QPainter) -> None:
        fps, ms = self.frame_stats()
        text = f"{fps:.0f} fps · {ms:.1f} ms"
        painter.save()
        painter.setFont(QtGui.QFont("DejaVu Sans Mono", 9, QtGui.QFont.Bold))
        box = QRectF(self.width() - 150, self.height() - 26, 142, 18)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor(6, 12, 18, 190))
        painter.drawRoundedRect(box, 4, 4)
        painter.setPen(theme.qcolor(theme.GOOD if fps >= 55 else (theme.CAUTION if fps >= 28 else theme.BAD)))
        painter.drawText(box, Qt.AlignCenter, text)
        painter.restore()

    def _paint_scene(self, painter: QtGui.QPainter, overlays: bool) -> None:
        w, h = self._w(), self._h()
        painter.fillRect(QRectF(0, 0, w, h), theme.qcolor(theme.LAND))
        painter.save()
        painter.setTransform(self.view_transform(), True)
        self._paint_static(painter)
        if self.natural_water:
            chart = self.chart if self.chart is not None and self.show_chart else None
            self.water.paint_waves(painter, self, self.vectors.get("wind"),
                                   chart if chart is not None and chart.land_in_view(self) else None)
        if self.survey is not None and self.show_survey:
            self.survey.paint(painter, self)
        if self.chart is not None and self.show_isobaths and self.show_chart \
                and self.view_mode == "north":
            self.chart.paint_spot_depths(painter, self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        if self.show_oil:
            self._paint_oil(painter)
        self._paint_geofence(painter)
        if self.show_track:
            self._paint_track(painter)
        painter.restore()

        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        with self.lock:
            self._paint_area(painter)
            self._paint_route(painter)
        self._paint_home(painter)
        self._paint_target(painter)
        if self.show_vectors:
            self._paint_vectors(painter)
        self._paint_boat(painter, hull=overlays)

    def _paint_chase(self, painter: QtGui.QPainter) -> None:
        w, h = self.width(), self.height()
        size = self._chase_size()
        # One ground image, kept: allocating 3 MB a frame is not free.
        if self._chase_image is None or self._chase_image.size() != QtCore.QSize(*size):
            self._chase_image = QtGui.QImage(size[0], size[1], QtGui.QImage.Format_RGB32)
        image = self._chase_image
        with self._chase_space():
            self._update_follow()
            scene = QtGui.QPainter(image)
            self._paint_scene(scene, overlays=False)
            scene.end()
            boat_view = self.project(self.boat[0], self.boat[1]) if self.boat else None
            tags = []
            if self.target is not None:
                tags.append((self.project(*self.target), self.target_label or "TARGET", theme.TARGET))
            if self.home is not None:
                tags.append((self.project(*self.home), "HOME", theme.HOME))

        horizon = h * HORIZON
        source = QtGui.QPolygonF([QPointF(0, 0), QPointF(size[0], 0),
                                  QPointF(size[0], size[1]), QPointF(0, size[1])])
        target = QtGui.QPolygonF([
            QPointF(w * 0.5 - w * 0.7, horizon), QPointF(w * 0.5 + w * 0.7, horizon),
            QPointF(w * 0.5 + w * 2.2, h * 1.25), QPointF(w * 0.5 - w * 2.2, h * 1.25),
        ])
        transform = QtGui.QTransform()
        if not QtGui.QTransform.quadToQuad(source, target, transform):
            self._paint_scene(painter, overlays=True)
            return
        self._perspective = transform

        natural = self.natural_water
        far = QtGui.QColor(*SKY_HORIZON) if natural else QtGui.QColor(70, 110, 150)
        sky = QtGui.QLinearGradient(0, 0, 0, horizon)
        sky.setColorAt(0.0, QtGui.QColor(*SKY_TOP) if natural else QtGui.QColor(14, 30, 58))
        sky.setColorAt(1.0, far)
        painter.fillRect(QRectF(0, 0, w, horizon + 1), sky)
        painter.save()
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.setClipRect(QRectF(0, horizon, w, h - horizon))
        painter.setTransform(transform)
        painter.drawImage(QPointF(0, 0), image)
        painter.restore()
        # Haze towards the horizon, which is what sells the distance.  Over
        # natural water it reaches further: far off, at a glancing angle,
        # water mirrors the sky.
        reach = 0.38 if natural else 0.22
        haze = QtGui.QLinearGradient(0, horizon, 0, horizon + h * reach)
        haze.setColorAt(0.0, QtGui.QColor(far.red(), far.green(), far.blue(), 235))
        haze.setColorAt(0.35, QtGui.QColor(far.red(), far.green(), far.blue(), 110))
        haze.setColorAt(1.0, QtGui.QColor(far.red(), far.green(), far.blue(), 0))
        painter.fillRect(QRectF(0, horizon, w, h * reach), haze)
        if natural:
            self._paint_sun(painter, w, h, horizon)

        # Upright tags over the places that matter, like markers in a game.
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setFont(QtGui.QFont("DejaVu Sans", 9, QtGui.QFont.Bold))
        for where, text, colour in tags:
            at = transform.map(where)
            if not (horizon < at.y() < h and -40 < at.x() < w + 40):
                continue
            metrics = QtGui.QFontMetrics(painter.font())
            box = QRectF(at.x() - metrics.horizontalAdvance(text) / 2 - 7, at.y() - 40,
                         metrics.horizontalAdvance(text) + 14, 20)
            painter.setPen(QtGui.QPen(theme.qcolor(colour), 1.5))
            painter.setBrush(QtGui.QColor(6, 12, 18, 210))
            painter.drawRoundedRect(box, 5, 5)
            painter.drawLine(QPointF(at.x(), box.bottom()), at)
            painter.drawText(box, Qt.AlignCenter, text)
        if boat_view is not None:
            self._paint_boat_3d(painter, transform.map(boat_view), h)
        painter.setPen(QtGui.QColor(theme.TEXT))
        painter.setFont(QtGui.QFont("DejaVu Sans", 9, QtGui.QFont.Bold))
        painter.drawText(QRectF(w - 210, horizon - 22, 200, 18), Qt.AlignRight,
                         f"CHASE CAM · {self.view_width_m:.0f} m")

    @staticmethod
    def sun_azimuth(now: datetime.datetime | None = None) -> float:
        """Roughly where the sun is from the local clock: east in the morning,
        south at about one o'clock, west in the evening."""
        now = now or datetime.datetime.now()
        hour = min(20.0, max(6.0, now.hour + now.minute / 60.0))
        return (180.0 + 15.0 * (hour - 13.0)) % 360.0

    def _paint_sun(self, painter: QtGui.QPainter, w: float, h: float, horizon: float) -> None:
        """The sun over the horizon when the camera faces it, and its glitter
        on the water: a path of sparkles widening towards the camera."""
        off = (self.sun_azimuth() - self.rotation + 540.0) % 360.0 - 180.0
        if abs(off) > 60.0:
            return
        sx = w / 2.0 + off / 45.0 * w * 0.5
        sy = horizon - h * 0.075
        painter.save()
        painter.setPen(Qt.NoPen)
        glow = QtGui.QRadialGradient(QPointF(sx, sy), h * 0.16)
        glow.setColorAt(0.0, QtGui.QColor(255, 250, 225, 255))
        glow.setColorAt(0.08, QtGui.QColor(255, 244, 210, 240))
        glow.setColorAt(0.25, QtGui.QColor(255, 230, 180, 90))
        glow.setColorAt(1.0, QtGui.QColor(255, 220, 170, 0))
        painter.setBrush(glow)
        painter.drawEllipse(QPointF(sx, sy), h * 0.16, h * 0.16)
        painter.setClipRect(QRectF(0, horizon, w, h - horizon))
        # The bright band under the sun.
        band = QtGui.QRadialGradient(QPointF(sx, horizon + 4), w * 0.12)
        band.setColorAt(0.0, QtGui.QColor(255, 245, 220, 150))
        band.setColorAt(1.0, QtGui.QColor(255, 245, 220, 0))
        painter.setBrush(band)
        painter.drawEllipse(QPointF(sx, horizon + 4), w * 0.12, h * 0.05)
        # Sparkles, reshuffled eight times a second: flat dashes, bigger
        # nearer the camera, drawn as one batch per brightness.
        rng = np.random.default_rng(int(time.monotonic() * 8))
        n = 160
        u = rng.random(n) ** 1.7
        ys = horizon + 2 + u * h * 0.62
        centre = sx + (w / 2.0 - sx) * u * 0.6
        xs = centre + rng.normal(0.0, 1.0, n) * (8 + u * w * 0.09)
        widths = 1.5 + u * 5.5
        heights = 0.8 + u * 1.6
        bright = rng.uniform(0.4, 1.0, n) * (1.0 - u * 0.5)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
        for low, high, alpha in ((0.0, 0.45, 110), (0.45, 0.7, 170), (0.7, 1.01, 235)):
            pick = (bright >= low) & (bright < high)
            painter.setBrush(QtGui.QColor(255, 250, 235, alpha))
            painter.drawRects([QRectF(x - wd / 2, y - ht / 2, wd, ht) for x, y, wd, ht in
                               zip(xs[pick], ys[pick], widths[pick], heights[pick], strict=True)])
        painter.restore()

    def _paint_boat_3d(self, painter: QtGui.QPainter, at: QPointF, h: float) -> None:
        """The catamaran from astern, scaled with distance, heeling with the roll."""
        # True size at the boat's distance (about twice the chart scale there),
        # but never so small it cannot be seen or so big it hides the water.
        true_px = self.boat_length_m / self.metres_per_pixel() * 2.0
        size = max(80.0, min(170.0, true_px * 0.9))
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.translate(at)
        painter.rotate(self.boat_roll)
        s = size / 100.0
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor(0, 0, 0, 90))
        painter.drawEllipse(QPointF(0, 8 * s), 62 * s, 12 * s)
        # Wake.
        painter.setBrush(QtGui.QColor(230, 245, 255, 70))
        for side in (-1, 1):
            painter.drawPolygon(QtGui.QPolygonF([QPointF(side * 40 * s, 4 * s),
                                                 QPointF(side * 75 * s, 34 * s),
                                                 QPointF(side * 22 * s, 34 * s)]))
        hull = theme.qcolor(theme.BOAT)
        dark = hull.darker(170)
        for side in (-1, 1):
            cx = side * 38 * s
            painter.setBrush(dark)
            painter.drawRoundedRect(QRectF(cx - 13 * s, -10 * s, 26 * s, 18 * s), 6 * s, 6 * s)
            painter.setBrush(hull)
            painter.drawRoundedRect(QRectF(cx - 13 * s, -22 * s, 26 * s, 16 * s), 6 * s, 6 * s)
        painter.setBrush(QtGui.QColor("#d6dde4"))
        painter.drawRect(QRectF(-44 * s, -30 * s, 88 * s, 10 * s))
        painter.setBrush(QtGui.QColor("#2b3846"))
        painter.drawRoundedRect(QRectF(-22 * s, -52 * s, 44 * s, 23 * s), 4 * s, 4 * s)
        painter.setPen(QtGui.QPen(QtGui.QColor("#d6dde4"), max(1.0, 2.5 * s)))
        painter.drawLine(QPointF(0, -52 * s), QPointF(0, -84 * s))
        painter.setPen(Qt.NoPen)
        painter.setBrush(theme.qcolor(theme.GOOD))
        painter.drawEllipse(QPointF(0, -86 * s), 4 * s, 4 * s)
        painter.restore()

    # ------------------------------------------------------------------
    # The cached map image
    # ------------------------------------------------------------------

    @property
    def natural_water(self) -> bool:
        return self.water_style == "natural"

    def set_water_style(self, style: str) -> None:
        self.water_style = style if style in WATER_STYLES else "natural"
        self.update()

    def _static_key(self) -> tuple:
        chart = self.chart if self.chart is not None and self.show_chart else None
        return (chart, chart and chart.shallow_alarm_m, chart and chart.image.cacheKey(),
                self.show_isobaths, self.show_osm, self.tiles.source, len(self.tiles.tiles),
                self._static_extent(), self.natural_water)

    def _static_extent(self) -> tuple[int, int]:
        """Half the size of the cached image: the view, any way it turns, plus spare."""
        w, h = self._w(), self._h()
        margin = STATIC_MARGIN * max(w, h)
        if self.view_mode == "north":
            return int(w / 2 + margin), int(h / 2 + margin)
        half = math.hypot(w, h) / 2 + margin
        return int(half), int(half)

    def _paint_static(self, painter: QtGui.QPainter) -> None:
        """Everything that does not move: the water or depth colour, the vector
        map and the isobaths, from the cached image if it will do."""
        key = self._static_key()
        cached = self._static
        if cached is not None and cached.key == key:
            ratio = self.scale / cached.scale
            lon_px = self.scale * self._lon_scale()
            min_lat, min_lon, max_lat, max_lon = self.view_bounds()
            south, west, north, east = cached.bounds
            covers = south <= min_lat and west <= min_lon and max_lat <= north and max_lon <= east
            sharp = abs(ratio - 1.0) < 1e-9 and abs(lon_px / cached.lon_px - 1.0) < 1e-3
            if sharp and covers:
                x, y = self.to_screen(cached.lat0, cached.lon0)
                painter.drawImage(QPointF(round(x), round(y)), cached.image)
                return
            if not sharp and self._zooming:
                # Stretched, however far, and redrawn sharp once the zoom
                # stops.  Zooming out it no longer reaches the edges, so the
                # water is drawn live round it meanwhile.
                if not covers:
                    self._paint_base(painter)
                x, y = self.to_screen(cached.lat0, cached.lon0)
                size = cached.image.size()
                painter.save()
                painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
                painter.drawImage(QRectF(x, y, size.width() * lon_px / cached.lon_px,
                                         size.height() * ratio), cached.image)
                painter.restore()
                return
        self._static = self._render_static(key)
        x, y = self.to_screen(self._static.lat0, self._static.lon0)
        painter.drawImage(QPointF(round(x), round(y)), self._static.image)

    def _paint_base(self, painter: QtGui.QPainter) -> None:
        """The water: natural colour or depth shading, or open water with no chart."""
        w, h = self._w(), self._h()
        if self.chart is not None and self.show_chart:
            image = self.water.images(self.chart)[0] if self.natural_water else None
            self.chart.paint(painter, self, image)
        else:
            colour = (16, 74, 110) if self.natural_water else theme.WATER_DEEP
            painter.fillRect(QRectF(-w, -h, 3 * w, 3 * h), QtGui.QColor(*colour))

    def _render_static(self, key: tuple) -> SimpleNamespace:
        ew, eh = self._static_extent()
        image = QtGui.QImage(2 * ew, 2 * eh, QtGui.QImage.Format_RGB32)
        image.fill(theme.qcolor(theme.LAND))
        w, h = self._w(), self._h()
        lat0, lon0 = self.to_geo(w / 2.0 - ew, h / 2.0 - eh)
        # Draw as if the view were the image: same centre and scale, no turn.
        saved = self._size_override, self.rotation
        self._size_override, self.rotation = (2 * ew, 2 * eh), 0.0
        try:
            bounds = self.view_bounds()
            painter = QtGui.QPainter(image)
            self._paint_base(painter)
            if self.show_osm:
                self._paint_osm(painter)
            chart = key[0]
            if chart is not None and self.show_isobaths:
                chart.paint_isobaths(painter, self, natural=self.natural_water)
            painter.end()
        finally:
            self._size_override, self.rotation = saved
        return SimpleNamespace(image=image, key=key, lat0=lat0, lon0=lon0, bounds=bounds,
                               scale=self.scale, lon_px=self.scale * self._lon_scale())

    def _layers_for_scale(self) -> list[str]:
        wanted = {"water", "road_major"}
        if self.scale > 40_000:
            wanted |= {"landuse", "rail"}
        if self.scale > 150_000:
            wanted.add("road_minor")
        if self.scale > 900_000:
            wanted.add("building")
        return [layer for layer in theme.MAP_LAYER_ORDER if layer in wanted]

    def _paint_osm(self, painter: QtGui.QPainter) -> None:
        if not self.tiles.is_loaded:
            return
        min_lat, min_lon, max_lat, max_lon = self.view_bounds()
        layers = self._layers_for_scale()
        # Lines under a pixel across would not show, nor would detail finer
        # than a pixel in the rest.
        tolerance = max((t for t in SIMPLIFY_DEG if t * self.scale <= 1.0), default=0.0)
        data = self.tiles.visible(min_lat, min_lon, max_lat, max_lon, layers,
                                  min_extent_deg=1.0 / self.scale, tolerance_deg=tolerance)
        if data is None:
            return
        transform = QtGui.QTransform()
        transform.translate(self._w() / 2.0, self._h() / 2.0)
        transform.scale(self.scale * self._lon_scale(), -self.scale)
        transform.translate(-self.centre_lon, -self.centre_lat)
        painter.save()
        painter.setTransform(transform, True)
        for layer in layers:
            r, g, b, a, width = theme.MAP_LAYERS[layer]
            pen = QtGui.QPen(QtGui.QColor(r, g, b, a))
            pen.setCosmetic(True)
            pen.setWidth(width)
            painter.setPen(pen)
            for polyline in data[layer]:
                painter.drawPolyline(polyline)
        painter.restore()

    def _paint_oil(self, painter: QtGui.QPainter) -> None:
        """The slick, coloured by film thickness; see :mod:`.slick`."""
        self.slick.paint(painter, self, self.oil)

    def _paint_geofence(self, painter: QtGui.QPainter) -> None:
        if not (self.home and self.geofence_m):
            return
        x, y = self.to_screen(*self.home)
        radius = self.geofence_m / self.metres_per_pixel()
        if not 10 < radius < 40000:
            return
        pen = QtGui.QPen(theme.qcolor(theme.CAUTION, 160), 1.5, Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(x, y), radius, radius)

    def _paint_track(self, painter: QtGui.QPainter) -> None:
        track = self.track
        if not track or len(track) < 2:
            return
        step = max(1, len(track) // TRACK_DRAW_LIMIT)
        points = [QPointF(*self.to_screen(p[1], p[2])) for p in track[::step]]
        points.append(QPointF(*self.to_screen(track[-1][1], track[-1][2])))
        pen = QtGui.QPen(theme.qcolor(theme.TRACK, 210), 2.0)
        painter.setPen(pen)
        painter.drawPolyline(QtGui.QPolygonF(points))

    def _paint_area(self, painter: QtGui.QPainter) -> None:
        area = self.mission.area
        if area is None:
            return
        corners = [self.project(c.lat, c.lon) for c in area.corners()]
        painter.setPen(QtGui.QPen(theme.qcolor(theme.AREA, 220), 2.0, Qt.DashLine))
        painter.setBrush(theme.qcolor(theme.AREA, 28))
        painter.drawPolygon(QtGui.QPolygonF(corners))
        if self.tool != "rect" or self.view_mode == "chase":
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("#08130b"), 1.5))
        painter.setBrush(theme.qcolor(theme.AREA))
        for corner in corners:
            painter.drawRect(QRectF(corner.x() - 8, corner.y() - 8, 16, 16))
        centre = self.project(area.centre.lat, area.centre.lon)
        painter.drawEllipse(centre, 9, 9)
        handle = self._rotate_handle(area)
        top_mid = (corners[2] + corners[3]) / 2
        painter.setPen(QtGui.QPen(theme.qcolor(theme.AREA), 1.5))
        painter.drawLine(top_mid, handle)
        painter.setBrush(theme.qcolor(theme.VALUE))
        painter.drawEllipse(handle, 10, 10)
        painter.setPen(QtGui.QColor(theme.TEXT))
        label = f"{area.width_m:.0f} × {area.height_m:.0f} m · {area.angle_deg:.0f}°"
        painter.drawText(QPointF(centre.x() + 14, centre.y() - 12), label)

    def _rotate_handle(self, area: Rectangle) -> QPointF:
        corners = area.corners()
        mid = LatLon((corners[2].lat + corners[3].lat) / 2, (corners[2].lon + corners[3].lon) / 2)
        # The handle sits a fixed number of pixels beyond the far edge.
        m = self.project(mid.lat, mid.lon)
        c = self.project(area.centre.lat, area.centre.lon)
        d = m - c
        length = math.hypot(d.x(), d.y()) or 1.0
        return QPointF(m.x() + d.x() / length * 36, m.y() + d.y() / length * 36)

    def _paint_route(self, painter: QtGui.QPainter) -> None:
        waypoints = self.mission.waypoints
        if not waypoints:
            return
        points = [self.project(w.lat, w.lon) for w in waypoints]
        font = painter.font()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)

        for i in range(len(points) - 1):
            done = self.active_index >= 0 and i + 1 < self.active_index
            active = i + 1 == self.active_index
            colour = theme.ROUTE_ACTIVE if active else (theme.WAYPOINT_DONE if done else theme.ROUTE)
            painter.setPen(QtGui.QPen(theme.qcolor(colour, 230), 3.0 if active else 1.6))
            painter.drawLine(points[i], points[i + 1])
            self._paint_arrow(painter, points[i], points[i + 1], colour)
            if self.show_leg_lengths and not self._ground_pass:
                length = geo.distance_m(waypoints[i].position, waypoints[i + 1].position)
                if QtCore.QLineF(points[i], points[i + 1]).length() > 70:
                    self._paint_leg_label(painter, points[i], points[i + 1], f"{length:.0f}m")
        if self.mission.loop and len(points) > 2:
            painter.setPen(QtGui.QPen(theme.qcolor(theme.ROUTE, 120), 1.2, Qt.DashLine))
            painter.drawLine(points[-1], points[0])
        if self.home and self.active_index < 0:
            painter.setPen(QtGui.QPen(theme.qcolor(theme.ROUTE, 90), 1.0, Qt.DotLine))
            painter.drawLine(self.project(*self.home), points[0])

        for index, point in enumerate(points):
            selected = index == self.selected
            active = index == self.active_index
            done = self.active_index >= 0 and index < self.active_index
            radius = 12 if selected or active else 10
            fill = theme.ROUTE_ACTIVE if active else (theme.WAYPOINT_DONE if done else theme.WAYPOINT)
            painter.setBrush(theme.qcolor(fill))
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff") if selected else QtGui.QColor("#061018"),
                                      2.5 if selected else 1.2))
            painter.drawEllipse(point, radius, radius)
            if not self._ground_pass:
                painter.setPen(QtGui.QColor("#061018"))
                painter.drawText(QRectF(point.x() - radius, point.y() - radius, 2 * radius, 2 * radius),
                                 Qt.AlignCenter, str(index + 1))

    def _paint_arrow(self, painter, a: QPointF, b: QPointF, colour: str, at: float = 0.5,
                     size: float = 7.0, min_length: float = 60.0) -> None:
        line = QtCore.QLineF(a, b)
        if line.length() < min_length:
            return
        mid = line.pointAt(at)
        angle = math.radians(-line.angle())
        tip = QPointF(mid.x() + math.cos(angle) * size, mid.y() + math.sin(angle) * size)
        left = QPointF(mid.x() + math.cos(angle + 2.5) * size, mid.y() + math.sin(angle + 2.5) * size)
        right = QPointF(mid.x() + math.cos(angle - 2.5) * size, mid.y() + math.sin(angle - 2.5) * size)
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(theme.qcolor(colour))
        painter.drawPolygon(QtGui.QPolygonF([tip, left, right]))
        painter.restore()

    def _paint_leg_label(self, painter, a: QPointF, b: QPointF, text: str) -> None:
        line = QtCore.QLineF(a, b)
        mid = line.pointAt(0.32)
        angle = -line.angle()
        if angle < -90 or angle > 90:
            angle += 180
        painter.save()
        painter.translate(mid)
        painter.rotate(angle)
        painter.setPen(QtGui.QColor(theme.TEXT))
        painter.drawText(QRectF(-30, -17, 60, 14), Qt.AlignCenter, text)
        painter.restore()

    def _paint_home(self, painter: QtGui.QPainter) -> None:
        for place, label, colour in ((self.station, "GCS", theme.ACCENT),
                                     (self.home, "H", theme.HOME)):
            if place is None:
                continue
            p = self.project(*place)
            x, y = p.x(), p.y()
            painter.setPen(QtGui.QPen(theme.qcolor(colour), 2))
            painter.setBrush(QtGui.QColor(6, 16, 24, 210))
            size = 11 if label == "H" else 8
            painter.drawRect(QRectF(x - size, y - size, 2 * size, 2 * size))
            font = painter.font()
            font.setPointSize(8 if label == "H" else 6)
            font.setBold(True)
            painter.setFont(font)
            if not self._ground_pass:
                painter.drawText(QRectF(x - size, y - size, 2 * size, 2 * size), Qt.AlignCenter, label)

    def _paint_target(self, painter: QtGui.QPainter) -> None:
        if self.target is None:
            return
        p = self.project(*self.target)
        painter.setPen(QtGui.QPen(theme.qcolor(theme.TARGET), 2))
        painter.setBrush(Qt.NoBrush)
        for radius in (7, 14):
            painter.drawEllipse(p, radius, radius)
        if self.boat:
            painter.setPen(QtGui.QPen(theme.qcolor(theme.TARGET, 150), 1.2, Qt.DashLine))
            painter.drawLine(self.project(self.boat[0], self.boat[1]), p)
        if self.target_label and not self._ground_pass:
            painter.setPen(theme.qcolor(theme.TARGET))
            painter.drawText(QPointF(p.x() + 16, p.y() + 4), self.target_label)

    # -- vectors ------------------------------------------------------------

    def _vector(self, painter, start: QPointF, direction: QPointF, length: float, colour: str,
                label: str = "", width: float = 3.0, dashed: bool = False) -> QPointF:
        end = QPointF(start.x() + direction.x() * length, start.y() + direction.y() * length)
        pen = QtGui.QPen(theme.qcolor(colour), width, Qt.DashLine if dashed else Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(start, end)
        if length > 6:
            self._paint_arrow(painter, start, end, colour, at=1.0, size=9.0, min_length=0)
        if label and not self._ground_pass:
            painter.setPen(theme.qcolor(colour))
            painter.setFont(QtGui.QFont("DejaVu Sans", 8, QtGui.QFont.Bold))
            painter.drawText(QPointF(end.x() + 6, end.y() + 4), label)
        return end

    def _paint_vectors(self, painter: QtGui.QPainter) -> None:
        """Thrust, true wind and current at the boat, plus heading and course."""
        if not self.boat:
            return
        lat, lon, heading = self.boat
        here = self.project(lat, lon)
        v = self.vectors

        thrust = v.get("thrust")
        if thrust is not None and abs(thrust) > 0.02:
            direction = self._direction((lat, lon), heading)
            self._vector(painter, here, direction, 90.0 * thrust, VECTOR_COLOURS["thrust"],
                         f"T {abs(thrust) * 100:.0f}%", width=5.0)
        turn = v.get("turn")
        if turn is not None and abs(turn) > 0.03:
            # Turning moment as an arc round the boat, the way it would turn.
            radius = 34.0
            bow = self._direction((lat, lon), heading)
            # Qt measures arcs anticlockwise from three o'clock; the arc starts
            # at the bow and sweeps the way the boat is being turned.
            bow_angle = math.degrees(math.atan2(-bow.y(), bow.x()))
            painter.setPen(QtGui.QPen(theme.qcolor(VECTOR_COLOURS["thrust"], 200), 3, cap=Qt.RoundCap))
            painter.setBrush(Qt.NoBrush)
            painter.drawArc(QRectF(here.x() - radius, here.y() - radius, 2 * radius, 2 * radius),
                            int(bow_angle * 16), int(-turn * 120.0 * 16))

        wind = v.get("wind")
        if wind is not None and wind[0] > 0.2:
            speed, source = wind
            to_source = self._direction((lat, lon), source)
            length = max(24.0, 14.0 * speed)
            start = QPointF(here.x() + to_source.x() * (length + 40), here.y() + to_source.y() * (length + 40))
            towards = QPointF(-to_source.x(), -to_source.y())
            self._vector(painter, start, towards, length, VECTOR_COLOURS["wind"],
                         f"TW {speed:.1f} m/s", width=3.0)

        current = v.get("current")
        if current is not None and current[0] > 0.02:
            drift, set_deg = current
            direction = self._direction((lat, lon), set_deg)
            self._vector(painter, here, direction, max(20.0, 180.0 * drift), VECTOR_COLOURS["current"],
                         f"Set {set_deg:03.0f}° {drift:.2f}", width=3.0, dashed=True)

    def _paint_legends(self, painter: QtGui.QPainter) -> None:
        if self.survey is not None and self.show_survey and self.survey.grid.soundings:
            low, high = self.survey.depth_range()
            paint_legend(painter, QRectF(self.width() - 34, 150, 12, 120), low, high, "survey m")
        if not self.show_vectors or not self.boat:
            return
        items = (("heading", "Heading"), ("cog", "COG/SOG 30 s"), ("thrust", "Thrust"),
                 ("wind", "True wind"), ("current", "Current"))
        box = QRectF(self.width() - 128, 286, 118, 16 * len(items) + 8)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor(6, 12, 18, 190))
        painter.drawRoundedRect(box, 6, 6)
        painter.setFont(QtGui.QFont("DejaVu Sans", 7))
        for i, (key, text) in enumerate(items):
            y = box.top() + 12 + i * 16
            painter.setPen(QtGui.QPen(theme.qcolor(VECTOR_COLOURS[key]), 3))
            painter.drawLine(QPointF(box.left() + 8, y), QPointF(box.left() + 26, y))
            painter.setPen(theme.qcolor(theme.TEXT))
            painter.drawText(QPointF(box.left() + 32, y + 4), text)

    def _paint_boat(self, painter: QtGui.QPainter, hull: bool = True) -> None:
        if not self.boat:
            return
        lat, lon, heading = self.boat
        here = self.project(lat, lon)
        px_per_m = 1.0 / self.metres_per_pixel()

        # Course-over-ground predictor: where the boat will be in 30 s.
        if self.boat_cog and self.boat_cog[1] > 0.15:
            cog, sog = self.boat_cog
            ahead = self.project(*geo.destination((lat, lon), cog, sog * 30.0))
            painter.setPen(QtGui.QPen(theme.qcolor(theme.GOOD, 220), 2.5))
            painter.drawLine(here, ahead)
            painter.setBrush(theme.qcolor(theme.GOOD))
            painter.drawEllipse(ahead, 3.5, 3.5)
        heading_dir = self._direction((lat, lon), heading)
        reach = max(60.0, 25.0 * px_per_m)
        painter.setPen(QtGui.QPen(theme.qcolor(theme.BOAT, 170), 1.4, Qt.DashLine))
        painter.drawLine(here, QPointF(here.x() + heading_dir.x() * reach, here.y() + heading_dir.y() * reach))

        if not hull:
            return
        screen_heading = math.degrees(math.atan2(heading_dir.x(), -heading_dir.y()))
        length_px = max(30.0, self.boat_length_m * px_per_m)
        beam_px = length_px * (self.boat_beam_m / self.boat_length_m)
        painter.save()
        painter.translate(here)
        painter.rotate(screen_heading)
        if self.skimmer_deployed:
            # Sweep arms: a V from the bows out to the full swath.
            swath_px = max(beam_px * 1.4, self.skimmer_swath_m * px_per_m)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ff5a1f"), 3, cap=Qt.RoundCap))
            for side in (-1, 1):
                painter.drawLine(QPointF(side * beam_px / 2, -length_px / 2),
                                 QPointF(side * swath_px / 2, -length_px * 1.15))
        hull_w = beam_px * 0.28
        painter.setPen(QtGui.QPen(QtGui.QColor("#1b1300"), 1.5))
        painter.setBrush(theme.qcolor(theme.BOAT))
        half_l = length_px / 2.0
        for side in (-1, 1):
            cx = side * beam_px / 2.0
            painter.drawPolygon(QtGui.QPolygonF([
                QPointF(cx, -half_l), QPointF(cx + hull_w / 2, -half_l * 0.55),
                QPointF(cx + hull_w / 2, half_l), QPointF(cx - hull_w / 2, half_l),
                QPointF(cx - hull_w / 2, -half_l * 0.55),
            ]))
        painter.setBrush(theme.qcolor(theme.BOAT, 200))
        painter.drawRect(QRectF(-beam_px / 2, -half_l * 0.25, beam_px, half_l * 0.75))
        painter.restore()

    def _paint_scale(self, painter: QtGui.QPainter) -> None:
        mpp = self.metres_per_pixel()
        rough = mpp * 120
        magnitude = 10 ** math.floor(math.log10(max(1e-6, rough)))
        nice = next((magnitude * s for s in (1, 2, 5, 10) if magnitude * s >= rough), magnitude * 10)
        length = nice / mpp
        x, y = 14, self.height() - 16
        painter.setPen(QtGui.QPen(QtGui.QColor("#e3ecf4"), 2))
        painter.drawLine(QPointF(x, y), QPointF(x + length, y))
        painter.drawLine(QPointF(x, y - 5), QPointF(x, y + 3))
        painter.drawLine(QPointF(x + length, y - 5), QPointF(x + length, y + 3))
        label = f"{nice / 1000:g} km" if nice >= 1000 else f"{nice:g} m"
        painter.drawText(QPointF(x + length + 8, y + 4), label)
        if self.view_mode == "heading":
            # North arrow, since north is no longer up.
            centre = QPointF(self.width() - 30, 30 + 230)
            d = self._direction((self.centre_lat, self.centre_lon), 0.0)
            tip = QPointF(centre.x() + d.x() * 16, centre.y() + d.y() * 16)
            painter.setPen(QtGui.QPen(theme.qcolor(theme.BAD), 3, cap=Qt.RoundCap))
            painter.drawLine(centre, tip)
            painter.setPen(theme.qcolor(theme.TEXT))
            painter.drawText(QRectF(tip.x() - 8, tip.y() - 16, 16, 14), Qt.AlignCenter, "N")

    # ------------------------------------------------------------------
    # Interaction
    # ------------------------------------------------------------------

    def _view_point(self, pos: QPointF) -> QPointF | None:
        """Widget coordinates to view-space coordinates (only differs in chase)."""
        if self.view_mode != "chase":
            return pos
        if self._perspective is None:
            return None
        inverse, ok = self._perspective.inverted()
        return inverse.map(pos) if ok else None

    def _waypoint_at(self, pos: QPointF) -> int:
        if self.view_mode == "chase":
            return -1
        best, best_d = -1, HIT_PX**2
        for i, w in enumerate(self.mission.waypoints):
            p = self.project(w.lat, w.lon)
            d = (p.x() - pos.x()) ** 2 + (p.y() - pos.y()) ** 2
            if d <= best_d:
                best, best_d = i, d
        return best

    def _rect_hit(self, pos: QPointF) -> str | None:
        area = self.mission.area
        if area is None or self.view_mode == "chase":
            return None
        handle = self._rotate_handle(area)
        if (handle - pos).manhattanLength() < HIT_PX * 1.4:
            return "rotate"
        for i, corner in enumerate(area.corners()):
            p = self.project(corner.lat, corner.lon)
            if (p - pos).manhattanLength() < HIT_PX * 1.4:
                return f"corner{i}"
        c = self.project(area.centre.lat, area.centre.lon)
        if (c - pos).manhattanLength() < HIT_PX * 1.4 or area.contains(LatLon(*self.unproject(pos))):
            return "move"
        return None

    def event(self, event) -> bool:
        if event.type() == QtCore.QEvent.Gesture:
            pinch = event.gesture(Qt.PinchGesture)
            if pinch is not None:
                if pinch.state() == Qt.GestureStarted:
                    self._last_pinch = 1.0
                    self._last_pinch_centre = pinch.centerPoint()
                factor = pinch.totalScaleFactor() / max(1e-3, self._last_pinch)
                self._last_pinch = pinch.totalScaleFactor()
                centre = self.mapFromGlobal(pinch.centerPoint().toPoint())
                self.set_scale(self.scale * factor, QPointF(centre))
                # Two fingers moving together pan as well.
                if self._last_pinch_centre is not None and not self.follow:
                    delta = pinch.centerPoint() - self._last_pinch_centre
                    self.pan_pixels(delta.x(), delta.y())
                self._last_pinch_centre = pinch.centerPoint()
                return True
        return super().event(event)

    def wheelEvent(self, event) -> None:
        self.zoom(1 if event.angleDelta().y() > 0 else -1, QPointF(event.pos()))

    def mousePressEvent(self, event) -> None:
        pos = QPointF(event.pos())
        self._press, self._moved = pos, False
        self._press_view = (self.centre_lat, self.centre_lon)
        located = self.event_geo(pos)
        if located is None:
            return
        here = LatLon(*located)

        if event.button() == Qt.RightButton:
            index = self._waypoint_at(pos)
            if index >= 0 and self.tool in ("points", "pan"):
                with self.lock:
                    self.mission.remove(index)
                self.selected = -1
                self.mission_edited.emit()
                self.update()
            return

        if self.tool == "points":
            index = self._waypoint_at(pos)
            if index >= 0:
                self.selected = index
                self._drag_wp = index
                self.selection_changed.emit(index)
                self.update()
        elif self.tool == "rect" and self.view_mode != "chase":
            hit = self._rect_hit(pos)
            if hit:
                self._rect_drag = hit
                self._rect_start = self.mission.area
                self._rect_press_geo = here
            else:
                self._rect_anchor = here
        elif self.tool == "pan":
            index = self._waypoint_at(pos)
            if index >= 0:
                self.selected = index
                self.selection_changed.emit(index)
                self.update()

    def mouseMoveEvent(self, event) -> None:
        pos = QPointF(event.pos())
        located = self.event_geo(pos)
        if located is not None:
            self.cursor_moved.emit(*located)
        if self._press is None:
            return
        if (pos - self._press).manhattanLength() > 6:
            self._moved = True
        if not self._moved or located is None:
            return
        lat, lon = located

        if self._drag_wp is not None:
            with self.lock:
                if self._drag_wp < len(self.mission.waypoints):
                    waypoint = self.mission.waypoints[self._drag_wp]
                    waypoint.lat, waypoint.lon = lat, lon
            self.update()
            return
        if self._rect_anchor is not None:
            area = Rectangle.from_corners(self._rect_anchor, LatLon(lat, lon), self.rect_angle_deg)
            with self.lock:
                self.mission.area = area
            self.area_edited.emit()
            self.update()
            return
        if self._rect_drag is not None and self._rect_start is not None:
            self._drag_rect(LatLon(lat, lon))
            return
        if self.view_mode == "chase":
            return
        # Otherwise: pan, the press point staying under the finger.
        lat0, lon0 = self._press_view
        self.centre_lat, self.centre_lon = lat0, lon0
        self.pan_pixels(pos.x() - self._press.x(), pos.y() - self._press.y())

    def _drag_rect(self, here: LatLon) -> None:
        start = self._rect_start
        kind = self._rect_drag
        if kind == "move":
            offset = geo.to_metres(self._rect_press_geo, here)
            centre = geo.to_lat_lon(start.centre, offset)
            area = Rectangle(centre, start.width_m, start.height_m, start.angle_deg)
        elif kind == "rotate":
            bearing = geo.bearing_deg(start.centre, here)
            # The handle is off the far edge, which is at +90° from the width axis.
            angle = (bearing + 90.0) % 360.0
            area = Rectangle(start.centre, start.width_m, start.height_m, angle)
            self.rect_angle_deg = angle
        else:
            index = int(kind[-1])
            opposite = start.corners()[(index + 2) % 4]
            area = Rectangle.from_corners(opposite, here, start.angle_deg)
            if area.width_m < 4 or area.height_m < 4:
                return
        with self.lock:
            self.mission.area = area
        self.area_edited.emit()
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        pos = QPointF(event.pos())
        tapped = self._press is not None and not self._moved and event.button() == Qt.LeftButton
        dragged_wp = self._drag_wp is not None and self._moved
        self._press = None
        self._drag_wp = None
        if dragged_wp:
            self.mission_edited.emit()
        if self._rect_anchor is not None or self._rect_drag is not None:
            self._rect_anchor = None
            self._rect_drag = None
            self._rect_start = None
            if not tapped:
                self.area_edited.emit()
            return
        if not tapped:
            return
        located = self.event_geo(pos)
        if located is None:
            return
        lat, lon = located
        if self.tool == "points" and self._waypoint_at(pos) < 0:
            with self.lock:
                self.mission.add(lat, lon)
                self.mission.area = None
                self.mission.drift_ms = None
            self.selected = len(self.mission.waypoints) - 1
            self.selection_changed.emit(self.selected)
            self.mission_edited.emit()
            self.update()
        elif self.tool == "goto":
            self.goto_requested.emit(lat, lon)

    def delete_selected(self) -> None:
        if 0 <= self.selected < len(self.mission.waypoints):
            with self.lock:
                self.mission.remove(self.selected)
            self.selected = min(self.selected, len(self.mission.waypoints) - 1)
            self.mission_edited.emit()
            self.update()
