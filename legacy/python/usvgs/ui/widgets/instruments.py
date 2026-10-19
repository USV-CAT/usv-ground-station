"""HUD instruments drawn over the chart.

Each is a small translucent widget that paints itself from a
:class:`~usvgs.vehicle.vehicle.NavState`.  They are the instruments a
helmsman expects on a boat: a heading tape with the course and the
bearing to the next mark on it, a speed log in knots, an echo sounder with its
recent history, the two throttles with the rate of turn, and the wind.
"""

from __future__ import annotations

import math
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QPointF, QRectF, Qt

from ...core import geo, units
from .. import theme

PANEL_BG = QtGui.QColor(6, 12, 18, 205)
PANEL_EDGE = QtGui.QColor(54, 198, 244, 90)


def _font(size: float, bold: bool = False, mono: bool = False) -> QtGui.QFont:
    font = QtGui.QFont("DejaVu Sans Mono" if mono else "DejaVu Sans")
    font.setPointSizeF(size)
    font.setBold(bold)
    return font


class Instrument(QtWidgets.QWidget):
    def __init__(self, parent=None, size=(200, 120)) -> None:
        super().__init__(parent)
        self.setFixedSize(*size)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.nav = None
        self.speed_units = "kn"

    def set_nav(self, nav) -> None:
        # No update() of its own: the instruments float over the chart, which
        # redraws every frame and repaints them with it.  Asking here as well
        # would make Qt redraw the whole chart under each one, again.
        self.nav = nav

    def _frame(self, painter: QtGui.QPainter, title: str = "") -> None:
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setPen(QtGui.QPen(PANEL_EDGE, 1))
        painter.setBrush(PANEL_BG)
        painter.drawRoundedRect(QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5), 9, 9)
        if title:
            painter.setPen(theme.qcolor(theme.ACCENT))
            painter.setFont(_font(7.5, True))
            painter.drawText(QRectF(10, 4, self.width() - 20, 14), Qt.AlignLeft, title)

    def _speed(self, ms: float | None) -> str:
        if ms is None:
            return "—"
        return f"{units.knots(ms):.1f}" if self.speed_units == "kn" else f"{ms:.2f}"

    @property
    def _speed_unit(self) -> str:
        return "kn" if self.speed_units == "kn" else "m/s"


class CompassTape(Instrument):
    """Heading tape, ±60° about the bow."""

    SPAN = 120.0

    def __init__(self, parent=None) -> None:
        super().__init__(parent, (520, 62))

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        self._frame(p)
        t = self.nav.telemetry if self.nav else None
        heading = t.heading_deg if t and t.heading_deg is not None else None
        w, h = self.width(), self.height()
        px_per_deg = (w - 20) / self.SPAN

        if heading is None:
            p.setPen(theme.qcolor(theme.TEXT_DIM))
            p.drawText(self.rect(), Qt.AlignCenter, "NO HEADING")
            return

        def x_of(bearing: float) -> float:
            return w / 2 + geo.angle_difference(bearing, heading) * px_per_deg

        p.setFont(_font(8, True))
        start = int(math.floor((heading - self.SPAN / 2) / 5.0) * 5)
        for b in range(start, start + int(self.SPAN) + 10, 5):
            x = x_of(b)
            if not 8 < x < w - 8:
                continue
            major = b % 30 == 0
            p.setPen(QtGui.QPen(theme.qcolor(theme.TEXT, 220 if major else 120), 1.4 if major else 1))
            p.drawLine(QPointF(x, h - 8), QPointF(x, h - (22 if major else 15)))
            if major:
                label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(b % 360, f"{b % 360:03d}")
                p.setPen(theme.qcolor(theme.ACCENT if label.isalpha() else theme.TEXT))
                p.drawText(QRectF(x - 20, h - 40, 40, 14), Qt.AlignCenter, label)

        def bug(bearing: float | None, colour: str, label: str, below: bool = True) -> None:
            if bearing is None:
                return
            x = max(14.0, min(w - 14.0, x_of(bearing)))
            y = h - 6 if below else 6
            sign = -1 if below else 1
            p.setPen(Qt.NoPen)
            p.setBrush(theme.qcolor(colour))
            p.drawPolygon(QtGui.QPolygonF([QPointF(x, y + sign * 10), QPointF(x - 6, y),
                                           QPointF(x + 6, y)]))
            p.setFont(_font(6.5, True))
            p.setPen(theme.qcolor(colour))
            p.drawText(QRectF(x - 20, (y + sign * 22) - 6, 40, 12), Qt.AlignCenter, label)

        nav = self.nav
        bug(t.cog_deg if (t.sog_ms or 0) > 0.2 else None, theme.GOOD, "COG")
        bug(nav.bearing_to_target_deg, theme.TARGET, nav.target_label.split(" ")[0] or "BTW")
        bug(nav.course_setpoint_deg, theme.CAUTION, "SET", below=False)
        if nav.home_bearing_deg is not None and nav.target_label != "HOME":
            bug(nav.home_bearing_deg, theme.HOME, "H", below=False)

        box = QRectF(w / 2 - 38, 4, 76, 26)
        p.setPen(QtGui.QPen(theme.qcolor(theme.VALUE), 1.5))
        p.setBrush(QtGui.QColor(0, 0, 0, 220))
        p.drawRoundedRect(box, 5, 5)
        p.setFont(_font(14, True, mono=True))
        p.setPen(theme.qcolor(theme.VALUE))
        p.drawText(box, Qt.AlignCenter, f"{heading % 360:03.0f}°")
        p.setPen(QtGui.QPen(theme.qcolor(theme.VALUE), 2))
        p.drawLine(QPointF(w / 2, 30), QPointF(w / 2, h - 6))


class SpeedDial(Instrument):
    """Speed over ground on a dial, with speed through water and the setpoint."""

    MAX_KN = 6.0

    def __init__(self, parent=None) -> None:
        super().__init__(parent, (168, 168))

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        self._frame(p, "SPEED")
        t = self.nav.telemetry if self.nav else None
        centre = QPointF(self.width() / 2, self.height() / 2 + 8)
        radius = 62
        start, sweep = 225.0, 270.0   # Qt angles, counter-clockwise from 3 o'clock

        def angle_of(kn: float) -> float:
            return math.radians(start - sweep * max(0.0, min(1.0, kn / self.MAX_KN)))

        p.setPen(QtGui.QPen(theme.qcolor(theme.BORDER), 8, cap=Qt.RoundCap))
        arc = QRectF(centre.x() - radius, centre.y() - radius, 2 * radius, 2 * radius)
        p.drawArc(arc, int(start * 16), int(-sweep * 16))
        sog_kn = units.knots(t.sog_ms) if t and t.sog_ms is not None else 0.0
        p.setPen(QtGui.QPen(theme.qcolor(theme.ACCENT), 8, cap=Qt.RoundCap))
        p.drawArc(arc, int(start * 16), int(-sweep * min(1.0, sog_kn / self.MAX_KN) * 16))

        p.setFont(_font(7))
        for kn in range(0, int(self.MAX_KN) + 1):
            a = angle_of(kn)
            inner = QPointF(centre.x() + math.cos(a) * (radius - 12), centre.y() - math.sin(a) * (radius - 12))
            outer = QPointF(centre.x() + math.cos(a) * (radius - 5), centre.y() - math.sin(a) * (radius - 5))
            p.setPen(QtGui.QPen(theme.qcolor(theme.TEXT_DIM), 1.2))
            p.drawLine(inner, outer)
            label = QPointF(centre.x() + math.cos(a) * (radius - 21), centre.y() - math.sin(a) * (radius - 21))
            p.drawText(QRectF(label.x() - 8, label.y() - 6, 16, 12), Qt.AlignCenter, str(kn))

        nav = self.nav
        if nav is not None and nav.mode.value in ("speed_hold", "course_hold"):
            a = angle_of(units.knots(nav.speed_setpoint_ms))
            tip = QPointF(centre.x() + math.cos(a) * (radius + 6), centre.y() - math.sin(a) * (radius + 6))
            p.setPen(Qt.NoPen)
            p.setBrush(theme.qcolor(theme.CAUTION))
            p.drawEllipse(tip, 4, 4)

        p.setPen(theme.qcolor(theme.VALUE))
        p.setFont(_font(20, True, mono=True))
        p.drawText(QRectF(0, centre.y() - 20, self.width(), 30), Qt.AlignCenter,
                   self._speed(t.sog_ms if t else None))
        p.setFont(_font(7.5))
        p.setPen(theme.qcolor(theme.TEXT_DIM))
        p.drawText(QRectF(0, centre.y() + 8, self.width(), 14), Qt.AlignCenter, f"SOG {self._speed_unit}")
        p.drawText(QRectF(0, centre.y() + 22, self.width(), 14), Qt.AlignCenter,
                   f"STW {self._speed(t.stw_ms if t else None)}")


class HelmGauge(Instrument):
    """Both throttles, the steering they add up to, and the rate of turn."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent, (250, 168))

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        self._frame(p, "HELM")
        t = self.nav.telemetry if self.nav else None
        port = t.throttle_port if t else 0.0
        stbd = t.throttle_stbd if t else 0.0
        cmd_port = self.nav.command_port if self.nav else 0.0
        cmd_stbd = self.nav.command_stbd if self.nav else 0.0
        h = self.height()

        def bar(x: float, actual: float, command: float, label: str, rpm: float | None) -> None:
            top, bottom = 26.0, h - 30.0
            mid = (top + bottom) / 2
            half = (bottom - top) / 2
            p.setPen(QtGui.QPen(theme.qcolor(theme.BORDER), 1))
            p.setBrush(QtGui.QColor(0, 0, 0, 150))
            p.drawRoundedRect(QRectF(x, top, 22, bottom - top), 4, 4)
            colour = theme.GOOD if actual >= 0 else theme.CAUTION
            p.setPen(Qt.NoPen)
            p.setBrush(theme.qcolor(colour))
            p.drawRect(QRectF(x + 3, mid - max(0, actual) * half, 16, abs(actual) * half))
            p.setPen(QtGui.QPen(theme.qcolor(theme.TEXT_DIM), 1))
            p.drawLine(QPointF(x - 3, mid), QPointF(x + 25, mid))
            p.setPen(QtGui.QPen(theme.qcolor(theme.VALUE), 2))
            p.drawLine(QPointF(x - 4, mid - command * half), QPointF(x + 26, mid - command * half))
            p.setFont(_font(7.5, True))
            p.setPen(theme.qcolor(theme.TEXT))
            p.drawText(QRectF(x - 14, bottom + 2, 50, 12), Qt.AlignCenter, label)
            p.setFont(_font(8, True, mono=True))
            p.setPen(theme.qcolor(theme.VALUE))
            p.drawText(QRectF(x - 14, bottom + 14, 50, 12), Qt.AlignCenter, f"{actual * 100:+.0f}%")

        bar(14, port, cmd_port, "PORT", t.rpm_port if t else None)
        bar(self.width() - 36, stbd, cmd_stbd, "STBD", t.rpm_stbd if t else None)

        # Steering semicircle: the differential, as a rudder angle would show.
        centre = QPointF(self.width() / 2, 92)
        radius = 62
        p.setPen(QtGui.QPen(theme.qcolor(theme.BORDER), 1.5))
        p.setBrush(QtGui.QColor(0, 20, 30, 160))
        p.drawPie(QRectF(centre.x() - radius, centre.y() - radius, 2 * radius, 2 * radius), 0, 180 * 16)
        steer = (port - stbd) / 2.0
        angle = math.radians(90 - steer * 80)
        tip = QPointF(centre.x() + math.cos(angle) * (radius - 6), centre.y() - math.sin(angle) * (radius - 6))
        p.setPen(QtGui.QPen(theme.qcolor(theme.BAD), 4, cap=Qt.RoundCap))
        p.drawLine(centre, tip)
        p.setPen(QtGui.QPen(theme.qcolor(theme.TEXT_DIM), 1))
        for mark in (-1.0, -0.5, 0.0, 0.5, 1.0):
            a = math.radians(90 - mark * 80)
            p.drawLine(QPointF(centre.x() + math.cos(a) * (radius - 2), centre.y() - math.sin(a) * (radius - 2)),
                       QPointF(centre.x() + math.cos(a) * (radius + 3), centre.y() - math.sin(a) * (radius + 3)))
        p.setBrush(theme.qcolor(theme.TEXT))
        p.drawEllipse(centre, 4, 4)

        # Rate of turn, in degrees a minute as marine ROT indicators read.
        rot = (t.rot_dps or 0.0) * 60.0 if t else 0.0
        p.setFont(_font(7.5))
        p.setPen(theme.qcolor(theme.TEXT_DIM))
        p.drawText(QRectF(centre.x() - 50, centre.y() + 8, 100, 12), Qt.AlignCenter, "ROT °/min")
        p.setFont(_font(12, True, mono=True))
        p.setPen(theme.qcolor(theme.VALUE))
        p.drawText(QRectF(centre.x() - 60, centre.y() + 20, 120, 22), Qt.AlignCenter,
                   f"{'◀' if rot < -5 else ' '}{abs(rot):.0f}{'▶' if rot > 5 else ' '}")
        p.setFont(_font(7.5))
        p.setPen(theme.qcolor(theme.TEXT_DIM))
        p.drawText(QRectF(centre.x() - 50, centre.y() - radius - 2, 100, 12), Qt.AlignCenter,
                   "◀ PORT  STBD ▶")


class EchoSounder(Instrument):
    """Depth under the keel and the seabed profile of the last two minutes."""

    HISTORY_S = 120.0

    def __init__(self, parent=None) -> None:
        super().__init__(parent, (300, 168))
        self.history: deque[tuple[float, float | None]] = deque(maxlen=1200)
        self.shallow_alarm_m = 1.2
        self.shallow_stop_m = 0.7

    def set_nav(self, nav) -> None:
        t = nav.telemetry if nav else None
        if t is not None and t.boat_time_s is not None:
            if not self.history or t.boat_time_s > self.history[-1][0] + 0.18:
                self.history.append((t.boat_time_s, t.depth_m))
            elif t.boat_time_s < self.history[-1][0]:
                self.history.clear()
        super().set_nav(nav)

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        self._frame(p, "ECHO SOUNDER")
        t = self.nav.telemetry if self.nav else None
        depth = t.depth_m if t else None
        graph = QRectF(10, 22, self.width() - 110, self.height() - 32)
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor(4, 18, 34, 230))
        p.drawRect(graph)

        samples = [(ts, d) for ts, d in self.history if d is not None]
        now = self.history[-1][0] if self.history else 0.0
        if samples:
            recent = [d for ts, d in samples if now - ts <= self.HISTORY_S]
            top_depth = max(4.0, (max(recent) if recent else 4.0) * 1.25)
            scale = graph.height() / top_depth

            def point(ts: float, d: float) -> QPointF:
                x = graph.right() - (now - ts) / self.HISTORY_S * graph.width()
                return QPointF(x, graph.top() + d * scale)

            seabed = [point(ts, d) for ts, d in samples if now - ts <= self.HISTORY_S]
            if len(seabed) > 1:
                polygon = QtGui.QPolygonF([QPointF(seabed[0].x(), graph.bottom()), *seabed,
                                           QPointF(seabed[-1].x(), graph.bottom())])
                gradient = QtGui.QLinearGradient(0, graph.top(), 0, graph.bottom())
                gradient.setColorAt(0.0, QtGui.QColor(255, 190, 60, 230))
                gradient.setColorAt(1.0, QtGui.QColor(120, 60, 20, 230))
                p.setBrush(gradient)
                p.setPen(QtGui.QPen(QtGui.QColor(255, 220, 120), 1.5))
                p.drawPolygon(polygon)
            for level, colour in ((self.shallow_alarm_m, theme.CAUTION), (self.shallow_stop_m, theme.BAD)):
                y = graph.top() + level * scale
                if y < graph.bottom():
                    p.setPen(QtGui.QPen(theme.qcolor(colour, 200), 1, Qt.DashLine))
                    p.drawLine(QPointF(graph.left(), y), QPointF(graph.right(), y))
            p.setPen(theme.qcolor(theme.TEXT_DIM))
            p.setFont(_font(6.5))
            p.drawText(QRectF(graph.left() + 2, graph.bottom() - 12, 60, 12), Qt.AlignLeft,
                       f"{top_depth:.0f} m")

        colour = theme.TEXT
        if depth is not None and depth < self.shallow_alarm_m:
            colour = theme.BAD
        elif depth is None:
            colour = theme.TEXT_DIM
        p.setPen(theme.qcolor(colour if colour != theme.TEXT else theme.VALUE))
        p.setFont(_font(24, True, mono=True))
        text = "—" if depth is None else (f"{depth:.1f}" if depth < 100 else f"{depth:.0f}")
        p.drawText(QRectF(graph.right() + 4, 40, 96, 40), Qt.AlignCenter, text)
        p.setFont(_font(8))
        p.setPen(theme.qcolor(theme.TEXT_DIM))
        p.drawText(QRectF(graph.right() + 4, 80, 96, 14), Qt.AlignCenter, "m below keel")
        if t and t.water.temperature_c is not None:
            p.drawText(QRectF(graph.right() + 4, 104, 96, 14), Qt.AlignCenter,
                       f"water {t.water.temperature_c:.1f} °C")
        if self.nav and self.nav.soundings:
            p.drawText(QRectF(graph.right() + 4, 122, 96, 14), Qt.AlignCenter,
                       f"{self.nav.soundings} pings")


class WindIndicator(Instrument):
    """Apparent wind relative to the bow."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent, (120, 120))

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        self._frame(p, "WIND")
        t = self.nav.telemetry if self.nav else None
        centre = QPointF(self.width() / 2, self.height() / 2 + 6)
        radius = 38
        p.setPen(QtGui.QPen(theme.qcolor(theme.BORDER), 1.5))
        p.setBrush(QtGui.QColor(0, 0, 0, 120))
        p.drawEllipse(centre, radius, radius)
        # Port red, starboard green, as on every wind instrument.
        arc = QRectF(centre.x() - radius, centre.y() - radius, 2 * radius, 2 * radius)
        p.setPen(QtGui.QPen(theme.qcolor(theme.BAD, 200), 4))
        p.drawArc(arc, 90 * 16, 60 * 16)
        p.setPen(QtGui.QPen(theme.qcolor(theme.GOOD, 200), 4))
        p.drawArc(arc, 30 * 16, 60 * 16)
        # The boat, bow up.
        p.setPen(QtGui.QPen(theme.qcolor(theme.BOAT), 1.5))
        p.drawLine(QPointF(centre.x(), centre.y() - 14), QPointF(centre.x() - 6, centre.y() + 10))
        p.drawLine(QPointF(centre.x(), centre.y() - 14), QPointF(centre.x() + 6, centre.y() + 10))
        if t and t.wind_apparent_deg is not None:
            a = math.radians(t.wind_apparent_deg)
            tip = QPointF(centre.x() + math.sin(a) * (radius - 2), centre.y() - math.cos(a) * (radius - 2))
            tail = QPointF(centre.x() + math.sin(a) * 14, centre.y() - math.cos(a) * 14)
            p.setPen(QtGui.QPen(theme.qcolor(theme.ACCENT), 3, cap=Qt.RoundCap))
            p.drawLine(tip, tail)
            p.setBrush(theme.qcolor(theme.ACCENT))
            p.drawEllipse(tip, 4, 4)
            p.setFont(_font(7.5, True, mono=True))
            p.setPen(theme.qcolor(theme.VALUE))
            p.drawText(QRectF(0, self.height() - 16, self.width(), 14), Qt.AlignCenter,
                       f"AWA {abs(t.wind_apparent_deg):.0f}°{'S' if t.wind_apparent_deg > 0 else 'P'}"
                       f" {self._speed(t.wind_apparent_ms)}")


class AlarmBanner(QtWidgets.QWidget):
    """The most urgent alarms, flashing, across the top of the chart."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.alarms = ()
        self._phase = False
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._blink)
        timer.start(450)
        self.setFixedHeight(34)

    def _blink(self) -> None:
        self._phase = not self._phase
        if self.alarms:
            self.update()

    def set_alarms(self, alarms) -> None:
        changed = alarms != self.alarms
        self.alarms = alarms
        self.setVisible(bool(alarms))
        if changed:
            self.update()

    def paintEvent(self, event) -> None:
        if not self.alarms:
            return
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        top = self.alarms[0]
        colour = theme.qcolor(theme.severity_colour(int(top.level)))
        if int(top.level) >= 4 and self._phase:
            colour = colour.darker(160)
        p.setBrush(colour)
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(QRectF(self.rect()), 8, 8)
        p.setPen(QtGui.QColor("#08080a"))
        p.setFont(_font(11, True))
        text = top.text + (f"   +{len(self.alarms) - 1} more" if len(self.alarms) > 1 else "")
        p.drawText(self.rect(), Qt.AlignCenter, "⚠  " + text)
