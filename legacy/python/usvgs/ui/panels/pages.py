"""Drawer pages with graphs: the controller, power, depth and the oil skimmer."""

from __future__ import annotations

import math
import time

import numpy as np
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QPointF, QRectF, Qt

from ...core import geo, units
from ...core.geo import LatLon
from ...core.mission import PatternSpec, Rectangle
from ...core.profile import route_profile
from ...input.gamepad import PadState
from ...input.mapping import BUTTON_HELP
from ...sim.oil import WINDAGE, appearance
from .. import theme


def _font(size: float, bold: bool = False, mono: bool = False) -> QtGui.QFont:
    font = QtGui.QFont("DejaVu Sans Mono" if mono else "DejaVu Sans")
    font.setPointSizeF(size)
    font.setBold(bold)
    return font


def _heading(text: str) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    label.setObjectName("heading")
    return label


class ValueGrid(QtWidgets.QFrame):
    """Big readouts in a grid: label above, value below."""

    def __init__(self, items: list[tuple[str, str]], columns: int = 2) -> None:
        super().__init__()
        self.setObjectName("card")
        grid = QtWidgets.QGridLayout(self)
        grid.setContentsMargins(8, 6, 8, 6)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(2)
        self.values: dict[str, QtWidgets.QLabel] = {}
        for i, (key, title) in enumerate(items):
            r, c = divmod(i, columns)
            name = QtWidgets.QLabel(title)
            name.setStyleSheet(f"color: {theme.TEXT_DIM}; background: transparent; font-size: 11px;")
            value = QtWidgets.QLabel("—")
            value.setStyleSheet(f"color: {theme.VALUE}; background: transparent; font-size: 16px;"
                                "font-weight: 700; font-family: 'DejaVu Sans Mono';")
            grid.addWidget(name, r * 2, c)
            grid.addWidget(value, r * 2 + 1, c)
            self.values[key] = value

    def set(self, key: str, text: str, colour: str = theme.VALUE) -> None:
        label = self.values[key]
        if label.text() != text:
            label.setText(text)
        style = (f"color: {colour}; background: transparent; font-size: 16px; font-weight: 700;"
                 "font-family: 'DejaVu Sans Mono';")
        if label.styleSheet() != style:
            label.setStyleSheet(style)


class Graph(QtWidgets.QWidget):
    """A small time-series plot frame: subclasses supply the series."""

    def __init__(self, height: int = 150) -> None:
        super().__init__()
        self.setMinimumHeight(height)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

    def plot_rect(self) -> QRectF:
        return QRectF(40, 16, self.width() - 52, self.height() - 34)

    def frame(self, p: QtGui.QPainter, title: str) -> QRectF:
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QtGui.QColor(theme.SURFACE))
        rect = self.plot_rect()
        p.fillRect(rect, QtGui.QColor(4, 12, 20))
        p.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 1))
        p.drawRect(rect)
        p.setFont(_font(7.5, True))
        p.setPen(QtGui.QColor(theme.ACCENT))
        p.drawText(QRectF(6, 0, self.width(), 15), Qt.AlignLeft | Qt.AlignVCenter, title)
        return rect

    @staticmethod
    def axis_label(p, rect: QRectF, y: float, text: str, colour=theme.TEXT_DIM, right=False) -> None:
        p.setFont(_font(6.5))
        p.setPen(QtGui.QColor(colour))
        if right:
            p.drawText(QRectF(rect.right() + 2, y - 6, 40, 12), Qt.AlignLeft | Qt.AlignVCenter, text)
        else:
            p.drawText(QRectF(0, y - 6, rect.left() - 3, 12), Qt.AlignRight | Qt.AlignVCenter, text)


# --------------------------------------------------------------------------
# The controller
# --------------------------------------------------------------------------


class DeckDiagram(QtWidgets.QWidget):
    """The Steam Deck, drawn, with its sticks and buttons live."""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(170)
        self.pad = PadState()
        self.keys: set[str] = set()
        self.helm = None

    def set_state(self, pad: PadState, keys: set[str], helm) -> None:
        self.pad, self.keys, self.helm = pad, set(keys), helm
        self.update()

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        w = self.width()
        # The Deck is about 2.5 : 1; fit it and work in a 300 × 120 design box.
        s = min(w / 300.0, (self.height() - 10) / 128.0)
        p.translate((w - 300 * s) / 2, 6)
        p.scale(s, s)
        pad = self.pad
        held = pad.held
        on = theme.qcolor(theme.GOOD)
        idle = QtGui.QColor("#2a3644")
        edge = QtGui.QColor("#52637a")

        # Body, screen.
        body = QtGui.QPainterPath()
        body.addRoundedRect(QRectF(0, 14, 300, 104), 42, 42)
        p.setPen(QtGui.QPen(edge, 1.5))
        p.setBrush(QtGui.QColor("#161d26"))
        p.drawPath(body)
        p.setBrush(QtGui.QColor("#05090d"))
        p.drawRoundedRect(QRectF(88, 24, 124, 76), 4, 4)
        p.setPen(QtGui.QColor(theme.TEXT_DIM))
        p.setFont(_font(6))
        p.drawText(QRectF(88, 24, 124, 76), Qt.AlignCenter,
                   f"{pad.name or 'no pad'}\n{pad.backend or 'keyboard'}")

        # Shoulders and triggers.
        def shoulder(x: float, pressed: bool, label: str) -> None:
            p.setPen(QtGui.QPen(edge, 1))
            p.setBrush(on if pressed else idle)
            p.drawRoundedRect(QRectF(x, 6, 46, 10), 4, 4)
            p.setPen(QtGui.QColor(theme.TEXT))
            p.drawText(QRectF(x, 6, 46, 10), Qt.AlignCenter, label)

        def trigger(x: float, value: float, label: str) -> None:
            p.setPen(QtGui.QPen(edge, 1))
            p.setBrush(idle)
            p.drawRoundedRect(QRectF(x, -6, 46, 10), 4, 4)
            p.setPen(Qt.NoPen)
            p.setBrush(theme.qcolor(theme.CAUTION))
            p.drawRoundedRect(QRectF(x + 1, -5, 44 * value, 8), 3, 3)
            p.setPen(QtGui.QColor(theme.TEXT))
            p.drawText(QRectF(x, -6, 46, 10), Qt.AlignCenter, f"{label} {value * 100:.0f}")

        shoulder(22, "LB" in held, "L1")
        shoulder(232, "RB" in held, "R1")
        trigger(22, pad.lt, "L2")
        trigger(232, pad.rt, "R2")

        def stick(cx: float, cy: float, x: float, y: float, pressed: bool, label: str) -> None:
            p.setPen(QtGui.QPen(edge, 1.2))
            p.setBrush(QtGui.QColor("#0c1218"))
            p.drawEllipse(QPointF(cx, cy), 15, 15)
            p.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 0.8, Qt.DotLine))
            p.drawLine(QPointF(cx - 15, cy), QPointF(cx + 15, cy))
            p.drawLine(QPointF(cx, cy - 15), QPointF(cx, cy + 15))
            knob = QPointF(cx + x * 10, cy - y * 10)
            p.setPen(QtGui.QPen(on if pressed else edge, 1.5))
            p.setBrush(theme.qcolor(theme.ACCENT) if (abs(x) + abs(y)) > 0.05 else idle)
            p.drawEllipse(knob, 8, 8)
            p.setPen(QtGui.QColor(theme.TEXT_DIM))
            p.drawText(QRectF(cx - 20, cy + 15, 40, 9), Qt.AlignCenter, label)

        stick(60, 42, pad.lx, pad.ly, "L3" in held, f"{pad.lx:+.1f} {pad.ly:+.1f}")
        stick(240, 42, pad.rx, pad.ry, "R3" in held, f"{pad.rx:+.1f} {pad.ry:+.1f}")

        # D-pad.
        cx, cy = 32, 76
        for name, dx, dy in (("UP", 0, -1), ("DOWN", 0, 1), ("LEFT", -1, 0), ("RIGHT", 1, 0)):
            p.setPen(QtGui.QPen(edge, 1))
            p.setBrush(on if name in held else idle)
            p.drawRect(QRectF(cx + dx * 9 - 4.5, cy + dy * 9 - 4.5, 9, 9))
        # Face buttons, in their places on the Deck.
        for name, dx, dy, colour in (("Y", 0, -1, "#e8c33d"), ("A", 0, 1, "#5ee07a"),
                                     ("X", -1, 0, "#4fa8ff"), ("B", 1, 0, "#ff5a4e")):
            p.setPen(QtGui.QPen(QtGui.QColor(colour), 1.2))
            p.setBrush(QtGui.QColor(colour) if name in held else idle)
            centre = QPointF(268 + dx * 10, 76 + dy * 10)
            p.drawEllipse(centre, 5.5, 5.5)
            p.setPen(QtGui.QColor("#061018") if name in held else QtGui.QColor(colour))
            p.setFont(_font(5.5, True))
            p.drawText(QRectF(centre.x() - 5, centre.y() - 5, 10, 10), Qt.AlignCenter, name)
        # View and menu.
        p.setFont(_font(5.5, True))
        for name, x, glyph in (("VIEW", 76, "⧉"), ("MENU", 216, "☰")):
            p.setPen(QtGui.QPen(edge, 1))
            p.setBrush(on if name in held else idle)
            p.drawEllipse(QPointF(x, 24), 5, 5)
            p.setPen(QtGui.QColor(theme.TEXT))
            p.drawText(QRectF(x - 5, 19, 10, 10), Qt.AlignCenter, glyph)
        # Trackpads.
        p.setPen(QtGui.QPen(edge, 1))
        p.setBrush(QtGui.QColor("#10161d"))
        p.drawRoundedRect(QRectF(46, 70, 32, 32), 6, 6)
        p.drawRoundedRect(QRectF(222, 70, 32, 32), 6, 6)
        p.setPen(QtGui.QColor(theme.TEXT_DIM))
        p.drawText(QRectF(222, 70, 32, 32), Qt.AlignCenter, "mouse")
        p.drawText(QRectF(46, 70, 32, 32), Qt.AlignCenter, "—")

        # What the sticks add up to: the two motors.
        helm = self.helm
        if helm is not None:
            for x, value, label in ((104, helm.port, "PORT"), (190, helm.stbd, "STBD")):
                p.setPen(QtGui.QPen(edge, 1))
                p.setBrush(QtGui.QColor("#0c1218"))
                p.drawRect(QRectF(x, 30, 6, 64))
                p.setPen(Qt.NoPen)
                p.setBrush(theme.qcolor(theme.GOOD if value >= 0 else theme.CAUTION))
                p.drawRect(QRectF(x, 62 - max(0.0, value) * 32, 6, abs(value) * 32))
                p.setPen(QtGui.QColor(theme.TEXT))
                p.setFont(_font(5.5, True))
                p.drawText(QRectF(x - 12, 94, 30, 8), Qt.AlignCenter, label)
        if self.keys:
            p.setPen(theme.qcolor(theme.CAUTION))
            p.setFont(_font(6, True))
            p.drawText(QRectF(88, 102, 124, 12), Qt.AlignCenter, "keys: " + " ".join(sorted(self.keys)))


class ControlsPage(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        control = window.settings.control
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(_heading("STEAM DECK — LIVE"))
        self.diagram = DeckDiagram()
        layout.addWidget(self.diagram)

        row = QtWidgets.QHBoxLayout()
        self.arcade = QtWidgets.QPushButton("Arcade sticks")
        self.tank = QtWidgets.QPushButton("Tank sticks")
        for b in (self.arcade, self.tank):
            b.setCheckable(True)
            row.addWidget(b)
        self.arcade.clicked.connect(lambda: self._mode("arcade"))
        self.tank.clicked.connect(lambda: self._mode("tank"))
        layout.addLayout(row)

        form = QtWidgets.QFormLayout()
        self.deadzone = QtWidgets.QSlider(Qt.Horizontal)
        self.deadzone.setRange(0, 30)
        self.deadzone.setValue(int(control.deadzone * 100))
        self.deadzone.valueChanged.connect(lambda v: setattr(control, "deadzone", v / 100.0))
        self.expo = QtWidgets.QSlider(Qt.Horizontal)
        self.expo.setRange(0, 90)
        self.expo.setValue(int(control.expo * 100))
        self.expo.valueChanged.connect(lambda v: setattr(control, "expo", v / 100.0))
        form.addRow("Dead zone", self.deadzone)
        form.addRow("Expo", self.expo)
        layout.addLayout(form)

        layout.addWidget(_heading("BUTTONS"))
        table = QtWidgets.QLabel("".join(
            f"<div><b style='color:{theme.VALUE}'>{k}</b> &nbsp; {v}</div>" for k, v in BUTTON_HELP))
        table.setWordWrap(True)
        table.setTextFormat(Qt.RichText)
        layout.addWidget(table)
        layout.addStretch(1)
        self._sync()

    def _mode(self, mode: str) -> None:
        self.window.settings.control.stick_mode = mode
        self._sync()

    def _sync(self) -> None:
        mode = self.window.settings.control.stick_mode
        self.arcade.setChecked(mode == "arcade")
        self.tank.setChecked(mode == "tank")

    def update_state(self, pad: PadState, keys: set[str], helm) -> None:
        self._sync()
        self.diagram.set_state(pad, keys, helm)


# --------------------------------------------------------------------------
# Power
# --------------------------------------------------------------------------


class PowerGraph(Graph):
    """State of charge and voltage over time, and where the charge is heading."""

    def __init__(self) -> None:
        super().__init__(190)
        self.history = []
        self.estimate = None
        self.window_s = 1800.0
        self.low_pct = 30.0
        self.critical_pct = 15.0

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        rect = self.frame(p, "CHARGE % (green) · VOLTS (yellow) · FORECAST (dashed)")
        history = self.history
        if len(history) < 2:
            p.setPen(QtGui.QColor(theme.TEXT_DIM))
            p.drawText(rect, Qt.AlignCenter, "Collecting…")
            return
        now = history[-1].t
        span = self.window_s
        # Leave a third of the plot for the forecast.
        t0, t1 = now - span * 0.67, now + span * 0.33

        def x_of(t: float) -> float:
            return rect.left() + (t - t0) / (t1 - t0) * rect.width()

        def y_pct(pct: float) -> float:
            return rect.bottom() - pct / 100.0 * rect.height()

        volts = [s.volts for s in history if s.t >= t0] or [history[-1].volts]
        v_low, v_high = min(11.5, min(volts)), max(16.8, max(volts))

        def y_v(v: float) -> float:
            return rect.bottom() - (v - v_low) / (v_high - v_low) * rect.height()

        for pct in (0, 25, 50, 75, 100):
            y = y_pct(pct)
            p.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 0.6, Qt.DotLine))
            p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            self.axis_label(p, rect, y, f"{pct}%")
        for v in (12, 14, 16):
            self.axis_label(p, rect, y_v(v), f"{v}V", theme.VALUE, right=True)
        for level, colour in ((self.low_pct, theme.CAUTION), (self.critical_pct, theme.BAD)):
            p.setPen(QtGui.QPen(theme.qcolor(colour, 170), 1, Qt.DashLine))
            p.drawLine(QPointF(rect.left(), y_pct(level)), QPointF(rect.right(), y_pct(level)))
        p.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT_DIM), 1))
        p.drawLine(QPointF(x_of(now), rect.top()), QPointF(x_of(now), rect.bottom()))

        p.save()
        p.setClipRect(rect)
        # Current as faint bars, so the cause of a dip is visible.
        p.setPen(Qt.NoPen)
        p.setBrush(theme.qcolor(theme.ACCENT, 60))
        visible = [s for s in history if s.t >= t0]
        step = max(1, len(visible) // 300)
        for s in visible[::step]:
            height = min(1.0, s.amps / 45.0) * rect.height() * 0.5
            p.drawRect(QRectF(x_of(s.t), rect.bottom() - height, max(1.0, rect.width() / 300), height))
        p.setPen(QtGui.QPen(theme.qcolor(theme.GOOD), 2))
        p.drawPolyline(QtGui.QPolygonF([QPointF(x_of(s.t), y_pct(s.percent)) for s in visible[::step]]
                                       + [QPointF(x_of(now), y_pct(history[-1].percent))]))
        p.setPen(QtGui.QPen(theme.qcolor(theme.VALUE), 1.4))
        p.drawPolyline(QtGui.QPolygonF([QPointF(x_of(s.t), y_v(s.volts)) for s in visible[::step]]))
        est = self.estimate
        if est is not None:
            line = est.projection(now, history[-1].percent, t1 - now)
            p.setPen(QtGui.QPen(theme.qcolor(theme.GOOD, 200), 2, Qt.DashLine))
            p.drawLine(QPointF(x_of(line[0][0]), y_pct(line[0][1])),
                       QPointF(x_of(line[1][0]), y_pct(line[1][1])))
            for seconds, colour in ((est.time_to_low_s, theme.CAUTION), (est.time_to_critical_s, theme.BAD)):
                if math.isfinite(seconds) and 0 < seconds < t1 - now:
                    x = x_of(now + seconds)
                    p.setPen(QtGui.QPen(theme.qcolor(colour), 1.5))
                    p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
        p.restore()
        p.setFont(_font(6.5))
        p.setPen(QtGui.QColor(theme.TEXT_DIM))
        p.drawText(QRectF(rect.left(), rect.bottom() + 2, rect.width(), 12), Qt.AlignLeft,
                   f"−{units.format_duration(span * 0.67)}")
        p.drawText(QRectF(rect.left(), rect.bottom() + 2, rect.width(), 12), Qt.AlignRight,
                   f"+{units.format_duration(span * 0.33)}")
        p.drawText(QRectF(x_of(now) - 20, rect.bottom() + 2, 40, 12), Qt.AlignCenter, "now")


class PowerPage(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(_heading("BATTERY"))
        self.values = ValueGrid([
            ("volts", "Voltage"), ("amps", "Current"), ("watts", "Power"), ("pct", "Charge"),
            ("used", "Energy used"), ("left", "Energy left"), ("ttg", "Time to empty"),
            ("range", "Range at this pace"), ("low", "Until low"), ("crit", "Until critical"),
            ("whkm", "Consumption"), ("home", "Reserve to get home"),
        ])
        layout.addWidget(self.values)
        self.graph = PowerGraph()
        layout.addWidget(self.graph)
        row = QtWidgets.QHBoxLayout()
        for label, seconds in (("10 min", 600), ("30 min", 1800), ("2 h", 7200)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _c, s=seconds: setattr(self.graph, "window_s", s) or self.graph.update())
            row.addWidget(b)
        layout.addLayout(row)
        note = QtWidgets.QLabel("The forecast takes the worse of two estimates: the recent draw against "
                                "the charge left, and how fast the charge has actually fallen. Reserve "
                                "to get home uses the energy per kilometre measured over the last minute, "
                                "plus 30 %.")
        note.setWordWrap(True)
        note.setObjectName("dim")
        layout.addWidget(note)
        layout.addStretch(1)

    def refresh(self, nav) -> None:
        vehicle = self.window.vehicle
        safety = self.window.settings.safety
        with vehicle.lock:
            history = list(vehicle.power_history)
        self.graph.history = history
        self.graph.estimate = nav.energy
        self.graph.low_pct = safety.battery_low_pct
        self.graph.critical_pct = safety.battery_critical_pct
        self.graph.update()
        t = nav.telemetry
        e = nav.energy
        v = self.values
        if t is None or e is None:
            return

        def dur(seconds: float) -> str:
            return "∞" if not math.isfinite(seconds) else units.format_duration(seconds)

        v.set("volts", f"{t.battery_v:.2f} V")
        v.set("amps", f"{t.battery_a:.1f} A")
        v.set("watts", f"{e.power_w:.0f} W")
        v.set("pct", f"{t.battery_pct:.0f} %", theme.battery_colour(t.battery_pct))
        v.set("used", f"{e.used_wh:.1f} Wh")
        v.set("left", f"{e.remaining_wh:.0f} Wh")
        v.set("ttg", dur(e.time_to_empty_s))
        v.set("range", "∞" if not math.isfinite(e.range_m) else units.format_distance(e.range_m))
        v.set("low", dur(e.time_to_low_s), theme.CAUTION if e.time_to_low_s < 600 else theme.VALUE)
        v.set("crit", dur(e.time_to_critical_s), theme.BAD if e.time_to_critical_s < 600 else theme.VALUE)
        v.set("whkm", "—" if e.wh_per_km is None else f"{e.wh_per_km:.0f} Wh/km")
        if e.home_reserve_pct is None:
            v.set("home", "—")
        else:
            short = (t.battery_pct or 0) - e.home_reserve_pct < safety.battery_critical_pct
            v.set("home", f"{e.home_reserve_pct:.1f} %", theme.BAD if short else theme.GOOD)


# --------------------------------------------------------------------------
# Depth
# --------------------------------------------------------------------------


class ProfileGraph(Graph):
    """Water depth under the route ahead, down to the seabed."""

    def __init__(self) -> None:
        super().__init__(200)
        self.profile = None
        self.draft = 0.25
        self.alarm = 1.2
        self.title = "ROUTE AHEAD: WATER DOWN TO THE SEABED"

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        rect = self.frame(p, self.title)
        prof = self.profile
        if prof is None or len(prof.distance) < 2:
            p.setPen(QtGui.QColor(theme.TEXT_DIM))
            p.drawText(rect, Qt.AlignCenter, "Plan a route, or start one,\nto see the seabed along it")
            return
        depths = [s if s is not None else (0.0 if c != c else c)
                  for c, s in zip(prof.chart, prof.surveyed, strict=True)]
        deepest = max(3.0, max(depths) * 1.15)
        length = max(1.0, prof.length)

        def x_of(d: float) -> float:
            return rect.left() + d / length * rect.width()

        def y_of(depth: float) -> float:
            return rect.top() + depth / deepest * rect.height()

        # Water above, seabed below, land where the depth is nil.
        water = QtGui.QLinearGradient(0, rect.top(), 0, rect.bottom())
        water.setColorAt(0.0, QtGui.QColor(*theme.WATER_SHALLOW))
        water.setColorAt(1.0, QtGui.QColor(*theme.WATER_DEEP))
        p.fillRect(rect, water)
        bed = [QPointF(x_of(d), y_of(z)) for d, z in zip(prof.distance, depths, strict=True)]
        ground = QtGui.QPolygonF([QPointF(rect.left(), rect.bottom()), *bed, QPointF(rect.right(), rect.bottom())])
        sand = QtGui.QLinearGradient(0, rect.top(), 0, rect.bottom())
        sand.setColorAt(0.0, QtGui.QColor(196, 160, 96))
        sand.setColorAt(1.0, QtGui.QColor(92, 66, 34))
        p.setPen(QtGui.QPen(QtGui.QColor(240, 210, 150), 1.5))
        p.setBrush(sand)
        p.drawPolygon(ground)
        # Surveyed stretches marked along the seabed.
        p.setPen(QtGui.QPen(theme.qcolor(theme.GOOD), 3))
        for i in range(len(bed) - 1):
            if prof.surveyed[i] is not None and prof.surveyed[i + 1] is not None:
                p.drawLine(bed[i], bed[i + 1])
        for level, colour, text in ((self.draft, theme.BAD, "draft"), (self.alarm, theme.CAUTION, "alarm")):
            y = y_of(level)
            p.setPen(QtGui.QPen(theme.qcolor(colour, 220), 1, Qt.DashLine))
            p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            self.axis_label(p, rect, y, text, colour, right=True)
        for depth in np.linspace(0, deepest, 5):
            self.axis_label(p, rect, y_of(depth), f"{depth:.0f}m")
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 120), 1, Qt.DotLine))
        for d in prof.waypoints[1:-1]:
            p.drawLine(QPointF(x_of(d), rect.top()), QPointF(x_of(d), rect.bottom()))
        # The boat at the start, sitting on the surface.
        p.setPen(Qt.NoPen)
        p.setBrush(theme.qcolor(theme.BOAT))
        p.drawPolygon(QtGui.QPolygonF([QPointF(rect.left(), rect.top()), QPointF(rect.left() + 12, rect.top()),
                                       QPointF(rect.left() + 6, rect.top() + 7)]))
        low = prof.shallowest()
        p.setFont(_font(6.5))
        p.setPen(QtGui.QColor(theme.TEXT))
        p.drawText(QRectF(rect.left(), rect.bottom() + 2, rect.width(), 12), Qt.AlignLeft, "boat")
        p.drawText(QRectF(rect.left(), rect.bottom() + 2, rect.width(), 12), Qt.AlignRight,
                   units.format_distance(prof.length))
        if low is not None:
            colour = theme.BAD if low[1] < self.alarm else theme.GOOD
            p.setPen(theme.qcolor(colour))
            p.drawText(QRectF(rect.left(), rect.bottom() + 2, rect.width(), 12), Qt.AlignCenter,
                       f"least water {low[1]:.1f} m at {units.format_distance(low[0])}")


class SounderGraph(Graph):
    """Five minutes of the echo sounder, at full size."""

    def __init__(self) -> None:
        super().__init__(150)
        self.samples: list[tuple[float, float]] = []
        self.alarm = 1.2

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        rect = self.frame(p, "ECHO SOUNDER · LAST 5 MIN · DEPTH BELOW KEEL")
        if len(self.samples) < 2:
            return
        now = self.samples[-1][0]
        recent = [(t, d) for t, d in self.samples if now - t <= 300]
        deepest = max(3.0, max(d for _, d in recent) * 1.2)

        def point(t, d):
            return QPointF(rect.right() - (now - t) / 300.0 * rect.width(), rect.top() + d / deepest * rect.height())

        bed = [point(t, d) for t, d in recent]
        polygon = QtGui.QPolygonF([QPointF(bed[0].x(), rect.bottom()), *bed, QPointF(bed[-1].x(), rect.bottom())])
        gradient = QtGui.QLinearGradient(0, rect.top(), 0, rect.bottom())
        gradient.setColorAt(0, QtGui.QColor(255, 190, 60))
        gradient.setColorAt(1, QtGui.QColor(120, 50, 20))
        p.setBrush(gradient)
        p.setPen(QtGui.QPen(QtGui.QColor(255, 225, 140), 1.2))
        p.drawPolygon(polygon)
        y = rect.top() + self.alarm / deepest * rect.height()
        p.setPen(QtGui.QPen(theme.qcolor(theme.CAUTION), 1, Qt.DashLine))
        p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
        for depth in np.linspace(0, deepest, 4):
            self.axis_label(p, rect, rect.top() + depth / deepest * rect.height(), f"{depth:.0f}m")
        values = [d for _, d in recent]
        p.setFont(_font(7, True, mono=True))
        p.setPen(QtGui.QColor(theme.VALUE))
        p.drawText(QRectF(rect.left() + 4, rect.top() + 2, rect.width() - 8, 12), Qt.AlignRight,
                   f"min {min(values):.1f}  avg {sum(values) / len(values):.1f}  max {max(values):.1f} m")


class DepthPage(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self.values = ValueGrid([("depth", "Under keel"), ("chart", "Chart depth here"),
                                 ("water", "Water depth"), ("least", "Least on route")])
        layout.addWidget(self.values)
        self.profile = ProfileGraph()
        layout.addWidget(self.profile)
        self.sounder = SounderGraph()
        layout.addWidget(self.sounder)
        note = QtWidgets.QLabel("The profile follows the remaining route — the planned one when no "
                                "mission is running. Green along the seabed marks where it has been "
                                "sounded; elsewhere it is the chart.")
        note.setWordWrap(True)
        note.setObjectName("dim")
        layout.addWidget(note)
        layout.addStretch(1)
        self._last_profile = 0.0

    def refresh(self, nav) -> None:
        window = self.window
        t = nav.telemetry
        safety = window.settings.safety
        self.profile.draft = window.settings.boat.draft_m
        self.profile.alarm = safety.shallow_alarm_m
        self.sounder.alarm = safety.shallow_alarm_m
        history = window.chart_area.sounder.history
        self.sounder.samples = [(ts, d) for ts, d in history if d is not None]
        self.sounder.update()
        if t is None or not t.has_position:
            return
        here = LatLon(t.lat, t.lon)
        chart_here = window.chart_grid.depth_at(t.lat, t.lon) if window.chart_grid else math.nan
        v = self.values
        v.set("depth", "—" if t.depth_m is None else f"{t.depth_m:.2f} m",
              theme.BAD if t.depth_m is not None and t.depth_m < safety.shallow_alarm_m else theme.VALUE)
        v.set("chart", "land" if chart_here != chart_here else f"{chart_here:.1f} m")
        v.set("water", "—" if t.depth_m is None else f"{t.depth_m + window.settings.boat.draft_m:.2f} m")

        now = time.monotonic()
        if now - self._last_profile < 2.0:
            return
        self._last_profile = now
        mission = window.vehicle.mission
        with window.vehicle.lock:
            waypoints = mission.positions
        start = nav.waypoint_index if nav.waypoint_index >= 0 else 0
        route = [here] + waypoints[start:]
        if len(route) < 2 and nav.target is not None:
            route = [here, nav.target]
        grid = window.vehicle.soundings

        def surveyed(lat: float, lon: float):
            return grid.depth_of(grid.cell_of(lat, lon))

        def chart_only(lat: float, lon: float) -> float:
            # The chart, not the simulator's hidden truth: plan on what is known.
            return window.chart_grid.depth_at(lat, lon) if window.chart_grid else math.nan

        with window.vehicle.lock:
            profile = route_profile(route, chart_only, surveyed) if len(route) >= 2 else None
        self.profile.profile = profile
        self.profile.update()
        low = profile.shallowest() if profile else None
        v.set("least", "—" if low is None else f"{low[1]:.1f} m",
              theme.BAD if low and low[1] < safety.shallow_alarm_m else theme.VALUE)


# --------------------------------------------------------------------------
# Oil
# --------------------------------------------------------------------------


class TankGauge(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setFixedHeight(54)
        self.state = None

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        s = self.state
        rect = QRectF(4, 14, self.width() - 8, 30)
        p.setPen(QtGui.QPen(QtGui.QColor(theme.BORDER), 1.5))
        p.setBrush(QtGui.QColor(4, 10, 16))
        p.drawRoundedRect(rect, 6, 6)
        fraction = 0.0 if s is None or not s.tank_capacity_l else s.tank_l / s.tank_capacity_l
        colour = theme.BAD if fraction > 0.95 else (theme.CAUTION if fraction > 0.75 else "#8a5a24")
        p.setPen(Qt.NoPen)
        p.setBrush(theme.qcolor(colour))
        p.drawRoundedRect(QRectF(rect.left() + 2, rect.top() + 2, (rect.width() - 4) * fraction,
                                 rect.height() - 4), 4, 4)
        p.setPen(QtGui.QColor(theme.TEXT))
        p.setFont(_font(8, True))
        p.drawText(QRectF(4, 0, self.width(), 14), Qt.AlignLeft, "RECOVERED-FLUID TANK")
        p.setFont(_font(10, True, mono=True))
        text = "—" if s is None else f"{s.tank_l:.1f} / {s.tank_capacity_l:.0f} L"
        p.drawText(rect, Qt.AlignCenter, text)


class SkimCurve(Graph):
    """Recovery efficiency against speed, with the boat's speed on it."""

    def __init__(self, skimmer_settings) -> None:
        super().__init__(110)
        self.settings = skimmer_settings
        self.speed = 0.0
        from ...sim.oil import Skimmer
        self.model = Skimmer(skimmer_settings)

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        rect = self.frame(p, "SKIMMING EFFICIENCY vs SPEED THROUGH WATER")
        top = self.settings.max_speed_ms * 2.4

        def x_of(v: float) -> float:
            return rect.left() + v / top * rect.width()

        def y_of(e: float) -> float:
            return rect.bottom() - e * rect.height()

        good = QRectF(x_of(0), rect.top(), x_of(self.settings.max_speed_ms) - x_of(0), rect.height())
        p.fillRect(good, theme.qcolor(theme.GOOD, 30))
        p.setPen(QtGui.QPen(theme.qcolor(theme.GOOD), 2))
        p.drawPolyline(QtGui.QPolygonF([QPointF(x_of(v), y_of(self.model.efficiency_at(v)))
                                        for v in np.linspace(0, top, 60)]))
        x = x_of(min(top, abs(self.speed)))
        p.setPen(QtGui.QPen(theme.qcolor(theme.BOAT), 2))
        p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
        for v_kn in (0, 1, 2, 3):
            v = units.from_knots(v_kn)
            if v <= top:
                p.setFont(_font(6.5))
                p.setPen(QtGui.QColor(theme.TEXT_DIM))
                p.drawText(QRectF(x_of(v) - 15, rect.bottom() + 1, 30, 12), Qt.AlignCenter, f"{v_kn} kn")
        self.axis_label(p, rect, y_of(0.85), "85%")
        self.axis_label(p, rect, y_of(0.0), "0")


class OilPage(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(_heading("OIL SKIMMER"))
        row = QtWidgets.QHBoxLayout()
        self.deploy = QtWidgets.QPushButton("⛭ Deploy skimmer")
        self.deploy.setCheckable(True)
        self.deploy.clicked.connect(lambda on: window.vehicle.set_skimmer(on))
        self.sweep = QtWidgets.QPushButton("▭ Plan sweep on slick")
        self.sweep.clicked.connect(self.plan_sweep)
        row.addWidget(self.deploy)
        row.addWidget(self.sweep)
        layout.addLayout(row)
        self.tank = TankGauge()
        layout.addWidget(self.tank)
        self.values = ValueGrid([
            ("state", "Skimmer"), ("rate", "Recovery rate"), ("eff", "Efficiency"),
            ("oil", "Oil recovered"), ("ppm", "Oil in water"), ("film", "Film here"),
            ("bonn", "Appearance (Bonn)"), ("drift", "Slick drift"),
        ])
        layout.addWidget(self.values)
        self.curve = SkimCurve(window.settings.skimmer)
        layout.addWidget(self.curve)

        layout.addWidget(_heading("THE SPILL"))
        self.spill = ValueGrid([("released", "Released"), ("floating", "On the water"),
                                ("recovered", "Recovered"), ("evaporated", "Evaporated"),
                                ("beached", "Ashore"), ("score", "Clean-up")])
        layout.addWidget(self.spill)
        if window.link.simulated:
            row = QtWidgets.QHBoxLayout()
            again = QtWidgets.QPushButton("New spill")
            again.clicked.connect(lambda: (window.link.release_spill(), window.toast("A new slick is on the water")))
            here = QtWidgets.QPushButton("Spill at view")
            here.clicked.connect(self._spill_here)
            clear = QtWidgets.QPushButton("Clear")
            clear.clicked.connect(window.link.clear_spill)
            for b in (again, here, clear):
                row.addWidget(b)
            layout.addLayout(row)
        note = QtWidgets.QLabel(
            "Sweep arms funnel the slick into a brush skimmer between the hulls. Booms only hold oil "
            "slowly — keep under about 1.5 kn, ideally 0.7 kn — and work up-drift through the "
            "thickest (darkest) oil. When the tank is full, bring the boat alongside the GCS with the "
            "skimmer stowed and it pumps out.")
        note.setWordWrap(True)
        note.setObjectName("dim")
        layout.addWidget(note)
        layout.addStretch(1)

    def _spill_here(self) -> None:
        canvas = self.window.canvas
        self.window.link.release_spill(LatLon(canvas.centre_lat, canvas.centre_lon))
        self.window.toast("Slick released in the middle of the view")

    def drift(self) -> tuple[float, float] | None:
        """How the slick moves: current plus 3 % of the wind (speed m/s, towards degrees)."""
        link = self.window.link
        if not link.simulated:
            return None
        wn, we = link.world.wind_vector()
        cn, ce = link.world.current_vector()
        n, e = cn + WINDAGE * wn, ce + WINDAGE * we
        return math.hypot(n, e), (math.degrees(math.atan2(e, n)) + 360) % 360

    def plan_sweep(self) -> None:
        """A lawnmower over the slick, lanes along its drift, a swath apart."""
        window = self.window
        oil = window.link.spill_snapshot() if window.link.simulated else None
        if oil is None or not (oil[2] > 1e-3).any():
            window.toast("No slick to sweep")
            return
        lat, lon, volume, beached = oil
        keep = (volume > np.percentile(volume[volume > 0], 20)) & ~beached
        lat, lon = lat[keep], lon[keep]
        drift = self.drift()
        angle = drift[1] if drift and drift[0] > 0.01 else 90.0
        centre = LatLon(float(lat.mean()), float(lon.mean()))
        theta = math.radians(angle)
        u = (math.sin(theta), math.cos(theta))
        v = (-u[1], u[0])
        along, across = [], []
        for a, b in zip(lat, lon, strict=True):
            m = geo.to_metres(centre, (a, b))
            along.append(m.east * u[0] + m.north * u[1])
            across.append(m.east * v[0] + m.north * v[1])
        along_lo, along_hi = np.percentile(along, [3, 97])
        across_lo, across_hi = np.percentile(across, [3, 97])
        mid = geo.to_lat_lon(centre, geo.Metres(
            (along_lo + along_hi) / 2 * u[0] + (across_lo + across_hi) / 2 * v[0],
            (along_lo + along_hi) / 2 * u[1] + (across_lo + across_hi) / 2 * v[1]))
        rect = Rectangle(mid, float(along_hi - along_lo) + 10, float(across_hi - across_lo) + 10, angle)
        swath = window.settings.skimmer.swath_m
        with window.vehicle.lock:
            window.vehicle.mission.area = rect
            window.vehicle.mission.pattern = PatternSpec("lawnmower", spacing_m=swath,
                                                         margin_m=1.0, along_long_side=True)
            window.vehicle.mission.regenerate()
            # Transit to the first lane at cruise speed; skim every lane at boom speed.
            skim = window.settings.skimmer.best_speed_ms * 1.3
            for waypoint in window.vehicle.mission.waypoints[1:]:
                waypoint.speed_ms = skim
            # The pattern moves with the oil, as a real sweep is planned.
            window.vehicle.mission.drift_ms = window.link.spill_drift()
        window.mission_tab.sync_from_mission()
        window.mission_edited()
        window.frame_mission()
        window.toast(f"Sweep planned: {len(window.vehicle.mission.waypoints)} waypoints, skimming at "
                     f"{units.knots(skim):.1f} kn — deploy the skimmer, then ▶")

    def refresh(self, nav) -> None:
        t = nav.telemetry
        s = t.skimmer if t is not None else None
        self.tank.state = s
        self.tank.update()
        if not self.deploy.isDown():
            self.deploy.setChecked(nav.skimmer_commanded)
            self.deploy.setText("⛭ Stow skimmer" if nav.skimmer_commanded else "⛭ Deploy skimmer")
        v = self.values
        if s is not None:
            state = "OFFLOADING" if s.offloading else ("RECOVERING" if s.recovering else (
                "DEPLOYED" if s.deployed else "STOWED"))
            colour = theme.GOOD if s.recovering else (theme.ACCENT if s.deployed or s.offloading else theme.TEXT_DIM)
            v.set("state", state, colour)
            v.set("rate", f"{s.recovery_lpm:.2f} L/min")
            v.set("eff", f"{s.efficiency * 100:.0f} %", theme.GOOD if s.efficiency > 0.6 else theme.CAUTION)
            v.set("oil", f"{s.recovered_oil_l:.1f} L")
            v.set("ppm", "—" if s.oil_ppm is None else f"{s.oil_ppm:.0f} ppm")
            v.set("film", "—" if s.film_um is None else f"{s.film_um:.1f} µm")
            code, text = appearance(s.film_um)
            v.set("bonn", f"{code} {text.split(' (')[0]}")
            self.curve.speed = t.stw_ms or 0.0
            self.curve.update()
        drift = self.drift()
        v.set("drift", "—" if drift is None else f"{drift[0]:.2f} m/s → {drift[1]:03.0f}°")
        if self.window.link.simulated:
            totals = self.window.link.spill.totals
            sp = self.spill
            sp.set("released", f"{totals.released_l:.0f} L")
            sp.set("floating", f"{totals.floating_l:.1f} L")
            sp.set("recovered", f"{totals.recovered_l:.1f} L", theme.GOOD)
            sp.set("evaporated", f"{totals.evaporated_l:.1f} L")
            sp.set("beached", f"{totals.beached_l:.1f} L", theme.BAD if totals.beached_l > 1 else theme.VALUE)
            sp.set("score", f"{totals.recovered_fraction * 100:.0f} %", theme.GOOD)
