"""The ground station window.

Laid out for the Steam Deck's 1280×800 screen, after the USV stations used in
the field: telemetry down the left, the chart in the middle with the
instruments over it, a tool strip on the right with a pull-out planning panel,
and the control modes along the bottom where the thumbs reach them.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

from .. import config
from ..core import units
from ..core.bathymetry import DepthGrid
from ..core.events import EventBus, Level
from ..core.geo import LatLon
from ..input import gamepad
from ..input.mapping import helm_from_keys, helm_from_pad
from ..link.sim import SimLink
from ..vehicle.modes import Mode
from ..vehicle.vehicle import Vehicle
from . import theme
from .mapview.canvas import VIEW_LABELS, ChartCanvas
from .mapview.depth import ChartLayer, SurveyLayer
from .mapview.tiles import TiledMap, find_map_index
from .panels.drawer import ChartTab, HelpTab, LogTab, MissionTab, SimTab
from .panels.pages import ControlsPage, DepthPage, OilPage, PowerPage
from .panels.telemetry import TelemetryPanel
from .widgets.instruments import AlarmBanner, CompassTape, EchoSounder, HelmGauge, SpeedDial, WindIndicator

log = logging.getLogger(__name__)

BUNDLED_CHART = Path(__file__).resolve().parents[1] / "data" / "riga_bathy.npz"
HOLD_PRESS_S = 0.8

MODE_BUTTONS = (
    (Mode.MANUAL, "🎮 Manual", "1"),
    (Mode.SPEED_HOLD, "Speed\nHold", "2"),
    (Mode.COURSE_HOLD, "Speed/Course\nHold", "3"),
    (Mode.POINT_NAV, "◎ Point\nNav", "4"),
    (Mode.AUTO, "▶ Pre-\nprogram", "5"),
    (Mode.HOLD, "⚓ Station\nHold", "6"),
    (Mode.RETURN_HOME, "⌂ Return\nHome", "7"),
)


def dms(value: float | None, positive: str, negative: str) -> str:
    if value is None:
        return "—"
    hemisphere = positive if value >= 0 else negative
    value = abs(value)
    degrees = int(value)
    minutes = (value - degrees) * 60
    return f"{degrees}°{minutes:06.3f}′{hemisphere}"


def gcs_battery() -> float | None:
    """The ground station's own battery — the Deck's — from sysfs."""
    for path in sorted(Path("/sys/class/power_supply").glob("BAT*")):
        try:
            return float((path / "capacity").read_text().strip())
        except (OSError, ValueError):
            continue
    return None


class GcsBatteryWatch:
    """:func:`gcs_battery`, read every ``period_s`` on a thread of its own.

    Reading the capacity asks the battery's controller, which can take ten
    milliseconds or more: on the interface thread that is a dropped frame."""

    def __init__(self, period_s: float = 20.0) -> None:
        self.level: float | None = None
        self._period = period_s
        self._stop = threading.Event()
        threading.Thread(target=self._run, name="gcs-battery", daemon=True).start()

    def _run(self) -> None:
        while not self._stop.is_set():
            self.level = gcs_battery()
            self._stop.wait(self._period)

    def stop(self) -> None:
        self._stop.set()


class StatusBar(QtWidgets.QWidget):
    """The strip across the top: who is connected, how well, and the power."""

    def __init__(self) -> None:
        super().__init__()
        self.setFixedHeight(40)
        self.nav = None
        self.link_name = ""
        self.pad = ""
        self.time_scale = 1.0
        self.gcs_battery: float | None = None

    def paintEvent(self, event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QtGui.QColor(theme.SURFACE))
        p.setPen(QtGui.QColor(theme.BORDER))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)

        x = 12
        p.setFont(QtGui.QFont("DejaVu Sans", 15, QtGui.QFont.Black))
        p.setPen(QtGui.QColor(theme.CAUTION))
        p.drawText(QtCore.QRectF(x, 0, 160, self.height()), Qt.AlignVCenter, "| USV·GS")
        x += 118

        nav = self.nav
        mode = nav.mode if nav else Mode.MANUAL
        armed = nav.armed if nav else False

        def chip(text: str, colour: str, fill: bool = False, width: float | None = None) -> None:
            nonlocal x
            font = QtGui.QFont("DejaVu Sans", 10, QtGui.QFont.Bold)
            p.setFont(font)
            w = width or QtGui.QFontMetrics(font).horizontalAdvance(text) + 22
            box = QtCore.QRectF(x, 7, w, self.height() - 14)
            p.setPen(QtGui.QPen(QtGui.QColor(colour), 1.5))
            p.setBrush(QtGui.QColor(colour) if fill else QtGui.QColor(0, 0, 0, 0))
            p.drawRoundedRect(box, 6, 6)
            p.setPen(QtGui.QColor("#061018") if fill else QtGui.QColor(colour))
            p.drawText(box, Qt.AlignCenter, text)
            x += w + 8

        chip(self.link_name + (f" ×{self.time_scale:g}" if self.time_scale != 1 else ""), theme.ACCENT)
        chip(mode.label.upper().replace("\n", " "), theme.GOOD if mode is not Mode.STOP else theme.BAD, True)
        chip("ARMED" if armed else "DISARMED", theme.CAUTION if armed else theme.TEXT_DIM, armed)
        if nav is not None and not nav.link_ok:
            chip("NO LINK", theme.BAD, True)

        # Right-hand side, laid out from the right edge.
        right = self.width() - 12
        p.setFont(QtGui.QFont("DejaVu Sans Mono", 11, QtGui.QFont.Bold))

        def right_text(text: str, colour: str, width: int) -> None:
            nonlocal right
            p.setPen(QtGui.QColor(colour))
            p.drawText(QtCore.QRectF(right - width, 0, width, self.height()),
                       Qt.AlignVCenter | Qt.AlignRight, text)
            right -= width + 12

        right_text(time.strftime("%H:%M:%S"), theme.TEXT, 86)
        self._bars(p, right - 30, nav)
        right -= 44
        t = nav.telemetry if nav else None
        usv = t.battery_pct if t else None
        right_text(f"USV {usv:.0f}% {t.battery_v:.1f}V" if usv is not None and t.battery_v else "USV —",
                   theme.battery_colour(usv), 150)
        right_text(f"GCS {self.gcs_battery:.0f}%" if self.gcs_battery is not None else "GCS ⏻",
                   theme.battery_colour(self.gcs_battery), 90)
        right_text(f"🎮 {self.pad}" if self.pad else "🎮 keyboard", theme.GOOD if self.pad else theme.TEXT_DIM, 200)

    def _bars(self, p: QtGui.QPainter, x: float, nav) -> None:
        rssi = nav.telemetry.rssi_dbm if nav and nav.telemetry and nav.link_ok else None
        level = 0 if rssi is None else max(1, min(5, int((rssi + 100) / 10)))
        for i in range(5):
            h = 6 + i * 4
            colour = theme.GOOD if i < level else theme.BORDER
            if level and level <= 2 and i < level:
                colour = theme.CAUTION
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(colour))
            p.drawRect(QtCore.QRectF(x + i * 6, self.height() - 10 - h, 4, h))


class DrawerPanel(QtWidgets.QFrame):
    """The pull-out panel: a grid of page buttons over the pages themselves.

    A grid rather than tabs, because nine tabs do not fit across the panel and
    a thumb needs targets this size.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(f"DrawerPanel {{ background: {theme.SURFACE}; border-left: 1px solid {theme.BORDER}; }}")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 0)
        layout.setSpacing(4)
        self._grid = QtWidgets.QGridLayout()
        self._grid.setSpacing(3)
        layout.addLayout(self._grid)
        self.stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.stack, 1)
        self._buttons: list[QtWidgets.QPushButton] = []
        self._group = QtWidgets.QButtonGroup(self)
        self._group.setExclusive(True)

    def add_page(self, widget: QtWidgets.QWidget, title: str) -> None:
        index = len(self._buttons)
        button = QtWidgets.QPushButton(title)
        button.setCheckable(True)
        button.setMinimumHeight(30)
        button.setStyleSheet("padding: 3px 4px; min-height: 22px; font-size: 12px;")
        button.clicked.connect(lambda: self.setCurrentIndex(index))
        self._group.addButton(button)
        self._grid.addWidget(button, index // 3, index % 3)
        self._buttons.append(button)
        self.stack.addWidget(widget)
        if index == 0:
            button.setChecked(True)

    def setCurrentIndex(self, index: int) -> None:
        index = max(0, min(self.count() - 1, index))
        self.stack.setCurrentIndex(index)
        self._buttons[index].setChecked(True)

    def currentIndex(self) -> int:
        return self.stack.currentIndex()

    def count(self) -> int:
        return self.stack.count()


class ZoomControls(QtWidgets.QFrame):
    """Big zoom buttons over the chart, for a finger rather than a mouse wheel.

    Holding a button keeps zooming; the label between them says how much
    water is on screen.
    """

    def __init__(self, parent, canvas: ChartCanvas, on_view) -> None:
        super().__init__(parent)
        self.canvas = canvas
        self.setStyleSheet(
            "ZoomControls { background: rgba(6,12,18,170); border: 1px solid rgba(54,198,244,90);"
            " border-radius: 12px; }"
            "QToolButton { font-size: 22px; font-weight: 800; min-width: 50px; min-height: 50px;"
            " border-radius: 10px; background: rgba(24,35,47,230); }"
            "QLabel { background: transparent; color: #e3ecf4; font-size: 11px; font-weight: 700; }")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)

        def button(text: str, slot, repeat: bool = False) -> QtWidgets.QToolButton:
            b = QtWidgets.QToolButton()
            b.setText(text)
            b.setAutoRepeat(repeat)
            b.setAutoRepeatDelay(350)
            b.setAutoRepeatInterval(110)
            b.clicked.connect(slot)
            layout.addWidget(b, 0, Qt.AlignHCenter)
            return b

        button("＋", lambda: canvas.zoom(0.5), repeat=True)
        self.readout = QtWidgets.QLabel()
        self.readout.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.readout)
        button("－", lambda: canvas.zoom(-0.5), repeat=True)
        self.view_button = button("N↑", on_view)
        self.view_button.setToolTip("North up → heading up → 3D chase camera")
        self.view_button.setStyleSheet("font-size: 15px;")
        canvas.view_changed.connect(self.refresh)
        self.refresh()

    def refresh(self) -> None:
        metres = self.canvas.view_width_m
        self.readout.setText(f"{metres / 1000:.1f} km" if metres >= 1000 else f"{metres:.0f} m")
        self.view_button.setText(VIEW_LABELS[self.canvas.view_mode])
        self.adjustSize()


class EdgeHandle(QtWidgets.QToolButton):
    """A tab on the edge of the chart that opens and closes the side panel there."""

    def __init__(self, parent, side: str, slot) -> None:
        super().__init__(parent)
        self.side = side
        self.setFixedSize(22, 72)
        self.setStyleSheet(
            "QToolButton { background: rgba(24,35,47,235); border: 1px solid rgba(54,198,244,120);"
            f" border-radius: 6px; color: {theme.ACCENT}; font-size: 14px; font-weight: 800; padding: 0; }}")
        self.clicked.connect(slot)

    def set_open(self, open_: bool) -> None:
        if self.side == "left":
            self.setText("◀" if open_ else "▶")
            self.setToolTip("Hide the telemetry" if open_ else "Show the telemetry")
        else:
            self.setText("▶" if open_ else "◀")
            self.setToolTip("Hide the panel" if open_ else "Show the panel")


class ChartArea(QtWidgets.QWidget):
    """The chart with the instruments floating over it."""

    def __init__(self, canvas: ChartCanvas) -> None:
        super().__init__()
        self.canvas = canvas
        layout = QtWidgets.QStackedLayout(self)
        layout.setStackingMode(QtWidgets.QStackedLayout.StackAll)
        layout.addWidget(canvas)
        self.compass = CompassTape(self)
        self.alarm = AlarmBanner(self)
        self.wind = WindIndicator(self)
        self.speed = SpeedDial(self)
        self.helm = HelmGauge(self)
        self.sounder = EchoSounder(self)
        self.toast = QtWidgets.QLabel(self)
        self.toast.setAlignment(Qt.AlignCenter)
        self.toast.setStyleSheet(
            f"background: rgba(6,12,18,220); color: {theme.TEXT}; border: 1px solid {theme.ACCENT};"
            "border-radius: 8px; padding: 8px 14px; font-size: 14px;")
        self.toast.hide()
        self._toast_timer = QtCore.QTimer(self, singleShot=True, timeout=self.toast.hide)
        self.instruments = (self.compass, self.wind, self.speed, self.helm, self.sounder)
        self.zoom: ZoomControls | None = None
        self.left_handle: EdgeHandle | None = None
        self.right_handle: EdgeHandle | None = None

    def instruments_visible(self, on: bool) -> None:
        for widget in self.instruments:
            widget.setVisible(on)

    def show_toast(self, text: str, seconds: float = 3.0) -> None:
        self.toast.setText(text)
        self.toast.adjustSize()
        self._place_toast()
        self.toast.show()
        self.toast.raise_()
        self._toast_timer.start(int(seconds * 1000))

    def _place_toast(self) -> None:
        self.toast.move((self.width() - self.toast.width()) // 2,
                        self.height() - self.toast.height() - 180)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        w, h = self.width(), self.height()
        # The compass keeps clear of the wind indicator on a narrow chart.
        compass_x = max(self.wind.width() + 18, (w - self.compass.width()) // 2)
        self.compass.setFixedWidth(max(300, min(520, w - compass_x - 10)))
        self.compass.move(max(self.wind.width() + 18, (w - self.compass.width()) // 2), 8)
        self.alarm.setFixedWidth(min(560, w - 40))
        self.alarm.move((w - self.alarm.width()) // 2, 76)
        self.wind.move(10, 10)
        self.speed.move(10, h - self.speed.height() - 10)
        self.helm.move(10 + self.speed.width() + 8, h - self.helm.height() - 10)
        self.sounder.move(w - self.sounder.width() - 10, h - self.sounder.height() - 10)
        # Squeeze on a narrow chart: the echo sounder gives way first.
        overlap = self.helm.x() + self.helm.width() + 8 > self.sounder.x()
        self.sounder.setVisible(self.speed.isVisible() and not overlap)
        self._place_toast()
        if self.zoom is not None:
            self.zoom.adjustSize()
            top = self.wind.y() + self.wind.height() + 12
            room = h - self.speed.height() - 20 - top
            self.zoom.move(30, top + max(0, (room - self.zoom.height()) // 2))
        if self.left_handle is not None:
            self.left_handle.move(0, (h - self.left_handle.height()) // 2)
            self.right_handle.move(w - self.right_handle.width(), (h - self.right_handle.height()) // 2)
        for widget in (*self.instruments, self.alarm, self.zoom, self.left_handle, self.right_handle):
            if widget is not None:
                widget.raise_()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, settings: config.Settings, chart: DepthGrid | None, tiles: TiledMap) -> None:
        super().__init__()
        self.settings = settings
        self.chart_grid = chart
        self.bus = EventBus()
        self.link = SimLink(settings, chart)
        self.vehicle = Vehicle(settings, self.link, self.bus)
        self._events: queue.Queue = queue.Queue()
        self.bus.subscribe(self._events.put)

        self.setWindowTitle("USV Ground Station")
        self.setStyleSheet(theme.STYLESHEET)

        self.canvas = ChartCanvas(tiles, self.vehicle.mission, self.vehicle.lock)
        self.canvas.boat_length_m = settings.boat.length_m
        self.canvas.boat_beam_m = settings.boat.beam_m
        self.canvas.geofence_m = settings.safety.geofence_radius_m
        self.canvas.station = self.link.station
        self.canvas.centre_on(settings.sim.start_lat, settings.sim.start_lon)
        self.canvas.survey = SurveyLayer(self.vehicle.soundings)
        if chart is not None:
            self.canvas.chart = ChartLayer(chart, settings.safety.shallow_alarm_m)
        self.chart_area = ChartArea(self.canvas)

        self.status = StatusBar()
        self.status.link_name = self.link.name
        self.telemetry_panel = TelemetryPanel()
        self.telemetry_panel.setFixedWidth(240)
        self.telemetry_panel.speed_units = settings.map.speed_units
        for widget in self.chart_area.instruments:
            widget.speed_units = settings.map.speed_units

        self.drawer = DrawerPanel()
        self.drawer.setFixedWidth(340)
        self.mission_tab = MissionTab(self)
        self.chart_tab = ChartTab(self)
        self.sim_tab = SimTab(self)
        self.log_tab = LogTab()
        self.depth_page = DepthPage(self)
        self.power_page = PowerPage(self)
        self.oil_page = OilPage(self)
        self.controls_page = ControlsPage(self)
        self.pages = {}
        for title, widget, scrolled in (
                ("Mission", self.mission_tab, True), ("Chart", self.chart_tab, True),
                ("Depth", self.depth_page, True), ("Power", self.power_page, True),
                ("Oil", self.oil_page, True), ("🎮 Pad", self.controls_page, True),
                ("Sim", self.sim_tab, True), ("Log", self.log_tab, False), ("Help", HelpTab(), False)):
            self.drawer.add_page(self._scrolled(widget) if scrolled else widget, title)
            self.pages[title] = self.drawer.count() - 1

        centre = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(centre)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        row.addWidget(self.telemetry_panel)
        row.addWidget(self.chart_area, 1)
        row.addWidget(self.drawer)
        row.addWidget(self._tool_strip())

        root = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(root)
        column.setContentsMargins(0, 0, 0, 0)
        column.setSpacing(0)
        column.addWidget(self.status)
        column.addWidget(centre, 1)
        column.addWidget(self._mode_bar())
        self.setCentralWidget(root)

        self.canvas.mission_edited.connect(self.mission_edited)
        self.canvas.area_edited.connect(self.area_edited)
        self.canvas.goto_requested.connect(self.goto)
        self.canvas.cursor_moved.connect(self.chart_tab.show_cursor)
        self.canvas.selection_changed.connect(lambda _i: self.mission_tab.refresh())

        area = self.chart_area
        area.zoom = ZoomControls(area, self.canvas, self.cycle_view)
        area.left_handle = EdgeHandle(area, "left", self.toggle_telemetry)
        area.right_handle = EdgeHandle(area, "right", self.toggle_drawer)
        ui = settings.ui
        self.canvas.show_vectors = ui.show_vectors
        self.canvas.set_water_style(ui.water_style)
        self.chart_tab.water_changed()
        self.canvas.skimmer_swath_m = settings.skimmer.swath_m
        self.telemetry_panel.setVisible(ui.show_telemetry)
        self.drawer.setVisible(ui.show_drawer)
        self.drawer.setCurrentIndex(ui.drawer_page)
        area.left_handle.set_open(ui.show_telemetry)
        area.right_handle.set_open(ui.show_drawer)
        self.drawer_button.setChecked(ui.show_drawer)
        self.hud_button.setChecked(ui.show_hud)
        area.instruments_visible(ui.show_hud)
        self._last_helm = None

        self._keys: set[str] = set()
        self._held_since: dict[str, float] = {}
        self._fired_hold: set[str] = set()
        self.pad = gamepad.open_backend(settings.control.input_backend)
        QtWidgets.QApplication.instance().installEventFilter(self)

        self._input_timer = QtCore.QTimer(self, timeout=self._poll_input, interval=20)
        # One clock for the picture: every tick moves the boat on between
        # position reports and redraws the chart, at up to 60 Hz.  The panels
        # refresh on the same clock, at 20 Hz and 2 Hz, on ticks of their own,
        # so their work never lands on top of a frame and makes it late.
        self._frame_timer = QtCore.QTimer(self, timeout=self._frame)
        self._frame_timer.setTimerType(Qt.PreciseTimer)
        self._frame_count = 0
        self._gcs_battery = GcsBatteryWatch()
        self.set_max_fps(ui.max_fps)
        self.canvas.show_fps = ui.show_fps

        self.vehicle.start()
        self._input_timer.start()
        self._frame_timer.start()
        self._refresh_slow()
        self.mission_tab.refresh()
        for problem in settings.validate():
            self.bus.message(f"Settings: {problem}", Level.WARNING)
        if self.pad is None:
            self.bus.message("No gamepad backend — driving from the keyboard (WASD)", Level.NOTICE)
        QtCore.QTimer.singleShot(400, lambda: self.toast(
            "Welcome aboard — hold ☰ (or Enter) to arm, then drive with the sticks", 5))

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _scrolled(widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        area.setWidget(widget)
        return area

    def _tool_strip(self) -> QtWidgets.QWidget:
        strip = QtWidgets.QFrame()
        strip.setFixedWidth(64)
        strip.setStyleSheet(f"QFrame {{ background: {theme.SURFACE}; border-left: 1px solid {theme.BORDER}; }}")
        layout = QtWidgets.QVBoxLayout(strip)
        layout.setContentsMargins(4, 6, 4, 6)
        layout.setSpacing(5)
        self.tool_buttons: dict[str, QtWidgets.QToolButton] = {}

        def button(text: str, slot, checkable: bool = False, key: str | None = None):
            b = QtWidgets.QToolButton()
            b.setObjectName("tool")
            b.setText(text)
            b.setCheckable(checkable)
            b.setToolButtonStyle(Qt.ToolButtonTextOnly)
            b.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            b.setMinimumHeight(46)
            b.clicked.connect(slot)
            layout.addWidget(b)
            if key:
                self.tool_buttons[key] = b
            return b

        button("✋\nPan", lambda: self.set_tool("pan"), True, "pan")
        button("＋\nPoints", lambda: self.set_tool("points"), True, "points")
        button("▭\nRect", lambda: self.set_tool("rect"), True, "rect")
        button("◎\nGo to", lambda: self.set_tool("goto"), True, "goto")
        layout.addSpacing(6)
        self.follow_button = button("⌖\nFollow", self.toggle_follow, True)
        self.view_button = button("N↑\nView", self.cycle_view)
        self.vector_button = button("➚\nVectors", self.toggle_vectors, True)
        self.vector_button.setChecked(self.settings.ui.show_vectors)
        self.skim_button = button("⛭\nSkim", lambda: self.vehicle.toggle_skimmer(), True)
        button("⤢\nRoute", self.frame_mission)
        layout.addStretch(1)
        button("⌂\nSet H", self._set_home)
        self.hud_button = button("◑\nHUD", self._toggle_hud, True)
        self.hud_button.setChecked(True)
        self.drawer_button = button("☰\nPanel", self.toggle_drawer, True)
        self.drawer_button.setChecked(True)
        self.tool_buttons["pan"].setChecked(True)
        self.follow_button.setChecked(self.canvas.follow)
        return strip

    def _mode_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        bar.setStyleSheet(f"QFrame {{ background: {theme.SURFACE}; border-top: 1px solid {theme.BORDER}; }}")
        bar.setFixedHeight(70)
        layout = QtWidgets.QHBoxLayout(bar)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)
        self.mode_buttons: dict[Mode, QtWidgets.QPushButton] = {}
        group = QtWidgets.QButtonGroup(self)
        group.setExclusive(True)

        self.arm_button = QtWidgets.QPushButton("ARM")
        self.arm_button.setObjectName("arm")
        self.arm_button.setCheckable(True)
        self.arm_button.setMinimumWidth(88)
        self.arm_button.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        self.arm_button.clicked.connect(lambda: (self.vehicle.toggle_arm(), self._refresh()))
        layout.addWidget(self.arm_button)

        for mode, text, _key in MODE_BUTTONS:
            b = QtWidgets.QPushButton(text)
            b.setObjectName("mode")
            b.setCheckable(True)
            b.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            b.clicked.connect(lambda _checked, m=mode: self.request_mode(m))
            group.addButton(b)
            layout.addWidget(b)
            self.mode_buttons[mode] = b

        stop = QtWidgets.QPushButton("■  USV STOP")
        stop.setObjectName("stop")
        stop.setCheckable(True)
        stop.setMinimumWidth(112)
        stop.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        stop.clicked.connect(lambda: self.request_mode(Mode.STOP))
        group.addButton(stop)
        layout.addWidget(stop)
        self.mode_buttons[Mode.STOP] = stop

        self.position_label = QtWidgets.QLabel()
        self.position_label.setStyleSheet(
            f"color: {theme.VALUE}; font-family: 'DejaVu Sans Mono'; font-size: 12px; font-weight: 700;"
            "background: transparent;")
        self.position_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.position_label.setMinimumWidth(124)
        layout.addWidget(self.position_label)
        return bar

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def toast(self, text: str, seconds: float = 3.0) -> None:
        self.chart_area.show_toast(text, seconds)

    def request_mode(self, mode: Mode) -> None:
        if mode is Mode.POINT_NAV and self.vehicle.point_target is None:
            self.set_tool("goto")
            self.toast("Tap the chart where the boat should go")
        elif mode is Mode.AUTO:
            self.start_mission()
        elif self.vehicle.set_mode(mode) and not self.vehicle.armed and mode is not Mode.STOP:
            self.toast(f"{mode.label} selected — arm to give it the motors")
        self._refresh()

    def start_mission(self) -> None:
        if self.vehicle.set_mode(Mode.AUTO):
            self.canvas.follow = True
            self.follow_button.setChecked(True)
            if not self.vehicle.armed:
                self.toast("Mission ready — arm to start it")

    def restart_mission(self) -> None:
        self.vehicle.restart_mission()
        self.toast("Mission back to waypoint 1")

    def goto(self, lat: float, lon: float) -> None:
        self.vehicle.go_to(lat, lon)
        self.set_tool("pan")
        if not self.vehicle.armed:
            self.toast("Point set — arm to go")

    def set_tool(self, tool: str) -> None:
        self.canvas.set_tool(tool)
        for name, b in self.tool_buttons.items():
            b.setChecked(name == tool)
        self.mission_tab.set_tool(tool)
        if tool == "rect":
            self.toast("Drag across the water to draw the survey rectangle")
        elif tool == "points":
            self.toast("Tap to add waypoints · drag to move · right-click to delete")

    def toggle_follow(self) -> None:
        self.canvas.follow = not self.canvas.follow
        self.follow_button.setChecked(self.canvas.follow)
        if self.canvas.follow and self.canvas.boat:
            self.canvas.centre_on(self.canvas.boat[0], self.canvas.boat[1])

    def toggle_drawer(self) -> None:
        shown = not self.drawer.isVisible()
        self.drawer.setVisible(shown)
        self.drawer_button.setChecked(shown)
        self.chart_area.right_handle.set_open(shown)
        self.settings.ui.show_drawer = shown

    def toggle_telemetry(self) -> None:
        shown = not self.telemetry_panel.isVisible()
        self.telemetry_panel.setVisible(shown)
        self.chart_area.left_handle.set_open(shown)
        self.settings.ui.show_telemetry = shown

    def show_page(self, title: str) -> None:
        if not self.drawer.isVisible():
            self.toggle_drawer()
        self.drawer.setCurrentIndex(self.pages[title])

    def cycle_view(self) -> None:
        mode = self.canvas.cycle_view()
        self.view_button.setText(f"{VIEW_LABELS[mode]}\nView")
        self.follow_button.setChecked(True)
        names = {"north": "North up", "heading": "Heading up", "chase": "3D chase camera"}
        self.toast(names[mode], 1.5)

    def toggle_vectors(self) -> None:
        self.canvas.show_vectors = not self.canvas.show_vectors
        self.vector_button.setChecked(self.canvas.show_vectors)
        self.settings.ui.show_vectors = self.canvas.show_vectors

    def _frame(self) -> None:
        self.canvas.advance()
        self.canvas.update()
        count = self._frame_count = self._frame_count + 1
        fps = self.settings.ui.max_fps
        refresh_every = max(1, fps // 20)
        if count % refresh_every == 0:
            self._refresh()
        elif count % (fps // 2) == 1:
            self._refresh_slow()

    def set_max_fps(self, fps: int) -> None:
        fps = 30 if fps <= 30 else 60
        self.settings.ui.max_fps = fps
        self._frame_timer.setInterval(1000 // fps)

    def set_show_fps(self, shown: bool) -> None:
        self.canvas.show_fps = self.settings.ui.show_fps = shown
        self.canvas.update()

    def set_water_style(self, style: str) -> None:
        self.canvas.set_water_style(style)
        self.settings.ui.water_style = self.canvas.water_style
        self.chart_tab.water_changed()
        self.toast("Natural water" if self.canvas.natural_water else "Depth chart colours", 1.5)

    def _toggle_hud(self) -> None:
        self.chart_area.instruments_visible(self.hud_button.isChecked())
        self.settings.ui.show_hud = self.hud_button.isChecked()

    def _set_home(self) -> None:
        self.vehicle.set_home_here()

    def frame_mission(self) -> None:
        with self.vehicle.lock:
            points = self.vehicle.mission.positions
            if self.vehicle.mission.area is not None:
                points = points + self.vehicle.mission.area.corners()
        if points:
            self.canvas.frame(points)
            self.follow_button.setChecked(False)

    def boat_position(self) -> LatLon | None:
        nav = self.vehicle.nav()
        t = nav.telemetry
        return LatLon(t.lat, t.lon) if t is not None and t.has_position else None

    def mission_edited(self) -> None:
        self.mission_tab.refresh()
        self.canvas.update()

    def area_edited(self) -> None:
        mission = self.vehicle.mission
        mission.drift_ms = None   # drawn by hand: fixed to the chart, not the water
        if mission.area is not None:
            blocker = QtCore.QSignalBlocker(self.mission_tab.angle)
            self.mission_tab.angle.setValue(mission.area.angle_deg)
            del blocker
        self.regenerate_route()

    def regenerate_route(self) -> None:
        with self.vehicle.lock:
            changed = self.vehicle.mission.regenerate()
        if changed:
            self.mission_edited()

    def set_chart(self, grid: DepthGrid) -> None:
        self.chart_grid = grid
        self.canvas.chart = ChartLayer(grid, self.settings.safety.shallow_alarm_m)
        self.link.world.chart = grid
        self.canvas.update()
        self.toast(f"Depth chart loaded: {grid.source}")

    def depth_alarms_changed(self) -> None:
        safety = self.settings.safety
        if self.canvas.chart is not None:
            self.canvas.chart.set_shallow_alarm(safety.shallow_alarm_m)
        self.chart_area.sounder.shallow_alarm_m = safety.shallow_alarm_m
        self.chart_area.sounder.shallow_stop_m = safety.shallow_stop_m
        self.canvas.update()

    # ------------------------------------------------------------------
    # Input
    # ------------------------------------------------------------------

    KEY_NAMES = {
        Qt.Key_W: "W", Qt.Key_A: "A", Qt.Key_S: "S", Qt.Key_D: "D", Qt.Key_Up: "Up",
        Qt.Key_Down: "Down", Qt.Key_Left: "Left", Qt.Key_Right: "Right", Qt.Key_Shift: "Shift",
    }

    def eventFilter(self, obj, event) -> bool:
        kind = event.type()
        if kind in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease):
            focus = QtWidgets.QApplication.focusWidget()
            typing = isinstance(focus, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                                        QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit))
            if typing:
                return False
            key = event.key()
            name = self.KEY_NAMES.get(key)
            if name is not None:
                if not event.isAutoRepeat():
                    if kind == QtCore.QEvent.KeyPress:
                        self._keys.add(name)
                    else:
                        self._keys.discard(name)
                return True
            if kind == QtCore.QEvent.KeyPress and not event.isAutoRepeat():
                return self._key_command(key)
        return False

    def _key_command(self, key: int) -> bool:
        number_modes = {getattr(Qt, f"Key_{k}"): m for m, _t, k in MODE_BUTTONS}
        if key in number_modes:
            self.request_mode(number_modes[key])
        elif key == Qt.Key_Space:
            self.request_mode(Mode.STOP)
        elif key in (Qt.Key_Return, Qt.Key_Enter):
            self.vehicle.toggle_arm()
        elif key in (Qt.Key_Plus, Qt.Key_Equal):
            self.canvas.zoom(1)
        elif key == Qt.Key_Minus:
            self.canvas.zoom(-1)
        elif key == Qt.Key_F:
            self.toggle_follow()
        elif key == Qt.Key_Tab:
            self.toggle_drawer()
        elif key == Qt.Key_T:
            self.toggle_telemetry()
        elif key == Qt.Key_V:
            self.cycle_view()
        elif key == Qt.Key_K:
            self.vehicle.toggle_skimmer()
        elif key == Qt.Key_G:
            self.toggle_vectors()
        elif key == Qt.Key_N:
            self.set_water_style("chart" if self.canvas.natural_water else "natural")
        elif key == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
        elif key in (Qt.Key_Delete, Qt.Key_Backspace):
            self.canvas.delete_selected()
        elif key == Qt.Key_Escape:
            self.set_tool("pan")
        else:
            return False
        return True

    def _poll_input(self) -> None:
        control = self.settings.control
        pad_state = self.pad.state() if self.pad is not None else gamepad.PadState()
        helm = helm_from_pad(pad_state, control.stick_mode, control.deadzone, control.expo)
        keyboard = helm_from_keys(self._keys, control.stick_mode)
        if keyboard.deflection > helm.deflection:
            helm = keyboard
        self.vehicle.set_operator(port=helm.port, stbd=helm.stbd, throttle=helm.throttle,
                                  steer=helm.steer, deflection=helm.deflection,
                                  connected=pad_state.connected or bool(self._keys))
        self.status.pad = pad_state.name[:24] if pad_state.connected else ""
        self._pad_state, self._last_helm = pad_state, helm

        if self.pad is None:
            return
        while True:
            try:
                button, pressed = self.pad.edges.get_nowait()
            except queue.Empty:
                break
            if pressed:
                self._held_since[button] = time.monotonic()
            else:
                started = self._held_since.pop(button, None)
                if button in self._fired_hold:
                    self._fired_hold.discard(button)
                elif started is not None:
                    self._button(button)
        now = time.monotonic()
        for button, started in list(self._held_since.items()):
            if button in ("MENU", "VIEW") and button not in self._fired_hold and now - started > HOLD_PRESS_S:
                self._fired_hold.add(button)
                self._button_hold(button)

    def _button(self, button: str) -> None:
        mode = self.vehicle.mode
        if button == "A":
            self.start_mission()
        elif button == "B":
            self.request_mode(Mode.STOP)
        elif button == "X":
            cycle = [Mode.MANUAL, Mode.SPEED_HOLD, Mode.COURSE_HOLD]
            following = cycle[(cycle.index(mode) + 1) % 3] if mode in cycle else Mode.MANUAL
            self.request_mode(following)
        elif button == "Y":
            self.request_mode(Mode.HOLD)
        elif button == "VIEW":
            self.toggle_drawer()
        elif button == "MENU":
            self.toast("Hold ☰ to arm or disarm")
        elif button == "LB":
            self.canvas.zoom(-1)
        elif button == "RB":
            self.canvas.zoom(1)
        elif button == "R3":
            self.cycle_view()
        elif button == "L3":
            self.vehicle.toggle_skimmer()
        elif button in ("UP", "DOWN"):
            setpoint = self.vehicle.adjust_speed(0.1 if button == "UP" else -0.1)
            self.toast(f"Speed setpoint {units.knots(setpoint):.1f} kn", 1.2)
        elif button in ("LEFT", "RIGHT"):
            if mode is Mode.COURSE_HOLD:
                self.vehicle.adjust_course(5.0 if button == "RIGHT" else -5.0)
            else:
                step = 1 if button == "RIGHT" else -1
                if not self.drawer.isVisible():
                    self.toggle_drawer()
                self.drawer.setCurrentIndex((self.drawer.currentIndex() + step) % self.drawer.count())

    def _button_hold(self, button: str) -> None:
        if button == "MENU":
            self.vehicle.toggle_arm()
        elif button == "VIEW":
            self.request_mode(Mode.RETURN_HOME)

    # ------------------------------------------------------------------
    # Refresh
    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        while True:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            if event.kind.value == "telemetry":
                continue
            self.log_tab.add(event)
            if event.level >= Level.WARNING or event.kind.value in ("mode", "arming_state"):
                self.toast(event.message, 3.5 if event.level >= Level.WARNING else 2.0)

        nav = self.vehicle.nav()
        t = nav.telemetry
        if t is not None and t.has_position:
            self.canvas.time_scale = self.settings.sim.time_scale if self.link.simulated else 1.0
            self.canvas.set_boat(t.lat, t.lon, t.heading_deg or 0.0, t.cog_deg, t.sog_ms,
                                 t.roll_deg or 0.0, t.rot_dps)
            self.canvas.vectors = {
                "thrust": (t.throttle_port + t.throttle_stbd) / 2.0,
                "turn": (t.throttle_port - t.throttle_stbd) / 2.0,
                "wind": nav.true_wind,
                "current": nav.current,
            }
            self.canvas.skimmer_deployed = t.skimmer.deployed
        if self.link.simulated and self.canvas.show_oil:
            self.canvas.oil = self.link.spill_snapshot()
        self.canvas.home = nav.home
        self.canvas.target = nav.target
        self.canvas.target_label = nav.target_label
        self.canvas.active_index = nav.waypoint_index
        self.canvas.track = self.vehicle.track
        self.canvas.station = self.link.station
        self.canvas.survey.refresh(self.vehicle.lock)
        self.follow_button.setChecked(self.canvas.follow)
        self.skim_button.setChecked(nav.skimmer_commanded)
        self.chart_area.zoom.refresh()
        if self.drawer.isVisible() and self.drawer.currentIndex() == self.pages["🎮 Pad"]:
            self.controls_page.update_state(getattr(self, "_pad_state", gamepad.PadState()),
                                            self._keys, self._last_helm)

        self.status.nav = nav
        self.status.time_scale = self.settings.sim.time_scale
        self.status.update()
        for widget in self.chart_area.instruments:
            widget.set_nav(nav)
        self.chart_area.alarm.set_alarms(nav.alarms)
        self.chart_area.alarm.raise_()

        self.arm_button.setChecked(nav.armed)
        self.arm_button.setText("ARMED\n(disarm)" if nav.armed else "ARM")
        button = self.mode_buttons.get(nav.mode)
        if button is not None and not button.isChecked():
            button.setChecked(True)
        if t is not None and t.has_position:
            self.position_label.setText(f"{dms(t.lat, 'N', 'S')}\n{dms(t.lon, 'E', 'W')}")

    def _refresh_slow(self) -> None:
        nav = self.vehicle.nav()
        self.telemetry_panel.update_from(nav)
        self.status.gcs_battery = self._gcs_battery.level
        if self.drawer.isVisible():
            page = self.drawer.currentIndex()
            if page == self.pages["Mission"]:
                self.mission_tab.refresh()
            elif page == self.pages["Depth"]:
                self.depth_page.refresh(nav)
            elif page == self.pages["Power"]:
                self.power_page.refresh(nav)
            elif page == self.pages["Oil"]:
                self.oil_page.refresh(nav)

    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        self._frame_timer.stop()
        self._gcs_battery.stop()
        self._input_timer.stop()
        self.settings.ui.drawer_page = self.drawer.currentIndex()
        self.vehicle.stop()
        if self.pad is not None:
            self.pad.stop()
        try:
            config.save(self.settings)
        except OSError as exc:
            log.warning("could not save settings: %s", exc)
        self._autosave_survey()
        super().closeEvent(event)

    def _autosave_survey(self) -> None:
        soundings = self.vehicle.soundings
        if not soundings.soundings:
            return
        folder = config.state_dir() / "surveys"
        try:
            folder.mkdir(parents=True, exist_ok=True)
            soundings.export_xyz(folder / time.strftime("autosave-%Y%m%d-%H%M%S.csv"))
        except OSError as exc:
            log.warning("could not autosave the survey: %s", exc)


def load_chart(settings: config.Settings) -> DepthGrid | None:
    path = Path(settings.map.depth_grid).expanduser() if settings.map.depth_grid else BUNDLED_CHART
    try:
        return DepthGrid.load(path)
    except (OSError, ValueError, KeyError) as exc:
        log.warning("no depth chart (%s): %s", path, exc)
        return None


def run(settings: config.Settings, fullscreen: bool | None = None) -> int:
    QtWidgets.QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setApplicationName("usvgs")
    tiles = TiledMap()
    index = find_map_index(settings.map.vector_index)
    if index is not None:
        try:
            tiles.load(index)
        except (OSError, ValueError, KeyError) as exc:
            log.warning("offline map %s unusable: %s", index, exc)
    window = MainWindow(settings, load_chart(settings), tiles)
    screen = app.primaryScreen().availableGeometry()
    if fullscreen is None:
        # The Deck's panel is 1280×800; fill it.  Elsewhere, a window of that size.
        fullscreen = screen.width() <= 1280 and screen.height() <= 800
    if fullscreen:
        window.showFullScreen()
    else:
        window.resize(min(1280, screen.width()), min(800, screen.height()))
        window.show()
    window.canvas.zoom_to_metres(600)
    return app.exec_()

