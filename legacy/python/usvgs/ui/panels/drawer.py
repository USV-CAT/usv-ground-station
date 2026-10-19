"""The pull-out panel on the right: Mission, Chart, Sim, Log and Help tabs."""

from __future__ import annotations

import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

from ...config import state_dir
from ...core import geo, units
from ...core.bathymetry import DepthGrid
from ...core.geo import LatLon
from ...core.mission import Mission, PatternSpec
from ...input.mapping import BUTTON_HELP
from .. import theme


def _spin(low: float, high: float, value: float, step: float, suffix: str, decimals: int = 1):
    box = QtWidgets.QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    box.setSuffix(suffix)
    box.setKeyboardTracking(False)
    return box


def _form() -> tuple[QtWidgets.QWidget, QtWidgets.QFormLayout]:
    widget = QtWidgets.QWidget()
    form = QtWidgets.QFormLayout(widget)
    form.setContentsMargins(8, 8, 8, 8)
    form.setVerticalSpacing(6)
    form.setLabelAlignment(Qt.AlignLeft)
    return widget, form


def _heading(text: str) -> QtWidgets.QLabel:
    label = QtWidgets.QLabel(text)
    label.setObjectName("heading")
    return label


class MissionTab(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        tools = QtWidgets.QHBoxLayout()
        self.tool_buttons = {}
        for tool, text in (("points", "＋ Points"), ("rect", "▭ Rect"), ("goto", "◎ Go to")):
            button = QtWidgets.QPushButton(text)
            button.setCheckable(True)
            button.clicked.connect(lambda checked, t=tool: window.set_tool(t if checked else "pan"))
            tools.addWidget(button)
            self.tool_buttons[tool] = button
        layout.addLayout(tools)

        layout.addWidget(_heading("RECTANGLE PATTERN"))
        form_widget, form = _form()
        spec = window.vehicle.mission.pattern
        self.pattern = QtWidgets.QComboBox()
        self.pattern.addItems(["Lawnmower (survey lanes)", "Perimeter (round the edge)"])
        self.spacing = _spin(1, 500, spec.spacing_m, 1, " m", 0)
        self.margin = _spin(0, 100, spec.margin_m, 1, " m", 0)
        self.laps = QtWidgets.QSpinBox()
        self.laps.setRange(1, 20)
        self.laps.setValue(spec.laps)
        self.angle = _spin(0, 359, 90, 5, "°", 0)
        self.along_long = QtWidgets.QCheckBox("Lanes along long side")
        self.along_long.setChecked(spec.along_long_side)
        self.speed = _spin(0.2, 4.5, units.knots(window.settings.autopilot.cruise_speed_ms), 0.1, " kn")
        self.loop = QtWidgets.QCheckBox("Loop the route")
        form.addRow("Pattern", self.pattern)
        form.addRow("Lane spacing", self.spacing)
        form.addRow("Edge margin", self.margin)
        form.addRow("Perimeter laps", self.laps)
        form.addRow("Rotation", self.angle)
        form.addRow("", self.along_long)
        form.addRow("Cruise speed", self.speed)
        form.addRow("", self.loop)
        layout.addWidget(form_widget)

        for widget in (self.spacing, self.margin, self.angle):
            widget.valueChanged.connect(self._pattern_changed)
        self.laps.valueChanged.connect(self._pattern_changed)
        self.pattern.currentIndexChanged.connect(self._pattern_changed)
        self.along_long.toggled.connect(self._pattern_changed)
        self.speed.valueChanged.connect(self._speed_changed)
        self.loop.toggled.connect(self._loop_changed)

        self.stats = QtWidgets.QLabel()
        self.stats.setWordWrap(True)
        self.stats.setObjectName("dim")
        layout.addWidget(self.stats)

        self.list = QtWidgets.QListWidget()
        self.list.setMinimumHeight(110)
        self.list.currentRowChanged.connect(self._row_selected)
        layout.addWidget(self.list, 1)

        rows = [
            [("Delete point", window.canvas.delete_selected), ("Reverse", self._reverse),
             ("Clear", self._clear)],
            [("Open…", self._open), ("Save…", self._save), ("Frame", window.frame_mission)],
            [("▶ Start", window.start_mission), ("⟲ Restart", window.restart_mission),
             ("Skip ⏭", lambda: window.vehicle.skip_waypoint(1))],
        ]
        for row in rows:
            line = QtWidgets.QHBoxLayout()
            for text, slot in row:
                button = QtWidgets.QPushButton(text)
                button.clicked.connect(slot)
                line.addWidget(button)
            layout.addLayout(line)

    # -- edits --------------------------------------------------------------

    def spec(self) -> PatternSpec:
        return PatternSpec(
            pattern="perimeter" if self.pattern.currentIndex() == 1 else "lawnmower",
            spacing_m=self.spacing.value(),
            margin_m=self.margin.value(),
            along_long_side=self.along_long.isChecked(),
            laps=self.laps.value(),
        )

    def _pattern_changed(self) -> None:
        mission = self.window.vehicle.mission
        with self.window.vehicle.lock:
            mission.pattern = self.spec()
            if mission.area is not None and abs(mission.area.angle_deg - self.angle.value()) > 0.5:
                mission.area.angle_deg = self.angle.value()
        self.window.canvas.rect_angle_deg = self.angle.value()
        self.window.regenerate_route()

    def _speed_changed(self, knots: float) -> None:
        self.window.settings.autopilot.cruise_speed_ms = units.from_knots(knots)
        self.refresh()

    def _loop_changed(self, on: bool) -> None:
        self.window.vehicle.mission.loop = on
        self.window.canvas.update()

    def _reverse(self) -> None:
        with self.window.vehicle.lock:
            self.window.vehicle.mission.waypoints.reverse()
        self.window.mission_edited()

    def _clear(self) -> None:
        with self.window.vehicle.lock:
            self.window.vehicle.mission.clear()
        self.window.mission_edited()

    def _open(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open mission", str(state_dir() / "missions"), "Missions (*.json)")
        if not path:
            return
        try:
            mission = Mission.load(path)
        except (OSError, ValueError, KeyError) as exc:
            QtWidgets.QMessageBox.warning(self, "Open mission", f"Could not read it:\n{exc}")
            return
        self.window.vehicle.set_mission(mission)
        self.window.canvas.set_mission(mission)
        self.sync_from_mission()
        self.window.mission_edited()
        self.window.frame_mission()

    def _save(self) -> None:
        folder = state_dir() / "missions"
        folder.mkdir(parents=True, exist_ok=True)
        suggested = folder / time.strftime("mission-%Y%m%d-%H%M.json")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save mission", str(suggested),
                                                        "Missions (*.json)")
        if path:
            self.window.vehicle.mission.save(path)
            self.window.toast(f"Mission saved to {Path(path).name}")

    def _row_selected(self, row: int) -> None:
        if row >= 0 and row != self.window.canvas.selected:
            self.window.canvas.selected = row
            self.window.canvas.update()

    # -- display ------------------------------------------------------------

    def sync_from_mission(self) -> None:
        mission = self.window.vehicle.mission
        spec = mission.pattern
        blockers = [QtCore.QSignalBlocker(w) for w in (self.pattern, self.spacing, self.margin,
                                                        self.laps, self.along_long, self.loop,
                                                        self.angle)]
        self.pattern.setCurrentIndex(1 if spec.pattern == "perimeter" else 0)
        self.spacing.setValue(spec.spacing_m)
        self.margin.setValue(spec.margin_m)
        self.laps.setValue(spec.laps)
        self.along_long.setChecked(spec.along_long_side)
        self.loop.setChecked(mission.loop)
        if mission.area is not None:
            self.angle.setValue(mission.area.angle_deg)
        del blockers

    def set_tool(self, tool: str) -> None:
        for name, button in self.tool_buttons.items():
            button.setChecked(name == tool)

    def refresh(self) -> None:
        mission = self.window.vehicle.mission
        with self.window.vehicle.lock:
            waypoints = list(mission.waypoints)
            area = mission.area
        self.list.blockSignals(True)
        self.list.clear()
        previous = None
        for index, w in enumerate(waypoints):
            leg = "" if previous is None else f" · {geo.distance_m(previous, w.position):.0f} m"
            self.list.addItem(f"{index + 1:>3}   {w.lat:.5f}, {w.lon:.5f}{leg}")
            previous = w.position
        if 0 <= self.window.canvas.selected < self.list.count():
            self.list.setCurrentRow(self.window.canvas.selected)
        self.list.blockSignals(False)

        boat = self.window.boat_position()
        cruise = self.window.settings.autopilot.cruise_speed_ms
        lines = [f"{len(waypoints)} waypoints · {units.format_distance(mission.length_m(boat))}"
                 f" · ETA {units.format_duration(mission.duration_s(cruise, boat)) if waypoints else '—'}"]
        if area is not None:
            lines.append(f"Area {area.width_m:.0f} × {area.height_m:.0f} m "
                         f"({area.area_m2 / 10_000:.2f} ha) at {area.angle_deg:.0f}°")
        self.stats.setText("\n".join(lines))


class ChartTab(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        canvas = window.canvas
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(_heading("WATER"))
        line = QtWidgets.QHBoxLayout()
        self.water_buttons = {}
        for style, text in (("natural", "Natural water"), ("chart", "Depth chart colours")):
            button = QtWidgets.QPushButton(text)
            button.setCheckable(True)
            button.clicked.connect(lambda _on, s=style: window.set_water_style(s))
            line.addWidget(button)
            self.water_buttons[style] = button
        layout.addLayout(line)

        layout.addWidget(_heading("DISPLAY"))
        line = QtWidgets.QHBoxLayout()
        self.fps_buttons = {}
        for fps in (60, 30):
            button = QtWidgets.QPushButton(f"{fps} fps")
            button.setCheckable(True)
            button.setChecked(window.settings.ui.max_fps == fps)
            button.clicked.connect(lambda _on, f=fps: self._fps_chosen(f))
            line.addWidget(button)
            self.fps_buttons[fps] = button
        self.fps_box = QtWidgets.QCheckBox("Show FPS")
        self.fps_box.setChecked(window.settings.ui.show_fps)
        self.fps_box.toggled.connect(window.set_show_fps)
        line.addWidget(self.fps_box)
        layout.addLayout(line)

        layout.addWidget(_heading("LAYERS"))
        for text, attribute in (("Depth chart (EMODnet)", "show_chart"),
                                ("Isobaths and spot depths", "show_isobaths"),
                                ("Surveyed depth (your soundings)", "show_survey"),
                                ("Street map (OpenStreetMap)", "show_osm"),
                                ("Track", "show_track"),
                                ("Leg lengths", "show_leg_lengths")):
            box = QtWidgets.QCheckBox(text)
            box.setChecked(getattr(canvas, attribute))
            box.toggled.connect(lambda on, a=attribute: (setattr(canvas, a, on), canvas.update()))
            layout.addWidget(box)

        layout.addWidget(_heading("DEPTH ALARMS"))
        form_widget, form = _form()
        safety = window.settings.safety
        self.alarm = _spin(0.1, 20, safety.shallow_alarm_m, 0.1, " m")
        self.stop = _spin(0.0, 20, safety.shallow_stop_m, 0.1, " m")
        self.alarm.valueChanged.connect(self._alarms_changed)
        self.stop.valueChanged.connect(self._alarms_changed)
        form.addRow("Shallow alarm", self.alarm)
        form.addRow("Auto stop below", self.stop)
        layout.addWidget(form_widget)

        layout.addWidget(_heading("SURVEY DATA"))
        for row in ((("Export soundings…", self._export_xyz), ("Export grid…", self._export_grid)),
                    (("Clear survey", self._clear_survey), ("Clear track", window.vehicle.clear_track)),
                    (("Load depth chart…", self._load_chart),)):
            line = QtWidgets.QHBoxLayout()
            for text, slot in row:
                button = QtWidgets.QPushButton(text)
                button.clicked.connect(slot)
                line.addWidget(button)
            layout.addLayout(line)

        self.info = QtWidgets.QLabel()
        self.info.setWordWrap(True)
        self.info.setObjectName("dim")
        layout.addWidget(self.info)
        self.cursor = QtWidgets.QLabel("Move over the chart for the depth there")
        self.cursor.setWordWrap(True)
        layout.addWidget(self.cursor)
        layout.addStretch(1)
        self.describe_chart()

    def _fps_chosen(self, fps: int) -> None:
        self.window.set_max_fps(fps)
        for value, button in self.fps_buttons.items():
            button.setChecked(value == self.window.settings.ui.max_fps)

    def water_changed(self) -> None:
        for style, button in self.water_buttons.items():
            button.setChecked(style == self.window.canvas.water_style)

    def describe_chart(self) -> None:
        chart = self.window.chart_grid
        if chart is None:
            self.info.setText("No depth chart loaded — the simulator uses an open lake.")
            return
        cell = chart.dlat * 111_320
        self.info.setText(f"Chart: {chart.source}\n{chart.shape[1]} × {chart.shape[0]} cells, "
                          f"≈{cell:.0f} m each. ~ marks an estimated depth (inland lakes).")

    def show_cursor(self, lat: float, lon: float) -> None:
        chart = self.window.chart_grid
        text = f"{lat:.5f}°N  {lon:.5f}°E"
        if chart is not None:
            depth = chart.depth_at(lat, lon)
            text += "   land" if depth != depth else f"   chart {depth:.1f} m"
        grid = self.window.vehicle.soundings
        with self.window.vehicle.lock:
            surveyed = grid.depth_of(grid.cell_of(lat, lon))
        if surveyed is not None:
            text += f" · surveyed {surveyed:.2f} m"
        if text != self.cursor.text():
            self.cursor.setText(text)

    def _alarms_changed(self) -> None:
        safety = self.window.settings.safety
        safety.shallow_alarm_m = self.alarm.value()
        safety.shallow_stop_m = min(self.stop.value(), self.alarm.value())
        self.window.depth_alarms_changed()

    def _export(self, title: str, stem: str, write) -> None:
        folder = state_dir() / "surveys"
        folder.mkdir(parents=True, exist_ok=True)
        suggested = folder / time.strftime(f"{stem}-%Y%m%d-%H%M.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, title, str(suggested), "CSV (*.csv *.xyz)")
        if path:
            with self.window.vehicle.lock:
                count = write(path)
            self.window.toast(f"Wrote {count} rows to {Path(path).name}")

    def _export_xyz(self) -> None:
        self._export("Export soundings", "soundings", self.window.vehicle.soundings.export_xyz)

    def _export_grid(self) -> None:
        self._export("Export depth grid", "depth-grid", self.window.vehicle.soundings.export_grid_csv)

    def _clear_survey(self) -> None:
        if QtWidgets.QMessageBox.question(self, "Clear survey",
                                          "Discard every sounding taken so far?") \
                == QtWidgets.QMessageBox.Yes:
            with self.window.vehicle.lock:
                self.window.vehicle.soundings.clear()

    def _load_chart(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load depth chart", "",
            "Depth grids (*.npz *.asc *.tif *.tiff *.xyz *.csv)")
        if not path:
            return
        try:
            grid = DepthGrid.load(path)
        except (OSError, ValueError, KeyError) as exc:
            QtWidgets.QMessageBox.warning(self, "Load depth chart", f"Could not read it:\n{exc}")
            return
        self.window.settings.map.depth_grid = path
        self.window.set_chart(grid)
        self.describe_chart()


class SimTab(QtWidgets.QWidget):
    def __init__(self, window) -> None:
        super().__init__()
        self.window = window
        sim = window.settings.sim
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        if not window.link.simulated:
            layout.addWidget(QtWidgets.QLabel("Connected to a real boat — no simulator controls."))
            layout.addStretch(1)
            return
        layout.addWidget(_heading("WEATHER"))
        form_widget, form = _form()
        self.wind = _spin(0, 20, sim.wind_speed_ms, 0.5, " m/s")
        self.wind_dir = _spin(0, 359, sim.wind_from_deg, 10, "° from", 0)
        self.current = _spin(0, 2, sim.current_speed_ms, 0.05, " m/s", 2)
        self.current_dir = _spin(0, 359, sim.current_to_deg, 10, "° to", 0)
        form.addRow("Wind", self.wind)
        form.addRow("Wind direction", self.wind_dir)
        form.addRow("Current", self.current)
        form.addRow("Current direction", self.current_dir)
        layout.addWidget(form_widget)
        for widget in (self.wind, self.wind_dir, self.current, self.current_dir):
            widget.valueChanged.connect(self._weather_changed)

        layout.addWidget(_heading("GAME"))
        form_widget, form = _form()
        self.speed = QtWidgets.QComboBox()
        self.speed.addItems(["×1", "×2", "×4", "×8"])
        self.speed.setCurrentIndex({1: 0, 2: 1, 4: 2, 8: 3}.get(int(sim.time_scale), 0))
        self.speed.currentIndexChanged.connect(self._speed_changed)
        self.range = _spin(100, 20000, sim.link_range_m, 100, " m", 0)
        self.range.valueChanged.connect(lambda v: setattr(sim, "link_range_m", v))
        self.noise = _spin(0, 10, sim.gps_noise_m, 0.1, " m")
        self.noise.valueChanged.connect(lambda v: setattr(sim, "gps_noise_m", v))
        form.addRow("Time", self.speed)
        form.addRow("Radio range", self.range)
        form.addRow("GNSS noise", self.noise)
        layout.addWidget(form_widget)

        for row in ((("Respawn at launch", self._respawn), ("Launch here", self._launch_here)),
                    (("Recharge battery", window.link.boat.recharge), ("Pause", self._pause))):
            line = QtWidgets.QHBoxLayout()
            for text, slot in row:
                button = QtWidgets.QPushButton(text)
                button.clicked.connect(slot)
                if text == "Pause":
                    button.setCheckable(True)
                    self.pause_button = button
                line.addWidget(button)
            layout.addLayout(line)
        note = QtWidgets.QLabel(
            "“Launch here” moves the launch point (and the ground station) to the middle of the "
            "chart view. Survey a rectangle and watch the real seabed appear under the track.")
        note.setWordWrap(True)
        note.setObjectName("dim")
        layout.addWidget(note)
        layout.addStretch(1)

    def _weather_changed(self) -> None:
        sim = self.window.settings.sim
        sim.wind_speed_ms = self.wind.value()
        sim.wind_from_deg = self.wind_dir.value()
        sim.current_speed_ms = self.current.value()
        sim.current_to_deg = self.current_dir.value()
        self.window.link.apply_environment()

    def _speed_changed(self, index: int) -> None:
        self.window.settings.sim.time_scale = (1.0, 2.0, 4.0, 8.0)[index]

    def _respawn(self) -> None:
        self.window.vehicle.disarm()
        self.window.link.respawn()
        self.window.vehicle.clear_track()
        self.window.toast("Boat back at the launch point, battery full")

    def _launch_here(self) -> None:
        canvas = self.window.canvas
        target = LatLon(canvas.centre_lat, canvas.centre_lon)
        link = self.window.link
        if not link.world.depth_at(*target) > self.window.settings.boat.draft_m + 0.3:
            self.window.toast("That is not deep enough to launch — choose open water")
            return
        sim = self.window.settings.sim
        sim.start_lat, sim.start_lon = target.lat, target.lon
        link.station = target
        self.window.vehicle.disarm()
        link.respawn(target)
        self.window.vehicle.clear_track()
        self.window.vehicle.set_home_here()
        self.window.toast("Launched here; home and the ground station moved too")

    def _pause(self, on: bool) -> None:
        self.window.link.paused = on


class LogTab(QtWidgets.QListWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWordWrap(True)

    def add(self, event) -> None:
        stamp = time.strftime("%H:%M:%S", time.localtime(event.timestamp))
        item = QtWidgets.QListWidgetItem(f"{stamp}  {event.message}")
        item.setForeground(QtGui.QColor(theme.severity_colour(int(event.level))))
        self.addItem(item)
        if self.count() > 500:
            self.takeItem(0)
        self.scrollToBottom()


class HelpTab(QtWidgets.QTextBrowser):
    def __init__(self) -> None:
        super().__init__()
        rows = "".join(f"<tr><td><b>{k}</b></td><td>{v}</td></tr>" for k, v in BUTTON_HELP)
        self.setHtml(f"""
        <h3 style="color:{theme.ACCENT}">Steam Deck controls</h3>
        <p>Arcade sticks: <b>left stick</b> or <b>R2/L2</b> throttle, <b>right stick</b> steer.
        Tank sticks: left = port motor, right = starboard motor.</p>
        <table cellpadding="3">{rows}</table>
        <h3 style="color:{theme.ACCENT}">Keyboard</h3>
        <p><b>W/S</b> or ↑/↓ throttle · <b>A/D</b> or ←/→ steer · <b>Shift</b> boost ·
        <b>Space</b> USV stop · <b>Enter</b> arm/disarm · <b>1–7</b> modes ·
        <b>+/−</b> zoom · <b>F</b> follow · <b>V</b> view (N↑ / H↑ / 3D) · <b>G</b> vectors ·
        <b>N</b> natural water / depth chart · <b>K</b> skimmer · <b>Tab</b> right panel · <b>T</b> telemetry panel · <b>F11</b> full screen</p>
        <h3 style="color:{theme.ACCENT}">Chart</h3>
        <p>Pinch with two fingers, or hold the big <b>＋/－</b> buttons, to zoom. The ◀ ▶ tabs on the
        chart's edges open and close the side panels. Vectors at the boat: <b>orange</b> thrust (and an
        arc for the turning moment), <b>cyan</b> true wind, <b>violet dashed</b> current,
        <b>green</b> course over ground for 30 s, <b>yellow dashed</b> heading.</p>
        <h3 style="color:{theme.ACCENT}">Planning</h3>
        <p><b>Points</b>: tap the water to add a waypoint, drag to move, right-click to delete.<br>
        <b>Rectangle</b>: drag out an area; drag a corner to resize, the middle to move, the yellow
        handle to rotate. The lawnmower route follows.<br>
        <b>Go to</b>: tap a point and the boat drives there and holds station.</p>
        <p>Moving a stick hard in any automatic mode takes manual control back.</p>
        """)
