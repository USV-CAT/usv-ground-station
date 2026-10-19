"""The telemetry table down the left of the screen.

Label on the left, value on the right, grouped the way a USV operator scans
them: position and attitude, propulsion, the link, power, the water, and the
navigation figures for wherever the boat is going.  A value with no reading
shows a dash rather than its last number.
"""

from __future__ import annotations

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt

from ...core import units
from .. import theme

FIX_NAMES = {0: "NO FIX", 2: "2D", 3: "3D", 4: "RTK FLT", 5: "RTK FIX"}


class TelemetryPanel(QtWidgets.QScrollArea):
    def __init__(self) -> None:
        super().__init__()
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.speed_units = "kn"
        body = QtWidgets.QWidget()
        self.setWidget(body)
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        self._values: dict[str, QtWidgets.QLabel] = {}

        sections = (
            ("NAVIGATION", [("gnss", "GNSS"), ("rtk", "RTK"), ("hdg", "Heading"), ("cog", "COG"),
                            ("sog", "SOG"), ("stw", "STW"), ("rot", "Rate of turn"),
                            ("roll", "Roll"), ("pitch", "Pitch")]),
            ("PROPULSION", [("thr_l", "Throttle (L)"), ("thr_r", "Throttle (R)"),
                            ("rpm_l", "RPM (L)"), ("rpm_r", "RPM (R)")]),
            ("LINK", [("down", "USV → GCS"), ("up", "GCS → USV"), ("rssi", "RSSI"),
                      ("range", "GCS – USV")]),
            ("POWER", [("volts", "Battery"), ("amps", "Current"), ("watts", "Power"),
                       ("soc", "Charge"), ("tte", "Time to empty"), ("used", "Energy used")]),
            ("ROUTE", [("target", "Target"), ("dtw", "Distance"), ("btw", "Bearing"),
                       ("xte", "Cross-track"), ("ttg", "Time to go"), ("home", "Home"),
                       ("trip", "Trip log"), ("runtime", "Run time")]),
            ("WATER", [("depth", "Depth in water"), ("wtemp", "Water temp"), ("ph", "pH"),
                       ("do", "DO"), ("o2", "O₂ sat"), ("ec", "EC"), ("tds", "TDS"),
                       ("sal", "Salinity"), ("orp", "ORP"), ("turb", "Turbidity")]),
            ("OIL SKIMMER", [("skim", "Skimmer"), ("tank", "Tank"), ("oil", "Oil recovered"),
                             ("oilppm", "Oil in water")]),
            ("VECTORS", [("tw", "True wind"), ("set", "Current")]),
            ("SURVEY", [("pings", "Soundings"), ("cover", "Area covered"),
                        ("aground", "Groundings")]),
        )
        for title, rows in sections:
            card = QtWidgets.QFrame()
            card.setObjectName("card")
            grid = QtWidgets.QGridLayout(card)
            grid.setContentsMargins(8, 6, 8, 6)
            grid.setHorizontalSpacing(6)
            grid.setVerticalSpacing(1)
            heading = QtWidgets.QLabel(title)
            heading.setObjectName("heading")
            grid.addWidget(heading, 0, 0, 1, 2)
            for row, (key, label) in enumerate(rows, start=1):
                name = QtWidgets.QLabel(label)
                name.setStyleSheet(f"color: {theme.TEXT}; background: transparent;")
                value = QtWidgets.QLabel("—")
                value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                value.setStyleSheet(self._value_style(theme.VALUE))
                grid.addWidget(name, row, 0)
                grid.addWidget(value, row, 1)
                self._values[key] = value
            layout.addWidget(card)
        layout.addStretch(1)

    @staticmethod
    def _value_style(colour: str) -> str:
        return (f"color: {colour}; background: transparent; font-weight: 700;"
                "font-family: 'DejaVu Sans Mono', monospace;")

    def _set(self, key: str, text: str, colour: str = theme.VALUE) -> None:
        label = self._values[key]
        if label.text() != text:
            label.setText(text)
        style = self._value_style(colour)
        if label.styleSheet() != style:
            label.setStyleSheet(style)

    def _speed(self, ms: float | None) -> str:
        if ms is None:
            return "—"
        if self.speed_units == "kn":
            return f"{units.knots(ms):.1f} kn"
        return f"{ms:.2f} m/s"

    def update_from(self, nav) -> None:
        t = nav.telemetry
        dim = theme.TEXT_DIM

        def f(value, fmt: str, suffix: str = "") -> str:
            return "—" if value is None else f"{value:{fmt}}{suffix}"

        if t is None:
            for key in self._values:
                self._set(key, "—", dim)
            return
        stale = not nav.link_ok
        colour = dim if stale else theme.VALUE

        fix_colour = theme.GOOD if t.gps_fix >= 3 else theme.BAD
        self._set("gnss", f"{t.satellites} sats · {FIX_NAMES.get(t.gps_fix, t.gps_fix)}",
                  dim if stale else fix_colour)
        self._set("rtk", "FIXED" if t.gps_fix == 5 else ("FLOAT" if t.gps_fix == 4 else "OFF"), colour)
        self._set("hdg", units.format_bearing(t.heading_deg), colour)
        self._set("cog", units.format_bearing(t.cog_deg), colour)
        self._set("sog", self._speed(t.sog_ms), colour)
        self._set("stw", self._speed(t.stw_ms), colour)
        self._set("rot", f(None if t.rot_dps is None else t.rot_dps * 60, "+.0f", " °/min"), colour)
        self._set("roll", f(t.roll_deg, "+.1f", "°"), colour)
        self._set("pitch", f(t.pitch_deg, "+.1f", "°"), colour)

        self._set("thr_l", f"{t.throttle_port * 100:+.0f} %", colour)
        self._set("thr_r", f"{t.throttle_stbd * 100:+.0f} %", colour)
        self._set("rpm_l", f"{t.rpm_port:.0f} rpm", colour)
        self._set("rpm_r", f"{t.rpm_stbd:.0f} rpm", colour)

        self._set("down", f"{nav.downlink_hz:.0f} Hz", theme.GOOD if nav.downlink_hz >= 5 else theme.BAD)
        self._set("up", f(t.uplink_hz, ".0f", " Hz"), colour)
        self._set("rssi", f(t.rssi_dbm, ".0f", " dBm"), colour)
        self._set("range", units.format_distance(nav.station_distance_m), colour)

        self._set("volts", f(t.battery_v, ".2f", " V"), colour)
        self._set("amps", f(t.battery_a, ".1f", " A"), colour)
        self._set("soc", f(t.battery_pct, ".0f", " %"),
                  dim if stale else theme.battery_colour(t.battery_pct))
        e = nav.energy
        self._set("watts", "—" if e is None else f"{e.power_w:.0f} W", colour)
        self._set("tte", "—" if e is None else ("∞" if e.time_to_empty_s == float("inf")
                                                 else units.format_duration(e.time_to_empty_s)), colour)
        self._set("used", "—" if e is None else f"{e.used_wh:.1f} Wh", colour)

        sk = t.skimmer
        if sk.fitted:
            state = "OFFLOAD" if sk.offloading else ("SKIMMING" if sk.recovering else
                                                     ("DEPLOYED" if sk.deployed else "STOWED"))
            self._set("skim", state, theme.GOOD if sk.recovering else colour)
            full = sk.tank_l >= sk.tank_capacity_l - 0.05
            self._set("tank", f"{sk.tank_l:.1f}/{sk.tank_capacity_l:.0f} L", theme.BAD if full else colour)
            self._set("oil", f"{sk.recovered_oil_l:.1f} L", colour)
            self._set("oilppm", f(sk.oil_ppm, ".0f", " ppm"), colour)
        else:
            for key in ("skim", "tank", "oil", "oilppm"):
                self._set(key, "—", dim)
        self._set("tw", "—" if nav.true_wind is None else
                  f"{nav.true_wind[0]:.1f} m/s {units.format_bearing(nav.true_wind[1])}", colour)
        self._set("set", "—" if nav.current is None else
                  f"{nav.current[0]:.2f} → {units.format_bearing(nav.current[1])}", colour)

        self._set("target", nav.target_label or "—", theme.TARGET if nav.target_label else dim)
        self._set("dtw", units.format_distance(nav.distance_to_target_m), colour)
        self._set("btw", units.format_bearing(nav.bearing_to_target_deg), colour)
        xte = nav.cross_track_m
        self._set("xte", "—" if xte is None else f"{abs(xte):.1f} m {'◀' if xte > 0 else '▶'}",
                  colour if xte is None or abs(xte) < 5 else theme.CAUTION)
        self._set("ttg", units.format_duration(nav.time_to_go_s), colour)
        self._set("home", "—" if nav.home_distance_m is None else
                  f"{units.format_distance(nav.home_distance_m)} {units.format_bearing(nav.home_bearing_deg)}",
                  colour)
        self._set("trip", units.format_distance(nav.trip_m), colour)
        self._set("runtime", units.format_duration(nav.run_time_s), colour)

        w = t.water
        depth_colour = theme.BAD if (t.depth_m is not None and any(a.key == "shallow" for a in nav.alarms)) \
            else colour
        self._set("depth", f(t.depth_m, ".2f", " m"), depth_colour)
        self._set("wtemp", f(w.temperature_c, ".1f", " °C"), colour)
        self._set("ph", f(w.ph, ".2f", " pH"), colour)
        self._set("do", f(w.dissolved_oxygen_mgl, ".2f", " mg/L"), colour)
        self._set("o2", f(w.oxygen_saturation_pct, ".1f", " %"), colour)
        self._set("ec", f(w.conductivity_ms, ".2f", " mS/cm"), colour)
        self._set("tds", f(w.tds_ppm, ".0f", " ppm"), colour)
        self._set("sal", f(w.salinity_psu, ".2f", " psu"), colour)
        self._set("orp", f(w.orp_mv, ".0f", " mV"), colour)
        self._set("turb", f(w.turbidity_ntu, ".1f", " NTU"), colour)

        self._set("pings", f"{nav.soundings}", colour)
        self._set("cover", f"{nav.coverage * 100:.0f} %" if nav.coverage else "—", colour)
        self._set("aground", f"{nav.groundings}", theme.BAD if nav.groundings else colour)
