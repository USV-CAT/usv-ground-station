"""The boat as the ground station sees it: the single source of truth.

:class:`Vehicle` owns the link, the latest telemetry, the mode, the mission,
the autopilot, the safety checks, the track and the survey.  It runs one
control loop at 20 Hz that decides what the motors should do and sends it.
The interface only reads :meth:`Vehicle.nav` and calls the command methods; it
holds no boat state of its own.
"""

from __future__ import annotations

import csv
import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from ..config import Settings, state_dir
from ..core import geo, vectors
from ..core.bathymetry import SoundingGrid
from ..core.energy import NOMINAL_V, EnergyEstimate, PowerSample, estimate
from ..core.events import EventBus, EventKind, Level
from ..core.geo import LatLon
from ..core.mission import Mission
from ..core.telemetry import Telemetry
from ..link.base import BoatLink
from .autopilot import Pilot, StationKeeper, line_of_sight, mix
from .modes import Mode

log = logging.getLogger(__name__)

CONTROL_HZ = 20.0
#: Track points are kept at least this far apart, metres.
TRACK_SPACING_M = 1.5
TRACK_LIMIT = 50_000


@dataclass(frozen=True)
class Alarm:
    key: str
    text: str
    level: Level


@dataclass(frozen=True)
class NavState:
    """Everything the displays need, computed once per control tick."""

    mode: Mode = Mode.MANUAL
    armed: bool = False
    link_ok: bool = False
    downlink_hz: float = 0.0
    telemetry: Telemetry | None = None
    home: LatLon | None = None
    home_distance_m: float | None = None
    home_bearing_deg: float | None = None
    station_distance_m: float | None = None
    #: Where the autopilot is going, and what it calls it.
    target: LatLon | None = None
    target_label: str = ""
    waypoint_index: int = -1
    waypoint_count: int = 0
    bearing_to_target_deg: float | None = None
    distance_to_target_m: float | None = None
    cross_track_m: float | None = None
    time_to_go_s: float | None = None
    mission_remaining_m: float | None = None
    speed_setpoint_ms: float = 0.0
    course_setpoint_deg: float | None = None
    #: What was sent to the motors this tick.
    command_port: float = 0.0
    command_stbd: float = 0.0
    alarms: tuple[Alarm, ...] = ()
    trip_m: float = 0.0
    run_time_s: float = 0.0
    coverage: float = 0.0
    soundings: int = 0
    groundings: int = 0
    manual_input: bool = False
    energy: EnergyEstimate | None = None
    #: Smoothed true wind (speed m/s, from degrees) and current (drift m/s, set degrees).
    true_wind: tuple[float, float] | None = None
    current: tuple[float, float] | None = None
    skimmer_commanded: bool = False


@dataclass
class _Operator:
    port: float = 0.0
    stbd: float = 0.0
    throttle: float = 0.0
    steer: float = 0.0
    deflection: float = 0.0
    connected: bool = False
    updated: float = field(default_factory=time.monotonic)


class Vehicle:
    def __init__(self, settings: Settings, link: BoatLink, bus: EventBus | None = None,
                 record: bool = True) -> None:
        self.settings = settings
        self.link = link
        self.bus = bus or EventBus()
        self.mission = Mission()
        self.pilot = Pilot()
        self.keeper = StationKeeper(settings.autopilot.hold_radius_m)

        self._lock = threading.RLock()
        #: Held while the mission or the survey is read or edited off this thread.
        self.lock = self._lock
        self._telemetry: Telemetry | None = None
        self._arrivals: deque[float] = deque(maxlen=64)
        self._operator = _Operator()
        self._mode = Mode.MANUAL
        self._armed = False
        self._home: LatLon | None = None
        self._hold_point: LatLon | None = None
        self._point_target: LatLon | None = None
        self._leg_start: LatLon | None = None
        self._wp_index = 0
        self._wp_hold_until: float | None = None
        self._speed_setpoint = settings.autopilot.cruise_speed_ms
        self._course_setpoint: float | None = None
        self._nav = NavState()
        self._alarms: dict[str, Alarm] = {}
        self._latched: set[str] = set()
        self._link_was_ok = False
        self._started = time.monotonic()
        self._armed_time = 0.0
        self._trip_m = 0.0
        self._last_track: Telemetry | None = None
        self.track: list[tuple[float, float, float, float | None]] = []
        self.track_version = 0

        origin = LatLon(settings.sim.start_lat, settings.sim.start_lon)
        self.soundings = SoundingGrid(origin, settings.survey.cell_m)
        self.groundings = 0
        self._was_grounded = False

        #: One sample a second of the battery, for the power page.
        self.power_history: deque[PowerSample] = deque(maxlen=4 * 3600)
        self._energy_wh = 0.0
        self._last_power_t: float | None = None
        self._last_energy: tuple[float, EnergyEstimate | None] = (-1.0, None)
        self._wind_vec: tuple[float, float] | None = None
        self._current_vec: tuple[float, float] | None = None
        self._skimmer = False

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._log_writer = None
        self._log_file = None
        self._last_log = 0.0
        self._record = record
        link.on_telemetry = self._on_telemetry

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self.link.open()
        if self._record:
            self._open_log()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="control", daemon=True)
        self._thread.start()
        self.bus.message(f"Ground station up — link {self.link.name}")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.link.send_command(0.0, 0.0, False)
        self.link.close()
        if self._log_file:
            self._log_file.close()
            self._log_file = None

    def _open_log(self) -> None:
        folder = state_dir() / "logs"
        try:
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / time.strftime("session-%Y%m%d-%H%M%S.csv")
            self._log_file = open(path, "w", newline="", encoding="utf-8")  # noqa: SIM115
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow([
                "time", "mode", "armed", "lat", "lon", "heading", "cog", "sog_ms", "depth_m",
                "battery_v", "battery_pct", "port", "stbd", "xte_m", "water_temp_c",
                "salinity_psu", "ph", "do_mgl",
            ])
            self.log_path: Path | None = path
        except OSError as exc:
            log.warning("session log disabled: %s", exc)
            self.log_path = None

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def _on_telemetry(self, telemetry: Telemetry) -> None:
        with self._lock:
            previous = self._telemetry
            self._telemetry = telemetry
            self._arrivals.append(telemetry.received)
            if telemetry.has_position:
                if self._home is None:
                    self._home = LatLon(telemetry.lat, telemetry.lon)
                    self.bus.message("Home set at first GNSS fix", Level.NOTICE)
                if telemetry.depth_m is not None:
                    self.soundings.add(telemetry.lat, telemetry.lon, telemetry.depth_m,
                                       self.settings.survey.swath_m)
                self._update_track(telemetry, previous)
            self._update_power(telemetry)
            self._update_vectors(telemetry)

    def _update_power(self, t: Telemetry) -> None:
        if t.boat_time_s is None or t.battery_v is None or t.battery_a is None:
            return
        last = self._last_power_t
        if last is not None and t.boat_time_s < last:
            last = None     # the boat restarted
        if last is not None:
            dt = t.boat_time_s - last
            if dt < 1.0:
                return
            self._energy_wh += t.battery_v * t.battery_a * dt / 3600.0
        self._last_power_t = t.boat_time_s
        self.power_history.append(PowerSample(t.boat_time_s, t.battery_v, t.battery_a,
                                              t.battery_pct or 0.0, t.sog_ms or 0.0,
                                              self._energy_wh))

    def _update_vectors(self, t: Telemetry) -> None:
        """Smoothed true wind and current, from the vector triangles."""
        if t.heading_deg is None or t.sog_ms is None:
            return
        alpha = 0.05
        if t.wind_apparent_ms is not None and t.wind_apparent_deg is not None:
            speed, source = vectors.true_wind(t.wind_apparent_ms, t.wind_apparent_deg,
                                              t.heading_deg, t.sog_ms, t.cog_deg)
            sample = vectors.from_polar(speed, source)
            self._wind_vec = sample if self._wind_vec is None else tuple(
                a + (b - a) * alpha for a, b in zip(self._wind_vec, sample, strict=True))
        if t.stw_ms is not None and t.cog_deg is not None and t.sog_ms > 0.15:
            sample = vectors.from_polar(*vectors.current(t.sog_ms, t.cog_deg, t.stw_ms, t.heading_deg))
            self._current_vec = sample if self._current_vec is None else tuple(
                a + (b - a) * alpha * 0.5 for a, b in zip(self._current_vec, sample, strict=True))

    def _update_track(self, telemetry: Telemetry, previous: Telemetry | None) -> None:
        last = self._last_track
        if last is not None:
            step = geo.distance_m((last.lat, last.lon), (telemetry.lat, telemetry.lon))
            if step < TRACK_SPACING_M:
                return
            if step < 50.0:
                self._trip_m += step
        self._last_track = telemetry
        self.track.append((time.time(), telemetry.lat, telemetry.lon, telemetry.depth_m))
        if len(self.track) > TRACK_LIMIT:
            del self.track[: TRACK_LIMIT // 10]
        self.track_version += 1

    def set_operator(self, *, port: float, stbd: float, throttle: float, steer: float,
                     deflection: float, connected: bool) -> None:
        """The operator's sticks, already mapped to motor and helm terms."""
        with self._lock:
            self._operator = _Operator(port, stbd, throttle, steer, deflection, connected)

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    @property
    def mode(self) -> Mode:
        return self._mode

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def home(self) -> LatLon | None:
        return self._home

    def arm(self) -> bool:
        with self._lock:
            t = self._telemetry
            if t is None or t.age() > self.settings.safety.link_timeout_s:
                self.bus.message("Cannot arm: no telemetry from the boat", Level.WARNING)
                return False
            if not t.has_position:
                self.bus.message("Cannot arm: no GNSS fix", Level.WARNING)
                return False
            if t.battery_pct is not None and t.battery_pct < self.settings.safety.battery_critical_pct:
                self.bus.message("Cannot arm: battery critical", Level.WARNING)
                return False
            self._armed = True
            self._set_mode(Mode.MANUAL, "armed")
            self.bus.emit(EventKind.ARMING_STATE, "ARMED — motors live", Level.NOTICE, armed=True)
            return True

    def disarm(self) -> None:
        with self._lock:
            if self._armed:
                self._armed = False
                self.bus.emit(EventKind.ARMING_STATE, "Disarmed", Level.NOTICE, armed=False)

    def toggle_arm(self) -> None:
        if self._armed:
            self.disarm()
        else:
            self.arm()

    def set_mode(self, mode: Mode) -> bool:
        with self._lock:
            return self._set_mode(mode, "operator")

    def _set_mode(self, mode: Mode, reason: str) -> bool:
        t = self._telemetry
        position = LatLon(t.lat, t.lon) if t is not None and t.has_position else None
        if mode in (Mode.AUTO,) and not self.mission.waypoints:
            self.bus.message("No mission to run — plan one on the chart first", Level.WARNING)
            return False
        if mode is Mode.POINT_NAV and self._point_target is None:
            self.bus.message("Tap the chart to choose a point first", Level.WARNING)
            return False
        if mode.autonomous and position is None:
            self.bus.message(f"{mode.label} needs a GNSS fix", Level.WARNING)
            return False

        self.pilot.reset()
        if mode is Mode.HOLD:
            self._hold_point = position
        elif mode is Mode.RETURN_HOME:
            self._hold_point = None
            self._leg_start = position
        elif mode is Mode.POINT_NAV:
            self._leg_start = position
            self._hold_point = None
        elif mode is Mode.AUTO:
            self._leg_start = position
            # Resume where the mission was left, or start it from the top.
            if self._wp_index >= len(self.mission.waypoints):
                self._wp_index = 0
            self._wp_hold_until = None
            self._hold_point = None
        elif mode is Mode.COURSE_HOLD:
            self._course_setpoint = t.cog_deg if t and t.cog_deg is not None and (t.sog_ms or 0) > 0.4 \
                else (t.heading_deg if t else 0.0)
            self._speed_setpoint = max(0.3, t.sog_ms if t and t.sog_ms else self._speed_setpoint)
        elif mode is Mode.SPEED_HOLD and t and t.sog_ms and t.sog_ms > 0.3:
            self._speed_setpoint = t.sog_ms

        if mode is not self._mode:
            previous = self._mode
            self._mode = mode
            level = Level.WARNING if reason not in ("operator", "armed") else Level.INFO
            self.bus.emit(EventKind.MODE, f"Mode {mode.label} ({reason})", level,
                          mode=mode.value, previous=previous.value)
        return True

    def set_mission(self, mission: Mission) -> None:
        with self._lock:
            self.mission = mission
            self._wp_index = 0
            if self._mode is Mode.AUTO:
                self._set_mode(Mode.HOLD, "mission replaced")

    def restart_mission(self) -> None:
        with self._lock:
            self._wp_index = 0
            if self._mode is Mode.AUTO:
                t = self._telemetry
                self._leg_start = LatLon(t.lat, t.lon) if t and t.has_position else None

    def skip_waypoint(self, offset: int = 1) -> None:
        with self._lock:
            count = len(self.mission.waypoints)
            if count:
                self._wp_index = max(0, min(count - 1, self._wp_index + offset))
                t = self._telemetry
                self._leg_start = LatLon(t.lat, t.lon) if t and t.has_position else None

    def go_to(self, lat: float, lon: float) -> None:
        """Point navigation: drive to a tapped point, then hold there."""
        with self._lock:
            self._point_target = LatLon(lat, lon)
            self._set_mode(Mode.POINT_NAV, "operator")
            if self._mode is Mode.POINT_NAV:
                t = self._telemetry
                self._leg_start = LatLon(t.lat, t.lon) if t and t.has_position else None
                self._hold_point = None

    def adjust_speed(self, delta_ms: float) -> float:
        """Nudge the speed setpoint; outside the hold modes it is the cruise speed."""
        with self._lock:
            self._speed_setpoint = max(0.0, min(2.3, self._speed_setpoint + delta_ms))
            if self._mode not in (Mode.SPEED_HOLD, Mode.COURSE_HOLD):
                self.settings.autopilot.cruise_speed_ms = max(0.3, self._speed_setpoint)
            return self._speed_setpoint

    def adjust_course(self, delta_deg: float) -> None:
        with self._lock:
            if self._course_setpoint is not None:
                self._course_setpoint = (self._course_setpoint + delta_deg) % 360.0

    def set_home_here(self) -> None:
        with self._lock:
            t = self._telemetry
            if t is not None and t.has_position:
                self._home = LatLon(t.lat, t.lon)
                self.bus.message("Home moved to the boat's position", Level.NOTICE)

    def clear_track(self) -> None:
        with self._lock:
            self.track.clear()
            self._last_track = None
            self.track_version += 1

    def set_skimmer(self, on: bool) -> None:
        with self._lock:
            self._skimmer = on
            self.link.send_payload("skimmer", on)
            self.bus.message("Skimmer deployed — keep under 1.5 kn in the oil" if on
                             else "Skimmer stowed", Level.NOTICE)

    def toggle_skimmer(self) -> None:
        self.set_skimmer(not self._skimmer)

    def acknowledge_alarms(self) -> None:
        with self._lock:
            self._latched.clear()

    @property
    def point_target(self) -> LatLon | None:
        return self._point_target

    # ------------------------------------------------------------------
    # The control loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        period = 1.0 / CONTROL_HZ
        deadline = time.monotonic()
        last = deadline
        while not self._stop.is_set():
            now = time.monotonic()
            dt = min(0.2, max(1e-3, now - last))
            last = now
            try:
                self.tick(dt)
            except Exception:
                log.exception("control tick failed")
                self.link.send_command(0.0, 0.0, self._armed)
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()

    def tick(self, dt: float) -> NavState:
        """One control step.  Public so tests can drive it with simulated time."""
        with self._lock:
            t = self._telemetry
            now = time.monotonic()
            link_ok = t is not None and t.age(now) <= self.settings.safety.link_timeout_s
            self._check_safety(t, link_ok)
            self._drift_route(t, dt)
            port, stbd, guidance = self._command(t, link_ok, dt)
            if not self._armed:
                port = stbd = 0.0
            self.link.send_command(port, stbd, self._armed)
            if t is not None and t.skimmer.fitted and t.skimmer.deployed != self._skimmer:
                self.link.send_payload("skimmer", self._skimmer)
            if self._armed:
                self._armed_time += dt
            self._nav = self._build_nav(t, link_ok, port, stbd, guidance)
            self._write_log(t, port, stbd)
            return self._nav

    def _drift_route(self, t: Telemetry | None, dt: float) -> None:
        """Carry a drift-relative route (an oil sweep) along with the water."""
        drift = self.mission.drift_ms
        if not drift or t is None or t.boat_time_s is None:
            return
        last = getattr(self, "_drift_t", None)
        self._drift_t = t.boat_time_s
        if last is None or not 0 < t.boat_time_s - last < 5:
            return
        step = t.boat_time_s - last
        self.mission.shift(drift[0] * step, drift[1] * step)
        if self._leg_start is not None:
            self._leg_start = geo.to_lat_lon(self._leg_start, geo.Metres(drift[1] * step, drift[0] * step))

    def nav(self) -> NavState:
        with self._lock:
            return self._nav

    def _command(self, t: Telemetry | None, link_ok: bool, dt: float):
        """Motor levels for the current mode, and the guidance used to get them."""
        op = self._operator
        mode = self._mode
        cfg = self.settings.autopilot
        guidance = None

        # Taking the sticks back from the autopilot.
        if mode.autonomous and op.connected and op.deflection > self.settings.control.override_threshold:
            self._set_mode(Mode.MANUAL, "stick override")
            mode = Mode.MANUAL

        if mode is Mode.MANUAL:
            return op.port, op.stbd, None
        if mode is Mode.STOP or t is None or not link_ok:
            return 0.0, 0.0, None

        heading = t.heading_deg or 0.0
        rot = t.rot_dps or 0.0
        sog = t.sog_ms or 0.0
        position = LatLon(t.lat, t.lon) if t.has_position else None

        if mode is Mode.SPEED_HOLD:
            throttle = self.pilot.speed.update(self._speed_setpoint, sog, dt)
            return (*mix(throttle, op.steer), None)

        if mode is Mode.COURSE_HOLD:
            if abs(op.steer) > 0.1:
                self._course_setpoint = ((self._course_setpoint or heading) + op.steer * 30.0 * dt) % 360.0
            if abs(op.throttle) > 0.2:
                self._speed_setpoint = max(0.0, min(2.3, self._speed_setpoint + op.throttle * 0.4 * dt))
            port, stbd = self.pilot.steer_course(self._course_setpoint or heading, self._speed_setpoint,
                                                 heading, rot, t.cog_deg, sog, dt)
            return port, stbd, None

        if position is None:
            return 0.0, 0.0, None

        if mode is Mode.HOLD:
            target = self._hold_point or position
            self._hold_point = target
            return (*self.keeper.update(self.pilot, target, position, heading, rot, t.cog_deg, sog,
                                        cfg.cruise_speed_ms, dt), None)

        if mode in (Mode.RETURN_HOME, Mode.POINT_NAV):
            target = self._home if mode is Mode.RETURN_HOME else self._point_target
            if target is None:
                return 0.0, 0.0, None
            if self._hold_point is not None:
                return (*self.keeper.update(self.pilot, self._hold_point, position, heading, rot,
                                            t.cog_deg, sog, cfg.cruise_speed_ms, dt), None)
            start = self._leg_start or position
            guidance = line_of_sight(start, target, position, cfg.lookahead_m)
            if guidance.distance_to_go_m < cfg.accept_radius_m:
                self._hold_point = target
                what = "home" if mode is Mode.RETURN_HOME else "the point"
                self.bus.message(f"Arrived at {what} — holding station", Level.NOTICE)
                return 0.0, 0.0, guidance
            speed = min(cfg.cruise_speed_ms, max(0.4, guidance.distance_to_go_m * 0.15))
            return (*self.pilot.steer_course(guidance.course_deg, speed, heading, rot, t.cog_deg,
                                             sog, dt), guidance)

        if mode is Mode.AUTO:
            return self._run_mission(t, position, heading, rot, sog, dt)
        return 0.0, 0.0, None

    def _run_mission(self, t: Telemetry, position: LatLon, heading: float, rot: float,
                     sog: float, dt: float):
        cfg = self.settings.autopilot
        waypoints = self.mission.waypoints
        if self._wp_index >= len(waypoints):
            self._set_mode(Mode.HOLD, "mission complete")
            self.bus.message("Mission complete", Level.NOTICE)
            return 0.0, 0.0, None
        waypoint = waypoints[self._wp_index]
        target = waypoint.position

        if self._wp_hold_until is not None:
            if time.monotonic() < self._wp_hold_until:
                return (*self.keeper.update(self.pilot, target, position, heading, rot, t.cog_deg,
                                            sog, cfg.cruise_speed_ms, dt), None)
            self._wp_hold_until = None
            self._advance(target)
            return 0.0, 0.0, None

        start = self._leg_start or position
        guidance = line_of_sight(start, target, position, cfg.lookahead_m)
        arrived = guidance.distance_to_go_m < cfg.accept_radius_m or (
            guidance.passed_end and guidance.distance_to_go_m < 3 * cfg.accept_radius_m
        )
        if arrived:
            if waypoint.hold_s > 0:
                self._wp_hold_until = time.monotonic() + waypoint.hold_s
            else:
                self._advance(target)
            return 0.0, 0.0, guidance

        speed = waypoint.speed_ms or cfg.cruise_speed_ms
        # Slow down for a sharp turn at the end of the leg.
        nxt = self._wp_index + 1
        if nxt < len(waypoints) and guidance.distance_to_go_m < 2 * cfg.lookahead_m:
            turn = abs(geo.angle_difference(
                geo.bearing_deg(target, waypoints[nxt].position), geo.bearing_deg(start, target)))
            if turn > 60:
                speed *= 0.6
        return (*self.pilot.steer_course(guidance.course_deg, speed, heading, rot, t.cog_deg, sog,
                                         dt), guidance)

    def _advance(self, reached: LatLon) -> None:
        count = len(self.mission.waypoints)
        self.bus.emit(EventKind.MISSION, f"Waypoint {self._wp_index + 1}/{count} reached",
                      Level.INFO, index=self._wp_index)
        self._wp_index += 1
        self._leg_start = reached
        if self._wp_index >= count and self.mission.loop:
            self._wp_index = 0

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def _alarm(self, key: str, active: bool, text: str, level: Level) -> bool:
        """Raise or clear an alarm.  Returns True on the tick it becomes active."""
        if active and key not in self._alarms:
            self._alarms[key] = Alarm(key, text, level)
            self.bus.emit(EventKind.SAFETY, text, level, alarm=key)
            return True
        if active:
            self._alarms[key] = Alarm(key, text, level)
        elif key in self._alarms:
            del self._alarms[key]
        return False

    def _check_safety(self, t: Telemetry | None, link_ok: bool) -> None:
        cfg = self.settings.safety

        lost = t is not None and not link_ok
        if self._alarm("link", lost, "LINK LOST — boat's own failsafe has stopped the motors",
                       Level.CRITICAL):
            self._latched.add("link")
        if link_ok and not self._link_was_ok and "link" in self._latched and self._mode is not Mode.STOP:
            self._latched.discard("link")
            # The boat has been drifting with its motors stopped; do not let it
            # carry on as though nothing happened.
            self._set_mode(Mode.HOLD, "link restored")
        self._link_was_ok = link_ok
        if t is None or not link_ok:
            return

        battery = t.battery_pct
        low = battery is not None and battery < cfg.battery_low_pct
        critical = battery is not None and battery < cfg.battery_critical_pct
        self._alarm("battery_low", low and not critical, f"Battery low ({battery or 0:.0f}%)",
                    Level.WARNING)
        if self._alarm("battery_critical", critical, f"BATTERY CRITICAL ({battery or 0:.0f}%) — returning home",
                       Level.CRITICAL) and self._armed:
            self._set_mode(Mode.RETURN_HOME, "battery critical")

        if self._home is not None and t.has_position:
            distance = geo.distance_m(self._home, (t.lat, t.lon))
            outside = distance > cfg.geofence_radius_m
            if outside and "geofence" not in self._latched and self._armed:
                self._latched.add("geofence")
                self._set_mode(Mode.RETURN_HOME, "geofence")
            if distance < cfg.geofence_radius_m * 0.9:
                self._latched.discard("geofence")
            self._alarm("geofence", outside, f"GEOFENCE — {distance:.0f} m from home", Level.ERROR)

        depth = t.depth_m
        shallow = depth is not None and depth < cfg.shallow_alarm_m
        self._alarm("shallow", shallow, f"SHALLOW WATER {depth or 0:.1f} m", Level.ERROR)
        if (depth is not None and depth < cfg.shallow_stop_m and self._mode.autonomous
                and self._armed and self._mode is not Mode.HOLD):
            self._set_mode(Mode.STOP, f"depth {depth:.1f} m")

        if t.grounded and not self._was_grounded:
            self.groundings += 1
        self._was_grounded = t.grounded
        self._alarm("grounded", t.grounded, "AGROUND — back off in Manual", Level.CRITICAL)
        self._alarm("gnss", t.gps_fix < 2, "No GNSS fix", Level.ERROR)

        sk = t.skimmer
        full = sk.fitted and sk.tank_capacity_l > 0 and sk.tank_l >= sk.tank_capacity_l - 0.05
        self._alarm("tank_full", full, "SKIMMER TANK FULL — return to the GCS to offload", Level.WARNING)
        too_fast = sk.deployed and (t.stw_ms or 0) > self.settings.skimmer.max_speed_ms
        self._alarm("skim_speed", too_fast, "Too fast to skim — oil escapes under the arms", Level.NOTICE)

    # ------------------------------------------------------------------
    # Navigation figures
    # ------------------------------------------------------------------

    def _build_nav(self, t: Telemetry | None, link_ok: bool, port: float, stbd: float,
                   guidance) -> NavState:
        now = time.monotonic()
        while self._arrivals and now - self._arrivals[0] > 1.0:
            self._arrivals.popleft()
        position = LatLon(t.lat, t.lon) if t is not None and t.has_position else None

        home_d = home_b = None
        if position and self._home:
            home_d = geo.distance_m(position, self._home)
            home_b = geo.bearing_deg(position, self._home)

        target, label = None, ""
        mode = self._mode
        if mode is Mode.AUTO and self._wp_index < len(self.mission.waypoints):
            target, label = self.mission.waypoints[self._wp_index].position, f"WP {self._wp_index + 1}"
        elif mode is Mode.RETURN_HOME:
            target, label = self._home, "HOME"
        elif mode is Mode.POINT_NAV:
            target, label = self._point_target, "POINT"
        elif mode is Mode.HOLD:
            target, label = self._hold_point, "HOLD"
        if mode in (Mode.POINT_NAV, Mode.RETURN_HOME) and self._hold_point is not None:
            label += " (holding)"

        btw = dtw = ttg = remaining = None
        if position and target:
            btw = geo.bearing_deg(position, target)
            dtw = geo.distance_m(position, target)
            speed = max(0.2, (t.sog_ms or 0.0)) if t else 0.2
            if mode is Mode.AUTO:
                rest = [target] + self.mission.positions[self._wp_index + 1:]
                remaining = dtw + geo.path_length_m(rest)
                ttg = remaining / max(0.2, self.settings.autopilot.cruise_speed_ms
                                      if speed < 0.3 else speed)
            else:
                ttg = dtw / speed

        station_d = None
        sim = getattr(self.link, "station", None)
        if position and sim is not None:
            station_d = geo.distance_m(position, sim)

        return NavState(
            mode=mode,
            armed=self._armed,
            link_ok=link_ok,
            downlink_hz=float(len(self._arrivals)),
            telemetry=t,
            home=self._home,
            home_distance_m=home_d,
            home_bearing_deg=home_b,
            station_distance_m=station_d,
            target=target,
            target_label=label,
            waypoint_index=self._wp_index if mode is Mode.AUTO else -1,
            waypoint_count=len(self.mission.waypoints),
            bearing_to_target_deg=btw,
            distance_to_target_m=dtw,
            cross_track_m=guidance.cross_track_m if guidance is not None else None,
            time_to_go_s=ttg,
            mission_remaining_m=remaining,
            speed_setpoint_ms=self._speed_setpoint,
            course_setpoint_deg=self._course_setpoint if mode is Mode.COURSE_HOLD else None,
            command_port=port,
            command_stbd=stbd,
            alarms=tuple(sorted(self._alarms.values(), key=lambda a: -a.level)),
            trip_m=self._trip_m,
            run_time_s=self._armed_time,
            coverage=self._coverage(),
            soundings=len(self.soundings.soundings),
            groundings=self.groundings,
            manual_input=self._operator.connected,
            energy=self._estimate_energy(home_d),
            # Stored pointing at the wind's source, so its polar form is (speed, from).
            true_wind=vectors.to_polar(*self._wind_vec) if self._wind_vec else None,
            current=vectors.to_polar(*self._current_vec) if self._current_vec else None,
            skimmer_commanded=self._skimmer,
        )

    def _estimate_energy(self, home_distance: float | None) -> EnergyEstimate | None:
        if not self.power_history:
            return None
        newest = self.power_history[-1].t
        if newest == self._last_energy[0]:
            return self._last_energy[1]
        safety = self.settings.safety
        result = estimate(self.power_history, self.settings.boat.battery_ah * NOMINAL_V,
                          safety.battery_low_pct, safety.battery_critical_pct, home_distance)
        self._last_energy = (newest, result)
        return result

    _coverage_cache: tuple[int, int, float] = (-1, -1, 0.0)

    def _coverage(self) -> float:
        area = self.mission.area
        if area is None:
            return 0.0
        key = (id(area), len(self.soundings))
        if self._coverage_cache[:2] == key:
            return self._coverage_cache[2]
        # Counting cells is cheap but not free; recount every few new cells.
        if self._coverage_cache[0] == key[0] and key[1] - self._coverage_cache[1] < 25:
            return self._coverage_cache[2]
        value = self.soundings.coverage(area)
        self._coverage_cache = (key[0], key[1], value)
        return value

    def _write_log(self, t: Telemetry | None, port: float, stbd: float) -> None:
        if self._log_writer is None or t is None:
            return
        now = time.monotonic()
        if now - self._last_log < 1.0:
            return
        self._last_log = now
        nav = self._nav
        w = t.water

        def f(value, digits=2):
            return "" if value is None or (isinstance(value, float) and math.isnan(value)) \
                else f"{value:.{digits}f}"

        self._log_writer.writerow([
            f"{time.time():.1f}", self._mode.value, int(self._armed), f(t.lat, 7), f(t.lon, 7),
            f(t.heading_deg, 1), f(t.cog_deg, 1), f(t.sog_ms), f(t.depth_m), f(t.battery_v),
            f(t.battery_pct, 0), f(port), f(stbd), f(nav.cross_track_m, 1), f(w.temperature_c),
            f(w.salinity_psu), f(w.ph), f(w.dissolved_oxygen_mgl),
        ])
        self._log_file.flush()
