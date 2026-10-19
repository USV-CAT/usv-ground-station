"""The simulated boat, behind the same link interface a real one will have.

Physics runs on its own thread at 50 Hz of simulated time.  The link between
it and the ground station is modelled too: telemetry at 10 Hz, a radio whose
signal fades with distance and drops packets towards the edge of its range,
and an on-board failsafe that stops the motors when commands stop arriving —
which is what a real boat must do, because the ground station's autopilot is
on the wrong side of a broken radio link.
"""

from __future__ import annotations

import math
import random
import threading
import time
from collections import deque

from ..config import Settings
from ..core import geo
from ..core.bathymetry import DepthGrid
from ..core.geo import LatLon
from ..core.telemetry import Telemetry
from ..sim.boat import MAX_RPM, Catamaran
from ..sim.oil import OilSpill, Skimmer, spill_start
from ..sim.world import World
from .base import BoatLink

PHYSICS_HZ = 50.0
TELEMETRY_HZ = 10.0
#: The boat stops its motors after this long without a command.
COMMAND_TIMEOUT_S = 1.0
#: Echo sounders lose the bottom beyond this.
SOUNDER_MAX_M = 120.0


class SimLink(BoatLink):
    name = "SIM"
    simulated = True

    def __init__(self, settings: Settings, chart: DepthGrid | None, seed: int = 7) -> None:
        super().__init__()
        self.settings = settings
        sim = settings.sim
        self.station = LatLon(sim.start_lat, sim.start_lon)
        self.world = World(chart, self.station, seed)
        start = self.station
        if chart is not None and not self.world.depth_at(*start) > settings.boat.draft_m + 0.5:
            start = chart.deepest_near(*start) or start
        self.boat = Catamaran(settings.boat, self.world, start, sim.start_heading_deg)
        self.apply_environment()
        self.spill = OilSpill(self.world, seed)
        self.skimmer = Skimmer(settings.skimmer)

        self._rng = random.Random(seed)
        self._lock = threading.Lock()
        self._command = (0.0, 0.0, False)
        self._command_time = -math.inf      # simulated time of the last command
        self._uplink_times: deque[float] = deque(maxlen=64)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._gps_error = [0.0, 0.0]
        self._failsafe = False
        self._oil_dt = 0.0
        self.paused = False
        self.release_spill()

    # -- control from the game ----------------------------------------------

    def apply_environment(self) -> None:
        sim = self.settings.sim
        self.world.wind_speed_ms = sim.wind_speed_ms
        self.world.wind_from_deg = sim.wind_from_deg
        self.world.current_speed_ms = sim.current_speed_ms
        self.world.current_to_deg = sim.current_to_deg

    def respawn(self, position: LatLon | None = None, heading_deg: float | None = None) -> None:
        """Put the boat back in the water, at the launch point unless told otherwise."""
        with self._lock:
            target = position or self.station
            too_shallow = not self.world.depth_at(*target) > self.settings.boat.draft_m + 0.3
            if too_shallow and self.world.chart is not None:
                target = self.world.chart.deepest_near(*target) or target
            heading = self.settings.sim.start_heading_deg if heading_deg is None else heading_deg
            self.boat.reset(target, heading)
            self.boat.recharge()

    def release_spill(self, centre: LatLon | None = None, volume_l: float | None = None) -> None:
        """Put a slick on the water: by default the configured one, off the launch point."""
        spill = self.settings.spill
        centre = centre or spill_start(self.station, spill)
        if self.world.chart is not None and not self.world.depth_at(*centre) > 1.0:
            centre = self.world.chart.deepest_near(*centre) or centre
        with self._lock:
            self.spill.release(centre, volume_l or spill.volume_l, spill.radius_m, spill.parcels)

    def spill_snapshot(self):
        """Copies of the slick's parcels, ``(lat, lon, litres, beached)``, for drawing."""
        with self._lock:
            if not self.spill.volume.size:
                return None
            lat, lon = self.spill.positions()
            return lat.copy(), lon.copy(), self.spill.volume.copy(), self.spill.beached.copy()

    def spill_drift(self) -> tuple[float, float]:
        """How surface oil is moving here: current plus windage, (north, east) m/s."""
        from ..sim.oil import WINDAGE
        wn, we = self.world.wind_vector()
        cn, ce = self.world.current_vector()
        return cn + WINDAGE * wn, ce + WINDAGE * we

    def clear_spill(self) -> None:
        with self._lock:
            self.spill.clear()

    def at_station(self) -> bool:
        """Alongside the ground station, slow enough to pump the tank out."""
        vn, ve = self.boat.ground_velocity
        return self.range_m() < 15.0 and math.hypot(vn, ve) < 0.3

    # -- link interface ------------------------------------------------------

    def send_payload(self, name: str, value) -> None:
        if not self._radio_delivers():
            return
        with self._lock:
            if name == "skimmer":
                self.skimmer.deployed = bool(value)

    def open(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim-boat", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def send_command(self, port: float, stbd: float, armed: bool) -> None:
        if not self._radio_delivers():
            return
        with self._lock:
            self._command = (port, stbd, armed)
            self._command_time = self.boat.state.t
            self._uplink_times.append(time.monotonic())

    # -- the radio -------------------------------------------------------------

    def range_m(self) -> float:
        return geo.distance_m(self.station, self.boat.position)

    def rssi_dbm(self) -> float:
        distance = max(1.0, self.range_m())
        return -38.0 - 20.0 * math.log10(distance / 10.0) * 0.9

    def _radio_delivers(self) -> bool:
        limit = self.settings.sim.link_range_m
        fraction = self.range_m() / max(1.0, limit)
        if fraction < 0.8:
            return True
        loss = min(1.0, (fraction - 0.8) / 0.4)
        return self._rng.random() > loss

    # -- the simulation loop -------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance the simulation by ``dt`` seconds.  Public for tests."""
        with self._lock:
            port, stbd, armed = self._command
            stale = self.boat.state.t - self._command_time > COMMAND_TIMEOUT_S
            if stale or not armed:
                port = stbd = 0.0
            self.world.step(dt)
            self.boat.state.extra_load_a = self.skimmer.load_a()
            self.boat.step(dt, port, stbd)
            # The slick and the skimmer change slowly; 10 Hz is plenty and
            # keeps a fast time scale from starving the rest of the program.
            self._oil_dt += dt
            if self._oil_dt >= 0.1:
                s = self.boat.state
                self.spill.step(self._oil_dt)
                self.skimmer.step(self._oil_dt, self.spill, s.north, s.east, s.heading, s.u,
                                  self.at_station())
                self._oil_dt = 0.0
            self._failsafe = stale and armed

    def telemetry(self) -> Telemetry:
        """What the boat would report right now.  Public for tests."""
        boat = self.boat
        s = boat.state
        with self._lock:
            noise = self.settings.sim.gps_noise_m
            # GNSS error wanders rather than jumping, as real receivers do.
            for i in range(2):
                self._gps_error[i] += -0.05 * self._gps_error[i] + self._rng.gauss(0, noise * 0.3)
            position = geo.to_lat_lon(
                boat.origin,
                geo.Metres(s.east + self._gps_error[1], s.north + self._gps_error[0]),
            )
            vn, ve = boat.ground_velocity
            sog = math.hypot(vn, ve) + abs(self._rng.gauss(0, 0.02))
            cog = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0 if sog > 0.15 else None
            depth = boat.depth_here()
            sounded = None
            if depth == depth and boat.settings.draft_m < depth < SOUNDER_MAX_M:
                sounded = max(0.0, depth - boat.settings.draft_m + self._rng.gauss(0, 0.03))
            elif s.grounded:
                sounded = 0.0
            air_x, air_y = boat.relative_air
            roll, pitch = boat.attitude()
            now = time.monotonic()
            while self._uplink_times and now - self._uplink_times[0] > 1.0:
                self._uplink_times.popleft()
            soc = s.battery_soc
            return Telemetry(
                received=now,
                boat_time_s=s.t,
                lat=position.lat,
                lon=position.lon,
                heading_deg=(boat.heading_deg + self._rng.gauss(0, 0.3)) % 360.0,
                cog_deg=cog,
                sog_ms=sog,
                stw_ms=max(0.0, s.u),
                rot_dps=math.degrees(s.r),
                roll_deg=roll,
                pitch_deg=pitch,
                gps_fix=5 if self.range_m() < 1500 else 3,
                satellites=14 + int(2 * math.sin(s.t / 60.0)),
                hdop=0.7,
                throttle_port=s.motor_port,
                throttle_stbd=s.motor_stbd,
                rpm_port=s.motor_port * MAX_RPM,
                rpm_stbd=s.motor_stbd * MAX_RPM,
                battery_v=boat.battery_voltage,
                battery_a=s.battery_current_a,
                battery_pct=soc * 100.0,
                depth_m=sounded,
                wind_apparent_ms=math.hypot(air_x, air_y),
                wind_apparent_deg=math.degrees(math.atan2(-air_y, -air_x)),
                water=self.world.water_quality(position.lat, position.lon, s.t),
                skimmer=self.skimmer.state(self.spill, s.north, s.east),
                armed=self._command[2],
                onboard_failsafe=self._failsafe,
                grounded=s.grounded,
                rssi_dbm=self.rssi_dbm(),
                uplink_hz=float(len(self._uplink_times)),
            )

    def _run(self) -> None:
        period = 1.0 / PHYSICS_HZ
        per_report = int(PHYSICS_HZ / TELEMETRY_HZ)
        tick = 0
        deadline = time.monotonic()
        while not self._stop.is_set():
            if not self.paused:
                scale = max(0.1, min(8.0, self.settings.sim.time_scale))
                # Large time scales take several fixed steps, so the physics
                # stays stable however fast the game runs.
                steps = max(1, int(round(scale)))
                for _ in range(steps):
                    self.step(period * scale / steps)
                tick += 1
                if tick % per_report == 0 and self._radio_delivers():
                    self._deliver(self.telemetry())
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()
