import pytest

from usvgs.config import Settings
from usvgs.core import geo
from usvgs.core.geo import LatLon
from usvgs.core.mission import Rectangle, lawnmower
from usvgs.link.sim import SimLink
from usvgs.vehicle.modes import Mode
from usvgs.vehicle.vehicle import Vehicle


class Rig:
    """A vehicle and a simulated boat, run in lock-step on a fake clock."""

    def __init__(self, chart, **sim):
        self.settings = Settings()
        self.settings.sim.gps_noise_m = 0.2
        self.settings.sim.wind_speed_ms = sim.get("wind", 3.0)
        self.settings.sim.current_speed_ms = sim.get("current", 0.1)
        self.link = SimLink(self.settings, chart)
        self.vehicle = Vehicle(self.settings, self.link, record=False)
        self.run(0.5)

    def run(self, seconds, dt=0.05):
        for i in range(int(seconds / dt)):
            self.link.step(dt)
            if i % 2 == 0:
                self.link._deliver(self.link.telemetry())
            self.vehicle.tick(dt)
        return self.vehicle.nav()

    def position(self):
        t = self.vehicle.nav().telemetry
        return LatLon(t.lat, t.lon)


def test_disarmed_boat_does_not_move(chart):
    rig = Rig(chart)
    rig.vehicle.set_operator(port=1, stbd=1, throttle=1, steer=0, deflection=1, connected=True)
    nav = rig.run(3)
    assert not nav.armed
    assert abs(nav.telemetry.throttle_port) < 0.01


def test_manual_drive(chart):
    rig = Rig(chart)
    assert rig.vehicle.arm()
    rig.vehicle.set_operator(port=1, stbd=1, throttle=1, steer=0, deflection=1, connected=True)
    nav = rig.run(10)
    assert nav.telemetry.sog_ms > 1.8


@pytest.mark.timeout(120)
def test_survey_mission_runs_and_tracks_the_lanes(chart):
    rig = Rig(chart)
    rig.vehicle.arm()
    start = rig.position()
    rect = Rectangle(geo.destination(start, 300.0, 90.0), 120.0, 40.0, 30.0)
    rig.vehicle.mission.area = rect
    rig.vehicle.mission.set_route(lawnmower(rect, 10.0, 2.0))
    assert rig.vehicle.set_mode(Mode.AUTO)
    worst = 0.0
    for _ in range(900):
        nav = rig.run(1.0)
        if nav.cross_track_m is not None and nav.waypoint_index > 1:
            worst = max(worst, abs(nav.cross_track_m))
        if nav.mode is not Mode.AUTO:
            break
    assert nav.mode is Mode.HOLD
    assert worst < 8.0
    assert nav.coverage > 0.7


def test_stick_override_takes_manual_control(chart):
    rig = Rig(chart)
    rig.vehicle.arm()
    rig.vehicle.set_mode(Mode.HOLD)
    rig.vehicle.set_operator(port=1, stbd=1, throttle=1, steer=0, deflection=0.9, connected=True)
    nav = rig.run(0.2)
    assert nav.mode is Mode.MANUAL


def test_point_nav_arrives_and_holds(chart):
    rig = Rig(chart)
    rig.vehicle.arm()
    target = geo.destination(rig.position(), 330.0, 80.0)
    rig.vehicle.go_to(target.lat, target.lon)
    for _ in range(200):
        nav = rig.run(1.0)
        if "holding" in nav.target_label:
            break
    assert "holding" in nav.target_label
    rig.run(30)
    assert geo.distance_m(rig.position(), target) < 8.0


def test_geofence_returns_home(chart):
    rig = Rig(chart)
    rig.settings.safety.geofence_radius_m = 40.0
    rig.vehicle.arm()
    rig.vehicle.set_operator(port=1, stbd=1, throttle=1, steer=0, deflection=0.3, connected=True)
    for _ in range(60):
        nav = rig.run(1.0)
        if nav.mode is Mode.RETURN_HOME:
            break
    assert nav.mode is Mode.RETURN_HOME
    assert any(a.key == "geofence" for a in nav.alarms)


def test_battery_critical_returns_home(chart):
    rig = Rig(chart)
    rig.vehicle.arm()
    rig.vehicle.set_mode(Mode.HOLD)
    rig.link.boat.state.battery_soc = 0.10
    nav = rig.run(1.0)
    assert nav.mode is Mode.RETURN_HOME


def test_shallow_water_stops_autonomous_modes(chart):
    rig = Rig(chart)
    rig.vehicle.arm()
    # Point Nav straight at the beach.
    target = LatLon(57.048, 24.006)
    rig.vehicle.go_to(target.lat, target.lon)
    for _ in range(600):
        nav = rig.run(1.0)
        if nav.mode is not Mode.POINT_NAV:
            break
    assert nav.mode is Mode.STOP
    assert rig.vehicle.groundings == 0


def test_link_loss_then_hold(chart):
    rig = Rig(chart)
    rig.vehicle.arm()
    rig.vehicle.set_mode(Mode.HOLD)
    # Stop delivering telemetry, but keep ticking with real elapsed time faked.
    import time
    real = time.monotonic
    offset = [0.0]
    time.monotonic = lambda: real() + offset[0]
    try:
        rig.vehicle.set_mode(Mode.MANUAL)
        offset[0] = 5.0
        nav = rig.vehicle.tick(0.05)
        assert not nav.link_ok
        assert any(a.key == "link" for a in nav.alarms)
        offset[0] = 0.0
    finally:
        time.monotonic = real
    nav = rig.run(0.5)
    assert nav.link_ok
    assert nav.mode is Mode.HOLD
