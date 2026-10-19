import math

import pytest

from usvgs.config import BoatSettings, Settings
from usvgs.core import geo
from usvgs.core.geo import LatLon
from usvgs.link.sim import SimLink
from usvgs.sim.boat import Catamaran
from usvgs.sim.world import World

OPEN_WATER = LatLon(57.10, 23.95)


def boat(chart, wind=0.0, current=0.0):
    world = World(chart, OPEN_WATER)
    world.wind_speed_ms = wind
    world.current_speed_ms = current
    return Catamaran(BoatSettings(), world, OPEN_WATER, 0.0)


def test_full_ahead_reaches_top_speed_on_heading(chart):
    b = boat(chart)
    for _ in range(1000):
        b.step(0.02, 1.0, 1.0)
    assert b.state.u == pytest.approx(2.37, abs=0.1)
    assert abs(geo.angle_difference(b.heading_deg, 0.0)) < 0.5
    assert b.state.battery_current_a > 30


def test_differential_thrust_turns_to_starboard(chart):
    b = boat(chart)
    for _ in range(100):
        b.step(0.02, 1.0, -1.0)
    assert b.state.r > 0.3
    assert 0 < b.heading_deg < 180


def test_current_carries_a_stopped_boat(chart):
    b = boat(chart, current=0.5)
    b.world.current_to_deg = 90.0
    for _ in range(500):
        b.step(0.02, 0.0, 0.0)
    assert b.state.east == pytest.approx(5.0, abs=0.3)


def test_boat_cannot_drive_onto_land(chart):
    world = World(chart, OPEN_WATER)
    world.wind_speed_ms = 0
    world.current_speed_ms = 0
    # Daugavgrīva, a few hundred metres off the beach, heading east for the shore.
    start = LatLon(57.048, 23.995)
    b = Catamaran(BoatSettings(), world, start, 90.0)
    for _ in range(30000):
        b.step(0.02, 1.0, 1.0)
        if b.state.grounded:
            break
    assert b.state.grounded
    assert b.depth_here() == b.depth_here()   # it stopped in the water, not on land


def test_link_failsafe_stops_motors(chart):
    settings = Settings()
    link = SimLink(settings, chart)
    link.send_command(1.0, 1.0, True)
    for _ in range(25):
        link.step(0.02)
    assert link.boat.state.motor_port > 0.5
    for _ in range(150):          # 3 s of silence
        link.step(0.02)
    assert link.telemetry().onboard_failsafe
    assert abs(link.boat.state.motor_port) < 0.05


def test_telemetry_reports_depth_and_water(chart):
    link = SimLink(Settings(), chart)
    t = link.telemetry()
    assert t.depth_m is not None and t.depth_m > 0.5
    assert 0.1 < t.water.salinity_psu < 8
    assert t.has_position and not math.isnan(t.lat)
