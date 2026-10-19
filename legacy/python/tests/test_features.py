import math
import time

import pytest

from usvgs.config import Settings
from usvgs.core import geo, vectors
from usvgs.core.energy import PowerSample, estimate
from usvgs.core.geo import LatLon
from usvgs.core.mission import Mission, Rectangle
from usvgs.core.profile import route_profile
from usvgs.link.sim import SimLink
from usvgs.sim.oil import OilSpill, Skimmer, appearance
from usvgs.sim.world import World

SEA = LatLon(57.10, 23.95)


# -- vectors ----------------------------------------------------------------

def test_true_wind_from_apparent_when_motoring_into_it():
    # 5 m/s from the north, boat doing 2 m/s north: 7 m/s apparent, dead ahead.
    speed, source = vectors.true_wind(7.0, 0.0, 0.0, 2.0, 0.0)
    assert speed == pytest.approx(5.0, abs=1e-6)
    assert source == pytest.approx(0.0, abs=1e-6)


def test_true_wind_on_the_beam():
    # Wind 3 m/s from the east, boat 3 m/s north: apparent 4.24 m/s from 45° to starboard.
    speed, source = vectors.true_wind(math.hypot(3, 3), 45.0, 0.0, 3.0, 0.0)
    assert speed == pytest.approx(3.0, abs=1e-6)
    assert source == pytest.approx(90.0, abs=1e-6)


def test_current_is_ground_minus_water():
    drift, set_deg = vectors.current(sog=1.0, cog_deg=90.0, stw=1.0, heading_deg=0.0)
    assert drift == pytest.approx(math.sqrt(2))
    assert set_deg == pytest.approx(135.0)


def test_vehicle_estimates_wind_and_current(chart):
    settings = Settings()
    settings.sim.wind_speed_ms, settings.sim.wind_from_deg = 5.0, 270.0
    settings.sim.current_speed_ms, settings.sim.current_to_deg = 0.3, 180.0
    from usvgs.vehicle.vehicle import Vehicle
    link = SimLink(settings, chart)
    vehicle = Vehicle(settings, link, record=False)
    vehicle.arm()
    vehicle.set_operator(port=0.6, stbd=0.6, throttle=0.6, steer=0, deflection=0.3, connected=True)
    for i in range(1200):
        link.step(0.05)
        if i % 2 == 0:
            link._deliver(link.telemetry())
        vehicle.tick(0.05)
    nav = vehicle.nav()
    assert nav.true_wind[0] == pytest.approx(5.0, abs=1.5)
    assert abs(geo.angle_difference(nav.true_wind[1], 270.0)) < 25
    assert nav.current[0] == pytest.approx(0.3, abs=0.15)


# -- energy -----------------------------------------------------------------

def test_energy_estimate_from_a_steady_draw():
    history = []
    energy = 0.0
    for t in range(0, 300):
        pct = 80.0 - t * 0.01
        energy += 15.0 * 10.0 / 3600.0
        history.append(PowerSample(float(t), 15.0, 10.0, pct, 1.5, energy))
    e = estimate(history, capacity_wh=296.0, low_pct=30.0, critical_pct=15.0, home_distance_m=500.0)
    by_draw = 296.0 * history[-1].percent / 100 / (10.0 * 14.8) * 3600
    by_trend = history[-1].percent / 0.01
    assert e.time_to_empty_s == pytest.approx(min(by_draw, by_trend), rel=0.02)
    assert 0 < e.time_to_low_s < e.time_to_critical_s < e.time_to_empty_s
    assert e.wh_per_km == pytest.approx(150.0 / 1.5 / 3.6, rel=0.05)
    assert e.home_reserve_pct is not None and e.home_reserve_pct > 0
    line = e.projection(history[-1].t, history[-1].percent, 600.0)
    assert line[1][1] < line[0][1]


def test_no_draw_means_no_deadline():
    history = [PowerSample(float(t), 16.0, 0.0, 90.0, 0.0, 0.0) for t in range(60)]
    e = estimate(history, 296.0, 30.0, 15.0, None)
    assert math.isinf(e.time_to_empty_s)
    assert e.home_reserve_pct is None


# -- depth profile ----------------------------------------------------------

def test_route_profile_finds_the_shallows():
    def depth(lat, lon):
        return 1.0 if lon > 23.951 else 8.0

    route = [SEA, LatLon(SEA.lat, 23.955)]
    profile = route_profile(route, depth, step_m=10.0)
    assert profile.length == pytest.approx(geo.distance_m(*route), abs=1)
    low = profile.shallowest()
    assert low[1] == 1.0 and low[0] > 50


# -- oil --------------------------------------------------------------------

def make_world(chart, wind=0.0, current=0.0):
    world = World(chart, SEA)
    world.wind_speed_ms, world.wind_from_deg = wind, 270.0
    world.current_speed_ms, world.current_to_deg = current, 0.0
    return world


def test_slick_drifts_with_current_and_three_percent_of_wind(chart):
    world = make_world(chart, wind=10.0, current=0.1)
    spill = OilSpill(world)
    spill.release(SEA, 100.0, 20.0, 400)
    north0, east0 = spill.north.mean(), spill.east.mean()
    world._gust = 0.0
    for _ in range(600):
        world._gust = 0.0
        spill.step(0.1)
    # 60 s: current 0.1 m/s north, windage 0.3 m/s east (wind from the west).
    assert spill.north.mean() - north0 == pytest.approx(6.0, abs=1.0)
    assert spill.east.mean() - east0 == pytest.approx(18.0, abs=1.5)
    assert spill.totals.evaporated_l > 0


def test_skimmer_recovers_oil_and_fills_the_tank(chart):
    settings = Settings().skimmer
    world = make_world(chart)
    spill = OilSpill(world)
    spill.release(SEA, 200.0, 10.0, 800)
    skimmer = Skimmer(settings)
    skimmer.deployed = True
    # Sit in the middle of the slick: the pump is the limit.
    for _ in range(600):
        skimmer.step(0.1, spill, 0.0, 0.0, 0.0, 0.3, at_station=False)
    assert skimmer.recovered_oil_l > 1.0
    assert spill.totals.recovered_l == pytest.approx(skimmer.recovered_oil_l)
    assert skimmer.rate_lpm <= settings.max_rate_lpm * settings.oil_content + 1e-6
    assert skimmer.tank_l == pytest.approx(skimmer.recovered_oil_l / settings.oil_content)


def test_skimming_too_fast_loses_oil():
    skimmer = Skimmer(Settings().skimmer)
    assert skimmer.efficiency_at(0.3) > 0.8
    assert skimmer.efficiency_at(1.5) < 0.2


def test_tank_offloads_alongside_with_skimmer_stowed(chart):
    skimmer = Skimmer(Settings().skimmer)
    skimmer.tank_l = 30.0
    spill = OilSpill(make_world(chart))
    skimmer.step(10.0, spill, 0, 0, 0, 0, at_station=True)
    assert skimmer.offloading and skimmer.tank_l < 30.0


def test_bonn_codes():
    assert appearance(0.1)[0] == "1"
    assert appearance(2.0)[0] == "2"
    assert appearance(100.0)[0] == "4"
    assert appearance(0.0)[0] == "—"


def test_sweep_route_moves_with_the_water():
    mission = Mission()
    mission.area = Rectangle(SEA, 40, 20, 90)
    mission.regenerate()
    first = mission.waypoints[0].position
    mission.shift(10.0, 0.0)
    assert geo.distance_m(first, mission.waypoints[0].position) == pytest.approx(10.0, abs=0.01)
    assert geo.bearing_deg(first, mission.waypoints[0].position) == pytest.approx(0.0, abs=0.5)


def test_simulated_skimmer_draws_power(chart):
    settings = Settings()
    link = SimLink(settings, chart)
    link.step(0.1)
    idle = link.boat.state.battery_current_a
    link.send_payload("skimmer", True)
    for _ in range(5):
        link.step(0.1)
    assert link.boat.state.battery_current_a == pytest.approx(idle + settings.skimmer.power_a, abs=0.2)
    assert link.telemetry().skimmer.deployed


# -- the chart views ---------------------------------------------------------

@pytest.mark.parametrize("mode", ["north", "heading", "chase"])
def test_taps_land_where_they_are_drawn(mode):
    QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
    from PyQt5.QtCore import QPointF

    from usvgs.ui.mapview.canvas import ChartCanvas
    from usvgs.ui.mapview.tiles import TiledMap

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    canvas = ChartCanvas(TiledMap(), Mission())
    canvas.resize(640, 640)
    canvas.set_boat(SEA.lat, SEA.lon, 50.0, None, None)
    canvas.set_view_mode(mode)
    # Let the turn settle: the easing runs on time, not on reports.
    start = time.monotonic()
    for i in range(1, 30):
        canvas.advance(start + 0.1 * i)
    canvas.grab()      # paint once, so the chase camera has its transform
    target = geo.destination(SEA, 70.0, 60.0)
    if mode == "chase":
        with canvas._chase_space():
            view = canvas.project(target.lat, target.lon)
        screen = canvas._perspective.map(view)
    else:
        screen = canvas.project(target.lat, target.lon)
    lat, lon = canvas.event_geo(QPointF(screen))
    assert geo.distance_m(target, (lat, lon)) < 1.0
    if mode != "north":
        # Heading up: the bow points up the screen.
        ahead = canvas.project(*geo.destination(SEA, 50.0, 50.0)) if mode == "heading" else None
        if ahead is not None:
            here = canvas.project(SEA.lat, SEA.lon)
            assert abs(ahead.x() - here.x()) < 3 and ahead.y() < here.y()
    del app
