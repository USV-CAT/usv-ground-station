import math

import pytest

from usvgs.core import geo
from usvgs.core.geo import LatLon
from usvgs.core.mission import Mission, PatternSpec, Rectangle, cross_track, lawnmower, perimeter

CENTRE = LatLon(57.05, 23.99)


def test_rectangle_corners_are_its_size():
    rect = Rectangle(CENTRE, 200.0, 80.0, 90.0)
    c = rect.corners()
    assert geo.distance_m(c[0], c[1]) == pytest.approx(200.0, abs=0.1)
    assert geo.distance_m(c[1], c[2]) == pytest.approx(80.0, abs=0.1)
    # Width axis runs east at 90°.
    assert geo.bearing_deg(c[0], c[1]) == pytest.approx(90.0, abs=0.1)


def test_rectangle_from_corners_round_trips_when_rotated():
    rect = Rectangle(CENTRE, 150.0, 60.0, 35.0)
    c = rect.corners()
    again = Rectangle.from_corners(c[0], c[2], 35.0)
    assert again.width_m == pytest.approx(150.0, abs=0.2)
    assert again.height_m == pytest.approx(60.0, abs=0.2)
    assert geo.distance_m(again.centre, CENTRE) < 0.2


def test_lawnmower_covers_the_rectangle_with_alternating_lanes():
    rect = Rectangle(CENTRE, 100.0, 40.0, 90.0)
    route = lawnmower(rect, spacing_m=10.0, margin_m=0.0)
    assert len(route) == 2 * 5        # 40 m across at 10 m: five lanes
    assert all(rect.contains(p) or geo.distance_m(p, rect.centre) < 60 for p in route)
    first = geo.bearing_deg(route[0], route[1])
    second = geo.bearing_deg(route[2], route[3])
    assert abs(geo.angle_difference(first, second)) == pytest.approx(180.0, abs=0.5)
    lanes = [geo.distance_m(route[i], route[i + 1]) for i in range(0, len(route), 2)]
    assert all(lane == pytest.approx(100.0, abs=0.2) for lane in lanes)


def test_lawnmower_runs_along_the_short_side_when_asked():
    rect = Rectangle(CENTRE, 100.0, 40.0, 90.0)
    route = lawnmower(rect, spacing_m=10.0, along_long_side=False)
    assert geo.distance_m(route[0], route[1]) == pytest.approx(40.0, abs=0.2)


def test_perimeter_closes():
    rect = Rectangle(CENTRE, 50.0, 30.0, 10.0)
    route = perimeter(rect, laps=2)
    assert len(route) == 9
    assert geo.distance_m(route[0], route[-1]) < 0.01
    assert geo.path_length_m(route) == pytest.approx(2 * 160.0, abs=0.5)


def test_mission_file_round_trip(tmp_path):
    mission = Mission(name="test", loop=True)
    mission.area = Rectangle(CENTRE, 80.0, 40.0, 45.0)
    mission.pattern = PatternSpec(spacing_m=8.0)
    assert mission.regenerate()
    mission.waypoints[0].hold_s = 5.0
    path = tmp_path / "m.json"
    mission.save(path)
    loaded = Mission.load(path)
    assert loaded.loop and loaded.name == "test"
    assert len(loaded.waypoints) == len(mission.waypoints)
    assert loaded.waypoints[0].hold_s == 5.0
    assert loaded.area.angle_deg == 45.0
    assert loaded.pattern.spacing_m == 8.0


def test_cross_track_sign_is_positive_to_starboard():
    start = CENTRE
    end = geo.destination(CENTRE, 0.0, 100.0)
    east_of_line = geo.destination(geo.destination(CENTRE, 0.0, 50.0), 90.0, 5.0)
    along, across, length = cross_track(start, end, east_of_line)
    assert along == pytest.approx(50.0, abs=0.1)
    assert across == pytest.approx(5.0, abs=0.1)
    assert length == pytest.approx(100.0, abs=0.1)
    assert not math.isnan(across)
