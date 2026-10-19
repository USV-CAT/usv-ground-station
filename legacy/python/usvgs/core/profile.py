"""The seabed along a route: how much water there is under the planned track."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from . import geo
from .geo import LatLon


@dataclass(frozen=True)
class Profile:
    #: Distance along the route, metres, for each sample.
    distance: list[float]
    #: Charted depth, metres (NaN on land).
    chart: list[float]
    #: Surveyed depth where the boat has sounded, else None.
    surveyed: list[float | None]
    #: Distance along the route of each waypoint.
    waypoints: list[float]

    @property
    def length(self) -> float:
        return self.distance[-1] if self.distance else 0.0

    def shallowest(self) -> tuple[float, float] | None:
        """(distance, depth) of the least water on the route, land counting as zero."""
        best = None
        for d, c, s in zip(self.distance, self.chart, self.surveyed, strict=True):
            depth = s if s is not None else (0.0 if c != c else c)
            if best is None or depth < best[1]:
                best = (d, depth)
        return best


def route_profile(points: Sequence[LatLon], chart_depth: Callable[[float, float], float],
                  surveyed_depth: Callable[[float, float], float | None] | None = None,
                  step_m: float = 10.0, max_samples: int = 600) -> Profile:
    if len(points) < 2:
        return Profile([], [], [], [])
    total = geo.path_length_m(points)
    step = max(step_m, total / max_samples)
    distance, chart, surveyed, marks = [], [], [], [0.0]
    travelled = 0.0
    for a, b in zip(points, points[1:], strict=False):
        leg = geo.distance_m(a, b)
        bearing = geo.bearing_deg(a, b)
        count = max(1, math.ceil(leg / step))
        for i in range(count):
            p = geo.destination(a, bearing, leg * i / count)
            distance.append(travelled + leg * i / count)
            chart.append(chart_depth(p.lat, p.lon))
            surveyed.append(surveyed_depth(p.lat, p.lon) if surveyed_depth else None)
        travelled += leg
        marks.append(travelled)
    last = points[-1]
    distance.append(travelled)
    chart.append(chart_depth(last.lat, last.lon))
    surveyed.append(surveyed_depth(last.lat, last.lon) if surveyed_depth else None)
    return Profile(distance, chart, surveyed, marks)
