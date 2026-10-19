"""Routes: waypoints, and the rectangle patterns that generate them.

A mission is an ordered list of waypoints.  It can be clicked onto the chart
point by point, or generated from a rectangle: a *lawnmower* sweeps it in
parallel lanes, which is how a single-beam echo sounder surveys an area, and a
*perimeter* runs round its edge.

The rectangle is kept with the mission so a survey can be regenerated with a
different lane spacing without drawing it again.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from . import geo
from .geo import LatLon, Metres

PATTERNS = ("lawnmower", "perimeter")


@dataclass
class Waypoint:
    lat: float
    lon: float
    #: Speed for the leg that ends here, m/s.  None uses the cruise speed.
    speed_ms: float | None = None
    #: Station-keep here this long before moving on, seconds.
    hold_s: float = 0.0

    @property
    def position(self) -> LatLon:
        return LatLon(self.lat, self.lon)

    def to_dict(self) -> dict:
        data = {"lat": round(self.lat, 8), "lon": round(self.lon, 8)}
        if self.speed_ms is not None:
            data["speed_ms"] = self.speed_ms
        if self.hold_s:
            data["hold_s"] = self.hold_s
        return data

    @classmethod
    def from_dict(cls, data: dict) -> Waypoint:
        speed = data.get("speed_ms")
        return cls(
            float(data["lat"]),
            float(data["lon"]),
            None if speed is None else float(speed),
            float(data.get("hold_s", 0.0)),
        )


@dataclass
class Rectangle:
    """A rectangle on the water, possibly rotated.

    ``angle_deg`` is the bearing of the rectangle's *width* axis, so 90° is a
    rectangle squared to the compass with its width running east.
    """

    centre: LatLon
    width_m: float
    height_m: float
    angle_deg: float = 90.0

    @classmethod
    def from_corners(cls, a: LatLon, b: LatLon, angle_deg: float = 90.0) -> Rectangle:
        """The rectangle with opposite corners ``a`` and ``b``, at ``angle_deg``."""
        mid = LatLon((a.lat + b.lat) / 2.0, (a.lon + b.lon) / 2.0)
        offset = geo.to_metres(a, b)
        u, v = _axes(angle_deg)
        width = abs(offset.east * u[0] + offset.north * u[1])
        height = abs(offset.east * v[0] + offset.north * v[1])
        return cls(mid, width, height, angle_deg % 360.0)

    def _point(self, along: float, across: float) -> LatLon:
        u, v = _axes(self.angle_deg)
        return geo.to_lat_lon(
            self.centre,
            Metres(along * u[0] + across * v[0], along * u[1] + across * v[1]),
        )

    def corners(self) -> list[LatLon]:
        w, h = self.width_m / 2.0, self.height_m / 2.0
        return [self._point(-w, -h), self._point(w, -h), self._point(w, h), self._point(-w, h)]

    def contains(self, point: LatLon) -> bool:
        local = geo.to_metres(self.centre, point)
        u, v = _axes(self.angle_deg)
        along = local.east * u[0] + local.north * u[1]
        across = local.east * v[0] + local.north * v[1]
        return abs(along) <= self.width_m / 2.0 and abs(across) <= self.height_m / 2.0

    @property
    def area_m2(self) -> float:
        return self.width_m * self.height_m

    def to_dict(self) -> dict:
        return {
            "lat": self.centre.lat,
            "lon": self.centre.lon,
            "width_m": self.width_m,
            "height_m": self.height_m,
            "angle_deg": self.angle_deg,
        }

    @classmethod
    def from_dict(cls, data: dict) -> Rectangle:
        return cls(
            LatLon(float(data["lat"]), float(data["lon"])),
            float(data["width_m"]),
            float(data["height_m"]),
            float(data.get("angle_deg", 90.0)),
        )


def _axes(angle_deg: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """Unit vectors (east, north) along the width and the height of a rectangle."""
    theta = math.radians(angle_deg)
    u = (math.sin(theta), math.cos(theta))
    v = (-u[1], u[0])
    return u, v


@dataclass
class PatternSpec:
    pattern: str = "lawnmower"
    #: Distance between lanes, metres.
    spacing_m: float = 10.0
    #: Distance kept inside the rectangle's edge, metres.
    margin_m: float = 2.0
    #: Lanes run along the longer side, which means fewer turns.
    along_long_side: bool = True
    #: Laps of a perimeter pattern.
    laps: int = 1

    def to_dict(self) -> dict:
        return dict(self.__dict__)

    @classmethod
    def from_dict(cls, data: dict) -> PatternSpec:
        spec = cls()
        for key in spec.__dict__:
            if key in data:
                setattr(spec, key, type(getattr(spec, key))(data[key]))
        return spec


def lawnmower(rect: Rectangle, spacing_m: float, margin_m: float = 0.0,
              along_long_side: bool = True) -> list[LatLon]:
    """Parallel lanes across ``rect``, alternating direction (a boustrophedon).

    Lanes are spread evenly so the outer two sit exactly ``margin_m`` inside the
    edge, which means the actual spacing is at most ``spacing_m``.
    """
    if spacing_m <= 0:
        raise ValueError("lane spacing must be positive")
    half_w = max(0.0, rect.width_m / 2.0 - margin_m)
    half_h = max(0.0, rect.height_m / 2.0 - margin_m)
    lanes_along_width = (rect.width_m >= rect.height_m) == along_long_side
    run, span = (half_w, half_h) if lanes_along_width else (half_h, half_w)

    lanes = max(1, math.ceil(2.0 * span / spacing_m) + 1) if span > 0 else 1
    step = 2.0 * span / (lanes - 1) if lanes > 1 else 0.0

    points: list[LatLon] = []
    for index in range(lanes):
        offset = -span + index * step
        start, end = (-run, run) if index % 2 == 0 else (run, -run)
        for along in (start, end):
            if lanes_along_width:
                points.append(rect._point(along, offset))
            else:
                points.append(rect._point(offset, along))
    return points


def perimeter(rect: Rectangle, margin_m: float = 0.0, laps: int = 1) -> list[LatLon]:
    """Round the edge of ``rect``, ``laps`` times, finishing where it started."""
    inner = Rectangle(
        rect.centre,
        max(0.0, rect.width_m - 2 * margin_m),
        max(0.0, rect.height_m - 2 * margin_m),
        rect.angle_deg,
    )
    corners = inner.corners()
    points: list[LatLon] = []
    for _ in range(max(1, laps)):
        points.extend(corners)
    points.append(corners[0])
    return points


def generate(rect: Rectangle, spec: PatternSpec) -> list[LatLon]:
    if spec.pattern == "perimeter":
        return perimeter(rect, spec.margin_m, spec.laps)
    if spec.pattern == "lawnmower":
        return lawnmower(rect, spec.spacing_m, spec.margin_m, spec.along_long_side)
    raise ValueError(f"unknown pattern {spec.pattern!r}")


@dataclass
class Mission:
    waypoints: list[Waypoint] = field(default_factory=list)
    name: str = "mission"
    #: Start again from the first waypoint after the last one.
    loop: bool = False
    #: The rectangle this route was generated from, if it was.
    area: Rectangle | None = None
    pattern: PatternSpec = field(default_factory=PatternSpec)
    #: Move the whole route with the water at this velocity, (north, east) m/s.
    #: A sweep of an oil slick is planned relative to the oil, which drifts.
    drift_ms: tuple[float, float] | None = None

    @property
    def positions(self) -> list[LatLon]:
        return [w.position for w in self.waypoints]

    def add(self, lat: float, lon: float, **kwargs) -> Waypoint:
        waypoint = Waypoint(lat, lon, **kwargs)
        self.waypoints.append(waypoint)
        return waypoint

    def remove(self, index: int) -> None:
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]

    def clear(self) -> None:
        self.waypoints.clear()
        self.area = None
        self.drift_ms = None

    def set_route(self, points: list[LatLon], speed_ms: float | None = None) -> None:
        self.waypoints = [Waypoint(p.lat, p.lon, speed_ms) for p in points]

    def regenerate(self) -> bool:
        """Rebuild the route from the stored rectangle.  Returns whether it did."""
        if self.area is None:
            return False
        self.set_route(generate(self.area, self.pattern))
        return True

    def shift(self, north_m: float, east_m: float) -> None:
        """Translate the route and its area, as the water carries them."""
        offset = geo.Metres(east_m, north_m)
        for w in self.waypoints:
            moved = geo.to_lat_lon(w.position, offset)
            w.lat, w.lon = moved.lat, moved.lon
        if self.area is not None:
            self.area = Rectangle(geo.to_lat_lon(self.area.centre, offset), self.area.width_m,
                                  self.area.height_m, self.area.angle_deg)

    def length_m(self, start: LatLon | None = None) -> float:
        points = ([start] if start else []) + self.positions
        if self.loop and len(self.positions) > 1:
            points.append(self.positions[0])
        return geo.path_length_m(points)

    def duration_s(self, cruise_speed_ms: float, start: LatLon | None = None) -> float:
        """Time to run the route at cruise speed, plus holds and a turn allowance."""
        if cruise_speed_ms <= 0:
            return math.inf
        turns = max(0, len(self.waypoints) - 1) * 4.0
        holds = sum(w.hold_s for w in self.waypoints)
        return self.length_m(start) / cruise_speed_ms + turns + holds

    # -- files --------------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "format": "usvgs-mission",
            "version": 1,
            "name": self.name,
            "loop": self.loop,
            "area": self.area.to_dict() if self.area else None,
            "pattern": self.pattern.to_dict(),
            "drift_ms": list(self.drift_ms) if self.drift_ms else None,
            "waypoints": [w.to_dict() for w in self.waypoints],
        }

    @classmethod
    def from_dict(cls, data: dict) -> Mission:
        area = data.get("area")
        return cls(
            waypoints=[Waypoint.from_dict(w) for w in data.get("waypoints", [])],
            name=str(data.get("name", "mission")),
            loop=bool(data.get("loop", False)),
            area=Rectangle.from_dict(area) if area else None,
            pattern=PatternSpec.from_dict(data.get("pattern", {})),
            drift_ms=tuple(data["drift_ms"]) if data.get("drift_ms") else None,
        )

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> Mission:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def cross_track(start: LatLon, end: LatLon, position: LatLon) -> tuple[float, float, float]:
    """Where ``position`` is relative to the leg ``start`` → ``end``.

    Returns ``(along, across, leg_length)`` in metres.  ``across`` is positive
    to starboard of the leg, which is the sign convention of an XTE display:
    positive means steer to port.
    """
    leg = geo.to_metres(start, end)
    here = geo.to_metres(start, position)
    length = math.hypot(leg.east, leg.north)
    if length < 1e-6:
        return 0.0, 0.0, 0.0
    ue, un = leg.east / length, leg.north / length
    along = here.east * ue + here.north * un
    across = here.east * un - here.north * ue
    return along, across, length
