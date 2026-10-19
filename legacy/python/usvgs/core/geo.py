"""Geodesy for a few kilometres of water.

Distances here are computed on a sphere.  Over the range a small surface vessel
works from its ground station — a few kilometres — the difference between a
spherical and an ellipsoidal model is well under a metre, which is far inside
the boat's own GNSS error.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

#: Mean Earth radius, metres.
EARTH_RADIUS_M = 6_371_008.8


@dataclass(frozen=True)
class LatLon:
    """A geographic position in degrees."""

    lat: float
    lon: float

    def __iter__(self):
        yield self.lat
        yield self.lon

    def __post_init__(self) -> None:
        if not -90.0 <= self.lat <= 90.0:
            raise ValueError(f"latitude out of range: {self.lat}")
        if not -180.0 <= self.lon <= 180.0:
            raise ValueError(f"longitude out of range: {self.lon}")


Point = LatLon | tuple[float, float]


def _pair(p: Point) -> tuple[float, float]:
    if isinstance(p, LatLon):
        return p.lat, p.lon
    return float(p[0]), float(p[1])


def distance_m(a: Point, b: Point) -> float:
    """Great-circle distance in metres (haversine)."""
    lat1, lon1 = _pair(a)
    lat2, lon2 = _pair(b)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    h = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, h)))


def bearing_deg(a: Point, b: Point) -> float:
    """Initial great-circle bearing from ``a`` to ``b``, degrees clockwise from north."""
    lat1, lon1 = _pair(a)
    lat2, lon2 = _pair(b)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def destination(origin: Point, bearing: float, distance: float) -> LatLon:
    """The position reached by travelling ``distance`` metres along ``bearing``."""
    lat1, lon1 = _pair(origin)
    phi1 = math.radians(lat1)
    lambda1 = math.radians(lon1)
    theta = math.radians(bearing)
    delta = distance / EARTH_RADIUS_M

    phi2 = math.asin(
        math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta)
    )
    lambda2 = lambda1 + math.atan2(
        math.sin(theta) * math.sin(delta) * math.cos(phi1),
        math.cos(delta) - math.sin(phi1) * math.sin(phi2),
    )
    lon = (math.degrees(lambda2) + 540.0) % 360.0 - 180.0
    return LatLon(math.degrees(phi2), lon)


def metres_per_degree(latitude: float) -> tuple[float, float]:
    """Metres per degree of latitude and of longitude at ``latitude``.

    Used for the local flat-Earth arithmetic in route planning, where working
    in degrees directly would distort line spacing with latitude.
    """
    lat_m = math.pi * EARTH_RADIUS_M / 180.0
    lon_m = lat_m * math.cos(math.radians(latitude))
    return lat_m, max(1e-6, lon_m)


def angle_difference(a: float, b: float) -> float:
    """Signed smallest difference ``a - b`` between two headings, in [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


# --------------------------------------------------------------------------
# Bounding boxes
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class BBox:
    """An axis-aligned latitude/longitude bounding box."""

    min_lat: float
    min_lon: float
    max_lat: float
    max_lon: float

    def __post_init__(self) -> None:
        if self.min_lat > self.max_lat or self.min_lon > self.max_lon:
            raise ValueError("bounding box corners are transposed")

    @classmethod
    def around(cls, points: Iterable[Point], margin_m: float = 0.0) -> BBox:
        pairs = [_pair(p) for p in points]
        if not pairs:
            raise ValueError("cannot bound an empty set of points")
        lats = [p[0] for p in pairs]
        lons = [p[1] for p in pairs]
        box = cls(min(lats), min(lons), max(lats), max(lons))
        return box.expanded(margin_m) if margin_m else box

    def expanded(self, margin_m: float) -> BBox:
        lat_m, lon_m = metres_per_degree(self.centre.lat)
        return BBox(
            max(-90.0, self.min_lat - margin_m / lat_m),
            max(-180.0, self.min_lon - margin_m / lon_m),
            min(90.0, self.max_lat + margin_m / lat_m),
            min(180.0, self.max_lon + margin_m / lon_m),
        )

    @property
    def centre(self) -> LatLon:
        return LatLon((self.min_lat + self.max_lat) / 2.0, (self.min_lon + self.max_lon) / 2.0)

    @property
    def corners(self) -> tuple[LatLon, LatLon, LatLon, LatLon]:
        return (
            LatLon(self.min_lat, self.min_lon),
            LatLon(self.min_lat, self.max_lon),
            LatLon(self.max_lat, self.max_lon),
            LatLon(self.max_lat, self.min_lon),
        )

    @property
    def width_m(self) -> float:
        return distance_m((self.centre.lat, self.min_lon), (self.centre.lat, self.max_lon))

    @property
    def height_m(self) -> float:
        return distance_m((self.min_lat, self.centre.lon), (self.max_lat, self.centre.lon))

    def contains(self, p: Point) -> bool:
        lat, lon = _pair(p)
        return self.min_lat <= lat <= self.max_lat and self.min_lon <= lon <= self.max_lon


def path_length_m(points: Sequence[Point]) -> float:
    """Total length of a polyline in metres."""
    return sum(distance_m(points[i], points[i + 1]) for i in range(len(points) - 1))


# --------------------------------------------------------------------------
# Polygons
# --------------------------------------------------------------------------
#
# Survey areas and routes are far easier to reason about in metres than in
# degrees, so the planners work in a local tangent plane about a reference point.


@dataclass(frozen=True)
class Metres:
    """East-north metres relative to a reference point."""

    east: float
    north: float


def to_metres(origin: Point, point: Point) -> Metres:
    """``point`` expressed in a local tangent plane about ``origin``."""
    origin_lat, origin_lon = _pair(origin)
    lat, lon = _pair(point)
    return Metres(
        east=math.radians(lon - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin_lat)),
        north=math.radians(lat - origin_lat) * EARTH_RADIUS_M,
    )


def to_lat_lon(origin: Point, offset: Metres) -> LatLon:
    """The inverse of :func:`to_metres`."""
    origin_lat, origin_lon = _pair(origin)
    scale = math.cos(math.radians(origin_lat)) or 1e-9
    return LatLon(
        origin_lat + math.degrees(offset.north / EARTH_RADIUS_M),
        origin_lon + math.degrees(offset.east / (EARTH_RADIUS_M * scale)),
    )


def centroid(points: Sequence[Point]) -> LatLon:
    """The mean of a set of positions.

    Only ever used as the origin of a local projection, so the arithmetic mean
    is enough; it is not the centre of mass of the polygon.
    """
    pairs = [_pair(p) for p in points]
    if not pairs:
        raise ValueError("cannot take the centroid of no points")
    return LatLon(
        sum(p[0] for p in pairs) / len(pairs),
        sum(p[1] for p in pairs) / len(pairs),
    )


def _projected(polygon: Sequence[Point]) -> list[Metres]:
    origin = centroid(polygon)
    return [to_metres(origin, p) for p in polygon]


def polygon_area_m2(polygon: Sequence[Point]) -> float:
    """Area of a simple polygon in square metres, by the shoelace formula."""
    if len(polygon) < 3:
        return 0.0
    local = _projected(polygon)
    total = 0.0
    previous = local[-1]
    for corner in local:
        total += (previous.east + corner.east) * (previous.north - corner.north)
        previous = corner
    return abs(total) / 2.0


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Whether ``point`` is inside ``polygon``, by the even-odd rule.

    A point exactly on an edge is not guaranteed either way, which does not
    matter for a survey grid whose lines are inset from the boundary anyway.
    """
    if len(polygon) < 3:
        return False
    origin = centroid(polygon)
    local = [to_metres(origin, p) for p in polygon]
    target = to_metres(origin, point)

    inside = False
    previous = local[-1]
    for corner in local:
        if (corner.north > target.north) != (previous.north > target.north):
            crossing = (previous.east - corner.east) * (target.north - corner.north) / (
                previous.north - corner.north
            ) + corner.east
            if target.east < crossing:
                inside = not inside
        previous = corner
    return inside


def _orientation(a: Metres, b: Metres, c: Metres) -> float:
    return (b.east - a.east) * (c.north - a.north) - (b.north - a.north) * (c.east - a.east)


def _on_segment(a: Metres, b: Metres, point: Metres) -> bool:
    return (
        abs(_orientation(a, b, point)) < 1e-9
        and min(a.east, b.east) - 1e-9 <= point.east <= max(a.east, b.east) + 1e-9
        and min(a.north, b.north) - 1e-9 <= point.north <= max(a.north, b.north) + 1e-9
    )


def _segments_cross(a: Metres, b: Metres, c: Metres, d: Metres) -> bool:
    d1 = _orientation(c, d, a)
    d2 = _orientation(c, d, b)
    d3 = _orientation(a, b, c)
    d4 = _orientation(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return any(
        _on_segment(*pair)
        for pair in ((c, d, a), (c, d, b), (a, b, c), (a, b, d))
    )


def polygon_is_simple(polygon: Sequence[Point]) -> bool:
    """Whether a polygon's edges meet only at their shared corners.

    A figure-of-eight area is not something a lawnmower pattern can be clipped
    to sensibly: the lanes would silently cover one lobe and skip the other.
    """
    count = len(polygon)
    if count < 3:
        return False
    local = _projected(polygon)
    for i in range(count):
        a, b = local[i], local[(i + 1) % count]
        for j in range(i + 1, count):
            # Edges that share a corner always touch there; only the pairs that
            # do not are worth testing.
            if j == i or (j + 1) % count == i or (i + 1) % count == j:
                continue
            if _segments_cross(a, b, local[j], local[(j + 1) % count]):
                return False
    return True
