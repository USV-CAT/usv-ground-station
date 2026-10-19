"""The vector triangles every navigator draws.

* **True wind** from the apparent wind the anemometer measures, by taking the
  boat's own motion out of it.
* **Current** (set and drift) as the difference between the velocity over the
  ground, from GNSS, and the velocity through the water, from the log and the
  compass.

Vectors are (north, east) in metres per second.  Bearings are degrees true;
wind is named for where it comes *from*, current for where it goes *to*.
"""

from __future__ import annotations

import math


def from_polar(speed: float, bearing_deg: float) -> tuple[float, float]:
    theta = math.radians(bearing_deg)
    return speed * math.cos(theta), speed * math.sin(theta)


def to_polar(north: float, east: float) -> tuple[float, float]:
    """(speed, bearing towards) of a vector."""
    return math.hypot(north, east), (math.degrees(math.atan2(east, north)) + 360.0) % 360.0


def true_wind(apparent_speed: float, apparent_angle_deg: float, heading_deg: float,
              sog: float, cog_deg: float | None) -> tuple[float, float]:
    """True wind (speed, direction it blows from), degrees true.

    The apparent wind is the true wind minus the boat's velocity; adding the
    boat's ground velocity back gives the wind over the ground.
    """
    air_from = heading_deg + apparent_angle_deg
    # The air's velocity relative to the boat, pointing where it goes.
    an, ae = from_polar(apparent_speed, air_from + 180.0)
    bn, be = from_polar(sog, cog_deg if cog_deg is not None else heading_deg)
    speed, towards = to_polar(an + bn, ae + be)
    return speed, (towards + 180.0) % 360.0


def current(sog: float, cog_deg: float | None, stw: float, heading_deg: float) -> tuple[float, float]:
    """Current (drift in m/s, set in degrees towards)."""
    gn, ge = from_polar(sog, cog_deg if cog_deg is not None else heading_deg)
    wn, we = from_polar(stw, heading_deg)
    return to_polar(gn - wn, ge - we)
