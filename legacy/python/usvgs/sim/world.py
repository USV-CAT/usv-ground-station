"""The simulated world the boat floats in: water depth, wind, current, water.

The chart grid is about 115 m a cell, far coarser than what an echo sounder
sees.  The world adds smooth detail on top of it — sand ridges, holes — so the
survey a player runs actually finds something the chart did not show.
"""

from __future__ import annotations

import math
import random

from ..core import geo
from ..core.bathymetry import DepthGrid
from ..core.geo import LatLon
from ..core.telemetry import WaterQuality


def _hash(ix: int, iy: int, seed: int) -> float:
    """A repeatable pseudo-random value in [-1, 1] for an integer lattice point."""
    h = (ix * 374761393 + iy * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((h ^ (h >> 16)) & 0xFFFF) / 32767.5 - 1.0


def value_noise(x: float, y: float, wavelength: float, seed: int = 0) -> float:
    """Smooth noise in [-1, 1] with features about ``wavelength`` across."""
    fx, fy = x / wavelength, y / wavelength
    ix, iy = math.floor(fx), math.floor(fy)
    tx, ty = fx - ix, fy - iy
    sx, sy = tx * tx * (3 - 2 * tx), ty * ty * (3 - 2 * ty)
    a = _hash(ix, iy, seed)
    b = _hash(ix + 1, iy, seed)
    c = _hash(ix, iy + 1, seed)
    d = _hash(ix + 1, iy + 1, seed)
    return (a + (b - a) * sx) * (1 - sy) + (c + (d - c) * sx) * sy


class World:
    def __init__(self, chart: DepthGrid | None, origin: LatLon, seed: int = 7) -> None:
        self.chart = chart
        self.origin = origin
        self.seed = seed
        self.wind_speed_ms = 4.0
        self.wind_from_deg = 240.0
        self.current_speed_ms = 0.15
        self.current_to_deg = 60.0
        #: A river mouth freshens and clouds the water around it.  The Daugava
        #: by default, since that is where the bundled chart is.
        self.river_mouth = LatLon(57.058, 24.025)
        self._gust = 0.0
        self._rng = random.Random(seed)

    # -- depth --------------------------------------------------------------

    def depth_at(self, lat: float, lon: float) -> float:
        """True water depth, metres; NaN on land."""
        if self.chart is None:
            # No chart: an endless lake that shoals at the edges of 2 km.
            local = geo.to_metres(self.origin, (lat, lon))
            r = math.hypot(local.east, local.north)
            base = 8.0 * max(0.0, 1.0 - (r / 2000.0) ** 2)
            if base <= 0:
                return math.nan
        else:
            base = self.chart.depth_at(lat, lon)
            if base != base:
                return math.nan
        local = geo.to_metres(self.origin, (lat, lon))
        detail = (
            0.22 * value_noise(local.east, local.north, 60.0, self.seed)
            + 0.10 * value_noise(local.east, local.north, 18.0, self.seed + 1)
        )
        # Detail scales with depth, so the shore stays a shore.
        depth = base * (1.0 + detail) if base < 3.0 else base + 0.66 * detail * 3.0
        return max(0.0, depth)

    def is_water_many(self, lat, lon):
        """Vectorised land test for many positions (the chart only, no detail)."""
        import numpy as np

        if self.chart is None:
            east = np.radians(np.asarray(lon) - self.origin.lon) * geo.EARTH_RADIUS_M * math.cos(
                math.radians(self.origin.lat))
            north = np.radians(np.asarray(lat) - self.origin.lat) * geo.EARTH_RADIUS_M
            return np.hypot(east, north) < 2000.0
        depth = self.chart.depth_nearest(lat, lon)
        return np.isfinite(depth) & (depth > 0.05)

    # -- weather ------------------------------------------------------------

    def step(self, dt: float) -> None:
        # Gusts: a slow random walk pulled back towards zero.
        self._gust += (-0.3 * self._gust) * dt + self._rng.gauss(0.0, 0.35) * math.sqrt(dt)
        self._gust = max(-0.45, min(0.6, self._gust))

    def wind_vector(self) -> tuple[float, float]:
        """True wind velocity (north, east) in m/s: where the air is going."""
        speed = max(0.0, self.wind_speed_ms * (1.0 + self._gust))
        towards = math.radians(self.wind_from_deg + 180.0)
        return speed * math.cos(towards), speed * math.sin(towards)

    def current_vector(self) -> tuple[float, float]:
        towards = math.radians(self.current_to_deg)
        return (
            self.current_speed_ms * math.cos(towards),
            self.current_speed_ms * math.sin(towards),
        )

    # -- water quality ------------------------------------------------------

    def water_quality(self, lat: float, lon: float, t: float) -> WaterQuality:
        """A plausible sonde reading for the Gulf of Riga in autumn."""
        local = geo.to_metres(self.origin, (lat, lon))
        mouth = geo.distance_m((lat, lon), self.river_mouth)
        plume = math.exp(-mouth / 2500.0)
        n = value_noise(local.east, local.north, 400.0, self.seed + 5)
        salinity = max(0.1, 5.9 - 4.8 * plume + 0.15 * n)
        temperature = 11.8 + 0.9 * plume + 0.3 * value_noise(local.east, local.north, 700.0, 9)
        conductivity = salinity * 1.72 + 0.25            # mS/cm, brackish water
        oxygen = 9.6 - 0.8 * plume + 0.25 * n
        saturation = oxygen / (14.6 - 0.39 * temperature) * 100.0
        jitter = 0.01 * math.sin(t * 0.7)
        return WaterQuality(
            temperature_c=round(temperature + jitter, 2),
            ph=round(8.05 - 0.35 * plume + 0.05 * n, 2),
            dissolved_oxygen_mgl=round(oxygen, 2),
            oxygen_saturation_pct=round(saturation, 1),
            conductivity_ms=round(conductivity, 2),
            tds_ppm=round(conductivity * 640.0),
            salinity_psu=round(salinity, 2),
            orp_mv=round(215.0 - 40.0 * plume + 8.0 * n, 1),
            turbidity_ntu=round(2.0 + 14.0 * plume + 1.5 * abs(n), 1),
        )
