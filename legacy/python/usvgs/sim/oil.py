"""An oil slick, and the skimmer that recovers it.

**The slick** is a cloud of Lagrangian parcels, the method operational spill
models (GNOME, OpenDrift) use.  Each parcel carries a volume of oil and moves
with the surface current plus about 3 % of the wind speed — the long-standing
rule of thumb for surface oil drift — with a random walk added for spreading.
Light fractions evaporate slowly, and a parcel that drifts ashore beaches and
stays there.

**The skimmer** is the payload this USV is built around: two sweep arms
funnel a swath of water into an oleophilic brush (or belt) skimmer between the
hulls, which lifts the oil into a tank.  It is limited three ways, as real ones
are:

* by the *oil it meets* — the parcels inside its swath;
* by *speed* — booms only hold oil at about a knot; faster, it is carried
  under them (entrainment), so efficiency falls away above the best speed;
* by the *pump* — a maximum lift rate — and by the *tank*, which then has to
  be emptied alongside the ground station.

The fluorometer under the hull reports oil in water from the film thickness at
the boat, which is also what the Bonn Agreement appearance code is read from.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import SkimmerSettings, SpillSettings
from ..core import geo
from ..core.geo import LatLon
from ..core.telemetry import SkimmerState
from .world import World

#: Fraction of the wind speed surface oil drifts at.
WINDAGE = 0.03
#: Horizontal diffusion for spreading, m²/s.
DIFFUSION = 0.03
#: Evaporation, fraction of the remaining volume per second (light crude, early hours).
EVAPORATION = 2.0e-5
#: Cell used to estimate film thickness, metres.
FILM_CELL_M = 6.0

#: Bonn Agreement Oil Appearance Code: (upper thickness µm, code, description).
BONN_CODES = (
    (0.3, "1", "Sheen (silvery/grey)"),
    (5.0, "2", "Rainbow"),
    (50.0, "3", "Metallic"),
    (200.0, "4", "Discontinuous true colour"),
    (math.inf, "5", "Continuous true colour"),
)


def appearance(film_um: float | None) -> tuple[str, str]:
    if film_um is None or film_um <= 0.04:
        return "—", "No oil"
    for limit, code, text in BONN_CODES:
        if film_um < limit:
            return code, text
    return BONN_CODES[-1][1:]


@dataclass
class SpillTotals:
    released_l: float = 0.0
    floating_l: float = 0.0
    evaporated_l: float = 0.0
    beached_l: float = 0.0
    recovered_l: float = 0.0

    @property
    def recovered_fraction(self) -> float:
        return self.recovered_l / self.released_l if self.released_l else 0.0


class OilSpill:
    def __init__(self, world: World, seed: int = 11) -> None:
        self.world = world
        self.origin = world.origin
        self._rng = np.random.default_rng(seed)
        self.north = np.zeros(0)
        self.east = np.zeros(0)
        self.volume = np.zeros(0)
        self.beached = np.zeros(0, bool)
        self.totals = SpillTotals()
        self._since_beach_check = 0.0

    @property
    def active(self) -> bool:
        return bool(self.volume.size) and self.totals.floating_l > 0.05

    def release(self, centre: LatLon, volume_l: float, radius_m: float, parcels: int = 1500) -> None:
        """A fresh spill, stretched downwind as a real one quickly is."""
        rng = self._rng
        offset = geo.to_metres(self.origin, centre)
        wn, we = self.world.wind_vector()
        heading = math.atan2(we, wn) if (wn or we) else 0.0
        along = rng.normal(0.0, radius_m * 0.75, parcels)
        across = rng.normal(0.0, radius_m * 0.35, parcels)
        self.north = offset.north + along * math.cos(heading) - across * math.sin(heading)
        self.east = offset.east + along * math.sin(heading) + across * math.cos(heading)
        # A thick core and a thin fringe, as a slick actually looks.
        weight = np.exp(-(along**2 + (2 * across) ** 2) / (2 * radius_m**2)) + 0.15
        self.volume = weight / weight.sum() * volume_l
        self.beached = np.zeros(parcels, bool)
        self.totals = SpillTotals(released_l=volume_l, floating_l=volume_l)
        self._drop_land()

    def clear(self) -> None:
        self.north = self.east = self.volume = np.zeros(0)
        self.beached = np.zeros(0, bool)
        self.totals = SpillTotals()

    def positions(self) -> tuple[np.ndarray, np.ndarray]:
        """Parcel latitudes and longitudes."""
        lat = self.origin.lat + np.degrees(self.north / geo.EARTH_RADIUS_M)
        lon = self.origin.lon + np.degrees(
            self.east / (geo.EARTH_RADIUS_M * math.cos(math.radians(self.origin.lat))))
        return lat, lon

    def step(self, dt: float) -> None:
        if not self.volume.size:
            return
        floating = ~self.beached
        wn, we = self.world.wind_vector()
        cn, ce = self.world.current_vector()
        spread = math.sqrt(2.0 * DIFFUSION * dt)
        count = int(floating.sum())
        self.north[floating] += (cn + WINDAGE * wn) * dt + self._rng.normal(0, spread, count)
        self.east[floating] += (ce + WINDAGE * we) * dt + self._rng.normal(0, spread, count)
        lost = self.volume[floating] * (EVAPORATION * dt)
        self.volume[floating] -= lost
        self.totals.evaporated_l += float(lost.sum())
        self._since_beach_check += dt
        if self._since_beach_check >= 1.0:
            self._since_beach_check = 0.0
            self._beach()
        self._tally()

    def _land_mask(self) -> np.ndarray:
        lat, lon = self.positions()
        return ~self.world.is_water_many(lat, lon)

    def _drop_land(self) -> None:
        """Parcels that start on land are discarded: the spill is what is on the water."""
        land = self._land_mask()
        if land.any():
            self.totals.released_l -= float(self.volume[land].sum())
            self.volume[land] = 0.0
        self._tally()

    def _beach(self) -> None:
        candidates = ~self.beached & (self.volume > 0)
        if not candidates.any():
            return
        ashore = candidates & self._land_mask()
        if ashore.any():
            self.beached |= ashore
            self.totals.beached_l += float(self.volume[ashore].sum())

    def _tally(self) -> None:
        self.totals.floating_l = float(self.volume[~self.beached].sum()) if self.volume.size else 0.0

    # -- at the boat --------------------------------------------------------

    def film_um(self, north: float, east: float, cell_m: float = FILM_CELL_M) -> float:
        """Film thickness around a point: oil volume over area, in micrometres."""
        if not self.volume.size:
            return 0.0
        near = (~self.beached & (np.abs(self.north - north) < cell_m / 2)
                & (np.abs(self.east - east) < cell_m / 2))
        litres = float(self.volume[near].sum())
        return litres * 1e-3 / (cell_m * cell_m) * 1e6

    def take(self, mask: np.ndarray, litres: float) -> float:
        """Remove ``litres`` of oil from the parcels in ``mask``, in proportion."""
        available = float(self.volume[mask].sum())
        if available <= 0 or litres <= 0:
            return 0.0
        taken = min(available, litres)
        self.volume[mask] *= 1.0 - taken / available
        self.totals.recovered_l += taken
        return taken


class Skimmer:
    def __init__(self, settings: SkimmerSettings) -> None:
        self.settings = settings
        self.deployed = False
        self.tank_l = 0.0
        self.recovered_oil_l = 0.0
        self.rate_lpm = 0.0
        self.efficiency = 0.0
        self.offloading = False
        self.recovering = False

    @property
    def tank_full(self) -> bool:
        return self.tank_l >= self.settings.tank_l - 1e-6

    def load_a(self) -> float:
        return self.settings.power_a if self.deployed else 0.0

    def efficiency_at(self, speed_ms: float) -> float:
        """Share of the oil met that the arms hold and the brush lifts."""
        s = self.settings
        speed = abs(speed_ms)
        if speed <= s.best_speed_ms:
            return 0.85
        if speed >= s.max_speed_ms * 2.0:
            return 0.05
        return 0.85 - 0.8 * (speed - s.best_speed_ms) / (s.max_speed_ms * 2.0 - s.best_speed_ms)

    def step(self, dt: float, spill: OilSpill, north: float, east: float, heading_rad: float,
             speed_through_water: float, at_station: bool) -> None:
        s = self.settings
        self.rate_lpm = 0.0
        self.recovering = False
        self.efficiency = self.efficiency_at(speed_through_water) if self.deployed else 0.0

        self.offloading = at_station and self.tank_l > 0.0 and not self.deployed
        if self.offloading:
            self.tank_l = max(0.0, self.tank_l - s.offload_lpm / 60.0 * dt)

        if not self.deployed or self.tank_full or not spill.volume.size:
            return
        # The intake: from the skimmer at mid-length to the tips of the arms
        # ahead of the bows, the full swath wide.
        dn, de = spill.north - north, spill.east - east
        ch, sh = math.cos(heading_rad), math.sin(heading_rad)
        ahead = dn * ch + de * sh
        across = -dn * sh + de * ch
        mask = (~spill.beached & (ahead > -0.4) & (ahead < 1.6) & (np.abs(across) < s.swath_m / 2))
        if not mask.any():
            return
        met = float(spill.volume[mask].sum())
        pump_limit = s.max_rate_lpm / 60.0 * dt * s.oil_content
        space = (s.tank_l - self.tank_l) * s.oil_content
        oil = spill.take(mask, min(met * self.efficiency, pump_limit, space))
        if oil <= 0:
            return
        self.recovering = True
        self.recovered_oil_l += oil
        self.tank_l = min(s.tank_l, self.tank_l + oil / s.oil_content)
        self.rate_lpm = oil / dt * 60.0

    def state(self, spill: OilSpill, north: float, east: float) -> SkimmerState:
        film = spill.film_um(north, east)
        # A fluorometer reads dispersed oil in the top metre; a film of 1 µm
        # mixes to roughly 1 ppm there.  Clipped at the instrument's range.
        ppm = min(5000.0, film * 1.0) if spill.volume.size else 0.0
        return SkimmerState(
            fitted=True,
            deployed=self.deployed,
            recovering=self.recovering,
            recovery_lpm=self.rate_lpm,
            efficiency=self.efficiency,
            tank_l=self.tank_l,
            tank_capacity_l=self.settings.tank_l,
            recovered_oil_l=self.recovered_oil_l,
            offloading=self.offloading,
            oil_ppm=round(ppm, 1),
            film_um=round(film, 2),
        )


def spill_start(origin: LatLon, settings: SpillSettings) -> LatLon:
    return geo.destination(origin, settings.bearing_deg, settings.distance_m)
