//! An oil slick, and the skimmer that recovers it.
//!
//! **The slick** is a cloud of Lagrangian parcels, the method operational
//! spill models (GNOME, OpenDrift) use.  Each parcel carries a volume of oil
//! and moves with the surface current plus about 3 % of the wind speed — the
//! long-standing rule of thumb for surface oil drift — with a random walk
//! added for spreading.  Light fractions evaporate slowly, and a parcel that
//! drifts ashore beaches and stays there.
//!
//! **The skimmer** is the payload this USV is built around: two sweep arms
//! funnel a swath of water into an oleophilic brush skimmer between the
//! hulls, which lifts the oil into a tank.  It is limited three ways, as real
//! ones are: by the *oil it meets*, by *speed* (booms only hold oil at about
//! a knot; faster, it is carried under them) and by the *pump* and the
//! *tank*, which then has to be emptied alongside the ground station.

use crate::config::{SkimmerSettings, SpillSettings};
use crate::core::geo::{self, LatLon};
use crate::core::rng::Rng;
use crate::core::telemetry::SkimmerState;

use super::world::World;

/// Fraction of the wind speed surface oil drifts at.
pub const WINDAGE: f64 = 0.03;
/// Horizontal diffusion for spreading, m²/s.
pub const DIFFUSION: f64 = 0.03;
/// Evaporation, fraction of the remaining volume per second.
pub const EVAPORATION: f64 = 2.0e-5;
/// Cell used to estimate film thickness, metres.
pub const FILM_CELL_M: f64 = 6.0;

/// Bonn Agreement Oil Appearance Code: (upper thickness µm, code, description).
pub const BONN_CODES: [(f64, &str, &str); 5] = [
    (0.3, "1", "Sheen (silvery/grey)"),
    (5.0, "2", "Rainbow"),
    (50.0, "3", "Metallic"),
    (200.0, "4", "Discontinuous true colour"),
    (f64::INFINITY, "5", "Continuous true colour"),
];

pub fn appearance(film_um: Option<f64>) -> (&'static str, &'static str) {
    match film_um {
        None => ("—", "No oil"),
        Some(f) if f <= 0.04 => ("—", "No oil"),
        Some(f) => BONN_CODES.iter().find(|(limit, _, _)| f < *limit).map(|(_, c, t)| (*c, *t)).unwrap_or(("5", "Continuous true colour")),
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpillTotals {
    pub released_l: f64,
    pub floating_l: f64,
    pub evaporated_l: f64,
    pub beached_l: f64,
    pub recovered_l: f64,
}

impl SpillTotals {
    pub fn recovered_fraction(&self) -> f64 {
        if self.released_l > 0.0 { self.recovered_l / self.released_l } else { 0.0 }
    }
}

/// The slick's parcels as positions, for drawing.
#[derive(Clone, Debug, Default)]
pub struct SpillSnapshot {
    pub lat: Vec<f64>,
    pub lon: Vec<f64>,
    pub volume: Vec<f64>,
    pub beached: Vec<bool>,
    /// Changes whenever the parcels move, so a drawing can tell it is stale.
    pub version: u64,
}

pub struct OilSpill {
    pub origin: LatLon,
    rng: Rng,
    pub north: Vec<f64>,
    pub east: Vec<f64>,
    pub volume: Vec<f64>,
    pub beached: Vec<bool>,
    pub totals: SpillTotals,
    since_beach_check: f64,
    pub version: u64,
}

impl OilSpill {
    pub fn new(world: &World, seed: u64) -> Self {
        OilSpill {
            origin: world.origin,
            rng: Rng::new(seed),
            north: Vec::new(),
            east: Vec::new(),
            volume: Vec::new(),
            beached: Vec::new(),
            totals: SpillTotals::default(),
            since_beach_check: 0.0,
            version: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.volume.is_empty()
    }

    pub fn active(&self) -> bool {
        !self.volume.is_empty() && self.totals.floating_l > 0.05
    }

    /// A fresh spill, stretched downwind as a real one quickly is.
    pub fn release(&mut self, world: &World, centre: LatLon, volume_l: f64, radius_m: f64, parcels: usize) {
        let parcels = parcels.max(1);
        let offset = geo::to_metres(self.origin, centre);
        let (wn, we) = world.wind_vector();
        let heading = if wn != 0.0 || we != 0.0 { we.atan2(wn) } else { 0.0 };
        let (ch, sh) = (heading.cos(), heading.sin());
        self.north.clear();
        self.east.clear();
        let mut weights = Vec::with_capacity(parcels);
        for _ in 0..parcels {
            let along = self.rng.gauss(0.0, radius_m * 0.75);
            let across = self.rng.gauss(0.0, radius_m * 0.35);
            self.north.push(offset.north + along * ch - across * sh);
            self.east.push(offset.east + along * sh + across * ch);
            // A thick core and a thin fringe, as a slick actually looks.
            weights.push((-(along * along + (2.0 * across).powi(2)) / (2.0 * radius_m * radius_m)).exp() + 0.15);
        }
        let total: f64 = weights.iter().sum();
        self.volume = weights.iter().map(|w| w / total * volume_l).collect();
        self.beached = vec![false; parcels];
        self.totals = SpillTotals { released_l: volume_l, floating_l: volume_l, ..SpillTotals::default() };
        self.drop_land(world);
        self.version += 1;
    }

    pub fn clear(&mut self) {
        self.north.clear();
        self.east.clear();
        self.volume.clear();
        self.beached.clear();
        self.totals = SpillTotals::default();
        self.version += 1;
    }

    pub fn position(&self, i: usize) -> LatLon {
        geo::to_lat_lon(self.origin, geo::Metres::new(self.east[i], self.north[i]))
    }

    pub fn snapshot(&self) -> SpillSnapshot {
        let mut s = SpillSnapshot { version: self.version, ..SpillSnapshot::default() };
        for i in 0..self.volume.len() {
            let p = self.position(i);
            s.lat.push(p.lat);
            s.lon.push(p.lon);
        }
        s.volume = self.volume.clone();
        s.beached = self.beached.clone();
        s
    }

    pub fn step(&mut self, world: &World, dt: f64) {
        if self.volume.is_empty() {
            return;
        }
        let (wn, we) = world.wind_vector();
        let (cn, ce) = world.current_vector();
        let spread = (2.0 * DIFFUSION * dt).sqrt();
        let mut lost = 0.0;
        for i in 0..self.volume.len() {
            if self.beached[i] {
                continue;
            }
            self.north[i] += (cn + WINDAGE * wn) * dt + self.rng.gauss(0.0, spread);
            self.east[i] += (ce + WINDAGE * we) * dt + self.rng.gauss(0.0, spread);
            let gone = self.volume[i] * EVAPORATION * dt;
            self.volume[i] -= gone;
            lost += gone;
        }
        self.totals.evaporated_l += lost;
        self.since_beach_check += dt;
        if self.since_beach_check >= 1.0 {
            self.since_beach_check = 0.0;
            self.beach(world);
        }
        self.tally();
        self.version += 1;
    }

    fn on_land(&self, world: &World, i: usize) -> bool {
        let p = self.position(i);
        !world.is_water_coarse(p.lat, p.lon)
    }

    /// Parcels that start on land are discarded: the spill is what is on the water.
    fn drop_land(&mut self, world: &World) {
        for i in 0..self.volume.len() {
            if self.on_land(world, i) {
                self.totals.released_l -= self.volume[i];
                self.volume[i] = 0.0;
            }
        }
        self.tally();
    }

    fn beach(&mut self, world: &World) {
        for i in 0..self.volume.len() {
            if !self.beached[i] && self.volume[i] > 0.0 && self.on_land(world, i) {
                self.beached[i] = true;
                self.totals.beached_l += self.volume[i];
            }
        }
    }

    fn tally(&mut self) {
        self.totals.floating_l =
            self.volume.iter().zip(&self.beached).filter(|(_, b)| !**b).map(|(v, _)| *v).sum();
    }

    // -- at the boat ------------------------------------------------------------

    /// Film thickness around a point: oil volume over area, in micrometres.
    pub fn film_um(&self, north: f64, east: f64, cell_m: f64) -> f64 {
        let half = cell_m / 2.0;
        let litres: f64 = (0..self.volume.len())
            .filter(|&i| !self.beached[i] && (self.north[i] - north).abs() < half && (self.east[i] - east).abs() < half)
            .map(|i| self.volume[i])
            .sum();
        litres * 1e-3 / (cell_m * cell_m) * 1e6
    }

    /// Remove `litres` of oil from the parcels in `mask`, in proportion.
    pub fn take(&mut self, mask: &[bool], litres: f64) -> f64 {
        let available: f64 = self.volume.iter().zip(mask).filter(|(_, m)| **m).map(|(v, _)| *v).sum();
        if available <= 0.0 || litres <= 0.0 {
            return 0.0;
        }
        let taken = available.min(litres);
        let keep = 1.0 - taken / available;
        for (v, m) in self.volume.iter_mut().zip(mask) {
            if *m {
                *v *= keep;
            }
        }
        self.totals.recovered_l += taken;
        taken
    }
}

pub struct Skimmer {
    pub settings: SkimmerSettings,
    pub deployed: bool,
    pub tank_l: f64,
    pub recovered_oil_l: f64,
    pub rate_lpm: f64,
    pub efficiency: f64,
    pub offloading: bool,
    pub recovering: bool,
}

impl Skimmer {
    pub fn new(settings: SkimmerSettings) -> Self {
        Skimmer {
            settings,
            deployed: false,
            tank_l: 0.0,
            recovered_oil_l: 0.0,
            rate_lpm: 0.0,
            efficiency: 0.0,
            offloading: false,
            recovering: false,
        }
    }

    pub fn tank_full(&self) -> bool {
        self.tank_l >= self.settings.tank_l - 1e-6
    }

    pub fn load_a(&self) -> f64 {
        if self.deployed { self.settings.power_a } else { 0.0 }
    }

    /// Share of the oil met that the arms hold and the brush lifts.
    pub fn efficiency_at(&self, speed_ms: f64) -> f64 {
        efficiency_at(&self.settings, speed_ms)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, dt: f64, spill: &mut OilSpill, north: f64, east: f64, heading_rad: f64, speed_through_water: f64, at_station: bool) {
        let s = self.settings.clone();
        self.rate_lpm = 0.0;
        self.recovering = false;
        self.efficiency = if self.deployed { self.efficiency_at(speed_through_water) } else { 0.0 };

        self.offloading = at_station && self.tank_l > 0.0 && !self.deployed;
        if self.offloading {
            self.tank_l = (self.tank_l - s.offload_lpm / 60.0 * dt).max(0.0);
        }
        if !self.deployed || self.tank_full() || spill.is_empty() {
            return;
        }
        // The intake: from the skimmer at mid-length to the tips of the arms
        // ahead of the bows, the full swath wide.
        let (ch, sh) = (heading_rad.cos(), heading_rad.sin());
        let mask: Vec<bool> = (0..spill.volume.len())
            .map(|i| {
                let (dn, de) = (spill.north[i] - north, spill.east[i] - east);
                let ahead = dn * ch + de * sh;
                let across = -dn * sh + de * ch;
                !spill.beached[i] && ahead > -0.4 && ahead < 1.6 && across.abs() < s.swath_m / 2.0
            })
            .collect();
        if !mask.iter().any(|m| *m) {
            return;
        }
        let met: f64 = spill.volume.iter().zip(&mask).filter(|(_, m)| **m).map(|(v, _)| *v).sum();
        let pump_limit = s.max_rate_lpm / 60.0 * dt * s.oil_content;
        let space = (s.tank_l - self.tank_l) * s.oil_content;
        let oil = spill.take(&mask, (met * self.efficiency).min(pump_limit).min(space));
        if oil <= 0.0 {
            return;
        }
        self.recovering = true;
        self.recovered_oil_l += oil;
        self.tank_l = (self.tank_l + oil / s.oil_content).min(s.tank_l);
        self.rate_lpm = oil / dt * 60.0;
    }

    pub fn state(&self, spill: &OilSpill, north: f64, east: f64) -> SkimmerState {
        let film = spill.film_um(north, east, FILM_CELL_M);
        // A fluorometer reads dispersed oil in the top metre; a film of 1 µm
        // mixes to roughly 1 ppm there.  Clipped at the instrument's range.
        let ppm = if spill.is_empty() { 0.0 } else { film.min(5000.0) };
        SkimmerState {
            fitted: true,
            deployed: self.deployed,
            recovering: self.recovering,
            recovery_lpm: self.rate_lpm,
            efficiency: self.efficiency,
            tank_l: self.tank_l,
            tank_capacity_l: self.settings.tank_l,
            recovered_oil_l: self.recovered_oil_l,
            offloading: self.offloading,
            oil_ppm: Some((ppm * 10.0).round() / 10.0),
            film_um: Some((film * 100.0).round() / 100.0),
        }
    }
}

/// Share of the oil met that the arms hold and the brush lifts, at a speed.
pub fn efficiency_at(s: &SkimmerSettings, speed_ms: f64) -> f64 {
    let speed = speed_ms.abs();
    if speed <= s.best_speed_ms {
        return 0.85;
    }
    if speed >= s.max_speed_ms * 2.0 {
        return 0.05;
    }
    0.85 - 0.8 * (speed - s.best_speed_ms) / (s.max_speed_ms * 2.0 - s.best_speed_ms)
}

pub fn spill_start(origin: LatLon, settings: &SpillSettings) -> LatLon {
    geo::destination(origin, settings.bearing_deg, settings.distance_m)
}
