//! The simulated world the boat floats in: water depth, wind, current, water.
//!
//! The chart grid is about 115 m a cell, far coarser than what an echo
//! sounder sees.  The world adds smooth detail on top of it — sand ridges,
//! holes — so the survey a player runs actually finds something the chart
//! did not show.

use std::sync::Arc;

use crate::core::bathymetry::DepthGrid;
use crate::core::geo::{self, LatLon};
use crate::core::rng::Rng;
use crate::core::telemetry::WaterQuality;

/// A repeatable pseudo-random value in [-1, 1] for an integer lattice point.
fn hash(ix: i64, iy: i64, seed: i64) -> f64 {
    let mut h = (ix as u32)
        .wrapping_mul(374_761_393)
        .wrapping_add((iy as u32).wrapping_mul(668_265_263))
        .wrapping_add((seed as u32).wrapping_mul(2_147_483_647));
    h = (h ^ (h >> 13)).wrapping_mul(1_274_126_177);
    ((h ^ (h >> 16)) & 0xFFFF) as f64 / 32767.5 - 1.0
}

/// Smooth noise in [-1, 1] with features about `wavelength` across.
pub fn value_noise(x: f64, y: f64, wavelength: f64, seed: i64) -> f64 {
    let (fx, fy) = (x / wavelength, y / wavelength);
    let (ix, iy) = (fx.floor(), fy.floor());
    let (tx, ty) = (fx - ix, fy - iy);
    let (sx, sy) = (tx * tx * (3.0 - 2.0 * tx), ty * ty * (3.0 - 2.0 * ty));
    let (ix, iy) = (ix as i64, iy as i64);
    let a = hash(ix, iy, seed);
    let b = hash(ix + 1, iy, seed);
    let c = hash(ix, iy + 1, seed);
    let d = hash(ix + 1, iy + 1, seed);
    (a + (b - a) * sx) * (1.0 - sy) + (c + (d - c) * sx) * sy
}

pub struct World {
    pub chart: Option<Arc<DepthGrid>>,
    pub origin: LatLon,
    pub seed: i64,
    pub wind_speed_ms: f64,
    pub wind_from_deg: f64,
    pub current_speed_ms: f64,
    pub current_to_deg: f64,
    /// A river mouth freshens and clouds the water around it.  The Daugava
    /// by default, since that is where the bundled chart is.
    pub river_mouth: LatLon,
    pub gust: f64,
    rng: Rng,
}

impl World {
    pub fn new(chart: Option<Arc<DepthGrid>>, origin: LatLon, seed: i64) -> Self {
        World {
            chart,
            origin,
            seed,
            wind_speed_ms: 4.0,
            wind_from_deg: 240.0,
            current_speed_ms: 0.15,
            current_to_deg: 60.0,
            river_mouth: LatLon::new(57.058, 24.025),
            gust: 0.0,
            rng: Rng::new(seed as u64),
        }
    }

    // -- depth ------------------------------------------------------------------

    /// True water depth, metres; NaN on land.
    pub fn depth_at(&self, lat: f64, lon: f64) -> f64 {
        let local = geo::to_metres(self.origin, LatLon::new(lat, lon));
        let base = match &self.chart {
            None => {
                // No chart: an endless lake that shoals at the edges of 2 km.
                let r = local.east.hypot(local.north);
                let base = 8.0 * (1.0 - (r / 2000.0).powi(2)).max(0.0);
                if base <= 0.0 {
                    return f64::NAN;
                }
                base
            }
            Some(chart) => {
                let base = chart.depth_at(lat, lon);
                if base.is_nan() {
                    return f64::NAN;
                }
                base
            }
        };
        let detail = 0.22 * value_noise(local.east, local.north, 60.0, self.seed)
            + 0.10 * value_noise(local.east, local.north, 18.0, self.seed + 1);
        // Detail scales with depth, so the shore stays a shore.
        let depth = if base < 3.0 { base * (1.0 + detail) } else { base + 0.66 * detail * 3.0 };
        depth.max(0.0)
    }

    /// Land test on the chart alone (no detail), cheap enough for every oil parcel.
    pub fn is_water_coarse(&self, lat: f64, lon: f64) -> bool {
        match &self.chart {
            None => {
                let local = geo::to_metres(self.origin, LatLon::new(lat, lon));
                local.east.hypot(local.north) < 2000.0
            }
            Some(chart) => {
                let d = chart.depth_nearest(lat, lon);
                d.is_finite() && d > 0.05
            }
        }
    }

    // -- weather ----------------------------------------------------------------

    pub fn step(&mut self, dt: f64) {
        // Gusts: a slow random walk pulled back towards zero.
        self.gust += (-0.3 * self.gust) * dt + self.rng.gauss(0.0, 0.35) * dt.sqrt();
        self.gust = self.gust.clamp(-0.45, 0.6);
    }

    /// True wind velocity (north, east) in m/s: where the air is going.
    pub fn wind_vector(&self) -> (f64, f64) {
        let speed = (self.wind_speed_ms * (1.0 + self.gust)).max(0.0);
        let towards = (self.wind_from_deg + 180.0).to_radians();
        (speed * towards.cos(), speed * towards.sin())
    }

    pub fn current_vector(&self) -> (f64, f64) {
        let towards = self.current_to_deg.to_radians();
        (self.current_speed_ms * towards.cos(), self.current_speed_ms * towards.sin())
    }

    /// The air and water temperature the batteries cool towards, °C.
    pub fn water_temp_c(&self) -> f64 {
        12.5
    }

    // -- water quality ----------------------------------------------------------

    /// A plausible sonde reading for the Gulf of Riga in autumn.
    pub fn water_quality(&self, lat: f64, lon: f64, t: f64) -> WaterQuality {
        let here = LatLon::new(lat, lon);
        let local = geo::to_metres(self.origin, here);
        let mouth = geo::distance_m(here, self.river_mouth);
        let plume = (-mouth / 2500.0).exp();
        let n = value_noise(local.east, local.north, 400.0, self.seed + 5);
        let salinity = (5.9 - 4.8 * plume + 0.15 * n).max(0.1);
        let temperature = 11.8 + 0.9 * plume + 0.3 * value_noise(local.east, local.north, 700.0, 9);
        let conductivity = salinity * 1.72 + 0.25; // mS/cm, brackish water
        let oxygen = 9.6 - 0.8 * plume + 0.25 * n;
        let saturation = oxygen / (14.6 - 0.39 * temperature) * 100.0;
        let jitter = 0.01 * (t * 0.7).sin();
        let round = |v: f64, digits: i32| {
            let k = 10f64.powi(digits);
            (v * k).round() / k
        };
        WaterQuality {
            temperature_c: Some(round(temperature + jitter, 2)),
            ph: Some(round(8.05 - 0.35 * plume + 0.05 * n, 2)),
            dissolved_oxygen_mgl: Some(round(oxygen, 2)),
            oxygen_saturation_pct: Some(round(saturation, 1)),
            conductivity_ms: Some(round(conductivity, 2)),
            tds_ppm: Some((conductivity * 640.0).round()),
            salinity_psu: Some(round(salinity, 2)),
            orp_mv: Some(round(215.0 - 40.0 * plume + 8.0 * n, 1)),
            turbidity_ntu: Some(round(2.0 + 14.0 * plume + 1.5 * n.abs(), 1)),
        }
    }
}
