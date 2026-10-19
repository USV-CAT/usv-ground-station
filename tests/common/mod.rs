#![allow(dead_code)]

use std::sync::{Arc, OnceLock};

use usvgs::config::{Settings, SharedSettings};
use usvgs::core::bathymetry::DepthGrid;
use usvgs::core::geo::LatLon;
use usvgs::link::sim::SimLink;
use usvgs::link::BoatLink;
use usvgs::vehicle::{NavState, Vehicle};

pub fn chart() -> Arc<DepthGrid> {
    static CHART: OnceLock<Arc<DepthGrid>> = OnceLock::new();
    CHART.get_or_init(|| Arc::new(DepthGrid::bundled().expect("bundled chart"))).clone()
}

/// A vehicle and a simulated boat, run in lock-step on a fake clock.
pub struct Rig {
    pub settings: SharedSettings,
    pub link: Arc<SimLink>,
    pub vehicle: Vehicle,
}

impl Rig {
    pub fn new(wind: f64, current: f64) -> Rig {
        let mut s = Settings::default();
        s.sim.gps_noise_m = 0.2;
        s.sim.wind_speed_ms = wind;
        s.sim.current_speed_ms = current;
        Rig::with(s)
    }

    pub fn with(settings: Settings) -> Rig {
        let settings = settings.shared();
        let link = Arc::new(SimLink::new(settings.clone(), Some(chart()), 7));
        let vehicle = Vehicle::new(settings.clone(), link.clone() as Arc<dyn BoatLink>, None, false);
        let rig = Rig { settings, link, vehicle };
        rig.run(0.5, 0.05);
        rig
    }

    pub fn run(&self, seconds: f64, dt: f64) -> Arc<NavState> {
        let steps = (seconds / dt).round() as usize;
        let mut nav = self.vehicle.nav();
        for i in 0..steps {
            self.link.step(dt);
            if i % 2 == 0 {
                self.link.deliver(self.link.telemetry());
            }
            nav = self.vehicle.tick(dt);
        }
        nav
    }

    pub fn position(&self) -> LatLon {
        self.vehicle.nav().position().expect("a fix")
    }
}
