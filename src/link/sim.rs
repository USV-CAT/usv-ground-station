//! The simulated boat, behind the same link interface a real one will have.
//!
//! Physics runs on its own thread at 50 Hz of simulated time.  The link
//! between it and the ground station is modelled too: telemetry at 10 Hz
//! over the best of three radios (Wi-Fi, 5G, LoRa — see [`super::sim_radio`]),
//! each fading with distance and dropping packets towards the edge of its
//! range, and an on-board failsafe that stops the motors when
//! commands stop arriving — which is what a real boat must do, because the
//! ground station's autopilot is on the wrong side of a broken radio link.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::config::{Settings, SharedSettings};
use crate::core::bathymetry::DepthGrid;
use crate::core::clock;
use crate::core::geo::{self, LatLon, Metres};
use crate::core::rng::Rng;
use crate::core::telemetry::Telemetry;
use crate::sim::boat::{Catamaran, MAX_RPM};
use crate::sim::oil::{spill_start, OilSpill, Skimmer, SpillSnapshot, SpillTotals, WINDAGE};
use crate::sim::world::World;

use super::flows::FlowStats;
use super::sim_radio::SimRadio;
use super::{BoatLink, TelemetryHandler};

pub const PHYSICS_HZ: f64 = 50.0;
pub const TELEMETRY_HZ: f64 = 10.0;
/// The boat stops its motors after this long without a command.
pub const COMMAND_TIMEOUT_S: f64 = 1.0;
/// Echo sounders lose the bottom beyond this.
pub const SOUNDER_MAX_M: f64 = 120.0;
/// The ground station's charger, amperes, when the boat is alongside.
pub const SHORE_CHARGE_A: f64 = 8.0;

/// Everything the simulation owns, behind one lock.
pub struct SimCore {
    pub world: World,
    pub boat: Catamaran,
    pub spill: OilSpill,
    pub skimmer: Skimmer,
    /// The ground station: the launch point and the radio's centre.
    pub station: LatLon,
    rng: Rng,
    command: (f64, f64, bool),
    /// Simulated time of the last command.
    command_time: f64,
    uplink_times: VecDeque<f64>,
    gps_error: [f64; 2],
    failsafe: bool,
    oil_dt: f64,
    /// Wi-Fi, 5G and LoRa.
    pub radio: SimRadio,
}

impl SimCore {
    pub fn range_m(&self) -> f64 {
        geo::distance_m(self.station, self.boat.position())
    }

    pub fn rssi_dbm(&self) -> f64 {
        let distance = self.range_m().max(1.0);
        -38.0 - 20.0 * (distance / 10.0).log10() * 0.9
    }

    /// A command or payload order up to the boat: whether it arrives.
    fn uplink(&mut self, settings: &Settings) -> bool {
        let range = self.range_m();
        self.radio.command(&settings.sim, range, clock::monotonic())
    }

    /// Alongside the ground station, slow enough to pump the tank out.
    pub fn at_station(&self) -> bool {
        let (vn, ve) = self.boat.ground_velocity;
        self.range_m() < 15.0 && vn.hypot(ve) < 0.3
    }

    /// How surface oil is moving here: current plus windage, (north, east) m/s.
    pub fn spill_drift(&self) -> (f64, f64) {
        let (wn, we) = self.world.wind_vector();
        let (cn, ce) = self.world.current_vector();
        (cn + WINDAGE * wn, ce + WINDAGE * we)
    }

    pub fn apply_environment(&mut self, settings: &Settings) {
        let sim = &settings.sim;
        self.world.wind_speed_ms = sim.wind_speed_ms;
        self.world.wind_from_deg = sim.wind_from_deg;
        self.world.current_speed_ms = sim.current_speed_ms;
        self.world.current_to_deg = sim.current_to_deg;
    }

    /// A launch point with water under it: the deepest cell nearby if not.
    fn launchable(&self, target: LatLon, need_m: f64) -> LatLon {
        if !(self.world.depth_at(target.lat, target.lon) > need_m) {
            if let Some(chart) = &self.world.chart {
                return chart.deepest_near(target.lat, target.lon, 6).unwrap_or(target);
            }
        }
        target
    }

    fn step(&mut self, settings: &Settings, dt: f64) {
        let (mut port, mut stbd, armed) = self.command;
        let stale = self.boat.state.t - self.command_time > COMMAND_TIMEOUT_S;
        if stale || !armed {
            port = 0.0;
            stbd = 0.0;
        }
        self.world.step(dt);
        self.boat.state.extra_load_a = self.skimmer.load_a();
        // Alongside the station the shore lead charges the pack, tapering
        // off as it fills.
        self.boat.state.charge_a =
            if self.at_station() { SHORE_CHARGE_A * ((1.0 - self.boat.state.battery_soc) * 25.0).clamp(0.0, 1.0) } else { 0.0 };
        self.boat.step(&self.world, dt, port, stbd);
        // The slick and the skimmer change slowly; 10 Hz is plenty and keeps
        // a fast time scale from starving the rest of the program.
        self.oil_dt += dt;
        if self.oil_dt >= 0.1 {
            let at_station = self.at_station();
            let s = self.boat.state.clone();
            self.spill.step(&self.world, self.oil_dt);
            self.skimmer.step(self.oil_dt, &mut self.spill, s.north, s.east, s.heading, s.u, at_station);
            self.oil_dt = 0.0;
        }
        let _ = settings;
        self.failsafe = stale && armed;
    }

    fn telemetry(&mut self, settings: &Settings) -> Telemetry {
        let noise = settings.sim.gps_noise_m;
        // GNSS error wanders rather than jumping, as real receivers do.
        for i in 0..2 {
            self.gps_error[i] += -0.05 * self.gps_error[i] + self.rng.gauss(0.0, noise * 0.3);
        }
        let s = self.boat.state.clone();
        let position = geo::to_lat_lon(
            self.boat.origin,
            Metres::new(s.east + self.gps_error[1], s.north + self.gps_error[0]),
        );
        let (vn, ve) = self.boat.ground_velocity;
        let sog = vn.hypot(ve) + self.rng.gauss(0.0, 0.02).abs();
        let cog = if sog > 0.15 { Some((ve.atan2(vn).to_degrees() + 360.0).rem_euclid(360.0)) } else { None };
        let depth = self.boat.depth_here(&self.world);
        let draft = self.boat.settings.draft_m;
        let sounded = if !depth.is_nan() && draft < depth && depth < SOUNDER_MAX_M {
            Some((depth - draft + self.rng.gauss(0.0, 0.03)).max(0.0))
        } else if s.grounded {
            Some(0.0)
        } else {
            None
        };
        let (air_x, air_y) = self.boat.relative_air;
        let (roll, pitch) = self.boat.attitude(&self.world);
        let now = clock::monotonic();
        while self.uplink_times.front().is_some_and(|&t| now - t > 1.0) {
            self.uplink_times.pop_front();
        }
        let range = self.range_m();
        let heading = (self.boat.heading_deg() + self.rng.gauss(0.0, 0.3)).rem_euclid(360.0);
        Telemetry {
            received: now,
            boat_time_s: Some(s.t),
            lat: Some(position.lat),
            lon: Some(position.lon),
            heading_deg: Some(heading),
            cog_deg: cog,
            sog_ms: Some(sog),
            stw_ms: Some(s.u.max(0.0)),
            rot_dps: Some(s.r.to_degrees()),
            roll_deg: Some(roll),
            pitch_deg: Some(pitch),
            gps_fix: if range < 1500.0 { 5 } else { 3 },
            satellites: (14.0 + (2.0 * (s.t / 60.0).sin()).trunc()) as u32,
            hdop: Some(0.7),
            throttle_port: s.motor_port,
            throttle_stbd: s.motor_stbd,
            rpm_port: s.motor_port * MAX_RPM,
            rpm_stbd: s.motor_stbd * MAX_RPM,
            battery_v: Some(self.boat.battery_voltage()),
            battery_a: Some(s.battery_current_a),
            battery_pct: Some(s.battery_soc * 100.0),
            depth_m: sounded,
            wind_apparent_ms: Some(air_x.hypot(air_y)),
            wind_apparent_deg: Some((-air_y).atan2(-air_x).to_degrees()),
            water: self.world.water_quality(position.lat, position.lon, s.t),
            skimmer: self.skimmer.state(&self.spill, s.north, s.east),
            power: Some(self.boat.power_state(self.at_station())),
            armed: self.command.2,
            onboard_failsafe: self.failsafe,
            grounded: s.grounded,
            rssi_dbm: self.radio.rssi_dbm().or(Some(self.rssi_dbm())),
            uplink_hz: Some(self.uplink_times.len() as f64),
        }
    }
}

struct Shared {
    settings: SharedSettings,
    core: Mutex<SimCore>,
    handler: Mutex<Option<TelemetryHandler>>,
    paused: AtomicBool,
    stop: AtomicBool,
}

impl Shared {
    fn settings(&self) -> Settings {
        self.settings.read().unwrap().clone()
    }

    fn deliver(&self, telemetry: Telemetry) {
        let handler = self.handler.lock().unwrap().clone();
        if let Some(h) = handler {
            h(telemetry);
        }
    }

    fn run(&self) {
        let period = 1.0 / PHYSICS_HZ;
        let per_report = (PHYSICS_HZ / TELEMETRY_HZ) as u64;
        let mut tick: u64 = 0;
        let mut deadline = Instant::now();
        while !self.stop.load(Ordering::Relaxed) {
            if !self.paused.load(Ordering::Relaxed) {
                let settings = self.settings();
                let scale = settings.sim.time_scale.clamp(0.1, 8.0);
                // Large time scales take several fixed steps, so the physics
                // stays stable however fast the game runs.
                let steps = (scale.round() as usize).max(1);
                let report = {
                    let mut core = self.core.lock().unwrap();
                    for _ in 0..steps {
                        core.step(&settings, period * scale / steps as f64);
                    }
                    tick += 1;
                    let range = core.range_m();
                    if tick % per_report == 0 && core.radio.report(&settings.sim, range, clock::monotonic()) {
                        Some(core.telemetry(&settings))
                    } else {
                        None
                    }
                };
                if let Some(t) = report {
                    self.deliver(t);
                }
            }
            deadline += Duration::from_secs_f64(period);
            let now = Instant::now();
            if deadline > now {
                std::thread::sleep(deadline - now);
            } else {
                deadline = now;
            }
        }
    }
}

pub struct SimLink {
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl SimLink {
    pub fn new(settings: SharedSettings, chart: Option<Arc<DepthGrid>>, seed: u64) -> SimLink {
        let s = settings.read().unwrap().clone();
        let station = LatLon::new(s.sim.start_lat, s.sim.start_lon);
        let world = World::new(chart, station, seed as i64);
        let boat = Catamaran::new(s.boat.clone(), &world, station, s.sim.start_heading_deg);
        let spill = OilSpill::new(&world, seed + 4);
        let mut core = SimCore {
            world,
            boat,
            spill,
            skimmer: Skimmer::new(s.skimmer.clone()),
            station,
            rng: Rng::new(seed),
            command: (0.0, 0.0, false),
            command_time: f64::NEG_INFINITY,
            uplink_times: VecDeque::new(),
            gps_error: [0.0, 0.0],
            failsafe: false,
            oil_dt: 0.0,
            radio: SimRadio::new(seed + 11),
        };
        let start = core.launchable(station, s.boat.draft_m + 0.5);
        core.boat.reset(start, s.sim.start_heading_deg);
        core.boat.state.battery_soc = (s.sim.start_battery_pct / 100.0).clamp(0.0, 1.0);
        core.apply_environment(&s);
        let link = SimLink {
            shared: Arc::new(Shared {
                settings,
                core: Mutex::new(core),
                handler: Mutex::new(None),
                paused: AtomicBool::new(false),
                stop: AtomicBool::new(false),
            }),
            thread: Mutex::new(None),
        };
        link.release_spill(None, None);
        link
    }

    /// The simulation itself, locked.  Keep the guard short.
    pub fn core(&self) -> MutexGuard<'_, SimCore> {
        self.shared.core.lock().unwrap()
    }

    fn settings(&self) -> Settings {
        self.shared.settings()
    }

    pub fn paused(&self) -> bool {
        self.shared.paused.load(Ordering::Relaxed)
    }

    pub fn set_paused(&self, on: bool) {
        self.shared.paused.store(on, Ordering::Relaxed);
    }

    // -- control from the game --------------------------------------------------

    pub fn apply_environment(&self) {
        let settings = self.settings();
        self.core().apply_environment(&settings);
    }

    /// Put the boat back in the water, at the launch point unless told otherwise.
    pub fn respawn(&self, position: Option<LatLon>, heading_deg: Option<f64>) {
        let settings = self.settings();
        let mut core = self.core();
        let target = position.unwrap_or(core.station);
        let target = core.launchable(target, settings.boat.draft_m + 0.3);
        core.boat.reset(target, heading_deg.unwrap_or(settings.sim.start_heading_deg));
        core.boat.recharge();
    }

    pub fn recharge(&self) {
        self.core().boat.recharge();
    }

    pub fn set_station(&self, station: LatLon) {
        self.core().station = station;
    }

    pub fn set_chart(&self, chart: Option<Arc<DepthGrid>>) {
        self.core().world.chart = chart;
    }

    /// Put a slick on the water: by default the configured one, off the launch point.
    pub fn release_spill(&self, centre: Option<LatLon>, volume_l: Option<f64>) {
        let settings = self.settings();
        let spill = &settings.spill;
        let mut core = self.core();
        let centre = centre.unwrap_or_else(|| spill_start(core.station, spill));
        let centre = core.launchable(centre, 1.0);
        let SimCore { world, spill: slick, .. } = &mut *core;
        slick.release(world, centre, volume_l.unwrap_or(spill.volume_l), spill.radius_m, spill.parcels);
    }

    pub fn clear_spill(&self) {
        self.core().spill.clear();
    }

    /// The slick's parcels, for drawing; None with no slick.
    pub fn spill_snapshot(&self) -> Option<SpillSnapshot> {
        let core = self.core();
        if core.spill.is_empty() { None } else { Some(core.spill.snapshot()) }
    }

    pub fn spill_version(&self) -> u64 {
        self.core().spill.version
    }

    pub fn spill_totals(&self) -> SpillTotals {
        self.core().spill.totals
    }

    pub fn spill_drift(&self) -> (f64, f64) {
        self.core().spill_drift()
    }

    pub fn range_m(&self) -> f64 {
        self.core().range_m()
    }

    pub fn depth_at(&self, lat: f64, lon: f64) -> f64 {
        self.core().world.depth_at(lat, lon)
    }

    // -- for tests ----------------------------------------------------------------

    /// Advance the simulation by `dt` seconds.
    pub fn step(&self, dt: f64) {
        let settings = self.settings();
        self.core().step(&settings, dt);
    }

    /// What the boat would report right now.
    pub fn telemetry(&self) -> Telemetry {
        let settings = self.settings();
        self.core().telemetry(&settings)
    }

    /// Hand a report to whoever listens, as the radio would.
    pub fn deliver(&self, telemetry: Telemetry) {
        self.shared.deliver(telemetry);
    }
}

impl BoatLink for SimLink {
    fn name(&self) -> &str {
        "SIM"
    }

    fn simulated(&self) -> bool {
        true
    }

    fn open(&self) {
        let mut thread = self.thread.lock().unwrap();
        if thread.is_some() {
            return;
        }
        self.shared.stop.store(false, Ordering::Relaxed);
        let shared = self.shared.clone();
        *thread = Some(
            std::thread::Builder::new().name("sim-boat".into()).spawn(move || shared.run()).expect("sim thread"),
        );
    }

    fn close(&self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.lock().unwrap().take() {
            let _ = handle.join();
        }
    }

    fn send_command(&self, port: f64, stbd: f64, armed: bool) {
        let settings = self.settings();
        let mut core = self.core();
        // Before the radio thread runs (in tests that step by hand) every
        // command arrives.
        if self.thread.lock().unwrap().is_some() && !core.uplink(&settings) {
            return;
        }
        core.command = (port, stbd, armed);
        core.command_time = core.boat.state.t;
        core.uplink_times.push_back(clock::monotonic());
        if core.uplink_times.len() > 64 {
            core.uplink_times.pop_front();
        }
    }

    fn send_payload(&self, name: &str, value: bool) {
        let settings = self.settings();
        let mut core = self.core();
        // A payload order is repeated until it is through: try a few times.
        if self.thread.lock().unwrap().is_some() && !(0..8).any(|_| core.uplink(&settings)) {
            return;
        }
        if name == "skimmer" {
            core.skimmer.deployed = value;
        }
    }

    fn set_telemetry_handler(&self, handler: Option<TelemetryHandler>) {
        *self.shared.handler.lock().unwrap() = handler;
    }

    fn station(&self) -> Option<LatLon> {
        Some(self.core().station)
    }

    fn flows(&self) -> Vec<FlowStats> {
        self.core().radio.flows(clock::monotonic())
    }

    fn set_flow_enabled(&self, index: usize, on: bool) {
        self.core().radio.set_enabled(index, on);
    }

    fn take_messages(&self) -> Vec<String> {
        std::mem::take(&mut self.core().radio.messages)
    }

    fn info(&self) -> Vec<(String, String)> {
        let core = self.core();
        let s = &core.boat.state;
        vec![
            ("Boat".into(), "Simulated catamaran".into()),
            ("Simulated time".into(), crate::core::units::format_duration(Some(s.t))),
            ("From the station".into(), crate::core::units::format_distance(Some(core.range_m()))),
            ("Path in use".into(), core.radio.primary().map(|b| b.label().to_string()).unwrap_or("none".into())),
            ("On-board failsafe".into(), if core.failsafe { "TRIGGERED".into() } else { "armed and watching".into() }),
        ]
    }

    fn as_sim(&self) -> Option<&SimLink> {
        Some(self)
    }
}

impl Drop for SimLink {
    fn drop(&mut self) {
        self.close();
    }
}
