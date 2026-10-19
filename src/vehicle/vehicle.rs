//! The boat as the ground station sees it: the single source of truth.
//!
//! [`VehicleCore`] owns the latest telemetry, the mode, the mission, the
//! autopilot, the safety checks, the track and the survey.  [`Vehicle`] runs
//! one control loop at 20 Hz that decides what the motors should do and
//! sends it.  The interface reads [`Vehicle::nav`] and calls the command
//! methods on the core; it holds no boat state of its own.

use std::collections::{HashSet, VecDeque};
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde_json::json;

use crate::config::{state_dir, Settings, SharedSettings};
use crate::core::bathymetry::SoundingGrid;
use crate::core::clock;
use crate::core::energy::{estimate, EnergyEstimate, PowerSample, NOMINAL_V};
use crate::core::events::{EventBus, EventKind, Level};
use crate::core::geo::{self, LatLon, Metres};
use crate::core::mission::{Mission, Rectangle};
use crate::core::telemetry::Telemetry;
use crate::core::vectors;
use crate::link::BoatLink;

use super::autopilot::{line_of_sight, mix, LegGuidance, Pilot, StationKeeper};
use super::modes::Mode;

pub const CONTROL_HZ: f64 = 20.0;
/// Track points are kept at least this far apart, metres.
pub const TRACK_SPACING_M: f64 = 1.5;
pub const TRACK_LIMIT: usize = 50_000;
/// One power sample a second, for four hours.
pub const POWER_HISTORY: usize = 4 * 3600;

#[derive(Clone, Debug, PartialEq)]
pub struct Alarm {
    pub key: &'static str,
    pub text: String,
    pub level: Level,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackPoint {
    pub t: f64,
    pub lat: f64,
    pub lon: f64,
    pub depth_m: Option<f64>,
}

/// Everything the displays need, computed once per control tick.
#[derive(Clone, Debug, PartialEq)]
pub struct NavState {
    pub mode: Mode,
    pub armed: bool,
    pub link_ok: bool,
    pub downlink_hz: f64,
    pub telemetry: Option<Telemetry>,
    pub home: Option<LatLon>,
    pub home_distance_m: Option<f64>,
    pub home_bearing_deg: Option<f64>,
    pub station_distance_m: Option<f64>,
    /// Where the autopilot is going, and what it calls it.
    pub target: Option<LatLon>,
    pub target_label: String,
    pub waypoint_index: Option<usize>,
    pub waypoint_count: usize,
    pub bearing_to_target_deg: Option<f64>,
    pub distance_to_target_m: Option<f64>,
    pub cross_track_m: Option<f64>,
    pub time_to_go_s: Option<f64>,
    pub mission_remaining_m: Option<f64>,
    pub speed_setpoint_ms: f64,
    pub course_setpoint_deg: Option<f64>,
    /// What was sent to the motors this tick.
    pub command_port: f64,
    pub command_stbd: f64,
    /// Most urgent first.
    pub alarms: Vec<Alarm>,
    pub trip_m: f64,
    pub run_time_s: f64,
    pub coverage: f64,
    pub soundings: usize,
    pub groundings: u32,
    pub manual_input: bool,
    pub energy: Option<EnergyEstimate>,
    /// Smoothed true wind (speed m/s, from degrees) and current (drift m/s, set degrees).
    pub true_wind: Option<(f64, f64)>,
    pub current: Option<(f64, f64)>,
    pub skimmer_commanded: bool,
    /// Energy put back from shore since the start, Wh.
    pub charged_wh: f64,
    /// The motion limit holding the boat back this tick, if any.
    pub limit: Option<super::limits::Limit>,
}

impl Default for NavState {
    fn default() -> Self {
        NavState {
            mode: Mode::Manual,
            armed: false,
            link_ok: false,
            downlink_hz: 0.0,
            telemetry: None,
            home: None,
            home_distance_m: None,
            home_bearing_deg: None,
            station_distance_m: None,
            target: None,
            target_label: String::new(),
            waypoint_index: None,
            waypoint_count: 0,
            bearing_to_target_deg: None,
            distance_to_target_m: None,
            cross_track_m: None,
            time_to_go_s: None,
            mission_remaining_m: None,
            speed_setpoint_ms: 0.0,
            course_setpoint_deg: None,
            command_port: 0.0,
            command_stbd: 0.0,
            alarms: Vec::new(),
            trip_m: 0.0,
            run_time_s: 0.0,
            coverage: 0.0,
            soundings: 0,
            groundings: 0,
            manual_input: false,
            energy: None,
            true_wind: None,
            current: None,
            skimmer_commanded: false,
            charged_wh: 0.0,
            limit: None,
        }
    }
}

impl NavState {
    pub fn position(&self) -> Option<LatLon> {
        self.telemetry.as_ref().and_then(Telemetry::position)
    }

    pub fn has_alarm(&self, key: &str) -> bool {
        self.alarms.iter().any(|a| a.key == key)
    }
}

/// The operator's sticks, already mapped to motor and helm terms.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Operator {
    pub port: f64,
    pub stbd: f64,
    pub throttle: f64,
    pub steer: f64,
    pub deflection: f64,
    pub connected: bool,
}

struct SessionLog {
    out: std::io::BufWriter<std::fs::File>,
    last: f64,
}

pub struct VehicleCore {
    settings: SharedSettings,
    link: Arc<dyn BoatLink>,
    bus: Arc<EventBus>,
    pub mission: Mission,
    pilot: Pilot,
    keeper: StationKeeper,
    limiter: super::limits::Limiter,

    telemetry: Option<Telemetry>,
    arrivals: VecDeque<f64>,
    operator: Operator,
    mode: Mode,
    armed: bool,
    home: Option<LatLon>,
    hold_point: Option<LatLon>,
    point_target: Option<LatLon>,
    leg_start: Option<LatLon>,
    wp_index: usize,
    wp_hold_until: Option<f64>,
    speed_setpoint: f64,
    course_setpoint: Option<f64>,
    nav: Arc<NavState>,
    alarms: Vec<Alarm>,
    latched: HashSet<&'static str>,
    link_was_ok: bool,
    armed_time: f64,
    trip_m: f64,
    last_track: Option<LatLon>,
    pub track: Vec<TrackPoint>,
    pub track_version: u64,

    pub soundings: SoundingGrid,
    pub groundings: u32,
    was_grounded: bool,

    /// One sample a second of the battery, for the power page.
    pub power_history: VecDeque<PowerSample>,
    energy_wh: f64,
    /// Energy put back into the pack from shore, Wh.
    pub charged_wh: f64,
    last_power_t: Option<f64>,
    last_energy: (f64, Option<EnergyEstimate>),
    wind_vec: Option<(f64, f64)>,
    current_vec: Option<(f64, f64)>,
    skimmer: bool,
    drift_t: Option<f64>,
    coverage_cache: (Option<Rectangle>, usize, f64),
    log: Option<SessionLog>,
    pub log_path: Option<PathBuf>,
}

impl VehicleCore {
    fn new(settings: SharedSettings, link: Arc<dyn BoatLink>, bus: Arc<EventBus>) -> Self {
        let s = settings.read().unwrap().clone();
        let origin = LatLon::new(s.sim.start_lat, s.sim.start_lon);
        VehicleCore {
            settings,
            link,
            bus,
            mission: Mission::default(),
            pilot: Pilot::default(),
            keeper: StationKeeper::new(s.autopilot.hold_radius_m),
            limiter: super::limits::Limiter::default(),
            telemetry: None,
            arrivals: VecDeque::new(),
            operator: Operator::default(),
            mode: Mode::Manual,
            armed: false,
            home: None,
            hold_point: None,
            point_target: None,
            leg_start: None,
            wp_index: 0,
            wp_hold_until: None,
            speed_setpoint: s.autopilot.cruise_speed_ms,
            course_setpoint: None,
            nav: Arc::new(NavState::default()),
            alarms: Vec::new(),
            latched: HashSet::new(),
            link_was_ok: false,
            armed_time: 0.0,
            trip_m: 0.0,
            last_track: None,
            track: Vec::new(),
            track_version: 0,
            soundings: SoundingGrid::new(origin, s.survey.cell_m),
            groundings: 0,
            was_grounded: false,
            power_history: VecDeque::new(),
            energy_wh: 0.0,
            charged_wh: 0.0,
            last_power_t: None,
            last_energy: (-1.0, None),
            wind_vec: None,
            current_vec: None,
            skimmer: false,
            drift_t: None,
            coverage_cache: (None, usize::MAX, 0.0),
            log: None,
            log_path: None,
        }
    }

    fn settings(&self) -> Settings {
        self.settings.read().unwrap().clone()
    }

    fn position(&self) -> Option<LatLon> {
        self.telemetry.as_ref().and_then(Telemetry::position)
    }

    // ------------------------------------------------------------------
    // Inputs
    // ------------------------------------------------------------------

    pub fn on_telemetry(&mut self, telemetry: Telemetry) {
        self.arrivals.push_back(telemetry.received);
        if self.arrivals.len() > 64 {
            self.arrivals.pop_front();
        }
        if let Some(here) = telemetry.position() {
            if self.home.is_none() {
                self.home = Some(here);
                self.bus.message("Home set at first GNSS fix", Level::Notice);
            }
            if let Some(depth) = telemetry.depth_m {
                let swath = self.settings.read().unwrap().survey.swath_m;
                self.soundings.add(here.lat, here.lon, depth, swath, None);
            }
            self.update_track(&telemetry, here);
        }
        self.update_power(&telemetry);
        self.update_vectors(&telemetry);
        self.telemetry = Some(telemetry);
    }

    fn update_power(&mut self, t: &Telemetry) {
        let (Some(boat_t), Some(volts), Some(amps)) = (t.boat_time_s, t.battery_v, t.battery_a) else { return };
        let mut last = self.last_power_t;
        if last.is_some_and(|l| boat_t < l) {
            last = None; // the boat restarted
        }
        if let Some(l) = last {
            let dt = boat_t - l;
            if dt < 1.0 {
                return;
            }
            // Energy drawn and energy taken in from shore are kept apart, so
            // "used" never runs backwards while the pack charges.
            let wh = volts * amps * dt / 3600.0;
            if wh >= 0.0 {
                self.energy_wh += wh;
            } else {
                self.charged_wh -= wh;
            }
        }
        self.last_power_t = Some(boat_t);
        self.power_history.push_back(PowerSample {
            t: boat_t,
            volts,
            amps,
            percent: t.battery_pct.unwrap_or(0.0),
            sog: t.sog_ms.unwrap_or(0.0),
            energy_wh: self.energy_wh,
        });
        if self.power_history.len() > POWER_HISTORY {
            self.power_history.pop_front();
        }
    }

    /// Smoothed true wind and current, from the vector triangles.
    fn update_vectors(&mut self, t: &Telemetry) {
        let (Some(heading), Some(sog)) = (t.heading_deg, t.sog_ms) else { return };
        let alpha = 0.05;
        let blend = |old: Option<(f64, f64)>, new: (f64, f64), k: f64| match old {
            None => Some(new),
            Some(o) => Some((o.0 + (new.0 - o.0) * k, o.1 + (new.1 - o.1) * k)),
        };
        if let (Some(aws), Some(awa)) = (t.wind_apparent_ms, t.wind_apparent_deg) {
            let (speed, source) = vectors::true_wind(aws, awa, heading, sog, t.cog_deg);
            self.wind_vec = blend(self.wind_vec, vectors::from_polar(speed, source), alpha);
        }
        if let (Some(stw), Some(cog)) = (t.stw_ms, t.cog_deg) {
            if sog > 0.15 {
                let (drift, set) = vectors::current(sog, Some(cog), stw, heading);
                self.current_vec = blend(self.current_vec, vectors::from_polar(drift, set), alpha * 0.5);
            }
        }
    }

    fn update_track(&mut self, telemetry: &Telemetry, here: LatLon) {
        if let Some(last) = self.last_track {
            let step = geo::distance_m(last, here);
            if step < TRACK_SPACING_M {
                return;
            }
            if step < 50.0 {
                self.trip_m += step;
            }
        }
        self.last_track = Some(here);
        self.track.push(TrackPoint { t: clock::unix_time(), lat: here.lat, lon: here.lon, depth_m: telemetry.depth_m });
        if self.track.len() > TRACK_LIMIT {
            self.track.drain(..TRACK_LIMIT / 10);
        }
        self.track_version += 1;
    }

    pub fn set_operator(&mut self, operator: Operator) {
        self.operator = operator;
    }

    // ------------------------------------------------------------------
    // Commands
    // ------------------------------------------------------------------

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn armed(&self) -> bool {
        self.armed
    }

    pub fn home(&self) -> Option<LatLon> {
        self.home
    }

    pub fn point_target(&self) -> Option<LatLon> {
        self.point_target
    }

    pub fn telemetry(&self) -> Option<&Telemetry> {
        self.telemetry.as_ref()
    }

    pub fn arm(&mut self) -> bool {
        let safety = self.settings().safety;
        let Some(t) = &self.telemetry else {
            self.bus.message("Cannot arm: no telemetry from the boat", Level::Warning);
            return false;
        };
        if t.age(clock::monotonic()) > safety.link_timeout_s {
            self.bus.message("Cannot arm: no telemetry from the boat", Level::Warning);
            return false;
        }
        if !t.has_position() {
            self.bus.message("Cannot arm: no GNSS fix", Level::Warning);
            return false;
        }
        if t.battery_pct.is_some_and(|b| b < safety.battery_critical_pct) {
            self.bus.message("Cannot arm: battery critical", Level::Warning);
            return false;
        }
        self.armed = true;
        self.set_mode_for(Mode::Manual, "armed");
        self.bus.emit(EventKind::ArmingState, "ARMED — motors live", Level::Notice, json!({"armed": true}));
        true
    }

    pub fn disarm(&mut self) {
        if self.armed {
            self.armed = false;
            self.bus.emit(EventKind::ArmingState, "Disarmed", Level::Notice, json!({"armed": false}));
        }
    }

    pub fn toggle_arm(&mut self) {
        if self.armed {
            self.disarm();
        } else {
            self.arm();
        }
    }

    pub fn set_mode(&mut self, mode: Mode) -> bool {
        self.set_mode_for(mode, "operator")
    }

    fn set_mode_for(&mut self, mode: Mode, reason: &str) -> bool {
        let t = self.telemetry.clone();
        let position = self.position();
        if mode == Mode::Auto && self.mission.waypoints.is_empty() {
            self.bus.message("No mission to run — plan one on the chart first", Level::Warning);
            return false;
        }
        if mode == Mode::PointNav && self.point_target.is_none() {
            self.bus.message("Tap the chart to choose a point first", Level::Warning);
            return false;
        }
        if mode.autonomous() && position.is_none() {
            self.bus.message(format!("{} needs a GNSS fix", mode.label()), Level::Warning);
            return false;
        }

        self.pilot.reset();
        match mode {
            Mode::Hold => self.hold_point = position,
            Mode::ReturnHome | Mode::PointNav => {
                self.hold_point = None;
                self.leg_start = position;
            }
            Mode::Auto => {
                self.leg_start = position;
                // Resume where the mission was left, or start it from the top.
                if self.wp_index >= self.mission.waypoints.len() {
                    self.wp_index = 0;
                }
                self.wp_hold_until = None;
                self.hold_point = None;
            }
            Mode::CourseHold => {
                let sog = t.as_ref().and_then(|t| t.sog_ms).unwrap_or(0.0);
                let cog = t.as_ref().and_then(|t| t.cog_deg);
                self.course_setpoint = match (cog, sog > 0.4) {
                    (Some(c), true) => Some(c),
                    _ => Some(t.as_ref().and_then(|t| t.heading_deg).unwrap_or(0.0)),
                };
                let current = t.as_ref().and_then(|t| t.sog_ms).filter(|s| *s != 0.0).unwrap_or(self.speed_setpoint);
                self.speed_setpoint = current.max(0.3);
            }
            Mode::SpeedHold => {
                if let Some(sog) = t.as_ref().and_then(|t| t.sog_ms) {
                    if sog > 0.3 {
                        self.speed_setpoint = sog;
                    }
                }
            }
            _ => {}
        }

        if mode != self.mode {
            let previous = self.mode;
            self.mode = mode;
            let level = if reason == "operator" || reason == "armed" { Level::Info } else { Level::Warning };
            self.bus.emit(
                EventKind::Mode,
                format!("Mode {} ({reason})", mode.label()),
                level,
                json!({"mode": mode.value(), "previous": previous.value()}),
            );
        }
        true
    }

    pub fn set_mission(&mut self, mission: Mission) {
        self.mission = mission;
        self.wp_index = 0;
        if self.mode == Mode::Auto {
            self.set_mode_for(Mode::Hold, "mission replaced");
        }
    }

    pub fn restart_mission(&mut self) {
        self.wp_index = 0;
        if self.mode == Mode::Auto {
            self.leg_start = self.position();
        }
    }

    pub fn skip_waypoint(&mut self, offset: i64) {
        let count = self.mission.waypoints.len();
        if count > 0 {
            self.wp_index = (self.wp_index as i64 + offset).clamp(0, count as i64 - 1) as usize;
            self.leg_start = self.position();
        }
    }

    /// Point navigation: drive to a tapped point, then hold there.
    pub fn go_to(&mut self, lat: f64, lon: f64) {
        self.point_target = Some(LatLon::new(lat, lon));
        self.set_mode_for(Mode::PointNav, "operator");
        if self.mode == Mode::PointNav {
            self.leg_start = self.position();
            self.hold_point = None;
        }
    }

    /// Nudge the speed setpoint; outside the hold modes it is the cruise speed.
    pub fn adjust_speed(&mut self, delta_ms: f64) -> f64 {
        let max = self.settings.read().unwrap().limits.max_speed_ms;
        self.speed_setpoint = (self.speed_setpoint + delta_ms).clamp(0.0, max);
        if !matches!(self.mode, Mode::SpeedHold | Mode::CourseHold) {
            self.settings.write().unwrap().autopilot.cruise_speed_ms = self.speed_setpoint.max(0.3);
        }
        self.speed_setpoint
    }

    pub fn adjust_course(&mut self, delta_deg: f64) {
        if let Some(c) = self.course_setpoint {
            self.course_setpoint = Some((c + delta_deg).rem_euclid(360.0));
        }
    }

    pub fn set_home_here(&mut self) {
        if let Some(here) = self.position() {
            self.home = Some(here);
            self.bus.message("Home moved to the boat's position", Level::Notice);
        }
    }

    pub fn clear_track(&mut self) {
        self.track.clear();
        self.last_track = None;
        self.track_version += 1;
    }

    pub fn set_skimmer(&mut self, on: bool) {
        self.skimmer = on;
        self.link.send_payload("skimmer", on);
        self.bus.message(
            if on { "Skimmer deployed — keep under 1.5 kn in the oil" } else { "Skimmer stowed" },
            Level::Notice,
        );
    }

    pub fn toggle_skimmer(&mut self) {
        let on = !self.skimmer;
        self.set_skimmer(on);
    }

    pub fn skimmer_commanded(&self) -> bool {
        self.skimmer
    }

    pub fn acknowledge_alarms(&mut self) {
        self.latched.clear();
    }

    // ------------------------------------------------------------------
    // The control step
    // ------------------------------------------------------------------

    /// One control step.
    pub fn tick(&mut self, dt: f64) -> Arc<NavState> {
        let mut cfg = self.settings();
        // No speed asked of the autopilot may exceed the limit.
        let max_speed = cfg.limits.max_speed_ms;
        cfg.autopilot.cruise_speed_ms = cfg.autopilot.cruise_speed_ms.min(max_speed);
        self.speed_setpoint = self.speed_setpoint.min(max_speed);
        let now = clock::monotonic();
        let link_ok = self.telemetry.as_ref().is_some_and(|t| t.age(now) <= cfg.safety.link_timeout_s);
        self.check_safety(&cfg, link_ok);
        self.drift_route();
        let (mut port, mut stbd, guidance) = self.command(&cfg, link_ok, dt);
        if !self.armed {
            port = 0.0;
            stbd = 0.0;
        }
        // The limits shape every command but a stop, which is immediate.
        let limited = self.armed && self.mode != Mode::Stop && (self.mode != Mode::Manual || cfg.limits.in_manual);
        if limited {
            let t = self.telemetry.as_ref();
            let sog = t.and_then(|t| t.sog_ms).unwrap_or(0.0);
            let rot = t.and_then(|t| t.rot_dps).unwrap_or(0.0);
            (port, stbd) = self.limiter.apply(port, stbd, &cfg.limits, sog, rot, dt);
        } else {
            self.limiter.reset();
        }
        self.link.send_command(port, stbd, self.armed);
        if let Some(t) = &self.telemetry {
            if t.skimmer.fitted && t.skimmer.deployed != self.skimmer {
                self.link.send_payload("skimmer", self.skimmer);
            }
        }
        if self.armed {
            self.armed_time += dt;
        }
        self.nav = Arc::new(self.build_nav(&cfg, link_ok, port, stbd, guidance));
        self.write_log(port, stbd);
        self.nav.clone()
    }

    pub fn nav(&self) -> Arc<NavState> {
        self.nav.clone()
    }

    /// Carry a drift-relative route (an oil sweep) along with the water.
    fn drift_route(&mut self) {
        let Some(drift) = self.mission.drift_ms else { return };
        let Some(boat_t) = self.telemetry.as_ref().and_then(|t| t.boat_time_s) else { return };
        let last = self.drift_t.replace(boat_t);
        let Some(last) = last else { return };
        let step = boat_t - last;
        if !(0.0 < step && step < 5.0) {
            return;
        }
        self.mission.shift(drift.0 * step, drift.1 * step);
        if let Some(start) = self.leg_start {
            self.leg_start = Some(geo::to_lat_lon(start, Metres::new(drift.1 * step, drift.0 * step)));
        }
    }

    /// Motor levels for the current mode, and the guidance used to get them.
    fn command(&mut self, cfg: &Settings, link_ok: bool, dt: f64) -> (f64, f64, Option<LegGuidance>) {
        let op = self.operator;
        let ap = &cfg.autopilot;
        let mut mode = self.mode;

        // Taking the sticks back from the autopilot.
        if mode.autonomous() && op.connected && op.deflection > cfg.control.override_threshold {
            self.set_mode_for(Mode::Manual, "stick override");
            mode = Mode::Manual;
        }
        if mode == Mode::Manual {
            return (op.port, op.stbd, None);
        }
        let Some(t) = self.telemetry.clone() else { return (0.0, 0.0, None) };
        if mode == Mode::Stop || !link_ok {
            return (0.0, 0.0, None);
        }
        let heading = t.heading_deg.unwrap_or(0.0);
        let rot = t.rot_dps.unwrap_or(0.0);
        let sog = t.sog_ms.unwrap_or(0.0);
        let cog = t.cog_deg;

        match mode {
            Mode::SpeedHold => {
                let throttle = self.pilot.speed.update(self.speed_setpoint, sog, dt);
                let (p, s) = mix(throttle, op.steer);
                return (p, s, None);
            }
            Mode::CourseHold => {
                if op.steer.abs() > 0.1 {
                    self.course_setpoint =
                        Some((self.course_setpoint.unwrap_or(heading) + op.steer * 30.0 * dt).rem_euclid(360.0));
                }
                if op.throttle.abs() > 0.2 {
                    self.speed_setpoint = (self.speed_setpoint + op.throttle * 0.4 * dt).clamp(0.0, cfg.limits.max_speed_ms);
                }
                let course = self.course_setpoint.unwrap_or(heading);
                let (p, s) = self.pilot.steer_course(course, self.speed_setpoint, heading, rot, cog, sog, dt, true);
                return (p, s, None);
            }
            _ => {}
        }

        let Some(position) = t.position() else { return (0.0, 0.0, None) };
        match mode {
            Mode::Hold => {
                let target = self.hold_point.unwrap_or(position);
                self.hold_point = Some(target);
                let (p, s) = self.keeper.update(&mut self.pilot, target, position, heading, rot, cog, sog, ap.cruise_speed_ms, dt);
                (p, s, None)
            }
            Mode::ReturnHome | Mode::PointNav => {
                let target = if mode == Mode::ReturnHome { self.home } else { self.point_target };
                let Some(target) = target else { return (0.0, 0.0, None) };
                if let Some(hold) = self.hold_point {
                    let (p, s) = self.keeper.update(&mut self.pilot, hold, position, heading, rot, cog, sog, ap.cruise_speed_ms, dt);
                    return (p, s, None);
                }
                let start = self.leg_start.unwrap_or(position);
                let guidance = line_of_sight(start, target, position, ap.lookahead_m);
                if guidance.distance_to_go_m < ap.accept_radius_m {
                    self.hold_point = Some(target);
                    let what = if mode == Mode::ReturnHome { "home" } else { "the point" };
                    self.bus.message(format!("Arrived at {what} — holding station"), Level::Notice);
                    return (0.0, 0.0, Some(guidance));
                }
                let speed = ap.cruise_speed_ms.min((guidance.distance_to_go_m * 0.15).max(0.4));
                let (p, s) = self.pilot.steer_course(guidance.course_deg, speed, heading, rot, cog, sog, dt, true);
                (p, s, Some(guidance))
            }
            Mode::Auto => self.run_mission(cfg, position, heading, rot, cog, sog, dt),
            _ => (0.0, 0.0, None),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn run_mission(
        &mut self,
        cfg: &Settings,
        position: LatLon,
        heading: f64,
        rot: f64,
        cog: Option<f64>,
        sog: f64,
        dt: f64,
    ) -> (f64, f64, Option<LegGuidance>) {
        let ap = &cfg.autopilot;
        let count = self.mission.waypoints.len();
        if self.wp_index >= count {
            self.set_mode_for(Mode::Hold, "mission complete");
            self.bus.message("Mission complete", Level::Notice);
            return (0.0, 0.0, None);
        }
        let waypoint = self.mission.waypoints[self.wp_index].clone();
        let target = waypoint.position();

        if let Some(until) = self.wp_hold_until {
            if clock::monotonic() < until {
                let (p, s) = self.keeper.update(&mut self.pilot, target, position, heading, rot, cog, sog, ap.cruise_speed_ms, dt);
                return (p, s, None);
            }
            self.wp_hold_until = None;
            self.advance(target);
            return (0.0, 0.0, None);
        }

        let start = self.leg_start.unwrap_or(position);
        let guidance = line_of_sight(start, target, position, ap.lookahead_m);
        let arrived = guidance.distance_to_go_m < ap.accept_radius_m
            || (guidance.passed_end() && guidance.distance_to_go_m < 3.0 * ap.accept_radius_m);
        if arrived {
            if waypoint.hold_s > 0.0 {
                self.wp_hold_until = Some(clock::monotonic() + waypoint.hold_s);
            } else {
                self.advance(target);
            }
            return (0.0, 0.0, Some(guidance));
        }

        let mut speed = waypoint.speed_ms.filter(|s| *s > 0.0).unwrap_or(ap.cruise_speed_ms).min(cfg.limits.max_speed_ms);
        // Slow down for a sharp turn at the end of the leg.
        let next = self.wp_index + 1;
        if next < count && guidance.distance_to_go_m < 2.0 * ap.lookahead_m {
            let after = self.mission.waypoints[next].position();
            let turn = geo::angle_difference(geo::bearing_deg(target, after), geo::bearing_deg(start, target)).abs();
            if turn > 60.0 {
                speed *= 0.6;
            }
        }
        let (p, s) = self.pilot.steer_course(guidance.course_deg, speed, heading, rot, cog, sog, dt, true);
        (p, s, Some(guidance))
    }

    fn advance(&mut self, reached: LatLon) {
        let count = self.mission.waypoints.len();
        self.bus.emit(
            EventKind::Mission,
            format!("Waypoint {}/{count} reached", self.wp_index + 1),
            Level::Info,
            json!({"index": self.wp_index}),
        );
        self.wp_index += 1;
        self.leg_start = Some(reached);
        if self.wp_index >= count && self.mission.looped {
            self.wp_index = 0;
        }
    }

    // ------------------------------------------------------------------
    // Safety
    // ------------------------------------------------------------------

    /// Raise or clear an alarm.  Returns true on the tick it becomes active.
    fn alarm(&mut self, key: &'static str, active: bool, text: String, level: Level) -> bool {
        let existing = self.alarms.iter().position(|a| a.key == key);
        match (active, existing) {
            (true, None) => {
                self.bus.emit(EventKind::Safety, text.clone(), level, json!({"alarm": key}));
                self.alarms.push(Alarm { key, text, level });
                true
            }
            (true, Some(i)) => {
                self.alarms[i] = Alarm { key, text, level };
                false
            }
            (false, Some(i)) => {
                self.alarms.remove(i);
                false
            }
            (false, None) => false,
        }
    }

    fn check_safety(&mut self, cfg: &Settings, link_ok: bool) {
        let safety = &cfg.safety;
        let lost = self.telemetry.is_some() && !link_ok;
        if self.alarm("link", lost, "LINK LOST — boat's own failsafe has stopped the motors".into(), Level::Critical) {
            self.latched.insert("link");
        }
        if link_ok && !self.link_was_ok && self.latched.contains("link") && self.mode != Mode::Stop {
            self.latched.remove("link");
            // The boat has been drifting with its motors stopped; do not let
            // it carry on as though nothing happened.
            self.set_mode_for(Mode::Hold, "link restored");
        }
        self.link_was_ok = link_ok;
        let Some(t) = self.telemetry.clone() else { return };
        if !link_ok {
            return;
        }

        let battery = t.battery_pct;
        let low = battery.is_some_and(|b| b < safety.battery_low_pct);
        let critical = battery.is_some_and(|b| b < safety.battery_critical_pct);
        let pct = battery.unwrap_or(0.0);
        self.alarm("battery_low", low && !critical, format!("Battery low ({pct:.0}%)"), Level::Warning);
        if self.alarm("battery_critical", critical, format!("BATTERY CRITICAL ({pct:.0}%) — returning home"), Level::Critical)
            && self.armed
        {
            self.set_mode_for(Mode::ReturnHome, "battery critical");
        }

        if let (Some(home), Some(here)) = (self.home, t.position()) {
            let distance = geo::distance_m(home, here);
            let outside = distance > safety.geofence_radius_m;
            if outside && !self.latched.contains("geofence") && self.armed {
                self.latched.insert("geofence");
                self.set_mode_for(Mode::ReturnHome, "geofence");
            }
            if distance < safety.geofence_radius_m * 0.9 {
                self.latched.remove("geofence");
            }
            self.alarm("geofence", outside, format!("GEOFENCE — {distance:.0} m from home"), Level::Error);
        }

        let depth = t.depth_m;
        let shallow = depth.is_some_and(|d| d < safety.shallow_alarm_m);
        self.alarm("shallow", shallow, format!("SHALLOW WATER {:.1} m", depth.unwrap_or(0.0)), Level::Error);
        if let Some(d) = depth {
            if d < safety.shallow_stop_m && self.mode.autonomous() && self.armed && self.mode != Mode::Hold {
                self.set_mode_for(Mode::Stop, &format!("depth {d:.1} m"));
            }
        }

        if t.grounded && !self.was_grounded {
            self.groundings += 1;
        }
        self.was_grounded = t.grounded;
        self.alarm("grounded", t.grounded, "AGROUND — back off in Manual".into(), Level::Critical);
        self.alarm("gnss", t.gps_fix < 2, "No GNSS fix".into(), Level::Error);

        let sk = &t.skimmer;
        let full = sk.fitted && sk.tank_capacity_l > 0.0 && sk.tank_l >= sk.tank_capacity_l - 0.05;
        self.alarm("tank_full", full, "SKIMMER TANK FULL — return to the GCS to offload".into(), Level::Warning);
        let too_fast = sk.deployed && t.stw_ms.unwrap_or(0.0) > cfg.skimmer.max_speed_ms;
        self.alarm("skim_speed", too_fast, "Too fast to skim — oil escapes under the arms".into(), Level::Notice);
    }

    // ------------------------------------------------------------------
    // Navigation figures
    // ------------------------------------------------------------------

    fn build_nav(&mut self, cfg: &Settings, link_ok: bool, port: f64, stbd: f64, guidance: Option<LegGuidance>) -> NavState {
        let now = clock::monotonic();
        while self.arrivals.front().is_some_and(|&a| now - a > 1.0) {
            self.arrivals.pop_front();
        }
        let position = self.position();
        let (home_d, home_b) = match (position, self.home) {
            (Some(p), Some(h)) => (Some(geo::distance_m(p, h)), Some(geo::bearing_deg(p, h))),
            _ => (None, None),
        };

        let mode = self.mode;
        let (target, mut label) = match mode {
            Mode::Auto if self.wp_index < self.mission.waypoints.len() => {
                (Some(self.mission.waypoints[self.wp_index].position()), format!("WP {}", self.wp_index + 1))
            }
            Mode::ReturnHome => (self.home, "HOME".to_string()),
            Mode::PointNav => (self.point_target, "POINT".to_string()),
            Mode::Hold => (self.hold_point, "HOLD".to_string()),
            _ => (None, String::new()),
        };
        if matches!(mode, Mode::PointNav | Mode::ReturnHome) && self.hold_point.is_some() {
            label.push_str(" (holding)");
        }

        let (mut btw, mut dtw, mut ttg, mut remaining) = (None, None, None, None);
        if let (Some(p), Some(target)) = (position, target) {
            let distance = geo::distance_m(p, target);
            btw = Some(geo::bearing_deg(p, target));
            dtw = Some(distance);
            let speed = self.telemetry.as_ref().and_then(|t| t.sog_ms).unwrap_or(0.0).max(0.2);
            if mode == Mode::Auto {
                let mut rest = vec![target];
                rest.extend(self.mission.waypoints[self.wp_index + 1..].iter().map(|w| w.position()));
                let left = distance + geo::path_length_m(&rest);
                remaining = Some(left);
                let pace = if speed < 0.3 { cfg.autopilot.cruise_speed_ms } else { speed };
                ttg = Some(left / pace.max(0.2));
            } else {
                ttg = Some(distance / speed);
            }
        }

        let station_d = match (position, self.link.station()) {
            (Some(p), Some(s)) => Some(geo::distance_m(p, s)),
            _ => None,
        };

        let mut alarms = self.alarms.clone();
        alarms.sort_by_key(|a| std::cmp::Reverse(a.level));
        let energy = self.estimate_energy(cfg, home_d);
        let coverage = self.coverage();
        NavState {
            mode,
            armed: self.armed,
            link_ok,
            downlink_hz: self.arrivals.len() as f64,
            telemetry: self.telemetry.clone(),
            home: self.home,
            home_distance_m: home_d,
            home_bearing_deg: home_b,
            station_distance_m: station_d,
            target,
            target_label: label,
            waypoint_index: if mode == Mode::Auto { Some(self.wp_index) } else { None },
            waypoint_count: self.mission.waypoints.len(),
            bearing_to_target_deg: btw,
            distance_to_target_m: dtw,
            cross_track_m: guidance.map(|g| g.cross_track_m),
            time_to_go_s: ttg,
            mission_remaining_m: remaining,
            speed_setpoint_ms: self.speed_setpoint,
            course_setpoint_deg: if mode == Mode::CourseHold { self.course_setpoint } else { None },
            command_port: port,
            command_stbd: stbd,
            alarms,
            trip_m: self.trip_m,
            run_time_s: self.armed_time,
            coverage,
            soundings: self.soundings.soundings.len(),
            groundings: self.groundings,
            manual_input: self.operator.connected,
            energy,
            // Stored pointing at the wind's source, so its polar form is (speed, from).
            true_wind: self.wind_vec.map(|(n, e)| vectors::to_polar(n, e)),
            current: self.current_vec.map(|(n, e)| vectors::to_polar(n, e)),
            skimmer_commanded: self.skimmer,
            charged_wh: self.charged_wh,
            limit: self.limiter.active,
        }
    }

    fn estimate_energy(&mut self, cfg: &Settings, home_distance: Option<f64>) -> Option<EnergyEstimate> {
        let newest = self.power_history.back()?.t;
        if newest == self.last_energy.0 {
            return self.last_energy.1;
        }
        let safety = &cfg.safety;
        let result = estimate(
            self.power_history.iter(),
            cfg.boat.battery_ah * NOMINAL_V,
            safety.battery_low_pct,
            safety.battery_critical_pct,
            home_distance,
            60.0,
        );
        self.last_energy = (newest, result);
        result
    }

    fn coverage(&mut self) -> f64 {
        let Some(area) = self.mission.area else { return 0.0 };
        let cells = self.soundings.len();
        let (cached_area, cached_cells, value) = self.coverage_cache;
        // Counting cells is cheap but not free; recount every few new cells.
        if cached_area == Some(area) && cells >= cached_cells && cells - cached_cells < 25 {
            return value;
        }
        let value = self.soundings.coverage(Some(&area));
        self.coverage_cache = (Some(area), cells, value);
        value
    }

    // ------------------------------------------------------------------
    // The session log
    // ------------------------------------------------------------------

    fn open_log(&mut self) {
        let folder = state_dir().join("logs");
        let path = folder.join(clock::local_strftime("session-%Y%m%d-%H%M%S.csv"));
        let opened = std::fs::create_dir_all(&folder).and_then(|_| std::fs::File::create(&path));
        match opened {
            Ok(file) => {
                let mut out = std::io::BufWriter::new(file);
                let _ = writeln!(
                    out,
                    "time,mode,armed,lat,lon,heading,cog,sog_ms,depth_m,battery_v,battery_pct,port,stbd,xte_m,\
                     water_temp_c,salinity_psu,ph,do_mgl"
                );
                self.log = Some(SessionLog { out, last: f64::NEG_INFINITY });
                self.log_path = Some(path);
            }
            Err(err) => log::warn!("session log disabled: {err}"),
        }
    }

    fn write_log(&mut self, port: f64, stbd: f64) {
        let (Some(log), Some(t)) = (&mut self.log, &self.telemetry) else { return };
        let now = clock::monotonic();
        if now - log.last < 1.0 {
            return;
        }
        log.last = now;
        let f = |v: Option<f64>, digits: usize| match v {
            Some(x) if !x.is_nan() => format!("{x:.digits$}"),
            _ => String::new(),
        };
        let w = &t.water;
        let row = [
            format!("{:.1}", clock::unix_time()),
            self.mode.value().to_string(),
            (self.armed as u8).to_string(),
            f(t.lat, 7),
            f(t.lon, 7),
            f(t.heading_deg, 1),
            f(t.cog_deg, 1),
            f(t.sog_ms, 2),
            f(t.depth_m, 2),
            f(t.battery_v, 2),
            f(t.battery_pct, 0),
            f(Some(port), 2),
            f(Some(stbd), 2),
            f(self.nav.cross_track_m, 1),
            f(w.temperature_c, 2),
            f(w.salinity_psu, 2),
            f(w.ph, 2),
            f(w.dissolved_oxygen_mgl, 2),
        ];
        let _ = writeln!(log.out, "{}", row.join(","));
        let _ = log.out.flush();
    }
}

/// The vehicle and its control loop.
pub struct Vehicle {
    core: Arc<Mutex<VehicleCore>>,
    link: Arc<dyn BoatLink>,
    pub bus: Arc<EventBus>,
    pub settings: SharedSettings,
    record: bool,
    stop: Arc<AtomicBool>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl Vehicle {
    pub fn new(settings: SharedSettings, link: Arc<dyn BoatLink>, bus: Option<Arc<EventBus>>, record: bool) -> Vehicle {
        let bus = bus.unwrap_or_else(|| Arc::new(EventBus::new()));
        let core = Arc::new(Mutex::new(VehicleCore::new(settings.clone(), link.clone(), bus.clone())));
        let weak: Weak<Mutex<VehicleCore>> = Arc::downgrade(&core);
        link.set_telemetry_handler(Some(Arc::new(move |t| {
            if let Some(core) = weak.upgrade() {
                core.lock().unwrap_or_else(|p| p.into_inner()).on_telemetry(t);
            }
        })));
        Vehicle { core, link, bus, settings, record, stop: Arc::new(AtomicBool::new(false)), thread: Mutex::new(None) }
    }

    /// The vehicle's state, locked: read it or call its commands, briefly.
    pub fn lock(&self) -> MutexGuard<'_, VehicleCore> {
        self.core.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    pub fn link(&self) -> &Arc<dyn BoatLink> {
        &self.link
    }

    pub fn nav(&self) -> Arc<NavState> {
        self.lock().nav()
    }

    pub fn tick(&self, dt: f64) -> Arc<NavState> {
        self.lock().tick(dt)
    }

    pub fn start(&self) {
        self.link.open();
        if self.record {
            self.lock().open_log();
        }
        self.stop.store(false, Ordering::Relaxed);
        let core = self.core.clone();
        let stop = self.stop.clone();
        let link = self.link.clone();
        let handle = std::thread::Builder::new()
            .name("control".into())
            .spawn(move || {
                let period = Duration::from_secs_f64(1.0 / CONTROL_HZ);
                let mut deadline = Instant::now();
                let mut last = Instant::now();
                while !stop.load(Ordering::Relaxed) {
                    let now = Instant::now();
                    let dt = (now - last).as_secs_f64().clamp(1e-3, 0.2);
                    last = now;
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        core.lock().unwrap_or_else(|p| p.into_inner()).tick(dt);
                    }));
                    if result.is_err() {
                        log::error!("control tick failed");
                        link.send_command(0.0, 0.0, false);
                    }
                    deadline += period;
                    let now = Instant::now();
                    if deadline > now {
                        std::thread::sleep(deadline - now);
                    } else {
                        deadline = now;
                    }
                }
            })
            .expect("control thread");
        *self.thread.lock().unwrap() = Some(handle);
        self.bus.message(format!("Ground station up — link {}", self.link.name()), Level::Info);
    }

    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.lock().unwrap().take() {
            let _ = handle.join();
        }
        self.link.send_command(0.0, 0.0, false);
        self.link.close();
        self.link.set_telemetry_handler(None);
        if let Some(log) = &mut self.lock().log {
            let _ = log.out.flush();
        }
    }
}
