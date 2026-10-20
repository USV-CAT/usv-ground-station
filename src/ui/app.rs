//! The ground station window.
//!
//! Laid out for the Steam Deck's 1280×800 screen, after the USV stations used
//! in the field: telemetry down the left, the chart in the middle with the
//! instruments over it, a tool strip on the right with a pull-out panel, and
//! the control modes along the bottom where the thumbs reach them.

use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use egui::{Align2, Color32, Context, Key, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Ui, Vec2};

use crate::config::{self, Settings, SharedSettings, WaterStyle};
use crate::core::bathymetry::DepthGrid;
use crate::core::clock;
use crate::core::events::{Event, EventBus, EventKind, Level};
use crate::core::geo::LatLon;
use crate::core::profile::Profile;
use crate::core::units;
use crate::input::gamepad::{self, Backend, Button, PadState};
use crate::input::mapping::{helm_from_keys, helm_from_pad, Helm};
use crate::config::LinkSettings;
use crate::link::mavlink::MavlinkLink;
use crate::link::sim::SimLink;
use crate::link::{BoatLink, Choice, Idle};
use crate::ui::chart::{BoatVectors, ChartEvent, ChartView, Tool, ViewMode};
use crate::ui::files::{FileDialog, Outcome, Purpose};
use crate::ui::instruments::{self, EchoSounder};
use crate::ui::{telemetry, theme};
use crate::vehicle::{Mode, NavState, Vehicle};

pub const HOLD_PRESS_S: f64 = 0.8;
/// Widths of the side panels, points.
pub const TELEMETRY_W: f32 = 240.0;
pub const DRAWER_W: f32 = 340.0;
pub const TOOLS_W: f32 = 64.0;

pub const MODE_BUTTONS: [(Mode, &str); 7] = [
    (Mode::Manual, "🎮 Manual"),
    (Mode::SpeedHold, "Speed\nHold"),
    (Mode::CourseHold, "Speed/Course\nHold"),
    (Mode::PointNav, "◎ Point\nNav"),
    (Mode::Auto, "▶ Pre-\nprogram"),
    (Mode::Hold, "⚓ Station\nHold"),
    (Mode::ReturnHome, "⌂ Return\nHome"),
];

pub const PAGES: [&str; 13] = ["Mission", "Chart", "Depth", "Power", "Oil", "📡 Link", "ℹ Info", "⚙ Limits", "🎮 Pad", "🔊 Sound", "Sim", "Log", "Help"];

/// The ground station's own battery — the Deck's — read on a thread of its
/// own, since asking the battery controller can take long enough to drop a frame.
fn watch_gcs_battery() -> Arc<Mutex<Option<f64>>> {
    let level = Arc::new(Mutex::new(None));
    let out = level.clone();
    let _ = std::thread::Builder::new().name("gcs-battery".into()).spawn(move || loop {
        let mut found = None;
        if let Ok(entries) = std::fs::read_dir("/sys/class/power_supply") {
            let mut names: Vec<PathBuf> = entries.filter_map(|e| e.ok().map(|e| e.path())).collect();
            names.sort();
            for path in names.into_iter().filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with("BAT"))) {
                if let Some(v) = std::fs::read_to_string(path.join("capacity")).ok().and_then(|s| s.trim().parse().ok()) {
                    found = Some(v);
                    break;
                }
            }
        }
        *out.lock().unwrap() = found;
        std::thread::sleep(Duration::from_secs(20));
    });
    level
}

pub struct Confirm {
    pub text: String,
    pub action: ConfirmAction,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ConfirmAction {
    ClearSurvey,
}

pub struct App {
    pub settings: SharedSettings,
    pub bus: Arc<EventBus>,
    events: Receiver<Event>,
    pub link: Arc<dyn BoatLink>,
    pub vehicle: Vehicle,
    pub chart_grid: Option<Arc<DepthGrid>>,
    pub canvas: ChartView,
    pad: Option<Backend>,
    pub pad_state: PadState,
    pub last_helm: Helm,
    pub keys: BTreeSet<&'static str>,
    held_since: HashMap<Button, f64>,
    fired_hold: HashSet<Button>,
    toast: Option<(String, f64)>,
    pub log: VecDeque<(f64, String, Level)>,
    pub nav: Arc<NavState>,
    pub sounder: EchoSounder,
    pub page: usize,
    pub profile: Option<Profile>,
    pub profile_at: f64,
    pub power_window_s: f64,
    pub cursor_text: String,
    pub file_dialog: Option<FileDialog>,
    pub confirm: Option<Confirm>,
    gcs_battery: Arc<Mutex<Option<f64>>>,
    fullscreen: Option<bool>,
    is_fullscreen: bool,
    started_at: f64,
    welcomed: bool,
    zoom_hold: Option<(f64, f64, f64)>,
    pub voice: crate::sound::Voice,
    announcer: crate::sound::Announcer,
    /// What the route planner said about its last plan.
    pub auto_summary: Option<String>,
    /// The power-flow diagram in place of the chart.
    pub power_view: bool,
    /// Fonts set in `new` only take hold on the next frame: draw nothing until then.
    fonts_pending: bool,
    /// The start dialog, while it is up.
    pub connect: Option<crate::ui::connect::ConnectDialog>,
    /// The link's data paths, measured (refreshed a few times a second).
    pub flows: Vec<crate::link::flows::FlowStats>,
    flows_at: f64,
    /// When the present link was made.
    pub connected_at: f64,
}

impl App {
    /// The station on the simulator, with no start dialog.
    pub fn new(ctx: &Context, settings: Settings, chart: Option<Arc<DepthGrid>>, fullscreen: Option<bool>, view: Option<ViewMode>) -> App {
        App::new_with(ctx, settings, chart, fullscreen, view, Some(Choice::Simulator))
    }

    /// The station connected to `start`, or asking which boat when `None`.
    pub fn new_with(ctx: &Context, mut settings: Settings, chart: Option<Arc<DepthGrid>>, fullscreen: Option<bool>, view: Option<ViewMode>, start: Option<Choice>) -> App {
        crate::ui::install_look(ctx);
        let problems = settings.validate();
        let ui = settings.ui.clone();
        let map_index = crate::ui::chart::tiles::find_map_index(&settings.map.vector_index);
        let shared = settings.clone().shared();
        let bus = Arc::new(EventBus::new());
        let events = bus.subscribe(None);
        // Until a boat is chosen, the station runs on no boat at all.
        let link: Arc<dyn BoatLink> = Arc::new(Idle);
        let vehicle = Vehicle::new(shared.clone(), link.clone(), Some(bus.clone()), false);

        let mut canvas = ChartView::new(ctx, map_index);
        canvas.set_chart(chart.clone());
        canvas.boat_length_m = settings.boat.length_m;
        canvas.boat_beam_m = settings.boat.beam_m;
        canvas.geofence_m = Some(settings.safety.geofence_radius_m);
        canvas.station = link.station();
        canvas.centre_on(LatLon::new(settings.sim.start_lat, settings.sim.start_lon));
        canvas.show_vectors = ui.show_vectors;
        canvas.water_style = ui.water_style;
        canvas.show_fps = ui.show_fps;
        canvas.skimmer_swath_m = settings.skimmer.swath_m;
        canvas.shallow_alarm_m = settings.safety.shallow_alarm_m;
        if let Some(v) = view {
            canvas.set_view_mode(v);
        }

        let pad = gamepad::open_backend(&settings.control.input_backend);
        vehicle.start();
        for p in problems {
            bus.message(format!("Settings: {p}"), Level::Warning);
        }
        if pad.is_none() {
            bus.message("No gamepad backend — driving from the keyboard (WASD)", Level::Notice);
        }
        let mut sounder = EchoSounder::default();
        sounder.shallow_alarm_m = settings.safety.shallow_alarm_m;
        sounder.shallow_stop_m = settings.safety.shallow_stop_m;
        let mut app = App {
            settings: shared,
            bus,
            events,
            link,
            vehicle,
            chart_grid: chart,
            canvas,
            pad,
            pad_state: PadState::default(),
            last_helm: Helm::default(),
            keys: BTreeSet::new(),
            held_since: HashMap::new(),
            fired_hold: HashSet::new(),
            toast: None,
            log: VecDeque::new(),
            nav: Arc::new(NavState::default()),
            sounder,
            page: ui.drawer_page.min(PAGES.len() - 1),
            profile: None,
            profile_at: f64::NEG_INFINITY,
            power_window_s: 1800.0,
            cursor_text: "Move over the chart for the depth there".into(),
            file_dialog: None,
            confirm: None,
            gcs_battery: watch_gcs_battery(),
            fullscreen,
            is_fullscreen: false,
            started_at: clock::monotonic(),
            welcomed: false,
            zoom_hold: None,
            voice: crate::sound::Voice::new(settings.sound.volume as f32),
            announcer: crate::sound::Announcer::new(),
            auto_summary: None,
            power_view: false,
            fonts_pending: true,
            connect: None,
            flows: Vec::new(),
            flows_at: f64::NEG_INFINITY,
            connected_at: clock::monotonic(),
        };
        match start {
            Some(choice) => {
                if let Err(e) = app.connect_to(choice.clone()) {
                    app.bus.message(format!("Could not connect to {}: {e}", choice.describe()), Level::Warning);
                    app.open_connect();
                    if let Some(d) = &mut app.connect {
                        d.error = Some(e);
                    }
                }
            }
            None => app.open_connect(),
        }
        app
    }

    /// Show the start dialog.
    pub fn open_connect(&mut self) {
        let s = self.settings.read().unwrap().link.clone();
        self.connect = Some(crate::ui::connect::ConnectDialog::new(&s, self.link.name() != Idle.name()));
    }

    /// Drop the boat (stopping it) and connect to another.  The mission,
    /// survey and track carry over.
    pub fn connect_to(&mut self, choice: Choice) -> Result<(), String> {
        let link: Arc<dyn BoatLink> = match &choice {
            Choice::Simulator => Arc::new(SimLink::new(self.settings.clone(), self.chart_grid.clone(), 7)),
            Choice::Mavlink(url) => {
                let s = self.settings.read().unwrap().link.clone();
                Arc::new(MavlinkLink::connect(&LinkSettings { mavlink_url: url.clone(), ..s })?)
            }
        };
        self.vehicle.shutdown();
        let (mission, soundings) = {
            let v = self.vehicle.lock();
            (v.mission.clone(), v.soundings.clone())
        };
        let vehicle = Vehicle::new(self.settings.clone(), link.clone(), Some(self.bus.clone()), true);
        {
            let mut v = vehicle.lock();
            v.mission = mission;
            v.soundings = soundings;
        }
        vehicle.start();
        self.vehicle = vehicle;
        self.link = link;
        self.connected_at = clock::monotonic();
        self.flows.clear();
        self.nav = Arc::new(NavState::default());
        self.canvas.station = self.link.station();
        {
            let mut s = self.settings.write().unwrap();
            match &choice {
                Choice::Simulator => s.link.kind = "sim".into(),
                Choice::Mavlink(url) => {
                    s.link.kind = "mavlink".into();
                    s.link.mavlink_url = url.clone();
                    s.link.remember(url);
                }
            }
        }
        if let Choice::Mavlink(_) = choice {
            self.toast(&format!("Connected on {} — waiting for the boat", choice.describe()), 5.0);
        }
        Ok(())
    }

    pub fn started_at(&self) -> f64 {
        self.started_at
    }

    pub fn gcs_battery(&self) -> Option<f64> {
        *self.gcs_battery.lock().unwrap()
    }

    pub fn sim(&self) -> Option<&SimLink> {
        self.link.as_sim()
    }

    pub fn ui_settings(&self) -> config::UiSettings {
        self.settings.read().unwrap().ui.clone()
    }

    pub fn edit_ui(&self, f: impl FnOnce(&mut config::UiSettings)) {
        f(&mut self.settings.write().unwrap().ui);
    }

    // ------------------------------------------------------------------
    // Commands
    // ------------------------------------------------------------------

    pub fn toast(&mut self, text: impl Into<String>, seconds: f64) {
        self.toast = Some((text.into(), clock::monotonic() + seconds));
    }

    pub fn request_mode(&mut self, mode: Mode) {
        let has_point = self.vehicle.lock().point_target().is_some();
        if mode == Mode::PointNav && !has_point {
            self.set_tool(Tool::Goto);
            self.toast("Tap the chart where the boat should go", 3.0);
        } else if mode == Mode::Auto {
            self.start_mission();
        } else {
            let (ok, armed) = {
                let mut v = self.vehicle.lock();
                (v.set_mode(mode), v.armed())
            };
            if ok && !armed && mode != Mode::Stop {
                self.toast(format!("{} selected — arm to give it the motors", mode.label()), 3.0);
            }
        }
    }

    pub fn start_mission(&mut self) {
        let (ok, armed) = {
            let mut v = self.vehicle.lock();
            (v.set_mode(Mode::Auto), v.armed())
        };
        if ok {
            self.canvas.follow = true;
            if !armed {
                self.toast("Mission ready — arm to start it", 3.0);
            }
        }
    }

    pub fn restart_mission(&mut self) {
        self.vehicle.lock().restart_mission();
        self.toast("Mission back to waypoint 1", 3.0);
    }

    pub fn goto(&mut self, p: LatLon) {
        let armed = {
            let mut v = self.vehicle.lock();
            v.go_to(p.lat, p.lon);
            v.armed()
        };
        self.set_tool(Tool::Pan);
        if !armed {
            self.toast("Point set — arm to go", 3.0);
        }
    }

    pub fn set_tool(&mut self, tool: Tool) {
        self.canvas.tool = tool;
        match tool {
            Tool::Rect => self.toast("Drag across the water to draw the survey rectangle", 3.0),
            Tool::Points => self.toast("Tap to add waypoints · drag to move · right-click to delete", 3.0),
            _ => {}
        }
    }

    pub fn toggle_follow(&mut self) {
        self.canvas.follow = !self.canvas.follow;
        if let (true, Some((lat, lon, _))) = (self.canvas.follow, self.canvas.boat) {
            self.canvas.centre_on(LatLon::new(lat, lon));
        }
    }

    pub fn toggle_drawer(&mut self) {
        self.edit_ui(|u| u.show_drawer = !u.show_drawer);
    }

    pub fn toggle_telemetry(&mut self) {
        self.edit_ui(|u| u.show_telemetry = !u.show_telemetry);
    }

    pub fn show_page(&mut self, page: usize) {
        self.edit_ui(|u| u.show_drawer = true);
        self.page = page.min(PAGES.len() - 1);
    }

    pub fn cycle_view(&mut self) {
        let mode = self.canvas.cycle_view();
        self.toast(mode.name(), 1.5);
    }

    pub fn toggle_vectors(&mut self) {
        self.canvas.show_vectors = !self.canvas.show_vectors;
        let on = self.canvas.show_vectors;
        self.edit_ui(|u| u.show_vectors = on);
    }

    pub fn set_water_style(&mut self, style: WaterStyle) {
        self.canvas.water_style = style;
        self.edit_ui(|u| u.water_style = style);
        let name = match style {
            WaterStyle::Natural => "Natural water",
            WaterStyle::Chart => "Depth chart colours",
            WaterStyle::Satellite => "Satellite imagery",
        };
        self.toast(name, 1.5);
    }

    pub fn set_max_fps(&mut self, fps: u32) {
        let fps = if fps <= 30 { 30 } else { 60 };
        self.edit_ui(|u| u.max_fps = fps);
    }

    pub fn frame_mission(&mut self) {
        let points = {
            let v = self.vehicle.lock();
            let mut points = v.mission.positions();
            if let Some(area) = v.mission.area {
                points.extend(area.corners());
            }
            points
        };
        if !points.is_empty() {
            self.canvas.frame(&points);
        }
    }

    pub fn boat_position(&self) -> Option<LatLon> {
        self.nav.position()
    }

    pub fn area_edited(&mut self) {
        let angle = {
            let mut v = self.vehicle.lock();
            v.mission.drift_ms = None; // drawn by hand: fixed to the chart, not the water
            v.mission.area.map(|a| a.angle_deg)
        };
        if let Some(a) = angle {
            self.canvas.rect_angle_deg = a;
        }
        self.regenerate_route();
    }

    pub fn regenerate_route(&mut self) {
        let (area, spec) = {
            let v = self.vehicle.lock();
            (v.mission.area, v.mission.pattern)
        };
        let Some(area) = area else { return };
        if spec.pattern != crate::core::mission::Pattern::Auto {
            self.auto_summary = None;
            self.vehicle.lock().mission.regenerate();
            return;
        }
        // The planner: from where the boat is, keeping off the charted shallows.
        let s = self.settings.read().unwrap().clone();
        let chart = self.chart_grid.clone();
        let depth = move |lat: f64, lon: f64| chart.as_ref().map(|c| c.depth_at(lat, lon)).unwrap_or(f64::NAN);
        let has_chart = self.chart_grid.is_some();
        let ctx = crate::core::mission::AutoContext {
            start: self.boat_position(),
            cruise_ms: s.autopilot.cruise_speed_ms,
            depth: if has_chart { Some(&depth) } else { None },
            shallow_m: s.safety.shallow_alarm_m + s.boat.draft_m,
        };
        let plan = crate::core::mission::plan_auto(&area, &spec, &ctx);
        self.vehicle.lock().mission.set_route(&plan.route, None);
        self.auto_summary = Some(plan.summary);
    }

    pub fn set_chart(&mut self, grid: DepthGrid) {
        let source = grid.source.clone();
        let grid = Arc::new(grid);
        self.chart_grid = Some(grid.clone());
        self.canvas.set_chart(Some(grid.clone()));
        if let Some(sim) = self.sim() {
            sim.set_chart(Some(grid));
        }
        self.toast(format!("Depth chart loaded: {source}"), 3.0);
    }

    pub fn depth_alarms_changed(&mut self) {
        let safety = self.settings.read().unwrap().safety.clone();
        self.canvas.shallow_alarm_m = safety.shallow_alarm_m;
        self.sounder.shallow_alarm_m = safety.shallow_alarm_m;
        self.sounder.shallow_stop_m = safety.shallow_stop_m;
    }

    fn set_home(&mut self) {
        self.vehicle.lock().set_home_here();
    }

    /// Whether the station asked for full screen (F11 or the status-bar button).
    pub fn is_fullscreen(&self) -> bool {
        self.is_fullscreen
    }

    fn toggle_fullscreen(&mut self, ctx: &Context) {
        // Follow the window's real state: the window manager can change it too.
        let now = ctx.input(|i| i.viewport().fullscreen).unwrap_or(self.is_fullscreen);
        self.is_fullscreen = !now;
        ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(self.is_fullscreen));
    }

    // ------------------------------------------------------------------
    // Input
    // ------------------------------------------------------------------

    fn poll_input(&mut self, ctx: &Context) {
        let typing = ctx.egui_wants_keyboard_input() || self.connect.is_some();
        let mut pressed: Vec<(Key, egui::Modifiers)> = Vec::new();
        self.keys.clear();
        if !typing {
            ctx.input(|i| {
                for (key, name) in [
                    (Key::W, "W"),
                    (Key::A, "A"),
                    (Key::S, "S"),
                    (Key::D, "D"),
                    (Key::ArrowUp, "Up"),
                    (Key::ArrowDown, "Down"),
                    (Key::ArrowLeft, "Left"),
                    (Key::ArrowRight, "Right"),
                ] {
                    if i.key_down(key) {
                        self.keys.insert(name);
                    }
                }
                if i.modifiers.shift && !self.keys.is_empty() {
                    self.keys.insert("Shift");
                }
                for e in &i.events {
                    if let egui::Event::Key { key, pressed: true, repeat: false, modifiers, .. } = e {
                        pressed.push((*key, *modifiers));
                    }
                }
            });
        }
        for (key, _) in pressed {
            self.key_command(ctx, key);
        }

        let control = self.settings.read().unwrap().control.clone();
        let pad_state = self.pad.as_ref().map(Backend::state).unwrap_or_default();
        let mut helm = helm_from_pad(&pad_state, control.stick_mode, control.deadzone, control.expo);
        let keyboard = helm_from_keys(&self.keys);
        if keyboard.deflection > helm.deflection {
            helm = keyboard;
        }
        self.vehicle.lock().set_operator(helm.operator(pad_state.connected || !self.keys.is_empty()));
        self.pad_state = pad_state;
        self.last_helm = helm;

        let edges = self.pad.as_ref().map(Backend::edges).unwrap_or_default();
        let now = clock::monotonic();
        for (button, down) in edges {
            if down {
                self.held_since.insert(button, now);
            } else {
                let started = self.held_since.remove(&button);
                if !self.fired_hold.remove(&button) && started.is_some() {
                    self.button(button);
                }
            }
        }
        let held: Vec<(Button, f64)> = self.held_since.iter().map(|(b, t)| (*b, *t)).collect();
        for (button, started) in held {
            if matches!(button, Button::Menu | Button::View) && !self.fired_hold.contains(&button) && now - started > HOLD_PRESS_S {
                self.fired_hold.insert(button);
                match button {
                    Button::Menu => self.vehicle.lock().toggle_arm(),
                    _ => self.request_mode(Mode::ReturnHome),
                }
            }
        }
    }

    fn key_command(&mut self, ctx: &Context, key: Key) {
        let modes = [Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7];
        if let Some(i) = modes.iter().position(|k| *k == key) {
            self.request_mode(MODE_BUTTONS[i].0);
            return;
        }
        match key {
            Key::Space => self.request_mode(Mode::Stop),
            Key::Enter => self.vehicle.lock().toggle_arm(),
            Key::Plus | Key::Equals => self.canvas.zoom(1.0, None),
            Key::Minus => self.canvas.zoom(-1.0, None),
            Key::F => self.toggle_follow(),
            Key::Tab => self.toggle_drawer(),
            Key::T => self.toggle_telemetry(),
            Key::V => self.cycle_view(),
            Key::K => self.vehicle.lock().toggle_skimmer(),
            Key::P => self.power_view = !self.power_view,
            Key::G => self.toggle_vectors(),
            Key::N => {
                let next = match self.canvas.water_style {
                    WaterStyle::Natural => WaterStyle::Chart,
                    WaterStyle::Chart => WaterStyle::Satellite,
                    WaterStyle::Satellite => WaterStyle::Natural,
                };
                self.set_water_style(next);
            }
            Key::F11 => self.toggle_fullscreen(ctx),
            Key::Delete | Key::Backspace => {
                if self.canvas.delete_selected(&self.vehicle) {
                    self.toast("Waypoint deleted", 1.5);
                }
            }
            Key::Escape => self.set_tool(Tool::Pan),
            _ => {}
        }
    }

    fn button(&mut self, button: Button) {
        let mode = self.vehicle.lock().mode();
        match button {
            Button::A => self.start_mission(),
            Button::B => self.request_mode(Mode::Stop),
            Button::X => {
                let cycle = [Mode::Manual, Mode::SpeedHold, Mode::CourseHold];
                let next = cycle.iter().position(|m| *m == mode).map(|i| cycle[(i + 1) % 3]).unwrap_or(Mode::Manual);
                self.request_mode(next);
            }
            Button::Y => self.request_mode(Mode::Hold),
            Button::View => self.toggle_drawer(),
            Button::Menu => self.toast("Hold ☰ to arm or disarm", 3.0),
            Button::LB => self.canvas.zoom(-1.0, None),
            Button::RB => self.canvas.zoom(1.0, None),
            Button::R3 => self.cycle_view(),
            Button::L3 => self.vehicle.lock().toggle_skimmer(),
            Button::Up | Button::Down => {
                let setpoint = self.vehicle.lock().adjust_speed(if button == Button::Up { 0.1 } else { -0.1 });
                self.toast(format!("Speed setpoint {:.1} kn", units::knots(setpoint)), 1.2);
            }
            Button::Left | Button::Right => {
                if mode == Mode::CourseHold {
                    self.vehicle.lock().adjust_course(if button == Button::Right { 5.0 } else { -5.0 });
                } else {
                    let step = if button == Button::Right { 1 } else { PAGES.len() - 1 };
                    let page = (self.page + step) % PAGES.len();
                    self.show_page(page);
                }
            }
            _ => {}
        }
    }

    // ------------------------------------------------------------------
    // Refresh
    // ------------------------------------------------------------------

    fn drain_events(&mut self) {
        let events: Vec<Event> = self.events.try_iter().collect();
        for e in events {
            if e.kind == EventKind::Telemetry {
                continue;
            }
            self.log.push_back((e.timestamp, e.message.clone(), e.level));
            if self.log.len() > 500 {
                self.log.pop_front();
            }
            if e.level >= Level::Warning || matches!(e.kind, EventKind::Mode | EventKind::ArmingState) {
                self.toast(e.message, if e.level >= Level::Warning { 3.5 } else { 2.0 });
            }
        }
    }

    /// Speak what changed, and keep repeating danger while it lasts.
    fn announce(&mut self, now: f64) {
        let sound = self.settings.read().unwrap().sound.clone();
        let calls = self.announcer.update(&self.nav, now, &sound);
        if !sound.enabled {
            return;
        }
        for call in calls {
            self.voice.say(call.alert, &sound.language, call.tone);
        }
    }

    fn refresh(&mut self, now: f64) {
        let nav = self.vehicle.nav();
        self.sounder.record(&nav);
        let time_scale = self.settings.read().unwrap().sim.time_scale;
        if let Some(t) = &nav.telemetry {
            if let Some(p) = t.position() {
                self.canvas.time_scale = if self.link.simulated() { time_scale } else { 1.0 };
                self.canvas.set_boat(p.lat, p.lon, t.heading_deg.unwrap_or(0.0), t.cog_deg, t.sog_ms, t.roll_deg.unwrap_or(0.0), t.rot_dps, now);
                self.canvas.vectors = BoatVectors {
                    thrust: (t.throttle_port + t.throttle_stbd) / 2.0,
                    turn: (t.throttle_port - t.throttle_stbd) / 2.0,
                    wind: nav.true_wind,
                    current: nav.current,
                };
                self.canvas.skimmer_deployed = t.skimmer.deployed;
            }
        }
        if self.canvas.show_oil {
            if let Some(sim) = self.link.as_sim() {
                let version = sim.spill_version();
                if self.canvas.oil.as_ref().map(|o| o.version) != Some(version) {
                    self.canvas.oil = sim.spill_snapshot();
                }
            }
        }
        // The waves follow what the boat measures; the simulator knows its
        // own weather before the estimates have settled.
        let sim_weather = self.link.simulated().then(|| {
            let s = self.settings.read().unwrap().sim.clone();
            ((s.wind_speed_ms, s.wind_from_deg), (s.current_speed_ms, s.current_to_deg))
        });
        self.canvas.sea = crate::ui::chart::layers::Sea {
            wind: nav.true_wind.or(sim_weather.map(|w| w.0)),
            current: nav.current.or(sim_weather.map(|w| w.1)),
        };
        {
            let map = &self.settings.read().unwrap().map;
            if self.canvas.satellite_url != map.satellite_url {
                self.canvas.satellite_url = map.satellite_url.clone();
            }
            if self.canvas.satellite_attribution != map.satellite_attribution {
                self.canvas.satellite_attribution = map.satellite_attribution.clone();
            }
        }
        self.canvas.home = nav.home;
        self.canvas.target = nav.target;
        self.canvas.target_label = nav.target_label.clone();
        self.canvas.active_index = nav.waypoint_index;
        self.canvas.station = self.link.station();
        self.nav = nav;
    }

    // ------------------------------------------------------------------
    // The frame
    // ------------------------------------------------------------------

    pub fn frame(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        if std::mem::take(&mut self.fonts_pending) {
            ctx.request_repaint();
            return;
        }
        let now = clock::monotonic();
        if let Some(full) = self.fullscreen.take() {
            self.is_fullscreen = full;
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(full));
        } else if !self.welcomed && now - self.started_at > 0.4 {
            self.welcomed = true;
            self.toast("Welcome aboard — hold ☰ (or Enter) to arm, then drive with the sticks", 5.0);
        }
        self.poll_input(&ctx);
        for text in self.link.take_messages() {
            self.bus.message(text, Level::Notice);
        }
        if now_flows(self.flows_at) {
            self.flows_at = clock::monotonic();
            self.flows = self.link.flows();
        }
        self.drain_events();
        self.refresh(now);
        self.announce(now);
        let ui_settings = self.ui_settings();
        let units_kn = self.settings.read().unwrap().map.speed_units != "ms";

        egui::Panel::top("status").exact_size(40.0).resizable(false).frame(egui::Frame::new().fill(theme::SURFACE)).show(ui, |ui| {
            self.status_bar(ui);
        });
        egui::Panel::bottom("modes")
            .exact_size(70.0)
            .resizable(false)
            .frame(egui::Frame::new().fill(theme::SURFACE).inner_margin(egui::Margin::symmetric(8, 6)))
            .show(ui, |ui| self.mode_bar(ui));
        // The chart is bounded by the side panels' fixed widths: egui sizes a
        // side panel by what it holds, and its central area could otherwise
        // reach in under one.
        let full = ui.max_rect();
        let left_edge = if ui_settings.show_telemetry { full.left() + TELEMETRY_W } else { f32::NEG_INFINITY };
        let right_edge = full.right() - TOOLS_W - if ui_settings.show_drawer { DRAWER_W } else { 0.0 };
        if ui_settings.show_telemetry {
            egui::Panel::left("telemetry")
                .exact_size(TELEMETRY_W)
                .resizable(false)
                .frame(egui::Frame::new().fill(theme::BACKGROUND).inner_margin(6))
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| telemetry::panel(ui, &self.nav, units_kn));
                });
        }
        egui::Panel::right("tools")
            .exact_size(TOOLS_W)
            .resizable(false)
            .frame(egui::Frame::new().fill(theme::SURFACE).inner_margin(egui::Margin::symmetric(4, 6)))
            .show(ui, |ui| self.tool_strip(ui));
        if ui_settings.show_drawer {
            egui::Panel::right("drawer")
                .exact_size(DRAWER_W)
                .resizable(false)
                .frame(egui::Frame::new().fill(theme::SURFACE).inner_margin(egui::Margin::symmetric(4, 4)))
                .show(ui, |ui| self.drawer(ui));
        }
        egui::CentralPanel::no_frame().show(ui, |ui| {
            let r = ui.max_rect();
            let bounded = Rect::from_min_max(Pos2::new(r.left().max(left_edge), r.top()), Pos2::new(r.right().min(right_edge), r.bottom()));
            let mut ui = ui.new_child(egui::UiBuilder::new().max_rect(bounded));
            ui.set_clip_rect(bounded.intersect(ui.clip_rect()));
            let ui = &mut ui;
            if self.power_view {
                self.power_area(ui, now);
            } else {
                self.chart_area(ui, &ui_settings, units_kn, now);
            }
        });

        self.dialogs(&ctx);
        let fps = ui_settings.max_fps.max(1);
        ctx.request_repaint_after(Duration::from_secs_f64(1.0 / fps as f64));
    }

    fn status_bar(&mut self, ui: &mut Ui) {
        let rect = ui.max_rect();
        let p = ui.painter();
        p.line_segment([rect.left_bottom(), rect.right_bottom()], Stroke::new(1.0, theme::BORDER));
        let cy = rect.center().y;
        p.text(Pos2::new(rect.left() + 12.0, cy), Align2::LEFT_CENTER, "| USV·GS", theme::bold(19.0), theme::CAUTION);
        let mut x = rect.left() + 130.0;
        let nav = &self.nav;
        let mut chip = |text: &str, colour: Color32, fill: bool| {
            let galley = p.layout_no_wrap(text.to_string(), theme::bold(12.5), if fill { theme::INK } else { colour });
            let w = galley.size().x + 22.0;
            let r = Rect::from_min_size(Pos2::new(x, rect.top() + 7.0), Vec2::new(w, rect.height() - 14.0));
            p.rect(r, 6.0, if fill { colour } else { Color32::TRANSPARENT }, Stroke::new(1.5, colour), StrokeKind::Middle);
            p.galley(r.center() - galley.size() / 2.0, galley, colour);
            x += w + 8.0;
            r
        };
        let scale = if self.link.simulated() { self.settings.read().unwrap().sim.time_scale } else { 1.0 };
        let link = if scale != 1.0 { format!("{} ×{scale}", self.link.name()) } else { self.link.name().to_string() };
        let path = self
            .flows
            .iter()
            .find(|f| f.state == crate::link::flows::FlowState::Active)
            .filter(|_| self.flows.len() > 1)
            .map(|f| format!(" · {} {}", f.bearer.icon(), f.bearer.label()))
            .unwrap_or_default();
        let link_chip = chip(&format!("{link}{path} ▾"), theme::ACCENT, false);
        let link_click = ui.interact(link_chip, ui.id().with("link-chip"), Sense::click());
        link_click.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, "Connection"));
        let open_connect = link_click.on_hover_text("Connect to another boat, or the simulator").clicked();
        chip(&nav.mode.label().to_uppercase(), if nav.mode == Mode::Stop { theme::BAD } else { theme::GOOD }, true);
        chip(if nav.armed { "ARMED" } else { "DISARMED" }, if nav.armed { theme::CAUTION } else { theme::TEXT_DIM }, nav.armed);
        if !nav.link_ok && self.link.name() != Idle.name() {
            chip("NO LINK", theme::BAD, true);
        }
        if let Some(limit) = nav.limit {
            chip(limit.label(), theme::CAUTION, false);
        }

        // The right-hand side, laid out from the right edge.
        let mut right = rect.right() - 12.0;
        let right_text = |right: &mut f32, text: String, colour: Color32, width: f32| {
            p.text(Pos2::new(*right, cy), Align2::RIGHT_CENTER, text, theme::mono_bold(14.0), colour);
            *right -= width + 12.0;
        };
        // Full-screen toggle at the right edge: the Deck has no F11 key.
        let fs_rect = Rect::from_center_size(Pos2::new(right - 13.0, cy), Vec2::splat(26.0));
        let fs = ui.interact(fs_rect, ui.id().with("fullscreen"), Sense::click());
        fs.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, "Full screen"));
        let full_now = self.is_fullscreen;
        let stroke = Stroke::new(2.0, if fs.hovered() { theme::TEXT } else { theme::TEXT_DIM });
        let r = fs_rect.shrink(5.0);
        let k = 5.0;
        // Four corner brackets: pointing out to enter full screen, in to leave it.
        for (corner, dx, dy) in [(r.left_top(), 1.0, 1.0), (r.right_top(), -1.0, 1.0), (r.left_bottom(), 1.0, -1.0), (r.right_bottom(), -1.0, -1.0)] {
            let (c, sx, sy) = if full_now { (corner + Vec2::new(dx * k, dy * k), -dx, -dy) } else { (corner, dx, dy) };
            p.line_segment([c, c + Vec2::new(sx * k, 0.0)], stroke);
            p.line_segment([c, c + Vec2::new(0.0, sy * k)], stroke);
        }
        let toggle_full = fs.on_hover_text(if full_now { "Leave full screen (F11)" } else { "Full screen (F11)" }).clicked();
        right -= 26.0 + 12.0;
        right_text(&mut right, clock::local_strftime("%H:%M:%S"), theme::TEXT, 86.0);
        // Signal bars.
        let rssi = nav.telemetry.as_ref().filter(|_| nav.link_ok).and_then(|t| t.rssi_dbm);
        let level = rssi.map(|r| (((r + 100.0) / 10.0) as i32).clamp(1, 5)).unwrap_or(0);
        let bx = right - 30.0;
        for i in 0..5 {
            let h = 6.0 + i as f32 * 4.0;
            let mut colour = if i < level { theme::GOOD } else { theme::BORDER };
            if level > 0 && level <= 2 && i < level {
                colour = theme::CAUTION;
            }
            p.rect_filled(Rect::from_min_size(Pos2::new(bx + i as f32 * 6.0, rect.bottom() - 10.0 - h), Vec2::new(4.0, h)), 0.0, colour);
        }
        right -= 44.0;
        let t = nav.telemetry.as_ref();
        let usv = t.and_then(|t| t.battery_pct);
        let usv_text = match (usv, t.and_then(|t| t.battery_v)) {
            (Some(pct), Some(v)) => format!("USV {pct:.0}% {v:.1}V"),
            _ => "USV —".into(),
        };
        right_text(&mut right, usv_text, theme::battery_colour(usv), 150.0);
        let gcs = *self.gcs_battery.lock().unwrap();
        right_text(&mut right, gcs.map(|g| format!("GCS {g:.0}%")).unwrap_or("GCS ⏻".into()), theme::battery_colour(gcs), 90.0);
        let pad = if self.pad_state.connected { format!("🎮 {}", self.pad_state.name.chars().take(24).collect::<String>()) } else { "🎮 keyboard".into() };
        right_text(&mut right, pad, if self.pad_state.connected { theme::GOOD } else { theme::TEXT_DIM }, 200.0);
        if open_connect {
            self.open_connect();
        }
        if toggle_full {
            let ctx = ui.ctx().clone();
            self.toggle_fullscreen(&ctx);
        }
    }

    fn mode_bar(&mut self, ui: &mut Ui) {
        let nav = self.nav.clone();
        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 6.0;
            let h = ui.available_height();
            let arm_text = if nav.armed { "ARMED\n(disarm)" } else { "ARM" };
            let (fill, stroke) = if nav.armed { (theme::hex(0x16451f), theme::GOOD) } else { (theme::hex(0x3c2a0a), theme::CAUTION) };
            if big_button(ui, Vec2::new(88.0, h), arm_text, fill, stroke, 15.0, nav.armed).clicked() {
                self.vehicle.lock().toggle_arm();
            }
            let fixed = 88.0 + 112.0 + 124.0 + 6.0 * 10.0;
            let each = ((ui.available_width() - fixed + 88.0) / MODE_BUTTONS.len() as f32).max(60.0);
            for (mode, text) in MODE_BUTTONS {
                let on = nav.mode == mode;
                let (fill, stroke) = if on { (theme::hex(0x163d22), theme::GOOD) } else { (theme::SURFACE_RAISED, theme::BORDER) };
                if big_button(ui, Vec2::new(each, h), text, fill, stroke, 14.0, on).clicked() {
                    self.request_mode(mode);
                }
            }
            let on = nav.mode == Mode::Stop;
            if big_button(ui, Vec2::new(112.0, h), "■  USV STOP", if on { theme::hex(0x8f1c1f) } else { theme::hex(0x4a1013) }, theme::BAD, 15.0, on).clicked() {
                self.request_mode(Mode::Stop);
            }
            if let Some(p) = nav.position() {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        RichText::new(format!("{}\n{}", units::dms(Some(p.lat), 'N', 'S'), units::dms(Some(p.lon), 'E', 'W')))
                            .font(theme::mono_bold(13.0))
                            .color(theme::VALUE),
                    );
                });
            }
        });
    }

    fn tool_strip(&mut self, ui: &mut Ui) {
        ui.set_min_width(ui.available_width());
        ui.spacing_mut().item_spacing.y = 5.0;
        let w = ui.available_width();
        let tool = self.canvas.tool;
        for (t, text) in [(Tool::Pan, "✋\nPan"), (Tool::Points, "+\nPoints"), (Tool::Rect, "▭\nRect"), (Tool::Goto, "◎\nGo to")] {
            if tool_button(ui, w, text, tool == t).clicked() {
                self.set_tool(if tool == t && t != Tool::Pan { Tool::Pan } else { t });
            }
        }
        ui.add_space(6.0);
        if tool_button(ui, w, "⌖\nFollow", self.canvas.follow).clicked() {
            self.toggle_follow();
        }
        if tool_button(ui, w, &format!("{}\nView", self.canvas.view_mode.label()), false).clicked() {
            self.cycle_view();
        }
        if tool_button(ui, w, "➚\nVectors", self.canvas.show_vectors).clicked() {
            self.toggle_vectors();
        }
        if tool_button(ui, w, "⛭\nSkim", self.nav.skimmer_commanded).clicked() {
            self.vehicle.lock().toggle_skimmer();
        }
        if tool_button(ui, w, "⤢\nRoute", false).clicked() {
            self.frame_mission();
        }
        if tool_button(ui, w, "⚡\nPower", self.power_view).clicked() {
            self.power_view = !self.power_view;
        }
        ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
            let ui_s = self.ui_settings();
            if tool_button(ui, w, "☰\nPanel", ui_s.show_drawer).clicked() {
                self.toggle_drawer();
            }
            if tool_button(ui, w, "◑\nHUD", ui_s.show_hud).clicked() {
                self.edit_ui(|u| u.show_hud = !u.show_hud);
            }
            if tool_button(ui, w, "⌂\nSet H", false).clicked() {
                self.set_home();
            }
        });
    }

    fn drawer(&mut self, ui: &mut Ui) {
        // Claim the whole panel: egui sizes a side panel by what it holds,
        // and the chart would otherwise reach in under a narrower one.
        ui.set_min_width(ui.available_width());
        let w = (ui.available_width() - 6.0) / 3.0;
        egui::Grid::new("pages").spacing([3.0, 3.0]).show(ui, |ui| {
            for (i, title) in PAGES.iter().enumerate() {
                let on = self.page == i;
                let (fill, stroke) = if on { (theme::hex(0x12384a), theme::ACCENT) } else { (theme::SURFACE_RAISED, theme::BORDER) };
                if big_button(ui, Vec2::new(w, 30.0), title, fill, stroke, 13.0, on).clicked() {
                    self.page = i;
                }
                if i % 3 == 2 {
                    ui.end_row();
                }
            }
        });
        ui.add_space(4.0);
        // Pinned to the left: a page is laid out to the panel's width and
        // never scrolls sideways (a wide row would otherwise nudge it over).
        egui::ScrollArea::vertical().id_salt(("drawer", self.page)).auto_shrink([false, false]).horizontal_scroll_offset(0.0).show(ui, |ui| {
            ui.set_width(ui.available_width() - 4.0);
            match PAGES[self.page.min(PAGES.len() - 1)] {
                "Mission" => self.page_mission(ui),
                "Chart" => self.page_chart(ui),
                "Depth" => self.page_depth(ui),
                "Power" => self.page_power(ui),
                "Oil" => self.page_oil(ui),
                "⚙ Limits" => self.page_limits(ui),
                "📡 Link" => self.page_link(ui),
                "ℹ Info" => self.page_info(ui),
                "🎮 Pad" => self.page_pad(ui),
                "🔊 Sound" => self.page_sound(ui),
                "Sim" => self.page_sim(ui),
                "Log" => self.page_log(ui),
                _ => self.page_help(ui),
            }
        });
    }

    fn chart_area(&mut self, ui: &mut Ui, ui_settings: &config::UiSettings, units_kn: bool, now: f64) {
        let rect = ui.max_rect();
        self.canvas.show_fps = ui_settings.show_fps;
        let events = self.canvas.ui(ui, rect, &self.vehicle);
        for e in events {
            match e {
                ChartEvent::AreaEdited => self.area_edited(),
                ChartEvent::Goto(p) => self.goto(p),
                ChartEvent::Cursor(p) => self.cursor_text = self.describe_point(p),
                ChartEvent::MissionEdited | ChartEvent::Selection(_) => {}
            }
        }
        let nav = self.nav.clone();
        let p = ui.painter_at(rect);
        let (w, h) = (rect.width(), rect.height());
        let at = |x: f32, y: f32| rect.min + Vec2::new(x, y);

        let wind_rect = Rect::from_min_size(at(10.0, 10.0), instruments::WIND_SIZE);
        let speed_rect = Rect::from_min_size(at(10.0, h - instruments::SPEED_SIZE.y - 10.0), instruments::SPEED_SIZE);
        let helm_rect = Rect::from_min_size(at(10.0 + instruments::SPEED_SIZE.x + 8.0, h - instruments::HELM_SIZE.y - 10.0), instruments::HELM_SIZE);
        let sounder_rect = Rect::from_min_size(at(w - instruments::SOUNDER_SIZE.x - 10.0, h - instruments::SOUNDER_SIZE.y - 10.0), instruments::SOUNDER_SIZE);
        if ui_settings.show_hud {
            // The compass keeps clear of the wind indicator on a narrow chart.
            let left = instruments::WIND_SIZE.x + 18.0;
            let cw = 520.0f32.min(w - left.max((w - 520.0) / 2.0) - 10.0).max(300.0);
            let cx = left.max((w - cw) / 2.0);
            instruments::compass_tape(&p, Rect::from_min_size(at(cx, 8.0), Vec2::new(cw, instruments::COMPASS_SIZE.y)), &nav);
            instruments::wind_indicator(&p, wind_rect, &nav, units_kn);
            let max_speed = self.settings.read().unwrap().limits.max_speed_ms;
            instruments::speed_dial(&p, speed_rect, &nav, units_kn, max_speed);
            instruments::helm_gauge(&p, helm_rect, &nav);
            // Squeezed on a narrow chart, the echo sounder gives way first.
            if helm_rect.right() + 8.0 <= sounder_rect.left() {
                self.sounder.paint(&p, sounder_rect, &nav);
            }
        }
        if !nav.alarms.is_empty() {
            let aw = 560.0f32.min(w - 40.0);
            instruments::alarm_banner(&p, Rect::from_min_size(at((w - aw) / 2.0, 76.0), Vec2::new(aw, 34.0)), &nav.alarms, now);
            ui.ctx().request_repaint_after(Duration::from_millis(100));
        }

        // Big zoom buttons, for a finger rather than a mouse wheel; holding
        // one keeps zooming.  Between them, how much water is on screen.
        let top = wind_rect.bottom() + 12.0 - rect.top();
        let zh = 192.0;
        let room = h - instruments::SPEED_SIZE.y - 20.0 - top;
        let zoom_rect = Rect::from_min_size(at(30.0, top + ((room - zh) / 2.0).max(0.0)), Vec2::new(62.0, zh));
        p.rect(zoom_rect, 12.0, Color32::from_rgba_premultiplied(4, 9, 13, 170), Stroke::new(1.0, Color32::from_rgba_premultiplied(19, 70, 86, 90)), StrokeKind::Middle);
        let b = |y: f32| Rect::from_min_size(zoom_rect.min + Vec2::new(6.0, y), Vec2::new(50.0, 50.0));
        let named = |r: egui::Response, name: &'static str| {
            r.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, name));
            r
        };
        let plus = named(ui.put(b(6.0), egui::Button::new(RichText::new("+").font(theme::bold(24.0))).corner_radius(10)), "Zoom in");
        let metres = self.canvas.view_width_m();
        let readout = if metres >= 1000.0 { format!("{:.1} km", metres / 1000.0) } else { format!("{metres:.0} m") };
        p.text(zoom_rect.min + Vec2::new(31.0, 66.0), Align2::CENTER_CENTER, readout, theme::bold(11.0), theme::TEXT);
        let minus = named(ui.put(b(76.0), egui::Button::new(RichText::new("−").font(theme::bold(24.0))).corner_radius(10)), "Zoom out");
        let view = named(ui.put(b(134.0), egui::Button::new(RichText::new(self.canvas.view_mode.label()).font(theme::bold(16.0))).corner_radius(10)), "Chart view");
        if view.on_hover_text("North up → heading up → 3D chase camera").clicked() {
            self.cycle_view();
        }
        for (response, steps) in [(plus, 0.5), (minus, -0.5)] {
            if response.is_pointer_button_down_on() {
                // Held: one step at once, then repeating.
                match self.zoom_hold {
                    Some((s, _, next)) if s == steps => {
                        if now >= next {
                            self.canvas.zoom(steps, None);
                            self.zoom_hold = Some((steps, now, now + 0.11));
                        }
                    }
                    _ => {
                        self.canvas.zoom(steps, None);
                        self.zoom_hold = Some((steps, now, now + 0.35));
                    }
                }
                ui.ctx().request_repaint();
            } else if response.clicked() {
                // A click that was not already counted as a press (keyboard,
                // screen reader, or a tap shorter than a frame) zooms once.
                if self.zoom_hold.is_none_or(|z| z.0 != steps) {
                    self.canvas.zoom(steps, None);
                }
                self.zoom_hold = None;
            } else if self.zoom_hold.is_some_and(|z| z.0 == steps) {
                self.zoom_hold = None;
            }
        }

        // The tabs on the chart's edges that open and close the side panels.
        // Thumb-sized: the Deck is used by touch as much as by mouse.
        let tab_w = crate::ui::chart::SIDE_TAB_W;
        let handle = |x: f32| Rect::from_min_size(at(x, (h - 96.0) / 2.0), Vec2::new(tab_w, 96.0));
        let left_text = if ui_settings.show_telemetry { "◀" } else { "▶" };
        if named(ui.put(handle(0.0), egui::Button::new(RichText::new(left_text).color(theme::ACCENT).font(theme::bold(18.0)))), "Telemetry panel").on_hover_text("Show or hide the telemetry").clicked() {
            self.toggle_telemetry();
        }
        let right_text = if ui_settings.show_drawer { "▶" } else { "◀" };
        if named(ui.put(handle(w - tab_w), egui::Button::new(RichText::new(right_text).color(theme::ACCENT).font(theme::bold(18.0)))), "Side panel").on_hover_text("Show or hide the panel").clicked() {
            self.toggle_drawer();
        }

        if let Some((text, until)) = &self.toast {
            if now < *until {
                let galley = p.layout(text.clone(), theme::font(15.0), theme::TEXT, w - 80.0);
                let size = galley.size() + Vec2::new(28.0, 16.0);
                let r = Rect::from_min_size(at((w - size.x) / 2.0, h - size.y - 180.0), size);
                p.rect(r, 8.0, Color32::from_rgba_premultiplied(5, 10, 15, 220), Stroke::new(1.0, theme::ACCENT), StrokeKind::Middle);
                p.galley(r.min + Vec2::new(14.0, 8.0), galley, theme::TEXT);
                ui.ctx().request_repaint_after(Duration::from_secs_f64((until - now).max(0.05)));
            } else {
                self.toast = None;
            }
        }
    }

    fn power_area(&mut self, ui: &mut Ui, now: f64) {
        let rect = ui.max_rect();
        let history: Vec<_> = self.vehicle.lock().power_history.iter().copied().collect();
        let nav = self.nav.clone();
        crate::ui::powerflow::paint(ui.painter(), rect, nav.telemetry.as_ref(), &history, nav.energy.as_ref(), self.power_window_s, now);
        let close = Rect::from_min_size(rect.right_top() + Vec2::new(-150.0, rect.height() - 52.0), Vec2::new(140.0, 40.0));
        if ui.put(close, egui::Button::new(RichText::new("🗺 Back to chart").strong())).clicked() {
            self.power_view = false;
        }
        ui.ctx().request_repaint();
    }

    pub fn describe_point(&self, p: LatLon) -> String {
        let mut text = format!("{:.5}°N  {:.5}°E", p.lat, p.lon);
        if let Some(chart) = &self.chart_grid {
            let depth = chart.depth_at(p.lat, p.lon);
            text += &if depth.is_nan() { "   land".to_string() } else { format!("   chart {depth:.1} m") };
        }
        let v = self.vehicle.lock();
        if let Some(surveyed) = v.soundings.depth_of(v.soundings.cell_of(p.lat, p.lon)) {
            text += &format!(" · surveyed {surveyed:.2} m");
        }
        text
    }

    fn dialogs(&mut self, ctx: &Context) {
        if let Some(dialog) = &mut self.file_dialog {
            let purpose = dialog.purpose;
            if let Some(outcome) = dialog.show(ctx) {
                self.file_dialog = None;
                if let Outcome::Chosen(path) = outcome {
                    self.file_chosen(purpose, path);
                }
            }
        }
        if let Some(dialog) = &mut self.connect {
            match dialog.show(ctx) {
                Some(crate::ui::connect::Answer::Cancel) => self.connect = None,
                Some(crate::ui::connect::Answer::Connect(choice, dont_ask)) => {
                    self.settings.write().unwrap().link.ask_at_start = !dont_ask;
                    match self.connect_to(choice) {
                        Ok(()) => self.connect = None,
                        Err(e) => {
                            if let Some(d) = &mut self.connect {
                                d.error = Some(e);
                            }
                        }
                    }
                }
                None => {}
            }
        }
        if let Some(confirm) = &self.confirm {
            let text = confirm.text.clone();
            let action = confirm.action;
            let mut decided = None;
            egui::Modal::new(egui::Id::new("confirm")).show(ctx, |ui| {
                ui.label(RichText::new(text).font(theme::font(15.0)));
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui.add_sized([110.0, 36.0], egui::Button::new("Cancel")).clicked() {
                        decided = Some(false);
                    }
                    if ui.add_sized([110.0, 36.0], egui::Button::new(RichText::new("Yes").strong())).clicked() {
                        decided = Some(true);
                    }
                });
            });
            if let Some(yes) = decided {
                self.confirm = None;
                if yes && action == ConfirmAction::ClearSurvey {
                    self.vehicle.lock().soundings.clear();
                    self.toast("Survey cleared", 2.0);
                }
            }
        }
    }

    /// Open a mission file, and replan it if it is a pattern.
    pub fn open_mission(&mut self, path: PathBuf) {
        self.file_chosen(Purpose::OpenMission, path);
    }

    fn file_chosen(&mut self, purpose: Purpose, path: PathBuf) {
        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        match purpose {
            Purpose::OpenMission => match crate::core::mission::Mission::load(&path) {
                Ok(mission) => {
                    if let Some(area) = mission.area {
                        self.canvas.rect_angle_deg = area.angle_deg;
                    }
                    let replan = mission.area.is_some() && (mission.pattern.pattern == crate::core::mission::Pattern::Auto || mission.waypoints.is_empty());
                    self.vehicle.lock().set_mission(mission);
                    self.canvas.selected = None;
                    if replan {
                        self.regenerate_route();
                    }
                    self.frame_mission();
                    self.toast(format!("Mission opened: {name}"), 3.0);
                }
                Err(e) => self.toast(format!("Could not read {name}: {e}"), 5.0),
            },
            Purpose::SaveMission => {
                let path = if path.extension().is_none() { path.with_extension("json") } else { path };
                let result = self.vehicle.lock().mission.save(&path);
                match result {
                    Ok(()) => self.toast(format!("Mission saved to {name}"), 3.0),
                    Err(e) => self.toast(format!("Could not save: {e}"), 5.0),
                }
            }
            Purpose::ExportWaypoints => {
                let path = if path.extension().is_none() { path.with_extension("waypoints") } else { path };
                let cruise = self.settings.read().unwrap().autopilot.cruise_speed_ms;
                let text = {
                    let v = self.vehicle.lock();
                    v.mission.to_waypoints_file(v.home(), cruise)
                };
                match std::fs::write(&path, text) {
                    Ok(()) => self.toast(format!("Waypoints for ArduPilot / QGroundControl saved to {name}"), 3.0),
                    Err(e) => self.toast(format!("Could not save: {e}"), 5.0),
                }
            }
            Purpose::ExportSoundings | Purpose::ExportGrid => {
                let result = {
                    let v = self.vehicle.lock();
                    if purpose == Purpose::ExportSoundings { v.soundings.export_xyz(&path) } else { v.soundings.export_grid_csv(&path) }
                };
                match result {
                    Ok(n) => self.toast(format!("Wrote {n} rows to {name}"), 3.0),
                    Err(e) => self.toast(format!("Could not write: {e}"), 5.0),
                }
            }
            Purpose::LoadChart => match DepthGrid::load(&path) {
                Ok(grid) => {
                    self.settings.write().unwrap().map.depth_grid = path.display().to_string();
                    self.set_chart(grid);
                }
                Err(e) => self.toast(format!("Could not read {name}: {e}"), 5.0),
            },
        }
    }

    pub fn shutdown(&mut self) {
        let page = self.page;
        self.edit_ui(|u| u.drawer_page = page);
        self.vehicle.shutdown();
        if let Some(mut pad) = self.pad.take() {
            pad.stop();
        }
        if let Err(e) = config::save(&self.settings.read().unwrap().clone(), None) {
            log::warn!("could not save settings: {e}");
        }
        let v = self.vehicle.lock();
        if !v.soundings.soundings.is_empty() {
            let folder = config::state_dir().join("surveys");
            let path = folder.join(clock::local_strftime("autosave-%Y%m%d-%H%M%S.csv"));
            if let Err(e) = std::fs::create_dir_all(&folder).and_then(|_| v.soundings.export_xyz(&path).map(|_| ())) {
                log::warn!("could not autosave the survey: {e}");
            }
        }
    }
}

/// A big button painted in the station's colours.
pub fn big_button(ui: &mut Ui, size: Vec2, text: &str, fill: Color32, stroke: Color32, font_size: f32, selected: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(size, Sense::click());
    // Its name and state for screen readers and the UI tests, as egui's own buttons have.
    let name = text.split_whitespace().collect::<Vec<_>>().join(" ");
    let enabled = ui.is_enabled();
    response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, enabled, selected, &name));
    let p = ui.painter();
    let fill = if response.is_pointer_button_down_on() { theme::darker(fill, 1.4) } else { fill };
    let edge = if response.hovered() && stroke == theme::BORDER { theme::ACCENT } else { stroke };
    let width = if stroke == theme::BORDER { 1.0 } else { 2.0 };
    p.rect(rect.shrink(1.0), 6.0, fill, Stroke::new(width, edge), StrokeKind::Middle);
    let lines: Vec<&str> = text.split('\n').collect();
    let mut size = font_size;
    let fits = |size: f32| {
        let widest = lines.iter().map(|l| p.layout_no_wrap(l.to_string(), theme::bold(size), theme::TEXT).size().x).fold(0.0, f32::max);
        widest <= rect.width() - 6.0 && size * 1.2 * lines.len() as f32 <= rect.height()
    };
    while size > 8.0 && !fits(size) {
        size -= 0.5;
    }
    let line_h = size * 1.2;
    let top = rect.center().y - line_h * lines.len() as f32 / 2.0 + line_h / 2.0;
    for (i, line) in lines.iter().enumerate() {
        p.text(Pos2::new(rect.center().x, top + i as f32 * line_h), Align2::CENTER_CENTER, *line, theme::bold(size), theme::TEXT);
    }
    response
}

/// A tool strip button: icon over name, lit when on.
pub fn tool_button(ui: &mut Ui, width: f32, text: &str, on: bool) -> egui::Response {
    let (fill, stroke) = if on { (theme::hex(0x12384a), theme::ACCENT) } else { (theme::SURFACE_RAISED, theme::BORDER) };
    big_button(ui, Vec2::new(width, 46.0), text, fill, stroke, 11.5, on)
}

/// Whether the flows are due a refresh (four times a second).
fn now_flows(last: f64) -> bool {
    clock::monotonic() - last >= 0.25
}
