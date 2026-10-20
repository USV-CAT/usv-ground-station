//! Every button in the station, clicked: the real window, run headless, driven
//! through egui's accessibility actions the way a screen reader would.  Each
//! check notes what went wrong and the test lists them all at the end.

mod common;

use std::time::{Duration, Instant};

use egui_kittest::kittest::Queryable;
use accesskit::Role;
use egui_kittest::Harness;
use usvgs::config::{Settings, StickMode, WaterStyle};
use usvgs::core::geo;
use usvgs::core::mission::{Pattern, Rectangle};
use usvgs::ui::app::{App, PAGES};
use usvgs::ui::chart::{Tool, ViewMode};
use usvgs::ui::files::Purpose;
use usvgs::vehicle::Mode;

type H<'a> = Harness<'a, Option<App>>;

/// The station's folders come from process-wide environment variables, so
/// these tests take turns rather than run at once.
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn take_turn() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|p| p.into_inner())
}

struct Audit<'a> {
    h: H<'a>,
    failures: Vec<String>,
}

impl Audit<'_> {
    fn app(&mut self) -> &mut App {
        self.h.state_mut().as_mut().unwrap()
    }

    fn frames(&mut self, n: usize) {
        for _ in 0..n {
            self.h.step();
        }
    }

    /// Click the first button (or else the first widget) with this exact
    /// label.  False if there is none.
    fn click(&mut self, label: &str) -> bool {
        let node = self
            .h
            .query_all_by_role_and_label(Role::Button, label)
            .next()
            .or_else(|| self.h.query_all_by_label(label).next());
        let found = match node {
            Some(node) => {
                node.click_accesskit();
                true
            }
            None => false,
        };
        if found {
            self.frames(2);
            // Let the 20 Hz control loop see it.
            std::thread::sleep(Duration::from_millis(70));
            self.frames(2);
        } else {
            self.failures.push(format!("no button labelled {label:?}"));
        }
        found
    }

    fn check(&mut self, label: &str, ok: impl FnOnce(&mut App) -> bool) {
        if self.click(label) && !ok(self.app()) {
            self.failures.push(format!("{label:?} did not do its job"));
        }
    }

    fn page(&mut self, title: &str) {
        let index = PAGES.iter().position(|p| *p == title).unwrap();
        self.check(title, |a| a.page == index);
    }
}

fn test_settings() -> Settings {
    let mut settings = Settings::default();
    settings.control.input_backend = "none".into();
    settings.sound.enabled = false;
    // Nothing is fetched from the internet in a test.
    settings.map.satellite_url = "http://127.0.0.1:9/{z}/{x}/{y}".into();
    settings
}

/// The station in a headless window, its folders in `dir`, once the boat reports.
fn station<'a>(dir: &std::path::Path, settings: Settings) -> Audit<'a> {
    std::env::set_var("XDG_STATE_HOME", dir.join("state"));
    std::env::set_var("XDG_CONFIG_HOME", dir.join("config"));
    std::env::set_var("XDG_CACHE_HOME", dir.join("cache"));
    let chart = common::chart();
    let h = Harness::builder().with_size(egui::vec2(1280.0, 800.0)).build_ui_state(
        move |ui, app: &mut Option<App>| {
            let a = app.get_or_insert_with(|| App::new(ui.ctx(), settings.clone(), Some((*chart).clone().into()), Some(false), None));
            a.frame(ui);
        },
        None,
    );
    let mut t = Audit { h, failures: Vec::new() };
    t.frames(3);
    let start = Instant::now();
    while t.app().nav.position().is_none() && start.elapsed() < Duration::from_secs(10) {
        std::thread::sleep(Duration::from_millis(50));
        t.frames(1);
    }
    assert!(t.app().nav.position().is_some(), "no telemetry from the simulator");
    t
}

impl Audit<'_> {
    /// Whether some text on screen contains `text`.
    fn shows(&self, text: &str) -> bool {
        self.h.query_all_by_label_contains(text).next().is_some()
    }

    fn wait(&mut self, seconds: f64) {
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            std::thread::sleep(Duration::from_millis(50));
            self.frames(1);
        }
    }

    fn expect(&mut self, what: &str, ok: bool) {
        if !ok {
            self.failures.push(what.to_string());
        }
    }
}

#[test]
fn the_battery_interface_tells_the_truth() {
    let _turn = take_turn();
    let dir = tempfile::tempdir().unwrap();
    let mut settings = test_settings();
    settings.sim.start_battery_pct = 40.0;
    settings.sim.time_scale = 8.0;
    settings.sim.wind_speed_ms = 0.0;
    settings.sim.current_speed_ms = 0.0;
    let mut t = station(dir.path(), settings);
    t.page("Power");
    t.wait(1.5);

    // Alongside the station: charging.
    let nav = t.app().nav.clone();
    let tel = nav.telemetry.clone().unwrap();
    let pct = tel.battery_pct.unwrap();
    t.expect(&format!("charge {pct:.0} % near the 40 % it launched with"), (39.0..45.0).contains(&pct));
    t.expect("current is negative while charging", tel.battery_a.unwrap() < 0.0);
    t.expect("the power page says it is charging", t.shows("charging"));
    t.expect("the telemetry shows the charge", t.shows(&format!("{pct:.0} %")) || t.shows(&format!("{:.0} %", pct + 1.0)));
    t.expect("no time to empty while charging", t.shows("∞"));
    t.expect("energy used never goes negative", nav.energy.is_some_and(|e| e.used_wh >= 0.0));
    t.expect("energy charged is counted", nav.charged_wh > 0.0);
    let p = tel.power.clone().unwrap();
    t.expect("both banks report", p.banks.len() == 2 && p.banks.iter().all(|b| b.amps < 0.0));
    let mean = (p.banks[0].soc_pct + p.banks[1].soc_pct) / 2.0;
    t.expect("the banks add up to the pack", (mean - pct).abs() < 1.0);

    // Away from the station, under way: discharging.
    t.check("ARM", |a| a.vehicle.lock().armed());
    // Ahead on the keyboard, as an operator would.
    t.h.key_down(egui::Key::W);
    t.wait(3.0);
    let nav = t.app().nav.clone();
    let tel = nav.telemetry.clone().unwrap();
    let p = tel.power.clone().unwrap();
    t.expect(&format!("drew current under way ({:?} A)", tel.battery_a), tel.battery_a.unwrap() > 5.0);
    t.expect("shore power off once under way", !p.shore_connected && p.shore_w == 0.0);
    t.expect("thrusters draw power", p.port_w > 20.0 && p.stbd_w > 20.0);
    let used = nav.energy.map(|e| e.used_wh).unwrap_or(0.0);
    t.expect(&format!("energy used grows ({used:.2} Wh)"), used > 0.0);
    t.expect("the power page shows not connected", t.shows("not connected"));
    t.expect("the time to empty is finite", nav.energy.is_some_and(|e| e.time_to_empty_s.is_finite()));
    t.h.key_up(egui::Key::W);
    t.check("⚡ Power flow diagram (P)", |a| a.power_view);
    t.frames(3);
    t.check("🗺 Back to chart", |a| !a.power_view);
    t.app().shutdown();
    assert!(t.failures.is_empty(), "battery interface:\n  {}", t.failures.join("\n  "));
}

#[test]
fn every_button_does_its_job() {
    let _turn = take_turn();
    let dir = tempfile::tempdir().unwrap();
    let mut t = station(dir.path(), test_settings());

    // -- the mode bar --------------------------------------------------------------
    t.check("ARM", |a| a.vehicle.lock().armed());
    for (label, mode) in [
        ("Speed Hold", Mode::SpeedHold),
        ("Speed/Course Hold", Mode::CourseHold),
        ("⚓ Station Hold", Mode::Hold),
        ("⌂ Return Home", Mode::ReturnHome),
        ("■ USV STOP", Mode::Stop),
        ("🎮 Manual", Mode::Manual),
    ] {
        t.check(label, |a| a.vehicle.lock().mode() == mode);
    }
    // Point Nav with no point asks for one: the Go-to tool.
    t.check("◎ Point Nav", |a| a.canvas.tool == Tool::Goto);
    // Preprogram with no mission stays put.
    t.check("▶ Pre- program", |a| a.vehicle.lock().mode() == Mode::Manual);
    t.check("ARMED (disarm)", |a| !a.vehicle.lock().armed());

    // -- the status bar: full screen on and off (the Deck has no F11) --------------
    let on = t.click("Full screen") && t.app().is_fullscreen();
    t.expect("the Full screen button did not turn full screen on", on);
    let off = t.click("Full screen") && !t.app().is_fullscreen();
    t.expect("the Full screen button did not turn full screen off", off);

    // -- the tool strip -----------------------------------------------------------
    for (label, tool) in [("+ Points", Tool::Points), ("▭ Rect", Tool::Rect), ("◎ Go to", Tool::Goto), ("✋ Pan", Tool::Pan)] {
        t.check(label, |a| a.canvas.tool == tool);
    }
    let follow = t.app().canvas.follow;
    t.check("⌖ Follow", |a| a.canvas.follow != follow);
    t.check("N↑ View", |a| a.canvas.view_mode == ViewMode::Heading);
    t.check("H↑ View", |a| a.canvas.view_mode == ViewMode::Chase);
    t.check("3D View", |a| a.canvas.view_mode == ViewMode::North);
    t.check("Chart view", |a| a.canvas.view_mode == ViewMode::Heading);
    t.check("Chart view", |a| a.canvas.view_mode == ViewMode::Chase);
    t.check("Chart view", |a| a.canvas.view_mode == ViewMode::North);
    let vectors = t.app().canvas.show_vectors;
    t.check("➚ Vectors", |a| a.canvas.show_vectors != vectors);
    t.check("➚ Vectors", |a| a.canvas.show_vectors == vectors);
    t.check("⛭ Skim", |a| a.vehicle.lock().skimmer_commanded());
    t.check("⛭ Skim", |a| !a.vehicle.lock().skimmer_commanded());
    t.check("⚡ Power", |a| a.power_view);
    t.check("🗺 Back to chart", |a| !a.power_view);
    t.check("⌂ Set H", |a| {
        let v = a.vehicle.lock();
        let home = v.home().unwrap();
        let here = v.telemetry().and_then(|t| t.position()).unwrap();
        geo::distance_m(home, here) < 5.0
    });
    let hud = t.app().ui_settings().show_hud;
    t.check("◑ HUD", |a| a.ui_settings().show_hud != hud);
    t.check("◑ HUD", |a| a.ui_settings().show_hud == hud);
    let drawer = t.app().ui_settings().show_drawer;
    t.check("☰ Panel", |a| a.ui_settings().show_drawer != drawer);
    t.check("Side panel", |a| a.ui_settings().show_drawer == drawer);
    let telemetry = t.app().ui_settings().show_telemetry;
    t.check("Telemetry panel", |a| a.ui_settings().show_telemetry != telemetry);
    t.check("Telemetry panel", |a| a.ui_settings().show_telemetry == telemetry);
    let scale = t.app().canvas.scale;
    t.check("Zoom in", |a| a.canvas.scale > scale);
    t.check("Zoom out", |a| (a.canvas.scale - scale).abs() / scale < 0.5);

    // -- the drawer's page buttons -------------------------------------------------
    for title in PAGES {
        t.page(title);
    }

    // -- Mission ---------------------------------------------------------------------
    t.page("Mission");
    let here = t.app().nav.position().unwrap();
    {
        let app = t.app();
        let area = Rectangle::new(geo::destination(here, 300.0, 120.0), 120.0, 60.0, 90.0);
        app.vehicle.lock().mission.area = Some(area);
        app.regenerate_route();
    }
    t.frames(2);
    for pattern in [Pattern::Grid, Pattern::SpiralIn, Pattern::SpiralOut, Pattern::Perimeter, Pattern::Auto, Pattern::Lawnmower] {
        t.check(pattern.label(), |a| {
            let v = a.vehicle.lock();
            v.mission.pattern.pattern == pattern && !v.mission.waypoints.is_empty()
        });
    }
    // The start corners, named by compass point: the first is the south-west.
    t.check("SW", |a| a.vehicle.lock().mission.pattern.start_corner == 0);
    t.check("NE", |a| a.vehicle.lock().mission.pattern.start_corner == 2);
    t.check("kn", |a| a.speed_unit() == usvgs::core::units::SpeedUnit::MetresPerSecond);
    t.check("m/s", |a| a.speed_unit() == usvgs::core::units::SpeedUnit::Knots);
    let (first, last) = {
        let v = t.app().vehicle.lock();
        (v.mission.waypoints[0].position(), v.mission.waypoints.last().unwrap().position())
    };
    t.check("Reverse", |a| {
        let v = a.vehicle.lock();
        v.mission.waypoints[0].position() == last && v.mission.waypoints.last().unwrap().position() == first
    });
    t.check("📂 Open…", |a| a.file_dialog.as_ref().is_some_and(|d| d.purpose == Purpose::OpenMission));
    t.check("Cancel", |a| a.file_dialog.is_none());
    t.check("💾 Save…", |a| a.file_dialog.as_ref().is_some_and(|d| d.purpose == Purpose::SaveMission));
    t.check("Save", |a| a.file_dialog.is_none());
    t.check("⬇ .waypoints", |a| a.file_dialog.as_ref().is_some_and(|d| d.purpose == Purpose::ExportWaypoints));
    t.check("Save", |a| a.file_dialog.is_none());
    let missions = dir.path().join("state").join("usvgs").join("missions");
    let saved: Vec<String> = std::fs::read_dir(&missions).map(|r| r.flatten().map(|e| e.file_name().to_string_lossy().to_string()).collect()).unwrap_or_default();
    if !saved.iter().any(|n| n.ends_with(".json")) || !saved.iter().any(|n| n.ends_with(".waypoints")) {
        t.failures.push(format!("saving wrote {saved:?}"));
    }
    t.check("▶ Start", |a| a.vehicle.lock().mode() == Mode::Auto);
    t.check("Skip ⏭", |a| a.nav.waypoint_index == Some(1));
    t.check("⟲ Restart", |a| a.nav.waypoint_index == Some(0));
    t.check("■ USV STOP", |a| a.vehicle.lock().mode() == Mode::Stop);
    t.check("Frame", |a| !a.canvas.follow);
    t.check("Clear", |a| a.vehicle.lock().mission.waypoints.is_empty());

    // -- Chart ---------------------------------------------------------------------
    t.page("Chart");
    t.check("Depth chart", |a| a.canvas.water_style == WaterStyle::Chart);
    t.check("🛰 Satellite", |a| a.canvas.water_style == WaterStyle::Satellite);
    t.check("Natural", |a| a.canvas.water_style == WaterStyle::Natural);
    t.check("30 fps", |a| a.ui_settings().max_fps == 30);
    t.check("60 fps", |a| a.ui_settings().max_fps == 60);
    t.check("Speed in m/s", |a| a.speed_unit() == usvgs::core::units::SpeedUnit::MetresPerSecond);
    t.check("Speed in knots", |a| a.speed_unit() == usvgs::core::units::SpeedUnit::Knots);
    t.check("Track", |a| !a.canvas.show_track);
    t.check("Track", |a| a.canvas.show_track);
    t.check("Export soundings…", |a| a.file_dialog.as_ref().is_some_and(|d| d.purpose == Purpose::ExportSoundings));
    t.check("Cancel", |a| a.file_dialog.is_none());
    t.check("Clear survey", |a| a.confirm.is_some());
    // The boat keeps sounding, so a ping or two may land straight after.
    t.check("Yes", |a| a.confirm.is_none() && a.vehicle.lock().soundings.soundings.len() <= 3);
    t.check("Clear track", |a| a.vehicle.lock().track.len() <= 1);
    t.check("Load depth chart…", |a| a.file_dialog.as_ref().is_some_and(|d| d.purpose == Purpose::LoadChart));
    t.check("Cancel", |a| a.file_dialog.is_none());

    // -- Power ---------------------------------------------------------------------
    t.page("Power");
    t.check("⚡ Power flow diagram (P)", |a| a.power_view);
    t.check("🗺 Back to chart", |a| !a.power_view);
    t.check("10 min", |a| a.power_window_s == 600.0);
    t.check("2 h", |a| a.power_window_s == 7200.0);

    // -- Oil -------------------------------------------------------------------------
    t.page("Oil");
    t.check("⛭ Deploy skimmer", |a| a.vehicle.lock().skimmer_commanded());
    t.check("⛭ Stow skimmer", |a| !a.vehicle.lock().skimmer_commanded());
    t.check("▭ Plan sweep on slick", |a| {
        let v = a.vehicle.lock();
        v.mission.drift_ms.is_some() && !v.mission.waypoints.is_empty()
    });
    t.check("Clear", |a| a.sim().unwrap().spill_snapshot().is_none());
    t.check("New spill", |a| a.sim().unwrap().spill_snapshot().is_some());
    t.check("Spill at view", |a| a.sim().unwrap().spill_snapshot().is_some());

    // -- Pad ---------------------------------------------------------------------------
    t.page("🎮 Pad");
    t.check("Tank sticks", |a| a.settings.read().unwrap().control.stick_mode == StickMode::Tank);
    t.check("Arcade sticks", |a| a.settings.read().unwrap().control.stick_mode == StickMode::Arcade);

    // -- Limits --------------------------------------------------------------------
    t.page("⚙ Limits");
    t.check("Limit the sticks too", |a| !a.settings.read().unwrap().limits.in_manual);
    t.app().settings.write().unwrap().limits.max_turn_dps = 12.0;
    t.check("Reset the limits", |a| a.settings.read().unwrap().limits == usvgs::config::LimitSettings::default());

    // -- Sound -----------------------------------------------------------------------
    t.page("🔊 Sound");
    t.check("Speak alerts", |a| a.settings.read().unwrap().sound.enabled);
    t.check("Русский", |a| a.settings.read().unwrap().sound.language == "ru");
    t.check("English", |a| a.settings.read().unwrap().sound.language == "en");
    t.check("Shallow water", |a| !a.settings.read().unwrap().sound.rule("shallow").enabled);
    t.check("Repeat warnings only while armed", |a| !a.settings.read().unwrap().sound.repeat_only_armed);
    t.check("Reset the rules", |a| {
        let s = a.settings.read().unwrap().sound.clone();
        s.rule("shallow").enabled && s.repeat_only_armed
    });
    t.check("Speak alerts", |a| !a.settings.read().unwrap().sound.enabled);

    // -- Sim -------------------------------------------------------------------------
    t.page("Sim");
    t.check("Pause", |a| a.sim().unwrap().paused());
    t.check("Pause", |a| !a.sim().unwrap().paused());
    t.app().sim().unwrap().core().boat.state.battery_soc = 0.5;
    t.check("Recharge battery", |a| a.sim().unwrap().core().boat.state.battery_soc > 0.999);
    t.check("Respawn at launch", |a| a.sim().unwrap().range_m() < 50.0);
    t.check("Launch here", |a| a.sim().unwrap().range_m() < 50.0);

    t.app().shutdown();
    assert!(t.failures.is_empty(), "{} buttons failed:\n  {}", t.failures.len(), t.failures.join("\n  "));
}

/// The side buttons clicked with the pointer, where they are drawn — so
/// anything lying over them, or a hit area in the wrong place, shows.
#[test]
fn side_buttons_work_under_the_pointer() {
    let _turn = take_turn();
    let dir = tempfile::tempdir().unwrap();
    let mut t = station(dir.path(), test_settings());
    let press = |t: &mut Audit, label: &str| -> bool {
        let node = t.h.query_all_by_role_and_label(Role::Button, label).next();
        match node {
            Some(n) => {
                n.click();
                t.frames(3);
                true
            }
            None => {
                t.failures.push(format!("no button {label:?}"));
                false
            }
        }
    };
    for round in 0..2 {
        let before = t.app().ui_settings();
        if press(&mut t, "Telemetry panel") && t.app().ui_settings().show_telemetry == before.show_telemetry {
            t.failures.push(format!("left edge tab did nothing (round {round})"));
        }
        let before = t.app().ui_settings();
        if press(&mut t, "Side panel") && t.app().ui_settings().show_drawer == before.show_drawer {
            t.failures.push(format!("right edge tab did nothing (round {round})"));
        }
    }
    for (label, check) in [
        ("+ Points", Box::new(|a: &mut App| a.canvas.tool == Tool::Points) as Box<dyn Fn(&mut App) -> bool>),
        ("✋ Pan", Box::new(|a: &mut App| a.canvas.tool == Tool::Pan)),
        ("⚡ Power", Box::new(|a: &mut App| a.power_view)),
        ("⚡ Power", Box::new(|a: &mut App| !a.power_view)),
        ("☰ Panel", Box::new(|a: &mut App| !a.ui_settings().show_drawer)),
        ("☰ Panel", Box::new(|a: &mut App| a.ui_settings().show_drawer)),
        ("◑ HUD", Box::new(|a: &mut App| !a.ui_settings().show_hud)),
        ("◑ HUD", Box::new(|a: &mut App| a.ui_settings().show_hud)),
        ("Zoom in", Box::new(|_: &mut App| true)),
    ] {
        if press(&mut t, label) && !check(t.app()) {
            t.failures.push(format!("{label:?} did nothing under the pointer"));
        }
    }
    t.app().shutdown();
    assert!(t.failures.is_empty(), "side buttons:\n  {}", t.failures.join("\n  "));
}

/// The start dialog: nothing runs until a boat is chosen; the simulator and
/// a MAVLink boat can be swapped, and the mission carries over.
#[test]
fn the_start_dialog_connects_and_reconnects() {
    let _turn = take_turn();
    let dir = tempfile::tempdir().unwrap();
    std::env::set_var("XDG_STATE_HOME", dir.path().join("state"));
    std::env::set_var("XDG_CONFIG_HOME", dir.path().join("config"));
    std::env::set_var("XDG_CACHE_HOME", dir.path().join("cache"));
    let settings = test_settings();
    let chart = common::chart();
    let h = Harness::builder().with_size(egui::vec2(1280.0, 800.0)).build_ui_state(
        move |ui, app: &mut Option<App>| {
            let a = app.get_or_insert_with(|| App::new_with(ui.ctx(), settings.clone(), Some((*chart).clone().into()), Some(false), None, None));
            a.frame(ui);
        },
        None,
    );
    let mut t = Audit { h, failures: Vec::new() };
    t.frames(3);
    let ok = t.app().connect.is_some();
    t.expect("the dialog is up at start", ok);
    let ok = t.app().link.name() == "NO BOAT";
    t.expect("no boat until one is chosen", ok);
    let ok = !t.shows("Cancel");
    t.expect("no Cancel with nothing to go back to", ok);

    t.check("Start the simulator", |a| a.connect.is_none() && a.link.simulated());
    t.wait(1.0);
    let ok = t.app().nav.position().is_some();
    t.expect("the simulator reports", ok);
    t.app().vehicle.lock().mission.add(57.04, 24.0);

    // The link chip in the status bar opens the dialog again.
    t.check("Connection", |a| a.connect.is_some());
    t.check("Cancel", |a| a.connect.is_none() && a.link.simulated());
    t.check("Connection", |a| a.connect.is_some());
    t.check("Boat", |a| a.connect.as_ref().is_some_and(|d| d.boat));
    t.check("Serial radio", |a| a.connect.as_ref().is_some_and(|d| d.kind == usvgs::ui::connect::Kind::Serial));
    t.app().connect.as_mut().unwrap().device = "/dev/no-such-radio".into();
    t.check("Connect to the boat", |a| a.connect.as_ref().is_some_and(|d| d.error.is_some()) && a.link.simulated());
    let ok = t.shows("Could not connect: /dev/no-such-radio");
    t.expect("the failure is explained", ok);
    t.check("UDP listen", |a| a.connect.as_ref().is_some_and(|d| d.kind == usvgs::ui::connect::Kind::UdpListen));
    t.app().connect.as_mut().unwrap().port = "0".into();
    t.check("Connect to the boat", |a| a.connect.is_none() && a.link.name().starts_with("MAVLINK UDP"));
    let ok = t.app().settings.read().unwrap().link.recent.first().map(String::as_str) == Some("udp-listen:0");
    t.expect("remembered", ok);
    let ok = t.app().vehicle.lock().mission.waypoints.len() == 1;
    t.expect("the mission carried over", ok);
    t.wait(0.5);
    let ok = !t.app().nav.link_ok && t.app().nav.telemetry.is_none();
    t.expect("no boat answering: NO LINK", ok);

    // And back.
    t.check("Connection", |a| a.connect.is_some());
    t.check("Simulator", |a| a.connect.as_ref().is_some_and(|d| !d.boat));
    t.check("Connect like this at start, without asking", |a| a.connect.as_ref().is_some_and(|d| d.dont_ask));
    t.check("Start the simulator", |a| a.link.simulated() && !a.settings.read().unwrap().link.ask_at_start);
    t.app().shutdown();
    assert!(t.failures.is_empty(), "start dialog:\n  {}", t.failures.join("\n  "));
}

/// The Link page shows each data path with its rate and loss, and taking a
/// path out moves the traffic to the next; the Info page tells the facts.
#[test]
fn the_link_and_info_pages() {
    let _turn = take_turn();
    let dir = tempfile::tempdir().unwrap();
    let mut t = station(dir.path(), test_settings());
    t.page("📡 Link");
    t.wait(2.0);
    let flows = t.app().flows.clone();
    let names: Vec<String> = flows.iter().map(|f| f.bearer.label().to_string()).collect();
    t.expect(&format!("three paths: {names:?}"), names == ["Wi-Fi", "5G / LTE", "LoRa"]);
    t.expect("Wi-Fi carries the telemetry at the station", flows.first().is_some_and(|f| f.state == usvgs::link::flows::FlowState::Active));
    t.expect("Wi-Fi's rate is real", flows.first().is_some_and(|f| f.rx_bps > 10_000.0));
    let ok = t.shows("Telemetry by") && t.shows("Wi-Fi");
    t.expect("the summary names the path", ok);
    for label in ["In", "Out", "Loss", "Latency"] {
        let ok = t.shows(label);
        t.expect(&format!("the {label} column"), ok);
    }
    let ok = t.shows("kbit/s") && t.shows(" %");
    t.expect("rates and loss on screen", ok);

    // Take Wi-Fi out: 5G takes over at once.
    t.check("Use Wi-Fi 5 GHz", |a| a.link.flows()[0].state == usvgs::link::flows::FlowState::Off);
    t.wait(1.5);
    let flows = t.app().flows.clone();
    t.expect("5G carries it now", flows.get(1).is_some_and(|f| f.state == usvgs::link::flows::FlowState::Active));
    let logged = t.app().log.iter().any(|(_, text, _)| text.contains("now on 5G"));
    t.expect("the handover is in the log", logged);
    let ok = t.app().nav.link_ok;
    t.expect("still linked", ok);
    t.check("Use Wi-Fi 5 GHz", |a| a.link.flows()[0].state != usvgs::link::flows::FlowState::Off);

    t.page("ℹ Info");
    for text in ["CONNECTION", "Simulated catamaran", "Path in use", "Software", "Connected for"] {
        let ok = t.shows(text);
        t.expect(&format!("Info shows {text:?}"), ok);
    }
    t.check("🔌 Change connection", |a| a.connect.is_some());
    t.check("Cancel", |a| a.connect.is_none());
    t.check("📡 Data paths", |a| PAGES[a.page] == "📡 Link");
    t.app().shutdown();
    assert!(t.failures.is_empty(), "link pages:\n  {}", t.failures.join("\n  "));
}
