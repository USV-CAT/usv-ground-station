//! The pages of the pull-out panel.

use egui::{Color32, Pos2, Rect, RichText, Sense, Ui, Vec2};

use crate::config::{state_dir, StickMode, WaterStyle};
use crate::core::clock;
use crate::core::geo::{self, LatLon, Metres};
use crate::core::mission::{Pattern, PatternSpec, Rectangle};
use crate::core::profile::route_profile;
use crate::core::units;
use crate::input::mapping::BUTTON_HELP;
use crate::sim::oil::appearance;
use crate::ui::app::{big_button, App, Confirm, ConfirmAction};
use crate::ui::chart::Tool;
use crate::ui::files::{FileDialog, Purpose};
use crate::ui::{graphs, theme};

pub fn heading(ui: &mut Ui, text: &str) {
    ui.add_space(4.0);
    ui.label(RichText::new(text).font(theme::bold(12.5)).color(theme::ACCENT));
}

pub fn note(ui: &mut Ui, text: &str) {
    ui.label(RichText::new(text).color(theme::TEXT_DIM).font(theme::font(12.0)));
}

/// A number with − and + either side, big enough for a thumb.
#[allow(clippy::too_many_arguments)]
pub fn spin(ui: &mut Ui, label: &str, value: &mut f64, low: f64, high: f64, step: f64, suffix: &str, decimals: usize) -> bool {
    let mut changed = false;
    ui.label(label);
    ui.horizontal(|ui| {
        if ui.add_sized([28.0, 28.0], egui::Button::new("−")).clicked() {
            *value = (*value - step).max(low);
            changed = true;
        }
        let r = ui.add_sized(
            [70.0, 28.0],
            egui::DragValue::new(value).range(low..=high).speed(step).suffix(suffix).fixed_decimals(decimals),
        );
        changed |= r.changed();
        if ui.add_sized([28.0, 28.0], egui::Button::new("+")).clicked() {
            *value = (*value + step).min(high);
            changed = true;
        }
    });
    ui.end_row();
    changed
}

/// Big readouts in a grid: label above, value below.
pub fn value_grid(ui: &mut Ui, id: &str, items: &[(&str, String, Color32)]) {
    egui::Frame::new()
        .fill(theme::SURFACE)
        .stroke(egui::Stroke::new(1.0, theme::BORDER))
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(8, 6))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            let w = (ui.available_width() - 10.0) / 2.0;
            egui::Grid::new(id).num_columns(2).spacing([10.0, 2.0]).min_col_width(w).show(ui, |ui| {
                for pair in items.chunks(2) {
                    for (label, _, _) in pair {
                        ui.label(RichText::new(*label).color(theme::TEXT_DIM).font(theme::font(11.5)));
                    }
                    ui.end_row();
                    for (_, value, colour) in pair {
                        ui.label(RichText::new(value).color(*colour).font(theme::mono_bold(16.0)));
                    }
                    ui.end_row();
                }
            });
        });
}

/// Room for a drawing, the full width of the page.
fn canvas(ui: &mut Ui, height: f32) -> (Rect, egui::Painter) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), height), Sense::hover());
    (rect, ui.painter_at(rect))
}

fn row_buttons(ui: &mut Ui, labels: &[&str]) -> Option<usize> {
    let mut clicked = None;
    let n = labels.len() as f32;
    let w = (ui.available_width() - 6.0 * (n - 1.0)) / n;
    ui.horizontal(|ui| {
        for (i, l) in labels.iter().enumerate() {
            if ui.add_sized([w, 34.0], egui::Button::new(*l)).clicked() {
                clicked = Some(i);
            }
        }
    });
    clicked
}

fn toggle_row(ui: &mut Ui, items: &[(&str, bool)]) -> Option<usize> {
    let mut clicked = None;
    let n = items.len() as f32;
    let w = (ui.available_width() - 6.0 * (n - 1.0)) / n;
    ui.horizontal(|ui| {
        for (i, (l, on)) in items.iter().enumerate() {
            let (fill, stroke) = if *on { (theme::hex(0x12384a), theme::ACCENT) } else { (theme::SURFACE_RAISED, theme::BORDER) };
            if big_button(ui, Vec2::new(w, 34.0), l, fill, stroke, 13.0, *on).clicked() {
                clicked = Some(i);
            }
        }
    });
    clicked
}

/// The value at `q` (0–100) of `values`, interpolated as numpy does.
pub fn percentile(values: &[f64], q: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut v = values.to_vec();
    v.sort_by(|a, b| a.total_cmp(b));
    let rank = q / 100.0 * (v.len() - 1) as f64;
    let (lo, hi) = (rank.floor() as usize, rank.ceil() as usize);
    v[lo] + (v[hi] - v[lo]) * (rank - lo as f64)
}

impl App {
    // ------------------------------------------------------------------
    // Mission
    // ------------------------------------------------------------------

    pub fn speed_unit(&self) -> units::SpeedUnit {
        units::SpeedUnit::from_setting(&self.settings.read().unwrap().map.speed_units)
    }

    pub fn toggle_speed_unit(&mut self) {
        let next = self.speed_unit().other();
        self.settings.write().unwrap().map.speed_units = next.setting().into();
    }

    pub fn page_mission(&mut self, ui: &mut Ui) {
        let tool = self.canvas.tool;
        if let Some(i) = toggle_row(ui, &[("+ Points", tool == Tool::Points), ("▭ Rect", tool == Tool::Rect), ("◎ Go to", tool == Tool::Goto)]) {
            let wanted = [Tool::Points, Tool::Rect, Tool::Goto][i];
            self.set_tool(if tool == wanted { Tool::Pan } else { wanted });
        }

        heading(ui, "PATTERN");
        let (mut spec, mut looped, area, waypoints) = {
            let v = self.vehicle.lock();
            (v.mission.pattern, v.mission.looped, v.mission.area, v.mission.waypoints.clone())
        };
        let before = spec;
        // Six pattern buttons, each with its drawing.
        let w = (ui.available_width() - 12.0) / 3.0;
        for row in Pattern::ALL.chunks(3) {
            ui.horizontal(|ui| {
                for &pattern in row {
                    let on = spec.pattern == pattern;
                    let (rect, response) = ui.allocate_exact_size(Vec2::new(w, 64.0), Sense::click());
                    let enabled = ui.is_enabled();
                    response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, enabled, on, pattern.label()));
                    let p = ui.painter();
                    let (fill, edge) = if on { (theme::hex(0x12384a), theme::ACCENT) } else { (theme::SURFACE_RAISED, theme::BORDER) };
                    p.rect(rect.shrink(1.0), 6.0, fill, egui::Stroke::new(if on { 2.0 } else { 1.0 }, edge), egui::StrokeKind::Middle);
                    let icon = Rect::from_min_size(rect.min + Vec2::new(rect.width() / 2.0 - 22.0, 4.0), Vec2::new(44.0, 38.0));
                    let colour = if pattern == Pattern::Auto { theme::TARGET } else if on { theme::VALUE } else { theme::TEXT };
                    graphs::pattern_icon(p, icon, pattern, colour);
                    p.text(Pos2::new(rect.center().x, rect.bottom() - 11.0), egui::Align2::CENTER_CENTER, pattern.label(), theme::bold(11.5), theme::TEXT);
                    if response.on_hover_text(pattern.description()).clicked() {
                        spec.pattern = pattern;
                    }
                }
            });
        }
        note(ui, spec.pattern.description());

        let unit = self.speed_unit();
        let max_speed = self.settings.read().unwrap().limits.max_speed_ms;
        let mut angle = area.map(|a| a.angle_deg).unwrap_or(self.canvas.rect_angle_deg);
        let mut cruise = unit.from_ms(self.settings.read().unwrap().autopilot.cruise_speed_ms);
        let (mut speed_changed, mut loop_changed, mut unit_clicked) = (false, false, false);
        egui::Grid::new("pattern").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            if matches!(spec.pattern, Pattern::Perimeter) {
                let mut laps = spec.laps as f64;
                if spin(ui, "Laps", &mut laps, 1.0, 20.0, 1.0, "", 0) {
                    spec.laps = laps as u32;
                }
            } else {
                if spin(ui, "Swath", &mut spec.swath_m, 0.5, 200.0, 0.5, " m", 1) | spin(ui, "Overlap", &mut spec.overlap_pct, 0.0, 90.0, 5.0, " %", 0) {
                    spec.apply_overlap();
                }
                ui.label("Lane spacing");
                ui.label(RichText::new(format!("{:.1} m", spec.spacing_m)).color(theme::VALUE).font(theme::mono_bold(13.0)));
                ui.end_row();
            }
            spin(ui, "Edge offset", &mut spec.margin_m, 0.0, 100.0, 1.0, " m", 0);
            spin(ui, "Rotation", &mut angle, 0.0, 359.0, 5.0, "°", 0);
            if matches!(spec.pattern, Pattern::Lawnmower | Pattern::Grid) {
                ui.label("");
                ui.checkbox(&mut spec.along_long_side, "Lanes along long side");
                ui.end_row();
            }
            if spec.pattern != Pattern::Auto {
                ui.label("Start corner");
                ui.horizontal(|ui| {
                    // Name the corners by where they lie from the middle.
                    let rect = area.unwrap_or(Rectangle::new(LatLon::new(57.0, 24.0), 100.0, 60.0, angle));
                    for (i, c) in rect.corners().iter().enumerate() {
                        let name = units::compass_point(geo::bearing_deg(rect.centre, *c));
                        let on = spec.start_corner as usize == i;
                        let (fill, stroke) = if on { (theme::hex(0x12384a), theme::ACCENT) } else { (theme::SURFACE_RAISED, theme::BORDER) };
                        if big_button(ui, Vec2::new(40.0, 28.0), name, fill, stroke, 12.0, on).clicked() {
                            spec.start_corner = i as u8;
                        }
                    }
                });
                ui.end_row();
            }
            ui.label("Cruise speed");
            ui.horizontal(|ui| {
                if ui.add_sized([28.0, 28.0], egui::Button::new("−")).clicked() {
                    cruise = (cruise - if unit == units::SpeedUnit::Knots { 0.1 } else { 0.05 }).max(unit.from_ms(0.1));
                    speed_changed = true;
                }
                let r = ui.add_sized(
                    [70.0, 28.0],
                    egui::DragValue::new(&mut cruise).range(unit.from_ms(0.1)..=unit.from_ms(max_speed)).speed(0.05).fixed_decimals(unit.decimals()),
                );
                speed_changed |= r.changed();
                if ui.add_sized([28.0, 28.0], egui::Button::new("+")).clicked() {
                    cruise = (cruise + if unit == units::SpeedUnit::Knots { 0.1 } else { 0.05 }).min(unit.from_ms(max_speed));
                    speed_changed = true;
                }
                unit_clicked = ui.add_sized([40.0, 28.0], egui::Button::new(RichText::new(unit.suffix()).strong())).on_hover_text("Knots or metres a second").clicked();
            });
            ui.end_row();
            ui.label("");
            loop_changed |= ui.checkbox(&mut looped, "Loop the route").changed();
            ui.end_row();
        });
        let angle_changed = area.is_some_and(|a| (a.angle_deg - angle).abs() > 0.5) || (area.is_none() && angle != self.canvas.rect_angle_deg);
        if spec != before || angle_changed {
            {
                let mut v = self.vehicle.lock();
                v.mission.pattern = spec;
                if let Some(a) = &mut v.mission.area {
                    a.angle_deg = angle;
                }
            }
            self.canvas.rect_angle_deg = angle;
            self.regenerate_route();
        }
        if speed_changed {
            self.settings.write().unwrap().autopilot.cruise_speed_ms = unit.to_ms(cruise);
        }
        if unit_clicked {
            self.toggle_speed_unit();
        }
        if loop_changed {
            self.vehicle.lock().mission.looped = looped;
        }
        if area.is_none() && waypoints.is_empty() {
            note(ui, "Draw a rectangle with ▭ Rect and the pattern fills it.");
        }
        if let Some(summary) = &self.auto_summary {
            ui.label(RichText::new(format!("🤖 {summary}")).color(theme::TARGET));
        }

        let boat = self.boat_position();
        let cruise_ms = self.settings.read().unwrap().autopilot.cruise_speed_ms;
        let (length, duration) = {
            let v = self.vehicle.lock();
            (v.mission.length_m(boat), v.mission.duration_s(cruise_ms, boat))
        };
        let eta = if waypoints.is_empty() { "—".to_string() } else { units::format_duration(Some(duration)) };
        note(ui, &format!("{} waypoints · {} · ETA {eta}", waypoints.len(), units::format_distance(Some(length))));
        if let Some(a) = area {
            note(ui, &format!("Area {:.0} × {:.0} m ({:.2} ha) at {:.0}°", a.width_m, a.height_m, a.area_m2() / 10_000.0, a.angle_deg));
        }

        egui::Frame::new().fill(theme::FIELD).stroke(egui::Stroke::new(1.0, theme::BORDER)).corner_radius(6).inner_margin(4).show(ui, |ui| {
            ui.set_width(ui.available_width());
            egui::ScrollArea::vertical().id_salt("waypoints").max_height(150.0).min_scrolled_height(110.0).auto_shrink([false, false]).show(ui, |ui| {
                let mut previous: Option<LatLon> = None;
                for (i, w) in waypoints.iter().enumerate() {
                    let leg = previous.map(|p| format!(" · {:.0} m", geo::distance_m(p, w.position()))).unwrap_or_default();
                    let speed = w.speed_ms.map(|s| format!(" · {}", unit.format(s))).unwrap_or_default();
                    let text = RichText::new(format!("{:>3}  {:.5}, {:.5}{leg}{speed}", i + 1, w.lat, w.lon)).font(theme::mono(11.5));
                    if ui.selectable_label(self.canvas.selected == Some(i), text).clicked() {
                        self.canvas.selected = Some(i);
                    }
                    previous = Some(w.position());
                }
            });
        });

        // The selected waypoint: its own speed and a hold.
        if let Some(i) = self.canvas.selected.filter(|i| *i < waypoints.len()) {
            let w = waypoints[i].clone();
            let mut own = w.speed_ms.is_some();
            let mut speed = unit.from_ms(w.speed_ms.unwrap_or(cruise_ms));
            let mut hold = w.hold_s;
            let mut changed = false;
            ui.label(RichText::new(format!("WAYPOINT {}", i + 1)).font(theme::bold(12.0)).color(theme::ACCENT));
            egui::Grid::new("waypoint").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
                ui.label("");
                changed |= ui.checkbox(&mut own, "Its own speed").changed();
                ui.end_row();
                if own {
                    let step = if unit == units::SpeedUnit::Knots { 0.1 } else { 0.05 };
                    changed |= spin(ui, "Speed", &mut speed, unit.from_ms(0.1), unit.from_ms(max_speed), step, &format!(" {}", unit.suffix()), unit.decimals());
                }
                changed |= spin(ui, "Hold", &mut hold, 0.0, 3600.0, 5.0, " s", 0);
            });
            if changed {
                if let Some(wp) = self.vehicle.lock().mission.waypoints.get_mut(i) {
                    wp.speed_ms = own.then(|| unit.to_ms(speed));
                    wp.hold_s = hold;
                }
            }
        }

        match row_buttons(ui, &["Delete point", "Reverse", "Clear"]) {
            Some(0) => {
                self.canvas.delete_selected(&self.vehicle);
            }
            Some(1) => self.vehicle.lock().mission.waypoints.reverse(),
            Some(2) => {
                self.vehicle.lock().mission.clear();
                self.canvas.selected = None;
                self.auto_summary = None;
            }
            _ => {}
        }
        let missions = state_dir().join("missions");
        match row_buttons(ui, &["📂 Open…", "💾 Save…", "⬇ .waypoints"]) {
            Some(0) => self.file_dialog = Some(FileDialog::open(Purpose::OpenMission, "Open mission", missions, &[".json"])),
            Some(1) => {
                let name = clock::local_strftime("mission-%Y%m%d-%H%M.json");
                self.file_dialog = Some(FileDialog::save(Purpose::SaveMission, "Save mission", missions, name, &[".json"]));
            }
            Some(2) => {
                let name = clock::local_strftime("mission-%Y%m%d-%H%M.waypoints");
                self.file_dialog = Some(FileDialog::save(Purpose::ExportWaypoints, "Export for ArduPilot / QGroundControl", missions, name, &[".waypoints", ".txt"]));
            }
            _ => {}
        }
        match row_buttons(ui, &["▶ Start", "⟲ Restart", "Skip ⏭", "Frame"]) {
            Some(0) => self.start_mission(),
            Some(1) => self.restart_mission(),
            Some(2) => self.vehicle.lock().skip_waypoint(1),
            Some(3) => self.frame_mission(),
            _ => {}
        }
    }

    // ------------------------------------------------------------------
    // Chart
    // ------------------------------------------------------------------

    pub fn page_chart(&mut self, ui: &mut Ui) {
        heading(ui, "WATER");
        let style = self.canvas.water_style;
        if let Some(i) = toggle_row(ui, &[("Natural", style == WaterStyle::Natural), ("Depth chart", style == WaterStyle::Chart), ("🛰 Satellite", style == WaterStyle::Satellite)]) {
            self.set_water_style([WaterStyle::Natural, WaterStyle::Chart, WaterStyle::Satellite][i]);
        }
        if style == WaterStyle::Satellite {
            self.satellite_section(ui);
        }
        heading(ui, "DISPLAY");
        let ui_s = self.ui_settings();
        ui.horizontal(|ui| {
            for fps in [60, 30] {
                let on = ui_s.max_fps == fps;
                let (fill, stroke) = if on { (theme::hex(0x12384a), theme::ACCENT) } else { (theme::SURFACE_RAISED, theme::BORDER) };
                if big_button(ui, Vec2::new(80.0, 32.0), &format!("{fps} fps"), fill, stroke, 13.0, on).clicked() {
                    self.set_max_fps(fps);
                }
            }
            let mut show = ui_s.show_fps;
            if ui.checkbox(&mut show, "Show FPS").changed() {
                self.edit_ui(|u| u.show_fps = show);
            }
        });
        let unit = self.speed_unit();
        if let Some(i) = toggle_row(ui, &[("Speed in knots", unit == units::SpeedUnit::Knots), ("Speed in m/s", unit == units::SpeedUnit::MetresPerSecond)]) {
            let wanted = if i == 0 { units::SpeedUnit::Knots } else { units::SpeedUnit::MetresPerSecond };
            self.settings.write().unwrap().map.speed_units = wanted.setting().into();
        }
        heading(ui, "LAYERS");
        let c = &mut self.canvas;
        ui.checkbox(&mut c.show_chart, "Depth chart (EMODnet)");
        ui.checkbox(&mut c.show_isobaths, "Isobaths and spot depths");
        ui.checkbox(&mut c.show_survey, "Surveyed depth (your soundings)");
        ui.checkbox(&mut c.show_osm, "Street map (OpenStreetMap)");
        ui.checkbox(&mut c.show_track, "Track");
        ui.checkbox(&mut c.show_leg_lengths, "Leg lengths");

        heading(ui, "DEPTH ALARMS");
        let mut safety = self.settings.read().unwrap().safety.clone();
        let mut changed = false;
        egui::Grid::new("alarms").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            changed |= spin(ui, "Shallow alarm", &mut safety.shallow_alarm_m, 0.1, 20.0, 0.1, " m", 1);
            changed |= spin(ui, "Auto stop below", &mut safety.shallow_stop_m, 0.0, 20.0, 0.1, " m", 1);
        });
        if changed {
            safety.shallow_stop_m = safety.shallow_stop_m.min(safety.shallow_alarm_m);
            self.settings.write().unwrap().safety = safety;
            self.depth_alarms_changed();
        }

        heading(ui, "SURVEY DATA");
        let surveys = state_dir().join("surveys");
        match row_buttons(ui, &["Export soundings…", "Export grid…"]) {
            Some(0) => {
                let name = clock::local_strftime("soundings-%Y%m%d-%H%M.csv");
                self.file_dialog = Some(FileDialog::save(Purpose::ExportSoundings, "Export soundings", surveys.clone(), name, &[".csv", ".xyz"]));
            }
            Some(1) => {
                let name = clock::local_strftime("depth-grid-%Y%m%d-%H%M.csv");
                self.file_dialog = Some(FileDialog::save(Purpose::ExportGrid, "Export depth grid", surveys.clone(), name, &[".csv", ".xyz"]));
            }
            _ => {}
        }
        match row_buttons(ui, &["Clear survey", "Clear track"]) {
            Some(0) => {
                self.confirm = Some(Confirm { text: "Discard every sounding taken so far?".into(), action: ConfirmAction::ClearSurvey })
            }
            Some(1) => self.vehicle.lock().clear_track(),
            _ => {}
        }
        if row_buttons(ui, &["Load depth chart…"]).is_some() {
            let dir = std::env::current_dir().map(|d| d.join("maps")).unwrap_or_else(|_| state_dir());
            self.file_dialog = Some(FileDialog::open(Purpose::LoadChart, "Load depth chart", dir, &[".npz", ".asc", ".tif", ".tiff", ".xyz", ".csv"]));
        }
        match &self.chart_grid {
            None => note(ui, "No depth chart loaded — the simulator uses an open lake."),
            Some(chart) => note(
                ui,
                &format!(
                    "Chart: {}\n{} × {} cells, ≈{:.0} m each. ~ marks an estimated depth (inland lakes).",
                    chart.source,
                    chart.cols,
                    chart.rows,
                    chart.dlat * 111_320.0
                ),
            ),
        }
        ui.label(self.cursor_text.clone());
    }

    // ------------------------------------------------------------------
    // Depth
    // ------------------------------------------------------------------

    pub fn page_depth(&mut self, ui: &mut Ui) {
        let nav = self.nav.clone();
        let s = self.settings.read().unwrap().clone();
        let t = nav.telemetry.as_ref();
        let here = nav.position();
        let chart_here = match (&self.chart_grid, here) {
            (Some(c), Some(p)) => Some(c.depth_at(p.lat, p.lon)),
            _ => None,
        };
        let now = clock::monotonic();
        if let (Some(here), true) = (here, now - self.profile_at >= 2.0) {
            self.profile_at = now;
            let (route, grid) = {
                let v = self.vehicle.lock();
                let start = nav.waypoint_index.unwrap_or(0);
                let mut route = vec![here];
                route.extend(v.mission.positions().into_iter().skip(start));
                if route.len() < 2 {
                    if let Some(target) = nav.target {
                        route = vec![here, target];
                    }
                }
                (route, v.soundings.clone())
            };
            let chart = self.chart_grid.clone();
            let surveyed = move |lat: f64, lon: f64| grid.depth_of(grid.cell_of(lat, lon));
            // The chart, not the simulator's hidden truth: plan on what is known.
            let charted = move |lat: f64, lon: f64| chart.as_ref().map(|c| c.depth_at(lat, lon)).unwrap_or(f64::NAN);
            self.profile = (route.len() >= 2).then(|| route_profile(&route, charted, Some(&surveyed), 10.0, 600));
        }
        let least = self.profile.as_ref().and_then(|p| p.shallowest());
        let depth = t.and_then(|t| t.depth_m);
        let alarm = s.safety.shallow_alarm_m;
        value_grid(
            ui,
            "depth-values",
            &[
                ("Under keel", depth.map(|d| format!("{d:.2} m")).unwrap_or("—".into()), if depth.is_some_and(|d| d < alarm) { theme::BAD } else { theme::VALUE }),
                ("Chart depth here", match chart_here {
                    None => "—".into(),
                    Some(d) if d.is_nan() => "land".into(),
                    Some(d) => format!("{d:.1} m"),
                }, theme::VALUE),
                ("Water depth", depth.map(|d| format!("{:.2} m", d + s.boat.draft_m)).unwrap_or("—".into()), theme::VALUE),
                ("Least on route", least.map(|l| format!("{:.1} m", l.1)).unwrap_or("—".into()), if least.is_some_and(|l| l.1 < alarm) { theme::BAD } else { theme::VALUE }),
            ],
        );
        let (rect, p) = canvas(ui, 200.0);
        graphs::profile_graph(&p, rect, self.profile.as_ref(), s.boat.draft_m, alarm);
        let samples: Vec<(f64, f64)> = self.sounder.history.iter().filter_map(|&(t, d)| d.map(|d| (t, d))).collect();
        let (rect, p) = canvas(ui, 150.0);
        graphs::sounder_graph(&p, rect, &samples, alarm);
        note(ui, "The profile follows the remaining route — the planned one when no mission is running. Green along the seabed marks where it has been sounded; elsewhere it is the chart.");
    }

    // ------------------------------------------------------------------
    // Power
    // ------------------------------------------------------------------

    pub fn page_power(&mut self, ui: &mut Ui) {
        let nav = self.nav.clone();
        let safety = self.settings.read().unwrap().safety.clone();
        heading(ui, "BATTERY");
        if row_buttons(ui, &["⚡ Power flow diagram (P)"]).is_some() {
            self.power_view = true;
        }
        let t = nav.telemetry.as_ref();
        let e = nav.energy.as_ref();
        let dur = |s: f64| if s.is_finite() { units::format_duration(Some(s)) } else { "∞".into() };
        let v = theme::VALUE;
        let items: Vec<(&str, String, Color32)> = match (t, e) {
            (Some(t), Some(e)) => {
                let reserve = match e.home_reserve_pct {
                    None => ("—".to_string(), v),
                    Some(r) => {
                        let short = t.battery_pct.unwrap_or(0.0) - r < safety.battery_critical_pct;
                        (format!("{r:.1} %"), if short { theme::BAD } else { theme::GOOD })
                    }
                };
                vec![
                    ("Voltage", t.battery_v.map(|x| format!("{x:.2} V")).unwrap_or("—".into()), v),
                    ("Current", t.battery_a.map(|x| format!("{x:.1} A")).unwrap_or("—".into()), v),
                    ("Power", format!("{:.0} W", e.power_w), v),
                    ("Charge", t.battery_pct.map(|x| format!("{x:.0} %")).unwrap_or("—".into()), theme::battery_colour(t.battery_pct)),
                    ("Energy used", format!("{:.1} Wh", e.used_wh), v),
                    ("Energy left", format!("{:.0} Wh", e.remaining_wh), v),
                    ("Time to empty", dur(e.time_to_empty_s), v),
                    ("Range at this pace", if e.range_m.is_finite() { units::format_distance(Some(e.range_m)) } else { "∞".into() }, v),
                    ("Until low", dur(e.time_to_low_s), if e.time_to_low_s < 600.0 { theme::CAUTION } else { v }),
                    ("Until critical", dur(e.time_to_critical_s), if e.time_to_critical_s < 600.0 { theme::BAD } else { v }),
                    ("Consumption", e.wh_per_km.map(|w| format!("{w:.0} Wh/km")).unwrap_or("—".into()), v),
                    ("Reserve to get home", reserve.0, reserve.1),
                    ("Charged from shore", format!("{:.1} Wh", nav.charged_wh), v),
                    (
                        "Shore power",
                        match &t.power {
                            Some(p) if p.shore_w > 1.0 => format!("charging {}", units::format_power(p.shore_w)),
                            Some(p) if p.shore_connected => "connected, full".into(),
                            Some(_) => "not connected".into(),
                            None => "—".into(),
                        },
                        if t.power.as_ref().is_some_and(|p| p.shore_w > 1.0) { theme::GOOD } else { v },
                    ),
                ]
            }
            _ => vec![("Voltage", "—".into(), v), ("Charge", "—".into(), v)],
        };
        value_grid(ui, "power-values", &items);
        let history: Vec<_> = self.vehicle.lock().power_history.iter().copied().collect();
        let (rect, p) = canvas(ui, 190.0);
        graphs::power_graph(&p, rect, &history, e, self.power_window_s, safety.battery_low_pct, safety.battery_critical_pct);
        if let Some(i) = row_buttons(ui, &["10 min", "30 min", "2 h"]) {
            self.power_window_s = [600.0, 1800.0, 7200.0][i];
        }
        note(ui, "The forecast takes the worse of two estimates: the recent draw against the charge left, and how fast the charge has actually fallen. Reserve to get home uses the energy per kilometre measured over the last minute, plus 30 %.");
    }

    // ------------------------------------------------------------------
    // Oil
    // ------------------------------------------------------------------

    /// How the slick moves: current plus 3 % of the wind (speed m/s, towards degrees).
    fn slick_drift(&self) -> Option<(f64, f64)> {
        let (n, e) = self.sim()?.spill_drift();
        Some((n.hypot(e), (e.atan2(n).to_degrees() + 360.0).rem_euclid(360.0)))
    }

    /// A lawnmower over the slick, lanes along its drift, a swath apart.
    pub fn plan_sweep(&mut self) {
        let Some(oil) = self.sim().and_then(|s| s.spill_snapshot()).filter(|o| o.volume.iter().any(|v| *v > 1e-3)) else {
            self.toast("No slick to sweep", 3.0);
            return;
        };
        let positive: Vec<f64> = oil.volume.iter().copied().filter(|v| *v > 0.0).collect();
        let cut = percentile(&positive, 20.0);
        let kept: Vec<LatLon> = (0..oil.volume.len())
            .filter(|&i| oil.volume[i] > cut && !oil.beached[i])
            .map(|i| LatLon::new(oil.lat[i], oil.lon[i]))
            .collect();
        if kept.is_empty() {
            self.toast("No slick to sweep", 3.0);
            return;
        }
        let drift = self.slick_drift();
        let angle = drift.filter(|d| d.0 > 0.01).map(|d| d.1).unwrap_or(90.0);
        let centre = geo::centroid(&kept).unwrap();
        let theta = angle.to_radians();
        let u = (theta.sin(), theta.cos());
        let v = (-u.1, u.0);
        let (mut along, mut across) = (Vec::new(), Vec::new());
        for p in &kept {
            let m = geo::to_metres(centre, *p);
            along.push(m.east * u.0 + m.north * u.1);
            across.push(m.east * v.0 + m.north * v.1);
        }
        let (a0, a1) = (percentile(&along, 3.0), percentile(&along, 97.0));
        let (c0, c1) = (percentile(&across, 3.0), percentile(&across, 97.0));
        let (ma, mc) = ((a0 + a1) / 2.0, (c0 + c1) / 2.0);
        let mid = geo::to_lat_lon(centre, Metres::new(ma * u.0 + mc * v.0, ma * u.1 + mc * v.1));
        let rect = Rectangle::new(mid, a1 - a0 + 10.0, c1 - c0 + 10.0, angle);
        let s = self.settings.read().unwrap().skimmer.clone();
        let skim = s.best_speed_ms * 1.3;
        let drift_ne = self.sim().map(|sim| sim.spill_drift());
        let count = {
            let mut vh = self.vehicle.lock();
            vh.mission.area = Some(rect);
            vh.mission.pattern = PatternSpec { pattern: Pattern::Lawnmower, spacing_m: s.swath_m, margin_m: 1.0, swath_m: s.swath_m, ..PatternSpec::default() };
            vh.mission.regenerate();
            // Transit to the first lane at cruise speed; skim every lane at boom speed.
            for w in vh.mission.waypoints.iter_mut().skip(1) {
                w.speed_ms = Some(skim);
            }
            // The pattern moves with the oil, as a real sweep is planned.
            vh.mission.drift_ms = drift_ne;
            vh.mission.waypoints.len()
        };
        self.canvas.rect_angle_deg = angle;
        self.frame_mission();
        self.toast(format!("Sweep planned: {count} waypoints, skimming at {:.1} kn — deploy the skimmer, then ▶", units::knots(skim)), 4.0);
    }

    pub fn page_oil(&mut self, ui: &mut Ui) {
        let nav = self.nav.clone();
        heading(ui, "OIL SKIMMER");
        let deployed = nav.skimmer_commanded;
        match toggle_row(ui, &[(if deployed { "⛭ Stow skimmer" } else { "⛭ Deploy skimmer" }, deployed), ("▭ Plan sweep on slick", false)]) {
            Some(0) => self.vehicle.lock().set_skimmer(!deployed),
            Some(1) => self.plan_sweep(),
            _ => {}
        }
        let t = nav.telemetry.as_ref();
        let sk = t.map(|t| &t.skimmer);
        let (rect, p) = canvas(ui, 54.0);
        graphs::tank_gauge(&p, rect, sk);
        let v = theme::VALUE;
        let drift = self.slick_drift();
        let mut items: Vec<(&str, String, Color32)> = Vec::new();
        if let Some(s) = sk {
            let state = if s.offloading {
                "OFFLOADING"
            } else if s.recovering {
                "RECOVERING"
            } else if s.deployed {
                "DEPLOYED"
            } else {
                "STOWED"
            };
            let colour = if s.recovering { theme::GOOD } else if s.deployed || s.offloading { theme::ACCENT } else { theme::TEXT_DIM };
            let (code, text) = appearance(s.film_um);
            items.extend([
                ("Skimmer", state.to_string(), colour),
                ("Recovery rate", format!("{:.2} L/min", s.recovery_lpm), v),
                ("Efficiency", format!("{:.0} %", s.efficiency * 100.0), if s.efficiency > 0.6 { theme::GOOD } else { theme::CAUTION }),
                ("Oil recovered", format!("{:.1} L", s.recovered_oil_l), v),
                ("Oil in water", s.oil_ppm.map(|x| format!("{x:.0} ppm")).unwrap_or("—".into()), v),
                ("Film here", s.film_um.map(|x| format!("{x:.1} µm")).unwrap_or("—".into()), v),
                ("Appearance (Bonn)", format!("{code} {}", text.split(" (").next().unwrap_or(text)), v),
            ]);
        }
        items.push(("Slick drift", drift.map(|(s, d)| format!("{s:.2} m/s → {d:03.0}°")).unwrap_or("—".into()), v));
        value_grid(ui, "oil-values", &items);
        let settings = self.settings.read().unwrap().skimmer.clone();
        let (rect, p) = canvas(ui, 110.0);
        graphs::skim_curve(&p, rect, &settings, t.and_then(|t| t.stw_ms).unwrap_or(0.0));

        heading(ui, "THE SPILL");
        if let Some(sim) = self.sim() {
            let totals = sim.spill_totals();
            value_grid(
                ui,
                "spill-values",
                &[
                    ("Released", format!("{:.0} L", totals.released_l), v),
                    ("On the water", format!("{:.1} L", totals.floating_l), v),
                    ("Recovered", format!("{:.1} L", totals.recovered_l), theme::GOOD),
                    ("Evaporated", format!("{:.1} L", totals.evaporated_l), v),
                    ("Ashore", format!("{:.1} L", totals.beached_l), if totals.beached_l > 1.0 { theme::BAD } else { v }),
                    ("Clean-up", format!("{:.0} %", totals.recovered_fraction() * 100.0), theme::GOOD),
                ],
            );
            match row_buttons(ui, &["New spill", "Spill at view", "Clear"]) {
                Some(0) => {
                    sim.release_spill(None, None);
                    self.toast("A new slick is on the water", 3.0);
                }
                Some(1) => {
                    sim.release_spill(Some(self.canvas.centre), None);
                    self.toast("Slick released in the middle of the view", 3.0);
                }
                Some(2) => sim.clear_spill(),
                _ => {}
            }
        }
        note(ui, "Sweep arms funnel the slick into a brush skimmer between the hulls. Booms only hold oil slowly — keep under about 1.5 kn, ideally 0.7 kn — and work up-drift through the thickest (darkest) oil. When the tank is full, bring the boat alongside the GCS with the skimmer stowed and it pumps out.");
    }

    // ------------------------------------------------------------------
    // The pad
    // ------------------------------------------------------------------

    /// Satellite imagery: what is cached, and fetching an area for offline use.
    fn satellite_section(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        let bounds = self.canvas.view_bounds();
        let mid = (bounds.0 + bounds.2) / 2.0;
        let z = crate::ui::chart::satellite::zoom_for(self.canvas.metres_per_pixel(), mid, ctx.pixels_per_point());
        let (z0, z1) = (z.saturating_sub(2).max(10), (z + 2).min(crate::ui::chart::satellite::MAX_Z));
        let wanted = crate::ui::chart::satellite::Satellite::count(bounds, z0..=z1);
        let sat = self.canvas.satellite_mut(&ctx);
        let (done, failed, queued) = sat.progress();
        let (bytes, files) = sat.cache_size_cached();
        let shown = (sat.shown, sat.missing);
        let size = if bytes >= 1_000_000 { format!("{:.0} MB", bytes as f64 / 1e6) } else { format!("{} kB", bytes / 1000) };
        note(ui, &format!("Tiles in view: {} shown, {} loading · cache {files} tiles, {size}", shown.0, shown.1));
        let label = format!("⬇ Save view offline ({wanted} tiles)");
        note(ui, &format!("Saves zoom {z0}–{z1} of what the chart shows now."));
        if row_buttons(ui, &[&label]).is_some() {
            let limit = 6000;
            let n = self.canvas.satellite_mut(&ctx).download(bounds, z0..=z1, limit);
            self.toast(if wanted > limit { format!("Fetching {n} tiles (the first {limit}); zoom in for a smaller area") } else { format!("Fetching {n} new tiles for offline use") }, 3.0);
        }
        if queued > 0 {
            let fraction = (done + failed) as f32 / queued as f32;
            ui.add(egui::ProgressBar::new(fraction.min(1.0)).text(format!("{done} saved, {failed} failed of {queued}")));
        }
        note(ui, "Imagery is cached in ~/.cache/usvgs/tiles and works offline. Source: Esri World Imagery unless map.satellite_url is set; respect the provider's terms.");
    }

    pub fn page_sound(&mut self, ui: &mut Ui) {
        use crate::config::{SoundSettings, SOUND_ALERTS};
        use crate::sound::Alert;
        heading(ui, "VOICE ALERTS");
        let mut sound = self.settings.read().unwrap().sound.clone();
        let before = sound.clone();
        if !self.voice.available() {
            ui.label(RichText::new("No audio output found — alerts are silent.").color(theme::BAD));
        }
        ui.checkbox(&mut sound.enabled, "Speak alerts");
        if let Some(i) = toggle_row(ui, &[("English", sound.language == "en"), ("Русский", sound.language == "ru")]) {
            sound.language = if i == 0 { "en".into() } else { "ru".into() };
        }
        ui.horizontal(|ui| {
            ui.label("Volume");
            let mut v = (sound.volume * 100.0).round();
            if ui.add(egui::Slider::new(&mut v, 0.0..=100.0).suffix(" %")).changed() {
                sound.volume = v / 100.0;
            }
        });

        heading(ui, "RULES");
        ui.checkbox(&mut sound.repeat_only_armed, "Repeat warnings only while armed");
        let mut play: Option<(Alert, bool)> = None;
        egui::Grid::new("sound-rules").num_columns(4).spacing([6.0, 4.0]).striped(true).show(ui, |ui| {
            ui.label(RichText::new("Alert").color(theme::TEXT_DIM).small());
            ui.label(RichText::new("Repeat").color(theme::TEXT_DIM).small());
            ui.label(RichText::new("Tone").color(theme::TEXT_DIM).small());
            ui.label("");
            ui.end_row();
            for (key, name, _, _) in SOUND_ALERTS {
                let Some(alert) = Alert::for_rule(key) else { continue };
                let rule = sound.rules.entry(key.to_string()).or_default();
                ui.checkbox(&mut rule.enabled, name);
                if alert.alarm().is_some() {
                    let text = if rule.repeat_s <= 0.0 { "once".to_string() } else { format!("{:.0} s", rule.repeat_s) };
                    egui::ComboBox::from_id_salt(("repeat", key)).selected_text(text).width(62.0).show_ui(ui, |ui| {
                        for (value, label) in [(0.0, "once"), (5.0, "5 s"), (10.0, "10 s"), (20.0, "20 s"), (30.0, "30 s"), (60.0, "60 s"), (120.0, "2 min")] {
                            ui.selectable_value(&mut rule.repeat_s, value, label);
                        }
                    });
                } else {
                    ui.label(RichText::new("once").color(theme::TEXT_DIM));
                }
                ui.checkbox(&mut rule.tone, "").on_hover_text("Two-tone warning before the words");
                if ui.add_sized([30.0, 26.0], egui::Button::new("▶")).on_hover_text(format!("Hear: {name}")).clicked() {
                    play = Some((alert, rule.tone));
                }
                ui.end_row();
            }
        });
        if row_buttons(ui, &["Reset the rules"]).is_some() {
            sound.rules = SoundSettings::default().rules;
            sound.repeat_only_armed = true;
        }
        if sound != before {
            if sound.volume != before.volume {
                self.voice.set_volume(sound.volume as f32);
            }
            self.settings.write().unwrap().sound = sound.clone();
        }
        if let Some((alert, tone)) = play {
            self.voice.say(alert, &sound.language, tone);
        }
        note(ui, "Running into water shallower than the alarm sounds a two-tone whoop and “Danger, danger” (in Russian “Слишком низко”), as a ground-proximity warning calls terrain; aground adds “Stop”. A warning that lasts is said again at its repeat interval; when several are due, only the most urgent is.");
        note(ui, "Voices: EdgeTX sound packs (GPL-2.0), in sounds/. Fetch again with ./run.sh sounds.");
    }

    pub fn page_limits(&mut self, ui: &mut Ui) {
        use crate::config::LimitSettings;
        let unit = self.speed_unit();
        let mut limits = self.settings.read().unwrap().limits.clone();
        let before = limits.clone();
        heading(ui, "MOTION LIMITS");
        egui::Grid::new("limits").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            let mut speed = unit.from_ms(limits.max_speed_ms);
            let step = if unit == units::SpeedUnit::Knots { 0.1 } else { 0.05 };
            if spin(ui, "Max speed", &mut speed, unit.from_ms(0.2), unit.from_ms(2.6), step, &format!(" {}", unit.suffix()), unit.decimals()) {
                limits.max_speed_ms = unit.to_ms(speed);
            }
            spin(ui, "Acceleration", &mut limits.max_accel_ms2, 0.05, 3.0, 0.05, " m/s²", 2);
            spin(ui, "Deceleration", &mut limits.max_decel_ms2, 0.05, 3.0, 0.05, " m/s²", 2);
            spin(ui, "Rate of turn", &mut limits.max_turn_dps, 2.0, 60.0, 1.0, " °/s", 0);
            ui.label("");
            ui.checkbox(&mut limits.in_manual, "Limit the sticks too");
            ui.end_row();
        });
        let full_throttle_s = crate::sim::boat::TOP_SPEED_MS / limits.max_accel_ms2.max(0.01);
        note(ui, &format!("Full throttle takes {full_throttle_s:.1} s to reach from rest. USV STOP always stops at once."));
        if row_buttons(ui, &["Reset the limits"]).is_some() {
            limits = LimitSettings::default();
        }
        if limits != before {
            self.settings.write().unwrap().limits = limits.clone();
        }

        heading(ui, "NOW");
        let nav = self.nav.clone();
        let t = nav.telemetry.as_ref();
        let sog = t.and_then(|t| t.sog_ms).unwrap_or(0.0);
        let rot = t.and_then(|t| t.rot_dps).unwrap_or(0.0).abs();
        let gauge = |ui: &mut Ui, label: &str, value: f64, limit: f64, text: String| {
            let fraction = (value / limit).clamp(0.0, 1.0) as f32;
            let colour = if value > limit * 1.02 { theme::BAD } else if fraction > 0.85 { theme::CAUTION } else { theme::GOOD };
            ui.horizontal(|ui| {
                ui.label(RichText::new(label).color(theme::TEXT_DIM));
                ui.add(egui::ProgressBar::new(fraction).fill(colour).desired_width(150.0).text(text));
            });
        };
        gauge(ui, "Speed ", sog, limits.max_speed_ms, format!("{} / {}", unit.format(sog), unit.format(limits.max_speed_ms)));
        gauge(ui, "Turn   ", rot, limits.max_turn_dps, format!("{rot:.0} / {:.0} °/s", limits.max_turn_dps));
        match nav.limit {
            Some(l) => ui.label(RichText::new(format!("Holding the boat back: {}", l.label().to_lowercase())).color(theme::CAUTION)),
            None => ui.label(RichText::new("No limit acting").color(theme::TEXT_DIM)),
        };
        note(ui, "Speed: the throttle is eased off as the boat nears the limit. Acceleration: the throttle may only rise (and fall) so fast. Rate of turn: steering fades as the turn nears the limit, with counter-steer past it. The autopilot's cruise and waypoint speeds are capped at the speed limit.");
    }

    pub fn page_pad(&mut self, ui: &mut Ui) {
        heading(ui, "STEAM DECK — LIVE");
        let (rect, p) = canvas(ui, 170.0);
        graphs::deck_diagram(&p, rect, &self.pad_state, &self.keys, Some(&self.last_helm));
        let mut control = self.settings.read().unwrap().control.clone();
        let mut changed = false;
        if let Some(i) = toggle_row(ui, &[("Arcade sticks", control.stick_mode == StickMode::Arcade), ("Tank sticks", control.stick_mode == StickMode::Tank)]) {
            control.stick_mode = if i == 0 { StickMode::Arcade } else { StickMode::Tank };
            changed = true;
        }
        egui::Grid::new("pad-shape").num_columns(2).spacing([8.0, 8.0]).show(ui, |ui| {
            ui.label("Dead zone");
            let mut dz = (control.deadzone * 100.0).round();
            if ui.add(egui::Slider::new(&mut dz, 0.0..=30.0).suffix(" %")).changed() {
                control.deadzone = dz / 100.0;
                changed = true;
            }
            ui.end_row();
            ui.label("Expo");
            let mut ex = (control.expo * 100.0).round();
            if ui.add(egui::Slider::new(&mut ex, 0.0..=90.0).suffix(" %")).changed() {
                control.expo = ex / 100.0;
                changed = true;
            }
            ui.end_row();
        });
        if changed {
            self.settings.write().unwrap().control = control;
        }
        heading(ui, "BUTTONS");
        for (k, v) in BUTTON_HELP {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(k).strong().color(theme::VALUE));
                ui.label(v);
            });
        }
    }

    // ------------------------------------------------------------------
    // The simulator
    // ------------------------------------------------------------------

    pub fn page_sim(&mut self, ui: &mut Ui) {
        if !self.link.simulated() {
            ui.label("Connected to a real boat — no simulator controls.");
            return;
        }
        heading(ui, "WEATHER");
        let mut sim = self.settings.read().unwrap().sim.clone();
        let mut weather = false;
        let mut other = false;
        egui::Grid::new("weather").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            weather |= spin(ui, "Wind", &mut sim.wind_speed_ms, 0.0, 20.0, 0.5, " m/s", 1);
            weather |= spin(ui, "Wind direction", &mut sim.wind_from_deg, 0.0, 359.0, 10.0, "° from", 0);
            weather |= spin(ui, "Current", &mut sim.current_speed_ms, 0.0, 2.0, 0.05, " m/s", 2);
            weather |= spin(ui, "Current direction", &mut sim.current_to_deg, 0.0, 359.0, 10.0, "° to", 0);
        });
        heading(ui, "GAME");
        egui::Grid::new("game").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            ui.label("Time");
            egui::ComboBox::from_id_salt("time-scale").selected_text(format!("×{}", sim.time_scale)).show_ui(ui, |ui| {
                for scale in [1.0, 2.0, 4.0, 8.0] {
                    other |= ui.selectable_value(&mut sim.time_scale, scale, format!("×{scale}")).changed();
                }
            });
            ui.end_row();
            other |= spin(ui, "Wi-Fi range", &mut sim.wifi_range_m, 50.0, 5_000.0, 50.0, " m", 0);
            other |= spin(ui, "5G reach", &mut sim.cellular_range_m, 100.0, 30_000.0, 250.0, " m", 0);
            other |= spin(ui, "LoRa range", &mut sim.link_range_m, 100.0, 30_000.0, 250.0, " m", 0);
            other |= spin(ui, "GNSS noise", &mut sim.gps_noise_m, 0.0, 10.0, 0.1, " m", 1);
        });
        if weather || other {
            self.settings.write().unwrap().sim = sim.clone();
            if weather {
                if let Some(s) = self.sim() {
                    s.apply_environment();
                }
            }
        }
        match row_buttons(ui, &["Respawn at launch", "Launch here"]) {
            Some(0) => {
                self.vehicle.lock().disarm();
                if let Some(s) = self.sim() {
                    s.respawn(None, None);
                }
                self.vehicle.lock().clear_track();
                self.toast("Boat back at the launch point, battery full", 3.0);
            }
            Some(1) => self.launch_here(),
            _ => {}
        }
        let paused = self.sim().is_some_and(|s| s.paused());
        if let Some(i) = toggle_row(ui, &[("Recharge battery", false), ("Pause", paused)]) {
            if let Some(s) = self.sim() {
                if i == 0 {
                    s.recharge();
                } else {
                    s.set_paused(!paused);
                }
            }
        }
        note(ui, "“Launch here” moves the launch point (and the ground station) to the middle of the chart view. Survey a rectangle and watch the real seabed appear under the track.");
    }

    fn launch_here(&mut self) {
        let target = self.canvas.centre;
        let draft = self.settings.read().unwrap().boat.draft_m;
        let Some(sim) = self.sim() else { return };
        if !(sim.depth_at(target.lat, target.lon) > draft + 0.3) {
            self.toast("That is not deep enough to launch — choose open water", 3.0);
            return;
        }
        {
            let mut s = self.settings.write().unwrap();
            s.sim.start_lat = target.lat;
            s.sim.start_lon = target.lon;
        }
        sim.set_station(target);
        self.vehicle.lock().disarm();
        sim.respawn(Some(target), None);
        // Let the boat report from its new place before home moves there.
        sim.deliver(sim.telemetry());
        let mut v = self.vehicle.lock();
        v.clear_track();
        v.set_home_here();
        drop(v);
        self.toast("Launched here; home and the ground station moved too", 3.0);
    }

    // ------------------------------------------------------------------
    // Log and help
    // ------------------------------------------------------------------

    pub fn page_log(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().id_salt("log").stick_to_bottom(true).auto_shrink([false, false]).show(ui, |ui| {
            for (t, message, level) in &self.log {
                ui.label(RichText::new(format!("{}  {message}", clock::local_hms(*t))).color(theme::severity_colour(*level)));
            }
        });
    }

    pub fn page_link(&mut self, ui: &mut Ui) {
        use crate::link::flows::{format_bytes, format_rate, FlowState};
        let flows = self.flows.clone();
        let nav = self.nav.clone();
        heading(ui, "DATA PATHS");
        if flows.is_empty() {
            note(ui, "This link does not report its paths.");
            return;
        }
        let active = flows.iter().find(|f| f.state == FlowState::Active);
        let rx: f64 = flows.iter().map(|f| f.rx_bps).sum();
        let tx: f64 = flows.iter().map(|f| f.tx_bps).sum();
        let summary = match active {
            Some(f) => format!("Telemetry by {} {} at {:.0} Hz", f.bearer.icon(), f.bearer.label(), nav.downlink_hz),
            None => "No path is working".to_string(),
        };
        ui.label(RichText::new(summary).font(theme::bold(15.0)).color(if active.is_some() { theme::GOOD } else { theme::BAD }));
        note(ui, &format!("In {} · out {} · {} paths", format_rate(rx), format_rate(tx), flows.len()));
        ui.add_space(4.0);
        let now = clock::monotonic();
        for (i, f) in flows.iter().enumerate() {
            let colour = graphs::bearer_colour(f.bearer);
            let state_colour = match f.state {
                FlowState::Active => theme::GOOD,
                FlowState::Standby => theme::ACCENT,
                FlowState::Down => theme::BAD,
                FlowState::Off => theme::TEXT_DIM,
            };
            egui::Frame::new()
                .fill(theme::SURFACE_RAISED)
                .stroke(egui::Stroke::new(if f.state == FlowState::Active { 2.0 } else { 1.0 }, if f.state == FlowState::Active { colour } else { theme::BORDER }))
                .corner_radius(8)
                .inner_margin(egui::Margin::symmetric(8, 6))
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(format!("{} {}", f.bearer.icon(), f.bearer.label())).font(theme::bold(15.0)).color(colour));
                        ui.label(RichText::new(f.state.label()).font(theme::bold(11.5)).color(state_colour));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let mut on = f.state != FlowState::Off;
                            let r = ui.checkbox(&mut on, "Use");
                            r.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Checkbox, true, on, format!("Use {}", f.name)));
                            if r.changed() {
                                self.link.set_flow_enabled(i, on);
                            }
                        });
                    });
                    let dim = |x: Option<f64>, unit: &str, d: usize| x.map(|v| format!("{v:.d$}{unit}")).unwrap_or("—".into());
                    let loss_colour = if f.loss_pct > 20.0 { theme::BAD } else if f.loss_pct > 5.0 { theme::CAUTION } else { theme::VALUE };
                    let w = (ui.available_width() - 48.0) / 4.0;
                    egui::Grid::new(("flow", i)).num_columns(4).spacing([10.0, 0.0]).min_col_width(w).show(ui, |ui| {
                        for label in ["In", "Out", "Loss", "Latency"] {
                            ui.label(RichText::new(label).color(theme::TEXT_DIM).font(theme::font(11.0)));
                        }
                        ui.end_row();
                        let big = |ui: &mut Ui, text: String, colour: Color32| {
                            ui.label(RichText::new(text).font(theme::mono_bold(13.0)).color(colour));
                        };
                        big(ui, format_rate(f.rx_bps), theme::VALUE);
                        big(ui, format_rate(f.tx_bps), theme::VALUE);
                        big(ui, format!("{:.1} %", f.loss_pct), loss_colour);
                        big(ui, dim(f.latency_ms, " ms", 0), theme::VALUE);
                        ui.end_row();
                    });
                    // How full the path is.
                    if let Some(cap) = f.capacity_bps {
                        let used = ((f.rx_bps + f.tx_bps) / cap).clamp(0.0, 1.0) as f32;
                        ui.add(
                            egui::ProgressBar::new(used.max(0.002))
                                .desired_height(9.0)
                                .fill(if used > 0.8 { theme::BAD } else { colour })
                                .text(RichText::new(format!("{:.1} % of {}", used * 100.0, format_rate(cap))).font(theme::font(9.5))),
                        );
                    }
                    let quiet = f.silent_s.map(|s| if s < 1.0 { "now".to_string() } else { format!("{s:.0} s ago") }).unwrap_or("never".into());
                    note(
                        ui,
                        &format!(
                            "{} · signal {} · SNR {} · {} in, {} lost, {} out ({} / {}) · heard {quiet}",
                            f.name,
                            dim(f.rssi_dbm, " dBm", 0),
                            dim(f.snr_db, " dB", 1),
                            f.rx_packets,
                            f.lost_packets,
                            f.tx_packets,
                            format_bytes(f.rx_bytes),
                            format_bytes(f.tx_bytes)
                        ),
                    );
                });
            ui.add_space(4.0);
        }
        let (rect, p) = canvas(ui, 120.0);
        graphs::flow_graph(&p, rect, &flows, graphs::FlowMetric::Rate, now);
        ui.add_space(4.0);
        let (rect, p) = canvas(ui, 100.0);
        graphs::flow_graph(&p, rect, &flows, graphs::FlowMetric::Loss, now);
        note(
            ui,
            "The best working path carries the telemetry and the commands — Wi-Fi, then 5G, then the radios. The others carry a keep-alive a second, so their loss is known and they can take over at once. Untick Use to take a path out (in the simulator, to try a failure).",
        );
    }

    pub fn page_info(&mut self, ui: &mut Ui) {
        let rows = |ui: &mut Ui, id: &str, items: &[(String, String)]| {
            egui::Frame::new()
                .fill(theme::SURFACE)
                .stroke(egui::Stroke::new(1.0, theme::BORDER))
                .corner_radius(8)
                .inner_margin(egui::Margin::symmetric(8, 6))
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    egui::Grid::new(id).num_columns(2).spacing([10.0, 4.0]).striped(true).show(ui, |ui| {
                        for (k, v) in items {
                            ui.label(RichText::new(k).color(theme::TEXT_DIM));
                            ui.label(RichText::new(v).font(theme::mono_bold(13.0)).color(theme::TEXT));
                            ui.end_row();
                        }
                    });
                });
        };
        let nav = self.nav.clone();
        let s = self.settings.read().unwrap().clone();
        heading(ui, "CONNECTION");
        let age = nav.telemetry.as_ref().map(|t| format!("{:.1} s", t.age(clock::monotonic()))).unwrap_or("—".into());
        let kind = if self.link.simulated() {
            "Simulator".to_string()
        } else if self.link.name() == "NO BOAT" {
            "None chosen".to_string()
        } else {
            format!("MAVLink · {}", crate::link::mavlink::describe_paths(&s.link.mavlink_url))
        };
        let mut connection = vec![
            ("Link".to_string(), self.link.name().to_string()),
            ("Kind".to_string(), kind),
            ("Connected for".to_string(), units::format_duration(Some(clock::monotonic() - self.connected_at))),
            ("State".to_string(), if nav.link_ok { "linked".into() } else { "NO LINK".into() }),
            ("Last report".to_string(), age),
            ("Boat → station".to_string(), format!("{:.0} Hz", nav.downlink_hz)),
            ("Station → boat".to_string(), nav.telemetry.as_ref().and_then(|t| t.uplink_hz).map(|h| format!("{h:.0} Hz")).unwrap_or("—".into())),
            ("Distance".to_string(), units::format_distance(nav.station_distance_m)),
        ];
        if !self.link.simulated() {
            connection.push(("Connection".to_string(), s.link.mavlink_url.clone()));
        }
        rows(ui, "info-connection", &connection);
        match row_buttons(ui, &["🔌 Change connection", "📡 Data paths"]) {
            Some(0) => self.open_connect(),
            Some(_) => self.page = crate::ui::app::PAGES.iter().position(|p| *p == "📡 Link").unwrap_or(0),
            None => {}
        }
        heading(ui, "BOAT");
        let boat = self.link.info();
        if boat.is_empty() {
            note(ui, "Nothing known yet.");
        } else {
            rows(ui, "info-boat", &boat);
        }
        heading(ui, "GROUND STATION");
        let gcs = vec![
            ("Software".to_string(), format!("usvgs {}", env!("CARGO_PKG_VERSION"))),
            ("Up for".to_string(), units::format_duration(Some(clock::monotonic() - self.started_at()))),
            ("Controller".to_string(), if self.pad_state.connected { self.pad_state.name.clone() } else { "keyboard".into() }),
            ("Steam Deck battery".to_string(), self.gcs_battery().map(|b| format!("{b:.0} %")).unwrap_or("—".into())),
            ("Settings".to_string(), crate::config::default_path().display().to_string()),
            ("Logs and surveys".to_string(), state_dir().display().to_string()),
        ];
        rows(ui, "info-gcs", &gcs);
    }

    pub fn page_help(&mut self, ui: &mut Ui) {
        let h = |ui: &mut Ui, t: &str| {
            ui.add_space(6.0);
            ui.label(RichText::new(t).font(theme::bold(15.0)).color(theme::ACCENT));
        };
        h(ui, "Steam Deck controls");
        ui.label("Arcade sticks: left stick or R2/L2 throttle, right stick steer. Tank sticks: left = port motor, right = starboard motor.");
        egui::Grid::new("help-buttons").num_columns(2).spacing([8.0, 3.0]).show(ui, |ui| {
            for (k, v) in BUTTON_HELP {
                ui.label(RichText::new(k).strong());
                ui.label(v);
                ui.end_row();
            }
        });
        h(ui, "Keyboard");
        ui.label("W/S or ↑/↓ throttle · A/D or ←/→ steer · Shift boost · Space USV stop · Enter arm/disarm · 1–7 modes · +/− zoom · F follow · V view (N↑ / H↑ / 3D) · G vectors · N natural water / depth chart · K skimmer · Tab right panel · T telemetry panel · F11 full screen");
        h(ui, "Chart");
        ui.label("Pinch with two fingers, or hold the big ＋/－ buttons, to zoom. The ◀ ▶ tabs on the chart's edges open and close the side panels. Vectors at the boat: orange thrust (and an arc for the turning moment), cyan true wind, violet dashed current, green course over ground for 30 s, yellow dashed heading.");
        h(ui, "Planning");
        ui.label("Points: tap the water to add a waypoint, drag to move, right-click to delete.\nRectangle: drag out an area; drag a corner to resize, the middle to move, the yellow handle to rotate. The lawnmower route follows.\nGo to: tap a point and the boat drives there and holds station.");
        ui.label("Moving a stick hard in any automatic mode takes manual control back.");
    }
}

#[cfg(test)]
mod tests {
    use super::percentile;

    #[test]
    fn percentiles_interpolate_like_numpy() {
        let v = [1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(percentile(&v, 50.0), 3.0);
        assert_eq!(percentile(&v, 25.0), 2.0);
        assert!((percentile(&v, 10.0) - 1.4).abs() < 1e-9);
    }
}
