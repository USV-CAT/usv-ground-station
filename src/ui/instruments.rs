//! HUD instruments drawn over the chart.
//!
//! The instruments a helmsman expects on a boat: a heading tape with the
//! course and the bearing to the next mark on it, a speed log in knots, an
//! echo sounder with its recent history, the two throttles with the rate of
//! turn, and the wind.  Each paints itself into a rectangle from a
//! [`NavState`].

use std::collections::VecDeque;

use egui::epaint::PathStroke;
use egui::{Align2, Color32, Painter, Pos2, Rect, Shape, Stroke, StrokeKind, Vec2};

use crate::core::events::Level;
use crate::core::geo;
use crate::core::units;
use crate::ui::chart::draw;
use crate::ui::theme;
use crate::vehicle::{Alarm, Mode, NavState};

pub const PANEL_BG: Color32 = Color32::from_rgba_premultiplied(5, 10, 15, 205);
pub const PANEL_EDGE: Color32 = Color32::from_rgba_premultiplied(19, 70, 86, 90);

pub const COMPASS_SIZE: Vec2 = Vec2::new(520.0, 62.0);
pub const SPEED_SIZE: Vec2 = Vec2::new(168.0, 168.0);
pub const HELM_SIZE: Vec2 = Vec2::new(250.0, 168.0);
pub const SOUNDER_SIZE: Vec2 = Vec2::new(300.0, 168.0);
pub const WIND_SIZE: Vec2 = Vec2::new(120.0, 120.0);

pub fn frame(p: &Painter, rect: Rect, title: &str) {
    p.rect(rect.shrink(0.5), 9.0, PANEL_BG, Stroke::new(1.0, PANEL_EDGE), StrokeKind::Middle);
    if !title.is_empty() {
        p.text(rect.min + Vec2::new(10.0, 4.0), Align2::LEFT_TOP, title, theme::bold(10.0), theme::ACCENT);
    }
}

pub fn speed_text(ms: Option<f64>, units_kn: bool) -> String {
    match ms {
        None => "—".into(),
        Some(v) if units_kn => format!("{:.1}", units::knots(v)),
        Some(v) => format!("{v:.2}"),
    }
}

fn arc_points(centre: Pos2, radius: f32, start_deg: f32, sweep_deg: f32) -> Vec<Pos2> {
    // Angles counter-clockwise from three o'clock, y up, as dials are drawn.
    let n = ((sweep_deg.abs() / 4.0).ceil() as usize).max(2);
    (0..=n)
        .map(|i| {
            let a = (start_deg + sweep_deg * i as f32 / n as f32).to_radians();
            Pos2::new(centre.x + radius * a.cos(), centre.y - radius * a.sin())
        })
        .collect()
}

fn arc(p: &Painter, centre: Pos2, radius: f32, start: f32, sweep: f32, stroke: Stroke) {
    p.add(Shape::line(arc_points(centre, radius, start, sweep), PathStroke::new(stroke.width, stroke.color)));
}

/// Heading tape, ±60° about the bow.
pub fn compass_tape(p: &Painter, rect: Rect, nav: &NavState) {
    const SPAN: f64 = 120.0;
    frame(p, rect, "");
    let t = nav.telemetry.as_ref();
    let Some(heading) = t.and_then(|t| t.heading_deg) else {
        p.text(rect.center(), Align2::CENTER_CENTER, "NO HEADING", theme::bold(13.0), theme::TEXT_DIM);
        return;
    };
    let w = rect.width() as f64;
    let px_per_deg = (w - 20.0) / SPAN;
    let x_of = |b: f64| rect.left() + (w / 2.0 + geo::angle_difference(b, heading) * px_per_deg) as f32;
    let start = ((heading - SPAN / 2.0) / 5.0).floor() as i64 * 5;
    for b in (start..start + SPAN as i64 + 10).step_by(5) {
        let x = x_of(b as f64);
        if !(rect.left() + 8.0 < x && x < rect.right() - 8.0) {
            continue;
        }
        let major = b.rem_euclid(30) == 0;
        let colour = theme::alpha(theme::TEXT, if major { 220 } else { 120 });
        let top = rect.bottom() - if major { 22.0 } else { 15.0 };
        p.line_segment([Pos2::new(x, rect.bottom() - 8.0), Pos2::new(x, top)], Stroke::new(if major { 1.4 } else { 1.0 }, colour));
        if major {
            let label = match b.rem_euclid(360) {
                0 => "N".to_string(),
                90 => "E".to_string(),
                180 => "S".to_string(),
                270 => "W".to_string(),
                d => format!("{d:03}"),
            };
            let c = if label.chars().all(char::is_alphabetic) { theme::ACCENT } else { theme::TEXT };
            p.text(Pos2::new(x, rect.bottom() - 33.0), Align2::CENTER_CENTER, label, theme::bold(11.0), c);
        }
    }
    let bug = |bearing: Option<f64>, colour: Color32, label: &str, below: bool| {
        let Some(b) = bearing else { return };
        let x = x_of(b).clamp(rect.left() + 14.0, rect.right() - 14.0);
        let y = if below { rect.bottom() - 6.0 } else { rect.top() + 6.0 };
        let sign = if below { -1.0 } else { 1.0 };
        p.add(Shape::convex_polygon(
            vec![Pos2::new(x, y + sign * 10.0), Pos2::new(x - 6.0, y), Pos2::new(x + 6.0, y)],
            colour,
            Stroke::NONE,
        ));
        p.text(Pos2::new(x, y + sign * 22.0), Align2::CENTER_CENTER, label, theme::bold(8.5), colour);
    };
    let tel = t.unwrap();
    bug(tel.cog_deg.filter(|_| tel.sog_ms.unwrap_or(0.0) > 0.2), theme::GOOD, "COG", true);
    let target = nav.target_label.split(' ').next().filter(|s| !s.is_empty()).unwrap_or("BTW");
    bug(nav.bearing_to_target_deg, theme::TARGET, target, true);
    bug(nav.course_setpoint_deg, theme::CAUTION, "SET", false);
    if nav.target_label != "HOME" {
        bug(nav.home_bearing_deg, theme::HOME, "H", false);
    }
    let cx = rect.center().x;
    let boxed = Rect::from_center_size(Pos2::new(cx, rect.top() + 17.0), Vec2::new(76.0, 26.0));
    p.rect(boxed, 5.0, Color32::from_black_alpha(220), Stroke::new(1.5, theme::VALUE), StrokeKind::Middle);
    p.text(boxed.center(), Align2::CENTER_CENTER, format!("{:03.0}°", heading.rem_euclid(360.0)), theme::mono_bold(18.0), theme::VALUE);
    p.line_segment([Pos2::new(cx, rect.top() + 30.0), Pos2::new(cx, rect.bottom() - 6.0)], Stroke::new(2.0, theme::VALUE));
}

/// Speed over ground on a dial, with speed through water and the setpoint.
pub fn speed_dial(p: &Painter, rect: Rect, nav: &NavState, units_kn: bool, max_speed_ms: f64) {
    const MAX_KN: f64 = 6.0;
    frame(p, rect, "SPEED");
    let t = nav.telemetry.as_ref();
    let centre = rect.center() + Vec2::new(0.0, 8.0);
    let radius = 62.0;
    let (start, sweep) = (225.0f32, 270.0f32);
    let angle_of = |kn: f64| (start - sweep * (kn / MAX_KN).clamp(0.0, 1.0) as f32).to_radians();
    arc(p, centre, radius, start, -sweep, Stroke::new(8.0, theme::BORDER));
    let sog_kn = t.and_then(|t| t.sog_ms).map(units::knots).unwrap_or(0.0);
    if sog_kn > 0.01 {
        arc(p, centre, radius, start, -sweep * (sog_kn / MAX_KN).min(1.0) as f32, Stroke::new(8.0, theme::ACCENT));
    }
    for kn in 0..=MAX_KN as i32 {
        let a = angle_of(kn as f64);
        let (s, c) = a.sin_cos();
        let at = |r: f32| Pos2::new(centre.x + c * r, centre.y - s * r);
        p.line_segment([at(radius - 12.0), at(radius - 5.0)], Stroke::new(1.2, theme::TEXT_DIM));
        p.text(at(radius - 21.0), Align2::CENTER_CENTER, kn.to_string(), theme::font(9.5), theme::TEXT_DIM);
    }
    // The speed limit, a red mark on the rim.
    if units::knots(max_speed_ms) < MAX_KN {
        let a = angle_of(units::knots(max_speed_ms));
        let (s, c) = a.sin_cos();
        let at = |r: f32| Pos2::new(centre.x + c * r, centre.y - s * r);
        p.line_segment([at(radius - 6.0), at(radius + 8.0)], Stroke::new(3.0, theme::BAD));
    }
    if matches!(nav.mode, Mode::SpeedHold | Mode::CourseHold) {
        let a = angle_of(units::knots(nav.speed_setpoint_ms));
        p.circle_filled(Pos2::new(centre.x + a.cos() * (radius + 6.0), centre.y - a.sin() * (radius + 6.0)), 4.0, theme::CAUTION);
    }
    p.text(centre + Vec2::new(0.0, -5.0), Align2::CENTER_CENTER, speed_text(t.and_then(|t| t.sog_ms), units_kn), theme::mono_bold(26.0), theme::VALUE);
    let unit = if units_kn { "kn" } else { "m/s" };
    p.text(centre + Vec2::new(0.0, 15.0), Align2::CENTER_CENTER, format!("SOG {unit}"), theme::font(10.0), theme::TEXT_DIM);
    p.text(
        centre + Vec2::new(0.0, 29.0),
        Align2::CENTER_CENTER,
        format!("STW {}", speed_text(t.and_then(|t| t.stw_ms), units_kn)),
        theme::font(10.0),
        theme::TEXT_DIM,
    );
}

/// Both throttles, the steering they add up to, and the rate of turn.
pub fn helm_gauge(p: &Painter, rect: Rect, nav: &NavState) {
    frame(p, rect, "HELM");
    let t = nav.telemetry.as_ref();
    let (port, stbd) = t.map(|t| (t.throttle_port, t.throttle_stbd)).unwrap_or((0.0, 0.0));
    let bar = |x: f32, actual: f64, command: f64, label: &str| {
        let (top, bottom) = (rect.top() + 26.0, rect.bottom() - 30.0);
        let mid = (top + bottom) / 2.0;
        let half = (bottom - top) / 2.0;
        let x = rect.left() + x;
        p.rect(Rect::from_min_max(Pos2::new(x, top), Pos2::new(x + 22.0, bottom)), 4.0, Color32::from_black_alpha(150), Stroke::new(1.0, theme::BORDER), StrokeKind::Middle);
        let colour = if actual >= 0.0 { theme::GOOD } else { theme::CAUTION };
        let a = actual as f32;
        let fill = Rect::from_min_max(Pos2::new(x + 3.0, mid - a.max(0.0) * half), Pos2::new(x + 19.0, mid - a.max(0.0) * half + a.abs() * half));
        p.rect_filled(fill, 0.0, colour);
        p.line_segment([Pos2::new(x - 3.0, mid), Pos2::new(x + 25.0, mid)], Stroke::new(1.0, theme::TEXT_DIM));
        let cy = mid - command as f32 * half;
        p.line_segment([Pos2::new(x - 4.0, cy), Pos2::new(x + 26.0, cy)], Stroke::new(2.0, theme::VALUE));
        p.text(Pos2::new(x + 11.0, bottom + 8.0), Align2::CENTER_CENTER, label, theme::bold(10.0), theme::TEXT);
        p.text(Pos2::new(x + 11.0, bottom + 20.0), Align2::CENTER_CENTER, format!("{:+.0}%", actual * 100.0), theme::mono_bold(11.0), theme::VALUE);
    };
    bar(14.0, port, nav.command_port, "PORT");
    bar(rect.width() - 36.0, stbd, nav.command_stbd, "STBD");

    // The differential, as a rudder angle would show.
    let centre = Pos2::new(rect.center().x, rect.top() + 92.0);
    let radius = 62.0f32;
    let mut half_disc = arc_points(centre, radius, 0.0, 180.0);
    half_disc.push(centre);
    p.add(Shape::convex_polygon(half_disc.clone(), Color32::from_rgba_unmultiplied(0, 20, 30, 160), Stroke::NONE));
    p.add(Shape::closed_line(half_disc, Stroke::new(1.5, theme::BORDER)));
    let steer = (port - stbd) / 2.0;
    let a = (90.0 - steer * 80.0).to_radians() as f32;
    let tip = Pos2::new(centre.x + a.cos() * (radius - 6.0), centre.y - a.sin() * (radius - 6.0));
    p.line_segment([centre, tip], Stroke::new(4.0, theme::BAD));
    for mark in [-1.0f32, -0.5, 0.0, 0.5, 1.0] {
        let a = (90.0 - mark * 80.0).to_radians();
        let at = |r: f32| Pos2::new(centre.x + a.cos() * r, centre.y - a.sin() * r);
        p.line_segment([at(radius - 2.0), at(radius + 3.0)], Stroke::new(1.0, theme::TEXT_DIM));
    }
    p.circle_filled(centre, 4.0, theme::TEXT);
    // Rate of turn, in degrees a minute as marine ROT indicators read.
    let rot = t.and_then(|t| t.rot_dps).unwrap_or(0.0) * 60.0;
    p.text(centre + Vec2::new(0.0, 14.0), Align2::CENTER_CENTER, "ROT °/min", theme::font(10.0), theme::TEXT_DIM);
    let text = format!("{}{:.0}{}", if rot < -5.0 { "◀" } else { " " }, rot.abs(), if rot > 5.0 { "▶" } else { " " });
    p.text(centre + Vec2::new(0.0, 31.0), Align2::CENTER_CENTER, text, theme::mono_bold(16.0), theme::VALUE);
    p.text(Pos2::new(centre.x, centre.y - radius + 4.0), Align2::CENTER_CENTER, "◀ PORT  STBD ▶", theme::font(10.0), theme::TEXT_DIM);
}

/// Depth under the keel and the seabed profile of the last two minutes.
pub struct EchoSounder {
    pub history: VecDeque<(f64, Option<f64>)>,
    pub shallow_alarm_m: f64,
    pub shallow_stop_m: f64,
}

impl Default for EchoSounder {
    fn default() -> Self {
        EchoSounder { history: VecDeque::new(), shallow_alarm_m: 1.2, shallow_stop_m: 0.7 }
    }
}

pub const SOUNDER_HISTORY_S: f64 = 120.0;

impl EchoSounder {
    pub fn record(&mut self, nav: &NavState) {
        let Some(t) = &nav.telemetry else { return };
        let Some(bt) = t.boat_time_s else { return };
        match self.history.back() {
            Some(&(last, _)) if bt < last => self.history.clear(),
            Some(&(last, _)) if bt <= last + 0.18 => return,
            _ => {}
        }
        self.history.push_back((bt, t.depth_m));
        if self.history.len() > 1600 {
            self.history.pop_front();
        }
    }

    pub fn paint(&self, p: &Painter, rect: Rect, nav: &NavState) {
        frame(p, rect, "ECHO SOUNDER");
        let t = nav.telemetry.as_ref();
        let depth = t.and_then(|t| t.depth_m);
        let graph = Rect::from_min_size(rect.min + Vec2::new(10.0, 22.0), Vec2::new(rect.width() - 110.0, rect.height() - 32.0));
        p.rect_filled(graph, 0.0, Color32::from_rgba_unmultiplied(4, 18, 34, 230));
        let now = self.history.back().map(|h| h.0).unwrap_or(0.0);
        let samples: Vec<(f64, f64)> =
            self.history.iter().filter_map(|&(ts, d)| d.map(|d| (ts, d))).filter(|(ts, _)| now - ts <= SOUNDER_HISTORY_S).collect();
        if !samples.is_empty() {
            let top_depth = (samples.iter().map(|s| s.1).fold(0.0, f64::max) * 1.25).max(4.0);
            let scale = graph.height() as f64 / top_depth;
            let point = |ts: f64, d: f64| {
                Pos2::new(graph.right() - ((now - ts) / SOUNDER_HISTORY_S) as f32 * graph.width(), graph.top() + (d * scale) as f32)
            };
            let bed: Vec<Pos2> = samples.iter().map(|&(ts, d)| point(ts, d)).collect();
            if bed.len() > 1 {
                seabed(p, graph, &bed, [Color32::from_rgba_unmultiplied(255, 190, 60, 230), Color32::from_rgba_unmultiplied(120, 60, 20, 230)]);
                p.add(Shape::line(bed, PathStroke::new(1.5, Color32::from_rgb(255, 220, 120))));
            }
            for (level, colour) in [(self.shallow_alarm_m, theme::CAUTION), (self.shallow_stop_m, theme::BAD)] {
                let y = graph.top() + (level * scale) as f32;
                if y < graph.bottom() {
                    p.extend(Shape::dashed_line(&[Pos2::new(graph.left(), y), Pos2::new(graph.right(), y)], Stroke::new(1.0, theme::alpha(colour, 200)), 5.0, 4.0));
                }
            }
            p.text(Pos2::new(graph.left() + 2.0, graph.bottom() - 2.0), Align2::LEFT_BOTTOM, format!("{top_depth:.0} m"), theme::font(9.0), theme::TEXT_DIM);
        }
        let colour = match depth {
            None => theme::TEXT_DIM,
            Some(d) if d < self.shallow_alarm_m => theme::BAD,
            Some(_) => theme::VALUE,
        };
        let text = match depth {
            None => "—".to_string(),
            Some(d) if d < 100.0 => format!("{d:.1}"),
            Some(d) => format!("{d:.0}"),
        };
        let x = graph.right() + 52.0;
        p.text(Pos2::new(x, rect.top() + 60.0), Align2::CENTER_CENTER, text, theme::mono_bold(32.0), colour);
        p.text(Pos2::new(x, rect.top() + 87.0), Align2::CENTER_CENTER, "m below keel", theme::font(10.5), theme::TEXT_DIM);
        if let Some(wt) = t.and_then(|t| t.water.temperature_c) {
            p.text(Pos2::new(x, rect.top() + 111.0), Align2::CENTER_CENTER, format!("water {wt:.1} °C"), theme::font(10.5), theme::TEXT_DIM);
        }
        if nav.soundings > 0 {
            p.text(Pos2::new(x, rect.top() + 129.0), Align2::CENTER_CENTER, format!("{} pings", nav.soundings), theme::font(10.5), theme::TEXT_DIM);
        }
    }
}

/// The seabed under a profile line: a gradient-filled band down to the graph's bottom.
pub fn seabed(p: &Painter, graph: Rect, bed: &[Pos2], colours: [Color32; 2]) {
    use egui::epaint::Mesh;
    let mut mesh = Mesh::default();
    let shade = |y: f32| {
        let t = ((y - graph.top()) / graph.height()).clamp(0.0, 1.0);
        let [a, b] = colours;
        let mix = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t) as u8;
        Color32::from_rgba_premultiplied(mix(a.r(), b.r()), mix(a.g(), b.g()), mix(a.b(), b.b()), mix(a.a(), b.a()))
    };
    for w in bed.windows(2) {
        let base = mesh.vertices.len() as u32;
        mesh.colored_vertex(w[0], shade(w[0].y));
        mesh.colored_vertex(w[1], shade(w[1].y));
        mesh.colored_vertex(Pos2::new(w[1].x, graph.bottom()), colours[1]);
        mesh.colored_vertex(Pos2::new(w[0].x, graph.bottom()), colours[1]);
        mesh.add_triangle(base, base + 1, base + 2);
        mesh.add_triangle(base, base + 2, base + 3);
    }
    p.add(Shape::mesh(mesh));
}

/// Apparent wind relative to the bow.
pub fn wind_indicator(p: &Painter, rect: Rect, nav: &NavState, units_kn: bool) {
    frame(p, rect, "WIND");
    let t = nav.telemetry.as_ref();
    let centre = rect.center() + Vec2::new(0.0, 6.0);
    let radius = 38.0;
    p.circle(centre, radius, Color32::from_black_alpha(120), Stroke::new(1.5, theme::BORDER));
    // Port red, starboard green, as on every wind instrument.
    arc(p, centre, radius, 90.0, 60.0, Stroke::new(4.0, theme::alpha(theme::BAD, 200)));
    arc(p, centre, radius, 30.0, 60.0, Stroke::new(4.0, theme::alpha(theme::GOOD, 200)));
    let boat = Stroke::new(1.5, theme::BOAT);
    p.line_segment([centre + Vec2::new(0.0, -14.0), centre + Vec2::new(-6.0, 10.0)], boat);
    p.line_segment([centre + Vec2::new(0.0, -14.0), centre + Vec2::new(6.0, 10.0)], boat);
    if let Some(awa) = t.and_then(|t| t.wind_apparent_deg) {
        let a = awa.to_radians() as f32;
        let tip = centre + Vec2::new(a.sin(), -a.cos()) * (radius - 2.0);
        let tail = centre + Vec2::new(a.sin(), -a.cos()) * 14.0;
        p.line_segment([tip, tail], Stroke::new(3.0, theme::ACCENT));
        p.circle_filled(tip, 4.0, theme::ACCENT);
        let side = if awa > 0.0 { 'S' } else { 'P' };
        let speed = speed_text(t.and_then(|t| t.wind_apparent_ms), units_kn);
        p.text(Pos2::new(rect.center().x, rect.bottom() - 9.0), Align2::CENTER_CENTER, format!("AWA {:.0}°{side} {speed}", awa.abs()), theme::mono_bold(10.0), theme::VALUE);
    }
}

/// The most urgent alarm, flashing when critical, across the top of the chart.
pub fn alarm_banner(p: &Painter, rect: Rect, alarms: &[Alarm], now: f64) {
    let Some(top) = alarms.first() else { return };
    let mut colour = theme::severity_colour(top.level);
    if top.level >= Level::Critical && ((now / 0.45) as i64) % 2 == 1 {
        colour = theme::darker(colour, 1.6);
    }
    p.rect_filled(rect, 8.0, colour);
    let more = if alarms.len() > 1 { format!("   +{} more", alarms.len() - 1) } else { String::new() };
    p.text(rect.center(), Align2::CENTER_CENTER, format!("⚠  {}{more}", top.text), theme::bold(15.0), Color32::from_rgb(8, 8, 10));
}

/// A dashed line across a graph.
pub fn dashed_h(p: &Painter, rect: Rect, y: f32, stroke: Stroke) {
    p.extend(Shape::dashed_line(&[Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)], stroke, 5.0, 4.0));
}

pub use draw::rounded_rect_points;
