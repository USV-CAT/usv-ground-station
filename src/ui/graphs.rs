//! The drawn parts of the side panel's pages: the Steam Deck with its
//! controls live, the battery forecast, the seabed along the route, the echo
//! sounder's history, the skimmer's tank and its efficiency curve.

use std::collections::BTreeSet;

use egui::epaint::PathStroke;
use egui::{Align2, Color32, Painter, Pos2, Rect, Shape, Stroke, StrokeKind, Vec2};

use crate::config::SkimmerSettings;
use crate::core::energy::{EnergyEstimate, PowerSample};
use crate::core::profile::Profile;
use crate::core::telemetry::SkimmerState;
use crate::core::units;
use crate::input::gamepad::{Button, PadState};
use crate::input::mapping::Helm;
use crate::sim::oil::efficiency_at;
use crate::ui::instruments::{dashed_h, seabed};
use crate::ui::theme;

/// The plot area inside a graph's rectangle.
pub fn plot_rect(rect: Rect) -> Rect {
    Rect::from_min_max(rect.min + Vec2::new(40.0, 16.0), rect.max - Vec2::new(12.0, 18.0))
}

pub fn frame(p: &Painter, rect: Rect, title: &str) -> Rect {
    p.rect_filled(rect, 0.0, theme::SURFACE);
    let plot = plot_rect(rect);
    p.rect(plot, 0.0, Color32::from_rgb(4, 12, 20), Stroke::new(1.0, theme::BORDER), StrokeKind::Inside);
    p.text(rect.min + Vec2::new(6.0, 7.0), Align2::LEFT_CENTER, title, theme::bold(9.5), theme::ACCENT);
    plot
}

fn axis_label(p: &Painter, plot: Rect, y: f32, text: &str, colour: Color32, right: bool) {
    if right {
        p.text(Pos2::new(plot.right() + 2.0, y), Align2::LEFT_CENTER, text, theme::font(9.0), colour);
    } else {
        p.text(Pos2::new(plot.left() - 3.0, y), Align2::RIGHT_CENTER, text, theme::font(9.0), colour);
    }
}

fn line(p: &Painter, points: Vec<Pos2>, stroke: Stroke) {
    if points.len() >= 2 {
        p.add(Shape::line(points, PathStroke::new(stroke.width, stroke.color)));
    }
}

// --------------------------------------------------------------------------
// The controller
// --------------------------------------------------------------------------

/// The Steam Deck, drawn, with its sticks and buttons live.
pub fn deck_diagram(p: &Painter, rect: Rect, pad: &PadState, keys: &BTreeSet<&'static str>, helm: Option<&Helm>) {
    // The Deck is about 2.5 : 1; fit it and work in a 300 × 120 design box.
    let s = (rect.width() / 300.0).min((rect.height() - 10.0) / 128.0);
    let origin = Pos2::new(rect.left() + (rect.width() - 300.0 * s) / 2.0, rect.top() + 12.0);
    let at = |x: f32, y: f32| Pos2::new(origin.x + x * s, origin.y + y * s);
    let r = |x: f32, y: f32, w: f32, h: f32| Rect::from_min_max(at(x, y), at(x + w, y + h));
    let held = |b: Button| pad.held.contains(&b);
    let on = theme::GOOD;
    let idle = theme::hex(0x2a3644);
    let edge = theme::hex(0x52637a);
    let font = |size: f32| theme::font(size * s);
    let bold = |size: f32| theme::bold(size * s);

    p.rect(r(0.0, 14.0, 300.0, 104.0), 42.0 * s, theme::hex(0x161d26), Stroke::new(1.5, edge), StrokeKind::Middle);
    p.rect(r(88.0, 24.0, 124.0, 76.0), 4.0 * s, theme::hex(0x05090d), Stroke::new(1.5, edge), StrokeKind::Middle);
    let name = if pad.name.is_empty() { "no pad" } else { &pad.name };
    let backend = if pad.backend.is_empty() { "keyboard" } else { &pad.backend };
    p.text(at(150.0, 56.0), Align2::CENTER_CENTER, name, font(8.0), theme::TEXT_DIM);
    p.text(at(150.0, 68.0), Align2::CENTER_CENTER, backend, font(8.0), theme::TEXT_DIM);

    for (x, b, label) in [(22.0, Button::LB, "L1"), (232.0, Button::RB, "R1")] {
        p.rect(r(x, 6.0, 46.0, 10.0), 4.0 * s, if held(b) { on } else { idle }, Stroke::new(1.0, edge), StrokeKind::Middle);
        p.text(r(x, 6.0, 46.0, 10.0).center(), Align2::CENTER_CENTER, label, font(7.5), theme::TEXT);
    }
    for (x, value, label) in [(22.0, pad.lt, "L2"), (232.0, pad.rt, "R2")] {
        p.rect(r(x, -6.0, 46.0, 10.0), 4.0 * s, idle, Stroke::new(1.0, edge), StrokeKind::Middle);
        if value > 0.0 {
            p.rect_filled(r(x + 1.0, -5.0, 44.0 * value as f32, 8.0), 3.0 * s, theme::CAUTION);
        }
        p.text(r(x, -6.0, 46.0, 10.0).center(), Align2::CENTER_CENTER, format!("{label} {:.0}", value * 100.0), font(7.5), theme::TEXT);
    }
    let stick = |cx: f32, cy: f32, x: f64, y: f64, pressed: bool| {
        p.circle(at(cx, cy), 15.0 * s, theme::hex(0x0c1218), Stroke::new(1.2, edge));
        let dotted = Stroke::new(0.8, theme::BORDER);
        p.line_segment([at(cx - 15.0, cy), at(cx + 15.0, cy)], dotted);
        p.line_segment([at(cx, cy - 15.0), at(cx, cy + 15.0)], dotted);
        let knob = at(cx + x as f32 * 10.0, cy - y as f32 * 10.0);
        let fill = if x.abs() + y.abs() > 0.05 { theme::ACCENT } else { idle };
        p.circle(knob, 8.0 * s, fill, Stroke::new(1.5, if pressed { on } else { edge }));
        p.text(at(cx, cy + 20.0), Align2::CENTER_CENTER, format!("{:+.1} {:+.1}", x, y), font(7.5), theme::TEXT_DIM);
    };
    stick(60.0, 42.0, pad.lx, pad.ly, held(Button::L3));
    stick(240.0, 42.0, pad.rx, pad.ry, held(Button::R3));
    let (cx, cy) = (32.0, 76.0);
    for (b, dx, dy) in [(Button::Up, 0.0, -1.0), (Button::Down, 0.0, 1.0), (Button::Left, -1.0, 0.0), (Button::Right, 1.0, 0.0)] {
        p.rect(r(cx + dx * 9.0 - 4.5, cy + dy * 9.0 - 4.5, 9.0, 9.0), 0.0, if held(b) { on } else { idle }, Stroke::new(1.0, edge), StrokeKind::Middle);
    }
    for (b, name, dx, dy, colour) in [
        (Button::Y, "Y", 0.0, -1.0, theme::hex(0xe8c33d)),
        (Button::A, "A", 0.0, 1.0, theme::hex(0x5ee07a)),
        (Button::X, "X", -1.0, 0.0, theme::hex(0x4fa8ff)),
        (Button::B, "B", 1.0, 0.0, theme::hex(0xff5a4e)),
    ] {
        let c = at(268.0 + dx * 10.0, 76.0 + dy * 10.0);
        let pressed = held(b);
        p.circle(c, 5.5 * s, if pressed { colour } else { idle }, Stroke::new(1.2, colour));
        p.text(c, Align2::CENTER_CENTER, name, bold(7.0), if pressed { theme::INK } else { colour });
    }
    for (b, x, glyph) in [(Button::View, 76.0, "⧉"), (Button::Menu, 216.0, "☰")] {
        p.circle(at(x, 24.0), 5.0 * s, if held(b) { on } else { idle }, Stroke::new(1.0, edge));
        p.text(at(x, 24.0), Align2::CENTER_CENTER, glyph, font(7.0), theme::TEXT);
    }
    for (x, label) in [(46.0, "—"), (222.0, "mouse")] {
        p.rect(r(x, 70.0, 32.0, 32.0), 6.0 * s, theme::hex(0x10161d), Stroke::new(1.0, edge), StrokeKind::Middle);
        p.text(r(x, 70.0, 32.0, 32.0).center(), Align2::CENTER_CENTER, label, font(7.0), theme::TEXT_DIM);
    }
    // What the sticks add up to: the two motors.
    if let Some(h) = helm {
        for (x, value, label) in [(104.0, h.port, "PORT"), (190.0, h.stbd, "STBD")] {
            p.rect(r(x, 30.0, 6.0, 64.0), 0.0, theme::hex(0x0c1218), Stroke::new(1.0, edge), StrokeKind::Middle);
            let colour = if value >= 0.0 { theme::GOOD } else { theme::CAUTION };
            let v = value as f32;
            p.rect_filled(r(x, 62.0 - v.max(0.0) * 32.0, 6.0, v.abs() * 32.0), 0.0, colour);
            p.text(at(x + 3.0, 98.0), Align2::CENTER_CENTER, label, bold(7.0), theme::TEXT);
        }
    }
    if !keys.is_empty() {
        let text = format!("keys: {}", keys.iter().copied().collect::<Vec<_>>().join(" "));
        p.text(at(150.0, 108.0), Align2::CENTER_CENTER, text, bold(8.0), theme::CAUTION);
    }
}

// --------------------------------------------------------------------------
// Power
// --------------------------------------------------------------------------

/// State of charge and voltage over time, and where the charge is heading.
pub fn power_graph(p: &Painter, rect: Rect, history: &[PowerSample], estimate: Option<&EnergyEstimate>, window_s: f64, low_pct: f64, critical_pct: f64) {
    let plot = frame(p, rect, "CHARGE % (green) · VOLTS (yellow) · FORECAST (dashed)");
    if history.len() < 2 {
        p.text(plot.center(), Align2::CENTER_CENTER, "Collecting…", theme::font(12.0), theme::TEXT_DIM);
        return;
    }
    let last = history[history.len() - 1];
    let now = last.t;
    // Leave a third of the plot for the forecast.
    let (t0, t1) = (now - window_s * 0.67, now + window_s * 0.33);
    let x_of = |t: f64| plot.left() + ((t - t0) / (t1 - t0)) as f32 * plot.width();
    let y_pct = |pct: f64| plot.bottom() - (pct / 100.0) as f32 * plot.height();
    let visible: Vec<&PowerSample> = history.iter().filter(|s| s.t >= t0).collect();
    let volts = visible.iter().map(|s| s.volts);
    let v_low = volts.clone().fold(11.5, f64::min);
    let v_high = volts.fold(16.8, f64::max);
    let y_v = |v: f64| plot.bottom() - ((v - v_low) / (v_high - v_low)) as f32 * plot.height();
    for pct in [0.0, 25.0, 50.0, 75.0, 100.0] {
        dashed_h(p, plot, y_pct(pct), Stroke::new(0.6, theme::BORDER));
        axis_label(p, plot, y_pct(pct), &format!("{pct:.0}%"), theme::TEXT_DIM, false);
    }
    for v in [12.0, 14.0, 16.0] {
        axis_label(p, plot, y_v(v), &format!("{v:.0}V"), theme::VALUE, true);
    }
    for (level, colour) in [(low_pct, theme::CAUTION), (critical_pct, theme::BAD)] {
        dashed_h(p, plot, y_pct(level), Stroke::new(1.0, theme::alpha(colour, 170)));
    }
    p.line_segment([Pos2::new(x_of(now), plot.top()), Pos2::new(x_of(now), plot.bottom())], Stroke::new(1.0, theme::TEXT_DIM));
    let clip = p.with_clip_rect(plot);
    let step = (visible.len() / 300).max(1);
    // Current as faint bars, so the cause of a dip is visible.
    for s in visible.iter().step_by(step) {
        let h = (s.amps / 45.0).min(1.0) as f32 * plot.height() * 0.5;
        clip.rect_filled(
            Rect::from_min_size(Pos2::new(x_of(s.t), plot.bottom() - h), Vec2::new((plot.width() / 300.0).max(1.0), h)),
            0.0,
            theme::alpha(theme::ACCENT, 60),
        );
    }
    let mut charge: Vec<Pos2> = visible.iter().step_by(step).map(|s| Pos2::new(x_of(s.t), y_pct(s.percent))).collect();
    charge.push(Pos2::new(x_of(now), y_pct(last.percent)));
    line(&clip, charge, Stroke::new(2.0, theme::GOOD));
    line(&clip, visible.iter().step_by(step).map(|s| Pos2::new(x_of(s.t), y_v(s.volts))).collect(), Stroke::new(1.4, theme::VALUE));
    if let Some(e) = estimate {
        let l = e.projection(now, last.percent, t1 - now);
        clip.extend(Shape::dashed_line(
            &[Pos2::new(x_of(l[0].0), y_pct(l[0].1)), Pos2::new(x_of(l[1].0), y_pct(l[1].1))],
            Stroke::new(2.0, theme::alpha(theme::GOOD, 200)),
            6.0,
            4.0,
        ));
        for (seconds, colour) in [(e.time_to_low_s, theme::CAUTION), (e.time_to_critical_s, theme::BAD)] {
            if seconds.is_finite() && 0.0 < seconds && seconds < t1 - now {
                let x = x_of(now + seconds);
                clip.line_segment([Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())], Stroke::new(1.5, colour));
            }
        }
    }
    let y = plot.bottom() + 8.0;
    p.text(Pos2::new(plot.left(), y), Align2::LEFT_CENTER, format!("−{}", units::format_duration(Some(window_s * 0.67))), theme::font(9.0), theme::TEXT_DIM);
    p.text(Pos2::new(plot.right(), y), Align2::RIGHT_CENTER, format!("+{}", units::format_duration(Some(window_s * 0.33))), theme::font(9.0), theme::TEXT_DIM);
    p.text(Pos2::new(x_of(now), y), Align2::CENTER_CENTER, "now", theme::font(9.0), theme::TEXT_DIM);
}

// --------------------------------------------------------------------------
// Depth
// --------------------------------------------------------------------------

/// Water depth under the route ahead, down to the seabed.
pub fn profile_graph(p: &Painter, rect: Rect, profile: Option<&Profile>, draft: f64, alarm: f64) {
    let plot = frame(p, rect, "ROUTE AHEAD: WATER DOWN TO THE SEABED");
    let Some(prof) = profile.filter(|pr| pr.distance.len() >= 2) else {
        p.text(plot.center() - Vec2::new(0.0, 8.0), Align2::CENTER_CENTER, "Plan a route, or start one,", theme::font(12.0), theme::TEXT_DIM);
        p.text(plot.center() + Vec2::new(0.0, 8.0), Align2::CENTER_CENTER, "to see the seabed along it", theme::font(12.0), theme::TEXT_DIM);
        return;
    };
    let depths: Vec<f64> = (0..prof.distance.len()).map(|i| prof.depth(i)).collect();
    let deepest = (depths.iter().copied().fold(0.0, f64::max) * 1.15).max(3.0);
    let length = prof.length().max(1.0);
    let x_of = |d: f64| plot.left() + (d / length) as f32 * plot.width();
    let y_of = |z: f64| plot.top() + (z / deepest) as f32 * plot.height();
    // Water above, seabed below, land where the depth is nil.
    let shallow = Color32::from_rgb(38, 130, 190);
    let deep = Color32::from_rgb(8, 22, 44);
    super::chart::draw::vertical_gradient(p, plot, &[(0.0, shallow), (1.0, deep)]);
    let bed: Vec<Pos2> = prof.distance.iter().zip(&depths).map(|(d, z)| Pos2::new(x_of(*d), y_of(*z))).collect();
    seabed(p, plot, &bed, [Color32::from_rgb(196, 160, 96), Color32::from_rgb(92, 66, 34)]);
    line(p, bed.clone(), Stroke::new(1.5, Color32::from_rgb(240, 210, 150)));
    // Surveyed stretches marked along the seabed.
    for i in 0..bed.len() - 1 {
        if prof.surveyed[i].is_some() && prof.surveyed[i + 1].is_some() {
            p.line_segment([bed[i], bed[i + 1]], Stroke::new(3.0, theme::GOOD));
        }
    }
    for (level, colour, text) in [(draft, theme::BAD, "draft"), (alarm, theme::CAUTION, "alarm")] {
        let y = y_of(level);
        dashed_h(p, plot, y, Stroke::new(1.0, theme::alpha(colour, 220)));
        axis_label(p, plot, y, text, colour, true);
    }
    for k in 0..5 {
        let depth = deepest * k as f64 / 4.0;
        axis_label(p, plot, y_of(depth), &format!("{depth:.0}m"), theme::TEXT_DIM, false);
    }
    for d in &prof.waypoints[1..prof.waypoints.len().saturating_sub(1).max(1)] {
        let x = x_of(*d);
        p.extend(Shape::dotted_line(&[Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())], Color32::from_white_alpha(120), 4.0, 0.8));
    }
    // The boat at the start, sitting on the surface.
    p.add(Shape::convex_polygon(
        vec![Pos2::new(plot.left(), plot.top()), Pos2::new(plot.left() + 12.0, plot.top()), Pos2::new(plot.left() + 6.0, plot.top() + 7.0)],
        theme::BOAT,
        Stroke::NONE,
    ));
    let y = plot.bottom() + 8.0;
    p.text(Pos2::new(plot.left(), y), Align2::LEFT_CENTER, "boat", theme::font(9.0), theme::TEXT);
    p.text(Pos2::new(plot.right(), y), Align2::RIGHT_CENTER, units::format_distance(Some(prof.length())), theme::font(9.0), theme::TEXT);
    if let Some((at, least)) = prof.shallowest() {
        let colour = if least < alarm { theme::BAD } else { theme::GOOD };
        p.text(Pos2::new(plot.center().x, y), Align2::CENTER_CENTER, format!("least water {least:.1} m at {}", units::format_distance(Some(at))), theme::font(9.0), colour);
    }
}

/// Five minutes of the echo sounder, at full size.
pub fn sounder_graph(p: &Painter, rect: Rect, samples: &[(f64, f64)], alarm: f64) {
    let plot = frame(p, rect, "ECHO SOUNDER · LAST 5 MIN · DEPTH BELOW KEEL");
    let Some(&(now, _)) = samples.last() else { return };
    let recent: Vec<(f64, f64)> = samples.iter().copied().filter(|(t, _)| now - t <= 300.0).collect();
    if recent.len() < 2 {
        return;
    }
    let deepest = (recent.iter().map(|r| r.1).fold(0.0, f64::max) * 1.2).max(3.0);
    let point = |t: f64, d: f64| Pos2::new(plot.right() - ((now - t) / 300.0) as f32 * plot.width(), plot.top() + (d / deepest) as f32 * plot.height());
    let bed: Vec<Pos2> = recent.iter().map(|&(t, d)| point(t, d)).collect();
    seabed(p, plot, &bed, [Color32::from_rgb(255, 190, 60), Color32::from_rgb(120, 50, 20)]);
    line(p, bed, Stroke::new(1.2, Color32::from_rgb(255, 225, 140)));
    dashed_h(p, plot, plot.top() + (alarm / deepest) as f32 * plot.height(), Stroke::new(1.0, theme::CAUTION));
    for k in 0..4 {
        let depth = deepest * k as f64 / 3.0;
        axis_label(p, plot, plot.top() + (depth / deepest) as f32 * plot.height(), &format!("{depth:.0}m"), theme::TEXT_DIM, false);
    }
    let values: Vec<f64> = recent.iter().map(|r| r.1).collect();
    let (min, max) = (values.iter().copied().fold(f64::INFINITY, f64::min), values.iter().copied().fold(0.0, f64::max));
    let avg = values.iter().sum::<f64>() / values.len() as f64;
    p.text(plot.right_top() + Vec2::new(-4.0, 8.0), Align2::RIGHT_CENTER, format!("min {min:.1}  avg {avg:.1}  max {max:.1} m"), theme::mono_bold(10.0), theme::VALUE);
}

// --------------------------------------------------------------------------
// Oil
// --------------------------------------------------------------------------

pub fn tank_gauge(p: &Painter, rect: Rect, state: Option<&SkimmerState>) {
    let bar = Rect::from_min_max(rect.min + Vec2::new(4.0, 16.0), rect.max - Vec2::new(4.0, 6.0));
    p.text(rect.min + Vec2::new(4.0, 7.0), Align2::LEFT_CENTER, "RECOVERED-FLUID TANK", theme::bold(10.5), theme::TEXT);
    p.rect(bar, 6.0, Color32::from_rgb(4, 10, 16), Stroke::new(1.5, theme::BORDER), StrokeKind::Middle);
    let fraction = state.filter(|s| s.tank_capacity_l > 0.0).map(|s| s.tank_l / s.tank_capacity_l).unwrap_or(0.0) as f32;
    let colour = if fraction > 0.95 { theme::BAD } else if fraction > 0.75 { theme::CAUTION } else { theme::hex(0x8a5a24) };
    if fraction > 0.0 {
        p.rect_filled(Rect::from_min_size(bar.min + Vec2::splat(2.0), Vec2::new((bar.width() - 4.0) * fraction, bar.height() - 4.0)), 4.0, colour);
    }
    let text = match state {
        None => "—".to_string(),
        Some(s) => format!("{:.1} / {:.0} L", s.tank_l, s.tank_capacity_l),
    };
    p.text(bar.center(), Align2::CENTER_CENTER, text, theme::mono_bold(13.0), theme::TEXT);
}

/// Recovery efficiency against speed, with the boat's speed on it.
pub fn skim_curve(p: &Painter, rect: Rect, settings: &SkimmerSettings, speed: f64) {
    let plot = frame(p, rect, "SKIMMING EFFICIENCY vs SPEED THROUGH WATER");
    let top = settings.max_speed_ms * 2.4;
    let x_of = |v: f64| plot.left() + (v / top) as f32 * plot.width();
    let y_of = |e: f64| plot.bottom() - e as f32 * plot.height();
    p.rect_filled(Rect::from_min_max(Pos2::new(x_of(0.0), plot.top()), Pos2::new(x_of(settings.max_speed_ms), plot.bottom())), 0.0, theme::alpha(theme::GOOD, 30));
    line(p, (0..60).map(|i| top * i as f64 / 59.0).map(|v| Pos2::new(x_of(v), y_of(efficiency_at(settings, v)))).collect(), Stroke::new(2.0, theme::GOOD));
    let x = x_of(speed.abs().min(top));
    p.line_segment([Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())], Stroke::new(2.0, theme::BOAT));
    for kn in 0..4 {
        let v = units::from_knots(kn as f64);
        if v <= top {
            p.text(Pos2::new(x_of(v), plot.bottom() + 8.0), Align2::CENTER_CENTER, format!("{kn} kn"), theme::font(9.0), theme::TEXT_DIM);
        }
    }
    axis_label(p, plot, y_of(0.85), "85%", theme::TEXT_DIM, false);
    axis_label(p, plot, y_of(0.0), "0", theme::TEXT_DIM, false);
}

// --------------------------------------------------------------------------
// Pattern icons
// --------------------------------------------------------------------------

/// A small drawing of a route pattern, for the pattern buttons.
pub fn pattern_icon(p: &Painter, rect: Rect, pattern: crate::core::mission::Pattern, colour: Color32) {
    use crate::core::mission::Pattern;
    let r = rect.shrink(rect.width().min(rect.height()) * 0.14);
    let at = |x: f32, y: f32| Pos2::new(r.left() + x * r.width(), r.top() + y * r.height());
    let stroke = Stroke::new(2.0, colour);
    let faint = Stroke::new(1.0, theme::alpha(colour, 90));
    let head = |p: &Painter, from: Pos2, to: Pos2| {
        let d = (to - from).normalized() * 6.0;
        let n = Vec2::new(-d.y, d.x) * 0.6;
        p.add(Shape::convex_polygon(vec![to + d * 0.4, to - d + n, to - d - n], colour, Stroke::NONE));
    };
    let lanes = |p: &Painter, vertical: bool, stroke: Stroke| -> Vec<Pos2> {
        let mut pts = Vec::new();
        for i in 0..4 {
            let o = i as f32 / 3.0;
            let (a, b) = if i % 2 == 0 { (0.0, 1.0) } else { (1.0, 0.0) };
            if vertical {
                pts.push(at(o, 1.0 - a));
                pts.push(at(o, 1.0 - b));
            } else {
                pts.push(at(a, 1.0 - o));
                pts.push(at(b, 1.0 - o));
            }
        }
        line(p, pts.clone(), stroke);
        pts
    };
    let spiral = || -> Vec<Pos2> {
        let (mut x0, mut y0, mut x1, mut y1) = (0.0f32, 0.0f32, 1.0f32, 1.0f32);
        let s = 0.2;
        let mut pts = vec![at(x0, 1.0 - y0)];
        while x1 - x0 > 0.05 && y1 - y0 > 0.05 {
            pts.push(at(x1, 1.0 - y0));
            pts.push(at(x1, 1.0 - y1));
            pts.push(at(x0, 1.0 - y1));
            if y0 + s >= y1 - 0.01 {
                break;
            }
            pts.push(at(x0, 1.0 - (y0 + s)));
            x0 += s;
            y0 += s;
            x1 -= s;
            y1 -= s;
        }
        pts
    };
    match pattern {
        Pattern::Lawnmower => {
            let pts = lanes(p, false, stroke);
            head(p, pts[pts.len() - 2], pts[pts.len() - 1]);
        }
        Pattern::Grid => {
            lanes(p, true, faint);
            let pts = lanes(p, false, stroke);
            head(p, pts[pts.len() - 2], pts[pts.len() - 1]);
        }
        Pattern::SpiralIn => {
            let pts = spiral();
            line(p, pts.clone(), stroke);
            head(p, pts[pts.len() - 2], pts[pts.len() - 1]);
        }
        Pattern::SpiralOut => {
            let mut pts = spiral();
            pts.reverse();
            line(p, pts.clone(), stroke);
            head(p, pts[pts.len() - 2], pts[pts.len() - 1]);
        }
        Pattern::Perimeter => {
            let pts = vec![at(0.0, 1.0), at(1.0, 1.0), at(1.0, 0.0), at(0.0, 0.0), at(0.0, 0.85)];
            line(p, pts.clone(), stroke);
            head(p, pts[3], pts[4]);
        }
        Pattern::Auto => {
            // A faint route, and the spark of the planner over it.
            lanes(p, false, faint);
            let c = r.center();
            let k = r.width().min(r.height()) * 0.42;
            let star: Vec<Pos2> = (0..8)
                .map(|i| {
                    let a = std::f32::consts::TAU * i as f32 / 8.0 - std::f32::consts::FRAC_PI_2;
                    let rr = if i % 2 == 0 { k } else { k * 0.32 };
                    Pos2::new(c.x + rr * a.cos(), c.y + rr * a.sin())
                })
                .collect();
            for i in 0..8 {
                p.add(Shape::convex_polygon(vec![c, star[i], star[(i + 1) % 8]], colour, Stroke::NONE));
            }
            p.text(r.right_top() + Vec2::new(-2.0, -2.0), Align2::RIGHT_TOP, "AI", theme::bold(9.0), colour);
        }
    }
}

// --------------------------------------------------------------------------
// Data paths
// --------------------------------------------------------------------------

/// Each kind of path in its own colour, on every graph.
pub fn bearer_colour(b: crate::link::flows::Bearer) -> Color32 {
    use crate::link::flows::Bearer;
    match b {
        Bearer::WiFi => theme::ACCENT,
        Bearer::Cellular => theme::hex(0xb07cff),
        Bearer::LoRa => theme::CAUTION,
        Bearer::Radio => theme::GOOD,
        Bearer::Satellite => theme::TARGET,
        Bearer::Ip => theme::TEXT,
    }
}

/// What a flow graph shows.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FlowMetric {
    /// Data in, on a log scale from 100 bit/s to 100 Mbit/s.
    Rate,
    /// Packet loss, 0–100 %.
    Loss,
}

/// The paths' history over the last two minutes, one line each.
pub fn flow_graph(p: &Painter, rect: Rect, flows: &[crate::link::flows::FlowStats], metric: FlowMetric, now: f64) {
    use crate::link::flows::HISTORY_S;
    let title = match metric {
        FlowMetric::Rate => "DATA IN · LAST 2 MIN",
        FlowMetric::Loss => "PACKET LOSS · LAST 2 MIN",
    };
    let plot = frame(p, rect, title);
    let y_of = |v: f64| -> f32 {
        let f = match metric {
            FlowMetric::Rate => ((v.max(100.0).log10() - 2.0) / 6.0).clamp(0.0, 1.0),
            FlowMetric::Loss => (v / 100.0).clamp(0.0, 1.0),
        };
        plot.bottom() - f as f32 * plot.height()
    };
    let ticks: &[(f64, &str)] = match metric {
        FlowMetric::Rate => &[(1e2, "100"), (1e3, "1k"), (1e4, "10k"), (1e5, "100k"), (1e6, "1M"), (1e7, "10M")],
        FlowMetric::Loss => &[(0.0, "0%"), (25.0, "25"), (50.0, "50"), (75.0, "75"), (100.0, "100")],
    };
    for (v, label) in ticks {
        let y = y_of(*v);
        p.line_segment([Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)], Stroke::new(1.0, Color32::from_rgb(20, 34, 48)));
        axis_label(p, plot, y, label, theme::TEXT_DIM, false);
    }
    let x_of = |t: f64| plot.right() - ((now - t) / HISTORY_S).clamp(0.0, 1.0) as f32 * plot.width();
    for f in flows {
        let points: Vec<Pos2> = f
            .history
            .iter()
            .map(|s| Pos2::new(x_of(s.t), y_of(if metric == FlowMetric::Rate { s.rx_bps } else { s.loss_pct })))
            .collect();
        let width = if f.state == crate::link::flows::FlowState::Active { 2.2 } else { 1.2 };
        line(p, points, Stroke::new(width, bearer_colour(f.bearer)));
    }
    p.text(Pos2::new(plot.left(), plot.bottom() + 9.0), Align2::LEFT_CENTER, "−2 min", theme::font(9.0), theme::TEXT_DIM);
    p.text(Pos2::new(plot.right(), plot.bottom() + 9.0), Align2::RIGHT_CENTER, "now", theme::font(9.0), theme::TEXT_DIM);
}
