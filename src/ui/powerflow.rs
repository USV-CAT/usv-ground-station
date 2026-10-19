//! The power system as a diagram: where the energy comes from and where it
//! goes.
//!
//! Shore power and the two battery banks on the left feed a bus that runs
//! down the boat to the equipment load and the two thrusters; dots run along
//! the lines in the direction the power flows, faster for more power.  On the
//! right, the runtime left and the charge over time with its forecast.

use egui::epaint::PathStroke;
use egui::{Align2, Color32, Painter, Pos2, Rect, Shape, Stroke, StrokeKind, Vec2};

use crate::core::energy::{EnergyEstimate, PowerSample};
use crate::core::telemetry::{BankState, PowerState, Telemetry};
use crate::core::units;
use crate::ui::theme;

/// The diagram is laid out in a box this size and scaled to fit.
const DESIGN: Vec2 = Vec2::new(1000.0, 640.0);

struct Canvas<'a> {
    p: &'a Painter,
    origin: Pos2,
    s: f32,
}

impl Canvas<'_> {
    fn at(&self, x: f32, y: f32) -> Pos2 {
        self.origin + Vec2::new(x, y) * self.s
    }

    fn r(&self, x: f32, y: f32, w: f32, h: f32) -> Rect {
        Rect::from_min_max(self.at(x, y), self.at(x + w, y + h))
    }

    fn text(&self, x: f32, y: f32, align: Align2, text: impl ToString, size: f32, colour: Color32) {
        self.p.text(self.at(x, y), align, text, theme::font(size * self.s), colour);
    }

    fn bold(&self, x: f32, y: f32, align: Align2, text: impl ToString, size: f32, colour: Color32) {
        self.p.text(self.at(x, y), align, text, theme::bold(size * self.s), colour);
    }

    fn mono(&self, x: f32, y: f32, align: Align2, text: impl ToString, size: f32, colour: Color32) {
        self.p.text(self.at(x, y), align, text, theme::mono_bold(size * self.s), colour);
    }

    fn card(&self, rect: Rect, title: Option<&str>) {
        self.p.rect(rect, 8.0 * self.s, theme::SURFACE, Stroke::new(1.0, theme::BORDER), StrokeKind::Middle);
        if let Some(t) = title {
            let band = Rect::from_min_size(rect.min, Vec2::new(rect.width(), 26.0 * self.s));
            self.p.rect_filled(band, egui::CornerRadius { nw: (8.0 * self.s) as u8, ne: (8.0 * self.s) as u8, sw: 0, se: 0 }, theme::SURFACE_RAISED);
            self.p.text(band.left_center() + Vec2::new(10.0 * self.s, 0.0), Align2::LEFT_CENTER, t, theme::bold(13.0 * self.s), theme::ACCENT);
        }
    }

    /// A line carrying power: dots run along it, `flow` > 0 forwards.
    fn wire(&self, points: &[(f32, f32)], flow: f64, colour: Color32, now: f64) {
        let pts: Vec<Pos2> = points.iter().map(|&(x, y)| self.at(x, y)).collect();
        let live = flow.abs() > 1.0;
        let base = if live { theme::alpha(colour, 150) } else { theme::BORDER };
        self.p.add(Shape::line(pts.clone(), PathStroke::new(2.0 * self.s, base)));
        if !live {
            return;
        }
        let length: f32 = pts.windows(2).map(|w| (w[1] - w[0]).length()).sum();
        let spacing = 26.0 * self.s;
        // Faster for more power, up to about a hundred points a second.
        let speed = (20.0 + 30.0 * (flow.abs() / 100.0).sqrt()).min(110.0) as f32 * self.s;
        let shift = ((now as f32 * speed) % spacing) * flow.signum() as f32;
        let mut d = shift.rem_euclid(spacing);
        while d < length {
            let mut rest = d;
            for w in pts.windows(2) {
                let seg = (w[1] - w[0]).length();
                if rest <= seg {
                    let p = w[0] + (w[1] - w[0]) * (rest / seg.max(1e-3));
                    self.p.circle_filled(p, 3.0 * self.s, colour);
                    break;
                }
                rest -= seg;
            }
            d += spacing;
        }
    }

    fn connector(&self, x: f32, y: f32, ok: bool, label: &str) {
        let c = self.at(x, y);
        let colour = if ok { theme::GOOD } else { theme::TEXT_DIM };
        self.p.circle(c, 13.0 * self.s, theme::SURFACE_RAISED, Stroke::new(1.5, colour));
        // Two links of a chain.
        for dx in [-4.0f32, 4.0] {
            let r = Rect::from_center_size(c + Vec2::new(dx * self.s, 0.0), Vec2::new(9.0, 6.0) * self.s);
            self.p.rect_stroke(r, 3.0 * self.s, Stroke::new(1.5, colour), StrokeKind::Middle);
        }
        self.text(x, y + 22.0, Align2::CENTER_TOP, label, 9.5, theme::TEXT_DIM);
    }
}

fn battery_icon(c: &Canvas, rect: Rect, soc_pct: f64, charging: bool) {
    let colour = if soc_pct > 40.0 { theme::GOOD } else if soc_pct > 15.0 { theme::CAUTION } else { theme::BAD };
    let s = c.s;
    let cap = Rect::from_center_size(Pos2::new(rect.center().x, rect.top() + 3.0 * s), Vec2::new(rect.width() * 0.4, 6.0 * s));
    c.p.rect_filled(cap, 2.0 * s, colour);
    let body = Rect::from_min_max(rect.min + Vec2::new(0.0, 6.0 * s), rect.max);
    c.p.rect(body, 6.0 * s, Color32::TRANSPARENT, Stroke::new(2.5 * s, colour), StrokeKind::Middle);
    let cells = 5;
    let inner = body.shrink(5.0 * s);
    let h = inner.height() / cells as f32;
    let lit = ((soc_pct / 100.0 * cells as f64).ceil() as usize).min(cells);
    for i in 0..cells {
        let r = Rect::from_min_size(Pos2::new(inner.left(), inner.bottom() - (i + 1) as f32 * h + 1.5 * s), Vec2::new(inner.width(), h - 3.0 * s));
        c.p.rect_filled(r, 2.0 * s, if i < lit { colour } else { theme::alpha(colour, 30) });
    }
    if charging {
        c.p.text(body.center(), Align2::CENTER_CENTER, "⚡", theme::bold(20.0 * s), theme::VALUE);
    }
}

fn bank_card(c: &Canvas, y: f32, bank: &BankState) {
    let rect = c.r(20.0, y, 270.0, 170.0);
    c.card(rect, None);
    battery_icon(c, c.r(34.0, y + 14.0, 48.0, 84.0), bank.soc_pct, bank.amps < -0.05);
    c.bold(98.0, y + 22.0, Align2::LEFT_CENTER, &bank.name, 15.0, theme::TEXT);
    let colour = theme::battery_colour(Some(bank.soc_pct));
    c.mono(98.0, y + 46.0, Align2::LEFT_CENTER, format!("{:.0}% SoC", bank.soc_pct), 17.0, colour);
    c.text(98.0, y + 70.0, Align2::LEFT_CENTER, format!("{:.1} V / {:.0} A max", bank.volts, bank.max_amps), 12.5, theme::TEXT);
    let (state, colour) = if bank.amps < -0.05 {
        (format!("CHARGING {:.1} A", -bank.amps), theme::GOOD)
    } else if bank.amps > 0.05 {
        (format!("DISCHARGING {:.1} A", bank.amps), if bank.amps > 0.8 * bank.max_amps && bank.max_amps > 0.0 { theme::BAD } else { theme::TEXT })
    } else {
        ("IDLE".to_string(), theme::TEXT_DIM)
    };
    c.bold(98.0, y + 92.0, Align2::LEFT_CENTER, state, 12.5, colour);
    let detail = c.r(30.0, y + 112.0, 250.0, 50.0);
    c.p.rect_filled(detail, 5.0 * c.s, theme::SURFACE_RAISED);
    let spread_mv = (bank.cell_max_v - bank.cell_min_v) * 1000.0;
    c.text(38.0, y + 120.0, Align2::LEFT_TOP, "CELL BALANCING:", 9.5, theme::TEXT_DIM);
    c.text(38.0, y + 136.0, Align2::LEFT_TOP, format!("{} (Δ {spread_mv:.0} mV)", if bank.balancing { "ACTIVE" } else { "IDLE" }), 10.5, theme::TEXT);
    c.text(165.0, y + 120.0, Align2::LEFT_TOP, "CELL TEMPS:", 9.5, theme::TEXT_DIM);
    let hot = bank.temp_max_c > 45.0;
    c.text(165.0, y + 136.0, Align2::LEFT_TOP, format!("{:.0} / {:.0} °C", bank.temp_min_c, bank.temp_max_c), 10.5, if hot { theme::BAD } else { theme::TEXT });
}

/// The banks to show: what the boat reports, or the pack as one bank.
fn banks_of(t: &Telemetry) -> Vec<BankState> {
    if let Some(p) = &t.power {
        if !p.banks.is_empty() {
            return p.banks.clone();
        }
    }
    vec![BankState {
        name: "MAIN PACK".into(),
        soc_pct: t.battery_pct.unwrap_or(0.0),
        volts: t.battery_v.unwrap_or(0.0),
        amps: t.battery_a.unwrap_or(0.0),
        max_amps: 0.0,
        ..BankState::default()
    }]
}

#[allow(clippy::too_many_arguments)]
pub fn paint(p: &Painter, rect: Rect, t: Option<&Telemetry>, history: &[PowerSample], estimate: Option<&EnergyEstimate>, window_s: f64, now: f64) {
    p.rect_filled(rect, 0.0, theme::BACKGROUND);
    let s = (rect.width() / DESIGN.x).min(rect.height() / DESIGN.y);
    let origin = rect.center() - DESIGN * s / 2.0;
    let c = Canvas { p, origin, s };
    let Some(t) = t else {
        c.bold(500.0, 320.0, Align2::CENTER_CENTER, "No telemetry from the boat", 20.0, theme::TEXT_DIM);
        return;
    };
    let power = t.power.clone().unwrap_or_else(|| PowerState {
        load_w: t.battery_v.unwrap_or(0.0) * t.battery_a.unwrap_or(0.0),
        ..PowerState::default()
    });
    let banks = banks_of(t);
    let volts = t.battery_v.unwrap_or(0.0);
    let charge = theme::GOOD;
    let drain = theme::CAUTION;

    // Shore power.
    let shore = c.r(20.0, 10.0, 270.0, 78.0);
    c.card(shore, None);
    c.bold(70.0, 30.0, Align2::LEFT_CENTER, "SHORE POWER", 15.0, theme::TEXT);
    c.text(40.0, 49.0, Align2::CENTER_CENTER, "🔌", 26.0, if power.shore_connected { theme::GOOD } else { theme::TEXT_DIM });
    let (text, colour) = if power.shore_w > 1.0 {
        (format!("CHARGING {}", units::format_power(power.shore_w)), theme::GOOD)
    } else if power.shore_connected {
        ("CONNECTED · FULL".to_string(), theme::GOOD)
    } else {
        ("NOT CONNECTED".to_string(), theme::TEXT_DIM)
    };
    c.mono(70.0, 56.0, Align2::LEFT_CENTER, text, 13.0, colour);
    c.text(70.0, 75.0, Align2::LEFT_CENTER, "alongside the GCS, motors still", 10.0, theme::TEXT_DIM);

    // The banks, their connectors, and the bus they feed.
    c.card(c.r(10.0, 100.0, 290.0, 400.0), Some("BATTERY BANKS"));
    for (i, bank) in banks.iter().take(2).enumerate() {
        let y = 134.0 + i as f32 * 182.0;
        bank_card(&c, y, bank);
        let cy = y + 60.0;
        c.wire(&[(290.0, cy), (380.0, cy)], bank.amps * bank.volts, if bank.amps < 0.0 { charge } else { drain }, now);
        c.connector(335.0, cy, bank.volts > 0.0, &format!("CONN {}", if i == 0 { "P" } else { "S" }));
    }
    // The bus: down the left of the boat.
    let bus_x = 380.0;
    c.wire(&[(290.0, 49.0), (bus_x, 49.0), (bus_x, 120.0)], power.shore_w, charge, now);
    let total_w = volts * t.battery_a.unwrap_or(0.0) + power.shore_w;
    c.p.line_segment([c.at(bus_x, 120.0), c.at(bus_x, 560.0)], Stroke::new(3.0 * s, theme::alpha(theme::ACCENT, 160)));
    let status_ok = banks.iter().all(|b| b.temp_max_c < 45.0) && volts > 12.5;
    c.text(520.0, 22.0, Align2::CENTER_CENTER, "POWER BUS STATUS:", 12.0, theme::TEXT);
    c.bold(520.0, 40.0, Align2::CENTER_CENTER, if status_ok { "NOMINAL" } else { "CHECK BATTERIES" }, 13.0, if status_ok { theme::GOOD } else { theme::BAD });
    c.mono(520.0, 60.0, Align2::CENTER_CENTER, format!("{volts:.1} V · {}", units::format_power(total_w)), 12.0, theme::VALUE);

    // The boat from above: two hulls, a bridge deck, the load and the thrusters.
    let hull = Stroke::new(2.0 * s, theme::alpha(theme::TEXT, 170));
    for x in [440.0f32, 600.0] {
        let pts = vec![c.at(x + 30.0, 90.0), c.at(x + 60.0, 190.0), c.at(x + 60.0, 560.0), c.at(x, 560.0), c.at(x, 190.0), c.at(x + 30.0, 90.0)];
        p.add(Shape::line(pts, PathStroke::new(hull.width, hull.color)));
    }
    c.text(470.0, 450.0, Align2::CENTER_CENTER, "PORT", 10.0, theme::TEXT_DIM);
    c.text(470.0, 464.0, Align2::CENTER_CENTER, "HULL", 10.0, theme::TEXT_DIM);
    c.text(630.0, 450.0, Align2::CENTER_CENTER, "STBD", 10.0, theme::TEXT_DIM);
    c.text(630.0, 464.0, Align2::CENTER_CENTER, "HULL", 10.0, theme::TEXT_DIM);
    c.p.rect(c.r(500.0, 180.0, 100.0, 40.0), 4.0 * s, Color32::TRANSPARENT, hull, StrokeKind::Middle);
    let load = c.r(495.0, 270.0, 110.0, 120.0);
    c.p.rect(load, 6.0 * s, theme::SURFACE_RAISED, Stroke::new(1.5, theme::BORDER), StrokeKind::Middle);
    c.bold(550.0, 305.0, Align2::CENTER_CENTER, "EQUIPMENT", 12.0, theme::TEXT);
    c.bold(550.0, 322.0, Align2::CENTER_CENTER, "LOAD", 12.0, theme::TEXT);
    c.mono(550.0, 352.0, Align2::CENTER_CENTER, units::format_power(power.load_w), 15.0, theme::VALUE);
    c.wire(&[(bus_x, 330.0), (495.0, 330.0)], power.load_w, drain, now);

    // Thrusters at the sterns.
    let rpm = [t.rpm_port, t.rpm_stbd];
    let watts = [power.port_w, power.stbd_w];
    for (i, x) in [470.0f32, 630.0].into_iter().enumerate() {
        let centre = c.at(x, 535.0);
        c.p.circle(centre, 14.0 * s, theme::SURFACE_RAISED, Stroke::new(1.5, theme::alpha(theme::TEXT, 170)));
        // The propeller turns with the motor.
        let turn = (now * rpm[i] / 60.0 * 0.2) as f32 * std::f32::consts::TAU;
        for k in 0..3 {
            let a = turn + k as f32 * std::f32::consts::TAU / 3.0;
            c.p.line_segment([centre, centre + Vec2::new(a.cos(), a.sin()) * 11.0 * s], Stroke::new(3.0 * s, if rpm[i].abs() > 50.0 { theme::THRUST } else { theme::TEXT_DIM }));
        }
        let path = if i == 0 { vec![(bus_x, 560.0), (bus_x, 600.0), (x, 600.0), (x, 549.0)] } else { vec![(bus_x, 600.0), (x, 600.0), (x, 549.0)] };
        c.wire(&path, watts[i], drain, now);
    }
    // The thruster cards, and the differential between them.
    for (i, (x, name)) in [(150.0f32, "PORT THRUSTER"), (850.0, "STBD THRUSTER")].into_iter().enumerate() {
        let card = c.r(x - 120.0, 520.0, 240.0, 70.0);
        c.card(card, None);
        c.bold(x, 540.0, Align2::CENTER_CENTER, name, 14.0, theme::TEXT);
        c.mono(x, 566.0, Align2::CENTER_CENTER, format!("{} / {:.0} RPM", units::format_power(watts[i]), rpm[i]), 15.0, theme::VALUE);
    }
    c.wire(&[(270.0, 555.0), (420.0, 555.0)], 0.0, drain, now);
    let diff = c.r(430.0, 610.0, 240.0, 24.0);
    c.p.rect_filled(diff, 4.0 * s, theme::SURFACE_RAISED);
    let d = (t.throttle_port - t.throttle_stbd) / 2.0;
    let mid = diff.center().x;
    let fill = Rect::from_min_max(Pos2::new(mid.min(mid + d as f32 * diff.width() / 2.0), diff.top()), Pos2::new(mid.max(mid + d as f32 * diff.width() / 2.0), diff.bottom()));
    c.p.rect_filled(fill, 4.0 * s, theme::alpha(theme::THRUST, 160));
    c.bold(550.0, 622.0, Align2::CENTER_CENTER, "DIFFERENTIAL THRUSTERS", 11.5, theme::TEXT);

    // Runtime.
    let runtime = c.r(700.0, 10.0, 290.0, 150.0);
    c.card(runtime, Some("ESTIMATED RUNTIME"));
    let text = match estimate {
        Some(e) if e.time_to_empty_s.is_finite() => {
            let total = e.time_to_empty_s.max(0.0) as i64;
            format!("{}h {:02}m", total / 3600, (total % 3600) / 60)
        }
        Some(_) if power.shore_w > 1.0 => "charging".into(),
        Some(_) => "∞".into(),
        None => "—".into(),
    };
    c.mono(845.0, 82.0, Align2::CENTER_CENTER, text, 34.0, theme::TEXT);
    let subtitle = if power.shore_w > 1.0 { "ON SHORE POWER" } else { "REMAINING AT CURRENT DRAW" };
    c.text(845.0, 125.0, Align2::CENTER_CENTER, subtitle, 11.0, theme::TEXT_DIM);
    if let Some(e) = estimate {
        c.text(845.0, 145.0, Align2::CENTER_CENTER, format!("{:.0} Wh left · range {}", e.remaining_wh, if e.range_m.is_finite() { units::format_distance(Some(e.range_m)) } else { "∞".into() }), 10.5, theme::TEXT_DIM);
    }

    // Charge over time, with its forecast.
    let chart = c.r(700.0, 175.0, 290.0, 330.0);
    c.card(chart, Some("DISCHARGE OVER TIME"));
    let plot = c.r(740.0, 215.0, 235.0, 245.0);
    p.rect_filled(plot, 0.0, Color32::from_rgb(4, 12, 20));
    for k in 0..=6 {
        let x = plot.left() + plot.width() * k as f32 / 6.0;
        p.line_segment([Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())], Stroke::new(0.5, theme::BORDER));
    }
    for k in 0..=4 {
        let y = plot.top() + plot.height() * k as f32 / 4.0;
        p.line_segment([Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)], Stroke::new(0.5, theme::BORDER));
        p.text(Pos2::new(plot.left() - 4.0 * s, y), Align2::RIGHT_CENTER, format!("{}%", 100 - k * 25), theme::font(9.0 * s), theme::TEXT_DIM);
    }
    if let Some(last) = history.last() {
        let (t0, t1) = (last.t - window_s * 2.0 / 3.0, last.t + window_s / 3.0);
        let x_of = |tt: f64| plot.left() + ((tt - t0) / (t1 - t0)) as f32 * plot.width();
        let y_of = |pct: f64| plot.bottom() - (pct / 100.0) as f32 * plot.height();
        let clip = p.with_clip_rect(plot);
        // Power drawn, faint behind.
        let peak = history.iter().filter(|h| h.t >= t0).map(|h| (h.volts * h.amps).abs()).fold(1.0, f64::max);
        let mut area: Vec<Pos2> = history.iter().filter(|h| h.t >= t0).map(|h| Pos2::new(x_of(h.t), plot.bottom() - ((h.volts * h.amps).max(0.0) / peak) as f32 * plot.height() * 0.45)).collect();
        if area.len() > 1 {
            area.dedup_by(|a, b| (a.x - b.x).abs() < 1.0);
            clip.add(Shape::line(area, PathStroke::new(1.0, theme::alpha(theme::ACCENT, 120))));
        }
        let soc: Vec<Pos2> = history.iter().filter(|h| h.t >= t0).map(|h| Pos2::new(x_of(h.t), y_of(h.percent))).collect();
        if soc.len() > 1 {
            clip.add(Shape::line(soc, PathStroke::new(2.0 * s, theme::GOOD)));
        }
        let now_x = x_of(last.t);
        clip.line_segment([Pos2::new(now_x, plot.top()), Pos2::new(now_x, plot.bottom())], Stroke::new(1.5, theme::TEXT));
        if let Some(e) = estimate {
            let l = e.projection(last.t, last.percent, t1 - last.t);
            clip.extend(Shape::dashed_line(&[Pos2::new(x_of(l[0].0), y_of(l[0].1)), Pos2::new(x_of(l[1].0), y_of(l[1].1))], Stroke::new(2.0 * s, theme::GOOD), 6.0 * s, 4.0 * s));
        }
        let span = |sec: f64| units::format_duration(Some(sec));
        c.text(740.0, 470.0, Align2::LEFT_CENTER, format!("−{}", span(window_s * 2.0 / 3.0)), 9.5, theme::TEXT_DIM);
        p.text(Pos2::new(now_x, c.at(0.0, 470.0).y), Align2::CENTER_CENTER, "now", theme::font(9.5 * s), theme::TEXT_DIM);
        c.text(975.0, 470.0, Align2::RIGHT_CENTER, format!("+{}", span(window_s / 3.0)), 9.5, theme::TEXT_DIM);
        c.text(845.0, 490.0, Align2::CENTER_CENTER, "charge (solid) · forecast (dashed) · power (faint)", 9.5, theme::TEXT_DIM);
    } else {
        p.text(plot.center(), Align2::CENTER_CENTER, "Collecting…", theme::font(12.0 * s), theme::TEXT_DIM);
    }
}
