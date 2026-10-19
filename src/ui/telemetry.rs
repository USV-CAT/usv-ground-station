//! The telemetry table down the left of the screen.
//!
//! Label on the left, value on the right, grouped the way a USV operator
//! scans them: position and attitude, propulsion, the link, power, the route,
//! the water, the skimmer, the vectors and the survey.  A value with no
//! reading shows a dash rather than its last number.

use egui::{Color32, RichText, Ui};

use crate::core::units::{self, format_bearing, format_distance, format_duration};
use crate::ui::theme;
use crate::vehicle::NavState;

fn fix_name(fix: u8) -> String {
    match fix {
        0 => "NO FIX".into(),
        2 => "2D".into(),
        3 => "3D".into(),
        4 => "RTK FLT".into(),
        5 => "RTK FIX".into(),
        n => n.to_string(),
    }
}

fn f(value: Option<f64>, digits: usize, suffix: &str) -> String {
    match value {
        Some(v) if v.is_finite() => format!("{v:.digits$}{suffix}"),
        _ => "—".into(),
    }
}

fn signed(value: Option<f64>, digits: usize, suffix: &str) -> String {
    match value {
        Some(v) if v.is_finite() => format!("{v:+.digits$}{suffix}"),
        _ => "—".into(),
    }
}

pub fn speed(ms: Option<f64>, units_kn: bool) -> String {
    match ms {
        None => "—".into(),
        Some(v) if units_kn => format!("{:.1} kn", units::knots(v)),
        Some(v) => format!("{v:.2} m/s"),
    }
}

/// A card: heading then rows of (label, value, colour).
pub fn card(ui: &mut Ui, title: &str, rows: &[(&str, String, Color32)]) {
    egui::Frame::new()
        .fill(theme::SURFACE)
        .stroke(egui::Stroke::new(1.0, theme::BORDER))
        .corner_radius(8)
        .inner_margin(egui::Margin::symmetric(8, 6))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(title).font(theme::bold(11.5)).color(theme::ACCENT));
            egui::Grid::new(title).num_columns(2).spacing([6.0, 1.0]).striped(false).show(ui, |ui| {
                for (label, value, colour) in rows {
                    ui.label(RichText::new(*label).font(theme::font(13.0)).color(theme::TEXT));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new(value).font(theme::mono_bold(13.0)).color(*colour));
                    });
                    ui.end_row();
                }
            });
        });
}

pub fn panel(ui: &mut Ui, nav: &NavState, units_kn: bool) {
    let dim = theme::TEXT_DIM;
    let Some(t) = &nav.telemetry else {
        for title in ["NAVIGATION", "PROPULSION", "LINK", "POWER", "ROUTE", "WATER"] {
            card(ui, title, &[("No telemetry", "—".into(), dim)]);
            ui.add_space(4.0);
        }
        return;
    };
    let stale = !nav.link_ok;
    let c = if stale { dim } else { theme::VALUE };
    let rows_nav = [
        ("GNSS", format!("{} sats · {}", t.satellites, fix_name(t.gps_fix)), if stale { dim } else if t.gps_fix >= 3 { theme::GOOD } else { theme::BAD }),
        ("RTK", match t.gps_fix { 5 => "FIXED", 4 => "FLOAT", _ => "OFF" }.to_string(), c),
        ("Heading", format_bearing(t.heading_deg), c),
        ("COG", format_bearing(t.cog_deg), c),
        ("SOG", speed(t.sog_ms, units_kn), c),
        ("STW", speed(t.stw_ms, units_kn), c),
        ("Rate of turn", signed(t.rot_dps.map(|r| r * 60.0), 0, " °/min"), c),
        ("Roll", signed(t.roll_deg, 1, "°"), c),
        ("Pitch", signed(t.pitch_deg, 1, "°"), c),
    ];
    card(ui, "NAVIGATION", &rows_nav);
    ui.add_space(4.0);
    card(
        ui,
        "PROPULSION",
        &[
            ("Throttle (L)", format!("{:+.0} %", t.throttle_port * 100.0), c),
            ("Throttle (R)", format!("{:+.0} %", t.throttle_stbd * 100.0), c),
            ("RPM (L)", format!("{:.0} rpm", t.rpm_port), c),
            ("RPM (R)", format!("{:.0} rpm", t.rpm_stbd), c),
        ],
    );
    ui.add_space(4.0);
    card(
        ui,
        "LINK",
        &[
            // LoRa carries 2 Hz by design: slow, but not a fault.
            ("USV → GCS", format!("{:.0} Hz", nav.downlink_hz), if nav.downlink_hz >= 5.0 { theme::GOOD } else if nav.downlink_hz >= 1.0 { theme::CAUTION } else { theme::BAD }),
            ("GCS → USV", f(t.uplink_hz, 0, " Hz"), c),
            ("RSSI", f(t.rssi_dbm, 0, " dBm"), c),
            ("GCS – USV", format_distance(nav.station_distance_m), c),
        ],
    );
    ui.add_space(4.0);
    let e = nav.energy.as_ref();
    card(
        ui,
        "POWER",
        &[
            ("Battery", f(t.battery_v, 2, " V"), c),
            ("Current", f(t.battery_a, 1, " A"), c),
            ("Power", e.map(|e| format!("{:.0} W", e.power_w)).unwrap_or("—".into()), c),
            ("Charge", f(t.battery_pct, 0, " %"), if stale { dim } else { theme::battery_colour(t.battery_pct) }),
            ("Time to empty", e.map(|e| format_duration(Some(e.time_to_empty_s))).unwrap_or("—".into()), c),
            ("Energy used", e.map(|e| format!("{:.1} Wh", e.used_wh)).unwrap_or("—".into()), c),
        ],
    );
    ui.add_space(4.0);
    let xte = nav.cross_track_m;
    card(
        ui,
        "ROUTE",
        &[
            ("Target", if nav.target_label.is_empty() { "—".into() } else { nav.target_label.clone() }, if nav.target_label.is_empty() { dim } else { theme::TARGET }),
            ("Distance", format_distance(nav.distance_to_target_m), c),
            ("Bearing", format_bearing(nav.bearing_to_target_deg), c),
            (
                "Cross-track",
                xte.map(|x| format!("{:.1} m {}", x.abs(), if x > 0.0 { '◀' } else { '▶' })).unwrap_or("—".into()),
                if xte.is_none_or(|x| x.abs() < 5.0) { c } else { theme::CAUTION },
            ),
            ("Time to go", format_duration(nav.time_to_go_s), c),
            (
                "Home",
                nav.home_distance_m.map(|d| format!("{} {}", format_distance(Some(d)), format_bearing(nav.home_bearing_deg))).unwrap_or("—".into()),
                c,
            ),
            ("Trip log", format_distance(Some(nav.trip_m)), c),
            ("Run time", format_duration(Some(nav.run_time_s)), c),
        ],
    );
    ui.add_space(4.0);
    let w = &t.water;
    card(
        ui,
        "WATER",
        &[
            ("Depth in water", f(t.depth_m, 2, " m"), if t.depth_m.is_some() && nav.has_alarm("shallow") { theme::BAD } else { c }),
            ("Water temp", f(w.temperature_c, 1, " °C"), c),
            ("pH", f(w.ph, 2, " pH"), c),
            ("DO", f(w.dissolved_oxygen_mgl, 2, " mg/L"), c),
            ("O₂ sat", f(w.oxygen_saturation_pct, 1, " %"), c),
            ("EC", f(w.conductivity_ms, 2, " mS/cm"), c),
            ("TDS", f(w.tds_ppm, 0, " ppm"), c),
            ("Salinity", f(w.salinity_psu, 2, " psu"), c),
            ("ORP", f(w.orp_mv, 0, " mV"), c),
            ("Turbidity", f(w.turbidity_ntu, 1, " NTU"), c),
        ],
    );
    ui.add_space(4.0);
    let sk = &t.skimmer;
    if sk.fitted {
        let state = if sk.offloading {
            "OFFLOAD"
        } else if sk.recovering {
            "SKIMMING"
        } else if sk.deployed {
            "DEPLOYED"
        } else {
            "STOWED"
        };
        let full = sk.tank_l >= sk.tank_capacity_l - 0.05;
        card(
            ui,
            "OIL SKIMMER",
            &[
                ("Skimmer", state.into(), if sk.recovering { theme::GOOD } else { c }),
                ("Tank", format!("{:.1}/{:.0} L", sk.tank_l, sk.tank_capacity_l), if full { theme::BAD } else { c }),
                ("Oil recovered", format!("{:.1} L", sk.recovered_oil_l), c),
                ("Oil in water", f(sk.oil_ppm, 0, " ppm"), c),
            ],
        );
        ui.add_space(4.0);
    }
    card(
        ui,
        "VECTORS",
        &[
            ("True wind", nav.true_wind.map(|(s, d)| format!("{s:.1} m/s {}", format_bearing(Some(d)))).unwrap_or("—".into()), c),
            ("Current", nav.current.map(|(s, d)| format!("{s:.2} → {}", format_bearing(Some(d)))).unwrap_or("—".into()), c),
        ],
    );
    ui.add_space(4.0);
    card(
        ui,
        "SURVEY",
        &[
            ("Soundings", nav.soundings.to_string(), c),
            ("Area covered", if nav.coverage > 0.0 { format!("{:.0} %", nav.coverage * 100.0) } else { "—".into() }, c),
            ("Groundings", nav.groundings.to_string(), if nav.groundings > 0 { theme::BAD } else { c }),
        ],
    );
}
