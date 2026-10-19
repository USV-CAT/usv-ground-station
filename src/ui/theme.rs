//! Colours and styling.
//!
//! Dark, high-contrast and large: the Steam Deck's 7-inch screen is read in
//! daylight at arm's length, often with a thumb rather than a mouse.  Colours
//! carry meaning and mean the same everywhere — green is good and live, amber
//! is attention, red is act now.

use egui::{Color32, CornerRadius, FontFamily, FontId, Stroke, Style, Visuals};

use crate::core::events::Level;

pub const fn hex(rgb: u32) -> Color32 {
    Color32::from_rgb((rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8)
}

pub const BACKGROUND: Color32 = hex(0x0b1016);
pub const SURFACE: Color32 = hex(0x111922);
pub const SURFACE_RAISED: Color32 = hex(0x18232f);
pub const BORDER: Color32 = hex(0x253445);
pub const TEXT: Color32 = hex(0xe3ecf4);
pub const TEXT_DIM: Color32 = hex(0x7f91a3);
pub const FIELD: Color32 = hex(0x0a1118);

/// Cyan: the HUD.
pub const ACCENT: Color32 = hex(0x36c6f4);
/// Green: live, armed, OK.
pub const GOOD: Color32 = hex(0x5ee07a);
pub const CAUTION: Color32 = hex(0xffb53d);
pub const BAD: Color32 = hex(0xff5a4e);
pub const CRITICAL: Color32 = hex(0xff2d55);
/// Telemetry numbers, as on the reference station.
pub const VALUE: Color32 = hex(0xffd84a);

// The chart.
pub const LAND: Color32 = hex(0x1a1d16);
pub const LAND_RGB: [u8; 3] = [0x1a, 0x1d, 0x16];
pub const LAND_EDGE: Color32 = hex(0x4b5a3b);
pub const WATER_DEEP: [f32; 3] = [8.0, 22.0, 44.0];
pub const WATER_SHALLOW: [f32; 3] = [38.0, 130.0, 190.0];
pub const WATER_HAZARD: [f32; 3] = [255.0, 120.0, 70.0];
pub const CONTOUR: [u8; 3] = [120, 190, 235];
pub const TRACK: Color32 = hex(0xff9f3c);
pub const ROUTE: Color32 = hex(0xffffff);
pub const ROUTE_ACTIVE: Color32 = hex(0xffb53d);
pub const WAYPOINT: Color32 = hex(0x36c6f4);
pub const WAYPOINT_DONE: Color32 = hex(0x5d6f80);
pub const AREA: Color32 = hex(0x5ee07a);
pub const HOME: Color32 = hex(0x5ee07a);
pub const BOAT: Color32 = hex(0xffd84a);
pub const TARGET: Color32 = hex(0xff5ad8);
pub const INK: Color32 = hex(0x061018);

pub const THRUST: Color32 = hex(0xff7a1a);
pub const WIND: Color32 = hex(0x7fe3ff);
pub const CURRENT: Color32 = hex(0xa48bff);

/// OSM vector layers over the chart: (red, green, blue, alpha, width).
pub const MAP_LAYERS: [(&str, [u8; 4], f32); 6] = [
    ("landuse", [70, 90, 60, 110], 1.0),
    ("water", [90, 170, 230, 150], 1.0),
    ("building", [110, 110, 120, 120], 1.0),
    ("rail", [150, 120, 95, 140], 1.0),
    ("road_minor", [130, 130, 135, 110], 1.0),
    ("road_major", [220, 185, 100, 170], 2.0),
];

/// A colour with its alpha replaced (unmultiplied).
pub fn alpha(c: Color32, a: u8) -> Color32 {
    Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), a)
}

pub fn darker(c: Color32, factor: f32) -> Color32 {
    let k = 1.0 / factor;
    Color32::from_rgba_unmultiplied(
        (c.r() as f32 * k) as u8,
        (c.g() as f32 * k) as u8,
        (c.b() as f32 * k) as u8,
        c.a(),
    )
}

pub fn severity_colour(level: Level) -> Color32 {
    match level {
        Level::Critical => CRITICAL,
        Level::Error => BAD,
        Level::Warning => CAUTION,
        Level::Notice => ACCENT,
        Level::Debug => TEXT_DIM,
        Level::Info => TEXT,
    }
}

pub fn battery_colour(percent: Option<f64>) -> Color32 {
    match percent {
        None => TEXT_DIM,
        Some(p) if p > 40.0 => GOOD,
        Some(p) if p > 15.0 => CAUTION,
        Some(_) => BAD,
    }
}

pub fn font(size: f32) -> FontId {
    FontId::new(size, FontFamily::Proportional)
}

pub fn bold(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name("bold".into()))
}

pub fn mono(size: f32) -> FontId {
    FontId::new(size, FontFamily::Monospace)
}

pub fn mono_bold(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name("mono-bold".into()))
}

/// The look of the widgets: big targets for thumbs, dark surfaces.
pub fn apply(style: &mut Style) {
    let mut v = Visuals::dark();
    v.panel_fill = SURFACE;
    v.window_fill = SURFACE_RAISED;
    v.extreme_bg_color = FIELD;
    v.faint_bg_color = SURFACE_RAISED;
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = hex(0x12384a);
    v.selection.stroke = Stroke::new(1.5, ACCENT);
    v.hyperlink_color = ACCENT;
    v.window_stroke = Stroke::new(1.0, BORDER);
    let r = CornerRadius::same(6);
    for (w, fill) in [
        (&mut v.widgets.noninteractive, SURFACE),
        (&mut v.widgets.inactive, SURFACE_RAISED),
        (&mut v.widgets.hovered, SURFACE_RAISED),
        (&mut v.widgets.active, hex(0x0d151d)),
        (&mut v.widgets.open, SURFACE_RAISED),
    ] {
        w.bg_fill = fill;
        w.weak_bg_fill = fill;
        w.corner_radius = r;
        w.bg_stroke = Stroke::new(1.0, BORDER);
        w.fg_stroke = Stroke::new(1.5, TEXT);
    }
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT);
    v.widgets.active.bg_stroke = Stroke::new(2.0, ACCENT);
    style.visuals = v;
    style.spacing.button_padding = egui::vec2(10.0, 6.0);
    style.spacing.item_spacing = egui::vec2(6.0, 5.0);
    style.spacing.interact_size = egui::vec2(36.0, 30.0);
    style.spacing.slider_width = 170.0;
    style.spacing.scroll.bar_width = 10.0;
    style.text_styles.insert(egui::TextStyle::Body, font(14.0));
    style.text_styles.insert(egui::TextStyle::Button, font(14.0));
    style.text_styles.insert(egui::TextStyle::Small, font(11.5));
    style.text_styles.insert(egui::TextStyle::Monospace, mono(13.0));
    style.text_styles.insert(egui::TextStyle::Heading, bold(15.0));
}
