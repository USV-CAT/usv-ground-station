//! The window: an egui app laid out for the Steam Deck's 1280×800 screen.

pub mod app;
pub mod chart;
pub mod connect;
pub mod files;
pub mod graphs;
pub mod instruments;
pub mod pages;
pub mod powerflow;
pub mod telemetry;
pub mod theme;

use std::path::PathBuf;
use std::sync::Arc;

use egui::{FontData, FontDefinitions, FontFamily};

use crate::config::Settings;
use crate::core::bathymetry::DepthGrid;

/// The depth chart named in the settings, or the bundled one.
pub fn load_chart(settings: &Settings) -> Option<DepthGrid> {
    let named = settings.map.depth_grid.trim();
    if !named.is_empty() {
        let path = match named.strip_prefix("~/") {
            Some(rest) => std::env::var_os("HOME").map(|h| PathBuf::from(h).join(rest)).unwrap_or_else(|| PathBuf::from(named)),
            None => PathBuf::from(named),
        };
        match DepthGrid::load(&path) {
            Ok(g) => return Some(g),
            Err(e) => log::warn!("no depth chart ({}): {e}; using the bundled one", path.display()),
        }
    }
    match DepthGrid::bundled() {
        Ok(g) => Some(g),
        Err(e) => {
            log::warn!("the bundled depth chart is unreadable: {e}");
            None
        }
    }
}

/// Fonts: egui's own, with the system's DejaVu and Noto symbol fonts behind
/// them where they exist, for the arrows, anchors and Deck glyphs.
fn fonts() -> FontDefinitions {
    let mut fonts = FontDefinitions::default();
    let candidates = |names: &[&str]| -> Option<Vec<u8>> {
        let dirs = ["/usr/share/fonts/TTF", "/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/dejavu", "/usr/share/fonts/noto", "/usr/share/fonts/truetype/noto", "/usr/share/fonts/google-noto"];
        for d in dirs {
            for n in names {
                if let Ok(bytes) = std::fs::read(PathBuf::from(d).join(n)) {
                    return Some(bytes);
                }
            }
        }
        None
    };
    let mut add = |name: &str, files: &[&str]| -> bool {
        match candidates(files) {
            Some(bytes) => {
                fonts.font_data.insert(name.to_string(), Arc::new(FontData::from_owned(bytes)));
                true
            }
            None => false,
        }
    };
    let sans = add("dejavu", &["DejaVuSans.ttf"]);
    let sans_bold = add("dejavu-bold", &["DejaVuSans-Bold.ttf"]);
    let mono = add("dejavu-mono", &["DejaVuSansMono.ttf"]);
    let mono_bold = add("dejavu-mono-bold", &["DejaVuSansMono-Bold.ttf"]);
    let symbols = add("symbols", &["NotoSansSymbols-Regular.ttf"]);
    let symbols2 = add("symbols2", &["NotoSansSymbols2-Regular.ttf"]);

    let fallbacks: Vec<String> = [(symbols, "symbols"), (symbols2, "symbols2")]
        .into_iter()
        .filter(|(ok, _)| *ok)
        .map(|(_, n)| n.to_string())
        .collect();
    let proportional = fonts.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
    let monospace = fonts.families.get(&FontFamily::Monospace).cloned().unwrap_or_default();
    let family = |first: Option<&str>, rest: &[String]| -> Vec<String> {
        let mut v: Vec<String> = first.map(|f| vec![f.to_string()]).unwrap_or_default();
        v.extend(rest.iter().cloned());
        v.extend(fallbacks.iter().cloned());
        v
    };
    fonts.families.insert(FontFamily::Proportional, family(sans.then_some("dejavu"), &proportional));
    fonts.families.insert(FontFamily::Monospace, family(mono.then_some("dejavu-mono"), &monospace));
    let bold_first = if sans_bold { Some("dejavu-bold") } else if sans { Some("dejavu") } else { None };
    fonts.families.insert(FontFamily::Name("bold".into()), family(bold_first, &proportional));
    let mono_bold_first = if mono_bold { Some("dejavu-mono-bold") } else if mono { Some("dejavu-mono") } else { None };
    fonts.families.insert(FontFamily::Name("mono-bold".into()), family(mono_bold_first, &monospace));
    fonts
}

/// The station's fonts and look, on a context.
pub fn install_look(ctx: &egui::Context) {
    ctx.set_fonts(fonts());
    ctx.global_style_mut(theme::apply);
}

struct StationApp {
    app: app::App,
}

impl eframe::App for StationApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.app.frame(ui);
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.app.shutdown();
    }

    fn clear_color(&self, _visuals: &egui::Visuals) -> [f32; 4] {
        let c = theme::BACKGROUND;
        [c.r() as f32 / 255.0, c.g() as f32 / 255.0, c.b() as f32 / 255.0, 1.0]
    }
}

/// Open the window and run until it closes.  `fullscreen` None decides by
/// the screen: the Deck's 1280×800 panel is filled, elsewhere a window.
pub fn run(settings: Settings, fullscreen: Option<bool>, view: Option<chart::ViewMode>, power: bool, mission: Option<PathBuf>, start: Option<crate::link::Choice>) -> Result<(), String> {
    // Without a choice on the command line: ask, unless told not to.
    let start = start.or_else(|| {
        (!settings.link.ask_at_start).then(|| match settings.link.kind.as_str() {
            "mavlink" => crate::link::Choice::Mavlink(settings.link.mavlink_url.clone()),
            _ => crate::link::Choice::Simulator,
        })
    });
    let chart = load_chart(&settings).map(Arc::new);
    let viewport = egui::ViewportBuilder::default()
        .with_title("USV Ground Station")
        .with_app_id("usvgs")
        .with_inner_size([1280.0, 800.0])
        .with_min_inner_size([960.0, 600.0]);
    let options = eframe::NativeOptions { viewport, ..Default::default() };
    eframe::run_native(
        "usvgs",
        options,
        Box::new(move |cc| {
            let monitor = cc.egui_ctx.input(|i| i.viewport().monitor_size);
            let full = fullscreen.or_else(|| monitor.map(|m| m.x <= 1280.0 && m.y <= 800.0));
            let mut app = app::App::new_with(&cc.egui_ctx, settings, chart, full, view, start);
            app.power_view = power;
            if let Some(m) = mission {
                app.open_mission(m);
            }
            Ok(Box::new(StationApp { app }))
        }),
    )
    .map_err(|e| e.to_string())
}
