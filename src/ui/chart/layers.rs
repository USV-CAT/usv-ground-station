//! The chart's picture layers, each held as GPU textures and rebuilt only
//! when what they show changes: the depth chart in either water style, the
//! survey (one texture per chunk, rebuilt when its cells change), the oil
//! slick, and the waves.

use std::collections::HashMap;
use std::sync::Arc;

use egui::epaint::{Mesh, Vertex};
use egui::{Align2, Color32, Context, Painter, Pos2, Rect, Shape, TextureHandle, TextureOptions};

use crate::config::WaterStyle;
use crate::core::bathymetry::{Cell, DepthGrid, SoundingGrid, ESTIMATED};
use crate::core::geo::{self, LatLon};
use crate::sim::oil::SpillSnapshot;
use crate::ui::theme;

use super::draw;
use super::images;
use super::projection::Camera;

const FULL_UV: Rect = Rect { min: Pos2::ZERO, max: Pos2 { x: 1.0, y: 1.0 } };

/// The depth chart as a picture, in the current water style.
#[derive(Default)]
pub struct ChartPicture {
    key: Option<(u64, u64, WaterStyle)>,
    texture: Option<TextureHandle>,
}

impl ChartPicture {
    pub fn paint(&mut self, ctx: &Context, p: &Painter, cam: &Camera, grid: &DepthGrid, chart_id: u64, alarm: f64, style: WaterStyle) {
        let key = (chart_id, alarm.to_bits(), style);
        if self.key != Some(key) || self.texture.is_none() {
            let image = match style {
                WaterStyle::Natural | WaterStyle::Satellite => images::natural_water(grid, alarm),
                WaterStyle::Chart => images::chart_shading(grid, alarm),
            };
            self.texture = Some(ctx.load_texture("chart", image, TextureOptions::LINEAR));
            self.key = Some(key);
        }
        let tex = self.texture.as_ref().unwrap();
        draw::geo_image(p, cam, tex.id(), [grid.north, grid.west, grid.south(), grid.east()], FULL_UV, Color32::WHITE);
    }
}

/// Depth figures in the cells, as on a paper chart, once they are legible.
pub fn paint_spot_depths(p: &Painter, cam: &Camera, grid: &DepthGrid) {
    let cell_px = grid.dlat * cam.view.scale;
    if cell_px < 46.0 {
        return;
    }
    let (min_lat, min_lon, max_lat, max_lon) = cam.bounds();
    let c0 = ((min_lon - grid.west) / grid.dlon).floor().max(0.0) as usize;
    let c1 = (((max_lon - grid.west) / grid.dlon).ceil().max(0.0) as usize).min(grid.cols);
    let r0 = ((grid.north - max_lat) / grid.dlat).floor().max(0.0) as usize;
    let r1 = (((grid.north - min_lat) / grid.dlat).ceil().max(0.0) as usize).min(grid.rows);
    if c1 <= c0 || r1 <= r0 || (c1 - c0) * (r1 - r0) > 900 {
        return;
    }
    let colour = Color32::from_rgba_unmultiplied(200, 225, 245, 170);
    for r in r0..r1 {
        for c in c0..c1 {
            let value = grid.at(r, c);
            if value.is_nan() {
                continue;
            }
            let Some(at) = cam.project(grid.cell_centre(r, c)) else { continue };
            let mut text = if value < 10.0 { format!("{value:.1}") } else { format!("{value:.0}") };
            if grid.quality_cell(r, c) == ESTIMATED {
                text = format!("~{text}");
            }
            p.text(at, Align2::CENTER_CENTER, text, theme::font(11.0), colour);
        }
    }
}

/// The survey: one small texture per chunk of the sounding grid.
#[derive(Default)]
pub struct SurveyLayer {
    textures: HashMap<Cell, TextureHandle>,
    range: (f64, f64),
    pub opacity: f32,
}

impl SurveyLayer {
    pub fn new() -> Self {
        SurveyLayer { textures: HashMap::new(), range: (0.0, 10.0), opacity: 0.85 }
    }

    pub fn depth_range(&self) -> (f64, f64) {
        self.range
    }

    pub fn has_data(&self) -> bool {
        !self.textures.is_empty()
    }

    /// Rebuild the chunks that changed.
    pub fn refresh(&mut self, ctx: &Context, grid: &mut SoundingGrid) {
        if grid.is_empty() {
            self.textures.clear();
            grid.dirty_chunks.clear();
            return;
        }
        let (low, high) = (grid.min_depth, grid.max_depth);
        // A big change of range recolours everything.
        if (low - self.range.0).abs() > 0.5 || (high - self.range.1).abs() > 0.5 {
            self.range = (low, high.max(low + 0.5));
            let all = grid.chunks();
            grid.dirty_chunks.extend(all);
        }
        let dirty: Vec<Cell> = grid.dirty_chunks.drain().collect();
        for chunk in dirty {
            let cells = grid.chunk_cells(chunk);
            if cells.is_empty() {
                self.textures.remove(&chunk);
                continue;
            }
            let image = images::survey_chunk(&cells, self.range.0, self.range.1);
            match self.textures.get_mut(&chunk) {
                Some(t) => t.set(image, TextureOptions::NEAREST),
                None => {
                    self.textures.insert(chunk, ctx.load_texture(format!("survey-{chunk:?}"), image, TextureOptions::NEAREST));
                }
            }
        }
    }

    pub fn paint(&self, p: &Painter, cam: &Camera, grid: &SoundingGrid) {
        let tint = Color32::from_white_alpha((self.opacity * 255.0) as u8);
        for (chunk, tex) in &self.textures {
            let (sw, ne) = grid.chunk_bounds(*chunk);
            draw::geo_image(p, cam, tex.id(), [ne.lat, sw.lon, sw.lat, ne.lon], FULL_UV, tint);
        }
    }
}

/// A vertical colour bar for the survey palette.
pub fn paint_survey_legend(p: &Painter, rect: Rect, low: f64, high: f64, title: &str) {
    p.rect_filled(rect.expand2(egui::vec2(0.0, 0.0)).with_min_x(rect.left() - 6.0).with_min_y(rect.top() - 20.0).with_max_x(rect.right() + 40.0).with_max_y(rect.bottom() + 6.0), 6.0, Color32::from_rgba_unmultiplied(8, 14, 20, 200));
    let steps = 24;
    let h = rect.height() / steps as f32;
    for i in 0..steps {
        let t = i as f64 / (steps - 1) as f64;
        let r = Rect::from_min_size(Pos2::new(rect.left(), rect.top() + i as f32 * h), egui::vec2(rect.width(), h + 0.5));
        p.rect_filled(r, 0.0, images::survey_rgb(low + t * (high - low), low, high));
    }
    p.text(Pos2::new(rect.left() - 2.0, rect.top() - 7.0), Align2::LEFT_BOTTOM, title, theme::font(10.5), theme::TEXT);
    p.text(Pos2::new(rect.right() + 4.0, rect.top()), Align2::LEFT_TOP, format!("{low:.1}"), theme::font(10.5), theme::TEXT);
    p.text(Pos2::new(rect.right() + 4.0, rect.bottom()), Align2::LEFT_BOTTOM, format!("{high:.1}"), theme::font(10.5), theme::TEXT);
}

/// The oil slick as one picture, rebuilt only when the slick has moved.
#[derive(Default)]
pub struct SlickLayer {
    version: Option<u64>,
    built_at: f64,
    texture: Option<TextureHandle>,
    edges: [f64; 4],
    beached: Vec<LatLon>,
}

/// Rebuilt at most this often (s); the slick drifts slowly.
pub const SLICK_REBUILD_S: f64 = 0.15;

impl SlickLayer {
    pub fn update(&mut self, ctx: &Context, oil: Option<&SpillSnapshot>, now: f64) {
        let Some(oil) = oil else {
            self.texture = None;
            self.beached.clear();
            self.version = None;
            return;
        };
        if self.version == Some(oil.version) || (self.version.is_some() && now - self.built_at < SLICK_REBUILD_S) {
            return;
        }
        self.version = Some(oil.version);
        self.built_at = now;
        let (mut lat, mut lon, mut vol) = (Vec::new(), Vec::new(), Vec::new());
        self.beached.clear();
        for i in 0..oil.volume.len() {
            if oil.volume[i] <= 1e-4 {
                continue;
            }
            if oil.beached[i] {
                self.beached.push(LatLon::new(oil.lat[i], oil.lon[i]));
            } else {
                lat.push(oil.lat[i]);
                lon.push(oil.lon[i]);
                vol.push(oil.volume[i]);
            }
        }
        match images::slick_image(&lat, &lon, &vol) {
            Some((image, edges)) => {
                self.edges = edges;
                match &mut self.texture {
                    Some(t) => t.set(image, TextureOptions::LINEAR),
                    None => self.texture = Some(ctx.load_texture("slick", image, TextureOptions::LINEAR)),
                }
            }
            None => self.texture = None,
        }
    }

    pub fn paint(&self, p: &Painter, cam: &Camera) {
        if let Some(t) = &self.texture {
            draw::geo_image(p, cam, t.id(), self.edges, FULL_UV, Color32::WHITE);
        }
        if !self.beached.is_empty() {
            let px = (4.0 / cam.metres_per_pixel()).max(3.0) as f32 / 2.0;
            for b in &self.beached {
                if let Some(at) = cam.project(*b) {
                    p.circle_filled(at, px, Color32::from_rgba_unmultiplied(20, 10, 4, 220));
                }
            }
        }
    }
}

/// What moves the water: true wind (speed m/s, from degrees) and surface
/// current (drift m/s, set degrees towards).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sea {
    pub wind: Option<(f64, f64)>,
    pub current: Option<(f64, f64)>,
}

/// Below this the wind no longer sets the waves; the current's flow does.
pub const LIGHT_AIR_MS: f64 = 1.5;
/// The waves turn to a new direction with this time constant, seconds.
pub const WAVE_TURN_S: f64 = 2.5;

impl Sea {
    /// Which way the waves run (degrees towards) and how strongly they are driven.
    ///
    /// Waves run downwind.  In light air the surface follows the current,
    /// so the ripples run with the flow instead.
    pub fn wave_direction(&self) -> (f64, f64) {
        match (self.wind, self.current) {
            (Some((speed, from)), _) if speed >= LIGHT_AIR_MS => ((from + 180.0).rem_euclid(360.0), speed),
            (wind, Some((drift, set))) if drift > 0.03 => (set, wind.map(|w| w.0).unwrap_or(0.0).max(drift * 4.0)),
            (Some((speed, from)), _) => ((from + 180.0).rem_euclid(360.0), speed),
            (None, _) => (90.0, 2.0),
        }
    }
}

/// Natural water's moving ripples: a loop of tileable frames laid over the
/// water running with the wind (or with the current in light air), fixed to
/// the water so it drifts with the flow, faded out over land.
pub struct WaveLayer {
    frames: Vec<TextureHandle>,
    started: f64,
    /// The direction the waves are drawn running, eased towards the sea's.
    towards: Option<f64>,
    last: f64,
}

impl WaveLayer {
    pub fn new(ctx: &Context, now: f64) -> Self {
        let options = TextureOptions { wrap_mode: egui::TextureWrapMode::Repeat, ..TextureOptions::LINEAR };
        let frames = images::wave_frames()
            .into_iter()
            .enumerate()
            .map(|(i, f)| ctx.load_texture(format!("waves-{i}"), f, options))
            .collect();
        WaveLayer { frames, started: now, towards: None, last: now }
    }

    /// How visible the waves are: a flat calm still shows a little texture.
    /// Looking straight down there is far less glare than at a low angle.
    pub fn strength(wind_ms: f64, overhead: bool) -> f64 {
        let level = (0.45 + wind_ms / 9.0).min(1.0);
        if overhead { level * 0.38 } else { level }
    }

    /// Ease the drawn direction towards `target`, so a noisy wind estimate
    /// does not make the sea swing about.
    pub fn ease(&mut self, target: f64, now: f64) -> f64 {
        let step = (now - self.last).clamp(0.0, 1.0);
        self.last = now;
        let towards = match self.towards {
            None => target,
            Some(t) => {
                let k = 1.0 - (-step / WAVE_TURN_S).exp();
                (t + geo::angle_difference(target, t) * k).rem_euclid(360.0)
            }
        };
        self.towards = Some(towards);
        towards
    }

    pub fn paint(&mut self, p: &Painter, cam: &Camera, sea: Sea, chart: Option<&DepthGrid>, now: f64) {
        let (target, drive) = sea.wave_direction();
        let towards = self.ease(target, now);
        let frame = (((now - self.started) / images::WAVE_LOOP_S * images::WAVE_FRAMES as f64) as usize) % images::WAVE_FRAMES;
        // The tile is this many metres across: a few wavelengths of chop,
        // bigger in more wind, never smaller on screen than a texture.
        let tile_m = 14.0 + 3.0 * drive.min(12.0);
        let tile_px = (tile_m / cam.metres_per_pixel()).max(220.0);
        // The pattern is fixed to the water, which the current carries along.
        let mut anchor_at = LatLon::new(57.0, 24.0);
        if let Some((drift, set)) = sea.current {
            anchor_at = geo::destination(anchor_at, set, drift * (now - self.started));
        }
        let anchor = cam.view.to_map(anchor_at);
        let turn = (towards - 90.0).to_radians(); // the tile's waves run along its +x
        let (sin, cos) = turn.sin_cos();
        let strength = Self::strength(drive, !cam.is_chase()) as f32;

        // A grid of vertices over the view: their texture position comes from
        // the ground, their opacity from how much water is under them.
        let v = &cam.view;
        let step = if cam.is_chase() { 14.0 } else { 26.0 };
        let (nx, ny) = (((v.w / step).ceil() as usize).max(1), ((v.h / step).ceil() as usize).max(1));
        let mut mesh = Mesh::with_texture(self.frames[frame].id());
        let mut uv_origin: Option<(f64, f64)> = None;
        let mut ok = Vec::with_capacity((nx + 1) * (ny + 1));
        for j in 0..=ny {
            for i in 0..=nx {
                let view = [v.ox + v.w * i as f64 / nx as f64, v.oy + v.h * j as f64 / ny as f64];
                let map = v.unrotate(view);
                let (dx, dy) = (map[0] - anchor[0], map[1] - anchor[1]);
                let (mut u, mut t) = ((dx * cos + dy * sin) / tile_px, (-dx * sin + dy * cos) / tile_px);
                // Keep the numbers small: the texture repeats.
                let origin = *uv_origin.get_or_insert((u.floor(), t.floor()));
                u -= origin.0;
                t -= origin.1;
                let water = match chart {
                    None => 1.0,
                    Some(g) => water_fraction(g, v.from_view(view)),
                };
                let a = (strength * water * 255.0) as u8;
                ok.push(cam.visible(view));
                mesh.vertices.push(Vertex {
                    pos: cam.screen_unclipped(view),
                    uv: Pos2::new(u as f32, t as f32),
                    color: Color32::from_rgba_premultiplied(a, a, a, a),
                });
            }
        }
        let idx = |i: usize, j: usize| (j * (nx + 1) + i) as u32;
        for j in 0..ny {
            for i in 0..nx {
                let c = [idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)];
                if c.iter().any(|&k| !ok[k as usize]) || c.iter().all(|&k| mesh.vertices[k as usize].color.a() == 0) {
                    continue;
                }
                mesh.add_triangle(c[0], c[1], c[2]);
                mesh.add_triangle(c[0], c[2], c[3]);
            }
        }
        if !mesh.indices.is_empty() {
            p.add(Shape::mesh(mesh));
        }
    }
}

/// How much of a place is water on the chart, smoothly (1 sea, 0 land).
pub fn water_fraction(grid: &DepthGrid, p: LatLon) -> f32 {
    let fr = (grid.north - p.lat) / grid.dlat - 0.5;
    let fc = (p.lon - grid.west) / grid.dlon - 0.5;
    let (r0, c0) = (fr.floor(), fc.floor());
    let (tr, tc) = ((fr - r0) as f32, (fc - c0) as f32);
    let cell = |r: f64, c: f64| -> f32 {
        if r < 0.0 || c < 0.0 || r >= grid.rows as f64 || c >= grid.cols as f64 {
            return 0.0;
        }
        if grid.at(r as usize, c as usize).is_nan() { 0.0 } else { 1.0 }
    };
    let top = cell(r0, c0) * (1.0 - tc) + cell(r0, c0 + 1.0) * tc;
    let bottom = cell(r0 + 1.0, c0) * (1.0 - tc) + cell(r0 + 1.0, c0 + 1.0) * tc;
    top * (1.0 - tr) + bottom * tr
}

/// Whether the chart has land anywhere in a camera's view.
pub fn land_in_view(grid: &Arc<DepthGrid>, cam: &Camera) -> bool {
    let (s, w, n, e) = cam.bounds();
    let c0 = ((w - grid.west) / grid.dlon).floor().max(0.0) as usize;
    let c1 = (((e - grid.west) / grid.dlon).ceil().max(0.0) as usize + 1).min(grid.cols);
    let r0 = ((grid.north - n) / grid.dlat).floor().max(0.0) as usize;
    let r1 = (((grid.north - s) / grid.dlat).ceil().max(0.0) as usize + 1).min(grid.rows);
    (r0..r1).any(|r| (c0..c1).any(|c| grid.at(r, c).is_nan()))
}

/// Metres to the nearest kilometre or metre, as a scale bar says it.
pub fn nice_length(metres: f64) -> String {
    if metres >= 1000.0 {
        let km = metres / 1000.0;
        if (km - km.round()).abs() < 1e-9 { format!("{km:.0} km") } else { format!("{km} km") }
    } else if (metres - metres.round()).abs() < 1e-9 {
        format!("{metres:.0} m")
    } else {
        format!("{metres} m")
    }
}

pub fn geofence_points(home: LatLon, radius_m: f64, cam: &Camera) -> Vec<[f64; 2]> {
    (0..72).map(|i| cam.view(geo::destination(home, i as f64 * 5.0, radius_m))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waves_run_downwind() {
        let sea = Sea { wind: Some((6.0, 240.0)), current: Some((0.2, 0.0)) };
        assert_eq!(sea.wave_direction().0, 60.0);
    }

    #[test]
    fn waves_follow_the_current_in_light_air() {
        let sea = Sea { wind: Some((0.5, 240.0)), current: Some((0.3, 135.0)) };
        assert_eq!(sea.wave_direction().0, 135.0);
        let calm = Sea { wind: Some((0.5, 240.0)), current: None };
        assert_eq!(calm.wave_direction().0, 60.0);
    }
}
