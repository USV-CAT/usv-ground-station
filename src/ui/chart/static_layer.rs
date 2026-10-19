//! The map layers that do not move: the street map and the isobaths.
//!
//! They are drawn once into an image a little bigger than the view, on a
//! thread of their own with tiny-skia, and the image is shown until the view
//! zooms or moves past its spare edge.  While a zoom is in progress the old
//! image is only stretched, so a held zoom button or a pinch never waits on
//! it, and a sharp one replaces it once the zoom stops.
//!
//! Isobaths are computed by marching squares once per level of detail:
//! zoomed out, they come from a coarser copy of the grid, which looks the
//! same at that size and has a fraction of the segments.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};

use egui::ColorImage;
use tiny_skia::{LineCap, Paint, PathBuilder, Pixmap, Stroke, Transform};

use crate::core::bathymetry::{contour_segments, DepthGrid};
use crate::core::geo::LatLon;
use crate::ui::theme;

use super::tiles::{TiledMap, SIMPLIFY_DEG};

pub const ISOBATHS: [f64; 8] = [0.0, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0];
/// Isobaths come from a grid this many times coarser...
pub const LOD_FACTORS: [usize; 4] = [1, 2, 4, 8];
/// ...the first whose cells are at least this many pixels across.
pub const LOD_CELL_PX: f64 = 5.0;
/// Over natural water the coastline is surf only this close in (metres per
/// pixel); further out it is a thin line.
pub const SURF_MAX_M_PER_PX: f64 = 2.5;

/// What the image shows: a change in any of it means drawing it again.
#[derive(Clone, Debug, PartialEq)]
pub struct StaticKey {
    pub chart_id: u64,
    pub shallow_alarm_m: f64,
    pub isobaths: bool,
    pub osm: bool,
    pub natural: bool,
    pub ppp: f32,
}

#[derive(Clone)]
pub struct StaticRequest {
    pub key: StaticKey,
    pub chart: Option<Arc<DepthGrid>>,
    /// Map space of the image: its north-west corner, pixels per degree of
    /// latitude and of longitude, and its size in points.
    pub north: f64,
    pub west: f64,
    pub scale: f64,
    pub lon_px: f64,
    pub size: [f64; 2],
}

impl StaticRequest {
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        (self.north - self.size[1] / self.scale, self.west, self.north, self.west + self.size[0] / self.lon_px)
    }
}

pub struct StaticImage {
    pub request: StaticRequest,
    pub image: ColorImage,
}

/// The layer types the street map shows at a scale, bottom first.
pub fn layers_for_scale(scale: f64) -> Vec<&'static str> {
    let mut wanted = vec!["water", "road_major"];
    if scale > 40_000.0 {
        wanted.extend(["landuse", "rail"]);
    }
    if scale > 150_000.0 {
        wanted.push("road_minor");
    }
    if scale > 900_000.0 {
        wanted.push("building");
    }
    theme::MAP_LAYERS.iter().map(|(name, _, _)| *name).filter(|n| wanted.contains(n)).collect()
}

/// The isobaths of a chart, by level of detail.
#[derive(Default)]
pub struct Isobaths {
    sets: HashMap<(u64, usize), Arc<Vec<(f64, Vec<[f64; 4]>)>>>,
}

impl Isobaths {
    pub fn at(&mut self, chart_id: u64, grid: &DepthGrid, factor: usize) -> Arc<Vec<(f64, Vec<[f64; 4]>)>> {
        self.sets.retain(|(id, _), _| *id == chart_id);
        self.sets
            .entry((chart_id, factor))
            .or_insert_with(|| {
                let coarse;
                let g = if factor > 1 {
                    coarse = grid.coarsened(factor);
                    &coarse
                } else {
                    grid
                };
                Arc::new(ISOBATHS.iter().map(|&level| (level, contour_segments(g, level))).collect())
            })
            .clone()
    }

    pub fn factor_for(grid: &DepthGrid, scale: f64) -> usize {
        let cell_px = grid.dlat * scale;
        LOD_FACTORS.iter().copied().find(|f| cell_px * *f as f64 >= LOD_CELL_PX).unwrap_or(8)
    }
}

fn paint(rgba: [u8; 4]) -> Paint<'static> {
    let mut p = Paint::default();
    p.set_color_rgba8(rgba[0], rgba[1], rgba[2], rgba[3]);
    p.anti_alias = true;
    p
}

fn stroke(width: f32, round: bool) -> Stroke {
    Stroke { width, line_cap: if round { LineCap::Round } else { LineCap::Butt }, ..Stroke::default() }
}

/// Draw one request.  Pure: everything comes in the request and the caches.
pub fn render(request: &StaticRequest, map: &mut TiledMap, isobaths: &mut Isobaths) -> ColorImage {
    let ppp = request.key.ppp as f64;
    let (w, h) = ((request.size[0] * ppp).ceil().max(1.0) as u32, (request.size[1] * ppp).ceil().max(1.0) as u32);
    let Some(mut pixmap) = Pixmap::new(w, h) else { return ColorImage::new([1, 1], vec![egui::Color32::TRANSPARENT]) };
    let to_px = |lat: f64, lon: f64| -> (f32, f32) {
        (((lon - request.west) * request.lon_px * ppp) as f32, ((request.north - lat) * request.scale * ppp) as f32)
    };
    let bounds = request.bounds();
    let inside = |lat: f64, lon: f64| bounds.0 <= lat && lat <= bounds.2 && bounds.1 <= lon && lon <= bounds.3;
    let ident = Transform::identity();

    if request.key.osm && map.is_loaded() {
        let layers = layers_for_scale(request.scale);
        // Lines under a pixel across would not show, nor would detail finer
        // than a pixel in the rest.
        let level = SIMPLIFY_DEG.iter().filter(|t| *t * request.scale <= 1.0).count() as u8;
        let min_extent = (1.0 / request.scale) as f32;
        if let Some(tiles) = map.visible(bounds, &layers, level) {
            for (name, rgba, width) in theme::MAP_LAYERS {
                if !layers.contains(&name) {
                    continue;
                }
                let mut pb = PathBuilder::new();
                for (_, tile) in tiles.iter().filter(|(l, _)| l == name) {
                    for &(start, count, extent) in &tile.lines {
                        if extent < min_extent || count < 2 {
                            continue;
                        }
                        let pts = &tile.points[start as usize..(start + count) as usize];
                        let (x, y) = to_px(pts[0][1], pts[0][0]);
                        pb.move_to(x, y);
                        for p in &pts[1..] {
                            let (x, y) = to_px(p[1], p[0]);
                            pb.line_to(x, y);
                        }
                    }
                }
                if let Some(path) = pb.finish() {
                    pixmap.stroke_path(&path, &paint(rgba), &stroke(width * ppp as f32, false), ident, None);
                }
            }
        }
    }

    if let (true, Some(grid)) = (request.key.isobaths, &request.chart) {
        let cell_px = grid.dlat * request.scale;
        let factor = Isobaths::factor_for(grid, request.scale);
        let sets = isobaths.at(request.key.chart_id, grid, factor);
        let natural = request.key.natural;
        let metres_px = crate::core::geo::metres_per_degree(bounds.0).0 / request.scale;
        for (level, segments) in sets.iter() {
            let level = *level;
            // Thin out the deep contours when zoomed far out.
            if segments.is_empty() || (cell_px < 3.0 && ![0.0, 5.0, 20.0].contains(&level)) {
                continue;
            }
            let mut pb = PathBuilder::new();
            let mut any = false;
            for s in segments {
                if !inside(s[1], s[0]) {
                    continue;
                }
                let (x0, y0) = to_px(s[1], s[0]);
                let (x1, y1) = to_px(s[3], s[2]);
                pb.move_to(x0, y0);
                pb.line_to(x1, y1);
                any = true;
            }
            if !any {
                continue;
            }
            let Some(path) = pb.finish() else { continue };
            let px = ppp as f32;
            if level == 0.0 && natural {
                if metres_px > SURF_MAX_M_PER_PX {
                    // Too far out for surf to show: a thin line of sand.
                    pixmap.stroke_path(&path, &paint([206, 196, 160, 150]), &stroke(1.3 * px, false), ident, None);
                } else {
                    // Surf: a soft band a few metres wide with a brighter edge.
                    let width = (5.0 / metres_px).max(2.0) as f32;
                    for (w, a) in [(width, 45u8), (width * 0.4, 85), (1.0, 150)] {
                        pixmap.stroke_path(&path, &paint([235, 245, 250, a]), &stroke(w * px, true), ident, None);
                    }
                }
                continue;
            }
            let (rgba, width) = if level == 0.0 {
                let c = theme::LAND_EDGE;
                ([c.r(), c.g(), c.b(), 255], 1.6)
            } else {
                let mut a: u8 = if [2.0, 5.0, 10.0, 20.0].contains(&level) { 150 } else { 90 };
                let mut c = theme::CONTOUR;
                if level <= request.key.shallow_alarm_m + 1e-6 {
                    c = [255, 120, 70];
                    a = 200;
                }
                if natural {
                    a /= 2;
                }
                ([c[0], c[1], c[2], a], 1.0)
            };
            pixmap.stroke_path(&path, &paint(rgba), &stroke(width * px, false), ident, None);
        }
    }

    ColorImage::from_rgba_premultiplied([w as usize, h as usize], pixmap.data())
}

/// The drawing thread, and the latest image it made.
pub struct StaticRenderer {
    requests: Sender<StaticRequest>,
    result: Arc<Mutex<Option<StaticImage>>>,
}

impl StaticRenderer {
    pub fn new(map_index: Option<PathBuf>, ctx: egui::Context) -> StaticRenderer {
        let (requests, rx): (Sender<StaticRequest>, Receiver<StaticRequest>) = channel();
        let result: Arc<Mutex<Option<StaticImage>>> = Arc::new(Mutex::new(None));
        let out = result.clone();
        std::thread::Builder::new()
            .name("chart-render".into())
            .spawn(move || {
                let mut map = TiledMap::default();
                if let Some(index) = map_index {
                    match map.load(&index) {
                        Ok(n) => log::info!("offline map {}: {n} tiles", index.display()),
                        Err(e) => log::warn!("offline map {} unusable: {e}", index.display()),
                    }
                }
                let mut isobaths = Isobaths::default();
                while let Ok(mut request) = rx.recv() {
                    // Only the newest request matters.
                    while let Ok(newer) = rx.try_recv() {
                        request = newer;
                    }
                    let image = render(&request, &mut map, &mut isobaths);
                    *out.lock().unwrap() = Some(StaticImage { request, image });
                    ctx.request_repaint();
                }
            })
            .expect("chart render thread");
        StaticRenderer { requests, result }
    }

    pub fn request(&self, request: StaticRequest) {
        let _ = self.requests.send(request);
    }

    pub fn take(&self) -> Option<StaticImage> {
        self.result.lock().unwrap().take()
    }
}

/// A position in the image's map space, for tests.
pub fn image_point(request: &StaticRequest, p: LatLon) -> (f64, f64) {
    ((p.lon - request.west) * request.lon_px, (request.north - p.lat) * request.scale)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coarse_isobaths_have_fewer_segments() {
        let grid = DepthGrid::bundled().unwrap();
        let mut iso = Isobaths::default();
        let fine: usize = iso.at(1, &grid, 1).iter().map(|(_, s)| s.len()).sum();
        let coarse: usize = iso.at(1, &grid, 4).iter().map(|(_, s)| s.len()).sum();
        assert!(coarse * 3 < fine, "{coarse} vs {fine}");
    }

    #[test]
    fn isobaths_render_into_the_image() {
        let grid = Arc::new(DepthGrid::bundled().unwrap());
        let request = StaticRequest {
            key: StaticKey { chart_id: 1, shallow_alarm_m: 1.2, isobaths: true, osm: false, natural: false, ppp: 1.0 },
            chart: Some(grid),
            // Across the beach at Daugavgrīva, so the coast and the shallow
            // contours are all in it.
            north: 57.07,
            west: 23.96,
            scale: 20_000.0,
            lon_px: 20_000.0 * 57.05f64.to_radians().cos(),
            size: [800.0, 600.0],
        };
        let image = render(&request, &mut TiledMap::default(), &mut Isobaths::default());
        assert_eq!(image.size, [800, 600]);
        assert!(image.pixels.iter().filter(|p| p.a() > 0).count() > 100);
    }
}
