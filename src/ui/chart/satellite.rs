//! Satellite imagery under the chart.
//!
//! Standard Web-Mercator tiles (`{z}/{x}/{y}`, 256 pixels), fetched by a few
//! worker threads and kept on disk in `~/.cache/usvgs/tiles/`, so imagery
//! once seen — or downloaded on purpose before a trip with *Download this
//! area* — is there with no internet.  While a tile is missing, a coarser one
//! already at hand is stretched over its place.
//!
//! Tiles are cut along lines of latitude in Mercator, not evenly in degrees,
//! so each is drawn in a few horizontal bands with its true latitudes.

use std::collections::{HashMap, HashSet};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};

use egui::{Color32, ColorImage, Context, Painter, Pos2, Rect, TextureHandle, TextureOptions};

use super::draw;
use super::projection::Camera;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Tile {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

/// Highest zoom used: about 30 cm a pixel.
pub const MAX_Z: u8 = 19;
const MAX_TEXTURES: usize = 400;
const WORKERS: usize = 4;
/// Retry a tile that failed after this long, seconds.
const RETRY_S: f64 = 60.0;

impl Tile {
    pub fn tiles_across(z: u8) -> f64 {
        (1u64 << z) as f64
    }

    /// The tile holding a place, and where in it (fractions).
    pub fn at(lat: f64, lon: f64, z: u8) -> (Tile, f64, f64) {
        let n = Self::tiles_across(z);
        let x = (lon + 180.0) / 360.0 * n;
        let lat = lat.clamp(-85.05, 85.05).to_radians();
        let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0 * n;
        let (xi, yi) = (x.floor().clamp(0.0, n - 1.0), y.floor().clamp(0.0, n - 1.0));
        (Tile { z, x: xi as u32, y: yi as u32 }, x - xi, y - yi)
    }

    /// Latitude of a tile row's top edge (rows may be fractional).
    pub fn lat_of_row(y: f64, z: u8) -> f64 {
        let n = Self::tiles_across(z);
        (std::f64::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan().to_degrees()
    }

    pub fn lon_of_col(x: f64, z: u8) -> f64 {
        x / Self::tiles_across(z) * 360.0 - 180.0
    }

    pub fn parent(self) -> Option<Tile> {
        (self.z > 0).then(|| Tile { z: self.z - 1, x: self.x / 2, y: self.y / 2 })
    }

    pub fn path(self, root: &Path) -> PathBuf {
        root.join(self.z.to_string()).join(self.x.to_string()).join(format!("{}.img", self.y))
    }

    pub fn url(self, template: &str) -> String {
        template.replace("{z}", &self.z.to_string()).replace("{x}", &self.x.to_string()).replace("{y}", &self.y.to_string())
    }
}

/// The zoom whose pixels are about the size of the screen's at this scale.
pub fn zoom_for(metres_per_point: f64, lat: f64, pixels_per_point: f32) -> u8 {
    let metres_per_pixel = metres_per_point / pixels_per_point as f64;
    let z = (156_543.034 * lat.to_radians().cos() / metres_per_pixel.max(1e-3)).log2().round();
    z.clamp(1.0, MAX_Z as f64) as u8
}

/// Every tile covering bounds (south, west, north, east) at a zoom.
pub fn tiles_in(bounds: (f64, f64, f64, f64), z: u8) -> Vec<Tile> {
    let (s, w, n, e) = bounds;
    let (a, _, _) = Tile::at(n, w, z);
    let (b, _, _) = Tile::at(s, e, z);
    let mut out = Vec::new();
    for x in a.x..=b.x {
        for y in a.y..=b.y {
            out.push(Tile { z, x, y });
        }
    }
    out
}

enum Job {
    /// Load for showing: from disk, else from the network.
    Show(Tile),
    /// Save to disk only, for use offline later.
    Keep(Tile),
}

struct Shared {
    root: PathBuf,
    url: Mutex<String>,
    jobs: Mutex<Receiver<Job>>,
    done: Mutex<Vec<(Tile, Result<ColorImage, String>)>>,
    /// Offline download progress: queued, done, failed.
    queued: AtomicUsize,
    kept: AtomicUsize,
    failed: AtomicUsize,
}

fn fetch(shared: &Shared, tile: Tile) -> Result<Vec<u8>, String> {
    let path = tile.path(&shared.root);
    if let Ok(bytes) = std::fs::read(&path) {
        return Ok(bytes);
    }
    let url = tile.url(&shared.url.lock().unwrap());
    let response = ureq::get(&url)
        .set("User-Agent", concat!("usvgs/", env!("CARGO_PKG_VERSION"), " (USV ground station)"))
        .timeout(std::time::Duration::from_secs(12))
        .call()
        .map_err(|e| e.to_string())?;
    let mut bytes = Vec::new();
    response.into_reader().take(4 << 20).read_to_end(&mut bytes).map_err(|e| e.to_string())?;
    if let Some(dir) = path.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let _ = std::fs::write(&path, &bytes);
    Ok(bytes)
}

fn decode(bytes: &[u8]) -> Result<ColorImage, String> {
    let image = image::load_from_memory(bytes).map_err(|e| e.to_string())?.to_rgba8();
    let size = [image.width() as usize, image.height() as usize];
    Ok(ColorImage::from_rgba_unmultiplied(size, image.as_raw()))
}

pub struct Satellite {
    shared: Arc<Shared>,
    jobs: Sender<Job>,
    textures: HashMap<Tile, (TextureHandle, u64)>,
    pending: HashSet<Tile>,
    failed: HashMap<Tile, f64>,
    tick: u64,
    /// Tiles drawn last frame, and how many were not there yet.
    pub shown: usize,
    pub missing: usize,
    cache_count: Option<(f64, (u64, usize))>,
}

impl Satellite {
    pub fn new(ctx: &Context, url: &str) -> Satellite {
        let (jobs, rx) = channel::<Job>();
        let shared = Arc::new(Shared {
            root: crate::config::cache_dir().join("tiles").join("satellite"),
            url: Mutex::new(url.to_string()),
            jobs: Mutex::new(rx),
            done: Mutex::new(Vec::new()),
            queued: AtomicUsize::new(0),
            kept: AtomicUsize::new(0),
            failed: AtomicUsize::new(0),
        });
        for i in 0..WORKERS {
            let shared = shared.clone();
            let ctx = ctx.clone();
            let _ = std::thread::Builder::new().name(format!("tiles-{i}")).spawn(move || loop {
                let job = match shared.jobs.lock().unwrap().recv() {
                    Ok(j) => j,
                    Err(_) => return,
                };
                match job {
                    Job::Show(tile) => {
                        let result = fetch(&shared, tile).and_then(|b| decode(&b));
                        shared.done.lock().unwrap().push((tile, result));
                        ctx.request_repaint();
                    }
                    Job::Keep(tile) => {
                        match fetch(&shared, tile) {
                            Ok(_) => shared.kept.fetch_add(1, Ordering::Relaxed),
                            Err(_) => shared.failed.fetch_add(1, Ordering::Relaxed),
                        };
                        ctx.request_repaint();
                    }
                }
            });
        }
        Satellite { shared, jobs, textures: HashMap::new(), pending: HashSet::new(), failed: HashMap::new(), tick: 0, shown: 0, missing: 0, cache_count: None }
    }

    pub fn set_url(&mut self, url: &str) {
        let mut current = self.shared.url.lock().unwrap();
        if *current != url {
            *current = url.to_string();
            self.failed.clear();
        }
    }

    /// Offline download progress: (done, failed, queued).
    pub fn progress(&self) -> (usize, usize, usize) {
        (self.shared.kept.load(Ordering::Relaxed), self.shared.failed.load(Ordering::Relaxed), self.shared.queued.load(Ordering::Relaxed))
    }

    /// Queue every tile of an area at a range of zooms for keeping on disk.
    /// Returns how many were queued (capped, to stay polite to the server).
    pub fn download(&mut self, bounds: (f64, f64, f64, f64), zooms: std::ops::RangeInclusive<u8>, limit: usize) -> usize {
        let mut count = 0;
        for z in zooms {
            for tile in tiles_in(bounds, z) {
                if count >= limit {
                    break;
                }
                if !tile.path(&self.shared.root).exists() {
                    let _ = self.jobs.send(Job::Keep(tile));
                    count += 1;
                }
            }
        }
        self.shared.queued.fetch_add(count, Ordering::Relaxed);
        count
    }

    /// How many tiles an area would take.
    pub fn count(bounds: (f64, f64, f64, f64), zooms: std::ops::RangeInclusive<u8>) -> usize {
        zooms.map(|z| tiles_in(bounds, z).len()).sum()
    }

    /// [`Self::cache_size`], recounted at most every ten seconds.
    pub fn cache_size_cached(&mut self) -> (u64, usize) {
        let now = crate::core::clock::monotonic();
        if self.cache_count.is_none_or(|(t, _)| now - t > 10.0) {
            self.cache_count = Some((now, self.cache_size()));
        }
        self.cache_count.unwrap().1
    }

    /// Size of the tile cache on disk, bytes, and the tile count.
    pub fn cache_size(&self) -> (u64, usize) {
        fn walk(dir: &Path, total: &mut (u64, usize)) {
            if let Ok(rd) = std::fs::read_dir(dir) {
                for e in rd.flatten() {
                    let p = e.path();
                    if p.is_dir() {
                        walk(&p, total);
                    } else if let Ok(m) = e.metadata() {
                        total.0 += m.len();
                        total.1 += 1;
                    }
                }
            }
        }
        let mut total = (0, 0);
        walk(&self.shared.root, &mut total);
        total
    }

    fn collect(&mut self, ctx: &Context, now: f64) {
        let done: Vec<_> = std::mem::take(&mut *self.shared.done.lock().unwrap());
        for (tile, result) in done {
            self.pending.remove(&tile);
            match result {
                Ok(image) => {
                    let tex = ctx.load_texture(format!("sat-{}-{}-{}", tile.z, tile.x, tile.y), image, TextureOptions::LINEAR);
                    self.textures.insert(tile, (tex, self.tick));
                }
                Err(e) => {
                    log::debug!("tile {tile:?}: {e}");
                    self.failed.insert(tile, now);
                }
            }
        }
        if self.textures.len() > MAX_TEXTURES {
            let mut ages: Vec<u64> = self.textures.values().map(|t| t.1).collect();
            ages.sort_unstable();
            let cutoff = ages[ages.len() - MAX_TEXTURES * 3 / 4];
            self.textures.retain(|_, t| t.1 >= cutoff);
        }
    }

    fn want(&mut self, tile: Tile, now: f64) {
        if self.textures.contains_key(&tile) || self.pending.contains(&tile) {
            return;
        }
        if self.failed.get(&tile).is_some_and(|t| now - t < RETRY_S) {
            return;
        }
        self.pending.insert(tile);
        let _ = self.jobs.send(Job::Show(tile));
    }

    /// Draw one tile, or the part of a coarser one covering it.
    fn draw(p: &Painter, cam: &Camera, texture: &TextureHandle, tile: Tile, source: Tile) {
        let span = 1u32 << (tile.z - source.z);
        let (fx, fy) = ((tile.x % span) as f32 / span as f32, (tile.y % span) as f32 / span as f32);
        let size = 1.0 / span as f32;
        let west = Tile::lon_of_col(tile.x as f64, tile.z);
        let east = Tile::lon_of_col(tile.x as f64 + 1.0, tile.z);
        // Bands along the Mercator rows, each with its true latitudes.
        const BANDS: usize = 4;
        for b in 0..BANDS {
            let (r0, r1) = (b as f64 / BANDS as f64, (b + 1) as f64 / BANDS as f64);
            let north = Tile::lat_of_row(tile.y as f64 + r0, tile.z);
            let south = Tile::lat_of_row(tile.y as f64 + r1, tile.z);
            let uv = Rect::from_min_max(
                Pos2::new(fx, fy + size * r0 as f32),
                Pos2::new(fx + size, fy + size * r1 as f32),
            );
            draw::geo_image(p, cam, texture.id(), [north, west, south, east], uv, Color32::WHITE);
        }
    }

    pub fn paint(&mut self, ctx: &Context, p: &Painter, cam: &Camera, now: f64) {
        self.tick += 1;
        self.collect(ctx, now);
        let bounds = cam.bounds();
        let mid_lat = (bounds.0 + bounds.2) / 2.0;
        let mut z = zoom_for(cam.metres_per_pixel(), mid_lat, ctx.pixels_per_point());
        let mut tiles = tiles_in(bounds, z);
        while tiles.len() > 160 && z > 1 {
            z -= 1;
            tiles = tiles_in(bounds, z);
        }
        // Nearest first, so the middle of the view fills in first.
        let centre = cam.view.centre;
        let (c, _, _) = Tile::at(centre.lat, centre.lon, z);
        tiles.sort_by_key(|t| (t.x as i64 - c.x as i64).abs() + (t.y as i64 - c.y as i64).abs());
        let (mut shown, mut missing) = (0, 0);
        for tile in tiles {
            if let Some(entry) = self.textures.get_mut(&tile) {
                entry.1 = self.tick;
                Self::draw(p, cam, &entry.0, tile, tile);
                shown += 1;
                continue;
            }
            missing += 1;
            self.want(tile, now);
            // A coarser tile meanwhile.
            let mut up = tile.parent();
            while let Some(parent) = up.filter(|t| tile.z - t.z <= 6) {
                if let Some(entry) = self.textures.get_mut(&parent) {
                    entry.1 = self.tick;
                    Self::draw(p, cam, &entry.0, tile, parent);
                    break;
                }
                up = parent.parent();
            }
        }
        self.shown = shown;
        self.missing = missing;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_maths_round_trips() {
        let (t, fx, fy) = Tile::at(57.05, 24.0, 15);
        assert!((0.0..1.0).contains(&fx) && (0.0..1.0).contains(&fy));
        let north = Tile::lat_of_row(t.y as f64, 15);
        let south = Tile::lat_of_row(t.y as f64 + 1.0, 15);
        assert!(south < 57.05 && 57.05 < north);
        assert!(Tile::lon_of_col(t.x as f64, 15) <= 24.0 && 24.0 < Tile::lon_of_col(t.x as f64 + 1.0, 15));
        assert_eq!(t.parent().unwrap(), Tile { z: 14, x: t.x / 2, y: t.y / 2 });
        assert_eq!(Tile { z: 3, x: 4, y: 2 }.url("https://a/{z}/{y}/{x}"), "https://a/3/2/4");
    }

    #[test]
    fn zoom_matches_the_screen() {
        // About a metre a pixel near Riga is zoom 16.
        assert_eq!(zoom_for(1.0, 57.0, 1.0), 16);
        assert!(zoom_for(10.0, 57.0, 1.0) < zoom_for(1.0, 57.0, 1.0));
        assert_eq!(Satellite::count((57.0, 24.0, 57.01, 24.01), 14..=14), tiles_in((57.0, 24.0, 57.01, 24.01), 14).len());
    }
}
