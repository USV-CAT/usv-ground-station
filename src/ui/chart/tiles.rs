//! Offline vector map tiles.
//!
//! A whole country's road, water and land-use geometry, built once by
//! `usvgs osm` into a tile index and a binary blob, then read lazily so that
//! only the tiles in view are decoded.  On the water the ground station
//! usually has no route to the internet, so a map that fetches anything is a
//! map that is blank exactly when it is needed.
//!
//! Decoded tiles are cached, least recently used first out.  Each line keeps
//! its size, so a zoomed-out view can leave out the ones smaller than a
//! pixel.  Zoomed out, the lines also come from simplified copies of the
//! tiles, with points closer than a pixel merged.

use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use serde_json::Value;

/// Above this many tiles in view, drawing is refused: decoding more would
/// stall for seconds.
pub const MAX_VISIBLE_TILES: i64 = 2500;
/// Simplified copies of the tiles, for zoomed-out views.  A view takes the
/// coarsest that stays under a pixel.
pub const SIMPLIFY_DEG: [f64; 3] = [0.0001, 0.0002, 0.0004];

/// One decoded layer of one tile: every line's points (lon, lat) end to end.
#[derive(Debug, Default)]
pub struct TileLayer {
    pub points: Vec<[f64; 2]>,
    /// (start, count, extent in degrees) of each line.
    pub lines: Vec<(u32, u32, f32)>,
}

type Key = (i64, i64, usize, u8);

pub struct TiledMap {
    pub tile_size_deg: f64,
    pub layers: Vec<String>,
    tiles: HashMap<(i64, i64), HashMap<String, (u64, u64)>>,
    pub source: Option<PathBuf>,
    blob: Option<File>,
    cache: HashMap<Key, (Arc<TileLayer>, u64)>,
    tick: u64,
    cache_limit: usize,
}

impl Default for TiledMap {
    fn default() -> Self {
        TiledMap::new(2000)
    }
}

/// Drop each point in the same `tolerance` cell as the one before; every
/// line keeps its two ends.
pub fn simplify(layer: &TileLayer, tolerance: f64) -> TileLayer {
    let mut out = TileLayer::default();
    for &(start, count, _) in &layer.lines {
        let pts = &layer.points[start as usize..(start + count) as usize];
        let first = out.points.len() as u32;
        let cell = |p: &[f64; 2]| ((p[1] / tolerance).floor(), (p[0] / tolerance).floor());
        for (i, p) in pts.iter().enumerate() {
            let keep = i == 0 || i + 1 == pts.len() || cell(p) != cell(&pts[i - 1]);
            if keep {
                out.points.push(*p);
            }
        }
        let n = out.points.len() as u32 - first;
        let extent = extent_of(&out.points[first as usize..]);
        out.lines.push((first, n, extent));
    }
    out
}

fn extent_of(points: &[[f64; 2]]) -> f32 {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in points {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (hi[0] - lo[0]).max(hi[1] - lo[1]) as f32
}

impl TiledMap {
    pub fn new(cache_tiles: usize) -> Self {
        TiledMap {
            tile_size_deg: 0.05,
            layers: Vec::new(),
            tiles: HashMap::new(),
            source: None,
            blob: None,
            cache: HashMap::new(),
            tick: 0,
            cache_limit: cache_tiles,
        }
    }

    pub fn is_loaded(&self) -> bool {
        !self.tiles.is_empty()
    }

    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Load a map index.  Returns the number of tiles available.
    pub fn load(&mut self, index_path: &Path) -> Result<usize, String> {
        let text = std::fs::read_to_string(index_path).map_err(|e| e.to_string())?;
        let data: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        self.tile_size_deg = data["tile"].as_f64().ok_or("no tile size")?;
        self.layers = data["layers"].as_array().map(|a| a.iter().filter_map(|v| v.as_str().map(String::from)).collect()).unwrap_or_default();
        let mut tiles = HashMap::new();
        for (key, record) in data["tiles"].as_object().ok_or("no tiles")? {
            let Some((ix, iy)) = key.split_once('_') else { continue };
            let (Ok(ix), Ok(iy)) = (ix.parse::<i64>(), iy.parse::<i64>()) else { continue };
            let mut layers = HashMap::new();
            for (layer, span) in record.as_object().into_iter().flatten() {
                if let (Some(off), Some(len)) = (span[0].as_u64(), span[1].as_u64()) {
                    layers.insert(layer.clone(), (off, len));
                }
            }
            tiles.insert((ix, iy), layers);
        }
        let bin = index_path.parent().unwrap_or(Path::new(".")).join(data["bin"].as_str().ok_or("no bin")?);
        self.blob = Some(File::open(&bin).map_err(|e| format!("{}: {e}", bin.display()))?);
        self.tiles = tiles;
        self.cache.clear();
        self.source = Some(index_path.to_path_buf());
        Ok(self.tiles.len())
    }

    fn read(&mut self, ix: i64, iy: i64, layer: &str) -> TileLayer {
        let mut out = TileLayer::default();
        let Some(&(offset, length)) = self.tiles.get(&(ix, iy)).and_then(|t| t.get(layer)) else { return out };
        let Some(blob) = &mut self.blob else { return out };
        let mut buffer = vec![0u8; length as usize];
        if blob.seek(SeekFrom::Start(offset)).is_err() || blob.read_exact(&mut buffer).is_err() {
            return out;
        }
        // Each line is a u16 count then that many (lat, lon) i32 pairs.
        let mut p = 0;
        while p + 2 <= buffer.len() {
            let n = u16::from_le_bytes([buffer[p], buffer[p + 1]]) as usize;
            let end = p + 2 + 8 * n;
            if end > buffer.len() {
                log::debug!("truncated tile {ix}_{iy} layer {layer}");
                break;
            }
            let start = out.points.len() as u32;
            for k in 0..n {
                let at = p + 2 + 8 * k;
                let lat = i32::from_le_bytes([buffer[at], buffer[at + 1], buffer[at + 2], buffer[at + 3]]);
                let lon = i32::from_le_bytes([buffer[at + 4], buffer[at + 5], buffer[at + 6], buffer[at + 7]]);
                out.points.push([lon as f64 * 1e-7, lat as f64 * 1e-7]);
            }
            if n > 0 {
                let extent = extent_of(&out.points[start as usize..]);
                out.lines.push((start, n as u32, extent));
            }
            p = end;
        }
        out
    }

    fn decode(&mut self, ix: i64, iy: i64, layer: &str, level: u8) -> Arc<TileLayer> {
        let layer_id = self.layers.iter().position(|l| l == layer).unwrap_or(usize::MAX);
        let key = (ix, iy, layer_id, level);
        self.tick += 1;
        if let Some(entry) = self.cache.get_mut(&key) {
            entry.1 = self.tick;
            return entry.0.clone();
        }
        let raw = self.read(ix, iy, layer);
        let decoded = if level == 0 {
            raw
        } else {
            let tolerance = SIMPLIFY_DEG[level as usize - 1];
            let mut s = simplify(&raw, tolerance);
            // A simplified copy is only drawn zoomed out, where lines smaller
            // than the tolerance are under a pixel.
            s.lines.retain(|l| l.2 as f64 >= tolerance);
            s
        };
        let entry = Arc::new(decoded);
        self.cache.insert(key, (entry.clone(), self.tick));
        if self.cache.len() > self.cache_limit {
            // Evict the oldest tenth in one go.
            let mut ages: Vec<u64> = self.cache.values().map(|v| v.1).collect();
            ages.sort_unstable();
            let cutoff = ages[ages.len() / 10];
            self.cache.retain(|_, v| v.1 > cutoff);
        }
        entry
    }

    /// The layers' tiles within the bounds, or None if the view is too wide.
    /// `level` 0 is full detail, 1.. the simplified copies.
    pub fn visible(
        &mut self,
        bounds: (f64, f64, f64, f64),
        layers: &[&str],
        level: u8,
    ) -> Option<Vec<(String, Arc<TileLayer>)>> {
        if self.tiles.is_empty() {
            return None;
        }
        let (min_lat, min_lon, max_lat, max_lon) = bounds;
        let size = self.tile_size_deg;
        let (ix0, ix1) = ((min_lat / size).floor() as i64, (max_lat / size).floor() as i64);
        let (iy0, iy1) = ((min_lon / size).floor() as i64, (max_lon / size).floor() as i64);
        if (ix1 - ix0 + 1) * (iy1 - iy0 + 1) > MAX_VISIBLE_TILES {
            return None;
        }
        let mut out = Vec::new();
        for ix in ix0..=ix1 {
            for iy in iy0..=iy1 {
                if !self.tiles.contains_key(&(ix, iy)) {
                    continue;
                }
                for layer in layers {
                    out.push((layer.to_string(), self.decode(ix, iy, layer, level)));
                }
            }
        }
        Some(out)
    }
}

/// The first map index that exists, searching the usual places.
pub fn find_map_index(candidate: &str) -> Option<PathBuf> {
    let mut search: Vec<PathBuf> = Vec::new();
    if !candidate.is_empty() {
        let expanded = if let Some(rest) = candidate.strip_prefix("~/") {
            std::env::var_os("HOME").map(|h| PathBuf::from(h).join(rest)).unwrap_or_else(|| PathBuf::from(candidate))
        } else {
            PathBuf::from(candidate)
        };
        search.push(expanded);
    }
    let mut folders = vec![PathBuf::from("maps")];
    if let Ok(exe) = std::env::current_exe() {
        // target/release/usvgs → the project's maps/.
        for up in exe.ancestors().skip(1).take(4) {
            folders.push(up.join("maps"));
        }
    }
    folders.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("maps"));
    for folder in folders {
        search.push(folder.join("latvia_map.idx.json"));
        if let Ok(entries) = std::fs::read_dir(&folder) {
            let mut found: Vec<PathBuf> = entries
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.to_string_lossy().ends_with(".idx.json"))
                .collect();
            found.sort();
            search.extend(found);
        }
    }
    search.into_iter().find(|p| p.exists())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simplify_merges_close_points_and_keeps_ends() {
        let mut layer = TileLayer::default();
        // One line of 50 points 1e-5° apart, and a short one of two points.
        for i in 0..50 {
            layer.points.push([24.0 + i as f64 * 1e-5, 57.0]);
        }
        layer.points.push([24.1, 57.1]);
        layer.points.push([24.10001, 57.1]);
        layer.lines = vec![(0, 50, 49e-5), (50, 2, 1e-5)];
        let s = simplify(&layer, 0.0001);
        let (_, n0, _) = s.lines[0];
        assert!(n0 < 50 && n0 >= 2);
        assert_eq!(s.points[0], layer.points[0]);
        assert_eq!(s.points[n0 as usize - 1], layer.points[49]);
        assert_eq!(s.lines[1].1, 2);
    }

    #[test]
    fn reads_the_bundled_map_when_present() {
        let Some(index) = find_map_index("") else { return };
        let mut map = TiledMap::default();
        assert!(map.load(&index).unwrap() > 100);
        let riga = (56.94, 24.08, 56.96, 24.12);
        let layers = map.visible(riga, &["road_major", "water"], 0).unwrap();
        assert!(layers.iter().any(|(_, t)| !t.lines.is_empty()));
    }
}
