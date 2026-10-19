//! Water depth: the chart, and what the boat has measured.
//!
//! Two different things, kept apart because they deserve different trust:
//!
//! * [`DepthGrid`] is the *chart* — a regular latitude/longitude grid loaded
//!   from a file before the trip.  The bundled one is EMODnet's Gulf of Riga
//!   digital terrain model, about 115 m per cell.  Inland lakes carry only a
//!   placeholder there, so the builder estimated their depth from the distance
//!   to the shore and the grid marks those cells as estimated.
//! * [`SoundingGrid`] is the *survey* — what the boat's echo sounder actually
//!   read, binned into small square cells as the boat crosses them.  This is
//!   the product of a survey and is what gets exported.
//!
//! Depths are metres, positive down.  Land and unknown cells are NaN.

use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::path::Path;

use super::clock;
use super::geo::{self, BBox, LatLon, Metres};
use super::mission::Rectangle;
use super::npy;

/// Quality flags stored alongside a chart grid.
pub const MEASURED: u8 = 0;
pub const ESTIMATED: u8 = 1;

/// A depth chart on a regular grid.  Row 0 is the northern edge.
#[derive(Clone, Debug)]
pub struct DepthGrid {
    /// `rows * cols` depths, row-major; NaN on land.
    pub depth: Vec<f32>,
    pub rows: usize,
    pub cols: usize,
    pub north: f64,
    pub west: f64,
    pub dlat: f64,
    pub dlon: f64,
    pub quality: Vec<u8>,
    pub source: String,
}

impl DepthGrid {
    pub fn new(depth: Vec<f32>, rows: usize, cols: usize, north: f64, west: f64, dlat: f64, dlon: f64) -> Self {
        assert_eq!(depth.len(), rows * cols, "depth grid size");
        DepthGrid { quality: vec![MEASURED; rows * cols], depth, rows, cols, north, west, dlat, dlon, source: String::new() }
    }

    pub fn with_quality(mut self, quality: Vec<u8>) -> Self {
        if quality.len() == self.depth.len() {
            self.quality = quality;
        }
        self
    }

    pub fn with_source(mut self, source: impl Into<String>) -> Self {
        self.source = source.into();
        self
    }

    // -- geometry -------------------------------------------------------------

    pub fn south(&self) -> f64 {
        self.north - self.rows as f64 * self.dlat
    }

    pub fn east(&self) -> f64 {
        self.west + self.cols as f64 * self.dlon
    }

    pub fn bounds(&self) -> BBox {
        BBox::new(self.south(), self.west, self.north, self.east())
    }

    pub fn contains(&self, lat: f64, lon: f64) -> bool {
        self.south() <= lat && lat < self.north && self.west <= lon && lon < self.east()
    }

    #[inline]
    pub fn at(&self, row: usize, col: usize) -> f32 {
        self.depth[row * self.cols + col]
    }

    #[inline]
    pub fn quality_cell(&self, row: usize, col: usize) -> u8 {
        self.quality[row * self.cols + col]
    }

    /// Row and column of a position, measured between cell centres.
    fn fractional(&self, lat: f64, lon: f64) -> (f64, f64) {
        ((self.north - lat) / self.dlat - 0.5, (lon - self.west) / self.dlon - 0.5)
    }

    /// The centre of a cell.
    pub fn cell_centre(&self, row: usize, col: usize) -> LatLon {
        LatLon::new(self.north - (row as f64 + 0.5) * self.dlat, self.west + (col as f64 + 0.5) * self.dlon)
    }

    // -- sampling -------------------------------------------------------------

    /// Interpolated depth, or NaN on land or off the grid.
    ///
    /// Bilinear between the four surrounding cells.  When some of them are
    /// land, the water cells are blended and the depth tapers to zero as the
    /// position approaches the land, so the shore is a slope, not a cliff.
    pub fn depth_at(&self, lat: f64, lon: f64) -> f64 {
        if !self.contains(lat, lon) {
            return f64::NAN;
        }
        let (fr, fc) = self.fractional(lat, lon);
        let (r0, c0) = (fr.floor() as i64, fc.floor() as i64);
        let (tr, tc) = (fr - r0 as f64, fc - c0 as f64);
        let (mut total, mut weight_water, mut weight_all) = (0.0, 0.0, 0.0);
        for (dr, wr) in [(0, 1.0 - tr), (1, tr)] {
            for (dc, wc) in [(0, 1.0 - tc), (1, tc)] {
                let r = (r0 + dr).clamp(0, self.rows as i64 - 1) as usize;
                let c = (c0 + dc).clamp(0, self.cols as i64 - 1) as usize;
                let w = wr * wc;
                weight_all += w;
                let value = self.at(r, c);
                if !value.is_nan() {
                    total += w * value as f64;
                    weight_water += w;
                }
            }
        }
        if weight_water <= 0.0 || weight_water < 0.35 * weight_all {
            return f64::NAN;
        }
        (total / weight_water) * (weight_water / weight_all)
    }

    /// Depth of the nearest cell; NaN on land or off the grid.
    pub fn depth_nearest(&self, lat: f64, lon: f64) -> f64 {
        let r = ((self.north - lat) / self.dlat).floor();
        let c = ((lon - self.west) / self.dlon).floor();
        if r < 0.0 || c < 0.0 || r >= self.rows as f64 || c >= self.cols as f64 {
            return f64::NAN;
        }
        self.at(r as usize, c as usize) as f64
    }

    pub fn quality_at(&self, lat: f64, lon: f64) -> u8 {
        if !self.contains(lat, lon) {
            return ESTIMATED;
        }
        let (fr, fc) = self.fractional(lat, lon);
        let r = (fr.round() as i64).clamp(0, self.rows as i64 - 1) as usize;
        let c = (fc.round() as i64).clamp(0, self.cols as i64 - 1) as usize;
        self.quality_cell(r, c)
    }

    pub fn is_water(&self, lat: f64, lon: f64) -> bool {
        !self.depth_at(lat, lon).is_nan()
    }

    /// The deepest water cell centre near a position, for spawning a boat.
    pub fn deepest_near(&self, lat: f64, lon: f64, radius_cells: i64) -> Option<LatLon> {
        let (fr, fc) = self.fractional(lat, lon);
        let (r0, c0) = (fr.round() as i64, fc.round() as i64);
        let mut best: Option<(f32, usize, usize)> = None;
        for r in (r0 - radius_cells).max(0)..(r0 + radius_cells + 1).min(self.rows as i64) {
            for c in (c0 - radius_cells).max(0)..(c0 + radius_cells + 1).min(self.cols as i64) {
                let value = self.at(r as usize, c as usize);
                if !value.is_nan() && best.is_none_or(|b| value > b.0) {
                    best = Some((value, r as usize, c as usize));
                }
            }
        }
        best.map(|(_, r, c)| self.cell_centre(r, c))
    }

    /// The grid averaged over `factor` × `factor` blocks (land as zero), for
    /// coarse isobaths.  Same north-west corner, bigger cells.
    pub fn coarsened(&self, factor: usize) -> DepthGrid {
        if factor <= 1 {
            return self.clone();
        }
        let (rows, cols) = (self.rows / factor, self.cols / factor);
        let mut depth = vec![0.0f32; rows * cols];
        let n = (factor * factor) as f32;
        for r in 0..rows {
            for c in 0..cols {
                let mut sum = 0.0f32;
                for dr in 0..factor {
                    for dc in 0..factor {
                        let v = self.at(r * factor + dr, c * factor + dc);
                        if !v.is_nan() {
                            sum += v;
                        }
                    }
                }
                depth[r * cols + c] = sum / n;
            }
        }
        DepthGrid::new(depth, rows, cols, self.north, self.west, self.dlat * factor as f64, self.dlon * factor as f64)
    }

    // -- files ----------------------------------------------------------------

    pub fn save_npz(&self, path: &Path) -> Result<(), String> {
        // Decimetres in an int16 keep a whole gulf in about a megabyte.
        let decimetres: Vec<i16> = self
            .depth
            .iter()
            .map(|&d| if d.is_nan() { -32768 } else { (d as f64 * 10.0).round().clamp(-32767.0, 32767.0) as i16 })
            .collect();
        let shape = vec![self.rows, self.cols];
        let depth = npy::Array::from_i16(shape.clone(), &decimetres);
        let quality = npy::Array::from_u8(shape, &self.quality);
        let geo = npy::Array::from_f64(vec![4], &[self.north, self.west, self.dlat, self.dlon]);
        let source = npy::Array::from_text(&self.source);
        let file = std::fs::File::create(path).map_err(|e| e.to_string())?;
        npy::write_npz(
            std::io::BufWriter::new(file),
            &[("depth_dm", &depth), ("quality", &quality), ("geo", &geo), ("source", &source)],
        )
    }

    /// Load any supported format, chosen by extension.
    pub fn load(path: &Path) -> Result<DepthGrid, String> {
        let suffix = path.extension().and_then(|s| s.to_str()).unwrap_or("").to_lowercase();
        match suffix.as_str() {
            "npz" => DepthGrid::load_npz(path),
            "asc" => DepthGrid::load_ascii_grid(path),
            "tif" | "tiff" => DepthGrid::load_geotiff(path),
            "xyz" | "csv" | "txt" => DepthGrid::load_xyz(path, 0.0005),
            _ => Err(format!("unsupported depth grid format: {}", path.display())),
        }
    }

    pub fn load_npz(path: &Path) -> Result<DepthGrid, String> {
        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        DepthGrid::from_npz_bytes(&bytes, &name)
    }

    pub fn from_npz_bytes(bytes: &[u8], fallback_name: &str) -> Result<DepthGrid, String> {
        let arrays = npy::read_npz_bytes(bytes)?;
        let depth_dm = arrays.get("depth_dm").ok_or("no depth_dm in the chart")?;
        if depth_dm.shape.len() != 2 {
            return Err("depth_dm is not two-dimensional".into());
        }
        let (rows, cols) = (depth_dm.shape[0], depth_dm.shape[1]);
        let depth: Vec<f32> = depth_dm
            .to_f64()?
            .into_iter()
            .map(|v| if v == -32768.0 { f32::NAN } else { (v / 10.0) as f32 })
            .collect();
        let geo = arrays.get("geo").ok_or("no geo in the chart")?.to_f64()?;
        if geo.len() < 4 {
            return Err("geo needs four numbers".into());
        }
        let quality = match arrays.get("quality") {
            Some(q) => q.to_f64()?.into_iter().map(|v| v as u8).collect(),
            None => vec![MEASURED; rows * cols],
        };
        let source = arrays.get("source").and_then(npy::Array::to_string).unwrap_or_else(|| fallback_name.to_string());
        Ok(DepthGrid::new(depth, rows, cols, geo[0], geo[1], geo[2], geo[3]).with_quality(quality).with_source(source))
    }

    /// The chart that is built into the program.
    pub fn bundled() -> Result<DepthGrid, String> {
        DepthGrid::from_npz_bytes(crate::BUNDLED_CHART, "riga_bathy.npz")
    }

    /// From a terrain model where the sea floor is negative and land positive.
    pub fn from_elevation(
        elevation: &[f32],
        rows: usize,
        cols: usize,
        north: f64,
        west: f64,
        dlat: f64,
        dlon: f64,
    ) -> DepthGrid {
        let depth = elevation.iter().map(|&e| if e < 0.0 { -e } else { f32::NAN }).collect();
        DepthGrid::new(depth, rows, cols, north, west, dlat, dlon)
    }

    /// ESRI ASCII grid, in geographic degrees, values as *elevation*.
    pub fn load_ascii_grid(path: &Path) -> Result<DepthGrid, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let mut header: HashMap<String, f64> = HashMap::new();
        let mut values: Vec<f32> = Vec::new();
        let mut in_header = true;
        for line in text.lines() {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }
            if in_header && header.len() < 6 && parts.len() == 2 && parts[0].chars().next().is_some_and(char::is_alphabetic) {
                let value = parts[1].parse().map_err(|_| format!("bad header line {line:?}"))?;
                header.insert(parts[0].to_lowercase(), value);
                continue;
            }
            in_header = false;
            for p in parts {
                values.push(p.parse().map_err(|_| format!("bad value {p:?}"))?);
            }
        }
        let get = |k: &str| header.get(k).copied();
        let cell = get("cellsize").ok_or("no cellsize")?;
        let rows = get("nrows").ok_or("no nrows")? as usize;
        let cols = get("ncols").ok_or("no ncols")? as usize;
        if values.len() < rows * cols {
            return Err(format!("expected {} values, found {}", rows * cols, values.len()));
        }
        values.truncate(rows * cols);
        if let Some(nodata) = get("nodata_value") {
            for v in &mut values {
                if (*v as f64) == nodata {
                    *v = f32::NAN;
                }
            }
        }
        let south = get("yllcorner").unwrap_or_else(|| get("yllcenter").unwrap_or(0.0) - cell / 2.0);
        let west = get("xllcorner").unwrap_or_else(|| get("xllcenter").unwrap_or(0.0) - cell / 2.0);
        Ok(DepthGrid::from_elevation(&values, rows, cols, south + rows as f64 * cell, west, cell, cell)
            .with_source(file_name(path)))
    }

    /// Scattered soundings `lon, lat, depth` binned onto a grid.
    ///
    /// Depth is taken as positive down, which is what survey exports (and
    /// [`SoundingGrid::export_xyz`]) write.
    pub fn load_xyz(path: &Path, cell_deg: f64) -> Result<DepthGrid, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let sample: String = text.chars().take(2048).collect();
        let delimiter = [',', ';', '\t'].into_iter().find(|d| sample.contains(*d));
        let mut points: Vec<(f64, f64, f64)> = Vec::new();
        for line in text.lines() {
            let fields: Vec<&str> = match delimiter {
                Some(d) => line.split(d).collect(),
                None => line.split_whitespace().collect(),
            };
            if fields.len() < 3 {
                continue;
            }
            let parsed: Option<Vec<f64>> = fields[..3].iter().map(|f| f.trim().parse().ok()).collect();
            if let Some(p) = parsed {
                points.push((p[0], p[1], p[2]));
            }
        }
        if points.is_empty() {
            return Err(format!("no soundings in {}", path.display()));
        }
        let min_lon = points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_lon = points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        let min_lat = points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let max_lat = points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        let (west, north) = (min_lon - cell_deg, max_lat + cell_deg);
        let cols = ((max_lon - west) / cell_deg) as usize + 2;
        let rows = ((north - min_lat) / cell_deg) as usize + 2;
        let mut total = vec![0.0f64; rows * cols];
        let mut count = vec![0u32; rows * cols];
        for (lon, lat, depth) in points {
            let r = (((north - lat) / cell_deg) as usize).min(rows - 1);
            let c = (((lon - west) / cell_deg) as usize).min(cols - 1);
            total[r * cols + c] += depth;
            count[r * cols + c] += 1;
        }
        let depth = total
            .iter()
            .zip(&count)
            .map(|(t, &n)| if n > 0 { (t / n as f64) as f32 } else { f32::NAN })
            .collect();
        Ok(DepthGrid::new(depth, rows, cols, north, west, cell_deg, cell_deg).with_source(file_name(path)))
    }

    /// A single-band float GeoTIFF in EPSG:4326 holding *elevation*.
    pub fn load_geotiff(path: &Path) -> Result<DepthGrid, String> {
        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let tiff = read_geotiff(&bytes)?;
        Ok(DepthGrid::from_elevation(&tiff.values, tiff.rows, tiff.cols, tiff.north, tiff.west, tiff.dlat, tiff.dlon)
            .with_source(file_name(path)))
    }
}

fn file_name(path: &Path) -> String {
    path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default()
}

// --------------------------------------------------------------------------
// GeoTIFF
// --------------------------------------------------------------------------

/// What [`read_geotiff`] returns.
pub struct GeoTiff {
    pub values: Vec<f32>,
    pub rows: usize,
    pub cols: usize,
    pub north: f64,
    pub west: f64,
    pub dlat: f64,
    pub dlon: f64,
}

struct Tiff<'a> {
    blob: &'a [u8],
    big: bool,
}

impl Tiff<'_> {
    fn u16(&self, at: usize) -> Result<u16, String> {
        let b = self.blob.get(at..at + 2).ok_or("TIFF truncated")?;
        Ok(if self.big { u16::from_be_bytes([b[0], b[1]]) } else { u16::from_le_bytes([b[0], b[1]]) })
    }

    fn u32(&self, at: usize) -> Result<u32, String> {
        let b = self.blob.get(at..at + 4).ok_or("TIFF truncated")?;
        let a = [b[0], b[1], b[2], b[3]];
        Ok(if self.big { u32::from_be_bytes(a) } else { u32::from_le_bytes(a) })
    }

    fn u64(&self, at: usize) -> Result<u64, String> {
        let b = self.blob.get(at..at + 8).ok_or("TIFF truncated")?;
        let mut a = [0u8; 8];
        a.copy_from_slice(b);
        Ok(if self.big { u64::from_be_bytes(a) } else { u64::from_le_bytes(a) })
    }
}

#[derive(Clone)]
struct Tag {
    kind: u16,
    count: usize,
    /// Where the value bytes are in the file.
    at: usize,
}

/// Read an uncompressed, single-band, 32-bit float GeoTIFF.
///
/// Enough of TIFF to read what map servers such as EMODnet's WCS return,
/// without pulling in GDAL.
pub fn read_geotiff(blob: &[u8]) -> Result<GeoTiff, String> {
    let big = match blob.get(..2) {
        Some(b"II") => false,
        Some(b"MM") => true,
        _ => return Err("not a TIFF file".into()),
    };
    let t = Tiff { blob, big };
    let offset = t.u32(4)? as usize;
    let count = t.u16(offset)? as usize;
    let size_of = |kind: u16| match kind {
        1 | 2 | 6 | 7 => 1,
        3 | 8 => 2,
        4 | 9 | 11 => 4,
        5 | 10 | 12 | 16 => 8,
        _ => 1,
    };
    let mut tags: HashMap<u16, Tag> = HashMap::new();
    for index in 0..count {
        let entry = offset + 2 + 12 * index;
        let tag = t.u16(entry)?;
        let kind = t.u16(entry + 2)?;
        let number = t.u32(entry + 4)? as usize;
        let bytes = size_of(kind) * number;
        let at = if bytes <= 4 { entry + 8 } else { t.u32(entry + 8)? as usize };
        tags.insert(tag, Tag { kind, count: number, at });
    }
    let ints = |tag: u16| -> Result<Vec<u64>, String> {
        let Some(e) = tags.get(&tag) else { return Ok(Vec::new()) };
        (0..e.count)
            .map(|i| match e.kind {
                1 => blob.get(e.at + i).map(|&b| b as u64).ok_or_else(|| "TIFF truncated".to_string()),
                3 => t.u16(e.at + 2 * i).map(u64::from),
                4 => t.u32(e.at + 4 * i).map(u64::from),
                16 => t.u64(e.at + 8 * i),
                _ => Err(format!("unexpected TIFF tag type {}", e.kind)),
            })
            .collect()
    };
    let doubles = |tag: u16| -> Option<Vec<f64>> {
        let e = tags.get(&tag)?;
        if e.kind != 12 {
            return None;
        }
        (0..e.count).map(|i| t.u64(e.at + 8 * i).ok().map(f64::from_bits)).collect()
    };
    let first = |tag: u16, default: u64| ints(tag).ok().and_then(|v| v.first().copied()).unwrap_or(default);

    let width = first(256, 0) as usize;
    let height = first(257, 0) as usize;
    if width == 0 || height == 0 {
        return Err("TIFF has no size".into());
    }
    if first(259, 1) != 1 {
        return Err("compressed TIFFs are not supported; request an uncompressed one".into());
    }
    if first(258, 32) != 32 || first(339, 3) != 3 {
        return Err("only 32-bit float TIFFs are supported".into());
    }
    let read_f32 = |at: usize| -> f32 {
        t.u32(at).map(f32::from_bits).unwrap_or(f32::NAN)
    };
    let mut values = vec![f32::NAN; width * height];
    if tags.contains_key(&322) {
        let tile_w = first(322, 0) as usize;
        let tile_h = first(323, 0) as usize;
        let offsets = ints(324)?;
        let across = width.div_ceil(tile_w.max(1));
        for (index, &start) in offsets.iter().enumerate() {
            let (row, col) = (index / across, index % across);
            let (y0, x0) = (row * tile_h, col * tile_w);
            for ty in 0..tile_h.min(height.saturating_sub(y0)) {
                for tx in 0..tile_w.min(width.saturating_sub(x0)) {
                    values[(y0 + ty) * width + x0 + tx] = read_f32(start as usize + 4 * (ty * tile_w + tx));
                }
            }
        }
    } else {
        let offsets = ints(273)?;
        let counts = ints(279)?;
        let mut i = 0;
        'strips: for (&start, &n) in offsets.iter().zip(&counts) {
            for k in 0..(n as usize / 4) {
                if i >= values.len() {
                    break 'strips;
                }
                values[i] = read_f32(start as usize + 4 * k);
                i += 1;
            }
        }
    }
    // GDAL_NODATA is stored as text.
    if let Some(e) = tags.get(&42113) {
        if e.kind == 2 {
            let text = String::from_utf8_lossy(&blob[e.at..(e.at + e.count).min(blob.len())]);
            if let Ok(nodata) = text.trim_end_matches('\0').trim().parse::<f32>() {
                for v in &mut values {
                    if *v == nodata {
                        *v = f32::NAN;
                    }
                }
            }
        }
    }
    let (dlon, west, dlat, north) = if let Some(m) = doubles(34264) {
        (m[0], m[3], -m[5], m[7])
    } else if let (Some(scale), Some(tie)) = (doubles(33550), doubles(33922)) {
        let (dlon, dlat) = (scale[0], scale[1]);
        (dlon, tie[3] - tie[0] * dlon, dlat, tie[4] + tie[1] * dlat)
    } else {
        return Err("TIFF has no georeferencing".into());
    };
    Ok(GeoTiff { values, rows: height, cols: width, north, west, dlat, dlon })
}

// --------------------------------------------------------------------------
// The survey
// --------------------------------------------------------------------------

/// Cells per side of one chunk of the sounding grid.  The map caches one
/// image per chunk and redraws only chunks whose cells changed.
pub const CHUNK: i64 = 64;

pub type Cell = (i64, i64);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sounding {
    pub t: f64,
    pub lat: f64,
    pub lon: f64,
    pub depth_m: f64,
}

/// Echo-sounder readings, binned into square cells of `cell_m` metres.
///
/// Cells are indexed in a local tangent plane about `origin`, so a cell is
/// the same size anywhere in the survey.  Each holds a running mean, and the
/// raw soundings are kept as well for export.
#[derive(Clone, Debug)]
pub struct SoundingGrid {
    pub origin: LatLon,
    pub cell_m: f64,
    cells: HashMap<Cell, (f64, u32)>,
    by_chunk: HashMap<Cell, HashSet<Cell>>,
    pub soundings: Vec<Sounding>,
    /// Chunks whose cells changed since the map last looked.
    pub dirty_chunks: HashSet<Cell>,
    pub min_depth: f64,
    pub max_depth: f64,
}

impl SoundingGrid {
    pub fn new(origin: LatLon, cell_m: f64) -> Self {
        SoundingGrid {
            origin,
            cell_m,
            cells: HashMap::new(),
            by_chunk: HashMap::new(),
            soundings: Vec::new(),
            dirty_chunks: HashSet::new(),
            min_depth: f64::INFINITY,
            max_depth: 0.0,
        }
    }

    /// How many cells have been sounded.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    pub fn cell_of(&self, lat: f64, lon: f64) -> Cell {
        let local = geo::to_metres(self.origin, LatLon::new(lat, lon));
        ((local.east / self.cell_m).floor() as i64, (local.north / self.cell_m).floor() as i64)
    }

    pub fn cell_centre(&self, cell: Cell) -> LatLon {
        geo::to_lat_lon(
            self.origin,
            Metres::new((cell.0 as f64 + 0.5) * self.cell_m, (cell.1 as f64 + 0.5) * self.cell_m),
        )
    }

    fn chunk_of(cell: Cell) -> Cell {
        (cell.0.div_euclid(CHUNK), cell.1.div_euclid(CHUNK))
    }

    /// Record a sounding; credit it to every cell within half a swath.
    /// Returns how many cells were seen for the first time.
    pub fn add(&mut self, lat: f64, lon: f64, depth_m: f64, swath_m: f64, t: Option<f64>) -> usize {
        if depth_m.is_nan() {
            return 0;
        }
        self.soundings.push(Sounding { t: t.unwrap_or_else(clock::unix_time), lat, lon, depth_m });
        self.min_depth = self.min_depth.min(depth_m);
        self.max_depth = self.max_depth.max(depth_m);
        let centre = self.cell_of(lat, lon);
        let reach = ((swath_m / 2.0).max(0.0) / self.cell_m).floor() as i64;
        let mut new = 0;
        for dx in -reach..=reach {
            for dy in -reach..=reach {
                if dx * dx + dy * dy > reach * reach + reach {
                    continue;
                }
                let cell = (centre.0 + dx, centre.1 + dy);
                let chunk = Self::chunk_of(cell);
                let entry = self.cells.entry(cell).or_insert_with(|| {
                    new += 1;
                    (0.0, 0)
                });
                entry.0 += depth_m;
                entry.1 += 1;
                self.by_chunk.entry(chunk).or_default().insert(cell);
                self.dirty_chunks.insert(chunk);
            }
        }
        new
    }

    pub fn depth_of(&self, cell: Cell) -> Option<f64> {
        self.cells.get(&cell).filter(|(_, n)| *n > 0).map(|(sum, n)| sum / *n as f64)
    }

    pub fn count_of(&self, cell: Cell) -> u32 {
        self.cells.get(&cell).map(|c| c.1).unwrap_or(0)
    }

    pub fn cells(&self) -> impl Iterator<Item = (Cell, f64)> + '_ {
        self.cells.iter().map(|(cell, (sum, n))| (*cell, sum / *n as f64))
    }

    /// Cells of one chunk as `(column, row-from-south, depth)` within it.
    pub fn chunk_cells(&self, chunk: Cell) -> Vec<(usize, usize, f64)> {
        let (x0, y0) = (chunk.0 * CHUNK, chunk.1 * CHUNK);
        self.by_chunk
            .get(&chunk)
            .map(|cells| {
                cells
                    .iter()
                    .filter_map(|c| self.depth_of(*c).map(|d| ((c.0 - x0) as usize, (c.1 - y0) as usize, d)))
                    .collect()
            })
            .unwrap_or_default()
    }

    pub fn chunks(&self) -> HashSet<Cell> {
        self.by_chunk.keys().copied().collect()
    }

    /// South-west and north-east corners of a chunk.
    pub fn chunk_bounds(&self, chunk: Cell) -> (LatLon, LatLon) {
        let size = CHUNK as f64 * self.cell_m;
        let sw = geo::to_lat_lon(self.origin, Metres::new(chunk.0 as f64 * size, chunk.1 as f64 * size));
        let ne = geo::to_lat_lon(self.origin, Metres::new((chunk.0 + 1) as f64 * size, (chunk.1 + 1) as f64 * size));
        (sw, ne)
    }

    /// Fraction of a rectangle that has been sounded.
    pub fn coverage(&self, rect: Option<&Rectangle>) -> f64 {
        let Some(rect) = rect else { return 0.0 };
        if rect.area_m2() <= 0.0 {
            return 0.0;
        }
        let total = (rect.area_m2() / (self.cell_m * self.cell_m)).max(1.0);
        let inside = self.cells.keys().filter(|c| rect.contains(self.cell_centre(**c))).count();
        (inside as f64 / total).min(1.0)
    }

    pub fn clear(&mut self) {
        let chunks = self.chunks();
        self.dirty_chunks.extend(chunks);
        self.cells.clear();
        self.by_chunk.clear();
        self.soundings.clear();
        self.min_depth = f64::INFINITY;
        self.max_depth = 0.0;
    }

    /// Raw soundings as `lon,lat,depth_m,unix_time`.  Returns the row count.
    pub fn export_xyz(&self, path: &Path) -> std::io::Result<usize> {
        let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
        writeln!(out, "lon,lat,depth_m,time")?;
        for s in &self.soundings {
            writeln!(out, "{:.7},{:.7},{:.2},{:.2}", s.lon, s.lat, s.depth_m, s.t)?;
        }
        out.flush()?;
        Ok(self.soundings.len())
    }

    /// The binned grid: one row per cell, `lon,lat,depth_m,count`.
    pub fn export_grid_csv(&self, path: &Path) -> std::io::Result<usize> {
        let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
        writeln!(out, "lon,lat,depth_m,count")?;
        let mut cells: Vec<_> = self.cells().collect();
        cells.sort_by_key(|(c, _)| (-c.1, c.0));
        for (cell, depth) in &cells {
            let centre = self.cell_centre(*cell);
            writeln!(out, "{:.7},{:.7},{:.2},{}", centre.lon, centre.lat, depth, self.count_of(*cell))?;
        }
        out.flush()?;
        Ok(cells.len())
    }
}

// --------------------------------------------------------------------------
// Contours
// --------------------------------------------------------------------------

/// Marching squares over the chart: the `level` isobath as line segments.
///
/// Returns `[lon0, lat0, lon1, lat1]` per segment.  Segments are not joined
/// into lines; the map only strokes them, and unjoined is enough.  Land
/// counts as zero depth, so the zero contour is the coastline.
pub fn contour_segments(grid: &DepthGrid, level: f64) -> Vec<[f64; 4]> {
    let z = |r: usize, c: usize| -> f64 {
        let v = grid.at(r, c);
        if v.is_nan() { 0.0 } else { v as f64 }
    };
    let lerp = |v0: f64, v1: f64| -> f64 {
        let t = (level - v0) / (v1 - v0);
        if t.is_nan() { 0.5 } else { t.clamp(0.0, 1.0) }
    };
    let mut out = Vec::new();
    if grid.rows < 2 || grid.cols < 2 {
        return out;
    }
    for r in 0..grid.rows - 1 {
        for c in 0..grid.cols - 1 {
            let (a, b, cc, d) = (z(r, c), z(r, c + 1), z(r + 1, c + 1), z(r + 1, c));
            let index = (a > level) as u8
                | ((b > level) as u8) << 1
                | ((cc > level) as u8) << 2
                | ((d > level) as u8) << 3;
            if index == 0 || index == 15 {
                continue;
            }
            let (rf, cf) = (r as f64, c as f64);
            // Edge crossings in (row, col) grid space.
            let edge = |e: u8| -> (f64, f64) {
                match e {
                    b't' => (rf, cf + lerp(a, b)),
                    b'r' => (rf + lerp(b, cc), cf + 1.0),
                    b'b' => (rf + 1.0, cf + lerp(d, cc)),
                    _ => (rf + lerp(a, d), cf),
                }
            };
            let pairs: &[[u8; 2]] = match index {
                1 => &[*b"lt"],
                2 => &[*b"tr"],
                3 => &[*b"lr"],
                4 => &[*b"rb"],
                5 => &[*b"lt", *b"rb"],
                6 => &[*b"tb"],
                7 => &[*b"lb"],
                8 => &[*b"bl"],
                9 => &[*b"tb"],
                10 => &[*b"tr", *b"bl"],
                11 => &[*b"rb"],
                12 => &[*b"lr"],
                13 => &[*b"tr"],
                14 => &[*b"lt"],
                _ => &[],
            };
            for pair in pairs {
                let (r0, c0) = edge(pair[0]);
                let (r1, c1) = edge(pair[1]);
                out.push([
                    grid.west + (c0 + 0.5) * grid.dlon,
                    grid.north - (r0 + 0.5) * grid.dlat,
                    grid.west + (c1 + 0.5) * grid.dlon,
                    grid.north - (r1 + 0.5) * grid.dlat,
                ]);
            }
        }
    }
    out
}
