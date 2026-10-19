//! Build an offline depth chart.
//!
//! Fetches a piece of the EMODnet Bathymetry digital terrain model while
//! there is an internet connection, and writes it as a compact `.npz` the
//! ground station loads with no network:
//!
//! ```text
//! usvgs chart --bbox 23.40,56.90,24.60,57.45 --out maps/riga.npz
//! usvgs chart --tif downloaded.tif --out maps/riga.npz
//! ```
//!
//! EMODnet covers European seas at 1/16 arc-minute (about 115 m).  Inland
//! lakes carry a flat placeholder rather than soundings, so this replaces
//! those with an *estimate* that deepens with distance from the shore, and
//! flags those cells as estimated so the interface can say so.

use std::io::Read;
use std::path::Path;

use crate::core::bathymetry::{read_geotiff, DepthGrid, ESTIMATED, MEASURED};

/// EMODnet's native resolution, degrees.
pub const NATIVE_RES: f64 = 1.0 / 960.0;

pub fn wcs_url(bbox: (f64, f64, f64, f64), res: f64) -> String {
    let (w, s, e, n) = bbox;
    format!(
        "https://ows.emodnet-bathymetry.eu/wcs?service=wcs&version=1.0.0&request=getcoverage\
         &coverage=emodnet:mean&crs=EPSG:4326&BBOX={w},{s},{e},{n}&format=image/tiff\
         &interpolation=nearest&resx={res}&resy={res}"
    )
}

pub fn fetch(bbox: (f64, f64, f64, f64), path: &Path, res: f64) -> Result<(), String> {
    let url = wcs_url(bbox, res);
    eprintln!("[*] fetching {url}");
    let response = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(180))
        .call()
        .map_err(|e| format!("download failed: {e}"))?;
    let mut bytes = Vec::new();
    response.into_reader().read_to_end(&mut bytes).map_err(|e| e.to_string())?;
    std::fs::write(path, bytes).map_err(|e| e.to_string())
}

/// Cells from each `true` cell to the nearest `false` one, capped at `limit`.
fn erode_distance(mask: &[bool], rows: usize, cols: usize, limit: usize) -> Vec<f32> {
    let mut distance = vec![0.0f32; mask.len()];
    let mut current = mask.to_vec();
    for step in 1..=limit {
        let mut any = false;
        let mut shrunk = current.clone();
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                if !current[i] {
                    continue;
                }
                distance[i] = step as f32;
                // The edges of the grid count as inside.
                shrunk[i] = (r == 0 || current[i - cols])
                    && (r + 1 == rows || current[i + cols])
                    && (c == 0 || current[i - 1])
                    && (c + 1 == cols || current[i + 1]);
                any |= shrunk[i];
            }
        }
        if !any {
            break;
        }
        current = shrunk;
    }
    distance
}

/// Replace EMODnet's flat lake placeholder with a shore-distance estimate.
/// Returns depth (positive down, NaN on land) and quality.
pub fn estimate_lakes(elevation: &[f32], rows: usize, cols: usize, cell_m: f64) -> (Vec<f32>, Vec<u8>) {
    const MAX_DEPTH_M: f64 = 4.5;
    const SLOPE_PER_M: f64 = 0.012;
    let mut depth: Vec<f32> = elevation.iter().map(|&e| if e < 0.0 { -e } else { f32::NAN }).collect();
    let mut quality = vec![MEASURED; elevation.len()];
    // The placeholder is a run of identical shallow values.  Real soundings
    // vary from cell to cell, so a flat patch at exactly that value is a lake.
    let placeholder: Vec<bool> = elevation.iter().map(|&e| (e + 1.0).abs() <= 1e-4).collect();
    if !placeholder.iter().any(|p| *p) {
        return (depth, quality);
    }
    let limit = (MAX_DEPTH_M / SLOPE_PER_M / cell_m) as usize + 2;
    let distance = erode_distance(&placeholder, rows, cols, limit);
    for i in 0..elevation.len() {
        if placeholder[i] {
            depth[i] = MAX_DEPTH_M.min(0.4 + distance[i] as f64 * cell_m * SLOPE_PER_M) as f32;
            quality[i] = ESTIMATED;
        }
    }
    (depth, quality)
}

pub fn build(tif: &Path, out: &Path) -> Result<DepthGrid, String> {
    let bytes = std::fs::read(tif).map_err(|e| e.to_string())?;
    let t = read_geotiff(&bytes)?;
    let cell_m = t.dlat * 111_320.0;
    let (depth, quality) = estimate_lakes(&t.values, t.rows, t.cols, cell_m);
    let grid = DepthGrid::new(depth, t.rows, t.cols, t.north, t.west, t.dlat, t.dlon)
        .with_quality(quality)
        .with_source("EMODnet Bathymetry DTM (lakes estimated)");
    grid.save_npz(out)?;
    let water = grid.depth.iter().filter(|d| !d.is_nan()).count();
    let deepest = grid.depth.iter().filter(|d| !d.is_nan()).fold(0.0f32, |a, &b| a.max(b));
    let estimated = grid.quality.iter().filter(|q| **q == ESTIMATED).count();
    eprintln!(
        "[*] wrote {}: {}×{} cells, {:.0}% water, deepest {:.1} m, {} estimated cells",
        out.display(),
        grid.cols,
        grid.rows,
        100.0 * water as f64 / grid.depth.len().max(1) as f64,
        deepest,
        estimated
    );
    Ok(grid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lakes_deepen_away_from_the_shore() {
        // A 7×7 lake of placeholder in land, one sea cell in the corner.
        let (rows, cols) = (9, 9);
        let mut elevation = vec![5.0f32; rows * cols];
        for r in 1..8 {
            for c in 1..8 {
                elevation[r * cols + c] = -1.0;
            }
        }
        elevation[0] = -12.0;
        let (depth, quality) = estimate_lakes(&elevation, rows, cols, 115.0);
        assert_eq!(quality[4 * cols + 4], ESTIMATED);
        assert_eq!(quality[0], MEASURED);
        assert_eq!(depth[0], 12.0);
        assert!(depth[4 * cols + 4] > depth[cols + 1]);
        assert!(depth[cols * 4].is_nan());
    }
}
