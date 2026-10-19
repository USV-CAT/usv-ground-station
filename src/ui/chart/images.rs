//! The pictures behind the chart, built once on the CPU and handed to the
//! GPU as textures: depth shading, natural water colour, the survey palette,
//! the oil slick, and the loop of wave frames.

use egui::{Color32, ColorImage};

use crate::core::bathymetry::{DepthGrid, CHUNK, ESTIMATED};
use crate::core::geo;
use crate::core::rng::Rng;
use crate::ui::theme;

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn interp(x: f32, stops: &[(f32, [f32; 4])]) -> [f32; 4] {
    if x <= stops[0].0 {
        return stops[0].1;
    }
    for w in stops.windows(2) {
        let ((x0, c0), (x1, c1)) = (w[0], w[1]);
        if x <= x1 {
            let t = (x - x0) / (x1 - x0);
            return [lerp(c0[0], c1[0], t), lerp(c0[1], c1[1], t), lerp(c0[2], c1[2], t), lerp(c0[3], c1[3], t)];
        }
    }
    stops[stops.len() - 1].1
}

fn rgb(c: [f32; 3]) -> Color32 {
    Color32::from_rgb(c[0].clamp(0.0, 255.0) as u8, c[1].clamp(0.0, 255.0) as u8, c[2].clamp(0.0, 255.0) as u8)
}

/// Chart colour for a depth: bright and shallow to dark and deep, hazards hot.
pub fn depth_rgb(depth: f64, shallow_alarm: f64) -> Color32 {
    if depth < shallow_alarm {
        return rgb(theme::WATER_HAZARD);
    }
    let f = (depth / 30.0).min(1.0).powf(0.55) as f32;
    let (s, d) = (theme::WATER_SHALLOW, theme::WATER_DEEP);
    rgb([lerp(s[0], d[0], f), lerp(s[1], d[1], f), lerp(s[2], d[2], f)])
}

/// The chart grid shaded as a nautical chart colours it, with relief.
pub fn chart_shading(grid: &DepthGrid, shallow_alarm_m: f64) -> ColorImage {
    let (rows, cols) = (grid.rows, grid.cols);
    let d = |r: usize, c: usize| {
        let v = grid.at(r, c);
        if v.is_nan() { 0.0 } else { v }
    };
    let mut pixels = Vec::with_capacity(rows * cols);
    for r in 0..rows {
        for c in 0..cols {
            let depth = grid.at(r, c);
            if depth.is_nan() {
                pixels.push(theme::LAND);
                continue;
            }
            let z = depth;
            let f = (z / 30.0).clamp(0.0, 1.0).powf(0.55);
            let (s, dd) = (theme::WATER_SHALLOW, theme::WATER_DEEP);
            let mut colour = [lerp(s[0], dd[0], f), lerp(s[1], dd[1], f), lerp(s[2], dd[2], f)];
            // Relief: light from the north-west, so banks and channels read as shapes.
            let gx = if cols < 2 {
                0.0
            } else if c == 0 {
                d(r, 1) - d(r, 0)
            } else if c == cols - 1 {
                d(r, c) - d(r, c - 1)
            } else {
                (d(r, c + 1) - d(r, c - 1)) / 2.0
            };
            let gy = if rows < 2 {
                0.0
            } else if r == 0 {
                d(1, c) - d(0, c)
            } else if r == rows - 1 {
                d(r, c) - d(r - 1, c)
            } else {
                (d(r + 1, c) - d(r - 1, c)) / 2.0
            };
            let shade = (1.0 + 0.10 * (gx - gy)).clamp(0.75, 1.3);
            for v in &mut colour {
                *v *= shade;
            }
            if (z as f64) < shallow_alarm_m {
                for (v, h) in colour.iter_mut().zip(theme::WATER_HAZARD) {
                    *v = *v * 0.45 + h * 0.55;
                }
            }
            if grid.quality_cell(r, c) == ESTIMATED {
                for (v, e) in colour.iter_mut().zip([90.0, 90.0, 120.0]) {
                    *v = *v * 0.75 + e * 0.25;
                }
            }
            pixels.push(rgb(colour));
        }
    }
    ColorImage::new([cols, rows], pixels)
}

/// Colour of water over a bottom at depth d (m), from the beach outwards.
pub const DEPTH_COLOURS: [(f32, [f32; 4]); 6] = [
    (0.0, [88.0, 172.0, 160.0, 255.0]),
    (1.5, [52.0, 146.0, 152.0, 255.0]),
    (4.0, [28.0, 112.0, 138.0, 255.0]),
    (10.0, [16.0, 74.0, 110.0, 255.0]),
    (25.0, [10.0, 48.0, 82.0, 255.0]),
    (50.0, [7.0, 32.0, 60.0, 255.0]),
];

pub fn water_rgb(depth: f32) -> [f32; 3] {
    let c = interp(depth, &DEPTH_COLOURS);
    [c[0], c[1], c[2]]
}

/// Natural water: the colour of the sea over that depth, land in the land colour.
pub fn natural_water(grid: &DepthGrid, shallow_alarm_m: f64) -> ColorImage {
    let pixels = grid
        .depth
        .iter()
        .map(|&depth| {
            if depth.is_nan() {
                return theme::LAND;
            }
            let mut c = water_rgb(depth);
            if (depth as f64) < shallow_alarm_m {
                for (v, h) in c.iter_mut().zip([200.0, 120.0, 80.0]) {
                    *v = *v * 0.7 + h * 0.3;
                }
            }
            rgb(c)
        })
        .collect();
    ColorImage::new([grid.cols, grid.rows], pixels)
}

pub const SURVEY_STOPS: [(f32, [f32; 4]); 5] = [
    (0.0, [230.0, 40.0, 40.0, 255.0]),
    (0.25, [250.0, 200.0, 40.0, 255.0]),
    (0.5, [60.0, 210.0, 90.0, 255.0]),
    (0.75, [40.0, 190.0, 230.0, 255.0]),
    (1.0, [60.0, 70.0, 220.0, 255.0]),
];

/// The multibeam palette: red shallow, through yellow and green, to blue deep.
pub fn survey_rgb(depth: f64, low: f64, high: f64) -> Color32 {
    let span = (high - low).max(0.5);
    let t = ((depth - low) / span).clamp(0.0, 1.0) as f32;
    let c = interp(t, &SURVEY_STOPS);
    rgb([c[0], c[1], c[2]])
}

/// One survey chunk: `(column, row-from-south, depth)` cells into a CHUNK² image.
pub fn survey_chunk(cells: &[(usize, usize, f64)], low: f64, high: f64) -> ColorImage {
    let n = CHUNK as usize;
    let mut pixels = vec![Color32::TRANSPARENT; n * n];
    for &(x, y, depth) in cells {
        if x < n && y < n {
            pixels[(n - 1 - y) * n + x] = survey_rgb(depth, low, high);
        }
    }
    ColorImage::new([n, n], pixels)
}

// --------------------------------------------------------------------------
// The slick
// --------------------------------------------------------------------------

/// Spread of each parcel, metres.
pub const SLICK_SIGMA_M: f64 = 2.5;
/// The slick grid is at most this many cells along its longer side.
pub const SLICK_MAX_CELLS: f64 = 220.0;

/// log10(film thickness in µm) → RGBA, from a barely-there sheen to black.
pub const FILM_STOPS: [(f32, [f32; 4]); 5] = [
    (-1.6, [150.0, 140.0, 190.0, 0.0]),
    (-0.6, [150.0, 140.0, 190.0, 70.0]),
    (0.3, [120.0, 100.0, 120.0, 105.0]),
    (1.0, [90.0, 60.0, 35.0, 150.0]),
    (1.8, [25.0, 15.0, 8.0, 210.0]),
];

pub fn colour_film(film_um: f64) -> Color32 {
    let t = film_um.max(1e-3).log10() as f32;
    let c = interp(t, &FILM_STOPS);
    let a = c[3] / 255.0;
    Color32::from_rgba_premultiplied((c[0] * a) as u8, (c[1] * a) as u8, (c[2] * a) as u8, c[3] as u8)
}

/// Gaussian blur, separable.
fn blur(grid: &mut [f64], rows: usize, cols: usize, sigma_cells: f64) {
    let radius = ((3.0 * sigma_cells) as i64).max(1);
    let mut kernel: Vec<f64> = (-radius..=radius).map(|x| (-0.5 * (x as f64 / sigma_cells).powi(2)).exp()).collect();
    let total: f64 = kernel.iter().sum();
    for k in &mut kernel {
        *k /= total;
    }
    let mut tmp = vec![0.0; grid.len()];
    for r in 0..rows {
        for c in 0..cols {
            let mut s = 0.0;
            for (k, w) in kernel.iter().enumerate() {
                let rr = r as i64 + k as i64 - radius;
                if rr >= 0 && (rr as usize) < rows {
                    s += w * grid[rr as usize * cols + c];
                }
            }
            tmp[r * cols + c] = s;
        }
    }
    for r in 0..rows {
        for c in 0..cols {
            let mut s = 0.0;
            for (k, w) in kernel.iter().enumerate() {
                let cc = c as i64 + k as i64 - radius;
                if cc >= 0 && (cc as usize) < cols {
                    s += w * tmp[r * cols + cc as usize];
                }
            }
            grid[r * cols + c] = s;
        }
    }
}

/// The floating oil as one image, coloured by film thickness, and the
/// (north, west, south, east) it covers.
pub fn slick_image(lat: &[f64], lon: &[f64], volume: &[f64]) -> Option<(ColorImage, [f64; 4])> {
    if lat.is_empty() {
        return None;
    }
    let n = lat.len() as f64;
    let lat0 = lat.iter().sum::<f64>() / n;
    let lon0 = lon.iter().sum::<f64>() / n;
    let (m_lat, m_lon) = geo::metres_per_degree(lat0);
    let north: Vec<f64> = lat.iter().map(|l| (l - lat0) * m_lat).collect();
    let east: Vec<f64> = lon.iter().map(|l| (l - lon0) * m_lon).collect();
    let margin = 4.0 * SLICK_SIGMA_M;
    let n0 = north.iter().copied().fold(f64::INFINITY, f64::min) - margin;
    let n1 = north.iter().copied().fold(f64::NEG_INFINITY, f64::max) + margin;
    let e0 = east.iter().copied().fold(f64::INFINITY, f64::min) - margin;
    let e1 = east.iter().copied().fold(f64::NEG_INFINITY, f64::max) + margin;
    let cell = ((n1 - n0).max(e1 - e0) / SLICK_MAX_CELLS).max(0.8);
    let rows = (((n1 - n0) / cell).ceil() as usize).max(2);
    let cols = (((e1 - e0) / cell).ceil() as usize).max(2);
    let (n1, e1) = (n0 + rows as f64 * cell, e0 + cols as f64 * cell);
    // Row 0 is the northern edge, like the chart.
    let mut litres = vec![0.0; rows * cols];
    for i in 0..lat.len() {
        let r = ((n1 - north[i]) / cell) as usize;
        let c = ((east[i] - e0) / cell) as usize;
        if r < rows && c < cols {
            litres[r * cols + c] += volume[i];
        }
    }
    blur(&mut litres, rows, cols, SLICK_SIGMA_M / cell);
    // 1 L/m² is a 1 mm film.
    let pixels = litres.iter().map(|l| colour_film(l / (cell * cell) * 1000.0)).collect();
    let image = ColorImage::new([cols, rows], pixels);
    Some((image, [lat0 + n1 / m_lat, lon0 + e0 / m_lon, lat0 + n0 / m_lat, lon0 + e1 / m_lon]))
}

// --------------------------------------------------------------------------
// Waves
// --------------------------------------------------------------------------

/// Wave tile: pixels a side, frames in the loop, and the loop's length.
pub const WAVE_TILE: usize = 256;
pub const WAVE_FRAMES: usize = 24;
pub const WAVE_LOOP_S: f64 = 3.0;

/// A loop of tileable ripple frames, as light and shadow over transparent
/// (premultiplied).
///
/// Each frame is a sum of travelling sine waves.  Every wave fits the tile a
/// whole number of times and goes round a whole number of times in the loop,
/// so the tile repeats in space and in time without a seam.  The waves run
/// along +x, spread about ±40°, and longer ones move faster, as on real water.
pub fn wave_frames() -> Vec<ColorImage> {
    let mut rng = Rng::new(11);
    let mut waves: Vec<(f32, f32, f32, f32, f32)> = Vec::new();
    // Each wave comes with its mirror image across the tile's x axis, so the
    // spectrum is symmetric about the wind and the crests lie square across it.
    for _ in 0..9 {
        let cycles = rng.uniform(3.0, 16.0);
        let angle = rng.gauss(0.0, 0.35).abs();
        for sign in [1.0, -1.0] {
            let kx = (cycles * angle.cos()).round();
            let ky = (sign * cycles * angle.sin()).round();
            let k = kx.hypot(ky);
            if k < 2.0 {
                continue;
            }
            // Deep-water dispersion: speed goes as the square root of
            // wavelength, so the turns per loop go as the square root of the
            // wavenumber.
            let turns = (1.6 * k.sqrt()).round().max(1.0);
            waves.push((kx as f32, ky as f32, turns as f32, k.powf(-1.3) as f32, rng.uniform(0.0, std::f64::consts::TAU) as f32));
        }
    }
    let light = [-1.0f32, 0.0f32]; // from ahead of the waves: the sky downwind
    let n = WAVE_TILE;
    let tau = std::f32::consts::TAU;
    let mut frames = Vec::with_capacity(WAVE_FRAMES);
    let mut slope = vec![0.0f32; n * n];
    let mut height = vec![0.0f32; n * n];
    for f in 0..WAVE_FRAMES {
        let t = f as f32 / WAVE_FRAMES as f32;
        for y in 0..n {
            for x in 0..n {
                let (fx, fy) = (x as f32 / n as f32, y as f32 / n as f32);
                let (mut gx, mut gy, mut h) = (0.0, 0.0, 0.0);
                for &(kx, ky, turns, amp, phase) in &waves {
                    let arg = tau * (kx * fx + ky * fy - turns * t) + phase;
                    let (s, c) = arg.sin_cos();
                    h += amp * s;
                    gx += amp * kx * c;
                    gy += amp * ky * c;
                }
                slope[y * n + x] = gx * light[0] + gy * light[1];
                height[y * n + x] = h;
            }
        }
        let mean = slope.iter().sum::<f32>() / slope.len() as f32;
        let std = (slope.iter().map(|s| (s - mean).powi(2)).sum::<f32>() / slope.len() as f32).sqrt();
        let norm = if std > 0.0 { 2.2 * std } else { 1.0 };
        let pixels = slope
            .iter()
            .zip(&height)
            .map(|(&s, &h)| {
                let s = s / norm;
                let bright = s.clamp(0.0, 1.0).powf(1.4);
                let dark = (-s).clamp(0.0, 1.0);
                // Crests catch the sky; the steepest faces flash like glints.
                let glint = s > 0.95 && h > 0.0;
                let a = if glint { 230.0 } else { (bright * 110.0).max(dark * 80.0) };
                let base = if glint { [255.0, 255.0, 255.0] } else if s > 0.0 { [215.0, 238.0, 255.0] } else { [0.0, 12.0, 26.0] };
                let k = a / 255.0;
                Color32::from_rgba_premultiplied((base[0] * k) as u8, (base[1] * k) as u8, (base[2] * k) as u8, a as u8)
            })
            .collect();
        frames.push(ColorImage::new([n, n], pixels));
    }
    frames
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn survey_palette_runs_red_to_blue() {
        let shallow = survey_rgb(1.0, 1.0, 9.0);
        let deep = survey_rgb(9.0, 1.0, 9.0);
        assert!(shallow.r() > 200 && shallow.b() < 80);
        assert!(deep.b() > 200 && deep.r() < 80);
    }

    #[test]
    fn slick_image_is_darkest_in_the_middle() {
        let mut lat = Vec::new();
        let mut lon = Vec::new();
        let mut vol = Vec::new();
        let mut rng = Rng::new(3);
        for _ in 0..500 {
            lat.push(57.1 + rng.gauss(0.0, 0.0001));
            lon.push(23.95 + rng.gauss(0.0, 0.0002));
            vol.push(0.2);
        }
        let (image, bounds) = slick_image(&lat, &lon, &vol).unwrap();
        assert!(bounds[0] > bounds[2] && bounds[3] > bounds[1]);
        let [w, h] = image.size;
        let centre = image.pixels[(h / 2) * w + w / 2];
        let corner = image.pixels[0];
        assert!(centre.a() > corner.a());
    }

    #[test]
    fn wave_frames_tile_and_move() {
        let frames = wave_frames();
        assert_eq!(frames.len(), WAVE_FRAMES);
        assert_eq!(frames[0].size, [WAVE_TILE, WAVE_TILE]);
        assert!(frames[0].pixels.iter().any(|p| p.a() > 100));
        assert_ne!(frames[0].pixels, frames[1].pixels);
    }

    /// The waves run along the tile's +x: crests lie across it, so the
    /// light changes far faster along x than along y.
    #[test]
    fn wave_frames_run_along_x() {
        let frame = &wave_frames()[0];
        let n = WAVE_TILE;
        let a = |x: usize, y: usize| frame.pixels[(y % n) * n + (x % n)].a() as f64;
        let (mut along_x, mut along_y) = (0.0, 0.0);
        for y in 0..n {
            for x in 0..n {
                along_x += (a(x + 1, y) - a(x, y)).abs();
                along_y += (a(x, y + 1) - a(x, y)).abs();
            }
        }
        assert!(along_x > 1.5 * along_y, "{along_x} vs {along_y}");
        // And square across x, not leaning: the light changes as much going
        // up-and-right as going down-and-right.
        let (mut up, mut down) = (0.0, 0.0);
        for y in 1..n {
            for x in 0..n {
                up += (a(x + 1, y - 1) - a(x, y)).abs();
                down += (a(x + 1, y + 1) - a(x, y)).abs();
            }
        }
        assert!((up / down - 1.0).abs() < 0.1, "leaning: {up} vs {down}");
    }
}
