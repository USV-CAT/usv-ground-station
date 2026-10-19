//! Build a tiled binary vector map of a whole country from a `.osm.pbf`,
//! once.  The station then loads only the tiles in view, so all of Latvia is
//! navigable offline.
//!
//! ```text
//! usvgs osm --pbf latvia-latest.osm.pbf --out maps/latvia_map [--buildings]
//! ```
//!
//! Output: `<out>.idx.json`, the tile index
//! `{tile, bbox, layers, bin, tiles: {"ix_iy": {layer: [offset, length]}}}`,
//! and `<out>.bin`, the tile blocks.  A block is repeated
//! `[u16 npts | npts × (i32 lat_e7, i32 lon_e7)]`, little-endian.

use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde_json::{json, Map, Value};

use super::osm_pbf::{for_each_primitive, Primitive};

pub const LAYERS: [&str; 6] = ["water", "landuse", "rail", "road_minor", "road_major", "building"];

pub fn classify(tags: &HashMap<&str, &str>) -> Option<&'static str> {
    let get = |k: &str| tags.get(k).copied();
    if let Some(hw) = get("highway") {
        if ["motorway", "trunk", "primary", "secondary", "tertiary", "motorway_link", "trunk_link", "primary_link"]
            .contains(&hw)
        {
            return Some("road_major");
        }
        if ["residential", "unclassified", "service", "living_street", "road", "track", "pedestrian", "footway", "path", "cycleway", "steps"]
            .contains(&hw)
        {
            return Some("road_minor");
        }
    }
    if get("railway").is_some_and(|r| ["rail", "light_rail", "subway", "tram", "narrow_gauge"].contains(&r)) {
        return Some("rail");
    }
    if get("natural").is_some_and(|n| n == "water" || n == "coastline")
        || get("waterway").is_some_and(|w| ["river", "riverbank", "stream", "canal", "drain"].contains(&w))
        || get("water").is_some_and(|w| !w.is_empty())
        || get("landuse") == Some("reservoir")
    {
        return Some("water");
    }
    if tags.contains_key("building") {
        return Some("building");
    }
    if get("landuse").is_some_and(|l| {
        ["forest", "grass", "meadow", "farmland", "residential", "industrial", "cemetery", "orchard", "vineyard"].contains(&l)
    }) || get("natural").is_some_and(|n| ["wood", "scrub", "grassland", "heath", "wetland"].contains(&n))
        || get("leisure").is_some_and(|l| ["park", "garden", "pitch", "golf_course"].contains(&l))
    {
        return Some("landuse");
    }
    None
}

pub fn build(pbf: &Path, out_prefix: &Path, tile: f64, with_buildings: bool) -> Result<(), String> {
    let t0 = Instant::now();
    let layer_index = |name: &str| LAYERS.iter().position(|l| *l == name).unwrap();

    eprintln!("[*] pass 1/2: ways…");
    let mut ways: Vec<(usize, Vec<i64>)> = Vec::new();
    let mut needed: HashSet<i64> = HashSet::new();
    for_each_primitive(pbf, false, true, |p| {
        if let Primitive::Way { refs, tags, .. } = p {
            let Some(layer) = classify(tags) else { return };
            if (layer == "building" && !with_buildings) || refs.len() < 2 {
                return;
            }
            needed.extend(refs.iter().copied());
            ways.push((layer_index(layer), refs.to_vec()));
        }
    })?;
    eprintln!("    {} ways, {} needed nodes, {:.0}s", ways.len(), needed.len(), t0.elapsed().as_secs_f64());

    eprintln!("[*] pass 2/2: node coords…");
    let mut coords: HashMap<i64, (i32, i32)> = HashMap::with_capacity(needed.len());
    for_each_primitive(pbf, true, false, |p| {
        if let Primitive::Node { id, lat, lon } = p {
            if needed.contains(&id) {
                coords.insert(id, ((lat * 1e7).round() as i32, (lon * 1e7).round() as i32));
            }
        }
    })?;
    drop(needed);
    eprintln!("    resolved {} coords, {:.0}s", coords.len(), t0.elapsed().as_secs_f64());

    eprintln!("[*] assembling tiles…");
    let inv = 1.0 / tile;
    let mut tiles: HashMap<(i64, i64), HashMap<usize, Vec<u8>>> = HashMap::new();
    let (mut minla, mut minlo, mut maxla, mut maxlo) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
    let mut emit = |layer: usize, pts: &[(i32, i32)]| {
        for chunk in pts.chunks(u16::MAX as usize) {
            if chunk.len() < 2 {
                continue;
            }
            let mut blob = Vec::with_capacity(2 + 8 * chunk.len());
            blob.extend_from_slice(&(chunk.len() as u16).to_le_bytes());
            let mut seen = HashSet::new();
            for &(a, b) in chunk {
                blob.extend_from_slice(&a.to_le_bytes());
                blob.extend_from_slice(&b.to_le_bytes());
                seen.insert(((a as f64 * 1e-7 * inv).floor() as i64, (b as f64 * 1e-7 * inv).floor() as i64));
                minla = minla.min(a);
                maxla = maxla.max(a);
                minlo = minlo.min(b);
                maxlo = maxlo.max(b);
            }
            for key in seen {
                tiles.entry(key).or_default().entry(layer).or_default().extend_from_slice(&blob);
            }
        }
    };
    for (layer, refs) in &ways {
        let mut line: Vec<(i32, i32)> = Vec::new();
        for r in refs {
            match coords.get(r) {
                Some(c) => line.push(*c),
                None => {
                    if line.len() >= 2 {
                        emit(*layer, &line);
                    }
                    line.clear();
                }
            }
        }
        if line.len() >= 2 {
            emit(*layer, &line);
        }
    }
    drop(ways);
    drop(coords);

    if let Some(dir) = out_prefix.parent() {
        if !dir.as_os_str().is_empty() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
    }
    let bin_path = PathBuf::from(format!("{}.bin", out_prefix.display()));
    let idx_path = PathBuf::from(format!("{}.idx.json", out_prefix.display()));
    let mut bin = std::io::BufWriter::new(std::fs::File::create(&bin_path).map_err(|e| e.to_string())?);
    let mut index_tiles = Map::new();
    let mut offset = 0usize;
    for ((ix, iy), layers) in &tiles {
        let mut record = Map::new();
        for (layer, buf) in layers {
            bin.write_all(buf).map_err(|e| e.to_string())?;
            record.insert(LAYERS[*layer].to_string(), json!([offset, buf.len()]));
            offset += buf.len();
        }
        index_tiles.insert(format!("{ix}_{iy}"), Value::Object(record));
    }
    bin.flush().map_err(|e| e.to_string())?;
    let index = json!({
        "version": 1,
        "tile": tile,
        "bbox": [minla as f64 * 1e-7, minlo as f64 * 1e-7, maxla as f64 * 1e-7, maxlo as f64 * 1e-7],
        "layers": LAYERS,
        "bin": bin_path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default(),
        "tiles": index_tiles,
    });
    std::fs::write(&idx_path, serde_json::to_string(&index).unwrap()).map_err(|e| e.to_string())?;
    eprintln!(
        "[*] wrote {} ({} tiles) and {} ({} MB) in {:.0}s",
        idx_path.display(),
        tiles.len(),
        bin_path.display(),
        offset / 1024 / 1024,
        t0.elapsed().as_secs_f64()
    );
    Ok(())
}
