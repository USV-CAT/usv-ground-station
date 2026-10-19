//! A minimal OSM PBF reader: enough of the format to stream nodes and ways
//! out of a `.osm.pbf` to build an offline vector map.  Not a general OSM
//! library.
//!
//! ```text
//! PBF layout:  repeated [ uint32 BE header-len | BlobHeader | Blob ]
//!   BlobHeader {1:type(str) 3:datasize(int32)}
//!   Blob       {1:raw 2:raw_size 3:zlib_data}   -> PrimitiveBlock (zlib)
//!   PrimitiveBlock {1:StringTable 2:PrimitiveGroup* 17:granularity 19:lat_off 20:lon_off}
//!   PrimitiveGroup {2:DenseNodes 3:Way*}
//!   DenseNodes {1:id(sint64,delta,packed) 8:lat 9:lon (delta,packed) 10:keys_vals}
//!   Way {1:id 2:keys(packed) 3:vals(packed) 8:refs(sint64,delta,packed)}
//! coord(deg) = 1e-9 * (offset + granularity * value)
//! ```

use std::collections::HashMap;
use std::io::{BufReader, Read};
use std::path::Path;

pub enum Primitive<'a> {
    Node { id: i64, lat: f64, lon: f64 },
    Way { id: i64, refs: &'a [i64], tags: &'a HashMap<&'a str, &'a str> },
}

fn varint(b: &[u8], p: &mut usize) -> Result<u64, String> {
    let mut result = 0u64;
    let mut shift = 0;
    loop {
        let c = *b.get(*p).ok_or("truncated varint")?;
        *p += 1;
        result |= ((c & 0x7F) as u64) << shift;
        if c & 0x80 == 0 {
            return Ok(result);
        }
        shift += 7;
        if shift > 63 {
            return Err("varint too long".into());
        }
    }
}

fn zigzag(n: u64) -> i64 {
    ((n >> 1) as i64) ^ -((n & 1) as i64)
}

fn packed(b: &[u8]) -> Result<Vec<u64>, String> {
    let mut p = 0;
    let mut out = Vec::with_capacity(b.len());
    while p < b.len() {
        out.push(varint(b, &mut p)?);
    }
    Ok(out)
}

enum Value<'a> {
    Int(u64),
    Bytes(&'a [u8]),
}

/// (field number, value) of a protobuf message.
fn fields(b: &[u8]) -> Result<Vec<(u64, Value<'_>)>, String> {
    let mut p = 0;
    let mut out = Vec::new();
    while p < b.len() {
        let tag = varint(b, &mut p)?;
        let (field, wire) = (tag >> 3, tag & 7);
        match wire {
            0 => out.push((field, Value::Int(varint(b, &mut p)?))),
            2 => {
                let len = varint(b, &mut p)? as usize;
                let bytes = b.get(p..p + len).ok_or("truncated field")?;
                out.push((field, Value::Bytes(bytes)));
                p += len;
            }
            1 => p += 8,
            5 => p += 4,
            _ => return Err(format!("bad wire type {wire}")),
        }
    }
    Ok(out)
}

/// Every decompressed PrimitiveBlock, header block skipped.
pub fn for_each_block(path: &Path, mut f: impl FnMut(&[u8]) -> Result<(), String>) -> Result<(), String> {
    let mut file = BufReader::new(std::fs::File::open(path).map_err(|e| e.to_string())?);
    loop {
        let mut len = [0u8; 4];
        if file.read_exact(&mut len).is_err() {
            return Ok(());
        }
        let mut header = vec![0u8; u32::from_be_bytes(len) as usize];
        file.read_exact(&mut header).map_err(|e| e.to_string())?;
        let (mut kind, mut size) = (String::new(), 0usize);
        for (field, v) in fields(&header)? {
            match (field, v) {
                (1, Value::Bytes(b)) => kind = String::from_utf8_lossy(b).to_string(),
                (3, Value::Int(n)) => size = n as usize,
                _ => {}
            }
        }
        let mut blob = vec![0u8; size];
        file.read_exact(&mut blob).map_err(|e| e.to_string())?;
        let mut raw: Option<Vec<u8>> = None;
        for (field, v) in fields(&blob)? {
            match (field, v) {
                (1, Value::Bytes(b)) => raw = Some(b.to_vec()),
                (3, Value::Bytes(b)) => {
                    let mut out = Vec::new();
                    flate2::read::ZlibDecoder::new(b).read_to_end(&mut out).map_err(|e| e.to_string())?;
                    raw = Some(out);
                }
                _ => {}
            }
        }
        if kind == "OSMData" {
            if let Some(raw) = raw {
                f(&raw)?;
            }
        }
    }
}

/// Stream the nodes and/or ways of a file.  Coordinates in degrees.
pub fn for_each_primitive(
    path: &Path,
    want_nodes: bool,
    want_ways: bool,
    mut f: impl FnMut(Primitive<'_>),
) -> Result<(), String> {
    for_each_block(path, |block| {
        let mut strings: Vec<&str> = Vec::new();
        let mut groups: Vec<&[u8]> = Vec::new();
        let (mut granularity, mut lat_off, mut lon_off) = (100i64, 0i64, 0i64);
        for (field, v) in fields(block)? {
            match (field, v) {
                (1, Value::Bytes(table)) => {
                    strings = fields(table)?
                        .into_iter()
                        .filter_map(|(f, v)| match (f, v) {
                            (1, Value::Bytes(s)) => Some(std::str::from_utf8(s).unwrap_or("")),
                            _ => None,
                        })
                        .collect();
                }
                (2, Value::Bytes(g)) => groups.push(g),
                (17, Value::Int(n)) => granularity = n as i64,
                (19, Value::Int(n)) => lat_off = n as i64,
                (20, Value::Int(n)) => lon_off = n as i64,
                _ => {}
            }
        }
        let string = |i: u64| strings.get(i as usize).copied().unwrap_or("");
        for group in groups {
            for (field, v) in fields(group)? {
                let Value::Bytes(body) = v else { continue };
                if field == 2 && want_nodes {
                    let (mut ids, mut lats, mut lons) = (Vec::new(), Vec::new(), Vec::new());
                    for (df, dv) in fields(body)? {
                        if let Value::Bytes(b) = dv {
                            match df {
                                1 => ids = packed(b)?,
                                8 => lats = packed(b)?,
                                9 => lons = packed(b)?,
                                _ => {}
                            }
                        }
                    }
                    let (mut id, mut lat, mut lon) = (0i64, 0i64, 0i64);
                    for i in 0..ids.len().min(lats.len()).min(lons.len()) {
                        id += zigzag(ids[i]);
                        lat += zigzag(lats[i]);
                        lon += zigzag(lons[i]);
                        f(Primitive::Node {
                            id,
                            lat: 1e-9 * (lat_off + granularity * lat) as f64,
                            lon: 1e-9 * (lon_off + granularity * lon) as f64,
                        });
                    }
                } else if field == 3 && want_ways {
                    let (mut id, mut keys, mut vals, mut refs) = (0i64, Vec::new(), Vec::new(), Vec::new());
                    for (wf, wv) in fields(body)? {
                        match (wf, wv) {
                            (1, Value::Int(n)) => id = n as i64,
                            (2, Value::Bytes(b)) => keys = packed(b)?,
                            (3, Value::Bytes(b)) => vals = packed(b)?,
                            (8, Value::Bytes(b)) => {
                                let mut r = 0i64;
                                for d in packed(b)? {
                                    r += zigzag(d);
                                    refs.push(r);
                                }
                            }
                            _ => {}
                        }
                    }
                    let tags: HashMap<&str, &str> =
                        keys.iter().zip(&vals).map(|(k, v)| (string(*k), string(*v))).collect();
                    f(Primitive::Way { id, refs: &refs, tags: &tags });
                }
            }
        }
        Ok(())
    })
}
