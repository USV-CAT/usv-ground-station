//! Just enough of NumPy's `.npy` / `.npz` formats to read and write depth
//! charts: little-endian integers, floats and unicode strings, C order.

use std::collections::HashMap;
use std::io::{Cursor, Read, Seek, Write};

#[derive(Clone, Debug)]
pub struct Array {
    /// The dtype string, such as `<i2`, `|u1`, `<f8` or `<U40`.
    pub descr: String,
    pub shape: Vec<usize>,
    pub data: Vec<u8>,
}

impl Array {
    pub fn len(&self) -> usize {
        self.shape.iter().product()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn kind(&self) -> (char, usize) {
        let body = self.descr.trim_start_matches(['<', '>', '|', '=']);
        let kind = body.chars().next().unwrap_or('?');
        let size = body[kind.len_utf8()..].parse().unwrap_or(0);
        (kind, size)
    }

    fn big_endian(&self) -> bool {
        self.descr.starts_with('>')
    }

    /// Every element as f64, whatever the numeric type.
    pub fn to_f64(&self) -> Result<Vec<f64>, String> {
        let (kind, size) = self.kind();
        let be = self.big_endian();
        let mut out = Vec::with_capacity(self.len());
        for chunk in self.data.chunks_exact(size.max(1)).take(self.len()) {
            let mut b = [0u8; 8];
            b[..size].copy_from_slice(chunk);
            if be {
                b[..size].reverse();
            }
            let v = match (kind, size) {
                ('f', 8) => f64::from_le_bytes(b),
                ('f', 4) => f32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                ('i', 1) => b[0] as i8 as f64,
                ('i', 2) => i16::from_le_bytes([b[0], b[1]]) as f64,
                ('i', 4) => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                ('i', 8) => i64::from_le_bytes(b) as f64,
                ('u', 1) | ('b', 1) => b[0] as f64,
                ('u', 2) => u16::from_le_bytes([b[0], b[1]]) as f64,
                ('u', 4) => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                ('u', 8) => u64::from_le_bytes(b) as f64,
                _ => return Err(format!("unsupported dtype {}", self.descr)),
            };
            out.push(v);
        }
        Ok(out)
    }

    /// A unicode (`U`) or byte-string (`S`) scalar.
    pub fn to_string(&self) -> Option<String> {
        let (kind, size) = self.kind();
        match kind {
            'U' => {
                let chars: String = self
                    .data
                    .chunks_exact(4)
                    .take(size)
                    .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                    .take_while(|&c| c != 0)
                    .filter_map(char::from_u32)
                    .collect();
                Some(chars)
            }
            'S' => Some(String::from_utf8_lossy(&self.data[..size.min(self.data.len())]).trim_end_matches('\0').to_string()),
            _ => None,
        }
    }

    pub fn from_i16(shape: Vec<usize>, values: &[i16]) -> Array {
        Array { descr: "<i2".into(), shape, data: values.iter().flat_map(|v| v.to_le_bytes()).collect() }
    }

    pub fn from_u8(shape: Vec<usize>, values: &[u8]) -> Array {
        Array { descr: "|u1".into(), shape, data: values.to_vec() }
    }

    pub fn from_f64(shape: Vec<usize>, values: &[f64]) -> Array {
        Array { descr: "<f8".into(), shape, data: values.iter().flat_map(|v| v.to_le_bytes()).collect() }
    }

    pub fn from_text(text: &str) -> Array {
        let chars: Vec<char> = text.chars().collect();
        let n = chars.len().max(1);
        let mut data: Vec<u8> = chars.iter().flat_map(|c| (*c as u32).to_le_bytes()).collect();
        data.resize(4 * n, 0);
        Array { descr: format!("<U{n}"), shape: Vec::new(), data }
    }
}

/// Parse one `.npy` file.
pub fn read_npy(bytes: &[u8]) -> Result<Array, String> {
    if bytes.len() < 10 || &bytes[..6] != b"\x93NUMPY" {
        return Err("not a .npy file".into());
    }
    let major = bytes[6];
    let (header_len, start) = if major == 1 {
        (u16::from_le_bytes([bytes[8], bytes[9]]) as usize, 10)
    } else {
        if bytes.len() < 12 {
            return Err("truncated .npy header".into());
        }
        (u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize, 12)
    };
    let header = std::str::from_utf8(bytes.get(start..start + header_len).ok_or("truncated .npy header")?)
        .map_err(|_| "bad .npy header")?;
    let descr = quoted_value(header, "descr").ok_or("no dtype in .npy header")?;
    if header.contains("'fortran_order': True") {
        return Err("Fortran-ordered arrays are not supported".into());
    }
    let shape_text = header.split("'shape':").nth(1).ok_or("no shape in .npy header")?;
    let open = shape_text.find('(').ok_or("bad shape")?;
    let close = shape_text.find(')').ok_or("bad shape")?;
    let shape: Vec<usize> = shape_text[open + 1..close]
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(|s| s.parse().map_err(|_| format!("bad shape entry {s}")))
        .collect::<Result<_, _>>()?;
    Ok(Array { descr, shape, data: bytes[start + header_len..].to_vec() })
}

fn quoted_value(header: &str, key: &str) -> Option<String> {
    let after = header.split(&format!("'{key}':")).nth(1)?;
    let first = after.find('\'')?;
    let rest = &after[first + 1..];
    let end = rest.find('\'')?;
    Some(rest[..end].to_string())
}

pub fn write_npy(array: &Array) -> Vec<u8> {
    let shape = match array.shape.len() {
        0 => "()".to_string(),
        1 => format!("({},)", array.shape[0]),
        _ => format!("({})", array.shape.iter().map(usize::to_string).collect::<Vec<_>>().join(", ")),
    };
    let mut header = format!("{{'descr': '{}', 'fortran_order': False, 'shape': {}, }}", array.descr, shape);
    // Pad so the data starts on a 64-byte boundary, ending with a newline.
    let total = 10 + header.len() + 1;
    header.push_str(&" ".repeat((64 - total % 64) % 64));
    header.push('\n');
    let mut out = Vec::with_capacity(10 + header.len() + array.data.len());
    out.extend_from_slice(b"\x93NUMPY\x01\x00");
    out.extend_from_slice(&(header.len() as u16).to_le_bytes());
    out.extend_from_slice(header.as_bytes());
    out.extend_from_slice(&array.data);
    out
}

/// Every array in an `.npz`, by name (without the `.npy`).
pub fn read_npz<R: Read + Seek>(reader: R) -> Result<HashMap<String, Array>, String> {
    let mut archive = zip::ZipArchive::new(reader).map_err(|e| format!("not an .npz file: {e}"))?;
    let mut arrays = HashMap::new();
    for i in 0..archive.len() {
        let mut file = archive.by_index(i).map_err(|e| e.to_string())?;
        let name = file.name().trim_end_matches(".npy").to_string();
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).map_err(|e| e.to_string())?;
        arrays.insert(name, read_npy(&bytes)?);
    }
    Ok(arrays)
}

pub fn read_npz_bytes(bytes: &[u8]) -> Result<HashMap<String, Array>, String> {
    read_npz(Cursor::new(bytes))
}

/// Write a compressed `.npz`, as `numpy.savez_compressed` does.
pub fn write_npz<W: Write + Seek>(writer: W, arrays: &[(&str, &Array)]) -> Result<(), String> {
    let mut zip = zip::ZipWriter::new(writer);
    let options = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
    for (name, array) in arrays {
        zip.start_file(format!("{name}.npy"), options).map_err(|e| e.to_string())?;
        zip.write_all(&write_npy(array)).map_err(|e| e.to_string())?;
    }
    zip.finish().map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn npz_round_trip() {
        let depth = Array::from_i16(vec![2, 3], &[1, -2, 3, -32768, 5, 6]);
        let source = Array::from_text("Ķīšezers");
        let mut buffer = Cursor::new(Vec::new());
        write_npz(&mut buffer, &[("depth_dm", &depth), ("source", &source)]).unwrap();
        let arrays = read_npz_bytes(buffer.get_ref()).unwrap();
        assert_eq!(arrays["depth_dm"].shape, vec![2, 3]);
        assert_eq!(arrays["depth_dm"].to_f64().unwrap(), vec![1.0, -2.0, 3.0, -32768.0, 5.0, 6.0]);
        assert_eq!(arrays["source"].to_string().unwrap(), "Ķīšezers");
    }
}
