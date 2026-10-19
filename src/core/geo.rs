//! Geodesy for a few kilometres of water.
//!
//! Distances here are computed on a sphere.  Over the range a small surface
//! vessel works from its ground station — a few kilometres — the difference
//! between a spherical and an ellipsoidal model is well under a metre, which
//! is far inside the boat's own GNSS error.

use serde::{Deserialize, Serialize};

/// Mean Earth radius, metres.
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// A geographic position in degrees.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

impl LatLon {
    pub const fn new(lat: f64, lon: f64) -> Self {
        LatLon { lat, lon }
    }

    pub fn is_valid(&self) -> bool {
        (-90.0..=90.0).contains(&self.lat) && (-180.0..=180.0).contains(&self.lon)
    }
}

impl From<(f64, f64)> for LatLon {
    fn from(p: (f64, f64)) -> Self {
        LatLon::new(p.0, p.1)
    }
}

/// Great-circle distance in metres (haversine).
pub fn distance_m(a: LatLon, b: LatLon) -> f64 {
    let phi1 = a.lat.to_radians();
    let phi2 = b.lat.to_radians();
    let dphi = (b.lat - a.lat).to_radians();
    let dlambda = (b.lon - a.lon).to_radians();
    let h = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.min(1.0).sqrt().asin()
}

/// Initial great-circle bearing from `a` to `b`, degrees clockwise from north.
pub fn bearing_deg(a: LatLon, b: LatLon) -> f64 {
    let phi1 = a.lat.to_radians();
    let phi2 = b.lat.to_radians();
    let dlambda = (b.lon - a.lon).to_radians();
    let y = dlambda.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlambda.cos();
    (y.atan2(x).to_degrees() + 360.0).rem_euclid(360.0)
}

/// The position reached by travelling `distance` metres along `bearing`.
pub fn destination(origin: LatLon, bearing: f64, distance: f64) -> LatLon {
    let phi1 = origin.lat.to_radians();
    let lambda1 = origin.lon.to_radians();
    let theta = bearing.to_radians();
    let delta = distance / EARTH_RADIUS_M;
    let phi2 = (phi1.sin() * delta.cos() + phi1.cos() * delta.sin() * theta.cos()).asin();
    let lambda2 = lambda1
        + (theta.sin() * delta.sin() * phi1.cos()).atan2(delta.cos() - phi1.sin() * phi2.sin());
    let lon = (lambda2.to_degrees() + 540.0).rem_euclid(360.0) - 180.0;
    LatLon::new(phi2.to_degrees(), lon)
}

/// Metres per degree of latitude and of longitude at `latitude`.
///
/// Used for the local flat-Earth arithmetic in route planning, where working
/// in degrees directly would distort line spacing with latitude.
pub fn metres_per_degree(latitude: f64) -> (f64, f64) {
    let lat_m = std::f64::consts::PI * EARTH_RADIUS_M / 180.0;
    let lon_m = lat_m * latitude.to_radians().cos();
    (lat_m, lon_m.max(1e-6))
}

/// Signed smallest difference `a - b` between two headings, in [-180, 180).
pub fn angle_difference(a: f64, b: f64) -> f64 {
    (a - b + 180.0).rem_euclid(360.0) - 180.0
}

// --------------------------------------------------------------------------
// Bounding boxes
// --------------------------------------------------------------------------

/// An axis-aligned latitude/longitude bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BBox {
    pub min_lat: f64,
    pub min_lon: f64,
    pub max_lat: f64,
    pub max_lon: f64,
}

impl BBox {
    pub fn new(min_lat: f64, min_lon: f64, max_lat: f64, max_lon: f64) -> Self {
        BBox { min_lat, min_lon, max_lat, max_lon }
    }

    /// The box round `points`, grown by `margin_m`.  None for no points.
    pub fn around(points: &[LatLon], margin_m: f64) -> Option<BBox> {
        let first = points.first()?;
        let mut b = BBox::new(first.lat, first.lon, first.lat, first.lon);
        for p in points {
            b.min_lat = b.min_lat.min(p.lat);
            b.max_lat = b.max_lat.max(p.lat);
            b.min_lon = b.min_lon.min(p.lon);
            b.max_lon = b.max_lon.max(p.lon);
        }
        Some(if margin_m != 0.0 { b.expanded(margin_m) } else { b })
    }

    pub fn expanded(&self, margin_m: f64) -> BBox {
        let (lat_m, lon_m) = metres_per_degree(self.centre().lat);
        BBox::new(
            (self.min_lat - margin_m / lat_m).max(-90.0),
            (self.min_lon - margin_m / lon_m).max(-180.0),
            (self.max_lat + margin_m / lat_m).min(90.0),
            (self.max_lon + margin_m / lon_m).min(180.0),
        )
    }

    pub fn centre(&self) -> LatLon {
        LatLon::new((self.min_lat + self.max_lat) / 2.0, (self.min_lon + self.max_lon) / 2.0)
    }

    pub fn corners(&self) -> [LatLon; 4] {
        [
            LatLon::new(self.min_lat, self.min_lon),
            LatLon::new(self.min_lat, self.max_lon),
            LatLon::new(self.max_lat, self.max_lon),
            LatLon::new(self.max_lat, self.min_lon),
        ]
    }

    pub fn width_m(&self) -> f64 {
        let c = self.centre();
        distance_m(LatLon::new(c.lat, self.min_lon), LatLon::new(c.lat, self.max_lon))
    }

    pub fn height_m(&self) -> f64 {
        let c = self.centre();
        distance_m(LatLon::new(self.min_lat, c.lon), LatLon::new(self.max_lat, c.lon))
    }

    pub fn contains(&self, p: LatLon) -> bool {
        self.min_lat <= p.lat && p.lat <= self.max_lat && self.min_lon <= p.lon && p.lon <= self.max_lon
    }
}

/// Total length of a polyline in metres.
pub fn path_length_m(points: &[LatLon]) -> f64 {
    points.windows(2).map(|w| distance_m(w[0], w[1])).sum()
}

// --------------------------------------------------------------------------
// Local tangent plane
// --------------------------------------------------------------------------
//
// Survey areas and routes are far easier to reason about in metres than in
// degrees, so the planners work in a local tangent plane about a reference.

/// East-north metres relative to a reference point.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Metres {
    pub east: f64,
    pub north: f64,
}

impl Metres {
    pub const fn new(east: f64, north: f64) -> Self {
        Metres { east, north }
    }
}

/// `point` expressed in a local tangent plane about `origin`.
pub fn to_metres(origin: LatLon, point: LatLon) -> Metres {
    Metres {
        east: (point.lon - origin.lon).to_radians() * EARTH_RADIUS_M * origin.lat.to_radians().cos(),
        north: (point.lat - origin.lat).to_radians() * EARTH_RADIUS_M,
    }
}

/// The inverse of [`to_metres`].
pub fn to_lat_lon(origin: LatLon, offset: Metres) -> LatLon {
    let mut scale = origin.lat.to_radians().cos();
    if scale == 0.0 {
        scale = 1e-9;
    }
    LatLon::new(
        origin.lat + (offset.north / EARTH_RADIUS_M).to_degrees(),
        origin.lon + (offset.east / (EARTH_RADIUS_M * scale)).to_degrees(),
    )
}

/// The mean of a set of positions; the origin of a local projection.
pub fn centroid(points: &[LatLon]) -> Option<LatLon> {
    if points.is_empty() {
        return None;
    }
    let n = points.len() as f64;
    Some(LatLon::new(
        points.iter().map(|p| p.lat).sum::<f64>() / n,
        points.iter().map(|p| p.lon).sum::<f64>() / n,
    ))
}

fn projected(polygon: &[LatLon]) -> Vec<Metres> {
    let origin = centroid(polygon).unwrap_or_default();
    polygon.iter().map(|p| to_metres(origin, *p)).collect()
}

/// Area of a simple polygon in square metres, by the shoelace formula.
pub fn polygon_area_m2(polygon: &[LatLon]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    let local = projected(polygon);
    let mut total = 0.0;
    let mut previous = local[local.len() - 1];
    for corner in &local {
        total += (previous.east + corner.east) * (previous.north - corner.north);
        previous = *corner;
    }
    total.abs() / 2.0
}

/// Whether `point` is inside `polygon`, by the even-odd rule.
pub fn point_in_polygon(point: LatLon, polygon: &[LatLon]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let origin = centroid(polygon).unwrap_or_default();
    let local: Vec<Metres> = polygon.iter().map(|p| to_metres(origin, *p)).collect();
    let target = to_metres(origin, point);
    let mut inside = false;
    let mut previous = local[local.len() - 1];
    for corner in &local {
        if (corner.north > target.north) != (previous.north > target.north) {
            let crossing = (previous.east - corner.east) * (target.north - corner.north)
                / (previous.north - corner.north)
                + corner.east;
            if target.east < crossing {
                inside = !inside;
            }
        }
        previous = *corner;
    }
    inside
}

fn orientation(a: Metres, b: Metres, c: Metres) -> f64 {
    (b.east - a.east) * (c.north - a.north) - (b.north - a.north) * (c.east - a.east)
}

fn on_segment(a: Metres, b: Metres, p: Metres) -> bool {
    orientation(a, b, p).abs() < 1e-9
        && a.east.min(b.east) - 1e-9 <= p.east
        && p.east <= a.east.max(b.east) + 1e-9
        && a.north.min(b.north) - 1e-9 <= p.north
        && p.north <= a.north.max(b.north) + 1e-9
}

fn segments_cross(a: Metres, b: Metres, c: Metres, d: Metres) -> bool {
    let d1 = orientation(c, d, a);
    let d2 = orientation(c, d, b);
    let d3 = orientation(a, b, c);
    let d4 = orientation(a, b, d);
    if ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) {
        return true;
    }
    on_segment(c, d, a) || on_segment(c, d, b) || on_segment(a, b, c) || on_segment(a, b, d)
}

/// Whether a polygon's edges meet only at their shared corners.
pub fn polygon_is_simple(polygon: &[LatLon]) -> bool {
    let count = polygon.len();
    if count < 3 {
        return false;
    }
    let local = projected(polygon);
    for i in 0..count {
        let (a, b) = (local[i], local[(i + 1) % count]);
        for j in (i + 1)..count {
            if (j + 1) % count == i || (i + 1) % count == j {
                continue;
            }
            if segments_cross(a, b, local[j], local[(j + 1) % count]) {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn destination_and_back() {
        let a = LatLon::new(57.0, 24.0);
        let b = destination(a, 73.0, 1234.0);
        assert!((distance_m(a, b) - 1234.0).abs() < 0.01);
        assert!((bearing_deg(a, b) - 73.0).abs() < 0.01);
    }

    #[test]
    fn angle_difference_wraps() {
        assert_eq!(angle_difference(10.0, 350.0), 20.0);
        assert_eq!(angle_difference(350.0, 10.0), -20.0);
    }

    #[test]
    fn polygons() {
        let o = LatLon::new(57.0, 24.0);
        let square: Vec<LatLon> = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
            .iter()
            .map(|&(e, n)| to_lat_lon(o, Metres::new(e, n)))
            .collect();
        assert!((polygon_area_m2(&square) - 10_000.0).abs() < 1.0);
        assert!(point_in_polygon(to_lat_lon(o, Metres::new(50.0, 50.0)), &square));
        assert!(!point_in_polygon(to_lat_lon(o, Metres::new(150.0, 50.0)), &square));
        assert!(polygon_is_simple(&square));
        let bow_tie = vec![square[0], square[2], square[1], square[3]];
        assert!(!polygon_is_simple(&bow_tie));
    }
}
