//! The seabed along a route: how much water there is under the planned track.

use super::geo::{self, LatLon};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Profile {
    /// Distance along the route, metres, for each sample.
    pub distance: Vec<f64>,
    /// Charted depth, metres (NaN on land).
    pub chart: Vec<f64>,
    /// Surveyed depth where the boat has sounded.
    pub surveyed: Vec<Option<f64>>,
    /// Distance along the route of each waypoint.
    pub waypoints: Vec<f64>,
}

impl Profile {
    pub fn length(&self) -> f64 {
        self.distance.last().copied().unwrap_or(0.0)
    }

    /// The depth used at sample `i`: surveyed if known, else charted, land as zero.
    pub fn depth(&self, i: usize) -> f64 {
        self.surveyed[i].unwrap_or(if self.chart[i].is_nan() { 0.0 } else { self.chart[i] })
    }

    /// (distance, depth) of the least water on the route, land counting as zero.
    pub fn shallowest(&self) -> Option<(f64, f64)> {
        let mut best: Option<(f64, f64)> = None;
        for i in 0..self.distance.len() {
            let depth = self.depth(i);
            if best.is_none_or(|b| depth < b.1) {
                best = Some((self.distance[i], depth));
            }
        }
        best
    }
}

pub fn route_profile(
    points: &[LatLon],
    chart_depth: impl Fn(f64, f64) -> f64,
    surveyed_depth: Option<&dyn Fn(f64, f64) -> Option<f64>>,
    step_m: f64,
    max_samples: usize,
) -> Profile {
    if points.len() < 2 {
        return Profile::default();
    }
    let total = geo::path_length_m(points);
    let step = step_m.max(total / max_samples.max(1) as f64);
    let mut p = Profile { waypoints: vec![0.0], ..Profile::default() };
    let mut travelled = 0.0;
    let sample = |p: &mut Profile, at: LatLon, d: f64| {
        p.distance.push(d);
        p.chart.push(chart_depth(at.lat, at.lon));
        p.surveyed.push(surveyed_depth.and_then(|f| f(at.lat, at.lon)));
    };
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let leg = geo::distance_m(a, b);
        let bearing = geo::bearing_deg(a, b);
        let count = ((leg / step).ceil() as usize).max(1);
        for i in 0..count {
            let along = leg * i as f64 / count as f64;
            sample(&mut p, geo::destination(a, bearing, along), travelled + along);
        }
        travelled += leg;
        p.waypoints.push(travelled);
    }
    sample(&mut p, points[points.len() - 1], travelled);
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_shallows() {
        let sea = LatLon::new(57.10, 23.95);
        let route = [sea, LatLon::new(sea.lat, 23.955)];
        let profile = route_profile(&route, |_, lon| if lon > 23.951 { 1.0 } else { 8.0 }, None, 10.0, 600);
        assert!((profile.length() - geo::distance_m(route[0], route[1])).abs() < 1.0);
        let low = profile.shallowest().unwrap();
        assert_eq!(low.1, 1.0);
        assert!(low.0 > 50.0);
    }
}
