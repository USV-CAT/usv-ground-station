//! The vector triangles every navigator draws.
//!
//! * **True wind** from the apparent wind the anemometer measures, by taking
//!   the boat's own motion out of it.
//! * **Current** (set and drift) as the difference between the velocity over
//!   the ground, from GNSS, and the velocity through the water, from the log
//!   and the compass.
//!
//! Vectors are (north, east) in metres per second.  Bearings are degrees
//! true; wind is named for where it comes *from*, current for where it goes.

pub fn from_polar(speed: f64, bearing_deg: f64) -> (f64, f64) {
    let theta = bearing_deg.to_radians();
    (speed * theta.cos(), speed * theta.sin())
}

/// (speed, bearing towards) of a vector.
pub fn to_polar(north: f64, east: f64) -> (f64, f64) {
    (north.hypot(east), (east.atan2(north).to_degrees() + 360.0).rem_euclid(360.0))
}

/// True wind (speed, direction it blows from), degrees true.
///
/// The apparent wind is the true wind minus the boat's velocity; adding the
/// boat's ground velocity back gives the wind over the ground.
pub fn true_wind(apparent_speed: f64, apparent_angle_deg: f64, heading_deg: f64, sog: f64, cog_deg: Option<f64>) -> (f64, f64) {
    let air_from = heading_deg + apparent_angle_deg;
    let (an, ae) = from_polar(apparent_speed, air_from + 180.0);
    let (bn, be) = from_polar(sog, cog_deg.unwrap_or(heading_deg));
    let (speed, towards) = to_polar(an + bn, ae + be);
    (speed, (towards + 180.0).rem_euclid(360.0))
}

/// Current (drift in m/s, set in degrees towards).
pub fn current(sog: f64, cog_deg: Option<f64>, stw: f64, heading_deg: f64) -> (f64, f64) {
    let (gn, ge) = from_polar(sog, cog_deg.unwrap_or(heading_deg));
    let (wn, we) = from_polar(stw, heading_deg);
    to_polar(gn - wn, ge - we)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn true_wind_when_motoring_into_it() {
        // 5 m/s from the north, boat doing 2 m/s north: 7 m/s apparent, dead ahead.
        let (speed, source) = true_wind(7.0, 0.0, 0.0, 2.0, Some(0.0));
        assert!(close(speed, 5.0, 1e-6));
        assert!(close(source, 0.0, 1e-6) || close(source, 360.0, 1e-6));
    }

    #[test]
    fn true_wind_on_the_beam() {
        // Wind 3 m/s from the east, boat 3 m/s north: apparent 4.24 m/s from 45° to starboard.
        let (speed, source) = true_wind(3f64.hypot(3.0), 45.0, 0.0, 3.0, Some(0.0));
        assert!(close(speed, 3.0, 1e-6));
        assert!(close(source, 90.0, 1e-6));
    }

    #[test]
    fn current_is_ground_minus_water() {
        let (drift, set) = current(1.0, Some(90.0), 1.0, 0.0);
        assert!(close(drift, 2f64.sqrt(), 1e-9));
        assert!(close(set, 135.0, 1e-9));
    }
}
