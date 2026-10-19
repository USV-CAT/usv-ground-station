//! Steering and speed control for a differential-thrust boat.
//!
//! Pure arithmetic with no threads and no I/O, so every piece can be tested
//! on its own.  The pieces are the usual ones:
//!
//! * a heading controller (PID, with the rate of turn as the derivative
//!   term, so it damps on what the gyro measures rather than on a noisy
//!   differentiated error);
//! * a speed controller (feed-forward plus PI on speed over ground);
//! * line-of-sight guidance along a leg, which steers for a point a fixed
//!   distance ahead on the line and so converges onto the line instead of
//!   chasing the waypoint;
//! * a crab-angle correction, so a cross-current does not leave the boat
//!   tracking parallel to its line;
//! * a mixer from (throttle, steer) to the two motors.

use crate::core::geo::{self, LatLon};
use crate::core::mission::cross_track;

/// Speed at full throttle, m/s, for the speed feed-forward.
pub const TOP_SPEED_MS: f64 = 2.37;
/// Heading error beyond which the boat stops and pivots rather than arcing.
pub const PIVOT_ERROR_DEG: f64 = 70.0;

pub fn clamp(value: f64, low: f64, high: f64) -> f64 {
    value.max(low).min(high)
}

fn unit(value: f64) -> f64 {
    clamp(value, -1.0, 1.0)
}

/// (throttle, steer) to (port, starboard).  Positive steer turns to starboard.
///
/// When the sum would saturate a motor, throttle gives way and the steering
/// is kept, because a boat that cannot turn is worse than one that slows down.
pub fn mix(throttle: f64, steer: f64) -> (f64, f64) {
    let (throttle, steer) = (unit(throttle), unit(steer));
    let headroom = 1.0 - steer.abs();
    let throttle = clamp(throttle, -headroom, headroom);
    (unit(throttle + steer), unit(throttle - steer))
}

pub fn unmix(port: f64, stbd: f64) -> (f64, f64) {
    ((port + stbd) / 2.0, (port - stbd) / 2.0)
}

#[derive(Clone, Debug)]
pub struct HeadingController {
    /// Steer per degree of error.
    pub kp: f64,
    /// Steer per degree/second of turn.
    pub kd: f64,
    /// Steer per degree-second.
    pub ki: f64,
    pub integral_limit: f64,
    integral: f64,
}

impl Default for HeadingController {
    fn default() -> Self {
        HeadingController { kp: 0.022, kd: 0.012, ki: 0.0015, integral_limit: 0.25, integral: 0.0 }
    }
}

impl HeadingController {
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    pub fn update(&mut self, target_deg: f64, heading_deg: f64, rot_dps: f64, dt: f64) -> f64 {
        let error = geo::angle_difference(target_deg, heading_deg);
        if error.abs() < 25.0 {
            self.integral = clamp(self.integral + error * dt * self.ki, -self.integral_limit, self.integral_limit);
        } else {
            self.integral *= 0.95;
        }
        unit(self.kp * error - self.kd * rot_dps + self.integral)
    }
}

#[derive(Clone, Debug)]
pub struct SpeedController {
    pub kp: f64,
    pub ki: f64,
    integral: f64,
}

impl Default for SpeedController {
    fn default() -> Self {
        SpeedController { kp: 0.35, ki: 0.12, integral: 0.0 }
    }
}

impl SpeedController {
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    pub fn update(&mut self, target_ms: f64, speed_ms: f64, dt: f64) -> f64 {
        let feed_forward = (target_ms.abs() / TOP_SPEED_MS).sqrt().copysign(target_ms) * 0.95;
        let error = target_ms - speed_ms;
        self.integral = clamp(self.integral + error * dt * self.ki, -0.3, 0.3);
        unit(feed_forward + self.kp * error + self.integral)
    }
}

/// Where to steer on the leg `start` → `end`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegGuidance {
    pub course_deg: f64,
    pub cross_track_m: f64,
    pub along_m: f64,
    pub leg_m: f64,
    pub distance_to_go_m: f64,
}

impl LegGuidance {
    pub fn passed_end(&self) -> bool {
        self.along_m >= self.leg_m
    }
}

pub fn line_of_sight(start: LatLon, end: LatLon, position: LatLon, lookahead_m: f64) -> LegGuidance {
    let (along, across, length) = cross_track(start, end, position);
    let distance = geo::distance_m(position, end);
    let course = if length < 1.0 {
        geo::bearing_deg(position, end)
    } else if distance < lookahead_m {
        // Close to the end, aim at it directly so the boat does not orbit it.
        geo::bearing_deg(position, end)
    } else {
        let leg_course = geo::bearing_deg(start, end);
        (leg_course - across.atan2(lookahead_m.max(1.0)).to_degrees()).rem_euclid(360.0)
    };
    LegGuidance { course_deg: course, cross_track_m: across, along_m: along, leg_m: length, distance_to_go_m: distance }
}

/// Learns the angle between heading and track, so the bow can be held into a set.
#[derive(Clone, Debug)]
pub struct CrabCorrection {
    pub limit: f64,
    pub rate: f64,
    pub angle: f64,
}

impl Default for CrabCorrection {
    fn default() -> Self {
        CrabCorrection { limit: 35.0, rate: 0.08, angle: 0.0 }
    }
}

impl CrabCorrection {
    pub fn reset(&mut self) {
        self.angle = 0.0;
    }

    pub fn update(&mut self, heading_deg: f64, cog_deg: Option<f64>, sog_ms: Option<f64>, dt: f64) -> f64 {
        if let (Some(cog), Some(sog)) = (cog_deg, sog_ms) {
            if sog > 0.4 {
                let measured = geo::angle_difference(cog, heading_deg);
                let alpha = (self.rate * dt).min(1.0);
                self.angle += (measured - self.angle) * alpha;
                self.angle = clamp(self.angle, -self.limit, self.limit);
            }
        }
        self.angle
    }
}

/// Turns "go this way at this speed" into motor commands.
#[derive(Clone, Debug, Default)]
pub struct Pilot {
    pub heading: HeadingController,
    pub speed: SpeedController,
    pub crab: CrabCorrection,
}

impl Pilot {
    pub fn reset(&mut self) {
        self.heading.reset();
        self.speed.reset();
        self.crab.reset();
    }

    /// Motor levels to make good `course_deg` at `speed_ms`.
    #[allow(clippy::too_many_arguments)]
    pub fn steer_course(
        &mut self,
        course_deg: f64,
        speed_ms: f64,
        heading_deg: f64,
        rot_dps: f64,
        cog_deg: Option<f64>,
        sog_ms: f64,
        dt: f64,
        track_over_ground: bool,
    ) -> (f64, f64) {
        let crab = if track_over_ground { self.crab.update(heading_deg, cog_deg, Some(sog_ms), dt) } else { 0.0 };
        let target_heading = (course_deg - crab).rem_euclid(360.0);
        let steer = self.heading.update(target_heading, heading_deg, rot_dps, dt);
        let error = geo::angle_difference(target_heading, heading_deg).abs();
        let throttle = if error > PIVOT_ERROR_DEG {
            // Way off: stop and pivot, which a twin-motor catamaran does well.
            self.speed.reset();
            0.0
        } else {
            // Ease off through a turn so it is tight rather than wide.
            self.speed.update(speed_ms, sog_ms, dt) * (1.0 - error / PIVOT_ERROR_DEG).max(0.35)
        };
        mix(throttle, steer)
    }
}

/// Hold a point: drive back to it when blown outside a radius, else drift.
#[derive(Clone, Debug)]
pub struct StationKeeper {
    pub radius_m: f64,
    correcting: bool,
}

impl StationKeeper {
    pub fn new(radius_m: f64) -> Self {
        StationKeeper { radius_m, correcting: false }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        pilot: &mut Pilot,
        target: LatLon,
        position: LatLon,
        heading_deg: f64,
        rot_dps: f64,
        cog_deg: Option<f64>,
        sog_ms: f64,
        cruise_ms: f64,
        dt: f64,
    ) -> (f64, f64) {
        let distance = geo::distance_m(position, target);
        if distance > self.radius_m {
            self.correcting = true;
        } else if distance < self.radius_m * 0.35 {
            self.correcting = false;
        }
        if !self.correcting {
            pilot.speed.reset();
            return (0.0, 0.0);
        }
        let speed = cruise_ms.min((0.2 * distance).max(0.3));
        let course = geo::bearing_deg(position, target);
        pilot.steer_course(course, speed, heading_deg, rot_dps, cog_deg, sog_ms, dt, distance > 10.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mixer_keeps_steering_at_full_throttle() {
        let (port, stbd) = mix(1.0, 0.5);
        assert!((port - stbd - 1.0).abs() < 1e-9);
        assert!(port.abs().max(stbd.abs()) <= 1.0);
    }

    #[test]
    fn line_of_sight_converges_onto_the_leg() {
        let start = LatLon::new(57.0, 24.0);
        let end = geo::destination(start, 0.0, 200.0);
        // To starboard (east) of a northbound leg: steer left of north.
        let off = geo::destination(geo::destination(start, 0.0, 50.0), 90.0, 10.0);
        let g = line_of_sight(start, end, off, 12.0);
        assert!(g.cross_track_m > 9.0);
        assert!(geo::angle_difference(g.course_deg, 0.0) < -30.0);
    }
}
