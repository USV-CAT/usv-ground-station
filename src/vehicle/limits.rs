//! Limits on speed, acceleration and rate of turn.
//!
//! Every motor command passes through the [`Limiter`] before it is sent.  It
//! works on the helm terms — throttle (the mean of the motors) and steer (half
//! their difference) — so that limiting one never twists the other:
//!
//! * **acceleration**: the throttle may rise only so fast, and fall only so
//!   fast, converted from m/s² through the boat's top speed;
//! * **speed**: above the limit the throttle is eased off in proportion to the
//!   excess, a governor on the speed over ground that acts at once;
//! * **rate of turn**: as the measured rate of turn nears the limit the steer
//!   towards it is faded out, and past it a little counter-steer is added.
//!
//! USV STOP and disarming are never limited: stopping must be immediate.

use crate::config::LimitSettings;
use crate::sim::boat::TOP_SPEED_MS;

use super::autopilot::{clamp, mix, unmix};

/// Which limit is holding the boat back, for the displays.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Limit {
    Speed,
    Accel,
    Decel,
    Turn,
}

impl Limit {
    pub fn label(self) -> &'static str {
        match self {
            Limit::Speed => "SPEED LIMIT",
            Limit::Accel => "ACCEL LIMIT",
            Limit::Decel => "DECEL LIMIT",
            Limit::Turn => "TURN LIMIT",
        }
    }
}

/// The speed governor's band, m/s: full throttle up to `SPEED_BAND_MS` below
/// `max + SPEED_BAND_TOP_MS`, none at it.  Steering fades over `TURN_BAND`
/// of the turn limit below it.
const SPEED_BAND_MS: f64 = 0.3;
const SPEED_BAND_TOP_MS: f64 = 0.05;
const TURN_BAND: f64 = 0.2;
/// Counter-steer per degree a second over the turn limit.
const COUNTER_STEER: f64 = 0.02;

#[derive(Clone, Debug, Default)]
pub struct Limiter {
    throttle: f64,
    /// What limited the last command, if anything.
    pub active: Option<Limit>,
}

impl Limiter {
    /// Forget the past command: after a stop, the next start ramps from zero.
    pub fn reset(&mut self) {
        self.throttle = 0.0;
        self.active = None;
    }

    /// Shape a command.  `sog` and `rot_dps` are what the boat reports.
    pub fn apply(&mut self, port: f64, stbd: f64, limits: &LimitSettings, sog: f64, rot_dps: f64, dt: f64) -> (f64, f64) {
        let (mut throttle, mut steer) = unmix(port, stbd);
        self.active = None;

        // Acceleration: the throttle moves only so fast.  Growing in either
        // direction is acceleration; shrinking towards zero is deceleration.
        let up = limits.max_accel_ms2 / TOP_SPEED_MS * dt;
        let down = limits.max_decel_ms2 / TOP_SPEED_MS * dt;
        let previous = self.throttle;
        let limited = if previous == 0.0 || throttle.signum() == previous.signum() {
            if throttle.abs() > previous.abs() {
                let t = previous + clamp(throttle - previous, -up, up);
                if t != throttle {
                    self.active.get_or_insert(Limit::Accel);
                }
                t
            } else {
                let t = previous + clamp(throttle - previous, -down, down);
                if t != throttle {
                    self.active.get_or_insert(Limit::Decel);
                }
                t
            }
        } else {
            // Ahead to astern (or back): slow down to zero first.
            self.active.get_or_insert(Limit::Decel);
            let t = previous + clamp(-previous, -down, down);
            if t.abs() < 1e-9 { 0.0 } else { t }
        };
        throttle = limited;
        self.throttle = throttle;

        // Speed: ease the throttle off above the limit — after the ramp, so
        // the governor acts at once and the ramp resumes when it lets go.
        // The band ends just past the limit, so the boat settles at the limit
        // rather than above it.
        let factor = clamp((limits.max_speed_ms + SPEED_BAND_TOP_MS - sog) / SPEED_BAND_MS, 0.0, 1.0);
        if factor < 1.0 && throttle > 0.0 {
            throttle *= factor;
            self.active = Some(Limit::Speed);
        }

        // Rate of turn: fade the steer towards the limit, counter-steer past it.
        let limit = limits.max_turn_dps;
        let turning = rot_dps.abs();
        if steer * rot_dps > 0.0 && turning > limit * (1.0 - TURN_BAND) {
            steer *= clamp((limit - turning) / (limit * TURN_BAND), 0.0, 1.0);
            self.active = Some(Limit::Turn);
        }
        if turning > limit {
            steer -= rot_dps.signum() * COUNTER_STEER * (turning - limit);
            steer = clamp(steer, -1.0, 1.0);
            self.active = Some(Limit::Turn);
        }
        mix(throttle, steer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> LimitSettings {
        LimitSettings { max_speed_ms: 1.0, max_accel_ms2: 0.474, max_decel_ms2: 0.948, max_turn_dps: 20.0, in_manual: true }
    }

    #[test]
    fn throttle_ramps_up_and_down_at_the_limits() {
        let mut l = Limiter::default();
        // 0.474 m/s² over a 2.37 m/s top speed: a fifth of full throttle a second.
        let (p, s) = l.apply(1.0, 1.0, &limits(), 0.0, 0.0, 1.0);
        assert!((p - 0.2).abs() < 1e-9 && (s - 0.2).abs() < 1e-9);
        assert_eq!(l.active, Some(Limit::Accel));
        for _ in 0..10 {
            l.apply(1.0, 1.0, &limits(), 0.0, 0.0, 1.0);
        }
        assert!((l.throttle - 1.0).abs() < 1e-9);
        // Slowing is twice as quick here.
        let (p, _) = l.apply(0.0, 0.0, &limits(), 0.0, 0.0, 1.0);
        assert!((p - 0.6).abs() < 1e-9);
        assert_eq!(l.active, Some(Limit::Decel));
    }

    #[test]
    fn going_astern_slows_to_zero_first() {
        let mut l = Limiter::default();
        l.throttle = 0.3;
        let (p, _) = l.apply(-1.0, -1.0, &limits(), 0.0, 0.0, 0.1);
        assert!(p > 0.0 && p < 0.3);
    }

    #[test]
    fn the_speed_governor_eases_off() {
        let mut l = Limiter { throttle: 1.0, active: None };
        let (p, _) = l.apply(1.0, 1.0, &limits(), 0.9, 0.0, 0.05);
        assert!((p - 0.5).abs() < 1e-9, "half way up the band: half throttle, got {p}");
        let (p, _) = l.apply(1.0, 1.0, &limits(), 1.05, 0.0, 0.05);
        assert_eq!(p, 0.0);
        assert_eq!(l.active, Some(Limit::Speed));
    }

    #[test]
    fn turning_fades_near_the_limit_and_counters_past_it() {
        let mut l = Limiter::default();
        let (p, s) = l.apply(0.5, -0.5, &limits(), 0.0, 10.0, 0.05);
        assert!(p - s > 0.99, "well under the limit the steer passes");
        let (p, s) = l.apply(0.5, -0.5, &limits(), 0.0, 19.0, 0.05);
        assert!((p - s) / 2.0 < 0.3, "near the limit it fades");
        let (p, s) = l.apply(0.5, -0.5, &limits(), 0.0, 25.0, 0.05);
        assert!(p - s < 0.0, "past it, counter-steer");
        assert_eq!(l.active, Some(Limit::Turn));
    }
}
