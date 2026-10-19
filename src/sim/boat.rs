//! A twin-hull, twin-motor catamaran in three degrees of freedom.
//!
//! The boat has no rudder: it steers by running its two motors at different
//! speeds, the way most small survey catamarans do, which also lets it pivot
//! on the spot.  The model is the standard surge–sway–yaw manoeuvring model
//! with linear and quadratic damping, integrated relative to the water so a
//! current carries the boat without it having to push against it:
//!
//! ```text
//! m_x (u' - v r) = T_port + T_stbd - X_u u - X_uu |u| u + F_wind,x
//! m_y (v' + u r) =                 - Y_v v - Y_vv |v| v + F_wind,y
//! I_z r'         = (T_port - T_stbd) B/2 - N_r r - N_rr |r| r + N_wind
//! ```
//!
//! `u` is forward and `v` to starboard through the water, `r` the yaw rate,
//! positive turning to starboard.  The coefficients are picked for a 1.6 m,
//! 55 kg boat with two 38 N thrusters: about 4.6 kn flat out, and a pivot of
//! about 40°/s.

use std::f64::consts::TAU;

use crate::config::BoatSettings;
use crate::core::geo::{self, LatLon, Metres};

use super::world::World;

pub const AIR_DENSITY: f64 = 1.2;
/// Fraction of forward thrust a propeller gives running backwards.
pub const REVERSE_EFFICIENCY: f64 = 0.55;
/// Motor and controller response, seconds.
pub const MOTOR_TAU_S: f64 = 0.25;
pub const MAX_RPM: f64 = 3200.0;
/// Speed at full throttle, m/s.
pub const TOP_SPEED_MS: f64 = 2.37;
/// The two battery banks: their most current each, internal resistance of a
/// bank (ohms), heat capacity (J/K) and cooling time constant (s).
pub const BANK_MAX_A: f64 = 40.0;
const BANK_R_OHM: f64 = 0.03;
const BANK_HEAT_J_PER_K: f64 = 400.0;
const BANK_COOL_S: f64 = 600.0;
/// Hotel load: electronics and sensors, amperes.
pub const HOTEL_A: f64 = 0.6;

/// The pack is two banks in parallel, one in each hull.  Their charge
/// drifts slightly apart and is pulled back together: the bank with more
/// charge carries more of the load, and the BMS balances them slowly.
#[derive(Clone, Debug, PartialEq)]
pub struct Banks {
    /// Each bank's charge relative to the pack's, fractions.
    pub offset: [f64; 2],
    pub temp_c: [f64; 2],
    pub amps: [f64; 2],
}

impl Default for Banks {
    fn default() -> Self {
        Banks { offset: [0.006, -0.006], temp_c: [15.0, 15.0], amps: [0.0, 0.0] }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoatState {
    /// Metres from the world origin.
    pub north: f64,
    pub east: f64,
    /// Radians, clockwise from north.
    pub heading: f64,
    /// Surge and sway through the water, m/s; yaw rate, rad/s.
    pub u: f64,
    pub v: f64,
    pub r: f64,
    /// What the motors are actually doing, -1..1.
    pub motor_port: f64,
    pub motor_stbd: f64,
    pub battery_soc: f64,
    pub battery_current_a: f64,
    pub grounded: bool,
    /// Odometer, over the ground.
    pub distance_m: f64,
    pub groundings: u32,
    /// Payload draw on the battery (the skimmer's pump), amperes.
    pub extra_load_a: f64,
    /// Charge current from the shore supply, amperes.
    pub charge_a: f64,
    pub t: f64,
}

/// A 4S Li-ion pack's resting voltage at a state of charge.
pub fn open_circuit_v(soc: f64) -> f64 {
    if soc > 0.0 { 12.0 + 4.6 * (0.15 * soc + 0.85 * soc.powf(0.6)) } else { 12.0 }
}

/// What one motor draws at a throttle level, amperes: faster than linearly.
pub fn motor_amps(level: f64) -> f64 {
    18.0 * level.abs().powf(1.5)
}

pub struct Catamaran {
    pub settings: BoatSettings,
    pub origin: LatLon,
    pub state: BoatState,
    mx: f64,
    my: f64,
    iz: f64,
    xu: f64,
    xuu: f64,
    yv: f64,
    yvv: f64,
    nr: f64,
    nrr: f64,
    /// Ground velocity (north, east) from the last step, for GNSS.
    pub ground_velocity: (f64, f64),
    /// Air velocity relative to the boat, body frame, for the anemometer.
    pub relative_air: (f64, f64),
    pub banks: Banks,
}

impl Catamaran {
    pub fn new(settings: BoatSettings, world: &World, start: LatLon, heading_deg: f64) -> Self {
        let m = settings.mass_kg;
        let xu = 6.0;
        let mut boat = Catamaran {
            xuu: (2.0 * settings.max_thrust_n - xu * TOP_SPEED_MS) / TOP_SPEED_MS.powi(2),
            settings,
            origin: world.origin,
            state: BoatState { battery_soc: 1.0, ..BoatState::default() },
            mx: m * 1.1,
            my: m * 1.6,
            iz: 25.0 * (m / 55.0),
            xu,
            yv: 30.0,
            yvv: 60.0,
            nr: 25.0,
            nrr: 40.0,
            ground_velocity: (0.0, 0.0),
            relative_air: (0.0, 0.0),
            banks: Banks::default(),
        };
        boat.reset(start, heading_deg);
        boat
    }

    pub fn reset(&mut self, start: LatLon, heading_deg: f64) {
        let offset = geo::to_metres(self.origin, start);
        let soc = if self.state.t != 0.0 { self.state.battery_soc } else { 1.0 };
        self.state = BoatState {
            north: offset.north,
            east: offset.east,
            heading: heading_deg.to_radians(),
            battery_soc: soc,
            ..BoatState::default()
        };
    }

    pub fn position(&self) -> LatLon {
        geo::to_lat_lon(self.origin, Metres::new(self.state.east, self.state.north))
    }

    pub fn heading_deg(&self) -> f64 {
        self.state.heading.to_degrees().rem_euclid(360.0)
    }

    pub fn recharge(&mut self) {
        self.state.battery_soc = 1.0;
    }

    /// 4S Li-ion: 12.0 V empty, 16.6 V full, less the sag under load.
    pub fn battery_voltage(&self) -> f64 {
        (open_circuit_v(self.state.battery_soc) - 0.03 * self.state.battery_current_a).max(10.0)
    }

    pub fn step(&mut self, world: &World, dt: f64, command_port: f64, command_stbd: f64) {
        let p = self.settings.clone();
        let s = &mut self.state;
        s.t += dt;
        let (mut command_port, mut command_stbd) = (command_port, command_stbd);
        if s.battery_soc <= 0.0 {
            command_port = 0.0;
            command_stbd = 0.0;
        }

        // Motors chase their commands.
        let alpha = (dt / MOTOR_TAU_S).min(1.0);
        s.motor_port += (command_port.clamp(-1.0, 1.0) - s.motor_port) * alpha;
        s.motor_stbd += (command_stbd.clamp(-1.0, 1.0) - s.motor_stbd) * alpha;
        let thrust =
            |level: f64| level * p.max_thrust_n * if level >= 0.0 { 1.0 } else { REVERSE_EFFICIENCY };
        let (t_port, t_stbd) = (thrust(s.motor_port), thrust(s.motor_stbd));

        // Wind on the superstructure, from the air's velocity relative to the
        // boat, in the body frame.
        let (ch, sh) = (s.heading.cos(), s.heading.sin());
        let (wn, we) = world.wind_vector();
        let (cn, ce) = world.current_vector();
        let gn = s.u * ch - s.v * sh + cn;
        let ge = s.u * sh + s.v * ch + ce;
        let (rel_n, rel_e) = (wn - gn, we - ge);
        let air_x = rel_n * ch + rel_e * sh;
        let air_y = -rel_n * sh + rel_e * ch;
        let air = air_x.hypot(air_y);
        self.relative_air = (air_x, air_y);
        let fx = 0.5 * AIR_DENSITY * 0.9 * 0.15 * air * air_x;
        let fy = 0.5 * AIR_DENSITY * 1.0 * 0.35 * air * air_y;
        // The superstructure sits a little aft, so the bow falls off the wind.
        let nz = -0.12 * fy;

        let x = t_port + t_stbd - self.xu * s.u - self.xuu * s.u.abs() * s.u + fx;
        let y = -self.yv * s.v - self.yvv * s.v.abs() * s.v + fy;
        let n = (t_port - t_stbd) * p.beam_m / 2.0 - self.nr * s.r - self.nrr * s.r.abs() * s.r + nz;

        s.u += (x / self.mx + s.v * s.r) * dt;
        s.v += (y / self.my - s.u * s.r) * dt;
        s.r += (n / self.iz) * dt;
        s.heading = (s.heading + s.r * dt).rem_euclid(TAU);

        let (ch, sh) = (s.heading.cos(), s.heading.sin());
        let mut vn = s.u * ch - s.v * sh + cn;
        let mut ve = s.u * sh + s.v * ch + ce;
        let (new_north, new_east) = (s.north + vn * dt, s.east + ve * dt);

        // Grounding: the boat will not move into water shallower than its
        // draft unless the move takes it deeper than where it already is,
        // which is what lets it back off a bank it has run onto.
        let depth = |north: f64, east: f64| {
            let at = geo::to_lat_lon(self.origin, Metres::new(east, north));
            world.depth_at(at.lat, at.lon)
        };
        let here = depth(s.north, s.east);
        let there = depth(new_north, new_east);
        if there.is_nan() || there < p.draft_m {
            if !s.grounded {
                s.groundings += 1;
            }
            s.grounded = true;
            if !there.is_nan() && !here.is_nan() && there > here {
                s.north = new_north;
                s.east = new_east;
            } else {
                s.u *= 0.2;
                s.v *= 0.2;
                s.r *= 0.5;
                vn = 0.0;
                ve = 0.0;
            }
        } else {
            s.grounded = false;
            s.distance_m += vn.hypot(ve) * dt;
            s.north = new_north;
            s.east = new_east;
        }
        self.ground_velocity = (vn, ve);

        // Current draw: hotel load, payload, and the motors rising faster than linearly.
        s.battery_current_a = HOTEL_A + s.extra_load_a + motor_amps(s.motor_port) + motor_amps(s.motor_stbd) - s.charge_a;
        s.battery_soc = (s.battery_soc - s.battery_current_a * dt / 3600.0 / p.battery_ah).clamp(0.0, 1.0);
        let current = s.battery_current_a;
        self.step_banks(dt, current, p.battery_ah, world.water_temp_c());
    }

    fn step_banks(&mut self, dt: f64, current: f64, capacity_ah: f64, ambient_c: f64) {
        let b = &mut self.banks;
        let half = (capacity_ah / 2.0).max(0.1);
        for i in 0..2 {
            // Discharging, the fuller bank gives more; charging, it takes less.
            let lean = (2.5 * b.offset[i]).clamp(-0.2, 0.2);
            let share = if current >= 0.0 { 0.5 + lean } else { 0.5 - lean };
            b.amps[i] = current * share;
            b.offset[i] -= (b.amps[i] - current / 2.0) * dt / 3600.0 / half;
            b.offset[i] *= 1.0 - dt / 7200.0; // the BMS balances them
            let heat = b.amps[i] * b.amps[i] * BANK_R_OHM / BANK_HEAT_J_PER_K;
            b.temp_c[i] += (heat - (b.temp_c[i] - ambient_c) / BANK_COOL_S) * dt;
        }
    }

    /// The two banks, as their BMS would report them.
    pub fn bank_states(&self) -> Vec<crate::core::telemetry::BankState> {
        let soc = self.state.battery_soc;
        (0..2)
            .map(|i| {
                let b = &self.banks;
                let bank_soc = (soc + b.offset[i]).clamp(0.0, 1.0);
                let volts = (open_circuit_v(bank_soc) - 2.0 * BANK_R_OHM * b.amps[i]).max(10.0);
                let cell = volts / 4.0;
                let spread = 0.004 + 0.0004 * b.amps[i].abs() + 0.4 * b.offset[i].abs();
                crate::core::telemetry::BankState {
                    name: if i == 0 { "PORT BANK".into() } else { "STBD BANK".into() },
                    soc_pct: bank_soc * 100.0,
                    volts,
                    amps: b.amps[i],
                    max_amps: BANK_MAX_A,
                    cell_min_v: cell - spread / 2.0,
                    cell_max_v: cell + spread / 2.0,
                    temp_min_c: b.temp_c[i] - 1.0,
                    temp_max_c: b.temp_c[i] + 1.5,
                    balancing: spread > 0.008,
                }
            })
            .collect()
    }

    /// Where the power is going right now.
    pub fn power_state(&self, shore_connected: bool) -> crate::core::telemetry::PowerState {
        let s = &self.state;
        let v = self.battery_voltage();
        crate::core::telemetry::PowerState {
            banks: self.bank_states(),
            shore_connected,
            shore_w: s.charge_a * v,
            load_w: (HOTEL_A + s.extra_load_a) * v,
            port_w: motor_amps(s.motor_port) * v,
            stbd_w: motor_amps(s.motor_stbd) * v,
        }
    }

    pub fn depth_here(&self, world: &World) -> f64 {
        let at = self.position();
        world.depth_at(at.lat, at.lon)
    }

    /// Roll and pitch, degrees: swell from the wind, and heel in a turn.
    pub fn attitude(&self, world: &World) -> (f64, f64) {
        let s = &self.state;
        let t = s.t;
        let sea = (world.wind_speed_ms / 10.0).min(1.0);
        let roll = sea * (2.5 * (t * 1.9).sin() + 1.0 * (t * 3.1 + 1.0).sin()) + (-0.06 * s.u * s.r).to_degrees();
        let pitch = sea * (1.5 * (t * 1.4 + 0.5).sin()) - 1.2 * s.u * (s.motor_port.abs() + s.motor_stbd.abs()) / 2.0;
        (roll, pitch)
    }
}
