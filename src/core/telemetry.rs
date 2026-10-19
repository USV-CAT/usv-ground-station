//! What the boat reports.
//!
//! One [`Telemetry`] per report.  Most fields are optional, because a real
//! boat's sensors fail independently and the display must be able to say "no
//! reading" rather than show a stale or invented number.

use super::clock;

/// The water-quality sonde most survey USVs carry.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WaterQuality {
    pub temperature_c: Option<f64>,
    pub ph: Option<f64>,
    pub dissolved_oxygen_mgl: Option<f64>,
    pub oxygen_saturation_pct: Option<f64>,
    pub conductivity_ms: Option<f64>,
    pub tds_ppm: Option<f64>,
    pub salinity_psu: Option<f64>,
    pub orp_mv: Option<f64>,
    pub turbidity_ntu: Option<f64>,
}

/// The oil-recovery payload: sweep arms, brush skimmer, tank and oil sensor.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SkimmerState {
    pub fitted: bool,
    pub deployed: bool,
    /// Pump actually lifting oil (deployed, tank not full, oil in the arms).
    pub recovering: bool,
    /// Litres of oil a minute going into the tank right now.
    pub recovery_lpm: f64,
    /// Fraction of the oil met that is being kept, given the speed.
    pub efficiency: f64,
    pub tank_l: f64,
    pub tank_capacity_l: f64,
    /// Oil recovered over the whole run, litres.
    pub recovered_oil_l: f64,
    pub offloading: bool,
    /// Oil-in-water fluorometer under the hull, parts per million.
    pub oil_ppm: Option<f64>,
    /// Estimated film thickness at the boat, micrometres.
    pub film_um: Option<f64>,
}

/// One battery bank, as its battery management system reports it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BankState {
    pub name: String,
    pub soc_pct: f64,
    pub volts: f64,
    /// Positive discharging, negative charging.
    pub amps: f64,
    pub max_amps: f64,
    pub cell_min_v: f64,
    pub cell_max_v: f64,
    pub temp_min_c: f64,
    pub temp_max_c: f64,
    /// The BMS is bleeding the high cells down to the low ones.
    pub balancing: bool,
}

/// Where the power goes: the banks, the shore supply and the consumers.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PowerState {
    pub banks: Vec<BankState>,
    /// Plugged in to the ground station's supply.
    pub shore_connected: bool,
    /// Power coming in from shore, watts.
    pub shore_w: f64,
    /// Electronics, sensors and payload, watts.
    pub load_w: f64,
    pub port_w: f64,
    pub stbd_w: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Telemetry {
    /// Monotonic receive time at the ground station.
    pub received: f64,
    /// Seconds since the boat booted, as the boat counts them.
    pub boat_time_s: Option<f64>,

    // -- position and motion
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    /// Compass heading, degrees true.
    pub heading_deg: Option<f64>,
    /// Course and speed over ground, from GNSS.
    pub cog_deg: Option<f64>,
    pub sog_ms: Option<f64>,
    /// Speed through the water (paddlewheel or Doppler log).
    pub stw_ms: Option<f64>,
    /// Rate of turn, degrees per second, positive to starboard.
    pub rot_dps: Option<f64>,
    pub roll_deg: Option<f64>,
    pub pitch_deg: Option<f64>,

    // -- GNSS
    /// 0 none, 2 = 2D, 3 = 3D, 4 = RTK float, 5 = RTK fixed.
    pub gps_fix: u8,
    pub satellites: u32,
    pub hdop: Option<f64>,

    // -- propulsion: what the motor controllers are actually doing, -1..1
    pub throttle_port: f64,
    pub throttle_stbd: f64,
    pub rpm_port: f64,
    pub rpm_stbd: f64,

    // -- power
    pub battery_v: Option<f64>,
    pub battery_a: Option<f64>,
    pub battery_pct: Option<f64>,

    // -- environment
    pub depth_m: Option<f64>,
    /// Apparent wind, measured on board: speed and angle off the bow (+ stbd).
    pub wind_apparent_ms: Option<f64>,
    pub wind_apparent_deg: Option<f64>,
    pub water: WaterQuality,
    pub skimmer: SkimmerState,
    /// The power system in detail, when the boat reports it.
    pub power: Option<PowerState>,

    // -- state the boat itself knows
    pub armed: bool,
    /// The boat's own failsafe stopped the motors (lost commands).
    pub onboard_failsafe: bool,
    pub grounded: bool,
    /// Signal strength at the boat's radio, dBm.
    pub rssi_dbm: Option<f64>,
    /// Command frames the boat received per second.
    pub uplink_hz: Option<f64>,
}

impl Default for Telemetry {
    fn default() -> Self {
        Telemetry {
            received: clock::monotonic(),
            boat_time_s: None,
            lat: None,
            lon: None,
            heading_deg: None,
            cog_deg: None,
            sog_ms: None,
            stw_ms: None,
            rot_dps: None,
            roll_deg: None,
            pitch_deg: None,
            gps_fix: 0,
            satellites: 0,
            hdop: None,
            throttle_port: 0.0,
            throttle_stbd: 0.0,
            rpm_port: 0.0,
            rpm_stbd: 0.0,
            battery_v: None,
            battery_a: None,
            battery_pct: None,
            depth_m: None,
            wind_apparent_ms: None,
            wind_apparent_deg: None,
            water: WaterQuality::default(),
            skimmer: SkimmerState::default(),
            power: None,
            armed: false,
            onboard_failsafe: false,
            grounded: false,
            rssi_dbm: None,
            uplink_hz: None,
        }
    }
}

impl Telemetry {
    pub fn age(&self, now: f64) -> f64 {
        now - self.received
    }

    pub fn has_position(&self) -> bool {
        self.lat.is_some() && self.lon.is_some() && self.gps_fix >= 2
    }

    /// The reported position, when there is a fix.
    pub fn position(&self) -> Option<super::geo::LatLon> {
        if self.has_position() {
            Some(super::geo::LatLon::new(self.lat?, self.lon?))
        } else {
            None
        }
    }
}
