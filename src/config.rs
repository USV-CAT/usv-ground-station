//! Settings.
//!
//! One struct per concern, gathered into [`Settings`], stored as readable
//! JSON in `~/.config/usvgs/config.json`.  Unknown keys are ignored, missing
//! ones take their defaults and a value of the wrong type is skipped, so a
//! file written by an older version (or by the Python station) still loads.

use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

use serde::{Deserialize, Serialize};
use serde_json::Value;

pub fn config_dir() -> PathBuf {
    match std::env::var_os("XDG_CONFIG_HOME") {
        Some(base) if !base.is_empty() => PathBuf::from(base).join("usvgs"),
        _ => home().join(".config").join("usvgs"),
    }
}

pub fn state_dir() -> PathBuf {
    match std::env::var_os("XDG_STATE_HOME") {
        Some(base) if !base.is_empty() => PathBuf::from(base).join("usvgs"),
        _ => home().join(".local").join("state").join("usvgs"),
    }
}

fn home() -> PathBuf {
    std::env::var_os("HOME").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("."))
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct BoatSettings {
    /// Hull length overall, metres.  Used to draw the boat to scale.
    pub length_m: f64,
    /// Distance between the two hull centrelines, metres.
    pub beam_m: f64,
    /// How deep the hulls sit.  Water shallower than this grounds the boat.
    pub draft_m: f64,
    /// Displacement including payload, kilograms.
    pub mass_kg: f64,
    /// Static thrust of one motor at full forward, newtons.
    pub max_thrust_n: f64,
    /// Battery capacity, ampere-hours (4S Li-ion pack).
    pub battery_ah: f64,
}

impl Default for BoatSettings {
    fn default() -> Self {
        BoatSettings { length_m: 1.6, beam_m: 0.9, draft_m: 0.25, mass_kg: 55.0, max_thrust_n: 38.0, battery_ah: 20.0 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SimSettings {
    /// Where the simulated boat is launched.  Also the ground station
    /// position, which is what the radio range is measured from.
    pub start_lat: f64,
    pub start_lon: f64,
    pub start_heading_deg: f64,
    /// True wind: speed in m/s and the direction it blows *from*.
    pub wind_speed_ms: f64,
    pub wind_from_deg: f64,
    /// Surface current: speed in m/s and the direction it flows *towards*.
    pub current_speed_ms: f64,
    pub current_to_deg: f64,
    /// LoRa range from the ground station before packets start dropping.
    pub link_range_m: f64,
    /// Wi-Fi range from the station, and how far offshore 5G / LTE reaches.
    pub wifi_range_m: f64,
    pub cellular_range_m: f64,
    /// GNSS noise, metres (one sigma).
    pub gps_noise_m: f64,
    /// Speed of simulated time relative to real time.
    pub time_scale: f64,
    /// The battery's charge at launch, percent.
    pub start_battery_pct: f64,
}

impl Default for SimSettings {
    fn default() -> Self {
        SimSettings {
            start_lat: 57.0480,
            start_lon: 23.9900,
            start_heading_deg: 320.0,
            wind_speed_ms: 4.0,
            wind_from_deg: 240.0,
            current_speed_ms: 0.08,
            current_to_deg: 60.0,
            link_range_m: 8000.0,
            wifi_range_m: 400.0,
            cellular_range_m: 4000.0,
            gps_noise_m: 0.4,
            time_scale: 1.0,
            start_battery_pct: 100.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum StickMode {
    /// Left stick throttle, right stick steering.
    #[default]
    Arcade,
    /// Left stick port motor, right stick starboard motor.
    Tank,
}

/// Names in the file; anything unknown is the default rather than an error.
macro_rules! lenient_enum {
    ($name:ident { $($text:literal => $variant:ident),* $(,)? }) => {
        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
                let text = String::deserialize(d).unwrap_or_default();
                Ok(match text.as_str() {
                    $($text => $name::$variant,)*
                    other => {
                        log::warn!("unknown {} {other:?}; using the default", stringify!($name));
                        $name::default()
                    }
                })
            }
        }
    };
}

lenient_enum!(StickMode { "arcade" => Arcade, "tank" => Tank });

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct ControlSettings {
    pub stick_mode: StickMode,
    /// Stick travel ignored around centre.
    pub deadzone: f64,
    /// 0 is linear; towards 1 the centre of the stick gets finer.
    pub expo: f64,
    /// `auto` tries gilrs (SDL-style mappings) then raw evdev; or name one;
    /// `none` is keyboard only.
    pub input_backend: String,
    /// Stick deflection that takes manual control back from an autonomous mode.
    pub override_threshold: f64,
}

impl Default for ControlSettings {
    fn default() -> Self {
        ControlSettings {
            stick_mode: StickMode::Arcade,
            deadzone: 0.08,
            expo: 0.35,
            input_backend: "auto".into(),
            override_threshold: 0.55,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct AutopilotSettings {
    /// Speed through autonomous legs, m/s (2.0 m/s ≈ 3.9 kn).
    pub cruise_speed_ms: f64,
    /// A waypoint counts as reached inside this radius.
    pub accept_radius_m: f64,
    /// Line-of-sight lookahead along the leg.  Shorter tracks the line more
    /// tightly and weaves more.
    pub lookahead_m: f64,
    /// Station keeping starts correcting outside this radius.
    pub hold_radius_m: f64,
}

impl Default for AutopilotSettings {
    fn default() -> Self {
        AutopilotSettings { cruise_speed_ms: 1.6, accept_radius_m: 4.0, lookahead_m: 12.0, hold_radius_m: 3.0 }
    }
}

/// Limits on how the boat may move.  They shape every motor command, in
/// every mode but USV STOP (and in Manual too unless switched off there).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct LimitSettings {
    /// Speed over ground the boat is held under, m/s.
    pub max_speed_ms: f64,
    /// How quickly it may gather way, m/s².
    pub max_accel_ms2: f64,
    /// How quickly it may slow, m/s² (stopping is always allowed at once).
    pub max_decel_ms2: f64,
    /// Rate of turn it is held under, degrees a second.
    pub max_turn_dps: f64,
    /// Apply the limits to the sticks as well as to the autopilot.
    pub in_manual: bool,
}

impl Default for LimitSettings {
    fn default() -> Self {
        LimitSettings { max_speed_ms: 2.3, max_accel_ms2: 0.6, max_decel_ms2: 1.2, max_turn_dps: 30.0, in_manual: true }
    }
}

/// How the station reaches the boat.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct LinkSettings {
    /// "sim" or "mavlink".
    pub kind: String,
    /// Show the start dialog; off means connect straight to `kind`.
    pub ask_at_start: bool,
    /// udp-listen:14550, udp:HOST:PORT, tcp:HOST:PORT or serial:/dev/ttyUSB0:57600.
    pub mavlink_url: String,
    /// Recent connections, newest first.
    pub recent: Vec<String>,
    /// The station's own MAVLink system id, and the boat's (0: whoever answers).
    pub gcs_sysid: u8,
    pub target_sysid: u8,
    /// RC channels the override drives (ArduPilot Rover: 3 throttle, 1 steering).
    pub rc_throttle: u8,
    pub rc_steer: u8,
    /// Servo outputs of the two motors (skid steering: 73/74, often SERVO1/3).
    pub servo_port: u8,
    pub servo_stbd: u8,
    pub pwm_min: u16,
    pub pwm_trim: u16,
    pub pwm_max: u16,
}

impl Default for LinkSettings {
    fn default() -> Self {
        LinkSettings {
            kind: "sim".into(),
            ask_at_start: true,
            mavlink_url: "udp-listen:14550".into(),
            recent: Vec::new(),
            gcs_sysid: 255,
            target_sysid: 1,
            rc_throttle: 3,
            rc_steer: 1,
            servo_port: 1,
            servo_stbd: 3,
            pwm_min: 1100,
            pwm_trim: 1500,
            pwm_max: 1900,
        }
    }
}

impl LinkSettings {
    /// Put a connection at the head of the recent list.
    pub fn remember(&mut self, url: &str) {
        self.recent.retain(|u| u != url);
        self.recent.insert(0, url.to_string());
        self.recent.truncate(6);
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SafetySettings {
    /// Distance from home that triggers return to launch.
    pub geofence_radius_m: f64,
    /// Depth alarm.  Sounds in every mode.
    pub shallow_alarm_m: f64,
    /// Autonomous modes stop and hold below this depth rather than ground.
    pub shallow_stop_m: f64,
    /// State of charge that warns, and that returns home.
    pub battery_low_pct: f64,
    pub battery_critical_pct: f64,
    /// Telemetry silence that counts as a lost link.
    pub link_timeout_s: f64,
}

impl Default for SafetySettings {
    fn default() -> Self {
        SafetySettings {
            geofence_radius_m: 2000.0,
            shallow_alarm_m: 1.2,
            shallow_stop_m: 0.7,
            battery_low_pct: 30.0,
            battery_critical_pct: 15.0,
            link_timeout_s: 2.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SurveySettings {
    /// Width of seabed the echo sounder is credited with on each ping, metres.
    pub swath_m: f64,
    /// Cell size of the surveyed depth grid, metres.
    pub cell_m: f64,
}

impl Default for SurveySettings {
    fn default() -> Self {
        SurveySettings { swath_m: 10.0, cell_m: 5.0 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SkimmerSettings {
    /// Width of water the sweep arms funnel into the skimmer, metres.
    pub swath_m: f64,
    /// Recovered-fluid tank, litres.
    pub tank_l: f64,
    /// Most the pump can lift, litres of fluid a minute.
    pub max_rate_lpm: f64,
    /// Fraction of the recovered fluid that is oil (a brush is 60–90 %).
    pub oil_content: f64,
    /// Through-water speed the arms work best at, and the speed beyond which
    /// oil escapes under them (entrainment): 0.7 kn and 1.5 kn.
    pub best_speed_ms: f64,
    pub max_speed_ms: f64,
    /// Pump and brush motor draw while running, amperes.
    pub power_a: f64,
    /// Tank empties at this rate alongside the ground station, litres a minute.
    pub offload_lpm: f64,
}

impl Default for SkimmerSettings {
    fn default() -> Self {
        SkimmerSettings {
            swath_m: 2.4,
            tank_l: 60.0,
            max_rate_lpm: 8.0,
            oil_content: 0.7,
            best_speed_ms: 0.36,
            max_speed_ms: 0.77,
            power_a: 4.0,
            offload_lpm: 12.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SpillSettings {
    /// Simulated spill: volume and how far from the launch point it starts.
    pub volume_l: f64,
    pub distance_m: f64,
    pub bearing_deg: f64,
    pub radius_m: f64,
    /// Lagrangian parcels the slick is modelled with.
    pub parcels: usize,
}

impl Default for SpillSettings {
    fn default() -> Self {
        SpillSettings { volume_l: 300.0, distance_m: 450.0, bearing_deg: 330.0, radius_m: 35.0, parcels: 1500 }
    }
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum WaterStyle {
    /// Water as it looks.
    #[default]
    Natural,
    /// Depth shading, as a nautical chart colours it.
    Chart,
    /// Satellite imagery, cached for use offline.
    Satellite,
}

lenient_enum!(WaterStyle { "natural" => Natural, "chart" => Chart, "satellite" => Satellite });

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct UiSettings {
    pub show_telemetry: bool,
    pub show_drawer: bool,
    pub drawer_page: usize,
    pub show_vectors: bool,
    pub show_hud: bool,
    pub water_style: WaterStyle,
    /// Chart frame rate: 60 for smooth, 30 to save battery.
    pub max_fps: u32,
    pub show_fps: bool,
}

impl Default for UiSettings {
    fn default() -> Self {
        UiSettings {
            show_telemetry: true,
            show_drawer: true,
            drawer_page: 0,
            show_vectors: true,
            show_hud: true,
            water_style: WaterStyle::Natural,
            max_fps: 60,
            show_fps: false,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct MapSettings {
    /// Offline OpenStreetMap index (`*.idx.json`).  Empty searches `maps/`.
    pub vector_index: String,
    /// Depth grid (`.npz`, `.asc`, `.tif` or `.xyz`).  Empty uses the
    /// bundled Gulf of Riga grid.
    pub depth_grid: String,
    /// Display units for speed: `kn` or `ms`.
    pub speed_units: String,
    /// Satellite tiles, `{z}`/`{x}`/`{y}` filled in.  Respect the provider's
    /// terms of use.
    pub satellite_url: String,
    /// Credit shown on the chart over the imagery.
    pub satellite_attribution: String,
}

pub const ESRI_IMAGERY: &str = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

impl Default for MapSettings {
    fn default() -> Self {
        MapSettings {
            vector_index: String::new(),
            depth_grid: String::new(),
            speed_units: "kn".into(),
            satellite_url: ESRI_IMAGERY.into(),
            satellite_attribution: "Imagery © Esri, Maxar, Earthstar Geographics".into(),
        }
    }
}

/// Where downloaded map tiles are kept.
pub fn cache_dir() -> PathBuf {
    match std::env::var_os("XDG_CACHE_HOME") {
        Some(base) if !base.is_empty() => PathBuf::from(base).join("usvgs"),
        _ => home().join(".cache").join("usvgs"),
    }
}

/// When and how one alert is spoken.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SoundRule {
    pub enabled: bool,
    /// Say it again every this many seconds while it lasts; 0 says it once.
    pub repeat_s: f64,
    /// Sound the two-tone warning before the words.
    pub tone: bool,
}

impl Default for SoundRule {
    fn default() -> Self {
        SoundRule { enabled: true, repeat_s: 0.0, tone: false }
    }
}

/// The alerts there are rules for, in the order the Sound page lists them,
/// with their default (repeat seconds, warning tone).
pub const SOUND_ALERTS: [(&str, &str, f64, bool); 12] = [
    ("aground", "Aground", 5.0, true),
    ("shallow", "Shallow water", 5.0, true),
    ("link_lost", "Link lost", 10.0, true),
    ("battery_critical", "Battery critical", 30.0, true),
    ("geofence", "Geofence", 20.0, true),
    ("gnss_lost", "GNSS lost", 30.0, false),
    ("battery_low", "Battery low", 60.0, false),
    ("tank_full", "Skimmer tank full", 0.0, false),
    ("link_restored", "Link restored", 0.0, false),
    ("gnss_found", "GNSS found", 0.0, false),
    ("arming", "Armed / disarmed", 0.0, false),
    ("mode", "Mode changes", 0.0, false),
];

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SoundSettings {
    /// Spoken alerts on.
    pub enabled: bool,
    /// `en` or `ru`: the folder under `sounds/`.
    pub language: String,
    /// 0..1.
    pub volume: f64,
    /// Repeat warnings only while the motors are armed: a boat tied up at the
    /// station is said to be aground once, not every five seconds.
    pub repeat_only_armed: bool,
    /// Per alert, keyed as in [`SOUND_ALERTS`].
    pub rules: std::collections::BTreeMap<String, SoundRule>,
}

impl Default for SoundSettings {
    fn default() -> Self {
        let rules = SOUND_ALERTS
            .iter()
            .map(|&(key, _, repeat_s, tone)| (key.to_string(), SoundRule { enabled: true, repeat_s, tone }))
            .collect();
        SoundSettings { enabled: true, language: "en".into(), volume: 0.8, repeat_only_armed: true, rules }
    }
}

impl SoundSettings {
    /// The rule for an alert; an unknown key gets the default rule.
    pub fn rule(&self, key: &str) -> SoundRule {
        self.rules.get(key).cloned().unwrap_or_default()
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Settings {
    pub boat: BoatSettings,
    pub sim: SimSettings,
    pub control: ControlSettings,
    pub autopilot: AutopilotSettings,
    pub safety: SafetySettings,
    pub limits: LimitSettings,
    pub link: LinkSettings,
    pub survey: SurveySettings,
    pub map: MapSettings,
    pub skimmer: SkimmerSettings,
    pub spill: SpillSettings,
    pub ui: UiSettings,
    pub sound: SoundSettings,
}

/// Settings shared by the threads: the interface edits, the rest read.
pub type SharedSettings = Arc<RwLock<Settings>>;

impl Settings {
    pub fn shared(self) -> SharedSettings {
        Arc::new(RwLock::new(self))
    }

    pub fn to_json(&self) -> Value {
        serde_json::to_value(self).unwrap_or(Value::Null)
    }

    /// From JSON, field by field: anything unreadable keeps its default.
    pub fn from_json(data: &Value) -> Settings {
        let mut merged = Settings::default().to_json();
        fill(&mut merged, data);
        serde_json::from_value(merged).unwrap_or_default()
    }

    /// Problems that would make the station misbehave, for the log.
    /// Values that cannot work are put back to their defaults.
    pub fn validate(&mut self) -> Vec<String> {
        let mut problems = Vec::new();
        if self.safety.shallow_stop_m > self.safety.shallow_alarm_m {
            problems.push("shallow_stop_m is deeper than shallow_alarm_m".to_string());
        }
        if self.safety.battery_critical_pct > self.safety.battery_low_pct {
            problems.push("battery_critical_pct is above battery_low_pct".to_string());
        }
        if !(0.0..0.5).contains(&self.control.deadzone) {
            problems.push("deadzone must be between 0 and 0.5".to_string());
            self.control.deadzone = 0.08;
        }
        let l = &mut self.limits;
        if !(l.max_speed_ms > 0.0 && l.max_accel_ms2 > 0.0 && l.max_decel_ms2 > 0.0 && l.max_turn_dps > 0.0) {
            problems.push("limits must all be positive; using the defaults".to_string());
            *l = LimitSettings::default();
        }
        if self.sim.time_scale <= 0.0 {
            problems.push("time_scale must be positive".to_string());
            self.sim.time_scale = 1.0;
        }
        problems
    }
}

/// Copy `data` into `target` where the types agree, recursively.
fn fill(target: &mut Value, data: &Value) {
    let (Value::Object(target), Value::Object(data)) = (target, data) else { return };
    for (key, current) in target.iter_mut() {
        let Some(value) = data.get(key) else { continue };
        match (&*current, value) {
            (Value::Object(_), Value::Object(_)) => fill(current, value),
            (Value::Bool(_), Value::Bool(_)) => *current = value.clone(),
            (Value::Bool(_), Value::Number(n)) => *current = Value::Bool(n.as_f64() != Some(0.0)),
            (Value::Number(c), Value::Number(n)) => {
                // Keep integers integral, so a usize field still parses.
                if c.is_f64() {
                    *current = Value::from(n.as_f64().unwrap_or(0.0));
                } else if let Some(i) = n.as_i64().or_else(|| n.as_f64().map(|f| f as i64)) {
                    *current = Value::from(i.max(0));
                }
            }
            (Value::Number(c), Value::String(s)) => {
                if let Ok(f) = s.trim().parse::<f64>() {
                    *current = if c.is_f64() { Value::from(f) } else { Value::from(f.max(0.0) as i64) };
                } else {
                    log::warn!("ignoring setting {key}={s:?}");
                }
            }
            (Value::String(_), Value::String(_)) => *current = value.clone(),
            // Lists of text (the recent connections): the text entries.
            (Value::Array(_), Value::Array(items)) => {
                *current = Value::Array(items.iter().filter(|v| v.is_string()).cloned().collect());
            }
            _ => log::warn!("ignoring setting {key}={value}"),
        }
    }
}

pub fn default_path() -> PathBuf {
    config_dir().join("config.json")
}

pub fn load(path: Option<&Path>) -> Settings {
    let path = path.map(Path::to_path_buf).unwrap_or_else(default_path);
    match std::fs::read_to_string(&path) {
        Ok(text) => match serde_json::from_str::<Value>(&text) {
            Ok(value) => Settings::from_json(&value),
            Err(err) => {
                log::warn!("could not read {} ({err}); using defaults", path.display());
                Settings::default()
            }
        },
        Err(_) => Settings::default(),
    }
}

pub fn save(settings: &Settings, path: Option<&Path>) -> std::io::Result<PathBuf> {
    let path = path.map(Path::to_path_buf).unwrap_or_else(default_path);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let tmp = path.with_extension("tmp");
    let text = serde_json::to_string_pretty(&settings.to_json()).unwrap_or_default();
    std::fs::write(&tmp, text)?;
    std::fs::rename(&tmp, &path)?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partial_and_wrong_values_keep_defaults() {
        let data = serde_json::json!({
            "boat": {"length_m": 2.0, "beam_m": "wide"},
            "control": {"stick_mode": "tank", "deadzone": "0.1"},
            "spill": {"parcels": 800.0},
            "ui": {"show_hud": false, "water_style": "chart"},
            "unknown": {"x": 1}
        });
        let s = Settings::from_json(&data);
        assert_eq!(s.boat.length_m, 2.0);
        assert_eq!(s.boat.beam_m, 0.9);
        assert_eq!(s.control.stick_mode, StickMode::Tank);
        assert_eq!(s.control.deadzone, 0.1);
        assert_eq!(s.spill.parcels, 800);
        assert!(!s.ui.show_hud);
        assert_eq!(s.ui.water_style, WaterStyle::Chart);
    }

    #[test]
    fn unknown_enum_value_falls_back_to_defaults() {
        let s = Settings::from_json(&serde_json::json!({"control": {"stick_mode": "joystick", "expo": 0.5}}));
        assert_eq!(s.control.stick_mode, StickMode::Arcade);
        assert_eq!(s.control.expo, 0.5);
    }

    #[test]
    fn sound_rules_merge_with_the_defaults() {
        let s = Settings::from_json(&serde_json::json!({
            "sound": {"rules": {"shallow": {"repeat_s": 12.0}, "mode": {"enabled": false}, "bogus": {"enabled": false}}}
        }));
        assert_eq!(s.sound.rule("shallow").repeat_s, 12.0);
        assert!(s.sound.rule("shallow").tone, "unset fields keep their defaults");
        assert!(!s.sound.rule("mode").enabled);
        assert!(s.sound.rule("aground").enabled);
        assert!(!s.sound.rules.contains_key("bogus"));
    }

    #[test]
    fn round_trip() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("c.json");
        let mut s = Settings::default();
        s.sim.wind_speed_ms = 7.5;
        save(&s, Some(&path)).unwrap();
        assert_eq!(load(Some(&path)), s);
    }
}
