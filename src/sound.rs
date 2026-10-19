//! Spoken alerts, the way an aircraft's cockpit calls them.
//!
//! The voices are the EdgeTX radio sound packs
//! (<https://github.com/EdgeTX/edgetx-sdcard-sounds>, GPL-2.0), kept as
//! separate files in `sounds/<language>/` and fetched with `usvgs sounds`.
//! English and Russian are wired up.  A phrase a pack lacks is left out, and
//! the danger calls are preceded by a synthesised two-tone "whoop", like a
//! ground-proximity warning:
//!
//! * running into shallow water: *whoop — "Danger, danger"* (in Russian,
//!   *"Слишком низко"*, as a Russian GPWS calls "too low"), repeated every
//!   few seconds while it lasts;
//! * aground: the same, then *"Stop"*;
//! * link lost and recovered, battery low and critical, geofence, GNSS lost,
//!   armed and disarmed, and the mode changes.

use std::collections::HashSet;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Sender};

use crate::core::clock;
use crate::vehicle::{Mode, NavState};

/// Where the packs come from.
pub const PACK_URL: &str = "https://raw.githubusercontent.com/EdgeTX/edgetx-sdcard-sounds/main/SOUNDS";
/// Danger calls repeat this often while the danger lasts, seconds.
pub const REPEAT_S: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Alert {
    Armed,
    Disarmed,
    Mode(ModeCall),
    LinkLost,
    LinkRestored,
    BatteryLow,
    BatteryCritical,
    Shallow,
    Aground,
    Geofence,
    GnssLost,
    GnssFound,
    TankFull,
}

/// Modes have their own calls; this is [`Mode`] without the data.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ModeCall {
    Manual,
    SpeedHold,
    CourseHold,
    PointNav,
    Auto,
    Hold,
    ReturnHome,
    Stop,
}

impl From<Mode> for ModeCall {
    fn from(m: Mode) -> Self {
        match m {
            Mode::Manual => ModeCall::Manual,
            Mode::SpeedHold => ModeCall::SpeedHold,
            Mode::CourseHold => ModeCall::CourseHold,
            Mode::PointNav => ModeCall::PointNav,
            Mode::Auto => ModeCall::Auto,
            Mode::Hold => ModeCall::Hold,
            Mode::ReturnHome => ModeCall::ReturnHome,
            Mode::Stop => ModeCall::Stop,
        }
    }
}

/// One piece of a call: a phrase from the pack, or a synthesised sound.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Clip {
    Phrase(&'static str),
    /// The rising two-tone "whoop" of a terrain warning.
    Whoop,
    /// A short chime, for a call the pack has no words for.
    Chime,
}

use Clip::{Chime, Phrase, Whoop};

impl Alert {
    /// By name, for `usvgs say`.
    pub fn parse(name: &str) -> Option<Alert> {
        Some(match name {
            "armed" => Alert::Armed,
            "disarmed" => Alert::Disarmed,
            "link-lost" => Alert::LinkLost,
            "link-restored" => Alert::LinkRestored,
            "battery-low" => Alert::BatteryLow,
            "battery-critical" => Alert::BatteryCritical,
            "shallow" => Alert::Shallow,
            "aground" => Alert::Aground,
            "geofence" => Alert::Geofence,
            "gnss-lost" => Alert::GnssLost,
            "gnss-found" => Alert::GnssFound,
            "tank-full" => Alert::TankFull,
            "hold" => Alert::Mode(ModeCall::Hold),
            "return-home" => Alert::Mode(ModeCall::ReturnHome),
            "stop" => Alert::Mode(ModeCall::Stop),
            "manual" => Alert::Mode(ModeCall::Manual),
            _ => return None,
        })
    }

    /// Seconds of audio a call takes, roughly, to wait for it to finish.
    pub fn rough_length_s(self) -> f64 {
        self.clips("en").len() as f64 * 1.2 + 1.0
    }

    /// What to play, in a language (`en` or `ru`).
    pub fn clips(self, language: &str) -> Vec<Clip> {
        let ru = language == "ru";
        match self {
            Alert::Armed => vec![Phrase("armed.wav")],
            Alert::Disarmed => vec![Phrase("disarm.wav")],
            Alert::Mode(m) => match (m, ru) {
                (ModeCall::Manual, false) => vec![Phrase("manmod.wav")],
                (ModeCall::Manual, true) => vec![Phrase("fm-man.wav")],
                (ModeCall::SpeedHold, false) => vec![Phrase("speed.wav")],
                (ModeCall::CourseHold, false) => vec![Phrase("crshld.wav")],
                (ModeCall::CourseHold, true) => vec![Phrase("fm-crs.wav")],
                (ModeCall::PointNav | ModeCall::Auto, false) => vec![Phrase("navwp.wav")],
                (ModeCall::Hold, false) => vec![Phrase("poshld.wav")],
                (ModeCall::Hold, true) => vec![Phrase("fm-hld.wav")],
                (ModeCall::ReturnHome, false) => vec![Phrase("rth.wav")],
                (ModeCall::ReturnHome, true) => vec![Phrase("fm-rth.wav")],
                (ModeCall::Stop, false) => vec![Phrase("stop.wav")],
                (ModeCall::Stop, true) => vec![Phrase("fson.wav")],
                _ => vec![Chime],
            },
            Alert::LinkLost => vec![Phrase("SYSTEM/telemko.wav")],
            Alert::LinkRestored => vec![Phrase("SYSTEM/telemok.wav")],
            Alert::BatteryLow => vec![Phrase("lowbat.wav")],
            Alert::BatteryCritical if ru => vec![Phrase("batcrt.wav")],
            Alert::BatteryCritical => vec![Phrase("clobat.wav")],
            Alert::Shallow if ru => vec![Phrase("tolow.wav"), Phrase("tolow.wav")],
            Alert::Shallow => vec![Phrase("danger.wav"), Phrase("danger.wav")],
            Alert::Aground if ru => vec![Phrase("tolow.wav"), Phrase("tolow.wav"), Phrase("fson.wav")],
            Alert::Aground => vec![Phrase("danger.wav"), Phrase("danger.wav"), Phrase("stop.wav")],
            Alert::Geofence if ru => vec![Phrase("fm-rth.wav")],
            Alert::Geofence => vec![Phrase("warnng.wav"), Phrase("rth.wav")],
            Alert::GnssLost if ru => vec![Phrase("satlst.wav")],
            Alert::GnssLost => vec![Phrase("gps.wav"), Phrase("SYSTEM/sensorko.wav")],
            Alert::GnssFound if ru => vec![Phrase("satfix.wav")],
            Alert::GnssFound => vec![Phrase("gps.wav"), Phrase("ready.wav")],
            Alert::TankFull if ru => vec![Chime, Chime],
            Alert::TankFull => vec![Chime, Phrase("warnng.wav")],
        }
    }

    /// The name of the rule that governs it (see [`crate::config::SOUND_ALERTS`]).
    pub fn key(self) -> &'static str {
        match self {
            Alert::Armed | Alert::Disarmed => "arming",
            Alert::Mode(_) => "mode",
            Alert::LinkLost => "link_lost",
            Alert::LinkRestored => "link_restored",
            Alert::BatteryLow => "battery_low",
            Alert::BatteryCritical => "battery_critical",
            Alert::Shallow => "shallow",
            Alert::Aground => "aground",
            Alert::Geofence => "geofence",
            Alert::GnssLost => "gnss_lost",
            Alert::GnssFound => "gnss_found",
            Alert::TankFull => "tank_full",
        }
    }

    /// An alert to try a rule with, by the rule's name.
    pub fn for_rule(key: &str) -> Option<Alert> {
        Some(match key {
            "arming" => Alert::Armed,
            "mode" => Alert::Mode(ModeCall::Hold),
            "link_lost" => Alert::LinkLost,
            "link_restored" => Alert::LinkRestored,
            "battery_low" => Alert::BatteryLow,
            "battery_critical" => Alert::BatteryCritical,
            "shallow" => Alert::Shallow,
            "aground" => Alert::Aground,
            "geofence" => Alert::Geofence,
            "gnss_lost" => Alert::GnssLost,
            "gnss_found" => Alert::GnssFound,
            "tank_full" => Alert::TankFull,
            _ => return None,
        })
    }

    /// The vehicle alarm that holds this alert, for alerts that last.
    pub fn alarm(self) -> Option<&'static str> {
        Some(match self {
            Alert::LinkLost => "link",
            Alert::BatteryLow => "battery_low",
            Alert::BatteryCritical => "battery_critical",
            Alert::Shallow => "shallow",
            Alert::Aground => "grounded",
            Alert::Geofence => "geofence",
            Alert::GnssLost => "gnss",
            Alert::TankFull => "tank_full",
            _ => return None,
        })
    }
}

/// Every phrase a language uses, for fetching the pack.
pub fn phrases(language: &str) -> Vec<&'static str> {
    let mut alerts = vec![
        Alert::Armed,
        Alert::Disarmed,
        Alert::LinkLost,
        Alert::LinkRestored,
        Alert::BatteryLow,
        Alert::BatteryCritical,
        Alert::Shallow,
        Alert::Aground,
        Alert::Geofence,
        Alert::GnssLost,
        Alert::GnssFound,
        Alert::TankFull,
    ];
    for m in [
        ModeCall::Manual,
        ModeCall::SpeedHold,
        ModeCall::CourseHold,
        ModeCall::PointNav,
        ModeCall::Auto,
        ModeCall::Hold,
        ModeCall::ReturnHome,
        ModeCall::Stop,
    ] {
        alerts.push(Alert::Mode(m));
    }
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for a in alerts {
        for c in a.clips(language) {
            if let Phrase(p) = c {
                if seen.insert(p) {
                    out.push(p);
                }
            }
        }
    }
    out
}

/// The folder holding the packs, searching the usual places.
pub fn sounds_dir() -> PathBuf {
    let mut candidates = vec![PathBuf::from("sounds")];
    if let Ok(exe) = std::env::current_exe() {
        for up in exe.ancestors().skip(1).take(4) {
            candidates.push(up.join("sounds"));
        }
    }
    candidates.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("sounds"));
    candidates.iter().find(|p| p.is_dir()).cloned().unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("sounds"))
}

/// Download a language's phrases into `dir/<language>/`.  `voice` is the
/// pack's folder in the EdgeTX repository (`en`, `ru`, `en_us-sara`, …).
pub fn fetch(dir: &Path, language: &str, voice: &str) -> Result<usize, String> {
    let mut count = 0;
    for phrase in phrases(language) {
        let target = dir.join(language).join(phrase);
        if let Some(parent) = target.parent() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        let url = format!("{PACK_URL}/{voice}/{phrase}");
        let response = ureq::get(&url).call().map_err(|e| format!("{url}: {e}"))?;
        let mut bytes = Vec::new();
        response.into_reader().read_to_end(&mut bytes).map_err(|e| e.to_string())?;
        std::fs::write(&target, bytes).map_err(|e| e.to_string())?;
        eprintln!("  {}", target.display());
        count += 1;
    }
    Ok(count)
}

// --------------------------------------------------------------------------
// Synthesised sounds
// --------------------------------------------------------------------------

pub const RATE: u32 = 16_000;

/// Two rising whoops, 400 → 1200 Hz, as a terrain warning sounds.
pub fn whoop() -> Vec<f32> {
    let mut out = Vec::new();
    for _ in 0..2 {
        let n = (RATE as f32 * 0.42) as usize;
        let mut phase = 0.0f32;
        for i in 0..n {
            let t = i as f32 / n as f32;
            let f = 400.0 + 800.0 * t * t;
            phase += std::f32::consts::TAU * f / RATE as f32;
            let envelope = (t * 12.0).min(1.0) * (1.0 - t).powf(0.3);
            // A square-ish tone, as cockpit alerts are, softened a little.
            out.push(0.45 * envelope * (phase.sin() + 0.3 * (3.0 * phase).sin()));
        }
        out.extend(std::iter::repeat_n(0.0, (RATE as f32 * 0.08) as usize));
    }
    out
}

/// Two short notes, a fourth apart.
pub fn chime() -> Vec<f32> {
    let mut out = Vec::new();
    for f in [880.0f32, 1174.7] {
        let n = (RATE as f32 * 0.16) as usize;
        for i in 0..n {
            let t = i as f32 / n as f32;
            out.push(0.35 * (1.0 - t).powi(2) * (std::f32::consts::TAU * f * i as f32 / RATE as f32).sin());
        }
    }
    out
}

// --------------------------------------------------------------------------
// Playback
// --------------------------------------------------------------------------

enum Command {
    Play(Vec<Clip>, PathBuf),
    Volume(f32),
}

/// The speaker: a thread that owns the audio output and plays calls in turn.
pub struct Voice {
    tx: Option<Sender<Command>>,
}

impl Voice {
    /// Open the audio output.  Without one the station runs silent.
    pub fn new(volume: f32) -> Voice {
        let (tx, rx) = channel::<Command>();
        let (ready_tx, ready_rx) = channel::<bool>();
        let spawned = std::thread::Builder::new().name("voice".into()).spawn(move || {
            let Ok((_stream, handle)) = rodio::OutputStream::try_default() else {
                let _ = ready_tx.send(false);
                return;
            };
            let Ok(sink) = rodio::Sink::try_new(&handle) else {
                let _ = ready_tx.send(false);
                return;
            };
            sink.set_volume(volume);
            let _ = ready_tx.send(true);
            while let Ok(command) = rx.recv() {
                match command {
                    Command::Volume(v) => sink.set_volume(v),
                    Command::Play(clips, dir) => {
                        for clip in clips {
                            match clip {
                                Whoop => sink.append(rodio::buffer::SamplesBuffer::new(1, RATE, whoop())),
                                Chime => sink.append(rodio::buffer::SamplesBuffer::new(1, RATE, chime())),
                                Phrase(p) => {
                                    let path = dir.join(p);
                                    match std::fs::read(&path).map(std::io::Cursor::new).map(rodio::Decoder::new) {
                                        Ok(Ok(d)) => sink.append(d),
                                        Ok(Err(e)) => log::warn!("cannot play {}: {e}", path.display()),
                                        Err(_) => log::debug!("no sound {}", path.display()),
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });
        let ok = spawned.is_ok() && ready_rx.recv_timeout(std::time::Duration::from_secs(3)).unwrap_or(false);
        if !ok {
            log::warn!("no audio output: voice alerts are off");
        }
        Voice { tx: ok.then_some(tx) }
    }

    pub fn available(&self) -> bool {
        self.tx.is_some()
    }

    pub fn set_volume(&self, volume: f32) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::Volume(volume));
        }
    }

    /// Speak an alert, after the warning tone if `tone`.
    pub fn say(&self, alert: Alert, language: &str, tone: bool) {
        if let Some(tx) = &self.tx {
            let mut clips = alert.clips(language);
            if tone {
                clips.insert(0, Whoop);
            }
            let _ = tx.send(Command::Play(clips, sounds_dir().join(language)));
        }
    }
}

// --------------------------------------------------------------------------
// What to say
// --------------------------------------------------------------------------

/// Alerts that last, most urgent first: when several are due to be said
/// again, only the first is, so they do not pile up.
pub const PRIORITY: [Alert; 8] = [
    Alert::Aground,
    Alert::Shallow,
    Alert::LinkLost,
    Alert::BatteryCritical,
    Alert::Geofence,
    Alert::GnssLost,
    Alert::BatteryLow,
    Alert::TankFull,
];

/// A call to make, and whether the warning tone goes first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Call {
    pub alert: Alert,
    pub tone: bool,
}

/// Watches the vehicle and decides which calls to make, by the rules: once
/// when something changes, and again every so often while a warning lasts.
#[derive(Default)]
pub struct Announcer {
    last: Option<(bool, Mode)>,
    alarms: HashSet<&'static str>,
    said_at: std::collections::HashMap<&'static str, f64>,
}

impl Announcer {
    pub fn new() -> Self {
        Self::default()
    }

    /// The calls for this moment.  `now` is monotonic seconds.
    pub fn update(&mut self, nav: &NavState, now: f64, rules: &crate::config::SoundSettings) -> Vec<Call> {
        let mut calls: Vec<Alert> = Vec::new();
        if let Some((armed, mode)) = self.last {
            if nav.armed != armed {
                calls.push(if nav.armed { Alert::Armed } else { Alert::Disarmed });
            } else if nav.mode != mode {
                calls.push(Alert::Mode(nav.mode.into()));
            }
        }
        let first = self.last.is_none();
        self.last = Some((nav.armed, nav.mode));

        let current: HashSet<&'static str> = nav.alarms.iter().map(|a| a.key).collect();
        let lasting = |alert: Alert| alert.alarm().is_some_and(|k| current.contains(k));
        // Aground says everything shallow water would, and more.
        let shadowed = |alert: Alert| alert == Alert::Shallow && lasting(Alert::Aground);
        for alert in PRIORITY {
            let key = alert.alarm().unwrap();
            let fresh = current.contains(key) && !self.alarms.contains(key);
            // At start-up only the dangers are worth saying.
            if fresh && !shadowed(alert) && (!first || matches!(alert, Alert::Shallow | Alert::Aground)) {
                calls.push(alert);
            }
        }
        if self.alarms.contains("link") && !current.contains("link") {
            calls.push(Alert::LinkRestored);
        }
        if self.alarms.contains("gnss") && !current.contains("gnss") {
            calls.push(Alert::GnssFound);
        }

        let mut out: Vec<Call> = Vec::new();
        for alert in calls {
            let rule = rules.rule(alert.key());
            if rule.enabled {
                self.said_at.insert(alert.key(), now);
                out.push(Call { alert, tone: rule.tone });
            }
        }
        // The most urgent lasting warning that is due, said again.
        let may_repeat = nav.armed || !rules.repeat_only_armed;
        if may_repeat && !out.iter().any(|c| c.alert.alarm().is_some()) {
            for alert in PRIORITY {
                let rule = rules.rule(alert.key());
                if !lasting(alert) || shadowed(alert) || !rule.enabled || rule.repeat_s <= 0.0 {
                    continue;
                }
                if self.said_at.get(alert.key()).is_none_or(|t| now - t >= rule.repeat_s) {
                    self.said_at.insert(alert.key(), now);
                    out.push(Call { alert, tone: rule.tone });
                    break;
                }
            }
        }
        self.alarms = current;
        out
    }
}

/// Monotonic now, for callers without a clock of their own.
pub fn now() -> f64 {
    clock::monotonic()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::events::Level;
    use crate::vehicle::Alarm;

    fn nav(armed: bool, mode: Mode, alarms: &[&'static str]) -> NavState {
        NavState {
            armed,
            mode,
            link_ok: true,
            alarms: alarms.iter().map(|k| Alarm { key: k, text: String::new(), level: Level::Error }).collect(),
            ..NavState::default()
        }
    }

    fn alerts(calls: Vec<Call>) -> Vec<Alert> {
        calls.into_iter().map(|c| c.alert).collect()
    }

    #[test]
    fn danger_is_called_and_repeated_while_it_lasts() {
        let rules = crate::config::SoundSettings::default();
        let mut a = Announcer::new();
        assert!(a.update(&nav(true, Mode::Auto, &[]), 0.0, &rules).is_empty());
        let calls = a.update(&nav(true, Mode::Auto, &["shallow"]), 1.0, &rules);
        assert_eq!(calls, vec![Call { alert: Alert::Shallow, tone: true }]);
        assert!(a.update(&nav(true, Mode::Auto, &["shallow"]), 3.0, &rules).is_empty());
        assert_eq!(alerts(a.update(&nav(true, Mode::Auto, &["shallow"]), 6.5, &rules)), vec![Alert::Shallow]);
        // Aground outranks shallow.
        assert_eq!(alerts(a.update(&nav(true, Mode::Auto, &["shallow", "grounded"]), 7.0, &rules)), vec![Alert::Aground]);
        assert!(a.update(&nav(true, Mode::Auto, &["shallow", "grounded"]), 9.0, &rules).is_empty());
        assert_eq!(alerts(a.update(&nav(true, Mode::Auto, &["shallow", "grounded"]), 12.5, &rules)), vec![Alert::Aground]);
        assert!(a.update(&nav(true, Mode::Auto, &[]), 13.0, &rules).is_empty());
    }

    #[test]
    fn the_rules_decide() {
        let mut rules = crate::config::SoundSettings::default();
        rules.rules.get_mut("shallow").unwrap().repeat_s = 2.0;
        rules.rules.get_mut("shallow").unwrap().tone = false;
        rules.rules.get_mut("mode").unwrap().enabled = false;
        let mut a = Announcer::new();
        a.update(&nav(true, Mode::Manual, &[]), 0.0, &rules);
        assert!(a.update(&nav(true, Mode::Hold, &[]), 1.0, &rules).is_empty(), "mode calls are off");
        assert_eq!(a.update(&nav(true, Mode::Hold, &["shallow"]), 2.0, &rules), vec![Call { alert: Alert::Shallow, tone: false }]);
        assert_eq!(alerts(a.update(&nav(true, Mode::Hold, &["shallow"]), 4.1, &rules)), vec![Alert::Shallow]);
        // Disarmed, it is said once and not again.
        let mut b = Announcer::new();
        b.update(&nav(false, Mode::Manual, &[]), 0.0, &rules);
        assert_eq!(alerts(b.update(&nav(false, Mode::Manual, &["grounded"]), 1.0, &rules)), vec![Alert::Aground]);
        assert!(b.update(&nav(false, Mode::Manual, &["grounded"]), 60.0, &rules).is_empty());
        rules.repeat_only_armed = false;
        assert_eq!(alerts(b.update(&nav(false, Mode::Manual, &["grounded"]), 61.0, &rules)), vec![Alert::Aground]);
    }

    #[test]
    fn arming_modes_and_link_are_called_once() {
        let rules = crate::config::SoundSettings::default();
        let mut a = Announcer::new();
        a.update(&nav(false, Mode::Manual, &[]), 0.0, &rules);
        assert_eq!(alerts(a.update(&nav(true, Mode::Manual, &[]), 1.0, &rules)), vec![Alert::Armed]);
        assert_eq!(alerts(a.update(&nav(true, Mode::Hold, &[]), 2.0, &rules)), vec![Alert::Mode(ModeCall::Hold)]);
        assert_eq!(alerts(a.update(&nav(true, Mode::Hold, &["link"]), 3.0, &rules)), vec![Alert::LinkLost]);
        assert_eq!(alerts(a.update(&nav(true, Mode::Hold, &[]), 4.0, &rules)), vec![Alert::LinkRestored]);
    }

    #[test]
    fn every_rule_has_an_alert() {
        for (key, ..) in crate::config::SOUND_ALERTS {
            let alert = Alert::for_rule(key).unwrap();
            assert_eq!(alert.key(), key);
        }
    }

    #[test]
    fn danger_sounds_like_an_aircraft() {
        assert_eq!(Alert::Shallow.clips("en"), vec![Phrase("danger.wav"), Phrase("danger.wav")]);
        assert_eq!(Alert::Shallow.clips("ru")[0], Phrase("tolow.wav"));
        assert!(crate::config::SoundSettings::default().rule("shallow").tone);
        assert!(phrases("en").contains(&"danger.wav") && phrases("ru").contains(&"tolow.wav"));
        assert!(!whoop().is_empty() && whoop().iter().all(|s| s.abs() <= 1.0));
    }

    #[test]
    fn every_phrase_decodes() {
        let dir = sounds_dir();
        for lang in ["en", "ru"] {
            for p in phrases(lang) {
                let bytes = std::fs::read(dir.join(lang).join(p)).unwrap();
                let decoder = rodio::Decoder::new(std::io::Cursor::new(bytes));
                assert!(decoder.is_ok(), "{lang}/{p} does not decode");
            }
        }
    }

    #[test]
    fn bundled_packs_have_every_phrase() {
        let dir = sounds_dir();
        for lang in ["en", "ru"] {
            for p in phrases(lang) {
                assert!(dir.join(lang).join(p).exists(), "missing {lang}/{p}: run `usvgs sounds`");
            }
        }
    }
}
