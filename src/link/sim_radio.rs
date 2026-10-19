//! The simulated boat's radios: Wi-Fi, 5G / LTE and LoRa side by side.
//!
//! Each path has a range from the ground station, a signal that fades with
//! distance, a background loss, a latency and a capacity.  The best working
//! path — Wi-Fi, then 5G, then LoRa — carries the telemetry and the
//! commands; the others carry a keep-alive a second each way, so their loss
//! is known and they can take over at once.  LoRa is too slow for the full
//! picture: on it the boat sends a compact report at 2 Hz and takes
//! commands at 5 Hz.
//!
//! Switching down (the path in use stops working) is immediate; switching
//! back up waits until the better path has worked for a few seconds, so a
//! boat on the edge of Wi-Fi does not flap between the two.

use crate::config::SimSettings;
use crate::core::rng::Rng;

use super::flows::{Bearer, FlowMeter, FlowStats};

/// A path whose loss is this high is no use for the main traffic.
const UNUSABLE_LOSS: f64 = 0.25;
/// A better path must have worked this long before traffic moves back to it.
const SWITCH_UP_S: f64 = 3.0;
/// Keep-alives on the paths standing by, every this many reports.
const KEEPALIVE_EVERY: u64 = 10;
const KEEPALIVE_BYTES: usize = 21;
const COMMAND_BYTES: usize = 38;

struct Path {
    meter: FlowMeter,
    base_loss: f64,
    latency_ms: f64,
    jitter_ms: f64,
    /// Telemetry every this many reports (of 10 a second), and its size.
    telemetry_every: u64,
    telemetry_bytes: usize,
    /// Commands: one in this many is sent.
    command_every: u64,
    /// Signal at the station and at the edge of range, dBm; SNR likewise.
    rssi: (f64, f64),
    snr: (f64, f64),
    usable_since: Option<f64>,
}

impl Path {
    fn range_m(&self, s: &SimSettings) -> f64 {
        match self.meter.bearer {
            Bearer::WiFi => s.wifi_range_m,
            Bearer::Cellular => s.cellular_range_m,
            _ => s.link_range_m,
        }
    }

    /// The chance a packet is lost at this distance.
    fn loss(&self, s: &SimSettings, distance_m: f64) -> f64 {
        if !self.meter.enabled {
            return 1.0;
        }
        let fraction = distance_m / self.range_m(s).max(1.0);
        let fade = if fraction < 0.8 { 0.0 } else { ((fraction - 0.8) / 0.4).min(1.0) };
        (self.base_loss + fade).min(1.0)
    }

    fn signal(&mut self, s: &SimSettings, distance_m: f64, rng: &mut Rng) {
        let fraction = (distance_m / self.range_m(s).max(1.0)).max(0.0);
        // Falls fast close in and slowly far out, as radio does.
        let fall = (1.0 + 9.0 * fraction).log10().min(1.3);
        self.meter.rssi_dbm = Some(self.rssi.0 - (self.rssi.0 - self.rssi.1) * fall + rng.gauss(0.0, 1.0));
        self.meter.snr_db = Some(self.snr.0 - (self.snr.0 - self.snr.1) * fall + rng.gauss(0.0, 0.5));
        let airtime = 8.0 * self.telemetry_bytes as f64 / self.meter.capacity_bps.unwrap_or(1e9) * 1000.0;
        self.meter.latency_ms = Some((self.latency_ms + airtime + rng.gauss(0.0, self.jitter_ms).abs()).max(0.5));
    }
}

pub struct SimRadio {
    paths: Vec<Path>,
    primary: Option<usize>,
    rng: Rng,
    tick: u64,
    commands: u64,
    pub messages: Vec<String>,
}

impl SimRadio {
    pub fn new(seed: u64) -> SimRadio {
        let path = |bearer: Bearer, name: &str, capacity: f64| {
            let mut meter = FlowMeter::new(bearer, name);
            meter.capacity_bps = Some(capacity);
            meter
        };
        SimRadio {
            paths: vec![
                Path {
                    meter: path(Bearer::WiFi, "Wi-Fi 5 GHz", 30e6),
                    base_loss: 0.003,
                    latency_ms: 3.0,
                    jitter_ms: 2.0,
                    telemetry_every: 1,
                    telemetry_bytes: 262,
                    command_every: 1,
                    rssi: (-35.0, -92.0),
                    snr: (55.0, 3.0),
                    usable_since: None,
                },
                Path {
                    meter: path(Bearer::Cellular, "5G / LTE modem", 50e6),
                    base_loss: 0.008,
                    latency_ms: 32.0,
                    jitter_ms: 14.0,
                    telemetry_every: 1,
                    telemetry_bytes: 262,
                    command_every: 1,
                    rssi: (-72.0, -121.0),
                    snr: (24.0, -4.0),
                    usable_since: None,
                },
                Path {
                    meter: path(Bearer::LoRa, "LoRa 868 MHz SF7", 5470.0),
                    base_loss: 0.012,
                    latency_ms: 25.0,
                    jitter_ms: 20.0,
                    telemetry_every: 5,
                    telemetry_bytes: 48,
                    command_every: 4,
                    rssi: (-58.0, -128.0),
                    snr: (11.0, -17.0),
                    usable_since: None,
                },
            ],
            primary: None,
            rng: Rng::new(seed),
            tick: 0,
            commands: 0,
            messages: Vec::new(),
        }
    }

    pub fn primary(&self) -> Option<Bearer> {
        self.primary.map(|i| self.paths[i].meter.bearer)
    }

    pub fn rssi_dbm(&self) -> Option<f64> {
        self.primary.and_then(|i| self.paths[i].meter.rssi_dbm)
    }

    pub fn set_enabled(&mut self, index: usize, on: bool) {
        if let Some(p) = self.paths.get_mut(index) {
            p.meter.enabled = on;
        }
    }

    /// Choose the path for the main traffic.
    fn choose(&mut self, s: &SimSettings, distance_m: f64, now: f64) {
        let usable: Vec<bool> = self.paths.iter().map(|p| p.loss(s, distance_m) < UNUSABLE_LOSS).collect();
        for (p, ok) in self.paths.iter_mut().zip(&usable) {
            if *ok {
                p.usable_since.get_or_insert(now);
            } else {
                p.usable_since = None;
            }
        }
        let current_ok = self.primary.is_some_and(|i| usable[i]);
        let chosen = if current_ok {
            // Up to a better path only once it has proved itself.
            let current = self.primary.unwrap();
            (0..current).find(|&i| self.paths[i].usable_since.is_some_and(|t| now - t >= SWITCH_UP_S)).or(Some(current))
        } else {
            // Down at once: to the best path that works, or failing that the
            // least bad one still switched on.
            (0..self.paths.len()).find(|&i| usable[i]).or_else(|| {
                (0..self.paths.len())
                    .filter(|&i| self.paths[i].meter.enabled)
                    .min_by(|&a, &b| self.paths[a].loss(s, distance_m).total_cmp(&self.paths[b].loss(s, distance_m)))
            })
        };
        if chosen != self.primary {
            if let (Some(old), Some(new)) = (self.primary, chosen) {
                let why = if usable[old] { "a better path is back" } else { "the old one stopped working" };
                self.messages.push(format!("Link: now on {} — {}", self.paths[new].meter.bearer.label(), why));
            }
            self.primary = chosen;
        }
        for (i, p) in self.paths.iter_mut().enumerate() {
            p.meter.active = Some(i) == self.primary;
        }
    }

    /// One report slot (10 a second).  Returns whether a telemetry report
    /// reached the station.
    pub fn report(&mut self, s: &SimSettings, distance_m: f64, now: f64) -> bool {
        self.choose(s, distance_m, now);
        let tick = self.tick;
        self.tick += 1;
        let mut delivered = false;
        for i in 0..self.paths.len() {
            let loss = self.paths[i].loss(s, distance_m);
            let p = &mut self.paths[i];
            if !p.meter.enabled {
                continue;
            }
            p.signal(s, distance_m, &mut self.rng);
            let keepalive = tick % KEEPALIVE_EVERY == 0;
            let bytes = if Some(i) == self.primary {
                (tick % p.telemetry_every == 0).then_some(p.telemetry_bytes)
            } else {
                keepalive.then_some(KEEPALIVE_BYTES)
            };
            if let Some(bytes) = bytes {
                if self.rng.random() >= loss {
                    p.meter.received(now, bytes);
                    delivered |= Some(i) == self.primary;
                } else {
                    p.meter.lost(now, 1);
                }
            }
            // The station's keep-alive the other way.
            if keepalive && Some(i) != self.primary {
                p.meter.sent(now, KEEPALIVE_BYTES);
            }
        }
        delivered
    }

    /// A command up to the boat.  Returns whether it arrived.
    pub fn command(&mut self, s: &SimSettings, distance_m: f64, now: f64) -> bool {
        let Some(i) = self.primary else { return false };
        let n = self.commands;
        self.commands += 1;
        if n % self.paths[i].command_every != 0 {
            return false; // not sent: the path is too slow for every one
        }
        let loss = self.paths[i].loss(s, distance_m);
        self.paths[i].meter.sent(now, COMMAND_BYTES);
        self.rng.random() >= loss
    }

    pub fn flows(&mut self, now: f64) -> Vec<FlowStats> {
        self.paths.iter_mut().map(|p| p.meter.stats(now)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(radio: &mut SimRadio, s: &SimSettings, distance_m: f64, from: f64, seconds: f64) -> f64 {
        let mut t = from;
        while t < from + seconds {
            radio.report(s, distance_m, t);
            radio.command(s, distance_m, t);
            radio.command(s, distance_m, t + 0.05);
            t += 0.1;
        }
        t
    }

    #[test]
    fn the_best_path_carries_the_traffic_and_hands_over() {
        let s = SimSettings::default();
        let mut r = SimRadio::new(3);
        let t = run(&mut r, &s, 50.0, 0.0, 6.0);
        assert_eq!(r.primary(), Some(Bearer::WiFi));
        let f = r.flows(t);
        assert!(f[0].rx_bps > 15_000.0, "Wi-Fi carries 10 Hz telemetry: {}", f[0].rx_bps);
        assert!(f[1].rx_bps < 300.0 && f[1].rx_packets > 0, "5G stands by with keep-alives");

        // Beyond Wi-Fi: 5G at once.
        let t = run(&mut r, &s, s.wifi_range_m * 1.3, t, 6.0);
        assert_eq!(r.primary(), Some(Bearer::Cellular));
        assert!(r.messages.iter().any(|m| m.contains("5G")));
        // Beyond 5G too: LoRa, at its slower rate.
        let t = run(&mut r, &s, s.cellular_range_m * 1.3, t, 8.0);
        assert_eq!(r.primary(), Some(Bearer::LoRa));
        let f = r.flows(t);
        assert!(f[2].rx_bps < 1200.0 && f[2].rx_bps > 400.0, "LoRa: 2 Hz of 48 bytes: {}", f[2].rx_bps);
        assert!(f[0].loss_pct > 90.0, "Wi-Fi is lost out here: {}", f[0].loss_pct);

        // Back in close: Wi-Fi again, but only after it has held a while.
        let t2 = run(&mut r, &s, 50.0, t, 1.0);
        assert_eq!(r.primary(), Some(Bearer::LoRa), "no flapping");
        run(&mut r, &s, 50.0, t2, 4.0);
        assert_eq!(r.primary(), Some(Bearer::WiFi));
    }

    #[test]
    fn a_path_switched_off_is_never_used() {
        let s = SimSettings::default();
        let mut r = SimRadio::new(5);
        r.set_enabled(0, false);
        let t = run(&mut r, &s, 10.0, 0.0, 5.0);
        assert_eq!(r.primary(), Some(Bearer::Cellular));
        assert_eq!(r.flows(t)[0].state, super::super::flows::FlowState::Off);
    }

    #[test]
    fn loss_at_the_edge_of_range_is_measured() {
        let s = SimSettings::default();
        let mut r = SimRadio::new(7);
        r.set_enabled(0, false);
        r.set_enabled(1, false);
        // LoRa at its full range: about half lost.
        let t = run(&mut r, &s, s.link_range_m, 0.0, 120.0);
        let lora = &r.flows(t)[2];
        let total = 100.0 * lora.lost_packets as f64 / (lora.lost_packets + lora.rx_packets) as f64;
        assert!((40.0..62.0).contains(&total), "{total}");
        assert!(lora.rssi_dbm.unwrap() < -110.0);
    }
}
