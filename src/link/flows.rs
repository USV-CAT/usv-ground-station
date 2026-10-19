//! Data flows: the paths the link's data travels by — LoRa, Wi-Fi, 5G and
//! so on — each measured for what it carries and what it loses.
//!
//! A [`FlowMeter`] counts the packets each way on one path and turns them
//! into a [`FlowStats`]: data rates over the last few seconds, packet loss,
//! signal, latency, and a two-minute history for the graphs.

use std::collections::VecDeque;

/// What carries a flow.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Bearer {
    WiFi,
    Cellular,
    LoRa,
    /// A telemetry radio (SiK, RFD) on a serial port.
    Radio,
    Satellite,
    /// Ethernet or any other IP path.
    Ip,
}

impl Bearer {
    pub const ALL: [Bearer; 6] = [Bearer::WiFi, Bearer::Cellular, Bearer::LoRa, Bearer::Radio, Bearer::Satellite, Bearer::Ip];

    pub fn label(self) -> &'static str {
        match self {
            Bearer::WiFi => "Wi-Fi",
            Bearer::Cellular => "5G / LTE",
            Bearer::LoRa => "LoRa",
            Bearer::Radio => "Telemetry radio",
            Bearer::Satellite => "Satellite",
            Bearer::Ip => "Network",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            Bearer::WiFi => "📶",
            Bearer::Cellular => "📱",
            Bearer::LoRa => "📡",
            Bearer::Radio => "📻",
            Bearer::Satellite => "🛰",
            Bearer::Ip => "🔗",
        }
    }

    /// As written in the settings: wifi, cellular, lora, radio, satellite, ip.
    pub fn key(self) -> &'static str {
        match self {
            Bearer::WiFi => "wifi",
            Bearer::Cellular => "cellular",
            Bearer::LoRa => "lora",
            Bearer::Radio => "radio",
            Bearer::Satellite => "satellite",
            Bearer::Ip => "ip",
        }
    }

    pub fn parse(text: &str) -> Option<Bearer> {
        Some(match text.trim().to_ascii_lowercase().as_str() {
            "wifi" | "wi-fi" | "wlan" => Bearer::WiFi,
            "cellular" | "5g" | "4g" | "lte" | "cell" => Bearer::Cellular,
            "lora" => Bearer::LoRa,
            "radio" | "sik" | "rfd" => Bearer::Radio,
            "satellite" | "sat" | "starlink" | "iridium" => Bearer::Satellite,
            "ip" | "ethernet" | "net" => Bearer::Ip,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlowState {
    /// Carrying the telemetry and the commands.
    Active,
    /// Up, carrying only keep-alives, ready to take over.
    Standby,
    /// Nothing heard lately.
    Down,
    /// Switched off.
    Off,
}

impl FlowState {
    pub fn label(self) -> &'static str {
        match self {
            FlowState::Active => "ACTIVE",
            FlowState::Standby => "STANDBY",
            FlowState::Down => "DOWN",
            FlowState::Off => "OFF",
        }
    }
}

/// One point of a flow's history, a second apart.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlowSample {
    pub t: f64,
    pub rx_bps: f64,
    pub tx_bps: f64,
    pub loss_pct: f64,
}

/// A flow as the displays see it.
#[derive(Clone, Debug, PartialEq)]
pub struct FlowStats {
    pub bearer: Bearer,
    /// What the operator calls it, e.g. "LoRa 868 MHz" or "udp-listen:14550".
    pub name: String,
    pub state: FlowState,
    /// Boat to station and station to boat, bits a second.
    pub rx_bps: f64,
    pub tx_bps: f64,
    /// Packets a second each way.
    pub rx_pps: f64,
    pub tx_pps: f64,
    /// Packets lost of those sent our way, over the window.
    pub loss_pct: f64,
    pub rx_packets: u64,
    pub tx_packets: u64,
    pub lost_packets: u64,
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub rssi_dbm: Option<f64>,
    pub snr_db: Option<f64>,
    pub latency_ms: Option<f64>,
    /// What the path can carry, bits a second, when known.
    pub capacity_bps: Option<f64>,
    /// Seconds since anything came in on it.
    pub silent_s: Option<f64>,
    pub history: Vec<FlowSample>,
}

/// Rates and loss are worked out over this many seconds.
pub const WINDOW_S: f64 = 5.0;
/// How much history the graphs keep.
pub const HISTORY_S: f64 = 120.0;
/// A flow with nothing heard for this long is down.
pub const DOWN_AFTER_S: f64 = 3.0;

#[derive(Clone, Copy, Debug)]
enum Event {
    Rx(u32),
    Tx(u32),
    Lost(u32),
}

/// Counts one flow's traffic.
#[derive(Clone, Debug)]
pub struct FlowMeter {
    pub bearer: Bearer,
    pub name: String,
    events: VecDeque<(f64, Event)>,
    rx_packets: u64,
    tx_packets: u64,
    lost_packets: u64,
    rx_bytes: u64,
    tx_bytes: u64,
    last_rx: Option<f64>,
    history: VecDeque<FlowSample>,
    last_sample: f64,
    /// The last sequence number seen from each sender, for counting gaps.
    sequences: Vec<(u16, u8)>,
    pub enabled: bool,
    pub active: bool,
    pub rssi_dbm: Option<f64>,
    pub snr_db: Option<f64>,
    pub latency_ms: Option<f64>,
    pub capacity_bps: Option<f64>,
}

impl FlowMeter {
    pub fn new(bearer: Bearer, name: impl Into<String>) -> FlowMeter {
        FlowMeter {
            bearer,
            name: name.into(),
            events: VecDeque::new(),
            rx_packets: 0,
            tx_packets: 0,
            lost_packets: 0,
            rx_bytes: 0,
            tx_bytes: 0,
            last_rx: None,
            history: VecDeque::new(),
            last_sample: f64::NEG_INFINITY,
            sequences: Vec::new(),
            enabled: true,
            active: false,
            rssi_dbm: None,
            snr_db: None,
            latency_ms: None,
            capacity_bps: None,
        }
    }

    pub fn received(&mut self, now: f64, bytes: usize) {
        self.rx_packets += 1;
        self.rx_bytes += bytes as u64;
        self.last_rx = Some(now);
        self.push(now, Event::Rx(bytes as u32));
    }

    pub fn sent(&mut self, now: f64, bytes: usize) {
        self.tx_packets += 1;
        self.tx_bytes += bytes as u64;
        self.push(now, Event::Tx(bytes as u32));
    }

    pub fn lost(&mut self, now: f64, packets: u32) {
        if packets > 0 {
            self.lost_packets += packets as u64;
            self.push(now, Event::Lost(packets));
        }
    }

    /// A packet received with a sequence number (MAVLink's wraps at 256) from
    /// `source`: the gap since the last one from it counts as lost.
    pub fn received_seq(&mut self, now: f64, source: u16, seq: u8, bytes: usize) {
        let mut gap = 0;
        match self.sequences.iter_mut().find(|(s, _)| *s == source) {
            Some((_, last)) => {
                gap = seq.wrapping_sub(*last).wrapping_sub(1);
                *last = seq;
            }
            None => self.sequences.push((source, seq)),
        }
        // A big jump is a restart or a reordering, not 200 losses.
        if gap < 128 {
            self.lost(now, gap as u32);
        }
        self.received(now, bytes);
    }

    /// Seconds since the last packet in.
    pub fn silent_s(&self, now: f64) -> Option<f64> {
        self.last_rx.map(|t| (now - t).max(0.0))
    }

    /// Packet loss over the window, percent.
    pub fn loss_pct(&self, now: f64) -> f64 {
        self.window(now).4
    }

    pub fn is_up(&self, now: f64) -> bool {
        self.enabled && self.silent_s(now).is_some_and(|s| s < DOWN_AFTER_S)
    }

    fn push(&mut self, now: f64, e: Event) {
        self.events.push_back((now, e));
        self.trim(now);
    }

    fn trim(&mut self, now: f64) {
        while self.events.front().is_some_and(|(t, _)| now - t > WINDOW_S) {
            self.events.pop_front();
        }
        // A sample a second for the graphs.
        if now - self.last_sample >= 1.0 {
            self.last_sample = now;
            let (rx_bps, tx_bps, _, _, loss) = self.window(now);
            self.history.push_back(FlowSample { t: now, rx_bps, tx_bps, loss_pct: loss });
            while self.history.front().is_some_and(|s| now - s.t > HISTORY_S) {
                self.history.pop_front();
            }
        }
    }

    /// (rx bits/s, tx bits/s, rx packets/s, tx packets/s, loss %) over the window.
    fn window(&self, now: f64) -> (f64, f64, f64, f64, f64) {
        let (mut rxb, mut txb, mut rxp, mut txp, mut lost) = (0u64, 0u64, 0u64, 0u64, 0u64);
        let mut first = now;
        for (t, e) in &self.events {
            first = first.min(*t);
            match e {
                Event::Rx(b) => {
                    rxb += *b as u64;
                    rxp += 1;
                }
                Event::Tx(b) => {
                    txb += *b as u64;
                    txp += 1;
                }
                Event::Lost(n) => lost += *n as u64,
            }
        }
        // Early on, rate over the time there has been, not the whole window.
        let span = (now - first).clamp(1.0, WINDOW_S);
        let loss = if rxp + lost > 0 { 100.0 * lost as f64 / (rxp + lost) as f64 } else { 0.0 };
        (rxb as f64 * 8.0 / span, txb as f64 * 8.0 / span, rxp as f64 / span, txp as f64 / span, loss)
    }

    pub fn stats(&mut self, now: f64) -> FlowStats {
        self.trim(now);
        let (rx_bps, tx_bps, rx_pps, tx_pps, loss_pct) = self.window(now);
        let state = if !self.enabled {
            FlowState::Off
        } else if !self.is_up(now) {
            FlowState::Down
        } else if self.active {
            FlowState::Active
        } else {
            FlowState::Standby
        };
        // A flow that has gone quiet has lost everything sent its way.
        let loss_pct = if state == FlowState::Down && self.last_rx.is_some() { 100.0 } else { loss_pct };
        FlowStats {
            bearer: self.bearer,
            name: self.name.clone(),
            state,
            rx_bps,
            tx_bps,
            rx_pps,
            tx_pps,
            loss_pct,
            rx_packets: self.rx_packets,
            tx_packets: self.tx_packets,
            lost_packets: self.lost_packets,
            rx_bytes: self.rx_bytes,
            tx_bytes: self.tx_bytes,
            rssi_dbm: self.rssi_dbm.filter(|_| state != FlowState::Down && state != FlowState::Off),
            snr_db: self.snr_db.filter(|_| state != FlowState::Down && state != FlowState::Off),
            latency_ms: self.latency_ms.filter(|_| state != FlowState::Down && state != FlowState::Off),
            capacity_bps: self.capacity_bps,
            silent_s: self.silent_s(now),
            history: self.history.iter().copied().collect(),
        }
    }
}

/// Bits a second, readable: 850 bit/s, 5.4 kbit/s, 12.1 Mbit/s.
pub fn format_rate(bps: f64) -> String {
    if bps >= 1e6 {
        format!("{:.1} Mbit/s", bps / 1e6)
    } else if bps >= 1e3 {
        format!("{:.1} kbit/s", bps / 1e3)
    } else {
        format!("{bps:.0} bit/s")
    }
}

/// Bytes, readable.
pub fn format_bytes(bytes: u64) -> String {
    let b = bytes as f64;
    if b >= 1e9 {
        format!("{:.2} GB", b / 1e9)
    } else if b >= 1e6 {
        format!("{:.1} MB", b / 1e6)
    } else if b >= 1e3 {
        format!("{:.1} kB", b / 1e3)
    } else {
        format!("{bytes} B")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rates_and_loss_over_the_window() {
        let mut m = FlowMeter::new(Bearer::LoRa, "LoRa");
        // 10 packets a second of 100 bytes for 5 s, a fifth of them lost.
        for i in 0..50 {
            let t = i as f64 * 0.1;
            if i % 5 == 4 {
                m.lost(t, 1);
            } else {
                m.received(t, 100);
            }
            m.sent(t, 20);
        }
        m.active = true;
        let s = m.stats(5.0);
        assert!((s.loss_pct - 20.0).abs() < 1.0, "{}", s.loss_pct);
        assert!((s.rx_bps - 6400.0).abs() < 400.0, "{}", s.rx_bps);
        assert!((s.tx_bps - 1600.0).abs() < 100.0, "{}", s.tx_bps);
        assert_eq!(s.state, FlowState::Active);
        assert_eq!((s.rx_packets, s.lost_packets), (40, 10));
        assert!(s.history.len() >= 5);
    }

    #[test]
    fn sequence_gaps_count_as_losses() {
        let mut m = FlowMeter::new(Bearer::WiFi, "Wi-Fi");
        for seq in [250u8, 251, 254, 255, 0, 3] {
            m.received_seq(0.0, 1, seq, 30);
        }
        // Missing: 252, 253, 1, 2.
        assert_eq!(m.stats(0.0).lost_packets, 4);
        // Another sender keeps its own count.
        m.received_seq(0.0, 2, 100, 30);
        assert_eq!(m.stats(0.0).lost_packets, 4);
    }

    #[test]
    fn a_quiet_flow_goes_down() {
        let mut m = FlowMeter::new(Bearer::Cellular, "5G");
        m.received(0.0, 50);
        assert_eq!(m.stats(1.0).state, FlowState::Standby);
        assert_eq!(m.stats(DOWN_AFTER_S + 1.0).state, FlowState::Down);
        m.enabled = false;
        assert_eq!(m.stats(10.0).state, FlowState::Off);
    }

    #[test]
    fn rates_read_well() {
        assert_eq!(format_rate(850.0), "850 bit/s");
        assert_eq!(format_rate(5400.0), "5.4 kbit/s");
        assert_eq!(format_rate(12.1e6), "12.1 Mbit/s");
        assert_eq!(format_bytes(1536), "1.5 kB");
    }
}
