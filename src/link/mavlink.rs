//! A real boat: an ArduPilot Rover (boat frame, skid steering) over MAVLink.
//!
//! Only the dozen messages a boat GCS needs are spoken, written out here by
//! hand (MAVLink v2 out; v1 and v2 in):
//!
//! * **in**: HEARTBEAT (armed), GLOBAL_POSITION_INT (position, heading, ground
//!   velocity), ATTITUDE (roll, pitch, rate of turn), GPS_RAW_INT (fix,
//!   satellites, course and speed over ground), SYS_STATUS (battery),
//!   SERVO_OUTPUT_RAW (what the two motors are doing), DISTANCE_SENSOR
//!   pointing down (the echo sounder), RADIO_STATUS (signal) and STATUSTEXT
//!   (the boat's own messages, into the log);
//! * **out**: HEARTBEAT, REQUEST_DATA_STREAM (so the boat streams at 10 Hz),
//!   COMMAND_LONG (arm, disarm, MANUAL mode) and RC_CHANNELS_OVERRIDE, which
//!   carries the station's throttle and steering.
//!
//! The station's autopilot runs here, on the ground, as with the simulator:
//! the boat sits in MANUAL and is driven by the override.  If the link
//! drops, ArduPilot's own override timeout and failsafe stop it — the same
//! arrangement as the simulated boat's on-board failsafe.
//!
//! Connections are written as `udp-listen:14550` (the boat or a telemetry
//! bridge sends to us), `udp:HOST:PORT`, `tcp:HOST:PORT` (ArduPilot SITL is
//! `tcp:127.0.0.1:5760`) or `serial:/dev/ttyUSB0:57600` (a SiK radio).
//!
//! A boat can be reached by several paths at once — a LoRa or SiK radio, the
//! boat's Wi-Fi, a 5G modem — written comma-separated with what carries each:
//! `lora=serial:/dev/ttyUSB0:57600, wifi=udp-listen:14550, cellular=udp:vpn:14550`.
//! Every path is read, and metered; the boat's messages are taken from the
//! path in use, and from the others only while it is quiet, so nothing is
//! handled twice.  Heartbeats and arming go on every path; the RC override
//! goes on the most preferred path that is working (Wi-Fi, network, 5G,
//! radio, LoRa), one in four on LoRa.  Each path is metered: data rates,
//! packet loss from MAVLink sequence gaps, the radio's RSSI and SNR
//! (RADIO_STATUS) and latency (TIMESYNC round trips).

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use crate::config::LinkSettings;
use crate::core::clock;
use crate::core::telemetry::Telemetry;

use super::flows::{Bearer, FlowMeter, FlowStats};
use super::{BoatLink, TelemetryHandler};

// --------------------------------------------------------------------------
// Framing
// --------------------------------------------------------------------------

/// X.25 / MCRF4XX checksum, as MAVLink uses.
pub fn crc_accumulate(crc: u16, byte: u8) -> u16 {
    let mut tmp = byte ^ (crc & 0xFF) as u8;
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4)
}

pub fn crc(bytes: &[u8], extra: u8) -> u16 {
    let mut c = 0xFFFF;
    for &b in bytes {
        c = crc_accumulate(c, b);
    }
    crc_accumulate(c, extra)
}

pub const HEARTBEAT: u32 = 0;
pub const SYS_STATUS: u32 = 1;
pub const GPS_RAW_INT: u32 = 24;
pub const ATTITUDE: u32 = 30;
pub const GLOBAL_POSITION_INT: u32 = 33;
pub const SERVO_OUTPUT_RAW: u32 = 36;
pub const REQUEST_DATA_STREAM: u32 = 66;
pub const RC_CHANNELS_OVERRIDE: u32 = 70;
pub const COMMAND_LONG: u32 = 76;
pub const RADIO_STATUS: u32 = 109;
pub const DISTANCE_SENSOR: u32 = 132;
pub const STATUSTEXT: u32 = 253;

/// (CRC extra, length of the message without extensions).
pub fn message_info(id: u32) -> Option<(u8, usize)> {
    Some(match id {
        HEARTBEAT => (50, 9),
        SYS_STATUS => (124, 31),
        GPS_RAW_INT => (24, 30),
        ATTITUDE => (39, 28),
        GLOBAL_POSITION_INT => (104, 28),
        SERVO_OUTPUT_RAW => (222, 21),
        REQUEST_DATA_STREAM => (148, 6),
        RC_CHANNELS_OVERRIDE => (124, 18),
        COMMAND_LONG => (152, 33),
        RADIO_STATUS => (185, 9),
        DISTANCE_SENSOR => (85, 14),
        STATUSTEXT => (83, 51),
        TIMESYNC => (34, 16),
        _ => return None,
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    pub id: u32,
    pub sysid: u8,
    pub compid: u8,
    pub seq: u8,
    /// The whole frame's length on the wire.
    pub frame_len: usize,
    /// Zero-padded to at least the message's base length.
    pub payload: Vec<u8>,
}

impl Message {
    fn u8(&self, at: usize) -> u8 {
        self.payload.get(at).copied().unwrap_or(0)
    }
    fn u16(&self, at: usize) -> u16 {
        u16::from_le_bytes([self.u8(at), self.u8(at + 1)])
    }
    fn i16(&self, at: usize) -> i16 {
        self.u16(at) as i16
    }
    fn u32(&self, at: usize) -> u32 {
        u32::from_le_bytes([self.u8(at), self.u8(at + 1), self.u8(at + 2), self.u8(at + 3)])
    }
    fn i32(&self, at: usize) -> i32 {
        self.u32(at) as i32
    }
    fn i64(&self, at: usize) -> i64 {
        i64::from_le_bytes(std::array::from_fn(|k| self.u8(at + k)))
    }
    fn f32(&self, at: usize) -> f32 {
        f32::from_bits(self.u32(at))
    }
}

/// One MAVLink v2 frame.
pub fn encode(seq: u8, sysid: u8, compid: u8, id: u32, payload: &[u8]) -> Vec<u8> {
    let (extra, _) = message_info(id).expect("a message this link knows");
    // v2 trims trailing zero bytes (but keeps at least one).
    let mut len = payload.len();
    while len > 1 && payload[len - 1] == 0 {
        len -= 1;
    }
    let mut frame = vec![0xFD, len as u8, 0, 0, seq, sysid, compid, id as u8, (id >> 8) as u8, (id >> 16) as u8];
    frame.extend_from_slice(&payload[..len]);
    let c = crc(&frame[1..], extra);
    frame.extend_from_slice(&c.to_le_bytes());
    frame
}

/// Pulls frames out of a byte stream, v1 or v2, dropping corrupt ones and
/// messages it does not know.
#[derive(Default)]
pub struct Parser {
    buffer: Vec<u8>,
    pub bad_crc: usize,
}

impl Parser {
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Message> {
        self.buffer.extend_from_slice(bytes);
        let mut out = Vec::new();
        loop {
            let Some(start) = self.buffer.iter().position(|&b| b == 0xFD || b == 0xFE) else {
                self.buffer.clear();
                break;
            };
            self.buffer.drain(..start);
            let v2 = self.buffer[0] == 0xFD;
            let header = if v2 { 10 } else { 6 };
            if self.buffer.len() < header {
                break;
            }
            // v2 knows one incompatibility flag (signing): anything else
            // means this was not a frame start, so look further on.
            if v2 && self.buffer[2] & !0x01 != 0 {
                self.buffer.drain(..1);
                continue;
            }
            let len = self.buffer[1] as usize;
            let signed = v2 && self.buffer[2] & 0x01 != 0;
            let total = header + len + 2 + if signed { 13 } else { 0 };
            if self.buffer.len() < total {
                break;
            }
            let seq = if v2 { self.buffer[4] } else { self.buffer[2] };
            let (sysid, compid, id) = if v2 {
                (self.buffer[5], self.buffer[6], self.buffer[7] as u32 | (self.buffer[8] as u32) << 8 | (self.buffer[9] as u32) << 16)
            } else {
                (self.buffer[3], self.buffer[4], self.buffer[5] as u32)
            };
            match message_info(id) {
                Some((extra, base)) => {
                    let got = u16::from_le_bytes([self.buffer[header + len], self.buffer[header + len + 1]]);
                    if crc(&self.buffer[1..header + len], extra) == got {
                        let mut payload = self.buffer[header..header + len].to_vec();
                        if payload.len() < base {
                            payload.resize(base, 0);
                        }
                        out.push(Message { id, sysid, compid, seq, frame_len: total, payload });
                        self.buffer.drain(..total);
                    } else {
                        // Not a frame after all: look for the next start.
                        self.bad_crc += 1;
                        self.buffer.drain(..1);
                    }
                }
                None => {
                    self.buffer.drain(..total);
                }
            }
        }
        out
    }
}

// --------------------------------------------------------------------------
// Payloads the station sends
// --------------------------------------------------------------------------

pub fn heartbeat_gcs() -> Vec<u8> {
    // custom_mode, type GCS (6), autopilot INVALID (8), base_mode, status ACTIVE (4), version 3.
    let mut p = vec![0u8; 4];
    p.extend_from_slice(&[6, 8, 0, 4, 3]);
    p
}

pub fn command_long(target: u8, command: u16, params: [f32; 7]) -> Vec<u8> {
    let mut p = Vec::with_capacity(33);
    for v in params {
        p.extend_from_slice(&v.to_le_bytes());
    }
    p.extend_from_slice(&command.to_le_bytes());
    p.extend_from_slice(&[target, 1, 0]);
    p
}

pub fn rc_override(target: u8, channels: [u16; 8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(18);
    for c in channels {
        p.extend_from_slice(&c.to_le_bytes());
    }
    p.extend_from_slice(&[target, 1]);
    p
}

pub fn request_streams(target: u8, rate_hz: u16) -> Vec<u8> {
    let mut p = rate_hz.to_le_bytes().to_vec();
    // stream ALL (0), start.
    p.extend_from_slice(&[target, 1, 0, 1]);
    p
}

/// PWM for a level in [-1, 1] about a trim.
pub fn pwm(level: f64, s: &LinkSettings) -> u16 {
    let level = level.clamp(-1.0, 1.0);
    let span = if level >= 0.0 { s.pwm_max as f64 - s.pwm_trim as f64 } else { s.pwm_trim as f64 - s.pwm_min as f64 };
    (s.pwm_trim as f64 + level * span).round() as u16
}

/// The level in [-1, 1] a PWM stands for.
pub fn level(pwm: u16, s: &LinkSettings) -> f64 {
    let p = pwm as f64;
    let trim = s.pwm_trim as f64;
    if p >= trim {
        ((p - trim) / (s.pwm_max as f64 - trim).max(1.0)).clamp(0.0, 1.0)
    } else {
        -((trim - p) / (trim - s.pwm_min as f64).max(1.0)).clamp(0.0, 1.0)
    }
}

// --------------------------------------------------------------------------
// Transports
// --------------------------------------------------------------------------

/// Where the boat is: parsed from a connection string.
#[derive(Clone, Debug, PartialEq)]
pub enum Endpoint {
    UdpListen(u16),
    Udp(String),
    Tcp(String),
    Serial(String, u32),
}

impl Endpoint {
    pub fn parse(text: &str) -> Result<Endpoint, String> {
        let text = text.trim();
        let (kind, rest) = text.split_once(':').ok_or_else(|| format!("{text:?}: write it as udp-listen:14550, udp:host:port, tcp:host:port or serial:/dev/ttyUSB0:57600"))?;
        match kind {
            "udp-listen" | "udpin" => rest.parse().map(Endpoint::UdpListen).map_err(|_| format!("bad port {rest:?}")),
            "udp" | "udpout" => Ok(Endpoint::Udp(rest.to_string())),
            "tcp" => Ok(Endpoint::Tcp(rest.to_string())),
            "serial" => {
                let (device, baud) = rest.rsplit_once(':').unwrap_or((rest, "57600"));
                Ok(Endpoint::Serial(device.to_string(), baud.parse().map_err(|_| format!("bad baud rate {baud:?}"))?))
            }
            _ => Err(format!("unknown connection type {kind:?}")),
        }
    }

    pub fn describe(&self) -> String {
        match self {
            Endpoint::UdpListen(p) => format!("UDP :{p}"),
            Endpoint::Udp(a) => format!("UDP {a}"),
            Endpoint::Tcp(a) => format!("TCP {a}"),
            Endpoint::Serial(d, b) => format!("{d} @ {b}"),
        }
    }
}

/// The sending half of a connection.
trait Sender: Send {
    fn send(&mut self, bytes: &[u8]) -> std::io::Result<()>;
}

/// The receiving half: separate, so waiting for the boat never holds up a
/// command to it.
trait Receiver: Send {
    /// Bytes received within about 100 ms; none if nothing came.
    fn recv(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;
}

fn quiet(e: &std::io::Error) -> bool {
    matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut)
}

/// Where a UDP reply goes: fixed, or whoever spoke last.
type Peer = Arc<Mutex<Option<SocketAddr>>>;

struct UdpOut(UdpSocket, Peer);
struct UdpIn(UdpSocket, Peer, bool);

impl Sender for UdpOut {
    fn send(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        match *self.1.lock().unwrap() {
            Some(peer) => self.0.send_to(bytes, peer).map(|_| ()),
            None => Ok(()), // nobody to talk to yet
        }
    }
}

impl Receiver for UdpIn {
    fn recv(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match self.0.recv_from(buf) {
            Ok((n, from)) => {
                if !self.2 {
                    *self.1.lock().unwrap() = Some(from);
                }
                Ok(n)
            }
            Err(e) if quiet(&e) => Ok(0),
            // A UDP "connection refused" only means nobody listens yet.
            Err(e) if e.kind() == std::io::ErrorKind::ConnectionRefused => Ok(0),
            Err(e) => Err(e),
        }
    }
}

impl Sender for TcpStream {
    fn send(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.write_all(bytes)
    }
}

impl Receiver for TcpStream {
    fn recv(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match self.read(buf) {
            Ok(0) => Err(std::io::Error::new(std::io::ErrorKind::ConnectionAborted, "closed by the boat")),
            Ok(n) => Ok(n),
            Err(e) if quiet(&e) => Ok(0),
            Err(e) => Err(e),
        }
    }
}

struct Serial(Box<dyn serialport::SerialPort>);

impl Sender for Serial {
    fn send(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.0.write_all(bytes)
    }
}

impl Receiver for Serial {
    fn recv(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match self.0.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if quiet(&e) => Ok(0),
            Err(e) => Err(e),
        }
    }
}

fn resolve(address: &str) -> Result<SocketAddr, String> {
    std::net::ToSocketAddrs::to_socket_addrs(address)
        .map_err(|e| format!("{address}: {e}"))?
        .next()
        .ok_or_else(|| format!("{address}: no address"))
}

type Halves = (Box<dyn Sender>, Box<dyn Receiver>);

fn open(endpoint: &Endpoint) -> Result<Halves, String> {
    let timeout = Some(Duration::from_millis(100));
    let e = |e: std::io::Error| e.to_string();
    Ok(match endpoint {
        Endpoint::UdpListen(port) => {
            let socket = UdpSocket::bind(("0.0.0.0", *port)).map_err(|e| format!("UDP port {port}: {e}"))?;
            socket.set_read_timeout(timeout).map_err(e)?;
            let peer: Peer = Arc::new(Mutex::new(None));
            (Box::new(UdpOut(socket.try_clone().map_err(e)?, peer.clone())), Box::new(UdpIn(socket, peer, false)))
        }
        Endpoint::Udp(address) => {
            let to = resolve(address)?;
            let socket = UdpSocket::bind(("0.0.0.0", 0)).map_err(e)?;
            socket.set_read_timeout(timeout).map_err(e)?;
            let peer: Peer = Arc::new(Mutex::new(Some(to)));
            (Box::new(UdpOut(socket.try_clone().map_err(e)?, peer.clone())), Box::new(UdpIn(socket, peer, true)))
        }
        Endpoint::Tcp(address) => {
            let stream = TcpStream::connect_timeout(&resolve(address)?, Duration::from_secs(5)).map_err(|e| format!("{address}: {e}"))?;
            stream.set_read_timeout(timeout).map_err(e)?;
            let _ = stream.set_nodelay(true);
            (Box::new(stream.try_clone().map_err(e)?), Box::new(stream))
        }
        Endpoint::Serial(device, baud) => {
            let port = serialport::new(device, *baud).timeout(Duration::from_millis(100)).open().map_err(|e| format!("{device}: {e}"))?;
            let writer = port.try_clone().map_err(|e| format!("{device}: {e}"))?;
            (Box::new(Serial(writer)), Box::new(Serial(port)))
        }
    })
}

/// Serial devices a radio or autopilot might be on.
pub fn serial_ports() -> Vec<String> {
    let mut found: Vec<String> = std::fs::read_dir("/dev")
        .map(|rd| {
            rd.flatten()
                .map(|e| e.file_name().to_string_lossy().to_string())
                .filter(|n| n.starts_with("ttyUSB") || n.starts_with("ttyACM") || n.starts_with("ttyAMA"))
                .map(|n| format!("/dev/{n}"))
                .collect()
        })
        .unwrap_or_default();
    if let Ok(rd) = std::fs::read_dir("/dev/serial/by-id") {
        found.extend(rd.flatten().map(|e| e.path().display().to_string()));
    }
    found.sort();
    found
}

// --------------------------------------------------------------------------
// The link
// --------------------------------------------------------------------------

/// One path to the boat: what carries it and where it connects.
#[derive(Clone, Debug, PartialEq)]
pub struct PathSpec {
    pub bearer: Bearer,
    pub endpoint: Endpoint,
}

impl PathSpec {
    /// `lora=serial:/dev/ttyUSB0:57600`, or a bare connection whose bearer
    /// is guessed (a serial port is a telemetry radio, anything else a network).
    pub fn parse(text: &str) -> Result<PathSpec, String> {
        let text = text.trim();
        if let Some((name, rest)) = text.split_once('=') {
            let bearer = Bearer::parse(name).ok_or_else(|| format!("{name:?} is not a kind of link: wifi, cellular, lora, radio, satellite or ip"))?;
            return Ok(PathSpec { bearer, endpoint: Endpoint::parse(rest)? });
        }
        let endpoint = Endpoint::parse(text)?;
        Ok(PathSpec { bearer: Self::guess(&endpoint), endpoint })
    }

    /// What probably carries a connection given bare.
    pub fn guess(endpoint: &Endpoint) -> Bearer {
        if matches!(endpoint, Endpoint::Serial(..)) {
            Bearer::Radio
        } else {
            Bearer::Ip
        }
    }

    pub fn to_url(&self) -> String {
        let url = match &self.endpoint {
            Endpoint::UdpListen(p) => format!("udp-listen:{p}"),
            Endpoint::Udp(a) => format!("udp:{a}"),
            Endpoint::Tcp(a) => format!("tcp:{a}"),
            Endpoint::Serial(d, b) => format!("serial:{d}:{b}"),
        };
        if self.bearer == Self::guess(&self.endpoint) {
            url
        } else {
            format!("{}={url}", self.bearer.key())
        }
    }
}

/// Several paths, comma-separated: `lora=serial:/dev/ttyUSB0:57600, wifi=udp-listen:14550`.
pub fn parse_paths(text: &str) -> Result<Vec<PathSpec>, String> {
    let paths: Vec<PathSpec> = text.split([',', ';']).filter(|p| !p.trim().is_empty()).map(PathSpec::parse).collect::<Result<_, _>>()?;
    if paths.is_empty() {
        return Err("no connection given".into());
    }
    Ok(paths)
}

/// The paths in a few words, for the status bar and the recent list.
pub fn describe_paths(text: &str) -> String {
    match parse_paths(text) {
        Ok(paths) if paths.len() == 1 => paths[0].endpoint.describe(),
        Ok(paths) => paths.iter().map(|p| p.bearer.label()).collect::<Vec<_>>().join(" + "),
        Err(_) => text.to_string(),
    }
}

/// Paths in this order of preference carry the commands.
fn preference(b: Bearer) -> u8 {
    match b {
        Bearer::WiFi => 0,
        Bearer::Ip => 1,
        Bearer::Cellular => 2,
        Bearer::Radio => 3,
        Bearer::LoRa => 4,
        Bearer::Satellite => 5,
    }
}

/// Too slow for every RC override: these take one in four.
fn slow(b: Bearer) -> bool {
    matches!(b, Bearer::LoRa | Bearer::Satellite)
}

pub const TIMESYNC: u32 = 111;

pub fn timesync(tc1: i64, ts1: i64) -> Vec<u8> {
    let mut p = tc1.to_le_bytes().to_vec();
    p.extend_from_slice(&ts1.to_le_bytes());
    p
}

fn now_ns() -> i64 {
    (clock::monotonic() * 1e9) as i64
}

struct Path {
    spec: PathSpec,
    sender: Mutex<Box<dyn Sender>>,
    receiver: Mutex<Option<Box<dyn Receiver>>>,
    meter: Mutex<FlowMeter>,
    enabled: AtomicBool,
}

/// What the boat has told us, gathered message by message.
#[derive(Default)]
struct BoatView {
    t: Telemetry,
    sysid: Option<u8>,
    time_boot_ms: u32,
    vehicle_type: Option<u8>,
    autopilot: Option<u8>,
    custom_mode: u32,
    firmware: Option<String>,
    /// When the last position came.
    position_at: f64,
}

struct Shared {
    settings: LinkSettings,
    paths: Vec<Path>,
    view: Mutex<BoatView>,
    handler: Mutex<Option<TelemetryHandler>>,
    seq: Mutex<u8>,
    stop: AtomicBool,
    /// What the station last asked for: (armed, when the command was last sent).
    arm_wanted: Mutex<(bool, f64)>,
    /// Messages from the boat for the log.
    texts: Mutex<Vec<String>>,
    received: AtomicUsize,
    duplicates: AtomicUsize,
    bad_crc: AtomicUsize,
    overrides: AtomicUsize,
}

impl Shared {
    fn target(&self) -> u8 {
        self.view.lock().unwrap().sysid.unwrap_or(self.settings.target_sysid)
    }

    fn frame(&self, id: u32, payload: &[u8]) -> Vec<u8> {
        let seq = {
            let mut s = self.seq.lock().unwrap();
            *s = s.wrapping_add(1);
            *s
        };
        encode(seq, self.settings.gcs_sysid, 190, id, payload)
    }

    fn send_on(&self, i: usize, frame: &[u8]) {
        let path = &self.paths[i];
        if !path.enabled.load(Ordering::Relaxed) {
            return;
        }
        match path.sender.lock().unwrap().send(frame) {
            Ok(()) => path.meter.lock().unwrap().sent(clock::monotonic(), frame.len()),
            Err(e) => log::debug!("mavlink send on {}: {e}", path.spec.endpoint.describe()),
        }
    }

    /// On every path: heartbeats, and orders that must get through.
    fn send_all(&self, id: u32, payload: &[u8]) {
        let frame = self.frame(id, payload);
        for i in 0..self.paths.len() {
            self.send_on(i, &frame);
        }
    }

    /// The path in use: the most preferred one that is working — heard
    /// lately, and losing less than a third — or failing that any one heard.
    fn primary(&self) -> Option<usize> {
        let now = clock::monotonic();
        let up: Vec<(usize, f64)> = (0..self.paths.len())
            .filter(|&i| self.paths[i].enabled.load(Ordering::Relaxed))
            .filter_map(|i| {
                let m = self.paths[i].meter.lock().unwrap();
                m.is_up(now).then(|| (i, m.loss_pct(now)))
            })
            .collect();
        let best = |good: bool| {
            up.iter().filter(|(_, loss)| !good || *loss < 33.0).map(|(i, _)| *i).min_by_key(|&i| (preference(self.paths[i].spec.bearer), i))
        };
        best(true).or_else(|| best(false))
    }

    /// Whether to take a message that came by path `i`.  The boat sends
    /// everything by every path (each with its own sequence numbers), so the
    /// path in use is believed, and the others fill in only while it is quiet.
    fn take(&self, i: usize, now: f64) -> bool {
        match self.primary() {
            Some(p) if p != i => self.paths[p].meter.lock().unwrap().silent_s(now).is_none_or(|s| s > 0.5),
            _ => true,
        }
    }

    fn handle(&self, m: &Message) {
        let s = &self.settings;
        let mut v = self.view.lock().unwrap();
        // Only the vehicle's own autopilot (component 1) counts.
        if m.compid != 1 {
            return;
        }
        if m.id == HEARTBEAT {
            // A GCS or another non-vehicle is not the boat.
            let kind = m.u8(4);
            if kind == 6 {
                return;
            }
            v.sysid.get_or_insert(m.sysid);
            v.vehicle_type = Some(kind);
            v.autopilot = Some(m.u8(5));
            v.custom_mode = m.u32(0);
        }
        if v.sysid.is_some_and(|id| id != m.sysid) {
            return;
        }
        let t = &mut v.t;
        match m.id {
            HEARTBEAT => t.armed = m.u8(6) & 0x80 != 0,
            SYS_STATUS => {
                let mv = m.u16(14);
                let ca = m.i16(16);
                let pct = m.payload.get(30).map(|b| *b as i8).unwrap_or(-1);
                t.battery_v = (mv != u16::MAX && mv > 0).then(|| mv as f64 / 1000.0);
                t.battery_a = (ca != -1).then(|| ca as f64 / 100.0);
                t.battery_pct = (pct >= 0).then_some(pct as f64);
            }
            GPS_RAW_INT => {
                let fix = m.u8(28);
                t.gps_fix = match fix {
                    0 | 1 => 0,
                    2 => 2,
                    3 | 4 => 3,
                    5 => 4,
                    _ => 5,
                };
                t.satellites = m.u8(29) as u32;
                // time_usec u64, lat, lon, alt, then eph, epv, vel, cog.
                let eph = m.u16(20);
                t.hdop = (eph != u16::MAX).then(|| eph as f64 / 100.0);
                let vel = m.u16(24);
                let cog = m.u16(26);
                if vel != u16::MAX {
                    t.sog_ms = Some(vel as f64 / 100.0);
                    t.cog_deg = (cog != u16::MAX && vel > 15).then(|| cog as f64 / 100.0);
                }
            }
            ATTITUDE => {
                t.roll_deg = Some((m.f32(4) as f64).to_degrees());
                t.pitch_deg = Some((m.f32(8) as f64).to_degrees());
                t.rot_dps = Some((m.f32(24) as f64).to_degrees());
                let yaw = (m.f32(12) as f64).to_degrees().rem_euclid(360.0);
                if t.heading_deg.is_none() {
                    t.heading_deg = Some(yaw);
                }
            }
            GLOBAL_POSITION_INT => {
                v.time_boot_ms = m.u32(0);
                let t = &mut v.t;
                let (lat, lon) = (m.i32(4), m.i32(8));
                // 0, 0 is how an autopilot without a position says so.
                if lat != 0 || lon != 0 {
                    t.lat = Some(lat as f64 * 1e-7);
                    t.lon = Some(lon as f64 * 1e-7);
                }
                let hdg = m.u16(26);
                if hdg != u16::MAX {
                    t.heading_deg = Some(hdg as f64 / 100.0);
                }
                // Ground velocity, when GPS_RAW_INT has not said.
                let (vx, vy) = (m.i16(20) as f64 / 100.0, m.i16(22) as f64 / 100.0);
                if t.sog_ms.is_none() {
                    t.sog_ms = Some(vx.hypot(vy));
                }
            }
            SERVO_OUTPUT_RAW => {
                let channel = |c: u8| m.u16(4 + 2 * (c.clamp(1, 8) as usize - 1));
                t.throttle_port = level(channel(s.servo_port), s);
                t.throttle_stbd = level(channel(s.servo_stbd), s);
            }
            DISTANCE_SENSOR => {
                // Pointing down (MAV_SENSOR_ROTATION_PITCH_270 = 25): the sounder.
                if m.u8(12) == 25 {
                    let cm = m.u16(8);
                    t.depth_m = Some(cm as f64 / 100.0);
                }
            }
            STATUSTEXT => {
                let text: String = m.payload[1..51.min(m.payload.len())].iter().take_while(|b| **b != 0).map(|&b| b as char).collect();
                if text.starts_with("Ardu") || text.starts_with("PX4") {
                    v.firmware = Some(text.clone());
                }
                drop(v);
                log::info!("boat: {text}");
                self.say(format!("Boat: {text}"));
                return;
            }
            _ => {}
        }
        // Reported on each position, and on a heartbeat when positions are
        // not coming (so a boat without a fix still shows as linked).
        let now = clock::monotonic();
        if m.id == GLOBAL_POSITION_INT {
            v.position_at = now;
        }
        if m.id == GLOBAL_POSITION_INT || (m.id == HEARTBEAT && now - v.position_at > 1.5) {
            let mut out = v.t.clone();
            out.received = clock::monotonic();
            out.boat_time_s = Some(v.time_boot_ms as f64 / 1000.0);
            out.stw_ms = None;
            drop(v);
            out.rssi_dbm = self.primary().and_then(|i| self.paths[i].meter.lock().unwrap().rssi_dbm);
            if let Some(h) = self.handler.lock().unwrap().clone() {
                h(out);
            }
        }
    }

    /// Reading one path.
    fn listen(self: Arc<Self>, i: usize) {
        let Some(mut receiver) = self.paths[i].receiver.lock().unwrap().take() else { return };
        let mut parser = Parser::default();
        let mut buf = vec![0u8; 2048];
        while !self.stop.load(Ordering::Relaxed) {
            let n = match receiver.recv(&mut buf) {
                Ok(n) => n,
                Err(e) => {
                    log::warn!("mavlink {}: {e}", self.paths[i].spec.endpoint.describe());
                    std::thread::sleep(Duration::from_millis(500));
                    continue;
                }
            };
            if n == 0 || !self.paths[i].enabled.load(Ordering::Relaxed) {
                continue;
            }
            let before = parser.bad_crc;
            let messages = parser.push(&buf[..n]);
            self.bad_crc.fetch_add(parser.bad_crc - before, Ordering::Relaxed);
            let now = clock::monotonic();
            for m in messages {
                {
                    let mut meter = self.paths[i].meter.lock().unwrap();
                    meter.received_seq(now, (m.sysid as u16) << 8 | m.compid as u16, m.seq, m.frame_len);
                    match m.id {
                        // The radio's own report: this path's signal.
                        RADIO_STATUS => {
                            let (rssi, noise) = (m.u8(4) as f64, m.u8(7) as f64);
                            meter.rssi_dbm = Some(rssi / 1.9 - 127.0);
                            meter.snr_db = Some((rssi - noise) / 1.9);
                        }
                        // The answer to our time request: this path's latency.
                        TIMESYNC => {
                            let (tc1, ts1) = (m.i64(0), m.i64(8));
                            if tc1 != 0 && ts1 > 0 {
                                let rtt_ms = (now_ns() - ts1) as f64 / 1e6;
                                if (0.0..10_000.0).contains(&rtt_ms) {
                                    meter.latency_ms = Some(rtt_ms / 2.0);
                                }
                            }
                        }
                        _ => {}
                    }
                }
                if m.id == RADIO_STATUS || m.id == TIMESYNC {
                    continue;
                }
                if self.take(i, now) {
                    self.received.fetch_add(1, Ordering::Relaxed);
                    self.handle(&m);
                } else {
                    self.duplicates.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    /// Heartbeats, stream requests, time requests and arming, once a second.
    fn housekeeping(self: Arc<Self>) {
        let mut streams_asked = f64::NEG_INFINITY;
        let mut refused = 0;
        while !self.stop.load(Ordering::Relaxed) {
            let now = clock::monotonic();
            self.send_all(HEARTBEAT, &heartbeat_gcs());
            self.send_all(TIMESYNC, &timesync(0, now_ns()));
            // Ask for the data streams until the position arrives.
            let has_position = self.view.lock().unwrap().t.lat.is_some();
            if !has_position && now - streams_asked >= 2.0 {
                streams_asked = now;
                self.send_all(REQUEST_DATA_STREAM, &request_streams(self.target(), 10));
            }
            // Keep asking for the arming state wanted until the boat agrees.
            let (wanted, sent) = *self.arm_wanted.lock().unwrap();
            let armed = self.view.lock().unwrap().t.armed;
            if wanted != armed && now - sent >= 1.0 {
                refused += 1;
                if refused == 3 && self.view.lock().unwrap().sysid.is_some() {
                    self.say(format!("The boat has not {} — check its messages and pre-arm checks", if wanted { "armed" } else { "disarmed" }));
                }
                self.send_arm(wanted);
            } else if wanted == armed {
                refused = 0;
            }
            for _ in 0..10 {
                if self.stop.load(Ordering::Relaxed) {
                    return;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    fn say(&self, text: String) {
        let mut texts = self.texts.lock().unwrap();
        if texts.len() < 50 {
            texts.push(text);
        }
    }

    fn send_arm(&self, armed: bool) {
        let target = self.target();
        if armed {
            // MAV_CMD_DO_SET_MODE: custom mode MANUAL (0) first.
            self.send_all(COMMAND_LONG, &command_long(target, 176, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        // MAV_CMD_COMPONENT_ARM_DISARM.
        self.send_all(COMMAND_LONG, &command_long(target, 400, [if armed { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        *self.arm_wanted.lock().unwrap() = (armed, clock::monotonic());
    }
}

/// ArduPilot Rover's mode numbers.
fn rover_mode(n: u32) -> &'static str {
    match n {
        0 => "MANUAL",
        1 => "ACRO",
        3 => "STEERING",
        4 => "HOLD",
        5 => "LOITER",
        6 => "FOLLOW",
        7 => "SIMPLE",
        8 => "DOCK",
        9 => "CIRCLE",
        10 => "AUTO",
        11 => "RTL",
        12 => "SMART_RTL",
        15 => "GUIDED",
        16 => "INITIALISING",
        _ => "?",
    }
}

pub struct MavlinkLink {
    shared: Arc<Shared>,
    label: String,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl MavlinkLink {
    /// Open every path (the boat need not be there yet).
    pub fn connect(settings: &LinkSettings) -> Result<MavlinkLink, String> {
        let specs = parse_paths(&settings.mavlink_url)?;
        let mut paths = Vec::new();
        for spec in specs {
            let (sender, receiver) = open(&spec.endpoint)?;
            let name = format!("{} · {}", spec.bearer.label(), spec.endpoint.describe());
            paths.push(Path {
                meter: Mutex::new(FlowMeter::new(spec.bearer, name)),
                spec,
                sender: Mutex::new(sender),
                receiver: Mutex::new(Some(receiver)),
                enabled: AtomicBool::new(true),
            });
        }
        let label = if paths.len() == 1 {
            format!("MAVLINK {}", paths[0].spec.endpoint.describe())
        } else {
            format!("MAVLINK {} paths", paths.len())
        };
        Ok(MavlinkLink {
            shared: Arc::new(Shared {
                settings: settings.clone(),
                paths,
                view: Mutex::new(BoatView::default()),
                handler: Mutex::new(None),
                seq: Mutex::new(0),
                stop: AtomicBool::new(false),
                arm_wanted: Mutex::new((false, f64::NEG_INFINITY)),
                texts: Mutex::new(Vec::new()),
                received: AtomicUsize::new(0),
                duplicates: AtomicUsize::new(0),
                bad_crc: AtomicUsize::new(0),
                overrides: AtomicUsize::new(0),
            }),
            label,
            threads: Mutex::new(Vec::new()),
        })
    }

    pub fn frames_received(&self) -> usize {
        self.shared.received.load(Ordering::Relaxed)
    }

    /// Messages that came by more than one path, dropped as copies.
    pub fn duplicates(&self) -> usize {
        self.shared.duplicates.load(Ordering::Relaxed)
    }
}

impl BoatLink for MavlinkLink {
    fn name(&self) -> &str {
        &self.label
    }

    fn open(&self) {
        let mut threads = self.threads.lock().unwrap();
        if !threads.is_empty() {
            return;
        }
        self.shared.stop.store(false, Ordering::Relaxed);
        for i in 0..self.shared.paths.len() {
            let shared = self.shared.clone();
            threads.extend(std::thread::Builder::new().name(format!("mavlink-{i}")).spawn(move || shared.listen(i)).ok());
        }
        let shared = self.shared.clone();
        threads.extend(std::thread::Builder::new().name("mavlink".into()).spawn(move || shared.housekeeping()).ok());
    }

    fn close(&self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        for t in self.threads.lock().unwrap().drain(..) {
            let _ = t.join();
        }
    }

    fn send_command(&self, port: f64, stbd: f64, armed: bool) {
        let shared = &self.shared;
        let s = &shared.settings;
        let (wanted, _) = *shared.arm_wanted.lock().unwrap();
        if armed != wanted {
            shared.send_arm(armed);
        }
        let (throttle, steer) = crate::vehicle::autopilot::unmix(port, stbd);
        let (throttle, steer) = if armed { (throttle, steer) } else { (0.0, 0.0) };
        let mut channels = [0u16; 8];
        channels[(s.rc_throttle.clamp(1, 8) - 1) as usize] = pwm(throttle, s);
        channels[(s.rc_steer.clamp(1, 8) - 1) as usize] = pwm(steer, s);
        let frame = shared.frame(RC_CHANNELS_OVERRIDE, &rc_override(shared.target(), channels));
        let n = shared.overrides.fetch_add(1, Ordering::Relaxed);
        match shared.primary() {
            Some(i) => {
                if !slow(shared.paths[i].spec.bearer) || n % 4 == 0 {
                    shared.send_on(i, &frame);
                }
            }
            // Nothing heard on any path yet: try them all.
            None => {
                for i in 0..shared.paths.len() {
                    if !slow(shared.paths[i].spec.bearer) || n % 4 == 0 {
                        shared.send_on(i, &frame);
                    }
                }
            }
        }
    }

    fn set_telemetry_handler(&self, handler: Option<TelemetryHandler>) {
        *self.shared.handler.lock().unwrap() = handler;
    }

    fn take_messages(&self) -> Vec<String> {
        std::mem::take(&mut *self.shared.texts.lock().unwrap())
    }

    fn flows(&self) -> Vec<FlowStats> {
        let now = clock::monotonic();
        let primary = self.shared.primary();
        self.shared
            .paths
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let mut m = p.meter.lock().unwrap();
                m.active = Some(i) == primary;
                m.enabled = p.enabled.load(Ordering::Relaxed);
                m.stats(now)
            })
            .collect()
    }

    fn set_flow_enabled(&self, index: usize, on: bool) {
        if let Some(p) = self.shared.paths.get(index) {
            p.enabled.store(on, Ordering::Relaxed);
        }
    }

    fn info(&self) -> Vec<(String, String)> {
        let sh = &self.shared;
        let v = sh.view.lock().unwrap();
        let autopilot = match v.autopilot {
            Some(3) => "ArduPilot".into(),
            Some(12) => "PX4".into(),
            Some(n) => format!("type {n}"),
            None => "not heard yet".into(),
        };
        let vehicle = match v.vehicle_type {
            Some(10) => "Ground rover".into(),
            Some(11) => "Surface boat".into(),
            Some(n) => format!("MAV_TYPE {n}"),
            None => "—".into(),
        };
        let mut out: Vec<(String, String)> = vec![
            ("Autopilot".into(), autopilot),
            ("Vehicle".into(), vehicle),
            ("Firmware".into(), v.firmware.clone().unwrap_or("—".into())),
            ("System id".into(), v.sysid.map(|s| s.to_string()).unwrap_or("—".into())),
            ("Autopilot mode".into(), if v.sysid.is_some() { rover_mode(v.custom_mode).into() } else { "—".into() }),
            ("Boat up for".into(), if v.time_boot_ms > 0 { crate::core::units::format_duration(Some(v.time_boot_ms as f64 / 1000.0)) } else { "—".into() }),
        ];
        drop(v);
        out.push(("Messages taken".into(), sh.received.load(Ordering::Relaxed).to_string()));
        out.push(("Copies dropped".into(), sh.duplicates.load(Ordering::Relaxed).to_string()));
        out.push(("Bad checksums".into(), sh.bad_crc.load(Ordering::Relaxed).to_string()));
        out.push(("Commands by".into(), sh.primary().map(|i| sh.paths[i].spec.bearer.label().to_string()).unwrap_or("every path (none heard)".into())));
        out
    }
}

impl Drop for MavlinkLink {
    fn drop(&mut self) {
        self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_the_mavlink_reference() {
        // The checksum of "123456789" with no extra, by MCRF4XX, is 0x6F91.
        let mut c = 0xFFFF;
        for b in b"123456789" {
            c = crc_accumulate(c, *b);
        }
        assert_eq!(c, 0x6F91);
    }

    #[test]
    fn frames_round_trip_through_the_parser() {
        let payload = command_long(1, 400, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let frame = encode(7, 255, 190, COMMAND_LONG, &payload);
        let mut p = Parser::default();
        // In pieces, with noise (and a stray start byte) before it.
        assert!(p.push(&[0x00, 0x13, 0xFD]).is_empty());
        let mut got = p.push(&frame[..5]);
        got.extend(p.push(&frame[5..]));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].id, COMMAND_LONG);
        assert_eq!(got[0].payload.len(), 33);
        assert_eq!(got[0].u16(28), 400);
        assert_eq!(got[0].f32(0), 1.0);
    }

    #[test]
    fn a_known_heartbeat_parses() {
        // A MAVLink v1 heartbeat from ArduPilot Rover (type 10 ground rover, autopilot 3), armed.
        let payload = [0u8, 0, 0, 0, 10, 3, 0x81, 4, 3];
        let mut frame = vec![0xFE, 9, 0, 1, 1, 0];
        frame.extend_from_slice(&payload);
        let c = crc(&frame[1..], 50);
        frame.extend_from_slice(&c.to_le_bytes());
        let got = Parser::default().push(&frame);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].u8(6) & 0x80, 0x80);
    }

    #[test]
    fn pwm_maps_both_ways() {
        let s = LinkSettings::default();
        assert_eq!(pwm(0.0, &s), 1500);
        assert_eq!(pwm(1.0, &s), 1900);
        assert_eq!(pwm(-1.0, &s), 1100);
        assert!((level(1700, &s) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn endpoints_parse() {
        assert_eq!(Endpoint::parse("udp-listen:14550").unwrap(), Endpoint::UdpListen(14550));
        assert_eq!(Endpoint::parse("tcp:127.0.0.1:5760").unwrap(), Endpoint::Tcp("127.0.0.1:5760".into()));
        assert_eq!(Endpoint::parse("serial:/dev/ttyUSB0:115200").unwrap(), Endpoint::Serial("/dev/ttyUSB0".into(), 115200));
        assert_eq!(Endpoint::parse("serial:/dev/ttyUSB0").unwrap(), Endpoint::Serial("/dev/ttyUSB0".into(), 57600));
        assert!(Endpoint::parse("carrier-pigeon").is_err());
    }
}
