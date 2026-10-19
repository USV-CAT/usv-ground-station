//! The MAVLink link against a fake ArduPilot boat on a local UDP socket.

use std::net::UdpSocket;
use std::sync::Arc;
use std::time::{Duration, Instant};

use usvgs::config::Settings;
use usvgs::link::mavlink::{self, Message, Parser, MavlinkLink};
use usvgs::link::BoatLink;
use usvgs::vehicle::{Mode, Vehicle};

/// One of the boat's channels: a socket, its own sequence numbers, and a
/// pattern of losses (every `drop_every`th frame out is lost; 0: none).
struct Channel {
    socket: UdpSocket,
    parser: Parser,
    seq: u8,
    sent: u32,
    drop_every: u32,
    peer: Option<std::net::SocketAddr>,
    heard: Vec<Message>,
}

/// A boat that answers like ArduPilot Rover, at 57.0°N 24.0°E, on one or
/// more channels at once (a radio and Wi-Fi, say).
struct FakeBoat {
    channels: Vec<Channel>,
    armed: bool,
    heard: Vec<Message>,
}

impl FakeBoat {
    fn new() -> FakeBoat {
        FakeBoat::with_channels(&[0])
    }

    fn with_channels(drops: &[u32]) -> FakeBoat {
        let channels = drops
            .iter()
            .map(|&drop_every| {
                let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
                socket.set_read_timeout(Some(Duration::from_millis(5))).unwrap();
                Channel { socket, parser: Parser::default(), seq: 0, sent: 0, drop_every, peer: None, heard: Vec::new() }
            })
            .collect();
        FakeBoat { channels, armed: false, heard: Vec::new() }
    }

    fn port(&self) -> u16 {
        self.port_of(0)
    }

    fn port_of(&self, c: usize) -> u16 {
        self.channels[c].socket.local_addr().unwrap().port()
    }

    /// Send on every channel that has heard from the station.
    fn send(&mut self, id: u32, payload: &[u8]) {
        for c in &mut self.channels {
            let Some(to) = c.peer else { continue };
            c.seq = c.seq.wrapping_add(1);
            c.sent += 1;
            if c.drop_every > 0 && c.sent % c.drop_every == 0 {
                continue; // lost on the way
            }
            c.socket.send_to(&mavlink::encode(c.seq, 1, 1, id, payload), to).unwrap();
        }
    }

    /// Answer for a while: stream telemetry to whoever spoke, obey arming,
    /// answer time requests.
    fn run(&mut self, seconds: f64) {
        let mut buf = [0u8; 2048];
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        let mut last = Instant::now() - Duration::from_secs(1);
        let mut first_text = true;
        while Instant::now() < end {
            for k in 0..self.channels.len() {
                let c = &mut self.channels[k];
                let Ok((n, from)) = c.socket.recv_from(&mut buf) else { continue };
                c.peer = Some(from);
                for m in c.parser.push(&buf[..n]) {
                    if m.id == mavlink::COMMAND_LONG && u16::from_le_bytes([m.payload[28], m.payload[29]]) == 400 {
                        self.armed = f32::from_le_bytes(m.payload[0..4].try_into().unwrap()) == 1.0;
                    }
                    if m.id == mavlink::TIMESYNC && m.payload[0..8] == [0u8; 8] {
                        // Answer on the channel it came by, as ArduPilot does.
                        let mut reply = 1_000_000i64.to_le_bytes().to_vec();
                        reply.extend_from_slice(&m.payload[8..16]);
                        c.seq = c.seq.wrapping_add(1);
                        c.socket.send_to(&mavlink::encode(c.seq, 1, 1, mavlink::TIMESYNC, &reply), from).unwrap();
                    }
                    c.heard.push(m.clone());
                    self.heard.push(m);
                }
            }
            if last.elapsed() >= Duration::from_millis(100) {
                last = Instant::now();
                self.stream(first_text);
                first_text = false;
            }
        }
    }

    fn stream(&mut self, text: bool) {
        // HEARTBEAT: ground rover, ArduPilot, armed flag.
        let mut hb = vec![0u8; 4];
        hb.extend_from_slice(&[10, 3, if self.armed { 0x81 } else { 0x01 }, 4, 3]);
        self.send(mavlink::HEARTBEAT, &hb);
        // GLOBAL_POSITION_INT: 57.0, 24.0, heading 90°, 1 m/s east.
        let mut p = Vec::new();
        p.extend_from_slice(&1000u32.to_le_bytes());
        p.extend_from_slice(&570_000_000i32.to_le_bytes());
        p.extend_from_slice(&240_000_000i32.to_le_bytes());
        p.extend_from_slice(&[0u8; 8]);
        p.extend_from_slice(&0i16.to_le_bytes());
        p.extend_from_slice(&100i16.to_le_bytes());
        p.extend_from_slice(&0i16.to_le_bytes());
        p.extend_from_slice(&9000u16.to_le_bytes());
        self.send(mavlink::GLOBAL_POSITION_INT, &p);
        // SYS_STATUS: 15.2 V, 6.5 A, 73 %.
        let mut s = vec![0u8; 14];
        s.extend_from_slice(&15200u16.to_le_bytes());
        s.extend_from_slice(&650i16.to_le_bytes());
        s.extend_from_slice(&[0u8; 12]);
        s.push(73);
        self.send(mavlink::SYS_STATUS, &s);
        // GPS_RAW_INT: RTK fixed, 17 satellites, 1.00 m/s on 88°.
        let mut g = vec![0u8; 20];
        g.extend_from_slice(&80u16.to_le_bytes());
        g.extend_from_slice(&120u16.to_le_bytes());
        g.extend_from_slice(&100u16.to_le_bytes());
        g.extend_from_slice(&8800u16.to_le_bytes());
        g.extend_from_slice(&[6, 17]);
        self.send(mavlink::GPS_RAW_INT, &g);
        // DISTANCE_SENSOR pointing down: 4.20 m.
        let mut d = vec![0u8; 8];
        d.extend_from_slice(&420u16.to_le_bytes());
        d.extend_from_slice(&[10, 0, 25, 0]);
        self.send(mavlink::DISTANCE_SENSOR, &d);
        if text {
            let mut text = vec![6u8];
            text.extend_from_slice(b"ArduRover V4.5.7");
            text.resize(51, 0);
            self.send(mavlink::STATUSTEXT, &text);
        }
    }
}

#[test]
fn a_fake_ardupilot_boat_reports_and_obeys() {
    let mut boat = FakeBoat::new();
    let mut settings = Settings::default();
    settings.link.mavlink_url = format!("udp:127.0.0.1:{}", boat.port());
    let shared = settings.clone().shared();
    let link: Arc<dyn BoatLink> = Arc::new(MavlinkLink::connect(&settings.link).unwrap());
    assert!(link.name().contains("UDP 127.0.0.1"));
    let vehicle = Vehicle::new(shared, link.clone(), None, false);
    vehicle.start();
    boat.run(1.5);

    let nav = vehicle.nav();
    assert!(nav.link_ok, "linked");
    let t = nav.telemetry.clone().expect("telemetry");
    let p = nav.position().expect("a position");
    assert!((p.lat - 57.0).abs() < 1e-6 && (p.lon - 24.0).abs() < 1e-6);
    assert_eq!(t.heading_deg, Some(90.0));
    assert_eq!(t.battery_v, Some(15.2));
    assert_eq!(t.battery_a, Some(6.5));
    assert_eq!(t.battery_pct, Some(73.0));
    assert_eq!((t.gps_fix, t.satellites), (5, 17));
    assert_eq!(t.sog_ms, Some(1.0));
    assert_eq!(t.cog_deg, Some(88.0));
    assert_eq!(t.depth_m, Some(4.2));
    assert!(boat.heard.iter().any(|m| m.id == mavlink::HEARTBEAT), "the station's heartbeat");
    assert!(boat.heard.iter().any(|m| m.id == mavlink::REQUEST_DATA_STREAM), "asked for the streams");
    assert!(link.take_messages().contains(&"Boat: ArduRover V4.5.7".to_string()), "the boat's messages reach the log");

    // Arm and drive: the boat is armed, in MANUAL, and given throttle.
    vehicle.lock().arm();
    vehicle.lock().set_mode(Mode::Manual);
    vehicle.lock().set_operator(usvgs::vehicle::vehicle::Operator { port: 0.6, stbd: 0.6, throttle: 0.6, steer: 0.0, deflection: 0.6, connected: true });
    boat.heard.clear();
    boat.run(2.0);
    assert!(boat.armed, "armed by COMMAND_LONG");
    assert!(boat.heard.iter().any(|m| m.id == mavlink::COMMAND_LONG && m.payload[28] == 176), "MANUAL mode asked for");
    let throttle: Vec<u16> =
        boat.heard.iter().filter(|m| m.id == mavlink::RC_CHANNELS_OVERRIDE).map(|m| u16::from_le_bytes([m.payload[4], m.payload[5]])).collect();
    assert!(throttle.len() > 20, "the override streams at the control rate ({})", throttle.len());
    assert!(*throttle.last().unwrap() > 1600, "throttle on channel 3: {:?}", throttle.last());
    assert!(vehicle.nav().armed);

    // USV STOP: neutral at once, then disarmed.
    vehicle.lock().set_mode(Mode::Stop);
    vehicle.lock().disarm();
    boat.heard.clear();
    boat.run(1.5);
    assert!(!boat.armed, "disarmed");
    let last = boat.heard.iter().rev().find(|m| m.id == mavlink::RC_CHANNELS_OVERRIDE).unwrap();
    assert_eq!(u16::from_le_bytes([last.payload[4], last.payload[5]]), 1500);
    let t0 = Instant::now();
    vehicle.shutdown();
    assert!(t0.elapsed() < Duration::from_millis(500), "shutting the link is prompt: {:?}", t0.elapsed());
}

#[test]
fn a_missing_boat_shows_as_no_link() {
    let mut settings = Settings::default();
    settings.link.mavlink_url = "udp-listen:0".into();
    let link: Arc<dyn BoatLink> = Arc::new(MavlinkLink::connect(&settings.link).unwrap());
    let vehicle = Vehicle::new(settings.shared(), link, None, false);
    vehicle.start();
    std::thread::sleep(Duration::from_millis(400));
    assert!(!vehicle.nav().link_ok);
    vehicle.shutdown();
}

#[test]
fn a_bad_connection_is_refused_with_a_reason() {
    let mut settings = Settings::default();
    settings.link.mavlink_url = "serial:/dev/does-not-exist:57600".into();
    let e = MavlinkLink::connect(&settings.link).err().expect("refused");
    assert!(e.contains("/dev/does-not-exist"), "{e}");
}

/// Two paths at once: a lossy LoRa radio and Wi-Fi.  Each is metered on its
/// own; the boat's messages are taken once; the commands go by Wi-Fi.
#[test]
fn two_paths_are_metered_apart() {
    use usvgs::link::flows::{Bearer, FlowState};
    // Channel 0 (LoRa) loses one frame in four; channel 1 (Wi-Fi) none.
    let mut boat = FakeBoat::with_channels(&[4, 0]);
    let mut settings = Settings::default();
    settings.link.mavlink_url = format!("lora=udp:127.0.0.1:{}, wifi=udp:127.0.0.1:{}", boat.port_of(0), boat.port_of(1));
    let link = Arc::new(MavlinkLink::connect(&settings.link).unwrap());
    assert_eq!(link.name(), "MAVLINK 2 paths");
    let dyn_link: Arc<dyn BoatLink> = link.clone();
    let vehicle = Vehicle::new(settings.shared(), dyn_link.clone(), None, false);
    vehicle.start();
    boat.run(3.0);

    let flows = dyn_link.flows();
    assert_eq!(flows.len(), 2);
    let (lora, wifi) = (&flows[0], &flows[1]);
    assert_eq!((lora.bearer, wifi.bearer), (Bearer::LoRa, Bearer::WiFi));
    assert_eq!(wifi.state, FlowState::Active, "Wi-Fi is preferred");
    assert_eq!(lora.state, FlowState::Standby);
    let lost = 100.0 * lora.lost_packets as f64 / (lora.lost_packets + lora.rx_packets) as f64;
    assert!((18.0..32.0).contains(&lost), "LoRa loses a quarter: {lost:.1} %");
    assert_eq!(wifi.lost_packets, 0);
    assert!(wifi.rx_bps > 1000.0 && lora.rx_bps > 500.0, "both carry data: {} / {}", wifi.rx_bps, lora.rx_bps);
    assert!(wifi.latency_ms.is_some(), "latency from TIMESYNC");

    // Telemetry is taken once, not once per path.
    let hz = vehicle.nav().downlink_hz;
    assert!((7.0..14.0).contains(&hz), "about 10 Hz, not 20: {hz}");
    assert!(link.duplicates() > 0, "the copies were seen and dropped");

    // The driving goes by Wi-Fi only; heartbeats by both.
    vehicle.lock().arm();
    vehicle.lock().set_operator(usvgs::vehicle::vehicle::Operator { port: 0.5, stbd: 0.5, throttle: 0.5, steer: 0.0, deflection: 0.5, connected: true });
    for c in &mut boat.channels {
        c.heard.clear();
    }
    boat.run(1.0);
    let overrides = |c: &Channel| c.heard.iter().filter(|m| m.id == mavlink::RC_CHANNELS_OVERRIDE).count();
    assert!(overrides(&boat.channels[1]) > 10);
    assert_eq!(overrides(&boat.channels[0]), 0);
    assert!(boat.channels.iter().all(|c| c.heard.iter().any(|m| m.id == mavlink::HEARTBEAT)));

    // Wi-Fi switched off: LoRa takes over, the commands follow it.
    dyn_link.set_flow_enabled(1, false);
    boat.run(1.5);
    let flows = dyn_link.flows();
    assert_eq!(flows[1].state, FlowState::Off);
    assert_eq!(flows[0].state, FlowState::Active);
    for c in &mut boat.channels {
        c.heard.clear();
    }
    boat.run(1.0);
    assert!(overrides(&boat.channels[0]) > 2, "LoRa now carries the override (one in four)");
    assert!(vehicle.nav().link_ok);
    let info = dyn_link.info();
    assert!(info.iter().any(|(k, v)| k == "Commands by" && v == "LoRa"), "{info:?}");
    assert!(info.iter().any(|(k, v)| k == "Autopilot" && v == "ArduPilot"));
    vehicle.shutdown();
}

#[test]
fn paths_parse_with_and_without_a_kind() {
    use usvgs::link::flows::Bearer;
    use usvgs::link::mavlink::{describe_paths, parse_paths};
    let p = parse_paths("lora=serial:/dev/ttyUSB0:57600, wifi=udp-listen:14550; udp:10.0.0.2:14550").unwrap();
    assert_eq!(p.iter().map(|p| p.bearer).collect::<Vec<_>>(), [Bearer::LoRa, Bearer::WiFi, Bearer::Ip]);
    assert_eq!(p[2].to_url(), "udp:10.0.0.2:14550");
    assert_eq!(p[0].to_url(), "lora=serial:/dev/ttyUSB0:57600");
    assert_eq!(describe_paths("5g=udp:a:1, lora=udp:b:2"), "5G / LTE + LoRa");
    assert!(parse_paths("pigeon=udp:a:1").is_err());
}
