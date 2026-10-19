//! Gamepads, and the Steam Deck's built-in controls in particular.
//!
//! Two backends, tried in order:
//!
//! * **gilrs**, which reads every pad through the SDL controller database,
//!   including Steam's virtual pad and the Deck itself, so it is the one to
//!   prefer;
//! * **evdev**, read straight from `/dev/input` with a few ioctls.  It covers
//!   the Deck's own controls (the `hid-steam` driver, whose triggers sit on
//!   odd axes) and Steam's virtual Xbox pad.
//!
//! Either way a backend runs on its own thread and is asked for a
//! [`PadState`] snapshot; button presses also arrive as edges, so a tap that
//! comes and goes between two polls is not lost.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Button {
    A,
    B,
    X,
    Y,
    LB,
    RB,
    View,
    Menu,
    Guide,
    L3,
    R3,
    Up,
    Down,
    Left,
    Right,
    L4,
    R4,
    L5,
    R5,
}

impl Button {
    pub fn name(self) -> &'static str {
        match self {
            Button::A => "A",
            Button::B => "B",
            Button::X => "X",
            Button::Y => "Y",
            Button::LB => "LB",
            Button::RB => "RB",
            Button::View => "VIEW",
            Button::Menu => "MENU",
            Button::Guide => "GUIDE",
            Button::L3 => "L3",
            Button::R3 => "R3",
            Button::Up => "UP",
            Button::Down => "DOWN",
            Button::Left => "LEFT",
            Button::Right => "RIGHT",
            Button::L4 => "L4",
            Button::R4 => "R4",
            Button::L5 => "L5",
            Button::R5 => "R5",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PadState {
    pub connected: bool,
    pub name: String,
    pub backend: String,
    /// Sticks in [-1, 1].  Y is positive *up*, unlike the raw devices.
    pub lx: f64,
    pub ly: f64,
    pub rx: f64,
    pub ry: f64,
    /// Triggers in [0, 1].
    pub lt: f64,
    pub rt: f64,
    pub held: BTreeSet<Button>,
}

impl PadState {
    pub fn held_names(&self) -> String {
        self.held.iter().map(|b| b.name()).collect::<Vec<_>>().join(" ")
    }
}

/// The state a backend thread keeps up to date.
struct Shared {
    state: Mutex<PadState>,
    edges: Mutex<Sender<(Button, bool)>>,
    stop: AtomicBool,
}

impl Shared {
    fn set_button(&self, button: Button, pressed: bool) {
        let changed = {
            let mut s = self.state.lock().unwrap();
            if pressed { s.held.insert(button) } else { s.held.remove(&button) }
        };
        if changed {
            let _ = self.edges.lock().unwrap().send((button, pressed));
        }
    }

    fn connected(&self, name: &str) {
        let mut s = self.state.lock().unwrap();
        s.connected = true;
        s.name = name.to_string();
    }

    fn disconnected(&self) {
        let released: Vec<Button> = {
            let mut s = self.state.lock().unwrap();
            let backend = s.backend.clone();
            let released = s.held.iter().copied().collect();
            *s = PadState { backend, ..PadState::default() };
            released
        };
        for b in released {
            let _ = self.edges.lock().unwrap().send((b, false));
        }
    }
}

pub struct Backend {
    pub name: &'static str,
    shared: Arc<Shared>,
    edges: Receiver<(Button, bool)>,
    thread: Option<JoinHandle<()>>,
}

impl Backend {
    fn spawn(name: &'static str, run: impl FnOnce(Arc<Shared>, Sender<Result<(), String>>) + Send + 'static) -> Result<Backend, String> {
        let (tx, edges) = channel();
        let shared = Arc::new(Shared {
            state: Mutex::new(PadState { backend: name.to_string(), ..PadState::default() }),
            edges: Mutex::new(tx),
            stop: AtomicBool::new(false),
        });
        let (ready_tx, ready_rx) = channel();
        let thread_shared = shared.clone();
        let thread = std::thread::Builder::new()
            .name(format!("pad-{name}"))
            .spawn(move || run(thread_shared, ready_tx))
            .map_err(|e| e.to_string())?;
        match ready_rx.recv_timeout(Duration::from_secs(3)) {
            Ok(Ok(())) => Ok(Backend { name, shared, edges, thread: Some(thread) }),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(format!("{name} backend did not start")),
        }
    }

    pub fn state(&self) -> PadState {
        self.shared.state.lock().unwrap().clone()
    }

    /// Button presses and releases since the last call.
    pub fn edges(&self) -> Vec<(Button, bool)> {
        self.edges.try_iter().collect()
    }

    pub fn stop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for Backend {
    fn drop(&mut self) {
        self.stop();
    }
}

// --------------------------------------------------------------------------
// gilrs
// --------------------------------------------------------------------------

fn gilrs_button(b: gilrs::Button) -> Option<Button> {
    use gilrs::Button as G;
    Some(match b {
        G::South => Button::A,
        G::East => Button::B,
        G::West => Button::X,
        G::North => Button::Y,
        G::LeftTrigger => Button::LB,
        G::RightTrigger => Button::RB,
        G::Select => Button::View,
        G::Start => Button::Menu,
        G::Mode => Button::Guide,
        G::LeftThumb => Button::L3,
        G::RightThumb => Button::R3,
        G::DPadUp => Button::Up,
        G::DPadDown => Button::Down,
        G::DPadLeft => Button::Left,
        G::DPadRight => Button::Right,
        _ => return None,
    })
}

fn run_gilrs(shared: Arc<Shared>, ready: Sender<Result<(), String>>) {
    let mut gilrs = match gilrs::Gilrs::new() {
        Ok(g) => g,
        Err(gilrs::Error::NotImplemented(g)) => {
            let _ = ready.send(Err("gilrs has no gamepad support on this platform".into()));
            drop(g);
            return;
        }
        Err(e) => {
            let _ = ready.send(Err(format!("gilrs: {e}")));
            return;
        }
    };
    let _ = ready.send(Ok(()));
    let mut active: Option<gilrs::GamepadId> = None;
    const BUTTONS: [gilrs::Button; 15] = [
        gilrs::Button::South,
        gilrs::Button::East,
        gilrs::Button::West,
        gilrs::Button::North,
        gilrs::Button::LeftTrigger,
        gilrs::Button::RightTrigger,
        gilrs::Button::Select,
        gilrs::Button::Start,
        gilrs::Button::Mode,
        gilrs::Button::LeftThumb,
        gilrs::Button::RightThumb,
        gilrs::Button::DPadUp,
        gilrs::Button::DPadDown,
        gilrs::Button::DPadLeft,
        gilrs::Button::DPadRight,
    ];
    while !shared.stop.load(Ordering::Relaxed) {
        while gilrs.next_event().is_some() {}
        if active.is_some_and(|id| !gilrs.gamepad(id).is_connected()) {
            log::info!("gamepad went away");
            shared.disconnected();
            active = None;
        }
        if active.is_none() {
            // Prefer the Deck itself, then an Xbox-style pad (Steam's virtual one).
            let rank = |name: &str| {
                let n = name.to_lowercase();
                if n.contains("steam deck") {
                    0
                } else if n.contains("x-box") || n.contains("xbox") || n.contains("steam") {
                    1
                } else if n.contains("motion") || n.contains("accel") {
                    9
                } else {
                    5
                }
            };
            active = gilrs.gamepads().min_by_key(|(_, g)| rank(g.name())).map(|(id, g)| {
                log::info!("gamepad (gilrs): {}", g.name());
                shared.connected(g.name());
                id
            });
            if active.is_none() {
                std::thread::sleep(Duration::from_millis(500));
                continue;
            }
        }
        let pad = gilrs.gamepad(active.unwrap());
        let trigger = |button: gilrs::Button, axis: gilrs::Axis| {
            let from_button = pad.button_data(button).map(|d| d.value() as f64).unwrap_or(0.0);
            let from_axis = pad.axis_data(axis).map(|d| d.value() as f64).unwrap_or(0.0);
            from_button.max(from_axis).clamp(0.0, 1.0)
        };
        {
            let mut s = shared.state.lock().unwrap();
            s.lx = (pad.value(gilrs::Axis::LeftStickX) as f64).clamp(-1.0, 1.0);
            s.ly = (pad.value(gilrs::Axis::LeftStickY) as f64).clamp(-1.0, 1.0);
            s.rx = (pad.value(gilrs::Axis::RightStickX) as f64).clamp(-1.0, 1.0);
            s.ry = (pad.value(gilrs::Axis::RightStickY) as f64).clamp(-1.0, 1.0);
            s.lt = trigger(gilrs::Button::LeftTrigger2, gilrs::Axis::LeftZ);
            s.rt = trigger(gilrs::Button::RightTrigger2, gilrs::Axis::RightZ);
        }
        for b in BUTTONS {
            if let Some(ours) = gilrs_button(b) {
                shared.set_button(ours, pad.is_pressed(b));
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

// --------------------------------------------------------------------------
// evdev
// --------------------------------------------------------------------------

const EV_KEY: u16 = 0x01;
const EV_ABS: u16 = 0x03;
const ABS_X: u16 = 0;
const ABS_Y: u16 = 1;
const ABS_Z: u16 = 2;
const ABS_RX: u16 = 3;
const ABS_RY: u16 = 4;
const ABS_RZ: u16 = 5;
const ABS_HAT0X: u16 = 16;
const ABS_HAT0Y: u16 = 17;
const ABS_HAT2X: u16 = 20;
const ABS_HAT2Y: u16 = 21;
const BTN_GAMEPAD: usize = 304;

/// Linux's gamepad convention (the `hid-steam` Deck driver follows it):
/// north is the top face button, west the left one.  The `xpad` driver, and
/// Steam's virtual pad that imitates it, send BTN_X (north) for X.
fn evdev_key(code: u16, xpad: bool) -> Option<Button> {
    Some(match code {
        304 => Button::A,
        305 => Button::B,
        307 => if xpad { Button::X } else { Button::Y },
        308 => if xpad { Button::Y } else { Button::X },
        310 => Button::LB,
        311 => Button::RB,
        314 => Button::View,
        315 => Button::Menu,
        316 => Button::Guide,
        317 => Button::L3,
        318 => Button::R3,
        544 => Button::Up,
        545 => Button::Down,
        546 => Button::Left,
        547 => Button::Right,
        704 => Button::L4,
        705 => Button::R4,
        706 => Button::L5,
        707 => Button::R5,
        _ => return None,
    })
}

#[cfg(target_os = "linux")]
mod sys {
    use std::os::fd::RawFd;

    const fn ioc(dir: u64, nr: u64, size: u64) -> u64 {
        (dir << 30) | (size << 16) | ((b'E' as u64) << 8) | nr
    }

    pub fn open(path: &str) -> Option<RawFd> {
        let c = std::ffi::CString::new(path).ok()?;
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDONLY | libc::O_NONBLOCK) };
        (fd >= 0).then_some(fd)
    }

    pub fn close(fd: RawFd) {
        unsafe { libc::close(fd) };
    }

    pub fn name(fd: RawFd) -> Option<String> {
        let mut buffer = [0u8; 256];
        let r = unsafe { libc::ioctl(fd, ioc(2, 0x06, 256) as _, buffer.as_mut_ptr()) };
        if r < 0 {
            return None;
        }
        let end = buffer.iter().position(|&b| b == 0).unwrap_or(buffer.len());
        Some(String::from_utf8_lossy(&buffer[..end]).to_string())
    }

    pub fn bits(fd: RawFd, event_type: u16, count: usize) -> Option<Vec<bool>> {
        let mut buffer = vec![0u8; count.div_ceil(8)];
        let r = unsafe { libc::ioctl(fd, ioc(2, 0x20 + event_type as u64, buffer.len() as u64) as _, buffer.as_mut_ptr()) };
        if r < 0 {
            return None;
        }
        Some((0..count).map(|i| buffer[i / 8] & (1 << (i % 8)) != 0).collect())
    }

    /// (minimum, maximum) of an absolute axis.
    pub fn abs_range(fd: RawFd, axis: u16) -> Option<(i32, i32)> {
        let mut info = [0i32; 6];
        let r = unsafe { libc::ioctl(fd, ioc(2, 0x40 + axis as u64, 24) as _, info.as_mut_ptr()) };
        if r < 0 {
            return None;
        }
        Some((info[1], info[2]))
    }

    /// Wait up to `timeout_ms` for events; (type, code, value) of each.
    pub fn read_events(fd: RawFd, timeout_ms: i32) -> Result<Vec<(u16, u16, i32)>, ()> {
        let mut pfd = libc::pollfd { fd, events: libc::POLLIN, revents: 0 };
        let ready = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        if ready < 0 {
            return Err(());
        }
        if ready == 0 {
            return Ok(Vec::new());
        }
        if pfd.revents & (libc::POLLERR | libc::POLLHUP) != 0 {
            return Err(());
        }
        let size = std::mem::size_of::<libc::input_event>();
        let mut buffer = vec![0u8; size * 64];
        let n = unsafe { libc::read(fd, buffer.as_mut_ptr() as *mut _, buffer.len()) };
        if n < 0 {
            let err = std::io::Error::last_os_error();
            return if err.kind() == std::io::ErrorKind::WouldBlock { Ok(Vec::new()) } else { Err(()) };
        }
        let mut out = Vec::new();
        for chunk in buffer[..n as usize].chunks_exact(size) {
            let event: libc::input_event = unsafe { std::ptr::read_unaligned(chunk.as_ptr() as *const _) };
            out.push((event.type_, event.code, event.value));
        }
        Ok(out)
    }
}

/// `(path, name)` of every readable gamepad, the likeliest first.
#[cfg(target_os = "linux")]
pub fn find_evdev_pads() -> Vec<(String, String)> {
    let mut found = Vec::new();
    let Ok(entries) = std::fs::read_dir("/dev/input") else { return found };
    let mut names: Vec<String> =
        entries.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().to_string()).collect();
    names.sort();
    for entry in names.into_iter().filter(|n| n.starts_with("event")) {
        let path = format!("/dev/input/{entry}");
        let Some(fd) = sys::open(&path) else { continue };
        let keys = sys::bits(fd, EV_KEY, 768);
        let axes = sys::bits(fd, EV_ABS, 64);
        if let (Some(keys), Some(axes)) = (keys, axes) {
            if keys[BTN_GAMEPAD] && axes[ABS_X as usize] {
                found.push((path, sys::name(fd).unwrap_or_default()));
            }
        }
        sys::close(fd);
    }
    let rank = |name: &str| {
        let n = name.to_lowercase();
        if n.contains("steam deck") {
            0
        } else if n.contains("x-box") || n.contains("xbox") || n.contains("steam") {
            1
        } else if n.contains("motion") || n.contains("accel") {
            9
        } else {
            5
        }
    };
    found.sort_by_key(|(_, name)| rank(name));
    found
}

#[cfg(not(target_os = "linux"))]
pub fn find_evdev_pads() -> Vec<(String, String)> {
    Vec::new()
}

#[cfg(target_os = "linux")]
fn run_evdev(shared: Arc<Shared>, ready: Sender<Result<(), String>>) {
    let _ = ready.send(Ok(()));
    while !shared.stop.load(Ordering::Relaxed) {
        let pads = find_evdev_pads();
        let Some((path, name)) = pads.into_iter().next() else {
            std::thread::sleep(Duration::from_secs(2));
            continue;
        };
        read_evdev(&shared, &path, &name);
        shared.disconnected();
        std::thread::sleep(Duration::from_secs(1));
    }
}

#[cfg(target_os = "linux")]
fn read_evdev(shared: &Shared, path: &str, name: &str) {
    let Some(fd) = sys::open(path) else { return };
    let lowered = name.to_lowercase();
    let xpad = lowered.contains("x-box") || lowered.contains("xbox") || lowered.contains("xpad");
    let available = sys::bits(fd, EV_ABS, 64).unwrap_or_default();
    let has = |axis: u16| available.get(axis as usize).copied().unwrap_or(false);
    let range = |axis: u16| sys::abs_range(fd, axis).unwrap_or((-1, 1));
    // Triggers: the Xbox layout puts them on Z/RZ; the Deck's own driver on
    // HAT2Y (left) and HAT2X (right).
    let (lt_axis, rt_axis) = if has(ABS_HAT2X) && !has(ABS_RZ) { (ABS_HAT2Y, ABS_HAT2X) } else { (ABS_Z, ABS_RZ) };
    let ranges: Vec<(i32, i32)> = (0..64).map(|a| if has(a) { range(a) } else { (-1, 1) }).collect();
    shared.connected(name);
    log::info!("gamepad: {name} ({path})");
    while !shared.stop.load(Ordering::Relaxed) {
        let Ok(events) = sys::read_events(fd, 250) else {
            log::info!("gamepad {name} went away");
            break;
        };
        for (kind, code, value) in events {
            if kind == EV_KEY {
                if let Some(b) = evdev_key(code, xpad) {
                    shared.set_button(b, value != 0);
                }
            } else if kind == EV_ABS {
                let (low, high) = ranges.get(code as usize).copied().unwrap_or((-1, 1));
                let (low, high) = (low as f64, high as f64);
                let stick = |sign: f64| {
                    let centre = (low + high) / 2.0;
                    let half = ((high - low) / 2.0).max(1.0);
                    (sign * (value as f64 - centre) / half).clamp(-1.0, 1.0)
                };
                let trigger = || ((value as f64 - low) / (high - low).max(1.0)).clamp(0.0, 1.0);
                let mut s = shared.state.lock().unwrap();
                match code {
                    ABS_X => s.lx = stick(1.0),
                    ABS_Y => s.ly = stick(-1.0),
                    ABS_RX => s.rx = stick(1.0),
                    ABS_RY => s.ry = stick(-1.0),
                    c if c == lt_axis => s.lt = trigger(),
                    c if c == rt_axis => s.rt = trigger(),
                    ABS_HAT0X => {
                        drop(s);
                        shared.set_button(Button::Left, value < 0);
                        shared.set_button(Button::Right, value > 0);
                    }
                    ABS_HAT0Y => {
                        drop(s);
                        shared.set_button(Button::Up, value < 0);
                        shared.set_button(Button::Down, value > 0);
                    }
                    _ => {}
                }
            }
        }
    }
    sys::close(fd);
}

#[cfg(not(target_os = "linux"))]
fn run_evdev(_shared: Arc<Shared>, ready: Sender<Result<(), String>>) {
    let _ = ready.send(Err("evdev is Linux only".into()));
}

/// Start the requested input backend, or the best available for `auto`.
pub fn open_backend(choice: &str) -> Option<Backend> {
    let order: &[&str] = match choice {
        "auto" => &["gilrs", "evdev"],
        "sdl" | "gilrs" | "pygame" => &["gilrs"],
        "evdev" => &["evdev"],
        _ => &[],
    };
    for name in order {
        let started = match *name {
            "gilrs" => Backend::spawn("gilrs", run_gilrs),
            _ => Backend::spawn("evdev", run_evdev),
        };
        match started {
            Ok(backend) => return Some(backend),
            Err(e) => log::info!("{name} input unavailable ({e}); trying the next backend"),
        }
    }
    None
}
