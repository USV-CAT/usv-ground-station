//! From sticks to helm orders.
//!
//! Two layouts, chosen on the Pad page of the side panel:
//!
//! **arcade** (the default, like a driving game): left stick up/down or the
//! triggers (R2 ahead, L2 astern) set the throttle; the right stick — or the
//! left one sideways — steers.
//!
//! **tank** (like the twin throttles of a real catamaran): the left stick is
//! the port motor, the right stick the starboard one.

use std::collections::BTreeSet;

use crate::config::StickMode;
use crate::vehicle::autopilot::{mix, unmix};
use crate::vehicle::vehicle::Operator;

use super::gamepad::PadState;

pub const BUTTON_HELP: [(&str, &str); 12] = [
    ("A", "Preprogram — run / resume the mission"),
    ("B", "USV STOP — all motors stop"),
    ("X", "Cycle Manual → Speed Hold → Course Hold"),
    ("Y", "Station hold here"),
    ("☰ hold", "Arm / disarm"),
    ("⧉ hold", "Return home"),
    ("⧉ tap", "Show / hide the side panel"),
    ("L1 / R1", "Zoom chart out / in"),
    ("D-pad ↑↓", "Speed setpoint ±0.1 m/s"),
    ("D-pad ←→", "Course setpoint ±5° (Course Hold) · else side panel tabs"),
    ("R3", "Chart view: north up → heading up → 3D chase camera"),
    ("L3", "Deploy / stow the oil skimmer"),
];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Helm {
    pub port: f64,
    pub stbd: f64,
    pub throttle: f64,
    pub steer: f64,
    /// The largest stick or trigger deflection, for the manual override.
    pub deflection: f64,
}

impl Helm {
    pub fn operator(&self, connected: bool) -> Operator {
        Operator {
            port: self.port,
            stbd: self.stbd,
            throttle: self.throttle,
            steer: self.steer,
            deflection: self.deflection,
            connected,
        }
    }
}

/// Dead zone, rescaled so the output still reaches ±1, then expo.
pub fn shape(value: f64, deadzone: f64, expo: f64) -> f64 {
    let magnitude = value.abs();
    if magnitude <= deadzone {
        return 0.0;
    }
    let m = ((magnitude - deadzone) / (1.0 - deadzone)).min(1.0);
    let m = (1.0 - expo) * m + expo * m.powi(3);
    if value > 0.0 { m } else { -m }
}

pub fn helm_from_pad(pad: &PadState, stick_mode: StickMode, deadzone: f64, expo: f64) -> Helm {
    if !pad.connected {
        return Helm::default();
    }
    let s = |v: f64| shape(v, deadzone, expo);
    let (lx, ly, rx, ry) = (s(pad.lx), s(pad.ly), s(pad.rx), s(pad.ry));
    let lt = if pad.lt > deadzone { pad.lt } else { 0.0 };
    let rt = if pad.rt > deadzone { pad.rt } else { 0.0 };
    let deflection = lx.abs().max(ly.abs()).max(rx.abs()).max(ry.abs()).max(lt).max(rt);

    if stick_mode == StickMode::Tank {
        let (port, stbd) = (ly, ry);
        let (throttle, steer) = unmix(port, stbd);
        return Helm { port, stbd, throttle, steer, deflection };
    }
    let throttle = (ly + rt - lt).clamp(-1.0, 1.0);
    let steer = if rx.abs() >= lx.abs() { rx } else { lx };
    let (port, stbd) = mix(throttle, steer);
    Helm { port, stbd, throttle, steer, deflection }
}

/// WASD or the arrow keys, for a laptop with no pad.  `keys` holds names
/// such as `W`, `Up` and `Shift`.
pub fn helm_from_keys(keys: &BTreeSet<&'static str>) -> Helm {
    let has = |k: &str| keys.contains(k);
    let ahead = (has("W") || has("Up")) as i32 - (has("S") || has("Down")) as i32;
    let turn = (has("D") || has("Right")) as i32 - (has("A") || has("Left")) as i32;
    if ahead == 0 && turn == 0 {
        return Helm::default();
    }
    let throttle = 0.8 * ahead as f64 * if has("Shift") { 1.25 } else { 1.0 };
    let steer = 0.35 * turn as f64;
    let (port, stbd) = mix(throttle, steer);
    Helm { port, stbd, throttle, steer, deflection: throttle.abs().max(steer.abs()) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pad() -> PadState {
        PadState { connected: true, ..PadState::default() }
    }

    #[test]
    fn deadzone_and_full_scale() {
        assert_eq!(shape(0.05, 0.08, 0.3), 0.0);
        assert!((shape(1.0, 0.08, 0.3) - 1.0).abs() < 1e-9);
        assert!((shape(-1.0, 0.08, 0.0) + 1.0).abs() < 1e-9);
        assert!(0.0 < shape(0.5, 0.08, 0.5) && shape(0.5, 0.08, 0.5) < shape(0.5, 0.08, 0.0));
    }

    #[test]
    fn arcade_forward_and_turn() {
        let helm = helm_from_pad(&PadState { ly: 1.0, ..pad() }, StickMode::Arcade, 0.08, 0.0);
        assert!((helm.port - 1.0).abs() < 1e-9 && (helm.stbd - 1.0).abs() < 1e-9);
        let helm = helm_from_pad(&PadState { rx: 1.0, ..pad() }, StickMode::Arcade, 0.08, 0.0);
        assert!(helm.port > 0.0 && 0.0 > helm.stbd); // pivot to starboard
    }

    #[test]
    fn triggers_drive_like_a_racing_game() {
        let helm = helm_from_pad(&PadState { rt: 1.0, ..pad() }, StickMode::Arcade, 0.08, 0.0);
        assert!((helm.throttle - 1.0).abs() < 1e-9);
        let helm = helm_from_pad(&PadState { lt: 1.0, ..pad() }, StickMode::Arcade, 0.08, 0.0);
        assert!((helm.throttle + 1.0).abs() < 1e-9);
    }

    #[test]
    fn tank_sticks_are_the_motors() {
        let helm = helm_from_pad(&PadState { ly: 0.6, ry: -0.6, ..pad() }, StickMode::Tank, 0.0, 0.0);
        assert!((helm.port - 0.6).abs() < 1e-9 && (helm.stbd + 0.6).abs() < 1e-9);
        assert!((helm.steer - 0.6).abs() < 1e-9);
    }

    #[test]
    fn disconnected_pad_is_neutral() {
        let p = PadState { connected: false, ly: 1.0, ..PadState::default() };
        assert_eq!(helm_from_pad(&p, StickMode::Arcade, 0.1, 0.0).deflection, 0.0);
    }

    #[test]
    fn keyboard() {
        let keys: BTreeSet<&'static str> = ["W", "D"].into_iter().collect();
        let helm = helm_from_keys(&keys);
        assert!(helm.port > helm.stbd && helm.stbd > 0.0);
    }
}
