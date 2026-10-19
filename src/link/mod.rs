//! The link to the boat.
//!
//! Everything above this layer talks to a [`BoatLink`] and never knows
//! whether the boat is real.  A link carries two things: actuator commands up
//! to the boat, and [`Telemetry`] back down.
//!
//! Three exist: the simulator, a real boat over MAVLink, and [`Idle`] — no
//! boat at all, while the start dialog is up.

pub mod flows;
pub mod mavlink;
pub mod sim;
pub mod sim_radio;

use std::sync::Arc;

use crate::core::geo::LatLon;
use crate::core::telemetry::Telemetry;

/// Called with every telemetry report, on the link's own thread.
pub type TelemetryHandler = Arc<dyn Fn(Telemetry) + Send + Sync>;

pub trait BoatLink: Send + Sync {
    /// Shown in the status bar.
    fn name(&self) -> &str;

    fn simulated(&self) -> bool {
        false
    }

    fn open(&self);

    fn close(&self);

    /// Motor levels in [-1, 1].  Sent continuously; the boat stops on silence.
    fn send_command(&self, port: f64, stbd: f64, armed: bool);

    /// A payload command, such as `("skimmer", true)`.  Ignored by default.
    fn send_payload(&self, _name: &str, _value: bool) {}

    fn set_telemetry_handler(&self, handler: Option<TelemetryHandler>);

    /// The paths the data travels by, measured.
    fn flows(&self) -> Vec<flows::FlowStats> {
        Vec::new()
    }

    /// Switch one path (by its place in [`flows`](Self::flows)) on or off.
    fn set_flow_enabled(&self, _index: usize, _on: bool) {}

    /// Facts about the link and the boat for the Info page: (what, value).
    fn info(&self) -> Vec<(String, String)> {
        Vec::new()
    }

    /// What the boat has said since the last call (its own text messages).
    fn take_messages(&self) -> Vec<String> {
        Vec::new()
    }

    /// Where the ground station is, when the link knows (the radio range is
    /// measured from it).
    fn station(&self) -> Option<LatLon> {
        None
    }

    /// The simulator behind this link, for the game controls.
    fn as_sim(&self) -> Option<&sim::SimLink> {
        None
    }
}

/// No boat: what the station runs on while the start dialog asks which.
pub struct Idle;

impl BoatLink for Idle {
    fn name(&self) -> &str {
        "NO BOAT"
    }
    fn open(&self) {}
    fn close(&self) {}
    fn send_command(&self, _port: f64, _stbd: f64, _armed: bool) {}
    fn set_telemetry_handler(&self, _handler: Option<TelemetryHandler>) {}
}

/// Which boat to connect to.
#[derive(Clone, Debug, PartialEq)]
pub enum Choice {
    Simulator,
    Mavlink(String),
}

impl Choice {
    pub fn describe(&self) -> String {
        match self {
            Choice::Simulator => "the simulator".into(),
            Choice::Mavlink(url) => mavlink::describe_paths(url),
        }
    }
}
