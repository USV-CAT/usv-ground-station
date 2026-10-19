//! The boat as the ground station sees it: modes, autopilot and safety.

pub mod autopilot;
pub mod limits;
pub mod modes;
#[allow(clippy::module_inception)]
pub mod vehicle;

pub use modes::Mode;
pub use vehicle::{Alarm, NavState, Vehicle, VehicleCore};
