//! The parts every other layer leans on: no threads, no I/O beyond files.

pub mod bathymetry;
pub mod clock;
pub mod energy;
pub mod events;
pub mod geo;
pub mod mission;
pub mod npy;
pub mod profile;
pub mod rng;
pub mod telemetry;
pub mod units;
pub mod vectors;
