//! usvgs — a portable ground station for a small catamaran USV.
//!
//! Built for the Steam Deck: the sticks drive the boat directly, the
//! touchscreen plans routes on an offline chart, and a simulated catamaran
//! stands in for the real one until there is hardware to talk to.
//!
//! The layers, bottom up:
//!
//! * [`core`] — geodesy, units, missions, bathymetry, telemetry, events and
//!   the navigator's arithmetic (vectors, energy, depth profiles);
//! * [`sim`] — the simulated catamaran, its world and the oil slick;
//! * [`link`] — the [`link::BoatLink`] interface and the simulated link;
//! * [`vehicle`] — the single source of truth: modes, autopilot, safety;
//! * [`input`] — gamepads and the stick/button mapping;
//! * [`ui`] — the egui window, chart and instruments;
//! * [`tools`] — offline chart and map builders.

pub mod config;
pub mod core;
pub mod input;
pub mod link;
pub mod sim;
pub mod sound;
pub mod tools;
pub mod ui;
pub mod vehicle;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// The Gulf of Riga depth chart, built into the program so it always has one.
pub static BUNDLED_CHART: &[u8] = include_bytes!("../data/riga_bathy.npz");
