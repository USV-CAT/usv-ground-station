//! The simulated boat and the water it floats in — "the game".
//!
//! Nothing here is used when a real boat is connected: the rest of the
//! program only talks to the boat through [`crate::link::BoatLink`].

pub mod boat;
pub mod oil;
pub mod world;
