//! Time.
//!
//! [`monotonic`] is seconds since the program started, for ages and rates;
//! [`unix_time`] is wall-clock time, for logs.  Tests can shift the monotonic
//! clock of their own thread with [`set_offset`] to fake a long silence.

use std::cell::Cell;
use std::sync::OnceLock;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

static START: OnceLock<Instant> = OnceLock::new();

thread_local! {
    static OFFSET: Cell<f64> = const { Cell::new(0.0) };
}

/// Monotonic seconds since the program started (plus this thread's test offset).
pub fn monotonic() -> f64 {
    let start = START.get_or_init(Instant::now);
    start.elapsed().as_secs_f64() + OFFSET.with(Cell::get)
}

/// Shift [`monotonic`] on the calling thread.  For tests.
pub fn set_offset(seconds: f64) {
    OFFSET.with(|o| o.set(seconds));
}

/// Seconds since 1970, UTC.
pub fn unix_time() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

/// Local time formatted with a strftime pattern.
pub fn local_strftime(pattern: &str) -> String {
    chrono::Local::now().format(pattern).to_string()
}

/// A Unix time as local `HH:MM:SS`.
pub fn local_hms(unix: f64) -> String {
    use chrono::TimeZone;
    match chrono::Local.timestamp_opt(unix as i64, 0) {
        chrono::LocalResult::Single(t) => t.format("%H:%M:%S").to_string(),
        _ => String::from("--:--:--"),
    }
}
