//! Marine units.
//!
//! Everything inside the program is SI — metres, metres per second, degrees —
//! and is converted to what a helmsman reads (knots, nautical miles) only
//! for display.

/// Metres per second to knots.
pub const MS_TO_KN: f64 = 3600.0 / 1852.0;
/// One nautical mile in metres.
pub const NM_M: f64 = 1852.0;

pub fn knots(speed_ms: f64) -> f64 {
    speed_ms * MS_TO_KN
}

pub fn from_knots(speed_kn: f64) -> f64 {
    speed_kn / MS_TO_KN
}

/// Short distances in metres, long ones in nautical miles, as charts do.
pub fn format_distance(metres: Option<f64>) -> String {
    match metres {
        None => "—".into(),
        Some(m) if !m.is_finite() => "∞".into(),
        Some(m) if m < 1000.0 => format!("{:.0} m", m.max(0.0) + 0.0),
        Some(m) => format!("{:.2} nm", m / NM_M),
    }
}

pub fn format_duration(seconds: Option<f64>) -> String {
    let Some(s) = seconds else { return "—".into() };
    if s.is_nan() || s < 0.0 {
        return "—".into();
    }
    if !s.is_finite() {
        return "∞".into();
    }
    let total = s.round() as i64;
    let (hours, rest) = (total / 3600, total % 3600);
    let (minutes, secs) = (rest / 60, rest % 60);
    if hours > 0 {
        format!("{hours}:{minutes:02}:{secs:02}")
    } else {
        format!("{minutes:02}:{secs:02}")
    }
}

pub fn format_bearing(degrees: Option<f64>) -> String {
    match degrees {
        None => "—".into(),
        Some(d) => format!("{:03.0}°", d.rem_euclid(360.0)),
    }
}

pub fn compass_point(degrees: f64) -> &'static str {
    const POINTS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    POINTS[(((degrees.rem_euclid(360.0)) + 22.5) / 45.0).floor() as usize % 8]
}

/// Degrees and decimal minutes, as a chart plotter shows a position.
pub fn dms(value: Option<f64>, positive: char, negative: char) -> String {
    let Some(v) = value else { return "—".into() };
    let hemisphere = if v >= 0.0 { positive } else { negative };
    let v = v.abs();
    let degrees = v.trunc();
    let minutes = (v - degrees) * 60.0;
    format!("{}°{:06.3}′{}", degrees as i64, minutes, hemisphere)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats() {
        assert_eq!(format_distance(Some(512.4)), "512 m");
        assert_eq!(format_distance(Some(1852.0)), "1.00 nm");
        assert_eq!(format_duration(Some(65.0)), "01:05");
        assert_eq!(format_duration(Some(3725.0)), "1:02:05");
        assert_eq!(format_bearing(Some(-5.0)), "355°");
        assert_eq!(format_bearing(Some(7.0)), "007°");
        assert_eq!(compass_point(350.0), "N");
        assert_eq!(dms(Some(57.5), 'N', 'S'), "57°30.000′N");
    }
}

/// The speed units the operator reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedUnit {
    Knots,
    MetresPerSecond,
}

impl SpeedUnit {
    /// From the settings' `speed_units`: `kn` or `ms`.
    pub fn from_setting(text: &str) -> SpeedUnit {
        if text == "ms" { SpeedUnit::MetresPerSecond } else { SpeedUnit::Knots }
    }

    pub fn setting(self) -> &'static str {
        match self {
            SpeedUnit::Knots => "kn",
            SpeedUnit::MetresPerSecond => "ms",
        }
    }

    pub fn suffix(self) -> &'static str {
        match self {
            SpeedUnit::Knots => "kn",
            SpeedUnit::MetresPerSecond => "m/s",
        }
    }

    pub fn from_ms(self, ms: f64) -> f64 {
        match self {
            SpeedUnit::Knots => knots(ms),
            SpeedUnit::MetresPerSecond => ms,
        }
    }

    pub fn to_ms(self, value: f64) -> f64 {
        match self {
            SpeedUnit::Knots => from_knots(value),
            SpeedUnit::MetresPerSecond => value,
        }
    }

    /// Decimals worth showing: a tenth of a knot, a hundredth of a m/s.
    pub fn decimals(self) -> usize {
        match self {
            SpeedUnit::Knots => 1,
            SpeedUnit::MetresPerSecond => 2,
        }
    }

    pub fn format(self, ms: f64) -> String {
        format!("{:.*} {}", self.decimals(), self.from_ms(ms), self.suffix())
    }

    pub fn other(self) -> SpeedUnit {
        match self {
            SpeedUnit::Knots => SpeedUnit::MetresPerSecond,
            SpeedUnit::MetresPerSecond => SpeedUnit::Knots,
        }
    }
}

/// Power in watts, or kilowatts once it is big enough.
pub fn format_power(watts: f64) -> String {
    if watts.abs() >= 1000.0 { format!("{:.2} kW", watts / 1000.0) } else { format!("{watts:.0} W") }
}

#[cfg(test)]
mod speed_tests {
    use super::*;

    #[test]
    fn speed_units_round_trip() {
        for unit in [SpeedUnit::Knots, SpeedUnit::MetresPerSecond] {
            assert!((unit.to_ms(unit.from_ms(1.234)) - 1.234).abs() < 1e-12);
            assert_eq!(SpeedUnit::from_setting(unit.setting()), unit);
        }
        assert_eq!(SpeedUnit::Knots.format(1.0), "1.9 kn");
        assert_eq!(SpeedUnit::MetresPerSecond.format(1.0), "1.00 m/s");
        assert_eq!(format_power(1500.0), "1.50 kW");
    }
}
