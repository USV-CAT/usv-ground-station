//! How much battery is left, and what it will buy.
//!
//! Estimates from a short history of voltage, current and speed rather than
//! from the instantaneous draw, which swings with every throttle change.  Two
//! independent estimates of the time left are made — from the current draw
//! against the remaining capacity, and from the rate the state of charge has
//! actually been falling — and the more pessimistic is shown.

/// A 4S lithium-ion pack's nominal voltage.
pub const NOMINAL_V: f64 = 14.8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PowerSample {
    /// Boat time, seconds.
    pub t: f64,
    pub volts: f64,
    pub amps: f64,
    pub percent: f64,
    /// Speed over ground, m/s.
    pub sog: f64,
    /// Energy used since the start, Wh.
    pub energy_wh: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnergyEstimate {
    pub power_w: f64,
    pub average_a: f64,
    pub used_wh: f64,
    pub remaining_wh: f64,
    /// Seconds until empty, until the low threshold, until the critical one.
    pub time_to_empty_s: f64,
    pub time_to_low_s: f64,
    pub time_to_critical_s: f64,
    /// Distance left at the recent average speed, metres.
    pub range_m: f64,
    /// Energy per kilometre over the ground at the recent pace.
    pub wh_per_km: Option<f64>,
    /// Charge needed to get home with a 30 % margin, percent.
    pub home_reserve_pct: Option<f64>,
}

impl EnergyEstimate {
    /// Charge against time, from now, at the estimated rate: the dashed line.
    pub fn projection(&self, now_t: f64, percent: f64, horizon_s: f64) -> [(f64, f64); 2] {
        if !self.time_to_empty_s.is_finite() || self.time_to_empty_s <= 0.0 {
            return [(now_t, percent), (now_t + horizon_s, percent)];
        }
        let rate = percent / self.time_to_empty_s;
        let end = horizon_s.min(self.time_to_empty_s);
        [(now_t, percent), (now_t + end, (percent - rate * end).max(0.0))]
    }
}

pub fn estimate<'a, I>(
    history: I,
    capacity_wh: f64,
    low_pct: f64,
    critical_pct: f64,
    home_distance_m: Option<f64>,
    window_s: f64,
) -> Option<EnergyEstimate>
where
    I: IntoIterator<Item = &'a PowerSample>,
    I::IntoIter: DoubleEndedIterator,
{
    let mut recent: Vec<PowerSample> = Vec::new();
    let mut iter = history.into_iter();
    let last = *iter.next_back()?;
    recent.push(last);
    for s in iter.rev() {
        if last.t - s.t > window_s {
            break;
        }
        recent.push(*s);
    }
    recent.reverse();
    let n = recent.len() as f64;
    let average_a = recent.iter().map(|s| s.amps).sum::<f64>() / n;
    let average_sog = recent.iter().map(|s| s.sog).sum::<f64>() / n;
    let remaining_wh = capacity_wh * last.percent / 100.0;
    let power = last.volts * last.amps;

    // 1. From the draw.
    let by_draw = if average_a > 0.05 { remaining_wh / (average_a * NOMINAL_V) * 3600.0 } else { f64::INFINITY };
    // 2. From how fast the charge has really been falling.
    let first = recent[0];
    let span = last.t - first.t;
    let drop = first.percent - last.percent;
    let by_trend = if span > 20.0 && drop > 0.05 { last.percent / (drop / span) } else { f64::INFINITY };
    let seconds = by_draw.min(by_trend);
    let rate = if seconds.is_finite() && seconds > 0.0 { last.percent / seconds } else { 0.0 };
    let until = |threshold: f64| {
        if last.percent <= threshold {
            0.0
        } else if rate > 0.0 {
            (last.percent - threshold) / rate
        } else {
            f64::INFINITY
        }
    };

    // Energy per distance, over the window, while actually moving.
    let (mut moved, mut spent) = (0.0, 0.0);
    for w in recent.windows(2) {
        let dt = w[1].t - w[0].t;
        if dt <= 0.0 {
            continue;
        }
        moved += w[1].sog * dt;
        spent += w[1].energy_wh - w[0].energy_wh;
    }
    let wh_per_km = if moved > 20.0 { Some(spent / (moved / 1000.0)) } else { None };
    let home_reserve_pct = match (home_distance_m, wh_per_km) {
        (Some(d), Some(per_km)) => Some(100.0 * per_km * d / 1000.0 * 1.3 / capacity_wh),
        _ => None,
    };

    Some(EnergyEstimate {
        power_w: power,
        average_a,
        used_wh: last.energy_wh,
        remaining_wh,
        time_to_empty_s: seconds,
        time_to_low_s: until(low_pct),
        time_to_critical_s: until(critical_pct),
        range_m: if seconds.is_finite() { average_sog * seconds } else { f64::INFINITY },
        wh_per_km,
        home_reserve_pct,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_draw() {
        let mut history = Vec::new();
        let mut energy = 0.0;
        for t in 0..300 {
            let pct = 80.0 - t as f64 * 0.01;
            energy += 15.0 * 10.0 / 3600.0;
            history.push(PowerSample { t: t as f64, volts: 15.0, amps: 10.0, percent: pct, sog: 1.5, energy_wh: energy });
        }
        let e = estimate(&history, 296.0, 30.0, 15.0, Some(500.0), 60.0).unwrap();
        let last = history.last().unwrap();
        let by_draw = 296.0 * last.percent / 100.0 / (10.0 * 14.8) * 3600.0;
        let by_trend = last.percent / 0.01;
        let want = by_draw.min(by_trend);
        assert!((e.time_to_empty_s - want).abs() / want < 0.02);
        assert!(0.0 < e.time_to_low_s && e.time_to_low_s < e.time_to_critical_s && e.time_to_critical_s < e.time_to_empty_s);
        let per_km = 150.0 / 1.5 / 3.6;
        assert!((e.wh_per_km.unwrap() - per_km).abs() / per_km < 0.05);
        assert!(e.home_reserve_pct.unwrap() > 0.0);
        let line = e.projection(last.t, last.percent, 600.0);
        assert!(line[1].1 < line[0].1);
    }

    #[test]
    fn no_draw_means_no_deadline() {
        let history: Vec<PowerSample> = (0..60)
            .map(|t| PowerSample { t: t as f64, volts: 16.0, amps: 0.0, percent: 90.0, sog: 0.0, energy_wh: 0.0 })
            .collect();
        let e = estimate(&history, 296.0, 30.0, 15.0, None, 60.0).unwrap();
        assert!(e.time_to_empty_s.is_infinite());
        assert!(e.home_reserve_pct.is_none());
    }
}
