"""How much battery is left, and what it will buy.

Estimates from a short history of voltage, current and speed rather than from
the instantaneous draw, which swings with every throttle change.  Two
independent estimates of the time left are made — from the current draw against
the remaining capacity, and from the rate the state of charge has actually been
falling — and the more pessimistic is shown.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

#: A 4S lithium-ion pack's nominal voltage.
NOMINAL_V = 14.8


@dataclass(frozen=True)
class PowerSample:
    t: float            # boat time, seconds
    volts: float
    amps: float
    percent: float
    sog: float          # m/s
    energy_wh: float    # used since start


@dataclass(frozen=True)
class EnergyEstimate:
    power_w: float
    average_a: float
    used_wh: float
    remaining_wh: float
    #: Seconds until empty, until the low threshold, until the critical one.
    time_to_empty_s: float
    time_to_low_s: float
    time_to_critical_s: float
    #: Distance left at the recent average speed, metres.
    range_m: float
    #: Energy per kilometre over the ground at the recent pace.
    wh_per_km: float | None
    #: Charge needed to get home with a 30 % margin, percent; None if unknown.
    home_reserve_pct: float | None

    def projection(self, now_t: float, percent: float, horizon_s: float) -> list[tuple[float, float]]:
        """Charge against time, from now, at the estimated rate: the dashed line."""
        if not math.isfinite(self.time_to_empty_s) or self.time_to_empty_s <= 0:
            return [(now_t, percent), (now_t + horizon_s, percent)]
        rate = percent / self.time_to_empty_s
        end = min(horizon_s, self.time_to_empty_s)
        return [(now_t, percent), (now_t + end, max(0.0, percent - rate * end))]


def estimate(history: Sequence[PowerSample], capacity_wh: float, low_pct: float,
             critical_pct: float, home_distance_m: float | None,
             window_s: float = 60.0) -> EnergyEstimate | None:
    if not history:
        return None
    last = history[-1]
    recent = [s for s in history if last.t - s.t <= window_s] or [last]
    average_a = sum(s.amps for s in recent) / len(recent)
    average_sog = sum(s.sog for s in recent) / len(recent)
    remaining_wh = capacity_wh * last.percent / 100.0
    power = last.volts * last.amps

    # 1. From the draw.
    by_draw = math.inf
    if average_a > 0.05:
        by_draw = remaining_wh / (average_a * NOMINAL_V) * 3600.0
    # 2. From how fast the charge has really been falling.
    by_trend = math.inf
    first = recent[0]
    span = last.t - first.t
    drop = first.percent - last.percent
    if span > 20.0 and drop > 0.05:
        by_trend = last.percent / (drop / span)
    seconds = min(by_draw, by_trend)
    rate = last.percent / seconds if math.isfinite(seconds) and seconds > 0 else 0.0

    def until(threshold: float) -> float:
        if last.percent <= threshold:
            return 0.0
        return (last.percent - threshold) / rate if rate > 0 else math.inf

    # Energy per distance, over the window, while actually moving.
    moved = 0.0
    spent = 0.0
    for a, b in zip(recent, recent[1:], strict=False):
        dt = b.t - a.t
        if dt <= 0:
            continue
        moved += b.sog * dt
        spent += b.energy_wh - a.energy_wh
    wh_per_km = spent / (moved / 1000.0) if moved > 20.0 else None

    reserve = None
    if home_distance_m is not None and wh_per_km is not None:
        need_wh = wh_per_km * home_distance_m / 1000.0 * 1.3
        reserve = 100.0 * need_wh / capacity_wh

    return EnergyEstimate(
        power_w=power,
        average_a=average_a,
        used_wh=last.energy_wh,
        remaining_wh=remaining_wh,
        time_to_empty_s=seconds,
        time_to_low_s=until(low_pct),
        time_to_critical_s=until(critical_pct),
        range_m=average_sog * seconds if math.isfinite(seconds) else math.inf,
        wh_per_km=wh_per_km,
        home_reserve_pct=reserve,
    )
