"""What the boat reports.

One immutable :class:`Telemetry` per report.  Every field may be None, because a
real boat's sensors fail independently and the display must be able to say
"no reading" rather than show a stale or invented number.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace


@dataclass(frozen=True)
class WaterQuality:
    """The water-quality sonde most survey USVs carry."""

    temperature_c: float | None = None
    ph: float | None = None
    dissolved_oxygen_mgl: float | None = None
    oxygen_saturation_pct: float | None = None
    conductivity_ms: float | None = None
    tds_ppm: float | None = None
    salinity_psu: float | None = None
    orp_mv: float | None = None
    turbidity_ntu: float | None = None


@dataclass(frozen=True)
class SkimmerState:
    """The oil-recovery payload: sweep arms, brush skimmer, tank and oil sensor."""

    fitted: bool = False
    deployed: bool = False
    #: Pump actually lifting oil (deployed, tank not full, oil in the arms).
    recovering: bool = False
    #: Litres of oil a minute going into the tank right now.
    recovery_lpm: float = 0.0
    #: Fraction of the oil met that is being kept, given the speed.
    efficiency: float = 0.0
    tank_l: float = 0.0
    tank_capacity_l: float = 0.0
    #: Oil recovered over the whole run, litres (tank fluid × oil content).
    recovered_oil_l: float = 0.0
    offloading: bool = False
    #: Oil-in-water fluorometer under the hull, parts per million.
    oil_ppm: float | None = None
    #: Estimated film thickness at the boat, micrometres.
    film_um: float | None = None


@dataclass(frozen=True)
class Telemetry:
    #: Monotonic receive time at the ground station.
    received: float = field(default_factory=time.monotonic)
    #: Seconds since the boat booted, as the boat counts them.
    boat_time_s: float | None = None

    # -- position and motion
    lat: float | None = None
    lon: float | None = None
    #: Compass heading, degrees true.
    heading_deg: float | None = None
    #: Course and speed over ground, from GNSS.
    cog_deg: float | None = None
    sog_ms: float | None = None
    #: Speed through the water (paddlewheel or Doppler log).
    stw_ms: float | None = None
    #: Rate of turn, degrees per second, positive to starboard.
    rot_dps: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None

    # -- GNSS
    gps_fix: int = 0          # 0 none, 2 = 2D, 3 = 3D, 4 = RTK float, 5 = RTK fixed
    satellites: int = 0
    hdop: float | None = None

    # -- propulsion
    #: What the motor controllers are actually doing, -1..1.
    throttle_port: float = 0.0
    throttle_stbd: float = 0.0
    rpm_port: float = 0.0
    rpm_stbd: float = 0.0

    # -- power
    battery_v: float | None = None
    battery_a: float | None = None
    battery_pct: float | None = None

    # -- environment
    depth_m: float | None = None
    #: Apparent wind, measured on board: speed and angle off the bow (+ stbd).
    wind_apparent_ms: float | None = None
    wind_apparent_deg: float | None = None
    water: WaterQuality = field(default_factory=WaterQuality)
    skimmer: SkimmerState = field(default_factory=SkimmerState)

    # -- state the boat itself knows
    armed: bool = False
    #: The boat's own failsafe stopped the motors (lost commands).
    onboard_failsafe: bool = False
    grounded: bool = False
    #: Signal strength at the boat's radio, dBm.
    rssi_dbm: float | None = None
    #: Command frames the boat received per second.
    uplink_hz: float | None = None

    def age(self, now: float | None = None) -> float:
        return (time.monotonic() if now is None else now) - self.received

    @property
    def has_position(self) -> bool:
        return self.lat is not None and self.lon is not None and self.gps_fix >= 2

    def with_(self, **changes) -> Telemetry:
        return replace(self, **changes)
