"""Settings.

One dataclass per concern, gathered into :class:`Settings`, stored as readable
JSON in ``~/.config/usvgs/config.json``.  Unknown keys are ignored and missing
ones take their defaults, so a file written by an older version still loads.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path

log = logging.getLogger(__name__)


def config_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config"
    return Path(base) / "usvgs"


def state_dir() -> Path:
    base = os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state"
    return Path(base) / "usvgs"


@dataclass
class BoatSettings:
    #: Hull length overall, metres.  Used to draw the boat to scale.
    length_m: float = 1.6
    #: Distance between the two hull centrelines, metres.
    beam_m: float = 0.9
    #: How deep the hulls sit.  Water shallower than this grounds the boat.
    draft_m: float = 0.25
    #: Displacement including payload, kilograms.
    mass_kg: float = 55.0
    #: Static thrust of one motor at full forward, newtons.
    max_thrust_n: float = 38.0
    #: Battery capacity, ampere-hours (4S Li-ion pack).
    battery_ah: float = 20.0


@dataclass
class SimSettings:
    #: Where the simulated boat is launched.  Also the ground station position,
    #: which is what the radio range is measured from.
    start_lat: float = 57.0480
    start_lon: float = 23.9900
    start_heading_deg: float = 320.0
    #: True wind: speed in m/s and the direction it blows *from*.
    wind_speed_ms: float = 4.0
    wind_from_deg: float = 240.0
    #: Surface current: speed in m/s and the direction it flows *towards*.
    current_speed_ms: float = 0.08
    current_to_deg: float = 60.0
    #: Radio range from the ground station before telemetry starts dropping.
    link_range_m: float = 2500.0
    #: GNSS noise, metres (one sigma).
    gps_noise_m: float = 0.4
    #: Speed of simulated time relative to real time.
    time_scale: float = 1.0


@dataclass
class ControlSettings:
    #: ``arcade``: left stick throttle, right stick steering.
    #: ``tank``: left stick port motor, right stick starboard motor.
    stick_mode: str = "arcade"
    #: Stick travel ignored around centre.
    deadzone: float = 0.08
    #: 0 is linear; towards 1 the centre of the stick gets finer.
    expo: float = 0.35
    #: ``auto`` tries SDL (pygame) then raw evdev; or name one; ``none`` = keyboard only.
    input_backend: str = "auto"
    #: Stick deflection that takes manual control back from an autonomous mode.
    override_threshold: float = 0.55


@dataclass
class AutopilotSettings:
    #: Speed through autonomous legs, m/s (2.0 m/s ≈ 3.9 kn).
    cruise_speed_ms: float = 1.6
    #: A waypoint counts as reached inside this radius.
    accept_radius_m: float = 4.0
    #: Line-of-sight lookahead along the leg.  Shorter tracks the line more
    #: tightly and weaves more.
    lookahead_m: float = 12.0
    #: Station keeping starts correcting outside this radius.
    hold_radius_m: float = 3.0


@dataclass
class SafetySettings:
    #: Distance from home that triggers return to launch.
    geofence_radius_m: float = 2000.0
    #: Depth alarm.  Sounds in every mode.
    shallow_alarm_m: float = 1.2
    #: Autonomous modes stop and hold below this depth rather than ground.
    shallow_stop_m: float = 0.7
    #: State of charge that warns, and that returns home.
    battery_low_pct: float = 30.0
    battery_critical_pct: float = 15.0
    #: Telemetry silence that counts as a lost link.
    link_timeout_s: float = 2.0


@dataclass
class SurveySettings:
    #: Width of seabed the echo sounder is credited with on each ping, metres.
    swath_m: float = 10.0
    #: Cell size of the surveyed depth grid, metres.
    cell_m: float = 5.0


@dataclass
class SkimmerSettings:
    #: Width of water the sweep arms funnel into the skimmer, metres.
    swath_m: float = 2.4
    #: Recovered-fluid tank, litres.
    tank_l: float = 60.0
    #: Most the pump can lift, litres of fluid a minute.
    max_rate_lpm: float = 8.0
    #: Fraction of the recovered fluid that is oil (an oleophilic brush is 60–90 %).
    oil_content: float = 0.7
    #: Through-water speed the arms work best at, and the speed beyond which
    #: oil escapes under them (entrainment).  Booms are slow things: 0.7 kn and 1.5 kn.
    best_speed_ms: float = 0.36
    max_speed_ms: float = 0.77
    #: Pump and brush motor draw while running, amperes.
    power_a: float = 4.0
    #: Tank empties at this rate when alongside the ground station, litres a minute.
    offload_lpm: float = 12.0


@dataclass
class SpillSettings:
    #: Simulated spill: volume and how far from the launch point it starts.
    volume_l: float = 300.0
    distance_m: float = 450.0
    bearing_deg: float = 330.0
    radius_m: float = 35.0
    #: Lagrangian parcels the slick is modelled with.
    parcels: int = 1500


@dataclass
class UiSettings:
    show_telemetry: bool = True
    show_drawer: bool = True
    drawer_page: int = 0
    show_vectors: bool = True
    show_hud: bool = True
    #: "natural" (water as it looks) or "chart" (depth shading).
    water_style: str = "natural"
    #: Chart frame rate: 60 for smooth, 30 to save battery.
    max_fps: int = 60
    show_fps: bool = False


@dataclass
class MapSettings:
    #: Offline OpenStreetMap index (``*.idx.json``).  Empty searches ``maps/``.
    vector_index: str = ""
    #: Depth grid (``.npz``, ``.asc``, ``.tif`` or ``.xyz``).  Empty uses the
    #: bundled Gulf of Riga grid.
    depth_grid: str = ""
    #: Display units for speed: ``kn`` or ``ms``.
    speed_units: str = "kn"


@dataclass
class Settings:
    boat: BoatSettings = field(default_factory=BoatSettings)
    sim: SimSettings = field(default_factory=SimSettings)
    control: ControlSettings = field(default_factory=ControlSettings)
    autopilot: AutopilotSettings = field(default_factory=AutopilotSettings)
    safety: SafetySettings = field(default_factory=SafetySettings)
    survey: SurveySettings = field(default_factory=SurveySettings)
    map: MapSettings = field(default_factory=MapSettings)
    skimmer: SkimmerSettings = field(default_factory=SkimmerSettings)
    spill: SpillSettings = field(default_factory=SpillSettings)
    ui: UiSettings = field(default_factory=UiSettings)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> Settings:
        return _fill(cls(), data)

    def validate(self) -> list[str]:
        """Problems that would make the station misbehave, for the log."""
        problems = []
        if self.safety.shallow_stop_m > self.safety.shallow_alarm_m:
            problems.append("shallow_stop_m is deeper than shallow_alarm_m")
        if self.safety.battery_critical_pct > self.safety.battery_low_pct:
            problems.append("battery_critical_pct is above battery_low_pct")
        if self.control.stick_mode not in ("arcade", "tank"):
            problems.append(f"unknown stick_mode {self.control.stick_mode!r}; using arcade")
            self.control.stick_mode = "arcade"
        if not 0.0 <= self.control.deadzone < 0.5:
            problems.append("deadzone must be between 0 and 0.5")
            self.control.deadzone = 0.08
        if self.sim.time_scale <= 0:
            problems.append("time_scale must be positive")
            self.sim.time_scale = 1.0
        return problems


def _fill(target, data: dict):
    """Copy ``data`` into the dataclass ``target``, recursively, keeping types."""
    if not isinstance(data, dict):
        return target
    for f in fields(target):
        if f.name not in data:
            continue
        current = getattr(target, f.name)
        value = data[f.name]
        if is_dataclass(current):
            _fill(current, value)
            continue
        try:
            if isinstance(current, bool):
                value = bool(value)
            elif isinstance(current, float):
                value = float(value)
            elif isinstance(current, int):
                value = int(value)
            elif isinstance(current, str):
                value = str(value)
        except (TypeError, ValueError):
            log.warning("ignoring setting %s=%r", f.name, value)
            continue
        setattr(target, f.name, value)
    return target


def default_path() -> Path:
    return config_dir() / "config.json"


def load(path: Path | None = None) -> Settings:
    path = path or default_path()
    try:
        return Settings.from_dict(json.loads(path.read_text(encoding="utf-8")))
    except FileNotFoundError:
        return Settings()
    except (OSError, ValueError) as exc:
        log.warning("could not read %s (%s); using defaults", path, exc)
        return Settings()


def save(settings: Settings, path: Path | None = None) -> Path:
    path = path or default_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(settings.to_dict(), indent=2), encoding="utf-8")
    tmp.replace(path)
    return path
