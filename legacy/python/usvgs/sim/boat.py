"""A twin-hull, twin-motor catamaran in three degrees of freedom.

The boat has no rudder: it steers by running its two motors at different
speeds, the way most small survey catamarans do, which also lets it pivot on
the spot.  The model is the standard surge–sway–yaw manoeuvring model with
linear and quadratic damping, integrated relative to the water so a current
carries the boat without it having to push against it::

    m_x (u' - v r) = T_port + T_stbd - X_u u - X_uu |u| u + F_wind,x
    m_y (v' + u r) =                 - Y_v v - Y_vv |v| v + F_wind,y
    I_z r'         = (T_port - T_stbd) B/2 - N_r r - N_rr |r| r + N_wind

``u`` is forward and ``v`` to starboard through the water, ``r`` the yaw rate,
positive turning to starboard.  The coefficients are picked for a 1.6 m, 55 kg
boat with two 38 N thrusters: about 4.6 kn flat out, and a pivot of about 40°/s.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import BoatSettings
from ..core import geo
from ..core.geo import LatLon, Metres
from .world import World

AIR_DENSITY = 1.2
#: Fraction of forward thrust a propeller gives running backwards.
REVERSE_EFFICIENCY = 0.55
#: Motor and controller response, seconds.
MOTOR_TAU_S = 0.25
MAX_RPM = 3200.0


@dataclass
class BoatState:
    north: float = 0.0          # metres from the origin
    east: float = 0.0
    heading: float = 0.0        # radians, clockwise from north
    u: float = 0.0              # surge through the water, m/s
    v: float = 0.0              # sway through the water, m/s
    r: float = 0.0              # yaw rate, rad/s
    motor_port: float = 0.0     # -1..1, what the motor is actually doing
    motor_stbd: float = 0.0
    battery_soc: float = 1.0
    battery_current_a: float = 0.0
    grounded: bool = False
    distance_m: float = 0.0     # odometer, over the ground
    groundings: int = 0
    #: Payload draw on the battery (the skimmer's pump), amperes.
    extra_load_a: float = 0.0
    t: float = 0.0


class Catamaran:
    def __init__(self, settings: BoatSettings, world: World, start: LatLon,
                 heading_deg: float = 0.0) -> None:
        self.settings = settings
        self.world = world
        self.origin = world.origin
        self.state = BoatState()
        self.reset(start, heading_deg)

        m = settings.mass_kg
        self.mx = m * 1.1
        self.my = m * 1.6
        self.iz = 25.0 * (m / 55.0)
        top_speed = 2.37
        self.xu = 6.0
        self.xuu = (2 * settings.max_thrust_n - self.xu * top_speed) / top_speed**2
        self.yv, self.yvv = 30.0, 60.0
        self.nr, self.nrr = 25.0, 40.0
        #: Ground velocity (north, east) from the last step, for GNSS.
        self.ground_velocity = (0.0, 0.0)
        #: Air velocity relative to the boat, body frame, for the anemometer.
        self.relative_air = (0.0, 0.0)

    def reset(self, start: LatLon, heading_deg: float = 0.0) -> None:
        offset = geo.to_metres(self.origin, start)
        soc = self.state.battery_soc if self.state.t else 1.0
        self.state = BoatState(north=offset.north, east=offset.east,
                               heading=math.radians(heading_deg), battery_soc=soc)

    @property
    def position(self) -> LatLon:
        return geo.to_lat_lon(self.origin, Metres(self.state.east, self.state.north))

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.state.heading) % 360.0

    def recharge(self) -> None:
        self.state.battery_soc = 1.0

    # -- the battery --------------------------------------------------------

    @property
    def battery_voltage(self) -> float:
        """4S Li-ion: 12.0 V empty, 16.6 V full, less the sag under load."""
        soc = self.state.battery_soc
        open_circuit = 12.0 + 4.6 * (0.15 * soc + 0.85 * soc**0.6) if soc > 0 else 12.0
        return max(10.0, open_circuit - 0.03 * self.state.battery_current_a)

    # -- one step -----------------------------------------------------------

    def step(self, dt: float, command_port: float, command_stbd: float) -> None:
        s = self.state
        p = self.settings
        s.t += dt

        if s.battery_soc <= 0.0:
            command_port = command_stbd = 0.0

        # Motors chase their commands.
        alpha = min(1.0, dt / MOTOR_TAU_S)
        s.motor_port += (max(-1.0, min(1.0, command_port)) - s.motor_port) * alpha
        s.motor_stbd += (max(-1.0, min(1.0, command_stbd)) - s.motor_stbd) * alpha

        def thrust(level: float) -> float:
            return level * p.max_thrust_n * (1.0 if level >= 0 else REVERSE_EFFICIENCY)

        t_port, t_stbd = thrust(s.motor_port), thrust(s.motor_stbd)

        # Wind on the superstructure, from the air's velocity relative to the
        # boat, in the body frame.
        ch, sh = math.cos(s.heading), math.sin(s.heading)
        wn, we = self.world.wind_vector()
        cn, ce = self.world.current_vector()
        gn = s.u * ch - s.v * sh + cn
        ge = s.u * sh + s.v * ch + ce
        rel_n, rel_e = wn - gn, we - ge
        air_x = rel_n * ch + rel_e * sh
        air_y = -rel_n * sh + rel_e * ch
        air = math.hypot(air_x, air_y)
        self.relative_air = (air_x, air_y)
        fx = 0.5 * AIR_DENSITY * 0.9 * 0.15 * air * air_x
        fy = 0.5 * AIR_DENSITY * 1.0 * 0.35 * air * air_y
        # The superstructure sits a little aft, so the bow falls off the wind.
        nz = -0.12 * fy

        x = t_port + t_stbd - self.xu * s.u - self.xuu * abs(s.u) * s.u + fx
        y = -self.yv * s.v - self.yvv * abs(s.v) * s.v + fy
        n = (t_port - t_stbd) * p.beam_m / 2.0 - self.nr * s.r - self.nrr * abs(s.r) * s.r + nz

        s.u += (x / self.mx + s.v * s.r) * dt
        s.v += (y / self.my - s.u * s.r) * dt
        s.r += (n / self.iz) * dt
        s.heading = (s.heading + s.r * dt) % (2 * math.pi)

        ch, sh = math.cos(s.heading), math.sin(s.heading)
        vn = s.u * ch - s.v * sh + cn
        ve = s.u * sh + s.v * ch + ce
        new_north, new_east = s.north + vn * dt, s.east + ve * dt

        # Grounding: the boat will not move into water shallower than its draft
        # unless the move takes it deeper than where it already is, which is
        # what lets it back off a bank it has run onto.
        here = self._depth(s.north, s.east)
        there = self._depth(new_north, new_east)
        if there != there or there < p.draft_m:
            if not s.grounded:
                s.groundings += 1
            s.grounded = True
            if there == there and here == here and there > here:
                s.north, s.east = new_north, new_east
            else:
                s.u *= 0.2
                s.v *= 0.2
                s.r *= 0.5
                vn = ve = 0.0
        else:
            s.grounded = False
            s.distance_m += math.hypot(vn, ve) * dt
            s.north, s.east = new_north, new_east
        self.ground_velocity = (vn, ve)

        # Current draw: hotel load, payload, and the motors rising faster than linearly.
        s.battery_current_a = (0.6 + s.extra_load_a
                               + 18.0 * (abs(s.motor_port) ** 1.5 + abs(s.motor_stbd) ** 1.5))
        s.battery_soc = max(0.0, s.battery_soc - s.battery_current_a * dt / 3600.0 / p.battery_ah)

    def _depth(self, north: float, east: float) -> float:
        position = geo.to_lat_lon(self.origin, Metres(east, north))
        return self.world.depth_at(position.lat, position.lon)

    def depth_here(self) -> float:
        return self._depth(self.state.north, self.state.east)

    def attitude(self) -> tuple[float, float]:
        """Roll and pitch, degrees: swell from the wind, and heel in a turn."""
        s = self.state
        t = s.t
        sea = min(1.0, self.world.wind_speed_ms / 10.0)
        roll = sea * (2.5 * math.sin(t * 1.9) + 1.0 * math.sin(t * 3.1 + 1.0))
        roll += math.degrees(-0.06 * s.u * s.r)
        pitch = sea * (1.5 * math.sin(t * 1.4 + 0.5)) - 1.2 * s.u * (abs(s.motor_port) + abs(s.motor_stbd)) / 2
        return roll, pitch
