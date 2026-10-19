"""Steering and speed control for a differential-thrust boat.

Pure arithmetic with no threads and no I/O, so every piece can be tested on its
own.  The pieces are the usual ones:

* a heading controller (PID, with the rate of turn as the derivative term, so
  it damps on what the gyro measures rather than on a noisy differentiated error);
* a speed controller (feed-forward plus PI on speed over ground);
* line-of-sight guidance along a leg, which steers for a point a fixed distance
  ahead on the line and so converges onto the line instead of chasing the
  waypoint;
* a crab-angle correction, so a cross-current does not leave the boat tracking
  parallel to its line;
* a mixer from (throttle, steer) to the two motors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import geo
from ..core.geo import LatLon
from ..core.mission import cross_track

#: Speed at full throttle, m/s, for the speed feed-forward.
TOP_SPEED_MS = 2.37
#: Heading error beyond which the boat stops and pivots rather than arcing.
PIVOT_ERROR_DEG = 70.0


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def mix(throttle: float, steer: float) -> tuple[float, float]:
    """(throttle, steer) to (port, starboard).  Positive steer turns to starboard.

    When the sum would saturate a motor, throttle gives way and the steering is
    kept, because a boat that cannot turn is worse than one that slows down.
    """
    throttle, steer = clamp(throttle), clamp(steer)
    headroom = 1.0 - abs(steer)
    throttle = clamp(throttle, -headroom, headroom)
    return clamp(throttle + steer), clamp(throttle - steer)


def unmix(port: float, stbd: float) -> tuple[float, float]:
    return (port + stbd) / 2.0, (port - stbd) / 2.0


@dataclass
class HeadingController:
    kp: float = 0.022       # steer per degree of error
    kd: float = 0.012       # steer per degree/second of turn
    ki: float = 0.0015      # steer per degree-second
    integral_limit: float = 0.25
    _integral: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def update(self, target_deg: float, heading_deg: float, rot_dps: float, dt: float) -> float:
        error = geo.angle_difference(target_deg, heading_deg)
        if abs(error) < 25.0:
            self._integral = clamp(self._integral + error * dt * self.ki,
                                   -self.integral_limit, self.integral_limit)
        else:
            self._integral *= 0.95
        return clamp(self.kp * error - self.kd * rot_dps + self._integral)


@dataclass
class SpeedController:
    kp: float = 0.35
    ki: float = 0.12
    _integral: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def update(self, target_ms: float, speed_ms: float, dt: float) -> float:
        feed_forward = math.copysign(math.sqrt(abs(target_ms) / TOP_SPEED_MS), target_ms) * 0.95
        error = target_ms - speed_ms
        self._integral = clamp(self._integral + error * dt * self.ki, -0.3, 0.3)
        return clamp(feed_forward + self.kp * error + self._integral)


@dataclass
class LegGuidance:
    """Where to steer on the leg ``start`` → ``end``."""

    course_deg: float
    cross_track_m: float
    along_m: float
    leg_m: float
    distance_to_go_m: float

    @property
    def passed_end(self) -> bool:
        return self.along_m >= self.leg_m


def line_of_sight(start: LatLon, end: LatLon, position: LatLon, lookahead_m: float) -> LegGuidance:
    along, across, length = cross_track(start, end, position)
    distance = geo.distance_m(position, end)
    if length < 1.0:
        course = geo.bearing_deg(position, end)
    else:
        leg_course = geo.bearing_deg(start, end)
        course = (leg_course - math.degrees(math.atan2(across, max(1.0, lookahead_m)))) % 360.0
        # Close to the end, aim at it directly so the boat does not orbit it.
        if distance < lookahead_m:
            course = geo.bearing_deg(position, end)
    return LegGuidance(course, across, along, length, distance)


class CrabCorrection:
    """Learns the angle between heading and track, so the bow can be held into a set."""

    def __init__(self, limit_deg: float = 35.0, rate: float = 0.08) -> None:
        self.limit = limit_deg
        self.rate = rate
        self.angle = 0.0

    def reset(self) -> None:
        self.angle = 0.0

    def update(self, heading_deg: float, cog_deg: float | None, sog_ms: float | None,
               dt: float) -> float:
        if cog_deg is not None and sog_ms is not None and sog_ms > 0.4:
            measured = geo.angle_difference(cog_deg, heading_deg)
            alpha = min(1.0, self.rate * dt)
            self.angle += (measured - self.angle) * alpha
            self.angle = clamp(self.angle, -self.limit, self.limit)
        return self.angle


class Pilot:
    """Turns "go this way at this speed" into motor commands."""

    def __init__(self) -> None:
        self.heading = HeadingController()
        self.speed = SpeedController()
        self.crab = CrabCorrection()

    def reset(self) -> None:
        self.heading.reset()
        self.speed.reset()
        self.crab.reset()

    def steer_course(self, course_deg: float, speed_ms: float, heading_deg: float,
                     rot_dps: float, cog_deg: float | None, sog_ms: float, dt: float,
                     track_over_ground: bool = True) -> tuple[float, float]:
        """Motor levels to make good ``course_deg`` at ``speed_ms``."""
        crab = self.crab.update(heading_deg, cog_deg, sog_ms, dt) if track_over_ground else 0.0
        target_heading = (course_deg - crab) % 360.0
        steer = self.heading.update(target_heading, heading_deg, rot_dps, dt)
        error = abs(geo.angle_difference(target_heading, heading_deg))
        if error > PIVOT_ERROR_DEG:
            # Way off: stop and pivot, which a twin-motor catamaran does well.
            throttle = 0.0
            self.speed.reset()
        else:
            throttle = self.speed.update(speed_ms, sog_ms, dt)
            # Ease off through a turn so it is tight rather than wide.
            throttle *= max(0.35, 1.0 - error / PIVOT_ERROR_DEG)
        return mix(throttle, steer)


@dataclass
class StationKeeper:
    """Hold a point: drive back to it when blown outside a radius, else drift."""

    radius_m: float = 3.0
    _correcting: bool = False

    def update(self, pilot: Pilot, target: LatLon, position: LatLon, heading_deg: float,
               rot_dps: float, cog_deg: float | None, sog_ms: float,
               cruise_ms: float, dt: float) -> tuple[float, float]:
        distance = geo.distance_m(position, target)
        if distance > self.radius_m:
            self._correcting = True
        elif distance < self.radius_m * 0.35:
            self._correcting = False
        if not self._correcting:
            pilot.speed.reset()
            return 0.0, 0.0
        speed = min(cruise_ms, max(0.3, 0.2 * distance))
        course = geo.bearing_deg(position, target)
        return pilot.steer_course(course, speed, heading_deg, rot_dps, cog_deg, sog_ms, dt,
                                  track_over_ground=distance > 10.0)
