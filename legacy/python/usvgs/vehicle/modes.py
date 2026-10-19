"""Control modes, named the way USV ground stations name them."""

from __future__ import annotations

import enum


class Mode(enum.Enum):
    #: Sticks straight to the motors.
    MANUAL = "manual"
    #: Speed held by the autopilot; the operator steers.
    SPEED_HOLD = "speed_hold"
    #: Speed and course both held; the operator nudges them.
    COURSE_HOLD = "course_hold"
    #: Go to one point on the chart, then keep station there.
    POINT_NAV = "point_nav"
    #: Run the planned mission.
    AUTO = "auto"
    #: Keep station where the boat is.
    HOLD = "hold"
    #: Return to the launch point, then keep station there.
    RETURN_HOME = "return_home"
    #: All stop: motors at zero, the boat drifts.
    STOP = "stop"

    @property
    def label(self) -> str:
        return LABELS[self]

    @property
    def autonomous(self) -> bool:
        return self in (Mode.POINT_NAV, Mode.AUTO, Mode.HOLD, Mode.RETURN_HOME)

    @property
    def operator_steers(self) -> bool:
        return self in (Mode.MANUAL, Mode.SPEED_HOLD)


LABELS = {
    Mode.MANUAL: "Manual",
    Mode.SPEED_HOLD: "Speed Hold",
    Mode.COURSE_HOLD: "Speed/Course Hold",
    Mode.POINT_NAV: "Point Nav",
    Mode.AUTO: "Preprogram",
    Mode.HOLD: "Station Hold",
    Mode.RETURN_HOME: "Return Home",
    Mode.STOP: "USV Stop",
}
