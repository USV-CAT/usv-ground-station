"""From sticks to helm orders.

Two layouts, chosen on the Pad page of the side panel:

**arcade** (the default, like a driving game)
    left stick up/down or the triggers (R2 ahead, L2 astern) set the throttle;
    the right stick — or the left one sideways — steers.

**tank** (like the twin throttles of a real catamaran)
    the left stick is the port motor, the right stick the starboard one.

The button map is the same in both; the layout is chosen on the Pad page.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..vehicle.autopilot import mix, unmix
from .gamepad import PadState

BUTTON_HELP = (
    ("A", "Preprogram — run / resume the mission"),
    ("B", "USV STOP — all motors stop"),
    ("X", "Cycle Manual → Speed Hold → Course Hold"),
    ("Y", "Station hold here"),
    ("☰ hold", "Arm / disarm"),
    ("⧉ hold", "Return home"),
    ("⧉ tap", "Show / hide the side panel"),
    ("L1 / R1", "Zoom chart out / in"),
    ("D-pad ↑↓", "Speed setpoint ±0.1 m/s"),
    ("D-pad ←→", "Course setpoint ±5° (Course Hold) · else side panel tabs"),
    ("R3", "Chart view: north up → heading up → 3D chase camera"),
    ("L3", "Deploy / stow the oil skimmer"),
)


@dataclass(frozen=True)
class Helm:
    port: float
    stbd: float
    throttle: float
    steer: float
    #: The largest stick or trigger deflection, for the manual override.
    deflection: float


def shape(value: float, deadzone: float, expo: float) -> float:
    """Dead zone, rescaled so the output still reaches ±1, then expo."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    magnitude = min(1.0, (magnitude - deadzone) / (1.0 - deadzone))
    magnitude = (1.0 - expo) * magnitude + expo * magnitude**3
    return magnitude if value > 0 else -magnitude


def helm_from_pad(pad: PadState, stick_mode: str, deadzone: float, expo: float) -> Helm:
    if not pad.connected:
        return Helm(0.0, 0.0, 0.0, 0.0, 0.0)

    def s(value: float) -> float:
        return shape(value, deadzone, expo)

    lx, ly, rx, ry = s(pad.lx), s(pad.ly), s(pad.rx), s(pad.ry)
    lt = pad.lt if pad.lt > deadzone else 0.0
    rt = pad.rt if pad.rt > deadzone else 0.0
    deflection = max(abs(lx), abs(ly), abs(rx), abs(ry), lt, rt)

    if stick_mode == "tank":
        port, stbd = ly, ry
        throttle, steer = unmix(port, stbd)
        return Helm(port, stbd, throttle, steer, deflection)

    throttle = max(-1.0, min(1.0, ly + rt - lt))
    steer = rx if abs(rx) >= abs(lx) else lx
    port, stbd = mix(throttle, steer)
    return Helm(port, stbd, throttle, steer, deflection)


def helm_from_keys(keys: set[str], stick_mode: str) -> Helm:
    """WASD or the arrow keys, for a laptop with no pad."""
    ahead = ("W" in keys or "Up" in keys) - ("S" in keys or "Down" in keys)
    turn = ("D" in keys or "Right" in keys) - ("A" in keys or "Left" in keys)
    throttle = 0.8 * ahead * (1.25 if "Shift" in keys else 1.0)
    steer = 0.35 * turn
    if not ahead and not turn:
        return Helm(0.0, 0.0, 0.0, 0.0, 0.0)
    port, stbd = mix(throttle, steer)
    return Helm(port, stbd, throttle, steer, max(abs(throttle), abs(steer)))
