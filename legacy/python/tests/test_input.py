import pytest

from usvgs.input.gamepad import PadState
from usvgs.input.mapping import helm_from_keys, helm_from_pad, shape
from usvgs.vehicle.autopilot import mix


def test_deadzone_and_full_scale():
    assert shape(0.05, 0.08, 0.3) == 0.0
    assert shape(1.0, 0.08, 0.3) == pytest.approx(1.0)
    assert shape(-1.0, 0.08, 0.0) == pytest.approx(-1.0)
    assert 0 < shape(0.5, 0.08, 0.5) < shape(0.5, 0.08, 0.0)


def test_arcade_forward_and_turn():
    pad = PadState(connected=True, ly=1.0)
    helm = helm_from_pad(pad, "arcade", 0.08, 0.0)
    assert helm.port == pytest.approx(1.0) and helm.stbd == pytest.approx(1.0)
    pad = PadState(connected=True, rx=1.0)
    helm = helm_from_pad(pad, "arcade", 0.08, 0.0)
    assert helm.port > 0 > helm.stbd           # pivot to starboard


def test_triggers_drive_like_a_racing_game():
    helm = helm_from_pad(PadState(connected=True, rt=1.0), "arcade", 0.08, 0.0)
    assert helm.throttle == pytest.approx(1.0)
    helm = helm_from_pad(PadState(connected=True, lt=1.0), "arcade", 0.08, 0.0)
    assert helm.throttle == pytest.approx(-1.0)


def test_tank_sticks_are_the_motors():
    helm = helm_from_pad(PadState(connected=True, ly=0.6, ry=-0.6), "tank", 0.0, 0.0)
    assert helm.port == pytest.approx(0.6) and helm.stbd == pytest.approx(-0.6)
    assert helm.steer == pytest.approx(0.6)


def test_disconnected_pad_is_neutral():
    assert helm_from_pad(PadState(connected=False, ly=1.0), "arcade", 0.1, 0.0).deflection == 0


def test_mixer_keeps_steering_at_full_throttle():
    port, stbd = mix(1.0, 0.5)
    assert port - stbd == pytest.approx(1.0)
    assert max(abs(port), abs(stbd)) <= 1.0


def test_keyboard():
    helm = helm_from_keys({"W", "D"}, "arcade")
    assert helm.port > helm.stbd > 0
