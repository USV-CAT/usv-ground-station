"""Gamepads, and the Steam Deck's built-in controls in particular.

Two backends, tried in order:

* **SDL** through ``pygame`` (``pip install pygame-ce``).  SDL knows the button
  layout of practically every pad, including the Steam Deck and the virtual
  pad Steam Input creates, so it is the one to prefer when it is installed.
* **evdev**, read straight from ``/dev/input`` with nothing but the standard
  library.  It covers the Deck's own controls (the ``hid-steam`` driver) and
  Steam's virtual Xbox pad.

Either way a backend runs on its own thread and is asked for a
:class:`PadState` snapshot; button presses also arrive as edges, so a tap
that comes and goes between two polls is not lost.
"""

from __future__ import annotations

import contextlib
import fcntl
import logging
import os
import queue
import select
import struct
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger(__name__)

BUTTONS = (
    "A", "B", "X", "Y", "LB", "RB", "VIEW", "MENU", "GUIDE", "L3", "R3",
    "UP", "DOWN", "LEFT", "RIGHT", "L4", "R4", "L5", "R5",
)


@dataclass
class PadState:
    connected: bool = False
    name: str = ""
    backend: str = ""
    #: Sticks in [-1, 1].  Y is positive *up*, unlike the raw devices.
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    #: Triggers in [0, 1].
    lt: float = 0.0
    rt: float = 0.0
    held: frozenset[str] = field(default_factory=frozenset)


class Backend:
    name = "none"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lt": 0.0, "rt": 0.0}
        self._held: set[str] = set()
        self._connected = False
        self._device = ""
        self.edges: queue.Queue[tuple[str, bool]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name=f"pad-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def state(self) -> PadState:
        with self._lock:
            return PadState(self._connected, self._device, self.name, held=frozenset(self._held),
                            **self._axes)

    def _set_axis(self, axis: str, value: float) -> None:
        with self._lock:
            self._axes[axis] = value

    def _set_button(self, button: str, pressed: bool) -> None:
        with self._lock:
            was = button in self._held
            if pressed:
                self._held.add(button)
            else:
                self._held.discard(button)
        if was != pressed:
            self.edges.put((button, pressed))

    def _disconnected(self) -> None:
        with self._lock:
            self._connected = False
            for axis in self._axes:
                self._axes[axis] = 0.0
            released = list(self._held)
            self._held.clear()
        for button in released:
            self.edges.put((button, False))

    def _run(self) -> None:  # pragma: no cover - overridden
        pass


# --------------------------------------------------------------------------
# evdev
# --------------------------------------------------------------------------

EV_KEY, EV_ABS = 0x01, 0x03
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0, 1, 2, 3, 4, 5
ABS_HAT0X, ABS_HAT0Y, ABS_HAT2X, ABS_HAT2Y = 16, 17, 20, 21
BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST = 304, 305, 307, 308
BTN_GAMEPAD = BTN_SOUTH

#: Linux's gamepad convention (the ``hid-steam`` Deck driver follows it):
#: north is the top face button, west the left one.
KEYS_STANDARD = {
    304: "A", 305: "B", 307: "Y", 308: "X", 310: "LB", 311: "RB",
    314: "VIEW", 315: "MENU", 316: "GUIDE", 317: "L3", 318: "R3",
    544: "UP", 545: "DOWN", 546: "LEFT", 547: "RIGHT",
    704: "L4", 705: "R4", 706: "L5", 707: "R5",
}
#: The ``xpad`` driver, and Steam's virtual pad that imitates it, send BTN_X
#: (== north) for the X button.
KEYS_XPAD = {**KEYS_STANDARD, 307: "X", 308: "Y"}

EVENT = struct.Struct("llHHi")


def _ioc(direction: int, number: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord("E") << 8) | number


def _eviocgname(fd: int) -> str:
    buffer = bytearray(256)
    fcntl.ioctl(fd, _ioc(2, 0x06, len(buffer)), buffer)
    return buffer.split(b"\0", 1)[0].decode(errors="replace")


def _eviocgbit(fd: int, event_type: int, count: int) -> set[int]:
    buffer = bytearray((count + 7) // 8)
    fcntl.ioctl(fd, _ioc(2, 0x20 + event_type, len(buffer)), buffer)
    return {i for i in range(count) if buffer[i // 8] & (1 << (i % 8))}


def _eviocgabs(fd: int, axis: int) -> tuple[int, int, int]:
    buffer = bytearray(24)
    fcntl.ioctl(fd, _ioc(2, 0x40 + axis, len(buffer)), buffer)
    value, minimum, maximum, _fuzz, _flat, _res = struct.unpack("6i", buffer)
    return value, minimum, maximum


def find_evdev_pads() -> list[tuple[str, str]]:
    """``(path, name)`` of every readable gamepad, the likeliest first."""
    found = []
    try:
        entries = sorted(os.listdir("/dev/input"))
    except OSError:
        return found
    for entry in entries:
        if not entry.startswith("event"):
            continue
        path = f"/dev/input/{entry}"
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            keys = _eviocgbit(fd, EV_KEY, 768)
            axes = _eviocgbit(fd, EV_ABS, 64)
            if BTN_GAMEPAD in keys and ABS_X in axes:
                found.append((path, _eviocgname(fd)))
        except OSError:
            pass
        finally:
            os.close(fd)

    def rank(item: tuple[str, str]) -> int:
        name = item[1].lower()
        if "steam deck" in name:
            return 0
        if "x-box" in name or "xbox" in name or "steam" in name:
            return 1
        if "motion" in name or "accel" in name:
            return 9
        return 5

    return sorted(found, key=rank)


class EvdevBackend(Backend):
    name = "evdev"

    def _run(self) -> None:
        while not self._stop.is_set():
            pads = find_evdev_pads()
            if not pads:
                self._stop.wait(2.0)
                continue
            path, name = pads[0]
            try:
                self._read(path, name)
            except OSError as exc:
                log.info("gamepad %s went away: %s", name, exc)
            self._disconnected()
            self._stop.wait(1.0)

    def _read(self, path: str, name: str) -> None:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            lowered = name.lower()
            keys = KEYS_XPAD if ("x-box" in lowered or "xbox" in lowered or "xpad" in lowered) \
                else KEYS_STANDARD
            available = _eviocgbit(fd, EV_ABS, 64)
            ranges = {axis: _eviocgabs(fd, axis)[1:] for axis in available}
            # Triggers: the Xbox layout puts them on Z/RZ; the Deck's own driver
            # on HAT2Y (left) and HAT2X (right).
            if ABS_HAT2X in available and ABS_RZ not in available:
                trigger_axes = {ABS_HAT2Y: "lt", ABS_HAT2X: "rt"}
            else:
                trigger_axes = {ABS_Z: "lt", ABS_RZ: "rt"}
            sticks = {ABS_X: ("lx", 1), ABS_Y: ("ly", -1), ABS_RX: ("rx", 1), ABS_RY: ("ry", -1)}
            with self._lock:
                self._connected = True
                self._device = name
            log.info("gamepad: %s (%s)", name, path)
            while not self._stop.is_set():
                ready, _, _ = select.select([fd], [], [], 0.25)
                if not ready:
                    continue
                data = os.read(fd, EVENT.size * 64)
                for offset in range(0, len(data) - EVENT.size + 1, EVENT.size):
                    _, _, kind, code, value = EVENT.unpack_from(data, offset)
                    if kind == EV_KEY and code in keys:
                        self._set_button(keys[code], value != 0)
                    elif kind == EV_ABS:
                        low, high = ranges.get(code, (-1, 1))
                        if code in sticks:
                            axis, sign = sticks[code]
                            centre = (low + high) / 2.0
                            half = max(1.0, (high - low) / 2.0)
                            self._set_axis(axis, max(-1.0, min(1.0, sign * (value - centre) / half)))
                        elif code in trigger_axes:
                            span = max(1.0, high - low)
                            self._set_axis(trigger_axes[code], max(0.0, min(1.0, (value - low) / span)))
                        elif code == ABS_HAT0X:
                            self._set_button("LEFT", value < 0)
                            self._set_button("RIGHT", value > 0)
                        elif code == ABS_HAT0Y:
                            self._set_button("UP", value < 0)
                            self._set_button("DOWN", value > 0)
        finally:
            os.close(fd)


# --------------------------------------------------------------------------
# SDL
# --------------------------------------------------------------------------


class SdlBackend(Backend):
    name = "sdl"

    def __init__(self) -> None:
        super().__init__()
        # The ground station's window has the focus, not SDL's (it has none),
        # so SDL must be told to keep reporting the pad regardless.
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame  # noqa: F401  (fails here, not in the thread, if missing)
        from pygame._sdl2 import controller  # noqa: F401

    def _run(self) -> None:
        import pygame
        from pygame._sdl2 import controller

        pygame.init()
        controller.init()
        axis_map = {
            pygame.CONTROLLER_AXIS_LEFTX: ("lx", 1), pygame.CONTROLLER_AXIS_LEFTY: ("ly", -1),
            pygame.CONTROLLER_AXIS_RIGHTX: ("rx", 1), pygame.CONTROLLER_AXIS_RIGHTY: ("ry", -1),
            pygame.CONTROLLER_AXIS_TRIGGERLEFT: ("lt", 0),
            pygame.CONTROLLER_AXIS_TRIGGERRIGHT: ("rt", 0),
        }
        button_map = {
            pygame.CONTROLLER_BUTTON_A: "A", pygame.CONTROLLER_BUTTON_B: "B",
            pygame.CONTROLLER_BUTTON_X: "X", pygame.CONTROLLER_BUTTON_Y: "Y",
            pygame.CONTROLLER_BUTTON_LEFTSHOULDER: "LB",
            pygame.CONTROLLER_BUTTON_RIGHTSHOULDER: "RB",
            pygame.CONTROLLER_BUTTON_BACK: "VIEW", pygame.CONTROLLER_BUTTON_START: "MENU",
            pygame.CONTROLLER_BUTTON_GUIDE: "GUIDE",
            pygame.CONTROLLER_BUTTON_LEFTSTICK: "L3", pygame.CONTROLLER_BUTTON_RIGHTSTICK: "R3",
            pygame.CONTROLLER_BUTTON_DPAD_UP: "UP", pygame.CONTROLLER_BUTTON_DPAD_DOWN: "DOWN",
            pygame.CONTROLLER_BUTTON_DPAD_LEFT: "LEFT",
            pygame.CONTROLLER_BUTTON_DPAD_RIGHT: "RIGHT",
        }
        pad = None
        while not self._stop.is_set():
            pygame.event.pump()
            if pad is None or not pad.attached():
                if pad is not None:
                    self._disconnected()
                    pad = None
                count = controller.get_count()
                for index in range(count):
                    if controller.is_controller(index):
                        pad = controller.Controller(index)
                        with self._lock:
                            self._connected = True
                            self._device = pad.name
                        log.info("gamepad (SDL): %s", pad.name)
                        break
                if pad is None:
                    self._stop.wait(1.0)
                    continue
            for axis, (name, sign) in axis_map.items():
                raw = pad.get_axis(axis)
                value = raw / 32767.0 if sign == 0 else sign * raw / 32767.0
                self._set_axis(name, max(-1.0, min(1.0, value)))
            for button, name in button_map.items():
                self._set_button(name, bool(pad.get_button(button)))
            time.sleep(0.01)


def open_backend(choice: str = "auto") -> Backend | None:
    """Start the requested input backend, or the best available for ``auto``."""
    order = {"auto": ["sdl", "evdev"], "sdl": ["sdl"], "pygame": ["sdl"], "evdev": ["evdev"]}
    for name in order.get(choice, []):
        try:
            backend = SdlBackend() if name == "sdl" else EvdevBackend()
        except ImportError:
            log.info("SDL input unavailable (pygame not installed); trying the next backend")
            continue
        with contextlib.suppress(Exception):
            backend.start()
            return backend
    return None
