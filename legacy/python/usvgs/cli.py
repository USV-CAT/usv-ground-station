"""Command line.

    usvgs                 the ground station (simulated boat)
    usvgs --windowed      not full screen, even on a small display
    usvgs pads            list gamepads and show live stick values (test the Deck's controls)
    usvgs chart ...       build an offline depth chart (see usvgs.tools.bathy_build)
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from . import __version__, config


def _pads(seconds: float) -> int:
    from .input import gamepad

    found = gamepad.find_evdev_pads()
    print("evdev gamepads:" if found else "no evdev gamepads readable (check /dev/input permissions)")
    for path, name in found:
        print(f"  {path}  {name}")
    backend = gamepad.open_backend("auto")
    if backend is None:
        print("no input backend could start")
        return 1
    print(f"reading with the {backend.name} backend for {seconds:.0f} s — move the sticks, press buttons")
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            s = backend.state()
            held = " ".join(sorted(s.held))
            sys.stdout.write(
                f"\r{'●' if s.connected else '○'} L({s.lx:+.2f},{s.ly:+.2f}) R({s.rx:+.2f},{s.ry:+.2f}) "
                f"LT {s.lt:.2f} RT {s.rt:.2f}  {held:<30}")
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        backend.stop()
        print()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="usvgs", description="Portable USV ground station")
    parser.add_argument("--version", action="version", version=f"usvgs {__version__}")
    parser.add_argument("--windowed", action="store_true", help="never go full screen")
    parser.add_argument("--fullscreen", action="store_true", help="always go full screen")
    parser.add_argument("--reset-settings", action="store_true", help="start from default settings")
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command")
    pads = sub.add_parser("pads", help="list gamepads and show their live state")
    pads.add_argument("--seconds", type=float, default=30.0)
    sub.add_parser("chart", help="build an offline depth chart", add_help=False)

    args, rest = parser.parse_known_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.command == "pads":
        return _pads(args.seconds)
    if args.command == "chart":
        from .tools import bathy_build
        return bathy_build.main(rest)
    if rest:
        parser.error(f"unrecognised arguments: {' '.join(rest)}")

    settings = config.Settings() if args.reset_settings else config.load()
    try:
        from .ui.app import run
    except ImportError as exc:
        print(f"The graphical station needs PyQt5 and numpy ({exc}).", file=sys.stderr)
        return 1
    fullscreen = True if args.fullscreen else (False if args.windowed else None)
    return run(settings, fullscreen)


if __name__ == "__main__":
    raise SystemExit(main())
