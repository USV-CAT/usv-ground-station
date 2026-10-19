#!/usr/bin/env bash
#
# Start the ground station.
#
#   ./run.sh               the station, with the simulated catamaran
#   ./run.sh --windowed    not full screen
#   ./run.sh pads          test the gamepad / Steam Deck controls
#
# The first run builds a virtual environment in .venv (a minute or two); after
# that it starts straight away.  Set NO_SDL=1 to skip installing pygame-ce.

set -euo pipefail
cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3}"
VENV=".venv"
STAMP="$VENV/.installed"

if ! "$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
  echo "!! needs Python 3.10 or newer (set PYTHON=...)" >&2
  exit 1
fi

if [ ! -x "$VENV/bin/python" ]; then
  echo "[*] first run: building $VENV…" >&2
  # System packages are inherited so a distribution PyQt5 is reused.
  "$PYTHON" -m venv --system-site-packages "$VENV"
fi
PY="$VENV/bin/python"

if [ ! -f "$STAMP" ] || [ pyproject.toml -nt "$STAMP" ]; then
  echo "[*] installing usvgs…" >&2
  "$PY" -m pip install --quiet --upgrade pip >/dev/null 2>&1 || true
  "$PY" -m pip install --quiet -e . || { echo "!! install failed" >&2; exit 1; }
  if [ "${NO_SDL:-0}" != "1" ]; then
    "$PY" -m pip install --quiet "pygame-ce>=2.4" || \
      echo "!! pygame-ce did not install; gamepads will be read through evdev instead" >&2
  fi
  touch "$STAMP"
fi

exec "$PY" -m usvgs "$@"
