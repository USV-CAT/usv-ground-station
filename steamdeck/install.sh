#!/usr/bin/env bash
#
# Install the ground station on a Steam Deck (or any Linux desktop).
#
# Run once from Desktop Mode, in Konsole:
#
#     ./steamdeck/install.sh
#
# It builds the station (or uses target/release/usvgs copied over from a PC),
# adds "USV Ground Station" to the
# application menu, and tells you how to add it to Steam so it starts from
# Game Mode with the Deck's controls.

set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"

echo "[*] building the station (first time: a few minutes)…"
"$HERE/run.sh" --version

APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$APPS"
sed "s|@HERE@|$HERE|g" "$HERE/steamdeck/usvgs.desktop.in" > "$APPS/usvgs.desktop"
chmod +x "$APPS/usvgs.desktop"
command -v update-desktop-database >/dev/null && update-desktop-database "$APPS" || true

cat <<NOTE

  Installed.  "USV Ground Station" is in the application menu.

  To play it from Game Mode:
    1. Steam (Desktop Mode) ▸ Games ▸ Add a Non-Steam Game to My Library…
    2. Tick "USV Ground Station", Add Selected Programs.
    3. In its Properties ▸ Controller, choose the "Gamepad with Mouse Trackpad"
       layout: sticks and buttons drive the boat, the right trackpad is the
       mouse for the chart, and the touchscreen works as well.

  Test the controls without the window:   $HERE/run.sh pads
NOTE
