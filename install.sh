#!/usr/bin/env bash
#
# Install the ground station on a Steam Deck (or any Linux desktop).
#
# Run once from Desktop Mode, in Konsole:
#
#     ./install.sh            use the prebuilt bin/usvgs (or Rust, if installed)
#     ./install.sh --build    rebuild bin/usvgs in a distrobox first
#
# It checks the station starts, adds "USV Ground Station" to the application
# menu, and tells you how to add it to Steam so it starts from Game Mode with
# the Deck's controls.

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ "${1:-}" = "--build" ]; then
  "$HERE/steamdeck/build.sh"
fi

chmod +x "$HERE/run.sh" "$HERE/steamdeck/build.sh" "$HERE/bin/usvgs" 2>/dev/null || true

echo "[*] checking the station starts…"
if ! "$HERE/run.sh" --version; then
  echo "!! the station did not start; try: ./install.sh --build" >&2
  exit 1
fi

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
