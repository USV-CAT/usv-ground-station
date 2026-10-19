#!/usr/bin/env bash
#
# Start the ground station.
#
#   ./run.sh               the station, with the simulated catamaran
#   ./run.sh --windowed    not full screen
#   ./run.sh --view chase  start in the 3D chase camera
#   ./run.sh pads          test the gamepad / Steam Deck controls
#
# With Rust installed it builds from source (the first build takes a few
# minutes, later runs rebuild only what changed). Without Rust it runs the
# prebuilt bin/usvgs, and if that is missing it builds one in a distrobox
# (see steamdeck/build.sh).

set -euo pipefail
cd "$(dirname "$0")"

if command -v cargo >/dev/null; then
  cargo build --release --quiet
  exec target/release/usvgs "$@"
fi

PREBUILT="bin/usvgs"
[ -f "$PREBUILT" ] && [ ! -x "$PREBUILT" ] && chmod +x "$PREBUILT"
if [ ! -f "$PREBUILT" ]; then
  if command -v distrobox >/dev/null; then
    ./steamdeck/build.sh >&2
  else
    echo "!! no Rust (https://rustup.rs), no prebuilt $PREBUILT and no distrobox to build one" >&2
    exit 1
  fi
fi
exec "$PREBUILT" "$@"
