#!/usr/bin/env bash
#
# Start the ground station.
#
#   ./run.sh               the station, with the simulated catamaran
#   ./run.sh --windowed    not full screen
#   ./run.sh --view chase  start in the 3D chase camera
#   ./run.sh pads          test the gamepad / Steam Deck controls
#
# The first run compiles the release build (a few minutes); after that it
# starts straight away, and rebuilds only when the sources change.

set -euo pipefail
cd "$(dirname "$0")"

BIN="target/release/usvgs"
if ! command -v cargo >/dev/null; then
  if [ -x "$BIN" ]; then exec "$BIN" "$@"; fi
  echo "!! needs Rust to build (https://rustup.rs), or a prebuilt $BIN" >&2
  exit 1
fi
cargo build --release --quiet
exec "$BIN" "$@"
