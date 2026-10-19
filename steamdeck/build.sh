#!/usr/bin/env bash
#
# Build bin/usvgs inside a Debian 12 distrobox, without Rust on the host.
#
#     ./steamdeck/build.sh
#
# Works on the Steam Deck (SteamOS ships distrobox and podman) and on any
# Linux PC with distrobox. Debian 12 has an older glibc (2.36) than SteamOS,
# so the binary built here runs on the Deck as well as on newer desktops.
#
# The Rust toolchain is kept apart from the host's (~/.local/share/usvgs-rust),
# so it never puts a `cargo` on the host PATH that can't link there.

set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
BOX="usvgs-build"
IMAGE="docker.io/library/debian:12"
TOOLS="${XDG_DATA_HOME:-$HOME/.local/share}/usvgs-rust"

if ! command -v distrobox >/dev/null; then
  echo "!! distrobox not found (it ships with SteamOS 3.5+; elsewhere: https://distrobox.it)" >&2
  exit 1
fi

if [ -e /run/.containerenv ] || [ -e /.flatpak-info ] || [ -n "${container:-}" ]; then
  echo "!! this terminal is inside a container (distrobox, toolbox or a Flatpak app)." >&2
  echo "   Open a normal Konsole on the host and run this again." >&2
  exit 1
fi

# After a reboot or a killed container, rootless podman can be left pointing
# at a pause process that no longer exists ("cannot join/set mount namespace").
# Clear the stale pause process and let podman start a fresh one.
if command -v podman >/dev/null && ! podman ps >/dev/null 2>&1; then
  echo "[*] podman is stuck, resetting it…"
  RUN="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  if [ -f "$RUN/libpod/tmp/pause.pid" ]; then
    kill "$(cat "$RUN/libpod/tmp/pause.pid")" 2>/dev/null || true
    rm -f "$RUN/libpod/tmp/pause.pid"
  fi
  podman system migrate || true
  if ! podman ps >/dev/null; then
    echo "!! podman still fails; reboot the Deck and run this again." >&2
    exit 1
  fi
fi

if ! distrobox list 2>/dev/null | awk -F'|' '{gsub(/ /,"",$2); print $2}' | grep -qx "$BOX"; then
  echo "[*] creating the $BOX container ($IMAGE)…"
  distrobox create --yes --no-entry --name "$BOX" --image "$IMAGE"
fi

echo "[*] building in $BOX (first time: a few minutes)…"
distrobox enter "$BOX" -- bash -euo pipefail -c '
  export CARGO_HOME="'"$TOOLS"'/cargo" RUSTUP_HOME="'"$TOOLS"'/rustup"
  export PATH="$CARGO_HOME/bin:$PATH"
  if ! dpkg -s libasound2-dev libudev-dev pkg-config build-essential >/dev/null 2>&1; then
    sudo apt-get update -qq
    sudo apt-get install -y -qq build-essential pkg-config libasound2-dev libudev-dev curl ca-certificates >/dev/null
  fi
  if ! command -v rustup >/dev/null; then
    curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --no-modify-path
  fi
  # A half-finished earlier install can leave rustup with no default toolchain.
  if ! cargo --version >/dev/null 2>&1; then
    rustup toolchain install stable --profile minimal
    rustup default stable
  fi
  cd "'"$HERE"'"
  CARGO_TARGET_DIR=target/distrobox cargo build --release --locked
'

mkdir -p "$HERE/bin"
cp "$HERE/target/distrobox/release/usvgs" "$HERE/bin/usvgs"
strip "$HERE/bin/usvgs" 2>/dev/null || true
echo "[*] built $HERE/bin/usvgs"
