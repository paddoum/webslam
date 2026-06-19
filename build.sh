#!/usr/bin/env bash
# Build webslam to WebAssembly. Run from the project root.
set -euo pipefail

# --- toolchain paths (edit if yours differ) ---
EMSDK_DIR="${EMSDK_DIR:-$HOME/emsdk}"
EIGEN_INCLUDE_DIR="${EIGEN_INCLUDE_DIR:-/opt/homebrew/include/eigen3}"

if [ ! -f "$EMSDK_DIR/emsdk_env.sh" ]; then
  echo "error: emsdk not found at $EMSDK_DIR (set EMSDK_DIR)" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$EMSDK_DIR/emsdk_env.sh" >/dev/null 2>&1

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$ROOT/build"
mkdir -p "$BUILD_DIR" "$ROOT/web/wasm"

emcmake cmake -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DEIGEN_INCLUDE_DIR="$EIGEN_INCLUDE_DIR" \
  -GNinja >/dev/null

cmake --build "$BUILD_DIR"

# Stamp the build date/time so the web header can show the running version.
printf 'export const BUILD = "%s";\n' "$(date '+%Y-%m-%d %H:%M')" > "$ROOT/web/version.js"

echo "✓ built -> web/wasm/slam.js + slam.wasm  (build $(cat "$ROOT/web/version.js" | sed 's/.*"\(.*\)".*/\1/'))"
