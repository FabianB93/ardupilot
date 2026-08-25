#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
VERSION="2.1.7"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

if [[ -f "$HERE/esp32-camera/CMakeLists.txt" ]]; then
    echo "esp32-camera already installed at $HERE/esp32-camera"
else
    echo "Downloading esp32-camera v${VERSION}..."
    curl -L --fail --retry 3 \
      "https://github.com/espressif/esp32-camera/archive/refs/tags/v${VERSION}.tar.gz" \
      -o "$TMP/esp32-camera.tar.gz"
    tar -xzf "$TMP/esp32-camera.tar.gz" -C "$TMP"
    mv "$TMP/esp32-camera-${VERSION}" "$HERE/esp32-camera"
fi

python3 "$HERE/patch_esp32_camera_lowprio.py" "$HERE/esp32-camera"
echo "esp32-camera v${VERSION} installed and patched for low-priority capture."
