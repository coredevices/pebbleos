#!/bin/bash
# Download PebbleOS release firmware images for the emulator.
#
# Usage: ./fetch-firmware.sh [board] [version]
#   board:   emery (default), flint, gabbro
#   version: release tag, default v4.35.0
set -euo pipefail

BOARD="${1:-emery}"
VERSION="${2:-v4.35.0}"
BASE_URL="https://github.com/coredevices/PebbleOS/releases/download/$VERSION"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$SCRIPT_DIR/web/firmware/$BOARD"
mkdir -p "$DEST"

for kind in micro spi; do
    asset="qemu_${BOARD}_${VERSION}_${kind}_flash.bin"
    out="$DEST/qemu_${kind}_flash.bin"
    echo "Fetching $asset..."
    curl -fL --progress-bar -o "$out" "$BASE_URL/$asset"
done

ls -lh "$DEST"
