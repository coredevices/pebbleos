#!/bin/bash
# Copy WASM build artifacts from a QEMU build dir into web/.
#
# Usage: ./get-artifacts.sh <qemu-build-dir>
set -euo pipefail

BUILD_DIR="${1:?usage: $0 <qemu-build-dir> (e.g. ~/coredevices/qemu/build-wasm)}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$SCRIPT_DIR/web"

[ -f "$BUILD_DIR/qemu-system-arm.js" ] || {
    echo "error: $BUILD_DIR/qemu-system-arm.js not found -- build not finished?" >&2
    exit 1
}

# Ship the main module under both names: the page and smoke test import the
# .mjs (so node treats it as ES6 without a package.json that would break the
# CommonJS pthread worker), while the worker resolves it by its built-in
# .js name.
cp "$BUILD_DIR/qemu-system-arm.js" "$WEB_DIR/qemu-system-arm.mjs"
cp "$BUILD_DIR/qemu-system-arm.js" "$WEB_DIR/"
cp "$BUILD_DIR/qemu-system-arm.wasm" "$WEB_DIR/"
# Older emscripten (< 3.1.58) emits a separate pthread worker script.
if [ -f "$BUILD_DIR/qemu-system-arm.worker.js" ]; then
    cp "$BUILD_DIR/qemu-system-arm.worker.js" "$WEB_DIR/"
fi

ls -lh "$WEB_DIR"/qemu-system-arm.*
