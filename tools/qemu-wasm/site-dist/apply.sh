#!/usr/bin/env bash
# Unpack the prebuilt emulator site into a pebble-qemu-wasm checkout.
# Usage: apply.sh <path-to-pebble-qemu-wasm-checkout>
set -euo pipefail

dst="${1:?usage: apply.sh <pebble-qemu-wasm checkout>}"
src="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$dst/firmware/emery"
cp "$src/index.html" "$src/coi-serviceworker.min.js" "$dst/"
cp "$src/pebble-transport.js" "$src/pbw.js" "$src/app-install.js" "$src/store.js" "$dst/"
rm -f "$dst/qemu-system-arm.worker.js"
gunzip -c "$src/qemu-system-arm.js.gz" > "$dst/qemu-system-arm.js"
gunzip -c "$src/qemu-system-arm.wasm.gz" > "$dst/qemu-system-arm.wasm"
gunzip -c "$src/qemu_micro_flash.bin.gz" > "$dst/firmware/emery/qemu_micro_flash.bin"
gunzip -c "$src/qemu_spi_flash.bin.gz" > "$dst/firmware/emery/qemu_spi_flash.bin"

echo "Applied. New site files:"
(cd "$dst" && ls -la index.html qemu-system-arm.js qemu-system-arm.wasm \
    coi-serviceworker.min.js firmware/emery/)
