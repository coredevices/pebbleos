#!/usr/bin/env bash
# Unpack the prebuilt emulator site into a pebble-qemu-wasm checkout.
# Usage: apply.sh <path-to-pebble-qemu-wasm-checkout>
set -euo pipefail

dst="${1:?usage: apply.sh <pebble-qemu-wasm checkout>}"
src="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$dst/firmware/emery"
# Copy the page and every JS module wholesale so a newly added module
# can never be missed (a missing import kills the whole module script).
cp "$src/index.html" "$src/config-return.html" "$dst/"
cp "$src"/*.js "$dst/"
mkdir -p "$dst/pkjs-proxy"
cp "$src/../pkjs-proxy/worker.js" "$src/../pkjs-proxy/README.md" "$dst/pkjs-proxy/"

# In-browser app compiler: page modules, the vendored WASI shim and
# esbuild, and the toolchain itself. These live in web/ rather than here
# because tools/ alone is ~39 MB and would otherwise be stored twice.
rm -rf "$dst/appbuild" "$dst/vendor" "$dst/tools"
cp -r "$src/../web/appbuild" "$src/../web/vendor" "$src/../web/tools" "$dst/"
rm -f "$dst"/appbuild/test-*.mjs
rm -f "$dst/qemu-system-arm.worker.js"
gunzip -c "$src/qemu-system-arm.js.gz" > "$dst/qemu-system-arm.js"
gunzip -c "$src/qemu-system-arm.wasm.gz" > "$dst/qemu-system-arm.wasm"
gunzip -c "$src/qemu_micro_flash.bin.gz" > "$dst/firmware/emery/qemu_micro_flash.bin"
gunzip -c "$src/qemu_spi_flash.bin.gz" > "$dst/firmware/emery/qemu_spi_flash.bin"
mkdir -p "$dst/firmware/gabbro"
gunzip -c "$src/qemu_micro_flash_gabbro.bin.gz" > "$dst/firmware/gabbro/qemu_micro_flash.bin"
gunzip -c "$src/qemu_spi_flash_gabbro.bin.gz" > "$dst/firmware/gabbro/qemu_spi_flash.bin"
mkdir -p "$dst/firmware/flint"
gunzip -c "$src/qemu_micro_flash_flint.bin.gz" > "$dst/firmware/flint/qemu_micro_flash.bin"
gunzip -c "$src/qemu_spi_flash_flint.bin.gz" > "$dst/firmware/flint/qemu_spi_flash.bin"

echo "Applied. New site files:"
(cd "$dst" && ls -la index.html qemu-system-arm.js qemu-system-arm.wasm \
    coi-serviceworker.min.js firmware/emery/ tools/)
