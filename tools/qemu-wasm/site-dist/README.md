# Prebuilt site bundle (transport only — do not merge)

A ready-to-deploy build of the browser emulator for the
ericmigi/pebble-qemu-wasm GitHub Pages site: the web shell from
`../web/`, the Pebble-only QEMU wasm build (coredevices/qemu
pebble-10.1 + `../patches/0001`, `--with-devices-arm=pebble`), and the
v4.35.0 emery firmware images, binaries gzipped.

This directory exists so another session with push access to
ericmigi/pebble-qemu-wasm can deploy without redoing the hour-long
wasm build: run `./apply.sh <checkout>` and commit the result there.

Remove this directory before merging the branch — the reproducible
build recipe lives in `../build-qemu-wasm.sh`.
