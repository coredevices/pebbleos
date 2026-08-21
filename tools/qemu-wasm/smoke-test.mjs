// Headless smoke test: boot the WASM QEMU build under node, watch the
// UART2 debug console, and pass once boot markers appear.
//
// Usage: node smoke-test.mjs [--board emery] [--seconds 300]
//
// Requires: web/qemu-system-arm.mjs + .wasm (get-artifacts.sh) and
// web/firmware/<board>/ (fetch-firmware.sh). Node 18+.

import { readFileSync, existsSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import process from 'node:process';

// Machine names and display sizes from board_cfg_* in
// coredevices/qemu hw/arm/pebble_generic.c.
const BOARDS = {
  emery: { machine: 'pebble-emery', width: 200, height: 228 },
  flint: { machine: 'pebble-flint', width: 144, height: 168 },
  gabbro: { machine: 'pebble-gabbro', width: 260, height: 260 },
};

// Ordered boot progress markers on the UART2 debug console.
// "Ready for communication." is logged by the firmware specifically so
// tooling (pebble-tool) can detect a completed boot.
const MARKERS = [
  { name: 'serial-output', re: /[ -~]{4}/ },
  { name: 'boot-complete', re: /Ready for communication/i },
];

function arg(name, dflt) {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1] : dflt;
}

const board = arg('board', 'emery');
const seconds = parseInt(arg('seconds', '300'), 10);
const boardCfg = BOARDS[board];
if (!boardCfg) {
  console.error(`unknown board '${board}' (expected: ${Object.keys(BOARDS).join(', ')})`);
  process.exit(1);
}
const machine = boardCfg.machine;

const here = dirname(fileURLToPath(import.meta.url));
const webDir = join(here, 'web');
const jsPath = join(webDir, 'qemu-system-arm.mjs');
const fwDir = join(webDir, 'firmware', board);

for (const p of [jsPath, join(fwDir, 'qemu_micro_flash.bin'), join(fwDir, 'qemu_spi_flash.bin')]) {
  if (!existsSync(p)) {
    console.error(`missing: ${p}`);
    console.error('run get-artifacts.sh and fetch-firmware.sh first');
    process.exit(1);
  }
}

const micro = readFileSync(join(fwDir, 'qemu_micro_flash.bin'));
const spi = readFileSync(join(fwDir, 'qemu_spi_flash.bin'));

console.log(`=== WASM smoke test: ${machine}, ${seconds}s budget ===`);

const factory = (await import(pathToFileURL(jsPath))).default;

const t0 = Date.now();
const elapsed = () => ((Date.now() - t0) / 1000).toFixed(0).padStart(4);

let serialText = '';
let serialPos = 0;
let matched = 0;

const cfg = {
  arguments: [
    '-machine', machine,
    '-kernel', '/firmware/qemu_micro_flash.bin',
    '-drive', 'if=mtd,format=raw,file=/firmware/qemu_spi_flash.bin',
    '-display', 'none',
    '-monitor', 'none',
    '-parallel', 'none',
    '-serial', 'null',                 // UART0
    '-serial', 'null',                 // UART1: pebble control protocol
    '-serial', 'file:/tmp/uart2.log',  // UART2: debug console
  ],
  print: (t) => console.log(`[${elapsed()}s] [qemu] ${t}`),
  printErr: (t) => console.log(`[${elapsed()}s] [qemu:err] ${t}`),
  preRun: [() => {
    cfg.FS.mkdir('/firmware');
    cfg.FS.writeFile('/firmware/qemu_micro_flash.bin', micro);
    cfg.FS.writeFile('/firmware/qemu_spi_flash.bin', spi);
  }],
  onAbort: (why) => {
    console.error(`[${elapsed()}s] QEMU aborted: ${why}`);
    process.exit(1);
  },
  locateFile: (p) => join(webDir, p),
};

const qemu = await factory(cfg);
console.log(`[${elapsed()}s] runtime initialized`);

function pollSerial() {
  let stat;
  try {
    stat = qemu.FS.stat('/tmp/uart2.log');
  } catch {
    return; // chardev not opened yet
  }
  if (stat.size <= serialPos) {
    return;
  }
  const stream = qemu.FS.open('/tmp/uart2.log', 'r');
  const chunk = new Uint8Array(stat.size - serialPos);
  qemu.FS.read(stream, chunk, 0, chunk.length, serialPos);
  qemu.FS.close(stream);
  serialPos = stat.size;

  let text = '';
  for (const b of chunk) {
    if ((b >= 32 && b <= 126) || b === 9 || b === 10 || b === 13) {
      text += String.fromCharCode(b);
    }
  }
  serialText += text;
  for (const line of text.split('\n')) {
    if (line.trim()) {
      console.log(`[${elapsed()}s] [uart2] ${line.trimEnd()}`);
    }
  }
}

// Display sanity: the pebble_wasm_* glue exports the rendered console
// surface; after boot the frame counter must be advancing and the guest
// dimensions must match the board config.
function checkDisplay() {
  const frames = qemu._pebble_wasm_display_frame_count();
  const w = qemu._pebble_wasm_display_width();
  const h = qemu._pebble_wasm_display_height();
  console.log(`[${elapsed()}s] display: ${w}x${h}, ${frames} frames rendered`);
  if (frames <= 0) {
    console.error('display check failed: frame count did not advance');
    return false;
  }
  if (w !== boardCfg.width || h !== boardCfg.height) {
    console.error(`display check failed: expected ${boardCfg.width}x${boardCfg.height}`);
    return false;
  }
  return true;
}

function finish(code) {
  if (code === 0 && !checkDisplay()) {
    code = 1;
  }
  console.log(code === 0
    ? `PASS: ${MARKERS.length} boot markers + display check after ${elapsed()}s`
    : `FAIL: ${matched}/${MARKERS.length} markers seen after ${elapsed()}s ` +
      `(${serialPos} serial bytes captured)`);
  process.exit(code);
}

const timer = setInterval(() => {
  pollSerial();
  while (matched < MARKERS.length && MARKERS[matched].re.test(serialText)) {
    console.log(`[${elapsed()}s] marker '${MARKERS[matched].name}' matched`);
    matched++;
  }
  if (matched === MARKERS.length) {
    clearInterval(timer);
    finish(0);
  } else if (Date.now() - t0 > seconds * 1000) {
    clearInterval(timer);
    finish(1);
  }
}, 500);
