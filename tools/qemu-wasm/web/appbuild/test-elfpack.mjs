// Golden-data test for elfpack.js against the native SDK pipeline
// (arm-none-eabi-objcopy + inject_metadata.py + mkbundle.py) outputs for
// the pebble-timer app. Run with: node test-elfpack.mjs

import { readFileSync, writeFileSync, mkdtempSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { objcopyToBin, injectMetadata, makeManifest, makePbw } from './elfpack.js';

const GOLD = '/tmp/claude-0/-home-user-PebbleOS/17b76f60-ac9b-5f29-8dd5-ab64856fe715/scratchpad';
const BUILD = '/home/user/aklitbo/pebble-timer/build';

const elf = readFileSync(join(BUILD, 'emery/pebble-app-wasm.elf'));
const pbpack = readFileSync(join(BUILD, 'emery/app_resources.pbpack'));
const goldRaw = readFileSync(join(GOLD, 'pebble-app.raw.bin'));
const goldBin = readFileSync(join(GOLD, 'pebble-app.bin'));

let failed = 0;
function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'} ${name}${detail ? ` (${detail})` : ''}`);
  if (!ok) failed++;
}

function firstDiff(a, b) {
  if (a.length !== b.length) return `length ${a.length} != ${b.length}`;
  for (let i = 0; i < a.length; i++) {
    if (a[i] !== b[i]) return `byte ${i}: ${a[i]} != ${b[i]}`;
  }
  return null;
}

// (a) objcopy
const raw = objcopyToBin(new Uint8Array(elf));
check('objcopyToBin == pebble-app.raw.bin', !firstDiff(raw, goldRaw),
  firstDiff(raw, goldRaw) || `${raw.length} bytes`);

// (b) injectMetadata, with the golden run's inputs recovered from its output:
// resource_timestamp @0x7c, allow_js/has_worker from the flags delta.
const gdv = new DataView(goldBin.buffer, goldBin.byteOffset, goldBin.byteLength);
const timestamp = gdv.getUint32(0x7c, true);
const flagsDelta = gdv.getUint32(0x60, true) ^
  new DataView(goldRaw.buffer, goldRaw.byteOffset).getUint32(0x60, true);
const allowJs = !!(flagsDelta & (1 << 3));
const hasWorker = !!(flagsDelta & (1 << 4));
console.log(`golden run: timestamp=${timestamp} allowJs=${allowJs} hasWorker=${hasWorker}`);

const bin = injectMetadata(raw, new Uint8Array(elf), new Uint8Array(pbpack),
  { timestamp, allowJs, hasWorker });
check('injectMetadata == pebble-app.bin', !firstDiff(bin, goldBin),
  firstDiff(bin, goldBin) || `${bin.length} bytes`);

// (c) manifest matches the one mkbundle.py put in the golden pbw
const goldManifest = JSON.parse(execFileSync('unzip',
  ['-p', join(GOLD, 'timer-wasm.pbw'), 'emery/manifest.json']).toString());
const manifest = makeManifest({
  appBin: bin, pbpack: new Uint8Array(pbpack), timestamp,
  sdkVersion: goldManifest.application.sdk_version, jsPresent: true,
});
check('makeManifest == golden emery/manifest.json',
  JSON.stringify(manifest) === JSON.stringify(goldManifest),
  JSON.stringify(manifest) === JSON.stringify(goldManifest) ? '' :
    `\n  ours:   ${JSON.stringify(manifest)}\n  golden: ${JSON.stringify(goldManifest)}`);

// (d) makePbw: archive integrity via unzip -t, then byte-compare the bin
const date = new Date(timestamp * 1000);
const appinfo = readFileSync(join(BUILD, 'appinfo.json'));
const js = readFileSync(join(BUILD, 'pebble-js-app.js'));
const pbw = makePbw([
  { name: 'appinfo.json', data: new Uint8Array(appinfo), date },
  { name: 'pebble-js-app.js', data: new Uint8Array(js), date },
  { name: 'emery/pebble-app.bin', data: bin, date },
  { name: 'emery/app_resources.pbpack', data: new Uint8Array(pbpack), date },
  { name: 'emery/manifest.json',
    data: new TextEncoder().encode(JSON.stringify(manifest)), date },
]);
const dir = mkdtempSync(join(tmpdir(), 'elfpack-'));
const pbwPath = join(dir, 'out.pbw');
writeFileSync(pbwPath, pbw);
try {
  execFileSync('unzip', ['-t', pbwPath], { stdio: 'pipe' });
  check('unzip -t on makePbw output', true);
} catch (e) {
  check('unzip -t on makePbw output', false, e.stderr?.toString() || e.message);
}
const extracted = execFileSync('unzip', ['-p', pbwPath, 'emery/pebble-app.bin'],
  { maxBuffer: 1 << 24 });
check('pebble-app.bin extracted from pbw == golden', !firstDiff(new Uint8Array(extracted), goldBin));

process.exit(failed ? 1 : 0);
