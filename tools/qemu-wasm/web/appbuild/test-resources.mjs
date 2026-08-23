// Validate resources.js against a golden waf build of pebble-timer (emery).
// The .reso intermediates are Python pickles wrapping the exact processed
// bytes, so the pbpack itself is the reference for every resource.
//
//   node test-resources.mjs [path-to-pebble-timer]
import { readFileSync } from 'node:fs';
import { buildResources, makePbpack, decodePng } from './resources.js';

const APP = process.argv[2] || '/home/user/aklitbo/pebble-timer';
const u8 = (p) => new Uint8Array(readFileSync(p));
const golden = u8(`${APP}/build/emery/app_resources.pbpack`);
const pkg = JSON.parse(readFileSync(`${APP}/package.json`, 'utf8'));
const media = pkg.pebble.resources.media;

let failures = 0;
const check = (name, ok, detail = '') => {
  console.log(`  ${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  if (!ok) failures++;
};

// --- golden pbpack -> per-resource blobs ---
function splitPack(pack) {
  const dv = new DataView(pack.buffer, pack.byteOffset, pack.byteLength);
  const n = dv.getUint32(0, true);
  const start = 12 + 256 * 16;
  const out = [];
  for (let i = 0; i < n; i++) {
    const p = 12 + i * 16;
    const off = dv.getUint32(p + 4, true), len = dv.getUint32(p + 8, true);
    out.push(pack.subarray(start + off, start + off + len));
  }
  return { blobs: out, crc: dv.getUint32(4, true) };
}
const gold = splitPack(golden);

// --- a node rasterizer that answers from the golden PFO files ---
// Browser builds rasterize with canvas; here we replay FreeType's own
// output so the serializer is what gets tested.
function parsePfo(pfo) {
  const dv = new DataView(pfo.buffer, pfo.byteOffset, pfo.byteLength);
  const version = pfo[0], height = pfo[1];
  const numGlyphs = dv.getUint16(2, true);
  const hashSize = pfo[6], cpBytes = pfo[7], infoSize = pfo[8], features = pfo[9];
  const offBytes = (features & 1) ? 2 : 4;
  const hashOff = infoSize;
  const tablesOff = hashOff + hashSize * 4;
  const glyphs = new Map();
  let cursor = tablesOff;
  const totalRecs = [];
  for (let i = 0; i < hashSize; i++) totalRecs.push(pfo[hashOff + i * 4 + 1]);
  const recCount = totalRecs.reduce((a, b) => a + b, 0);
  const glyphTableOff = tablesOff + recCount * (cpBytes + offBytes);
  for (let i = 0; i < hashSize; i++) {
    for (let k = 0; k < totalRecs[i]; k++) {
      const cp = cpBytes === 4 ? dv.getUint32(cursor, true) : dv.getUint16(cursor, true);
      const off = offBytes === 4 ? dv.getUint32(cursor + cpBytes, true)
                                 : dv.getUint16(cursor + cpBytes, true);
      cursor += cpBytes + offBytes;
      const g = glyphTableOff + off;
      const width = pfo[g], gh = pfo[g + 1];
      const left = dv.getInt8(g + 2), top = dv.getInt8(g + 3), advance = dv.getInt8(g + 4);
      // unpack LE 32-bit words, LSB first, no row padding
      const stride = Math.ceil(width / 8);
      const bitmap = new Uint8Array(stride * gh);
      for (let bit = 0; bit < width * gh; bit++) {
        const w = dv.getUint32(g + 5 + ((bit / 32) | 0) * 4, true);
        if (w & (1 << (bit % 32))) {
          const x = bit % width, y = (bit / width) | 0;
          bitmap[y * stride + (x >> 3)] |= 0x80 >> (x & 7);
        }
      }
      glyphs.set(cp, { bitmap, width, height: gh, left, top, advance });
    }
  }
  return { version, height, numGlyphs, features, cpBytes, offBytes, glyphs };
}

// media order -> golden blob index (png-trans takes two)
const goldenFor = {};
let gi = 0;
for (const d of media) {
  if (d.targetPlatforms && !d.targetPlatforms.includes('emery')) continue;
  if (d.type === 'png-trans') {
    goldenFor[d.name + '_WHITE'] = gold.blobs[gi++];
    goldenFor[d.name + '_BLACK'] = gold.blobs[gi++];
  } else goldenFor[d.name] = gold.blobs[gi++];
}

const goldenFonts = new Map();
for (const d of media) {
  if (d.type !== 'font') continue;
  const parsed = parsePfo(goldenFor[d.name]);
  goldenFonts.set(`${d.file}|${parsed.height}`, parsed);
}

const rasterizeGlyph = async (fontData, codepoint, px) => {
  for (const [key, f] of goldenFonts) {
    if (!key.endsWith('|' + px)) continue;
    const g = f.glyphs.get(codepoint);
    if (g) return { ...g, top: f.height - g.top };  // store as baseline-relative
    return { bitmap: new Uint8Array(0), width: 0, height: 0, left: 0, top: 0, advance: 0 };
  }
  throw new Error('no golden font for size ' + px);
};

console.log('resources.js vs golden waf build (emery)\n');

// 1. pbpack container: rebuild from the golden blobs, expect byte equality
const rebuilt = makePbpack(gold.blobs);
check('pbpack container byte-exact', Buffer.compare(Buffer.from(rebuilt), Buffer.from(golden)) === 0,
      `${rebuilt.length} vs ${golden.length} bytes`);

// 2. full pipeline from the original sources
const readFile = async (p) => u8(`${APP}/resources/${p}`);
const { pbpack, resourceNames: rawNames } =
  await buildResources(media, readFile, 'emery', rasterizeGlyph);
// entries carry their aliases now; the ids are what this check is about
const resourceNames = rawNames.map((r) => (typeof r === 'string' ? r : r.name));

// 3. resource id order
const expectNames = Object.keys(goldenFor);
check('resource id order', JSON.stringify(resourceNames) === JSON.stringify(expectNames),
      resourceNames.join(','));

// 4. per-resource comparison
const mine = splitPack(pbpack);
let exact = 0, sameSize = 0;
for (let i = 0; i < expectNames.length; i++) {
  const name = expectNames[i];
  const a = mine.blobs[i], b = goldenFor[name];
  const same = a && b && Buffer.compare(Buffer.from(a), Buffer.from(b)) === 0;
  if (same) exact++;
  else {
    sameSize++;
    console.log(`    ~ ${name}: ${a ? a.length : '-'} bytes vs golden ${b.length}` +
                ' (byte layout differs — checked pixel-wise below)');
  }
}
console.log(`  ---   ${exact}/${expectNames.length} resources byte-exact, ` +
            `${sameSize} compared by content`);
check('every resource produced', mine.blobs.length === expectNames.length);

// 5. images must decode to the same pixels even when bytes differ
//    (the SDK's PBI palette order comes from a Python set, so the byte
//    layout is not reproducible; what matters is the rendered image)
function decodePbi(pbi) {
  const dv = new DataView(pbi.buffer, pbi.byteOffset, pbi.byteLength);
  const rowSize = dv.getUint16(0, true), flags = dv.getUint16(2, true);
  const w = dv.getInt16(8, true), h = dv.getInt16(10, true);
  const format = (flags >> 1) & 7;
  const depth = { 0: 1, 1: 8, 2: 1, 3: 2, 4: 4 }[format];
  const bits = pbi.subarray(12, 12 + rowSize * h);
  const pal = pbi.subarray(12 + rowSize * h);
  const px = new Uint32Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      let argb;
      if (format === 0) {
        const word = dv.getUint32(12 + y * rowSize + ((x >> 5) << 2), true);
        argb = ((word >>> (x & 31)) & 1) ? 0xff : 0x00;   // 1 = white
      } else if (format === 1) {
        argb = bits[y * rowSize + x];
      } else {
        const per = 8 / depth;
        const idx = (bits[y * rowSize + ((x / per) | 0)] >> (depth * (per - 1 - (x % per))))
                    & ((1 << depth) - 1);
        argb = pal[idx];
      }
      px[y * w + x] = argb;
    }
  }
  return { w, h, px, format };
}

for (const d of media) {
  if (d.targetPlatforms && !d.targetPlatforms.includes('emery')) continue;
  const names = d.type === 'png-trans' ? [d.name + '_WHITE', d.name + '_BLACK'] : [d.name];
  for (const name of names) {
    const i = expectNames.indexOf(name);
    const a = mine.blobs[i], b = goldenFor[name];
    if (Buffer.compare(Buffer.from(a), Buffer.from(b)) === 0) continue;   // already exact
    if (d.type === 'bitmap' || d.type === 'png') {
      const pa = await decodePng(a), pb = await decodePng(b);
      check(`  ${name} pixels identical`,
            pa.width === pb.width && pa.height === pb.height &&
            Buffer.compare(Buffer.from(pa.rgba), Buffer.from(pb.rgba)) === 0,
            `${pa.width}x${pa.height} png`);
    } else if (d.type === 'pbi' || d.type === 'pbi8' || d.type === 'png-trans') {
      const pa = decodePbi(a), pb = decodePbi(b);
      check(`  ${name} pixels identical`,
            pa.w === pb.w && pa.h === pb.h && pa.format === pb.format &&
            pa.px.every((v, k) => v === pb.px[k]),
            `${pa.w}x${pa.h} pbi fmt${pa.format}`);
    }
  }
}

// 6. fonts must round-trip
for (const d of media) {
  if (d.type !== 'font') continue;
  const i = expectNames.indexOf(d.name);
  const a = parsePfo(mine.blobs[i]), b = parsePfo(goldenFor[d.name]);
  let glyphsMatch = a.glyphs.size === b.glyphs.size;
  for (const [cp, g] of b.glyphs) {
    const m = a.glyphs.get(cp);
    if (!m || m.width !== g.width || m.height !== g.height || m.left !== g.left ||
        m.top !== g.top || m.advance !== g.advance ||
        Buffer.compare(Buffer.from(m.bitmap), Buffer.from(g.bitmap)) !== 0) {
      glyphsMatch = false;
      console.log(`    ~ ${d.name} U+${cp.toString(16)} differs`);
    }
  }
  check(`  ${d.name} pfo round-trip (${b.glyphs.size} glyphs, v${a.version} h${a.height})`,
        glyphsMatch && a.numGlyphs === b.numGlyphs && a.features === b.features);
}

console.log(`\n${failures ? failures + ' CHECK(S) FAILED' : 'all checks passed'}`);
process.exit(failures ? 1 : 0);
