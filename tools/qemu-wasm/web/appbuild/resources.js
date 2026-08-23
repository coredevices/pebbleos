// Resource compilation, the browser stand-in for the SDK's waf resource
// generators. Produces the app_resources.pbpack an app is bundled with.
//
// Ported from common/tools: pbpack.py (container), bitmapgen.py (PBI),
// png2pblpng.py + pebble_image_routines.py (palettised PNG) and
// font/fontgen.py (PFO v3). Font rasterisation is delegated to the host
// via rasterizeGlyph, since FreeType is not available here.
import { legacyDefectiveCrc } from '../app-install.js';
import { PLATFORMS } from './codegen.js';

const te = new TextEncoder();

// --- zlib via the streams API (browser and node alike) --------------------

async function inflate(bytes) {
  const s = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Uint8Array(await new Response(s).arrayBuffer());
}

async function deflate(bytes) {
  const s = new Blob([bytes]).stream().pipeThrough(new CompressionStream('deflate'));
  return new Uint8Array(await new Response(s).arrayBuffer());
}

const PNG_CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xedb88320 ^ (c >>> 1)) : (c >>> 1);
    t[n] = c >>> 0;
  }
  return t;
})();

function pngCrc32(data) {
  let c = 0xffffffff;
  for (let i = 0; i < data.length; i++) c = PNG_CRC_TABLE[(c ^ data[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

// --- PNG decode -----------------------------------------------------------

const PNG_MAGIC = [0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a];

function paeth(a, b, c) {
  const p = a + b - c, pa = Math.abs(p - a), pb = Math.abs(p - b), pc = Math.abs(p - c);
  return (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);
}

// Decode any PNG to straight RGBA8. Returns {width, height, rgba}.
export async function decodePng(bytes) {
  for (let i = 0; i < 8; i++) {
    if (bytes[i] !== PNG_MAGIC[i]) throw new Error('not a PNG file');
  }
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let width = 0, height = 0, depth = 0, colorType = 0, interlace = 0;
  let plte = null, trns = null;
  const idat = [];
  for (let p = 8; p + 8 <= bytes.length;) {
    const len = dv.getUint32(p);
    const type = String.fromCharCode(bytes[p + 4], bytes[p + 5], bytes[p + 6], bytes[p + 7]);
    const data = bytes.subarray(p + 8, p + 8 + len);
    if (type === 'IHDR') {
      width = dv.getUint32(p + 8);
      height = dv.getUint32(p + 12);
      depth = data[8]; colorType = data[9]; interlace = data[12];
    } else if (type === 'PLTE') plte = data;
    else if (type === 'tRNS') trns = data;
    else if (type === 'IDAT') idat.push(data);
    else if (type === 'IEND') break;
    p += 12 + len;
  }
  if (interlace) throw new Error('interlaced PNGs are not supported');

  const raw = await inflate(concat(idat));
  const channels = { 0: 1, 2: 3, 3: 1, 4: 2, 6: 4 }[colorType];
  if (!channels) throw new Error('unsupported PNG colour type ' + colorType);
  const bpp = Math.max(1, (channels * depth) >> 3);      // filter unit, bytes
  const stride = Math.ceil(width * channels * depth / 8);

  // unfilter in place
  const lines = new Uint8Array(height * stride);
  let src = 0;
  for (let y = 0; y < height; y++) {
    const filter = raw[src++];
    const row = y * stride, prev = row - stride;
    for (let x = 0; x < stride; x++) {
      const a = x >= bpp ? lines[row + x - bpp] : 0;
      const b = y > 0 ? lines[prev + x] : 0;
      const c = (x >= bpp && y > 0) ? lines[prev + x - bpp] : 0;
      let v = raw[src++];
      if (filter === 1) v += a;
      else if (filter === 2) v += b;
      else if (filter === 3) v += (a + b) >> 1;
      else if (filter === 4) v += paeth(a, b, c);
      lines[row + x] = v & 0xff;
    }
  }

  // expand to RGBA8
  const rgba = new Uint8Array(width * height * 4);
  const sample = (row, i) => {
    if (depth === 8) return lines[row + i];
    if (depth === 16) return lines[row + i * 2];
    const per = 8 / depth, mask = (1 << depth) - 1;
    const v = (lines[row + ((i / per) | 0)] >> (depth * (per - 1 - (i % per)))) & mask;
    return v;
  };
  const scale = depth < 8 ? 255 / ((1 << depth) - 1) : 1;
  for (let y = 0; y < height; y++) {
    const row = y * stride;
    for (let x = 0; x < width; x++) {
      const o = (y * width + x) * 4;
      if (colorType === 3) {
        const idx = sample(row, x);
        rgba[o] = plte[idx * 3]; rgba[o + 1] = plte[idx * 3 + 1]; rgba[o + 2] = plte[idx * 3 + 2];
        rgba[o + 3] = (trns && idx < trns.length) ? trns[idx] : 255;
      } else if (colorType === 0 || colorType === 4) {
        const g = Math.round(sample(row, x * channels) * scale);
        rgba[o] = rgba[o + 1] = rgba[o + 2] = g;
        rgba[o + 3] = colorType === 4 ? Math.round(sample(row, x * channels + 1) * scale) : 255;
      } else {
        for (let c = 0; c < 3; c++) rgba[o + c] = Math.round(sample(row, x * channels + c) * scale);
        rgba[o + 3] = colorType === 6 ? Math.round(sample(row, x * channels + 3) * scale) : 255;
      }
    }
  }
  return { width, height, rgba };
}

function concat(chunks) {
  let n = 0;
  for (const c of chunks) n += c.length;
  const out = new Uint8Array(n);
  let o = 0;
  for (const c of chunks) { out.set(c, o); o += c.length; }
  return out;
}

// --- PNG encode (palettised / greyscale, as png2pblpng writes) ------------

async function encodePng({ width, height, bitdepth, indices, palette, greyscale, transparent }) {
  const chunks = [];
  const chunk = (type, data) => {
    const out = new Uint8Array(12 + data.length);
    const d = new DataView(out.buffer);
    d.setUint32(0, data.length);
    out.set(te.encode(type), 4);
    out.set(data, 8);
    d.setUint32(8 + data.length, pngCrc32(out.subarray(4, 8 + data.length)));
    chunks.push(out);
  };

  const ihdr = new Uint8Array(13);
  const idv = new DataView(ihdr.buffer);
  idv.setUint32(0, width); idv.setUint32(4, height);
  ihdr[8] = bitdepth;
  ihdr[9] = greyscale ? 0 : 3;
  chunks.push(new Uint8Array(PNG_MAGIC));
  chunk('IHDR', ihdr);

  if (!greyscale) {
    const hasAlpha = palette.length > 0 && palette[0].length === 4;
    const plte = new Uint8Array(palette.length * 3);
    for (let i = 0; i < palette.length; i++) plte.set(palette[i].slice(0, 3), i * 3);
    chunk('PLTE', plte);
    if (hasAlpha) {
      chunk('tRNS', new Uint8Array(palette.map((p) => p[3])));
    }
  } else if (transparent !== null && transparent !== undefined) {
    const t = new Uint8Array(2);
    new DataView(t.buffer).setUint16(0, transparent);
    chunk('tRNS', t);
  }

  // rows, each prefixed with filter type 0 (none)
  const per = 8 / bitdepth;
  const stride = Math.ceil(width / per);
  const rows = new Uint8Array(height * (stride + 1));
  for (let y = 0; y < height; y++) {
    const base = y * (stride + 1) + 1;
    for (let x = 0; x < width; x++) {
      const v = indices[y * width + x];
      if (bitdepth === 8) rows[base + x] = v;
      else rows[base + ((x / per) | 0)] |= v << (bitdepth * (per - 1 - (x % per)));
    }
  }
  chunk('IDAT', await deflate(rows));
  chunk('IEND', new Uint8Array(0));
  return concat(chunks);
}

// --- colour reduction (pebble_image_routines.py) --------------------------

function nearestPebble64(r, g, b, a) {
  a = Math.floor((a + 42) / 85) * 85;
  if (a === 0) return [0, 0, 0, 0];
  return [Math.floor((r + 42) / 85) * 85, Math.floor((g + 42) / 85) * 85,
          Math.floor((b + 42) / 85) * 85, a];
}

function nearestPebble2(r, g, b, a) {
  const luma = r * 0.2126 + g * 0.7152 + b * 0.11;
  const bit = (v) => (v > 255 / 2 ? 255 : 0);
  const l = bit(luma);
  return [l, l, l, bit(a)];
}

const reduceFor = (paletteName) =>
  (paletteName === 'pebble2' ? nearestPebble2 : nearestPebble64);

function rgbaToArgb8(r, g, b, a) {
  return ((a >> 6) << 6) | ((r >> 6) << 4) | ((g >> 6) << 2) | (b >> 6);
}

function numColorsToBitdepth(n) {
  let d = Math.ceil(Math.log2(Math.max(n, 1)));
  if (d === 0) return 1;
  if (d === 3) return 4;
  if (d > 4) return 8;
  return d;
}

// --- palettised PNG (png2pblpng.py) ---------------------------------------

// Walk the image once to decide greyscale/alpha/bitdepth and collect the
// palette, exactly as get_palette_for_png() does.
function analysePng({ width, height, rgba }, paletteName) {
  const reduce = reduceFor(paletteName);
  const palette = [];
  const seen = new Map();
  let isGrey = true, hasAlpha = false;
  for (let i = 0; i < width * height; i++) {
    const [r, g, b, a] = reduce(rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2], rgba[i * 4 + 3]);
    const key = (r << 24 | g << 16 | b << 8 | a) >>> 0;
    if (seen.has(key)) continue;
    seen.set(key, palette.length);
    palette.push([r, g, b, a]);
    if (a !== 0xff) hasAlpha = true;
    if (isGrey && !((r === g && g === b && a === 255) || (r === 0 && g === 0 && b === 0 && a === 0))) {
      isGrey = false;
    }
  }

  let bitdepth = numColorsToBitdepth(palette.length);
  if (isGrey) {
    const has = (r) => palette.some((p) => p[0] === r && p[1] === r && p[2] === r && p[3] === 255);
    let greyDepth;
    if (has(85) || has(170)) greyDepth = palette.length >= 5 ? 4 : 2;
    else greyDepth = palette.length >= 3 ? 2 : 1;
    if (greyDepth > bitdepth) isGrey = false;
    else bitdepth = greyDepth;
  }
  return { isGrey, hasAlpha, bitdepth, palette, seen };
}

async function pebblePng(image, paletteName, forceBitdepth) {
  const reduce = reduceFor(paletteName);
  let { isGrey, hasAlpha, bitdepth, palette, seen } = analysePng(image, paletteName);

  if (forceBitdepth !== undefined && forceBitdepth !== null) {
    if (bitdepth > forceBitdepth) {
      throw new Error(`tried to force ${forceBitdepth} bits; need at least ${bitdepth}`);
    }
    if (bitdepth !== forceBitdepth) isGrey = false;
    bitdepth = forceBitdepth;
  }

  let transparentGrey = null;
  if (isGrey && hasAlpha) {
    if (bitdepth === 4) transparentGrey = 0xc;
    else {
      for (const lum of [0, 255, 85, 170]) {
        if (!palette.some((p) => p[0] === lum && p[1] === lum && p[2] === lum && p[3] === 255)) {
          transparentGrey = lum >> (8 - bitdepth);
          break;
        }
      }
    }
  }

  const { width, height, rgba } = image;
  const indices = new Uint8Array(width * height);
  for (let i = 0; i < width * height; i++) {
    const [r, g, b, a] = reduce(rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2], rgba[i * 4 + 3]);
    if (isGrey) {
      indices[i] = a === 0 ? transparentGrey : (r >> (8 - bitdepth));
    } else {
      indices[i] = seen.get((r << 24 | g << 16 | b << 8 | a) >>> 0);
    }
  }
  // A PNG without alpha carries an RGB palette (no tRNS).
  const outPalette = hasAlpha ? palette : palette.map((p) => p.slice(0, 3));
  return encodePng({
    width, height, bitdepth, indices,
    palette: isGrey ? null : outPalette,
    greyscale: isGrey, transparent: transparentGrey,
  });
}

// --- PBI (bitmapgen.py) ---------------------------------------------------

const WHITE_COLOR_MAP = { white: 1, black: 0, transparent: 0 };
const BLACK_COLOR_MAP = { white: 0, black: 1, transparent: 0 };
// bitdepth -> GBitmapFormat
const BITDEPTH_FORMAT = { 0: 0, 8: 1, 1: 2, 2: 3, 4: 4 };

// Box of non-transparent pixels, reproducing bitmapgen._set_bbox().
// Note its right edge is derived by walking *rows* from the bottom -- the
// python transposes the columns back into rows before that loop -- so empty
// right-hand columns are not trimmed. Matching waf's bounds matters more
// than tightening the box, so the behaviour is kept as-is.
function inkBox({ width, height, rgba }, crop) {
  if (!crop) return { x: 0, y: 0, w: width, h: height };
  const rowHasInk = (y) => {
    for (let x = 0; x < width; x++) if (rgba[(y * width + x) * 4 + 3]) return true;
    return false;
  };
  const colHasInk = (x) => {
    for (let y = 0; y < height; y++) if (rgba[(y * width + x) * 4 + 3]) return true;
    return false;
  };
  let left = 0, top = 0, right = width, bottom = height;
  while (top < height && !rowHasInk(top)) top++;
  while (bottom > 0 && !rowHasInk(bottom - 1)) bottom--;
  while (left < width && !colHasInk(left)) left++;
  for (let y = height - 1; y >= 0 && !rowHasInk(y); y--) right--;
  return { x: left, y: top, w: right - left, h: bottom - top };
}

function pbiHeader(rowSizeBytes, bitdepth, box) {
  const out = new Uint8Array(12);
  const dv = new DataView(out.buffer);
  dv.setUint16(0, rowSizeBytes, true);
  dv.setUint16(2, (1 << 12) | (BITDEPTH_FORMAT[bitdepth] << 1), true);
  dv.setInt16(4, box.x, true);
  dv.setInt16(6, box.y, true);
  dv.setInt16(8, box.w, true);
  dv.setInt16(10, box.h, true);
  return out;
}

// Legacy 1-bit PBI: rows padded to whole 32-bit words, LSB is leftmost.
function pbiBw(image, box, colorMap) {
  const { width, rgba } = image;
  const rowWords = (box.w + 31) >> 5;
  const rowSize = rowWords * 4;
  const bits = new Uint8Array(rowSize * box.h);
  const dv = new DataView(bits.buffer);
  for (let row = 0; row < box.h; row++) {
    for (let w = 0; w < rowWords; w++) {
      let word = 0;
      for (let col = 0; col < 32; col++) {
        const x = box.x + w * 32 + col;
        if (x >= width) continue;
        const o = ((box.y + row) * width + x) * 4;
        let v;
        if (rgba[o + 3] < 127) v = colorMap.transparent;
        else if ((rgba[o] + rgba[o + 1] + rgba[o + 2]) / 3 < 127) v = colorMap.black;
        else v = colorMap.white;
        word |= v << col;
      }
      dv.setUint32((row * rowWords + w) * 4, word >>> 0, true);
    }
  }
  return { rowSize, bits, palette: null, bitdepth: 0 };
}

// Palettised (or 8-bit) colour PBI.
function pbiColor(image, box, paletteName, forceBitdepth) {
  const { width, rgba } = image;
  const reduce = reduceFor(paletteName);
  const argb = new Uint8Array(box.w * box.h);
  const colors = new Set();
  for (let row = 0; row < box.h; row++) {
    for (let col = 0; col < box.w; col++) {
      const o = ((box.y + row) * width + (box.x + col)) * 4;
      const [r, g, b, a] = reduce(rgba[o], rgba[o + 1], rgba[o + 2], rgba[o + 3]);
      const v = rgbaToArgb8(r, g, b, a);
      argb[row * box.w + col] = v;
      colors.add(v);
    }
  }
  // The SDK derives the palette from a Python set, whose iteration order we
  // cannot reproduce; any order renders the same, so use ascending.
  const palette = [...colors].sort((a, b) => a - b);
  const minDepth = numColorsToBitdepth(palette.length);
  const bitdepth = forceBitdepth || minDepth;
  if (bitdepth < minDepth) {
    throw new Error(`required bitdepth ${bitdepth} is lower than needed ${minDepth}`);
  }

  const perByte = bitdepth === 8 ? 1 : 8 / bitdepth;
  const rowSize = bitdepth === 8 ? box.w : Math.ceil(box.w / perByte);
  const bits = new Uint8Array(rowSize * box.h);
  const index = new Map(palette.map((v, i) => [v, i]));
  for (let row = 0; row < box.h; row++) {
    for (let col = 0; col < box.w; col++) {
      const v = argb[row * box.w + col];
      if (bitdepth === 8) bits[row * rowSize + col] = v;
      else {
        bits[row * rowSize + ((col / perByte) | 0)] |=
          index.get(v) << (bitdepth * (perByte - 1 - (col % perByte)));
      }
    }
  }
  return { rowSize, bits, palette: bitdepth < 8 ? palette : null, bitdepth };
}

function makePbi(image, { format = 'bw', colorMap = WHITE_COLOR_MAP, crop = true,
                          paletteName = 'pebble64', bitdepth } = {}) {
  const box = inkBox(image, crop);
  const r = format === 'bw'
    ? pbiBw(image, box, colorMap)
    : pbiColor(image, box, paletteName, bitdepth);
  const head = pbiHeader(r.rowSize, r.bitdepth, box);
  const palLen = r.palette ? (1 << r.bitdepth) : 0;
  const out = new Uint8Array(head.length + r.bits.length + palLen);
  out.set(head, 0);
  out.set(r.bits, head.length);
  if (r.palette) {
    for (let i = 0; i < palLen; i++) {
      out[head.length + r.bits.length + i] = i < r.palette.length ? r.palette[i] : 0;
    }
  }
  return out;
}

// --- PFO v3 fonts (font/fontgen.py) --------------------------------------

const WILDCARD_CODEPOINT = 0x25af;
const ELLIPSIS_CODEPOINT = 0x2026;
const HASH_TABLE_SIZE = 255;
const MAX_GLYPHS = 256;
const MAX_GLYPHS_EXTENDED = HASH_TABLE_SIZE * 128;
const FEATURE_OFFSET_16 = 0x01;

// Locate the sfnt header. It is usually at 0, but fonts carrying a wrapper
// (or a collection) put it elsewhere, so fall back to scanning — FreeType
// probes for it the same way.
function sfntBase(dv, font) {
  const isSfnt = (o) => {
    if (o + 12 > font.length) return false;
    const ver = dv.getUint32(o);
    if (ver !== 0x00010000 && ver !== 0x4f54544f && ver !== 0x74727565) return false;
    const n = dv.getUint16(o + 4);
    if (n < 1 || n > 64 || o + 12 + n * 16 > font.length) return false;
    for (let i = 0; i < n; i++) {
      for (let c = 0; c < 4; c++) {
        const ch = font[o + 12 + i * 16 + c];
        if (ch < 0x20 || ch > 0x7e) return false;
      }
    }
    return true;
  };
  if (dv.getUint32(0) === 0x74746366) return dv.getUint32(12);   // 'ttcf'
  if (isSfnt(0)) return 0;
  for (let o = 4; o < Math.min(font.length - 12, 4096); o += 4) {
    if (isSfnt(o)) return o;
  }
  throw new Error('no sfnt header found — not a TrueType/OpenType font?');
}

// The font as a browser FontFace will accept it: table offsets are
// relative to the sfnt header, so dropping anything before it yields a
// valid font. FreeType tolerates the prefix; Chromium does not.
export function sfntFont(font) {
  const dv = new DataView(font.buffer, font.byteOffset, font.byteLength);
  const base = sfntBase(dv, font);
  return base ? font.subarray(base) : font;
}

// Codepoints a TrueType/OpenType font's cmap can render. Handles the two
// formats that matter in practice: 4 (BMP) and 12 (full range).
export function cmapCodepoints(font) {
  const dv = new DataView(font.buffer, font.byteOffset, font.byteLength);
  const base = sfntBase(dv, font);
  const numTables = dv.getUint16(base + 4);
  let cmapOff = 0;
  for (let i = 0; i < numTables; i++) {
    const rec = base + 12 + i * 16;
    const tag = String.fromCharCode(font[rec], font[rec + 1], font[rec + 2], font[rec + 3]);
    // Table offsets are relative to the sfnt header, not the file.
    if (tag === 'cmap') { cmapOff = base + dv.getUint32(rec + 8); break; }
  }
  if (!cmapOff) throw new Error('font has no cmap table');

  // prefer a full-range (4,10)/(3,10) subtable, else BMP
  let best = 0, bestScore = -1;
  const n = dv.getUint16(cmapOff + 2);
  for (let i = 0; i < n; i++) {
    const p = dv.getUint16(cmapOff + 4 + i * 8);
    const e = dv.getUint16(cmapOff + 6 + i * 8);
    const off = dv.getUint32(cmapOff + 8 + i * 8);
    let score = -1;
    if (p === 3 && e === 10) score = 4;
    else if (p === 0 && e >= 4) score = 3;
    else if (p === 3 && e === 1) score = 2;
    else if (p === 0) score = 1;
    else if (p === 1 && e === 0) score = 0;
    if (score > bestScore) { bestScore = score; best = cmapOff + off; }
  }
  if (bestScore < 0) throw new Error('font cmap has no usable subtable');

  const out = [];
  const format = dv.getUint16(best);
  if (format === 4) {
    const segX2 = dv.getUint16(best + 6);
    const ends = best + 14, starts = ends + segX2 + 2;
    const deltas = starts + segX2, ranges = deltas + segX2;
    for (let s = 0; s < segX2 / 2; s++) {
      const end = dv.getUint16(ends + s * 2), start = dv.getUint16(starts + s * 2);
      if (start === 0xffff) continue;
      const delta = dv.getUint16(deltas + s * 2), ro = dv.getUint16(ranges + s * 2);
      for (let c = start; c <= end && c !== 0x10000; c++) {
        let g;
        if (ro === 0) g = (c + delta) & 0xffff;
        else {
          const gi = ranges + s * 2 + ro + (c - start) * 2;
          if (gi + 1 >= font.byteLength) continue;
          g = dv.getUint16(gi);
          if (g) g = (g + delta) & 0xffff;
        }
        if (g) out.push(c);
      }
    }
  } else if (format === 12) {
    const groups = dv.getUint32(best + 12);
    for (let i = 0; i < groups; i++) {
      const p = best + 16 + i * 12;
      const start = dv.getUint32(p), end = dv.getUint32(p + 4);
      for (let c = start; c <= end; c++) out.push(c);
    }
  } else if (format === 6) {
    const first = dv.getUint16(best + 6), count = dv.getUint16(best + 8);
    for (let i = 0; i < count; i++) if (dv.getUint16(best + 10 + i * 2)) out.push(first + i);
  } else if (format === 0) {
    for (let c = 0; c < 256; c++) if (font[best + 6 + c]) out.push(c);
  } else {
    throw new Error('unsupported cmap format ' + format);
  }
  return [...new Set(out)].sort((a, b) => a - b);
}

// Pack a glyph's rows of bits into little-endian 32-bit words, LSB first,
// with no per-row padding — the layout the firmware's glyph reader expects.
function packGlyph(glyph) {
  const { width, height, bitmap } = glyph;
  const stride = Math.ceil(width / 8);
  const total = width * height;
  const words = Math.ceil(total / 32);
  const out = new Uint8Array(words * 4);
  const dv = new DataView(out.buffer);
  for (let i = 0; i < total; i++) {
    const x = i % width, y = (i / width) | 0;
    if (!(bitmap[y * stride + (x >> 3)] & (0x80 >> (x & 7)))) continue;
    const w = (i / 32) | 0;
    dv.setUint32(w * 4, (dv.getUint32(w * 4, true) | (1 << (i % 32))) >>> 0, true);
  }
  return out;
}

// A BDF is a bitmap font: the glyphs are already rendered at one fixed
// size, so there is nothing to rasterize. The canvas font API only accepts
// sfnt outlines and rejects these outright, which is why they need their
// own reader — the rows come out in exactly the layout packGlyph wants.
export function isBdf(data) {
  return data.length > 9 && String.fromCharCode(...data.subarray(0, 9)) === 'STARTFONT';
}

export function parseBdf(data) {
  const glyphs = new Map();
  const lines = new TextDecoder('latin1').decode(data).split(/\r?\n/);
  let cp = -1, advance = 0, box = null, rows = null;
  for (const line of lines) {
    if (rows) {
      if (line.startsWith('ENDCHAR')) {
        const [w, h, xoff, yoff] = box;
        const stride = Math.ceil(w / 8);
        const bitmap = new Uint8Array(stride * h);
        for (let y = 0; y < Math.min(h, rows.length); y++) {
          const hex = rows[y].trim();
          for (let b = 0; b < stride && b * 2 < hex.length; b++) {
            bitmap[y * stride + b] = parseInt(hex.substr(b * 2, 2), 16) || 0;
          }
        }
        // yoff places the bottom row relative to the baseline; the PFO
        // wants the distance from the baseline up to the first row.
        if (cp >= 0) glyphs.set(cp, { bitmap, width: w, height: h,
                                      left: xoff, top: yoff + h, advance });
        rows = null; box = null; cp = -1;
      } else {
        rows.push(line);
      }
      continue;
    }
    const [key, ...rest] = line.trim().split(/\s+/);
    if (key === 'ENCODING') cp = parseInt(rest[0], 10);
    else if (key === 'DWIDTH') advance = parseInt(rest[0], 10) || 0;
    else if (key === 'BBX') box = rest.slice(0, 4).map((n) => parseInt(n, 10) || 0);
    else if (key === 'BITMAP') rows = [];
  }
  return glyphs;
}

async function makeFont(fontData, name, def, rasterizeGlyph) {
  const height = def.pixelHeight || fontHeightFromName(name);
  const baseline = def.extended ? fontHeightFromName(name) : height;
  const maxGlyphs = def.extended ? MAX_GLYPHS_EXTENDED : MAX_GLYPHS;
  const regex = (def.characterRegex && def.characterRegex !== '.*')
    ? new RegExp(def.characterRegex) : null;
  const allowed = def.characterList ? new Set(def.characterList) : null;
  const tracking = def.trackingAdjust || 0;
  // A bitmap font carries its own glyphs and its own character set.
  const bdf = isBdf(fontData) ? parseBdf(fontData) : null;
  const rasterize = bdf ? async (d, cp) => bdf.get(cp) || null : rasterizeGlyph;
  const codepoints = bdf ? [...bdf.keys()].sort((a, b) => a - b)
                         : cmapCodepoints(fontData);

  const inSubset = (cp) => {
    if (cp === WILDCARD_CODEPOINT || cp === ELLIPSIS_CODEPOINT) return true;
    if (regex && !regex.test(String.fromCodePoint(cp))) return false;
    if (allowed && !allowed.has(cp)) return false;
    return cp >= 0x20;
  };

  const glyphTable = [new Uint8Array(4)];   // offset 0 means "no glyph"
  const entries = [];
  let nextOffset = 4;
  let numGlyphs = 0;
  let codepointBytes = 2;

  const addGlyph = async (cp) => {
    const g = await rasterize(fontData, cp, height);
    const packed = (g && g.width && g.height) ? packGlyph(g) : new Uint8Array(0);
    const head = new Uint8Array(5);
    const dv = new DataView(head.buffer);
    head[0] = g ? g.width : 0;
    head[1] = g ? g.height : 0;
    dv.setInt8(2, g ? clampI8(g.left) : 0);
    dv.setInt8(3, g ? clampI8(baseline - g.top) : 0);
    dv.setInt8(4, g ? clampI8(g.advance + tracking) : 0);
    const bits = new Uint8Array(head.length + packed.length);
    bits.set(head, 0); bits.set(packed, head.length);
    const offset = nextOffset;
    glyphTable.push(bits);
    nextOffset += bits.length;
    if (cp > 0xffff) codepointBytes = 4;
    numGlyphs++;
    return offset;
  };

  entries.push([WILDCARD_CODEPOINT, await addGlyph(WILDCARD_CODEPOINT)]);
  for (const cp of codepoints) {
    if (numGlyphs > maxGlyphs) break;
    if (cp === WILDCARD_CODEPOINT || !inSubset(cp)) continue;
    entries.push([cp, await addGlyph(cp)]);
  }

  const glyphBytes = glyphTable.reduce((s, g) => s + g.length, 0);
  let features = 0, offsetBytes = 4;
  if (glyphBytes < 65536) { features |= FEATURE_OFFSET_16; offsetBytes = 2; }

  entries.sort((a, b) => a[0] - b[0]);
  const buckets = Array.from({ length: HASH_TABLE_SIZE }, () => []);
  for (const [cp, off] of entries) {
    const rec = new Uint8Array(codepointBytes + offsetBytes);
    const dv = new DataView(rec.buffer);
    if (codepointBytes === 4) dv.setUint32(0, cp, true); else dv.setUint16(0, cp, true);
    if (offsetBytes === 4) dv.setUint32(codepointBytes, off, true);
    else dv.setUint16(codepointBytes, off, true);
    buckets[cp % HASH_TABLE_SIZE].push(rec);
  }

  const hashTable = new Uint8Array(HASH_TABLE_SIZE * 4);
  const hdv = new DataView(hashTable.buffer);
  let acc = 0;
  for (let i = 0; i < HASH_TABLE_SIZE; i++) {
    hashTable[i * 4] = i;
    hashTable[i * 4 + 1] = buckets[i].length;
    hdv.setUint16(i * 4 + 2, acc, true);
    acc += buckets[i].length * (offsetBytes + codepointBytes);
  }

  const info = new Uint8Array(10);
  const idv = new DataView(info.buffer);
  info[0] = 3;                       // version
  info[1] = height;
  idv.setUint16(2, numGlyphs, true);
  idv.setUint16(4, WILDCARD_CODEPOINT, true);
  info[6] = HASH_TABLE_SIZE;
  info[7] = codepointBytes;
  info[8] = info.length;
  info[9] = features;

  return concat([info, hashTable, ...buckets.flat(), ...glyphTable]);
}

const clampI8 = (v) => Math.max(-128, Math.min(127, Math.round(v)));

function fontHeightFromName(name) {
  const m = /([0-9]+)/.exec(name);
  if (m) return parseInt(m[1], 10);
  if (name === 'FONT_FALLBACK' || name === 'FONT_FALLBACK_INTERNAL') return 14;
  throw new Error(`font ${name}: no height found in name`);
}

// --- resource pack (pbpack.py) -------------------------------------------

const TABLE_SIZE = 256;                 // app packs; system packs use 768
const MANIFEST_SIZE = 12;

export function makePbpack(contents) {
  // Deduplicate identical resources, as add_resource() does.
  const unique = [];
  const keys = new Map();
  const entries = contents.map((data) => {
    const key = keyOf(data);
    let idx = keys.get(key);
    if (idx === undefined) { idx = unique.length; keys.set(key, idx); unique.push(data); }
    return { contentIndex: idx, length: data.length, crc: legacyDefectiveCrc(data), offset: -1 };
  });
  if (entries.length > TABLE_SIZE) {
    throw new Error(`too many resources: ${entries.length} > ${TABLE_SIZE}`);
  }

  // Offsets are assigned back to front so the last table entry always ends
  // at the end of the pack — the firmware sizes the pack from it.
  let offset = unique.reduce((s, c) => s + c.length, 0);
  for (let i = entries.length - 1; i >= 0; i--) {
    if (entries[i].offset !== -1) continue;
    offset -= entries[i].length;
    for (const e of entries) if (e.contentIndex === entries[i].contentIndex) e.offset = offset;
  }

  const body = [];
  const written = new Set();
  for (const e of [...entries].sort((a, b) => a.offset - b.offset)) {
    if (written.has(e.contentIndex)) continue;
    written.add(e.contentIndex);
    body.push(unique[e.contentIndex]);
  }
  const content = concat(body);

  const head = new Uint8Array(MANIFEST_SIZE + TABLE_SIZE * 16);
  const dv = new DataView(head.buffer);
  dv.setUint32(0, entries.length, true);
  dv.setUint32(4, legacyDefectiveCrc(content), true);
  dv.setUint32(8, 0, true);                     // timestamp is unused
  entries.forEach((e, i) => {
    const p = MANIFEST_SIZE + i * 16;
    dv.setUint32(p, i + 1, true);
    dv.setUint32(p + 4, e.offset, true);
    dv.setUint32(p + 8, e.length, true);
    dv.setUint32(p + 12, e.crc, true);
  });
  return concat([head, content]);
}

function keyOf(data) {
  let s = data.length + ':';
  for (let i = 0; i < data.length; i += 1 + (data.length >> 9)) s += data[i].toString(36);
  return s + ':' + legacyDefectiveCrc(data);
}

// --- driver ---------------------------------------------------------------

// Pick the ~tagged variant of a resource for this platform, following
// find_resource_filename.py: a candidate's tags must all be valid for the
// platform, and the one sharing the most tags wins.
export function findTaggedFile(wanted, available, tags) {
  const dot = wanted.lastIndexOf('.');
  const base = dot < 0 ? wanted : wanted.slice(0, dot);
  const ext = dot < 0 ? '' : wanted.slice(dot);
  const valid = new Set(tags);
  let best = null, bestScore = -1, tied = [];
  for (const cand of available) {
    if (!cand.startsWith(base) || !cand.endsWith(ext)) continue;
    const middle = cand.slice(base.length, cand.length - ext.length);
    if (middle && !middle.startsWith('~')) continue;
    const candTags = middle ? middle.slice(1).split('~') : [];
    if (candTags.some((t) => !valid.has(t))) continue;
    const score = candTags.length;
    if (score > bestScore) { best = cand; bestScore = score; tied = [cand]; }
    else if (score === bestScore) tied.push(cand);
  }
  if (tied.length > 1) {
    throw new Error(`ambiguous resource "${wanted}": ${tied.join(', ')} all match equally`);
  }
  return best;
}

// media:    the package.json pebble.resources.media array
// readFile: async (path) => Uint8Array, relative to the project's resources/
// returns {pbpack, resourceNames} — names in resource_ids.auto.h order.
export async function buildResources(media, readFile, platform, rasterizeGlyph, log = () => {}) {
  const plat = PLATFORMS[platform];
  if (!plat) throw new Error(`unknown platform "${platform}"`);
  const isColor = plat.color;
  const paletteName = isColor ? 'pebble64' : 'pebble2';
  const contents = [];
  const resourceNames = [];

  for (const def of media) {
    if (def.targetPlatforms && !def.targetPlatforms.includes(platform)) continue;
    const data = await readFile(def.file);
    const type = def.type;

    const push = (n) => resourceNames.push({ name: n, aliases: def.aliases || [] });
    if (type === 'raw') {
      push(def.name);
      contents.push(data);
    } else if (type === 'font') {
      if (/\.pbf$/i.test(def.file)) {
        push(def.name);
        contents.push(data);
      } else {
        push(def.name);
        contents.push(await makeFont(data, def.name, def, rasterizeGlyph));
      }
    } else if (type === 'png-trans') {
      // Two 1-bit bitmaps: the white pixels, then the black ones.
      const image = await decodePng(data);
      for (const [suffix, colorMap] of [['_WHITE', WHITE_COLOR_MAP], ['_BLACK', BLACK_COLOR_MAP]]) {
        resourceNames.push({ name: def.name + suffix, aliases: (def.aliases || []).map((a) => a + suffix) });
        contents.push(makePbi(image, { format: 'bw', colorMap }));
      }
    } else if (type === 'pbi' || type === 'pbi8') {
      const image = await decodePng(data);
      const format = (type === 'pbi8' && isColor) ? 'color' : 'bw';
      push(def.name);
      contents.push(makePbi(image, { format, paletteName }));
    } else if (type === 'bitmap') {
      push(def.name);
      contents.push(await makeBitmap(data, def, isColor, paletteName));
    } else if (type === 'png') {
      push(def.name);
      contents.push(await pebblePng(await decodePng(data), paletteName));
    } else {
      throw new Error(`unsupported resource type "${type}" (${def.name})`);
    }
    log(`  ${def.name}: ${type} -> ${contents[contents.length - 1].length} bytes`);
  }
  return { pbpack: makePbpack(contents), resourceNames };
}

// resource_generator_bitmap.py: memoryFormat picks the in-RAM layout,
// storageFormat picks PNG or PBI on flash.
async function makeBitmap(data, def, isColor, paletteName) {
  const image = await decodePng(data);
  const { bitdepth } = analysePng(image, paletteName);
  let memoryFormat = (def.memoryFormat || 'smallest').toLowerCase();
  let storageFormat = def.storageFormat;
  if (!storageFormat && def.spaceOptimization) {
    storageFormat = { storage: 'png', memory: 'pbi' }[def.spaceOptimization];
    if (!storageFormat) throw new Error(`${def.name}: invalid spaceOptimization`);
  }
  if (!storageFormat) storageFormat = 'png';

  if (memoryFormat === 'smallest') {
    memoryFormat = bitdepth <= 4 ? 'smallestpalette' : '8bit';
  }
  if (memoryFormat.includes('palette')) {
    if (memoryFormat === 'smallestpalette') {
      if (bitdepth > 4) throw new Error(`${def.name}: too many colours for a palettised image`);
      memoryFormat = `${bitdepth}bitpalette`;
    }
    const bits = parseInt(/^(\d+)bitpalette$/.exec(memoryFormat)[1], 10);
    if (bits < bitdepth) throw new Error(`${def.name}: requires at least ${bitdepth} bits`);
    if (bits > 2 && !isColor) throw new Error(`${def.name}: >2 bits on a b&w platform`);
    return storageFormat === 'pbi'
      ? makePbi(image, { format: 'color', bitdepth: bits, crop: false, paletteName })
      : pebblePng(image, paletteName, bits);
  }
  if (memoryFormat === '1bit') return makePbi(image, { format: 'bw', crop: false });
  if (memoryFormat === '8bit') {
    if (!isColor) throw new Error(`${def.name}: 8-bit on a b&w platform`);
    return storageFormat === 'pbi'
      ? makePbi(image, { format: 'color', bitdepth: 8, crop: false, paletteName })
      : pebblePng(image, paletteName);
  }
  throw new Error(`${def.name}: invalid memoryFormat ${def.memoryFormat}`);
}

export { makePbi, pebblePng, makeFont, analysePng };
