// In-browser post-link packaging for Pebble apps: the equivalents of
// arm-none-eabi-objcopy -O binary, the SDK's inject_metadata.py and the
// per-platform pieces of mkbundle.py, so a freshly linked ELF can be
// turned into a pebble-app.bin + manifest.json + .pbw without native
// binutils. Field offsets follow PebbleProcessInfo struct version 16
// (src/fw/process_management/pebble_process_info.h).

import { legacyDefectiveCrc } from '../app-install.js';

// --- ELF32 little-endian parsing -----------------------------------------

const SHT_SYMTAB = 2;
const SHT_NOBITS = 8;
const SHT_REL = 9;
const SHF_ALLOC = 0x2;

function parseElf(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (dv.getUint32(0, false) !== 0x7f454c46 || bytes[4] !== 1 || bytes[5] !== 1) {
    throw new Error('not a 32-bit little-endian ELF');
  }
  const shoff = dv.getUint32(0x20, true);
  const shentsize = dv.getUint16(0x2e, true);
  const shnum = dv.getUint16(0x30, true);
  const shstrndx = dv.getUint16(0x32, true);

  const sections = [];
  for (let i = 0; i < shnum; i++) {
    const o = shoff + i * shentsize;
    sections.push({
      nameOff: dv.getUint32(o + 0, true),
      type: dv.getUint32(o + 4, true),
      flags: dv.getUint32(o + 8, true),
      addr: dv.getUint32(o + 12, true),
      off: dv.getUint32(o + 16, true),
      size: dv.getUint32(o + 20, true),
      link: dv.getUint32(o + 24, true),
    });
  }
  const strAt = (strtab, off) => {
    let end = strtab.off + off;
    while (end < bytes.length && bytes[end] !== 0) end++;
    return new TextDecoder().decode(bytes.subarray(strtab.off + off, end));
  };
  for (const s of sections) s.name = strAt(sections[shstrndx], s.nameOff);
  return { bytes, dv, sections, strAt };
}

// Address of a defined symbol, like `arm-none-eabi-nm` + grep would find
// it: nm clears the Thumb bit on ARM function symbols.
function symbolAddr(elf, name) {
  const symtab = elf.sections.find((s) => s.type === SHT_SYMTAB);
  if (!symtab) throw new Error('ELF has no .symtab');
  const strtab = elf.sections[symtab.link];
  for (let o = symtab.off; o < symtab.off + symtab.size; o += 16) {
    const shndx = elf.dv.getUint16(o + 14, true);
    if (shndx === 0) continue; // undefined
    if (elf.strAt(strtab, elf.dv.getUint32(o, true)) === name) {
      const value = elf.dv.getUint32(o + 4, true);
      const isFunc = (elf.bytes[o + 12] & 0x0f) === 2; // STT_FUNC
      return isFunc ? value & ~1 : value;
    }
  }
  throw new Error(`Could not locate symbol <${name}> in binary! ` +
    'Failed to inject app metadata');
}

// --- objcopy -O binary ----------------------------------------------------

const OBJCOPY_REMOVED = new Set(['.stack', '.priv_bss', '.bss', '.retained']);

// arm-none-eabi-objcopy -S -R .stack -R .priv_bss -R .bss -R .retained
// -O binary: allocated sections with contents, laid out at their load
// address relative to the lowest one, gaps zero-filled.
export function objcopyToBin(elfBytes) {
  const elf = parseElf(elfBytes);
  const keep = elf.sections.filter((s) =>
    (s.flags & SHF_ALLOC) && s.type !== SHT_NOBITS && s.size > 0 &&
    !OBJCOPY_REMOVED.has(s.name));
  if (keep.length === 0) return new Uint8Array(0);
  const base = Math.min(...keep.map((s) => s.addr));
  const end = Math.max(...keep.map((s) => s.addr + s.size));
  const out = new Uint8Array(end - base);
  for (const s of keep) {
    out.set(elfBytes.subarray(s.off, s.off + s.size), s.addr - base);
  }
  return out;
}

// --- inject_metadata.py ---------------------------------------------------

// PebbleProcessInfo field offsets (struct version 16).
const LOAD_SIZE_ADDR = 0x0e;
const OFFSET_ADDR = 0x10; // entry point
const CRC_ADDR = 0x14;
const JUMP_TABLE_ADDR = 0x5c;
const FLAGS_ADDR = 0x60;
const NUM_RELOC_ENTRIES_ADDR = 0x64;
const RESOURCE_CRC_ADDR = 0x78;
const RESOURCE_TIMESTAMP_ADDR = 0x7c;
const VIRTUAL_SIZE_ADDR = 0x80;
const STRUCT_SIZE_BYTES = 0x82;

const PROCESS_INFO_ALLOW_JS = 1 << 3;
const PROCESS_INFO_HAS_WORKER = 1 << 4;

const MAX_APP_BINARY_SIZE = 0x10000;
const MAX_PROCESS_INFO_SIZE_FIELD = 0xffff;

// Locations needing the load-address offset applied at install time:
// every .rel.data* relocation target, plus each word of .got.
function relocateEntries(elf) {
  const entries = [];
  for (const s of elf.sections) {
    if (s.type !== SHT_REL || !s.name.startsWith('.rel.data')) continue;
    for (let o = s.off; o < s.off + s.size; o += 8) {
      entries.push(elf.dv.getUint32(o, true));
    }
  }
  const got = elf.sections.find((s) => s.name === '.got');
  if (got) {
    for (let a = got.addr; a < got.addr + got.size; a += 4) entries.push(a);
  }
  return entries;
}

// Static memory usage: the highest address any allocated section reaches.
// inject_metadata.py takes the end of .bss because GNU ld folds .got into
// .data and leaves .bss last; lld gives .got its own section after .bss,
// so stopping at .bss understates the footprint and the firmware then
// refuses to launch ("App image exceeds virtual size").
function virtualSize(elf) {
  let end = 0;
  for (const s of elf.sections) {
    if (!(s.flags & SHF_ALLOC)) continue;
    end = Math.max(end, s.addr + s.size);
  }
  if (end === 0) throw new Error('no allocated sections; cannot size the app');
  return end;
}

// Content CRC of an app resource pack: unique contents in offset order,
// exactly like pbpack.py ResourcePack.get_content_crc().
function pbpackContentCrc(pack) {
  const dv = new DataView(pack.buffer, pack.byteOffset, pack.byteLength);
  const numFiles = dv.getUint32(0, true);
  const contentStart = 12 + 256 * 16; // app packs have a 256-entry table
  const unique = new Map(); // "offset,length" -> [offset, length]
  for (let n = 0; n < numFiles; n++) {
    const o = 12 + n * 16;
    const fileId = dv.getUint32(o, true);
    if (fileId === 0) break;
    if (fileId !== n + 1) throw new Error(`bad pbpack file id ${fileId}`);
    const off = dv.getUint32(o + 4, true);
    const len = dv.getUint32(o + 8, true);
    unique.set(`${off},${len}`, [off, len]);
  }
  const parts = [...unique.values()].sort((a, b) => a[0] - b[0]);
  const total = parts.reduce((s, [, len]) => s + len, 0);
  const content = new Uint8Array(total);
  let at = 0;
  for (const [off, len] of parts) {
    content.set(pack.subarray(contentStart + off, contentStart + off + len), at);
    at += len;
  }
  return legacyDefectiveCrc(content);
}

// Port of inject_metadata.py inject_metadata(): fills the header of the
// objcopy output and appends the relocation table. Returns a new buffer;
// pbpackBytes may be null (resource_crc 0).
export function injectMetadata(binBytes, elfBytes, pbpackBytes,
    { timestamp, allowJs = false, hasWorker = false,
      maxBinarySize = MAX_APP_BINARY_SIZE } = {}) {
  const elf = parseElf(elfBytes);

  let entryPoint;
  try {
    entryPoint = symbolAddr(elf, 'main');
  } catch {
    throw new Error('Missing app entry point! Must be `int main(void) { ... }`');
  }
  const jumpTable = symbolAddr(elf, 'pbl_table_addr');
  const relocs = relocateEntries(elf);

  const loadSize = binBytes.length;
  const totalSize = loadSize + relocs.length * 4;
  if (totalSize > maxBinarySize) {
    throw new Error(`App image size is ${totalSize} (app ${loadSize} ` +
      `relocation table ${relocs.length * 4}). Must be smaller than ` +
      `${maxBinarySize} bytes`);
  }
  if (loadSize > MAX_PROCESS_INFO_SIZE_FIELD) {
    throw new Error(`App load size is ${loadSize} bytes; load_size is a uint16_t`);
  }
  const vsize = virtualSize(elf);
  if (vsize > MAX_PROCESS_INFO_SIZE_FIELD) {
    throw new Error(`App virtual size is ${vsize} bytes; virtual_size is a uint16_t`);
  }

  const out = new Uint8Array(totalSize);
  out.set(binBytes, 0);
  const dv = new DataView(out.buffer);

  // CRC over everything past the header, as found in the raw binary.
  const appCrc = legacyDefectiveCrc(binBytes.subarray(STRUCT_SIZE_BYTES));

  let flags = dv.getUint32(FLAGS_ADDR, true);
  if (allowJs) flags |= PROCESS_INFO_ALLOW_JS;
  if (hasWorker) flags |= PROCESS_INFO_HAS_WORKER;

  dv.setUint16(LOAD_SIZE_ADDR, loadSize, true);
  dv.setUint32(OFFSET_ADDR, entryPoint, true);
  dv.setUint32(CRC_ADDR, appCrc, true);
  dv.setUint32(RESOURCE_CRC_ADDR, pbpackBytes ? pbpackContentCrc(pbpackBytes) : 0, true);
  dv.setUint32(RESOURCE_TIMESTAMP_ADDR, timestamp, true);
  dv.setUint32(JUMP_TABLE_ADDR, jumpTable, true);
  dv.setUint32(FLAGS_ADDR, flags, true);
  dv.setUint32(NUM_RELOC_ENTRIES_ADDR, relocs.length, true);
  dv.setUint16(VIRTUAL_SIZE_ADDR, vsize, true);

  // Relocation table past the end of the loaded image.
  for (let i = 0; i < relocs.length; i++) {
    dv.setUint32(loadSize + i * 4, relocs[i], true);
  }
  return out;
}

// --- mkbundle.py manifest -------------------------------------------------

// The per-platform manifest.json object as make_watchapp_bundle() emits it
// for a watchapp (application + resources; the pbw crc fields use the
// legacy defective CRC, same as stm32_crc.crc32). jsPresent does not
// change this manifest — JS files sit at the pbw top level — but is
// accepted for symmetry with the bundler inputs.
export function makeManifest({ appBin, pbpack, timestamp, sdkVersion,
    jsPresent = false } = {}) {  // eslint-disable-line no-unused-vars
  const manifestVersion =
    (sdkVersion.major === 5 && sdkVersion.minor < 20) ? 1 : 2;
  return {
    manifestVersion,
    generatedAt: timestamp,
    generatedBy: '',
    debug: {},
    application: {
      timestamp,
      sdk_version: { major: sdkVersion.major, minor: sdkVersion.minor },
      name: 'pebble-app.bin',
      size: appBin.length,
      crc: legacyDefectiveCrc(appBin),
    },
    resources: {
      name: 'app_resources.pbpack',
      timestamp,
      size: pbpack.length,
      crc: legacyDefectiveCrc(pbpack),
    },
    type: 'application',
  };
}

// --- minimal ZIP writer ---------------------------------------------------

// Standard (IEEE, reflected) CRC-32 as used by ZIP — not the legacy
// defective CRC used everywhere else in the Pebble formats.
const ZIP_CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xedb88320 ^ (c >>> 1)) : (c >>> 1);
    t[n] = c >>> 0;
  }
  return t;
})();

function zipCrc32(data) {
  let c = 0xffffffff;
  for (let i = 0; i < data.length; i++) {
    c = ZIP_CRC_TABLE[(c ^ data[i]) & 0xff] ^ (c >>> 8);
  }
  return (c ^ 0xffffffff) >>> 0;
}

function dosDateTime(date) {
  const d = date instanceof Date ? date : new Date((date || 0) * 1000);
  const year = Math.max(d.getFullYear(), 1980);
  return {
    time: (d.getHours() << 11) | (d.getMinutes() << 5) | (d.getSeconds() >> 1),
    date: ((year - 1980) << 9) | ((d.getMonth() + 1) << 5) | d.getDate(),
  };
}

// entries: [{ name, data: Uint8Array, date: Date|unix-seconds }].
// All entries are STORED (no compression), which is all a .pbw needs.
export function makePbw(entries) {
  const enc = new TextEncoder();
  const locals = [];
  const centrals = [];
  let offset = 0;

  for (const e of entries) {
    if (!e.name) throw new Error('makePbw: entry has no name');
    const name = enc.encode(e.name);
    const crc = zipCrc32(e.data);
    const { time, date } = dosDateTime(e.date);

    const lh = new Uint8Array(30 + name.length);
    const ldv = new DataView(lh.buffer);
    ldv.setUint32(0, 0x04034b50, true);
    ldv.setUint16(4, 20, true);            // version needed
    ldv.setUint16(6, 0, true);             // flags
    ldv.setUint16(8, 0, true);             // method: STORED
    ldv.setUint16(10, time, true);
    ldv.setUint16(12, date, true);
    ldv.setUint32(14, crc, true);
    ldv.setUint32(18, e.data.length, true); // compressed size
    ldv.setUint32(22, e.data.length, true); // uncompressed size
    ldv.setUint16(26, name.length, true);
    ldv.setUint16(28, 0, true);            // extra length
    lh.set(name, 30);
    locals.push(lh, e.data);

    const ch = new Uint8Array(46 + name.length);
    const cdv = new DataView(ch.buffer);
    cdv.setUint32(0, 0x02014b50, true);
    cdv.setUint16(4, 20, true);            // version made by
    cdv.setUint16(6, 20, true);            // version needed
    cdv.setUint16(12, time, true);
    cdv.setUint16(14, date, true);
    cdv.setUint32(16, crc, true);
    cdv.setUint32(20, e.data.length, true);
    cdv.setUint32(24, e.data.length, true);
    cdv.setUint16(28, name.length, true);
    cdv.setUint32(42, offset, true);       // local header offset
    ch.set(name, 46);
    centrals.push(ch);

    offset += lh.length + e.data.length;
  }

  const cdSize = centrals.reduce((s, c) => s + c.length, 0);
  const eocd = new Uint8Array(22);
  const edv = new DataView(eocd.buffer);
  edv.setUint32(0, 0x06054b50, true);
  edv.setUint16(8, entries.length, true);  // entries on this disk
  edv.setUint16(10, entries.length, true); // total entries
  edv.setUint32(12, cdSize, true);
  edv.setUint32(16, offset, true);         // central directory offset

  const parts = [...locals, ...centrals, eocd];
  const out = new Uint8Array(parts.reduce((s, p) => s + p.length, 0));
  let at = 0;
  for (const p of parts) { out.set(p, at); at += p.length; }
  return out;
}
