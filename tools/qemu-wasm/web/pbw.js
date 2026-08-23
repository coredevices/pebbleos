// Minimal .pbw (zip) reader for the browser emulator.
// Uses the native DecompressionStream — no external libraries.

async function inflateRaw(bytes) {
  const ds = new DecompressionStream('deflate-raw');
  const stream = new Blob([bytes]).stream().pipeThrough(ds);
  const buf = await new Response(stream).arrayBuffer();
  return new Uint8Array(buf);
}

// Parse a zip archive into {name: Uint8Array}.
export async function unzip(data) {
  const u8 = data instanceof Uint8Array ? data : new Uint8Array(data);
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);

  // End-of-central-directory: scan backwards for PK\x05\x06
  let eocd = -1;
  for (let i = u8.length - 22; i >= Math.max(0, u8.length - 22 - 65536); i--) {
    if (dv.getUint32(i, true) === 0x06054b50) { eocd = i; break; }
  }
  if (eocd < 0) throw new Error('not a zip file (no end-of-central-directory)');

  const count = dv.getUint16(eocd + 10, true);
  let off = dv.getUint32(eocd + 16, true);
  const files = {};
  const td = new TextDecoder();

  for (let i = 0; i < count; i++) {
    if (dv.getUint32(off, true) !== 0x02014b50) throw new Error('bad central directory');
    const method = dv.getUint16(off + 10, true);
    const csize = dv.getUint32(off + 20, true);
    const usize = dv.getUint32(off + 24, true);
    const nameLen = dv.getUint16(off + 28, true);
    const extraLen = dv.getUint16(off + 30, true);
    const commentLen = dv.getUint16(off + 32, true);
    const lho = dv.getUint32(off + 42, true);
    const name = td.decode(u8.subarray(off + 46, off + 46 + nameLen));

    // Local header: skip its own (possibly different) name/extra lengths
    if (dv.getUint32(lho, true) !== 0x04034b50) throw new Error('bad local header for ' + name);
    const lNameLen = dv.getUint16(lho + 26, true);
    const lExtraLen = dv.getUint16(lho + 28, true);
    const dataOff = lho + 30 + lNameLen + lExtraLen;
    const raw = u8.subarray(dataOff, dataOff + csize);

    if (!name.endsWith('/')) {
      if (method === 0) files[name] = raw.slice();
      else if (method === 8) {
        const out = await inflateRaw(raw);
        if (out.length !== usize) throw new Error(`bad inflate size for ${name}`);
        files[name] = out;
      } else throw new Error(`unsupported zip method ${method} for ${name}`);
    }
    off += 46 + nameLen + extraLen + commentLen;
  }
  return files;
}

// Pick the best platform directory in a pbw for a given watch platform.
// Emery runs emery-native apps first, then falls back through compatible
// older platforms (basalt bezel mode, then legacy root/aplite layout).
// Which platform directories a board will accept, best first. A board's
// own directory has to come first: store apps predate flint and gabbro so
// they only ever carry the older ones, but an app built here does have a
// matching directory. Round boards fall back to chalk, rect ones to the
// 144x168 platforms.
const PLATFORM_PREFERENCE = {
  emery: ['emery', 'basalt', 'diorite', 'aplite'],
  basalt: ['basalt', 'aplite'],
  chalk: ['chalk'],
  diorite: ['diorite', 'aplite'],
  flint: ['flint', 'diorite', 'aplite', 'basalt'],
  gabbro: ['gabbro', 'chalk'],
};

export async function parsePbw(data, platform = 'emery') {
  const files = await unzip(data);
  const td = new TextDecoder();
  if (!files['appinfo.json']) throw new Error('pbw has no appinfo.json');
  const appinfo = JSON.parse(td.decode(files['appinfo.json']));

  const pick = (name) => {
    for (const dir of PLATFORM_PREFERENCE[platform] || [platform]) {
      if (files[`${dir}/${name}`]) return { data: files[`${dir}/${name}`], dir };
    }
    return files[name] ? { data: files[name], dir: '(root)' } : null;
  };

  const binary = pick('pebble-app.bin');
  if (!binary) throw new Error(`pbw has no app binary for platform "${platform}" ` +
    `(contains: ${Object.keys(files).join(', ')})`);
  const resources = pick('app_resources.pbpack');
  const worker = pick('pebble-worker.bin');

  return {
    appinfo,
    platformDir: binary.dir,
    binary: binary.data,
    resources: resources && resources.dir === binary.dir ? resources.data : (resources?.data ?? null),
    worker: worker && worker.dir === binary.dir ? worker.data : null,
    // PebbleKit JS lives at the pbw root (both single-JS and multiJS builds)
    js: files['pebble-js-app.js'] ? td.decode(files['pebble-js-app.js']) : null,
  };
}
