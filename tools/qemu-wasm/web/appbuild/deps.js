// Fetch a project's Pebble SDK packages from npm, the browser stand-in for
// `npm install` plus the SDK's package handling (waflib/extras/sdk_helpers
// process_package + pebble_sdk_common setup_pebble_c).
//
// A package arrives in one of two shapes: a library with a dist.zip holding
// prebuilt per-platform binaries and headers, or a plain JS package (Clay
// and friends) that only the PebbleKit JS bundle needs.
import { unzip } from '../pbw.js';

const td = new TextDecoder();
const REGISTRY = 'https://registry.npmjs.org/';

// Most SDK libraries predate the newest watches, so their dist.zip has no
// binaries for them. Cortex-M thumb code is compatible across the line, so
// stand in the closest sibling: same shape and color first.
const BINARY_FALLBACK = {
  emery: ['basalt', 'diorite', 'aplite'],
  flint: ['diorite', 'aplite', 'basalt'],
  gabbro: ['chalk', 'basalt'],
  diorite: ['aplite', 'basalt'],
  basalt: ['diorite'],
};

// --- tar (npm tarballs are gzipped ustar) ---------------------------------

function untar(bytes) {
  const files = {};
  const name = (b, o, n) => {
    let s = '';
    for (let i = o; i < o + n && b[i]; i++) s += String.fromCharCode(b[i]);
    return s;
  };
  for (let p = 0; p + 512 <= bytes.length;) {
    const path = name(bytes, p, 100);
    if (!path) { p += 512; continue; }
    const sizeField = name(bytes, p + 124, 12).trim();
    const size = parseInt(sizeField, 8) || 0;
    const type = String.fromCharCode(bytes[p + 156] || 0x30);
    const prefix = name(bytes, p + 345, 155);
    const full = prefix ? `${prefix}/${path}` : path;
    p += 512;
    if (type === '0' || type === '\0') files[full] = bytes.subarray(p, p + size);
    p += Math.ceil(size / 512) * 512;
  }
  return files;
}

async function gunzip(bytes) {
  const s = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
  return new Uint8Array(await new Response(s).arrayBuffer());
}

// --- npm ------------------------------------------------------------------

// Resolve a dependency range to a concrete tarball. Exact versions are
// honoured; anything else takes the latest, which is what a fresh
// `npm install` would do for the carets these projects use.
const URL_DEP = /^(https?:\/\/|git\+|github:)/;

async function resolvePackage(name, range, get) {
  // npm also accepts a plain tarball URL in place of a version range.
  if (URL_DEP.test(String(range || ''))) {
    return { version: String(range), tarball: String(range), pkg: {} };
  }
  const meta = JSON.parse(td.decode(await get(REGISTRY + name.replace('/', '%2f'))));
  const versions = meta.versions || {};
  let version = null;
  if (range && versions[range]) version = range;
  if (!version) {
    const clean = String(range || '').replace(/^[\^~=v\s]+/, '');
    if (versions[clean]) version = clean;
  }
  if (!version) version = (meta['dist-tags'] || {}).latest;
  if (!version || !versions[version]) throw new Error(`no usable version of ${name}`);
  return { version, tarball: versions[version].dist.tarball, pkg: versions[version] };
}

// opts: { platforms: string[], get(url) -> Uint8Array, log }
// Returns everything the compiler, linker and JS bundler need. Binaries
// are collected per platform, since one fetch serves every board.
export async function fetchDependencies(rootPkg, { platforms, get, vendored = {}, log = () => {} }) {
  const wanted = { ...(rootPkg.dependencies || {}) };
  const seen = new Set();
  const includes = {};          // path -> bytes, mounted at /deps/include
  const libs = {};              // platform -> [{name, data}]
  for (const p of platforms) libs[p] = [];
  const packages = {};          // npm name -> {files, main, isLibrary}
  const resources = [];         // {def, data} contributed by libraries
  const messageKeys = [];

  const queue = Object.entries(wanted);
  while (queue.length) {
    const [name, range] = queue.shift();
    if (seen.has(name)) continue;
    seen.add(name);

    // A repository that commits its node_modules already has the package;
    // the real SDK just uses what is there, so prefer it over the network.
    let files;
    if (vendored[name]) {
      log(`dependency: ${name} (vendored in the repo)`);
      files = vendored[name];
    } else {
      let resolved;
      try {
        resolved = await resolvePackage(name, range, get);
      } catch (e) {
        throw new Error(`dependency "${name}" could not be resolved: ${e.message}`);
      }
      log(`dependency: ${name}@${resolved.version}`);
      let tar;
      try {
        tar = untar(await gunzip(await get(resolved.tarball)));
      } catch (e) {
        throw new Error(`dependency "${name}" could not be downloaded ` +
          `(${resolved.tarball}): ${e.message}`);
      }
      // npm tarballs put everything under package/
      files = {};
      for (const [p, d] of Object.entries(tar)) {
        if (p.startsWith('package/')) files[p.slice('package/'.length)] = d;
      }
    }
    let meta = {};
    try { meta = JSON.parse(td.decode(files['package.json'])); } catch (e) { /* keep going */ }

    if (files['dist.zip']) {
      // A built SDK library: headers, per-platform archive, JS, resources.
      const dist = await unzip(files['dist.zip']);
      const missing = [];
      const bins = {};              // platform -> [{name, data}] in this dist
      for (const [p, d] of Object.entries(dist)) {
        if (p.startsWith('include/')) includes[p.slice('include/'.length)] = d;
        else if (p.startsWith('binaries/') && p.endsWith('.a')) {
          // A scoped package puts its scope in the path
          // (binaries/emery/@rebble/libclay.a), so match on the directory
          // rather than reconstructing the filename.
          const plat = p.split('/')[1];
          (bins[plat] = bins[plat] || [])
            .push({ name: p.slice(p.lastIndexOf('/') + 1), data: d });
        } else if (p.startsWith('js/')) {
          packages[name] = packages[name] || { files: {}, isLibrary: true };
          packages[name].files[p.slice('js/'.length)] = d;
        } else if (p.startsWith('resources/')) {
          resources.push({ name, path: p.slice('resources/'.length), data: d });
        }
      }
      for (const plat of platforms) {
        let take = plat;
        if (!bins[take]) {
          take = (BINARY_FALLBACK[plat] || []).find((f) => bins[f]);
          if (take) {
            log(`${name} has no ${plat} build; using its ${take} binaries`);
            // The per-platform generated headers travel with the binaries
            // (include/<pkg>/<plat>/…), so alias those over as well.
            for (const [p, d] of Object.entries(includes)) {
              const from = `${name}/${take}/`;
              if (p.startsWith(from) && !includes[`${name}/${plat}/` + p.slice(from.length)]) {
                includes[`${name}/${plat}/` + p.slice(from.length)] = d;
              }
            }
          }
        }
        if (take && bins[take]) libs[plat].push(...bins[take]);
        else if (!libs[plat].length) missing.push(plat);
      }
      if (missing.length) {
        // Several long-abandoned packages ship binaries only for the
        // platforms that existed when they were published. Most of those
        // are JS-only anyway (Clay's archive is an empty 8-byte stub), so
        // carry on: if the app really does need C symbols from it, the
        // link fails straight after with an undefined symbol naming them.
        const have = [...new Set(Object.keys(dist)
          .filter((p) => p.startsWith('binaries/')).map((p) => p.split('/')[1]))];
        log(`warning: ${name} has no build for ${missing.join(', ')}` +
          (have.length ? ` (only ${have.join(', ')})` : '') +
          '; continuing without its C library there');
      }
      if (meta.pebble && meta.pebble.messageKeys) messageKeys.push(meta.pebble.messageKeys);
    } else {
      // A plain JS package; the bundler resolves into it normally.
      packages[name] = { files, main: meta.main || 'index.js', isLibrary: false };
    }
    // A package's own dependencies have to come too, whichever shape it
    // is: a C library links against them (nightstand needs
    // pebble-effect-layer), a JS one imports them.
    for (const [d, r] of Object.entries(meta.dependencies || {})) {
      if (!seen.has(d)) queue.push([d, r]);
    }
  }
  return { includes, libs, packages, resources, messageKeys };
}
