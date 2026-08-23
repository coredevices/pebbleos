// Module worker: builds a Pebble app from a GitHub repo zip, entirely
// client-side. Fetches the toolchain + SDK pack (cached via Cache API),
// unzips the repo, runs the full pipeline, posts back the .pbw.
//
// in:  {cmd:'build', zipUrl | zipBytes, platform, proxy, base}
// out: {type:'log'|'error'|'done', ...}
import { unzip } from '../pbw.js';
import { sfntFont } from './resources.js';
import { buildApp } from './appbuilder.js';
import { bundlePkjs } from './jsbundle.js';
import * as esbuild from '../vendor/esbuild/browser.min.js';

const post = (m) => self.postMessage(m);
const log = (msg) => post({ type: 'log', msg });

let toolsPromise = null;
let xsPromise = null;
const sdkPacks = new Map();          // platform -> Promise<{path: bytes}>
let esbuildReady = null;

async function fetchCached(url) {
  const cache = await caches.open('pebble-buildtools-v1').catch(() => null);
  if (cache) {
    const hit = await cache.match(url);
    if (hit) return new Uint8Array(await hit.arrayBuffer());
  }
  const r = await fetch(url);
  if (!r.ok) throw new Error(`HTTP ${r.status} fetching ${url}`);
  const clone = cache ? r.clone() : null;
  const bytes = new Uint8Array(await r.arrayBuffer());
  if (cache && clone) await cache.put(url, clone).catch(() => {});
  return bytes;
}

async function gunzip(bytes) {
  const ds = new DecompressionStream('gzip');
  const stream = new Blob([bytes]).stream().pipeThrough(ds);
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

// The compiler is the same whatever the board is; only the SDK pack
// differs, so switching boards does not re-fetch the 33 MB of tools.
async function loadTools(base) {
  if (!toolsPromise) {
    toolsPromise = (async () => {
      log('downloading the toolchain (one-time, ~36 MB — cached after this)…');
      const [clangGz, lldGz, esbuildGz] = await Promise.all([
        fetchCached(base + 'tools/clang.wasm.gz'),
        fetchCached(base + 'tools/lld.wasm.gz'),
        fetchCached(base + 'tools/esbuild.wasm.gz'),
      ]);
      log('decompressing + compiling the toolchain…');
      const [clangBytes, lldBytes, esbuildBytes] =
        await Promise.all([gunzip(clangGz), gunzip(lldGz), gunzip(esbuildGz)]);
      const [clang, lld, esbuildMod] = await Promise.all([
        WebAssembly.compile(clangBytes.buffer),
        WebAssembly.compile(lldBytes.buffer),
        WebAssembly.compile(esbuildBytes.buffer),
      ]);
      log('toolchain ready');
      return { clang, lld, esbuildMod };
    })();
    toolsPromise.catch(() => { toolsPromise = null; });
  }
  return toolsPromise;
}

// Moddable's compiler and linker, plus the SDK manifests and typings a
// project's manifest may include. Only a Moddable project needs them, so
// they are fetched the first time one is built rather than up front.
async function loadXsTools(base) {
  if (!xsPromise) {
    xsPromise = (async () => {
      log('this is a Moddable project — downloading xsc and xsl…');
      const [xscGz, xslGz, packZip] = await Promise.all([
        fetchCached(base + 'tools/xsc.wasm.gz'),
        fetchCached(base + 'tools/xsl.wasm.gz'),
        fetchCached(base + 'tools/modpack.zip'),
      ]);
      const [xscBytes, xslBytes] = await Promise.all([gunzip(xscGz), gunzip(xslGz)]);
      const [xsc, xsl] = await Promise.all([
        WebAssembly.compile(xscBytes.buffer),
        WebAssembly.compile(xslBytes.buffer),
      ]);
      return { xsc, xsl, modFiles: await unzip(packZip) };
    })();
    xsPromise.catch(() => { xsPromise = null; });
  }
  return xsPromise;
}

async function loadSdkPack(base, platform) {
  if (!sdkPacks.has(platform)) {
    const p = (async () => {
      log(`downloading the ${platform} SDK…`);
      const url = `${base}tools/sdkpack-${platform}.zip`;
      let bytes;
      try {
        bytes = await fetchCached(url);
      } catch (e) {
        throw new Error(`no SDK available for ${platform} (${url})`);
      }
      return unzip(bytes);
    })();
    p.catch(() => sdkPacks.delete(platform));
    sdkPacks.set(platform, p);
  }
  return sdkPacks.get(platform);
}

// ---- canvas font rasterizer ----
const fontFaces = new Map(); // fontData -> family name
let fontCounter = 0;

async function fontFamilyFor(fontData) {
  if (fontFaces.has(fontData)) return fontFaces.get(fontData);
  const family = 'PblAppFont' + (fontCounter++);
  const sfnt = sfntFont(fontData);
  const face = new FontFace(family, sfnt.buffer.slice(
    sfnt.byteOffset, sfnt.byteOffset + sfnt.byteLength));
  await face.load();
  self.fonts.add(face);
  fontFaces.set(fontData, family);
  return family;
}

async function rasterizeGlyph(fontData, codepoint, pxSize) {
  const family = await fontFamilyFor(fontData);
  const pad = Math.ceil(pxSize / 2);
  const cw = pxSize * 3 + pad * 2, ch = pxSize * 3 + pad * 2;
  const canvas = new OffscreenCanvas(cw, ch);
  const ctx = canvas.getContext('2d', { willReadFrequently: true });
  ctx.font = `${pxSize}px ${family}`;
  ctx.textBaseline = 'alphabetic';
  ctx.fillStyle = '#fff';
  const s = String.fromCodePoint(codepoint);
  const m = ctx.measureText(s);
  const ox = pad + pxSize, oy = pad + pxSize * 2; // draw origin (baseline)
  ctx.fillText(s, ox, oy);
  const img = ctx.getImageData(0, 0, cw, ch).data;

  // tight bounding box of alpha >= 128
  let minX = cw, minY = ch, maxX = -1, maxY = -1;
  for (let y = 0; y < ch; y++) {
    for (let x = 0; x < cw; x++) {
      if (img[(y * cw + x) * 4 + 3] >= 128) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
    }
  }
  const advance = Math.round(m.width);
  if (maxX < 0) {
    return { bitmap: new Uint8Array(0), width: 0, height: 0, left: 0, top: 0, advance };
  }
  const width = maxX - minX + 1, height = maxY - minY + 1;
  const stride = Math.ceil(width / 8);
  const bitmap = new Uint8Array(stride * height);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      if (img[((minY + y) * cw + (minX + x)) * 4 + 3] >= 128) {
        bitmap[y * stride + (x >> 3)] |= 0x80 >> (x & 7);
      }
    }
  }
  return {
    bitmap, width, height,
    left: minX - ox,          // bearing X from pen position
    top: oy - minY,           // distance from baseline up to first row
    advance,
  };
}

self.onmessage = async (e) => {
  const { cmd, zipUrl, zipUrls, zipBytes, platform = 'emery', proxy, appHint } = e.data;
  if (cmd !== 'build') return;
  // Tools live at the site root; this module sits one level down in
  // appbuild/, so resolve against that rather than the page's URL.
  const base = new URL(e.data.base || '../', import.meta.url).href;
  try {
    let repoZip = zipBytes ? new Uint8Array(zipBytes) : null;
    if (!repoZip) {
      log('downloading repository…');
      const urls = zipUrls || (zipUrl ? [zipUrl] : []);
      let r = null;
      for (const u of urls) {
        try { r = await fetch(u, { mode: 'cors' }); } catch (err) { r = null; }
        if ((!r || !r.ok) && proxy) {
          r = await fetch(proxy + encodeURIComponent(u), { mode: 'cors' })
            .catch(() => null);
        }
        if (r && r.ok) break;
      }
      if (!r || !r.ok) throw new Error('could not download the repository zip');
      repoZip = new Uint8Array(await r.arrayBuffer());
    }
    log(`repository: ${(repoZip.length / 1024).toFixed(0)} KB — unpacking…`);
    const repoFiles = await unzip(repoZip);

    const { clang, lld, esbuildMod } = await loadTools(base);
    // The SDK bundles every target platform into one pbw. Build the
    // selected board first, then any other board we hold an SDK for that
    // the project also targets, so the result installs anywhere.
    const wanted = [platform, ...(Array.isArray(e.data.alsoPlatforms)
      ? e.data.alsoPlatforms : [])];
    const packs = {};
    for (const p of [...new Set(wanted)]) {
      try {
        packs[p] = await loadSdkPack(base, p);
      } catch (err) {
        if (p === platform) throw err;
        log(`no SDK for ${p}, skipping it`);
      }
    }

    if (!esbuildReady) {
      esbuildReady = esbuild.initialize({ wasmModule: esbuildMod, worker: false })
        .catch((err) => { esbuildReady = null; throw err; });
    }
    await esbuildReady;

    // npm serves CORS headers, but fall back to the proxy like the repo zip.
    const fetchUrl = async (url) => {
      let r = await fetch(url, { mode: 'cors' }).catch(() => null);
      if ((!r || !r.ok) && proxy) {
        r = await fetch(proxy + encodeURIComponent(url), { mode: 'cors' }).catch(() => null);
      }
      if (!r || !r.ok) throw new Error(`could not fetch ${url}`);
      return new Uint8Array(await r.arrayBuffer());
    };

    const t0 = performance.now();
    const { pbw, name } = await buildApp({
      repoFiles, clang, lld, sdkPacks: packs, appHint, fetchUrl,
      rasterizeGlyph, xsTools: () => loadXsTools(base),
      // mcrun runs tsc over a Moddable project's TypeScript modules; esbuild
      // strips the types just as well and is already loaded for PebbleKit JS.
      transpileTs: async (source, path) =>
        (await esbuild.transform(source, { loader: 'ts', target: 'esnext',
                                           sourcefile: path })).code,
      bundleJs: (args) => bundlePkjs(esbuild, args),
      log,
    });
    log(`build finished in ${((performance.now() - t0) / 1000).toFixed(1)}s`);
    post({ type: 'done', pbw: pbw.buffer, name }, [pbw.buffer]);
  } catch (err) {
    post({ type: 'error', msg: err.message || String(err) });
  }
};
