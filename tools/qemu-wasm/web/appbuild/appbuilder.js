// Build a Pebble app from source, entirely client-side: clang.wasm
// compiles, lld.wasm links, and the waf-equivalent codegen, resource
// compilation and packaging run as JS. Produces a ready-to-install .pbw.
import {
  genAppinfoJson, genAppinfoC, genResourceIdsH, genResourceIdsC,
  genMessageKeysC, genMessageKeysH, genLdScript, PLATFORM_DEFINES,
} from './codegen.js';
import { buildResources } from './resources.js';
import { objcopyToBin, injectMetadata, makePbw, makeManifest } from './elfpack.js';
import { makeTree, readTree, runTool } from './wasi-run.js';

const te = new TextEncoder();
const td = new TextDecoder();

const CFLAGS = [
  '--target=arm-none-eabi', '-mcpu=cortex-m3', '-mthumb',
  '-std=c99', '-ffunction-sections', '-fdata-sections', '-fcommon',
  '-g', '-fPIE', '-Os', '-D_TIME_H_', '-Dtime_t=long',
  '-Wall', '-Wno-typedef-redefinition', '-Wno-missing-field-initializers',
  '-resource-dir', '/clang-res',
  '-isystem', '/newlib', '-I', '/sdk/include',
  '-I', '/proj/build', '-I', '/proj/build/include', '-I', '/proj/build/plat',
  '-I', '/proj/build/src', '-I', '/proj/src', '-I', '/proj',
];

const LDFLAGS = ['--gc-sections', '--warn-common', '--build-id=sha1',
  '--emit-relocs', '-Bstatic', '-EL', '--target2=rel'];

// Locate package.json in a repo file map; returns {prefix, pkg}. Handles
// the extra top-level directory GitHub zips carry.
function findProject(files) {
  const candidates = Object.keys(files)
    .filter((p) => p.endsWith('package.json') && !p.includes('node_modules'))
    .sort((a, b) => a.length - b.length);
  for (const p of candidates) {
    try {
      const pkg = JSON.parse(td.decode(files[p]));
      if (pkg.pebble) return { prefix: p.slice(0, -'package.json'.length), pkg };
    } catch (e) { /* not it */ }
  }
  throw new Error('no package.json with a "pebble" section found in the repo');
}

// The PebbleKit JS entry point is declared in the project's wscript
// (js_entry_file=), so read it from there and fall back to the two
// conventional layouts.
function findJsEntry(rel, wscript) {
  const m = wscript && wscript.match(/js_entry_file\s*=\s*['"]([^'"]+)['"]/);
  if (m && rel(m[1])) return m[1];
  for (const p of ['src/pkjs/index.js', 'src/js/pebble-js-app.js',
                   'src/js/index.js', 'src/pkjs/app.js', 'src/js/app.js']) {
    if (rel(p)) return p;
  }
  return null;
}

// opts: {
//   repoFiles: {path: Uint8Array},          // unzipped repository
//   platform: 'emery',
//   clang, lld: WebAssembly.Module,
//   sdkPack: {path: Uint8Array},            // unzipped sdkpack.zip
//   rasterizeGlyph: async (fontData, cp, px) => glyph,   // see resources.js
//   bundleJs: async ({entryPath, files, appinfo}) => string | null,
//   log: (msg) => void,
//   timestamp: unix seconds,
// }
export async function buildApp(opts) {
  const { repoFiles, platform, clang, lld, sdkPack, log = () => {} } = opts;
  const timestamp = opts.timestamp || Math.floor(Date.now() / 1000);

  const { prefix, pkg } = findProject(repoFiles);
  const rel = (p) => repoFiles[prefix + p];
  const appinfo = genAppinfoJson(pkg);
  if (!appinfo.uuid) throw new Error('package.json pebble section has no uuid');
  const name = appinfo.shortName;
  log(`project: ${name} (${appinfo.versionLabel}) by ${appinfo.companyName}`);

  if (!appinfo.targetPlatforms.includes(platform)) {
    log(`note: ${platform} not in targetPlatforms [${appinfo.targetPlatforms}] — building anyway`);
  }

  // ---- resources ----
  const media = (appinfo.resources && appinfo.resources.media) || [];
  const readResource = async (path) => {
    const data = rel('resources/' + path);
    if (!data) throw new Error('missing resource file: resources/' + path);
    return data;
  };
  log(`compiling ${media.length} resources…`);
  const { pbpack, resourceNames } =
    await buildResources(media, readResource, platform, opts.rasterizeGlyph);

  // ---- codegen ----
  const generated = {
    'build/appinfo.auto.c': te.encode(genAppinfoC(appinfo, platform)),
    'build/src/resource_ids.auto.h': te.encode(genResourceIdsH(resourceNames)),
    'build/src/resource_ids.auto.c': te.encode(genResourceIdsC(resourceNames)),
    'build/include/message_keys.auto.h': te.encode(genMessageKeysH(appinfo.appKeys)),
    'build/message_keys.auto.c': te.encode(genMessageKeysC(appinfo.appKeys)),
  };
  const ldscript = te.encode(genLdScript(platform));

  // ---- project tree for the compiler ----
  const projFiles = {};
  for (const [p, data] of Object.entries(repoFiles)) {
    if (!p.startsWith(prefix)) continue;
    const q = p.slice(prefix.length);
    if (q.startsWith('src/') && /\.(c|h)$/.test(q)) projFiles[q] = data;
  }
  Object.assign(projFiles, generated);
  // resource_ids.auto.h is included as "src/resource_ids.auto.h" by
  // appinfo.auto.c (via -I/proj/build) and directly by app sources.
  projFiles['build/plat/src/resource_ids.auto.h'] = generated['build/src/resource_ids.auto.h'];
  const projTree = makeTree(projFiles);

  const sdkTree = makeTree(Object.fromEntries(
    Object.entries(sdkPack).filter(([p]) => p.startsWith('sdk/'))
      .map(([p, d]) => [p.slice(4), d])));
  const newlibTree = makeTree(Object.fromEntries(
    Object.entries(sdkPack).filter(([p]) => p.startsWith('newlib/'))
      .map(([p, d]) => [p.slice(7), d])));
  const clangResTree = makeTree(Object.fromEntries(
    Object.entries(sdkPack).filter(([p]) => p.startsWith('clang-res/'))
      .map(([p, d]) => [p.slice(10), d])));
  const libTree = makeTree({
    'libgcc.a': sdkPack['libgcc.a'],
    'libc.a': sdkPack['libc.a'],
  });

  // ---- compile ----
  const sources = Object.keys(projFiles).filter((p) => p.endsWith('.c') &&
    !p.startsWith('src/js/') && !p.startsWith('src/pkjs/'));
  const defines = (PLATFORM_DEFINES[platform] || [])
    .concat(['PBL_SDK_3', 'RELEASE']).map((d) => '-D' + d);
  const objTree = new Map();
  for (const src of sources) {
    const objName = src.replace(/[\/]/g, '_') + '.o';
    log(`clang.wasm: ${src}`);
    const { code, stderr } = await runTool(clang, 'clang',
      [...CFLAGS, ...defines, '-c', '/proj/' + src, '-o', '/obj/' + objName],
      { '/sdk': sdkTree, '/newlib': newlibTree, '/clang-res': clangResTree,
        '/proj': projTree, '/obj': objTree }, log);
    if (code !== 0) throw new Error(`compile failed: ${src}\n${stderr.slice(0, 2000)}`);
  }

  // ---- link ----
  log('lld.wasm: linking…');
  const ldTree = makeTree({ 'pebble_app.ld': ldscript });
  const outTree = new Map();
  const objArgs = [...objTree.keys()].map((n) => '/obj/' + n);
  const { code, stderr } = await runTool(lld, 'ld.lld',
    [...LDFLAGS, '-T', '/ld/pebble_app.ld', ...objArgs,
     '-L/sdk/lib', '-lpebble', '/lib/libgcc.a', '/lib/libc.a',
     '-o', '/out/app.elf'],
    { '/sdk': sdkTree, '/obj': objTree, '/ld': ldTree, '/lib': libTree,
      '/out': outTree }, log);
  if (code !== 0) throw new Error('link failed\n' + stderr.slice(0, 2000));
  const elf = readTree(outTree, 'app.elf');
  if (!elf) throw new Error('linker produced no output');
  log(`linked: ${elf.length} bytes`);

  // ---- JS ----
  let jsText = null;
  const multiJs = appinfo.enableMultiJS !== false;
  const wscript = rel('wscript') ? td.decode(rel('wscript')) : null;
  const entryPath = findJsEntry(rel, wscript);
  if (entryPath && multiJs && opts.bundleJs) {
    log(`bundling PebbleKit JS from ${entryPath}…`);
    jsText = await opts.bundleJs({
      entryPath, prefix, files: repoFiles, pkg,
      appKeys: appinfo.appKeys,
      sharedAdditions: sdkPack['js/_pkjs_shared_additions.js']
        ? td.decode(sdkPack['js/_pkjs_shared_additions.js']) : null,
    });
  } else if (entryPath) {
    // No bundler (or multiJS off): ship the entry file verbatim, which is
    // what waf does for a single-file pebble-js-app.js project.
    jsText = td.decode(rel(entryPath));
  }

  // ---- package ----
  const raw = objcopyToBin(elf);
  const appBin = injectMetadata(raw, elf, pbpack,
    { timestamp, allowJs: !!jsText, hasWorker: false });
  const manifest = makeManifest({
    appBin, pbpack, timestamp,
    sdkVersion: { major: 5, minor: 106 },
    jsPresent: !!jsText,
  });

  const entries = {
    'appinfo.json': te.encode(JSON.stringify(appinfo, null, 4)),
    [`${platform}/pebble-app.bin`]: appBin,
    [`${platform}/app_resources.pbpack`]: pbpack,
    [`${platform}/manifest.json`]: te.encode(JSON.stringify(manifest)),
  };
  if (jsText) entries['pebble-js-app.js'] = te.encode(jsText);
  const pbw = makePbw(Object.entries(entries).map(([name, data]) => ({
    name, data, date: new Date(timestamp * 1000),
  })));
  log(`pbw ready: ${pbw.length} bytes`);
  return { pbw, appinfo, name };
}
