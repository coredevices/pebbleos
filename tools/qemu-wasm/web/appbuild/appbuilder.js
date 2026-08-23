// Build a Pebble app from source, entirely client-side: clang.wasm
// compiles, lld.wasm links, and the waf-equivalent codegen, resource
// compilation and packaging run as JS. Produces a ready-to-install .pbw.
import {
  genAppinfoJson, genAppinfoC, genResourceIdsH, genResourceIdsC,
  genMessageKeysC, genMessageKeysH, genLdScript, PLATFORM_DEFINES, PLATFORMS,
} from './codegen.js';
import { buildResources, findTaggedFile } from './resources.js';
import { objcopyToBin, injectMetadata, makePbw, makeManifest } from './elfpack.js';
import { makeTree, readTree, runTool } from './wasi-run.js';
import { fetchDependencies } from './deps.js';

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
function findProject(files, hint) {
  const apps = [];
  for (const p of Object.keys(files)) {
    if (!p.endsWith('package.json') || p.includes('node_modules')) continue;
    let pkg;
    try { pkg = JSON.parse(td.decode(files[p])); } catch (e) { continue; }
    if (!pkg || !pkg.pebble) continue;
    // A "package" is an SDK library, not something we can build and run.
    if (pkg.pebble.projectType === 'package') continue;
    apps.push({ path: p, prefix: p.slice(0, -'package.json'.length), pkg });
  }
  if (!apps.length) {
    // Older projects predate package.json and describe themselves in an
    // appinfo.json at the project root instead.
    for (const p of Object.keys(files)) {
      if (!p.endsWith('appinfo.json') || p.includes('node_modules')) continue;
      let info;
      try { info = JSON.parse(td.decode(files[p])); } catch (e) { continue; }
      if (!info || !info.uuid) continue;
      const { uuid, shortName, longName, companyName, versionLabel, sdkVersion,
              targetPlatforms, appKeys, resources, watchapp, capabilities } = info;
      apps.push({
        path: p,
        prefix: p.slice(0, -'appinfo.json'.length),
        pkg: {
          name: shortName, version: versionLabel, author: companyName,
          pebble: { uuid, sdkVersion, targetPlatforms, messageKeys: appKeys,
                    resources, watchapp, capabilities,
                    displayName: longName || shortName, projectType: 'native',
                    enableMultiJS: false },
        },
      });
    }
  }
  if (!apps.length) {
    throw new Error('no package.json with a "pebble" section (or appinfo.json) found in the repo');
  }

  // Repos holding many apps are common (SDK sample collections, personal
  // monorepos), so the shallowest package.json is usually not the one the
  // user meant. Prefer a name match, then the shallowest.
  const depth = (a) => a.path.split('/').length;
  if (hint) {
    const want = String(hint).toLowerCase().replace(/[^a-z0-9]/g, '');
    const score = (a) => {
      const names = [a.pkg.pebble.displayName, a.pkg.name, a.prefix]
        .filter(Boolean).map((s) => String(s).toLowerCase().replace(/[^a-z0-9]/g, ''));
      if (names.some((n) => n === want)) return 3;
      if (names.some((n) => n.includes(want) || want.includes(n))) return 2;
      return 0;
    };
    const ranked = apps.map((a) => ({ a, s: score(a) })).filter((r) => r.s > 0)
      .sort((x, y) => y.s - x.s || depth(x.a) - depth(y.a));
    if (ranked.length) return { ...ranked[0].a, others: apps.length - 1 };
  }
  apps.sort((a, b) => depth(a) - depth(b) || a.path.length - b.path.length);
  return { ...apps[0], others: apps.length - 1 };
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
//   appHint: name to disambiguate a repo holding several apps,
//   log: (msg) => void,
//   timestamp: unix seconds,
// }
export async function buildApp(opts) {
  const { repoFiles, clang, lld, log = () => {} } = opts;
  const timestamp = opts.timestamp || Math.floor(Date.now() / 1000);
  // sdkPacks maps platform -> unzipped pack. The SDK builds every target
  // platform into one bundle; we build every one we hold an SDK for.
  const sdkPacks = opts.sdkPacks || { [opts.platform]: opts.sdkPack };
  let platforms = Object.keys(sdkPacks);
  const primary = platforms[0];

  const { prefix, pkg, others } = findProject(repoFiles, opts.appHint);
  const rel = (p) => repoFiles[prefix + p];
  const projectType = (pkg.pebble && pkg.pebble.projectType) || 'native';
  if (projectType !== 'native') {
    throw new Error(`this is a "${projectType}" project; only native (C) apps ` +
      'can be built here');
  }
  const appinfo = genAppinfoJson(pkg);
  if (!appinfo.uuid) throw new Error('package.json pebble section has no uuid');
  const name = appinfo.shortName;
  log(`project: ${name} (${appinfo.versionLabel}) by ${appinfo.companyName}`);
  if (others) log(`note: repo holds ${others + 1} apps; built ${prefix || './'}`);

  if (!appinfo.targetPlatforms.includes(primary)) {
    log(`note: ${primary} not in targetPlatforms [${appinfo.targetPlatforms}] — building anyway`);
  }
  // Bundle the other boards the project actually targets, as the SDK does.
  platforms = platforms.filter((p) => p === primary || appinfo.targetPlatforms.includes(p));

  // ---- dependencies ----
  let deps = { includes: {}, libs: {}, packages: {}, resources: [], messageKeys: [] };
  if (pkg.dependencies && Object.keys(pkg.dependencies).length) {
    if (!opts.fetchUrl) {
      log('note: app has dependencies but no fetcher was supplied — skipping');
    } else {
      log(`resolving ${Object.keys(pkg.dependencies).length} dependencies…`);
      deps = await fetchDependencies(pkg, { platforms, get: opts.fetchUrl, log });
    }
  }

  // Everything from resources to the linked binary depends on the
  // board, so it runs once per platform being bundled.
  const perPlatform = {};
  for (const platform of platforms) {
    const sdkPack = sdkPacks[platform];
    log(`--- ${platform} ---`);
    // ---- resources ----
    const media = (appinfo.resources && appinfo.resources.media) || [];
    const resourceFiles = Object.keys(repoFiles)
      .filter((p) => p.startsWith(prefix + 'resources/'))
      .map((p) => p.slice((prefix + 'resources/').length));
    const readResource = async (path) => {
      // A resource may only exist as ~tagged variants (menu-icon~color.png).
      const pick = findTaggedFile(path, resourceFiles, PLATFORMS[platform].tags);
      const data = pick ? rel('resources/' + pick) : null;
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
    const depIncludeTree = makeTree(deps.includes);
    const libTree = makeTree({
      'libgcc.a': sdkPack['libgcc.a'],
      'libc.a': sdkPack['libc.a'],
      'libm.a': sdkPack['libm.a'],
    });

    // ---- compile ----
    const sources = Object.keys(projFiles).filter((p) => p.endsWith('.c') &&
      !p.startsWith('src/js/') && !p.startsWith('src/pkjs/'));
    const defines = (PLATFORM_DEFINES[platform] || [])
      .concat(['PBL_SDK_3', 'RELEASE']).map((d) => '-D' + d);
    // The SDK puts a package's include root and its per-platform directory
    // on the search path (setup_pebble_c).
    const depIncludes = [];
    if (Object.keys(deps.includes).length) {
      depIncludes.push('-I', '/deps');
      for (const name of new Set(Object.keys(deps.includes).map((p) => p.split('/')[0]))) {
        depIncludes.push('-I', `/deps/${name}/${platform}`);
      }
    }
    const objTree = new Map();
    for (const src of sources) {
      const objName = src.replace(/[\/]/g, '_') + '.o';
      log(`clang.wasm: ${src}`);
      const { code, stderr } = await runTool(clang, 'clang',
        [...CFLAGS, ...defines, ...depIncludes, '-c', '/proj/' + src, '-o', '/obj/' + objName],
        { '/sdk': sdkTree, '/newlib': newlibTree, '/clang-res': clangResTree,
          '/proj': projTree, '/obj': objTree, '/deps': depIncludeTree }, log);
      if (code !== 0) throw new Error(`compile failed: ${src}\n${stderr.slice(0, 2000)}`);
    }

    // ---- link ----
    log('lld.wasm: linking…');
    const ldTree = makeTree({ 'pebble_app.ld': ldscript });
    const depLibTree = makeTree(Object.fromEntries(
      (deps.libs[platform] || []).map((l) => [l.name, l.data])));
    const outTree = new Map();
    const objArgs = [...objTree.keys()].map((n) => '/obj/' + n);
    const { code, stderr } = await runTool(lld, 'ld.lld',
      [...LDFLAGS, '-T', '/ld/pebble_app.ld', ...objArgs,
       ...(deps.libs[platform] || []).map((l) => `/deplib/${l.name}`),
       '-L/sdk/lib', '-lpebble', '/lib/libgcc.a', '/lib/libc.a', '/lib/libm.a',
       '-o', '/out/app.elf'],
      { '/sdk': sdkTree, '/obj': objTree, '/ld': ldTree, '/lib': libTree,
        '/deplib': depLibTree, '/out': outTree }, log);
    if (code !== 0) throw new Error('link failed\n' + stderr.slice(0, 2000));
    const elf = readTree(outTree, 'app.elf');
    if (!elf) throw new Error('linker produced no output');
    log(`linked: ${elf.length} bytes`);

    // ---- background worker ----
    // The SDK builds worker_src/c/**/*.c into a second binary that runs
    // outside the app's lifetime (app wscript, bin_type='worker').
    const workerSources = Object.keys(repoFiles)
      .filter((p) => p.startsWith(prefix + 'worker_src/') && p.endsWith('.c'))
      .map((p) => p.slice(prefix.length));
    let workerBin = null;
    if (workerSources.length) {
      log(`building background worker (${workerSources.length} sources)…`);
      const workerFiles = {};
      for (const p of workerSources) workerFiles[p] = repoFiles[prefix + p];
      for (const [p, d] of Object.entries(repoFiles)) {
        const q = p.startsWith(prefix) ? p.slice(prefix.length) : null;
        if (q && q.startsWith('worker_src/') && q.endsWith('.h')) workerFiles[q] = d;
      }
      Object.assign(workerFiles, generated);
      workerFiles['build/plat/src/resource_ids.auto.h'] = generated['build/src/resource_ids.auto.h'];
      const workerTree = makeTree(workerFiles);
      const wObjTree = new Map();
      for (const src of workerSources) {
        const objName = src.replace(/[\/]/g, '_') + '.o';
        const { code, stderr } = await runTool(clang, 'clang',
          [...CFLAGS, ...defines, ...depIncludes, '-c', '/proj/' + src, '-o', '/obj/' + objName],
          { '/sdk': sdkTree, '/newlib': newlibTree, '/clang-res': clangResTree,
            '/proj': workerTree, '/obj': wObjTree, '/deps': depIncludeTree }, log);
        if (code !== 0) throw new Error(`worker compile failed: ${src}\n${stderr.slice(0, 2000)}`);
      }
      const wOut = new Map();
      const w = await runTool(lld, 'ld.lld',
        [...LDFLAGS, '-T', '/ld/pebble_app.ld', ...[...wObjTree.keys()].map((n) => '/obj/' + n),
         ...(deps.libs[platform] || []).map((l) => `/deplib/${l.name}`),
         '-L/sdk/lib', '-lpebble', '/lib/libgcc.a', '/lib/libc.a', '/lib/libm.a',
         '-o', '/out/worker.elf'],
        { '/sdk': sdkTree, '/obj': wObjTree, '/ld': ldTree, '/lib': libTree,
          '/deplib': depLibTree, '/out': wOut }, log);
      if (w.code !== 0) throw new Error('worker link failed\n' + w.stderr.slice(0, 2000));
      const wElf = readTree(wOut, 'worker.elf');
      workerBin = injectMetadata(objcopyToBin(wElf), wElf, null,
        { timestamp, allowJs: false, hasWorker: false });
      log(`worker: ${workerBin.length} bytes`);
    }

    perPlatform[platform] = { pbpack, elf, workerBin };
  }

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
      sharedAdditions: sdkPacks[primary]['js/_pkjs_shared_additions.js']
        ? td.decode(sdkPacks[primary]['js/_pkjs_shared_additions.js']) : null,
      packages: deps.packages,
    });
  } else if (entryPath) {
    // No bundler (or multiJS off): ship the entry file verbatim, which is
    // what waf does for a single-file pebble-js-app.js project.
    jsText = td.decode(rel(entryPath));
  }

  // ---- package ----
  const entries = { 'appinfo.json': te.encode(JSON.stringify(appinfo, null, 4)) };
  for (const plat of platforms) {
    const { pbpack, elf, workerBin } = perPlatform[plat];
    const appBin = injectMetadata(objcopyToBin(elf), elf, pbpack,
      { timestamp, allowJs: !!jsText, hasWorker: !!workerBin });
    entries[`${plat}/pebble-app.bin`] = appBin;
    entries[`${plat}/app_resources.pbpack`] = pbpack;
    entries[`${plat}/manifest.json`] = te.encode(JSON.stringify(makeManifest({
      appBin, pbpack, timestamp, sdkVersion: { major: 5, minor: 106 },
      jsPresent: !!jsText,
    })));
    if (workerBin) entries[`${plat}/pebble-worker.bin`] = workerBin;
  }
  if (jsText) entries['pebble-js-app.js'] = te.encode(jsText);
  const pbw = makePbw(Object.entries(entries).map(([name, data]) => ({
    name, data, date: new Date(timestamp * 1000),
  })));
  log(`pbw ready: ${pbw.length} bytes for ${platforms.join(', ')}`);
  return { pbw, appinfo, name };
}
