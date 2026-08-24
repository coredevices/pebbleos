// Build a Pebble app from source, entirely client-side: clang.wasm
// compiles, lld.wasm links, and the waf-equivalent codegen, resource
// compilation and packaging run as JS. Produces a ready-to-install .pbw.
import {
  genAppinfoJson, genAppinfoC, genResourceIdsH, genResourceIdsC,
  genMessageKeysC, genMessageKeysH, genLdScript, PLATFORM_DEFINES, PLATFORMS,
  messageKeysFor,
} from './codegen.js';
import { buildResources, findTaggedFile } from './resources.js';
import { objcopyToBin, injectMetadata, makePbw, makeManifest } from './elfpack.js';
import { makeTree, readTree, runTool } from './wasi-run.js';
import { fetchDependencies } from './deps.js';
import { buildMod, findModManifest } from './moddable.js';

const te = new TextEncoder();
// stands in for a file path when the resource comes from memory
const MOD_SENTINEL = '\u0000mod';
const td = new TextDecoder();

const CFLAGS = [
  '--target=arm-none-eabi', '-mcpu=cortex-m3', '-mthumb',
  // the SDK's arm-none-eabi-gcc packs enums (clang defaults to int-sized);
  // firmware structs like Tuplet depend on it, so match the ABI
  '-std=c99', '-fshort-enums',
  '-ffunction-sections', '-fdata-sections', '-fcommon',
  // the SDK's era of newlib exposed BSD names (uint, strcasecmp, …) even
  // under -std=c99; today's headers want _DEFAULT_SOURCE asked for
  '-g', '-fPIE', '-Os', '-D_TIME_H_', '-Dtime_t=long', '-D_DEFAULT_SOURCE',
  '-Wall', '-Wno-typedef-redefinition', '-Wno-missing-field-initializers',
  '-resource-dir', '/clang-res',
  '-isystem', '/newlib', '-I', '/sdk/include',
  '-I', '/proj/build', '-I', '/proj/build/include', '-I', '/proj/build/plat',
  '-I', '/proj/build/src', '-I', '/proj/src', '-I', '/proj',
  // setup_pebble_c also puts the project's own include/ on the path
  '-I', '/proj/include',
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


// Some projects change the compiler flags in their wscript. Pebble.js is
// the common case: its menu types are built from anonymous members of a
// named struct type, which is a Microsoft extension, so it compiles with
// -std=c11 -fms-extensions and nothing else will do. waf runs that file as
// Python; lifting the flag literals out of it covers what these projects
// actually do without running anything.
//
// Only -std, -f and -D are taken. Warning options are left alone because a
// project's -Werror would fail this build over warnings its own toolchain
// never emitted.
function wscriptCflags(wscript) {
  if (!wscript) return [];
  const flags = [];
  // Flags follow the line that names cflags, one per line. The window has
  // to travel with the list — these lists run long, and the warning options
  // we skip would otherwise end it before the defines at the bottom.
  //
  // A -D value can hold spaces and quotes of its own
  // (-DNAME="Home Assistant WS"), so a flag runs to its matching quote. waf
  // hands these to the compiler without a shell and so do we, which keeps
  // the inner quotes as part of the macro body.
  const ANY_FLAG = /(['"])(-(?:(?!\1).)+)\1/g;
  let near = 0;
  for (const line of wscript.split('\n')) {
    if (/cflags/i.test(line)) near = 3;
    else if (near) near--;
    if (!near) continue;
    for (const m of line.matchAll(ANY_FLAG)) {
      near = 3;
      // A literal the wscript formats ('-DX={}'.format(...), f-strings,
      // '%'-interpolation) gets its real value at build time, usually from
      // the environment; the placeholder text would poison the compile.
      const after = line.slice(m.index + m[0].length);
      if (/^\s*\.\s*format\s*\(/.test(after) || /^\s*%/.test(after) ||
          line[m.index - 1] === 'f') continue;
      if (!/^-(std=|f|D)/.test(m[2])) continue;
      if (!flags.includes(m[2])) flags.push(m[2]);
    }
  }
  return flags;
}

// A project's wscript can glob more than the default src/c/**/*.c —
// extra trees like src/modules are real, and waf honours them. Lift the
// literal .c glob patterns; a wscript that builds its list dynamically
// yields nothing and the caller falls back to the layout heuristic.
function wscriptSourceGlobs(wscript) {
  if (!wscript) return null;
  const globs = [];
  for (const m of wscript.matchAll(/['"]([^'"\n]*\*[^'"\n]*\.c)['"]/g)) {
    if (!m[1].startsWith('worker_src')) globs.push(m[1]);
  }
  return globs.length ? globs : null;
}

function globToRegExp(glob) {
  return new RegExp('^' + glob
    .replace(/[.+^${}()|[\]\\]/g, '\\$&')
    .replace(/\*\*\//g, '\u0000')
    .replace(/\*/g, '[^/]*')
    .replace(/\u0000/g, '(?:[^/]+/)*') + '$');
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
//   xsTools: async () => ({xsc, xsl, modFiles}), Moddable's compiler and
//     linker as WebAssembly.Modules plus the SDK files a manifest may
//     include (unzipped modpack.zip). Called only for a Moddable project,
//     so an ordinary build never fetches them,
//   transpileTs: async (source, path) => js, for Moddable modules written
//     in TypeScript (mcrun shells out to tsc for these),
//   mods: {platform: Uint8Array}, prebuilt XS archives, if the caller has
//     them already and wants to skip the xsc/xsl step,
//   log: (msg) => void,
//   timestamp: unix seconds,
// }
export async function buildApp(opts) {
  const { repoFiles, clang, lld, mods, xsTools, log = () => {} } = opts;
  const timestamp = opts.timestamp || Math.floor(Date.now() / 1000);
  // sdkPacks maps platform -> unzipped pack. The SDK builds every target
  // platform into one bundle; we build every one we hold an SDK for.
  const sdkPacks = opts.sdkPacks || { [opts.platform]: opts.sdkPack };
  let platforms = Object.keys(sdkPacks);
  const primary = platforms[0];

  const { prefix, pkg, others } = findProject(repoFiles, opts.appHint);
  const rel = (p) => repoFiles[prefix + p];
  const projectType = (pkg.pebble && pkg.pebble.projectType) || 'native';
  const appinfo = genAppinfoJson(pkg);
  // Block keys stay out of appinfo.json but still need C declarations.
  const allKeys = messageKeysFor(pkg.pebble && pkg.pebble.messageKeys).all;
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
      // Some repositories commit node_modules; use what is there first.
      const vendored = {};
      const nm = prefix + 'node_modules/';
      for (const [p, d] of Object.entries(repoFiles)) {
        if (!p.startsWith(nm)) continue;
        const rest = p.slice(nm.length);
        const parts = rest.split('/');
        const name = rest.startsWith('@') ? parts.slice(0, 2).join('/') : parts[0];
        const rel = rest.slice(name.length + 1);
        if (!rel) continue;
        (vendored[name] = vendored[name] || {})[rel] = d;
      }
      deps = await fetchDependencies(pkg, {
        platforms, get: opts.fetchUrl, vendored, log,
      });
    }
  }

  // ---- Moddable archive ----
  // A Moddable project's JavaScript is compiled to an XS archive and carried
  // as a raw resource named MOD, appended after the app's own resources
  // (process_sdk_resources.py). The archive holds no native code, so one
  // build covers every platform in the bundle.
  let modBytes = null;
  if (projectType === 'moddable') {
    if (mods) {
      modBytes = mods[primary];
    } else if (xsTools) {
      const { xsc, xsl, modFiles } = await xsTools();
      const files = new Map();
      for (const [path, data] of Object.entries(modFiles || {})) {
        files.set('/mod/' + path, data);
      }
      for (const [path, data] of Object.entries(repoFiles)) files.set('/proj/' + path, data);
      // A manifest may pull modules out of a dependency, so the packages we
      // fetched have to look installed even though nothing wrote them to disk.
      for (const [pkgName, pkg] of Object.entries(deps.packages)) {
        for (const [path, data] of Object.entries(pkg.files || {})) {
          files.set(`/proj/${prefix}node_modules/${pkgName}/${path}`, data);
        }
      }
      const manifestPath = findModManifest(
        Object.keys(repoFiles).filter((p) => p.startsWith(prefix)).map((p) => '/proj/' + p));
      if (!manifestPath) throw new Error('Moddable project has no manifest.json');
      modBytes = await buildMod({ manifestPath, files, xsc, xsl,
                                  transpile: opts.transpileTs, log });
    }
    if (!modBytes) {
      throw new Error('this is a Moddable project; building it needs xsc and xsl');
    }
  }

  // Everything from resources to the linked binary depends on the
  // board, so it runs once per platform being bundled.
  const perPlatform = {};
  for (const platform of platforms) {
    const sdkPack = sdkPacks[platform];
    log(`--- ${platform} ---`);
    // ---- resources ----
    let media = (appinfo.resources && appinfo.resources.media) || [];
    const publishedMedia = (appinfo.resources && appinfo.resources.publishedMedia) || [];
    if (projectType === 'moddable') {
      media = media.concat([{ type: 'raw', name: 'MOD', file: MOD_SENTINEL }]);
    }
    const resourceFiles = Object.keys(repoFiles)
      .filter((p) => p.startsWith(prefix + 'resources/'))
      .map((p) => p.slice((prefix + 'resources/').length));
    const readResource = async (path) => {
      if (path === MOD_SENTINEL) return modBytes;
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
      'build/src/resource_ids.auto.h': te.encode(genResourceIdsH(resourceNames, publishedMedia)),
      'build/src/resource_ids.auto.c': te.encode(genResourceIdsC(resourceNames, publishedMedia)),
      'build/include/message_keys.auto.h': te.encode(genMessageKeysH(allKeys)),
      'build/message_keys.auto.c': te.encode(genMessageKeysC(allKeys)),
    };
    const ldscript = te.encode(genLdScript(platform));

    // ---- project tree for the compiler ----
    const projFiles = {};
    for (const [p, data] of Object.entries(repoFiles)) {
      if (!p.startsWith(prefix)) continue;
      const q = p.slice(prefix.length);
      // .inc/.def are #included data tables; keep them visible to cpp.
      if (/^(src|include)\//.test(q) && /\.(c|h|inc|def)$/.test(q)) projFiles[q] = data;
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
    const cFiles = Object.keys(projFiles).filter((p) => p.endsWith('.c') &&
      !p.startsWith('src/js/') && !p.startsWith('src/pkjs/'));
    // The project's own wscript names its source trees; follow it when it
    // holds literal globs (src/modules next to src/c is real).
    const wscriptText = rel('wscript') ? td.decode(rel('wscript')) : null;
    const srcGlobs = wscriptSourceGlobs(wscriptText);
    let sources = null;
    if (srcGlobs) {
      // The globs pick among the repo's own sources; the generated
      // build/*.auto.c files always compile.
      const res = srcGlobs.map(globToRegExp);
      const picked = cFiles.filter((p) =>
        !p.startsWith('src/') || res.some((r) => r.test(p)));
      if (picked.some((p) => p.startsWith('src/'))) sources = picked;
    }
    // An SDK 4 project keeps its C under src/c; older ones put it straight
    // in src. A repo that has lived through both layouts still carries the
    // old copy, and building the two together gives duplicate symbols, so
    // src/c wins wherever it exists — which is what such a project's own
    // wscript globs.
    if (!sources) {
      sources = cFiles.some((p) => p.startsWith('src/c/'))
        ? cFiles.filter((p) => !p.startsWith('src/') || p.startsWith('src/c/'))
        : cFiles;
    }
    // Nothing but the generated files means the app is not written in C.
    if (!sources.some((p) => p.startsWith('src/'))) {
      const langs = new Set();
      for (const p of Object.keys(repoFiles)) {
        if (/\.rs$/.test(p)) langs.add('Rust');
        else if (/\.go$/.test(p)) langs.add('Go');
        else if (/\.(ts|js)$/.test(p) && p.includes('/src/')) langs.add('JavaScript');
      }
      throw new Error('no C sources found under src/' +
        (langs.size ? ` — this looks like a ${[...langs].join('/')} project, ` +
          'which needs its own toolchain' : ''));
    }
    const defines = (PLATFORM_DEFINES[platform] || [])
      .concat(['PBL_SDK_3', 'RELEASE']).map((d) => '-D' + d);
    // A project's own flags win over ours, and a -std of its own replaces
    // the one the SDK would otherwise pick.
    const extraCflags = wscriptCflags(wscriptText);
    const baseCflags = extraCflags.some((f) => f.startsWith('-std='))
      ? CFLAGS.filter((f) => !f.startsWith('-std=')) : CFLAGS;
    if (extraCflags.length) log(`wscript flags: ${extraCflags.join(' ')}`);
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
        [...baseCflags, ...extraCflags, ...defines, ...depIncludes,
         '-c', '/proj/' + src, '-o', '/obj/' + objName],
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
    const workerFound = Object.keys(repoFiles)
      .filter((p) => p.startsWith(prefix + 'worker_src/') && p.endsWith('.c'))
      .map((p) => p.slice(prefix.length));
    // worker_src carries the same two layouts as src, and so the same trap
    // of a repo holding both copies.
    const workerSources = workerFound.some((p) => p.startsWith('worker_src/c/'))
      ? workerFound.filter((p) => p.startsWith('worker_src/c/')) : workerFound;
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
          [...baseCflags, ...extraCflags, ...defines, ...depIncludes,
         '-c', '/proj/' + src, '-o', '/obj/' + objName],
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
  // The manifest's sdk_version has to match the firmware the platform's
  // SDK was exported from: the classic platforms are on 5.78 while the
  // current ones are on 5.106, and the constants live in each pack's own
  // pebble_process_info.h.
  const sdkVersionOf = (pack) => {
    const src = td.decode(pack['sdk/include/pebble_process_info.h'] || new Uint8Array());
    const major = src.match(/CURRENT_SDK_VERSION_MAJOR\s+(0x[0-9a-fA-F]+|\d+)/);
    const minor = src.match(/CURRENT_SDK_VERSION_MINOR\s+(0x[0-9a-fA-F]+|\d+)/);
    return { major: major ? Number(major[1]) : 5, minor: minor ? Number(minor[1]) : 106 };
  };

  const entries = { 'appinfo.json': te.encode(JSON.stringify(appinfo, null, 4)) };
  for (const plat of platforms) {
    const { pbpack, elf, workerBin } = perPlatform[plat];
    const appBin = injectMetadata(objcopyToBin(elf), elf, pbpack,
      { timestamp, allowJs: !!jsText, hasWorker: !!workerBin });
    entries[`${plat}/pebble-app.bin`] = appBin;
    entries[`${plat}/app_resources.pbpack`] = pbpack;
    entries[`${plat}/manifest.json`] = te.encode(JSON.stringify(makeManifest({
      appBin, pbpack, timestamp, sdkVersion: sdkVersionOf(sdkPacks[plat]),
      jsPresent: !!jsText,
    })));
    if (workerBin) entries[`${plat}/pebble-worker.bin`] = workerBin;
  }
  if (jsText) entries['pebble-js-app.js'] = te.encode(jsText);
  const pbw = makePbw(Object.entries(entries).map(([name, data]) => ({
    name, data, date: new Date(timestamp * 1000),
  })));
  log(`pbw ready: ${pbw.length} bytes for ${platforms.join(', ')}`);
  return { pbw, appinfo, name, platforms: perPlatform };
}
