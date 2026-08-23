// Build the mc.xsa archive of a Moddable (XS) Pebble project.
//
// On a desktop this is `mcrun -m -p pebble`: it resolves the project's
// manifest, compiles every JavaScript module to XS bytecode with xsc, and
// links the results into an archive with xsl. The SDK then embeds that
// archive as the app's MOD resource and the firmware's XS virtual machine
// runs it. Both tools are here as wasm, so the whole thing happens in the
// browser; what this file replaces is mcrun's manifest handling.
//
// Only the mod subset of the manifest format is implemented — modules,
// resources, data, includes and platform overrides. A mod cannot add native
// code or preload modules, so the parts of a manifest that describe a
// firmware build have nothing to do here.
import { Directory, File } from '../vendor/wasi/index.js';
import { runTool, readTree } from './wasi-run.js';

// mcrun signs archives with the reversed namespace and the manifest's
// directory name; the default namespace comes from the Moddable SDK.
const NAMESPACE = 'moddable.tech';

// What xsc can compile. TypeScript declarations describe types for the
// editor and produce no module, which is why a typings directory full of
// .d.ts files contributes only its handful of .json files. A manifest is
// JSON too, so a directory glob would otherwise turn the manifest driving
// the build into a module of its own.
const MODULE_EXTS = ['.js', '.mjs', '.json', '.ts'];
const isModule = (p) =>
  MODULE_EXTS.some((e) => p.endsWith(e)) && !p.endsWith('.d.ts') &&
  !(p.endsWith('.json') && basename(p).startsWith('manifest'));

const dirname = (p) => p.slice(0, p.lastIndexOf('/'));
const basename = (p) => p.slice(p.lastIndexOf('/') + 1);
const stem = (p) => {
  const b = basename(p), dot = b.lastIndexOf('.');
  return dot > 0 ? b.slice(0, dot) : b;
};

// Collapse '.' and '..' so paths from different manifests compare equal.
function normalize(path) {
  const out = [];
  for (const part of path.split('/')) {
    if (!part || part === '.') continue;
    if (part === '..') out.pop();
    else out.push(part);
  }
  return '/' + out.join('/');
}

function resolve(base, path) {
  return path.startsWith('/') ? normalize(path) : normalize(base + '/' + path);
}

// $(VAR) references may nest, so expand until nothing changes.
function expand(str, vars) {
  let out = str;
  for (let i = 0; i < 8 && out.includes('$('); i++) {
    out = out.replace(/\$\(([A-Za-z_][A-Za-z0-9_]*)\)/g,
                      (m, name) => (name in vars ? vars[name] : m));
  }
  return out;
}

// A manifest value is a string or an array of strings; treat both alike.
const list = (v) => (v === undefined ? [] : Array.isArray(v) ? v : [v]);

// Resolve a source path with no extension against the files we hold.
function findSource(files, path) {
  if (files.has(path) && isModule(path)) return path;
  for (const ext of MODULE_EXTS) {
    if (files.has(path + ext) && isModule(path + ext)) return path + ext;
  }
  return null;
}

// '/dir/*' — every compilable module directly inside the directory.
function globDir(files, dir) {
  const prefix = dir + '/';
  const found = [];
  for (const path of files.keys()) {
    if (!path.startsWith(prefix) || path.slice(prefix.length).includes('/')) continue;
    if (!isModule(path)) continue;
    found.push(path);
  }
  return found.sort();
}

// Walk a manifest and everything it includes, collecting the module,
// resource and data entries a mod contributes. Includes are processed
// before the manifest's own entries, which is the order mcrun links in.
function collectManifest(start, files, platform, vars, log) {
  const modules = new Map();   // target specifier -> source path
  const resources = new Map(); // target file name  -> source path
  const data = new Map();
  const seen = new Set();
  let config = {};

  const addEntries = (into, entries, dir, vars, compilable) => {
    for (const [key, value] of Object.entries(entries || {})) {
      for (const raw of list(value)) {
        const path = resolve(dir, expand(raw, vars));
        // 'key/*': 'dir/*' maps a whole directory under a prefix
        if (path.endsWith('/*')) {
          const prefix = key === '*' ? '' : key.slice(0, -1);
          for (const src of globDir(files, path.slice(0, -2))) {
            into.set(prefix + stem(src), src);
          }
          continue;
        }
        const src = compilable ? findSource(files, path) : findExact(files, path);
        if (!src) {
          log(`  manifest: ${raw} not found, skipping`);
          continue;
        }
        into.set(key === '*' ? (compilable ? stem(src) : basename(src)) : key, src);
      }
    }
  };

  // Resources keep their extension and are stored verbatim, so a manifest
  // entry without one has to be matched against whatever is on disk.
  const findExact = (fs, path) => {
    if (fs.has(path)) return path;
    const dir = dirname(path) + '/', want = basename(path) + '.';
    for (const p of fs.keys()) {
      if (p.startsWith(dir) && !p.slice(dir.length).includes('/') &&
          p.startsWith(dir + want)) return p;
    }
    return null;
  };

  const visit = (path, vars) => {
    if (seen.has(path)) return;
    seen.add(path);
    const raw = files.get(path);
    if (!raw) throw new Error(`manifest not found: ${path}`);
    let manifest;
    try {
      manifest = JSON.parse(new TextDecoder().decode(raw));
    } catch (e) {
      throw new Error(`${path}: ${e.message}`);
    }
    const dir = dirname(path);
    const scope = { ...vars };
    for (const [k, v] of Object.entries(manifest.build || {})) scope[k] = expand(v, scope);

    for (const inc of list(manifest.include)) visit(resolve(dir, expand(inc, scope)), scope);

    // A platform section overrides the manifest it appears in; '...' is the
    // catch-all for platforms with no section of their own.
    const platforms = manifest.platforms || {};
    const override = platforms[platform] || platforms['...'] || {};
    for (const source of [manifest, override]) {
      addEntries(modules, source.modules, dir, scope, true);
      addEntries(resources, source.resources, dir, scope, false);
      addEntries(data, source.data, dir, scope, false);
      Object.assign(config, source.config || {});
    }
  };

  visit(start, vars);
  return { modules, resources, data, config };
}

// mcrun writes this alongside the project's own modules; the host calls it
// before running the mod. Asset-format checks only apply to builds that
// convert images, which a mod archive never does.
const CHECK_JS = `/* WARNING: This file is automatically generated. Do not edit. */

import config from "mc/config";

export default function() {
}
`;

function configJs(config) {
  let out = '/* WARNING: This file is automatically generated. Do not edit. */\n\nexport default {\n';
  for (const key of Object.keys(config)) {
    out += `\t"${key}": ${JSON.stringify(config[key], null, '\t')},\n`;
  }
  return out + '};\n';
}

// Make (and return) a directory inside a wasi-shim tree.
function mkdirp(root, path) {
  let dir = root;
  for (const part of path.split('/').filter(Boolean)) {
    if (!dir.has(part)) dir.set(part, new Directory(new Map()));
    dir = dir.get(part).contents;
  }
  return dir;
}

// files: Map of absolute virtual path -> Uint8Array, covering both the
// project (rooted wherever manifestPath lives) and the Moddable SDK files
// the manifest may include, rooted at moddableRoot ($(MODDABLE)).
export async function buildMod(opts) {
  const { manifestPath, files, xsc, xsl, platform = 'pebble',
          moddableRoot = '/mod', log = () => {} } = opts;

  const { modules, resources, data, config } =
    collectManifest(manifestPath, files, platform, { MODDABLE: moddableRoot }, log);
  if (!modules.size) throw new Error(`${manifestPath}: no JavaScript modules`);

  // xsc and xsl only see /w: sources go in as they are named so error
  // messages point at real paths, outputs land in /w/files and /w/bin.
  const root = new Map();
  for (const source of new Set([...modules.values(), ...resources.values(), ...data.values()])) {
    mkdirp(root, dirname(source)).set(basename(source), new File(files.get(source)));
  }
  const enc = new TextEncoder();
  const gen = mkdirp(root, '/gen');
  gen.set('check.js', new File(enc.encode(CHECK_JS)));
  modules.set('check', '/gen/check.js');
  if (Object.keys(config).length) {
    gen.set('config.js', new File(enc.encode(configJs(config))));
    modules.set('mod/config', '/gen/config.js');
  }
  const filesDir = mkdirp(root, '/files');
  mkdirp(root, '/bin');
  const mounts = { '/w': root };

  log(`moddable: ${modules.size} modules, ${resources.size + data.size} resources`);
  for (const [target, source] of modules) {
    mkdirp(root, '/files/' + (target.includes('/') ? dirname(target) : ''));
    const outDir = '/w/files' + (target.includes('/') ? '/' + dirname(target) : '');
    const { code, stderr } = await runTool(
      xsc, 'xsc', ['/w' + source, '-e', '-o', outDir, '-r', basename(target)],
      mounts, (l) => log('  ' + l));
    if (code) throw new Error(`xsc failed on ${source}\n${stderr}`);
  }

  // Resources and data are stored in the archive untouched.
  for (const [target, source] of [...data, ...resources]) {
    mkdirp(root, '/files/' + (target.includes('/') ? dirname(target) : ''))
      .set(basename(target), new File(files.get(source)));
  }

  const signature = NAMESPACE.split('.').reverse().concat(basename(dirname(manifestPath)))
    .join('.');
  const args = ['-a', '-b', '/w/files', '-n', signature, '-o', '/w/bin'];
  for (const target of [...data.keys(), ...modules.keys()]) {
    args.push('/w/files/' + target + (data.has(target) ? '' : '.xsb'));
  }
  for (const target of resources.keys()) args.push('/w/files/' + target);

  const { code, stderr } = await runTool(xsl, 'xsl', args, mounts, (l) => log('  ' + l));
  if (code) throw new Error(`xsl failed\n${stderr}`);
  const xsa = readTree(root, 'bin/mc.xsa');
  if (!xsa) throw new Error('xsl produced no mc.xsa');
  log(`moddable: mc.xsa ${xsa.length} bytes`);
  return xsa;
}

// Locate a Moddable project's manifest inside a repo. Every Pebble mod
// keeps it under src/embeddedjs, but accept any manifest.json that sits
// next to JavaScript rather than assuming the layout.
export function findModManifest(paths) {
  const candidates = paths.filter((p) => p.endsWith('/manifest.json') || p === 'manifest.json');
  return candidates.find((p) => p.includes('embeddedjs')) ||
         candidates.sort((a, b) => a.split('/').length - b.split('/').length)[0] || null;
}
