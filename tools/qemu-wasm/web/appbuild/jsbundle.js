// Bundle a multiJS app's PebbleKit JS with esbuild (wasm build), the
// browser stand-in for the SDK's webpack step (waflib/extras/process_js.py).
// Mirrors it in the ways that matter: _pkjs_shared_additions.js runs first,
// 'message_keys' resolves to the app's numeric keys, 'app_package.json'
// resolves to the project's package.json, and src/common is on the path.
const td = new TextDecoder();

// webpack ProvidePlugin installs _message_key_wrapper as the global
// `require` so a plain (non-module) pkjs script can still ask for
// message_keys. esbuild resolves static require() calls itself; this only
// backstops the dynamic ones.
// The SDK bundles PebbleKit JS with webpack's target: 'node', which leaves
// node's own modules as runtime requires instead of failing the build. npm
// packages reach for them constantly — usually in a branch that only runs
// under node — so a package that merely mentions 'util' must not stop an
// app from building. Anything that really does call one fails at runtime
// with the message below, which is what the SDK does too.
const NODE_BUILTINS = [
  'assert', 'async_hooks', 'buffer', 'child_process', 'cluster', 'console',
  'constants', 'crypto', 'dgram', 'dns', 'domain', 'events', 'fs', 'http',
  'http2', 'https', 'module', 'net', 'os', 'path', 'perf_hooks', 'process',
  'punycode', 'querystring', 'readline', 'repl', 'stream', 'string_decoder',
  'sys', 'timers', 'tls', 'tty', 'url', 'util', 'v8', 'vm', 'worker_threads',
  'zlib',
];
const EXTERNALS = NODE_BUILTINS.concat(NODE_BUILTINS.map((m) => 'node:' + m));

const REQUIRE_SHIM = `var require = typeof require !== 'undefined' ? require : function (m) {
  if (m === 'message_keys') return __pebble_message_keys;
  throw new Error('Module not found: ' + m);
};
`;

// True when the source requires `spec` only from inside try {} blocks.
// Node lets a try/catch absorb a missing module, and pkjs authors use
// that for optional dev-only files (which are often gitignored); such a
// require must not fail the build, it fails at runtime into the catch.
export function requiredOnlyInTry(src, spec) {
  const needles = [`'${spec}'`, `"${spec}"`];
  const hits = [];
  for (const n of needles) {
    for (let i = src.indexOf(n); i !== -1; i = src.indexOf(n, i + 1)) hits.push(i);
  }
  if (!hits.length) return false;
  hits.sort((a, b) => a - b);
  // One pass over the source, tracking whether each brace opened a try
  // block. Strings and comments are skipped so their braces don't count.
  const stack = [];
  let h = 0, lastWord = '';
  for (let i = 0; i < src.length && h < hits.length; i++) {
    while (h < hits.length && hits[h] <= i) {
      if (!stack.some(Boolean)) return false;
      h++;
    }
    const c = src[i];
    if (c === '/' && src[i + 1] === '/') {
      i = src.indexOf('\n', i);
      if (i === -1) break;
    } else if (c === '/' && src[i + 1] === '*') {
      i = src.indexOf('*/', i + 2);
      if (i === -1) break;
      i++;
    } else if (c === "'" || c === '"' || c === '`') {
      const start = i;
      for (i++; i < src.length && src[i] !== c; i++) {
        if (src[i] === '\\') i++;
      }
      // A string can be the spec itself; let the hit check above see it.
      while (h < hits.length && hits[h] >= start && hits[h] <= i) {
        if (!stack.some(Boolean)) return false;
        h++;
      }
    } else if (c === '{') {
      stack.push(lastWord === 'try');
      lastWord = '';
    } else if (c === '}') {
      stack.pop();
      lastWord = '';
    } else if (/[A-Za-z_$]/.test(c)) {
      let j = i;
      while (j < src.length && /[A-Za-z0-9_$]/.test(src[j])) j++;
      lastWord = src.slice(i, j);
      i = j - 1;
    } else if (!/\s/.test(c)) {
      lastWord = '';
    }
  }
  return h >= hits.length;
}

export async function bundlePkjs(esbuild, opts) {
  const { entryPath, prefix, files, pkg, appKeys, sharedAdditions, packages = {} } = opts;
  const read = (p) => files[prefix + p];
  const exists = (p) => read(p) !== undefined;

  const resolveFile = (base) => {
    for (const cand of [base, base + '.js', base + '.json',
                        base + '/index.js', base + '/index.json']) {
      if (exists(cand)) return cand;
    }
    return null;
  };

  const virtual = (path) => ({ path, namespace: 'virtual' });

  // Resolve into a fetched npm package: "name" or "name/sub/path".
  const resolveInPackage = (name, sub) => {
    const p = packages[name];
    if (!p) return null;
    // An SDK library publishes its JS under dist/js, which the SDK aliases
    // to the package name itself.
    const cands = sub
      ? [sub, sub + '.js', sub + '.json', sub + '/index.js']
      : [p.main, 'index.js', 'index.json'].filter(Boolean);
    for (const c of cands) {
      if (p.files[c] !== undefined) return { path: `${name}\u0000${c}`, namespace: 'pkg' };
    }
    return null;
  };
  const resolvePackage = (spec) => {
    const parts = spec.split('/');
    const name = spec.startsWith('@') ? parts.slice(0, 2).join('/') : parts[0];
    const sub = spec.slice(name.length + 1);
    return resolveInPackage(name, sub);
  };

  const plugin = {
    name: 'pebble-repo',
    setup(build) {
      build.onResolve({ filter: /^(message_keys|app_package\.json)$/ }, (a) => virtual(a.path));
      // esbuild's own `external` never gets a say: a plugin's onResolve runs
      // first, so the builtins have to be answered here or the error below
      // fires instead.
      const external = (args) =>
        (EXTERNALS.includes(args.path) ? { path: args.path, external: true } : null);
      build.onResolve({ filter: /.*/ }, (args) => {
        // This runs for every namespace, so it has to stand aside for the
        // ones with a resolver of their own further down. Without that, a
        // package's own './lib/decoder' was looked for in the repo.
        if (args.namespace === 'virtual' || args.namespace === 'pkg') return null;
        const ext = external(args);
        if (ext) return ext;
        let base;
        if (args.path.startsWith('./') || args.path.startsWith('../')) {
          const dir = args.importer.includes('/')
            ? args.importer.slice(0, args.importer.lastIndexOf('/')) : '';
          const out = [];
          for (const part of (dir + '/' + args.path).split('/')) {
            if (part === '' || part === '.') continue;
            if (part === '..') out.pop();
            else out.push(part);
          }
          base = out.join('/');
        } else if (args.kind === 'entry-point') {
          base = args.path;
        } else {
          // A bare specifier is an npm package first, then src/common,
          // which the SDK also puts on the resolve path.
          const hit = resolvePackage(args.path);
          if (hit) return hit;
          base = resolveFile('src/common/' + args.path) !== null
            ? 'src/common/' + args.path : args.path;
        }
        const found = resolveFile(base);
        if (!found) {
          const src = read(args.importer);
          if (src && requiredOnlyInTry(td.decode(src), args.path)) {
            return { path: args.path, namespace: 'missing' };
          }
          return { errors: [{ text: `cannot resolve "${args.path}" from ${args.importer}` }] };
        }
        return { path: found, namespace: 'repo' };
      });
      build.onResolve({ filter: /.*/, namespace: 'pkg' }, (args) => {
        const ext = external(args);
        if (ext) return ext;
        // Relative import inside a package stays inside it.
        const owner = args.importer.split('\u0000')[0];
        const dir = args.importer.split('\u0000')[1] || '';
        let hit;
        if (args.path.startsWith('.')) {
          const base = dir.includes('/') ? dir.slice(0, dir.lastIndexOf('/')) : '';
          const out = [];
          for (const part of (base + '/' + args.path).split('/')) {
            if (part === '' || part === '.') continue;
            if (part === '..') out.pop();
            else out.push(part);
          }
          hit = resolveInPackage(owner, out.join('/'));
        } else {
          hit = resolvePackage(args.path);
        }
        if (!hit) {
          const src = packages[owner] && packages[owner].files[dir];
          if (src && requiredOnlyInTry(td.decode(src), args.path)) {
            return { path: args.path, namespace: 'missing' };
          }
        }
        return hit;
      });
      build.onLoad({ filter: /.*/, namespace: 'pkg' }, (args) => {
        const [name, rel] = args.path.split('\u0000');
        return {
          contents: td.decode(packages[name].files[rel]),
          loader: rel.endsWith('.json') ? 'json' : 'js',
        };
      });
      // A module only ever required inside try {}: fail at runtime, into
      // the catch, the way node would for a genuinely absent file.
      build.onLoad({ filter: /.*/, namespace: 'missing' }, (args) => ({
        contents: `throw new Error(${JSON.stringify(
          `Cannot find module '${args.path}'`)});`,
        loader: 'js',
      }));
      build.onLoad({ filter: /.*/, namespace: 'repo' }, (args) => ({
        contents: td.decode(read(args.path)),
        loader: args.path.endsWith('.json') ? 'json' : 'js',
      }));
      build.onLoad({ filter: /.*/, namespace: 'virtual' }, (args) => {
        if (args.path === 'message_keys') {
          return { contents: JSON.stringify(appKeys || {}), loader: 'json' };
        }
        return { contents: JSON.stringify(pkg || {}), loader: 'json' };
      });
    },
  };

  const result = await esbuild.build({
    entryPoints: [entryPath],
    bundle: true,
    write: false,
    format: 'iife',
    platform: 'browser',
    target: 'es2017',
    banner: { js: `var __pebble_message_keys = ${JSON.stringify(appKeys || {})};\n` + REQUIRE_SHIM },
    plugins: [plugin],
    logLevel: 'silent',
  });
  if (result.errors && result.errors.length) {
    throw new Error('JS bundling failed: ' + result.errors.map((e) => e.text).join('; '));
  }
  const body = result.outputFiles[0].text;
  // The SDK makes _pkjs_shared_additions.js the first webpack entry, so
  // its Pebble.on/off aliases exist before app code runs.
  return sharedAdditions ? sharedAdditions + '\n' + body : body;
}
