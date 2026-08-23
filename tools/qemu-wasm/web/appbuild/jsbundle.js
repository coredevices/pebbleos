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
const REQUIRE_SHIM = `var require = typeof require !== 'undefined' ? require : function (m) {
  if (m === 'message_keys') return __pebble_message_keys;
  throw new Error('Module not found: ' + m);
};
`;

export async function bundlePkjs(esbuild, opts) {
  const { entryPath, prefix, files, pkg, appKeys, sharedAdditions } = opts;
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

  const plugin = {
    name: 'pebble-repo',
    setup(build) {
      build.onResolve({ filter: /^(message_keys|app_package\.json)$/ }, (a) => virtual(a.path));
      build.onResolve({ filter: /.*/ }, (args) => {
        if (args.namespace === 'virtual') return null;
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
          // bare specifier: the SDK puts src/common on the resolve path
          base = resolveFile('src/common/' + args.path) !== null
            ? 'src/common/' + args.path : args.path;
        }
        const found = resolveFile(base);
        if (!found) {
          return { errors: [{ text: `cannot resolve "${args.path}" from ${args.importer}` }] };
        }
        return { path: found, namespace: 'repo' };
      });
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
