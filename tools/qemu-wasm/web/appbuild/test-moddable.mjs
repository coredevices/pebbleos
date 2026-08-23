// Validate moddable.js against golden `mcrun -m -p pebble` builds.
//
// The archive is the whole output, so byte equality with what the desktop
// tools produce is the only check that matters.
//
//   node test-moddable.mjs <xstools-dir> <moddable-sdk-dir> <golden-dir> <projects-dir>
import { readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import { buildMod, findModManifest } from './moddable.js';

const [TOOLS, SDK, GOLD, PROJECTS] = [
  process.argv[2] || '/tmp/xstools',
  process.argv[3] || '/home/user/pebble-v4350/third_party/moddable/moddable',
  process.argv[4] || '/tmp/modgold',
  process.argv[5] || '/tmp/modtest',
];

const xsc = await WebAssembly.compile(readFileSync(`${TOOLS}/xsc.wasm`));
const xsl = await WebAssembly.compile(readFileSync(`${TOOLS}/xsl.wasm`));

// Everything a manifest may reach lives in one virtual filesystem.
function collect(hostDir, virtualDir, into, skip = () => false) {
  for (const entry of readdirSync(hostDir)) {
    const host = `${hostDir}/${entry}`, virt = `${virtualDir}/${entry}`;
    if (skip(virt)) continue;
    if (statSync(host).isDirectory()) collect(host, virt, into, skip);
    else into.set(virt, new Uint8Array(readFileSync(host)));
  }
  return into;
}

// $(MODDABLE) resolves here; only the manifests and typings a mod can
// reference are needed, not the whole SDK.
const sdkFiles = new Map();
collect(`${SDK}/examples`, '/mod/examples', sdkFiles,
        (p) => !p.endsWith('.json') || !p.includes('manifest'));
collect(`${SDK}/typings`, '/mod/typings', sdkFiles, (p) => !p.endsWith('.json'));

let failures = 0;
for (const name of readdirSync(PROJECTS)) {
  const golden = `${GOLD}/${name}/mc.xsa`;
  if (!existsSync(golden)) { console.log(`SKIP ${name} (no golden archive)`); continue; }

  const files = new Map(sdkFiles);
  collect(`${PROJECTS}/${name}`, '/proj', files, (p) => p.includes('/.git'));
  const manifest = findModManifest([...files.keys()].filter((p) => p.startsWith('/proj')));

  let xsa = null, error = null;
  try {
    xsa = await buildMod({ manifestPath: manifest, files, xsc, xsl,
                           log: (l) => process.env.VERBOSE && console.log('   ', l) });
  } catch (e) { error = e.message; }

  const want = new Uint8Array(readFileSync(golden));
  const ok = xsa && Buffer.compare(Buffer.from(xsa), Buffer.from(want)) === 0;
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'} ${name.padEnd(24)} ` +
              (error ? error.split('\n')[0]
                     : `${xsa.length} bytes vs golden ${want.length}`));
}
console.log(`\n${failures ? failures + ' FAILED' : 'all archives byte-identical'}`);
process.exit(failures ? 1 : 0);
