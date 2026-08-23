// Run a WASI-compiled LLVM tool (clang.wasm / lld.wasm) against an
// in-memory filesystem, in the browser or under node. The modules are
// wasm32-wasi-threads builds: they import shared linear memory and a
// wasi thread-spawn hook, but never actually spawn threads
// (LLVM_ENABLE_THREADS=OFF), so a stub suffices.
import {
  WASI, WASIProcExit, PreopenDirectory, Directory, File, ConsoleStdout,
} from '../vendor/wasi/index.js';

// Build a Directory tree from {path: Uint8Array} (paths may contain /).
export function makeTree(files) {
  const root = new Map();
  for (const [path, data] of Object.entries(files)) {
    const parts = path.split('/').filter(Boolean);
    let dir = root;
    for (let i = 0; i < parts.length - 1; i++) {
      if (!dir.has(parts[i])) dir.set(parts[i], new Directory(new Map()));
      dir = dir.get(parts[i]).contents;
    }
    dir.set(parts[parts.length - 1], new File(data));
  }
  return root;
}

// Read a file back out of a mount tree ('a/b/c.o'), or null.
export function readTree(tree, path) {
  const parts = path.split('/').filter(Boolean);
  let dir = tree;
  for (let i = 0; i < parts.length - 1; i++) {
    const d = dir.get(parts[i]);
    if (!d) return null;
    dir = d.contents;
  }
  const f = dir.get(parts[parts.length - 1]);
  return f && f.data ? f.data : null;
}

// mounts: {guestDir: Map (from makeTree) | {files}} — '/obj' etc.
// Returns {code, stderr}.
export async function runTool(module, argv0, args, mounts, log = () => {}) {
  const fds = [
    new ConsoleStdout(() => {}), // stdin (never read)
    ConsoleStdout.lineBuffered((l) => log(l)),
    ConsoleStdout.lineBuffered((l) => log(l)),
  ];
  const stderrLines = [];
  fds[2] = ConsoleStdout.lineBuffered((l) => { stderrLines.push(l); log(l); });

  for (const [guest, tree] of Object.entries(mounts)) {
    const contents = tree instanceof Map ? tree : makeTree(tree);
    fds.push(new PreopenDirectory(guest, contents));
  }

  // debug defaults to on when the options object is omitted, and it logs
  // once per path lookup — far too noisy (and slow) for a real compile.
  const wasi = new WASI([argv0, ...args], [], fds, { debug: false });
  const imports = { wasi_snapshot_preview1: wasi.wasiImport };
  let memory = null;
  for (const im of WebAssembly.Module.imports(module)) {
    if (im.module === 'env' && im.name === 'memory') {
      memory = new WebAssembly.Memory({ initial: 1088, maximum: 65536, shared: true });
      imports.env = { memory };
    }
    if (im.module === 'wasi' && im.name === 'thread-spawn') {
      imports.wasi = { 'thread-spawn': () => -1 };
    }
  }

  const instance = await WebAssembly.instantiate(module, imports);
  // The shim reads inst.exports.memory; threads builds import it instead.
  const exports = Object.create(null);
  Object.assign(exports, instance.exports);
  if (!exports.memory && memory) exports.memory = memory;

  let code = 0;
  try {
    code = wasi.start({ exports });
  } catch (e) {
    if (e instanceof WASIProcExit) code = e.code;
    else { log('tool crashed: ' + (e.message || e)); code = 70; }
  }
  return { code, stderr: stderrLines.join('\n') };
}
