#!/usr/bin/env bash
# Cross-compile Moddable's xsc (JavaScript -> XS bytecode) and xsl (link
# the bytecode into an mc.xsa archive) for wasm32-wasi, so a Moddable
# project can be built in the browser the same way clang.wasm builds a C
# one. The pair replaces what `mcrun -m -p pebble` does on a desktop.
#
# Usage: build-xs-tools.sh <moddable-dir> <wasi-sdk-dir> <out-dir>
set -euo pipefail

MODDABLE="${1:?path to third_party/moddable/moddable}"
WASI_SDK="${2:?path to a wasi-sdk}"
OUT="${3:?output directory}"

XS="$MODDABLE/xs"
CC="$WASI_SDK/bin/clang"
mkdir -p "$OUT"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# The wasm platform layer reaches for emscripten only to schedule promise
# jobs on a browser event loop. A compiler runs to completion and exits,
# so no-ops are enough and keep us on plain WASI.
mkdir -p "$work/shim/emscripten"
cat > "$work/shim/emscripten.h" <<'EOH'
#pragma once
typedef void (*em_timeout_cb)(void *);
static inline long emscripten_set_timeout(em_timeout_cb cb, double ms, void *ud) {
  (void)cb; (void)ms; (void)ud; return 0;
}
static inline void emscripten_clear_timeout(long id) { (void)id; }
EOH
: > "$work/shim/emscripten/html5.h"

# Moddable already carries a wasm platform layer; unlike the Linux one
# it does not drag in GLib for the debugger socket.
CFLAGS=(
  --target=wasm32-wasi -Os
  # XS unwinds with setjmp/longjmp, which on wasm needs the exception
  # handling proposal; every browser we target implements it.
  -mllvm -wasm-enable-sjlj
  # the same set the native tools build uses (build/makefiles/lin/tools.mk)
  -DXS_ARCHIVE=1 -DINCLUDE_XSPLATFORM=1 -DXSPLATFORM=\"wasm_xs.h\"
  -DXSTOOLS=1 -DmxStringInfoCacheLength=4
  -DkModdableToolsVersion=\"pebble-wasm\"
  -Wno-misleading-indentation -Wno-implicit-fallthrough -Wno-empty-body
  -I"$work/shim"
  -I"$XS/includes" -I"$XS/platforms" -I"$XS/sources" -I"$XS/tools"
  -Wno-implicit-function-declaration -Wno-int-conversion
  -Wno-incompatible-pointer-types -Wno-macro-redefined
)

sources=(
  platforms/wasm_xs.c
  sources/xsAll.c sources/xsAPI.c sources/xsArguments.c sources/xsArray.c
  sources/xsAtomics.c sources/xsBigInt.c sources/xsBoolean.c sources/xsCode.c
  sources/xsCommon.c sources/xsDataView.c sources/xsDate.c sources/xsDebug.c
  sources/xsError.c sources/xsFunction.c sources/xsGenerator.c
  sources/xsGlobal.c sources/xsJSON.c sources/xsLexical.c sources/xsMapSet.c
  sources/xsMarshall.c sources/xsMath.c sources/xsMemory.c sources/xsModule.c
  sources/xsNumber.c sources/xsObject.c sources/xsPlatforms.c
  sources/xsProfile.c sources/xsPromise.c sources/xsProperty.c
  sources/xsProxy.c sources/xsRegExp.c sources/xsRun.c sources/xsScope.c
  sources/xsScript.c sources/xsSourceMap.c sources/xsString.c
  sources/xsSymbol.c sources/xsSyntaxical.c sources/xsTree.c sources/xsType.c
  sources/xsdtoa.c sources/xsmc.c sources/xsre.c
)

echo "compiling ${#sources[@]} shared sources for wasm32-wasi..."
objs=()
for s in "${sources[@]}"; do
  o="$work/$(echo "$s" | tr / _).o"
  "$CC" "${CFLAGS[@]}" -c "$XS/$s" -o "$o"
  objs+=("$o")
done

# xsa.c, xsc.c and xsl.c each carry their own main and their own copies
# of the reporting helpers, so a tool links exactly one of them.
echo "linking xsc.wasm..."
"$CC" "${CFLAGS[@]}" -c "$XS/tools/xsc.c" -o "$work/main_xsc.o"
"$CC" --target=wasm32-wasi -mllvm -wasm-enable-sjlj \
  -o "$OUT/xsc.wasm" "$work/main_xsc.o" "${objs[@]}" -lm

echo "linking xsl.wasm..."
xsl_objs=()
for e in xslBase.c xslOpt.c xslSlot.c xslStrip.c; do
  "$CC" "${CFLAGS[@]}" -c "$XS/tools/$e" -o "$work/tool_$e.o"
  xsl_objs+=("$work/tool_$e.o")
done
"$CC" "${CFLAGS[@]}" -c "$XS/tools/xsl.c" -o "$work/main_xsl.o"
"$CC" --target=wasm32-wasi -mllvm -wasm-enable-sjlj \
  -o "$OUT/xsl.wasm" "$work/main_xsl.o" "${xsl_objs[@]}" "${objs[@]}" -lm

ls -la "$OUT/xsc.wasm" "$OUT/xsl.wasm"
echo "done: $OUT/xsc.wasm $OUT/xsl.wasm"
