#!/usr/bin/env bash
# Cross-compile Moddable's xsc (JavaScript -> XS bytecode) and xsl (link the
# bytecode into an mc.xsa archive) for wasm32-wasi, so a Moddable project can
# be built in the browser the same way clang.wasm builds a C one. The pair is
# what `mcrun -m -p pebble` shells out to on a desktop.
#
# Each tool mirrors xs/makefiles/lin/{xsc,xsl}.mk: the defines and the object
# list differ between them, and getting either wrong produces confusing type
# errors deep inside the sources.
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

# XS's platform headers gate on the host architecture and reject anything they
# do not know. EMSCRIPTEN is the one non-native target they already accept, and
# it selects exactly the branch we want: plain C library, no Windows sockets.
# xslOpt.h includes the headers a desktop debugger connection needs. WASI has
# sockets but no name resolution, so netdb.h is missing; xsl only ever uses it
# when it opens a debugger connection, which the linker never does.
mkdir -p "$work/shim"
echo '#pragma once' > "$work/shim/netdb.h"

# On Linux xsAll.c decorates profiler frames with the native symbol name via
# dladdr. There is no dynamic linker under WASI and nothing profiles the
# linker, so a stub that always fails keeps the anonymous-frame fallback.
cat > "$work/shim/xs_wasi_compat.h" <<'EOH'
#pragma once
typedef struct { const char *dli_fname, *dli_sname; void *dli_fbase, *dli_saddr; } Dl_info;
static inline int dladdr(const void *addr, Dl_info *info) { (void)addr; (void)info; return 0; }
EOH

# fxCStackLimit's Linux branch measures the thread stack through a glibc
# extension wasi-libc does not carry. Failing the call makes XS fall back to
# no C-stack limit, which is what the other single-stack platforms do.
cat > "$work/shim/stubs.c" <<'EOC'
#include <pthread.h>
int pthread_getattr_np(pthread_t thread, pthread_attr_t *attr);
int pthread_getattr_np(pthread_t thread, pthread_attr_t *attr) {
  (void)thread; (void)attr; return -1;
}
EOC

COMMON=(
  --target=wasm32-wasi -O2
  # XS unwinds with setjmp/longjmp, which on wasm needs the exception handling
  # proposal; every browser we target implements it.
  -mllvm -wasm-enable-sjlj
  -DEMSCRIPTEN=1 -fno-common
  # xslOpt.h pulls in signal.h for the desktop debugger's interrupt handling
  -D_WASI_EMULATED_SIGNAL
  -include xs_wasi_compat.h
  -I"$work/shim"
  -I"$XS/includes" -I"$XS/platforms" -I"$XS/sources" -I"$XS/tools"
  -Wno-misleading-indentation -Wno-implicit-fallthrough
  -Wno-implicit-function-declaration -Wno-int-conversion
  -Wno-incompatible-pointer-types -Wno-macro-redefined
)

# $1 = tool name, $2 = extra defines (space separated), rest = source list
build_tool() {
  local name="$1"; shift
  local defines="$1"; shift
  local objs=() src o
  echo "compiling $name ($# sources)..."
  for src in "$@"; do
    o="$work/${name}_$(echo "$src" | tr / _).o"
    # a leading / names one of the shim sources above, anything else is XS
    # shellcheck disable=SC2086
    case "$src" in
      /*) "$CC" "${COMMON[@]}" ${defines} -c "$src" -o "$o" ;;
      *)  "$CC" "${COMMON[@]}" ${defines} -c "$XS/$src" -o "$o" ;;
    esac
    objs+=("$o")
  done
  echo "linking $name.wasm..."
  "$CC" --target=wasm32-wasi -mllvm -wasm-enable-sjlj \
    -o "$OUT/$name.wasm" "${objs[@]}" -lm -lwasi-emulated-signal
}

# xsc: the JavaScript front end only. It parses, builds a syntax tree and
# emits bytecode, so it needs neither the runtime nor a platform header.
build_tool xsc '-DmxCompile=1' \
  sources/xsBigInt.c sources/xsCode.c sources/xsCommon.c sources/xsdtoa.c \
  sources/xsLexical.c sources/xsre.c sources/xsScope.c sources/xsScript.c \
  sources/xsSourceMap.c sources/xsSyntaxical.c sources/xsTree.c \
  tools/xsc.c

# xsl: the linker runs a real XS machine to preload modules, so it pulls in
# the whole runtime. xslOpt.h stands in for the platform header (that is where
# sxMachine's fakeCallback comes from) and mxLink=1 switches the runtime over
# to the linking behaviour.
build_tool xsl \
  '-DINCLUDE_XSPLATFORM -DXSPLATFORM="xslOpt.h" -DmxLink=1' \
  sources/xsAll.c sources/xsAPI.c sources/xsArguments.c sources/xsArray.c \
  sources/xsAtomics.c sources/xsBigInt.c sources/xsBoolean.c sources/xsCode.c \
  sources/xsCommon.c sources/xsDataView.c sources/xsDate.c sources/xsDebug.c \
  sources/xsDefaults.c sources/xsError.c sources/xsFunction.c \
  sources/xsGenerator.c sources/xsGlobal.c sources/xsJSON.c \
  sources/xsLexical.c sources/xsMapSet.c sources/xsMarshall.c \
  sources/xsMath.c sources/xsMemory.c sources/xsModule.c sources/xsNumber.c \
  sources/xsObject.c sources/xsPlatforms.c sources/xsPromise.c \
  sources/xsProperty.c sources/xsProxy.c sources/xsRegExp.c sources/xsRun.c \
  sources/xsScope.c sources/xsScript.c sources/xsSourceMap.c \
  sources/xsString.c sources/xsSymbol.c sources/xsSyntaxical.c \
  sources/xsTree.c sources/xsType.c sources/xsdtoa.c sources/xsre.c \
  tools/xslBase.c tools/xslOpt.c tools/xslSlot.c tools/xslStrip.c tools/xsl.c \
  "$work/shim/stubs.c"

ls -la "$OUT/xsc.wasm" "$OUT/xsl.wasm"
echo "done: $OUT/xsc.wasm $OUT/xsl.wasm"
