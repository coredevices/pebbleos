#!/usr/bin/env bash
# Assemble sdkpack.zip: everything the in-browser compiler needs beyond
# clang.wasm/lld.wasm. Layout inside the zip:
#   sdk/include/**  sdk/lib/libpebble.a     (exported SDK, one platform)
#   newlib/**                               (newlib C headers, no c++)
#   libgcc.a libc.a libm.a                  (arm-none-eabi thumb/v7-m)
#   clang-res/include/*.h                   (clang builtin headers, pruned)
#   js/*.js                                 (SDK pkjs helper modules)
#
# Usage: make-sdkpack.sh <sdk-platform-dir> <clang-res-dir> <out.zip> [sdk-common-dir]
set -euo pipefail

SDK="${1:?sdk platform dir (build/sdk/emery)}"
CLANGRES="${2:?clang-res dir}"
COMMON="${4:-}"
OUT="$(cd "$(dirname "${3:?out.zip}")" && pwd)/$(basename "$3")"

GCC_LIB="$(arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -print-libgcc-file-name)"
C_LIB="$(arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -print-file-name=libc.a)"
M_LIB="$(arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -print-file-name=libm.a)"

stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT

mkdir -p "$stage/sdk/include" "$stage/sdk/lib" "$stage/newlib" "$stage/clang-res/include"
cp -r "$SDK/include/." "$stage/sdk/include/"
rm -rf "$stage/sdk/include/doxygen"
cp "$SDK/lib/libpebble.a" "$stage/sdk/lib/"
cp -r /usr/include/newlib/. "$stage/newlib/"
rm -rf "$stage/newlib/c++"
cp "$GCC_LIB" "$stage/libgcc.a"
cp "$C_LIB" "$stage/libc.a"
cp "$M_LIB" "$stage/libm.a"
if [ -n "$COMMON" ] && [ -d "$COMMON/include" ]; then
  mkdir -p "$stage/js"
  cp "$COMMON"/include/*.js "$stage/js/"
fi
cd "$CLANGRES/include"
cp stddef.h stdarg.h stdbool.h stdint.h float.h limits.h stdalign.h \
   stdnoreturn.h iso646.h inttypes.h stdatomic.h varargs.h \
   __stddef_*.h __stdarg_*.h "$stage/clang-res/include/" 2>/dev/null || true

cd "$stage"
rm -f "$OUT"
zip -qr9 "$OUT" .
echo "sdkpack: $(du -h "$OUT" | cut -f1) at $OUT"
