#!/bin/bash
# Reproducible WASM build of the Pebble QEMU fork (coredevices/qemu).
#
# Cross-builds QEMU 10.1.x-pebble for wasm32 with emscripten and the TCG
# interpreter (TCI). Mirrors qemu's tests/docker/dockerfiles/
# emsdk-wasm32-cross.docker, but runs directly on the host so the build
# tree can be iterated on with ninja.
#
# Usage: ./build-qemu-wasm.sh <path-to-coredevices-qemu> [workdir]
#
# Host prerequisites: git, curl, python3, pip, ninja, pkg-config,
# autoconf/automake/libtool (for zlib/libffi/libpng autotools builds).
set -euo pipefail

QEMU_SRC="${1:?usage: $0 <path-to-coredevices-qemu-checkout> [workdir]}"
WORKDIR="${2:-$HOME/pebble-qemu-wasm-build}"

# Pinned versions (from emsdk-wasm32-cross.docker; emsdk verified against
# the reference build).
EMSDK_VERSION=3.1.50
ZLIB_VERSION=1.3.1
GLIB_MINOR_VERSION=2.84
GLIB_VERSION=${GLIB_MINOR_VERSION}.0
PIXMAN_VERSION=0.44.2
FFI_VERSION=v3.4.7
LIBPNG_VERSION=1.6.47
MESON_VERSION=1.5.0

EMSDK_DIR="$WORKDIR/emsdk"
TARGET="$WORKDIR/wasm-deps"     # dependency install prefix
SRCDIR="$WORKDIR/src"
BUILD_DIR="$QEMU_SRC/build-wasm"

[ -f "$QEMU_SRC/hw/arm/pebble_generic.c" ] || {
    echo "error: $QEMU_SRC does not look like a coredevices/qemu checkout" >&2
    exit 1
}

mkdir -p "$WORKDIR" "$SRCDIR" "$TARGET"

# ---------------------------------------------------------------------
# 1. emsdk bootstrap
# ---------------------------------------------------------------------
if [ ! -d "$EMSDK_DIR" ]; then
    git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_DIR"
fi
"$EMSDK_DIR/emsdk" install "$EMSDK_VERSION"
"$EMSDK_DIR/emsdk" activate "$EMSDK_VERSION"
# shellcheck disable=SC1091
source "$EMSDK_DIR/emsdk_env.sh"

# meson + tomli (QEMU configure needs tomli on python < 3.11)
python3 -m pip install --user "meson==$MESON_VERSION" tomli

# ---------------------------------------------------------------------
# 2. Cross-build environment (mirrors the dockerfile)
# ---------------------------------------------------------------------
export CPATH="$TARGET/include"
export PKG_CONFIG_PATH="$TARGET/lib/pkgconfig"
export EM_PKG_CONFIG_PATH="$PKG_CONFIG_PATH"
export CFLAGS="-O3 -pthread -DWASM_BIGINT"
export CXXFLAGS="$CFLAGS"
export LDFLAGS="-sWASM_BIGINT -sASYNCIFY=1 -L$TARGET/lib"

CROSS_MESON="$WORKDIR/cross.meson"
cat > "$CROSS_MESON" <<EOT
[host_machine]
system = 'emscripten'
cpu_family = 'wasm32'
cpu = 'wasm32'
endian = 'little'

[binaries]
c = 'emcc'
cpp = 'em++'
ar = 'emar'
ranlib = 'emranlib'
pkgconfig = ['pkg-config', '--static']
EOT

meson_cross_args() {
    # Append [built-in options] with the current CFLAGS/LDFLAGS to a copy
    # of the base cross file ($1 = output path, $2 = extra c_args).
    local out="$1" extra="${2:-}"
    cp "$CROSS_MESON" "$out"
    {
        echo "[built-in options]"
        # shellcheck disable=SC2086
        echo "c_args = [$(printf "'%s', " $CFLAGS $extra | sed 's/, $//')]"
        # shellcheck disable=SC2086
        echo "cpp_args = [$(printf "'%s', " $CFLAGS $extra | sed 's/, $//')]"
        # shellcheck disable=SC2086
        echo "c_link_args = [$(printf "'%s', " $LDFLAGS | sed 's/, $//')]"
        # shellcheck disable=SC2086
        echo "cpp_link_args = [$(printf "'%s', " $LDFLAGS | sed 's/, $//')]"
    } >> "$out"
}

# ---------------------------------------------------------------------
# 3. Dependencies
# ---------------------------------------------------------------------

# zlib
if [ ! -f "$TARGET/lib/libz.a" ]; then
    mkdir -p "$SRCDIR/zlib"
    curl -Ls "https://zlib.net/zlib-$ZLIB_VERSION.tar.xz" |
        tar xJC "$SRCDIR/zlib" --strip-components=1
    (cd "$SRCDIR/zlib" &&
        emconfigure ./configure --prefix="$TARGET" --static &&
        emmake make install -j"$(nproc)")
fi

# libffi (glib dependency)
if [ ! -f "$TARGET/lib/libffi.a" ]; then
    [ -d "$SRCDIR/libffi" ] || git clone https://github.com/libffi/libffi "$SRCDIR/libffi"
    (cd "$SRCDIR/libffi" &&
        git checkout "$FFI_VERSION" &&
        autoreconf -fiv &&
        emconfigure ./configure --host=wasm32-unknown-linux \
            --prefix="$TARGET" --enable-static \
            --disable-shared --disable-dependency-tracking \
            --disable-builddir --disable-multi-os-directory \
            --disable-raw-api --disable-docs &&
        emmake make install SUBDIRS='include' -j"$(nproc)")
fi

# pixman
if [ ! -f "$TARGET/lib/libpixman-1.a" ]; then
    [ -d "$SRCDIR/pixman" ] ||
        git clone https://gitlab.freedesktop.org/pixman/pixman "$SRCDIR/pixman"
    meson_cross_args "$WORKDIR/cross-pixman.meson"
    (cd "$SRCDIR/pixman" &&
        git checkout "pixman-$PIXMAN_VERSION" &&
        meson setup _build --prefix="$TARGET" \
            --cross-file="$WORKDIR/cross-pixman.meson" \
            --default-library=static --buildtype=release \
            -Dtests=disabled -Ddemos=disabled &&
        meson install -C _build)
fi

# libresolv stub (emscripten lacks res_query; glib links it)
if [ ! -f "$TARGET/lib/libresolv.a" ]; then
    mkdir -p "$SRCDIR/stub"
    cat > "$SRCDIR/stub/res_query.c" <<'EOT'
#include <netdb.h>
int res_query(const char *name, int class,
              int type, unsigned char *dest, int len)
{
    h_errno = HOST_NOT_FOUND;
    return -1;
}
EOT
    (cd "$SRCDIR/stub" &&
        # shellcheck disable=SC2086
        emcc $CFLAGS -c res_query.c -fPIC -o libresolv.o &&
        emar rcs libresolv.a libresolv.o &&
        cp libresolv.a "$TARGET/lib/")
fi

# glib (pcre2 via meson subproject fallback)
if [ ! -f "$TARGET/lib/libglib-2.0.a" ]; then
    mkdir -p "$SRCDIR/glib"
    curl -Lks "https://download.gnome.org/sources/glib/$GLIB_MINOR_VERSION/glib-$GLIB_VERSION.tar.xz" |
        tar xJC "$SRCDIR/glib" --strip-components=1
    meson_cross_args "$WORKDIR/cross-glib.meson" "-Wno-incompatible-function-pointer-types"
    (cd "$SRCDIR/glib" &&
        meson setup _build --prefix="$TARGET" \
            --cross-file="$WORKDIR/cross-glib.meson" \
            --default-library=static --buildtype=release \
            --force-fallback-for=pcre2 \
            -Dselinux=disabled -Dxattr=false -Dlibmount=disabled -Dnls=disabled \
            -Dtests=false -Dglib_debug=disabled -Dglib_assert=false \
            -Dglib_checks=false &&
        # emscripten doesn't provide these in the final link, which meson
        # setup doesn't detect (same workaround as the dockerfile)
        sed -i -E "/#define HAVE_POSIX_SPAWN 1/d" _build/config.h &&
        sed -i -E "/#define HAVE_PTHREAD_GETNAME_NP 1/d" _build/config.h &&
        meson install -C _build)
fi

# libpng (optional: enables PNG screendump in the monitor)
if [ ! -f "$TARGET/lib/libpng16.a" ]; then
    mkdir -p "$SRCDIR/libpng"
    curl -Ls "https://download.sourceforge.net/libpng/libpng-$LIBPNG_VERSION.tar.xz" |
        tar xJC "$SRCDIR/libpng" --strip-components=1
    (cd "$SRCDIR/libpng" &&
        emconfigure ./configure --host=wasm32-unknown-linux \
            --prefix="$TARGET" --disable-shared &&
        emmake make install -j"$(nproc)")
fi

# SDL2 comes from emscripten's port (-sUSE_SDL=2); the emsdk sysroot ships
# sdl2-config, which QEMU's meson picks up automatically.

# ---------------------------------------------------------------------
# 4. QEMU configure + build
# ---------------------------------------------------------------------
# Browser bridge (display/button shared-memory exports) until it lands in
# coredevices/qemu.
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/patches"
if ! grep -q pebble_wasm_display_data "$QEMU_SRC/hw/display/pebble_display.c"; then
    patch -d "$QEMU_SRC" -p1 < "$PATCH_DIR/0001-pebble-generic-browser-glue.patch"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -f config-host.mak ]; then
    emconfigure ../configure \
        --static \
        --target-list=arm-softmmu \
        --enable-tcg-interpreter \
        --disable-docs \
        --disable-tools
fi

ninja qemu-system-arm.js

echo ""
echo "=== Build complete ==="
ls -lh qemu-system-arm.js qemu-system-arm.wasm qemu-system-arm.worker.js \
    2>/dev/null || true

# Copy artifacts next to the shell
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/get-artifacts.sh" "$BUILD_DIR"
