#!/bin/bash
# emsdk 4.0.23 + wasm64 (MEMORY64=2) deps for the QEMU JIT build.
# Mirrors ericmigi/pebble-qemu-wasm Dockerfile.wasm-jit, native instead of docker.
set -ex

EMSDK4=/home/user/emsdk-4
TARGET=/home/user/wasm-deps64
SRC=/home/user/wasm64-src
mkdir -p "$TARGET" "$SRC"

if [ ! -x "$EMSDK4/emsdk" ]; then
  GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 https://github.com/emscripten-core/emsdk "$EMSDK4"
fi
cd "$EMSDK4"
./emsdk install 4.0.23
./emsdk activate 4.0.23
source "$EMSDK4/emsdk_env.sh"
emcc --version | head -1

export CPATH="$TARGET/include"
export PKG_CONFIG_PATH="$TARGET/lib/pkgconfig"
export EM_PKG_CONFIG_PATH="$PKG_CONFIG_PATH"
export CFLAGS="-O3 -pthread -DWASM_BIGINT -sMEMORY64=2"
export CXXFLAGS="$CFLAGS"
export LDFLAGS="-sWASM_BIGINT -sASYNCIFY=1 -sMEMORY64=2 -L$TARGET/lib"

CROSS=$SRC/cross64.meson
cat > "$CROSS" <<EOT
[host_machine]
system = 'emscripten'
cpu_family = 'wasm64'
cpu = 'wasm64'
endian = 'little'

[binaries]
c = 'emcc'
cpp = 'em++'
ar = 'emar'
ranlib = 'emranlib'
pkgconfig = ['pkg-config', '--static']

[built-in options]
c_args = ['-O3', '-pthread', '-DWASM_BIGINT', '-sMEMORY64=2', '-Wno-incompatible-function-pointer-types']
cpp_args = ['-O3', '-pthread', '-DWASM_BIGINT', '-sMEMORY64=2']
objc_args = ['-O3', '-pthread', '-DWASM_BIGINT', '-sMEMORY64=2']
c_link_args = ['-sWASM_BIGINT', '-sASYNCIFY=1', '-sMEMORY64=2', '-L$TARGET/lib']
cpp_link_args = ['-sWASM_BIGINT', '-sASYNCIFY=1', '-sMEMORY64=2', '-L$TARGET/lib']
EOT

# --- zlib 1.3.1 (github clone; zlib.net is blocked by the proxy) ---
if [ ! -f "$TARGET/lib/libz.a" ]; then
  if [ ! -d $SRC/zlib/.git ]; then GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 -b v1.3.1 https://github.com/madler/zlib $SRC/zlib; fi
  cd $SRC/zlib
  emconfigure ./configure --prefix=$TARGET --static
  emmake make install -j4
fi

# --- libffi v3.5.2 ---
if [ ! -f "$TARGET/lib/libffi.a" ]; then
  rm -rf $SRC/libffi && mkdir -p $SRC/libffi
  curl -Ls https://github.com/libffi/libffi/releases/download/v3.5.2/libffi-3.5.2.tar.gz | tar xzC $SRC/libffi --strip-components=1
  cd $SRC/libffi
  emconfigure ./configure --host=wasm64-unknown-linux --prefix=$TARGET --enable-static \
    --disable-shared --disable-dependency-tracking --disable-builddir \
    --disable-multi-os-directory --disable-raw-api --disable-docs
  emmake make install SUBDIRS='include' -j4
fi

# --- res_query stub ---
cd $SRC
cat > res_query.c <<'EOT'
#include <netdb.h>
int res_query(const char *name, int class,
              int type, unsigned char *dest, int len)
{
    h_errno = HOST_NOT_FOUND;
    return -1;
}
EOT
emcc $CFLAGS -c res_query.c -fPIC -o libresolv.o
emar rcs $TARGET/lib/libresolv.a libresolv.o

# --- pixman 0.44.2 ---
if [ ! -f "$TARGET/lib/libpixman-1.a" ]; then
  if [ ! -d $SRC/pixman ]; then cp -r /home/user/wasm-deps/src/pixman $SRC/pixman; fi
  cd $SRC/pixman
  rm -rf _build
  meson setup _build --prefix=$TARGET --cross-file="$CROSS" \
    --default-library=static --buildtype=release -Dtests=disabled -Ddemos=disabled
  meson install -C _build
fi

# --- glib 2.84.0 ---
if [ ! -f "$TARGET/lib/libglib-2.0.a" ]; then
  if [ ! -d $SRC/glib ]; then cp -r /home/user/wasm-deps/src/glib $SRC/glib; fi
  cd $SRC/glib
  rm -rf _build
  meson setup _build --prefix=$TARGET --cross-file="$CROSS" \
    --default-library=static --buildtype=release --force-fallback-for=pcre2 \
    -Dselinux=disabled -Dxattr=false -Dlibmount=disabled -Dnls=disabled \
    -Dtests=false -Dglib_debug=disabled -Dglib_assert=false -Dglib_checks=false
  sed -i -E "/#define HAVE_POSIX_SPAWN 1/d" ./_build/config.h
  sed -i -E "/#define HAVE_PTHREAD_GETNAME_NP 1/d" ./_build/config.h
  meson install -C _build
fi

echo DEPS64_DONE
ls "$TARGET/lib"
