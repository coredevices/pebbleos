#!/bin/sh
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

#
# Run this script to update waf to a newer version and get rid of the 
# files we do not need for Pebble SDK.

set -x

VERSION=2.1.4
DOWNLOAD="https://waf.io/waf-$VERSION.tar.bz2"

TMPFILE=`mktemp -t waf-tar-bz`

# Remove existing waf folder
rm -fr waf

# Download and extract what we need from waf distrib
wget -O - $DOWNLOAD |tar -yx         \
        --include "waf-$VERSION/waf-light" \
        --include "waf-$VERSION/waflib/*" \
        --include "waf-$VERSION/wscript" \
        --exclude "waf-$VERSION/waflib/extras" \
        -s "/waf-$VERSION/waf/"

# Add some python magic for our lib to work
# (they will be copied in extras and require the init)
mkdir waf/waflib/extras
touch waf/waflib/extras/__init__.py

# waf's own --make-waf minifier drops the separator between a keyword and a
# following prefixed string literal, mangling `return f"..."` into `returnf"..."`.
patch -d waf -p1 < waf.patch
