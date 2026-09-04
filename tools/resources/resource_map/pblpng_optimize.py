# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
"""Pixel-identical PNG shrink pass for pack resources.

Runs on png2pblpng's output. Keeps pixel indices, palette order, colortype and
BIT DEPTH exactly (decoded gbitmap RAM layout unchanged); shrinks by
  - emitting the minimal chunk set (IHDR, PLTE, tRNS, one IDAT, IEND),
  - trimming PLTE to the used prefix (indices untouched),
  - dropping trailing fully-opaque tRNS entries (gbitmap_png defaults missing
    alpha entries to opaque),
  - per-scanline filter search, and
  - zopfli (fallback: zlib-9 across strategies) for the IDAT stream.
Output is deterministic (pbpack content-dedup keeps working) and always
single-IDAT + non-interlaced (uPNG requirements). optimize_png_bytes() verifies
its own output round-trips to the identical index grid and returns the ORIGINAL
bytes on any surprise, so the build can never regress correctness.
"""

import struct
import zlib

try:
    import zopfli.zlib as _zopfli

    _HAVE_ZOPFLI = True
except ImportError:
    _HAVE_ZOPFLI = False

_SIG = b"\x89PNG\r\n\x1a\n"


def _chunks(data):
    assert data[:8] == _SIG
    off = 8
    out = []
    while off < len(data):
        (ln,) = struct.unpack(">I", data[off : off + 4])
        typ = data[off + 4 : off + 8]
        out.append((typ, data[off + 8 : off + 8 + ln]))
        off += 12 + ln
    return out


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _unfilter(raw, height, stride):
    out = bytearray()
    prev = bytearray(stride)
    off = 0
    for _ in range(height):
        ft = raw[off]
        off += 1
        line = bytearray(raw[off : off + stride])
        off += stride
        if ft == 1:
            for i in range(1, stride):
                line[i] = (line[i] + line[i - 1]) & 0xFF
        elif ft == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ft == 3:
            for i in range(stride):
                left = line[i - 1] if i else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif ft == 4:
            for i in range(stride):
                left = line[i - 1] if i else 0
                ul = prev[i - 1] if i else 0
                line[i] = (line[i] + _paeth(left, prev[i], ul)) & 0xFF
        out += line
        prev = line
    return bytes(out)


def _apply_filter(ft, line, prev):
    stride = len(line)
    if ft == 0:
        return bytes(line)
    out = bytearray(stride)
    for i in range(stride):
        left = line[i - 1] if i else 0
        if ft == 1:
            out[i] = (line[i] - left) & 0xFF
        elif ft == 2:
            out[i] = (line[i] - prev[i]) & 0xFF
        elif ft == 3:
            out[i] = (line[i] - ((left + prev[i]) >> 1)) & 0xFF
        else:
            ul = prev[i - 1] if i else 0
            out[i] = (line[i] - _paeth(left, prev[i], ul)) & 0xFF
    return bytes(out)


def _filtered_stream(rows, mode):
    out = bytearray()
    prev = bytes(len(rows[0]))
    for line in rows:
        if mode == "msad":
            best = None
            for ft in range(5):
                f = _apply_filter(ft, line, prev)
                cost = sum(x if x < 128 else 256 - x for x in f)
                if best is None or cost < best[1]:
                    best = ((ft, f), cost)
            (ft, f), _ = best
            out.append(ft)
            out += f
        else:
            out.append(mode)
            out += _apply_filter(mode, line, prev)
        prev = line
    return bytes(out)


def _deflate_best(raw):
    cands = []
    for strat in (zlib.Z_DEFAULT_STRATEGY, zlib.Z_FILTERED, zlib.Z_RLE):
        c = zlib.compressobj(9, zlib.DEFLATED, 15, 9, strat)
        cands.append(c.compress(raw) + c.flush())
    if _HAVE_ZOPFLI:
        cands.append(_zopfli.compress(raw, numiterations=100))
    return min(cands, key=len)


def _chunk(typ, payload):
    return (
        struct.pack(">I", len(payload))
        + typ
        + payload
        + struct.pack(">I", zlib.crc32(typ + payload) & 0xFFFFFFFF)
    )


def optimize_png_bytes(data):
    """Return a smaller, pixel-identical PNG, or `data` unchanged on any doubt."""
    try:
        ch = _chunks(data)
        ihdr = {t: p for t, p in ch}[b"IHDR"]
        w, h, bd, ct, comp, filt, inter = struct.unpack(">IIBBBBB", ihdr)
        # only single-channel indexed/gray, non-interlaced input (pblpng output)
        if inter != 0 or comp != 0 or filt != 0 or ct not in (0, 3) or bd > 8:
            return data
        plte = trns = None
        idat = b""
        for typ, payload in ch:
            if typ == b"PLTE":
                plte = payload
            elif typ == b"tRNS":
                trns = payload
            elif typ == b"IDAT":
                idat += payload
        stride = (w * bd + 7) // 8
        flat = _unfilter(zlib.decompress(idat), h, stride)
        rows = [flat[y * stride : (y + 1) * stride] for y in range(h)]

        # highest used palette index (bit depth is PINNED — indices unchanged)
        maxidx = 0
        mask = (1 << bd) - 1
        for r in rows:
            for x in range(w):
                v = (r[(x * bd) // 8] >> (8 - bd - ((x * bd) % 8))) & mask
                maxidx = max(maxidx, v)
        nplte = plte[: 3 * (maxidx + 1)] if (ct == 3 and plte) else plte
        ntrns = None
        if trns is not None:
            t = trns[: maxidx + 1]
            while t and t[-1] == 255:  # trailing opaque: decoder defaults to opaque
                t = t[:-1]
            ntrns = t if t else None

        best_stream = None
        for mode in (0, 1, 2, 3, 4, "msad"):
            raw = _filtered_stream(rows, mode)
            z = _deflate_best(raw)
            if best_stream is None or len(z) < len(best_stream):
                best_stream = z

        out = _SIG + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, bd, ct, 0, 0, 0))
        if nplte is not None:
            out += _chunk(b"PLTE", nplte)
        if ntrns is not None:
            out += _chunk(b"tRNS", ntrns)
        out += _chunk(b"IDAT", best_stream) + _chunk(b"IEND", b"")

        # verify: decoded index grid identical; palette prefix identical
        vflat = _unfilter(zlib.decompress(best_stream), h, stride)
        if vflat != flat:
            return data
        if plte is not None and nplte != plte[: len(nplte)]:
            return data
        return out if len(out) < len(data) else data
    except Exception:  # noqa: BLE001
        return data
