#!/usr/bin/env python3
"""Assemble the single-file emulator artifact from template + payloads."""
import base64
import gzip
import sys
from pathlib import Path

HERE = Path(__file__).parent
FW = Path('/home/user/qemu-wasm-firmware')


# In pthread workers Module.locateFile is lost to structured clone, so the
# runtime computes new URL(..., import.meta.url) — which throws for blob:
# bases. The worker receives the compiled wasm module by message and never
# fetches it, so a plain relative name is safe.
MJS_PATCH = (
    "wasmBinaryFile = new URL('qemu-system-arm.wasm', import.meta.url).href;",
    "wasmBinaryFile = 'qemu-system-arm.wasm';",
)


def pack(path):
    raw = Path(path).read_bytes()
    if path.name.endswith('.js') and not path.name.endswith('.worker.js'):
        text = raw.decode()
        assert text.count(MJS_PATCH[0]) == 1, 'mjs patch anchor not found'
        raw = text.replace(*MJS_PATCH).encode()
    gz = gzip.compress(raw, 9)
    return base64.b64encode(gz).decode(), len(raw), len(gz)


def main(build_dir, out_path):
    build = Path(build_dir)
    tpl = (HERE / 'template.html').read_text()
    total_unpacked = 0
    sizes = {}
    for key, path in [
        ('__MJS_B64__', build / 'qemu-system-arm.js'),
        ('__WORKER_B64__', build / 'qemu-system-arm.worker.js'),
        ('__WASM_B64__', build / 'qemu-system-arm.wasm'),
        ('__MICRO_B64__', FW / 'qemu_micro_flash.bin'),
        ('__SPI_B64__', FW / 'qemu_spi_flash.bin'),
    ]:
        b64, raw, gz = pack(path)
        tpl = tpl.replace(key, b64)
        total_unpacked += raw
        sizes[key] = (raw, gz, len(b64))
        print(f'{path.name}: raw={raw:,} gz={gz:,} b64={len(b64):,}')
    tpl = tpl.replace('__UNPACK_MB__', str(round(total_unpacked / 1e6)))
    tpl = tpl.replace('__PAYLOAD_NOTE__',
                      f'{round(sum(s[2] for s in sizes.values())/1e6, 1)} MB embedded, '
                      f'{round(total_unpacked/1e6)} MB unpacked in memory')
    Path(out_path).write_text(tpl)
    print(f'TOTAL page: {len(tpl):,} bytes ({len(tpl)/1e6:.2f} MB; limit 16MB)')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
