# Sparse length encoding (SLE)

SLE is a small run-length encoding tuned for binary blobs that mix long
zero runs with otherwise incompressible data. The encoder is
`tools/waf/sparse_length_encoding.py`; the firmware-side decoder is
`src/fw/util/sle.c`.

The encoded stream is:

- **1 header byte**: the escape byte, chosen by the encoder as a
  least-frequent byte of the input (never `0x00`).
- **Literal bytes**, copied through verbatim, until an escape byte
  starts a 2–3 byte sequence:
  - `ESC 0x00` — end of stream
  - `ESC 0x01` — one literal escape byte
  - `ESC b` (`0x02 ≤ b ≤ 0x7f`) — a run of `b` zero bytes (2–127)
  - `ESC b c` (`b ≥ 0x80`) — a run of `((b & 0x7f) << 8 | c) + 0x80`
    zero bytes (128–32895); longer runs are split

A single isolated zero byte is emitted literally (a run of 1 cannot be
encoded). Minimum overhead is 3 bytes: header plus end-of-stream.

Both the encoder and the C decoder are currently dormant: the last
in-tree user (compressed FPGA bitstreams for the Spalding board) was
removed with the Spalding boards, and only the unit tests exercise them
today.
