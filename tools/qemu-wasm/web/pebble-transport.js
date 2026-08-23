// JS "phone" for the wasm emulator: talks QemuProtocol over the
// shared-memory serial rings exported by pebble_control.c, and Pebble
// Protocol inside QemuProtocol_SPP frames.
//
// Wire formats (from the firmware source):
//   QEMU frame:  FEED(2,BE) proto(2,BE) len(2,BE) payload footer BEEF(2,BE)
//   PP message:  length(2,BE) endpoint(2,BE) payload   (length excludes hdr)
// One QEMU frame does not align with PP message boundaries in either
// direction — both sides are byte streams and must be reassembled.

const QEMU_HDR = 0xFEED;
const QEMU_FTR = 0xBEEF;
const QEMU_MAX_DATA = 2048;

export const QemuProtocol = {
  SPP: 1,
  Tap: 2,
  BluetoothConnection: 3,
  Compass: 4,
  Battery: 5,
  Accel: 6,
  Vibration: 7,
  Button: 8,
  TimeFormat: 9,
};

const EP_PHONE_VERSION = 0x0011; // 17

export class PebblePhone {
  // mod: the emscripten module (needs HEAPU8/HEAPU32 and
  // _pebble_wasm_serial_ctrl). log: optional (msg) => void.
  constructor(mod, log = () => {}) {
    this.mod = mod;
    this.log = log;
    this.ctrl = Number(mod._pebble_wasm_serial_ctrl());
    this.sendQueue = [];     // bytes waiting for ring space
    this.rxStream = [];      // QEMU-frame reassembly buffer (host <- watch)
    this.ppStream = [];      // PP reassembly buffer (from SPP payloads)
    this.ppHandlers = new Map();     // endpoint -> (payload: Uint8Array) => void
    this.qemuHandlers = new Map();   // protocol -> (payload: Uint8Array) => void
    this.onPhoneVersionRequest = null; // fired after auto-reply

    this.onPP(EP_PHONE_VERSION, (payload) => this._handlePhoneVersion(payload));
  }

  _u32(off) { return Atomics.load(this.mod.HEAPU32, (this.ctrl + off) >> 2); }
  _setU32(off, v) { Atomics.store(this.mod.HEAPU32, (this.ctrl + off) >> 2, v); }

  // ---- host -> watch ring (rx from QEMU's point of view) ----
  _ringSend(bytes) {
    this.sendQueue.push(...bytes);
    this._flushSendQueue();
  }

  _flushSendQueue() {
    if (!this.sendQueue.length) return;
    const buf = this._u32(0), size = this._u32(4);
    const head = this._u32(16), tail = this._u32(20);
    const free = size - (head - tail);
    const n = Math.min(free, this.sendQueue.length);
    if (n <= 0) return;
    const heap = this.mod.HEAPU8;
    for (let i = 0; i < n; i++) {
      heap[buf + ((head + i) % size)] = this.sendQueue[i];
    }
    this.sendQueue.splice(0, n);
    this._setU32(16, (head + n) >>> 0);
  }

  // ---- watch -> host ring (tx from QEMU's point of view) ----
  poll() {
    this._flushSendQueue();
    const buf = this._u32(8), size = this._u32(12);
    const head = this._u32(24);
    let tail = this._u32(28);
    if (tail === head) return;
    const heap = this.mod.HEAPU8;
    while (tail !== head) {
      this.rxStream.push(heap[buf + (tail % size)]);
      tail = (tail + 1) >>> 0;
    }
    this._setU32(28, tail);
    this._parseQemuFrames();
  }

  _parseQemuFrames() {
    const s = this.rxStream;
    for (;;) {
      // resync to 0xFE 0xED
      let start = 0;
      while (start + 1 < s.length && !(s[start] === 0xFE && s[start + 1] === 0xED)) start++;
      if (start > 0) s.splice(0, start);
      if (s.length < 8) return;
      const proto = (s[2] << 8) | s[3];
      const len = (s[4] << 8) | s[5];
      if (len > QEMU_MAX_DATA) { s.splice(0, 2); continue; }
      if (s.length < 6 + len + 2) return;
      const payload = new Uint8Array(s.slice(6, 6 + len));
      const ftr = (s[6 + len] << 8) | s[6 + len + 1];
      s.splice(0, 6 + len + 2);
      if (ftr !== QEMU_FTR) { this.log(`bad qemu frame footer 0x${ftr.toString(16)}`); continue; }
      if (proto === QemuProtocol.SPP) {
        this.ppStream.push(...payload);
        this._parsePPMessages();
      } else {
        const h = this.qemuHandlers.get(proto);
        if (h) h(payload);
      }
    }
  }

  _parsePPMessages() {
    const s = this.ppStream;
    while (s.length >= 4) {
      const len = (s[0] << 8) | s[1];
      const ep = (s[2] << 8) | s[3];
      if (s.length < 4 + len) return;
      const payload = new Uint8Array(s.slice(4, 4 + len));
      s.splice(0, 4 + len);
      const h = this.ppHandlers.get(ep);
      if (h) h(payload);
      else this.log(`PP message on unhandled endpoint ${ep} (0x${ep.toString(16)}), ${len} bytes`);
    }
  }

  onPP(endpoint, handler) { this.ppHandlers.set(endpoint, handler); }
  onQemu(protocol, handler) { this.qemuHandlers.set(protocol, handler); }

  sendQemuFrame(protocol, payload) {
    const out = new Uint8Array(6 + payload.length + 2);
    const dv = new DataView(out.buffer);
    dv.setUint16(0, QEMU_HDR);
    dv.setUint16(2, protocol);
    dv.setUint16(4, payload.length);
    out.set(payload, 6);
    dv.setUint16(6 + payload.length, QEMU_FTR);
    this._ringSend(out);
  }

  // Send one Pebble Protocol message (payload <= 2044 bytes fits one frame).
  sendPP(endpoint, payload) {
    const msg = new Uint8Array(4 + payload.length);
    const dv = new DataView(msg.buffer);
    dv.setUint16(0, payload.length);
    dv.setUint16(2, endpoint);
    msg.set(payload, 4);
    // Split the PP byte stream at the QEMU frame cap; the firmware
    // reassembles across SPP frames.
    for (let off = 0; off < msg.length; off += QEMU_MAX_DATA) {
      this.sendQemuFrame(QemuProtocol.SPP, msg.subarray(off, Math.min(off + QEMU_MAX_DATA, msg.length)));
    }
  }

  // ---- session establishment ----
  // The watch sends a 1-byte 0x00 request on endpoint 17. The response
  // REPLACES the session capabilities, so it must re-assert at least
  // 0xA3 (RunState | InfiniteLogDumping | AppMessage8k | VoiceApi).
  _handlePhoneVersion(payload) {
    if (payload.length < 1 || payload[0] !== 0x00) return;
    const resp = new Uint8Array(21);
    const dv = new DataView(resp.buffer);
    resp[0] = 0x01;               // Response
    dv.setUint32(1, 0, true);     // deprecated library version
    dv.setUint32(5, 0, true);     // deprecated capabilities
    dv.setUint32(9, 2, false);    // platform bitfield, big-endian: Android
    resp[13] = 2;                 // response_version
    resp[14] = 4; resp[15] = 4; resp[16] = 0;  // phone app "4.4.0"
    dv.setUint32(17, 0x000000a3, true);        // capabilities, little-endian
    this.sendPP(EP_PHONE_VERSION, resp);
    this.log('phone version handshake complete');
    if (this.onPhoneVersionRequest) this.onPhoneVersionRequest();
  }
}
