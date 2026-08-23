// PULSEv2 console client for the wasm emulator.
//
// The qemu firmware's dbgserial (UART2) speaks PULSEv2, not plain text:
//   frame   = 0x55 | COBS(datagram || crc32le(datagram)) with 0x55->0x00 | 0x55
//   datagram = u16be link protocol || payload
//     0x5021 PUSH:      u16be port | u16be length | information
//        port 0x0003 = logging: 29-byte record (<c16sccQH) + message text
//     0xC021 LCP, 0xBA33 reliable NCP: PPP control (conf-req/ack)
//     0x3A33/0x3A35 reliable command/response (LAPB): prompt on port 0x3E20
//
// Formats verified against src/fw/console/pulse2.c, subsys/logging/
// pulse_logging.c and tools/libs/pulse2 (the desktop implementation).

const FLAG = 0x55;
const PROTO_LCP = 0xC021;
const PROTO_PUSH = 0x5021;
const PROTO_RELIABLE_NCP = 0xBA33;
const PROTO_RELIABLE_CMD = 0x3A33;
const PROTO_RELIABLE_RSP = 0x3A35;
const PORT_LOGGING = 0x0003;
const PORT_PROMPT = 0x3E20;

// ---- CRC32 (IEEE) ----
const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    t[n] = c >>> 0;
  }
  return t;
})();
function crc32(data) {
  let c = 0xFFFFFFFF;
  for (let i = 0; i < data.length; i++) c = CRC_TABLE[(c ^ data[i]) & 0xFF] ^ (c >>> 8);
  return (c ^ 0xFFFFFFFF) >>> 0;
}
const CRC_RESIDUE = crc32(new Uint8Array(4)); // crc32 of 4 zero bytes

// ---- COBS ----
function cobsDecode(src) {
  const out = [];
  let i = 0;
  while (i < src.length) {
    const code = src[i++];
    if (code === 0) return null;
    for (let j = 1; j < code; j++) {
      if (i >= src.length) return null;
      out.push(src[i++]);
    }
    if (code < 0xFF && i < src.length) out.push(0);
  }
  return new Uint8Array(out);
}
function cobsEncode(src) {
  const out = [];
  let codeIdx = 0;
  out.push(0);
  let code = 1;
  for (let i = 0; i < src.length; i++) {
    if (src[i] === 0) {
      out[codeIdx] = code;
      codeIdx = out.length; out.push(0); code = 1;
    } else {
      out.push(src[i]);
      if (++code === 0xFF) { out[codeIdx] = code; codeIdx = out.length; out.push(0); code = 1; }
    }
  }
  out[codeIdx] = code;
  return new Uint8Array(out);
}

// PPP control protocol codes
const CONF_REQ = 1, CONF_ACK = 2, CONF_NAK = 3, CONF_REJ = 4, TERM_REQ = 5, TERM_ACK = 6, ECHO_REQ = 9, ECHO_REP = 10;

const LEVEL_NAME = { '*': '', E: 'err', W: 'wrn', I: 'inf', D: 'dbg', V: 'vrb', '?': '?' };

export class PulseConsole {
  // sendBytes(Uint8Array): writes toward the firmware's dbgserial RX.
  // onLog({time, level, file, line, message}), onPrompt(text, done),
  // onRaw(text) for unframed bytes, log(debugMsg).
  constructor(sendBytes, hooks = {}) {
    this.sendBytes = sendBytes;
    this.onLog = hooks.onLog || (() => {});
    this.onPrompt = hooks.onPrompt || (() => {});
    this.onRaw = hooks.onRaw || (() => {});
    this.dbg = hooks.log || (() => {});

    this.rxBuf = [];
    this.inFrame = false;
    this.frame = [];

    this.lcpUp = false;
    this.ncpUp = false;
    this.opened = false;      // reliable transport confirmed by RR response
    this.idCounter = 1;
    this.sendVar = 0;         // V(S)
    this.recvVar = 0;         // V(R)
    this.waitingAck = false;
    this.txQueue = [];        // pending prompt commands
    this.inflight = null;     // {port, info} for retransmit
    this.retryTimer = null;

    // Kick off link bring-up; re-request periodically until up.
    this._configureTimer = setInterval(() => this._pumpConfigure(), 1000);
    this._pumpConfigure();
  }

  get promptReady() { return this.opened; }

  // ---- byte stream input (from the uart2 log file) ----
  feed(bytes) {
    for (let i = 0; i < bytes.length; i++) {
      const b = bytes[i];
      if (b === FLAG) {
        if (this.inFrame && this.frame.length) this._onFrame(Uint8Array.from(this.frame));
        else if (!this.inFrame && this.raw && this.raw.length) this._flushRaw();
        this.inFrame = true;
        this.frame = [];
      } else if (this.inFrame) {
        this.frame.push(b);
        if (this.frame.length > 2048) { this.inFrame = false; this.frame = []; }
      } else {
        (this.raw = this.raw || []).push(b);
        if (this.raw.length > 512) this._flushRaw();
      }
    }
  }

  _flushRaw() {
    const text = (this.raw || []).filter((c) => (c >= 32 && c <= 126) || c === 10 || c === 13 || c === 9)
      .map((c) => String.fromCharCode(c)).join('').trim();
    this.raw = [];
    if (text) this.onRaw(text);
  }

  _onFrame(escaped) {
    const unescaped = escaped.map ? escaped.slice() : escaped;
    for (let i = 0; i < unescaped.length; i++) if (unescaped[i] === 0) unescaped[i] = FLAG;
    const datagram = cobsDecode(unescaped);
    if (!datagram || datagram.length < 6) return;
    if (crc32(datagram) !== CRC_RESIDUE) return;
    const body = datagram.subarray(0, datagram.length - 4);
    const proto = (body[0] << 8) | body[1];
    const payload = body.subarray(2);
    switch (proto) {
      case PROTO_PUSH: return this._onPush(payload);
      case PROTO_LCP: return this._onControl('lcp', payload);
      case PROTO_RELIABLE_NCP: return this._onControl('ncp', payload);
      case PROTO_RELIABLE_CMD: return this._onReliable(payload, true);
      case PROTO_RELIABLE_RSP: return this._onReliable(payload, false);
      default: /* other transports: ignore */
    }
  }

  // ---- frame output ----
  _sendDatagram(proto, payload) {
    const body = new Uint8Array(2 + payload.length + 4);
    body[0] = proto >> 8; body[1] = proto & 0xFF;
    body.set(payload, 2);
    const fcs = crc32(body.subarray(0, 2 + payload.length));
    new DataView(body.buffer).setUint32(2 + payload.length, fcs, true);
    const encoded = cobsEncode(body);
    for (let i = 0; i < encoded.length; i++) if (encoded[i] === FLAG) encoded[i] = 0;
    const out = new Uint8Array(encoded.length + 2);
    out[0] = FLAG; out.set(encoded, 1); out[out.length - 1] = FLAG;
    this.sendBytes(out);
  }

  // ---- PUSH (logs) ----
  _onPush(p) {
    if (p.length < 4) return;
    const port = (p[0] << 8) | p[1];
    const len = (p[2] << 8) | p[3];
    const info = p.subarray(4, Math.min(len, p.length));
    if (port !== PORT_LOGGING || info.length < 29) return;
    const dv = new DataView(info.buffer, info.byteOffset);
    let file = '';
    for (let i = 1; i < 17 && info[i]; i++) file += String.fromCharCode(info[i]);
    const level = String.fromCharCode(info[17]);
    const timeMs = dv.getUint32(19, true) + dv.getUint32(23, true) * 4294967296;
    const line = dv.getUint16(27, true);
    const message = new TextDecoder().decode(info.subarray(29));
    this.onLog({ timeMs, level, levelName: LEVEL_NAME[level] ?? level, file, line, message });
  }

  // ---- PPP control protocols (LCP + reliable NCP) ----
  _pumpConfigure() {
    if (!this.lcpUp) this._sendControl(PROTO_LCP, CONF_REQ, this.idCounter++, new Uint8Array(0));
    else if (!this.ncpUp) this._sendControl(PROTO_RELIABLE_NCP, CONF_REQ, this.idCounter++, new Uint8Array(0));
    else if (!this.opened) this._sendSupervisory(true, 'RR', this.recvVar, true, false);
    else clearInterval(this._configureTimer);
  }

  _sendControl(proto, code, id, data) {
    const p = new Uint8Array(4 + data.length);
    p[0] = code; p[1] = id & 0xFF;
    p[2] = (4 + data.length) >> 8; p[3] = (4 + data.length) & 0xFF;
    p.set(data, 4);
    this._sendDatagram(proto, p);
  }

  _onControl(layer, p) {
    if (p.length < 4) return;
    const code = p[0], id = p[1];
    const len = (p[2] << 8) | p[3];
    const data = p.subarray(4, len);
    const proto = layer === 'lcp' ? PROTO_LCP : PROTO_RELIABLE_NCP;
    if (code === CONF_REQ) {
      // Accept whatever the peer asks (no options in practice).
      this._sendControl(proto, CONF_ACK, id, data);
    } else if (code === CONF_ACK) {
      if (layer === 'lcp' && !this.lcpUp) { this.lcpUp = true; this.dbg('pulse: link up'); this._pumpConfigure(); }
      if (layer === 'ncp' && !this.ncpUp) { this.ncpUp = true; this.dbg('pulse: reliable transport negotiating'); this._pumpConfigure(); }
    } else if (code === ECHO_REQ) {
      this._sendControl(proto, ECHO_REP, id, data);
    } else if (code === TERM_REQ) {
      this._sendControl(proto, TERM_ACK, id, new Uint8Array(0));
      if (layer === 'lcp') { this.lcpUp = false; this.ncpUp = false; this.opened = false; }
    }
  }

  // ---- reliable transport (LAPB-ish, stop-and-wait) ----
  _sendSupervisory(asCommand, kind, ackNumber, poll) {
    const kinds = { RR: 0, RNR: 1, REJ: 2 };
    const p = new Uint8Array(2);
    p[0] = (kinds[kind] << 2) | 0b01;
    p[1] = ((ackNumber & 0x7F) << 1) | (poll ? 1 : 0);
    this._sendDatagram(asCommand ? PROTO_RELIABLE_CMD : PROTO_RELIABLE_RSP, p);
  }

  _onReliable(p, isCommand) {
    if (p.length < 2) return;
    const b0 = p[0], b1 = p[1];
    const ackNumber = (b1 >> 1) & 0x7F;
    const pollFinal = b1 & 1;
    if ((b0 & 0x01) === 0) {
      // Information packet (discriminator bit 0 = 0)
      const seq = (b0 >> 1) & 0x7F;
      if (p.length < 6) return;
      const port = (p[2] << 8) | p[3];
      const len = (p[4] << 8) | p[5];
      const info = p.subarray(6, Math.min(len, p.length));
      this._processAck(ackNumber);
      if (isCommand) {
        if (seq === this.recvVar) {
          this.recvVar = (this.recvVar + 1) % 128;
          this._deliver(port, info);
        }
        this._sendSupervisory(false, 'RR', this.recvVar, pollFinal);
      }
    } else if ((b0 & 0x03) === 0x01) {
      // Supervisory
      if (!this.opened) {
        this.opened = true;
        this.dbg('pulse: prompt ready');
        this._pumpSend();
      }
      this._processAck(ackNumber);
      if (isCommand && pollFinal) this._sendSupervisory(false, 'RR', this.recvVar, true);
    }
  }

  _processAck(ackNumber) {
    if (this.waitingAck && ((ackNumber + 127) % 128) === this.sendVar) {
      this.waitingAck = false;
      this.inflight = null;
      if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
      this.sendVar = (this.sendVar + 1) % 128;
      this._pumpSend();
    }
  }

  _deliver(port, info) {
    if (port === PORT_PROMPT && info.length >= 9) {
      const dv = new DataView(info.buffer, info.byteOffset);
      const type = info[0];  // 101 done, 102 message
      const text = new TextDecoder().decode(info.subarray(9));
      this.onPrompt(text, type === 101);
    } else if (port === 0x0001 && info.length >= 1 && info[0] === 1) {
      // PCMP echo request -> reply
      const reply = info.slice(); reply[0] = 2;
      this._sendInfo(0x0001, reply);
    }
  }

  _sendInfo(port, info) {
    const p = new Uint8Array(6 + info.length);
    p[0] = (this.sendVar & 0x7F) << 1;
    p[1] = ((this.recvVar & 0x7F) << 1) | 1; // poll
    p[2] = port >> 8; p[3] = port & 0xFF;
    p[4] = (info.length + 6) >> 8; p[5] = (info.length + 6) & 0xFF;
    p.set(info, 6);
    this._sendDatagram(PROTO_RELIABLE_CMD, p);
    this.waitingAck = true;
    this.inflight = { port, info };
    this.retryTimer = setTimeout(() => {
      if (this.waitingAck && this.inflight) {
        this.dbg('pulse: retransmit');
        this.waitingAck = false;
        this._sendInfo(this.inflight.port, this.inflight.info);
      }
    }, 2000);
  }

  _pumpSend() {
    if (!this.opened || this.waitingAck || !this.txQueue.length) return;
    const cmd = this.txQueue.shift();
    this._sendInfo(PORT_PROMPT, new TextEncoder().encode(cmd));
  }

  // Public: run a prompt command (e.g. "help", "click short back").
  command(text) {
    this.txQueue.push(text);
    this._pumpSend();
    return this.opened;
  }
}
