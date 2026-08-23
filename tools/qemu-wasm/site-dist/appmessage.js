// AppMessage (Pebble Protocol endpoint 48 / 0x30) client for the JS phone.
//
// Wire format (verified against src/fw/services/normal/app_message):
//   PUSH:  0x01 | txid u8 | uuid[16] | dict
//   ACK:   0xff | txid u8
//   NACK:  0x7f | txid u8
//   dict:  count u8, then per tuple: key u32 LE | type u8 | length u16 LE | data
//   tuple types: 0 = byte array, 1 = cstring (NUL-terminated), 2 = uint, 3 = int

const EP_APP_MESSAGE = 0x0030; // 48

const CMD_PUSH = 0x01;
const CMD_ACK = 0xff;
const CMD_NACK = 0x7f;

const TYPE_BYTE_ARRAY = 0;
const TYPE_CSTRING = 1;
const TYPE_UINT = 2;
const TYPE_INT = 3;

const ACK_TIMEOUT_MS = 10000;

function encodeDict(entries) {
  // entries: [{key: number, value: string|number|boolean|Uint8Array|number[]}]
  const parts = [];
  for (const { key, value } of entries) {
    let type, data;
    if (value instanceof Uint8Array) { type = TYPE_BYTE_ARRAY; data = value; }
    else if (Array.isArray(value)) { type = TYPE_BYTE_ARRAY; data = Uint8Array.from(value.map((b) => b & 0xff)); }
    else if (typeof value === 'string') {
      const s = new TextEncoder().encode(value);
      data = new Uint8Array(s.length + 1);
      data.set(s, 0);
      type = TYPE_CSTRING;
    } else if (typeof value === 'boolean') {
      type = TYPE_INT; data = new Uint8Array([value ? 1 : 0, 0, 0, 0]);
    } else if (typeof value === 'number') {
      type = TYPE_INT;
      data = new Uint8Array(4);
      new DataView(data.buffer).setInt32(0, Math.trunc(value) | 0, true);
    } else {
      throw new Error(`cannot encode value for key ${key}: ${typeof value}`);
    }
    const t = new Uint8Array(7 + data.length);
    const dv = new DataView(t.buffer);
    dv.setUint32(0, key >>> 0, true);
    t[4] = type;
    dv.setUint16(5, data.length, true);
    t.set(data, 7);
    parts.push(t);
  }
  const total = parts.reduce((n, p) => n + p.length, 1);
  const out = new Uint8Array(total);
  out[0] = entries.length;
  let off = 1;
  for (const p of parts) { out.set(p, off); off += p.length; }
  return out;
}

function decodeDict(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = bytes[0];
  const out = []; // [{key, type, value}]
  let off = 1;
  for (let i = 0; i < count; i++) {
    const key = dv.getUint32(off, true);
    const type = bytes[off + 4];
    const len = dv.getUint16(off + 5, true);
    const data = bytes.subarray(off + 7, off + 7 + len);
    off += 7 + len;
    let value;
    if (type === TYPE_CSTRING) {
      const nul = data.indexOf(0);
      value = new TextDecoder().decode(nul >= 0 ? data.subarray(0, nul) : data);
    } else if (type === TYPE_UINT) {
      value = len === 1 ? data[0]
        : len === 2 ? new DataView(data.buffer, data.byteOffset).getUint16(0, true)
        : new DataView(data.buffer, data.byteOffset).getUint32(0, true);
    } else if (type === TYPE_INT) {
      value = len === 1 ? (data[0] << 24 >> 24)
        : len === 2 ? new DataView(data.buffer, data.byteOffset).getInt16(0, true)
        : new DataView(data.buffer, data.byteOffset).getInt32(0, true);
    } else {
      value = data.slice();
    }
    out.push({ key, value });
  }
  return out;
}

export class AppMessageClient {
  constructor(phone, log = () => {}) {
    this.phone = phone;
    this.log = log;
    this.uuid = null;
    this.keyByName = new Map();
    this.nameByKey = new Map();
    this.txid = 1;
    this.pending = null;   // {txid, resolve, reject, timer}
    this.onPush = null;    // (payloadObj) => void
    // Pushes that arrive before the app's JS has attached its handler
    // (the watch sends at app launch, we start JS on the run-state
    // notification a beat later). Drained by drainPushes().
    this.queuedPushes = []; // [{uuidHex, entries, at}]

    phone.onPP(EP_APP_MESSAGE, (p) => this._onMessage(p));
  }

  // appKeys: {NAME: number} from the pbw's appinfo.json
  setApp(uuid, appKeys) {
    this.uuid = uuid;
    this.keyByName = new Map(Object.entries(appKeys || {}));
    this.nameByKey = new Map([...this.keyByName].map(([n, k]) => [k, n]));
    if (this.pending) {
      clearTimeout(this.pending.timer);
      this.pending.reject(new Error('app changed'));
      this.pending = null;
    }
  }

  _onMessage(p) {
    if (p.length < 2) return;
    const cmd = p[0], txid = p[1];
    if (cmd === CMD_ACK || cmd === CMD_NACK) {
      if (this.pending && this.pending.txid === txid) {
        clearTimeout(this.pending.timer);
        const { resolve, reject } = this.pending;
        this.pending = null;
        cmd === CMD_ACK ? resolve() : reject(new Error('watch NACKed the message'));
      }
      return;
    }
    if (cmd === CMD_PUSH) {
      if (p.length < 18) return;
      // Always ACK; the JS side has no notion of refusing a push.
      this.phone.sendPP(EP_APP_MESSAGE, new Uint8Array([CMD_ACK, txid]));
      const entries = decodeDict(p.subarray(18));
      if (this.onPush) {
        this.onPush(this._payload(entries));
      } else {
        const uuidHex = [...p.subarray(2, 18)].map((b) => b.toString(16).padStart(2, '0')).join('');
        this.queuedPushes.push({ uuidHex, entries, at: Date.now() });
        if (this.queuedPushes.length > 8) this.queuedPushes.shift();
      }
      return;
    }
    this.log(`appmessage: unknown command 0x${cmd.toString(16)}`);
  }

  _payload(entries) {
    const payload = {};
    for (const { key, value } of entries) {
      payload[this.nameByKey.get(key) ?? key] = value;
    }
    return payload;
  }

  // Deliver pushes for `uuid` that were queued before onPush attached.
  drainPushes(uuid) {
    const uuidHex = [...uuid].map((b) => b.toString(16).padStart(2, '0')).join('');
    const cutoff = Date.now() - 10000;
    const mine = this.queuedPushes.filter((q) => q.uuidHex === uuidHex && q.at > cutoff);
    this.queuedPushes = [];
    if (this.onPush) for (const q of mine) this.onPush(this._payload(q.entries));
  }

  // dict: {NAME_or_numeric_key: value}. Resolves on watch ACK. Sends are
  // serialized: one PUSH in flight, the rest queue (like the phone apps).
  send(dict) {
    if (!this.uuid) return Promise.reject(new Error('no app active'));
    const entries = [];
    for (const [name, value] of Object.entries(dict)) {
      const key = this.keyByName.has(name) ? this.keyByName.get(name)
        : (/^\d+$/.test(name) ? Number(name) : undefined);
      if (key === undefined) return Promise.reject(new Error(`unknown appKey "${name}"`));
      entries.push({ key, value });
    }
    const dictBytes = encodeDict(entries);
    return new Promise((resolve, reject) => {
      this.sendQueue = this.sendQueue || [];
      this.sendQueue.push({ dictBytes, uuid: this.uuid, resolve, reject });
      this._pumpSend();
    });
  }

  _pumpSend() {
    if (this.pending || !this.sendQueue || !this.sendQueue.length) return;
    const { dictBytes, uuid, resolve, reject } = this.sendQueue.shift();
    if (uuid !== this.uuid) { reject(new Error('app changed')); return this._pumpSend(); }
    const msg = new Uint8Array(18 + dictBytes.length);
    msg[0] = CMD_PUSH;
    const txid = this.txid = (this.txid + 1) & 0xff;
    msg[1] = txid;
    msg.set(uuid, 2);
    msg.set(dictBytes, 18);
    const timer = setTimeout(() => {
      if (this.pending && this.pending.txid === txid) {
        this.pending = null;
        reject(new Error('timed out waiting for watch ACK'));
        this._pumpSend();
      }
    }, ACK_TIMEOUT_MS);
    this.pending = {
      txid, timer,
      resolve: (v) => { resolve(v); this._pumpSend(); },
      reject: (e) => { reject(e); this._pumpSend(); },
    };
    this.phone.sendPP(EP_APP_MESSAGE, msg);
  }
}
