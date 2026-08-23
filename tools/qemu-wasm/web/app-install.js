// In-browser Pebble app installer: BlobDB insert + AppFetch + PutBytes
// over a PebblePhone transport. Wire formats match the firmware source
// (blob_db/endpoint.c, app_fetch_endpoint/service.c, put_bytes.c).

const EP_APP_RUN_STATE = 0x0034; // 52
const EP_APP_FETCH = 0x1771;     // 6001
const EP_BLOB_DB = 0xb1db;       // 45531
const EP_PUT_BYTES = 0xbeef;     // 48879

const BLOB_DB_ID_APPS = 0x02;
const BLOB_DB_SUCCESS = 0x01;
const BLOB_DB_TRY_LATER = 0x0b;

const PB_OBJ_APP_RESOURCES = 0x04;
const PB_OBJ_WATCH_APP = 0x05;
const PB_OBJ_WATCH_WORKER = 0x07;
const PB_MAX_CHUNK = 2044;
// The watch acks a chunk when it arrives but writes it to flash from a
// system task, so acks alone do not throttle us: on a large resource pack
// the queue fills, the firmware panics ("System task queue full") and
// resets mid-install. Pause between chunks to let that queue drain --
// 15 ms costs about a second over a 150 KB pack.
const PB_CHUNK_PACING_MS = 15;

const PROCESS_INFO_HAS_WORKER = 0x10;

// "Legacy defective checksum": CRC-32 poly 0x04C11DB7, init 0xFFFFFFFF,
// no reflection, no final xor; whole words fed byte-reversed, the final
// partial word zero-padded on the left and fed forward.
const CRC_TABLE = new Uint32Array([
  0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
  0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
  0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
  0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
]);

function crcByte(crc, b) {
  crc = ((crc << 4) ^ CRC_TABLE[((crc >>> 28) ^ (b >> 4)) & 0x0f]) >>> 0;
  crc = ((crc << 4) ^ CRC_TABLE[((crc >>> 28) ^ (b & 0x0f)) & 0x0f]) >>> 0;
  return crc;
}

export function legacyDefectiveCrc(data) {
  let crc = 0xffffffff;
  const whole = data.length & ~3;
  for (let i = 0; i < whole; i += 4) {
    crc = crcByte(crc, data[i + 3]);
    crc = crcByte(crc, data[i + 2]);
    crc = crcByte(crc, data[i + 1]);
    crc = crcByte(crc, data[i]);
  }
  const tail = data.length - whole;
  if (tail) {
    for (let p = 4 - tail; p > 0; p--) crc = crcByte(crc, 0);
    for (let i = whole; i < data.length; i++) crc = crcByte(crc, data[i]);
  }
  return crc >>> 0;
}

// PebbleProcessInfo, struct version 16.x (pebble_process_info.h)
export function parseProcessInfo(bin) {
  const td = new TextDecoder();
  if (td.decode(bin.subarray(0, 6)) !== 'PBLAPP') {
    throw new Error('app binary missing PBLAPP magic');
  }
  const dv = new DataView(bin.buffer, bin.byteOffset, bin.byteLength);
  const cstr = (off, len) => {
    const raw = bin.subarray(off, off + len);
    const nul = raw.indexOf(0);
    return td.decode(nul >= 0 ? raw.subarray(0, nul) : raw);
  };
  // struct_version < 16 (SDK 2.x) has an extra reloc_list_start u32
  // before num_reloc_entries, shifting the uuid from 104 to 108.
  const uuidOff = bin[8] >= 0x10 ? 104 : 108;
  return {
    structVersion: [bin[8], bin[9]],
    sdkVersion: [bin[10], bin[11]],
    processVersion: [bin[12], bin[13]],
    name: cstr(24, 32),
    company: cstr(56, 32),
    iconResourceId: dv.getUint32(88, true),
    flags: dv.getUint32(96, true),
    uuid: bin.slice(uuidOff, uuidOff + 16),
  };
}

export function uuidToString(u) {
  const h = [...u].map((b) => b.toString(16).padStart(2, '0')).join('');
  return `${h.slice(0, 8)}-${h.slice(8, 12)}-${h.slice(12, 16)}-${h.slice(16, 20)}-${h.slice(20)}`;
}

// AppDBEntry: 126 bytes (blob_db/app_db.h)
function buildAppDbEntry(info) {
  const e = new Uint8Array(126);
  const dv = new DataView(e.buffer);
  e.set(info.uuid, 0);
  dv.setUint32(16, info.flags, true);          // info_flags
  dv.setUint32(20, info.iconResourceId, true);
  e[24] = info.processVersion[0]; e[25] = info.processVersion[1];
  e[26] = info.sdkVersion[0]; e[27] = info.sdkVersion[1];
  e[28] = 0;                                    // app_face_bg_color: auto
  e[29] = 0;                                    // template_id
  const name = new TextEncoder().encode(info.name).subarray(0, 95);
  e.set(name, 30);
  return e;
}

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

export class AppInstaller {
  // phone: PebblePhone; onProgress({phase, detail, sent, total})
  constructor(phone, onProgress = () => {}, log = () => {}) {
    this.phone = phone;
    this.onProgress = onProgress;
    this.log = log;
    this.blobDbToken = (Math.random() * 0xfffe + 1) & 0xffff;
    this._blobDbWaiter = null;
    this._putBytesWaiter = null;
    this._appFetchWaiter = null;

    phone.onPP(EP_BLOB_DB, (p) => {
      if (p.length >= 3 && this._blobDbWaiter) {
        const dv = new DataView(p.buffer, p.byteOffset);
        this._blobDbWaiter({ token: dv.getUint16(0, true), status: p[2] });
      }
    });
    phone.onPP(EP_PUT_BYTES, (p) => {
      if (p.length >= 5 && this._putBytesWaiter) {
        const dv = new DataView(p.buffer, p.byteOffset);
        this._putBytesWaiter({ ack: p[0] === 0x01, token: dv.getUint32(1, false) });
      }
    });
    phone.onPP(EP_APP_FETCH, (p) => {
      if (p.length >= 21 && p[0] === 0x01) {
        const dv = new DataView(p.buffer, p.byteOffset);
        const req = { uuid: p.slice(1, 17), appId: dv.getInt32(17, true) };
        this.log(`app fetch request: app_id=${req.appId} uuid=${uuidToString(req.uuid)}`);
        if (this._appFetchWaiter) this._appFetchWaiter(req);
      }
    });
  }

  _wait(slot, timeoutMs, what) {
    return new Promise((resolve, reject) => {
      const t = setTimeout(() => { this[slot] = null; reject(new Error(`timeout waiting for ${what}`)); }, timeoutMs);
      this[slot] = (v) => { clearTimeout(t); this[slot] = null; resolve(v); };
    });
  }

  async _blobDbInsert(uuid, value) {
    for (let attempt = 0; attempt < 30; attempt++) {
      const token = this.blobDbToken = (this.blobDbToken % 0xfffe) + 1;
      const msg = new Uint8Array(1 + 2 + 1 + 1 + 16 + 2 + value.length);
      const dv = new DataView(msg.buffer);
      msg[0] = 0x01;                    // INSERT
      dv.setUint16(1, token, true);
      msg[3] = BLOB_DB_ID_APPS;
      msg[4] = 16;
      msg.set(uuid, 5);
      dv.setUint16(21, value.length, true);
      msg.set(value, 23);
      const p = this._wait('_blobDbWaiter', 5000, 'blobdb response');
      this.phone.sendPP(EP_BLOB_DB, msg);
      let resp;
      try {
        resp = await p;
      } catch (e) {
        // A silent watch usually means the emulator is starved rather than
        // wedged — an in-browser app build saturates every core — so give
        // it another go before failing the install.
        if (attempt >= 5) throw e;
        this.log('no blobdb response yet, the watch may be catching up…');
        await delay(2000);
        continue;
      }
      if (resp.status === BLOB_DB_SUCCESS) return;
      if (resp.status === BLOB_DB_TRY_LATER) {
        this.log('blobdb not ready yet, retrying…');
        await delay(1000);
        continue;
      }
      throw new Error(`blobdb insert failed, status 0x${resp.status.toString(16)}`);
    }
    throw new Error('blobdb still not accepting after 30 tries');
  }

  async _putBytesObject(type, appId, data, label) {
    // INIT
    const init = new Uint8Array(10);
    const dv = new DataView(init.buffer);
    init[0] = 0x01;
    dv.setUint32(1, data.length, false);
    init[5] = type | 0x80;
    dv.setUint32(6, appId, false);
    let p = this._wait('_putBytesWaiter', 20000, `putbytes INIT ack (${label})`);
    this.phone.sendPP(EP_PUT_BYTES, init);
    let resp = await p;
    if (!resp.ack) throw new Error(`putbytes INIT nacked (${label})`);
    const token = resp.token;

    // PUT chunks, one in flight
    for (let off = 0; off < data.length; off += PB_MAX_CHUNK) {
      const chunk = data.subarray(off, Math.min(off + PB_MAX_CHUNK, data.length));
      const put = new Uint8Array(9 + chunk.length);
      const pdv = new DataView(put.buffer);
      put[0] = 0x02;
      pdv.setUint32(1, token, false);
      pdv.setUint32(5, chunk.length, false);
      put.set(chunk, 9);
      p = this._wait('_putBytesWaiter', 30000, `putbytes PUT ack (${label})`);
      this.phone.sendPP(EP_PUT_BYTES, put);
      resp = await p;
      if (!resp.ack) throw new Error(`putbytes PUT nacked at ${off} (${label})`);
      this.onProgress({ phase: 'transfer', detail: label, sent: Math.min(off + chunk.length, data.length), total: data.length });
      if (off + PB_MAX_CHUNK < data.length) await delay(PB_CHUNK_PACING_MS);
    }

    // COMMIT with legacy CRC
    const commit = new Uint8Array(9);
    const cdv = new DataView(commit.buffer);
    commit[0] = 0x03;
    cdv.setUint32(1, token, false);
    cdv.setUint32(5, legacyDefectiveCrc(data), false);
    p = this._wait('_putBytesWaiter', 30000, `putbytes COMMIT ack (${label})`);
    this.phone.sendPP(EP_PUT_BYTES, commit);
    resp = await p;
    if (!resp.ack) throw new Error(`putbytes COMMIT nacked — CRC mismatch? (${label})`);

    // INSTALL (historical compat; ack expected)
    const inst = new Uint8Array(5);
    new DataView(inst.buffer).setUint32(1, token, false);
    inst[0] = 0x05;
    p = this._wait('_putBytesWaiter', 20000, `putbytes INSTALL ack (${label})`);
    this.phone.sendPP(EP_PUT_BYTES, inst);
    resp = await p;
    if (!resp.ack) throw new Error(`putbytes INSTALL nacked (${label})`);
    this.log(`${label}: ${data.length} bytes transferred`);
  }

  // pbw: result of parsePbw(). Resolves when all binaries are on the watch.
  async install(pbw) {
    const info = parseProcessInfo(pbw.binary);
    this.log(`installing "${info.name}" by ${info.company} ` +
      `(sdk ${info.sdkVersion[0]}.${info.sdkVersion[1]}, flags 0x${info.flags.toString(16)}, ${pbw.platformDir})`);
    if (!pbw.resources) throw new Error('pbw has no app_resources.pbpack (required)');

    this.onProgress({ phase: 'blobdb', detail: info.name });
    await this._blobDbInsert(info.uuid, buildAppDbEntry(info));
    this.log('app registered in launcher (blobdb ack)');

    // Launch it; the watch discovers missing binaries and requests a fetch.
    const fetchP = this._wait('_appFetchWaiter', 15000, 'app fetch request');
    const run = new Uint8Array(17);
    run[0] = 0x01;
    run.set(info.uuid, 1);
    this.phone.sendPP(EP_APP_RUN_STATE, run);
    this.onProgress({ phase: 'fetch-wait', detail: info.name });
    const req = await fetchP;

    // STARTING
    this.phone.sendPP(EP_APP_FETCH, new Uint8Array([0x01, 0x01]));

    await this._putBytesObject(PB_OBJ_WATCH_APP, req.appId, pbw.binary, 'app binary');
    if (info.flags & PROCESS_INFO_HAS_WORKER) {
      if (!pbw.worker) throw new Error('app declares a worker but pbw has no pebble-worker.bin');
      await this._putBytesObject(PB_OBJ_WATCH_WORKER, req.appId, pbw.worker, 'worker');
    }
    await this._putBytesObject(PB_OBJ_APP_RESOURCES, req.appId, pbw.resources, 'resources');

    this.onProgress({ phase: 'done', detail: info.name });
    this.log(`install complete — watch is launching ${info.name}`);
    return info;
  }
}
