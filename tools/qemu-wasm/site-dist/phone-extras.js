// Notifications and voice dictation for the JS phone.
// Wire formats verified against the firmware source (notif_db.c,
// timeline/item.h, voice_endpoint/service.c, audio_endpoint/service.c).

const EP_BLOB_DB = 0xb1db;
const EP_VOICE_CONTROL = 11000; // 0x2af8
const EP_AUDIO_DATA = 10000;    // 0x2710

const BLOB_DB_ID_NOTIFS = 0x04;

// timeline attribute ids
const ATTR_TITLE = 1, ATTR_BODY = 3, ATTR_ICON_TINY = 4, ATTR_SENDER = 12,
      ATTR_CANNED_RESPONSES = 8, ATTR_APP_NAME = 30;
const ACTION_RESPONSE = 0x03, ACTION_DISMISS = 0x04;
const TIMELINE_RES_NOTIFICATION_GENERIC = 0x80000001; // system flag | id 1
const LAYOUT_NOTIFICATION = 4;
const ITEM_TYPE_NOTIFICATION = 1;

const enc = new TextEncoder();

function attr(id, data) {
  const d = typeof data === 'string' ? enc.encode(data) : data;
  const out = new Uint8Array(3 + d.length);
  out[0] = id;
  out[1] = d.length & 0xff; out[2] = d.length >> 8;
  out.set(d, 3);
  return out;
}
function u32le(v) {
  const b = new Uint8Array(4);
  new DataView(b.buffer).setUint32(0, v >>> 0, true);
  return b;
}
function cat(parts) {
  const total = parts.reduce((n, p) => n + p.length, 0);
  const out = new Uint8Array(total);
  let off = 0;
  for (const p of parts) { out.set(p, off); off += p.length; }
  return out;
}

export function randomUuid() {
  const u = crypto.getRandomValues(new Uint8Array(16));
  u[6] = (u[6] & 0x0f) | 0x40;
  u[8] = (u[8] & 0x3f) | 0x80;
  return u;
}

// Build the BlobDB value for a notification. Returns {uuid, value}.
export function buildNotification({ title, body, appName = 'Browser', cannedReplies }) {
  const uuid = randomUuid();
  const attrs = [
    attr(ATTR_TITLE, String(title).slice(0, 64)),
    attr(ATTR_BODY, String(body).slice(0, 512)),
    attr(ATTR_APP_NAME, appName.slice(0, 64)),
    attr(ATTR_ICON_TINY, u32le(TIMELINE_RES_NOTIFICATION_GENERIC)),
  ];
  // Actions: Reply (Response type, opens Voice/Canned submenu) + Dismiss.
  const canned = (cannedReplies && cannedReplies.length ? cannedReplies : ['OK', 'Thanks!', 'On my way'])
    .map((s) => s.slice(0, 60));
  const replyAttrs = [attr(ATTR_TITLE, 'Reply'), attr(ATTR_CANNED_RESPONSES, canned.join('\0'))];
  const dismissAttrs = [attr(ATTR_TITLE, 'Dismiss')];
  const actions = [
    cat([new Uint8Array([1, ACTION_RESPONSE, replyAttrs.length]), ...replyAttrs]),
    cat([new Uint8Array([2, ACTION_DISMISS, dismissAttrs.length]), ...dismissAttrs]),
  ];

  const payload = cat([...attrs, ...actions]);
  const header = new Uint8Array(46);
  const dv = new DataView(header.buffer);
  header.set(uuid, 0);
  // parent uuid = zeros (offset 16)
  dv.setUint32(32, Math.floor(Date.now() / 1000), true); // timestamp
  dv.setUint16(36, 0, true);                             // duration
  header[38] = ITEM_TYPE_NOTIFICATION;
  header[39] = 0;                                        // flags
  header[40] = 0;                                        // status: MUST be 0
  header[41] = LAYOUT_NOTIFICATION;
  dv.setUint16(42, payload.length, true);                // payload_length: exact
  header[44] = attrs.length;
  header[45] = actions.length;
  return { uuid, value: cat([header, payload]) };
}

// Insert a notification via an AppInstaller's BlobDB machinery.
// installer must expose _blobDbInsertRaw-ish access; we reuse its token
// counter by sending through the same endpoint handler.
export class NotificationSender {
  constructor(phone, log = () => {}) {
    this.phone = phone;
    this.log = log;
    this.token = (Math.random() * 0xfffe + 1) & 0xffff;
    this.waiter = null;
    // NOTE: shares endpoint 0xb1db with AppInstaller. Route: whoever has
    // a pending waiter gets the response (tokens disambiguate).
    const prev = phone.ppHandlers.get(EP_BLOB_DB);
    phone.onPP(EP_BLOB_DB, (p) => {
      if (p.length >= 3 && this.waiter) {
        const dv = new DataView(p.buffer, p.byteOffset);
        const token = dv.getUint16(0, true);
        if (token === this.lastToken) {
          const w = this.waiter; this.waiter = null;
          w({ status: p[2] });
          return;
        }
      }
      if (prev) prev(p);
    });
  }

  send(opts) {
    const { uuid, value } = buildNotification(opts);
    const token = this.lastToken = (this.token = (this.token % 0xfffe) + 1);
    const msg = new Uint8Array(1 + 2 + 1 + 1 + 16 + 2 + value.length);
    const dv = new DataView(msg.buffer);
    msg[0] = 0x01; // INSERT
    dv.setUint16(1, token, true);
    msg[3] = BLOB_DB_ID_NOTIFS;
    msg[4] = 16;
    msg.set(uuid, 5);
    dv.setUint16(21, value.length, true);
    msg.set(value, 23);
    return new Promise((resolve, reject) => {
      const t = setTimeout(() => { this.waiter = null; reject(new Error('blobdb timeout')); }, 5000);
      this.waiter = (r) => {
        clearTimeout(t);
        if (r.status === 0x01) { this.log('notification delivered'); resolve(uuid); }
        else reject(new Error(`notification rejected, status 0x${r.status.toString(16)}`));
      };
      this.phone.sendPP(EP_BLOB_DB, msg);
    });
  }
}

// Voice dictation service. When the watch opens a dictation session we
// accept it, ignore the (silent) Speex audio, and on stop ask
// `getTranscript()` — browser speech recognition, a typed prompt, or a
// canned string — then return it as a one-sentence transcription.
export class VoiceService {
  // getTranscript: async ({secondsRecorded}) => string|null (null = fail)
  constructor(phone, getTranscript, log = () => {}) {
    this.phone = phone;
    this.getTranscript = getTranscript;
    this.log = log;
    this.session = null;
    phone.onPP(EP_VOICE_CONTROL, (p) => this._onControl(p));
    phone.onPP(EP_AUDIO_DATA, (p) => this._onAudio(p));
  }

  _onControl(p) {
    if (p.length < 9 || p[0] !== 0x01) return; // session setup
    const dv = new DataView(p.buffer, p.byteOffset);
    const flags = dv.getUint32(1, true);
    const sessionType = p[5];
    const sessionId = dv.getUint16(6, true);
    // App-initiated setups carry the app uuid as attribute 0x03 (first).
    let appUuid = null;
    if (p.length > 9) {
      let off = 9;
      const n = p[8];
      for (let i = 0; i < n && off + 3 <= p.length; i++) {
        const id = p[off];
        const len = p[off + 1] | (p[off + 2] << 8);
        if (id === 0x03 && len === 16) appUuid = p.slice(off + 3, off + 19);
        off += 3 + len;
      }
    }
    this.session = { id: sessionId, type: sessionType, flags, appUuid, startedAt: Date.now(), frames: 0 };
    this.log(`voice session ${sessionId} setup (type ${sessionType}${flags & 1 ? ', app-initiated' : ''})`);
    // Accept: [0x01][flags u32le][session_type][result=0]
    const resp = new Uint8Array(7);
    resp[0] = 0x01;
    new DataView(resp.buffer).setUint32(1, flags & 1, true);
    resp[5] = sessionType;
    resp[6] = 0x00;
    this.phone.sendPP(EP_VOICE_CONTROL, resp);
    if (this.onSessionStart) this.onSessionStart();
  }

  _onAudio(p) {
    if (!this.session || p.length < 3) return;
    const sessionId = p[1] | (p[2] << 8);
    if (sessionId !== this.session.id) return;
    if (p[0] === 0x02) {
      this.session.frames++;
    } else if (p[0] === 0x03) {
      const secs = (Date.now() - this.session.startedAt) / 1000;
      this.log(`voice session ${sessionId}: recording stopped (${this.session.frames} frames, ${secs.toFixed(1)}s)`);
      this._finish(secs);
    }
  }

  async _finish(secondsRecorded) {
    const s = this.session;
    if (!s) return;
    let text = null;
    try {
      text = await this.getTranscript({ secondsRecorded });
    } catch (e) {
      this.log('transcript source failed: ' + e.message);
    }
    if (!this.session || this.session.id !== s.id) return; // cancelled meanwhile
    this.session = null;

    const flags = u32le(s.flags & 1);
    if (!text || !text.trim()) {
      // 9-byte failure: result = FailRecognizerError (0x03)
      const fail = cat([new Uint8Array([0x02]), flags,
        new Uint8Array([s.id & 0xff, s.id >> 8, 0x03, 0x00])]);
      this.phone.sendPP(EP_VOICE_CONTROL, fail);
      this.log('voice: sent failure (no transcript)');
      return;
    }
    const words = text.trim().split(/\s+/).slice(0, 32);
    const wordParts = [new Uint8Array([words.length & 0xff, words.length >> 8])];
    for (const w of words) {
      const wb = enc.encode(w.replace(/[\x00-\x1f]/g, '')).subarray(0, 100);
      wordParts.push(cat([new Uint8Array([85, wb.length & 0xff, wb.length >> 8]), wb]));
    }
    const transcription = cat([new Uint8Array([0x01, 0x01]), ...wordParts]); // SentenceList, 1 sentence
    const attrs = [attr(0x02, transcription)];
    if ((s.flags & 1) && s.appUuid) attrs.push(attr(0x03, s.appUuid));
    const msg = cat([
      new Uint8Array([0x02]), flags,
      new Uint8Array([s.id & 0xff, s.id >> 8, 0x00, attrs.length]),
      ...attrs,
    ]);
    this.phone.sendPP(EP_VOICE_CONTROL, msg);
    this.log(`voice: transcription sent ("${words.join(' ')}")`);
  }
}
