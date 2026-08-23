// PebbleKit JS runtime for the wasm emulator.
//
// Runs a pbw's pebble-js-app.js in a sandbox (hidden same-origin iframe in
// the browser, node:vm in tests) with a `Pebble` shim wired to the
// AppMessage client. App lifecycle follows the watch's app_run_state
// notifications on endpoint 52 (0x34): RUNNING starts the app's JS,
// NOT_RUNNING tears it down.

const EP_APP_RUN_STATE = 0x0034;
const STATE_RUNNING = 0x01;
const STATE_NOT_RUNNING = 0x02;

const uuidKey = (u) => [...u].map((b) => b.toString(16).padStart(2, '0')).join('');

// XMLHttpRequest implemented over fetch, with an optional CORS-proxy
// fallback: requests are tried direct first; on a network failure
// (typically CORS) the same request is retried through proxyUrl.
// Covers the subset PebbleKit JS apps use.
export function makeXhrOverFetch(fetchFn, proxyUrl, log = () => {}) {
  const proxied = (url) => proxyUrl ? proxyUrl + encodeURIComponent(url) : null;

  async function robustFetch(url, opts) {
    try {
      return await fetchFn(url, opts);
    } catch (e) {
      const p = /^https?:/i.test(url) && proxied(url);
      if (!p) throw e;
      log(`direct fetch blocked (${String(url).slice(0, 80)}), retrying via proxy`);
      return fetchFn(p, opts);
    }
  }

  return class XMLHttpRequest {
    constructor() {
      this.readyState = 0;
      this.status = 0;
      this.responseText = '';
      this.response = '';
      this.responseType = '';
      this.timeout = 0;
      this._headers = {};
      this.onload = null; this.onerror = null; this.ontimeout = null;
      this.onreadystatechange = null;
    }
    open(method, url) { this._method = method; this._url = url; this.readyState = 1; }
    setRequestHeader(k, v) { this._headers[k] = v; }
    getAllResponseHeaders() { return this._rawHeaders || ''; }
    getResponseHeader(k) { return (this._headerMap && this._headerMap.get(k.toLowerCase())) || null; }
    abort() { this._aborted = true; }
    addEventListener(ev, fn) { this['on' + ev] = fn; }
    send(body) {
      const ctrl = typeof AbortController !== 'undefined' ? new AbortController() : null;
      let timer = null;
      if (this.timeout > 0 && ctrl) {
        timer = setTimeout(() => { this._timedOut = true; ctrl.abort(); }, this.timeout);
      }
      robustFetch(this._url, {
        method: this._method || 'GET',
        headers: this._headers,
        body: body ?? undefined,
        signal: ctrl ? ctrl.signal : undefined,
      }).then(async (r) => {
        if (timer) clearTimeout(timer);
        if (this._aborted) return;
        this.status = r.status;
        this.statusText = r.statusText;
        this._headerMap = new Map();
        let raw = '';
        r.headers.forEach((v, k) => { this._headerMap.set(k, v); raw += k + ': ' + v + '\r\n'; });
        this._rawHeaders = raw;
        if (this.responseType === 'arraybuffer') {
          this.response = await r.arrayBuffer();
        } else {
          this.responseText = await r.text();
          this.response = this.responseText;
          if (this.responseType === 'json') { try { this.response = JSON.parse(this.responseText); } catch (e) { this.response = null; } }
        }
        this.readyState = 4;
        if (this.onreadystatechange) this.onreadystatechange();
        if (this.onload) this.onload();
      }).catch((e) => {
        if (timer) clearTimeout(timer);
        if (this._aborted && !this._timedOut) return;
        this.readyState = 4;
        log(`XHR ${this._timedOut ? 'timeout' : 'failed'}: ${String(this._url).slice(0, 80)} (${e.message})`);
        if (this._timedOut && this.ontimeout) this.ontimeout(e);
        else if (this.onerror) this.onerror(e);
      });
    }
  };
}

export class PkjsRuntime {
  // appMessage: AppMessageClient; opts: {createSandbox, storage, proxyUrl,
  // openUrl, log}. createSandbox(globals) -> {run(code), dispose()}.
  constructor(phone, appMessage, opts = {}) {
    this.phone = phone;
    this.am = appMessage;
    this.log = opts.log || (() => {});
    this.opts = opts;
    this.apps = new Map();      // uuidKey -> {js, appKeys, name, uuid}
    this.current = null;        // {app, sandbox, listeners}
    phone.onPP(EP_APP_RUN_STATE, (p) => this._onRunState(p));
  }

  registerApp(uuid, { js, appKeys, name }) {
    this.apps.set(uuidKey(uuid), { js, appKeys: appKeys || {}, name: name || 'app', uuid });
    this.log(`pkjs registered for ${name} (${js ? js.length + ' chars' : 'no js'})`);
  }

  _onRunState(p) {
    if (p.length < 17) return;
    const state = p[0];
    const key = uuidKey(p.subarray(1, 17));
    if (state === STATE_RUNNING) {
      const app = this.apps.get(key);
      this._stop();
      if (app && app.js) this._start(app);
    } else if (state === STATE_NOT_RUNNING) {
      if (this.current && uuidKey(this.current.app.uuid) === key) this._stop();
    }
  }

  _stop() {
    if (!this.current) return;
    this.log(`pkjs stop: ${this.current.app.name}`);
    try { this.current.sandbox.dispose(); } catch (e) { /* already gone */ }
    this.am.setApp(null, null);
    this.current = null;
  }

  _start(app) {
    this.log(`pkjs start: ${app.name}`);
    const listeners = new Map(); // event -> [fn]
    const on = (ev) => listeners.get(ev) || [];

    this.am.setApp(app.uuid, app.appKeys);
    this.am.onPush = (payload) => {
      for (const fn of on('appmessage')) {
        try { fn({ type: 'appmessage', payload }); } catch (e) { this.log('pkjs appmessage handler error: ' + e.message); }
      }
    };

    const storage = this.opts.storage
      ? this.opts.storage(uuidKey(app.uuid))
      : makeMemoryStorage();

    const Pebble = {
      addEventListener: (ev, fn) => {
        if (!listeners.has(ev)) listeners.set(ev, []);
        listeners.get(ev).push(fn);
      },
      removeEventListener: (ev, fn) => {
        const l = listeners.get(ev);
        if (l) { const i = l.indexOf(fn); if (i >= 0) l.splice(i, 1); }
      },
      sendAppMessage: (dict, onOk, onErr) => {
        this.am.send(dict).then(
          () => { if (onOk) onOk({ data: { transactionId: 0 } }); },
          (e) => { this.log('sendAppMessage failed: ' + e.message); if (onErr) onErr({ data: { transactionId: 0 } }, e); },
        );
        return 0;
      },
      getAccountToken: () => this._token('account'),
      getWatchToken: () => this._token('watch'),
      getActiveWatchInfo: () => ({
        platform: 'emery',
        model: 'pebble_time_2',
        language: 'en_US',
        firmware: { major: 4, minor: 35, patch: 0, suffix: '' },
      }),
      getTimelineToken: (ok, err) => { if (err) err('timeline not available in the emulator'); },
      timelineSubscribe: (t, ok, err) => { if (err) err('timeline not available'); },
      timelineUnsubscribe: (t, ok, err) => { if (err) err('timeline not available'); },
      timelineSubscriptions: (ok, err) => { if (err) err('timeline not available'); },
      showSimpleNotificationOnPebble: (title, body) => {
        this.log(`notification: ${title} — ${body}`);
      },
      openURL: (url) => {
        this._configUrl = url;
        if (this.opts.openUrl) this.opts.openUrl(url, (responseData) => {
          for (const fn of on('webviewclosed')) {
            try { fn({ type: 'webviewclosed', response: responseData }); } catch (e) { /* app handler error */ }
          }
        });
        else this.log('openURL (no handler): ' + url);
      },
      platform: 'pkjs',
    };

    const sandbox = this.opts.createSandbox({
      Pebble,
      localStorage: storage,
      console: {
        log: (...a) => this.log('[js] ' + a.join(' ')),
        warn: (...a) => this.log('[js] ' + a.join(' ')),
        error: (...a) => this.log('[js!] ' + a.join(' ')),
      },
    });
    this.current = { app, sandbox, listeners };

    try {
      sandbox.run(app.js);
    } catch (e) {
      this.log('pkjs app threw during load: ' + (e.stack || e.message));
    }
    // 'ready' fires after the script settles, like the phone apps do.
    setTimeout(() => {
      if (this.current && this.current.app === app) {
        for (const fn of on('ready')) {
          try { fn({ type: 'ready' }); } catch (e) { this.log('pkjs ready handler error: ' + (e.stack || e.message)); }
        }
        // Deliver any pushes the watch sent while the JS was starting.
        if (this.am.drainPushes) this.am.drainPushes(app.uuid);
      }
    }, 0);
  }

  // Show a config page for the current app (user pressed settings).
  showConfiguration() {
    if (!this.current) return false;
    const l = this.current.listeners.get('showConfiguration');
    if (l && l.length) { l.forEach((fn) => fn({ type: 'showConfiguration' })); return true; }
    return false;
  }

  _token(kind) {
    const store = this.opts.tokenStore || (this._mem = this._mem || {});
    const k = 'pkjs-token-' + kind;
    let v = store[k];
    if (!v) {
      v = [...crypto.getRandomValues(new Uint8Array(16))].map((b) => b.toString(16).padStart(2, '0')).join('');
      store[k] = v;
    }
    return v;
  }
}

export function makeMemoryStorage() {
  const m = new Map();
  return {
    getItem: (k) => (m.has(k) ? m.get(k) : null),
    setItem: (k, v) => m.set(String(k), String(v)),
    removeItem: (k) => m.delete(k),
    clear: () => m.clear(),
    key: (i) => [...m.keys()][i] ?? null,
    get length() { return m.size; },
  };
}

// IP-based location, used when the browser's own geolocation fails
// (common: OS location services disabled). City-level accuracy is
// plenty for weather watchfaces. Cached per page load.
let ipLocationCache = null;
async function ipLocate(fetchFn) {
  if (ipLocationCache) return ipLocationCache;
  const r = await fetchFn('https://ipwho.is/');
  const j = await r.json();
  if (!j || j.success === false || typeof j.latitude !== 'number') {
    throw new Error('IP lookup returned no location');
  }
  ipLocationCache = { latitude: j.latitude, longitude: j.longitude, city: j.city };
  return ipLocationCache;
}

function fakePosition(lat, lon, accuracy) {
  return {
    coords: { latitude: lat, longitude: lon, accuracy,
              altitude: null, altitudeAccuracy: null, heading: null, speed: null },
    timestamp: Date.now(),
  };
}

// Browser sandbox: hidden same-origin iframe. The app JS gets the page's
// real fetch/XHR (patched with the proxy fallback), geolocation, etc.
// fixedLoc: optional {latitude, longitude} override (?loc=lat,lon).
export function makeIframeSandbox(proxyUrl, log = () => {}, fixedLoc = null) {
  return (globals) => {
    const frame = document.createElement('iframe');
    frame.style.display = 'none';
    frame.setAttribute('allow', 'geolocation *');
    document.body.appendChild(frame);
    const w = frame.contentWindow;
    // Window properties like localStorage are getter-only; use
    // defineProperty for everything rather than plain assignment.
    for (const [k, v] of Object.entries(globals)) {
      Object.defineProperty(w, k, { value: v, configurable: true, writable: true });
    }
    const XHR = makeXhrOverFetch(w.fetch.bind(w), proxyUrl, log);
    w.XMLHttpRequest = XHR;
    const nativeFetch = w.fetch.bind(w);
    w.fetch = (url, opts) => nativeFetch(url, opts).catch((e) => {
      if (proxyUrl && typeof url === 'string' && /^https?:/i.test(url)) {
        log(`direct fetch blocked (${url.slice(0, 80)}), retrying via proxy`);
        return nativeFetch(proxyUrl + encodeURIComponent(url), opts);
      }
      throw e;
    });
    // Geolocation with fallbacks: ?loc= override -> native -> IP-based.
    // An OS-level denial must not strand weather watchfaces.
    try {
      const geo = w.navigator.geolocation;
      // Use the TOP page's geolocation, not the iframe's — it removes
      // the permissions-delegation variable, and the permission grant
      // is attributed to the visible page either way.
      const pageGeo = window.navigator.geolocation;
      const origGet = pageGeo.getCurrentPosition.bind(pageGeo);
      const fallback = (ok, err, cause) => {
        log(`geolocation failed (${cause}); trying IP-based location…`);
        ipLocate(w.fetch).then((loc) => {
          log(`IP location: ~${loc.city || 'unknown'}`);
          ok(fakePosition(loc.latitude, loc.longitude, 25000));
        }).catch((e2) => {
          log('IP location failed too: ' + e2.message);
          if (err) err({ code: 2, message: cause });
        });
      };
      const resolvePosition = (ok, err, opts) => {
        log('app requested geolocation…');
        if (fixedLoc) {
          log(`using fixed location ${fixedLoc.latitude},${fixedLoc.longitude}`);
          ok(fakePosition(fixedLoc.latitude, fixedLoc.longitude, 10));
          return;
        }
        origGet(
          (pos) => { log(`geolocation ok (±${Math.round(pos.coords.accuracy)}m)`); ok(pos); },
          (e) => fallback(ok, err, `${e.message} (code ${e.code})`),
          opts,
        );
      };
      geo.getCurrentPosition = resolvePosition;
      // Single-shot semantics are fine for watchfaces polling weather.
      geo.watchPosition = (ok, err, opts) => { resolvePosition(ok, err, opts); return 0; };
      geo.clearWatch = () => {};
    } catch (e) { /* geolocation unavailable in this context */ }
    return {
      run: (code) => w.eval(code),
      dispose: () => frame.remove(),
    };
  };
}
