// Resolve an apps.repebble.com / apps.rebble.io link (or bare app id,
// or direct .pbw URL) to a downloaded .pbw.
//
// Store API (pebble-dev/rebble-appstore-api, CORS-enabled):
//   GET {base}/api/v1/apps/id/{id}  ->  {data: [{title, latest_release: {pbw_file}}]}

const API_BASES = [
  'https://appstore-api.repebble.com',
  'https://appstore-api.rebble.io',
];

// "just-big_554102e564edb7fb2c000080" -> "554102e564edb7fb2c000080"
export function extractAppId(input) {
  const s = input.trim();
  const hex24 = /[0-9a-f]{24}/gi;
  const matches = s.match(hex24);
  return matches ? matches[matches.length - 1] : null;
}

// Fetch with a CORS-proxy fallback: direct first, then through the
// proxy prefix (target URL appended URI-encoded) when the direct
// request is CORS-blocked — e.g. the store's pbw asset bucket.
async function robustFetch(url, proxy) {
  try {
    const r = await fetch(url, { mode: 'cors' });
    if (r.ok || !proxy) return r;
    throw new Error(`HTTP ${r.status}`);
  } catch (e) {
    if (!proxy) throw e;
    return fetch(proxy + encodeURIComponent(url), { mode: 'cors' });
  }
}

async function fetchJson(url, proxy) {
  const r = await robustFetch(url, proxy);
  if (!r.ok) throw new Error(`HTTP ${r.status} from ${new URL(url).host}`);
  return r.json();
}

// Returns {name, pbw: Uint8Array}
export async function fetchPbwFromStore(input, hardware = 'emery', onStatus = () => {}, proxy = null) {
  const direct = input.trim();
  if (/\.pbw($|\?)/i.test(direct)) {
    onStatus('downloading .pbw…');
    const r = await robustFetch(direct, proxy);
    if (!r.ok) throw new Error(`HTTP ${r.status} downloading pbw`);
    return { name: direct.split('/').pop(), pbw: new Uint8Array(await r.arrayBuffer()) };
  }

  const id = extractAppId(direct);
  if (!id) {
    throw new Error('could not find an app id in that link — paste an ' +
      'apps.repebble.com app URL (…_<24 hex chars>) or a direct .pbw URL');
  }

  let lastErr = null;
  for (const base of API_BASES) {
    try {
      onStatus(`looking up app on ${new URL(base).host}…`);
      const json = await fetchJson(`${base}/api/v1/apps/id/${id}?hardware=${hardware}`, proxy);
      const app = json.data && json.data[0];
      if (!app) throw new Error('app not found in store');
      const pbwUrl = app.latest_release && app.latest_release.pbw_file;
      if (!pbwUrl) throw new Error(`"${app.title}" has no downloadable release`);
      onStatus(`downloading "${app.title}"…`);
      const r = await robustFetch(pbwUrl, proxy);
      if (!r.ok) throw new Error(`HTTP ${r.status} downloading pbw`);
      return { name: app.title, pbw: new Uint8Array(await r.arrayBuffer()) };
    } catch (e) {
      lastErr = e;
    }
  }
  throw new Error(`store lookup failed (${lastErr?.message}). ` +
    'You can also download the .pbw yourself and drag-drop it onto the watch.');
}
