// CORS proxy for PebbleKit JS apps running in the wasm emulator.
//
// The emulator page retries failed cross-origin requests through
//   https://<this-worker>/?url=<URI-encoded target>
// adding the CORS headers the target API never sends. Deploy with
// wrangler (see README.md).

const ALLOWED_ORIGINS = [
  'https://ericmigi.github.io',
  'http://localhost:8080',
  'http://localhost:8000',
  'http://127.0.0.1:8080',
];

const ALLOWED_METHODS = ['GET', 'HEAD', 'POST', 'OPTIONS'];

// Never proxy toward anything that smells internal.
const BLOCKED_HOST = /^(localhost|127\.|10\.|192\.168\.|169\.254\.|172\.(1[6-9]|2\d|3[01])\.|\[::1\]|metadata\.google|.*\.internal)$/i;

function corsHeaders(origin) {
  return {
    'Access-Control-Allow-Origin': origin || '*',
    'Access-Control-Allow-Methods': ALLOWED_METHODS.join(', '),
    'Access-Control-Allow-Headers': '*',
    'Access-Control-Expose-Headers': 'Content-Length, Content-Type, ETag',
    'Access-Control-Max-Age': '86400',
    'Vary': 'Origin',
  };
}

export default {
  async fetch(request) {
    const reqUrl = new URL(request.url);
    const origin = request.headers.get('Origin');

    // Only the emulator origins may use the proxy.
    if (!origin || !ALLOWED_ORIGINS.includes(origin)) {
      return new Response('origin not allowed', { status: 403 });
    }
    if (!ALLOWED_METHODS.includes(request.method)) {
      return new Response('method not allowed', { status: 405 });
    }
    if (request.method === 'OPTIONS') {
      return new Response(null, { status: 204, headers: corsHeaders(origin) });
    }

    const target = reqUrl.searchParams.get('url');
    if (!target) return new Response('missing ?url=', { status: 400 });

    let t;
    try { t = new URL(target); } catch (e) {
      return new Response('bad target url', { status: 400 });
    }
    if (t.protocol !== 'https:' && t.protocol !== 'http:') {
      return new Response('unsupported scheme', { status: 400 });
    }
    if (BLOCKED_HOST.test(t.hostname)) {
      return new Response('target not allowed', { status: 403 });
    }

    // Forward the request with credentials stripped in both directions.
    const fwdHeaders = new Headers();
    for (const [k, v] of request.headers) {
      const lk = k.toLowerCase();
      if (['cookie', 'authorization', 'origin', 'referer', 'host'].includes(lk)) continue;
      if (lk.startsWith('cf-') || lk.startsWith('x-forwarded')) continue;
      fwdHeaders.set(k, v);
    }

    let upstream;
    try {
      upstream = await fetch(t.href, {
        method: request.method,
        headers: fwdHeaders,
        body: request.method === 'POST' ? request.body : undefined,
        redirect: 'follow',
      });
    } catch (e) {
      return new Response('upstream fetch failed: ' + e.message,
        { status: 502, headers: corsHeaders(origin) });
    }

    const respHeaders = new Headers(corsHeaders(origin));
    for (const [k, v] of upstream.headers) {
      const lk = k.toLowerCase();
      if (['set-cookie', 'access-control-allow-origin'].includes(lk)) continue;
      if (['content-type', 'content-length', 'cache-control', 'etag', 'last-modified', 'expires'].includes(lk)) {
        respHeaders.set(k, v);
      }
    }
    return new Response(upstream.body, { status: upstream.status, headers: respHeaders });
  },
};
