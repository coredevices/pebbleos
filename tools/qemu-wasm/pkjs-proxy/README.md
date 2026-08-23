# PebbleKit JS CORS proxy (Cloudflare Worker)

PebbleKit JS apps in the wasm emulator call 2016-era weather/transit APIs
that never send CORS headers. The emulator page tries every request
direct first and, on failure, retries through this worker, which adds
`Access-Control-Allow-Origin` and streams the body.

Hardening built in: origin allowlist (only the emulator pages may use
it), GET/HEAD/POST only, cookies and Authorization stripped both ways,
private/internal hostnames refused.

## Deploy

```sh
npm i -g wrangler
wrangler deploy worker.js --name pkjs-proxy --compatibility-date 2026-08-01
```

Then route it at `pkjs-proxy.repebble.com` (Workers → Custom Domains),
or pass the workers.dev URL to the page instead:

```
https://ericmigi.github.io/pebble-qemu-wasm/?pkjs_proxy=https%3A%2F%2Fpkjs-proxy.<acct>.workers.dev%2F%3Furl%3D
```

The page's default proxy prefix is `https://pkjs-proxy.repebble.com/?url=`
(see `PKJS_PROXY` in index.html). Edit `ALLOWED_ORIGINS` in worker.js if
the site moves.
