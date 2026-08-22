// Test the assembled single-file emulator page under chromium.
// Usage: node page-test.mjs <html-file> <coi|nocoi> <wait-seconds> <shot.png>
import { chromium } from 'playwright';
import { createServer } from 'node:http';
import { readFileSync } from 'node:fs';

const [file, mode, waitS, shot] = process.argv.slice(2);
const html = readFileSync(file);

const server = createServer((req, res) => {
  const headers = { 'Content-Type': 'text/html', 'Cache-Control': 'no-store' };
  if (mode === 'coi') {
    headers['Cross-Origin-Opener-Policy'] = 'same-origin';
    headers['Cross-Origin-Embedder-Policy'] = 'require-corp';
  }
  res.writeHead(200, headers);
  res.end(html);
});
await new Promise((r) => server.listen(8099, r));

const browser = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium', args: ['--no-sandbox'] });
const page = await browser.newPage({ viewport: { width: 760, height: 1100 } });
page.on('pageerror', (e) => console.log('[pageerror]', String(e).slice(0, 300)));
page.on('console', (m) => { const t = m.text(); if (/err|fail|abort|BLOCKED/i.test(t)) console.log('[console]', t.slice(0, 200)); });

await page.goto('http://localhost:8099/?auto', { waitUntil: 'load' });
console.log('loaded, mode=' + mode);

const t0 = Date.now();
let last = '';
while ((Date.now() - t0) / 1000 < Number(waitS)) {
  await new Promise((r) => setTimeout(r, 5000));
  const state = await page.evaluate(() => ({
    status: document.getElementById('status')?.textContent || '',
    fps: document.getElementById('fps')?.textContent || '',
    checks: [...document.querySelectorAll('.chk')].map((c) => c.textContent).join(' | '),
    fail: document.getElementById('fail')?.style.display !== 'none',
    nonWhite: (() => {
      const c = document.getElementById('screen');
      const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
      let n = 0;
      for (let i = 0; i < d.length; i += 4) if (d[i] < 240 || d[i + 1] < 240 || d[i + 2] < 240) n++;
      return n;
    })(),
  }));
  const line = JSON.stringify(state);
  if (line !== last) { console.log(`[${Math.round((Date.now() - t0) / 1000)}s]`, line); last = line; }
  if (mode === 'nocoi' && state.fail) { console.log('DIAGNOSTIC SHOWN AS EXPECTED'); break; }
  if (mode === 'coi' && state.status.includes('Display active') && state.nonWhite > 200 && state.nonWhite < 40000) {
    console.log('PIXELS ON CANVAS — BOOT OK'); break;
  }
}
await page.screenshot({ path: shot });
console.log('screenshot: ' + shot);
await browser.close();
server.close();
process.exit(0);
