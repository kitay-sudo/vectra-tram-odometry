// Демо-развёртывание (simulator/deploy/compose.demo.yml): страница с веб-сервера,
// мост по тому же адресу /ros. Открывается http://<web>/?mode=live БЕЗ ?ros= —
// страница должна сама взять ws://<хост>/ros и подключиться.
//   node /sim/test/demo_test.js http://web /out
const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer-core');
const BASE = process.argv[2] || 'http://web';
const OUT = process.argv[3] || '/out';
const sleep = ms => new Promise(r => setTimeout(r, ms));
(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  const browser = await puppeteer.launch({ executablePath: process.env.PUPPETEER_EXECUTABLE_PATH || '/usr/bin/chromium-browser', headless: 'new', args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'] });
  const page = await browser.newPage();
  await page.setViewport({ width: 1440, height: 900 });
  const errors = [];
  page.on('pageerror', e => errors.push(String(e.message || e)));
  page.on('console', m => { if (m.type() === 'error' && !/WebSocket connection to .* failed/.test(m.text())) errors.push(m.text()); });
  await page.goto(BASE + '/?mode=live', { waitUntil: 'load' });
  const url = await page.evaluate(() => document.getElementById('rvUrl').value);
  let st, t0 = Date.now();
  while (Date.now() - t0 < 240000) {
    st = await page.evaluate(() => ({ ...window.__tvRun.state(), pill: document.getElementById('rvLiveSt').textContent }));
    if (st.rows > 200 && st.nodeHz) break;
    await sleep(1000);
  }
  await page.screenshot({ path: path.join(OUT, 'demo_live.png') });
  // прогон с того же сервера: replays/*.js отдаются веб-сервером
  await page.goto(BASE + '/?mode=replay', { waitUntil: 'load' });
  await page.waitForFunction(() => window.__tvRun && window.__tvRun.state().n > 0, { timeout: 60000 }).catch(() => {});
  const rp = await page.evaluate(() => window.__tvRun.state());
  const ok = /^ws:\/\/[^/]+\/ros$/.test(url) && /на связи/.test(st.pill) && st.rows > 200 && st.nodeHz > 18 && st.nodeHz < 22 && rp.n > 0 && errors.length === 0;
  const rep = { base: BASE, default_ros_url: url, live: st, replay_rows: rp.n, errors, ok };
  console.log(JSON.stringify(rep, null, 1));
  fs.writeFileSync(path.join(OUT, 'demo_test.json'), JSON.stringify(rep, null, 1));
  await browser.close();
  process.exit(ok ? 0 : 1);
})().catch(e => { console.error(e); process.exit(2); });
