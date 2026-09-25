// Проверка страницы в настоящем Chromium без сети (headless, puppeteer-core).
//
//   docker run --rm --network none -v <worktree>/simulator:/sim:ro -v <out>:/out \
//     vectra/tram:sim node /sim/test/page_test.js [/sim/index.html] [/out]
//
// Образ vectra/tram:sim — node:22-alpine + chromium + puppeteer-core (Dockerfile
// рядом: simulator/test/Dockerfile). Страница открывается с file://, сеть
// контейнера выключена: любая внешняя загрузка (CDN, шрифты) — ошибка теста.
// Проверяется: 0 JS-исключений и ошибок консоли во всех режимах; песочница
// крутится; каждый экспортированный прогон грузится, проигрывается, и метрики
// страницы в конце прогона совпадают с итогом экспортёра; подпись JS-порта
// следует отпечатку ядра прогонов; живой режим с имитатором rosbridge (ws в этом
// же контейнере): правка поля адреса при подключении не останавливает страницу,
// переход границы квадратов MGRS 37U CB | DB — без скачка в обоих соглашениях;
// без моста — «нет связи». Скриншоты — в <out>.
const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer-core');

const PAGE = process.argv[2] || '/sim/index.html';
const OUT = process.argv[3] || '/out';
const URL0 = 'file://' + PAGE;
const sleep = ms => new Promise(r => setTimeout(r, ms));

(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  const browser = await puppeteer.launch({
    executablePath: process.env.PUPPETEER_EXECUTABLE_PATH || '/usr/bin/chromium-browser',
    headless: 'new', args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage', '--font-render-hinting=none'],
  });
  const report = { page: PAGE, errors: [], external_requests: [], checks: [], shots: [] };
  const check = (name, ok, info) => { report.checks.push({ name, ok: !!ok, info }); console.log(`${ok ? 'OK  ' : 'FAIL'} ${name}${info !== undefined ? ' — ' + JSON.stringify(info) : ''}`); };
  async function open(query, vp = { width: 1440, height: 900 }) {
    const page = await browser.newPage();
    await page.setViewport(vp);
    page.on('pageerror', e => report.errors.push({ query, kind: 'pageerror', msg: String(e.message || e) }));
    page.on('console', m => { if (m.type() === 'error') report.errors.push({ query, kind: 'console', msg: m.text() }); });
    page.on('request', r => { const u = r.url(); if (!/^(file|data|blob):/.test(u) && !/^wss?:/.test(u)) report.external_requests.push(u); });
    page.on('requestfailed', r => { const u = r.url(); if (/^file:/.test(u)) report.errors.push({ query, kind: 'requestfailed', msg: u }); });
    await page.goto(URL0 + query, { waitUntil: 'load' });
    return page;
  }
  const shot = async (page, name) => { const f = path.join(OUT, name); await page.screenshot({ path: f }); report.shots.push(name); };

  // ---------------- песочница
  {
    const page = await open('?mode=sandbox');
    await sleep(12000);
    const st = await page.evaluate(() => ({ mode: window.__tvMode, speed: document.getElementById('bV').textContent, s: document.getElementById('bS').textContent, font: getComputedStyle(document.body).fontFamily, grid: getComputedStyle(document.querySelector('#dash .grid')).display }));
    check('песочница: режим и табло', st.mode === 'sandbox' && st.grid === 'grid', st);
    await shot(page, 'sandbox_desktop.png');
    // модальные окна песочницы
    for (const k of ['model', 'ros', 'metrics']) { await page.click(`[data-open="${k}"]`); await sleep(1500); if (k === 'metrics') await shot(page, 'sandbox_metrics.png'); await page.keyboard.press('Escape'); }
    await page.setViewport({ width: 390, height: 844, isMobile: true, hasTouch: true });
    await sleep(2500);
    const ov = await page.evaluate(() => document.documentElement.scrollWidth - innerWidth);
    check('песочница, телефон: нет горизонтальной прокрутки', ov <= 1, { overflow_px: ov });
    await shot(page, 'sandbox_mobile.png');
    await page.close();
  }

  // ---------------- прогон данных комиссии
  const idxPage = await open('?mode=replay');
  await idxPage.waitForFunction(() => window.__tvRun && window.__tvRun.state().n > 0, { timeout: 30000 });
  const index = await idxPage.evaluate(() => (window.TV_REPLAY_INDEX || []).map(e => ({ key: e.key, ru: e.ru })));
  check('прогон: список экспортированных вариантов', index.length > 0, index.map(e => e.key));
  await idxPage.close();
  report.replays = {};
  for (const e of index) {
    const page = await open(`?mode=replay&run=${e.key}&win=300`);
    await page.waitForFunction(k => window.__tvRun && window.__tvRun.state().key === k && window.__tvRun.state().n > 0, { timeout: 30000 }, e.key);
    // проигрывание ×60: время идёт
    const t0 = await page.evaluate(() => window.__tvRun.state().t);
    await page.evaluate(() => window.__tvRun.play(true, 60));
    await sleep(3000);
    const t1 = await page.evaluate(() => window.__tvRun.state().t);
    await page.evaluate(() => window.__tvRun.play(false));
    const inj = await page.evaluate(() => { const s = window.__tvRun.summary(); return s && s.window ? s.window : null; });
    if (inj) await page.evaluate(w => window.__tvRun.seek(w.t0_rel + Math.min(w.eval, 20) + 5), inj);
    else await page.evaluate(() => window.__tvRun.seek(600));
    await sleep(1200);
    await shot(page, `replay_${e.key}.png`);
    // конец прогона: метрики страницы = итог экспортёра
    const res = await page.evaluate(() => { const r = window.__tvRun; r.seek(1e9); const m = r.metrics(), s = r.summary(); return { m, s }; });
    const m = res.m, s = res.s;
    const same = (a, b, d) => a !== null && b !== null && isFinite(a) && isFinite(b) && a.toFixed(d) === b.toFixed(d);
    const cmp = {
      mae: [m.mae, s.v_mae], mae_naive: [m.maeN, s.naive_v_mae], drift: [m.drift, s.drift_pct], drift_naive: [m.driftN, s.naive_drift_pct],
      p2d: [m.h, s.p2d_mean], cov2s: [m.cov, s.cov2s_v],
    };
    // точность показа на странице: MAE 4 знака, дрейф 3, план 2, доля 3; допуск — квантование файла
    const near = (a, b, tol) => isFinite(a) && isFinite(b) && Math.abs(a - b) <= tol;
    // дрейф — допуск и относительный: у базы с отказом датчиков дрейф ~3 %, и
    // квантование файла даёт 3e-5 (интеграция 26.09, both_zero: 3,06203 / 3,06200)
    const nearRel = (a, b, tol) => near(a, b, Math.max(tol, tol * Math.abs(b)));
    const ok = near(m.mae, s.v_mae, 2e-5) && near(m.maeN, s.naive_v_mae, 2e-5) && nearRel(m.drift, s.drift_pct, 2e-5) && nearRel(m.driftN, s.naive_drift_pct, 2e-5)
      && near(m.h, s.p2d_mean, 2e-3) && near(m.cov, s.cov2s_v, 1e-6);
    const shown = same(m.mae, s.v_mae, 4) && same(m.maeN, s.naive_v_mae, 4) && same(m.drift, s.drift_pct, 3) && same(m.driftN, s.naive_drift_pct, 3) && same(m.h, s.p2d_mean, 2) && same(m.cov, s.cov2s_v, 3);
    cmp.shown_equal = [shown, true];
    report.replays[e.key] = { played_s_in_3s_x60: +(t1 - t0).toFixed(1), cmp };
    check(`прогон ${e.key}: проигрывается (×60)`, t1 - t0 > 60, { dt: +(t1 - t0).toFixed(1) });
    check(`прогон ${e.key}: метрики страницы = итог экспортёра`, ok, Object.fromEntries(Object.entries(cmp).map(([k, [a, b]]) => [k, [typeof a === 'number' ? +a.toFixed(6) : a, typeof b === 'number' ? +b.toFixed(6) : b]])));
    await page.close();
  }
  // телефон и окно «весь прогон»
  if (index.length) {
    const key = (index.find(e => /front_zero/.test(e.key)) || index[0]).key;
    const page = await open(`?mode=replay&run=${key}&win=0`, { width: 390, height: 844, isMobile: true, hasTouch: true });
    await page.waitForFunction(k => window.__tvRun && window.__tvRun.state().key === k && window.__tvRun.state().n > 0, { timeout: 30000 }, key);
    await page.evaluate(() => window.__tvRun.seek(400));
    await sleep(1200);
    const ov = await page.evaluate(() => { const rv = document.getElementById('runView'); return Math.max(document.documentElement.scrollWidth - innerWidth, rv.scrollWidth - rv.clientWidth); });
    check('прогон, телефон: нет горизонтальной прокрутки', ov <= 1, { overflow_px: ov });
    await shot(page, 'replay_mobile_top.png');
    await page.evaluate(() => { document.getElementById('runView').scrollTop = 900; });
    await sleep(600);
    await shot(page, 'replay_mobile_panels.png');
    await page.setViewport({ width: 1440, height: 900 });
    await page.click('#themeBtn');                    // ночная тема
    await page.evaluate(() => { document.getElementById('runView').scrollTop = 0; });
    await sleep(1200);
    await shot(page, 'replay_dark_desktop.png');
    await page.close();
  }

  // ---------------- подпись JS-порта: прогоны посчитаны другим ядром -> «упрощённое ядро»
  {
    const badge = p => p.evaluate(() => ({ badge: document.querySelector('header .badge-sub').textContent, note: document.getElementById('portNote').textContent }));
    let page = await open('?mode=sandbox');
    await sleep(2500);
    const a = await badge(page);
    const idxSha = await page.evaluate(() => (window.TV_REPLAY_INDEX || []).map(e => e.core_sha1));
    const portSha = await page.evaluate(() => window.TV_EST_PORT.core_sha1);
    const same = idxSha.every(s => !s || s === portSha);      // нет отпечатка (старый экспорт) — подпись не меняется
    check('подпись порта по отпечатку ядра прогонов', same ? /сверен/.test(a.badge) && !/упрощ/.test(a.badge) : /упрощённое ядро/.test(a.badge), { port: portSha, replays: [...new Set(idxSha)], ...a });
    await page.close();
    // подмена отпечатка в списке прогонов (как после правки ядра без переноса порта)
    page = await browser.newPage();
    page.on('pageerror', e => report.errors.push({ query: 'port-stale', kind: 'pageerror', msg: String(e.message || e) }));
    await page.evaluateOnNewDocument(() => {
      let v; Object.defineProperty(window, 'TV_REPLAY_INDEX', { configurable: true, get: () => v, set: x => { v = x.map(e => ({ ...e, core_sha1: 'deadbeef0000' })); } });
    });
    await page.goto(URL0 + '?mode=sandbox', { waitUntil: 'load' });
    await sleep(2500);
    const b = await badge(page);
    await page.click('[data-open="model"]'); await sleep(300);
    const mtxt = await page.evaluate(() => document.getElementById('ghost').textContent);
    await page.keyboard.press('Escape');
    check('ядро изменилось после сверки -> «упрощённое ядро»', /упрощённое ядро/.test(b.badge) && /упрощённое ядро/.test(b.note) && /упрощённом ядре/.test(mtxt) && !/10⁻¹²/.test(mtxt), { ...b, model: mtxt.slice(0, 90) });
    await page.close();
  }

  // ---------------- живой ROS 2 с имитатором rosbridge (ws на 127.0.0.1:9090 в этом же контейнере)
  {
    const { WebSocketServer } = require('ws');
    // UTM (как на странице и в tools/export_replay.py): x — восток, y — север
    function utmEN(lat, lon, zone) {
      const a = 6378137, fl = 1 / 298.257223563, n = fl / (2 - fl), A = a / (1 + n) * (1 + n * n / 4 + n ** 4 / 64);
      const al = [n / 2 - 2 * n * n / 3 + 5 * n ** 3 / 16 + 41 * n ** 4 / 180, 13 * n * n / 48 - 3 * n ** 3 / 5 + 557 * n ** 4 / 1440, 61 * n ** 3 / 240 - 103 * n ** 4 / 140, 49561 * n ** 4 / 161280];
      const phi = lat * Math.PI / 180, dl = (lon - (zone * 6 - 183)) * Math.PI / 180, c = 2 * Math.sqrt(n) / (1 + n);
      const t = Math.sinh(Math.atanh(Math.sin(phi)) - c * Math.atanh(c * Math.sin(phi)));
      const xi = Math.atan(t / Math.cos(dl)), eta = Math.atanh(Math.sin(dl) / Math.sqrt(1 + t * t));
      let E = eta, N = xi;
      for (let j = 1; j <= 4; j++) { E += al[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta); N += al[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta); }
      return [500000 + 0.9996 * A * E, 0.9996 * A * N];
    }
    const LAT = 55.79;
    let lo = 37.3, hi = 37.5;                            // долгота, где E = 399 800 м (западнее границы 37U CB | DB)
    for (let i = 0; i < 60; i++) { const mid = (lo + hi) / 2; if (utmEN(LAT, mid, 37)[0] < 399800) lo = mid; else hi = mid; }
    const LON0 = lo, DLON = 10 / (111320 * Math.cos(LAT * Math.PI / 180));   // ~10 м на восток за такт 0,1 с
    const sent = { xmin: Infinity, xmax: -Infinity };
    let scenario = 'basic';
    const wss = new WebSocketServer({ host: '127.0.0.1', port: 9090 });
    wss.on('connection', ws => {
      let k = 0;
      const iv = setInterval(() => {
        const t = 1000 + 0.1 * k, hdr = { stamp: { sec: Math.floor(t), nanosec: Math.round((t % 1) * 1e9) }, frame_id: 'map' };
        const pub = (topic, msg) => { try { ws.send(JSON.stringify({ op: 'publish', topic, msg })); } catch (_) {} };
        pub('/result/velocity', { header: hdr, velocity: 10 });
        pub('/tram/estimator_status', { header: hdr, mode: 0, v: 10, s: k, d: 0, k_traction: 1, k_brake: 1, mu: 0.2, sigma_v: 0.1, sigma_s: 1, wheel_healthy: [true, true], slip: false, ambiguous: false, n_accepted: 2, n_rejected: 0, valid: true, frame_count: 2 * k, step_time_us: 500 });
        if (scenario !== 'basic') {
          const lat = LAT, lon = LON0 + DLON * k, [E, N] = utmEN(lat, lon, 37);
          // (а) перенос по точке (Autoware): координаты внутри квадрата 100 км; (б) непрерывно от 37UDB
          const x = scenario === 'wrap' ? ((E % 1e5) + 1e5) % 1e5 : E - 400000, y = scenario === 'wrap' ? N % 1e5 : N - 6100000;
          sent.xmin = Math.min(sent.xmin, x); sent.xmax = Math.max(sent.xmax, x);
          pub('/result/position', { header: hdr, pose: { pose: { position: { x, y, z: 150 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, covariance: [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] } });
          pub('/sensing/gnss/master/fix', { header: hdr, latitude: lat, longitude: lon, altitude: 150, status: { status: 2, service: 1 } });
        }
        k++;
      }, 100);
      ws.on('close', () => clearInterval(iv));
    });
    await new Promise(r => wss.on('listening', r));
    // правка поля адреса при подключении: страница не замирает (review: new URL в цикле кадра)
    {
      const page = await open('?mode=live&ros=ws://127.0.0.1:9090');
      await sleep(3500);
      const rows = () => page.evaluate(() => window.__tvRun.state().rows);
      const r0 = await rows();
      await page.evaluate(() => { const u = document.getElementById('rvUrl'); u.value = ''; u.dispatchEvent(new Event('input')); });
      await sleep(2500);
      const r1 = await rows();
      await page.evaluate(() => { const u = document.getElementById('rvUrl'); u.value = 'ws://нед'; u.dispatchEvent(new Event('input')); });
      await sleep(2500);
      const r2 = await rows(), tail = await page.evaluate(() => window.__tvRun.liveTail(1));
      await page.evaluate(() => window.__tvRun.setMode('replay'));
      await page.waitForFunction(() => window.__tvRun.state().n > 0, { timeout: 30000 });
      await page.evaluate(() => window.__tvRun.play(true, 60));
      const t0 = await page.evaluate(() => window.__tvRun.state().t);
      await sleep(2500);
      const t1 = await page.evaluate(() => window.__tvRun.state().t);
      check('живой режим: правка адреса при подключении не останавливает страницу', r1 > r0 + 10 && r2 > r1 + 10 && /на связи/.test(tail.pill) && t1 - t0 > 60,
        { rows: [r0, r1, r2], pill: tail.pill, host: tail.host, replay_dt_x60: +(t1 - t0).toFixed(1) });
      await page.close();
    }
    // переход границы квадратов 37U CB | DB (E = 400 км) в обоих соглашениях MGRS
    for (const sc of ['wrap', 'grid']) {
      scenario = sc; sent.xmin = Infinity; sent.xmax = -Infinity;
      const page = await open('?mode=live&ros=ws://127.0.0.1:9090');
      await sleep(6000);
      const tl = await page.evaluate(() => window.__tvRun.liveTail(5000));
      const xs = tl.x.filter(Number.isFinite), hs = tl.h.filter(Number.isFinite);
      let jump = 0; for (let i = 1; i < xs.length; i++) jump = Math.max(jump, Math.abs(xs[i] - xs[i - 1]));
      const hmax = hs.length ? Math.max(...hs) : NaN, span = xs.length ? Math.max(...xs) - Math.min(...xs) : 0;
      const crossed = sc === 'wrap' ? sent.xmax > 99000 && sent.xmin < 1000 : sent.xmin < 0 && sent.xmax > 0;
      check(`живой режим: MGRS ${sc === 'wrap' ? 'с переносом по точке' : 'от квадрата 37UDB'} — переход E = 400 км без скачка`, crossed && xs.length > 20 && span > 200 && jump < 50 && hs.length > 20 && hmax < 0.5,
        { sent_x: [+sent.xmin.toFixed(1), +sent.xmax.toFixed(1)], rows: xs.length, span_m: +span.toFixed(1), max_step_m: +jump.toFixed(2), pairs: hs.length, max_plan_err_m: +hmax.toFixed(3) });
      if (sc === 'wrap') await shot(page, 'live_mgrs_crossing.png');
      await page.close();
    }
    await new Promise(r => wss.close(r));
  }

  // ---------------- живой ROS 2 без моста: «нет связи», без исключений
  {
    const page = await open('?mode=live&ros=ws://127.0.0.1:9');
    await sleep(4000);
    const st = await page.evaluate(() => ({ pill: document.getElementById('rvLiveSt').textContent, over: document.getElementById('rvOver').hidden ? '' : document.getElementById('rvOver').textContent }));
    check('живой режим без моста: состояние «нет связи»', /нет связи/.test(st.pill) && /нет связи/.test(st.over), st);
    await shot(page, 'live_nolink.png');
    await page.close();
  }

  const ignorable = e => e.kind === 'console' && /WebSocket connection to 'ws:\/\/127\.0\.0\.1:9\/' failed/.test(e.msg);
  const errs = report.errors.filter(e => !ignorable(e));
  check('0 JS-исключений и ошибок консоли', errs.length === 0, errs.slice(0, 5));
  check('0 внешних загрузок (страница офлайн)', report.external_requests.length === 0, report.external_requests.slice(0, 5));
  report.ok = report.checks.every(c => c.ok);
  fs.writeFileSync(path.join(OUT, 'page_test.json'), JSON.stringify(report, null, 1));
  console.log(report.ok ? 'ВСЁ OK' : 'ЕСТЬ ОШИБКИ', '->', path.join(OUT, 'page_test.json'));
  await browser.close();
  process.exit(report.ok ? 0 : 1);
})().catch(e => { console.error(e); process.exit(2); });
