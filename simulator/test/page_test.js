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
// страницы в конце прогона совпадают с итогом экспортёра; живой режим без
// моста показывает «нет связи». Скриншоты — в <out>.
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
    const ok = same(m.mae, s.v_mae, 4) && same(m.maeN, s.naive_v_mae, 4) && same(m.drift, s.drift_pct, 3) && same(m.driftN, s.naive_drift_pct, 3) && same(m.h, s.p2d_mean, 2) && same(m.cov, s.cov2s_v, 3);
    report.replays[e.key] = { played_s_in_3s_x60: +(t1 - t0).toFixed(1), cmp };
    check(`прогон ${e.key}: проигрывается (×60)`, t1 - t0 > 60, { dt: +(t1 - t0).toFixed(1) });
    check(`прогон ${e.key}: метрики страницы = итог экспортёра`, ok, Object.fromEntries(Object.entries(cmp).map(([k, [a, b]]) => [k, [a === null ? null : +a.toFixed(5), b === null ? null : +b.toFixed(5)]])));
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
