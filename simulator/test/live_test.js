// Проверка режима «Живой ROS 2» против настоящего rosbridge (headless Chromium).
//
// Оркестровка — simulator/test/live_test.sh (контейнер ROS: нода + rosbridge +
// ros2 bag play; контейнер браузера: этот скрипт). Фазы:
//   1) подключение, данные идут: строки буфера растут, частота ноды по frame_count
//      около 20 Гц, есть скорость, положение, статус, тележки, пары с GNSS;
//   2) скрипт пишет <out>/live_phase1.done, оркестратор останавливает ROS —
//      страница должна показать «нет связи» и повторять подключение;
//   3) <out>/live_phase2.done — ROS запущен снова: страница переподключается сама.
const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer-core');

const PAGE = process.argv[2] || '/sim/index.html';
const ROS = process.argv[3] || 'ws://sim-ros:9090';
const OUT = process.argv[4] || '/out';
const sleep = ms => new Promise(r => setTimeout(r, ms));

(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  const browser = await puppeteer.launch({
    executablePath: process.env.PUPPETEER_EXECUTABLE_PATH || '/usr/bin/chromium-browser',
    headless: 'new', args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
  });
  const rep = { ros: ROS, checks: [], errors: [], timeline: [] };
  const check = (name, ok, info) => { rep.checks.push({ name, ok: !!ok, info }); console.log(`${ok ? 'OK  ' : 'FAIL'} ${name} — ${JSON.stringify(info)}`); };
  const page = await browser.newPage();
  await page.setViewport({ width: 1440, height: 900 });
  page.on('pageerror', e => rep.errors.push(String(e.message || e)));
  page.on('console', m => { if (m.type() === 'error' && !/WebSocket connection to .* failed/.test(m.text())) rep.errors.push(m.text()); });
  await page.goto(`file://${PAGE}?mode=live&ros=${encodeURIComponent(ROS)}`, { waitUntil: 'load' });
  const st = () => page.evaluate(() => ({ ...window.__tvRun.state(), pill: document.getElementById('rvLiveSt').textContent, rates: document.getElementById('rvRates').textContent,
    over: document.getElementById('rvOver').hidden ? '' : document.getElementById('rvOver').textContent, m: window.__tvRun.metrics() }));
  const waitFor = async (pred, ms, label) => {
    const t0 = Date.now(); let s;
    while (Date.now() - t0 < ms) { s = await st(); if (pred(s)) { rep.timeline.push({ label, after_s: (Date.now() - t0) / 1000, pill: s.pill }); return s; } await sleep(500); }
    rep.timeline.push({ label: label + ' (таймаут)', pill: s && s.pill }); return s;
  };

  // фаза 1: данные идут
  let s = await waitFor(x => x.rows > 300 && x.nodeHz, 180000, 'данные идут');
  const t1 = Date.now(), rows1 = s.rows;
  await sleep(10000);
  s = await st();
  const rowHz = (s.rows - rows1) / ((Date.now() - t1) / 1000);
  const probe = await page.evaluate(() => {
    const D = window.__tvRun; const r = document.getElementById('rvStatus').textContent;
    return { status_has_mode: /Тяга|Выбег|Торможение|Стоянка|Смена режима|Срыв|Нет данных/.test(r) };
  });
  check('живой: подключение и поток', /на связи/.test(s.pill) && s.rows > 300, { pill: s.pill, rows: s.rows });
  check('живой: частота ноды ~20 Гц (по frame_count)', s.nodeHz > 18 && s.nodeHz < 22, { nodeHz: s.nodeHz && +s.nodeHz.toFixed(2), rates: s.rates });
  check('живой: строки экрана ~10 Гц (троттлинг подписки 100 мс)', rowHz > 8 && rowHz < 12, { rowHz: +rowHz.toFixed(2) });
  check('живой: пары скорости с GNSS и метрики', s.pairsV > 20 && isFinite(s.m.mae), { pairs: s.pairsV, mae: s.m.mae, maeN: s.m.maeN, plan_m: s.m.h });
  check('живой: панель состояния заполнена', probe.status_has_mode, probe);
  await page.screenshot({ path: path.join(OUT, 'live_connected.png') });
  fs.writeFileSync(path.join(OUT, 'live_phase1.done'), String(Date.now()));

  // фаза 2: мост остановлен
  s = await waitFor(x => /нет связи/.test(x.pill), 60000, 'обрыв замечен');
  check('живой: обрыв — «нет связи» и повтор', /нет связи/.test(s.pill) && /нет связи/.test(s.over), { pill: s.pill, over: s.over });
  await page.screenshot({ path: path.join(OUT, 'live_down.png') });
  fs.writeFileSync(path.join(OUT, 'live_phase2.done'), String(Date.now()));

  // фаза 3: мост снова поднят — переподключение без действий пользователя
  const rowsDown = s.rows;
  s = await waitFor(x => /на связи/.test(x.pill) && x.rows > rowsDown + 50, 240000, 'переподключение');
  check('живой: переподключение с отсрочкой', /на связи/.test(s.pill) && s.rows > rowsDown + 50, { pill: s.pill, rows: s.rows, rowsDown });
  await page.screenshot({ path: path.join(OUT, 'live_reconnected.png') });

  check('живой: 0 JS-исключений и ошибок консоли', rep.errors.length === 0, rep.errors.slice(0, 5));
  rep.ok = rep.checks.every(c => c.ok);
  fs.writeFileSync(path.join(OUT, 'live_test.json'), JSON.stringify(rep, null, 1));
  console.log(rep.ok ? 'ВСЁ OK' : 'ЕСТЬ ОШИБКИ');
  await browser.close();
  process.exit(rep.ok ? 0 : 1);
})().catch(e => { console.error(e); process.exit(2); });
