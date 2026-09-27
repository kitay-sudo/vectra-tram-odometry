// Проверка режима «Живой ROS 2» против настоящего rosbridge (headless Chromium).
//
// Оркестровка - simulator/test/live_test.sh (контейнер ROS: нода + rosbridge +
// ros2 bag play; контейнер браузера: этот скрипт). Фазы:
//   1) подключение, данные идут: строки буфера растут, частота ноды по frame_count
//      около 20 Гц, есть скорость, положение, статус, тележки, пары с GNSS;
//   2) скрипт пишет <out>/live_phase1.done, оркестратор останавливает ROS -
//      страница должна показать «нет связи» и повторять подключение;
//   3) <out>/live_phase2.done - ROS запущен снова: страница переподключается сама.
const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer-core');

const PAGE = process.argv[2] || '/sim/index.html';
const ROS = process.argv[3] || 'ws://sim-ros:9090';
const OUT = process.argv[4] || '/out';
const sleep = ms => new Promise(r => setTimeout(r, ms));

// Отдельное соединение с мостом (как чужой клиент с домена): подписка на /vehicle/front_bogie_velocity
// (разрешена), попытка опубликовать туда 777 км/ч (должна быть отвергнута), вызов сервиса ноды
// list_parameters (закрыт) и /rosapi/topics (контроль: вызов сервисов как механизм работает),
// подписка на /rosout (вне списка).
async function bridgeProbe(url) {
  const WebSocket = require('ws');
  const ws = new WebSocket(url);
  const r = { sub_front: 0, injected: 0, node_srv_ok: false, node_srv_reply: null, rosapi_ok: false, rosapi_topics: null, rosout: 0, status: [] };
  await new Promise((res, rej) => { ws.on('open', res); ws.on('error', rej); });
  ws.on('message', raw => {
    let m; try { m = JSON.parse(String(raw)); } catch (_) { return; }
    if (m.op === 'publish' && m.topic === '/vehicle/front_bogie_velocity') { r.sub_front++; if (Math.abs(m.msg.velocity - 777) < 1e-6) r.injected++; }
    if (m.op === 'publish' && m.topic === '/rosout') r.rosout++;
    if (m.op === 'service_response' && m.id === 'probe-node') { r.node_srv_reply = m.result; r.node_srv_ok = m.result === true; }
    if (m.op === 'service_response' && m.id === 'probe-rosapi') { r.rosapi_ok = m.result === true; r.rosapi_topics = m.values && m.values.topics ? m.values.topics.length : null; }
    if (m.op === 'status') r.status.push(String(m.msg).slice(0, 120));
  });
  const send = o => ws.send(JSON.stringify(o));
  send({ op: 'subscribe', id: 'probe-sub', topic: '/vehicle/front_bogie_velocity', type: 'tram_vehicle_msgs/msg/VelocitySensor' });
  send({ op: 'subscribe', id: 'probe-rosout', topic: '/rosout', type: 'rcl_interfaces/msg/Log' });
  send({ op: 'advertise', id: 'probe-adv', topic: '/vehicle/front_bogie_velocity', type: 'tram_vehicle_msgs/msg/VelocitySensor' });
  for (let i = 0; i < 20; i++) {
    const now = Date.now() / 1000;
    send({ op: 'publish', id: 'probe-pub', topic: '/vehicle/front_bogie_velocity', msg: { header: { stamp: { sec: Math.floor(now), nanosec: 0 }, frame_id: '' }, velocity: 777 } });
    await sleep(100);
  }
  send({ op: 'call_service', id: 'probe-node', service: '/tram_state_estimator/list_parameters', type: 'rcl_interfaces/srv/ListParameters', args: {} });
  send({ op: 'call_service', id: 'probe-rosapi', service: '/rosapi/topics', type: 'rosapi_msgs/srv/Topics', args: {} });
  await sleep(4000);
  ws.close();
  r.status = [...new Set(r.status)].slice(0, 5);
  return r;
}

(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  const browser = await puppeteer.launch({
    executablePath: process.env.PUPPETEER_EXECUTABLE_PATH || '/usr/bin/chromium-browser',
    headless: 'new', args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
  });
  const rep = { ros: ROS, checks: [], errors: [], timeline: [] };
  const check = (name, ok, info) => { rep.checks.push({ name, ok: !!ok, info }); console.log(`${ok ? 'OK  ' : 'FAIL'} ${name} - ${JSON.stringify(info)}`); };
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
  const t1 = Date.now(), rows1 = s.rows, fc1 = s.fc;
  await sleep(10000);
  s = await st();
  const dtw = (Date.now() - t1) / 1000, rowHz = (s.rows - rows1) / dtw;
  // частота ноды - средняя за 10 с по frame_count статуса (мгновенная на странице скачет под нагрузкой хоста)
  const nodeHz10 = fc1 != null && s.fc != null ? (s.fc - fc1) / dtw : s.nodeHz;
  const probe = await page.evaluate(() => {
    const D = window.__tvRun; const r = document.getElementById('rvStatus').textContent;
    return { status_has_mode: /Тяга|Выбег|Торможение|Стоянка|Смена режима|Срыв|Нет данных/.test(r) };
  });
  check('живой: подключение и поток', /на связи/.test(s.pill) && s.rows > 300, { pill: s.pill, rows: s.rows });
  check('живой: частота ноды ~20 Гц (по frame_count за 10 с)', nodeHz10 > 18 && nodeHz10 < 22, { nodeHz_10s: nodeHz10 && +nodeHz10.toFixed(2), page_nodeHz: s.nodeHz && +s.nodeHz.toFixed(2), rates: s.rates });
  check('живой: строки экрана ~10 Гц (троттлинг подписки 100 мс)', rowHz > 8 && rowHz < 12, { rowHz: +rowHz.toFixed(2) });
  check('живой: пары скорости с GNSS и метрики', s.pairsV > 20 && isFinite(s.m.mae), { pairs: s.pairsV, mae: s.m.mae, maeN: s.m.maeN, plan_m: s.m.h });
  check('живой: панель состояния заполнена', probe.status_has_mode, probe);
  // мост только для чтения (simulator/deploy/live_demo.sh): публиковать и звать сервисы ноды нельзя
  const sec = await bridgeProbe(ROS);
  check('мост только для чтения: подписка на топики страницы есть, публикация и сервисы ноды закрыты',
    sec.sub_front > 0 && sec.injected === 0 && !sec.node_srv_ok && sec.rosapi_ok && sec.rosout === 0, sec);
  await page.screenshot({ path: path.join(OUT, 'live_connected.png') });
  fs.writeFileSync(path.join(OUT, 'live_phase1.done'), String(Date.now()));

  // фаза 2: мост остановлен
  s = await waitFor(x => /нет связи/.test(x.pill) && /нет связи/.test(x.over), 60000, 'обрыв замечен');
  check('живой: обрыв - «нет связи» и повтор', /нет связи/.test(s.pill) && /нет связи/.test(s.over), { pill: s.pill, over: s.over });
  await page.screenshot({ path: path.join(OUT, 'live_down.png') });
  fs.writeFileSync(path.join(OUT, 'live_phase2.done'), String(Date.now()));

  // фаза 3: мост снова поднят - переподключение без действий пользователя
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
