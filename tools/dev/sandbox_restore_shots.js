// Снимки «было / стало» песочницы: исходная страница (110a5e0) и восстановленная (ядро пакета).
//   git worktree add <old> 110a5e0   # исходная страница, только чтение
//   docker run --rm --network none -v <old>/simulator:/old:ro -v <repo>/simulator:/new:ro \
//     -v <repo>/tools/dev:/dev-tools:ro -v <repo>/out/restore:/out \
//     vectra/tram:sim node /dev-tools/sandbox_restore_shots.js /old/index.html /new/index.html /out
// Для каждого состояния: old_<имя>.png, new_<имя>.png и рядом — sbs_<имя>.png.
const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer-core');
const [,, OLD, NEW, OUT, ...ONLY] = process.argv;   // ONLY — имена состояний (по умолчанию все)
const sleep = ms => new Promise(r => setTimeout(r, ms));
const DESK = { width: 1440, height: 900 }, MOB = { width: 390, height: 844, isMobile: true, hasTouch: true };
const click = sel => `document.querySelector('${sel}').click()`;
// состояние: [имя, вид, действия старой, действия новой, подпись]
const STATES = [
  ['start', DESK, [[800]], [[800]], 'старт: сразу после открытия'],
  ['moving_dry', DESK, [[14000]], [[14000]], 'едет, сухо (старая: автопоездка по кольцу; новая: свободная поездка по pathgraph)'],
  ['rain_slip', DESK, [[4000], [click('[data-weather="rain"]')], [16000]], [["__tvSandbox.load('rain_spin', { fade: false }); document.querySelector('[data-sbsp=\"1\"]').click(); __tvSandbox.play(false); __tvSandbox.run(7.3); __tvSandbox.play(true)"], [150]], 'дождь и срыв (новая: сценарий «Дождь + полная тяга», передняя буксует и отброшена)'],
  ['ice_skid', DESK, [[4000], [click('[data-weather="snow"]')], [16000]], [["__tvSandbox.load('ice_skid', { fade: false }); document.querySelector('[data-sbsp=\"1\"]').click(); __tvSandbox.play(false); __tvSandbox.run(54.8, true); __tvSandbox.play(true)"], [900]], 'снег / наледь (новая: юз при торможении, колёса встали — «стоим или скользим?»)'],
  ['night', DESK, [[1500], [click('#themeBtn')], [9000]], [[1500], [click('#themeBtn')], [9000]], 'ночь'],
  ['sensor_fault', DESK, [[6000], [click('[data-sens="axle"]')], [5000]], [[6000], [click('[data-sens="front"]')], [5000]], 'отказ датчика (старая: «Ось 1», новая: «Передняя» тележка)'],
  ['urban', DESK, [[5000], [click('#swUrban')], [12000]], [[5000], [click('#swUrban')], [12000]], 'застройка: спутники пропадают'],
  ['stop', DESK, [[12000], [click('#bRun')], [2500]], [[12000], [click('#bRun')], [2500]], '«Стоп» в свободной поездке: вагон тормозит до остановки, время идёт (2,5 с после нажатия)'],
  ['rain_user', DESK, [[8000], [click('[data-weather="rain"]')], [900]], [[8000], [click('[data-sbwx="rain"]')], [900]], 'нажат «Дождь»: мокрый участок начинается в 100 м впереди — «Впереди: мокрые рельсы через N м»'],
  ['snow_user', DESK, [[8000], [click('[data-weather="snow"]')], [5000]], [[8000], [click('[data-sbwx="ice"]')], [5000]], 'нажат «Снег»: вагон въехал на наледь, идёт снег'],
  ['urban_ahead', DESK, [[8000], [click('#swUrban')], [900]], [[8000], [click('#swUrban')], [900]], 'нажата «Застройка»: дома и пропажа спутников — в 100 м впереди'],
  ['laptop_1366', { width: 1366, height: 768 }, [[9000]], [[9000]], 'ноутбук 1366×768: карта открыта, панель и график как в исходной'],
  ['laptop_1280', { width: 1280, height: 720 }, [[9000]], [[9000]], '1280×720 (экран 1080p при масштабе 150 %, проектор): карта открыта, план карты ниже'],
  ['view_model', DESK, [[1500], [click('[data-open="model"]')], [5000]], [[1500], [click('[data-open="model"]')], [5000]], 'окно «Как устроена модель» (печатается)'],
  ['view_ros', DESK, [[5000], [click('[data-open="ros"]')], [4000]], [[5000], [click('[data-open="ros"]')], [4000]], 'окно «Пакет ROS 2»: топики, схема ноды, время шага'],
  ['view_metrics', DESK, [[12000], [click('[data-open="metrics"]')], [1500]], [[12000], [click('[data-open="metrics"]')], [1500]], 'окно «Метрики»'],
  ['view_tour', DESK, [[1000], [click('#presBtn')], [12000]], [[1000], [click('#presBtn')], [12000]], 'показ 60 с (часы): шаг 2'],
  ['view_record', DESK, [[12000], ["document.getElementById('recBtn').click()"], [1500]], [[12000], [click('#recBtn')], [1500]], 'окно «Запись поездки»'],
  ['new_scenarios', DESK, null, [[2000], [click('#scBtn')], [600]], 'новое: список сценариев в панели управления (сверху — что за сценарий и скорость времени)'],
  ['new_pause', DESK, null, [["__tvSandbox.load('dry', { fade: false })"], [3000], [click('#bRun')], [800]], 'новое: в сценарии кнопка — «Пауза / Дальше» (время сценария расписано)'],
  ['new_think', DESK, null, [["__tvSandbox.load('front_fail', { fade: false }); __tvSandbox.play(false); __tvSandbox.run(40, true); __tvSandbox.think(true); __tvSandbox.play(true)"], [1200]], 'новое: строка «Модель думает» над графиком, открыто «подробнее»'],
  ['new_help', DESK, null, [[1000], [click('#helpBtn')], [600]], 'новое: «Как читать экран»'],
  ['m_start', MOB, [[800]], [[800]], 'телефон: старт'],
  ['m_moving_dry', MOB, [[12000]], [[12000]], 'телефон: едет, сухо'],
  ['m_rain', MOB, [[4000], [click('[data-weather="rain"]')], [14000]], [["__tvSandbox.load('rain_spin', { fade: false }); document.querySelector('[data-sbsp=\"1\"]').click(); __tvSandbox.play(false); __tvSandbox.run(7.3); __tvSandbox.play(true)"], [150]], 'телефон: дождь'],
  ['m_night', MOB, [[1500], [click('#themeBtn')], [8000]], [[1500], [click('#themeBtn')], [8000]], 'телефон: ночь'],
  ['m_view_model', MOB, [[1500], [click('[data-open="model"]')], [5000]], [[1500], [click('[data-open="model"]')], [5000]], 'телефон: «Как устроена модель»'],
  ['m_view_metrics', MOB, [[10000], [click('[data-open="metrics"]')], [1500]], [[10000], [click('[data-open="metrics"]')], [1500]], 'телефон: «Метрики»'],
  ['m_think', MOB, null, [["__tvSandbox.load('front_fail', { fade: false }); __tvSandbox.play(false); __tvSandbox.run(40, true); __tvSandbox.think(true); __tvSandbox.play(true); document.getElementById('dash').scrollTop = 420"], [1200]], 'телефон, новое: панель модели (табло прокручивается)'],
];

(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  const b = await puppeteer.launch({ executablePath: '/usr/bin/chromium-browser', headless: 'new', args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage', '--font-render-hinting=none'] });
  const report = { states: [], errors: [] };
  async function run(file, vp, acts, name, tag) {
    const p = await b.newPage();
    p.on('pageerror', e => report.errors.push({ name, tag, msg: String(e.message || e) }));
    p.on('console', m => { if (m.type() === 'error') report.errors.push({ name, tag, msg: m.text() }); });
    await p.setViewport(vp);
    await p.evaluateOnNewDocument(() => { try { localStorage.clear(); } catch (_) {} });
    await p.goto('file://' + file + '?mode=sandbox', { waitUntil: 'load' });
    for (const a of acts) { if (typeof a[0] === 'number') await sleep(a[0]); else await p.evaluate(a[0]); }
    const f = path.join(OUT, `${tag}_${name}.png`);
    await p.screenshot({ path: f });
    await p.close();
    return f;
  }
  for (const [name, vp, oa, na, label] of STATES.filter(x => !ONLY.length || ONLY.includes(x[0]))) {
    const fo = oa ? await run(OLD, vp, oa, name, 'old') : null;
    const fn = await run(NEW, vp, na, name, 'new');
    // рядом: слева — было, справа — стало
    const img = f => `data:image/png;base64,${fs.readFileSync(f).toString('base64')}`;
    const w = vp.width < 700 ? 390 : 960, cell = f => `<img src="${img(f)}" style="width:${w}px;display:block;border:1px solid #ccc">`;
    const html = `<html><body style="margin:0;background:#fff;font:15px system-ui,sans-serif;color:#111"><div style="padding:10px 14px 4px;font-weight:600">${label}</div>
      <div style="display:flex;gap:14px;padding:6px 14px 14px">${fo ? `<div><div style="padding:0 0 4px;opacity:.7">было — 110a5e0 (JS-порт ядра от 25.09, лист по умолчанию: 4 оси)</div>${cell(fo)}</div>` : ''}<div><div style="padding:0 0 4px;opacity:.7">стало — ядро пакета, лист жюри, 2 тележки</div>${cell(fn)}</div></div></body></html>`;
    const p = await b.newPage();
    await p.setViewport({ width: (fo ? 2 : 1) * (w + 14) + 16, height: 300 });
    await p.setContent(html, { waitUntil: 'load' });
    await p.screenshot({ path: path.join(OUT, `sbs_${name}.png`), fullPage: true });
    await p.close();
    report.states.push({ name, label, old: !!fo });
    console.log('ok', name);
  }
  if (!ONLY.length) fs.writeFileSync(path.join(OUT, 'sbs.json'), JSON.stringify(report, null, 1));
  console.log('ошибок страниц:', report.errors.length, JSON.stringify(report.errors.slice(0, 5)));
  await b.close();
})().catch(e => { console.error(e); process.exit(2); });
