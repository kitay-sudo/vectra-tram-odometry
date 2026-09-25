// Headless-прогон песочницы simulator/index.html в Node без браузера
// (перенесено из tools/audit/sim_headless.js; добавлены проверки ложного
// состояния S1 и новой метрики обнаружения срыва).
//
//   node simulator/test/sim_headless.js [minutes_of_free_run=10]   (SEED=n TRACE=1)
//
// Встроенный скрипт страницы исполняется с заглушками DOM/Canvas/Audio,
// кадры (requestAnimationFrame) подаются вручную с dt = 50 мс (как ограничено
// в frame()). Кнопки нажимаются через их обработчики. Ловим исключения и NaN,
// собираем таблицу метрик страницы (MET) — то, что увидит жюри.
// Страница НЕ изменяется: в копию скрипта в памяти добавляется одна строка,
// выставляющая внутреннее состояние наружу (globalThis.__sim).
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const ROOT = path.resolve(__dirname, '..', '..');
const html = fs.readFileSync(process.env.SIM_HTML || path.join(ROOT, 'simulator', 'index.html'), 'utf8');
const FREE_MIN = +(process.argv[2] || 10);

// ---------- fake DOM ----------
const byId = new Map();
const all = [];
const listeners = new Map(); // el -> {type: [fn]}
function on(el, type, fn) { if (!listeners.has(el)) listeners.set(el, {}); (listeners.get(el)[type] ||= []).push(fn); }
function off(el, type, fn) { const L = listeners.get(el); if (L && L[type]) L[type] = L[type].filter(f => f !== fn); }
function fire(el, type, ev = {}) {
  const L = listeners.get(el); if (!L || !L[type]) return 0;
  const e = Object.assign({ type, preventDefault() {}, target: el, currentTarget: el, clientX: 100, clientY: 50 }, ev);
  for (const f of L[type].slice()) f(e); return L[type].length;
}
const gradient = () => ({ addColorStop() {} });
function fakeCtx() {
  const base = { canvas: null, measureText: t => ({ width: String(t).length * 6 }), createLinearGradient: gradient, createRadialGradient: gradient, getImageData: () => ({ data: new Uint8ClampedArray(4) }) };
  return new Proxy(base, { get(t, p) { if (p in t) return t[p]; return () => {}; }, set(t, p, v) { t[p] = v; return true; } });
}
function makeEl(tag, attrs) {
  const dataset = {};
  for (const [k, v] of Object.entries(attrs)) if (k.startsWith('data-')) dataset[k.slice(5).replace(/-([a-z])/g, (_, c) => c.toUpperCase())] = v;
  let inner = '';
  const el = {
    tagName: tag.toUpperCase(), attrs: { ...attrs }, dataset, style: {}, textContent: '', title: '', value: attrs.value || '', max: attrs.max || '',
    width: 300, height: 150, clientWidth: 300, clientHeight: 150, offsetHeight: 100, offsetWidth: 300,
    classList: { add() {}, remove() {}, contains: () => false, toggle() {} },
    setAttribute(k, v) { this.attrs[k] = String(v); if (k.startsWith('data-')) dataset[k.slice(5)] = String(v); },
    getAttribute(k) { return k in this.attrs ? this.attrs[k] : null; },
    addEventListener(t, f) { on(el, t, f); }, removeEventListener(t, f) { off(el, t, f); },
    getContext() { const c = fakeCtx(); c.canvas = el; return c; },
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 300, height: 150, right: 300, bottom: 150 }),
    focus() {}, closest() { return null; },
    get innerHTML() { return inner; },
    set innerHTML(h) { inner = String(h); registerFrom(inner); },
  };
  all.push(el);
  if (attrs.id) byId.set(attrs.id, el);
  return el;
}
function registerFrom(fragment) {
  const re = /<([a-zA-Z][a-zA-Z0-9]*)\b([^>]*)>/g; let m;
  while ((m = re.exec(fragment))) {
    const attrs = {}; const ar = /([a-zA-Z_:][-a-zA-Z0-9_:.]*)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s"'>]+)))?/g; let a;
    while ((a = ar.exec(m[2]))) attrs[a[1]] = a[2] ?? a[3] ?? a[4] ?? '';
    if (attrs.id || Object.keys(attrs).some(k => k.startsWith('data-'))) makeEl(m[1], attrs);
  }
}
const body = html.slice(html.indexOf('<body'), html.indexOf('<script>', html.indexOf('<body')));
registerFrom(body);
const docEl = makeEl('html', {});
const document = {
  documentElement: docEl, hidden: false,
  getElementById: id => byId.get(id) || null,
  querySelectorAll(sel) {
    const m = /^\[([a-z-]+)\]$/.exec(sel); if (!m) throw new Error('unsupported selector ' + sel);
    return all.filter(e => m[1] in e.attrs);
  },
};
docEl.dataset = {};

// ---------- window, time, rAF ----------
let nowMs = 0, rafQ = [];
const winListeners = {};
const errors = [];
let rs = +(process.env.SEED || 0) >>> 0;
const seeded = () => { rs = (rs + 0x6D2B79F5) | 0; let t = Math.imul(rs ^ (rs >>> 15), 1 | rs); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const SMath = Object.create(Math); if (process.env.SEED) SMath.random = seeded;
const sandbox = {
  document, console, Math: SMath, JSON, Date, Array, Object, Number, String, Boolean, Symbol, Map, Set, Promise, Infinity, NaN,
  Float32Array, Float64Array, Uint8ClampedArray, Int32Array, isFinite, isNaN, parseFloat, parseInt, Error,
  performance: { now: () => nowMs },
  requestAnimationFrame: f => { rafQ.push(f); return rafQ.length; },
  setInterval: () => 0, clearInterval: () => {}, setTimeout: () => 0, clearTimeout: () => {},
  matchMedia: () => ({ matches: false, addEventListener() {} }),
  addEventListener: (t, f) => { (winListeners[t] ||= []).push(f); },
  removeEventListener: (t, f) => { if (winListeners[t]) winListeners[t] = winListeners[t].filter(x => x !== f); },
  getComputedStyle: () => ({ getPropertyValue: k => (k === '--fg' ? '#000000' : 'rgba(0,0,0,.12)') }),
  localStorage: { getItem: () => null, setItem() {} },
  innerWidth: 1440, innerHeight: 900, devicePixelRatio: 1,
  Path2D: class { moveTo() {} lineTo() {} quadraticCurveTo() {} closePath() {} bezierCurveTo() {} arc() {} rect() {} },
};
sandbox.window = sandbox; sandbox.globalThis = sandbox;
vm.createContext(sandbox);

// ---------- page script with one exposing line ----------
// скрипт песочницы — блок <script>, где объявлен TramEst (режимы прогона и ROS в другом блоке)
const s0 = html.lastIndexOf('<script>', html.indexOf('const TramEst')) + 8, s1 = html.indexOf('</script>', s0);
let code = html.slice(s0, s1);
const tail = code.lastIndexOf('})();');
code = code.slice(0, tail) +
  `globalThis.__sim = { get est() { return est; }, MET, COND, slipDelays, REC, get v() { return v; }, get s() { return s; }, get simT() { return simT; },
    get plant() { return plant; }, get estimator() { return estimator; }, get lastOut() { return lastOut; }, get mTrue() { return mTrue; },
    get slipMissed() { return typeof slipMissed !== 'undefined' ? slipMissed : { n: null }; }, get presT() { return presT; }, get notch() { return notch; }, get running() { return running; }, get dwell() { return dwell; }, nextStop, get handle() { return handle; }, gradeAt, get zones() { return zones; }, get D() { return D; }, openInfo, closeInfo, renderMetrics };\n` + code.slice(tail);
try { vm.runInContext(code, sandbox, { filename: 'simulator/index.html#inline' }); }
catch (e) { errors.push(['init', e.stack]); }
const S = sandbox.__sim;
// REC_INPUTS=1: входы оценщика на каждом шаге (notch, 8 показаний, истина v, s) — для прогона Python-ядра
const INPUTS = [];
if (process.env.REC_INPUTS) {
  const es = S.estimator, orig = es.step;
  es.step = function (n, m) { INPUTS.push([n, Array.from(m, x => +x.toFixed(6)), +S.plant.v.toFixed(5), +S.plant.s.toFixed(4)]); return orig.call(this, n, m); };
}

function frames(n, label, check) {
  for (let i = 0; i < n; i++) {
    nowMs += 50;
    const q = rafQ; rafQ = [];
    for (const f of q) {
      try { f(nowMs); } catch (e) { errors.push([label, e.stack.split('\n').slice(0, 3).join(' | ')]); }
    }
    if (check) check();
  }
}
const nanSeen = new Set();
const TRACE = [];
let traceN = 0;
let falseT = 0, falseMaxT = 0, spinStillMax = 0, runT = 0, confT = 0, confRunT = 0, confMaxT = 0;
const nanCheck = () => {
  // S1: оценка > 20 км/ч, пока вагон стоит (< 1 км/ч); буксование на месте
  const vt = S.v * 3.6, ve = S.est.v;
  if (ve > 20 && vt < 1) { runT += 0.05 * 4; falseT += 0.05 * 4; falseMaxT = Math.max(falseMaxT, runT); } else runT = 0;
  // S1 в строгом смысле: ложная скорость при «достоверно» и узкой σ (< 2 км/ч). При
  // неоднозначности «стоим или скользим» скорость держится намеренно, но valid = false и σ широкая.
  const o = S.lastOut, conf = o && o.valid && o.sigma_v * 3.6 < 2;
  if (ve > 20 && vt < 1 && conf) { confRunT += 0.05 * 4; confT += 0.05 * 4; confMaxT = Math.max(confMaxT, confRunT); } else confRunT = 0;
  if (S.plant.v < 0.1) for (let i = 0; i < 8; i++) spinStillMax = Math.max(spinStillMax, Math.abs(S.plant.w[i] * 0.35 * 3.6));
  if (process.env.TRACE && ++traceN % 20 === 0) { const p = S.plant, es = S.estimator; const z = S.zones.wx; TRACE.push({ t: +S.simT.toFixed(1), v: +(p.v * 3.6).toFixed(2), w: Array.from(p.w, x => +(x * 0.35 * 3.6).toFixed(1)), asr: Array.from(p.asr, x => +x.toFixed(2)), grade_pm: +(S.gradeAt(S.s) * 1000).toFixed(1), wx: z && S.D > z.a && S.D < z.b ? z.kind : '-', notch: S.notch, healthy: es.healthy.map(h => +h).join(''), est: +(S.est.v).toFixed(1), mode: S.est.st, mass: +S.mTrue.toFixed(1) }); }
  const e = S.est;
  for (const k of ['v', 'lo', 'hi', 's', 'sig', 'kt', 'kb', 'mu', 'd']) if (!Number.isFinite(e[k])) nanSeen.add(k);
  if (!Number.isFinite(S.plant.v)) nanSeen.add('plant.v');
};
const click = id => { const el = document.getElementById(id); if (!el) throw new Error('no #' + id); return fire(el, 'click'); };
const clickData = (attr, val) => { const el = all.find(e => e.attrs[attr] === val); if (!el) throw new Error(`no [${attr}=${val}]`); return fire(el, 'click'); };
const key = k => (winListeners.keydown || []).forEach(f => f({ key: k, target: { closest: () => null }, preventDefault() {} }));
const log = [];
const snap = label => { const e = S.est, o = S.lastOut || {}; log.push({ label, simT: +S.simT.toFixed(1), v_true_kmh: +(S.v * 3.6).toFixed(2), v_est_kmh: +e.v.toFixed(2), wheel_kmh: +(e.sensor * 3.6).toFixed(2), sigma_kmh: +((e.hi - e.lo) / 4).toFixed(2), mode: e.st, n_acc: o.n_accepted, n_rej: o.n_rejected, mu: +e.mu.toFixed(3), k_t: +e.kt.toFixed(3), mass_t: +S.mTrue.toFixed(1), notch: S.notch, running: S.running, stop_d_m: +S.nextStop().d.toFixed(1), next: S.nextStop().q.n }); };

const t0 = process.hrtime.bigint();
frames(40, 'warmup', nanCheck); snap('старт');
// A. сценарий 60 с (кнопка «Сценарий 60 с»)
click('presBtn');
for (let k = 0; k < 13; k++) { frames(100, 'presentation', nanCheck); snap(`сценарий, ${(k + 1) * 5} с`); }
// B. все модальные окна (ROS-схема, бенчмарк, метрики, запись)
for (const m of ['model', 'ros', 'metrics']) { clickData('data-open', m); frames(60, 'modal ' + m, nanCheck); }
click('recBtn'); frames(60, 'modal replay', nanCheck); S.closeInfo();
// C. ручное управление, клавиши, час пик, застройка, тема
clickData('data-drive', 'manual'); key('w'); frames(200, 'manual traction', nanCheck); snap('ручное: тяга');
key('ArrowRight'); frames(100, 'manual coast', nanCheck); key('s'); frames(300, 'manual brake', nanCheck); snap('ручное: тормоз');
clickData('data-drive', 'auto'); click('swRush'); click('swUrban'); click('themeBtn'); frames(400, 'rush+urban', nanCheck); snap('час пик + застройка');
key(' '); frames(200, 'stopped', nanCheck); snap('Стоп (пробел)'); key(' ');
// D. отказ оси на снегу, затем все оси, затем починка
clickData('data-weather', 'snow'); clickData('data-sens', 'axle'); frames(400, 'snow+axle', nanCheck); snap('снег + отказ оси 1');
clickData('data-sens', 'all'); frames(400, 'all sensors', nanCheck); snap('отказ всех осей');
clickData('data-sens', 'ok'); frames(400, 'repaired', nanCheck); snap('датчики починены');
// E. свободная езда: погода по кругу
const perW = Math.max(1, Math.round(FREE_MIN * 60 / 3 / 0.05));
for (const w of ['rain', 'clear', 'snow']) { clickData('data-weather', w); frames(perW, 'free ' + w, nanCheck); snap('свободно: ' + w); }
const wallS = Number(process.hrtime.bigint() - t0) / 1e9;

// ---------- отчёт ----------
S.renderMetrics();
const MET = S.MET;
const rows = S.COND.map(c => { const m = MET[c]; return m.t < 0.5 ? { cond: c, t_s: 0 } : {
  cond: c, t_s: +m.t.toFixed(1), rmse_model_kmh: +Math.sqrt(m.sqM / m.t).toFixed(2), rmse_wheel_kmh: +Math.sqrt(m.sqB / m.t).toFixed(2),
  max_model_kmh: +m.maxM.toFixed(2), seg_err_model_m: m.segM.length ? +(m.segM.reduce((a, b) => a + b, 0) / m.segM.length).toFixed(1) : null,
  seg_err_wheel_m: m.segB.length ? +(m.segB.reduce((a, b) => a + b, 0) / m.segB.length).toFixed(1) : null, in_2sigma_pct: +(100 * m.inInt / m.t).toFixed(1) }; });
const sd = S.slipDelays.slice().sort((a, b) => a - b);
if (process.env.REC_INPUTS) fs.writeFileSync(path.join(ROOT, 'out', 'sim_wp17', `sim_inputs_seed${process.env.SEED || 'rnd'}.json`), JSON.stringify(INPUTS));
if (process.env.TRACE) fs.writeFileSync(path.join(ROOT, 'out', 'sim_wp17', `sim_trace_seed${process.env.SEED || 'rnd'}.json`), JSON.stringify(TRACE));
const res = { wall_s: +wallS.toFixed(1), sim_time_s: +S.simT.toFixed(1), frames_errors: errors.length, errors: errors.slice(0, 20), nan_fields: [...nanSeen],
  slip_delay_ms_median: sd.length ? Math.round(sd[sd.length >> 1]) : null, slip_events: sd.length, slip_missed: S.slipMissed.n, slip_short: S.slipMissed.short,
  false_moving_s: +falseT.toFixed(1), false_moving_max_run_s: +falseMaxT.toFixed(1), false_confident_s: +confT.toFixed(1), false_confident_max_run_s: +confMaxT.toFixed(1), wheel_spin_at_standstill_max_kmh: +spinStillMax.toFixed(1),
  metrics: rows, timeline: log };
const out = path.join(ROOT, 'out', 'sim_wp17', `sim_headless${process.env.OUTTAG ? '_' + process.env.OUTTAG : ''}${process.env.SEED ? '_seed' + process.env.SEED : ''}.json`);
fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out, JSON.stringify(res, null, 1));
console.log(`прогон: ${res.sim_time_s} с модели за ${res.wall_s} с, ошибок ${errors.length}, NaN: ${res.nan_fields.join(',') || 'нет'}`);
if (errors.length) console.log(errors.slice(0, 5));
console.table(rows);
console.table(log);
console.log('обнаружение срыва: медиана, мс', res.slip_delay_ms_median, 'обнаружено', res.slip_events, 'пропущено', res.slip_missed);
console.log('S1: оценка > 20 км/ч при стоящем вагоне, с:', res.false_moving_s, '(макс. подряд', res.false_moving_max_run_s, 'с); из них «достоверно» и σ < 2 км/ч:', res.false_confident_s, 'с (подряд', res.false_confident_max_run_s, 'с); буксование на месте, макс. км/ч:', res.wheel_spin_at_standstill_max_kmh);
console.log('->', path.relative(ROOT, out));
