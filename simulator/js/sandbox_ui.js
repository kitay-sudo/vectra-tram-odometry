// ---- Песочница: сцена, табло и панели в исходном стиле страницы (110a5e0) ----
// Числа — TramSandbox.Engine: имитатор (приближение) + НАСТОЯЩЕЕ ядро пакета через связку
// (JS-порт estimator_core.py и Runner, сверен с Python) с листом жюри. Сцена — js/scene.js.
// Сценарии, «Что сейчас думает модель» и справка встроены в исходные панели. Метрики сценария
// считает движок (тем же кодом, что таблица docs/SANDBOX.md — simulator/test/sandbox_report.js).
(() => {
  const $ = id => document.getElementById(id);
  const SB = TramSandbox, KMH = 3.6, G = 9.81;
  const SHEET = window.TV_SHEET, PORT = TramEst.PORT, HALF = TramScene.HALF;
  const store = { get: k => { try { return localStorage.getItem(k); } catch (_) { return null; } }, set: (k, v) => { try { localStorage.setItem(k, v); } catch (_) {} } };
  const qs = new URLSearchParams(location.search);
  const C_OK = '#0E9AA7', C_NAIVE = '#E89B0C', C_BOG = '#8a7fd0', C_REAR = '#b07fd0', C_BAD = '#D6443A', C_WET = '#5AA9D6';
  const FONT11 = '11px system-ui, "Segoe UI", Roboto, sans-serif';

  // ---------------------------------------------------------------- режимы модели
  const ST_COL = { TRANSITION: '#7E8B98', COAST: '#0E9AA7', TRACTION: '#9AA7B4', BRAKE: '#6B7785', SLIP: '#5AA9D6', STANDSTILL: '#3C4450', DEGRADED: '#D6443A' };
  const ST_SUB = { TRANSITION: 'привод набирает или сбрасывает момент', COAST: 'колёса дают точную скорость', TRACTION: 'колесо может только завышать', BRAKE: 'колесо может только занижать', SLIP: 'показания сорвавшихся колёс отвергнуты', STANDSTILL: 'вагон стоит, скорость ноль', DEGRADED: 'нечему верить, скорость по модели' };
  const ICO = {
    TRANSITION: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 9h13l-3-3M20 15H7l3 3"/></svg>',
    COAST: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 12h13"/><path d="m13 7 5 5-5 5"/></svg>',
    TRACTION: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M13 3 5 14h6l-1 7 8-11h-6z"/></svg>',
    BRAKE: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="12" cy="12" r="8"/><path d="M8 12h8"/></svg>',
    SLIP: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 9c2-2 4-2 6 0s4 2 6 0 4-2 6 0"/><path d="M3 15c2-2 4-2 6 0s4 2 6 0 4-2 6 0"/></svg>',
    STANDSTILL: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><rect x="4" y="4" width="16" height="16" rx="3"/><path d="M10 16V8h3a2.5 2.5 0 0 1 0 5h-3"/></svg>',
    DEGRADED: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M12 4 2.5 20h19z"/><path d="M12 10v4"/><path d="M12 17h.01"/></svg>',
  };
  window.__tvIcons = ICO;                 // значки режимов — общие с экраном прогонов
  const modeCol = m => ST_COL[m] === '#3C4450' ? 'var(--fg)' : (ST_COL[m] || 'var(--fg)');
  const BOG_COL = { ok: C_OK, agree: C_OK, spin: C_NAIVE, skid: C_NAIVE, jump: C_NAIVE, forced: C_NAIVE, held: C_NAIVE, gate: C_NAIVE, slipall: C_NAIVE, stuck: C_BAD, dead: C_BAD, stale: C_BAD, frozen: C_BAD, wait: '' };

  // ---------------------------------------------------------------- подпись порта ядра
  // TramEst.PORT — ядро пакета, с которым сверен JS-порт. Экран прогонов читает replays/index.js:
  // если прогоны посчитаны другим ядром (core_sha1), ядро менялось после сверки, и песочница
  // честно подписывается «упрощённое ядро».
  function portClaim(kind) {
    const stale = window.TV_PORT_STALE || (SHEET && SHEET.core_sha1 !== PORT.core_sha1 ? SHEET.core_sha1 : null);
    if (kind === 'badge') return stale ? `Песочница · упрощённое ядро (JS-порт от ${PORT.checked})` : `Песочница · ядро пакета (JS-порт, сверен ${PORT.checked})`;
    if (kind === 'note') return stale ? `упрощённое ядро: JS-порт ядра от ${PORT.checked} (${PORT.core_git}), а ядро пакета после этого менялось (${stale})`
      : `настоящее ядро пакета (JS-порт estimator_core.py ${PORT.core_git}, сверен с Python ${PORT.checked}, расхождение ${PORT.dv})`;
    if (kind === 'short') return stale ? `упрощённое ядро (JS-порт от ${PORT.checked})` : `ядро пакета (JS-порт, сверен с Python ${PORT.checked})`;
    return stale ? `Песочница работает на JS-порте ядра от ${PORT.checked}, а ядро пакета после этого менялось (прогоны комиссии посчитаны ядром ${stale}): поведение может отличаться. Числа пакета — в «Прогоне данных комиссии».`
      : `Оценку в песочнице считает ядро пакета: JS-порт estimator_core.py и связки Runner (сверены с Python ${PORT.checked}: 15 сценариев и запись комиссии, расхождение ${PORT.dv}).`;
  }
  function applyPort() {
    const n = $('sbPort'); if (n) n.textContent = portClaim('note');
    const bs = document.querySelector('header .badge-sub');
    if (bs && (window.__tvMode || 'sandbox') === 'sandbox') bs.textContent = portClaim('badge');
  }
  window.TV_PORT_CLAIM = portClaim; window.TV_PORT_APPLY = applyPort; window.TV_EST_PORT = PORT;

  // ---------------------------------------------------------------- сценарии
  // «Свободная поездка» — исходная песочница: вагон идёт по всей линии, условия меняете сами.
  // Это сценарий «Ручное управление» с автоводителем на весь маршрут (числа сценариев не трогает).
  const FREE = { key: 'free', ru: 'Свободная поездка',
    about: 'Вагон идёт по всей линии (pathgraph организаторов) от остановки к остановке. Погоду, отказы датчиков, застройку и массу меняйте сами — переключателями ниже.',
    expect: 'На сухом рельсе обе тележки принимаются, модель идёт по колёсам; при срыве сцепления или отказе датчика — отбрасывает или исключает тележку (смотрите подписи под тележками и панель «Что сейчас думает модель»).' };
  const GROUPS = [['Поездка', ['free', 'manual']], ['Погода и сцепление', ['dry', 'rain_spin', 'ice_skid', 'leaves']],
    ['Отказы датчиков', ['front_fail', 'both_fail', 'both_fail_start', 'both_fail_stop', 'stuck', 'dropout', 'noise']], ['Условия', ['rush', 'hill', 'urban']]];
  for (const p of SB.PRESETS) if (!GROUPS.some(g => g[1].includes(p.key))) GROUPS[GROUPS.length - 1][1].push(p.key);
  const presetOf = k => k === 'free' ? FREE : SB.PRESETS.find(p => p.key === k);

  // ---------------------------------------------------------------- движок
  // скорость показа из ссылки или памяти: только 1, 4 или 10 (иначе ?sbsp=abc давал NaN и песочница вставала)
  const sbsp0 = +(qs.get('sbsp') || store.get('tv.sbsp') || 4);
  const UI = { eng: null, key: null, play: true, speed: [1, 4, 10].includes(sbsp0) ? sbsp0 : 4, saved: null, exp: null, fade: 0,
    wheelAng: [0, 0], uz: [], uzOn: false, pax: new Map(), board: null, arrivals: 0, stops: [], onboard: 24, boardN: 0, alightN: 0, rush: false };
  // свободная поездка, как в исходной песочнице: масса вагона в имитаторе — пустой вагон и пассажиры
  // (75 кг на человека); на остановках одни выходят, другие садятся, в час пик народу больше. Модель
  // массу не знает. В сценариях масса — как в сценарии (таблица docs/SANDBOX.md), «Час пик» — +30 %.
  const M_EMPTY = 26000, M_PAX = 75;
  const paxMass = E => { if (UI.key === 'free') E.userMass = Math.round((M_EMPTY + M_PAX * UI.onboard) / E.sheet.M_nom * 1000) / 1000; };
  const seed = +(qs.get('seed') || 7);
  function makeDriver(E) { E.route = E.track.stops.filter(x => x > E.plant.s + 30); return new SB.Driver(E, {}); }
  function load(key, opts = {}) {
    if (!presetOf(key)) key = 'free';
    const E = new SB.Engine({ sheet: SHEET, track: window.TV_TRACK, preset: key === 'free' ? 'manual' : key, seed });
    if (key === 'free') E.driver = makeDriver(E);
    UI.key = key; UI.onboard = 24; UI.boardN = UI.alightN = 0; paxMass(E);
    instrument(E);
    UI.eng = E; UI.key = key; UI.saved = null; UI.play = true; UI.exp = null;
    if (!opts.quiet) store.set('tv.preset', key);
    sceneReset(opts.fade !== false);
    buildTrackBar();
    syncScenario(); syncControls(true); render(true);
    layoutBoard();
  }

  // ---------------------------------------------------------------- линия для сцены и карты
  // высоты сцены — интеграл уклона сценария (профиль pathgraph, у «Подъёма / спуска» — подменённый)
  const ZS = { z: null, n: 0 };
  const zAt = m => { if (!ZS.z) return 0; const q = Math.max(0, Math.min(ZS.n - 1.000001, m / 5)), i = Math.floor(q); return ZS.z[i] + (ZS.z[i + 1] - ZS.z[i]) * (q - i); };
  const gradeAt = m => UI.eng.track.grade(Math.max(0, Math.min(UI.eng.track.L, m)));
  const stopName = (T, p) => `Остановка ${T.stops.indexOf(p) + 1}`;
  const lineEnds = () => {
    const d = UI.eng.trackDoc.dirs[0], nm = (d.name || '').split(/\s+[-—–]\s+/).map(x => x.replace(/(^|[\s-])\S/g, c => c.toUpperCase()));
    return UI.eng.trackDoc.synthetic ? ['начало', 'конец'] : [nm[0] || 'начало', nm[1] || 'конец'];
  };
  function sceneReset(fade) {
    const E = UI.eng, T = E.track, n = Math.ceil(T.L / 5) + 2, z = new Float32Array(n);
    z[0] = T.zAt(0);
    for (let i = 1; i < n; i++) z[i] = z[i - 1] + 5 * Math.tan(T.grade(Math.min(T.L, (i - 0.5) * 5)));
    ZS.z = z; ZS.n = n;
    UI.stops = (UI.key === 'free' ? T.stops.slice() : [E.route0(), ...E.route]).filter((p, i, a) => a.indexOf(p) === i);
    for (const p of UI.stops) if (!UI.pax.has(p)) UI.pax.set(p, 2 + Math.floor(Math.random() * 5));
    UI.uz = []; UI.uzOn = false; UI.boardStop = null; UI.arrivals = 0; UI.wheelAng = [0, 0];
    if (fade) UI.fade = 1;
  }

  // ---------------------------------------------------------------- управление
  const setSeg = (attr, val) => document.querySelectorAll(`[${attr}]`).forEach(b => b.setAttribute('aria-checked', String(b.getAttribute(attr) === String(val))));
  function setRunLabel() {
    const E = UI.eng, done = E && E.done, run = UI.play && !done;
    $('bRunTxt').textContent = done ? 'Заново' : run ? 'Стоп' : 'Старт';
    $('bRun').setAttribute('aria-label', $('bRunTxt').textContent);
    $('icoStop').style.display = run ? '' : 'none';
    $('icoStart').style.display = !run && !done ? '' : 'none';
    $('icoAgain').style.display = done ? '' : 'none';
  }
  $('bRun').addEventListener('click', () => { if (UI.eng.done) { load(UI.key); return; } UI.play = !UI.play; setRunLabel(); });
  document.querySelectorAll('[data-sbsp]').forEach(b => b.addEventListener('click', () => { UI.speed = +b.dataset.sbsp; store.set('tv.sbsp', UI.speed); setSeg('data-sbsp', UI.speed); }));
  setSeg('data-sbsp', UI.speed);
  function setDrive(manual) {
    const E = UI.eng;
    if (manual && E.driver) { UI.saved = E.driver; E.driver = null; E.setManual(E.plant.notch); }
    if (!manual && !E.driver) { E.driver = UI.saved || makeDriver(E); UI.saved = null; }
    syncControls(true); layoutBoard(false);
  }
  document.querySelectorAll('[data-drive]').forEach(b => b.addEventListener('click', () => setDrive(b.dataset.drive === 'manual')));
  function setHandle(n) {
    const E = UI.eng;
    if (E.driver) setDrive(true);
    E.setManual(n);
    syncHandle();
  }
  function syncHandle() {
    const n = UI.eng.manualNotch;
    $('sbHandleV').textContent = (n > 0 ? '+' : '') + n;
    setSeg('data-sbh', n > 0 ? 15 : n < 0 ? -15 : 0);
  }
  document.querySelectorAll('[data-sbh]').forEach(b => b.addEventListener('click', () => setHandle(+b.dataset.sbh)));
  // погода: подсвечена та, что сейчас под передней тележкой; нажатие задаёт её поверх сценария, ↺ — вернуть
  document.querySelectorAll('[data-sbwx]').forEach(b => b.addEventListener('click', () => { const E = UI.eng, w = b.dataset.sbwx; E.userWeather = E.userWeather === w ? null : w; syncControls(true); }));
  $('wxLbl').addEventListener('click', () => { UI.eng.userWeather = null; syncControls(true); });
  // датчики: подсвечено, что сейчас с тележками (сценарий или вы); «Норма» — норма поверх сценария
  const FAULT = { zero: { kind: 'zero' }, stuck: { kind: 'stuck' }, dropout: { kind: 'dropout' }, noise: { kind: 'noise', sigma_kmh: 0.9 }, outliers: { kind: 'outliers', p: 0.05 } };
  function setSens(sel, kind) {
    const E = UI.eng, f = () => Object.assign({}, FAULT[kind || $('sbKind').value] || FAULT.zero);
    E.userFaults = sel === 'ok' ? ['none', 'none'] : sel === 'front' ? [f(), 'none'] : sel === 'rear' ? ['none', f()] : [f(), f()];
    E.userDrop = false;
    syncControls(true); layoutBoard(false);
  }
  document.querySelectorAll('[data-sens]').forEach(b => b.addEventListener('click', () => setSens(b.dataset.sens)));
  $('sbKind').addEventListener('change', e => { const cur = sensSel(); setSens(cur === 'ok' ? 'front' : cur, e.target.value); });
  $('snLbl').addEventListener('click', () => { UI.eng.userFaults = [null, null]; UI.eng.userDrop = false; syncControls(true); layoutBoard(false); });
  $('swUrban').addEventListener('click', () => { const E = UI.eng; E.userGnssOff = !(E.userGnssOff || E.gnssOff); syncControls(true); });
  $('swRush').addEventListener('click', () => {
    const E = UI.eng;
    if (UI.key === 'free') UI.rush = !UI.rush;
    else { const on = (E.userMass ?? (E.cond ? E.cond.mass : 1)) > 1.001; E.userMass = on ? 1.0 : 1.3; }
    syncControls(true);
  });
  $('swWsp').addEventListener('click', () => { const E = UI.eng; E.plant.C.wsp = !E.plant.C.wsp; syncControls(true); });
  // что сейчас с датчиками: ручной выбор (сразу после нажатия) или сценарий
  const userFault = () => UI.eng.userFaults.some(x => x !== null) ? UI.eng.userFaults.map(x => x === 'none' ? null : x) : null;
  function sensSel() {
    const c = UI.eng.cond; if (!c) return 'ok';
    const u = userFault(), f = u || c.faults, d = !u && c.drop;
    const fr = !!f[0] || d, rr = !!f[1] || d;
    return fr && rr ? 'both' : fr ? 'front' : rr ? 'rear' : 'ok';
  }
  let lastRows = '';
  function syncControls(force) {
    const E = UI.eng, c = E.cond; if (!c) return;
    setSeg('data-sbwx', E.userWeather || c.weatherB[0]);
    $('wxLbl').classList.toggle('ovr', E.userWeather !== null);
    $('wxLbl').title = E.userWeather !== null ? 'Погода задана вручную. Нажмите, чтобы вернуть как в сценарии' : 'Погода под передней тележкой (как в сценарии)';
    const sel = sensSel(); setSeg('data-sens', sel);
    const uf = userFault(), kind = !uf && c.drop ? 'dropout' : (((uf || c.faults)[0] || (uf || c.faults)[1]) || {}).kind;
    if (kind && document.activeElement !== $('sbKind')) $('sbKind').value = kind;
    const ovr = E.userFaults.some(x => x !== null);
    $('snLbl').classList.toggle('ovr', ovr);
    $('snLbl').title = ovr ? 'Датчики заданы вручную. Нажмите, чтобы вернуть как в сценарии' : 'Датчики скорости тележек (как в сценарии)';
    $('swUrban').setAttribute('aria-checked', String(!!(E.userGnssOff || E.gnssOff)));
    $('swRush').setAttribute('aria-checked', String(UI.key === 'free' ? UI.rush : (E.userMass ?? c.mass) > 1.001));
    $('swWsp').setAttribute('aria-checked', String(!!E.plant.C.wsp));
    setSeg('data-drive', E.driver ? 'auto' : 'manual');
    const rows = `${!!E.driver}|${sel !== 'ok'}`;
    $('handleRow').hidden = !!E.driver;
    $('kindRow').hidden = $('kindLbl').hidden = sel === 'ok';
    if (!E.driver) syncHandle();
    if (rows !== lastRows) { lastRows = rows; if (!force) layoutBoard(false); }
    setRunLabel();
  }

  // ---------------------------------------------------------------- список сценариев
  function buildScPop() {
    let h = '';
    for (const [g, keys] of GROUPS) {
      h += `<div class="sc-g">${g}</div><div class="sc-list" role="radiogroup" aria-label="${g}">`;
      for (const k of keys) { const p = presetOf(k); if (p) h += `<button role="radio" aria-checked="false" data-preset="${k}" title="${p.about.replace(/"/g, '&quot;')}">${p.ru}</button>`; }
      h += '</div>';
    }
    h += '<div id="scFull" class="sc-full"></div>';
    $('scPop').innerHTML = h;
    $('scPop').querySelectorAll('[data-preset]').forEach(b => b.addEventListener('click', () => { closePop(); stopPres(); load(b.dataset.preset); }));
  }
  function syncScenario() {
    const p = presetOf(UI.key);
    $('scName').textContent = p.ru;
    $('scBtn').title = `${p.ru}: ${p.about}`;
    $('scAbout').textContent = p.about;
    setSeg('data-preset', UI.key);
    $('scFull').innerHTML = `<p><b>${p.ru}.</b> ${p.about}</p><p><span class="opacity-60">Модель должна:</span> ${p.expect}</p>`;
    $('thExpect').textContent = p.expect;
    const syn = !!UI.eng.trackDoc.synthetic;
    $('sbTrackH').textContent = syn ? 'Карта · линия-заглушка' : 'Карта маршрута';
    $('mapHead').title = 'Свернуть или развернуть карту. ' + (syn ? 'Файла js/track.js с pathgraph организаторов нет (его делает simulator/tools/gen_track.py): линия, остановки и профиль придуманы, числа не совпадут с docs/SANDBOX.md.'
      : `Линия — pathgraph организаторов: ${lineEnds().join(' — ')}, ${(UI.eng.track.L / 1000).toFixed(2).replace('.', ',')} км; остановки — из карты пакета.`);
  }
  function placePop() {
    const pop = $('scPop'), r = $('scBtn').getBoundingClientRect(), cr = $('ctrl').getBoundingClientRect();
    const w = Math.max(280, Math.min(cr.width - 12, innerWidth - 24)), left = Math.max(12, Math.min(innerWidth - w - 12, cr.left + (cr.width - w) / 2));
    pop.style.left = left + 'px'; pop.style.width = w + 'px';
    const below = innerHeight - r.bottom - 16, above = r.top - 70;
    if (below >= 300 || below >= above) { pop.style.top = (r.bottom + 6) + 'px'; pop.style.bottom = ''; pop.style.maxHeight = Math.max(160, below) + 'px'; }
    else { pop.style.top = ''; pop.style.bottom = (innerHeight - r.top + 6) + 'px'; pop.style.maxHeight = Math.max(160, above) + 'px'; }
  }
  function openPop() { placePop(); $('scPop').hidden = false; $('scBtn').setAttribute('aria-expanded', 'true'); const cur = $('scPop').querySelector('[aria-checked="true"]'); if (cur) cur.focus({ preventScroll: true }); }
  function closePop() { $('scPop').hidden = true; $('scBtn').setAttribute('aria-expanded', 'false'); }
  $('scBtn').addEventListener('click', e => { e.stopPropagation(); $('scPop').hidden ? openPop() : closePop(); });
  addEventListener('pointerdown', e => { if (!$('scPop').hidden && !e.target.closest('#scPop, #scBtn')) closePop(); });
  addEventListener('resize', () => { if (!$('scPop').hidden) placePop(); });
  $('dash').addEventListener('scroll', () => { if (!$('scPop').hidden) placePop(); }, { passive: true });

  // ---------------------------------------------------------------- «Что сейчас думает модель»
  $('thMoreBtn').addEventListener('click', () => { const open = $('sbThinkMore').hidden; $('sbThinkMore').hidden = !open; $('thMoreBtn').setAttribute('aria-expanded', String(open)); $('thMoreBtn').textContent = open ? 'свернуть' : 'подробнее'; store.set('tv.think', open ? '1' : '0'); });
  if (store.get('tv.think') === '1') $('thMoreBtn').click();
  const fk = (x, d = 1) => isFinite(x) ? x.toFixed(d) : '—';
  function renderThink() {
    const E = UI.eng, x = UI.exp = SB.explain(E), o = E.last ? E.last.o : null, R = E.rows, n = R.n - 1;
    $('thHead').textContent = x.head;
    const m = x.mode;
    $('thMode').innerHTML = `<span class="mode-ico" style="color:${modeCol(m)}">${ICO[m] || ''}</span><span style="color:${modeCol(m)}">${SB.M_RU[m]}</span>`;
    for (let b = 0; b < 2; b++) {
      const g = x.bogies[b], el = $('thBog' + b);
      if (!g) { el.innerHTML = `<span class="k">${b ? 'задняя' : 'передняя'}</span> —`; continue; }
      const col = BOG_COL[g.code] || '', kmh = n >= 0 ? (b ? R.rr[n] : R.fr[n]) : NaN;
      el.innerHTML = `<span class="k">${b ? 'задняя' : 'передняя'}</span><span class="num">${fk(kmh)} км/ч</span><span class="th-chip" style="${col ? `color:${col}` : 'opacity:.6'}" title="${g.why.replace(/"/g, '&quot;')}">${g.st}</span>`;
    }
    const flag = (on, txt, col, title) => on ? `<span class="rv-flag on" style="color:${col}" title="${title}">${txt}</span>` : '';
    $('thFlags').innerHTML = [
      flag(o && !o.valid, 'недостоверно', C_BAD, 'valid = false: нет принятых показаний колёс за последнюю секунду'),
      flag(o && o.slip && !o.slip_all, 'срыв', C_WET, 'slip: признак срыва сцепления'),
      flag(o && o.slip_all, 'срыв обеих', C_WET, 'slip_all: обе тележки сорвались разом — скорость ведёт модель'),
      flag(o && o.ambiguous, 'стоим или скользим?', C_WET, 'ambiguous: колёса разом показывают ноль на ходу — не различить остановку и юз'),
      flag(o && o.frozen, 'залипли обе', C_BAD, 'frozen: показания обеих тележек застыли при команде тяги или тормоза'),
      flag(o && o.wheels_stale, 'колёса молчат', C_BAD, 'wheels_stale: нет показаний тележек дольше 1 с — только модель'),
      flag(o && o.valid && !o.slip && !o.ambiguous, 'достоверно', C_OK, 'valid: есть принятые показания колёс за последнюю секунду'),
    ].join('');
    $('thLines').innerHTML = x.lines.map(l => `<li>${l}</li>`).join('');
    $('thTruth').innerHTML = x.truth.map(l => `<li>${l}</li>`).join('');
  }

  // ---------------------------------------------------------------- клавиши
  addEventListener('keydown', e => {
    if ((window.__tvMode || 'sandbox') !== 'sandbox') return;
    if (e.key === 'Escape') { closePop(); closeHelp(); if (current) closeInfo(); return; }
    if (e.target.closest && e.target.closest('input, select, textarea')) return;
    if (current || !$('sbHelpModal').hidden) return;
    const E = UI.eng; if (!E) return;
    const up = ['w', 'W', 'ц', 'Ц', 'ArrowUp'].includes(e.key), down = ['s', 'S', 'ы', 'Ы', 'ArrowDown'].includes(e.key);
    if (!E.driver && (up || down)) { e.preventDefault(); setHandle(E.manualNotch + (up ? 1 : -1)); return; }
    if (!E.driver && (e.key === 'ArrowLeft' || e.key === 'ArrowRight')) { e.preventDefault(); setHandle(0); return; }
    if (e.key === ' ' && !(e.target.closest && e.target.closest('button'))) {
      e.preventDefault();
      if (!E.driver) setHandle(0); else $('bRun').click();
    }
  });

  // ---------------------------------------------------------------- тема
  const mq = matchMedia('(prefers-color-scheme: dark)');
  let night = false;
  function setTheme(t) {
    document.documentElement.dataset.theme = t; night = t === 'dark';
    $('themeTxt').textContent = night ? 'Ночь' : 'День';
    $('icoSun').style.display = night ? 'none' : ''; $('icoMoon').style.display = night ? '' : 'none';
    $('themeBtn').setAttribute('aria-pressed', String(night));
    store.set('vectra-theme', t);
  }
  const savedTheme = store.get('vectra-theme');
  setTheme(savedTheme === 'dark' || savedTheme === 'light' ? savedTheme : (mq.matches ? 'dark' : 'light'));
  $('themeBtn').addEventListener('click', () => setTheme(night ? 'light' : 'dark'));

  // ---------------------------------------------------------------- окна: модель, пакет ROS 2, метрики, запись
  const INFO = {
    model: ['Как устроена модель', 'Состояние — пять величин: путь, скорость, возмущение d (уклон, масса, ветер) и масштабы тяги и торможения. Прогноз строится по ручке −15…+15 через физику вагона: таблица ускорений A(u, v), снятая с записей, запаздывание привода, сопротивление движению; сила ограничена сцеплением μ.\nКоррекция — по двум тележкам в сигма-точечном фильтре (UKF), шаг 20 Гц по меткам сообщений. Каждое показание проходит проверки: невязка против прогноза, физический предел ускорения колеса, скачок за такт, согласие тележек между собой. Отброшенное показание состояние не меняет, поэтому буксующая тележка не тянет оценку за собой. Сорвались обе разом — скорость ведёт модель, пока колёса не вернутся к вагону. Нули всех колёс на ходу — это «стоим или скользим?»: модель им не верит и честно расширяет полосу ±2σ.\nСцепление μ оценивается по признакам срыва, возмущение d подстраивается на выбеге, на остановке ошибка скорости обнуляется. GNSS нужен только в первые 3 с — для выставки положения.\n{PORT} Лист — жюри (config/tram.yaml): 2 тележки, км/ч, таблица привода; масштабы привода в нём постоянны. Имитатор вагона — приближение; числа на реальных записях — в «Прогоне данных комиссии».'],
    ros: ['Пакет ROS 2', ''], metrics: ['Метрики: модель против колеса', ''], replay: ['Запись поездки', ''],
  };
  const EXTRA = {
    ros: [760, `<p class="text-[16px] leading-[1.5]">Ядро оценщика написано без зависимостей от ROS, нода — тонкая обёртка вокруг него. Та же связка (Runner) считает офлайн-оценку по записям, поэтому числа оценки и работа ноды совпадают; в песочнице работает её JS-порт.</p>
      <div class="grid sm:grid-cols-2 gap-3 mt-4 text-[13px]">
        <div class="rounded-xl border border-[var(--line)] p-3">
          <div class="text-[12px] opacity-60 mb-2">Вход (по ТЗ)</div>
          <div class="flex justify-between gap-2"><span>/vehicle/front_bogie_velocity</span><span class="opacity-60">~9 Гц, км/ч</span></div>
          <div class="flex justify-between gap-2 mt-1"><span>/vehicle/rear_bogie_velocity</span><span class="opacity-60">~9 Гц, км/ч</span></div>
          <div class="flex justify-between gap-2 mt-1"><span>/vehicle/driver_position_cmd</span><span class="opacity-60">20 Гц, ±15</span></div>
          <div class="flex justify-between gap-2 mt-1 opacity-60"><span>/sensing/gnss/*/fix</span><span>только выставка, 3 с</span></div>
        </div>
        <div class="rounded-xl border border-[var(--line)] p-3">
          <div class="text-[12px] opacity-60 mb-2">Выход</div>
          <div class="flex justify-between gap-2"><span>/result/velocity</span><span class="opacity-60">скорость, м/с</span></div>
          <div class="flex justify-between gap-2 mt-1"><span>/result/position</span><span class="opacity-60">x, y (MGRS), z, σ</span></div>
          <div class="flex justify-between gap-2 mt-1"><span>/result/acceleration</span><span class="opacity-60">ускорение</span></div>
          <div class="flex justify-between gap-2 mt-1"><span>/tram/estimator_status</span><span class="opacity-60">режим, μ, k, σ, тележки</span></div>
        </div>
      </div>
      <div class="flex flex-wrap gap-2 mt-3 text-[12px]">
        <span class="rounded-full border border-[var(--line)] px-3 py-1">выход 20 Гц</span>
        <span class="rounded-full border border-[var(--line)] px-3 py-1">ROS 2 Humble</span>
        <span class="rounded-full border border-[var(--line)] px-3 py-1">шаг по меткам header.stamp</span>
        <span class="rounded-full border border-[var(--line)] px-3 py-1">UKF, состояние s, v, d, k_т, k_б</span>
        <span class="rounded-full border border-[var(--line)] px-3 py-1">положение: MGRS 37UCB, base_link</span>
      </div>
      <div class="text-[12px] opacity-60 mt-6 mb-2">Схема ноды: топики по ТЗ, значения — из песочницы (ядро пакета, лист жюри: 2 тележки)</div>
      <canvas id="rosC" class="block w-full h-[240px]"></canvas>
      <div class="text-[12px] opacity-60 mt-6">Время шага ядра (связка + UKF) в этом браузере</div>
      <div class="grid grid-cols-3 gap-4 mt-2">
        <div><div class="k">медиана</div><div class="big text-[24px] num"><span id="bMed">…</span> <span class="text-[13px] opacity-60">мкс</span></div></div>
        <div><div class="k">99 % шагов быстрее</div><div class="big text-[24px] num"><span id="bP99">…</span> <span class="text-[13px] opacity-60">мкс</span></div></div>
        <div><div class="k">занято от бюджета 20 Гц</div><div class="big text-[24px] num" style="color: var(--ok);"><span id="bLoad">…</span> <span class="text-[13px] opacity-60">%</span></div></div>
      </div>
      <canvas id="benchC" class="block w-full h-[64px] mt-3"></canvas>
      <div class="text-[11px] opacity-50 mt-2">Замер JS-порта ядра пакета: связка Runner и UKF с листом жюри повторяют входы текущей поездки (сообщения тележек и ручки), время — на один шаг 50 мс. Это не замер ноды: время Python-ноды на 20 Гц — в docs/EVAL.md.</div>`],
    metrics: [900, '<div id="sbMetrics"></div>'],
    replay: [940, `<canvas id="repC" class="block w-full h-[260px] cursor-crosshair"></canvas>
      <input id="repR" type="range" min="0" max="1" value="1" class="w-full mt-3" aria-label="Перемотка записи" style="accent-color: var(--ok);">
      <div id="repInfo" class="flex flex-wrap gap-x-4 gap-y-1 text-[12px] mt-2 num"></div>
      <div class="text-[11px] opacity-50 mt-3">Запись — весь текущий сценарий (до 30 минут, 20 Гц): истина имитатора, модель ±2σ, «только колесо», полоса режимов. Наведите на график или двигайте ползунок.</div>`],
  };
  const typed = $('typed'), cursorEl = $('cursor'), ghost = $('ghost'), modal = $('modal');
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  let typeTimer = 0, current = null, metTimer = 0, repI = -1;
  function openInfo(key) {
    closeHelp(); closePop();
    current = key;
    const [title, txt0] = INFO[key];
    const txt = txt0.replace('{PORT}', portClaim('model'));
    const ex = EXTRA[key];
    $('mCard').style.maxWidth = (ex ? ex[0] : 600) + 'px';
    $('mExtra').innerHTML = ex ? ex[1] : '';
    $('mText').style.display = txt ? '' : 'none';
    clearInterval(metTimer);
    if (key === 'metrics') { renderMetrics(); metTimer = setInterval(renderMetrics, 1000); }
    if (key === 'replay') {
      repI = -1;
      $('repR').addEventListener('input', e => { repI = +e.target.value; });
      $('repC').addEventListener('mousemove', e => { const r = e.currentTarget.getBoundingClientRect(), N = UI.eng.rows.n; repI = Math.max(0, Math.min(N - 1, Math.round((e.clientX - r.left - 28) / (r.width - 28) * (N - 1)))); });
    }
    $('mTitle').textContent = title; ghost.textContent = txt; typed.textContent = '';
    modal.style.display = 'flex'; cursorEl.style.display = '';
    document.querySelectorAll('[data-open]').forEach(p => p.setAttribute('aria-pressed', String(p.dataset.open === key)));
    clearInterval(typeTimer);
    if (reduce) { typed.textContent = txt; cursorEl.style.display = 'none'; }
    else {
      let i = 0;
      typeTimer = setInterval(() => {
        i += 2; typed.textContent = txt.slice(0, i);
        if (i >= txt.length) { clearInterval(typeTimer); cursorEl.style.display = 'none'; }
      }, 16);
    }
    $('mClose').focus();
  }
  function closeInfo() {
    current = null; clearInterval(typeTimer); clearInterval(metTimer); modal.style.display = 'none';
    document.querySelectorAll('[data-open]').forEach(p => p.setAttribute('aria-pressed', 'false'));
  }
  document.querySelectorAll('[data-open]').forEach(el => el.addEventListener('click', e => { e.preventDefault(); current === el.dataset.open ? closeInfo() : openInfo(el.dataset.open); }));
  $('mClose').addEventListener('click', closeInfo);
  modal.addEventListener('click', e => { if (e.target === modal) closeInfo(); });
  // «Как читать экран»
  function openHelp() { if (current) closeInfo(); closePop(); $('sbHelpModal').hidden = false; $('helpBtn').setAttribute('aria-pressed', 'true'); $('sbHelpClose').focus(); }
  function closeHelp() { $('sbHelpModal').hidden = true; $('helpBtn').setAttribute('aria-pressed', 'false'); }
  $('helpBtn').addEventListener('click', () => { $('sbHelpModal').hidden ? openHelp() : closeHelp(); });
  $('sbHelpBtn').addEventListener('click', openHelp);
  $('sbHelpClose').addEventListener('click', closeHelp);
  $('sbHelpModal').addEventListener('click', e => { if (e.target === $('sbHelpModal')) closeHelp(); });

  // ---------------------------------------------------------------- метрики по условиям (за сессию)
  // Как в исходной песочнице: считается с открытия страницы по всем поездкам и сценариям, по шагам
  // ядра (20 Гц), пока вагон едет (> 0,3 м/с): MAE скорости, наибольшая ошибка, доля «истина в ±2σ»,
  // ошибка пути на перегоне (от остановки до остановки). Метрики текущего сценария — движок.
  const COND = ['Сухо', 'Дождь', 'Листопад', 'Наледь', 'Застройка (нет GNSS)', 'Час пик', 'Подъём / спуск', 'Отказ одной тележки', 'Отказ обеих тележек', 'Пропуски сообщений'];
  const MET = Object.fromEntries(COND.map(c => [c, { t: 0, ae: 0, aeN: 0, max: 0, inInt: 0, seg: [], segN: [] }]));
  const SLIPEV = { delays: [], missed: 0, short: 0 };
  let metCur = { eng: null };
  function metCond(R, i) {
    const f0 = R.f0[i], f1 = R.f1[i];
    if (R.dr[i] || (f0 === 2 && f1 === 2)) return ['Пропуски сообщений'];
    if (f0 >= 0 && f1 >= 0) return ['Отказ обеих тележек'];
    if (f0 >= 0 || f1 >= 0) return ['Отказ одной тележки'];
    const c = [], wx = R.wx[i];
    if (wx === 1) c.push('Дождь'); else if (wx === 2) c.push('Листопад'); else if (wx === 3) c.push('Наледь');
    if (R.gn[i]) c.push('Застройка (нет GNSS)');
    if (R.m[i] > 1.1) c.push('Час пик');
    if (Math.abs(R.g[i]) > 0.02) c.push('Подъём / спуск');
    if (!c.length) c.push('Сухо');
    return c;
  }
  function metUpdate() {
    const E = UI.eng, R = E.rows; if (!R.n) return;
    if (metCur.eng !== E) metCur = { eng: E, t: -Infinity, arr: 0, seg: { s: 0, st: 0, ns: 0, cond: {} }, rampT: -1 };
    const dt = E.sheet.dt;
    for (let i = upper(R.t, metCur.t); i < R.n; i++) {
      const vt = R.vt[i];
      // срыв по истине (любая тележка: |обод − вагон| > max(0,3; 0,05·v)) -> первый флаг модели (срыв или «стоим или скользим?»)
      const lim = Math.max(0.3, 0.05 * vt), slipTrue = Math.abs(R.wf[i] - vt) > lim || Math.abs(R.wr[i] - vt) > lim, flag = (R.fl[i] & 6) !== 0, t = R.t[i];
      if (metCur.rampT === -1 && slipTrue && !flag) metCur.rampT = t;
      else if (metCur.rampT >= 0 && flag) { SLIPEV.delays.push((t - metCur.rampT) * 1000); metCur.rampT = -2; }
      else if (metCur.rampT >= 0 && !slipTrue) { if (t - metCur.rampT >= 0.5) SLIPEV.missed++; else SLIPEV.short++; metCur.rampT = -2; }
      if (metCur.rampT === -2 && !slipTrue && !flag) metCur.rampT = -1;
      if (!(vt > 0.3)) continue;
      const e = Math.abs(R.v[i] - vt), eN = Math.abs(R.nv[i] - vt), inI = e <= 2 * R.sv[i] + 1e-9;
      for (const c of metCond(R, i)) { const M = MET[c]; M.t += dt; M.ae += e * dt; M.aeN += eN * dt; M.max = Math.max(M.max, e); if (inI) M.inInt += dt; metCur.seg.cond[c] = (metCur.seg.cond[c] || 0) + dt; }
    }
    metCur.t = R.t[R.n - 1];
    while (metCur.arr < E.arrivals.length) {
      const a = E.arrivals[metCur.arr++], k = Math.max(0, upper(R.t, a.t) - 1), sg = metCur.seg;
      const dM = R.s[k] - sg.s, dT = R.st[k] - sg.st, dN = R.ns[k] - sg.ns, tot = Object.values(sg.cond).reduce((p, q) => p + q, 0) || 1;
      if (dT > 50) for (const [c, tc] of Object.entries(sg.cond)) if (tc / tot > 0.3) { MET[c].seg.push(Math.abs(dM - dT)); MET[c].segN.push(Math.abs(dN - dT)); }
      metCur.seg = { s: R.s[k], st: R.st[k], ns: R.ns[k], cond: {} };
    }
  }
  function renderMetrics() {
    const el = $('sbMetrics'); if (!el) return;
    const E = UI.eng, S = E.summary(), W = S.win, o = E.last ? E.last.o : null;
    const fx = (x, d = 2) => (x === null || !isFinite(x)) ? '<span class="opacity-40">нет</span>' : x.toFixed(d);
    const avg = a => a.length ? a.reduce((p, c) => p + c, 0) / a.length : null;
    const td = 'class="py-1.5 pr-3"';
    const srow = (name, a, b, title) => `<tr style="border-top:1px solid var(--line)" title="${title || ''}"><td ${td}>${name}</td><td ${td} style="color:var(--ok)">${a}</td><td class="py-1.5" style="color:var(--warn)">${b}</td></tr>`;
    let cur = srow('Скорость: MAE, м/с', fx(S.v_mae, 3), fx(S.naive_v_mae, 3), 'Средняя абсолютная ошибка против истины имитатора, все шаги 20 Гц')
      + srow('Скорость: наибольшая ошибка, км/ч', fx(S.v_max * KMH, 1), fx(S.naive_v_max * KMH, 1))
      + srow(`Путь от старта: ошибка, м (истина ${fx(S.path, 0)} м)`, fx(S.s_err, 1), fx(S.naive_s_err, 1), 'Путь от старта сценария: оценка минус истина (чистая одометрия, без привязок к остановкам)')
      + srow('Истина в ±2σ скорости', fx(100 * S.cov2s, 1) + ' %', '—', 'Доля шагов, где |ошибка| ≤ 2σ. У честной σ около 95 %');
    if (W) cur += srow(`В окне условий сценария (${fx(W.s, 0)} с): MAE, м/с`, fx(W.v_mae, 3), fx(W.naive_v_mae, 3), 'Пока действуют необычные условия сценария и 10 с после');
    let rows = '';
    for (const c of COND) {
      const m = MET[c];
      if (m.t < 0.5 && !m.seg.length) { rows += `<tr class="opacity-40" style="border-top:1px solid var(--line)"><td ${td}>${c}</td><td colspan="6" class="py-1.5">ещё не встречалось</td></tr>`; continue; }
      rows += `<tr style="border-top:1px solid var(--line)"><td ${td}>${c}</td>
        <td ${td} style="color:var(--ok)">${fx(m.ae / m.t, 3)}</td><td ${td} style="color:var(--warn)">${fx(m.aeN / m.t, 3)}</td>
        <td ${td}>${fx(m.max * KMH, 1)}</td>
        <td ${td} style="color:var(--ok)">${fx(avg(m.seg), 1)}</td><td ${td} style="color:var(--warn)">${fx(avg(m.segN), 1)}</td>
        <td class="py-1.5">${m.t ? (m.inInt / m.t * 100).toFixed(1) + ' %' : '—'}</td></tr>`;
    }
    const sd = SLIPEV.delays.slice().sort((a, b) => a - b), med = sd.length ? sd[Math.floor(sd.length / 2)] : null;
    el.innerHTML = `<div class="text-[13px] opacity-60">Сценарий «${presetOf(UI.key).ru}», ${Math.floor(E.t / 60)} мин ${String(Math.floor(E.t % 60)).padStart(2, '0')} с${E.done ? ' — закончен' : ''}</div>
      <div class="overflow-x-auto"><table class="w-full text-[13px] num text-left mt-1"><thead class="opacity-60 text-[12px]"><tr><th class="pb-2 pr-3 font-normal"></th><th class="pb-2 pr-3 font-normal" style="color:var(--ok)">модель</th><th class="pb-2 font-normal" style="color:var(--warn)">только колесо</th></tr></thead><tbody>${cur}</tbody></table></div>
      <div class="text-[13px] opacity-60 mt-5">По условиям — с открытия страницы, все поездки и сценарии, пока вагон едет</div>
      <div class="overflow-x-auto"><table class="w-full text-[13px] num text-left mt-1">
      <thead class="opacity-60 text-[12px]"><tr>
        <th class="pb-2 pr-3 font-normal">Условие</th>
        <th class="pb-2 pr-3 font-normal" colspan="2">MAE скорости, м/с<br><span style="color:var(--ok)">модель</span> / <span style="color:var(--warn)">только колесо</span></th>
        <th class="pb-2 pr-3 font-normal">Макс. ошибка<br>модели, км/ч</th>
        <th class="pb-2 pr-3 font-normal" colspan="2">Ошибка пути на конец<br>перегона, м: <span style="color:var(--ok)">модель</span> / <span style="color:var(--warn)">колесо</span></th>
        <th class="pb-2 font-normal" title="Доля времени, когда истинная скорость внутри ±2σ оценки. У честной σ около 95 %">Истина в ±2σ<br><span class="opacity-60">цель ≈ 95 %</span></th></tr></thead>
      <tbody>${rows}</tbody></table></div>
      <div class="flex flex-wrap gap-x-6 gap-y-1 mt-4 text-[13px]">
        <span title="От начала срыва колеса по истине до первого флага модели («срыв» или «стоим или скользим?»); событие засчитывается, только если флаг в начале срыва не горел"><span class="opacity-60">Обнаружение срыва, медиана:</span> ${med === null ? (SLIPEV.missed ? 'флаг не включался' : 'срывов ещё не было') : Math.round(med) + ' мс'} <span class="opacity-60">(обнаружено ${sd.length}, пропущено ${SLIPEV.missed}; срывы короче 0,5 с не считаются)</span></span>
        <span><span class="opacity-60">Масштаб тяги k<sub>т</sub> / торможения k<sub>б</sub>:</span> ${o ? o.k_t.toFixed(2) + ' / ' + o.k_b.toFixed(2) : '—'}</span>
        <span><span class="opacity-60">Сцепление μ, оценка:</span> ${o ? o.mu.toFixed(3) : '—'}</span>
        <span><span class="opacity-60">Масса вагона (имитатор):</span> ${(E.plant.mass_factor * E.sheet.M_nom / 1000).toFixed(1)} т</span>
      </div>
      <div class="text-[11px] opacity-50 mt-3">Сравнение с истиной имитатора, а не с GNSS. Ядро — ${portClaim('short')}, лист жюри; «только колесо» — среднее свежих показаний тележек (база из tools/eval.py). Метрики сценария с тем же зерном равны таблице docs/SANDBOX.md. Числа на реальных записях — в режиме «Прогон данных комиссии».</div>`;
  }

  // ---------------------------------------------------------------- схема ноды и время шага ядра
  const KB = { samples: [], inp: [], runner: null, pass: null, i: 0, off: 0, last: -Infinity, tick: 0 };
  // входы связки текущей поездки — для замера шага (только запись, на числа не влияет)
  function instrument(E) {
    const r = E.runner, ow = r.on_wheel.bind(r), oh = r.on_handle.bind(r);
    const rec = m => { KB.inp.push(m); if (KB.inp.length > 1600) KB.inp.splice(0, 400); };
    r.on_wheel = (i, st, v) => { rec([0, i, st, v]); return ow(i, st, v); };
    r.on_handle = (st, v) => { rec([1, 0, st, v]); return oh(st, v); };
    KB.inp = []; KB.runner = null; KB.pass = null;
  }
  function bench() {
    if (++KB.tick % 6 || KB.inp.length < 200) return;
    const E = UI.eng;
    if (!KB.runner) { KB.runner = new TramRunner.Runner(E.sheet, E.node); KB.pass = null; KB.last = -Infinity; KB.off = 0; }
    const R = KB.runner, t0 = performance.now();
    let n = 0, k = 0;
    while (n < 60 && k < 3000) {
      if (!KB.pass || KB.i >= KB.pass.length) {
        const prevEnd = KB.pass ? KB.pass[KB.pass.length - 1][2] + KB.off : 0;
        KB.pass = KB.inp.slice(); KB.off = prevEnd - KB.pass[0][2] + 0.1; KB.i = 0;
      }
      const m = KB.pass[KB.i++], st = m[2] + KB.off; k++;
      if (st <= KB.last) continue;
      KB.last = st;
      n += (m[0] ? R.on_handle(st, m[3]) : R.on_wheel(m[1], st, m[3])).length;
    }
    if (n) { KB.samples.push((performance.now() - t0) * 1000 / n); if (KB.samples.length > 400) KB.samples.shift(); }
  }
  function drawBench() {
    const c0 = $('benchC'); if (!c0) return;
    const [c, w, hh] = fit(c0), { fg, line } = css();
    c.clearRect(0, 0, w, hh);
    const S = KB.samples; if (S.length < 3) return;
    const a = S.slice().sort((p, q) => p - q);
    const med = a[Math.floor(a.length / 2)], p99 = a[Math.min(a.length - 1, Math.floor(a.length * 0.99))];
    $('bMed').textContent = med.toFixed(1); $('bP99').textContent = p99.toFixed(1);
    $('bLoad').textContent = (p99 / 50000 * 100).toFixed(2);
    const top = Math.max(p99 * 1.6, med * 2, 1), arr = S.slice(-120);
    const X = i => i / Math.max(1, arr.length - 1) * w, Y = v => hh - 2 - Math.min(1, v / top) * (hh - 6);
    c.strokeStyle = line; c.lineWidth = 1;
    c.beginPath(); c.moveTo(0, hh - 1.5); c.lineTo(w, hh - 1.5); c.stroke();
    c.setLineDash([4, 4]); c.strokeStyle = fg; c.globalAlpha = .35;
    c.beginPath(); c.moveTo(0, Y(p99)); c.lineTo(w, Y(p99)); c.stroke(); c.setLineDash([]); c.globalAlpha = 1;
    const g = c.createLinearGradient(0, 0, 0, hh);
    g.addColorStop(0, 'rgba(14,154,167,0.35)'); g.addColorStop(1, 'rgba(14,154,167,0)');
    c.beginPath(); c.moveTo(X(0), hh); arr.forEach((v, i) => c.lineTo(X(i), Y(v))); c.lineTo(X(arr.length - 1), hh); c.closePath();
    c.fillStyle = g; c.fill();
    c.beginPath(); arr.forEach((v, i) => c.lineTo(X(i), Y(v))); c.strokeStyle = C_OK; c.lineWidth = 1.6; c.lineJoin = 'round'; c.stroke();
    c.fillStyle = fg; c.globalAlpha = .5; c.font = FONT11; c.textAlign = 'right'; c.textBaseline = 'bottom';
    c.fillText(`99 %: ${p99.toFixed(1)} мкс`, w, Y(p99) - 2); c.globalAlpha = 1;
  }
  function drawRos() {
    const c0 = $('rosC'); if (!c0) return;
    const [c, w, hh] = fit(c0), { fg, line } = css(), E = UI.eng, o = E.last ? E.last.o : null, R = E.rows, n = R.n - 1, pl = E.plant;
    const mode = o ? SB.MODES[o.mode] : 'DEGRADED', cond = E.cond || { faults: [null, null] };
    c.clearRect(0, 0, w, hh);
    const box = (x, y, bw, bh, title, sub, col) => {
      c.strokeStyle = col || line; c.lineWidth = 1; c.beginPath(); c.roundRect(x, y, bw, bh, 8); c.stroke();
      c.fillStyle = fg; c.font = '500 12px system-ui, "Segoe UI", Roboto, sans-serif'; c.textAlign = 'center'; c.textBaseline = 'middle';
      c.fillText(title, x + bw / 2, y + bh / 2 - (sub ? 7 : 0), bw - 8);
      if (sub) { c.globalAlpha = .6; c.font = FONT11; c.fillText(sub, x + bw / 2, y + bh / 2 + 8, bw - 8); c.globalAlpha = 1; }
    };
    const flow = (x1, y1, x2, y2, sp, col) => {
      c.strokeStyle = line; c.lineWidth = 1.2; c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
      const k = 4, tt = performance.now() / 1000;
      c.fillStyle = col;
      for (let i = 0; i < k; i++) {
        const f = (tt * sp + i / k) % 1, x = x1 + (x2 - x1) * f, y = y1 + (y2 - y1) * f;
        c.globalAlpha = Math.sin(Math.PI * f) ** 0.8;
        c.beginPath(); c.arc(x, y, 2.6, 0, Math.PI * 2); c.fill();
      }
      c.globalAlpha = 1;
    };
    const iw = w * 0.2, ow = w * 0.22, mx = w * 0.29, mw = w * 0.42, pad = 16;
    const ys = [hh * 0.3, hh * 0.7], yo = [hh * 0.2, hh * 0.5, hh * 0.8];
    const bad = !!(cond.faults[0] || cond.faults[1] || cond.drop);
    box(0, ys[0] - 22, iw, 44, '/vehicle/driver_position_cmd', `20 Гц, ручка ${pl.notch > 0 ? '+' : ''}${pl.notch}`);
    box(0, ys[1] - 22, iw, 44, '/vehicle/*_bogie_velocity', n >= 0 ? `~9 Гц, ${fk(R.fr[n])} / ${fk(R.rr[n])} км/ч` : '~9 Гц', bad ? C_BAD : null);
    c.strokeStyle = C_OK; c.lineWidth = 1.5; c.beginPath(); c.roundRect(mx, 8, mw, hh - 16, 12); c.stroke();
    c.fillStyle = C_OK; c.font = '500 12px system-ui, "Segoe UI", Roboto, sans-serif'; c.textAlign = 'left'; c.textBaseline = 'top'; c.fillText('tram_state_estimator (Runner → ядро)', mx + pad, 16, mw - 2 * pad);
    const sw = (mw - pad * 2 - 10) / 2, sh = (hh - 16 - 30 - pad - 12) / 3;
    const subs = [['Привод и режим', o ? `ручка ${o.notch > 0 ? '+' : ''}${Math.round(o.notch)}, ${SB.M_RU[mode].toLowerCase()}` : ''], ['Проверка тележек', o ? `принято ${o.n_accepted}, отброшено ${o.n_rejected}` : ''],
      ['UKF: s, v, d', o ? `v ${(o.v * KMH).toFixed(1)} км/ч` : ''], ['Масштабы привода', o ? `k_т ${o.k_t.toFixed(2)}, k_б ${o.k_b.toFixed(2)}` : ''],
      ['Сцепление μ', o ? o.mu.toFixed(3) : ''], ['Возмущение d', o ? `${o.d.toFixed(2)} м/с²` : '']];
    subs.forEach(([t, sub], i) => box(mx + pad + (i % 2) * (sw + 10), 8 + 30 + Math.floor(i / 2) * (sh + 6), sw, sh, t, sub));
    const labels = [['/result/velocity', o ? `${(o.v * KMH).toFixed(1)} ± ${(2 * o.sigma_v * KMH).toFixed(1)} км/ч` : ''], ['/result/position', o ? `путь ${o.s.toFixed(0)} м ± ${(2 * o.sigma_s).toFixed(0)}` : ''], ['/tram/estimator_status', mode]];
    labels.forEach(([t, sub], i) => box(w - ow, yo[i] - 22, ow, 44, t, sub, i === 2 ? ST_COL[mode] : null));
    const run = UI.play && !E.done ? 1 : 0;
    flow(iw + 4, ys[0], mx - 4, ys[0], 0.35 * run, fg);
    flow(iw + 4, ys[1], mx - 4, ys[1], 0.5 * run, bad ? C_BAD : fg);
    yo.forEach(y => flow(mx + mw + 4, y, w - ow - 4, y, 0.5 * run, C_OK));
  }

  // ---------------------------------------------------------------- запись поездки
  function drawReplay() {
    const c0 = $('repC'), E = UI.eng, R = E.rows; if (!c0 || R.n < 2) return;
    const [c, w, hh] = fit(c0), { fg, line } = css();
    c.clearRect(0, 0, w, hh);
    const PL = 28, PB = 18, pw = w - PL, ph = hh - PB - 6, N = R.n, stride = Math.max(1, Math.floor(N / (pw * 1.5)));
    let top = 40; for (let i = 0; i < N; i += stride) for (const x of [R.vt[i], R.v[i] + 2 * R.sv[i], R.nv[i]]) if (x * KMH < 90) top = Math.max(top, x * KMH);
    const ymax = Math.min(100, Math.ceil((top + 4) / 10) * 10);
    const X = i => PL + i / (N - 1) * pw, Y = val => 6 + ph - Math.max(0, Math.min(ymax, val)) / ymax * ph;
    c.font = FONT11; c.strokeStyle = line; c.fillStyle = fg; c.textAlign = 'right'; c.textBaseline = 'middle';
    for (let y = 0; y <= ymax; y += ymax > 60 ? 20 : 10) { c.beginPath(); c.moveTo(PL, Y(y) + .5); c.lineTo(w, Y(y) + .5); c.stroke(); c.globalAlpha = .5; c.fillText(y, PL - 6, Y(y)); c.globalAlpha = 1; }
    c.beginPath(); for (let i = 0; i < N; i += stride) c.lineTo(X(i), Y((R.v[i] + 2 * R.sv[i]) * KMH)); for (let i = N - 1; i >= 0; i -= stride) c.lineTo(X(i), Y((R.v[i] - 2 * R.sv[i]) * KMH)); c.closePath(); c.fillStyle = 'rgba(14,154,167,0.22)'; c.fill();
    const path = (arr, col, lw, al) => { c.beginPath(); c.strokeStyle = col; c.lineWidth = lw; c.globalAlpha = al; for (let i = 0; i < N; i += stride) c.lineTo(X(i), Y(arr[i] * KMH)); c.stroke(); c.globalAlpha = 1; };
    path(R.v, C_OK, 2, 1); path(R.nv, C_NAIVE, 1.2, .9); c.setLineDash([4, 4]); path(R.vt, fg, 1.2, .85); c.setLineDash([]);
    let a = 0;
    for (let i = 1; i <= N; i++) { if (i < N && R.mode[i] === R.mode[a]) continue; c.fillStyle = ST_COL[SB.MODES[R.mode[a]]]; c.fillRect(X(a), hh - 10, Math.max(1, X(Math.min(i, N - 1)) - X(a)), 6); a = i; }
    const i = repI < 0 ? N - 1 : Math.min(N - 1, repI);
    c.strokeStyle = fg; c.globalAlpha = .7; c.beginPath(); c.moveTo(X(i) + .5, 6); c.lineTo(X(i) + .5, hh - 10); c.stroke(); c.globalAlpha = 1;
    const t = R.t[i], m = SB.MODES[R.mode[i]], h = R.h[i];
    $('repInfo').innerHTML = `<span class="opacity-60">${Math.floor(t / 60)} мин ${String(Math.floor(t % 60)).padStart(2, '0')} с</span>
      <span>режим <b style="color:${modeCol(m)}">${SB.M_RU[m].toLowerCase()}</b></span>
      <span>ручка ${h > 0 ? '+' : ''}${Math.round(h)}</span>
      <span>истина ${(R.vt[i] * KMH).toFixed(1)}</span>
      <span style="color:var(--warn)">колесо ${(R.nv[i] * KMH).toFixed(1)}</span>
      <span style="color:var(--ok)">модель ${(R.v[i] * KMH).toFixed(1)} [${Math.max(0, (R.v[i] - 2 * R.sv[i]) * KMH).toFixed(1)}; ${((R.v[i] + 2 * R.sv[i]) * KMH).toFixed(1)}] км/ч</span>
      <span>путь ${R.s[i].toFixed(0)} м (истина ${R.st[i].toFixed(0)})</span><span class="opacity-60">${metCond(R, i).join(', ')}</span>`;
    const r = $('repR'); r.max = N - 1; if (repI < 0) r.value = N - 1;
  }

  // ---------------------------------------------------------------- показ 60 с: главные сценарии подряд
  // Каждый шаг загружает сценарий, проматывает его к нужному месту (тем же ядром, быстрее) и
  // показывает несколько секунд; в конце — метрики по условиям. Моменты — docs/SANDBOX.md §5.
  const PRES = [
    { t: 0, key: 'dry', ff: 12, sp: 4, txt: 'Сухо. Модель получает только то, что даст комиссия: две тележки в км/ч и ручку. Бирюзовая линия — оценка, пунктир — истина имитатора, оранжевая — просто среднее колёс.' },
    { t: 8, key: 'rain_spin', ff: 3, sp: 1, txt: 'Дождь и полная тяга: передняя тележка буксует. Её показание отброшено — модель не идёт за колесом и снижает оценку сцепления μ.' },
    { t: 17, key: 'ice_skid', ff: 51, sp: 1, txt: 'Наледь, экстренное торможение: колёса встали, вагон скользит. Простая одометрия сказала бы «стоим»; модель нулям не верит, держит скорость по ручке и расширяет полосу ±2σ.' },
    { t: 26, key: 'front_fail', ff: 33, sp: 4, txt: 'Отказ передней тележки: 0 км/ч на ходу. Модель исключает её как оборванную и едет по задней.' },
    { t: 33, key: 'both_fail', ff: 38, sp: 4, txt: 'Обе тележки разом показывают 0 км/ч — так вагон затормозить не может. Скорость по ручке и физике, полоса растёт. Здесь это лучший случай: на реальных записях ошибка больше (docs/SANDBOX.md).' },
    { t: 42, key: 'urban', ff: 17, sp: 4, txt: 'Застройка: спутники пропадают — модели всё равно. GNSS ей нужен только в первые 3 с для выставки положения.' },
    { t: 49, key: 'rush', ff: 20, sp: 4, txt: 'Час пик: вагон на 30 % тяжелее, модель об этом не знает. Прогноз по ручке ошибается, но тележки согласны — модель идёт по колёсам.' },
    { t: 60, end: true },
  ];
  let presT = -1, presI = 0, mapBeforePres = false, speedBeforePres = null;
  function startPres() {
    closeInfo(); closeHelp(); closePop();
    presT = 0; presI = 0; speedBeforePres = UI.speed;
    $('pres').style.display = ''; $('presBtn').setAttribute('aria-pressed', 'true');
    mapBeforePres = mapOn; if (mapOn && innerWidth >= 900) setMap(false);
    layoutBoard();
  }
  function stopPres() {
    if (presT < 0) return;
    presT = -1; $('pres').style.display = 'none'; $('presBtn').setAttribute('aria-pressed', 'false');
    if (speedBeforePres) { UI.speed = speedBeforePres; setSeg('data-sbsp', UI.speed); speedBeforePres = null; }
    if (mapBeforePres) setMap(true); mapBeforePres = false;
    layoutBoard();
  }
  $('presBtn').addEventListener('click', () => presT >= 0 ? stopPres() : startPres());
  $('presStop').addEventListener('click', stopPres);
  function presTick(dt) {
    if (presT < 0) return;
    presT += dt;
    while (presI < PRES.length && presT >= PRES[presI].t) {
      const st = PRES[presI++];
      if (st.end) { stopPres(); openInfo('metrics'); return; }
      load(st.key, { quiet: true });
      const E = UI.eng; while (E.t < st.ff - 1e-6 && !E.done) E.advance(Math.min(1, st.ff - E.t));
      UI.speed = st.sp; setSeg('data-sbsp', UI.speed); UI.fade = 0.6;
      $('presTxt').textContent = st.txt; $('presStep').textContent = `Шаг ${presI} из ${PRES.length - 1} · ${presetOf(st.key).ru}${st.sp !== 4 ? ` · время ×${st.sp}` : ''}`;
      render(true); layoutBoard(false);
    }
    if (presT >= 0) $('presBar').style.width = Math.min(100, presT / 60 * 100) + '%';
  }

  // ---------------------------------------------------------------- звук: гул мотора, сигнал на остановке
  let actx = null, hum = null;
  function ensureAudio() {
    try { actx = actx || new (window.AudioContext || window.webkitAudioContext)(); if (actx.state !== 'running') actx.resume(); } catch (_) { actx = null; }
    if (actx && !hum) {
      const o1 = actx.createOscillator(), o2 = actx.createOscillator(), lp = actx.createBiquadFilter(), g = actx.createGain();
      o1.type = 'sawtooth'; o2.type = 'triangle'; lp.type = 'lowpass'; lp.frequency.value = 320; lp.Q.value = 0.7;
      g.gain.value = 0;
      o1.connect(lp); o2.connect(lp); lp.connect(g); g.connect(actx.destination);
      o1.start(); o2.start();
      hum = { o1, o2, lp, g };
    }
  }
  // браузер даёт звук только после первого жеста: тогда и включаем, тихо
  const unlock = () => { ensureAudio(); removeEventListener('pointerdown', unlock); removeEventListener('keydown', unlock); };
  addEventListener('pointerdown', unlock); addEventListener('keydown', unlock);
  function tone(freq, start, dur, gain) {
    const o = actx.createOscillator(), g = actx.createGain(), lp = actx.createBiquadFilter();
    o.type = 'sine'; o.frequency.value = freq;
    lp.type = 'lowpass'; lp.frequency.value = 1400; lp.Q.value = 0.3;
    o.connect(g); g.connect(lp); lp.connect(actx.destination);
    g.gain.setValueAtTime(0.0001, start);
    g.gain.exponentialRampToValueAtTime(gain, start + 0.08);
    g.gain.exponentialRampToValueAtTime(0.0001, start + dur);
    o.start(start); o.stop(start + dur + 0.05);
  }
  function chime() {
    if (!actx || actx.state !== 'running') return;
    const t = actx.currentTime + 0.03;
    tone(587.33, t, 1.8, 0.09); tone(440.00, t + 0.55, 2.4, 0.08); tone(880.00, t + 0.55, 1.2, 0.012);
  }
  function updateSound() {
    if (!actx || !hum) return;
    const E = UI.eng, t = actx.currentTime, on = !document.hidden && (window.__tvMode || 'sandbox') === 'sandbox' && UI.play && !E.done, v = E.plant.v;
    const f = 34 + v * 6.5;
    hum.o1.frequency.setTargetAtTime(f, t, 0.15); hum.o2.frequency.setTargetAtTime(f * 1.5, t, 0.15);
    hum.g.gain.setTargetAtTime(on && v > 0.2 ? Math.min(1, v / 4) * (E.plant.notch > 0 ? 0.03 : 0.014) : 0, t, 0.25);
  }

  // ---------------------------------------------------------------- рисование: общее
  function fit(c) {
    const dpr = Math.min(2, devicePixelRatio || 1), w = c.clientWidth, h = c.clientHeight;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    const x = c.getContext('2d'); x.setTransform(dpr, 0, 0, dpr, 0, 0);
    return [x, w, h];
  }
  const css = () => { const cs = getComputedStyle(document.documentElement); return { fg: cs.getPropertyValue('--fg').trim() || '#000', line: cs.getPropertyValue('--line').trim() || 'rgba(0,0,0,.12)' }; };
  function upper(arr, t) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] <= t) lo = m + 1; else hi = m; } return lo; }
  const fmtT = s => { s = Math.max(0, s); const m = Math.floor(s / 60), x = Math.floor(s % 60); return `${m}:${String(x).padStart(2, '0')}`; };

  // ---------------------------------------------------------------- график скорости (последние 60 с сценария)
  const WIN = 60;
  const SPAN_COL = { bad: 'rgba(214,68,58,0.10)', wet: 'rgba(90,169,214,0.13)', cond: 'rgba(232,155,12,0.08)' };
  function spans(R, i0, i1) {
    const out = []; let a = -1, kind = null;
    const kindOf = i => (R.f0[i] >= 0 || R.f1[i] >= 0 || R.dr[i]) ? 'bad' : R.wx[i] > 0 ? 'wet' : (Math.abs(R.m[i] - 1) > 0.1 || R.gn[i] || Math.abs(R.g[i]) > 0.03) ? 'cond' : null;
    for (let i = i0; i <= i1; i++) {
      const k = i < i1 ? kindOf(i) : null;
      if (k !== kind) { if (kind && a >= 0) out.push([a, i - 1, kind]); a = i; kind = k; }
    }
    return out;
  }
  function drawChart() {
    const cv = $('bChart'); if (!cv.clientWidth) return;
    const [c, w, h] = fit(cv), { fg, line } = css(), R = UI.eng.rows;
    c.clearRect(0, 0, w, h);
    const PL = 28, PR = 4, PT = 6, PB = 16, pw = w - PL - PR, ph = h - PT - PB;
    const tEnd = R.n ? R.t[R.n - 1] : 0, t0 = tEnd - WIN, X = t => PL + (t - t0) / WIN * pw;
    const i0 = Math.max(0, upper(R.t, t0) - 1), i1 = R.n;
    let top = 40;
    for (let i = i0; i < i1; i++) for (const x of [(R.v[i] + 2 * R.sv[i]) * KMH, R.vt[i] * KMH, R.nv[i] * KMH, R.fr[i], R.rr[i]]) if (isFinite(x) && x < 90) top = Math.max(top, x);
    const ymax = Math.min(100, Math.ceil((top + 4) / 10) * 10), Y = val => PT + ph - Math.max(0, Math.min(ymax, val)) / ymax * ph;
    c.font = FONT11; c.strokeStyle = line; c.lineWidth = 1; c.fillStyle = fg; c.textAlign = 'right'; c.textBaseline = 'middle';
    for (let y = 0; y <= ymax; y += ymax > 60 ? 20 : 10) { c.beginPath(); c.moveTo(PL, Y(y) + .5); c.lineTo(w - PR, Y(y) + .5); c.stroke(); c.globalAlpha = .5; c.fillText(y, PL - 6, Y(y)); c.globalAlpha = 1; }
    c.textBaseline = 'top'; c.globalAlpha = .5;
    [[-40, 1 / 3], [-20, 2 / 3], [0, 1]].forEach(([t, f]) => { c.textAlign = f === 1 ? 'right' : 'center'; c.fillText(t === 0 ? 'сейчас' : `${t} с`, PL + f * pw, h - PB + 3); });
    c.globalAlpha = 1;
    if (R.n < 2) return;
    c.save(); c.beginPath(); c.rect(PL, PT, pw, ph); c.clip();
    for (const [a, b, k] of spans(R, i0, i1)) { c.fillStyle = SPAN_COL[k]; c.fillRect(X(R.t[a]), PT, Math.max(2, X(R.t[b]) - X(R.t[a])), ph); }
    c.beginPath();
    for (let i = i0; i < i1; i++) c.lineTo(X(R.t[i]), Y((R.v[i] + 2 * R.sv[i]) * KMH));
    for (let i = i1 - 1; i >= i0; i--) c.lineTo(X(R.t[i]), Y(Math.max(0, R.v[i] - 2 * R.sv[i]) * KMH));
    c.closePath(); c.fillStyle = 'rgba(14,154,167,0.22)'; c.fill();
    const path = (arr, k, col, lw, al, gap) => {
      c.beginPath(); c.strokeStyle = col; c.lineWidth = lw; c.globalAlpha = al; c.lineJoin = 'round';
      let pen = false, tp = -Infinity;
      for (let i = i0; i < i1; i++) {
        const y = arr[i] * k;
        if (!isFinite(y) || (gap && R.t[i] - tp > 0.45)) pen = false;
        tp = R.t[i]; if (!isFinite(y)) continue;
        if (pen) c.lineTo(X(R.t[i]), Y(y)); else { c.moveTo(X(R.t[i]), Y(y)); pen = true; }
      }
      c.stroke(); c.globalAlpha = 1;
    };
    // тележки — тонкий пунктир; модель обычно лежит на истине, поэтому истина — пунктиром сверху
    c.setLineDash([1.5, 3]); path(R.fr, 1, C_BOG, 1.2, .7, true); path(R.rr, 1, C_REAR, 1.2, .7, true); c.setLineDash([]);
    path(R.v, KMH, C_OK, 2.5, 1);
    path(R.nv, KMH, C_NAIVE, 1.5, .95);
    c.setLineDash([5, 5]); path(R.vt, KMH, fg, 1.4, .85); c.setLineDash([]);
    c.restore();
    // soft fade of grid and lines at both sides
    c.save(); c.globalCompositeOperation = 'destination-out';
    const fl = pw * 0.1, fr = pw * 0.06;
    let eg = c.createLinearGradient(PL - 1, 0, PL + fl, 0);
    eg.addColorStop(0, 'rgba(0,0,0,1)'); eg.addColorStop(1, 'rgba(0,0,0,0)');
    c.fillStyle = eg; c.fillRect(PL - 1, 0, fl + 1, PT + ph + 1);
    eg = c.createLinearGradient(w - PR - fr, 0, w, 0);
    eg.addColorStop(0, 'rgba(0,0,0,0)'); eg.addColorStop(1, 'rgba(0,0,0,0.85)');
    c.fillStyle = eg; c.fillRect(w - PR - fr, 0, fr + PR, PT + ph + 1);
    c.restore();
    const k = R.n - 1;
    c.fillStyle = C_OK; c.beginPath(); c.arc(X(R.t[k]), Y(R.v[k] * KMH), 3, 0, Math.PI * 2); c.fill();
  }
  function drawStrip() {
    const cv = $('bStrip'); if (!cv.clientWidth) return;
    const [c, w, h] = fit(cv), R = UI.eng.rows;
    c.clearRect(0, 0, w, h);
    if (R.n < 2) return;
    const PL = 28, PR = 4, pw = w - PL - PR, tEnd = R.t[R.n - 1], t0 = tEnd - WIN, X = t => PL + (t - t0) / WIN * pw;
    let a = Math.max(0, upper(R.t, t0) - 1);
    for (let i = a + 1; i <= R.n; i++) {
      if (i < R.n && R.mode[i] === R.mode[a]) continue;
      const x0 = Math.round(Math.max(PL, X(R.t[a]))), x1 = Math.round(X(R.t[Math.min(i, R.n - 1)]));
      c.fillStyle = ST_COL[SB.MODES[R.mode[a]]] || C_BAD;
      if (x1 > x0) c.fillRect(x0, 0, x1 - x0, h);
      a = i;
    }
  }

  // ---------------------------------------------------------------- карта линии и профиль высот
  let mapOn = false, mapUser = null, mapHover = -1;
  const mapGeo = { sc: 1, ox: 0, oy: 0, idx: [] };
  function setMap(on) { mapOn = on; $('map').style.display = on ? '' : 'none'; layoutBoard(); }
  function toggleMapBody() { mapUser = !$('map').classList.contains('collapsed'); layoutBoard(); }
  $('mapHead').addEventListener('click', toggleMapBody);
  $('mapHead').addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); toggleMapBody(); } });
  function mapIdxAt(ev) {
    const T = UI.eng.track, c = $('mapC'), r = c.getBoundingClientRect(), mx = ev.clientX - r.left, my = ev.clientY - r.top;
    let best = -1, bd = 1e9;
    for (let i = 0; i < T.s.length; i += 2) {
      const x = mapGeo.ox + T.x[i] * mapGeo.sc, y = mapGeo.oy - T.y[i] * mapGeo.sc, d = (x - mx) ** 2 + (y - my) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    return bd < 400 ? T.s[best] : -1;
  }
  $('mapC').addEventListener('mousemove', e => { mapHover = mapIdxAt(e); });
  $('mapC').addEventListener('mouseleave', () => { mapHover = -1; });
  $('profC').addEventListener('mousemove', e => { const r = e.currentTarget.getBoundingClientRect(); mapHover = Math.max(0, Math.min(UI.eng.track.L, (e.clientX - r.left) / r.width * UI.eng.track.L)); });
  $('profC').addEventListener('mouseleave', () => { mapHover = -1; });
  const ZONE_COL = { rain: C_WET, ice: '#9fd3f0', leaves: '#c98b3a' }, ZONE_TXT = { rain: C_WET, ice: C_WET, leaves: '#b0762e' };
  function drawMap() {
    if (!mapOn || $('map').classList.contains('collapsed')) return;
    const [c, w, hh] = fit($('mapC')), { fg } = css(), E = UI.eng, T = E.track, doc = E.trackDoc;
    c.clearRect(0, 0, w, hh);
    let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (let i = 0; i < T.x.length; i++) { x0 = Math.min(x0, T.x[i]); x1 = Math.max(x1, T.x[i]); y0 = Math.min(y0, T.y[i]); y1 = Math.max(y1, T.y[i]); }
    const padX = 68, padY = 20, sc = Math.min((w - padX * 2) / Math.max(1, x1 - x0), (hh - padY * 2) / Math.max(1, y1 - y0));
    const ox = (w - (x1 - x0) * sc) / 2 - x0 * sc, oy = (hh + (y1 - y0) * sc) / 2 + y0 * sc;
    Object.assign(mapGeo, { sc, ox, oy });
    const P = (x, y) => [ox + x * sc, oy - y * sc], PS = s => P(...T.xy(s));
    c.lineCap = 'round'; c.lineJoin = 'round';
    // base line: pathgraph, both directions
    for (const d of doc.dirs) { c.strokeStyle = fg; c.globalAlpha = .28; c.lineWidth = 5; c.beginPath(); for (let i = 0; i < d.x.length; i++) c.lineTo(...P(d.x[i], d.y[i])); c.stroke(); }
    c.globalAlpha = 1;
    // curves: R < 250 m
    c.lineWidth = 5; c.strokeStyle = '#E8C15A';
    for (let i = 0; i < T.s.length - 1; i++) if (T.k[i] > 1 / 250) { c.beginPath(); c.moveTo(...P(T.x[i], T.y[i])); c.lineTo(...P(T.x[i + 1], T.y[i + 1])); c.stroke(); }
    // zones
    const seg = (a, b, col) => { c.strokeStyle = col; c.lineWidth = 3; c.beginPath(); for (let s = Math.max(0, a); s <= Math.min(T.L, b); s += 5) c.lineTo(...PS(s)); c.stroke(); };
    for (const z of sceneZones()) seg(z[0], z[1], ZONE_COL[z[2]] || C_WET);
    if (E.hill) seg(E.hill[0], E.hill[2], '#E0795A');
    for (const z of UI.uz) seg(z[0], Math.min(z[1], E.plant.s + 400), C_NAIVE);
    // stops: route stops with numbers, other map stops small
    c.font = FONT11; c.textBaseline = 'middle';
    for (const p of T.allStops) { if (UI.stops.includes(p)) continue; const [x, y] = PS(p); c.fillStyle = fg; c.globalAlpha = .35; c.beginPath(); c.arc(x, y, 1.8, 0, 7); c.fill(); }
    c.globalAlpha = 1;
    UI.stops.forEach((p, j) => {
      const [x, y] = PS(p); c.fillStyle = fg; c.beginPath(); c.arc(x, y, 3.2, 0, Math.PI * 2); c.fill();
      const up = j % 2 === 0; c.globalAlpha = .8; c.textAlign = 'center'; c.fillText(String(T.stops.indexOf(p) + 1), x, y + (up ? -11 : 11)); c.globalAlpha = 1;
    });
    // line ends
    const [n0, n1] = lineEnds(), [ax, ay] = PS(0), [bx, by] = PS(T.L);
    c.globalAlpha = .75; c.textAlign = ax > bx ? 'left' : 'right'; c.fillText(n0, ax + (ax > bx ? 8 : -8), ay);
    c.textAlign = ax > bx ? 'right' : 'left'; c.fillText(n1, bx + (ax > bx ? -8 : 8), by); c.globalAlpha = 1;
    // tram
    const s = E.plant.s, [tx, ty] = PS(s), [qx, qy] = PS(Math.min(T.L, s + 8)), [rx, ry] = PS(Math.max(0, s - 8)), ang = Math.atan2(qy - ry, qx - rx);
    const pulse = 6 + 3 * Math.abs(Math.sin(performance.now() / 400));
    c.fillStyle = 'rgba(14,154,167,0.25)'; c.beginPath(); c.arc(tx, ty, pulse, 0, Math.PI * 2); c.fill();
    c.save(); c.translate(tx, ty); c.rotate(ang);
    c.fillStyle = C_OK; c.beginPath(); c.moveTo(7, 0); c.lineTo(-5, -4.5); c.lineTo(-3, 0); c.lineTo(-5, 4.5); c.closePath(); c.fill();
    c.restore();
    // hover marker
    const tip = $('mapTip');
    if (mapHover >= 0) {
      const m = mapHover, [hx, hy] = PS(m);
      c.strokeStyle = fg; c.lineWidth = 1.5; c.beginPath(); c.arc(hx, hy, 6, 0, Math.PI * 2); c.stroke();
      const k = Math.max(...[-10, -5, 0, 5, 10].map(d => T.k[T._i(Math.max(0, Math.min(T.L, m + d)))] || 0)), R = k > 1e-6 ? 1 / k : Infinity, gpm = Math.tan(gradeAt(m)) * 1000;
      tip.innerHTML = `<div class="opacity-60 mb-1">${Math.round(m)} м от начала линии</div>
        <div>${R < 250 ? `поворот R ${Math.round(R / 5) * 5} м` : 'прямой участок'}</div>
        <div>${Math.abs(gpm) < 5 ? 'ровно' : (gpm > 0 ? 'подъём ' : 'спуск ') + Math.abs(gpm).toFixed(0) + ' ‰'}, высота ${(zAt(m) + ((doc.origin && doc.origin.z) || 0)).toFixed(1)} м</div>`;
      tip.style.display = 'block';
      tip.style.left = Math.min(w - tip.offsetWidth, Math.max(0, hx - tip.offsetWidth / 2)) + 'px';
      tip.style.top = (hy > hh / 2 ? hy - tip.offsetHeight - 12 : hy + 12) + 'px';
    } else tip.style.display = 'none';
    drawProfile(fg);
  }
  function drawProfile(fg) {
    const [c, w, hh] = fit($('profC')), E = UI.eng, T = E.track, N = 240;
    c.clearRect(0, 0, w, hh);
    let z0 = 1e9, z1 = -1e9; for (let i = 0; i <= N; i++) { const z = zAt(i / N * T.L); z0 = Math.min(z0, z); z1 = Math.max(z1, z); }
    if (z1 - z0 < 1) { z1 += 0.5; z0 -= 0.5; }
    const X = s => s / T.L * w, Y = z => hh - 4 - (z - z0) / (z1 - z0) * (hh - 10);
    for (let i = 0; i < N; i++) {
      const a = i / N * T.L, b = (i + 1) / N * T.L, g = Math.tan(gradeAt((a + b) / 2)) * 1000;
      c.fillStyle = Math.abs(g) < 5 ? 'rgba(128,128,128,0.18)' : g > 0 ? 'rgba(224,121,90,0.35)' : 'rgba(111,168,220,0.35)';
      c.beginPath(); c.moveTo(X(a), hh); c.lineTo(X(a), Y(zAt(a))); c.lineTo(X(b), Y(zAt(b))); c.lineTo(X(b), hh); c.closePath(); c.fill();
    }
    c.strokeStyle = fg; c.globalAlpha = .6; c.lineWidth = 1.2; c.beginPath();
    for (let i = 0; i <= N; i++) { const s = i / N * T.L; c.lineTo(X(s), Y(zAt(s))); } c.stroke(); c.globalAlpha = 1;
    UI.stops.forEach(p => { c.fillStyle = fg; c.globalAlpha = .5; c.fillRect(X(p) - .5, 0, 1, hh); c.globalAlpha = 1; });
    const s = E.plant.s;
    c.fillStyle = C_OK; c.fillRect(X(s) - 1, 0, 2, hh);
    c.beginPath(); c.arc(X(s), Y(zAt(s)), 4, 0, Math.PI * 2); c.fill();
    if (mapHover >= 0) { c.fillStyle = fg; c.globalAlpha = .7; c.fillRect(X(mapHover) - .5, 0, 1, hh); c.globalAlpha = 1; }
    $('mapZ').textContent = `${(zAt(s) + ((E.trackDoc.origin && E.trackDoc.origin.z) || 0)).toFixed(1)} м`;
  }

  // ---------------------------------------------------------------- полоса линии внизу
  const edgeFade = x => Math.min(1, x / 0.1, (1 - x) / 0.1);
  function buildTrackBar() {
    const E = UI.eng, T = E.track, L = T.L, [n0, n1] = lineEnds();
    $('tStops').innerHTML = UI.stops.map(p => `<div class="absolute top-1/2 w-[2px] h-[10px] -mt-[5px] -ml-[1px]" title="${stopName(T, p)}" style="left:${p / L * 100}%;background:var(--fg);opacity:${(.45 * Math.max(.25, edgeFade(p / L))).toFixed(2)}"></div>`).join('');
    // названия концов линии — если рядом с краем есть место (иначе наезжают на подпись крайней остановки)
    const room = p => p / L * innerWidth > 92, ends = (UI.stops.length && room(UI.stops[0]) ? `<span class="absolute whitespace-nowrap" style="left:0;opacity:.45">${n0}</span>` : '')
      + (UI.stops.length && room(L - UI.stops[UI.stops.length - 1]) ? `<span class="absolute whitespace-nowrap" style="right:0;opacity:.45">${n1}</span>` : '');
    $('tNames').innerHTML = ends
      + UI.stops.map(p => `<span class="absolute whitespace-nowrap" style="left:${p / L * 100}%;transform:translateX(-50%);opacity:${(.6 * Math.max(.5, edgeFade(p / L))).toFixed(2)}">ост. ${T.stops.indexOf(p) + 1}</span>`).join('');
    UI.barKey = null;
  }
  // зоны погоды для сцены, карты и полосы: [от, до, погода]; погода на всей линии — от края до края
  function sceneZones() {
    const E = UI.eng, out = [], glob = E.userWeather || E.baseWeather;
    if (glob && glob !== 'dry') out.push([-1e6, 1e6, glob]);
    if (!E.userWeather) for (const z of E.zones) if (z.weather !== 'dry') out.push([z.a, z.b, z.weather]);
    return out;
  }
  const WX_RU = { rain: 'мокрые рельсы', ice: 'наледь', leaves: 'листопад' };
  function syncTrackBar() {
    const E = UI.eng, L = E.track.L, zs = sceneZones(), s = E.plant.s;
    const key = JSON.stringify([zs, E.hill, UI.uz.map(z => [Math.round(z[0]), isFinite(z[1]) ? Math.round(z[1]) : 0])]);
    if (key !== UI.barKey) {
      UI.barKey = key;
      const div = (a, b, col) => { const x0 = Math.max(0, a / L), x1 = Math.min(1, b / L); return x1 > x0 ? `<div style="left:${x0 * 100}%;width:${(x1 - x0) * 100}%;background:${col}"></div>` : ''; };
      let h = zs.map(z => div(z[0], z[1], ZONE_COL[z[2]] || C_WET)).join('');
      if (E.hill) h += div(E.hill[0], E.hill[2], '#E0795A');
      h += UI.uz.map(z => div(z[0], isFinite(z[1]) ? z[1] : s + 200, C_NAIVE)).join('');
      $('tZones').innerHTML = h;
      const k = zs.length ? zs[zs.length - 1][2] : null;
      $('lgWx').style.display = k ? '' : 'none'; if (k) { $('lgWx').textContent = WX_RU[k]; $('lgWx').style.color = ZONE_TXT[k]; }
      $('lgHill').style.display = E.hill ? '' : 'none';
      $('lgUrban').style.display = UI.uz.length ? '' : 'none';
    }
    $('tPos').style.left = Math.max(0, Math.min(100, s / L * 100)) + '%';
  }

  // ---------------------------------------------------------------- табло
  const shown = { st: 'STANDSTILL', pend: null, since: 0 };
  let lastSt = null, lastSub = null, lastPanel = 0;
  const massHist = [];
  function aheadText() {
    const E = UI.eng, s = E.plant.s, parts = [], note = (a, b) => s < a ? `через ${Math.ceil(a - s)} м` : s - 7.55 <= b ? 'сейчас' : null;
    const glob = E.userWeather || (E.baseWeather !== 'dry' ? E.baseWeather : null);
    if (glob && glob !== 'dry') parts.push(`<span style="color:${ZONE_TXT[glob]}">${WX_RU[glob]} на всей линии</span>`);
    if (!E.userWeather) for (const z of E.zones) { const n = note(z.a, z.b); if (n && z.a - s < 800) parts.push(`<span style="color:${ZONE_TXT[z.weather] || C_WET}">${WX_RU[z.weather] || z.weather} ${n}</span>`); }
    if (E.hill) { const up = note(E.hill[0], E.hill[1]), dn = note(E.hill[1], E.hill[2]); if (up && E.hill[0] - s < 800) parts.push(`<span style="color:#E0795A">подъём ${up}</span>`); else if (dn) parts.push(`<span style="color:#6FA8DC">спуск ${dn}</span>`); }
    if (E.gnssOff) parts.push('<span style="color:var(--warn)">застройка: спутников нет</span>');
    return parts.length ? parts.join(', ') : 'путь чистый';
  }
  function updateBoard(force) {
    const E = UI.eng, o = E.last ? E.last.o : null, pl = E.plant, S = E.stats, nowT = performance.now() / 1000;
    const st = o ? SB.MODES[o.mode] : 'DEGRADED';
    if (st !== shown.st) {
      if (shown.pend !== st) { shown.pend = st; shown.since = nowT; }
      else if (nowT - shown.since > 0.5 || force) shown.st = st;
    } else shown.pend = null;
    if (force) shown.st = st;
    const vk = o ? o.v * KMH : 0, sv = o ? o.sigma_v : 0;
    $('bV').textContent = vk.toFixed(1);
    $('bW').textContent = o ? (o.naive_v * KMH).toFixed(1) : '0.0';
    const vUp = (o ? o.v : 0) + 2 * sv, aAvail = Math.max(0.3, Math.min(1.2, (o ? o.mu : 0.25) * G * 0.5));
    $('bBrake').textContent = pl.v > 0.3 ? `≤ ${Math.round(vUp * vUp / (2 * aAvail))} м` : '0 м';
    const slick = (E.cond && E.cond.weatherB[0] !== 'dry') || shown.st === 'SLIP';
    $('bBrake').style.color = slick ? 'var(--wet)' : 'var(--ok)';
    $('bEm').textContent = S.n ? (S.ae / S.n * KMH).toFixed(2) : '0.00';
    $('bEb').textContent = S.n ? (S.aeN / S.n * KMH).toFixed(2) : '0.00';
    $('bS').textContent = o ? o.s.toFixed(1) : '0.0';
    $('bSig').textContent = o ? (2 * o.sigma_s).toFixed(1) : '0.0';
    const sub = o && o.ambiguous ? 'колёса не свидетельствуют: стоим или скользим?' : o && o.slip_all ? 'обе тележки сорвались — скорость ведёт модель'
      : o && o.frozen ? 'обе тележки залипли — скорость по ручке' : o && o.wheels_stale ? 'колёса молчат — скорость по ручке и физике' : ST_SUB[shown.st];
    if (shown.st !== lastSt || sub !== lastSub) {
      lastSt = shown.st; lastSub = sub;
      $('bH').innerHTML = `<span class="mode-ico" style="color:${modeCol(shown.st)}">${ICO[shown.st]}</span><span>${SB.M_RU[shown.st]}</span>`;
      $('bH').title = `Режим модели: ${shown.st}`;
      $('bHsub').textContent = sub; $('bHsub').title = sub;
    }
    const noG = !!E.gnssOff, gc = noG ? 'var(--warn)' : 'var(--ok)';
    $('bG').textContent = noG ? 'нет сигнала' : 'на связи'; $('gPill').style.color = gc; $('gDot').style.background = gc;
    const now = performance.now();
    if (force || now - lastPanel > 150) {
      lastPanel = now;
      renderThink();
      $('bMass').textContent = `${(pl.mass_factor * E.sheet.M_nom / 1000).toFixed(1)} т (имитатор${UI.key === 'free' ? `, ${UI.onboard} пасс.` : ''})${o ? `, k_т ${o.k_t.toFixed(2)}` : ''}`;
      drawMassSpark(pl.mass_factor * E.sheet.M_nom / 1000);
      $('aheadTxt').innerHTML = aheadText();
      const T = E.preset.T;
      $('sbTime').textContent = `${fmtT(E.t)}${isFinite(T) ? ' / ' + fmtT(T) : ''}`;
      $('scBar').style.width = isFinite(T) ? Math.min(100, E.t / T * 100) + '%' : '0';
      syncControls(false);
    }
    syncTrackBar();
  }
  function drawMassSpark(m) {
    massHist.push(m); if (massHist.length > 600) massHist.shift();
    const [c, w, hh] = fit($('massSpark')), { fg } = css();
    c.clearRect(0, 0, w, hh);
    let lo = Math.min(...massHist), hi = Math.max(...massHist);
    if (hi - lo < 1) { hi += 0.5; lo -= 0.5; }
    const X = i => i / 599 * w, Y = v => hh - 2 - (v - lo) / (hi - lo) * (hh - 4);
    c.lineWidth = 1; c.strokeStyle = fg; c.globalAlpha = .4; c.beginPath(); massHist.forEach((a, i) => c.lineTo(X(i + 600 - massHist.length), Y(a))); c.stroke();
    c.globalAlpha = 1;
  }

  // ---------------------------------------------------------------- сцена: состояние из движка
  const scene = TramScene.create($('scene'));
  function sceneState(dt, simDt) {
    const E = UI.eng, pl = E.plant, R = E.rows, n = R.n - 1, s = pl.s;
    for (let b = 0; b < 2; b++) UI.wheelAng[b] += pl.w[b] / 0.35 * simDt;
    // застройка на экране: дома вырастают вокруг вагона, пока GNSS нет, и заканчиваются, когда он вернулся
    if (E.gnssOff && !UI.uzOn) { UI.uz.push([s - 60, Infinity]); UI.uzOn = true; }
    else if (!E.gnssOff && UI.uzOn) { UI.uz[UI.uz.length - 1][1] = s + 40; UI.uzOn = false; }
    UI.uz = UI.uz.filter(z => z[1] > s - 600);
    // пассажиры: приходят на остановки, садятся, пока вагон стоит
    const rush = UI.key === 'free' ? UI.rush : E.cond && E.cond.mass > 1.001, dr = E.driver;
    for (const p of UI.stops) if (p !== UI.boardStop && Math.random() < dt * (rush ? 0.9 : 0.25)) UI.pax.set(p, Math.min(rush ? 26 : 12, (UI.pax.get(p) || 0) + 1));
    const dwelling = dr && dr.state === 'dwell' && dr.t_state > 0.3 && pl.v < 0.05;
    const here = dwelling ? UI.stops.find(p => Math.abs(p - s) < 30) : undefined;
    if (here !== undefined && UI.boardStop !== here) { UI.boardStop = here; UI.boardN = UI.pax.get(here) || 0; UI.alightN = Math.round(UI.onboard * (0.15 + Math.random() * 0.3)); }
    else if (here === undefined && UI.boardStop !== null) { UI.onboard = Math.max(0, Math.min(160, UI.onboard + UI.boardN - UI.alightN)); UI.pax.set(UI.boardStop, 0); UI.boardStop = null; paxMass(E); }
    while (UI.arrivals < E.arrivals.length) { UI.arrivals++; chime(); }
    UI.fade = Math.max(0, UI.fade - dt * 1.6);
    // погода для частиц: под передней тележкой или в зоне впереди (снег начинается до наледи)
    let wx = E.cond ? E.cond.weatherB[0] : 'dry';
    if (wx === 'dry' && !E.userWeather) for (const z of E.zones) if (s > z.a - 150 && s < z.b + 50 && z.weather !== 'dry') { wx = z.weather; break; }
    const x = UI.exp;
    const bog = [0, 1].map(b => {
      const g = x && x.bogies[b], sl = pl.slip(b), spin = sl > 0.03 && pl.w[b] - pl.v > 0.2, skid = sl < -0.03 && pl.v - pl.w[b] > 0.2, locked = skid && pl.w[b] < 0.05;
      const kmh = n >= 0 ? (b ? R.rr[n] : R.fr[n]) : NaN;
      return { spin, skid, locked, col: g ? (BOG_COL[g.code] || '') : '', kmhTxt: isFinite(kmh) ? `${kmh.toFixed(1)} км/ч` : '— км/ч',
        label: (g ? g.st : 'ждёт') + (spin ? ' · буксует' : skid ? ' · юз' : '') };
    });
    return { s: s - HALF, v: pl.v, zAt, gradeAt, notch: pl.notch, mode: shown.st, gnssOff: E.gnssOff, night,
      stops: UI.stops.map(p => ({ p, n: stopName(E.track, p), pax: UI.pax.get(p) || 0, boardP: p === UI.boardStop && dr && dr.dwell ? Math.min(1, Math.max(0, dr.t_state / dr.dwell) * 1.25) : 0 })),
      iz: sceneZones(), uz: UI.uz, wx: wx === 'dry' ? null : wx, wheelAng: UI.wheelAng, bog, fade: UI.fade, paused: !UI.play || E.done, dt };
  }

  // ---------------------------------------------------------------- раскладка (как в исходной странице)
  function layoutBoard(full) {
    const c = $('ctrl'), pr = $('pres'), mp = $('map');
    const presOn = pr.style.display !== 'none';
    if (innerWidth >= 900) {
      c.classList.add('ctrl-side');
      let top = 80;
      if (presOn) { pr.style.top = top + 'px'; top += pr.offsetHeight + 12; }
      mp.style.top = top + 'px';
      if (mapOn) {
        if (full !== false || mapUser !== null) {
          mp.classList.remove('collapsed');
          const fits = top + mp.offsetHeight + 12 + c.offsetHeight + 44 <= innerHeight;
          const col = mapUser === null ? !fits : mapUser;
          mp.classList.toggle('collapsed', col); $('mapHead').setAttribute('aria-expanded', String(!col));
        }
        top += mp.offsetHeight + 12;
      }
      c.style.top = top + 'px';
      c.style.maxHeight = Math.max(240, innerHeight - top - 40) + 'px';
      c.style.overflowY = top + c.scrollHeight + 40 > innerHeight ? 'auto' : '';
    } else { c.classList.remove('ctrl-side'); c.style.top = ''; mp.style.top = ''; c.style.maxHeight = ''; c.style.overflowY = ''; pr.style.top = ''; }
    if (!$('scPop').hidden) placePop();
  }
  function resize() { scene.resize(); layoutBoard(); if (UI.eng) buildTrackBar(); }
  addEventListener('resize', () => { resize(); if (presT < 0) setMap(innerWidth >= 900); });

  // ---------------------------------------------------------------- цикл
  function render(force) {
    if (!UI.eng) return;
    updateBoard(!!force);
    drawChart(); drawStrip(); drawMap();
  }
  let last = performance.now();
  const frameErr = new Set();
  function frame(now) {
    requestAnimationFrame(frame);
    try { frameBody(now); } catch (e) { const k = String(e && e.message || e); if (!frameErr.has(k)) { frameErr.add(k); console.error('кадр песочницы:', e); } }
  }
  function frameBody(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if ((window.__tvMode || 'sandbox') !== 'sandbox' || !UI.eng) { if (hum && actx) hum.g.gain.setTargetAtTime(0, actx.currentTime, 0.1); return; }
    presTick(dt);
    let E = UI.eng;
    const t0 = E.t;
    if (UI.play && !E.done && !document.hidden) E.advance(dt * UI.speed);
    // свободная поездка: в конце линии — новая поездка от начала
    if (UI.key === 'free' && E.driver && E.driver.target === null && E.driver.state === 'dwell' && E.driver.t_state > 12) { load('free', { quiet: true }); E = UI.eng; }
    const simDt = Math.max(0, E.t - t0);
    scene.draw(now, sceneState(dt, simDt));
    render(false);
    metUpdate();
    if (current === 'ros') { bench(); drawRos(); drawBench(); }
    if (current === 'replay') drawReplay();
    updateSound();
  }

  // ---------------------------------------------------------------- старт
  buildScPop();
  if (SHEET) { $('sbSheet').textContent = `${SHEET.path.split('/').pop()} (${SHEET.sheet_sha1})`; $('sbSheet').title = `${SHEET.path}: ${SHEET.label}`; }
  scene.resize();
  mapOn = innerWidth >= 900; $('map').style.display = mapOn ? '' : 'none';
  const want = qs.get('preset') || store.get('tv.preset');
  load(presetOf(want) ? want : 'free', { fade: false });
  UI.fade = 0;
  applyPort();
  requestAnimationFrame(frame);
  // экран прогонов вернул песочницу: пересчитать раскладку (элементы были скрыты)
  window.TV_SANDBOX_SHOW = () => { requestAnimationFrame(() => { resize(); setMap(presT < 0 ? innerWidth >= 900 : mapOn); }); };
  // для проверок (simulator/test/page_test.js)
  window.__tvSandbox = {
    load, state: () => ({ key: UI.key, t: UI.eng.t, n: UI.eng.rows.n, done: UI.eng.done, play: UI.play, summary: UI.eng.summary() }),
    run: (sec, quiet) => { const e = UI.eng; const t1 = e.t + sec; while (!e.done && e.t < t1 - 5e-4) e.advance(Math.min(1, t1 - e.t)); if (!quiet) render(true); return UI.eng.summary(); },
    explain: () => SB.explain(UI.eng), play: on => { UI.play = on; setRunLabel(); }, render: () => render(true),
    think: open => { if (open !== !$('sbThinkMore').hidden) $('thMoreBtn').click(); return $('sbThink').innerText; },
    openInfo, closeInfo, tour: () => startPres(), engine: () => UI.eng,
  };
})();
