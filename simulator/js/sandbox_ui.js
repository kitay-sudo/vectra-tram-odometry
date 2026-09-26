// ---- Экран песочницы: сценарии, сцена, графики, «Что сейчас думает модель» ----
// Данные — TramSandbox.Engine (имитатор + настоящее ядро через связку). Всё рисуется из
// строк движка (20 Гц), поэтому экран и таблица docs/SANDBOX.md считаются одним кодом.
(() => {
  const $ = id => document.getElementById(id);
  const SB = TramSandbox, KMH = 3.6;
  const store = { get: k => { try { return localStorage.getItem(k); } catch (_) { return null; } }, set: (k, v) => { try { localStorage.setItem(k, v); } catch (_) {} } };
  const qs = new URLSearchParams(location.search);
  const C_OK = '#0E9AA7', C_NAIVE = '#E89B0C', C_BOG = '#8a7fd0', C_BAD = '#D6443A', C_WET = '#5AA9D6', C_REAR = '#b07fd0';
  const M_COL = { TRANSITION: '#7E8B98', COAST: '#0E9AA7', TRACTION: '#9AA7B4', BRAKE: '#6B7785', SLIP: '#5AA9D6', STANDSTILL: '#3C4450', DEGRADED: '#D6443A' };
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

  // ---------------------------------------------------------------- подпись порта ядра
  // TramEst.PORT — ядро пакета, с которым сверен JS-порт. Экран прогонов читает
  // replays/index.js: если прогоны посчитаны другим ядром (core_sha1), ядро менялось после
  // сверки, и песочница честно подписывается «упрощённое ядро».
  const PORT = TramEst.PORT, SHEET = window.TV_SHEET;
  function portClaim(kind) {
    const stale = window.TV_PORT_STALE || (SHEET && SHEET.core_sha1 !== PORT.core_sha1 ? SHEET.core_sha1 : null);
    if (kind === 'badge') return stale ? `Песочница · упрощённое ядро (JS-порт от ${PORT.checked})` : `Песочница · ядро пакета (JS-порт, сверен ${PORT.checked})`;
    if (kind === 'note') return stale ? `упрощённое ядро: JS-порт ядра от ${PORT.checked} (${PORT.core_git}), а ядро пакета после этого менялось (${stale})`
      : `настоящее ядро пакета: JS-порт estimator_core.py (${PORT.core_git}), сверен с Python ${PORT.checked}, расхождение ${PORT.dv}`;
    return stale ? `Песочница работает на JS-порте ядра от ${PORT.checked}, а ядро пакета после этого менялось (прогоны комиссии посчитаны ядром ${stale}): поведение может отличаться. Числа пакета — в «Прогоне данных комиссии».`
      : `Оценку в песочнице считает ядро пакета: JS-порт estimator_core.py и связки Runner (сверены с Python ${PORT.checked}: 15 сценариев и запись комиссии, расхождение ${PORT.dv}).`;
  }
  function applyPort() {
    const n = $('sbPort'); if (n) n.textContent = portClaim('note');
    const bs = document.querySelector('header .badge-sub');
    if (bs && (window.__tvMode || 'sandbox') === 'sandbox') bs.textContent = portClaim('badge');
  }
  window.TV_PORT_CLAIM = portClaim; window.TV_PORT_APPLY = applyPort; window.TV_EST_PORT = PORT;

  // ---------------------------------------------------------------- движок
  const UI = { eng: null, play: true, speed: +(qs.get('sbsp') || store.get('tv.sbsp') || 4), win: 60, zoom: 'near', visible: true, keys: {} };
  function load(key) {
    UI.eng = new SB.Engine({ sheet: SHEET, track: window.TV_TRACK, preset: key, seed: +(qs.get('seed') || 7) });
    UI.key = key; store.set('tv.preset', key);
    document.querySelectorAll('[data-preset]').forEach(b => b.setAttribute('aria-checked', b.dataset.preset === key));
    const pr = UI.eng.preset;
    $('sbAbout').innerHTML = `<b>${pr.ru}.</b> ${pr.about} <span class="sb-exp"><b>Модель должна:</b> ${pr.expect}</span>`;
    const syn = !!UI.eng.trackDoc.synthetic;
    $('sbTrackH').textContent = syn ? 'Линия-заглушка (нет js/track.js)' : 'Линия (pathgraph организаторов)';
    $('sbTrackSrc').textContent = syn ? 'Файла js/track.js с pathgraph организаторов нет (его делает simulator/tools/gen_track.py): линия, остановки и профиль придуманы, числа не совпадут с docs/SANDBOX.md.' : 'Точки — остановки карты пакета. Профиль высот и кривые — из pathgraph.';
    $('sbManual').hidden = !pr.manual;
    UI.play = true; syncPlay();
    resetControls();
    render(true);
  }
  function resetControls() {
    document.querySelectorAll('[data-sbwx]').forEach(b => b.setAttribute('aria-checked', b.dataset.sbwx === ''));
    for (const id of ['sbF0', 'sbF1']) $(id).value = '';
    for (const id of ['sbMass', 'sbGnss', 'sbDrop']) $(id).setAttribute('aria-checked', 'false');
    $('sbWsp').setAttribute('aria-checked', String(UI.eng.plant.C.wsp));
    setHandle(0);
  }
  const PRESETS = SB.PRESETS;
  const bar = $('sbPresets');
  PRESETS.forEach((p, i) => {
    const b = document.createElement('button');
    b.setAttribute('role', 'radio'); b.dataset.preset = p.key; b.textContent = p.ru; b.title = p.about;
    b.addEventListener('click', () => load(p.key));
    bar.appendChild(b);
  });

  // ---------------------------------------------------------------- управление
  function syncPlay() { $('sbPlay').textContent = UI.eng && UI.eng.done ? 'Заново' : UI.play ? 'Пауза' : 'Пуск'; $('sbPlay').setAttribute('aria-pressed', String(!UI.play)); }
  $('sbPlay').addEventListener('click', () => { if (UI.eng.done) { load(UI.key); return; } UI.play = !UI.play; syncPlay(); });
  $('sbRestart').addEventListener('click', () => load(UI.key));
  document.querySelectorAll('[data-sbsp]').forEach(b => {
    b.setAttribute('aria-checked', String(+b.dataset.sbsp === UI.speed));
    b.addEventListener('click', () => { UI.speed = +b.dataset.sbsp; store.set('tv.sbsp', UI.speed); document.querySelectorAll('[data-sbsp]').forEach(x => x.setAttribute('aria-checked', String(x === b))); });
  });
  document.querySelectorAll('[data-sbwin]').forEach(b => b.addEventListener('click', () => { UI.win = +b.dataset.sbwin; document.querySelectorAll('[data-sbwin]').forEach(x => x.setAttribute('aria-checked', String(x === b))); render(true); }));
  document.querySelectorAll('[data-sbzoom]').forEach(b => b.addEventListener('click', () => { UI.zoom = b.dataset.sbzoom; document.querySelectorAll('[data-sbzoom]').forEach(x => x.setAttribute('aria-checked', String(x === b))); }));
  document.querySelectorAll('[data-sbwx]').forEach(b => b.addEventListener('click', () => { UI.eng.userWeather = b.dataset.sbwx || null; document.querySelectorAll('[data-sbwx]').forEach(x => x.setAttribute('aria-checked', String(x === b))); }));
  const FAULT = { '': null, zero: { kind: 'zero' }, stuck: { kind: 'stuck' }, dropout: { kind: 'dropout' }, noise: { kind: 'noise', sigma_kmh: 0.9 }, outliers: { kind: 'outliers', p: 0.05 } };
  $('sbF0').addEventListener('change', e => { UI.eng.userFaults[0] = FAULT[e.target.value]; });
  $('sbF1').addEventListener('change', e => { UI.eng.userFaults[1] = FAULT[e.target.value]; });
  const sw = (id, fn) => $(id).addEventListener('click', () => { const on = $(id).getAttribute('aria-checked') !== 'true'; $(id).setAttribute('aria-checked', String(on)); fn(on); });
  sw('sbMass', on => { UI.eng.userMass = on ? 1.3 : null; });
  sw('sbGnss', on => { UI.eng.userGnssOff = on; });
  sw('sbDrop', on => { UI.eng.userDrop = on; });
  sw('sbWsp', on => { UI.eng.plant.C.wsp = on; });
  function setHandle(n) {
    n = Math.max(-15, Math.min(15, Math.round(n)));
    if (UI.eng) UI.eng.setManual(n);
    $('sbHandle').value = n; $('sbHandleV').textContent = (n > 0 ? '+' : '') + n;
  }
  $('sbHandle').addEventListener('input', e => setHandle(+e.target.value));
  document.querySelectorAll('[data-sbh]').forEach(b => b.addEventListener('click', () => setHandle(+b.dataset.sbh)));
  addEventListener('keydown', e => {
    if ((window.__tvMode || 'sandbox') !== 'sandbox' || (e.target.closest && e.target.closest('input, select, textarea'))) return;
    if (e.key === 'Escape') { closeHelp(); return; }
    if (!UI.eng || !UI.eng.preset.manual) { if (e.key === ' ') { e.preventDefault(); $('sbPlay').click(); } return; }
    const h = UI.eng.manualNotch;
    if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp' || e.key === 'ц' || e.key === 'Ц') { e.preventDefault(); setHandle(h + 1); }
    if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown' || e.key === 'ы' || e.key === 'Ы') { e.preventDefault(); setHandle(h - 1); }
    if (e.key === ' ') { e.preventDefault(); setHandle(0); }
  });

  // ---------------------------------------------------------------- справка
  function openHelp() { $('sbHelpModal').hidden = false; $('helpBtn').setAttribute('aria-pressed', 'true'); $('sbHelpClose').focus(); }
  function closeHelp() { $('sbHelpModal').hidden = true; $('helpBtn').setAttribute('aria-pressed', 'false'); }
  $('helpBtn').addEventListener('click', () => { if ($('sbHelpModal').hidden) openHelp(); else closeHelp(); });
  $('sbHelpBtn').addEventListener('click', openHelp);
  $('sbHelpClose').addEventListener('click', closeHelp);
  $('sbHelpModal').addEventListener('click', e => { if (e.target === $('sbHelpModal')) closeHelp(); });

  // ---------------------------------------------------------------- тема
  const mq = matchMedia('(prefers-color-scheme: dark)');
  let night = false;
  function setTheme(t) {
    document.documentElement.dataset.theme = t; night = t === 'dark';
    $('icoSun').style.display = night ? 'none' : ''; $('icoMoon').style.display = night ? '' : 'none';
    $('themeBtn').setAttribute('aria-pressed', String(night)); $('themeTxt').textContent = night ? 'Ночь' : 'День';
    store.set('vectra-theme', t);
  }
  const savedTheme = store.get('vectra-theme');
  setTheme(savedTheme === 'dark' || savedTheme === 'light' ? savedTheme : (mq.matches ? 'dark' : 'light'));
  $('themeBtn').addEventListener('click', () => setTheme(night ? 'light' : 'dark'));

  // ---------------------------------------------------------------- рисование: общее
  const FONT = '11px system-ui, "Segoe UI", Roboto, sans-serif';
  function fit(c) {
    const dpr = Math.min(2, devicePixelRatio || 1), w = c.clientWidth, h = c.clientHeight;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    const x = c.getContext('2d'); x.setTransform(dpr, 0, 0, dpr, 0, 0);
    return [x, w, h];
  }
  const css = () => { const cs = getComputedStyle(document.documentElement); return { fg: cs.getPropertyValue('--fg').trim() || '#000', bg: cs.getPropertyValue('--bg').trim() || '#fff', line: cs.getPropertyValue('--line').trim() || 'rgba(0,0,0,.12)' }; };
  const fmtT = s => { s = Math.max(0, s); const m = Math.floor(s / 60), x = Math.floor(s % 60); return `${m}:${String(x).padStart(2, '0')}`; };
  const f = (x, d = 2) => (x === null || x === undefined || !isFinite(x)) ? '—' : x.toFixed(d).replace('.', ',').replace(/^-/, '−');
  function upper(arr, t) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] <= t) lo = m + 1; else hi = m; } return lo; }
  function niceCeil(x) { if (!(x > 0)) return 0; const e = Math.pow(10, Math.floor(Math.log10(x))), m = x / e; return (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10) * e; }
  function fmtNum(v) { const a = Math.abs(v); return (a >= 10 ? v.toFixed(0) : a >= 1 ? v.toFixed(1).replace(/\.0$/, '') : v.toFixed(2)).replace('.', ',').replace(/^-/, '−'); }
  function viewRange() {
    const R = UI.eng.rows, tEnd = R.n ? R.t[R.n - 1] : 0, t0 = R.n ? R.t[0] : 0;
    if (UI.win === 0) return [t0, Math.max(tEnd, t0 + 30)];
    return tEnd - UI.win < t0 ? [t0, t0 + UI.win] : [tEnd - UI.win, tEnd];
  }
  function strokeSeries(c, T, Y, i0, i1, X, Yp, stride, gap = 0.35) {
    c.beginPath(); let pen = false, tp = -Infinity;
    for (let i = i0; i < i1; i += stride) {
      const y = Y[i];
      if (!isFinite(y) || !isFinite(T[i]) || T[i] - tp > gap * stride + 0.3) { pen = false; }
      tp = T[i];
      if (!isFinite(y)) continue;
      const px = X(T[i]), py = Yp(y);
      if (pen) c.lineTo(px, py); else { c.moveTo(px, py); pen = true; }
    }
    c.stroke();
  }
  function timeAxis(c, t0, t1, X, y, w) {
    const span = t1 - t0, steps = [5, 10, 30, 60, 120, 300, 600];
    const st = steps.find(s => span / s <= 7) || 1200;
    c.textAlign = 'center'; c.textBaseline = 'top'; c.globalAlpha = .55;
    for (let t = Math.ceil(t0 / st) * st; t <= t1; t += st) { const x = X(t); if (t >= 0 && x > 30 && x < w - 16) c.fillText(fmtT(t), x, y); }
    c.globalAlpha = 1;
  }
  // полосы условий сценария: отказы и пропуски (красные), скользкий рельс (синие)
  function spans(R, i0, i1) {
    const out = []; let a = -1, kind = null;
    const kindOf = i => (R.f0[i] >= 0 || R.f1[i] >= 0 || R.dr[i]) ? 'bad' : R.wx[i] > 0 ? 'wet' : (R.m[i] !== 1 || R.gn[i] || Math.abs(R.g[i]) > 0.03) ? 'cond' : null;
    for (let i = i0; i <= i1; i++) {
      const k = i < i1 ? kindOf(i) : null;
      if (k !== kind) { if (kind && a >= 0) out.push([a, i - 1, kind]); a = i; kind = k; }
    }
    return out;
  }
  const SPAN_COL = { bad: 'rgba(214,68,58,0.12)', wet: 'rgba(90,169,214,0.16)', cond: 'rgba(232,155,12,0.10)' };

  // ---------------------------------------------------------------- графики
  function drawSpeed() {
    const [c, w, h] = fit($('sbSpeed')), { fg, line } = css(), R = UI.eng.rows;
    c.clearRect(0, 0, w, h);
    const PL = 34, PR = 8, PT = 8, PB = 18, pw = w - PL - PR, ph = h - PT - PB;
    const [t0, t1] = viewRange(), X = t => PL + (t - t0) / Math.max(1e-6, t1 - t0) * pw;
    const i0 = Math.max(0, upper(R.t, t0) - 1), i1 = Math.min(R.n, upper(R.t, t1) + 1);
    let top = 20;
    // масштаб — по истине, модели, базе и тележкам; полоса ±2σ обрезается (при «стоим или скользим?» она до ±40 км/ч)
    for (let i = i0; i < i1; i++) for (const x of [R.v[i] * KMH, R.vt[i] * KMH, R.nv[i] * KMH, R.fr[i], R.rr[i]]) if (isFinite(x) && x < 90) top = Math.max(top, x);
    const ymax = Math.ceil((top + 3) / 10) * 10, Y = k => PT + ph - Math.min(ymax, Math.max(0, k)) / ymax * ph;
    c.font = FONT; c.strokeStyle = line; c.lineWidth = 1; c.fillStyle = fg; c.textAlign = 'right'; c.textBaseline = 'middle';
    for (let y = 0; y <= ymax; y += ymax > 60 ? 20 : 10) { c.beginPath(); c.moveTo(PL, Y(y) + .5); c.lineTo(w - PR, Y(y) + .5); c.stroke(); c.globalAlpha = .55; c.fillText(y, PL - 6, Y(y)); c.globalAlpha = 1; }
    timeAxis(c, t0, t1, X, h - PB + 4, w);
    if (!R.n) return;
    c.save(); c.beginPath(); c.rect(PL, PT - 2, pw, ph + 4); c.clip();
    for (const [a, b, k] of spans(R, i0, i1)) { c.fillStyle = SPAN_COL[k]; c.fillRect(X(R.t[a]), PT, Math.max(2, X(R.t[b]) - X(R.t[a])), ph); }
    const stride = Math.max(1, Math.floor((i1 - i0) / (pw * 1.5)));
    c.beginPath(); let st = false;
    for (let i = i0; i < i1; i += stride) { const px = X(R.t[i]), py = Y((R.v[i] + 2 * R.sv[i]) * KMH); if (st) c.lineTo(px, py); else { c.moveTo(px, py); st = true; } }
    for (let i = i1 - 1; i >= i0; i -= stride) c.lineTo(X(R.t[i]), Y((R.v[i] - 2 * R.sv[i]) * KMH));
    c.closePath(); c.fillStyle = 'rgba(14,154,167,0.20)'; c.fill();
    c.lineJoin = 'round';
    c.lineWidth = 1; c.globalAlpha = .6; c.setLineDash([2, 3]);
    c.strokeStyle = C_BOG; strokeSeries(c, R.t, R.fr, i0, i1, X, Y, stride);
    c.strokeStyle = C_REAR; strokeSeries(c, R.t, R.rr, i0, i1, X, Y, stride);
    c.setLineDash([]); c.globalAlpha = 1;
    c.strokeStyle = C_NAIVE; c.lineWidth = 1.5; strokeSeries(c, R.t, R.nv, i0, i1, X, k => Y(k * KMH), stride);
    c.strokeStyle = C_OK; c.lineWidth = 2.2; strokeSeries(c, R.t, R.v, i0, i1, X, k => Y(k * KMH), stride);
    c.strokeStyle = fg; c.lineWidth = 1.4; c.globalAlpha = .9; c.setLineDash([5, 4]); strokeSeries(c, R.t, R.vt, i0, i1, X, k => Y(k * KMH), stride); c.setLineDash([]); c.globalAlpha = 1;
    c.restore();
  }
  function drawStrip() {
    const [c, w, h] = fit($('sbStrip')), R = UI.eng.rows;
    c.clearRect(0, 0, w, h);
    if (!R.n) return;
    const PL = 34, PR = 8, pw = w - PL - PR, [t0, t1] = viewRange(), X = t => PL + (t - t0) / Math.max(1e-6, t1 - t0) * pw;
    const i0 = Math.max(0, upper(R.t, t0) - 1), i1 = Math.min(R.n, upper(R.t, t1) + 1);
    let a = i0;
    for (let i = i0 + 1; i <= i1; i++) {
      if (i < i1 && R.mode[i] === R.mode[a]) continue;
      c.fillStyle = M_COL[SB.MODES[R.mode[a]]] || C_BAD;
      const x0 = Math.max(PL, X(R.t[a])), x1 = Math.min(PL + pw, X(R.t[Math.min(i, R.n - 1)]));
      if (x1 > x0) c.fillRect(x0, 0, Math.max(1, x1 - x0), h);
      a = i;
    }
  }
  // ошибка относительно истины: модель (с полосой ±2σ) и база «только колесо»
  function drawErr(id, A, B, band, unitK, minR) {
    const [c, w, h] = fit($(id)), { fg, line } = css(), R = UI.eng.rows;
    c.clearRect(0, 0, w, h);
    const PL = 34, PR = 8, PT = 6, PB = 16, pw = w - PL - PR, ph = h - PT - PB;
    const [t0, t1] = viewRange(), X = t => PL + (t - t0) / Math.max(1e-6, t1 - t0) * pw;
    const i0 = Math.max(0, upper(R.t, t0) - 1), i1 = Math.min(R.n, upper(R.t, t1) + 1);
    const vals = [];
    for (let i = i0; i < i1; i++) { for (const g of [A, B]) { const x = g(i) * unitK; if (isFinite(x)) vals.push(Math.abs(x)); } }
    vals.sort((p, q) => p - q);
    const p98 = vals.length ? vals[Math.min(vals.length - 1, Math.floor(vals.length * 0.98))] : 0;
    const Rr = Math.max(minR, niceCeil(p98 * 1.15));
    const Y = v => PT + ph / 2 - Math.max(-Rr, Math.min(Rr, v)) / Rr * ph / 2;
    c.font = FONT; c.fillStyle = fg; c.strokeStyle = line; c.lineWidth = 1; c.textAlign = 'right'; c.textBaseline = 'middle';
    for (const v of [-Rr, 0, Rr]) { c.beginPath(); c.moveTo(PL, Y(v) + .5); c.lineTo(w - PR, Y(v) + .5); c.stroke(); c.globalAlpha = .55; c.fillText((v > 0 ? '+' : '') + fmtNum(v), PL - 6, Y(v)); c.globalAlpha = 1; }
    timeAxis(c, t0, t1, X, h - PB + 3, w);
    if (!R.n) return;
    c.save(); c.beginPath(); c.rect(PL, PT, pw, ph); c.clip();
    for (const [a, b, k] of spans(R, i0, i1)) { c.fillStyle = SPAN_COL[k]; c.fillRect(X(R.t[a]), PT, Math.max(2, X(R.t[b]) - X(R.t[a])), ph); }
    const stride = Math.max(1, Math.floor((i1 - i0) / (pw * 1.5)));
    c.beginPath(); let s0 = false;
    for (let i = i0; i < i1; i += stride) { const b = band(i) * 2 * unitK; if (!isFinite(b)) continue; const px = X(R.t[i]); if (s0) c.lineTo(px, Y(b)); else { c.moveTo(px, Y(b)); s0 = true; } }
    for (let i = i1 - 1; i >= i0; i -= stride) { const b = band(i) * 2 * unitK; if (!isFinite(b)) continue; c.lineTo(X(R.t[i]), Y(-b)); }
    c.closePath(); c.fillStyle = 'rgba(14,154,167,0.16)'; c.fill();
    const series = (g, col, lw) => { c.strokeStyle = col; c.lineWidth = lw; c.beginPath(); let p = false; for (let i = i0; i < i1; i += stride) { const v = g(i); if (!isFinite(v)) { p = false; continue; } const px = X(R.t[i]), py = Y(v * unitK); if (p) c.lineTo(px, py); else { c.moveTo(px, py); p = true; } } c.stroke(); };
    c.lineJoin = 'round';
    series(B, C_NAIVE, 1.4); series(A, C_OK, 1.8);
    c.restore();
  }

  // ---------------------------------------------------------------- план линии
  function drawTrack() {
    const [c, w, h] = fit($('sbTrack')), { fg } = css(), E = UI.eng, T = E.track;
    c.clearRect(0, 0, w, h);
    const R = E.rows, k = R.n - 1, s0 = E.route0();
    const sT = E.plant.s, sM = k >= 0 ? s0 + R.s[k] : s0, sN = k >= 0 ? s0 + R.ns[k] : s0;
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    const acc = (x, y) => { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); };
    if (UI.zoom === 'near') { const [cx, cy] = T.xy(sT), r = 180; acc(cx - r, cy - r); acc(cx + r, cy + r); }
    else for (let i = 0; i < T.x.length; i += 4) acc(T.x[i], T.y[i]);
    const pad = 14, sc = Math.min((w - 2 * pad) / Math.max(1, x1 - x0), (h - 2 * pad) / Math.max(1, y1 - y0));
    const ox = (w - (x1 - x0) * sc) / 2 - x0 * sc, oy = (h + (y1 - y0) * sc) / 2 + y0 * sc;
    const P = (x, y) => [ox + x * sc, oy - y * sc];
    c.lineCap = 'round'; c.lineJoin = 'round';
    // pathgraph организаторов: оба направления
    for (const d of E.trackDoc.dirs) {
      c.strokeStyle = fg; c.globalAlpha = .16; c.lineWidth = 6; c.beginPath();
      for (let i = 0; i < d.x.length; i++) { const [px, py] = P(d.x[i], d.y[i]); if (i) c.lineTo(px, py); else c.moveTo(px, py); }
      c.stroke(); c.globalAlpha = 1;
    }
    // зоны скользкого рельса и подъём
    const seg = (a, b, col, lw) => { c.strokeStyle = col; c.lineWidth = lw; c.beginPath(); for (let s = Math.max(0, a), first = true; s <= Math.min(T.L, b); s += 4, first = false) { const [px, py] = P(...T.xy(s)); if (first) c.moveTo(px, py); else c.lineTo(px, py); } c.stroke(); };
    for (const z of E.zones) seg(z.a, z.b, z.weather === 'ice' ? '#9fd3f0' : z.weather === 'leaves' ? '#c98b3a' : C_WET, 5);
    if (E.hill) seg(E.hill[0], E.hill[2], 'rgba(232,155,12,.8)', 5);
    // остановки: точки карты пакета; маршрут водителя — крупнее
    c.font = FONT; c.textBaseline = 'bottom'; c.textAlign = 'center';
    for (const s of T.allStops) { const [px, py] = P(...T.xy(s)); const on = E.route.includes(s) || Math.abs(s - s0) < 1; c.fillStyle = fg; c.globalAlpha = on ? .9 : .35; c.beginPath(); c.arc(px, py, on ? 3.5 : 2.2, 0, 7); c.fill(); }
    c.globalAlpha = 1;
    // путь по модели и по колёсам — вдоль той же линии (карта), истина — кольцо
    const dot = (s, col, r) => { const [px, py] = P(...T.xy(s)); c.fillStyle = col; c.beginPath(); c.arc(px, py, r, 0, 7); c.fill(); };
    dot(sN, C_NAIVE, 4); dot(sM, C_OK, 5);
    { const [px, py] = P(...T.xy(sT)); c.strokeStyle = fg; c.lineWidth = 1.6; c.beginPath(); c.arc(px, py, 8, 0, 7); c.stroke(); }
    const want = (w - 2 * pad) * 0.25 / sc, L = niceCeil(want) / (niceCeil(want) / want > 1.8 ? 2 : 1);
    c.strokeStyle = fg; c.fillStyle = fg; c.lineWidth = 1.5; c.globalAlpha = .7;
    c.beginPath(); c.moveTo(pad, h - pad); c.lineTo(pad + L * sc, h - pad); c.stroke();
    c.textAlign = 'left'; c.fillText(`${L >= 1000 ? (L / 1000) + ' км' : L + ' м'}`, pad, h - pad - 3);
    c.textAlign = 'right'; c.textBaseline = 'top'; c.fillText('С ↑', w - 6, 6); c.globalAlpha = 1;
  }

  // ---------------------------------------------------------------- сцена (вид сбоку)
  const wheelAng = [0, 0];
  let lastSceneT = null;
  function drawScene() {
    const [c, w, h] = fit($('sbScene')), { fg, bg } = css(), E = UI.eng, pl = E.plant, cond = E.cond || {};
    c.clearRect(0, 0, w, h);
    const ppm = Math.max(9, Math.min(30, w / 30));       // пикселей на метр
    const railY = h - 26, cx = w * 0.5;
    const tilt = -Math.atan(Math.tan(pl.grade) * 5);       // уклон на экране ×5, чтобы было видно
    // центр экрана — середина кузова: передняя тележка (путь s, base_link) справа, задняя — на 7,55 м левее
    const wx = cond.weather || 'dry', s = pl.s - pl.C.bogie_base / 2;
    const R = E.rows, k = R.n - 1, dt = lastSceneT === null ? 0 : Math.max(0, Math.min(1, E.t - lastSceneT));
    lastSceneT = E.t;
    const exp = E.last ? SB.explain(E) : null;
    const Lb = 15 * ppm, bodyH = 2.4 * ppm, xF = pl.C.bogie_base / 2 * ppm, xR = -pl.C.bogie_base / 2 * ppm, wr = Math.max(5, 0.36 * ppm);
    const bodyBottom = -wr * 2 - 7, by = bodyBottom - bodyH;
    // --- в системе рельса (повёрнута на уклон)
    c.save(); c.translate(cx, railY); c.rotate(tilt);
    c.strokeStyle = fg; c.globalAlpha = .14; c.lineWidth = 2;
    for (let q = Math.floor((s - w / ppm) / 25) * 25; q < s + w / ppm; q += 25) { const x = (q - s) * ppm; c.beginPath(); c.moveTo(x, 0); c.lineTo(x, -h); c.stroke(); }
    c.globalAlpha = .12; c.lineWidth = 1; c.beginPath(); c.moveTo(-w, -h + 30); c.lineTo(w, -h + 30); c.stroke();
    c.globalAlpha = .25;
    for (let q = Math.floor((s - w / ppm) / 0.75) * 0.75; q < s + w / ppm; q += 0.75) { const x = (q - s) * ppm; c.beginPath(); c.moveTo(x, 2); c.lineTo(x, 7); c.stroke(); }
    // рельс раскрашен по месту: зона скользкого рельса видна, и задняя тележка въезжает в неё позже
    const wxCol = x => x === 'dry' ? fg : x === 'ice' ? '#9fd3f0' : x === 'leaves' ? '#c98b3a' : C_WET;
    c.globalAlpha = 1; c.lineWidth = 3;
    for (let x = -w, prev = null, x0 = -w; x <= w + 4; x += 4) {
      const cur = x <= w ? E.weatherAt(s + x / ppm) : null;
      if (prev !== null && cur !== prev) { c.strokeStyle = wxCol(prev); c.beginPath(); c.moveTo(x0, 0); c.lineTo(Math.min(x, w), 0); c.stroke(); x0 = x; }
      if (prev === null) x0 = x;
      prev = cur;
    }
    // пантограф и кузов (15 м, шкворни тележек в 7,55 м; передняя — справа, едет вправо)
    c.strokeStyle = fg; c.globalAlpha = .55; c.lineWidth = 1.2; c.beginPath(); c.moveTo(-12, by); c.lineTo(0, -h + 31); c.lineTo(12, by); c.stroke(); c.globalAlpha = 1;
    c.fillStyle = night ? '#2a3038' : '#e9edf1'; c.strokeStyle = fg; c.lineWidth = 1.2;
    c.beginPath(); if (c.roundRect) c.roundRect(-Lb / 2, by, Lb, bodyH, 6); else c.rect(-Lb / 2, by, Lb, bodyH); c.fill(); c.stroke();
    c.fillStyle = C_OK; c.globalAlpha = .9; c.fillRect(-Lb / 2, by + bodyH * 0.74, Lb, bodyH * 0.1); c.globalAlpha = 1;
    c.fillStyle = night ? 'rgba(255,212,140,.55)' : 'rgba(40,60,80,.22)';
    for (let i = 0; i < 6; i++) c.fillRect(-Lb / 2 + 10 + i * (Lb - 20) / 6, by + 5, (Lb - 20) / 6 - 6, bodyH * 0.4);
    const bog = [];
    [[xF, 0], [xR, 1]].forEach(([bx, b]) => {
      wheelAng[b] += pl.w[b] * dt / 0.35;
      const sl = pl.slip(b), spin = sl > 0.03 && pl.w[b] - pl.v > 0.2, skid = sl < -0.03 && pl.v - pl.w[b] > 0.2, locked = skid && pl.w[b] < 0.05;
      bog.push({ bx, b, spin, skid, locked });
      c.fillStyle = fg; c.globalAlpha = .75; c.fillRect(bx - 1.1 * ppm, bodyBottom + 1, 2.2 * ppm, 5); c.globalAlpha = 1;
      for (const dx of [-0.9, 0.9]) {
        const x = bx + dx * ppm, y = -wr;
        c.fillStyle = bg; c.strokeStyle = locked ? C_BAD : spin ? C_WET : skid ? C_NAIVE : fg; c.lineWidth = spin || skid ? 2.6 : 1.4;
        c.beginPath(); c.arc(x, y, wr, 0, 7); c.fill(); c.stroke();
        c.lineWidth = 1.2;
        for (let q = 0; q < 3; q++) { const a = wheelAng[b] + q * 2.094; c.beginPath(); c.moveTo(x, y); c.lineTo(x + Math.cos(a) * wr * 0.8, y + Math.sin(a) * wr * 0.8); c.stroke(); }
        if (skid) { c.fillStyle = C_NAIVE; c.globalAlpha = .75; for (let q = 0; q < 6; q++) c.fillRect(x - wr - 3 - q * 3.5, -3 - ((q * 7 + Math.floor(E.t * 20)) % 5), 2, 2); c.globalAlpha = 1; }
        if (spin) { c.strokeStyle = C_WET; c.globalAlpha = .6; c.lineWidth = 2; c.beginPath(); c.arc(x, y, wr + 4, 0, 7); c.stroke(); c.globalAlpha = 1; }
      }
    });
    c.restore();
    // --- подписи тележек (экранные координаты): показание датчика и решение модели
    c.font = '12px system-ui, "Segoe UI", Roboto, sans-serif'; c.textAlign = 'center'; c.textBaseline = 'bottom';
    for (const g of bog) {
      const px = cx + Math.cos(tilt) * g.bx, py = railY + Math.sin(tilt) * g.bx + by - 6;
      const kmh = k >= 0 ? (g.b ? R.rr[k] : R.fr[k]) : NaN;
      const code = exp ? exp.bogies[g.b].code : 'wait';
      // на узком экране подписи расходятся от середины: задняя — влево, передняя — вправо
      const nar = w < 700, ax = nar ? (g.b ? px + 12 : px - 12) : px;
      c.textAlign = nar ? (g.b ? 'right' : 'left') : 'center';
      c.fillStyle = fg; c.fillText(`${g.b ? 'задняя' : 'передняя'}: ${isFinite(kmh) ? f(kmh, 1) + ' км/ч' : '—'}`, ax, py - 14);
      if (exp) { c.fillStyle = BOG_COL[code] || fg; c.font = '11px system-ui, "Segoe UI", Roboto, sans-serif'; c.fillText(exp.bogies[g.b].st + (g.spin ? (nar ? ' · буксует' : ' · колесо буксует') : g.skid ? (g.locked ? (nar ? ' · юз' : ' · колесо стоит, юз') : ' · юз') : ''), ax, py); c.font = '12px system-ui, "Segoe UI", Roboto, sans-serif'; }
      // отказ датчика (правда имитатора) — красным над подписью
      const fl = (cond.faults || [])[g.b], drop = cond.drop;
      if (fl || drop) { c.fillStyle = C_BAD; c.font = '11px system-ui, "Segoe UI", Roboto, sans-serif'; c.fillText('⚠ датчик: ' + (drop ? 'нет сообщений' : FAULT_RU[fl.kind]), ax, py - 30); c.font = '12px system-ui, "Segoe UI", Roboto, sans-serif'; }
    }
    // --- погода
    if (wx === 'rain' || wx === 'ice' || wx === 'leaves') {
      c.save(); c.globalAlpha = wx === 'rain' ? .35 : .6;
      const n = Math.round(w / 9), tt = E.t;
      for (let i = 0; i < n; i++) {
        const x0 = ((i * 97.3 + tt * (wx === 'rain' ? 40 : 12) * 7) % (w + 40)) - 20, y0 = ((i * 53.7 + tt * (wx === 'rain' ? 260 : 30)) % railY);
        if (wx === 'rain') { c.strokeStyle = C_WET; c.lineWidth = 1; c.beginPath(); c.moveTo(x0, y0); c.lineTo(x0 - 3, y0 + 9); c.stroke(); }
        else if (wx === 'ice') { c.fillStyle = '#cfe6f5'; c.beginPath(); c.arc(x0, y0, 1.6, 0, 7); c.fill(); }
        else if (i % 3 === 0) { c.fillStyle = i % 2 ? '#c98b3a' : '#a4652a'; c.fillRect(x0, y0, 4, 3); }
      }
      c.restore();
    }
    // --- табличка: истина, модель, колёса; справа — GNSS и условия
    c.font = '12px system-ui, "Segoe UI", Roboto, sans-serif'; c.textAlign = 'left'; c.textBaseline = 'top';
    const o = E.last ? E.last.o : null;
    [[fg, `истина (имитатор): ${f(pl.v * KMH, 1)} км/ч`],
     [C_OK, o ? `модель: ${f(o.v * KMH, 1)} ± ${f(2 * o.sigma_v * KMH, 1)} км/ч` : 'модель: —'],
     [C_NAIVE, o ? `только колесо: ${f(o.naive_v * KMH, 1)} км/ч` : 'только колесо: —']].forEach(([col, t], i) => { c.fillStyle = col; c.fillText(t, 8, 6 + i * 16); });
    c.textAlign = 'right'; c.fillStyle = E.gnssOff ? C_BAD : fg; c.globalAlpha = .85;
    const narrow = w < 560;
    c.fillText(E.gnssOff ? (narrow ? 'GNSS: нет (не нужен)' : 'GNSS: нет сигнала — модели не нужен') : E.t < 3 ? (narrow ? 'GNSS: выставка' : 'GNSS: выставка положения (первые 3 с)') : (narrow ? 'GNSS: не читается' : 'GNSS: есть, но модель его не читает'), w - 8, 6);
    c.fillStyle = fg; c.globalAlpha = .6;
    c.fillText(`ручка ${pl.notch > 0 ? '+' : ''}${pl.notch} · уклон ${f(1000 * Math.tan(pl.grade), 0)} ‰`, w - 8, 22);
    c.fillText(`${SB_WX[wx] || wx} · ${Math.round(pl.mass_factor * E.sheet.M_nom / 1000)} т`, w - 8, 38);
    c.globalAlpha = 1;
  }
  const SB_WX = { dry: 'сухо', rain: 'дождь', leaves: 'листопад', ice: 'наледь' };
  const FAULT_RU = { zero: 'показывает 0', stuck: 'залип', dropout: 'нет сообщений', noise: 'шум ×5', outliers: 'выбросы ×3' };

  // ---------------------------------------------------------------- панели
  const BOG_COL = { ok: C_OK, agree: C_OK, spin: C_NAIVE, skid: C_NAIVE, jump: C_NAIVE, forced: C_NAIVE, held: C_NAIVE, gate: C_NAIVE, stuck: C_BAD, dead: C_BAD, stale: C_BAD, frozen: C_BAD, wait: '' };
  function thinkHTML() {
    const E = UI.eng, x = SB.explain(E);
    const o = E.last ? E.last.o : null;
    const flag = (on, txt, col, title) => `<span class="rv-flag ${on ? 'on' : ''}" style="${on ? `color:${col}` : ''}" title="${title}">${txt}</span>`;
    const bog = (b) => {
      const g = x.bogies[b]; if (!g) return '';
      const col = BOG_COL[g.code] ?? '';
      const kmh = E.rows.n ? (b ? E.rows.rr[E.rows.n - 1] : E.rows.fr[E.rows.n - 1]) : NaN;
      return `<div class="rv-bog"><div class="k">${b ? 'Задняя тележка' : 'Передняя тележка'}</div><div class="big num">${f(kmh, 1)} <span class="k">км/ч</span></div>
        <span class="rv-chip" style="${col ? `color:${col}` : 'opacity:.6'}">${g.st}</span><div class="sb-why">${g.why}</div></div>`;
    };
    const m = x.mode;
    return `<div class="rv-h"><b>Что сейчас думает модель</b><span class="k num">t = ${fmtT(E.t)}</span></div>
      <div class="sb-head">${x.head}</div>
      <div class="rv-mode" style="color:${m === 'STANDSTILL' ? 'var(--fg)' : M_COL[m]}"><span class="mode-ico">${ICO[m] || ''}</span><span>${SB.M_RU[m]}</span></div>
      <div class="rv-flags">
        ${flag(o && o.valid, o && o.valid ? 'достоверно' : 'недостоверно', o && o.valid ? C_OK : C_BAD, 'valid: есть принятые показания колёс за последнюю секунду')}
        ${flag(o && o.slip, 'срыв', C_WET, 'slip: признак срыва сцепления')}
        ${flag(o && o.ambiguous, 'стоим или скользим?', C_WET, 'ambiguous: все колёса в срыве при торможении — по ним не различить остановку и юз')}
        ${flag(o && o.frozen, 'залипли обе', C_BAD, 'frozen: показания обеих тележек застыли при команде тяги или тормоза')}
        ${flag(o && o.wheels_stale, 'колёса молчат', C_BAD, 'wheels_stale: нет показаний тележек дольше 1 с — только модель')}
      </div>
      <div class="rv-bogies">${bog(0)}${bog(1)}</div>
      <ul class="sb-lines">${x.lines.map(l => `<li>${l}</li>`).join('')}</ul>
      <div class="sb-truth"><div class="k">На самом деле (имитатор, модели это не известно):</div><ul class="sb-lines">${x.truth.map(l => `<li>${l}</li>`).join('')}</ul></div>`;
  }
  function metricsHTML() {
    const S = UI.eng.summary(), W = S.win;
    const row = (name, a, b, title) => `<tr title="${title || ''}"><td>${name}</td><td style="color:${C_OK}">${a}</td><td style="color:${C_NAIVE}">${b}</td></tr>`;
    let rows = row('Скорость: MAE, м/с', f(S.v_mae, 3), f(S.naive_v_mae, 3), 'Средняя абсолютная ошибка против истины имитатора, все шаги 20 Гц')
      + row('Скорость: макс. ошибка, км/ч', f(S.v_max * KMH, 1), f(S.naive_v_max * KMH, 1), 'Наибольшая |ошибка| скорости')
      + row(`Путь от старта, м (истина ${f(S.path, 1)})`, f(S.path + S.s_err, 1), f(S.path + S.naive_s_err, 1), 'Пройденный путь по оценке и по базе')
      + row('Путь: ошибка сейчас, м', f(S.s_err, 1), f(S.naive_s_err, 1), 'Путь от старта: оценка минус истина (привязки к остановкам в песочнице нет — чистая одометрия)')
      + row('Истина в ±2σ скорости', f(100 * S.cov2s, 1) + ' %', '—', 'Доля шагов, где |ошибка| ≤ 2σ_v. У честной σ около 95 %');
    if (W) rows += row(`В окне условий (${f(W.s, 0)} с): MAE, м/с`, f(W.v_mae, 3), f(W.naive_v_mae, 3), 'Только пока действуют необычные условия сценария и 10 с после');
    return `<div class="rv-h"><b>Метрики</b><span class="k">с начала сценария · ${f(S.path / 1000, 2)} км</span></div>
      <table class="rv-tbl num"><thead><tr><th></th><th>модель</th><th>только колесо</th></tr></thead><tbody>${rows}</tbody></table>
      <div class="rv-note">Сравнение с истиной имитатора, а не с GNSS. «Только колесо» — база из tools/eval.py: среднее свежих показаний тележек.${UI.eng.done ? ' <b>Сценарий закончен.</b>' : ''}</div>`;
  }
  let lastPanel = 0;
  function render(force) {
    if (!UI.eng) return;
    drawScene(); drawSpeed(); drawStrip();
    const R = UI.eng.rows;
    drawErr('sbErrV', i => R.v[i] - R.vt[i], i => R.nv[i] - R.vt[i], i => R.sv[i], KMH, 1);
    drawErr('sbErrS', i => R.s[i] - R.st[i], i => R.ns[i] - R.st[i], i => R.ss[i], 1, 2);
    drawTrack();
    const now = performance.now();
    if (force || now - lastPanel > 150) {
      lastPanel = now;
      $('sbThink').innerHTML = thinkHTML();
      $('sbMetrics').innerHTML = metricsHTML();
      $('sbTime').textContent = `${fmtT(UI.eng.t)}${isFinite(UI.eng.preset.T) ? ' / ' + fmtT(UI.eng.preset.T) : ''}`;
      if (UI.eng.preset.manual) { $('sbHandle').value = UI.eng.manualNotch; }
      else $('sbHandleAuto').textContent = `ручка ${UI.eng.plant.notch > 0 ? '+' : ''}${UI.eng.plant.notch} — ведёт автоводитель`;
      syncPlay();
    }
  }

  // ---------------------------------------------------------------- цикл
  let last = performance.now();
  const frameErr = new Set();
  function frame(now) {
    requestAnimationFrame(frame);
    try {
      const dt = Math.min(0.1, (now - last) / 1000); last = now;
      if ((window.__tvMode || 'sandbox') !== 'sandbox' || !UI.eng) return;
      if (UI.play && !UI.eng.done && !document.hidden) UI.eng.advance(dt * UI.speed);
      render(false);
    } catch (e) { const k = String(e && e.message || e); if (!frameErr.has(k)) { frameErr.add(k); console.error('кадр песочницы:', e); } }
  }
  if (SHEET) { $('sbSheet').textContent = `${SHEET.path.split('/').pop()} (${SHEET.sheet_sha1})`; $('sbSheet').title = `${SHEET.path}: ${SHEET.label}`; }
  const want = qs.get('preset') || store.get('tv.preset');
  load(PRESETS.some(p => p.key === want) ? want : 'dry');
  applyPort();
  requestAnimationFrame(frame);
  // для проверок (simulator/test/page_test.js)
  window.__tvSandbox = {
    load, state: () => ({ key: UI.key, t: UI.eng.t, n: UI.eng.rows.n, done: UI.eng.done, play: UI.play, summary: UI.eng.summary() }),
    run: (sec, quiet) => { const e = UI.eng; const t1 = e.t + sec; while (!e.done && e.t < t1 - 5e-4) e.advance(Math.min(1, t1 - e.t)); if (!quiet) render(true); return UI.eng.summary(); },
    explain: () => SB.explain(UI.eng), play: on => { UI.play = on; syncPlay(); }, render: () => render(true),
  };
})();
