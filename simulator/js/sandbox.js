// ---- Песочница: имитатор + НАСТОЯЩЕЕ ядро (JS-порт) + связка, сценарии, объяснение ----
// Работает и в браузере (simulator/index.html), и в Node (simulator/test/sandbox_report.js):
// один и тот же код считает и экран, и таблицу docs/SANDBOX.md.
//
//   имитатор (plant.js, приближение)  --сообщения /vehicle/* (км/ч, ручка)-->  связка (runner.js)
//        |                                                                         |
//      истина v, s                                          ядро (est.js, лист жюri) -> v, s, σ, режим
//
// База «только колесо» — та же, что в tools/eval.py: среднее свежих показаний тележек.
const TramSandbox = (() => {
  const isNode = typeof module !== 'undefined' && typeof window === 'undefined';
  const E = isNode ? require('./est.js') : TramEst;
  const R = isNode ? require('./runner.js') : TramRunner;
  const P = isNode ? require('./plant.js') : TramPlant;
  const KMH = 3.6;
  const MODES = E.MODE_NAMES;

  // ------------------------------------------------------------------ линия
  class Track {
    constructor(tr) {
      const d = tr.dirs[0];
      this.src = tr; this.s = d.s; this.x = d.x; this.y = d.y; this.z = d.z; this.k = d.k;
      this.L = d.s[d.s.length - 1];
      // остановки карты пакета на линии; для маршрута водителя — не ближе 300 м друг к другу
      // (соседние точки карты — это и платформы, и стоянки у светофоров)
      this.allStops = tr.stops.slice();
      this.stops = [];
      for (const x of this.allStops) if (!this.stops.length || x - this.stops[this.stops.length - 1] >= 300) this.stops.push(x);
      this.gradeFn = null;                // сценарий может подменить профиль
    }
    _i(s) { let lo = 0, hi = this.s.length - 1; if (s <= this.s[0]) return 0; if (s >= this.s[hi]) return hi - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (this.s[m] <= s) lo = m; else hi = m; } return lo; }
    _at(arr, s) { const i = this._i(s), w = Math.max(0, Math.min(1, (s - this.s[i]) / (this.s[i + 1] - this.s[i]))); return arr[i] + w * (arr[i + 1] - arr[i]); }
    xy(s) { s = Math.max(0, Math.min(this.L, s)); return [this._at(this.x, s), this._at(this.y, s)]; }
    zAt(s) { return this._at(this.z, Math.max(0, Math.min(this.L, s))); }
    // уклон, рад (+ подъём): профиль высот pathgraph, сглаженный на ±20 м
    grade(s) {
      if (this.gradeFn) return this.gradeFn(s);
      const h = 20, a = Math.max(0, s - h), b = Math.min(this.L, s + h);
      return b > a ? Math.atan((this.zAt(b) - this.zAt(a)) / (b - a)) : 0;
    }
    kMax(s0, s1) { let m = 0; for (let i = this._i(Math.max(0, s0)); i < this.s.length && this.s[i] <= s1; i++) m = Math.max(m, this.k[i]); return m; }
  }
  // Линия-заглушка, если js/track.js нет (pathgraph организаторов не кладём в git, пока не
  // ясна лицензия; файл пересоздаёт simulator/tools/gen_track.py из _incoming/pathgraph):
  // 4,7 км, шаг 5 м, кривые R 150–300 м, профиль ±10 м, остановки через ~470 м.
  // Числа песочницы на ней не совпадут с docs/SANDBOX.md (там настоящая линия).
  Track.synthetic = () => {
    const s = [], x = [], y = [], z = [], k = [];
    let th = 0, px = 0, py = 0;
    for (let i = 0; i <= 940; i++) {
      const si = 5 * i, ph = si % 1100, sg = Math.floor(si / 1100) % 2 ? 1 : -1;
      const ki = ph > 700 && ph < 880 ? sg / (si > 2200 ? 300 : 150) : 0;
      if (i) { th += ki * 5; px += 5 * Math.cos(th); py += 5 * Math.sin(th); }
      s.push(si); x.push(px); y.push(py); z.push(150 + 10 * Math.sin(si / 750)); k.push(Math.abs(ki));
    }
    const stops = []; for (let a = 60; a < 4650; a += 470) stops.push(a);
    return { synthetic: true, source: 'линия-заглушка (нет js/track.js)', origin: null, stops, dirs: [{ s, x, y, z, k, name: 'заглушка', length_m: 4700 }] };
  };

  // ------------------------------------------------------------------ водитель (автомат)
  // Видит истинную скорость и путь (как настоящий водитель), ведёт вагон от остановки к
  // остановке: разгон позицией acc_notch, поддержание скорости, торможение к точке
  // остановки (позиция тормоза подбирается по той же таблице привода), стоянка.
  class Driver {
    constructor(eng, o) {
      this.eng = eng; this.o = Object.assign({ acc_notch: 11, v_max: 40 / KMH, a_brake: 0.9, brake: 'normal', dwell: 12, a_lat: 0.6 }, o || {});
      this.state = 'dwell'; this.t_state = 0; this.target = null; this.nextStop();
      this.dwell = this.o.first_dwell ?? 4;
    }
    nextStop() {
      const s = this.eng.plant.s, st = this.eng.route;
      this.target = st.find(x => x > s + 30) ?? null;
    }
    vLimit(s) {
      const k = this.eng.track.kMax(s - 10, s + 80);
      return Math.min(this.o.v_max, k > 1e-4 ? Math.sqrt(this.o.a_lat / k) : Infinity);
    }
    // позиция тормоза для замедления a (>0) при скорости v — по таблице листа
    brakeNotch(a, v) {
      const p = this.eng.sheet; let best = -1, err = Infinity;
      for (let n = -1; n >= -15; n--) { const acc = E.table_acc(E.notch_to_u(n, p), v, p); const e = Math.abs(-acc - a); if (e < err) { err = e; best = n; } }
      return best;
    }
    update(dt) {
      const pl = this.eng.plant, v = pl.v, s = pl.s, o = this.o;
      this.t_state += dt;
      if (this.state === 'dwell') {
        if (this.t_state >= this.dwell && this.target !== null) { this.state = 'go'; this.t_state = 0; }
        return -5;
      }
      if (this.target === null) return v > 0.1 ? this.brakeNotch(0.8, v) : -5;
      const d = this.target - s, gs = 9.81 * Math.sin(this.eng.track.grade(s));   // уклон: + подъём помогает тормозить
      const vlim = Math.min(this.vLimit(s), this.vLimit(s + Math.max(0, v * v / (2 * 0.6))));
      if (this.state === 'go') {
        const a_req = v * v / (2 * Math.max(0.5, d - 2));
        if (a_req - gs >= o.a_brake || d < 1.5) { this.state = 'brake'; this.t_state = 0; }
        else if (v < vlim - 1.5 && v < o.v_max) return o.acc_notch;
        else if (v > vlim + 0.4) return this.brakeNotch(Math.min(0.8, (v - vlim) * 0.8 + 0.2), v);
        else return v < vlim - 0.5 ? 4 : 0;
      }
      if (this.state === 'brake') {
        if (v < 0.05 && d < 25) {
          this.state = 'dwell'; this.t_state = 0; this.dwell = o.dwell; const tgt = this.target; this.target = null;
          this.eng.onArrive && this.eng.onArrive(tgt); this.nextStop(); return -5;
        }
        if (o.brake === 'hard') return v > 0.3 ? -15 : -6;
        const a_req = v * v / (2 * Math.max(0.3, d - 1));
        if (d < 0.5 || (v < 0.6 && d < 3)) return -8;
        if (a_req - gs < 0.45 * o.a_brake && v > 1.5) return 0;
        return this.brakeNotch(Math.min(1.3, Math.max(0.05, a_req - gs)), v);
      }
      return 0;
    }
  }

  // ------------------------------------------------------------------ сценарии
  // t — секунды от начала сценария; зоны сцепления — по пути (м от начала линии).
  // real — вид инъекции в реальные записи с тем же отказом (docs/EVAL.md §6; таблица docs/SANDBOX.md
  // берёт оттуда числа) или строка — почему аналога нет.
  const PRESETS = [
    { key: 'dry', ru: 'Сухо (норма)', route: [1, 3], T: 200,
      about: 'Сухой рельс, водитель ведёт вагон от остановки к остановке по настоящей линии (pathgraph организаторов, профиль высот оттуда же).',
      expect: 'Обе тележки принимаются, модель идёт по колёсам; ошибка скорости того же порядка, что на реальных записях.' },
    { key: 'rain_spin', real: ['spin_traction'],  ru: 'Дождь + полная тяга (буксование)', route: [1, 3], T: 200, weather: 'rain', driver: { acc_notch: 15, a_brake: 0.5 },
      about: 'Мокрый рельс (μ ≈ 0,08–0,10), водитель трогается полной тягой (+15): колёса срываются, передняя тележка — сильнее.',
      expect: 'Показание буксующей тележки отбрасывается («колесо разгоняется быстрее, чем может вагон»), модель снижает оценку сцепления и не разгоняет вагон вслед за колёсами.' },
    { key: 'ice_skid', real: 'нет: юз с блокировкой колёс на записях не вводили',  ru: 'Снег / наледь при торможении (юз)', route: [1, 3], T: 200, driver: { brake: 'hard' }, wsp: false,
      zones: [{ from: 'stop+1', before: 260, after: 40, weather: 'ice' }],
      about: 'Перед остановкой наледь (μ ≈ 0,05), водитель тормозит экстренно (−15), противоюзная защита не справляется: колёса блокируются, вагон скользит.',
      expect: 'Нули заблокированных колёс не принимаются за остановку: режим «стоим или скользим?», скорость по модели, полоса ±2σ широкая, пока колёса снова не покатятся.' },
    { key: 'leaves', real: ['skid_brake'],  ru: 'Листопад', route: [1, 3], T: 200, driver: { acc_notch: 13 },
      zones: [{ from: 'stop+0', before: 0, after: 220, weather: 'leaves' }, { from: 'stop+1', before: 240, after: 200, weather: 'leaves' }, { from: 'stop+2', before: 240, after: 60, weather: 'leaves' }],
      about: 'Участки листопадной плёнки (μ ≈ 0,05) и на разгоне, и на торможении; защита от буксования и юза работает — колёса проскальзывают на 10–30 %, но не блокируются.',
      expect: 'Резкие срывы отбрасываются; плавное проскальзывание под защитой модель от настоящей скорости не отличает (как юз −30 % на реальных записях, docs/EVAL.md) — это слабое место.' },
    { key: 'front_fail', real: ['front_zero'],  ru: 'Отказ передней тележки', route: [0, 1], T: 165,
      events: [{ t0: 35, t1: 95, fault: [{ kind: 'zero' }, null] }],
      about: 'Датчик передней тележки на ходу начинает показывать 0 км/ч (60 с).',
      expect: 'Передняя исключается как оборванная (ноль при движении), скорость — по задней; после отказа датчик возвращается, когда 2 с согласуется с задней.' },
    { key: 'both_fail', real: ['both_zero'],  ru: 'Отказ обеих тележек', route: [0, 1], T: 165,
      events: [{ t0: 40, t1: 60, fault: [{ kind: 'zero' }, { kind: 'zero' }] }],
      about: 'Обе тележки на ходу разом показывают 0 км/ч (20 с).',
      expect: 'Скачок в ноль физически невозможен — нули отбрасываются, скорость и путь идут по модели (ручка + физика), оценка помечена недостоверной, σ растёт. Здесь это лучший случай: вагон идёт ровно, а имитатор разгоняется по той же таблице привода, что у модели; на реальных записях ошибка пути в разы больше (docs/SANDBOX.md), но всё равно меньше, чем у колёс.' },
    { key: 'both_fail_start', real: 'нет: на записях отказы вводятся только на ходу (≥ 4 м/с)',  ru: 'Отказ обеих при трогании', route: [0, 1], T: 165,
      events: [{ t0: 6, t1: 26, fault: [{ kind: 'zero' }, { kind: 'zero' }] }],
      about: 'Вагон только тронулся (≈ 1,2 м/с, ручка +11), и обе тележки разом начинают показывать 0 км/ч (20 с).',
      expect: 'Слабое место: на скорости ниже 2 м/с ноль обеих тележек похож на обычную остановку — модель принимает «стоим», хотя ручка держит тягу; оценка при этом считается достоверной. Ошибка пути — как у простой одометрии.' },
    { key: 'both_fail_stop', real: 'нет: на записях отказы вводятся только на ходу (≥ 4 м/с)',  ru: 'Отказ обеих при торможении к остановке', route: [0, 1], T: 175,
      events: [{ t0: 140, t1: 160, fault: [{ kind: 'zero' }, { kind: 'zero' }] }],
      about: 'Вагон тормозит к остановке (≈ 8 м/с), и обе тележки разом начинают показывать 0 км/ч (20 с): вагон останавливается и стоит, а датчики молчат нулями.',
      expect: 'Слабое место: нули приняты за юз («стоим или скользим?»), модель держит скорость по ручке и после остановки — ошибка пути больше, чем у простой одометрии, пока колёса снова не покажут ход или стоянку.' },
    { key: 'stuck', real: ['both_stuck'],  ru: 'Залипание датчиков', route: [0, 1], T: 165,
      events: [{ t0: 8, t1: 40, fault: [{ kind: 'stuck' }, null] }, { t0: 100, t1: 118, fault: [{ kind: 'stuck' }, { kind: 'stuck' }] }],
      about: 'Сначала на разгоне залипает передняя (32 с одно и то же значение), потом на ходу обе разом (18 с).',
      expect: 'Одна: исключается, когда задняя меняется, а она нет. Обе: признак «залипли все разом» при команде тяги или тормоза — только прогноз по ручке.' },
    { key: 'dropout', real: ['dropout'],  ru: 'Пропуски сообщений', route: [0, 1], T: 165,
      events: [{ t0: 32, t1: 34, drop: true }, { t0: 50, t1: 50.6, drop: true }, { t0: 70, t1: 75, drop: true }, { t0: 139, t1: 144, drop: true }],
      about: 'Сообщения обеих тележек пропадают: 2 с, 0,6 с и 5 с на ходу, затем 5 с на торможении к остановке.',
      expect: 'До 1 с — прогноз между показаниями; дольше — разомкнутый режим (скорость по ручке и физике, полоса ±2σ растёт, оценка недостоверна); после возврата данных — снова по колёсам.' },
    { key: 'noise', real: ['noise', 'outliers'],  ru: 'Шум датчика', route: [0, 1], T: 165,
      events: [{ t0: 30, t1: 70, fault: [{ kind: 'noise', sigma_kmh: 0.9 }, { kind: 'noise', sigma_kmh: 0.9 }] }, { t0: 85, t1: 110, fault: [null, { kind: 'outliers', p: 0.05 }] }],
      about: 'Шум ×5 (σ ≈ 0,25 м/с) на обеих тележках 40 с, затем выбросы ×3 на задней.',
      expect: 'Шум сглаживается фильтром (модель тише колёс), выбросы отбрасываются проверкой невязки.' },
    { key: 'rush', ru: 'Час пик (+30 % массы)', route: [1, 3], T: 200, mass: 1.3,
      about: 'Вагон на 30 % тяжелее номинала: та же ручка разгоняет и тормозит слабее. Модель массу не знает.',
      expect: 'Прогноз по ручке ошибается, но колёса согласны между собой — модель идёт по колёсам; ошибка почти как в норме.' },
    { key: 'hill', ru: 'Подъём / спуск', route: [1, 3], T: 200, grade: { at: 'stop+0', up: 0.04, len: 250 },
      about: 'Подъём 40 ‰ сразу после остановки, затем такой же спуск (профиль подменён; у настоящей линии до ±36–40 ‰ на коротких участках).',
      expect: 'Уклон модель не знает: его берёт на себя возмущение d (подстраивается на выбеге), скорость по колёсам.' },
    { key: 'urban', ru: 'Застройка без GNSS', route: [1, 3], T: 200, gnss_off: 20,
      about: 'С 20-й секунды спутники пропадают (плотная застройка). GNSS модели нужен только в первые 3 с для выставки положения.',
      expect: 'Ничего не меняется: скорость и путь от GNSS не зависят (вход GNSS читается только в окне выставки).' },
    { key: 'manual', ru: 'Ручное управление', route: [0, 99], T: Infinity, manual: true,
      about: 'Вы ведёте вагон сами: ручка −15…+15 (клавиши W/S или ↑/↓, пробел — выбег), условия и отказы — переключателями.',
      expect: 'Попробуйте полную тягу в дождь, экстренное торможение на наледи или отказ тележки и смотрите панель «Что сейчас думает модель».' },
  ];

  // ------------------------------------------------------------------ движок
  class Engine {
    constructor(opts) {
      this.sheetDoc = opts.sheet;              // TV_SHEET
      this.sheet = Object.assign({}, E.DEFAULT, opts.sheet.core);
      this.node = opts.sheet.node || {};
      this.trackDoc = opts.track && opts.track.dirs ? opts.track : Track.synthetic();   // TV_TRACK или заглушка
      this.seed = opts.seed ?? 7;
      this.maxRows = opts.maxRows ?? 36000;
      this.load(opts.preset || 'dry');
    }
    load(key) {
      const pr = PRESETS.find(x => x.key === key) || PRESETS[0];
      this.preset = pr;
      this.track = new Track(this.trackDoc);
      const st = this.track.stops;
      const i0 = Math.min(pr.route[0], st.length - 1), i1 = Math.min(st.length - 1, i0 + pr.route[1]);
      this.route = st.slice(i0 + 1, i1 + 1);
      if (pr.manual) this.route = [];
      const s0 = st[i0];
      this.plant = new P.Plant(this.sheet, { seed: this.seed, s0 });
      this.plant.mass_factor = pr.mass || 1.0;
      if (pr.wsp === false) this.plant.C.wsp = false;
      this.baseWeather = pr.weather || 'dry';
      this.plant.setWeather(this.baseWeather);
      this.zones = (pr.zones || []).map(z => { const k = +z.from.split('+')[1], sc = st[Math.min(st.length - 1, i0 + k)]; return { a: sc - z.before, b: sc + z.after, weather: z.weather }; });
      if (pr.grade) {
        const k = +pr.grade.at.split('+')[1], a = st[Math.min(st.length - 1, i0 + k)] + 20, L = pr.grade.len, g = pr.grade.up, base = this.track;
        const real = s => { const h = 20, x = Math.max(0, s - h), y = Math.min(base.L, s + h); return Math.atan((base.zAt(y) - base.zAt(x)) / (y - x)); };
        base.gradeFn = s => (s >= a && s < a + L) ? Math.atan(g) : (s >= a + L && s < a + 2 * L) ? -Math.atan(g) : real(s);
        this.hill = [a, a + L, a + 2 * L];
      } else this.hill = null;
      this.runner = new R.Runner(this.sheet, this.node);
      this.core = this.runner.core;
      this.dg = this.core.diag();
      this.driver = pr.manual ? null : new Driver(this, pr.driver);
      this.manualNotch = 0;
      this.t = 0; this.acc = 0;
      this.rows = this.emptyRows(); this.hist = [];
      this.userFaults = [null, null]; this.userDrop = false; this.userWeather = null; this.userMass = null; this.userGnssOff = false;
      this.last = null; this.lastMsgT = [-Infinity, -Infinity]; this.lastKmh = [NaN, NaN];
      this.lastWhy = [12, 12]; this.lastWhyT = [-Infinity, -Infinity];
      this.events = []; this.arrivals = [];
      this.gnssOff = false;
      this.stats = this.emptyStats();
      this._winT = -Infinity;
      this.done = false;
      this.onArrive = s => this.arrivals.push({ t: this.t, s, err: this.plant.s - s });
      this.applyConditions();
    }
    emptyRows() { const o = { n: 0 }; for (const k of Engine.KEYS) o[k] = []; return o; }
    emptyStats() { return { n: 0, ae: 0, aeN: 0, cov: 0, mode: new Array(7).fill(0), slip: 0, amb: 0, invalid: 0, stale: 0, frozen: 0, win: { n: 0, ae: 0, aeN: 0, maxE: 0, maxEN: 0, cov: 0 }, rej: [0, 0], acc: [0, 0], exc: [0, 0], agree: 0, winRej: [0, 0], winSeen: [0, 0], maxE: 0, maxEN: 0, muMin: Infinity, svMax: 0, overconf: 0 }; }
    // активные условия на момент t (сценарий + ручные переключатели)
    active(t) {
      const pr = this.preset, s = this.plant.s;
      let faults = [null, null], drop = false;
      for (const ev of pr.events || []) if (t >= ev.t0 && t < ev.t1) { if (ev.fault) faults = ev.fault.map((f, b) => f || faults[b]); if (ev.drop) drop = true; }
      // сцепление — по месту каждой тележки: передняя на s, задняя на 7,55 м позади
      const weatherB = [0, 1].map(b => this.weatherAt(s - b * this.plant.C.bogie_base));
      // ручной переключатель по тележке: объект отказа; 'none' — «норма» поверх отказа сценария;
      // null — как в сценарии (по умолчанию: числа сценариев и таблицы docs/SANDBOX.md не меняются)
      faults = faults.map((f, b) => this.userFaults[b] === 'none' ? null : (this.userFaults[b] || f));
      return { faults, drop: drop || this.userDrop, weather: weatherB[0], weatherB, mass: this.userMass ?? (pr.mass || 1.0), gnssOff: this.userGnssOff || (pr.gnss_off !== undefined && t >= pr.gnss_off) };
    }
    // погода на рельсе в точке линии s (зоны сценария, общий фон, ручной переключатель)
    weatherAt(s) {
      if (this.userWeather) return this.userWeather;
      let w = this.baseWeather;
      for (const z of this.zones) if (s >= z.a && s <= z.b) w = z.weather;
      return w;
    }
    // окно сценария для метрик: пока действует необычное условие и 10 с после
    inWindow() { return this.t - this._winT <= 10; }
    applyConditions() {
      const c = this.active(this.t), pl = this.plant;
      const key = JSON.stringify(c.faults);
      if (key !== this._fkey) { this._fkey = key; pl.setFault(0, c.faults[0]); pl.setFault(1, c.faults[1]); }
      pl.dropAll = c.drop;
      const wk = c.weatherB.join('|');
      if (wk !== this._wkey) { this._wkey = wk; pl.setWeather(c.weatherB[0], 0); pl.setWeather(c.weatherB[1], 1); }
      const hill = this.hill && pl.s >= this.hill[0] && pl.s <= this.hill[2];
      if (c.faults[0] || c.faults[1] || c.drop || c.weatherB.some(w => w !== 'dry') || c.mass !== 1 || c.gnssOff || hill) this._winT = this.t;
      pl.mass_factor = c.mass;
      this.gnssOff = c.gnssOff;
      this.cond = c;
    }
    setManual(n) { this.manualNotch = Math.max(-15, Math.min(15, Math.round(n))); }
    // шаг имитации на dt секунд (имитатор 1 кГц, связка и ядро — по сообщениям)
    advance(dt) {
      if (this.done) return;
      const h = 0.001, pl = this.plant;
      this.acc += dt;
      while (this.acc >= h - 1e-12) {
        this.acc -= h;
        if (Math.abs((this.t / 0.05) - Math.round(this.t / 0.05)) < 1e-6) {
          this.applyConditions();
          pl.notch = this.driver ? this.driver.update(0.05) : this.manualNotch;
        }
        pl.grade = this.track.grade(pl.s);
        pl.step(h);
        this.t = pl.t;
        this.hist.push([this.t, pl.v, pl.s, pl.w[0], pl.w[1]]);
        if (this.hist.length > 3000) this.hist.splice(0, 1000);
        for (const m of pl.poll()) {
          const outs = m.kind === 'wheel' ? this.runner.on_wheel(m.i, m.stamp, m.value) : this.runner.on_handle(m.stamp, m.value);
          if (m.kind === 'wheel') { this.lastMsgT[m.i] = m.stamp; this.lastKmh[m.i] = m.value; }
          for (const o of outs) this.record(o);
        }
        if (this.t >= this.preset.T) { this.done = true; break; }
      }
    }
    truthAt(t) {
      const H = this.hist; let lo = 0, hi = H.length - 1;
      if (!H.length) return [0, 0, 0, 0];
      if (t <= H[0][0]) return H[0].slice(1);
      while (hi - lo > 1) { const m = (lo + hi) >> 1; if (H[m][0] <= t) lo = m; else hi = m; }
      const a = H[lo], b = H[hi], w = Math.max(0, Math.min(1, (t - a[0]) / Math.max(1e-9, b[0] - a[0])));
      return [a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * (b[3] - a[3]), a[4] + w * (b[4] - a[4])];
    }
    record(o) {
      const [vt, st, wf, wr] = this.truthAt(o.stamp), c = this.cond, pl = this.plant, R0 = this.rows, dg = this.dg;
      const fresh = b => o.stamp - this.lastMsgT[b] <= 0.3;
      const why = b => o.frozen ? 13 : this.core.healthy[b] === false ? (dg.bad[b] === 'stuck' ? 9 : 10) : (o.stamp - this.lastMsgT[b] > this.runner.wheel_timeout) ? 11 : (o.stamp - dg.t[b] <= 0.15 ? Engine.WHY.indexOf(dg.why[b]) : 12);
      const row = {
        t: o.stamp, vt, st: st - this.route0(), v: o.v, sv: o.sigma_v, s: o.s, ss: o.sigma_s, mode: o.mode,
        fl: (o.valid ? 1 : 0) | (o.slip ? 2 : 0) | (o.ambiguous ? 4 : 0) | (o.handle_ok ? 8 : 0) | (o.wheels_stale ? 16 : 0) | (o.frozen ? 32 : 0),
        nv: o.naive_v, ns: o.naive_s, fr: fresh(0) ? this.lastKmh[0] : NaN, rr: fresh(1) ? this.lastKmh[1] : NaN, h: o.notch,
        wf, wr, mu: o.mu, kt: o.k_t, kb: o.k_b, d: o.d, w0: why(0), w1: why(1), a: o.a,
        muf: pl.mu_peak[0], mur: pl.mu_peak[1], m: c.mass, f0: c.faults[0] ? Engine.FAULTS.indexOf(c.faults[0].kind) : -1, f1: c.faults[1] ? Engine.FAULTS.indexOf(c.faults[1].kind) : -1,
        dr: c.drop ? 1 : 0, g: pl.grade, wx: Engine.WX.indexOf(c.weather), gn: this.gnssOff ? 1 : 0, win: this.inWindow() ? 1 : 0,
      };
      for (const k of Engine.KEYS) R0[k].push(row[k]);
      for (let b = 0; b < 2; b++) { const w = row['w' + b]; if (w !== 12) { this.lastWhy[b] = w; this.lastWhyT[b] = o.stamp; } }
      R0.n++;
      if (R0.n > this.maxRows) { const cut = Math.floor(this.maxRows / 6); for (const k of Engine.KEYS) R0[k].splice(0, cut); R0.n -= cut; }
      this.last = { o, row };
      // накопленные метрики
      const S = this.stats, e = o.v - vt, eN = o.naive_v - vt;
      S.n++; S.ae += Math.abs(e); S.aeN += Math.abs(eN); S.cov += Math.abs(e) <= 2 * o.sigma_v ? 1 : 0; S.mode[o.mode]++;
      S.maxE = Math.max(S.maxE, Math.abs(e)); S.maxEN = Math.max(S.maxEN, Math.abs(eN));
      if (o.slip) S.slip++; if (o.ambiguous) S.amb++; if (!o.valid) S.invalid++; if (o.wheels_stale) S.stale++; if (o.frozen) S.frozen++;
      // «уверена, но ошибается»: оценка помечена достоверной, а истина вне ±2σ дальше 0,5 м/с
      if (o.valid && Math.abs(e) > Math.max(0.5, 2 * o.sigma_v)) S.overconf++;
      S.muMin = Math.min(S.muMin, o.mu); S.svMax = Math.max(S.svMax, o.sigma_v);
      if (row.w0 === 1 || row.w1 === 1) S.agree++;
      for (let b = 0; b < 2; b++) { const w = row['w' + b]; if (w === 9 || w === 10) S.exc[b]++; if ((w >= 2 && w <= 7) || w === 13 || w === 14) { S.rej[b]++; if (row.win) S.winRej[b]++; } else if (w <= 1) S.acc[b]++; if (row.win && w !== 12) S.winSeen[b]++; }
      if (row.win) { const W = S.win; W.n++; W.ae += Math.abs(e); W.aeN += Math.abs(eN); W.maxE = Math.max(W.maxE, Math.abs(e)); W.maxEN = Math.max(W.maxEN, Math.abs(eN)); W.cov += Math.abs(e) <= 2 * o.sigma_v ? 1 : 0; }
    }
    route0() { return this.track.stops[Math.min(this.preset.route[0], this.track.stops.length - 1)]; }
    // сводка для таблицы и панели метрик
    summary() {
      const S = this.stats, n = Math.max(1, S.n), W = S.win, R0 = this.rows, k = R0.n - 1, dt = this.sheet.dt;
      return {
        preset: this.preset.key, ru: this.preset.ru, T: this.t, rows: S.n,
        v_mae: S.ae / n, naive_v_mae: S.aeN / n, v_max: S.maxE, naive_v_max: S.maxEN, cov2s: S.cov / n,
        s_err: k >= 0 ? R0.s[k] - R0.st[k] : NaN, naive_s_err: k >= 0 ? R0.ns[k] - R0.st[k] : NaN, path: k >= 0 ? R0.st[k] : 0,
        slip_s: S.slip * dt, amb_s: S.amb * dt, invalid_s: S.invalid * dt, stale_s: S.stale * dt, frozen_s: S.frozen * dt,
        rej_front: S.rej[0] * dt, rej_rear: S.rej[1] * dt, exc_front: S.exc[0] * dt, exc_rear: S.exc[1] * dt,
        agree_steps: S.agree, mu_min: S.muMin, sv_max: S.svMax, overconf_s: S.overconf * dt,
        win: W.n ? { s: W.n * dt, v_mae: W.ae / W.n, naive_v_mae: W.aeN / W.n, v_max: W.maxE, naive_v_max: W.maxEN, cov2s: W.cov / W.n,
          rej_front: S.winSeen[0] ? S.winRej[0] / S.winSeen[0] : 0, rej_rear: S.winSeen[1] ? S.winRej[1] / S.winSeen[1] : 0 } : null,
        arrivals: this.arrivals.slice(),
        mode_s: Object.fromEntries(MODES.map((m, i) => [m, S.mode[i] * dt])),
      };
    }
  }
  Engine.KEYS = ['t', 'vt', 'st', 'v', 'sv', 's', 'ss', 'mode', 'fl', 'nv', 'ns', 'fr', 'rr', 'h', 'wf', 'wr', 'mu', 'kt', 'kb', 'd', 'w0', 'w1', 'a', 'muf', 'mur', 'm', 'f0', 'f1', 'dr', 'g', 'wx', 'gn', 'win'];
  // решение по тележке: 0 ok … 8 gate — из ядра (est.js diag), 9–12 — связка/исправность
  Engine.WHY = ['ok', 'agree', 'spin', 'skid', 'jump', 'forced', 'held', 'gate', 'x', 'stuck', 'dead', 'stale', 'wait', 'frozen', 'slipall'];
  Engine.FAULTS = ['zero', 'stuck', 'dropout', 'noise', 'outliers'];
  Engine.WX = ['dry', 'rain', 'leaves', 'ice'];

  // ------------------------------------------------------------------ объяснение простым языком
  const M_RU = { COAST: 'Выбег', TRACTION: 'Тяга', BRAKE: 'Торможение', TRANSITION: 'Смена режима', SLIP: 'Срыв сцепления', STANDSTILL: 'Стоянка', DEGRADED: 'Нет данных колёс' };
  const M_SUB = { COAST: 'привод не тянет: колёса дают точную скорость, модель подстраивает возмущение (уклон, масса)', TRACTION: 'привод тянет: колесо может только завышать (буксование)', BRAKE: 'торможение: колесо может только занижать (юз)', TRANSITION: 'привод набирает или сбрасывает момент', SLIP: 'признаки срыва сцепления: показания сорвавшихся колёс не принимаются', STANDSTILL: 'вагон стоит, скорость ноль', DEGRADED: 'принятых показаний колёс нет — скорость только по модели' };
  const WHY_RU = {
    ok: ['принята', 'согласна с прогнозом модели'],
    agree: ['принята', 'модель разошлась с колёсами, но обе тележки согласны между собой — модель уступила колёсам'],
    spin: ['отброшена', 'колесо разгоняется быстрее, чем может вагон, — буксование'],
    skid: ['отброшена', 'колесо замедляется быстрее, чем может вагон, — юз'],
    jump: ['отброшена', 'скачок показания за один такт больше физически возможного — срыв или отказ датчика'],
    forced: ['отброшена', 'только что был срыв, а показание снова уходит в сторону срыва'],
    held: ['отброшена', 'пока неясно «стоим или скользим», нули колёс не принимаются'],
    gate: ['отброшена', 'слишком далеко от прогноза модели (дальше 3σ) — выброс'],
    stuck: ['исключена', 'датчик залип: показание не меняется, а у другой тележки меняется; вернётся, когда 2 с будет согласен с остальными'],
    dead: ['исключена', 'датчик показывает 0 при движении — обрыв; вернётся, когда 2 с будет согласен с остальными'],
    stale: ['молчит', 'сообщений нет дольше 1 с'],
    wait: ['ждёт', 'нового показания ещё нет (датчик шлёт ~9 раз в секунду)'],
    frozen: ['не принята', 'обе тележки залипли разом — показания не принимаются, скорость по ручке'],
    slipall: ['отброшена', 'обе тележки сорвались разом (скачок показаний в одну сторону) — показание в сторону срыва говорит лишь о границе скорости вагона, скорость ведёт модель'],
  };
  const f1 = x => (x < 0 ? '−' : '') + Math.abs(x).toFixed(1).replace('.', ',');
  const f2 = x => (x < 0 ? '−' : '') + Math.abs(x).toFixed(2).replace('.', ',');
  function explain(eng) {
    if (!eng.last) return { head: 'Вагон на старте: ждём первых сообщений тележек и ручки.', lines: [], bogies: [], truth: [], mode: 'DEGRADED' };
    const o = eng.last.o, r = eng.last.row, p = eng.sheet, mode = MODES[o.mode];
    const u = eng.core.u_filt, T_d = p.T_dead;
    const bog = [0, 1].map(b => {
      // между показаниями (~0,1 с) — последнее решение, если оно свежее 1 с: табло не мигает
      let code = Engine.WHY[r['w' + b]] || 'wait';
      if (code === 'wait' && r.t - eng.lastWhyT[b] <= 1.0) code = Engine.WHY[eng.lastWhy[b]];
      let [st, why] = WHY_RU[code] || WHY_RU.wait;
      if (code === 'gate') {       // выброс по невязке: в какую сторону и при какой команде
        const nu = eng.dg.nu[b];
        if (u > T_d && nu > 0) why = 'показание выше прогноза под тягой, дальше 3σ, — похоже на буксование';
        else if (u < -T_d && nu < 0) why = 'показание ниже прогноза при торможении, дальше 3σ, — похоже на юз';
      }
      return { code, st, why, kmh: eng.lastKmh[b] };
    });
    const name = ['передняя', 'задняя'], Name = ['Передняя', 'Задняя'];
    let head;
    const rej = bog.map(x => ['spin', 'skid', 'jump', 'forced', 'held', 'gate', 'slipall'].includes(x.code));
    const out = bog.map(x => ['stuck', 'dead', 'stale'].includes(x.code));
    const okB = bog.map(x => ['ok', 'agree'].includes(x.code));
    const Gen = ['передней', 'задней'];
    const verb = b => out[b] ? (bog[b].code === 'stale' ? 'молчит' : 'исключена') : 'отброшена';
    if (o.frozen) head = 'Обе тележки залипли разом: при команде тяги или тормоза их показания не меняются. Модель их не принимает и ведёт скорость по ручке и физике вагона; полоса неопределённости растёт.';
    else if (o.wheels_stale) head = 'Показаний колёс нет дольше 1 с: модель ведёт скорость только по ручке и физике вагона (разомкнутый режим). Полоса ±2σ растёт, пока данные не вернутся.';
    else if (o.slip_all) head = `Обе тележки сорвались разом (${o.slip_all > 0 ? 'буксование' : 'юз'}): показания обеих скакнули в одну сторону, и согласие тележек между собой здесь ничего не доказывает. Скорость ведёт модель — ручка и физика вагона; показания принимаются снова, когда колёса вернутся к вагону (скачком, плавно или ${f1(p.slip_ok_s ?? 1)} с подряд согласны с моделью), но не дольше ${Math.round(p.slip_t_max ?? 12)} с.`;
    else if (o.ambiguous) head = 'Стоим или скользим? Вагон только что ехал, а колёса разом показывают почти ноль — так выглядит и остановка, и юз с заблокированными колёсами (и отказ обоих датчиков). Модель не верит нулям: держит скорость по ручке с пониженным сцеплением и широкой полосой ±2σ, пока колёса снова не покатятся.';
    else if (rej[0] && rej[1] && bog[0].code === bog[1].code) head = `Обе тележки отброшены: ${bog[0].why}. Скорость идёт по модели${o.slip ? ' с пониженным сцеплением' : ''}.`;
    else if ((rej[0] || out[0]) && (rej[1] || out[1])) head = `Ни одной тележке сейчас верить нельзя: передняя ${verb(0)} (${bog[0].why.split(';')[0]}), задняя ${verb(1)} (${bog[1].why.split(';')[0]}). Скорость — только по модели: ручка и физика вагона${o.slip ? ' с пониженным сцеплением' : ''}.`;
    else if (rej[0] || rej[1] || out[0] || out[1]) {
      const b = (rej[0] || out[0]) ? 0 : 1;
      head = `${Name[b]} тележка ${verb(b)}: ${bog[b].why.split(';')[0]}. Скорость — ${okB[1 - b] ? `по ${Gen[1 - b]} тележке и модели` : 'по модели до следующего показания'}.`;
    }
    else if (bog.some(x => x.code === 'agree')) head = 'Прогноз модели разошёлся с колёсами, но обе тележки согласны между собой: модель уступила колёсам (так она переживает ошибки ручки, уклон и лишнюю массу).';
    else if (mode === 'STANDSTILL') head = 'Вагон стоит: обе тележки показывают ноль, скорость ноль, ошибка скорости обнулена.';
    else if (bog.every(x => x.code === 'ok')) head = 'Едет по колёсам: обе тележки согласны с прогнозом модели.';
    else if (bog.some(x => x.code === 'ok')) head = 'Едет по колёсам: принятая тележка согласна с прогнозом модели.';
    else head = 'Между показаниями датчиков: до следующего показания скорость идёт по прогнозу.';
    const lines = [];
    lines.push(`Режим: ${M_RU[mode]} — ${M_SUB[mode]}.`);
    const muN = p.mu_nominal;
    lines.push(o.mu < muN - 0.01 ? `Сцепление по оценке модели μ = ${f2(o.mu)} (номинал ${f2(muN)}): были признаки срыва, поэтому прогноз не разгоняет и не тормозит вагон силой, которой на скользком рельсе нет. Восстанавливается медленно (~${Math.round(p.tau_rec)} с).`
      : `Сцепление по оценке модели μ = ${f2(o.mu)} — номинал сухого рельса, признаков срыва нет.`);
    lines.push(`Масштабы привода: тяга k_т = ${f2(o.k_t)}, тормоз k_б = ${f2(o.k_b)}${p.adapt_on ? '' : ' (в листе жюри адаптация выключена — масштабы постоянны)'}. Возмущение d = ${f2(o.d)} м/с² — уклон, масса, ветер; подстраивается только на выбеге.`);
    lines.push(`Неопределённость: скорость ±2σ = ±${f1(2 * o.sigma_v * KMH)} км/ч, путь ±2σ = ±${f1(2 * o.sigma_s)} м. ${o.valid ? 'Оценка достоверна.' : o.ambiguous ? 'Оценка помечена недостоверной до конца неоднозначности.' : 'Оценка недостоверна: принятых показаний колёс нет дольше 1 с.'}`);
    lines.push(eng.gnssOff ? 'GNSS: сигнала нет — модели всё равно: спутники нужны ей только в первые 3 с для выставки положения.' : 'GNSS: нужен модели только в первые 3 с (выставка положения на старте); скорость и путь от него не зависят.');
    // правда имитатора
    const pl = eng.plant, truth = [], c = eng.cond;
    const slipTxt = b => { const x = pl.slip(b); return Math.abs(x) < 0.03 || pl.v < 0.3 && pl.w[b] < 0.3 ? 'катится нормально' : x > 0 ? `буксует: колесо ${f1(pl.w[b] * KMH)} км/ч при вагоне ${f1(pl.v * KMH)} км/ч` : pl.w[b] < 0.05 ? `юз: колесо заблокировано, вагон скользит на ${f1(pl.v * KMH)} км/ч` : `юз: колесо ${f1(pl.w[b] * KMH)} км/ч при вагоне ${f1(pl.v * KMH)} км/ч`; };
    const FR = { zero: 'датчик показывает 0', stuck: 'датчик залип', dropout: 'сообщений нет', noise: 'датчик шумит ×5', outliers: 'выбросы ×3' };
    for (let b = 0; b < 2; b++) truth.push(`${Name[b]}: ${slipTxt(b)}${c.faults[b] ? '; ' + FR[c.faults[b].kind] : ''}${c.drop ? '; сообщения пропали' : ''}.`);
    const wxTxt = c.weatherB[0] === c.weatherB[1] ? P.WEATHER[c.weatherB[0]].ru : `под передней — ${P.WEATHER[c.weatherB[0]].ru}, под задней — ${P.WEATHER[c.weatherB[1]].ru}`;
    truth.push(`Рельс: ${wxTxt} (μ пика ${f2(pl.mu_peak[0])} / ${f2(pl.mu_peak[1])}); масса ×${f2(c.mass)} от номинала листа; уклон ${f1(1000 * Math.tan(pl.grade))} ‰; ручка ${pl.notch > 0 ? '+' : ''}${pl.notch}.`);
    return { head, lines, bogies: bog, truth, mode, valid: o.valid, amb: o.ambiguous };
  }

  // прогон сценария целиком (Node: таблица docs/SANDBOX.md; страница: проверка)
  function runPreset(key, opts) {
    const eng = new Engine(Object.assign({}, opts, { preset: key }));
    while (!eng.done) eng.advance(1.0);
    return eng;
  }
  return { Engine, Driver, Track, PRESETS, explain, runPreset, M_RU, M_SUB, WHY_RU, MODES, KMH };
})();
if (typeof module !== 'undefined') module.exports = TramSandbox;
