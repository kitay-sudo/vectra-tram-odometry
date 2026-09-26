// ---- Имитатор трамвая для песочницы («истина»): 2 тележки, сцепление, колёса, датчики ----
// ЭТО ПРИБЛИЖЕНИЕ, а не модель пакета. Оценку в песочнице считает настоящее ядро (est.js —
// порт estimator_core.py, сверен с Python) с листом жюри; имитатор только даёт ему входы,
// похожие на настоящие, и знает правду, с которой сравнивается оценка.
//
// Что взято из данных и листа: сила привода и торможения — та же откалиброванная таблица
// A(u, v) листа (сила = M_ном·(A(u, v) − A(0, v)), сопротивление — строка выбега), позиции ручки
// и запаздывание привода — из листа; датчики — как в записях: км/ч, общая метка обеих тележек,
// ~9,2 Гц (0,1 с с разбросом, ~8 % тактов пропущено), шум ~0,04 км/ч, масштаб колёс = 1/meas_scale.
// Что придумано (подобрано на глаз, «имитатор»): характеристика сцепления μ(проскальзывание) с
// падающей ветвью, приведённая масса колёс, защита от буксования и юза, μ для погоды.
const TramPlant = (() => {
  const G = 9.81;
  const T = (typeof TramEst !== 'undefined') ? TramEst : require('./est.js');

  // Константы имитатора. Все значения — приближение (подписано на странице «имитатор»).
  const PLANT = {
    m_wheel: 1000,        // кг: приведённая к ободу масса вращающихся частей одной тележки
    xi_peak: 0.012,       // проскальзывание в пике сцепления (крип 1,2 %)
    mu_fall: 0.55,        // доля μ пика при большом проскальзывании (падающая ветвь)
    v_eps: 0.5,           // м/с: регуляризация проскальзывания у нуля
    wsp: true,            // защита от буксования и юза (снимает момент тележки при срыве)
    wsp_on: 0.10,         // проскальзывание: включить
    wsp_off: 0.03,        // выключить
    wsp_min: 0.15,        // до какой доли снимается момент
    wsp_down: 4.0,        // 1/с: темп снятия
    wsp_up: 0.8,          // 1/с: темп возврата
    sens_sigma_kmh: 0.04, // шум датчика, км/ч (оценка по записям: разность тележек σ 0,06 км/ч)
    sens_delay: 0.05,     // с: показание — среднее за такт, закончившийся столько назад (возраст ~0,1 с, docs/audit/CORE.md)
    wheel_scale: [1.002, 0.998], // износ бандажей тележек: ±0,2 % от номинала (по вагонам и датам ±0,5 %)
    sens_period: 0.1,     // с: такт датчиков
    sens_jitter: 0.012,   // с: разброс метки такта
    sens_skip: 0.08,      // доля пропущенных тактов (в записях ~8 % интервалов около 0,2 с)
    handle_period: 0.05,  // с: ручка 20 Гц
  };
  // Погода: μ пика для передней и задней тележки (передняя «чистит» рельс, у неё хуже).
  const WEATHER = {
    dry: { ru: 'сухо', mu: [0.30, 0.32] },
    rain: { ru: 'дождь', mu: [0.075, 0.095] },
    leaves: { ru: 'листопад', mu: [0.045, 0.06] },
    ice: { ru: 'снег / наледь', mu: [0.05, 0.065] },
  };

  function rng(seed) {                  // mulberry32: воспроизводимо
    let a = seed >>> 0;
    return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  }
  function gauss(r) { let u = 0, v = 0; while (u === 0) u = r(); v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

  // нормированная характеристика сцепления: 0 -> 1 (пик при xi_peak) -> mu_fall при большом срыве
  function muShape(xi) {
    const x = Math.abs(xi) / PLANT.xi_peak, f = 2 * x / (1 + x * x);
    return x <= 1 ? f : PLANT.mu_fall + (1 - PLANT.mu_fall) * f;
  }

  class Plant {
    // sheet — лист ядра (как Params; таблица A(u, v), позиции ручки, запаздывание привода)
    constructor(sheet, opts = {}) {
      this.p = Object.assign({}, T.DEFAULT, sheet);
      this.C = Object.assign({}, PLANT, opts.plant || {});
      this.r = rng(opts.seed ?? 1);
      this.t = 0; this.s = opts.s0 ?? 0; this.v = 0;
      this.w = [0, 0];                   // скорость обода тележек, м/с (0 — передняя, 1 — задняя)
      this.wsp_g = [1, 1];               // доля момента после защиты
      this.wsp_act = [false, false];
      this.mass_factor = 1.0;
      this.grade = 0;                    // уклон, рад (+ подъём)
      this.mu_peak = WEATHER.dry.mu.slice();
      this.notch = 0;
      this.u = 0;                        // нормированный момент после запаздывания и инерции привода
      const nd = Math.max(1, Math.round(this.p.delay_drive / 0.001));
      this.u_hist = new Array(nd).fill(0); this.u_hi = 0;
      this.Ft = [0, 0]; this.Fa = [0, 0];
      // датчики
      this.faults = [null, null];        // { kind: zero|stuck|dropout|noise|outliers }
      this.sens_next = this.C.sens_period;
      this.sens_k = 1;
      this.w_hist = [];                  // скорость обода за последние ~0,3 с (для задержки датчика)
      this.win_t0 = 0;
      this.last_out = [0, 0];
      this.scale_true = 1 / this.p.meas_scale;     // колёса читают меньше на meas_scale
      this.handle_next = 0.0;
      this.dropAll = false;              // пропуск сообщений обеих тележек
      this.handleOff = false;            // нет сообщений ручки
    }
    // сила привода тележки по таблице листа (на ободе); уставку привода и тормоза задаёт
    // регулятор по скорости вагона, поэтому таблица берётся по скорости корпуса
    _drive(u, w) {
      const p = this.p;
      if (u === 0) return 0;
      return 0.5 * p.M_nom * (T.table_acc(u, w, p) - T.table_acc(0.0, w, p));
    }
    _resist(v) {          // сопротивление движению — строка выбега таблицы, пропорционально массе
      const p = this.p, a = Math.abs(v);
      if (a <= 0) return 0;
      return this.mass_factor * p.M_nom * Math.max(0.0, -T.table_acc(0.0, a, p));
    }
    _Fa(b, w, v, N) {     // сила сцепления тележки b на корпус
      const xi = (w - v) / Math.max(Math.abs(v), this.C.v_eps);
      return Math.sign(xi) * this.mu_peak[b] * muShape(xi) * N;
    }
    step(dt) {
      const p = this.p, C = this.C, M = this.mass_factor * p.M_nom, N = M * G / 2;
      this.t += dt;
      // привод: ручка -> доля момента по таблице позиций, чистое запаздывание, инерция
      const target = T.notch_to_u(this.notch, p);
      this.u_hist[this.u_hi] = target; this.u_hi = (this.u_hi + 1) % this.u_hist.length;
      const delayed = this.u_hist[this.u_hi];
      this.u += (delayed - this.u) * dt / p.tau_drive;
      if (Math.abs(this.u) < 1e-6) this.u = 0;
      // колёса: линейно-неявный шаг (жёсткая характеристика крипа), затем корпус
      let Fsum = 0;
      for (let b = 0; b < 2; b++) {
        const w = this.w[b], F_dem = this._drive(this.u, this.v);
        // защита от буксования и юза: при срыве момент тележки снимается и подаётся снова
        const xi = (w - this.v) / Math.max(Math.abs(this.v), C.v_eps);
        if (C.wsp) {
          if (!this.wsp_act[b] && Math.abs(xi) > C.wsp_on && Math.abs(w - this.v) > 0.3) this.wsp_act[b] = true;
          if (this.wsp_act[b] && Math.abs(xi) < C.wsp_off) this.wsp_act[b] = false;
          this.wsp_g[b] += this.wsp_act[b] ? (C.wsp_min - this.wsp_g[b]) * Math.min(1, C.wsp_down * dt) : (1 - this.wsp_g[b]) * Math.min(1, C.wsp_up * dt);
        } else this.wsp_g[b] = 1;
        const Ft = F_dem * this.wsp_g[b];
        const Fa0 = this._Fa(b, w, this.v, N), h = 1e-4;
        const dF = Math.max(0, (this._Fa(b, w + h, this.v, N) - Fa0) / h);
        let wn = w + dt * (Ft - Fa0) / C.m_wheel / (1 + dt * dF / C.m_wheel);
        if (wn < 0) wn = 0;                                  // тормоз держит колесо: назад не крутится (юз до блокировки)
        this.w[b] = wn;
        this.Ft[b] = Ft;
        this.Fa[b] = this._Fa(b, wn, this.v, N);
        Fsum += this.Fa[b];
      }
      let a = (Fsum - this._resist(this.v) - M * G * Math.sin(this.grade)) / M;
      let vn = this.v + a * dt;
      if (vn < 0) { vn = 0; a = 0; }                          // стоит на тормозе (назад не катится)
      this.s += 0.5 * (this.v + vn) * dt;
      this.v = vn; this.a = a;
      // датчики: история скорости обода (среднее за такт берётся с задержкой sens_delay)
      this.w_hist.push([this.t, this.w[0], this.w[1]]);
      if (this.w_hist.length > 600) this.w_hist.splice(0, 200);
    }
    // сообщения за шаг: [{kind:'wheel', i, stamp, value} | {kind:'handle', stamp, value}]
    poll() {
      const out = [], C = this.C;
      if (this.t + 1e-9 >= this.handle_next) {
        if (!this.handleOff) out.push({ kind: 'handle', stamp: this.t, value: this.notch });
        this.handle_next += C.handle_period;
      }
      if (this.t + 1e-9 >= this.sens_next) {
        const b1 = this.t - C.sens_delay, b0 = Math.min(b1 - 0.02, this.win_t0 - C.sens_delay), mean = [0, 0];
        let n = 0;
        for (const [t, w0, w1] of this.w_hist) if (t > b0 && t <= b1) { mean[0] += w0; mean[1] += w1; n++; }
        if (n) { mean[0] /= n; mean[1] /= n; } else { mean[0] = this.w[0]; mean[1] = this.w[1]; }
        this.win_t0 = this.t;
        const skip = this.r() < C.sens_skip;
        if (!skip && !this.dropAll) {
          for (let b = 0; b < 2; b++) {
            const f = this.faults[b];
            if (f && f.kind === 'dropout') continue;
            let kmh = mean[b] * 3.6 * this.scale_true * C.wheel_scale[b] + C.sens_sigma_kmh * gauss(this.r);
            if (f && f.kind === 'noise') kmh += (f.sigma_kmh ?? 0.9) * gauss(this.r);
            if (f && f.kind === 'outliers' && this.r() < (f.p ?? 0.03)) kmh *= 3;
            if (f && f.kind === 'zero') kmh = 0;
            if (f && f.kind === 'stuck') kmh = f.value ?? (f.value = this.last_out[b]);
            kmh = Math.max(0, Math.round(kmh * 1000) / 1000);
            this.last_out[b] = kmh;
            out.push({ kind: 'wheel', i: b, stamp: this.t, value: kmh });
          }
        }
        this.sens_k++;
        this.sens_next = this.sens_k * C.sens_period + C.sens_jitter * Math.max(-2.5, Math.min(2.5, gauss(this.r)));
        if (this.sens_next <= this.t) this.sens_next = this.t + 0.02;
      }
      return out;
    }
    setFault(b, f) { this.faults[b] = f ? Object.assign({}, f) : null; }
    setWeather(key) { this.mu_peak = (WEATHER[key] || WEATHER.dry).mu.slice(); }
    slip(b) { return (this.w[b] - this.v) / Math.max(Math.abs(this.v), this.C.v_eps); }
  }
  return { Plant, PLANT, WEATHER, rng, gauss, muShape };
})();
if (typeof module !== 'undefined') module.exports = TramPlant;
