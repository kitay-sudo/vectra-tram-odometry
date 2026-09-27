// ---- Связка ядра с входными сообщениями: JS-зеркало части runner.py (Runner), которую видит ядро ----
// Сетка шага по меткам сообщений (узлы кратны dt), маска свежих показаний по тележкам,
// разомкнутый режим при молчании колёс (wheel_timeout), достоверность ручки (handle_timeout),
// приведение свежего показания к моменту шага по ускорению модели (age_comp),
// ускорение выхода, σ пути выхода (position_sigma от точки выставки), причинная база
// «только колесо» (NaiveCore из tools/eval_replay.py: среднее свежих показаний, путь - интеграл
// на сетке). Сверка с Python-связкой на реальной записи: js-port/record_runner.py +
// compare_runner.js (входит в check.sh).
//
// ЗЕРКАЛО (ros2_ws/src/tram_state_estimator/tram_state_estimator/runner.py, класс Runner):
//   __init__ (поля входов), on_wheel, on_handle, _start, _node, _due, _run_to, _v_limit,
//   _meas_at_step, _step (без положения); tools/eval_replay.py: NaiveCore._naive.
//   Не перенесено (в песочнице метки идут подряд, GNSS после старта нет): своя база времени
//   (_in_base, кандидаты, reset, отложенные сообщения), сброс ядра при неконечном состоянии,
//   выставка и карта (Position). Сверка проверяет, что в записи эти ветки не сработали.
const TramRunner = (() => {
  const T = (typeof TramEst !== 'undefined') ? TramEst : require('./est.js');
  const LINEAR = { m_s: 1.0, km_h: 1.0 / 3.6 };

  class Runner {
    // params - лист ядра (как Params), opts - параметры ноды: wheel_timeout_s, handle_timeout_s
    constructor(params, opts = {}) {
      this.core = new T.Estimator(params);
      this.p = this.core.p;
      this.nw = this.core.nw;
      this.meas = new Array(this.nw).fill(0);
      this.fresh = new Array(this.nw).fill(false);
      this.t_wheel = new Array(this.nw).fill(-Infinity);
      this.notch = 0.0;
      this.t_notch = -Infinity;
      this.t = null;
      this.wheel_timeout = opts.wheel_timeout_s ?? 1.0;
      this.handle_timeout = opts.handle_timeout_s ?? 0.5;
      this.last = null;
      this._k0 = null; this._n = 0; this._chg = null;
      this.stamp_ok = false; this.stamp_last = null;
      this.rejected_stamps = 0; this.rejected_values = 0;
      this.MAX_STEPS = 200; this.V_LIMIT = 1.5; this.HANDLE_LIMIT = 15.0; this.AGE_MAX_S = 0.3;
      this.age_comp = true;
      this._vlim = null;
      this._perUnit = T.sensor_to_speed([1.0], this.p)[0];     // м/с на единицу показания
      // база «только колесо» - на той же сетке, из тех же входов
      this.naive = { v: 0.0, s: 0.0, n: 0 };
      this.s_ref = 0.0;                                        // путь ядра в момент выставки (старт)
    }
    _node(n) { return (this._k0 + n) * this.p.dt; }
    _start(stamp) {
      this._k0 = Math.floor(stamp / this.p.dt + 1e-6);
      this._n = 0;
      this.t = this._node(0);
      this.stamp_ok = true; this.stamp_last = stamp;
    }
    _due(stamp) { return Math.floor((stamp + 1e-6 - this._node(this._n)) / this.p.dt); }
    _advance(stamp) {
      this.stamp_ok = false;
      if (!Number.isFinite(stamp) || stamp <= 0.0) { this.rejected_stamps++; return []; }
      if (this.t === null) { this._start(stamp); return []; }
      this.stamp_ok = true; this.stamp_last = stamp;
      return this._run_to(stamp);
    }
    _run_to(stamp) {
      let due = this._due(stamp);
      if (due <= 0) return [];
      due = Math.min(due, this.MAX_STEPS);
      const outs = [];
      for (let k = 0; k < due; k++) { this._n += 1; this.t = this._node(this._n); outs.push(this._step()); }
      return outs;
    }
    // шаги сетки до stamp без входного сообщения (пульс: входы молчат)
    tick(stamp) { if (this.t === null || !Number.isFinite(stamp)) return []; return this._run_to(stamp); }
    _v_limit() {
      if (this._vlim === null) this._vlim = this.V_LIMIT * this.p.v_max_line / this._perUnit;
      return this._vlim;
    }
    on_wheel(i, stamp, value) {
      const out = this._advance(stamp);
      if (!this.stamp_ok) return out;
      const v = Number(value);
      if (!(i >= 0 && i < this.nw && Number.isFinite(v) && Math.abs(v) <= this._v_limit())) { this.rejected_values++; return out; }
      if (this._chg === null) this._chg = new Array(this.nw).fill(false);
      this._chg[i] = v !== this.meas[i];
      this.meas[i] = v;
      this.fresh[i] = true;
      this.t_wheel[i] = this.stamp_last;
      return out;
    }
    on_handle(stamp, position) {
      const out = this._advance(stamp);
      if (!this.stamp_ok) return out;
      const n = Number(position);
      if (!Number.isFinite(n)) { this.rejected_values++; return out; }
      this.notch = Math.min(this.HANDLE_LIMIT, Math.max(-this.HANDLE_LIMIT, n));
      this.t_notch = this.stamp_last;
      return out;
    }
    _meas_at_step() {
      if (!this.age_comp || this.last === null || this._chg === null) return this.meas;
      const m = this.meas.map((x, i) => this.fresh[i] && this._chg[i] && x > 0.0);
      if (!m.some(Boolean)) return this.meas;
      return this.meas.map((x, i) => {
        if (!m[i]) return x;
        const age = Math.min(this.AGE_MAX_S, Math.max(0.0, this.t - this.t_wheel[i]));
        return Math.max(x + this.last.a * age / this._perUnit, 0.0);
      });
    }
    _naive() {
      const p = this.p, vals = [];
      for (let i = 0; i < this.nw; i++) if (this.t - this.t_wheel[i] <= this.wheel_timeout && Number.isFinite(this.meas[i])) vals.push(this.meas[i]);
      const kv = LINEAR[p.meas_units] !== undefined ? p.meas_scale * LINEAR[p.meas_units] : this._perUnit;
      if (vals.length) this.naive.v = Math.max(0.0, vals.reduce((a, b) => a + b, 0) / vals.length * kv);
      this.naive.s += this.naive.v * p.dt;
      this.naive.n = vals.length;
    }
    _step() {
      const c = this.core, t = this.t, p = this.p;
      const wheels_stale = (t - Math.max(...this.t_wheel)) > this.wheel_timeout;
      const handle_ok = (t - this.t_notch) <= this.handle_timeout;
      this._naive();
      let o;
      if (wheels_stale) o = c.step_open_loop(this.notch);
      else o = c.step(this.notch, this._meas_at_step(), this.fresh.slice(), handle_ok);
      this.fresh.fill(false);
      const s = c.x[T.IS], v = c.x[T.IV];
      // σ положения выхода: путь после выставки (в песочнице выставка - на старте)
      o.sigma_s_core = o.sigma_s;
      o.sigma_s = T.position_sigma(o.sigma_s, s - this.s_ref, p);
      const a = (T.body_force(c.u_filt, v, c.x[T.IKT], c.x[T.IKB], c.mu, p) - T.resistance(v, p)) / p.M_nom + c.x[T.ID];
      Object.assign(o, {
        stamp: t, a: (v > 0 || a > 0) ? a : 0.0, handle_ok, wheels_stale, notch: this.notch,
        naive_v: this.naive.v, naive_s: this.naive.s, naive_n: this.naive.n,
      });
      this.last = o;
      return o;
    }
  }
  return { Runner };
})();
if (typeof module !== 'undefined') module.exports = TramRunner;
