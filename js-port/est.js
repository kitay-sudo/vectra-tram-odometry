// ---- Tram state estimator: JavaScript port of estimator_core.py (UKF over the motion model) ----
// Порт ядра пакета строка в строку (сверка: bash js-port/check.sh, допуск 1e-6 по v, s, σ,
// μ, k, d; режимы и неоднозначность - совпадение на каждом шаге).
//
// ЗЕРКАЛО (что перенесено из ros2_ws/src/tram_state_estimator/tram_state_estimator/
// estimator_core.py; при правке ядра перенести те же места):
//   Params (DEFAULT ниже: поля и значения по умолчанию), notch_to_u, sensor_to_speed,
//   sensor_layout, _cell, table_acc, _drive_nominal, drive_force, rated_force, resistance,
//   axle_load, body_force, axle_shares, creep, f_process, h_axles, h_axles_batch,
//   position_sigma, MAD_TO_SIGMA; Estimator: __init__, _drive_model, _mode, _diagnose,
//   _axle_speeds, _noise, _dot_tol, _calibrate, _chol (+ eigh), _sigma, _predict, _a_limit,
//   _meas_var (+ шум сверх паспортного), _a_model, _live, _slip_all_update,
//   _slip_all_return, _slip_all_end, _level_agree, _correct (срыв всех осей: ep / ep_prev /
//   ended_zero / ep_rej, признак насыщения с apart, slip_ok_t0), step (маска fresh, notch_last,
//   оценка шума, скачок против тренда с правилом отголоска, axle_jump, первая инициализация),
//   _frozen_all, _adapt_scale (окно по реальному времени, adapt_on), step_open_loop,
//   _sigma_v_out, _finish (slip_all, meas_noise).
//   Срыв всех осей и оценка шума показаний перенесены вместе с ядром.
//   Не перенесено (на выход не влияет): проверка листа Params.__post_init__, adapt_stats.
//   Добавлено (не из ядра, на числа не влияет): diag() - запись решений фильтра по осям для
//   панели «Что сейчас думает модель» в песочнице.
//
// PORT - ядро, с которым порт сверен: sha1 estimator_core.py (переводы строк LF, 12 знаков;
// так же его пишут tools/export_replay.py и js-port/record*.py). Страница сравнивает его с
// ядром, которым посчитаны прогоны комиссии, и при расхождении подписывает песочницу
// «упрощённое ядро». Копия для страницы - simulator/js/est.js (байт в байт; её пишет
// node js-port/sync.js, проверяют js-port/check.sh и simulator/test/page_test.js).
const TramEst = (() => {
  const G = 9.81;
  const MAD_TO_SIGMA = 1.4826;                      // σ = 1,4826 · медиана |нормальной величины|
  const PORT = { core_git: '3466ac4', core_sha1: '684530832766', dv: '≤ 2·10⁻¹² м/с' };
  const DEFAULT = {
    M_nom: 28000.0, r_nom: 0.35, n_axles: 4, driven: [true, true, false, false], braked: [true, true, true, true], v_max_line: 20.0,
    F_notch: 36000.0, F_brake: 30000.0, v_base: 8.0, v_ed_fade: 1.5, brake_hold_frac: 1.0,
    notch_tract: [0.25, 0.5, 0.75, 1.0], notch_brake: [0.25, 0.5, 0.75, 1.0], acc_u: [], acc_v: [], acc_table: [],
    tau_drive: 0.30, delay_drive: 0.10, T_dead: 0.05, t_confirm: 0.15,
    res_A: 2400.0, res_B: 55.0, res_C: 3.5,
    mu_nominal: 0.25, mu_min: 0.02, tau_sat: 0.3, tau_rec: 30.0, c_creep: 0.020, c_creep_drag: 0.002, sigma_creep: 0.004,
    a_max_acc: 1.5, a_max_brake: 3.0, theta_max: 0.06, d_extra: 0.30, a_free_decel: 0.75, lock_ratio: 0.5, a_slip_margin: 1.0,
    slip_all_on: true, slip_jump: 0.3, slip_rel: 0.2, slip_pair_s: 0.5, slip_cont_k: 2.0, slip_t_max: 12.0, slip_ok_s: 1.0,
    slip_ret_a: 0.7, slip_sigma_a: 0.15, noise_n: 12, noise_k: 3.0,
    meas_units: 'rad_s', ppr: 200, meas_scale: 1.0, sensor_ratio: 1.0, sensors_per_axle: 2, sensor_axles: [true, true, true, true],
    curve_ratio_max: 0.038, sigma_meas: 0.15, stuck_n: 60, stuck_dv: 0.5, recover_tol: 0.5, recover_rel: 0.1, t_recover: 2.0,
    v_dead_ref: 2.0, axle_dot_alpha: 0.2, calib_gain: 0.01, calib_v_min: 2.0, scale_range: 0.10,
    gate_nis: 9.0, sigma_rej_frac: 0.5, q_v: 0.05, q_d: 0.02, q_k: 0.003, p0_s: 1.0, p0_v: 1.0, p0_d: 0.30, p0_k: 0.10,
    k_min: 0.4, k_max: 1.6, adapt_on: true, t_adapt: 3.0, sigma_k_meas: 0.05, adapt_f_min: 0.3, t_sat_hold: 3.0, v_adapt_min: 2.0, t_valid: 1.0,
    agree_tol: 0.3, agree_age: 0.3, v_standstill: 0.30, t_standstill: 0.30,
    sv_gain: 1.0, sv_floor: 0.0, sv_floor_stand: 0.0, sv_age: 0.0, sv_rel: 0.0, ss_map: 0.0, ss_rel: 0.0,
    dt: 0.01
  };
  const ANGULAR_UNITS = { rad_s: 1.0, rpm: 2.0 * Math.PI / 60.0, hz: 2.0 * Math.PI };
  const LINEAR_UNITS = { m_s: 1.0, km_h: 1.0 / 3.6 };
  const COAST = 0, TRACTION = 1, BRAKE = 2, TRANSITION = 3, SLIP = 4, STANDSTILL = 5, DEGRADED = 6;
  const MODE_NAMES = ['COAST', 'TRACTION', 'BRAKE', 'TRANSITION', 'SLIP', 'STANDSTILL', 'DEGRADED'];
  const NX = 5, IS = 0, IV = 1, ID = 2, IKT = 3, IKB = 4;
  const clip = (x, a, b) => Math.min(b, Math.max(a, x));
  const count = a => a.reduce((p, c) => p + (c ? 1 : 0), 0);
  const median = a => { const s = a.slice().sort((p, q) => p - q), n = s.length; return n % 2 ? s[(n - 1) / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]); };

  // ---------------- модель ----------------
  function interpGrid(x, fp) {                      // np.interp(x, 0..N, fp): за краем - край
    const N = fp.length - 1;
    if (x >= N) return fp[N];
    if (x <= 0) return fp[0];
    const j = Math.floor(x);
    if (j === x) return fp[j];
    const slope = (fp[j + 1] - fp[j]) / 1.0;
    return slope * (x - j) + fp[j];
  }
  function notch_to_u(n, p) {
    if (n > 0) return interpGrid(n, [0.0].concat(p.notch_tract));
    if (n < 0) return -interpGrid(-n, [0.0].concat(p.notch_brake));
    return 0.0;
  }
  function sensor_to_speed(meas, p) {
    const unit = p.meas_units;
    return meas.map(v => {
      const m = v * p.meas_scale;
      if (unit in LINEAR_UNITS) return m * LINEAR_UNITS[unit];
      const k = unit === 'pulses_s' ? 2.0 * Math.PI / p.ppr : ANGULAR_UNITS[unit];
      return m * k / p.sensor_ratio * p.r_nom;
    });
  }
  function sensor_layout(p) {
    const slots = []; let i = 0;
    for (let a = 0; a < p.n_axles; a++) {
      slots.push([]);
      if (p.sensor_axles[a]) { for (let k = 0; k < p.sensors_per_axle; k++) slots[a].push(i + k); i += p.sensors_per_axle; }
    }
    return [slots, i];
  }
  const hasTable = p => p.acc_table && p.acc_table.length > 0;
  function cell(grid, x) {                          // bisect_right - 1, края - край
    let i = -1; for (let k = 0; k < grid.length; k++) if (grid[k] <= x) i = k; else break;
    i = i < 0 ? 0 : (i > grid.length - 2 ? grid.length - 2 : i);
    const w = (x - grid[i]) / (grid[i + 1] - grid[i]);
    return [i, w < 0.0 ? 0.0 : w > 1.0 ? 1.0 : w];
  }
  function table_acc(u, v, p) {
    const nv = p.acc_v.length, T = (i, j) => p.acc_table[i * nv + j];
    const [i, wu] = cell(p.acc_u, u), [j, wv] = cell(p.acc_v, Math.abs(v));
    const lo = (1.0 - wv) * T(i, j) + wv * T(i, j + 1);
    const hi = (1.0 - wv) * T(i + 1, j) + wv * T(i + 1, j + 1);
    return (1.0 - wu) * lo + wu * hi;
  }
  function drive_nominal(u, v, p) {
    if (hasTable(p)) return p.M_nom * (table_acc(u, v, p) - table_acc(0.0, v, p));
    const va = Math.abs(v);
    if (u > 0) return p.F_notch * u * Math.min(1.0, p.v_base / Math.max(va, 1e-3));
    const ed = Math.min(1.0, va / p.v_ed_fade);
    return p.F_brake * u * (p.brake_hold_frac + (1.0 - p.brake_hold_frac) * ed);
  }
  const drive_force = (u, v, kt, kb, p) => u === 0 ? 0.0 : (u > 0 ? kt : kb) * drive_nominal(u, v, p);
  function rated_force(traction, p) {
    if (!hasTable(p)) return traction ? p.F_notch : p.F_brake;
    return Math.max(...p.acc_v.map(v => Math.abs(drive_nominal(traction ? 1.0 : -1.0, v, p))));
  }
  function resistance(v, p) {
    const a = Math.abs(v);
    let W = p.res_A * (a > 0 ? 1 : 0) + p.res_B * a + p.res_C * a * a;
    if (hasTable(p) && a > 0) W += Math.max(0.0, -p.M_nom * table_acc(0.0, a, p));
    return W;
  }
  const axle_load = p => p.M_nom * G / p.n_axles;
  function body_force(u, v, kt, kb, mu, p) {
    const F = drive_force(u, v, kt, kb, p), N = axle_load(p);
    if (F > 0) return Math.min(F, mu * N * Math.max(count(p.driven), 1));
    return Math.max(F, -mu * N * Math.max(count(p.braked), 1));
  }
  function axle_shares(F, p) {
    let sel, n;
    if (F > 0) { sel = p.driven; n = Math.max(count(p.driven), 1); }
    else if (F < 0) { sel = p.braked; n = Math.max(count(p.braked), 1); }
    else return new Array(p.n_axles).fill(0);
    return sel.map(s => s ? 1.0 / n : 0.0);
  }
  const creep = (F, p) => { const N = axle_load(p); return axle_shares(F, p).map(sh => p.c_creep * F * sh / N - p.c_creep_drag); };
  function f_process(x, u, mu, p) {
    const [s, v, d, kt, kb] = x;
    let a = (body_force(u, v, kt, kb, mu, p) - resistance(v, p)) / p.M_nom + d;
    if (v <= 0.0 && a < 0.0) a = 0.0;
    return [s + v * p.dt, v + a * p.dt, d, kt, kb];
  }
  function h_axles(x, u, mu, p) {
    const v = x[IV], F = body_force(u, v, 1.0, 1.0, mu, p);
    return creep(F, p).map(xi => v * (1.0 + xi));
  }
  // публикуемая σ положения вдоль пути (связка: путь ds после выставки или привязки)
  const position_sigma = (sigma_s, ds, p) => Math.sqrt(sigma_s * sigma_s + p.ss_map * p.ss_map + (p.ss_rel * ds) ** 2);
  function h_axles_batch(pts, u, mu, p) {
    const N = axle_load(p), sh = axle_shares(Math.sign(u), p);
    return pts.map(pt => {
      const v = pt[IV];
      let F;
      if (u > 0) F = Math.min(drive_nominal(u, v, p), mu * N * Math.max(count(p.driven), 1));
      else if (u < 0) F = Math.max(drive_nominal(u, v, p), -mu * N * Math.max(count(p.braked), 1));
      else F = 0.0;
      return sh.map(s => v * (1.0 + (p.c_creep * F * s / N - p.c_creep_drag)));
    });
  }

  // ---------------- линейная алгебра 5×5 ----------------
  const zeros = n => Array.from({ length: n }, () => new Array(n).fill(0));
  function cholRaw(A) {
    const n = A.length, L = zeros(n);
    for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
      let s = A[i][j];
      for (let k = 0; k < j; k++) s -= L[i][k] * L[j][k];
      if (i === j) { if (!(s > 0)) return null; L[i][i] = Math.sqrt(s); } else L[i][j] = s / L[j][j];
    }
    return L;
  }
  function eighJacobi(A) {
    const n = A.length, a = A.map(r => r.slice()), V = zeros(n);
    for (let i = 0; i < n; i++) V[i][i] = 1;
    for (let sweep = 0; sweep < 60; sweep++) {
      let off = 0;
      for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
      if (off < 1e-30) break;
      for (let pI = 0; pI < n; pI++) for (let q = pI + 1; q < n; q++) {
        if (Math.abs(a[pI][q]) < 1e-300) continue;
        const th = (a[q][q] - a[pI][pI]) / (2 * a[pI][q]);
        const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) { const akp = a[k][pI], akq = a[k][q]; a[k][pI] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
        for (let k = 0; k < n; k++) { const apk = a[pI][k], aqk = a[q][k]; a[pI][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
        for (let k = 0; k < n; k++) { const vkp = V[k][pI], vkq = V[k][q]; V[k][pI] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
      }
    }
    return { w: a.map((r, i) => r[i]), V };
  }
  function chol(A0) {
    const n = A0.length, A = zeros(n);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) A[i][j] = 0.5 * (A0[i][j] + A0[j][i]);
    const B0 = zeros(n);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) B0[i][j] = A[i][j] + (i === j ? 1e-12 : 0);
    const L = cholRaw(B0);
    if (L) return L;
    const { w, V } = eighJacobi(A), B = zeros(n);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      let s = 0; for (let k = 0; k < n; k++) s += V[i][k] * Math.max(w[k], 1e-10) * V[j][k];
      B[i][j] = s + (i === j ? 1e-12 : 0);
    }
    return cholRaw(B);
  }
  const sym = P => { for (let i = 0; i < NX; i++) for (let j = i + 1; j < NX; j++) { const m = 0.5 * (P[i][j] + P[j][i]); P[i][j] = m; P[j][i] = m; } return P; };

  // ---------------- фильтр ----------------
  class Estimator {
    constructor(params) {
      const p = this.p = Object.assign({}, DEFAULT, params || {});
      this.x = [0, 0, 0, 1, 1];
      this.P = zeros(NX);
      [p.p0_s ** 2, p.p0_v ** 2, p.p0_d ** 2, p.p0_k ** 2, p.p0_k ** 2].forEach((v, i) => { this.P[i][i] = v; });
      this.mu = p.mu_nominal;
      this.mode = DEGRADED; this.mode_cand = DEGRADED; this.mode_timer = 0; this.u_filt = 0;
      const nd = Math.round(p.delay_drive / p.dt);
      this.notch_hist = new Array(nd + 1).fill(0.0);
      [this.slots, this.nw] = sensor_layout(p);
      this.healthy = new Array(this.nw).fill(true);
      this.stuck_cnt = new Array(this.nw).fill(0);
      this.stuck_ref = new Array(this.nw).fill(NaN);
      this.agree_t = new Array(this.nw).fill(0);
      this.prev_meas = new Array(this.nw).fill(0);
      this.axle_prev = new Array(p.n_axles).fill(0);
      this.axle_dot = new Array(p.n_axles).fill(0);
      this.axle_seen = new Array(p.n_axles).fill(false);
      this.axle_jump = new Array(p.n_axles).fill(false);   // скачок оси за интервал больше физически возможного
      this.axle_scale = new Array(p.n_axles).fill(1);
      // шум показаний оси: остатки линейной экстраполяции (вторые разности), окно noise_n, медиана
      this.nz_buf = Array.from({ length: p.n_axles }, () => []);   // deque(maxlen=noise_n)
      this.nz_cnt = new Array(p.n_axles).fill(0);
      this.axle_prev2 = new Array(p.n_axles).fill(0);
      this.gap_prev = new Array(p.n_axles).fill(p.dt);
      this.axle_gap = new Array(p.n_axles).fill(p.dt);
      this.nz_sigma = new Array(p.n_axles).fill(0);
      this.nz_excess = new Array(p.n_axles).fill(0);
      // срыв всех осей: момент, знак и величина последнего скачка оси против её тренда
      this.jump_t = new Array(p.n_axles).fill(-Infinity);
      this.jump_s = new Array(p.n_axles).fill(0);
      this.jump_m = new Array(p.n_axles).fill(0);
      this.slip_all = 0; this.slip_t0 = -Infinity; this.slip_j = 0.0;
      this.slip_back = new Set(); this.slip_timeout = [0, -Infinity]; this.slip_ok_t0 = null;
      this.slip_ret = new Set(); this.n_slip_all = 0; this.notch_last = 0.0;
      this.initialised = false; this.have_meas = false;
      this.t_meas_i = new Array(this.nw).fill(-Infinity);
      this.t_axle = new Array(p.n_axles).fill(-Infinity);
      this.amb = false; this.amb_v = 0; this.s_extra = 0; this.last_acc_n = 0;
      this.zero_t = 0; this.frozen = false; this.t_zero_start = null; this.adapting = false; this.mode_pre = DEGRADED; this.win_mode = null; this.t_last_sat = -1e9;
      this.win_t = 0; this.win_t_last = 0; this.win_v0 = 0; this.win_F = 0; this.win_W = 0; this.t_since_acc = 0; this.t = 0; this.sat = false;
      this.n_acc = 0; this.n_rej = 0; this.n_override = 0;
      const n = NX, alpha = 1.0, beta = 2.0, kappa = 1.0;
      this.lam = alpha * alpha * (n + kappa) - n;
      this.wm = new Array(2 * n + 1).fill(1.0 / (2.0 * (n + this.lam)));
      this.wc = this.wm.slice();
      this.wm[0] = this.lam / (n + this.lam);
      this.wc[0] = this.wm[0] + (1.0 - alpha * alpha + beta);
    }
    // Диагностика для песочницы (не из ядра): решение фильтра по каждой оси на последней
    // коррекции (why: ok | agree | spin | skid | jump | forced | held | gate | slipall), невязка и причина
    // исключения датчика (bad: stuck | dead). Только запись, на числа не влияет (check.sh).
    diag() {
      const na = this.p.n_axles;
      this.dg = { t: new Array(na).fill(-Infinity), nu: new Array(na).fill(NaN), nis: new Array(na).fill(NaN), why: new Array(na).fill(null), bad: new Array(this.nw).fill(null) };
      return this.dg;
    }
    _drive_model(notch) {
      const p = this.p;
      this.notch_hist.push(notch); this.notch_hist.shift();
      const target = notch_to_u(this.notch_hist[0], p);
      this.u_filt += (target - this.u_filt) * p.dt / p.tau_drive;
      return this.u_filt;
    }
    _mode(u) {
      const p = this.p, cand = Math.abs(u) < p.T_dead ? COAST : (u > 0 ? TRACTION : BRAKE);
      if (cand !== this.mode_cand) { this.mode_cand = cand; this.mode_timer = 0; return TRANSITION; }
      this.mode_timer += p.dt;
      return this.mode_timer >= p.t_confirm ? cand : TRANSITION;
    }
    _diagnose(meas, dts, fm) {
      const p = this.p, v_body = Math.abs(this.x[IV]);
      const body_ok = this.initialised && this.t_since_acc <= p.t_valid;
      const recent = this.t_meas_i.map(tm => (this.t - tm) <= p.t_valid);
      for (let i = 0; i < this.nw; i++) {
        if (!fm[i]) continue;
        const dt = dts[i], others = [];
        for (let j = 0; j < this.nw; j++) if (j !== i && this.healthy[j] && recent[j]) others.push(Math.abs(meas[j]));
        const ref = others.length ? median(others) : null;
        if (Math.abs(meas[i] - this.prev_meas[i]) > 1e-9) this.stuck_cnt[i] = 0;
        else {
          if (this.stuck_cnt[i] === 0) this.stuck_ref[i] = ref === null ? NaN : ref;
          this.stuck_cnt[i] += 1;
        }
        if (this.healthy[i]) {
          const stuck = this.stuck_cnt[i] > p.stuck_n && ref !== null && isFinite(this.stuck_ref[i]) && Math.abs(ref - this.stuck_ref[i]) > p.stuck_dv;
          const dead = ref !== null && Math.abs(meas[i]) < 1e-6 && ref > p.v_dead_ref && v_body > p.v_dead_ref;
          if (stuck || dead) { this.healthy[i] = false; this.agree_t[i] = 0.0; if (this.dg) this.dg.bad[i] = stuck ? 'stuck' : 'dead'; }
        } else {
          const w = Math.abs(meas[i]);
          const tol = p.recover_tol + p.recover_rel * Math.max(ref || 0.0, v_body);
          const agree = this.stuck_cnt[i] <= p.stuck_n && ((ref !== null && Math.abs(w - ref) <= tol) || (body_ok && Math.abs(w - v_body) <= tol));
          this.agree_t[i] = agree ? this.agree_t[i] + dt : 0.0;
          if (this.agree_t[i] >= p.t_recover) { this.healthy[i] = true; this.stuck_cnt[i] = 0; if (this.dg) this.dg.bad[i] = null; }
        }
      }
      for (let i = 0; i < this.nw; i++) if (fm[i]) this.prev_meas[i] = meas[i];
    }
    _axle_speeds(meas, fm) {
      const p = this.p, out = new Array(p.n_axles).fill(0), ok = new Array(p.n_axles).fill(false);
      for (let a = 0; a < p.n_axles; a++) {
        const hs = this.slots[a].filter(i => this.healthy[i] && fm[i]).map(i => meas[i]);
        if (hs.length) { out[a] = hs.reduce((q, c) => q + c, 0) / hs.length * this.axle_scale[a]; ok[a] = true; }
      }
      return [out, ok];
    }
    _noise(a, za, gap) {
      // оценка шума показаний оси: остаток экстраполяции по двум прошлым, медиана по окну
      const p = this.p, buf = this.nz_buf[a];
      if (gap > p.t_valid) this.nz_cnt[a] = 0;
      if (this.nz_cnt[a] >= 2) {
        const r = gap / this.gap_prev[a];
        const e = (za - this.axle_prev[a]) - (this.axle_prev[a] - this.axle_prev2[a]) * r;
        buf.push(Math.abs(e) / Math.sqrt(1.0 + (1.0 + r) ** 2 + r * r));
        if (buf.length > p.noise_n) buf.shift();
      }
      if (buf.length * 2 >= p.noise_n) {
        const s = MAD_TO_SIGMA * median(buf);
        this.nz_sigma[a] = s;
        this.nz_excess[a] = Math.sqrt(Math.max(0.0, s * s - p.sigma_meas ** 2));
      }
      this.axle_prev2[a] = this.axle_prev[a];
      this.gap_prev[a] = gap;
      this.nz_cnt[a] += 1;
    }
    _dot_tol() {
      // запас порога производной оси на шум сверх паспортного
      const p = this.p, al = p.axle_dot_alpha;
      return this.nz_excess.map((ne, a) => p.noise_k * Math.sqrt(2.0) * ne / this.axle_gap[a] * Math.sqrt(al / (2.0 - al)));
    }
    _calibrate(z, acc) {
      const p = this.p;
      if (this.mode !== COAST || acc.length < 2 || Math.min(...acc.map(a => z[a])) < p.calib_v_min) return;
      const med = median(acc.map(a => z[a]));
      for (const a of acc) this.axle_scale[a] += p.calib_gain * (med / z[a] - 1.0);
      this.axle_scale = this.axle_scale.map(s => clip(s, 1.0 - p.scale_range, 1.0 + p.scale_range));
    }
    _sigma() {
      const n = NX, A = zeros(n);
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) A[i][j] = (n + this.lam) * this.P[i][j];
      const S = chol(A), pts = [this.x.slice()];
      for (let i = 0; i < n; i++) pts.push(this.x.map((xv, r) => xv + S[r][i]));
      for (let i = 0; i < n; i++) pts.push(this.x.map((xv, r) => xv - S[r][i]));
      return pts;
    }
    _predict(u) {
      const p = this.p, pts = this._sigma(), prop = pts.map(pt => f_process(pt, u, this.mu, p));
      const x = new Array(NX).fill(0);
      prop.forEach((pr, j) => { for (let i = 0; i < NX; i++) x[i] += this.wm[j] * pr[i]; });
      const P = zeros(NX);
      prop.forEach((pr, j) => { for (let a = 0; a < NX; a++) for (let b = 0; b < NX; b++) P[a][b] += (pr[a] - x[a]) * this.wc[j] * (pr[b] - x[b]); });
      P[IV][IV] += (p.q_v * p.dt) ** 2;
      if (Math.abs(u) < p.T_dead) P[ID][ID] += p.q_d ** 2 * p.dt;
      if (u > p.T_dead) P[IKT][IKT] += p.q_k ** 2 * p.dt;
      if (u < -p.T_dead) P[IKB][IKB] += p.q_k ** 2 * p.dt;
      this.x = x; this.P = sym(P);
    }
    _a_limit(u, accelerating) {
      const p = this.p;
      if (accelerating) return p.a_max_acc;
      return u < -p.T_dead ? p.a_max_brake : p.a_free_decel;
    }
    _meas_var(F, v) {
      const p = this.p;
      return axle_shares(F, p).map((sh, i) => {
        let r = p.sigma_meas ** 2 + (v * p.sigma_creep) ** 2 * (sh > 0 ? 1 : 0);
        if (p.sensors_per_axle === 1) r = r + (v * p.curve_ratio_max) ** 2;
        return r + this.nz_excess[i] ** 2;          // шум показаний выше паспортного
      });
    }
    _a_model(u) {
      // ускорение корпуса по модели в текущем состоянии (как в выходе связки)
      const p = this.p, v = this.x[IV];
      const a = (body_force(u, v, this.x[IKT], this.x[IKB], this.mu, p) - resistance(v, p)) / p.M_nom + this.x[ID];
      return v <= 0.0 && a < 0.0 ? 0.0 : a;
    }
    _live() {
      // оси с датчиками, показания которых не старше slip_pair_s
      const p = this.p, out = [];
      for (let a = 0; a < p.n_axles; a++) if (this.slots[a].length && this.t - this.t_axle[a] <= p.slip_pair_s) out.push(a);
      return out;
    }
    _slip_all_update(z, ok, u, hbar, v0) {
      // СРЫВ ВСЕХ ОСЕЙ (юз или буксование обеих тележек): начало и конец - см. докстроку ядра
      const p = this.p, live = this._live();
      const maxLive = arr => Math.max(...live.map(a => arr[a])), minLive = arr => Math.min(...live.map(a => arr[a]));
      if (this.slip_all) {
        const s = this.slip_all;
        for (const a of live) if (this.jump_t[a] > this.slip_t0 && this.jump_s[a] === -s) this.slip_back.add(a);
        if (p.slip_ret_a > 0) this._slip_all_return(s, u, z, ok, hbar, live);
        const fresh = []; for (let a = 0; a < p.n_axles; a++) if (ok[a]) fresh.push(a);
        const zeros = v0 > p.v_dead_ref && fresh.length > 0 && fresh.every(a => z[a] < p.v_standstill);
        const noisy = live.length > 0 && (p.slip_jump + p.noise_k * Math.sqrt(2.0) * maxLive(this.nz_excess) >= this.slip_j);
        const back = live.length > 0 && live.every(a => this.slip_back.has(a));
        const agree = p.slip_ok_s > 0 && this.slip_ok_t0 !== null && this.t - this.slip_ok_t0 >= p.slip_ok_s;
        const timeout = this.t - this.slip_t0 > p.slip_t_max;
        if (timeout || zeros || noisy || back || agree) {
          this._slip_all_end(u, zeros);
          if (timeout) this.slip_timeout = [s, this.t];
        }
        return;
      }
      if (!p.slip_all_on || this.amb || v0 <= p.v_adapt_min) return;
      if (live.length < 2 || live.some(a => this.t - this.jump_t[a] > p.slip_pair_s)) return;
      const s = this.jump_s[live[0]];
      const zl = live.map(a => ok[a] ? z[a] : this.axle_prev[a]);
      const [s_to, t_to] = this.slip_timeout;
      if (s_to === -s && this.t - t_to <= p.slip_t_max) return;
      const nl = this.notch_last;
      if ((s > 0 && nl < 0) || (s < 0 && nl > 0)) return;
      if (live.some(a => this.jump_s[a] !== s) || minLive(this.jump_m) < p.slip_rel * v0
          || live.some((a, k) => s * (zl[k] - hbar[a]) <= 0) || zl.every(zb => zb < p.v_standstill)) return;
      this.slip_all = s > 0 ? 1 : s < 0 ? -1 : 0;   // int(s): s = ±1
      this.slip_t0 = this.t;
      this.slip_j = minLive(this.jump_m);
      this.slip_back = new Set(); this.slip_ok_t0 = null; this.slip_ret = new Set();
      this.n_slip_all += 1;
      // дальше скорость ведёт модель: σ возмущения - ошибка разомкнутого прогноза
      const sd = Math.sqrt(Math.max(this.P[ID][ID], 0.0));
      if (sd > 0.0) {
        const f = p.slip_sigma_a / sd;
        for (let j = 0; j < NX; j++) this.P[ID][j] *= f;
        for (let i = 0; i < NX; i++) this.P[i][ID] *= f;
      } else this.P[ID][ID] = p.slip_sigma_a ** 2;
      const am = this._a_model(u);
      for (const a of live) this.axle_dot[a] = am;
    }
    _slip_all_return(s, u, z, ok, hbar, live) {
      // плавное возвращение колеса к корпусу: производная колеса против ρ·(ускорение модели)
      const p = this.p, a_m = this._a_model(u), tol = this._dot_tol();
      for (const a of live) {
        if (!ok[a] || this.slip_back.has(a) || hbar[a] <= p.v_adapt_min) continue;
        const rho = z[a] / hbar[a];
        const r = -s * (this.axle_dot[a] - rho * a_m);
        if (r > p.slip_ret_a + tol[a]) this.slip_ret.add(a);
        else if (this.slip_ret.has(a) && r < 0.5 * p.a_slip_margin) this.slip_back.add(a);
      }
    }
    _slip_all_end(u, keep_jump = false) {
      // конец срыва всех осей; keep_jump - закончился нулями: скачок и производная шага остаются
      this.slip_all = 0; this.slip_back = new Set(); this.slip_ret = new Set(); this.slip_ok_t0 = null;
      if (!keep_jump) { this.axle_dot.fill(this._a_model(u)); this.axle_jump.fill(false); }
      this.jump_t.fill(-Infinity);
    }
    _level_agree(a, z, ok, recent) {
      // согласие оси с последними показаниями остальных (без проверки производных)
      const p = this.p, others = [];
      for (let b = 0; b < p.n_axles; b++) if (b !== a && (recent[b] || ok[b])) others.push(b);
      const tol = p.agree_tol + p.noise_k * Math.sqrt(2.0) * Math.max(...this.nz_excess);
      return others.length > 0 && others.every(b => Math.abs(z[a] - (ok[b] ? z[b] : this.axle_prev[b])) <= tol);
    }
    _correct(z, ok, u, mode, handle_ok = true) {
      const p = this.p, axles = [];
      for (let a = 0; a < p.n_axles; a++) if (ok[a]) axles.push(a);
      this.n_acc = this.n_rej = 0; this.sat = false;
      const acc = [], rej = [];
      if (!axles.length) return acc;
      const v0 = this.x[IV], F0 = body_force(u, v0, 1.0, 1.0, this.mu, p);
      const hbar = h_axles(this.x, u, this.mu, p);
      // срыв всех осей: начало и конец (может раздуть σ возмущения - до сигма-точек)
      const ep_prev = this.slip_all;
      this._slip_all_update(z, ok, u, hbar, v0);
      // снят нулями всех новых показаний (юз перешёл в блокировку): нули не принимаются
      const ended_zero = ep_prev !== 0 && this.slip_all === 0 && v0 > p.v_dead_ref && axles.every(a => z[a] < p.v_standstill);
      const var0 = this._meas_var(F0, v0);
      const key = a => Math.abs(z[a] - hbar[a]) / Math.sqrt(var0[a] + this.P[IV][IV]);
      const order = axles.slice().sort((a, b) => key(a) - key(b));
      const shares0 = axle_shares(F0, p);
      let recent_sat = (this.t - this.t_last_sat) <= p.t_sat_hold;
      const fast = v0 > p.v_adapt_min;
      const mask = [1, 1, 1, 1, 1];
      const moving = this.x[IV] > p.v_adapt_min;
      mask[ID] = (mode === COAST && moving && handle_ok) ? 1 : 0;
      mask[IKT] = 0; mask[IKB] = 0;
      const dot_tol = this._dot_tol();
      const spin_all = this.axle_dot.map((d, b) => Math.abs(d) > (d > 0 ? p.a_max_acc : p.a_max_brake) + p.a_slip_margin + dot_tol[b] || this.axle_jump[b]);
      const recent = this.t_axle.map(ta => (this.t - ta) <= p.agree_age);
      const agreed = a => {
        const others = [];
        for (let b = 0; b < p.n_axles; b++) if (b !== a && (recent[b] || ok[b])) others.push(b);
        if (!others.length || [a].concat(others).some(b => spin_all[b])) return false;
        return others.every(b => Math.abs(z[a] - (ok[b] ? z[b] : this.axle_prev[b])) <= p.agree_tol);
      };
      const ep = this.slip_all, ep_rej = new Set();   // отвергнуты срывом всех осей
      const dgw = (a, nu, nis, why) => { if (this.dg) { this.dg.t[a] = this.t; this.dg.nu[a] = nu; this.dg.nis[a] = nis; this.dg.why[a] = why; } };
      let pts = this._sigma(), Zall = h_axles_batch(pts, u, this.mu, p);
      for (const a of order) {
        const dot = this.axle_dot[a], lim = dot > 0 ? p.a_max_acc : p.a_max_brake;
        let spinning = Math.abs(dot) > lim + p.a_slip_margin + dot_tol[a] || this.axle_jump[a];
        let Z = Zall.map(r => r[a]);
        let zh = 0; for (let j = 0; j < Z.length; j++) zh += this.wm[j] * Z[j];
        let dz = Z.map(q => q - zh);
        const Fv = body_force(u, this.x[IV], 1.0, 1.0, this.mu, p);
        const R = this._meas_var(Fv, this.x[IV])[a];
        let S = 0; for (let j = 0; j < dz.length; j++) S += this.wc[j] * (dz[j] * dz[j]); S += R;
        let nu = z[a] - zh, nis = nu * nu / S;
        let forced, held, override;
        if (ep) {
          // срыв всех осей: показание в сторону срыва - лишь граница скорости корпуса
          const returned = this.slip_back.has(a);
          if (!returned && ep * nu > p.slip_cont_k * Math.sqrt(R)) { rej.push([a, nu, nis, false]); ep_rej.add(a); dgw(a, nu, nis, 'slipall'); continue; }
          spinning = forced = held = false;
          override = nis > p.gate_nis && (returned || this._level_agree(a, z, ok, recent));
          if (nis > p.gate_nis && !override) { rej.push([a, nu, nis, false]); ep_rej.add(a); dgw(a, nu, nis, 'gate'); continue; }
        } else {
          const lock = F0 < 0 && nu < 0 && z[a] < p.lock_ratio * hbar[a];
          const spin_dir = F0 > 0 && nu > 0 && this.t_since_acc <= p.t_valid;
          const in_sat_dir = (spin_dir || lock) && shares0[a] > 0 && fast;
          forced = recent_sat && in_sat_dir && nis > 0.25 * p.gate_nis;
          held = (this.amb || ended_zero) && z[a] < p.v_standstill;
          override = nis > p.gate_nis && !(spinning || forced || held) && agreed(a);
        }
        let over = false;
        if (override) {
          this.P[IV][IV] = Math.max(this.P[IV][IV], nu * nu);
          this.P[ID][ID] = Math.max(this.P[ID][ID], (p.sigma_rej_frac * Math.max(this._a_limit(u, true), this._a_limit(u, false))) ** 2);
          pts = this._sigma(); Zall = h_axles_batch(pts, u, this.mu, p);
          Z = Zall.map(r => r[a]);
          zh = 0; for (let j = 0; j < Z.length; j++) zh += this.wm[j] * Z[j];
          dz = Z.map(q => q - zh);
          S = 0; for (let j = 0; j < dz.length; j++) S += this.wc[j] * (dz[j] * dz[j]); S += R;
          nu = z[a] - zh; nis = 0.0;
          this.n_override += 1;
          over = true;
        }
        if (this.dg) {       // диагностика песочницы: только запись, на числа не влияет
          const dot_up = this.axle_dot[a] > 0;
          this.dg.t[a] = this.t; this.dg.nu[a] = nu; this.dg.nis[a] = nis;
          this.dg.why[a] = spinning ? (this.axle_jump[a] ? 'jump' : dot_up ? 'spin' : 'skid') : forced ? 'forced' : held ? 'held'
            : nis > p.gate_nis ? 'gate' : over ? 'agree' : 'ok';
        }
        if (spinning || forced || held || nis > p.gate_nis) { rej.push([a, nu, nis, spinning]); continue; }
        const C = new Array(NX).fill(0);
        for (let j = 0; j < pts.length; j++) for (let i = 0; i < NX; i++) C[i] += (pts[j][i] - this.x[i]) * this.wc[j] * dz[j];
        const K = C.map((c, i) => c / S * mask[i]);
        this.x = this.x.map((xv, i) => xv + K[i] * nu);
        const P = zeros(NX);
        for (let i = 0; i < NX; i++) for (let j = 0; j < NX; j++) P[i][j] = this.P[i][j] - K[i] * C[j] - C[i] * K[j] + K[i] * K[j] * S;
        this.P = sym(P);
        acc.push(a);
        pts = this._sigma(); Zall = h_axles_batch(pts, u, this.mu, p);
      }
      this.n_acc = acc.length; this.n_rej = rej.length;
      // срыв всех осей идёт: все новые показания приняты - отсчёт для его снятия
      if (this.slip_all && acc.length && !rej.length) { if (this.slip_ok_t0 === null) this.slip_ok_t0 = this.t; }
      else this.slip_ok_t0 = null;
      // признак насыщения: нагруженная ось отвергнута в сторону срыва, значимо, и срыв подтверждён
      // не одной моделью (блокировка, скачок/производная за пределом или другая ось читает меньше)
      if (rej.length && fast) {
        const sh = axle_shares(F0, p);
        const tol = p.agree_tol + p.noise_k * Math.sqrt(2.0) * Math.max(...this.nz_excess);
        for (const [a, nu, nis, spun] of rej) {
          if (ep_rej.has(a)) continue;
          const lock = F0 < 0 && nu < 0 && z[a] < p.lock_ratio * hbar[a];
          if (!(sh[a] > 0 && ((F0 > 0 && nu > 0) || lock) && nis > 0.25 * p.gate_nis)) continue;
          const sgn = nu > 0 ? 1.0 : -1.0;
          let apart = false;
          for (let b = 0; b < p.n_axles; b++) if (b !== a && (recent[b] || ok[b]) && sgn * (z[a] - (ok[b] ? z[b] : this.axle_prev[b])) > tol) { apart = true; break; }
          if (lock || spun || apart) this.sat = true;
        }
      }
      if (this.sat) this.t_last_sat = this.t;
      recent_sat = (this.t - this.t_last_sat) <= p.t_sat_hold;
      const idle_rejected = rej.some(([a]) => shares0[a] === 0);
      const explained = ep_rej.size === rej.length;
      if (!acc.length && rej.length && !explained && (!recent_sat || idle_rejected)) {
        const up = median(rej.map(r => r[1])) > 0;
        this.P[ID][ID] = Math.max(this.P[ID][ID], (p.sigma_rej_frac * this._a_limit(u, up)) ** 2);
      }
      const entry_ok = this.amb || this.t_since_acc <= p.t_valid || ep !== 0 || ep_prev !== 0;
      const all_zero = axles.every(a => z[a] < p.v_standstill) && v0 > p.v_dead_ref;
      if (!acc.length && rej.length && entry_ok && ((this.sat && F0 < 0) || all_zero)) {
        if (!this.amb) this.amb_v = v0;
        this.amb = true;
        this.amb_v = Math.max(this.amb_v, v0);
      }
      return acc;
    }
    step(notch, meas, fresh = true, handle_ok = true) {
      const p = this.p;
      this.t += p.dt;
      this.notch_last = notch;
      const u = this._drive_model(notch);
      const fm = Array.isArray(fresh) ? fresh.map(Boolean) : new Array(this.nw).fill(Boolean(fresh));
      const anyFresh = fm.some(Boolean);
      let z = new Array(p.n_axles).fill(0), ok = new Array(p.n_axles).fill(false);
      if (anyFresh) {
        meas = Array.from(meas, Number);
        if (meas.length !== this.nw) throw new Error(`ожидается ${this.nw} показаний датчиков, получено ${meas.length}`);
        const dts = this.t_meas_i.map(tm => isFinite(tm) ? Math.min(this.t - tm, p.t_valid) : p.dt);
        for (let i = 0; i < this.nw; i++) if (fm[i]) this.t_meas_i[i] = this.t;
        this.have_meas = true;
        const ms = sensor_to_speed(meas, p);
        this._diagnose(ms, dts, fm);
        this._frozen_all(ms, u);
        [z, ok] = this._axle_speeds(ms, fm);
        this.axle_jump.fill(false);
        for (let a = 0; a < p.n_axles; a++) {
          if (!ok[a]) continue;
          if (!this.axle_seen[a]) { this.axle_prev[a] = z[a]; this.axle_seen[a] = true; this.t_axle[a] = this.t - p.dt; this.nz_cnt[a] = 0; }
          const gap = this.t - this.t_axle[a];
          const raw = (z[a] - this.axle_prev[a]) / gap;
          this._noise(a, z[a], gap);
          const nz = p.noise_k * Math.sqrt(2.0) * this.nz_excess[a];   // шум сверх паспортного расширяет пороги
          // скачок против СОБСТВЕННОГО тренда оси - срыв этой оси (все оси одного знака - срыв всех осей)
          if (this.nz_cnt[a] > 2 && gap <= p.slip_pair_s && (this.t_since_acc <= p.slip_pair_s || this.slip_all)
              && !(this.amb || this.frozen)) {
            const dzj = z[a] - this.axle_prev[a];
            const jr = dzj - this.axle_dot[a] * gap;
            const thr = p.slip_jump + p.a_slip_margin * gap + nz;
            const sg = jr > 0 ? 1.0 : -1.0;
            // отголосок своего скачка: ровное показание после скачка - не скачок обратно
            const echo = !this.slip_all && sg === -this.jump_s[a] && this.t - this.jump_t[a] <= p.slip_pair_s && Math.abs(dzj) <= thr;
            if (Math.abs(jr) > thr && !echo) { this.jump_t[a] = this.t; this.jump_s[a] = sg; this.jump_m[a] = Math.abs(jr); }
          }
          // скачок за один интервал больше физически возможного - срыв или отказ сразу
          const dz = z[a] - this.axle_prev[a], lim_j = dz > 0 ? p.a_max_acc : p.a_max_brake;
          this.axle_jump[a] = Math.abs(dz) > (lim_j + p.a_slip_margin) * gap + p.agree_tol + nz;
          this.axle_dot[a] += p.axle_dot_alpha * (raw - this.axle_dot[a]);
          this.axle_gap[a] = gap;
          this.axle_prev[a] = z[a];
          this.t_axle[a] = this.t;
        }
        if (!this.initialised && ok.some(Boolean)) {
          const zs = z.filter((_, a) => ok[a]);
          const v0 = notch > 0 ? Math.min(...zs) : notch < 0 ? Math.max(...zs) : median(zs);
          this.x[IV] = v0;
          this.P[IV][IV] = p.sigma_meas ** 2 + (v0 * p.sigma_creep) ** 2;
          this.initialised = true;
        }
      }
      if (this.mode === STANDSTILL && Math.abs(u) > p.T_dead) this.P[IV][IV] = Math.max(this.P[IV][IV], p.sigma_meas ** 2);
      this._predict(u);
      const mode = this._mode(u);
      this.mode_pre = mode;
      if (this.frozen) {
        // залипли все датчики разом: только прогноз по ручке, неопределённость по ускорению - до предела
        const lim = Math.max(this._a_limit(u, true), this._a_limit(u, false));
        this.P[ID][ID] = Math.max(this.P[ID][ID], (p.sigma_rej_frac * lim) ** 2);
        this.adapting = false; this.n_acc = this.n_rej = 0; this.sat = false; this.slip_all = 0; this.last_acc_n = 0;
        return this._finish(u, z, new Array(p.n_axles).fill(false), [], false);
      }
      let acc = [];
      if (anyFresh) {
        acc = this._correct(z, ok, u, mode, handle_ok);
        this.last_acc_n = acc.length;
        if (handle_ok) this._adapt_scale(u, mode, acc); else this.adapting = false;
      }
      return this._finish(u, z, ok, acc, anyFresh);
    }
    _frozen_all(meas, u) {
      // залипание ВСЕХ датчиков разом: у каждого больше stuck_n одинаковых показаний, на ходу, при команде
      const p = this.p;
      if (!this.stuck_cnt.every(c => c > p.stuck_n)) this.frozen = false;
      else if (!this.frozen && Math.abs(u) > p.T_dead && Math.min(...meas.map(Math.abs)) > p.v_dead_ref) this.frozen = true;
    }
    _adapt_scale(u, mode, acc) {
      const p = this.p;
      const active = p.adapt_on && (mode === TRACTION || mode === BRAKE) && acc.length >= 2 && this.x[IV] > p.v_adapt_min && !this.sat;
      this.adapting = active;
      if (!active || mode !== this.win_mode) {
        this.win_mode = active ? mode : null;
        this.win_t = 0; this.win_t_last = this.t; this.win_v0 = this.x[IV]; this.win_F = 0; this.win_W = 0;
        return;
      }
      const v = this.x[IV];
      const h = this.t - this.win_t_last;           // окно копит реальное прошедшее время
      this.win_t_last = this.t;
      this.win_F += drive_force(u, v, 1.0, 1.0, p) * h;
      this.win_W += resistance(v, p) * h;
      this.win_t += h;
      if (this.win_t < p.t_adapt) return;
      const rated = rated_force(mode === TRACTION, p);
      if (Math.abs(this.win_F) >= p.adapt_f_min * rated * this.win_t) {
        const dv = v - this.win_v0;
        const k_meas = (p.M_nom * (dv - this.x[ID] * this.win_t) + this.win_W) / this.win_F;
        const idx = mode === TRACTION ? IKT : IKB;
        const S = this.P[idx][idx] + p.sigma_k_meas ** 2;
        if ((k_meas - this.x[idx]) ** 2 <= p.gate_nis * S) {
          const K = this.P.map(r => r[idx] / S), row = this.P[idx].slice(), inn = k_meas - this.x[idx];
          this.x = this.x.map((xv, i) => xv + K[i] * inn);
          const P = zeros(NX);
          for (let i = 0; i < NX; i++) for (let j = 0; j < NX; j++) P[i][j] = this.P[i][j] - K[i] * row[j];
          this.P = sym(P);
        }
      }
      this.win_t = 0; this.win_v0 = v; this.win_F = 0; this.win_W = 0;
    }
    step_open_loop(notch) {
      const p = this.p;
      this.t += p.dt;
      this.notch_last = notch;
      const u = this._drive_model(notch);
      this._predict(u);
      this.mode_pre = this._mode(u);
      this.adapting = false; this.n_acc = this.n_rej = 0; this.sat = false;
      const lim = Math.max(this._a_limit(u, true), this._a_limit(u, false));
      this.P[ID][ID] = Math.max(this.P[ID][ID], (p.sigma_rej_frac * lim) ** 2);
      this.have_meas = false;
      this.axle_seen.fill(false);
      this.slip_all = 0;
      this.last_acc_n = 0;
      return this._finish(u, new Array(p.n_axles).fill(0), new Array(p.n_axles).fill(false), [], false);
    }
    _sigma_v_out(sv, u) {
      // публикуемая σ скорости: σ фильтра с множителем, пол, возраст показаний·|a|, масштаб колёс·v
      const p = this.p, v = this.x[IV];
      const a = p.sv_age ? this._a_model(u) : 0.0;   // ускорение модели - как в выходе связки
      const floor = this.mode === STANDSTILL ? p.sv_floor_stand : p.sv_floor;
      return Math.sqrt((p.sv_gain * sv) ** 2 + floor * floor + (p.sv_age * a) ** 2 + (p.sv_rel * v) ** 2);
    }
    _finish(u, z, ok, acc, fresh) {
      const p = this.p;
      if (this.sat) this.mu += (p.mu_min - this.mu) * p.dt / p.tau_sat;
      else if (this.last_acc_n) this.mu += (p.mu_nominal - this.mu) * p.dt / p.tau_rec;
      this.t_since_acc = acc.length ? 0.0 : this.t_since_acc + p.dt;
      if (fresh && this.amb && acc.length && !this.sat && Math.max(...acc.map(a => z[a])) > p.v_standstill) { this.amb = false; this.amb_v = 0.0; }
      const d_max = G * Math.sin(p.theta_max) + p.d_extra;
      this.x[IV] = clip(this.x[IV], 0.0, p.v_max_line);
      this.x[ID] = clip(this.x[ID], -d_max, d_max);
      this.x[IKT] = clip(this.x[IKT], p.k_min, p.k_max);
      this.x[IKB] = clip(this.x[IKB], p.k_min, p.k_max);
      if (fresh) {
        const zero = acc.length > 0 && Math.max(...acc.map(a => z[a])) < p.v_standstill;
        if (!zero) this.t_zero_start = null;
        else if (this.t_zero_start === null) this.t_zero_start = this.t - p.dt;
      } else if (!this.have_meas) this.t_zero_start = null;
      this.zero_t = this.t_zero_start === null ? 0.0 : this.t - this.t_zero_start;
      const standstill = this.zero_t >= p.t_standstill && this.x[IV] < 2.0 * p.v_standstill && !this.amb;
      if (standstill) { this.x[IV] = 0.0; this.x[ID] = 0.0; this.P[IV][IV] = Math.min(this.P[IV][IV], 1e-4); }
      if (fresh) this._calibrate(z, acc);
      this.mode = this.mode_pre;
      if (this.sat || this.amb || this.slip_all) this.mode = SLIP;
      if (standstill) this.mode = STANDSTILL;
      const valid = this.have_meas && this.t_since_acc <= p.t_valid && !this.amb && !this.frozen;
      // при срыве всех осей скорость ведёт модель: режим - срыв, а не отказ
      if (!this.have_meas || (!valid && !this.amb && !this.slip_all)) this.mode = DEGRADED;
      const sv_filt = Math.sqrt(Math.max(this.P[IV][IV], 0.0));
      let sv = this._sigma_v_out(sv_filt, u);
      if (this.amb) { sv = Math.max(sv, 0.5 * this.amb_v); this.s_extra += 0.5 * this.amb_v * p.dt; }
      const ss = Math.sqrt(Math.max(this.P[IS][IS], 0.0) + this.s_extra ** 2);
      return {
        v: this.x[IV], s: this.x[IS], d: this.x[ID], k_t: this.x[IKT], k_b: this.x[IKB], mu: this.mu,
        sigma_v: sv, sigma_s: ss, sigma_v_filt: sv_filt, mode: this.mode, healthy: this.healthy.slice(), slip: Boolean(this.sat || this.slip_all), slip_all: this.slip_all,
        meas_noise: Math.max(...this.nz_sigma), ambiguous: this.amb, frozen: this.frozen,
        n_accepted: this.n_acc, n_rejected: this.n_rej, odometry_used: acc.length > 0, valid, u
      };
    }
  }
  return { Estimator, DEFAULT, MODE_NAMES, COAST, TRACTION, BRAKE, TRANSITION, SLIP, STANDSTILL, DEGRADED, G, IS, IV, ID, IKT, IKB,
    notch_to_u, sensor_to_speed, sensor_layout, body_force, resistance, drive_force, table_acc, position_sigma, PORT };
})();
if (typeof module !== 'undefined') module.exports = TramEst;
