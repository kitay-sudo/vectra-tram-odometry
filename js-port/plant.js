// ---- Tram plant: JavaScript port of plant.py (the "truth" the estimator never sees directly) ----
const TramPlant = (() => {
  const G = 9.81;
  const P = {
    M: 30000.0, r: 0.35, b: 0.762, n_axles: 4, motored: [true, true, false, false],
    res_A: 2500.0, res_B: 60.0, res_C: 4.0,
    J_motor: 30.0, J_wheel: 30.0, J_trailer: 12.0, k_tors: 4.0e4, c_tors: 1.6e2, T_max: 6300.0, v_base: 8.0,
    tau_drive: 0.25, delay_drive: 0.08, jerk_lim: 3.0,
    T_mech_max: 4000.0, v_ed_fade: 1.5, F_track_brake: 30000.0,
    dv_peak_rel: 0.01, dv_peak_min: 0.05, B_wheel: 3.0,
    ppr: 200, t_meas: 0.10, sensor_noise: 0.01, meas_delay: 0.02, dt_est: 0.01
  };
  function rng(seed) {
    let a = seed >>> 0, spare = null;
    const u = () => { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    const normal = (m, sd) => {
      if (spare !== null) { const r = spare; spare = null; return m + sd * r; }
      let x, y, q; do { x = 2 * u() - 1; y = 2 * u() - 1; q = x * x + y * y; } while (q >= 1 || q === 0);
      const f = Math.sqrt(-2 * Math.log(q) / q); spare = y * f; return m + sd * x * f;
    };
    return { uniform: (lo, hi) => lo + (hi - lo) * u(), normal };
  }
  function adhesion_mu(dv, v, mu_max) {
    const dv0 = Math.max(P.dv_peak_rel * Math.abs(v), P.dv_peak_min), x = dv / dv0;
    return 2.0 * mu_max * x / (1.0 + x * x);
  }
  class Plant {
    // track: { grade(s), radius(s), mu(s, t) }
    constructor(track, dt = 0.001, seed = 0) {
      this.tr = track; this.dt = dt; this.rng = rng(seed);
      this.nw = 2 * P.n_axles;
      this.v = 0; this.s = 0; this.t = 0; this._accel = 0; this.u_filt = 0; this.notch_hist = [];
      this.w = new Array(this.nw).fill(0); this.wm = new Array(this.nw).fill(0); this.phi = new Array(this.nw).fill(0);
      this.scale = Array.from({ length: this.nw }, () => 1.0 + this.rng.uniform(-0.015, 0.015));
      this.angle = new Array(this.nw).fill(0);
      this.angle_hist = []; this.meas_hist = [];
      this.faults = {}; this._stuck = {};
      this.M = P.M;           // total mass, may change with passengers
      // Addition for the demo (not in plant.py): wheel slip / slide protection, as fitted on real trams.
      // Without it the plant lets a motored wheel spin up to hundreds of km/h on ice and even turn backwards under ED braking.
      this.protect = true;
      this.asr = new Array(this.nw).fill(1);
      this.wheel_scale = 1;   // global odometry scale (worn tyres)
    }
    _axle_of(i) { return Math.floor(i / 2); }
    _side(i) { return i % 2 === 0 ? 1 : -1; }
    _rail_speed(i) {
      const R = this.tr.radius(this.s);
      if (!isFinite(R)) return this.v;
      return this.v * (R + this._side(i) * P.b) / R;
    }
    _normal_load(i) {
      const base = this.M * G / this.nw, transfer = -this._accel * 0.06 * base / G;
      const sign = this._axle_of(i) < P.n_axles / 2 ? 1 : -1;
      return Math.max(base + sign * transfer, 0.2 * base);
    }
    _drive(notch) {
      const nd = Math.floor(P.delay_drive / this.dt);
      this.notch_hist.push(notch);
      if (this.notch_hist.length > nd + 1) this.notch_hist.shift();
      const u_del = this.notch_hist.length > nd ? this.notch_hist[this.notch_hist.length - nd - 1] : 0;
      const target = Math.min(1, Math.max(-1, u_del / 4.0));
      const lag = (target - this.u_filt) * this.dt / P.tau_drive;
      this.u_filt += Math.min(P.jerk_lim * this.dt, Math.max(-P.jerk_lim * this.dt, lag));
      return this.u_filt;
    }
    step(notch, sand = false, track_brake = false) {
      const dt = this.dt, u = this._drive(notch);
      const fade = Math.abs(this.v) < P.v_base ? 1.0 : P.v_base / Math.abs(this.v);
      let T_cmd_axle = u * P.T_max * fade;
      if (u < 0 && Math.abs(this.v) < P.v_ed_fade) T_cmd_axle *= Math.abs(this.v) / P.v_ed_fade;
      let F_tot = 0;
      for (let i = 0; i < this.nw; i++) {
        const ax = this._axle_of(i), mot = P.motored[ax], v_rail = this._rail_speed(i), dv = this.w[i] * P.r - v_rail;
        if (this.protect) {
          const lim = Math.max(0.4, 0.08 * Math.abs(this.v));
          const slipping = (u > 0 && dv > lim) || (u < 0 && -dv > lim);
          this.asr[i] = slipping ? Math.max(0.05, this.asr[i] - dt * 6) : Math.min(1, this.asr[i] + dt * 0.7);
        }
        const g = this.protect ? this.asr[i] : 1;
        let mu_max = this.tr.mu(this.s, this.t);
        if (sand) mu_max = Math.min(mu_max * 2.5, 0.30);
        const F = adhesion_mu(dv, this.v, mu_max) * this._normal_load(i);
        F_tot += F;
        let T_sh = 0;
        if (mot) {
          T_sh = P.k_tors * this.phi[i] + P.c_tors * (this.wm[i] - this.w[i]);
          this.wm[i] += (T_cmd_axle / 2.0 * g - T_sh) / P.J_motor * dt;
          this.phi[i] += (this.wm[i] - this.w[i]) * dt;
        }
        let T = T_sh - P.B_wheel * this.w[i];
        if (u < 0) {
          const blend = Math.abs(this.v) < P.v_ed_fade ? 1.0 : 0.3;
          T -= Math.abs(this.w[i]) > 1e-2 ? Math.sign(this.w[i]) * P.T_mech_max * Math.abs(u) * blend * g : 0.0;
        }
        const J = mot ? P.J_wheel : P.J_trailer;
        let w_new = this.w[i] + (T - P.r * F) / J * dt;
        if (u < 0 && this.w[i] > 0 && w_new < 0) w_new = 0.0;
        if (this.protect && w_new < 0 && this.v >= 0) w_new = 0.0;   // a wheel does not turn backwards while the tram rolls forward
        this.w[i] = w_new;
      }
      const F_res = P.res_A + P.res_B * Math.abs(this.v) + P.res_C * this.v * this.v;
      const F_grade = this.M * G * Math.sin(this.tr.grade(this.s));
      const F_tb = track_brake ? P.F_track_brake : 0.0;
      const F_drag = Math.abs(this.v) > 1e-3 ? (F_res + F_tb) * Math.sign(this.v) : 0.0;
      let F_net = F_tot - F_grade - F_drag;
      if (Math.abs(this.v) < 1e-3 && Math.abs(F_tot - F_grade) < F_res) F_net = 0.0;
      this._accel = F_net / this.M;
      this.v += this._accel * dt;
      this.s += this.v * dt;
      this.t += dt;
      for (let i = 0; i < this.nw; i++) this.angle[i] += this.w[i] * this.scale[i] * this.wheel_scale * dt;
      return this._accel;
    }
    measure() {
      const nwin = Math.floor(P.t_meas / this.dt), quant = 2 * Math.PI / P.ppr;
      this.angle_hist.push(this.angle.slice());
      if (this.angle_hist.length > nwin + 1) this.angle_hist.shift();
      let meas;
      if (this.angle_hist.length > nwin) {
        const old = this.angle_hist[this.angle_hist.length - nwin - 1];
        meas = this.angle.map((a, i) => (Math.floor(a / quant) * quant - Math.floor(old[i] / quant) * quant) / P.t_meas);
      } else meas = new Array(this.nw).fill(0);
      meas = meas.map(m => m + this.rng.normal(0, P.sensor_noise));
      for (const [k, [t0, kind]] of Object.entries(this.faults)) {
        const i = +k;
        if (this.t >= t0) {
          if (kind === 'zero') meas[i] = 0.0;
          else if (kind === 'stuck') { if (!(i in this._stuck)) this._stuck[i] = meas[i]; meas[i] = this._stuck[i]; }
          else if (kind === 'noise') meas[i] += this.rng.normal(0, 3.0);
        }
      }
      const nd = Math.floor(P.meas_delay / this.dt);
      this.meas_hist.push(meas.slice());
      if (this.meas_hist.length > nd + 1) this.meas_hist.shift();
      return this.meas_hist.length > nd ? this.meas_hist[this.meas_hist.length - nd - 1] : new Array(this.nw).fill(0);
    }
  }
  return { Plant, P, adhesion_mu };
})();
if (typeof module !== 'undefined') module.exports = TramPlant;
