// Аудит JS-порта: атрибуция расхождения с текущим ядром по отдельным фичам.
// Порождает варианты js-port/est.js (сам файл не меняется) в out/sim/port_variants/:
//   v1_bhf    - brake_hold_frac = 1 (текущее значение по умолчанию): тормозная
//               сила не гаснет ниже v_ed_fade
//   v2_adapt  - v1 + окно адаптации k по drive_force (без предела сцепления)
//               и гейт по невязке k_meas (как в estimator_core._adapt_scale)
// Сверка: node tools/dev/compare_port.js out/sim/rec_current.json out/sim/port_variants/est_v2_adapt.js
const fs = require('fs');
const path = require('path');
const ROOT = path.resolve(__dirname, '..', '..');
const OUT = path.join(ROOT, 'out', 'sim', 'port_variants');
const src = fs.readFileSync(path.join(ROOT, 'js-port', 'est.js'), 'utf8');
function rep(s, a, b) { if (!s.includes(a)) throw new Error('не найдено: ' + a.slice(0, 60)); return s.split(a).join(b); }
let s1 = rep(src, 'const shape_brake = (v, p) => Math.min(1.0, Math.abs(v) / p.v_ed_fade);', 'const shape_brake = (v, p) => 1.0;');
s1 = rep(s1, 'p.F_brake * u * Math.min(1.0, Math.abs(v) / p.v_ed_fade)', 'p.F_brake * u');
let s2 = rep(s1, 'this.win_F += body_force(u, v, 1.0, 1.0, this.mu, p) * p.dt;', 'this.win_F += drive_force(u, v, 1.0, 1.0, p) * p.dt;');
s2 = rep(s2, 'const S = this.P[idx][idx] + p.sigma_k_meas ** 2;\n        const K', 'const S = this.P[idx][idx] + p.sigma_k_meas ** 2;\n        if ((k_meas - this.x[idx]) ** 2 <= p.gate_nis * S) {\n        const K');
s2 = rep(s2, 'this.P = sym(P);\n      }\n      this.win_t = 0;', 'this.P = sym(P);\n      }}\n      this.win_t = 0;');
fs.mkdirSync(OUT, { recursive: true });
fs.writeFileSync(path.join(OUT, 'est_v1_bhf.js'), s1);
fs.writeFileSync(path.join(OUT, 'est_v2_adapt.js'), s2);
console.log('->', path.relative(ROOT, OUT));
