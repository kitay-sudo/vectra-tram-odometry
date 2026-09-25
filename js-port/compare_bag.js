// Сверка JS-порта с Python-ядром на листе вагона по реальной записи (rec_bag.json из record_bag.py).
//   node compare_bag.js   -> код выхода 1, если расхождение > 1e-6 или режимы не совпали
const T = require('./est.js');
const rec = require('./rec_bag.json');
const es = new T.Estimator(rec.params);
let mv = 0, ms = 0, msv = 0, mss = 0, mk = 0, mmu = 0, md = 0, modeMis = 0, ambMis = 0;
const t0 = process.hrtime.bigint();
for (const r of rec.rows) {
  const [kind, notch, meas, fm, handle_ok, v, s, mode, sv, kt, mu, d, ss, amb] = r;
  const o = kind === 1 ? es.step_open_loop(notch) : es.step(notch, meas, fm, handle_ok);
  mv = Math.max(mv, Math.abs(o.v - v)); ms = Math.max(ms, Math.abs(o.s - s)); msv = Math.max(msv, Math.abs(o.sigma_v - sv));
  mss = Math.max(mss, Math.abs(o.sigma_s - ss)); mk = Math.max(mk, Math.abs(o.k_t - kt)); mmu = Math.max(mmu, Math.abs(o.mu - mu)); md = Math.max(md, Math.abs(o.d - d));
  if (o.mode !== mode) modeMis++;
  if (o.ambiguous !== amb) ambMis++;
}
const us = Number(process.hrtime.bigint() - t0) / 1000 / rec.rows.length;
const worst = Math.max(mv, ms, msv, mss, mk, mmu, md);
console.log(`${rec.bag} ${rec.variant}: шагов ${rec.rows.length}; max|dv| ${mv.toExponential(2)} м/с, |ds| ${ms.toExponential(2)} м, |dσv| ${msv.toExponential(2)}, |dσs| ${mss.toExponential(2)}, |dk| ${mk.toExponential(2)}, |dμ| ${mmu.toExponential(2)}, |dd| ${md.toExponential(2)}; режим ≠ ${modeMis}, неоднозначность ≠ ${ambMis}; ${us.toFixed(1)} мкс/шаг`);
process.exit(worst <= 1e-6 && modeMis === 0 && ambMis === 0 ? 0 : 1);
