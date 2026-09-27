// Сверка JS-порта с Python-ядром на 15 сценариях prototype/scenarios.py:
//   python3 record.py   (в Docker vectra/tram:dev: нужен numpy) -> rec.json
//   node compare.js     -> максимальные расхождения по сценариям; код выхода 1, если > 1e-6
//                          или ядро не то, с которым сверен порт (TramEst.PORT.core_sha1)
const T = require('./est.js');
const rec = require('./rec.json');
let worst = { v: 0, s: 0, sv: 0, mu: 0, k: 0, d: 0, mode: 0 }, steps = 0;
for (const [name, rows] of Object.entries(rec)) {
  if (name.startsWith('_')) continue;
  const es = new T.Estimator();
  let mv = 0, ms = 0, mk = 0, msv = 0, mmu = 0, md = 0, modeMis = 0;
  const t0 = process.hrtime.bigint();
  for (const r of rows) {
    const [notch, meas, v, s, mode, sv, kt, mu, d] = r;
    const o = es.step(notch, meas);
    mv = Math.max(mv, Math.abs(o.v - v)); ms = Math.max(ms, Math.abs(o.s - s)); mk = Math.max(mk, Math.abs(o.k_t - kt));
    msv = Math.max(msv, Math.abs(o.sigma_v - sv)); mmu = Math.max(mmu, Math.abs(o.mu - mu)); md = Math.max(md, Math.abs(o.d - d));
    if (o.mode !== mode) modeMis++;
  }
  steps += rows.length;
  const us = Number(process.hrtime.bigint() - t0) / 1000 / rows.length;
  worst = { v: Math.max(worst.v, mv), s: Math.max(worst.s, ms), sv: Math.max(worst.sv, msv), mu: Math.max(worst.mu, mmu), k: Math.max(worst.k, mk), d: Math.max(worst.d, md), mode: worst.mode + modeMis };
  console.log(name.padEnd(28), 'max|dv|', mv.toExponential(2), 'max|ds|', ms.toExponential(2), 'max|dσv|', msv.toExponential(2), 'max|dk|', mk.toExponential(2), 'режим ≠', modeMis, `${us.toFixed(1)} мкс/шаг`);
}
console.log(`шагов ${steps}; худшее: dv ${worst.v.toExponential(2)} м/с, ds ${worst.s.toExponential(2)} м, dσv ${worst.sv.toExponential(2)}, dμ ${worst.mu.toExponential(2)}, dk ${worst.k.toExponential(2)}, dd ${worst.d.toExponential(2)}; режим ≠ ${worst.mode}`);
const numOk = Math.max(worst.v, worst.s, worst.sv, worst.mu, worst.k, worst.d) <= 1e-6 && worst.mode === 0;
const shaOk = !rec._core_sha1 || rec._core_sha1 === T.PORT.core_sha1;
if (!shaOk) console.log(`ядро ${rec._core_sha1} ≠ ядро сверки порта ${T.PORT.core_sha1} (PORT в est.js): ${numOk ? 'числа совпали - обнови PORT в est.js и во вшитой копии simulator/index.html' : 'перенеси правку ядра в est.js (и во вшитую копию) либо оставь ярлык «упрощённое ядро»'}`);
process.exit(numOk && shaOk ? 0 : 1);
