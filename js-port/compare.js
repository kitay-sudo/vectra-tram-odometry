const T = require('./est.js');
const rec = require('./rec.json');
let worst = 0;
for (const [name, rows] of Object.entries(rec)) {
  const es = new T.Estimator();
  let mv = 0, ms = 0, mk = 0, modeMis = 0, t0 = process.hrtime.bigint();
  for (const r of rows) {
    const [notch, meas, v, s, mode, sv, kt, mu] = r;
    const o = es.step(notch, meas);
    mv = Math.max(mv, Math.abs(o.v - v)); ms = Math.max(ms, Math.abs(o.s - s)); mk = Math.max(mk, Math.abs(o.k_t - kt));
    if (o.mode !== mode) modeMis++;
  }
  const us = Number(process.hrtime.bigint() - t0) / 1000 / rows.length;
  worst = Math.max(worst, mv);
  console.log(name.padEnd(28), 'max|dv|', mv.toExponential(2), 'max|ds|', ms.toExponential(2), 'max|dk|', mk.toExponential(2), 'mode mismatches', modeMis, `${us.toFixed(1)} мкс/шаг`);
}
console.log('worst dv', worst);
