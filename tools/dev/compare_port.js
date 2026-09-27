// Аудит JS-порта: прогоняет js-port/est.js на входах, записанных текущим
// Python-ядром (tools/dev/record_current.py), и считает расхождение.
//
//   node tools/dev/compare_port.js [out/sim/rec_current.json] [js-port/est.js]
//
// Кроме расхождения порт–ядро считает точность каждого против истины имитатора,
// чтобы было видно, в какую сторону врёт устаревший порт.
const path = require('path');
const fs = require('fs');
const ROOT = path.resolve(__dirname, '..', '..');
const recPath = process.argv[2] || path.join(ROOT, 'out', 'sim', 'rec_current.json');
const estPath = path.resolve(process.argv[3] || path.join(ROOT, 'js-port', 'est.js'));
const T = require(estPath);
const rec = JSON.parse(fs.readFileSync(recPath, 'utf8'));
const MODE = T.MODE_NAMES;

const rows = [];
let totSteps = 0, totModeMis = 0;
for (const [name, data] of Object.entries(rec)) {
  const es = new T.Estimator();
  let mv = 0, sv2 = 0, mk = 0, msig = 0, mmu = 0, modeMis = 0, jsSlip = 0, pySlip = 0;
  let eJs2 = 0, ePy2 = 0, n = 0, firstMis = -1, dsEnd = 0, validMis = 0, covJs = 0, covPy = 0;
  const confusion = {};
  const t0 = process.hrtime.bigint();
  const outs = [];
  for (const r of data) outs.push(es.step(r[0], r[1]));
  const us = Number(process.hrtime.bigint() - t0) / 1000 / data.length;
  data.forEach((r, i) => {
    const [, , v, s, mode, sig, kt, mu, , vTrue, , , , , , , valid] = r;
    const o = outs[i];
    const dv = Math.abs(o.v - v);
    mv = Math.max(mv, dv); sv2 += dv * dv;
    mk = Math.max(mk, Math.abs(o.k_t - kt));
    msig = Math.max(msig, Math.abs(o.sigma_v - sig));
    mmu = Math.max(mmu, Math.abs(o.mu - mu));
    if (o.mode !== mode) {
      modeMis++; if (firstMis < 0) firstMis = i;
      const k = `${MODE[mode]}->${MODE[o.mode]}`; confusion[k] = (confusion[k] || 0) + 1;
    }
    if (mode === 4) pySlip++;
    if (o.mode === 4) jsSlip++;
    if (Boolean(o.valid) !== Boolean(valid)) validMis++;
    eJs2 += (o.v - vTrue) ** 2; ePy2 += (v - vTrue) ** 2; n++;
    if (Math.abs(o.v - vTrue) <= 2 * o.sigma_v) covJs++;
    if (Math.abs(v - vTrue) <= 2 * sig) covPy++;
  });
  const last = data[data.length - 1], oL = outs[outs.length - 1];
  dsEnd = oL.s - last[3];
  const posErrJs = oL.s - last[10], posErrPy = last[3] - last[10];
  totSteps += data.length; totModeMis += modeMis;
  const topConf = Object.entries(confusion).sort((a, b) => b[1] - a[1]).slice(0, 2).map(([k, c]) => `${k}:${c}`).join(' ');
  rows.push({
    name, steps: data.length, max_dv: mv, rms_dv: Math.sqrt(sv2 / n), ds_end: dsEnd,
    max_dk: mk, max_dsig: msig, max_dmu: mmu,
    mode_mis_pct: 100 * modeMis / data.length, first_mis_t: firstMis < 0 ? null : firstMis * 0.01,
    valid_mis_pct: 100 * validMis / data.length,
    rmse_v_js: Math.sqrt(eJs2 / n), rmse_v_py: Math.sqrt(ePy2 / n),
    pos_err_js: posErrJs, pos_err_py: posErrPy, slip_js: jsSlip, slip_py: pySlip, cov_js: 100 * covJs / n, cov_py: 100 * covPy / n,
    us_per_step_js: us, confusion: topConf
  });
}

const f = (x, d = 3) => (x === null ? '—' : x.toFixed(d));
console.log(`Порт: ${path.relative(ROOT, estPath)}; эталон: ${path.relative(ROOT, recPath)}`);
console.log('| Сценарий | max|dv|, м/с | RMS dv | ds конец, м | max|dk_t| | max|dσv| | режим ≠, % | 1-е расхожд., с | RMSE v порт/ядро, м/с | ошибка пути порт/ядро, м | SLIP порт/ядро | ±2σ порт/ядро, % | мкс/шаг JS |');
console.log('|---|---|---|---|---|---|---|---|---|---|---|---|---|');
for (const r of rows) {
  console.log(`| ${r.name} | ${f(r.max_dv)} | ${f(r.rms_dv)} | ${f(r.ds_end, 2)} | ${f(r.max_dk)} | ${f(r.max_dsig)} | ${f(r.mode_mis_pct, 1)} | ${f(r.first_mis_t, 2)} | ${f(r.rmse_v_js)} / ${f(r.rmse_v_py)} | ${f(r.pos_err_js, 2)} / ${f(r.pos_err_py, 2)} | ${r.slip_js} / ${r.slip_py} | ${f(r.cov_js, 1)} / ${f(r.cov_py, 1)} | ${f(r.us_per_step_js, 1)} |`);
}
console.log(`\nВсего шагов ${totSteps}, несовпадений режима ${totModeMis} (${(100 * totModeMis / totSteps).toFixed(2)} %)`);
console.log('Чаще всего режим ядро->порт:');
for (const r of rows) if (r.confusion) console.log(`  ${r.name}: ${r.confusion}`);
const outJson = recPath.replace(/\.json$/, '') + '_vs_' + path.basename(estPath, '.js') + '.json';
fs.writeFileSync(outJson, JSON.stringify(rows, null, 1));
console.log('\nтаблица ->', path.relative(ROOT, outJson));
