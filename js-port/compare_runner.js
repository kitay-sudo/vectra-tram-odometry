// Сверка JS-связки (runner.js поверх est.js) с Python-связкой пакета на реальной записи
// (rec_runner.json из record_runner.py): те же сообщения тележек и ручки -> те же шаги сетки.
//   node compare_runner.js   -> код выхода 1, если расхождение > 1e-6, число шагов, режимы или
//                               флаги не совпали, в записи сработали ветки устойчивости связки
//                               (их JS-связка не повторяет) или ядро не то, с которым сверен порт
const T = require('./est.js');
const { Runner } = require('./runner.js');
const rec = require('./rec_runner.json');
const r = new Runner(rec.params, rec.node);
r.core.diag();                    // как в песочнице: запись решений по осям включена (на числа не влияет)
let k = 0, n = 0, mv = 0, ms = 0, msv = 0, mss = 0, ma = 0, mnv = 0, mns = 0, modeMis = 0, flagMis = 0, stampMis = 0, countMis = 0;
const t0 = process.hrtime.bigint();
const want = new Map();
for (const row of rec.rows) { if (!want.has(row[0])) want.set(row[0], []); want.get(row[0]).push(row); }
rec.events.forEach((e, idx) => {
  const [kind, i, th, val] = e;
  const outs = kind === 0 ? r.on_wheel(i, th, val) : r.on_handle(th, val);
  const exp = want.get(idx) || [];
  if (outs.length !== exp.length) countMis++;
  for (let j = 0; j < Math.min(outs.length, exp.length); j++) {
    const o = outs[j], [, stamp, v, s, sv, ss, mode, valid, amb, a, stale, hok, nv, ns, slip, slip_all] = exp[j];
    n++;
    if (Math.abs(o.stamp - stamp) > 1e-9) stampMis++;
    mv = Math.max(mv, Math.abs(o.v - v)); ms = Math.max(ms, Math.abs(o.s - s)); msv = Math.max(msv, Math.abs(o.sigma_v - sv));
    mss = Math.max(mss, Math.abs(o.sigma_s_core - ss)); ma = Math.max(ma, Math.abs(o.a - a));
    mnv = Math.max(mnv, Math.abs(o.naive_v - nv)); mns = Math.max(mns, Math.abs(o.naive_s - ns));
    if (o.mode !== mode) modeMis++;
    if (o.valid !== valid || o.ambiguous !== amb || o.wheels_stale !== stale || o.handle_ok !== hok
        || (slip !== undefined && (o.slip !== slip || (o.slip_all || 0) !== slip_all))) flagMis++;
  }
  k++;
});
const us = Number(process.hrtime.bigint() - t0) / 1000 / Math.max(1, n);
const worst = Math.max(mv, ms, msv, mss, ma, mnv, mns);
const rb = rec.robust || {}, robustOk = !rb.resets && !rb.gaps && !rb.core_resets && !rb.skipped_steps;
console.log(`связка ${rec.bag} ${rec.variant}: сообщений ${k}, шагов ${n}/${rec.rows.length}; max|dv| ${mv.toExponential(2)} м/с, |ds| ${ms.toExponential(2)} м, |dσv| ${msv.toExponential(2)}, |dσs| ${mss.toExponential(2)}, |da| ${ma.toExponential(2)}; база: |dv| ${mnv.toExponential(2)}, |ds| ${mns.toExponential(2)}; режим ≠ ${modeMis}, флаги ≠ ${flagMis}, метка ≠ ${stampMis}, число шагов ≠ ${countMis}; ветки устойчивости ${JSON.stringify(rb)}; ${us.toFixed(1)} мкс/шаг`);
const numOk = worst <= 1e-6 && modeMis === 0 && flagMis === 0 && stampMis === 0 && countMis === 0 && n === rec.rows.length;
const shaOk = !rec.core_sha1 || rec.core_sha1 === T.PORT.core_sha1;
if (!robustOk) console.log('в записи сработали ветки устойчивости связки (сброс, провал, пропуск узлов): JS-связка их не повторяет - выберите другую запись');
if (!shaOk) console.log(`ядро ${rec.core_sha1} ≠ ядро сверки порта ${T.PORT.core_sha1}`);
process.exit(numOk && shaOk && robustOk ? 0 : 1);
