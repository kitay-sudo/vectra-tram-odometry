// Сводка песочницы для docs/SANDBOX.md: все сценарии без браузера, тем же кодом, что страница.
// Короткая таблица с числами и итогом, под ней описания по сценариям списком.
//
//   node simulator/test/sandbox_report.js            -> печать таблицы и simulator/test/sandbox_report.json
//   node simulator/test/sandbox_report.js --write    -> то же + сводка в docs/SANDBOX.md (между метками)
//
// Воспроизводимо: имитатор с зерном (SEED, по умолчанию 7), ядро детерминировано; страница с тем же
// зерном даёт те же числа (simulator/test/page_test.js сверяет с sandbox_report.json).
const fs = require('fs');
const path = require('path');
const JS = path.join(__dirname, '..', 'js');
const SB = require(path.join(JS, 'sandbox.js'));
const sheet = require(path.join(JS, 'sheet.js'));
// линия - pathgraph организаторов (js/track.js); без файла - заглушка, и в docs такое не пишется
let track = null;
try { track = require(path.join(JS, 'track.js')); } catch (_) { console.error('внимание: нет simulator/js/track.js (simulator/tools/gen_track.py) - линия-заглушка, числа не для docs/SANDBOX.md'); }
const T = require(path.join(JS, 'est.js'));
const SEED = +(process.env.SEED || 7);
const write = process.argv.includes('--write');

const f = (x, d = 2) => (x === null || x === undefined || !isFinite(x)) ? 'н/д' : x.toFixed(d).replace('.', ',').replace(/^-/, '−');
const t0 = Date.now();
const rows = [];
for (const p of SB.PRESETS.filter(x => !x.manual && !x.record)) {
  const e = SB.runPreset(p.key, { sheet, track, seed: SEED });
  rows.push({ key: p.key, ru: p.ru, about: p.about, expect: p.expect, real: p.real || null, summary: e.summary() });
}
// разброс «сухо» по зёрнам имитатора (шум датчиков, пропуски тактов)
const drySeeds = [1, 2, 3, 4, 5, 6, 7, 8].map(seed => { const s = SB.runPreset('dry', { sheet, track, seed }).summary(); return { seed, v_mae: s.v_mae, naive_v_mae: s.naive_v_mae, cov2s: s.cov2s }; });

function does(S) {
  const out = [];
  if (S.rej_front > 0.4) out.push(`отбрасывала переднюю ${f(S.rej_front, 1)} с`);
  if (S.rej_rear > 0.4) out.push(`отбрасывала заднюю ${f(S.rej_rear, 1)} с`);
  if (S.exc_front > 0.4) out.push(`исключала переднюю как неисправную ${f(S.exc_front, 1)} с`);
  if (S.exc_rear > 0.4) out.push(`исключала заднюю как неисправную ${f(S.exc_rear, 1)} с`);
  if (S.slip_s > 0.2) out.push(`признак срыва ${f(S.slip_s, 1)} с, μ до ${f(S.mu_min, 2)}`);
  if (S.amb_s > 0.2) out.push(`«стоим или скользим?» ${f(S.amb_s, 1)} с`);
  if (S.frozen_s > 0.2) out.push(`«залипли обе» ${f(S.frozen_s, 1)} с`);
  if (S.stale_s > 0.5) out.push(`без колёс (разомкнутый режим) ${f(S.stale_s, 1)} с`);
  if (S.invalid_s > 0.5) out.push(`недостоверно ${f(S.invalid_s, 1)} с`);
  if (S.overconf_s > 0.5) out.push(`**считала оценку достоверной, хотя ошибка больше 0,5 м/с и вне ±2σ, ${f(S.overconf_s, 1)} с**`);
  if (S.agree_steps > 0) out.push(`уступала колёсам по согласию тележек (${S.agree_steps} ${plural(S.agree_steps, 'шаг', 'шага', 'шагов')})`);
  return out.length ? out.join('; ') : 'шла по колёсам, отбрасываний нет';
}
const plural = (n, one, few, many) => { const m10 = n % 10, m100 = n % 100; return m10 === 1 && m100 !== 11 ? one : m10 >= 2 && m10 <= 4 && (m100 < 12 || m100 > 14) ? few : many; };
// по средней ошибке в окне условий (для «Сухо» - по всему прогону) и по наибольшей
function verdict(S) {
  const W = S.win, a = W ? W.v_mae : S.v_mae, b = W ? W.naive_v_mae : S.naive_v_mae;
  const ma = W ? W.v_max : S.v_max, mb = W ? W.naive_v_max : S.naive_v_max;
  const avg = a < 0.8 * b ? 'лучше' : a > 1.2 * b ? 'хуже' : 'на уровне';
  const worstMax = ma > 1.2 * mb;
  if (avg === 'лучше') return worstMax ? '**лучше колеса** в среднем, но наибольшая ошибка больше' : '**лучше колеса**';
  if (avg === 'хуже') return '**хуже колеса**';
  return 'на уровне колеса' + (worstMax ? ', наибольшая ошибка больше' : '');
}
// тот же отказ на реальных записях: сводка инъекций docs/EVAL.md раздел 6 (3 отложенные записи,
// окно - на ходу; путь - остаток вдоль пути через 300 с, с картой и привязками к остановкам)
const EVAL_MD = path.join(__dirname, '..', '..', 'docs', 'EVAL.md');
function evalInject() {
  if (!fs.existsSync(EVAL_MD)) return null;
  const src = fs.readFileSync(EVAL_MD, 'utf8'), i = src.indexOf('| вид | прогонов | MAE модели | MAE базы');
  if (i < 0) return null;
  const out = {};
  for (const line of src.slice(i).split('\n').slice(2)) {
    if (!line.startsWith('|')) break;
    const c = line.split('|').slice(1, -1).map(x => x.trim());
    out[c[0]] = { mae: c[2], maeN: c[3], along: c[4], alongN: c[5], verdict: c[8] };
  }
  return out;
}
const EV = evalInject();
if (!EV) console.error('внимание: в docs/EVAL.md нет сводки инъекций (раздел 6) - столбец «на реальных записях» пуст');
// тот же отказ на реальных записях одной фразой: MAE и остаток пути, модель / база
function realTxt(r) {
  if (typeof r.real === 'string') return r.real.replace(/^нет:\s*/, 'нет, ');
  return r.real.map(k => { const x = EV && EV[k]; return x ? `${k}: MAE ${x.mae} / ${x.maeN} м/с, остаток пути ${x.along} / ${x.alongN} м` : `${k}: нет в docs/EVAL.md`; }).join('; ');
}
// короткая таблица: только числа и итог; описания - списком под ней
const head = '| Сценарий | MAE скорости, м/с | Путь в конце, м | Истина в ±2σ | Итог |\n|---|---|---|---|---|';
const lines = rows.map(r => {
  const S = r.summary;
  return `| ${r.ru} | ${f(S.v_mae, 3)} / ${f(S.naive_v_mae, 3)} | ${f(S.s_err, 1)} / ${f(S.naive_s_err, 1)} | ${f(100 * S.cov2s, 1)} % | ${verdict(S)} |`;
});
const notes = rows.map(r => {
  const S = r.summary, W = S.win;
  const win = W ? `, в окне условий (${f(W.s, 0)} с) MAE ${f(W.v_mae, 3)} / ${f(W.naive_v_mae, 3)} м/с` : '';
  const real = r.real ? ` На реальных записях (docs/EVAL.md, раздел 6): ${realTxt(r)}.` : '';
  return `- **${r.ru}.** ${r.about} Что делала модель: ${does(S)}. Наибольшая ошибка ${f(S.v_max * 3.6, 1)} / ${f(S.naive_v_max * 3.6, 1)} км/ч${win}.${real}`;
});
const dm = a => a.reduce((x, y) => x + y, 0) / a.length;
const dryTxt = `«Сухо» по 8 зёрнам имитатора: MAE модели ${f(dm(drySeeds.map(x => x.v_mae)), 4)} м/с (от ${f(Math.min(...drySeeds.map(x => x.v_mae)), 4)} до ${f(Math.max(...drySeeds.map(x => x.v_mae)), 4)}), `
  + `«только колесо» ${f(dm(drySeeds.map(x => x.naive_v_mae)), 4)} м/с (от ${f(Math.min(...drySeeds.map(x => x.naive_v_mae)), 4)} до ${f(Math.max(...drySeeds.map(x => x.naive_v_mae)), 4)}), истина в ±2σ ${f(100 * dm(drySeeds.map(x => x.cov2s)), 1)} %.`;
const stamp = `Ядро ${T.PORT.core_sha1} (${T.PORT.core_git}), лист ${sheet.path} (${sheet.sheet_sha1}), зерно имитатора ${SEED}; \`node simulator/test/sandbox_report.js --write\`.`;
const md = 'Числа в таблице и ниже: модель / колесо (среднее двух тележек).\n\n' + [head, ...lines].join('\n')
  + '\n\nПодробно по сценариям:\n\n' + notes.join('\n') + '\n\n' + dryTxt + '\n\n' + stamp;
console.log(md);
const doc = { made_by: 'simulator/test/sandbox_report.js', seed: SEED, core_sha1: T.PORT.core_sha1, sheet_sha1: sheet.sheet_sha1, presets: rows, dry_seeds: drySeeds };
if (track) fs.writeFileSync(path.join(__dirname, 'sandbox_report.json'),JSON.stringify(doc, null, 1) + '\n');
if (write && !track) { console.error('--write без js/track.js не делаю: таблица docs/SANDBOX.md - по настоящей линии'); process.exit(1); }
if (write) {
  const p = path.join(__dirname, '..', '..', 'docs', 'SANDBOX.md');
  const src = fs.readFileSync(p, 'utf8');
  const a = src.indexOf('<!-- sandbox_report:begin -->'), b = src.indexOf('<!-- sandbox_report:end -->');
  if (a < 0 || b < 0) { console.error('нет меток <!-- sandbox_report:begin/end --> в docs/SANDBOX.md'); process.exit(1); }
  fs.writeFileSync(p, src.slice(0, a) + '<!-- sandbox_report:begin -->\n' + md + '\n' + src.slice(b));
  console.log('Готово: сводка записана в docs/SANDBOX.md');
}
console.error(`сценариев ${rows.length}, ${((Date.now() - t0) / 1000).toFixed(1)} с`);
