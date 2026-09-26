"""Сверка JS-связки (js-port/runner.js) с Python-связкой пакета (Runner) на
реальной записи: входы тележек и ручки в порядке записи bag, выходы каждого
шага сетки (скорость, путь, σ, режим, флаги, ускорение) и причинная база
«только колесо» (NaiveCore из tools/eval_replay.py). Пишет rec_runner.json;
затем `node compare_runner.js`.

    docker run --rm -v <repo>:/repo -v <data>:/repo/data:ro \
      -v <cache>:/repo/analysis/cache:ro -w /repo/js-port vectra/tram:integration \
      python3 record_runner.py [bag] [variant] [лист]

GNSS в связку не подаётся: скорость и путь ядра от него не зависят (он идёт
только в выставку, docs/POSITION_FRAME.md), а JS-связка песочницы выставку не
повторяет. Лист по умолчанию — лист жюри config/tram.yaml (как в песочнице).
"""
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_replay as X  # noqa: E402  (тот же лист, та же связка, те же аномалии)
import eval_replay as ER  # noqa: E402  (база «только колесо»)

bag = sys.argv[1] if len(sys.argv) > 1 else X.DEFAULT_RUN
variant = sys.argv[2] if len(sys.argv) > 2 else "clean"
sheet = sys.argv[3] if len(sys.argv) > 3 else os.path.join(X.PKG, "config", "tram.yaml")
params, node, _ = X.resolve_sheet(sheet)
a = X.bagio.load(bag)
b, info = X.make_variant(a, bag, variant)
if b is None:
    sys.exit(f"вариант {variant}: окно не найдено")
r = X.make_runner(params, node, None)
rn = ER.make_naive(params, node, None)

events, rows = [], []
for tb, kind, i, th, val in X.E.events(b):
    if kind == 2:
        continue                              # GNSS — только выставка, см. docstring
    if kind == 0:
        outs, outn = r.on_wheel(i, th, val), rn.on_wheel(i, th, val)
    else:
        outs, outn = r.on_handle(th, val), rn.on_handle(th, val)
    events.append([kind, int(i) if kind == 0 else 0, float(th), float(val)])
    assert len(outs) == len(outn)
    for o, q in zip(outs, outn):
        rows.append([len(events) - 1, o["stamp"], o["v"], o["s"], o["sigma_v"], o["sigma_s"],
                     int(o["mode"]), bool(o["valid"]), bool(o["ambiguous"]), o["a"],
                     bool(o["wheels_stale"]), bool(o["handle_ok"]), q["v"], q["s"]])
robust = dict(resets=r.resets, gaps=r.gaps, core_resets=r.core_resets,
              skipped_steps=r.skipped_steps, rejected_stamps=r.rejected_stamps,
              rejected_values=r.rejected_values)
node_used = {k: node[k] for k in ("wheel_timeout_s", "handle_timeout_s") if k in node}
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rec_runner.json")
import dataclasses  # noqa: E402
pd = {k: (list(v) if isinstance(v, tuple) else v) for k, v in dataclasses.asdict(params).items()}
with open(out, "w", encoding="utf-8") as fh:
    json.dump({"bag": bag, "variant": variant, "params": pd, "node": node_used,
               "events": events, "rows": rows, "robust": robust,
               "core_sha1": X.core_sha1()}, fh)
print("ok", bag, variant, len(events), "сообщений,", len(rows), "шагов; ветки устойчивости:", robust)
