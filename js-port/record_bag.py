"""Сверка JS-порта на листе вагона (config/tram.yaml: 2 тележки, км/ч, табличный
привод, 20 Гц) по реальной записи: пишет входы и выходы Estimator.step /
step_open_loop так, как их вызывает Runner (маска свежих показаний, handle_ok,
разомкнутый режим), в rec_bag.json. Затем `node compare_bag.js`.

    docker run --rm -v <repo>:/repo -v <repo>/analysis/cache:/repo/analysis/cache:ro \
      -w /repo/js-port vectra/tram:dev python3 record_bag.py [bag] [variant]

variant — как в tools/export_replay.py (clean, front_zero, both_zero, dropout, skid_brake).
"""
import dataclasses
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_replay as X  # noqa: E402  (тот же лист, та же связка, те же аномалии)

bag = sys.argv[1] if len(sys.argv) > 1 else X.DEFAULT_RUN
variant = sys.argv[2] if len(sys.argv) > 2 else "clean"
# лист вагона (2 тележки, км/ч, табличный привод) — для сверки порта годится любой лист вагона;
# третий аргумент — другой лист (путь), например оценочный config/eval/tram.yaml
sheet = sys.argv[3] if len(sys.argv) > 3 else os.path.join(X.PKG, "config", "tram.yaml")
params, node, _ = X.resolve_sheet(sheet)
a = X.bagio.load(bag)
b, info = X.make_variant(a, bag, variant)
C = X.EC.Estimator
rows = []
o_step, o_ol = C.step, C.step_open_loop


def step(self, notch, meas, fresh=True, handle_ok=True):
    fm = [bool(x) for x in (fresh if hasattr(fresh, "__len__") else [fresh] * self.nw)]
    m = [float(x) for x in meas]
    o = o_step(self, notch, meas, fresh=fresh, handle_ok=handle_ok)
    rows.append([0, float(notch), m, fm, bool(handle_ok), o["v"], o["s"], int(o["mode"]), o["sigma_v"],
                 o["k_t"], o["mu"], o["d"], o["sigma_s"], bool(o["ambiguous"])])
    return o


def step_open_loop(self, notch):
    o = o_ol(self, notch)
    rows.append([1, float(notch), [], [], True, o["v"], o["s"], int(o["mode"]), o["sigma_v"],
                 o["k_t"], o["mu"], o["d"], o["sigma_s"], bool(o["ambiguous"])])
    return o


C.step, C.step_open_loop = step, step_open_loop
r = X.make_runner(params, node, None)       # без карты: сверяется ядро
X.replay(b, [r])
pd = {k: (list(v) if isinstance(v, tuple) else v) for k, v in dataclasses.asdict(params).items()}
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rec_bag.json")
with open(out, "w", encoding="utf-8") as fh:
    json.dump({"bag": bag, "variant": variant, "params": pd, "rows": rows,
               "core_sha1": X.core_sha1()}, fh)
print("ok", bag, variant, len(rows), "шагов;", sum(1 for x in rows if x[0] == 1), "разомкнутых")
