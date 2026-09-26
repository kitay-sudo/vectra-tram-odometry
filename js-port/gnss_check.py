"""Сценарий песочницы «Застройка без GNSS» на настоящей связке: GNSS после окна
выставки на выход не влияет. Запись прогоняется через Runner пакета (лист жюри,
карта пакета) три раза и сравниваются выходы каждого шага:

  (1) GNSS master/rover на ВЕСЬ прогон  против  (2) без GNSS вовсе —
      скорость, путь, σ, режим (ядру GNSS не нужен совсем);
  (1) GNSS на весь прогон  против  (3) GNSS только первые 3 с (как в bag жюри,
      analysis/evaluate.events) — всё то же плюс положение x, y, z и признаки
      «положение выставлено / опубликовано».

    docker run --rm -v <repo>:/repo -v <data>:/repo/data:ro \
      -v <cache>:/repo/analysis/cache:ro -w /repo/js-port vectra/tram:integration \
      python3 gnss_check.py [bag]
"""
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_replay as X  # noqa: E402

INIT_S = 3.0
bag = sys.argv[1] if len(sys.argv) > 1 else X.DEFAULT_RUN
params, node, _ = X.resolve_sheet(os.path.join(X.PKG, "config", "tram.yaml"))
MAP = os.path.join(X.PKG, "config", "track_map.npz")
a = X.bagio.load(bag)
ev = []
for i, key in enumerate(("front", "rear")):
    for tb, th, v in a[key]:
        ev.append((tb, 0, i, th, v))
for tb, th, n in a["cmd"]:
    ev.append((tb, 1, 0, th, n))
gn = [(row[0], 2, ant, row[1], (row[2], row[3], row[4], int(row[5])))
      for key, ant in (("mfix", "master"), ("rfix", "rover")) for row in a[key]]
t_end = a["mfix"][0, 0] + INIT_S          # окно выставки — от первой записи master (evaluate.events)
gn3 = [g for g in gn if g[0] <= t_end]


def run(gnss):
    tm = X.TrackMap.load(MAP) if os.path.exists(MAP) else None
    r = X.make_runner(params, node, tm)
    out = []
    for tb, kind, i, th, val in sorted(ev + gnss, key=lambda e: e[0]):
        if kind == 0:
            out += r.on_wheel(i, th, val)
        elif kind == 1:
            out += r.on_handle(th, val)
        else:
            out += r.on_fix(th, i, *val)
    return out


def diff(P, Q, keys):
    """max |Δ| по числовым ключам (NaN = NaN), число несовпадений по остальным."""
    n = min(len(P), len(Q))
    res = {}
    for k in keys:
        vals = [(P[j].get(k), Q[j].get(k)) for j in range(n)]
        if all(isinstance(p, (int, float)) and not isinstance(p, bool) or p is None for p, _ in vals):
            m = 0.0
            for p, q in vals:
                if p is None and q is None:
                    continue
                if p is None or q is None:
                    m = math.inf
                    continue
                if math.isnan(p) and math.isnan(q):
                    continue
                m = max(m, abs(p - q)) if not (math.isnan(p) or math.isnan(q)) else math.inf
            res[k] = m
        else:
            res[k] = sum(p != q for p, q in vals)
    return res


A, B, C = run(gn), run([]), run(gn3)
core = ("v", "s", "sigma_v", "mode")
pos = ("x", "y", "z", "pos_ready", "pos_valid")
dAB = diff(A, B, core)
dAC = diff(A, C, core + tuple(k for k in pos if k in A[0]))
fmt = lambda d: ", ".join(f"{k} {v:.3g}" if isinstance(v, float) else f"{k}: расхождений {v}" for k, v in d.items())  # noqa: E731
print(f"{bag}: точек GNSS {len(gn)} (весь прогон), {len(gn3)} (первые {INIT_S:.0f} с); "
      f"шагов {len(A)} / {len(B)} / {len(C)}")
print(f"  весь прогон против без GNSS: {fmt(dAB)}")
print(f"  весь прогон против первых {INIT_S:.0f} с: {fmt(dAC)}")
ok = len(A) == len(B) == len(C) and all(v == 0 for v in dAB.values()) and all(v == 0 for v in dAC.values())
sys.exit(0 if ok else 1)
