"""Сценарий песочницы «Застройка без GNSS» на настоящей связке: скорость и путь
ядра не зависят от GNSS. Запись прогоняется через Runner пакета дважды — с
GNSS master/rover на ВЕСЬ прогон и без GNSS вовсе — и сравниваются выходы
каждого шага (v, s, σ, режим).

    docker run --rm -v <repo>:/repo -v <data>:/repo/data:ro \
      -v <cache>:/repo/analysis/cache:ro -w /repo/js-port vectra/tram:integration \
      python3 gnss_check.py [bag]
"""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_replay as X  # noqa: E402

bag = sys.argv[1] if len(sys.argv) > 1 else X.DEFAULT_RUN
params, node, _ = X.resolve_sheet(os.path.join(X.PKG, "config", "tram.yaml"))
a = X.bagio.load(bag)
ev = []
for i, key in enumerate(("front", "rear")):
    for tb, th, v in a[key]:
        ev.append((tb, 0, i, th, v))
for tb, th, n in a["cmd"]:
    ev.append((tb, 1, 0, th, n))
gn = [(row[0], 2, ant, row[1], (row[2], row[3], row[4], int(row[5])))
      for key, ant in (("mfix", "master"), ("rfix", "rover")) for row in a[key]]


def run(with_gnss):
    r = X.make_runner(params, node, None)
    out = []
    for tb, kind, i, th, val in sorted(ev + (gn if with_gnss else []), key=lambda e: e[0]):
        if kind == 0:
            out += r.on_wheel(i, th, val)
        elif kind == 1:
            out += r.on_handle(th, val)
        else:
            out += r.on_fix(th, i, *val)
    return out


A, B = run(True), run(False)
n = min(len(A), len(B))
dv = max(abs(A[k]["v"] - B[k]["v"]) for k in range(n))
ds = max(abs(A[k]["s"] - B[k]["s"]) for k in range(n))
dsv = max(abs(A[k]["sigma_v"] - B[k]["sigma_v"]) for k in range(n))
dm = sum(A[k]["mode"] != B[k]["mode"] for k in range(n))
print(f"{bag}: точек GNSS {len(gn)} (весь прогон); шагов {len(A)} / {len(B)}; "
      f"max|dv| {dv:.3g} м/с, max|ds| {ds:.3g} м, max|dσv| {dsv:.3g}, режим ≠ {dm}")
sys.exit(0 if len(A) == len(B) and dv == 0 and ds == 0 and dm == 0 else 1)
