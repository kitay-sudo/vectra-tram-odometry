"""Таблица «по вагонам» из нескольких прогонов tools/eval.py (docs/VEHICLES.md).

Каждый прогон eval.py - свой вариант параметра vehicle (`--set vehicle=…`) в
своём каталоге --out. Скрипт читает их runs.json и печатает markdown: по
группам вагонов (и по датам записи, если --by-date) - скорость MAE и
смещение, положение ср. 3D, конец (ср. / медиана / макс), дрейф 3D по концу
(% пути: медиана / ср. / макс), вдоль пути RMSE. Средние скорости и 3D
взвешены числом пар скорости прогона (как eval.py); конец и дрейф - по прогонам.

    python3 tools/vehicle_report.py auto=out/vehicle/auto 30618=out/vehicle/v30618 \\
        match=out/vehicle/match [--by-date] [--json out/vehicle/report.json]
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BAGS = ROOT / "docs" / "data" / "bags.csv"


def load(path):
    return json.loads((Path(path) / "runs.json").read_text(encoding="utf-8"))


def dates():
    return {r["bag"]: r["start_msk"][:10] for r in csv.DictReader(BAGS.open(encoding="utf-8"))}


def _fin(xs):
    return [x for x in xs if x is not None and math.isfinite(x)]


def group(runs, ids, est="model"):
    rows = [runs[b][est] for b in ids if b in runs and est in runs[b]]
    rows = [r for r in rows if r.get("v_pairs", 0) > 0]
    if not rows:
        return None
    w = np.array([r["v_pairs"] for r in rows], float)

    def wavg(k):
        x = np.array([r.get(k, np.nan) for r in rows], float)
        m = np.isfinite(x)
        return float(np.average(x[m], weights=w[m])) if m.any() else float("nan")

    def stat(k, fn):
        x = _fin([r.get(k) for r in rows])
        return float(fn(x)) if x else float("nan")
    # вдоль RMSE - пул: корень из взвешенного среднего квадратов
    ar = np.array([r.get("along_rmse", np.nan) for r in rows], float)
    m = np.isfinite(ar)
    return dict(runs=len(rows), v_pairs=int(w.sum()), v_mae=wavg("v_mae"), v_bias=wavg("v_bias"),
                p3d_mean=wavg("p3d_mean"),
                end_mean=stat("p3d_end", np.mean), end_median=stat("p3d_end", np.median),
                end_max=stat("p3d_end", np.max),
                drift_median=stat("drift_pct_3d", np.median), drift_mean=stat("drift_pct_3d", np.mean),
                drift_max=stat("drift_pct_3d", np.max),
                along_rmse=float(np.sqrt(np.average(ar[m] ** 2, weights=w[m]))) if m.any() else float("nan"))


def fmt(x, nd):
    if x is None or not math.isfinite(x):
        return "-"
    s = f"{x:+.{nd}f}" if nd < 0 else f"{x:.{nd}f}"
    return s.replace(".", ",").replace("-", "−")


def row(label, g):
    return (f"| {label} | {g['runs']} | {fmt(g['v_mae'], 4)} | "
            f"{('+' if g['v_bias'] >= 0 else '') + fmt(g['v_bias'], 4)} | {fmt(g['p3d_mean'], 2)} | "
            f"{fmt(g['end_mean'], 1)} / {fmt(g['end_median'], 2)} / {fmt(g['end_max'], 1)} | "
            f"{fmt(g['drift_median'], 3)} / {fmt(g['drift_mean'], 3)} / {fmt(g['drift_max'], 3)} | "
            f"{fmt(g['along_rmse'], 2)} |")


HEAD = ("| вариант | прогонов | скорость MAE, м/с | смещение, м/с | 3D ср., м | "
        "конец 3D ср. / мед. / макс, м | дрейф 3D по концу, % мед. / ср. / макс | вдоль RMSE, м |\n"
        "|---|---|---|---|---|---|---|---|")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("variants", nargs="+", help="подпись=каталог out eval.py")
    ap.add_argument("--by-date", action="store_true", help="и группы «вагон, дата»")
    ap.add_argument("--naive", action="store_true", help="и база «только колесо»")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    var = [v.rsplit("=", 1) for v in a.variants]
    data = {k: load(p) for k, p in var}
    ids = sorted(set.intersection(*(set(d) for d in data.values())))
    dt = dates()
    groups = {"оба вагона": ids}
    for veh in ("30618", "30639"):
        sel = [b for b in ids if b.startswith(veh)]
        if sel:
            groups[veh] = sel
    if a.by_date:
        for b in ids:
            groups.setdefault(f"{b[:5]}, {dt.get(b, '?')}", []).append(b)
    out = {}
    for gname, gids in groups.items():
        print(f"\n**{gname}** ({len(gids)} прогонов)\n")
        print(HEAD)
        out[gname] = {}
        for k, _ in var:
            for est in (("model", "naive") if a.naive else ("model",)):
                g = group(data[k], gids, est)
                if g is None:
                    continue
                label = k if est == "model" else f"{k} (база)"
                out[gname][label] = g
                print(row(label, g))
    if a.json:
        Path(a.json).parent.mkdir(parents=True, exist_ok=True)
        Path(a.json).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
