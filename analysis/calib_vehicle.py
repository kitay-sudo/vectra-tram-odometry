"""Различия трамваев: масштаб колёс по вагону и дате, листы по вагонам.

Организаторы 26.09: проверка — только вагон 30618, учёт различий трамваев
засчитывается в плюс. Здесь:

  1. масштаб показаний тележек (скорость тележек / скорость GNSS на
     установившемся движении, как analysis/calib_sheet.speed_stats) по
     каждой записи с GNSS, затем медианы по вагону, по вагону и дате;
  2. масштаб колёс каждого вагона для листа ОЦЕНКИ (только train_unique
     вагона) и листа ЖЮРИ (все уникальные записи вагона) — блок "vehicles"
     калибровки config/[eval/]tram_calibration.json (его пишет и
     calib_sheet.py); tools/gen_params.py переносит его в лист ноды
     (параметры vehicle, vehicle_ids, vehicle_meas_scale;
     tram_state_estimator/vehicle.py).

    python3 analysis/calib_vehicle.py            # таблицы
    python3 analysis/calib_vehicle.py --runs     # и строка на каждую запись
    python3 analysis/calib_vehicle.py --write    # и блок "vehicles" в обе калибровки

Отложенные записи (holdout) в лист ОЦЕНКИ не попадают.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bagio                                   # noqa: E402
import calib_drive                             # noqa: E402

KMH = 3.6                                      # км/ч в м/с (как calib_sheet)

ROOT = HERE.parent
BAGS = ROOT / "docs" / "data" / "bags.csv"
SPLIT = ROOT / "tools" / "split.json"
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
CALIB = {"eval": PKG / "config" / "eval" / "tram_calibration.json",
         "jury": PKG / "config" / "tram_calibration.json"}
VEHICLES = ("30618", "30639")
# вагон по умолчанию в листе: проверка организаторов — только 30618 (26.09)
DEFAULT_VEHICLE = "30618"


def run_ratio(bag):
    """(отношение «тележки / GNSS» — медиана по записи, число отсчётов) или
    None. Отбор отсчётов — как calib_sheet.speed_stats: |v| GNSS > 3 м/с,
    |ускорение| < 0,1 м/с², тележки согласны (< 0,2 м/с)."""
    a = bagio.load(bag)
    f, r, g = a["front"], a["rear"], a["mvel"]
    if len(g) < 100 or len(f) < 100 or len(r) < 100:
        return None
    tg, vg = g[:, 1], np.hypot(g[:, 2], g[:, 3])
    fi = np.interp(tg, f[:, 1], f[:, 2]) / KMH
    ri = np.interp(tg, r[:, 1], r[:, 2]) / KMH
    acc = np.gradient(vg, tg)
    m = (vg > 3.0) & (np.abs(acc) < 0.1) & (np.abs(fi - ri) < 0.2)
    if m.sum() < 50:
        return None
    w = 0.5 * (fi + ri)
    return w[m] / vg[m]


def meta():
    rows = {r["bag"]: r for r in csv.DictReader(BAGS.open(encoding="utf-8"))}
    return {b: dict(vehicle=r["vehicle"], date=r["start_msk"][:10]) for b, r in rows.items()}


def fit_scale(ids, per_run):
    """Масштаб колёс набора записей: 1 / медиана отношения по всем отсчётам
    (как calib_sheet: пул отсчётов, а не медиана медиан)."""
    x = [per_run[b] for b in ids if b in per_run]
    if not x:
        return None, 0, 0
    ratio = float(np.median(np.concatenate(x)))
    return 1.0 / ratio, len(x), int(sum(len(v) for v in x))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", action="store_true", help="строка на каждую запись")
    ap.add_argument("--write", action="store_true",
                    help="записать блок vehicles в config/[eval/]tram_calibration.json")
    a = ap.parse_args()
    md = meta()
    split = json.loads(SPLIT.read_text(encoding="utf-8"))
    sets = {"eval": calib_drive.fit_ids("eval"), "jury": calib_drive.fit_ids("jury")}
    hold = set(split["holdout_scored"])
    uniq = sorted(set(sets["jury"]))
    per_run = all_ratios(uniq)
    if a.runs:
        print("запись            вагон  дата        набор     отношение  отсчётов")
        for b in sorted(per_run):
            s = "holdout" if b in hold else ("train" if b in sets["eval"] else "—")
            print(f"{b}  {md[b]['vehicle']}  {md[b]['date']}  {s:8s}  "
                  f"{np.median(per_run[b]):.5f}  {len(per_run[b])}")

    print("\nмасштаб колёс (meas_scale = GNSS / тележки), медиана отсчётов:")
    print("вагон  дата        записей  отсчётов  meas_scale  к общему, %")
    s_all, _, _ = fit_scale(list(per_run), per_run)
    groups = {}
    for b in per_run:
        groups.setdefault((md[b]["vehicle"], md[b]["date"]), []).append(b)
    for (v, d) in sorted(groups):
        s, n, k = fit_scale(groups[(v, d)], per_run)
        print(f"{v}  {d}  {n:7d}  {k:8d}  {s:.5f}     {100 * (s / s_all - 1):+.2f}")
    for v in VEHICLES:
        s, n, k = fit_scale([b for b in per_run if md[b]["vehicle"] == v], per_run)
        print(f"{v}  все даты    {n:7d}  {k:8d}  {s:.5f}     {100 * (s / s_all - 1):+.2f}")
    print(f"оба    все даты    {len(per_run):7d}            {s_all:.5f}")

    for which in ("eval", "jury"):
        blk = vehicle_block(which, per_run, md)
        print(f"\nлист {which}: " + ", ".join(
            f"{v} {blk[v]['meas_scale']:.6f} ({blk[v]['_fit_runs_gnss']} записей с GNSS)"
            for v in VEHICLES) + f"; оба {blk['_mixed_meas_scale']:.6f}")
        if a.write:
            f = CALIB[which]
            d = json.loads(f.read_text(encoding="utf-8"))
            d["vehicles"] = blk
            f.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
            print("записано:", f.relative_to(ROOT).as_posix())


def all_ratios(ids):
    return {b: x for b, x in ((b, run_ratio(b)) for b in ids) if x is not None}


def vehicle_block(which, per_run=None, md=None):
    """Блок "vehicles" калибровки листа which: масштаб колёс каждого вагона
    по записям подгонки листа (eval — train_unique, jury — все уникальные)."""
    md = md or meta()
    ids = calib_drive.fit_ids(which)
    per_run = per_run if per_run is not None else all_ratios(ids)
    out = {"_source": "analysis/calib_vehicle.py: масштаб колёс (GNSS / тележки) по "
                      "записям вагона с GNSS, набор "
                      + ("train_unique" if which == "eval" else "train_unique + holdout"),
           "_default": DEFAULT_VEHICLE}
    s, n, _ = fit_scale([b for b in ids if b in per_run], per_run)
    out["_mixed_meas_scale"] = round(s, 6)
    for v in VEHICLES:
        vids = sorted(b for b in ids if md[b]["vehicle"] == v)
        s, n, k = fit_scale(vids, per_run)
        out[v] = dict(meas_scale=round(s, 6), _fit_runs=len(vids), _fit_runs_gnss=n,
                      _samples=k, _fit_ids=vids)
    return out


if __name__ == "__main__":
    main()
