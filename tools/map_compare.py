#!/usr/bin/env python3
"""Сравнение карт путей для следования по карте.

Три варианта карты (все — по точке base_link, только обучающие прогоны
tools/split.json:train для EVAL-набора):
  gnss       наша карта из GNSS (analysis/build_map.py --source gnss);
  pathgraph  только pathgraph организаторов (за его концами карты нет:
             курсор держится у известной конечной или идёт по прямой);
  hybrid     pathgraph, где он есть, плюс наша карта за его пределами.
Остановки и конечные у всех трёх — из GNSS тех же прогонов, множитель пути —
калибровка по тем же прогонам.

    python3 tools/map_compare.py build [--set train]          # out/maps/cmp_<set>_<вариант>.npz
    python3 tools/map_compare.py run --ids train [--maps gnss,pathgraph,hybrid]
    python3 tools/map_compare.py table --ids train             # таблица по out/mapcmp/*
    python3 tools/map_compare.py run --ids holdout_scored --maps hybrid   # один раз после выбора

Выбор — на train по средней 3D и ошибке в конце (base_link, MGRS 37UCB);
holdout считается один раз для выбранного варианта. Прогоны без GNSS
пропускаются (эталона нет). Прогон — tools/eval.py (--no-inject --no-gnss-full
--no-doc) с лист ОЦЕНКИ; pathgraph для метрик — _incoming/pathgraph.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))
import bagio                        # noqa: E402

SPLIT = ROOT / "tools" / "split.json"
MAPS = ("gnss", "pathgraph", "hybrid")
OUT = ROOT / "out" / "mapcmp"
MAPDIR = ROOT / "out" / "maps"


def ids_of(key):
    sp = json.loads(SPLIT.read_text(encoding="utf-8"))
    ids = list(sp[key]) if key in sp else key.split(",")
    return [b for b in ids if (bagio.CACHE / f"{b}.npz").exists()
            and len(bagio.load(b)["mfix"]) >= 100]


def map_file(which, set_):
    return MAPDIR / f"cmp_{set_}_{which}.npz"


def cmd_build(a):
    MAPDIR.mkdir(parents=True, exist_ok=True)
    for which in a.maps.split(","):
        out = map_file(which, a.set)
        c = [sys.executable, str(ROOT / "analysis" / "build_map.py"), "--set", a.set,
             "--source", which, "--out", str(out)]
        if a.calib:
            c += ["--calib", a.calib]
        print(" ".join(c), flush=True)
        subprocess.run(c, check=True)


def cmd_run(a):
    ids = ids_of(a.ids)
    print(f"прогонов с GNSS: {len(ids)}", flush=True)
    for which in a.maps.split(","):
        f = Path(a.map_file) if a.map_file else map_file(which, a.set)
        out = OUT / f"{a.ids}_{which}{a.tag}"
        c = [sys.executable, str(ROOT / "tools" / "eval.py"), "--runs", ",".join(ids),
             "--map", str(f), "--no-inject", "--no-gnss-full", "--no-doc", "--out",
             str(out.relative_to(ROOT)), "--label", f"map_compare {which} {a.ids}"]
        if a.workers:
            c += ["--workers", str(a.workers)]
        if a.extra:
            c += a.extra.split()
        print(" ".join(c[:3] + ["…"] + c[5:]), flush=True)
        subprocess.run(c, check=True)


ROWS = (("p3d_mean", "ср. 3D, м", 2), ("p3d_end_mean", "конец ср., м", 2),
        ("p3d_end_median", "конец мед., м", 2), ("p3d_end_max", "конец макс., м", 1),
        ("along_rmse", "вдоль RMSE, м", 2), ("cross_mean", "поперёк ср., м", 2),
        ("pz_mean", "|z| ср., м", 2), ("p3d_on_pg", "3D на pathgraph, м", 2),
        ("p3d_off_pg", "3D за концами pathgraph, м", 2), ("p_pairs_off_pg", "пар за концами", 0),
        ("pg_cross_mean", "поперёк pathgraph ср., м", 3), ("pg_cross_p95", "поперёк pathgraph 95 %, м", 2),
        ("pg_along_mean", "вдоль pathgraph ср., м", 2), ("p_gap_steps", "шагов без положения", 0),
        ("cov2s_along", "|вдоль| ≤ 2σ", 3), ("runs", "прогонов", 0))


def load_tot(path):
    f = path / "summary.json"
    if not f.exists():
        return None
    return json.loads(f.read_text(encoding="utf-8"))["totals"]["all"]["model"]


def cmd_table(a):
    cols = [(w, load_tot(OUT / f"{a.ids}_{w}{a.tag}")) for w in a.maps.split(",")]
    cols = [(w, t) for w, t in cols if t]
    print(f"| {a.ids} | " + " | ".join(w for w, _ in cols) + " |")
    print("|---|" + "---:|" * len(cols))
    for k, name, nd in ROWS:
        vals = []
        for _, t in cols:
            v = t.get(k)
            vals.append("—" if v is None else (f"{v:.{nd}f}" if nd else f"{int(round(v))}"))
        print(f"| {name} | " + " | ".join(vals) + " |")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("build", "run", "table"))
    ap.add_argument("--set", default="train", help="набор прогонов карты (build_map --set)")
    ap.add_argument("--ids", default="train", help="прогоны оценки: ключ split.json или список")
    ap.add_argument("--maps", default=",".join(MAPS))
    ap.add_argument("--map-file", default="", help="run: своя карта вместо cmp_<set>_<вариант>")
    ap.add_argument("--calib", default="")
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--tag", default="", help="суффикс каталога результатов")
    ap.add_argument("--extra", default="", help="доп. ключи tools/eval.py (строкой)")
    a = ap.parse_args()
    {"build": cmd_build, "run": cmd_run, "table": cmd_table}[a.cmd](a)


if __name__ == "__main__":
    main()
