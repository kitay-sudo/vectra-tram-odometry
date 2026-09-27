"""Таблица «заглушки против подогнанных» на holdout_scored.

Собирает готовые прогоны calib_eval.py (out/calib/speed_<tag>.json и
full_<tag>.json) в одну таблицу и пишет analysis/calib_holdout_table.json.

    python3 analysis/calib_table.py a_orig_yaml:"(a) исходный tram.yaml" \\
        b_orig_json:"(b) tram_calibration.json" c_eval:"(c) лист оценки"

Строка «база» - причинная база «только колесо» (та же карта, выставка и
привязки) из последнего переданного прогона.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out" / "calib"
PH = ("standstill", "traction", "coast", "brake")
PH_RU = {"standstill": "стоянка", "traction": "тяга", "coast": "выбег", "brake": "торможение"}


def load(tag):
    sp = json.loads((OUT / f"speed_{tag}.json").read_text(encoding="utf-8"))
    fu = json.loads((OUT / f"full_{tag}.json").read_text(encoding="utf-8"))
    return sp, fu


def row(label, sp, fu, naive=False):
    s, f = sp["summary"], fu["summary"]
    if naive:
        m = s["naive"]
        r = dict(label=label, runs=s["runs"], rmse=m["rmse"], mae=m["mae"], bias=m["bias"],
                 cov2=None, phase={k: dict(mae=v["naive"]["mae"], bias=v["naive"]["bias"],
                                          cov2=None) for k, v in s["by_phase"].items()},
                 p3d=f["naive"]["p3d_mean_w"], end_med=f["naive"]["end3d_median"],
                 end_mean=f["naive"]["end3d_mean"], cov2_along=None,
                 rover_mae=s.get("rover", {}).get("naive", {}).get("mae"),
                 rover_bias=s.get("rover", {}).get("naive", {}).get("bias"))
        return r
    m = s["all"]
    return dict(label=label, runs=s["runs"], rmse=m["rmse"], mae=m["mae"], bias=m["bias"],
                cov2=s["cov2s"], phase={k: dict(mae=v["mae"], bias=v["bias"], cov2=v["cov2s"])
                                        for k, v in s["by_phase"].items()},
                p3d=f["model"]["p3d_mean_w"], end_med=f["model"]["end3d_median"],
                end_mean=f["model"]["end3d_mean"], cov2_along=f["model"]["cov2s_along"],
                rover_mae=s.get("rover", {}).get("model", {}).get("mae"),
                rover_bias=s.get("rover", {}).get("model", {}).get("bias"))


def fmt(x, nd=4, sign=False):
    if x is None:
        return "-"
    return f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"


def main():
    pairs = [a.split(":", 1) for a in sys.argv[1:]]
    rows, last = [], None
    for tag, label in pairs:
        sp, fu = load(tag)
        rows.append(row(label, sp, fu))
        last = (sp, fu)
    rows.append(row("база «только колесо» (причинная)", *last, naive=True))
    print("| Вариант | RMSE | MAE | смещение | ±2σ | ср. 3D, м | конец 3D, медиана / ср., м |"
          " along ±2σ_s | rover: MAE / смещение |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['label']} | {fmt(r['rmse'])} | {fmt(r['mae'])} | {fmt(r['bias'], sign=True)} | "
              f"{fmt(r['cov2'] and 100 * r['cov2'], 1)} | {fmt(r['p3d'], 2)} | "
              f"{fmt(r['end_med'], 2)} / {fmt(r['end_mean'], 1)} | "
              f"{fmt(r['cov2_along'] and 100 * r['cov2_along'], 1)} | "
              f"{fmt(r['rover_mae'])} / {fmt(r['rover_bias'], sign=True)} |")
    print()
    print("| Вариант | " + " | ".join(f"{PH_RU[k]}: MAE / смещение / ±2σ" for k in PH) + " |")
    print("|---|" + "---|" * len(PH))
    for r in rows:
        cells = []
        for k in PH:
            v = r["phase"].get(k)
            cells.append("-" if v is None else
                         f"{fmt(v['mae'])} / {fmt(v['bias'], sign=True)} / "
                         f"{fmt(v['cov2'] and 100 * v['cov2'], 0)}")
        print(f"| {r['label']} | " + " | ".join(cells) + " |")
    out = ROOT / "analysis" / "calib_holdout_table.json"
    out.write_text(json.dumps(dict(ids="holdout_scored", rows=rows, sources=[t for t, _ in pairs]),
                              ensure_ascii=False, indent=1), encoding="utf-8")
    print("\nзаписано:", out)


if __name__ == "__main__":
    main()
