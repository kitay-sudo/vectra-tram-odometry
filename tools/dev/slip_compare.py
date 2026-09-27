#!/usr/bin/env python3
"""До / после по двум прогонам tools/eval.py (правки срыва): инъекции по видам и
чистые метрики. Таблицы - Markdown.

  python3 tools/dev/slip_compare.py out/eval_before15 out/eval_after15 [--runs a,b,c]

--runs - только эти прогоны инъекций (например, три прогона по умолчанию
tools/eval.py: 30618_3e9f4952,30639_d3c43d69,30618_e9a34502).
"""

import argparse
import json
from pathlib import Path

import numpy as np


def f(x, n=3):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "-"
    return f"{x:.{n}f}".replace(".", ",")


def pct(x):
    return "-" if x is None or not np.isfinite(x) else f"{100 * x:.1f} %".replace(".", ",")


def load(d):
    d = Path(d)
    return (json.loads((d / "summary.json").read_text(encoding="utf-8")),
            json.loads((d / "inject.json").read_text(encoding="utf-8")))


def kinds_table(inj, runs=None):
    rows = {}
    for r in inj:
        if r.get("skipped") or (runs and r["bag"] not in runs):
            continue
        rows.setdefault(r["kind"], []).append(r)
    out = {}
    for k, rs in rows.items():
        def m(est, w, key):
            xs = [r["est"][est][w].get(key) for r in rs if est in r["est"]]
            xs = [x for x in xs if x is not None and np.isfinite(x)]
            return float(np.mean(xs)) if xs else float("nan")
        rec = [r["est"]["model"]["recovery_s"] for r in rs]
        recf = [x for x in rec if x is not None]
        tail = [abs(r["est"]["model"]["d_along_tail"]) for r in rs
                if r["est"]["model"].get("d_along_tail") is not None]
        tail_n = [abs(r["est"]["naive"]["d_along_tail"]) for r in rs
                  if r["est"]["naive"].get("d_along_tail") is not None]
        d = [r["est"]["model"]["during"] for r in rs]
        out[k] = dict(
            n=len(rs), mae=m("model", "during", "v_mae"), mae_n=m("naive", "during", "v_mae"),
            after=m("model", "after", "v_mae"), after_n=m("naive", "after", "v_mae"),
            valid=min(x.get("frac_valid", np.nan) for x in d),
            slip=float(np.mean([x.get("frac_slip", np.nan) for x in d])),
            amb=max(x.get("frac_amb", np.nan) for x in d),
            cov=float(np.nanmean([x.get("cov2s", np.nan) for x in d])),
            rec_med=float(np.median(recf)) if recf else float("nan"),
            rec_max=float(max(recf)) if recf else float("nan"), rec_none=sum(x is None for x in rec),
            tail=max(tail) if tail else float("nan"), tail_n=max(tail_n) if tail_n else float("nan"),
            crash=sum(1 for r in rs if r["est"]["model"].get("crash")))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("before")
    ap.add_argument("after")
    ap.add_argument("--runs", default="")
    a = ap.parse_args()
    runs = [x for x in a.runs.split(",") if x] or None
    sb, ib = load(a.before)
    sa, ia = load(a.after)
    tb, ta = kinds_table(ib, runs), kinds_table(ia, runs)
    print("| вид | прогонов | MAE модели во время, до → после | MAE базы | MAE после окна, модель до → после (база) | "
          "slip во время, до → после | valid мин., до → после | ±2σ во время, до → после | "
          "восст. медиана / макс, с, до → после | \\|Δ вдоль\\| через 300 с макс, м, до → после (база) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for k in ta:
        b, x = tb.get(k), ta[k]
        if b is None:
            continue
        print(f"| {k} | {x['n']} | {f(b['mae'])} → **{f(x['mae'])}** | {f(x['mae_n'])} | "
              f"{f(b['after'])} → {f(x['after'])} ({f(x['after_n'])}) | {pct(b['slip'])} → {pct(x['slip'])} | "
              f"{pct(b['valid'])} → {pct(x['valid'])} | {pct(b['cov'])} → {pct(x['cov'])} | "
              f"{f(b['rec_med'], 1)} / {f(b['rec_max'], 1)} → {f(x['rec_med'], 1)} / {f(x['rec_max'], 1)} | "
              f"{f(b['tail'], 1)} → {f(x['tail'], 1)} ({f(x['tail_n'], 1)}) |")
    print()
    tb_, ta_ = sb["totals"]["all"]["model"], sa["totals"]["all"]["model"]
    nb = sa["totals"]["all"]["naive"]
    keys = [("v_mae", "скорость MAE, м/с", 4), ("v_rmse", "скорость RMSE, м/с", 4), ("v_bias", "смещение, м/с", 4),
            ("cov2s_v", "±2σ скорости", None), ("p3d_mean", "3D ср., м", 3), ("p3d_end_mean", "конец 3D ср., м", 2),
            ("along_rmse", "вдоль RMSE, м", 2), ("false_ss_rate", "ложные стоянки", 5)]
    print("| чистые (15 отложенных) | до | после | изменение | база «только колесо» |")
    print("|---|---|---|---|---|")
    for key, name, n in keys:
        b, x = tb_.get(key), ta_.get(key)
        if b is None or x is None:
            continue
        if n is None:
            print(f"| {name} | {pct(b)} | {pct(x)} | {100 * (x - b):+.2f} п. п. | - |")
        else:
            ch = f"{100 * (x - b) / abs(b):+.2f} %" if b else "-"
            print(f"| {name} | {f(b, n)} | {f(x, n)} | {ch} | {f(nb.get(key), n)} |")
    ph_b = sb.get("pooled", {}).get("model", {}).get("by_phase", {})
    ph_a = sa.get("pooled", {}).get("model", {}).get("by_phase", {})
    if ph_b and ph_a:
        print()
        print("| фаза | MAE до → после | смещение до → после | ±2σ до → после |")
        print("|---|---|---|---|")
        for k in ph_a:
            b, x = ph_b.get(k, {}), ph_a[k]
            print(f"| {k} | {f(b.get('mae'), 4)} → {f(x.get('mae'), 4)} | {f(b.get('bias'), 4)} → "
                  f"{f(x.get('bias'), 4)} | {pct(b.get('cov2s', float('nan')))} → {pct(x.get('cov2s', float('nan')))} |")


if __name__ == "__main__":
    main()
