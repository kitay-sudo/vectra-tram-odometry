#!/usr/bin/env python3
"""robust_eval — офлайн-проверка правок устойчивости связки Runner на реальных
прогонах (WP3, WP4, WP6, WP23; docs/ROBUST.md).

Запуск из корня репозитория в образе vectra/tram:dev (numpy есть только там):

    git show main:ros2_ws/src/tram_state_estimator/tram_state_estimator/runner.py \
        > out/robust/runner_main.py                      # связка до правок (на хосте)
    python3 tools/robust_eval.py compare --set holdout --workers 2
    python3 tools/robust_eval.py compare --set train --variants main,same --sheets json

Варианты связки:
  main    runner.py из ветки main (до правок) — out/robust/runner_main.py;
  same    новая связка с сеткой как в main (t += dt от первой метки) и без
          приведения: проверка, что защита входов и времени (WP3, WP4) на
          реальных данных ничего не меняет;
  wp23    новая связка, узлы сетки кратны dt, целый счётчик (WP23), без
          приведения;
  wp23_6  новая связка по умолчанию: WP23 + приведение показаний к шагу (WP6).
Листы: json — config/tram_calibration.json (как analysis/evaluate.py);
json_nocreep — он же с c_creep = c_creep_drag = 0 (правка WP5 потока calib).
Карта: analysis/cache/track_map_train.npz (только обучающие прогоны). GNSS —
первые 3 с (analysis/evaluate.events), эталон — |v| GNSS master (и rover).
Набор holdout — 15 чистых отложенных (tools/split.json: holdout_scored),
средние взвешены по числу пар скорости.

Результат: out/robust/<set>_<sheets>.json и таблицы в stdout.
"""

import argparse
import json
import math
import sys
import time
import types
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
sys.path.insert(0, str(ROOT / "analysis"))
sys.path.insert(0, str(PKG))

import bagio                                                     # noqa: E402
import evaluate as E                                             # noqa: E402
from tram_state_estimator.runner import Enu, Runner              # noqa: E402
from tram_state_estimator.track_map import TrackMap              # noqa: E402

OUT = ROOT / "out" / "robust"
MAP = ROOT / "analysis" / "cache" / "track_map_train.npz"
MAIN_SRC = OUT / "runner_main.py"
PHASES = ("standstill", "traction", "coast", "brake")
V_STAND = 0.2
SHIFTS = np.round(np.arange(-0.2, 0.2001, 0.005), 3)


def params(sheet):
    p = E.tram_params()
    if sheet.endswith("nocreep"):
        p = replace(p, c_creep=0.0, c_creep_drag=0.0)
    return p


_MAIN = None


def main_runner():
    """Runner из ветки main: исходник исполняется как модуль пакета."""
    global _MAIN
    if _MAIN is None:
        mod = types.ModuleType("tram_state_estimator.runner_main")
        mod.__package__ = "tram_state_estimator"
        mod.__file__ = str(MAIN_SRC)
        sys.modules[mod.__name__] = mod
        exec(compile(MAIN_SRC.read_text(encoding="utf-8"), str(MAIN_SRC), "exec"),
             mod.__dict__)
        _MAIN = mod.Runner
    return _MAIN


class LegacyGrid(Runner):
    """Новая связка (защита входов и времени), но сетка как в main:
    t += dt от первой метки. Отличие от main — только в отброшенном."""
    grid_align = False
    age_comp = False

    def _run_to(self, stamp):
        outs = []
        while self.t + self.p.dt <= stamp + 1e-9 and len(outs) < self.MAX_STEPS:
            self.t += self.p.dt
            self._n += 1
            outs.append(self._step_safe())
        return outs


def make(variant, p):
    tmap = TrackMap.load(MAP) if MAP.exists() else None
    if variant == "main":
        return main_runner()(p, track_map=tmap)
    if variant == "same":
        return LegacyGrid(p, track_map=tmap)
    r = Runner(p, track_map=tmap)
    r.age_comp = variant == "wp23_6"
    return r


def zoh(t_src, v_src, t_q):
    o = np.argsort(t_src, kind="stable")
    j = np.searchsorted(t_src[o], t_q, side="right") - 1
    return np.where(j >= 0, v_src[o][np.clip(j, 0, None)], 0.0)


def vel_metrics(T, V, g, cmd):
    """Ошибка скорости против |v| GNSS: пары по ближайшей метке <= 0,05 с."""
    j, ok = E.nearest(T, g[:, 1])
    vg = np.hypot(g[:, 2], g[:, 3])[ok]
    tg = g[ok, 1]
    e = V[j[ok]] - vg
    n_h = zoh(cmd[:, 1], cmd[:, 2], tg)
    ph = np.where(vg < V_STAND, 0, np.where(n_h > 0, 1, np.where(n_h < 0, 3, 2)))
    res = dict(pairs=int(ok.sum()), mae=float(np.mean(np.abs(e))),
               rmse=float(np.sqrt(np.mean(e ** 2))), bias=float(np.mean(e)),
               node_gap_ms_med=float(np.median(np.abs(T[j[ok]] - tg)) * 1e3),
               node_gap_ms_max=float(np.max(np.abs(T[j[ok]] - tg)) * 1e3),
               node_exact=float(np.mean(np.abs(T[j[ok]] - tg) < 1e-3)))
    for k, name in enumerate(PHASES):
        m = ph == k
        res[f"n_{name}"] = int(m.sum())
        res[f"bias_{name}"] = float(np.mean(e[m])) if m.any() else math.nan
        res[f"mae_{name}"] = float(np.mean(np.abs(e[m]))) if m.any() else math.nan
    # сдвиг выхода к эталону: v_out(t + s) лучше всего совпадает с v_GNSS(t)
    mv = (tg > T[0] + 5) & (tg < T[-1] - 1)
    good = np.abs(e) < 2.0
    maes = [float(np.mean(np.abs(np.interp(tg[mv & good] + s, T, V)
                                 - vg[mv & good]))) for s in SHIFTS]
    res["shift_s"] = float(SHIFTS[int(np.argmin(maes))])
    return res


def one(job):
    bag, sheet, variant = job
    t_wall = time.time()
    a = bagio.load(bag)
    p = params(sheet)
    r = make(variant, p)
    outs = []
    for tb, kind, i, th, val in E.events(a):
        if kind == 0:
            outs += r.on_wheel(i, th, val)
        elif kind == 1:
            outs += r.on_handle(th, val)
        else:
            outs += r.on_fix(th, i, *val)
    T = np.array([o["stamp"] for o in outs])
    V = np.array([o["v"] for o in outs])
    X = np.array([[o["x"], o["y"], o["z"]] for o in outs])
    res = dict(bag=bag, sheet=sheet, variant=variant, n_out=len(T),
               wall_s=time.time() - t_wall)
    for k in ("resets", "core_resets", "rejected_stamps", "rejected_values",
              "skipped_steps"):
        res[k] = int(getattr(r, k, -1))
    res["finite"] = bool(np.isfinite(V).all() and np.isfinite(X).all())
    res["vel"] = vel_metrics(T, V, a["mvel"], a["cmd"])
    if len(a["rvel"]) > 50:
        res["vel_rover"] = vel_metrics(T, V, a["rvel"], a["cmd"])
    m = a["mfix"]
    enu = Enu(m[0, 2], m[0, 3], m[0, 4])
    ref = np.array([enu.fwd(la, lo, al) for la, lo, al in m[:, 2:5]])
    j2, ok2 = E.nearest(T, m[:, 1])
    d3 = np.linalg.norm(X[j2[ok2]] - ref[ok2], axis=1)
    res["pos"] = dict(mean3d=float(d3.mean()), end3d=float(d3[-1]),
                      max3d=float(d3.max()))
    return res, (T, V, X)


def deltas(base, other):
    """Разница выходов двух вариантов: на общих метках (±1 мс), а если сетки
    сдвинуты (WP23) — линейной интерполяцией другого на метки базы."""
    Tb, Vb, Xb = base
    To, Vo, Xo = other
    j = np.clip(np.searchsorted(To, Tb), 1, len(To) - 1)
    j = np.where(np.abs(To[j - 1] - Tb) < np.abs(To[j] - Tb), j - 1, j)
    ok = np.abs(To[j] - Tb) < 1e-3
    if ok.mean() > 0.5:
        dv = np.abs(Vo[j[ok]] - Vb[ok])
        dp = np.linalg.norm(Xo[j[ok]] - Xb[ok], axis=1)
    else:
        ok = (Tb >= To[0]) & (Tb <= To[-1])
        dv = np.abs(np.interp(Tb[ok], To, Vo) - Vb[ok])
        Xi = np.stack([np.interp(Tb[ok], To, Xo[:, c]) for c in range(3)], 1)
        dp = np.linalg.norm(Xi - Xb[ok], axis=1)
    return dict(common=int(ok.sum()), n_base=len(Tb), n_other=len(To),
                dv_max=float(dv.max()) if len(dv) else math.nan,
                dv_mean=float(dv.mean()) if len(dv) else math.nan,
                n_dv_gt_1e9=int((dv > 1e-9).sum()),
                dp_max=float(dp.max()) if len(dp) else math.nan,
                dp_mean=float(dp.mean()) if len(dp) else math.nan)


def wavg(rows, key, sub="vel"):
    w = np.array([r[sub]["pairs"] for r in rows], float)
    x = np.array([r[sub][key] for r in rows], float)
    m = np.isfinite(x)
    return float(np.average(x[m], weights=w[m])) if m.any() else math.nan


def phase_avg(rows, name, what="bias"):
    w = np.array([r["vel"][f"n_{name}"] for r in rows], float)
    x = np.array([r["vel"][f"{what}_{name}"] for r in rows], float)
    m = np.isfinite(x) & (w > 0)
    return float(np.average(x[m], weights=w[m])) if m.any() else math.nan


def bag_set(name):
    sp = json.loads((ROOT / "tools" / "split.json").read_text(encoding="utf-8"))
    if name == "holdout":
        ids = sp["holdout_scored"]
    elif name == "train":
        ids = sp["train"]
    elif name == "all":
        ids = sp["train"] + sp["holdout"]
    else:
        ids = name.split(",")
    have = []
    for b in ids:
        f = bagio.CACHE / f"{b}.npz"
        if f.exists():
            z = np.load(f)
            if len(z["mvel"]) >= 50 and len(z["mfix"]) >= 50:
                have.append(b)
    return have


def cmd_compare(args):
    bags = bag_set(args.set)
    variants = args.variants.split(",")
    sheets = args.sheets.split(",")
    if "main" in variants and not MAIN_SRC.exists():
        sys.exit(f"нет {MAIN_SRC}: git show main:.../runner.py > {MAIN_SRC}")
    jobs = [(b, s, v) for s in sheets for v in variants for b in bags]
    t0 = time.time()
    res, arrs = [], {}
    with ProcessPoolExecutor(args.workers) as ex:
        for r, a in ex.map(one, jobs):
            res.append(r)
            arrs[(r["bag"], r["sheet"], r["variant"])] = a
            print(f"  {r['bag']} {r['sheet']:13s} {r['variant']:7s} MAE "
                  f"{r['vel']['mae']:.4f} 3D {r['pos']['mean3d']:.2f} "
                  f"resets {r['resets']} rej {r['rejected_stamps']}/"
                  f"{r['rejected_values']} {r['wall_s']:.0f} с", flush=True)
    print(f"прогонов {len(res)}, {time.time() - t0:.0f} с; карта "
          f"{'train' if MAP.exists() else 'НЕТ'}; набор {args.set}: {len(bags)} bag")
    summary = {}
    for s in sheets:
        print(f"\n### Лист {s} ({len(bags)} прогонов, веса — пары скорости)\n")
        print("| вариант | MAE | RMSE | смещение | разгон | выбег | торм. | стоянка "
              "| MAE rover | сдвиг, мс | узел−GNSS мед/макс, мс | узел=GNSS | ср. 3D, м "
              "| сбросы | отбр. меток/знач. |")
        print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for v in variants:
            rows = [r for r in res if r["sheet"] == s and r["variant"] == v]
            rr = [r for r in rows if "vel_rover" in r]
            w = np.array([r["vel"]["pairs"] for r in rows], float)
            d = dict(
                mae=wavg(rows, "mae"), rmse=wavg(rows, "rmse"),
                bias=wavg(rows, "bias"),
                **{f"bias_{k}": phase_avg(rows, k) for k in PHASES},
                **{f"mae_{k}": phase_avg(rows, k, "mae") for k in PHASES},
                mae_rover=wavg(rr, "mae", "vel_rover") if rr else math.nan,
                shift_ms=float(np.average([r["vel"]["shift_s"] for r in rows],
                                          weights=w)) * 1e3,
                gap_med=float(np.median([r["vel"]["node_gap_ms_med"] for r in rows])),
                gap_max=float(np.max([r["vel"]["node_gap_ms_max"] for r in rows])),
                exact=float(np.average([r["vel"]["node_exact"] for r in rows],
                                       weights=w)),
                mean3d=float(np.average([r["pos"]["mean3d"] for r in rows],
                                        weights=w)),
                resets=int(sum(max(r["resets"], 0) for r in rows)),
                core_resets=int(sum(max(r["core_resets"], 0) for r in rows)),
                rej_st=int(sum(max(r["rejected_stamps"], 0) for r in rows)),
                rej_val=int(sum(max(r["rejected_values"], 0) for r in rows)),
                finite=all(r["finite"] for r in rows))
            summary[f"{s}/{v}"] = d
            print(f"| {v} | {d['mae']:.4f} | {d['rmse']:.4f} | {d['bias']:+.4f} | "
                  f"{d['bias_traction']:+.4f} | {d['bias_coast']:+.4f} | "
                  f"{d['bias_brake']:+.4f} | {d['bias_standstill']:+.4f} | "
                  f"{d['mae_rover']:.4f} | {d['shift_ms']:+.0f} | "
                  f"{d['gap_med']:.1f}/{d['gap_max']:.1f} | {d['exact']:.1%} | "
                  f"{d['mean3d']:.2f} | "
                  f"{d['resets']}+{d['core_resets']} | {d['rej_st']}/{d['rej_val']} |")
        if "main" in variants:
            print(f"\nРазница с main (лист {s}): по прогонам, на общих метках ±1 мс\n")
            print("| вариант | общих меток | max |dv| | ср. |dv| | выходов с |dv|>1e-9 "
                  "| max |dp|, м | ср. |dp|, м |")
            print("|---|---|---|---|---|---|---|")
            for v in variants:
                if v == "main":
                    continue
                ds = [deltas(arrs[(b, s, "main")], arrs[(b, s, v)]) for b in bags]
                tot = sum(d["common"] for d in ds)
                summary[f"{s}/{v}/delta"] = dict(
                    common=tot, n_base=sum(d["n_base"] for d in ds),
                    dv_max=max(d["dv_max"] for d in ds),
                    dv_mean=float(np.average([d["dv_mean"] for d in ds],
                                             weights=[d["common"] for d in ds])),
                    n_dv=sum(d["n_dv_gt_1e9"] for d in ds),
                    dp_max=max(d["dp_max"] for d in ds),
                    dp_mean=float(np.average([d["dp_mean"] for d in ds],
                                             weights=[d["common"] for d in ds])),
                    per_bag={b: d for b, d in zip(bags, ds)})
                q = summary[f"{s}/{v}/delta"]
                print(f"| {v} | {tot} из {q['n_base']} | {q['dv_max']:.2e} | "
                      f"{q['dv_mean']:.2e} | {q['n_dv']} | {q['dp_max']:.3f} | "
                      f"{q['dp_mean']:.4f} |")
    OUT.mkdir(parents=True, exist_ok=True)
    name = args.set if "," not in args.set else f"list{len(bags)}"
    f = OUT / f"{name}_{'-'.join(sheets)}_{'-'.join(variants)}.json"
    f.write_text(json.dumps(dict(set=args.set, bags=bags, summary=summary,
                                 runs=res), ensure_ascii=False, indent=1),
                 encoding="utf-8")
    print(f"\nзаписано {f.relative_to(ROOT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["compare"])
    ap.add_argument("--set", default="holdout")
    ap.add_argument("--variants", default="main,same,wp23,wp23_6")
    ap.add_argument("--sheets", default="json,json_nocreep")
    ap.add_argument("--workers", type=int, default=2)
    args = ap.parse_args()
    cmd_compare(args)


if __name__ == "__main__":
    main()
