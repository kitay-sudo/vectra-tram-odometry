"""Оценка листов параметров на фиксированном разбиении tools/split.json.

Метрики те же, что в tools/core_metrics.py: прогон в порядке
записи через Runner, GNSS в связку только первые 3 с, пары «выход - эталон»
по ближайшей метке <= 0,05 с, фазы по ручке и скорости GNSS, причинная база
«только колесо» на той же карте и с теми же привязками. Средние взвешены
числом пар скорости.

Режимы (из корня репозитория, в образе vectra/tram:dev):

  # полные метрики (скорость, фазы, ±2σ, положение, база) - для отчёта
  python3 analysis/calib_eval.py full  --sheet <лист> --ids holdout_scored \
      --map analysis/cache/track_map_train.npz --tag <имя>
  # только скорость, без карты и базы - быстро, для подбора на обучающих
  python3 analysis/calib_eval.py speed --sheet <лист> --ids train_unique

<лист> - tram_calibration.json (ключ "params") или tram.yaml (ROS-лист);
поля, которых нет в Params (параметры ноды), отбрасываются. --set k=v
подменяет поле (значение в JSON: --set c_creep=0 --set q_v=0.3).
--ids: имя списка из tools/split.json (train, train_unique, holdout,
holdout_scored), all_unique (train_unique + holdout) или id через запятую.

Результат печатается и пишется в out/calib/<режим>_<tag>.json.
"""

import argparse
import json
import os
import sys
import time
from dataclasses import fields, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
SPLIT = ROOT / "tools" / "split.json"
OUT = ROOT / "out" / "calib"
for p in (ROOT / "analysis", PKG, ROOT / "tools"):
    sys.path.insert(0, str(p))

import bagio                                                    # noqa: E402
import core_metrics as CM                                       # noqa: E402
from tram_state_estimator.estimator_core import Params          # noqa: E402
from tram_state_estimator.runner import Runner                  # noqa: E402

PHASES = CM.PHASES
_P = None           # лист текущего прогона (наследуется дочерними процессами)


# ------------------------------------------------------------ входные данные

def split_ids(name):
    d = json.loads(SPLIT.read_text(encoding="utf-8"))
    if name in d and isinstance(d[name], list):
        return list(d[name])
    if name == "all_unique":
        return list(d["train_unique"]) + list(d["holdout"])
    return [x for x in name.split(",") if x]


def parse_sets(items):
    out = {}
    for s in items or ():
        k, v = s.split("=", 1)
        try:
            out[k] = json.loads(v)
        except json.JSONDecodeError:
            out[k] = v
    return out


def load_sheet(path, overrides=None):
    """Params из calibration json или ROS-листа yaml (+ подмены)."""
    path = Path(path)
    if path.suffix == ".json":
        d = json.loads(path.read_text(encoding="utf-8"))["params"]
    else:
        import yaml
        doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        d = next(iter(doc.values()))["ros__parameters"]
    names = {f.name for f in fields(Params)}
    d = {k: v for k, v in d.items() if k in names}
    p = Params.from_dict(d)
    if overrides:
        p = replace(p, **{k: (tuple(v) if isinstance(v, list) else v)
                          for k, v in overrides.items()})
        p = Params.from_dict({f.name: getattr(p, f.name) for f in fields(Params)})
    return p


# ------------------------------------------------------------ только скорость

def phase_of(a, tg, vg):
    n_h = np.nan_to_num(CM.zoh(a["cmd"][:, 1], a["cmd"][:, 2], tg), nan=0.0)
    return np.where(vg < CM.V_STAND_GNSS, 0,
                    np.where(n_h > 0, 1, np.where(n_h < 0, 3, 2))).astype(np.int8)


def _pairs(T, g):
    """Ближайший выход (<= 0,05 с) к каждой метке эталона g (строки
    [t_bag, t_hdr, vx, vy, ...]); индексы выхода, модуль скорости, метки."""
    if len(g) < 50:
        return None
    j, ok = CM.E.nearest(T, g[:, 1])
    return j[ok], np.hypot(g[ok, 2], g[ok, 3]), g[ok, 1]


def run_speed(b):
    """Скорость модели и причинной базы «только колесо» против |v| GNSS
    master (основной эталон) и rover (второй эталон); без карты: карта на
    скорость не влияет (положение не возвращается в ядро)."""
    p = _P
    a = bagio.load(b)
    if len(a["mvel"]) < 50 or len(a["mfix"]) < 50:
        return b, None
    r = Runner(p, track_map=None)
    nv = CM.NaiveRunner(p, None)
    outs, nouts = CM.replay(a, [r, nv])
    if len(outs) < 10:
        return b, None
    T = CM.arr(outs, "stamp")
    V = CM.arr(outs, "v")
    Vn = CM.arr(nouts, "v")
    SV = CM.arr(outs, "sigma_v")
    jj, vg, tg = _pairs(T, a["mvel"])
    s = dict(ev=(V[jj] - vg).astype(np.float32), en=(Vn[jj] - vg).astype(np.float32),
             sv=SV[jj].astype(np.float32), vg=vg.astype(np.float32),
             v=V[jj].astype(np.float32), phase=phase_of(a, tg, vg))
    for key in ("sigma_v_filt", "a", "mode", "k_t", "k_b", "valid"):
        if key in outs[0]:
            s[key] = CM.arr(outs, key)[jj].astype(np.float32)
    rv = _pairs(T, a.get("rvel", np.zeros((0, 4))))
    if rv is not None:
        jr, vr, tr = rv
        s["ev_r"] = (V[jr] - vr).astype(np.float32)
        s["en_r"] = (Vn[jr] - vr).astype(np.float32)
        s["phase_r"] = phase_of(a, tr, vr)
    core = r.core
    info = dict(bag=b, pairs=int(len(jj)), k_t_end=float(core.x[3]),
                k_b_end=float(core.x[4]))
    if hasattr(core, "adapt_stats"):
        info["adapt"] = dict(core.adapt_stats)
    return b, dict(s=s, info=info)


def vstats(e):
    e = np.asarray(e, float)
    if not len(e):
        return None
    return dict(n=int(len(e)), rmse=float(np.sqrt(np.mean(e ** 2))),
                mae=float(np.mean(np.abs(e))), bias=float(np.mean(e)))


def _cat(rs, key):
    xs = [R["s"][key] for R in rs if key in R["s"]]
    return np.concatenate(xs).astype(float) if xs else np.zeros(0)


def speed_summary(res, ids):
    rs = [res[b] for b in ids if res.get(b) is not None]
    ev, en, sv = _cat(rs, "ev"), _cat(rs, "en"), _cat(rs, "sv")
    ph = np.concatenate([R["s"]["phase"] for R in rs])
    out = dict(runs=len(rs), all=vstats(ev), naive=vstats(en),
               cov1s=float(np.mean(np.abs(ev) <= sv)),
               cov2s=float(np.mean(np.abs(ev) <= 2 * sv)),
               sigma_v_median=float(np.median(sv)), by_phase={})
    for k, name in enumerate(PHASES):
        m = ph == k
        if m.any():
            out["by_phase"][name] = dict(share=float(m.mean()), **vstats(ev[m]),
                                         cov2s=float(np.mean(np.abs(ev[m]) <= 2 * sv[m])),
                                         naive=vstats(en[m]))
    evr, enr = _cat(rs, "ev_r"), _cat(rs, "en_r")
    if len(evr):
        phr = np.concatenate([R["s"]["phase_r"] for R in rs if "phase_r" in R["s"]])
        out["rover"] = dict(model=vstats(evr), naive=vstats(enr), by_phase={
            name: dict(model=vstats(evr[phr == k]), naive=vstats(enr[phr == k]))
            for k, name in enumerate(PHASES) if (phr == k).any()})
    out["per_vehicle"] = {}
    for veh in ("30618", "30639"):
        sel = [R for R in rs if R["info"]["bag"].startswith(veh)]
        if sel:
            out["per_vehicle"][veh] = dict(model=vstats(_cat(sel, "ev")),
                                           naive=vstats(_cat(sel, "en")))
    ad = [R["info"]["adapt"] for R in rs if "adapt" in R["info"]]
    if ad:
        tot = {k: sum(d[k] for d in ad) for k in ad[0]}
        nk = max(tot["force_ok"], 1)
        km = tot["k_sum"] / nk
        out["adapt"] = dict(windows=tot["windows"], force_ok=tot["force_ok"],
                            gate_ok=tot["gate_ok"],
                            gate_rate=tot["gate_ok"] / max(tot["force_ok"], 1),
                            k_meas_mean=km,
                            k_meas_sd=float(np.sqrt(max(tot["k_sq"] / nk - km * km, 0.0))),
                            window_mean_s=tot["win_sum"] / max(tot["windows"], 1))
    return out


def fmt_speed(tag, s):
    a = s["all"]
    ph = " ".join(f"{n[:5]} {v['mae']:.4f}/{v['bias']:+.4f}/{v['cov2s']:.2f}"
                  for n, v in s["by_phase"].items())
    txt = (f"[{tag}] {s['runs']} прогонов: RMSE {a['rmse']:.4f} MAE {a['mae']:.4f} "
           f"bias {a['bias']:+.4f} ±2σ {s['cov2s']:.3f} | {ph} | база MAE "
           f"{s['naive']['mae']:.4f}")
    if "rover" in s:
        txt += f" | rover: MAE {s['rover']['model']['mae']:.4f} (база {s['rover']['naive']['mae']:.4f})"
    if "adapt" in s:
        d = s["adapt"]
        txt += (f" | адаптация: окон {d['windows']}, сила {d['force_ok']}, ворота "
                f"{d['gate_ok']} ({100 * d['gate_rate']:.0f} %), k_изм {d['k_meas_mean']:.2f}"
                f"±{d['k_meas_sd']:.2f}, окно {d['window_mean_s']:.2f} с")
    return txt


# ------------------------------------------------------------ полные метрики

def full_summary(res, ids):
    """Сводка для таблицы: из агрегата core_metrics плюс взвешенные по парам
    скорости средние по прогонам."""
    agg = CM.aggregate(res, ids)
    done = [b for b in ids if res.get(b) is not None]
    rows = [CM.per_run_row(b, res[b]) for b in done]
    w = np.array([r["v_pairs"] for r in rows], float)

    def wavg(k):
        return float(np.average([r[k] for r in rows], weights=w))
    sp, pos = agg["speed"], agg["position"]
    phases = {}
    for name, v in sp["by_phase"].items():
        phases[name] = dict(share=v["share"], mae=v["model"]["mae"],
                            bias=v["model"]["bias"], rmse=v["model"]["rmse"],
                            cov2s=v["cov2s"], naive_mae=v["naive_causal"]["mae"],
                            naive_bias=v["naive_causal"]["bias"])
    summ = dict(
        runs=len(done),
        model=dict(rmse=sp["model"]["rmse"], mae=sp["model"]["mae"],
                   bias=sp["model"]["bias"], cov1s=sp["cov1s"], cov2s=sp["cov2s"],
                   sigma_v_median=sp["sigma_v_median"],
                   p3d_mean_w=wavg("p3d_mean"), end3d_median=pos["per_run"]["end3d_median"],
                   end3d_mean=pos["per_run"]["end3d_mean"],
                   drift_pct_along_median=pos["per_run"]["drift_pct_along_median"],
                   along_rmse=pos["model"]["along"]["rmse"],
                   cov2s_along=pos["cov2s_along"], sigma_s_median=pos["sigma_s_median"],
                   false_ss=sp["false_standstill"]["rate_vg_gt_0_5"]),
        naive=dict(rmse=sp["naive_causal"]["rmse"], mae=sp["naive_causal"]["mae"],
                   bias=sp["naive_causal"]["bias"], p3d_mean_w=wavg("naive_p3d_mean"),
                   end3d_median=pos["per_run"]["naive_end3d_median"],
                   end3d_mean=float(np.mean([r["naive_p3d_end"] for r in rows])),
                   along_rmse=pos["naive"]["along"]["rmse"]),
        by_phase=phases,
    )
    return summ, agg, rows


def fmt_full(tag, s):
    m, n = s["model"], s["naive"]
    ph = " ".join(f"{k[:5]} {v['mae']:.4f}/{v['bias']:+.4f}/{v['cov2s']:.2f}"
                  for k, v in s["by_phase"].items())
    return (f"[{tag}] {s['runs']} прогонов: v RMSE {m['rmse']:.4f} MAE {m['mae']:.4f} "
            f"bias {m['bias']:+.4f} ±2σ {m['cov2s']:.3f} | 3D ср. {m['p3d_mean_w']:.2f} "
            f"конец мед. {m['end3d_median']:.2f} ср. {m['end3d_mean']:.2f} | along ±2σ "
            f"{m['cov2s_along']:.3f} | база: MAE {n['mae']:.4f} bias {n['bias']:+.4f} "
            f"3D {n['p3d_mean_w']:.2f} конец мед. {n['end3d_median']:.2f}\n    фазы (MAE/bias/±2σ): {ph}")


# ------------------------------------------------------------ main

def speed_eval(params, ids, workers):
    """Прогон «только скорость» по списку прогонов с данным листом.
    Возвращает (результаты по прогонам, сводка)."""
    global _P
    from concurrent.futures import ProcessPoolExecutor
    _P = params                     # дочерние процессы наследуют лист (fork)
    with ProcessPoolExecutor(workers) as ex:
        res = dict(ex.map(run_speed, ids))
    return res, speed_summary(res, ids)


def save_samples(path, res, ids):
    done = [b for b in ids if res.get(b) is not None]
    keys = [k for k in res[done[0]]["s"] if not k.endswith("_r")]
    np.savez_compressed(path, bag=np.concatenate([[b] * len(res[b]["s"]["ev"]) for b in done]),
                        **{k: np.concatenate([res[b]["s"][k] for b in done]) for k in keys})


def main():
    global _P
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("full", "speed", "inject"))
    ap.add_argument("--kinds", default="freeze,zero,drop",
                    help="inject: отказ обеих тележек 20 с (core_metrics.inject)")
    ap.add_argument("--dur", type=float, default=20.0)
    ap.add_argument("--sheet", required=True)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--ids", default="holdout_scored")
    ap.add_argument("--map", default=str(ROOT / "analysis" / "cache" / "track_map_train.npz"))
    ap.add_argument("--tag", default=None)
    ap.add_argument("--save-samples", action="store_true",
                    help="speed: сохранить пары в out/calib/samples_<tag>.npz")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    over = parse_sets(args.set)
    _P = load_sheet(ROOT / args.sheet if not os.path.isabs(args.sheet) else args.sheet, over)
    ids = split_ids(args.ids)
    tag = args.tag or f"{Path(args.sheet).stem}_{args.ids if ',' not in args.ids else 'custom'}"
    OUT.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    from concurrent.futures import ProcessPoolExecutor
    meta = dict(sheet=args.sheet, overrides=over, ids=args.ids, n_ids=len(ids),
                map=args.map if args.mode == "full" else None)
    if args.mode == "speed":
        res, s = speed_eval(_P, ids, args.workers)
        s["runs_info"] = [res[b]["info"] for b in ids if res.get(b) is not None]
        print(fmt_speed(tag, s), f"({time.time() - t0:.0f} с)", flush=True)
        (OUT / f"speed_{tag}.json").write_text(
            json.dumps(dict(meta=meta, summary=s), ensure_ascii=False, indent=1),
            encoding="utf-8")
        if args.save_samples:
            save_samples(OUT / f"samples_{tag}.npz", res, ids)
        return
    CM.MAPS["calib"] = Path(args.map) if args.map else None
    CM.load_params = lambda cfg: _P
    if args.mode == "inject":
        jobs = [(b, k, args.dur, "calib", "calib") for b in ids
                for k in args.kinds.split(",")]
        with ProcessPoolExecutor(args.workers) as ex:
            out = list(ex.map(CM._inj_one, jobs))
        rep = {f"{b}|{k}": r for b, k, r in out}
        for key, r in rep.items():
            if r is None:
                print(key, "нет подходящего окна")
                continue
            c, f = r["clean"], r[key.split("|")[1]]
            print(f"{key}: along к концу окна чисто {c['along_at_end_of_window']:+.1f} / "
                  f"отказ {f['along_at_end_of_window']:+.1f} м (база "
                  f"{f['naive_along_at_end_of_window']:+.1f}); +60 с {f['along_60s_after']:+.1f}; "
                  f"конец {c['along_end']:+.1f} / {f['along_end']:+.1f} м; v MAE в окне "
                  f"{f['v_mae_window']:.2f} (база {f['naive_v_mae_window']:.2f}); valid в окне "
                  f"{f['frac_valid_window']:.2f}, amb {f['frac_amb_window']:.2f}; σv/σs в конце "
                  f"окна {f['sigma_v_end_window']:.2f} / {f['sigma_s_end_window']:.1f}", flush=True)
        (OUT / f"inject_{tag}.json").write_text(
            json.dumps(dict(meta=meta, runs=rep), ensure_ascii=False, indent=1), encoding="utf-8")
        return
    with ProcessPoolExecutor(args.workers) as ex:
        res = dict(ex.map(CM.run_one, [(b, "calib", "calib") for b in ids]))
    summ, agg, rows = full_summary(res, ids)
    print(fmt_full(tag, summ), f"({time.time() - t0:.0f} с)", flush=True)
    (OUT / f"full_{tag}.json").write_text(
        json.dumps(dict(meta=meta, summary=summ, per_run=rows), ensure_ascii=False,
                   indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
