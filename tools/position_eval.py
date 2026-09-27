#!/usr/bin/env python3
"""position_eval — оценка положения на отложенных прогонах.

Прогон воспроизводится офлайн через Runner выбранного исходника пакета
(--pkg: текущий или выгрузка другой ветки) в порядке записи bag; GNSS —
первые 3 с по времени записи (как analysis/evaluate.py) или весь прогон.
Эталон — GNSS master, посчитанный НЕЗАВИСИМО от кода ноды (analysis/georef.py).
Пары «выход — эталон» — ближайшая метка в пределах 0,05 с; выходы без якоря
(pos_valid = False, нода их не публикует) в пары не входят. Средние по
прогонам взвешены числом пар скорости GNSS.

Сравнение идёт в НЕПРЕРЫВНЫХ координатах UTM: выход MGRS с переносом по
квадратам разворачивается к ближайшему к эталону 100-км квадрату, чтобы
несовпадение соглашения на границе не забивало метрики. Отдельно считаются
отсчёты, где выход и эталон попали в разные 100-км квадраты (риск границы
37UCB/37UDB при соглашении «каждая точка в своём квадрате»), и доля эталона
западнее E = 400 км (там соглашения «перенос» и «от 37UDB» расходятся на 100 км).

Подкоманды (в образе vectra/tram:dev, из корня репозитория):

  run    --tag T [--pkg DIR] [--map FILE|none] [--ids holdout|train|a,b]
         [--gnss window|full] [--scenario normal|norover|nomaster|cut|cut_norover]
         [--opt key=value ...] [--frame rel_equirect] [--replay] [--workers N]
         -> out/position_eval/T/<bag>.npz
  score  --tag T[,T2...]                     -> out/position_eval/T/score.json
  report --tags T1,T2,... [--runs] [--vel]   -> таблицы markdown
  diff   --tags A,B                          -> max |dv|, |dp| по прогонам
  boundary                                   -> какие прогоны пересекают E = 400 км

Сценарии: normal — GNSS первые 3 с; full — весь прогон; norover — без rover;
nomaster — без master (выставка по rover со сдвигом на базу);
cut — запись начинается на ходу (первая метка master/vel после 300 с со
скоростью > 8 м/с, всё раньше отброшено).
--frame rel_equirect — выход старого кода (относительный equirect от первой
точки master): сравнивается с эталоном в той же формуле.
"""

import argparse
import ast
import inspect
import json
import math
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))
import bagio    # noqa: E402
import georef   # noqa: E402

OUT = ROOT / "out" / "position_eval"
PKG_DEFAULT = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
CALIB = PKG_DEFAULT / "config" / "tram_calibration.json"
EVAL_MAP = PKG_DEFAULT / "config" / "eval" / "track_map.npz"
INIT_S = 3.0
TOL = 0.05

_CFG = {}


def split_ids(which):
    sp = json.loads((ROOT / "tools" / "split.json").read_text(encoding="utf-8"))
    if which == "holdout":
        return sp["holdout_scored"]
    if which == "train":
        return sp["train"]
    return which.split(",")


def cut_time(a, v_min=8.0, after=300.0):
    g = a["mvel"]
    sp = np.hypot(g[:, 2], g[:, 3])
    k = np.flatnonzero((g[:, 1] > g[0, 1] + after) & (sp > v_min))
    return float(g[k[0], 1]) if len(k) else None


def scenario_data(a, scenario):
    if scenario.startswith("cut"):
        tc = cut_time(a)
        if tc is None:
            return None
        a = {k: (v[v[:, 1] >= tc] if v.ndim == 2 and len(v) else v) for k, v in a.items()}
    return a


def events(a, gnss="window", drop_rover=False, drop_master=False):
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in a[key]:
            ev.append((tb, 0, i, th, v, None))
    for tb, th, n in a["cmd"]:
        ev.append((tb, 1, 0, th, n, None))
    t_end = (a["mfix"][0, 0] + INIT_S) if (gnss == "window" and len(a["mfix"])) else math.inf
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        if (drop_rover and ant == "rover") or (drop_master and ant == "master"):
            continue
        for row in a[key]:
            if row[0] <= t_end:
                ev.append((row[0], 2, ant, row[1], (row[2], row[3], row[4]), int(row[5])))
    ev.sort(key=lambda e: e[0])
    return ev


class _Rec:
    """Запись вызовов Position (точки GNSS и шаги сетки) для повторного
    проигрывания только положения: ядро от параметров положения не зависит."""

    def __init__(self, pos, log):
        self.__dict__["_p"], self.__dict__["_log"] = pos, log

    def __getattr__(self, k):
        return getattr(self._p, k)

    def __setattr__(self, k, v):
        setattr(self._p, k, v)

    def on_fix(self, stamp, antenna, lat, lon, alt, s=None, v=0.0, t=None, status=0):
        self._log.append(("f", stamp, antenna, lat, lon, alt, s, v, t, status))
        return self._p.on_fix(stamp, antenna, lat, lon, alt, s=s, v=v, t=t, status=status)

    def step(self, s, standing, dt, t=None, v=0.0):
        self._log.append(("s", s, standing, dt, t, v))
        return self._p.step(s, standing, dt, t=t, v=v)


def _core_file(cfg, b):
    cal = Path(cfg.get("calib") or "sheet").stem
    return OUT / "_core" / f"{cfg['scenario']}_{cfg['gnss']}_{cal}" / f"{b}.pkl"


def _run_one(b):
    cfg = _CFG
    sys.path.insert(0, cfg["pkg"])
    from tram_state_estimator.estimator_core import Params
    from tram_state_estimator.runner import Runner
    from tram_state_estimator.track_map import TrackMap
    f = OUT / cfg["tag"] / f"{b}.npz"
    a = scenario_data(bagio.load(b), cfg["scenario"])
    if a is None or len(a["mfix"]) < 100 or len(a["mvel"]) < 50:
        return b, "skip"
    calib = cfg.get("calib") or str(Path(cfg["pkg"]) / "config" / "tram_calibration.json")
    params = Params.from_dict(json.loads(Path(calib).read_text(encoding="utf-8"))["params"])
    tmap = None if cfg["map"] in ("", "none") else TrackMap.load(cfg["map"])
    opts = dict(cfg["opts"])
    import tram_state_estimator.runner as runner_mod
    for k in [k for k in opts if k.startswith("P_")]:     # атрибуты класса Position
        setattr(runner_mod.Position, k[2:], opts.pop(k))
    mscale = opts.pop("map_scale_mult", None)
    if tmap is not None and mscale is not None:
        tmap.scale *= float(mscale)
    for k in [k for k in opts if k.startswith("M_")]:     # атрибуты карты
        v = opts.pop(k)
        if tmap is not None:
            setattr(tmap, k[2:], v)
    new_pkg = hasattr(runner_mod.Position, "step")
    cf = _core_file(cfg, b)
    if new_pkg and cfg.get("replay") and cf.exists():
        # только положение: запись ядра этого прогона уже есть
        with open(cf, "rb") as fh:
            C = pickle.load(fh)
        pos = runner_mod.Position(tmap, None, **opts)
        X, valid = [], []
        k = 0
        for e in C["log"]:
            if e[0] == "f":
                pos.on_fix(*e[1:6], s=e[6], v=e[7], t=e[8], status=e[9])
            else:
                pq = pos.step(e[1], e[2], e[3], t=e[4], v=e[5])
                X.append(pq[:3] if pq is not None else (C["S"][k], 0.0, 0.0))
                valid.append(pq is not None)
                k += 1
        T, V, S = C["T"], C["V"], C["S"]
        X, valid = np.array(X), np.array(valid)
    else:
        r = Runner(params, track_map=tmap, **opts)
        log = []
        if new_pkg:
            r.pos = _Rec(r.pos, log)
        fix_status = "status" in inspect.signature(r.on_fix).parameters
        outs = []
        for tb, kind, i, th, val, st in events(a, cfg["gnss"], "norover" in cfg["scenario"],
                                               "nomaster" in cfg["scenario"]):
            if kind == 0:
                o = r.on_wheel(i, th, val)
            elif kind == 1:
                o = r.on_handle(th, val)
            elif fix_status:
                o = r.on_fix(th, i, *val, status=st)
            else:
                o = r.on_fix(th, i, *val)
            outs += o or []
        T = np.array([o["stamp"] for o in outs])
        X = np.array([[o["x"], o["y"], o["z"]] for o in outs])
        V = np.array([o["v"] for o in outs])
        S = np.array([o["s"] for o in outs])
        valid = np.array([bool(o.get("pos_valid", True)) for o in outs])
        pos = r.pos._p if new_pkg else r.pos
        if new_pkg and not cf.exists():
            cf.parent.mkdir(parents=True, exist_ok=True)
            with open(cf, "wb") as fh:
                pickle.dump(dict(log=log, T=T, V=V, S=S), fh)
    fr = getattr(pos, "frame", None)
    extra = dict(anchors=int(getattr(pos, "anchors", 0)),
                 mult=float(getattr(pos, "mult", 1.0)),
                 adapt_on=bool(getattr(pos, "scale_adapt", False)),
                 ready=bool(getattr(pos, "ready", False)))
    if fr is not None and hasattr(fr, "projection"):
        extra.update(projection=fr.projection, grid=fr.grid, zone=fr.zone,
                     E0=fr.E0, N0=fr.N0)
    slog = getattr(pos, "scale_log", None)
    f.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(f, T=T, X=X, V=V, S=S, valid=valid,
                        scale_log=np.array(slog if slog else np.zeros((0, 4))),
                        extra=json.dumps(extra))
    return b, "ok"


def cmd_run(args):
    pkg = Path(args.pkg) if args.pkg else PKG_DEFAULT
    opts = {}
    for kv in args.opt or []:
        k, v = kv.split("=", 1)
        try:
            opts[k] = ast.literal_eval(v)
        except (ValueError, SyntaxError):
            opts[k] = v
    _CFG.update(tag=args.tag, map=args.map, gnss=args.gnss, scenario=args.scenario,
                opts=opts, pkg=str(pkg), calib=args.calib, replay=args.replay)
    ids = split_ids(args.ids)
    (OUT / args.tag).mkdir(parents=True, exist_ok=True)
    (OUT / args.tag / "config.json").write_text(json.dumps(
        dict(pkg=str(pkg), map=args.map, gnss=args.gnss, scenario=args.scenario,
             opts=opts, ids=ids, frame=args.frame, calib=args.calib),
        ensure_ascii=False, indent=1), encoding="utf-8")
    with ProcessPoolExecutor(args.workers) as ex:
        for b, st in ex.map(_run_one, ids):
            if st != "ok":
                print(f"  {b}: {st}", flush=True)
    print(f"run {args.tag}: {len(ids)} прогонов", flush=True)


# ------------------------------------------------------------------ метрики
def nearest(t_out, t_ref):
    j = np.clip(np.searchsorted(t_out, t_ref), 1, len(t_out) - 1)
    j = np.where(np.abs(t_out[j - 1] - t_ref) < np.abs(t_out[j] - t_ref), j - 1, j)
    return j, np.abs(t_out[j] - t_ref) <= TOL


def polyline(P, step=1.0):
    """Прореженная ломаная эталона (точки не ближе step м) и её длина дуги.
    Возвращает (вершины K×2, дуга K, индекс вершины для каждой точки P)."""
    keep, idx = [0], np.zeros(len(P), int)
    last = P[0, :2]
    for i in range(1, len(P)):
        if np.hypot(*(P[i, :2] - last)) >= step:
            keep.append(i)
            last = P[i, :2]
        idx[i] = len(keep) - 1
    V = P[keep, :2]
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(V, axis=0).T))]
    return V, s, idx


def along_cross(X2, V, s, s_ref, win=60.0):
    """Проекция выхода на ломаную эталона в окне дуги ±win вокруг s_ref.
    along = s_out − s_ref, cross — расстояние до ломаной (со знаком: + слева)."""
    if len(V) < 2:
        return np.zeros(len(X2)), np.zeros(len(X2))
    A_, B_ = V[:-1], V[1:]
    D = B_ - A_
    L2 = np.maximum((D ** 2).sum(1), 1e-12)
    al, cr = np.empty(len(X2)), np.empty(len(X2))
    for k in range(len(X2)):
        lo = max(np.searchsorted(s, s_ref[k] - win) - 1, 0)
        hi = min(np.searchsorted(s, s_ref[k] + win) + 1, len(D))
        if hi <= lo:
            lo, hi = 0, len(D)
        d = D[lo:hi]
        u = np.clip(((X2[k] - A_[lo:hi]) * d).sum(1) / L2[lo:hi], 0.0, 1.0)
        F_ = A_[lo:hi] + d * u[:, None]
        dist = np.hypot(*(X2[k] - F_).T)
        m = int(np.argmin(dist))
        sgn = np.sign(d[m, 0] * (X2[k, 1] - F_[m, 1]) - d[m, 1] * (X2[k, 0] - F_[m, 0]))
        al[k] = s[lo + m] + u[m] * math.sqrt(L2[lo + m]) - s_ref[k]
        cr[k] = sgn * dist[m]
    return al, cr


def output_continuous(X, P_utm, cfg, extra):
    """Выход -> непрерывные UTM (E, N, z) для сравнения; (E, N) выхода и
    число отсчётов, где выход и эталон в разных 100-км квадратах."""
    proj = extra.get("projection", "rel")
    Y = X.copy()
    if proj == "mgrs":
        grid = extra.get("grid") or ""
        if grid:
            E0, N0 = georef.grid_origin(grid, float(P_utm[0, 1]))
            Y[:, 0] += E0
            Y[:, 1] += N0
        else:
            for k in (0, 1):
                Y[:, k] = X[:, k] + 1e5 * np.round((P_utm[:, k] - X[:, k]) / 1e5)
    elif proj != "utm":
        raise ValueError(proj)
    sq = ((np.floor(Y[:, 0] / 1e5) != np.floor(P_utm[:, 0] / 1e5))
          | (np.floor(Y[:, 1] / 1e5) != np.floor(P_utm[:, 1] / 1e5)))
    return Y, int(sq.sum())


def score_run(b, tag):
    f = OUT / tag / f"{b}.npz"
    if not f.exists():
        return None
    z = np.load(f)
    T, X = z["T"], z["X"]
    valid = z["valid"] if "valid" in z.files else np.ones(len(T), bool)
    if len(T) < 10:
        return None
    cfg = json.loads((OUT / tag / "config.json").read_text(encoding="utf-8"))
    extra = json.loads(str(z["extra"]))
    a = scenario_data(bagio.load(b), cfg["scenario"])
    g = a["mvel"]
    jv, okv = nearest(T, g[:, 1])
    vg = np.hypot(g[:, 2], g[:, 3])
    ev = z["V"][jv[okv]] - vg[okv]
    res = dict(v_pairs=int(okv.sum()), extra=extra,
               v_mae=float(np.mean(np.abs(ev))), v_rmse=float(np.sqrt(np.mean(ev ** 2))),
               v_bias=float(np.mean(ev)))
    gr = a["rvel"]
    if len(gr) > 50:
        jr, okr = nearest(T, gr[:, 1])
        er = z["V"][jr[okr]] - np.hypot(gr[okr, 2], gr[okr, 3])
        res["v_mae_rover"] = float(np.mean(np.abs(er)))
    dtg = np.clip(np.diff(g[:, 1]), 0, 1.0)
    res["dist_m"] = dist = float(np.sum(0.5 * (vg[1:] + vg[:-1]) * dtg))
    proj = extra.get("projection")
    frame = cfg.get("frame") or ("rel_equirect" if proj is None else
                                 "abs" if proj in ("mgrs", "utm") else f"rel_{proj}")
    tr, P = georef.reference(a["mfix"], "utm" if frame == "abs" else
                             frame.replace("rel_", ""))
    j, ok = nearest(T, tr)
    ok &= valid[j]
    if ok.sum() < 10:
        return b, res
    Pk = P[ok]
    Xk = X[j[ok]]
    if frame == "abs":
        Xk, nsq = output_continuous(Xk, Pk, cfg, extra)
        res["sq_mismatch"] = nsq
        if proj == "mgrs" and not extra.get("grid"):
            # как у судьи, сравнивающего плоские координаты MGRS напрямую:
            # эталон тоже «каждая точка в своём квадрате», без разворота
            _, Pm = georef.reference(a["mfix"], "mgrs")
            res["mean3d_insq"] = float(np.linalg.norm(X[j[ok]] - Pm[ok], axis=1).mean())
        res["west_frac"] = float(np.mean(Pk[:, 0] < 4e5))
        res["crosses_400km"] = bool((Pk[:, 0] < 4e5).any() and (Pk[:, 0] >= 4e5).any())
    E = Xk - Pk
    d3 = np.linalg.norm(E, axis=1)
    V, s, idx = polyline(P)
    sp = s[idx][ok]
    al0, _ = along_cross(Pk[:, :2], V, s, sp)
    s_ref = sp + al0
    al, cr = along_cross(Xk[:, :2], V, s, s_ref)
    res.update(
        n=int(ok.sum()), mean3d=float(d3.mean()), end3d=float(d3[-1]),
        max3d=float(d3.max()), rmse3d=float(np.sqrt(np.mean(d3 ** 2))),
        mean2d=float(np.linalg.norm(E[:, :2], axis=1).mean()),
        mz=float(np.abs(E[:, 2]).mean()),
        drift_end_pct=float(100.0 * d3[-1] / max(dist, 1.0)),
        along_mean=float(np.mean(np.abs(al))), along_max=float(np.max(np.abs(al))),
        along_rmse=float(np.sqrt(np.mean(al ** 2))), along_end=float(al[-1]),
        cross_mean=float(np.mean(np.abs(cr))), cross_max=float(np.max(np.abs(cr))),
        unpaired_ref=int((~ok).sum()))
    return b, res


def _score_job(args):
    return score_run(*args)


def cmd_score(args):
    for tag in args.tag.split(","):
        cfg = json.loads((OUT / tag / "config.json").read_text(encoding="utf-8"))
        jobs = [(b, tag) for b in cfg["ids"]]
        with ProcessPoolExecutor(args.workers) as ex:
            res = {r[0]: r[1] for r in ex.map(_score_job, jobs) if r is not None}
        (OUT / tag / "score.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
        print(f"score {tag}: {len(res)} прогонов", flush=True)


def summary(res):
    rows = [(b, r, r["v_pairs"]) for b, r in res.items() if "mean3d" in r]
    if not rows:
        return None
    w = np.array([x[2] for x in rows], float)

    def wm(k):
        return float(np.average([x[1][k] for x in rows], weights=w))
    ends = np.array([x[1]["end3d"] for x in rows])
    return dict(n=len(rows), mean3d=wm("mean3d"), end_mean=float(ends.mean()),
                end_median=float(np.median(ends)), end_max=float(ends.max()),
                drift_pct=float(np.mean([x[1]["drift_end_pct"] for x in rows])),
                along_mean=wm("along_mean"),
                along_max=float(max(x[1]["along_max"] for x in rows)),
                along_rmse=wm("along_rmse"), cross_mean=wm("cross_mean"),
                mz=wm("mz"), v_mae=wm("v_mae"), v_bias=wm("v_bias"),
                v_mae_rover=float(np.mean([x[1].get("v_mae_rover", np.nan) for x in rows])),
                sq=int(sum(x[1].get("sq_mismatch", 0) for x in rows)),
                insq=(wm("mean3d_insq") if all("mean3d_insq" in x[1] for x in rows)
                      else float("nan")))


def load_score(tag):
    return json.loads((OUT / tag / "score.json").read_text(encoding="utf-8"))


def cmd_report(args):
    tags = args.tags.split(",")
    print("\nСредние взвешены числом пар скорости GNSS; конец — ошибка 3D в конце "
          "прогона; дрейф — конец / путь GNSS.\n")
    print("| вариант | прогонов | ср. 3D, м | конец ср. / мед. / макс, м | дрейф, % | "
          "along ср. / RMSE / макс, м | cross ср., м | \\|z\\|, м | MAE v, м/с | разн. квадрат | "
          "ср. 3D в квадрате, м |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for t in tags:
        s = summary(load_score(t))
        if s is None:
            continue
        print(f"| {t} | {s['n']} | {s['mean3d']:.2f} | {s['end_mean']:.1f} / {s['end_median']:.1f} / "
              f"{s['end_max']:.1f} | {s['drift_pct']:.3f} | {s['along_mean']:.2f} / {s['along_rmse']:.2f} / "
              f"{s['along_max']:.1f} | {s['cross_mean']:.2f} | {s['mz']:.2f} | {s['v_mae']:.4f} | {s['sq']} | "
              f"{s['insq']:.2f} |")
    if args.runs:
        S = {t: load_score(t) for t in tags}
        ids = sorted(set().union(*[set(v) for v in S.values()]))
        print("\nПо прогонам: ср. 3D / конец, м\n")
        print("| прогон | " + " | ".join(tags) + " |")
        print("|---|" + "---:|" * len(tags))
        for b in ids:
            cells = []
            for t in tags:
                r = S[t].get(b, {})
                cells.append(f"{r['mean3d']:.2f} / {r['end3d']:.1f}" if "mean3d" in r else "—")
            print(f"| {b} | " + " | ".join(cells) + " |")
    if args.detail:
        t = args.detail
        S = load_score(t)
        print(f"\nПодробно: {t}\n")
        print("| прогон | пар | ср. 3D | конец | дрейф % | along ср./RMSE/макс | cross ср./макс | "
              "\\|z\\| | привязок | mult | MAE v / rover | E<400 км | разн. кв. |")
        print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
        for b in sorted(S):
            r = S[b]
            if "mean3d" not in r:
                continue
            e = r["extra"]
            print(f"| {b} | {r['n']} | {r['mean3d']:.2f} | {r['end3d']:.1f} | {r['drift_end_pct']:.3f} | "
                  f"{r['along_mean']:.2f} / {r['along_rmse']:.2f} / {r['along_max']:.1f} | "
                  f"{r['cross_mean']:.2f} / {r['cross_max']:.1f} | {r['mz']:.2f} | {e.get('anchors', 0)} | "
                  f"{e.get('mult', 1.0):.4f} | {r['v_mae']:.4f} / {r.get('v_mae_rover', float('nan')):.4f} | "
                  f"{100 * r.get('west_frac', 0):.0f} % | {r.get('sq_mismatch', 0)} |")


def cmd_diff(args):
    """Максимальные расхождения двух вариантов по одинаковым меткам выхода."""
    A_, B_ = args.tags.split(",")
    print(f"| прогон | max \\|dv\\|, м/с | max \\|dp\\|, м | ср. 3D {A_} / {B_} |")
    print("|---|---:|---:|---:|")
    SA = SB = {}
    if (OUT / A_ / "score.json").exists() and (OUT / B_ / "score.json").exists():
        SA, SB = load_score(A_), load_score(B_)
    for f in sorted((OUT / A_).glob("*.npz")):
        g = OUT / B_ / f.name
        if not g.exists():
            continue
        za, zb = np.load(f), np.load(g)
        common, ia, ib = np.intersect1d(np.round(za["T"], 3), np.round(zb["T"], 3),
                                        return_indices=True)
        dv = np.abs(za["V"][ia] - zb["V"][ib]).max() if len(common) else float("nan")
        va = za["valid"][ia] if "valid" in za.files else np.ones(len(ia), bool)
        vb = zb["valid"][ib] if "valid" in zb.files else np.ones(len(ib), bool)
        m = va & vb
        dp = (np.linalg.norm(za["X"][ia][m] - zb["X"][ib][m], axis=1).max()
              if m.any() else float("nan"))
        ra, rb = SA.get(f.stem, {}), SB.get(f.stem, {})
        cell = (f"{ra['mean3d']:.2f} / {rb['mean3d']:.2f}"
                if "mean3d" in ra and "mean3d" in rb else "—")
        print(f"| {f.stem} | {dv:.2e} | {dp:.3f} | {cell} |")


def cmd_boundary(args):
    ids = split_ids(args.ids)
    n = 0
    print("| прогон | E мин / макс, км | доля эталона E < 400 км | квадраты |")
    print("|---|---:|---:|---|")
    for b in ids:
        a = bagio.load(b)
        if len(a["mfix"]) < 100:
            continue
        tr, P = georef.reference(a["mfix"], "utm")
        m = a["mfix"][(a["mfix"][:, 5] >= 0) & np.isfinite(a["mfix"][:, 2])]
        zone = georef.zone_of(m[0, 3])
        sq = sorted({georef.square_code(e, nn, zone, la) for e, nn, la in
                     zip(P[::50, 0], P[::50, 1], m[::50, 2])})
        west = float(np.mean(P[:, 0] < 4e5))
        cross = (P[:, 0] < 4e5).any() and (P[:, 0] >= 4e5).any()
        n += cross
        print(f"| {b} | {P[:, 0].min() / 1e3:.2f} / {P[:, 0].max() / 1e3:.2f} | "
              f"{100 * west:.1f} % | {', '.join(sq)} |")
    print(f"\nпересекают E = 400 км: {n} из {len(ids)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--tag", required=True)
    r.add_argument("--pkg")
    r.add_argument("--map", default=str(EVAL_MAP))
    r.add_argument("--calib")
    r.add_argument("--ids", default="holdout")
    r.add_argument("--gnss", default="window", choices=["window", "full"])
    r.add_argument("--scenario", default="normal",
                   choices=["normal", "norover", "nomaster", "cut", "cut_norover"])
    r.add_argument("--frame", default=None, help="abs (по умолчанию) | rel_equirect | rel_enu")
    r.add_argument("--opt", action="append")
    r.add_argument("--replay", action="store_true",
                   help="только положение по записи ядра (out/position_eval/_core)")
    r.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "2")))
    s = sub.add_parser("score")
    s.add_argument("--tag", required=True)
    s.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "2")))
    p = sub.add_parser("report")
    p.add_argument("--tags", required=True)
    p.add_argument("--runs", action="store_true")
    p.add_argument("--detail")
    d = sub.add_parser("diff")
    d.add_argument("--tags", required=True)
    bnd = sub.add_parser("boundary")
    bnd.add_argument("--ids", default="holdout")
    args = ap.parse_args()
    {"run": cmd_run, "score": cmd_score, "report": cmd_report, "diff": cmd_diff,
     "boundary": cmd_boundary}[args.cmd](args)


if __name__ == "__main__":
    main()
