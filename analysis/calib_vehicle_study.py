"""Нужны ли вагону свои параметры, кроме масштаба колёс? Выбор только на train.

Сравнение «общий лист (оба вагона)» против «свой вагон» на обучающих записях
(tools/split.json: train_unique) каждого вагона, перекрёстной проверкой по 5
частям: подгонка на обучающих без части, проверка на части. Отложенные записи
(holdout) не используются вовсе.

    python3 analysis/calib_vehicle_study.py drive   # таблица привода A(u, v):
        СКО ускорения модели привода на отложенной части, м/с²
    python3 analysis/calib_vehicle_study.py speed   # полная связка (ядро):
        MAE и смещение скорости против GNSS на отложенной части при
        таблице привода и масштабе колёс «общий» / «свой вагон»
    python3 analysis/calib_vehicle_study.py online  # «что если»: онлайн-масштаб
        пути по привязкам к остановкам (Position.mult, лист ОЦЕНКИ, vehicle
        auto, карта ОЦЕНКИ) умножить и на скорость - MAE и смещение по
        вагонам и датам (первый «что если»)
    python3 analysis/calib_vehicle_study.py grid    # перебор онлайн-масштаба
        колёс (vehicle.OnlineWheelScale, параметр ноды wheel_scale_online):
        MAE и смещение скорости по вагонам и датам

Результат: out/vehicle/study_<режим>.json и таблица в консоли.
"""

import argparse
import json
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE.parent / "tools"))

import bagio                                                     # noqa: E402
import calib_drive as CD                                         # noqa: E402
import calib_eval as CE                                          # noqa: E402
import calib_vehicle as CV                                       # noqa: E402

ROOT = HERE.parent
OUT = ROOT / "out" / "vehicle"
SHEET = CE.PKG / "config" / "eval" / "tram_calibration.json"
K = 5


def folds(ids):
    ids = sorted(ids)
    return [ids[k::K] for k in range(K)]


def train_by_vehicle():
    md = CV.meta()
    ids = CD.fit_ids("eval")
    return ids, {v: [b for b in ids if md[b]["vehicle"] == v] for v in CV.VEHICLES}


def cached_samples(ids, delay, tau):
    out = {}
    for b in ids:
        try:
            U, V, A = CD.samples([b], delay, tau)
        except ValueError:          # нет отсчётов
            continue
        out[b] = (U, V, A)
    return out


def cat(S, ids):
    xs = [S[b] for b in ids if b in S]
    return tuple(np.concatenate([x[i] for x in xs]) for i in range(3))


def sheet_table(U, V, A, lam=(0.3, 0.3)):
    """Таблица как в листе: подгонка + знаковые ограничения calib_sheet."""
    W = CD.fit(U, V, A, lam)
    T = W.reshape(len(CD.U_GRID), len(CD.V_GRID))
    coast = T[CD.U_GRID == 0][0].copy()
    for i, u in enumerate(CD.U_GRID):
        if u < 0:
            T[i] = np.minimum(T[i], np.minimum(coast, 0.0))
        elif u > 0:
            T[i] = np.maximum(T[i], coast)
    return T


def rmse_on(T, U, V, A):
    e = A - CD.predict(T.ravel(), U, V)
    return float(np.sqrt(np.mean(e ** 2))), float(np.mean(e))


def study_drive(delay, tau):
    ids, byv = train_by_vehicle()
    S = cached_samples(ids, delay, tau)
    res = {}
    for v in CV.VEHICLES:
        rows = []
        for fold in folds([b for b in byv[v] if b in S]):
            rest_all = [b for b in ids if b in S and b not in fold]
            rest_own = [b for b in byv[v] if b in S and b not in fold]
            Uf, Vf, Af = cat(S, fold)
            T_all = sheet_table(*cat(S, rest_all))
            T_own = sheet_table(*cat(S, rest_own))
            r_all, b_all = rmse_on(T_all, Uf, Vf, Af)
            r_own, b_own = rmse_on(T_own, Uf, Vf, Af)
            rows.append(dict(n=int(len(Af)), runs=len(fold), rmse_all=r_all, rmse_own=r_own,
                             bias_all=b_all, bias_own=b_own))
            print(f"  {v} часть ({len(fold)} записей, {len(Af)} отсчётов): общий {r_all:.4f} "
                  f"(смещ. {b_all:+.4f}), свой {r_own:.4f} (смещ. {b_own:+.4f}) м/с²", flush=True)
        n = np.array([r["n"] for r in rows], float)
        tot = {k: float(np.sqrt(np.sum(n * np.array([r[k] for r in rows]) ** 2) / n.sum()))
               for k in ("rmse_all", "rmse_own")}
        res[v] = dict(folds=rows, **tot)
        print(f"{v}: СКО ускорения на отложенных частях - общий {tot['rmse_all']:.4f}, "
              f"свой {tot['rmse_own']:.4f} м/с²")
    return res


_P = None           # лист текущего прогона (наследуется дочерними процессами)


def _speed_run(b):
    """Скорость полной связки (как tools/eval.py: Runner ноды, GNSS первые
    3 с, без карты - карта на скорость не влияет) против |v| GNSS master."""
    import eval_metrics as EM
    import eval_replay as ER
    a = bagio.load(b)
    if len(a["mvel"]) < 100:
        return b, None
    node = ER.resolve_sheet("eval")["node"]
    r, _ = ER.make_runner(_P, node, None)
    (O, crash), = ER.replay(ER.events(a, "3"), [r], node)
    row, s = EM.score_speed(O, a)
    if s is None or crash is not None:
        return b, None
    return b, dict(ev=s["ev"], phase=s["phase"])


def speed_eval(p, ids, workers):
    global _P
    from concurrent.futures import ProcessPoolExecutor
    _P = p
    with ProcessPoolExecutor(workers) as ex:
        return {b: r for b, r in ex.map(_speed_run, ids) if r is not None}


def speed_stats(res):
    ev = np.concatenate([r["ev"] for r in res.values()])
    ph = np.concatenate([r["phase"] for r in res.values()])
    out = dict(runs=len(res), n=int(len(ev)), mae=float(np.mean(np.abs(ev))),
               bias=float(np.mean(ev)), rmse=float(np.sqrt(np.mean(ev ** 2))), by_phase={})
    for k, name in enumerate(("стоянка", "тяга", "выбег", "торможение")):
        m = ph == k
        if m.any():
            out["by_phase"][name] = dict(mae=float(np.mean(np.abs(ev[m]))),
                                         bias=float(np.mean(ev[m])))
    return out


def study_speed(delay, tau, workers):
    ids, byv = train_by_vehicle()
    S = cached_samples(ids, delay, tau)
    base = CE.load_sheet(SHEET)
    per_run = {b: x for b, x in ((b, CV.run_ratio(b)) for b in ids) if x is not None}
    res = {}
    for v in CV.VEHICLES:
        pooled = {}
        for fold in folds([b for b in byv[v] if b in S]):
            rest_all = [b for b in ids if b in S and b not in fold]
            rest_own = [b for b in byv[v] if b in S and b not in fold]
            T_all = sheet_table(*cat(S, rest_all))
            T_own = sheet_table(*cat(S, rest_own))
            ms_all = CV.fit_scale([b for b in rest_all if b in per_run], per_run)[0]
            ms_own = CV.fit_scale([b for b in rest_own if b in per_run], per_run)[0]
            variants = {
                "общий": dict(acc_table=tuple(np.round(T_all.ravel(), 4)), meas_scale=ms_all),
                "свой масштаб": dict(acc_table=tuple(np.round(T_all.ravel(), 4)), meas_scale=ms_own),
                "своя таблица": dict(acc_table=tuple(np.round(T_own.ravel(), 4)), meas_scale=ms_all),
                "свой вагон": dict(acc_table=tuple(np.round(T_own.ravel(), 4)), meas_scale=ms_own),
            }
            gids = [b for b in fold if b in per_run]
            for name, ov in variants.items():
                p = replace(base, delay_drive=delay, tau_drive=tau, **ov)
                pooled.setdefault(name, {}).update(speed_eval(p, gids, workers))
        out = {}
        for name, r in pooled.items():
            s = speed_stats(r)
            out[name] = s
            print(f"{v} {name:13s}: {s['runs']} записей, MAE {s['mae']:.4f} "
                  f"смещение {s['bias']:+.4f} RMSE {s['rmse']:.4f}", flush=True)
        res[v] = out
    return res


SEG_MIN_L, SEG_MAX_DEV, SEG_MIN_N, SEG_CAP = 300.0, 0.03, 2, 0.02
DEADBANDS = (0.003, 0.005)


def seg_scale(log, s0, nlog):
    """Причинная оценка масштаба колёс по отрезкам между привязками к
    остановкам: у отрезка k = (L·s0·mult + δ) / (L·s0) (как Position.
    _adapt_scale, но без ворот ±1 % и отключения); медиана отрезков длиннее
    SEG_MIN_L с |k − 1| < SEG_MAX_DEV, не меньше SEG_MIN_N отрезков, в пределах
    ±SEG_CAP. nlog - длина журнала привязок на каждом выходе."""
    ks = []
    used = 1.0          # mult, с которым курсор шёл по отрезку (в журнале - после привязки)
    for (_ds, d, L, mult) in log:
        k = (L * s0 * used + d) / (L * s0) if L > 0 else float("nan")
        ks.append(k if L >= SEG_MIN_L and abs(k - 1.0) < SEG_MAX_DEV else float("nan"))
        used = mult
    est = np.ones(len(log) + 1)
    for n in range(1, len(log) + 1):
        x = [k for k in ks[:n] if np.isfinite(k)]
        if len(x) >= SEG_MIN_N:
            est[n] = min(max(float(np.median(x)), 1.0 - SEG_CAP), 1.0 + SEG_CAP)
    return est[np.asarray(nlog, int)]


def _online_run(b):
    """Связка как в eval.py (лист ОЦЕНКИ, vehicle auto, карта ОЦЕНКИ, GNSS 3 с);
    на каждом выходе запоминается текущий Position.mult (причинно)."""
    import eval_metrics as EM
    import eval_replay as ER
    a = bagio.load(b)
    if len(a["mvel"]) < 100:
        return b, None
    sheet = ER.resolve_sheet("eval")
    node = dict(sheet["node"], vehicle="auto")
    p = ER.make_params(dict(sheet, node=node))
    tmap = ER.load_map(CE.PKG / "config" / "eval" / "track_map.npz")
    r, _ = ER.make_runner(p, node, tmap)
    gl = ER.Glue(r, node)
    T, Vv, Mm, Nl = [], [], [], []
    for tb, kind, i, th, val in ER.events(a, "3"):
        for o in gl.feed(tb, kind, i, th, val):
            T.append(o["stamp"])
            Vv.append(o["v"])
            Mm.append(r.pos.mult)
            Nl.append(len(r.pos.scale_log))
    T, Vv, Mm, Nl = map(np.asarray, (T, Vv, Mm, Nl))
    Ks = seg_scale(r.pos.scale_log, tmap.scale, Nl)
    O = dict(T=T, V=Vv)
    _, s0 = EM.score_speed(O, a)
    _, s1 = EM.score_speed(dict(T=T, V=Vv * Mm), a)
    _, s2 = EM.score_speed(dict(T=T, V=Vv * Ks), a)
    if s0 is None:
        return b, None
    dead = {}
    for db in DEADBANDS:            # мёртвая зона: поправка, только если |k − 1| > db
        Kd = np.where(np.abs(Ks - 1.0) > db, Ks, 1.0)
        dead[f"ev_d{db:g}"] = EM.score_speed(dict(T=T, V=Vv * Kd), a)[1]["ev"]
    ratio = CV.run_ratio(b)
    return b, dict(ev=s0["ev"], ev_m=s1["ev"], ev_s=s2["ev"], **dead, mult_end=float(Mm[-1]),
                   seg_end=float(Ks[-1]),
                   anchors=int(r.pos.anchors), adapt_on=bool(r.pos.scale_adapt),
                   k_run=(float(1.0 / np.median(ratio) / p.meas_scale) if ratio is not None
                          else float("nan")))


def study_online(workers):
    from concurrent.futures import ProcessPoolExecutor
    md = CV.meta()
    ids = CD.fit_ids("eval")
    with ProcessPoolExecutor(workers) as ex:
        res = {b: r for b, r in ex.map(_online_run, ids) if r is not None}
    groups = {}
    for b in res:
        groups.setdefault((md[b]["vehicle"], md[b]["date"]), []).append(b)
    out = {}
    print("вагон  дата        записей  MAE как есть / × mult  смещение как есть / × mult  "
          "k записи  mult в конце (ср.)")
    for key in sorted(groups):
        rs = [res[b] for b in groups[key]]
        e0 = np.concatenate([r["ev"] for r in rs])
        e1 = np.concatenate([r["ev_m"] for r in rs])
        e2 = np.concatenate([r["ev_s"] for r in rs])
        row = dict(runs=len(rs), mae=float(np.mean(np.abs(e0))), mae_mult=float(np.mean(np.abs(e1))),
                   bias=float(np.mean(e0)), bias_mult=float(np.mean(e1)),
                   mae_seg=float(np.mean(np.abs(e2))), bias_seg=float(np.mean(e2)),
                   seg_end=float(np.mean([r["seg_end"] for r in rs])),
                   k_run=float(np.nanmean([r["k_run"] for r in rs])),
                   mult_end=float(np.mean([r["mult_end"] for r in rs])),
                   adapt_off=int(sum(not r["adapt_on"] for r in rs)))
        out[f"{key[0]} {key[1]}"] = row
        print(f"{key[0]}  {key[1]}  {row['runs']:7d}  {row['mae']:.4f} / {row['mae_mult']:.4f}"
              f"        {row['bias']:+.4f} / {row['bias_mult']:+.4f}              "
              f"{row['k_run']:.4f}    {row['mult_end']:.4f} (выкл. {row['adapt_off']})")
        print(f"                           по отрезкам: MAE {row['mae_seg']:.4f}, смещение "
              f"{row['bias_seg']:+.4f}, оценка в конце (ср.) {row['seg_end']:.4f}")
        for db in DEADBANDS:
            e = np.concatenate([r[f"ev_d{db:g}"] for r in rs])
            row[f"mae_d{db:g}"], row[f"bias_d{db:g}"] = float(np.mean(np.abs(e))), float(np.mean(e))
            print(f"                           по отрезкам, мёртвая зона {100 * db:.1f} %: MAE "
                  f"{row[f'mae_d{db:g}']:.4f}, смещение {row[f'bias_d{db:g}']:+.4f}")
    allr = list(res.values())
    for key, name in (("ev", "как есть"), ("ev_m", "× mult"), ("ev_s", "по отрезкам"),
                      *((f"ev_d{db:g}", f"по отрезкам, мёртвая зона {100 * db:.1f} %")
                        for db in DEADBANDS)):
        e = np.concatenate([r[key] for r in allr])
        out[f"_all_{key}"] = dict(mae=float(np.mean(np.abs(e))), bias=float(np.mean(e)))
        print(f"все {len(allr)} записей, {name}: MAE {np.mean(np.abs(e)):.4f}, смещение "
              f"{np.mean(e):+.4f}")
    out["_runs"] = {b: {k: v for k, v in r.items() if not k.startswith("ev")}
                    for b, r in res.items()}
    return out


# перебор онлайн-масштаба колёс (vehicle.OnlineWheelScale): (имя, параметры)
GRID = [("выкл.", None)]
for _n in (1, 2, 3):
    for _db in (0.0, 0.003, 0.005, 0.007, 0.01):
        GRID.append((f"n{_n} мз{100 * _db:.1f}", dict(min_n=_n, deadband=_db)))
for _l in (200.0, 500.0):
    GRID.append((f"n2 мз0.5 L{_l:.0f}", dict(min_n=2, deadband=0.005, min_l=_l)))
for _dev in (0.02, 0.05):
    GRID.append((f"n2 мз0.5 dev{100 * _dev:.0f}", dict(min_n=2, deadband=0.005, max_dev=_dev)))
GRID.append(("n2 мз0.5 cap1", dict(min_n=2, deadband=0.005, cap=0.01)))
GRID.append(("n2 мз0.5 cap3", dict(min_n=2, deadband=0.005, cap=0.03)))
# без первого отрезка (от выставки по GNSS, а не от остановки) и/или оценка
# Σ L·k / Σ L вместо медианы (варианты появились после разбора отложенного
# 30618_21dd3af3: медиана двух отрезков = среднее, первый отрезок 473 м с
# k 0,9835 дал поправку −0,8 % на 1,7 км; выбор - только по train)
for _sf in (False, True):
    for _w in (False, True):
        if not (_sf or _w):
            continue
        for _n in (1, 2, 3):
            for _db in (0.003, 0.005, 0.007):
                GRID.append((f"n{_n} мз{100 * _db:.1f}{' б1' if _sf else ''}{' взв' if _w else ''}",
                             dict(min_n=_n, deadband=_db, skip_first=_sf, weighted=_w)))


def k_series(log, s0, kw):
    """Поправка скорости после каждой записи журнала привязок (0..N) - тем же
    классом, что в ноде, на растущем журнале (причинно)."""
    from tram_state_estimator import vehicle as V
    ws = V.OnlineWheelScale(**kw)
    cur = []
    out = [ws.factor(cur, s0)]
    for e in log:
        cur.append(e)
        out.append(ws.factor(cur, s0))
    return np.asarray(out)


def _grid_run(b):
    """Связка как в eval.py (лист и карта ОЦЕНКИ, vehicle - вагон записи,
    GNSS 3 с), без онлайн-масштаба; поправки перебора - после связки, по
    журналу привязок на каждом выходе (как делает нода: скорость выхода ×
    поправка на этом шаге)."""
    import eval_metrics as EM
    import eval_replay as ER
    a = bagio.load(b)
    if len(a["mvel"]) < 100:
        return b, None
    sheet = ER.resolve_sheet("eval")
    node = dict(sheet["node"], vehicle="match", wheel_scale_online=False)
    p = ER.make_params(dict(sheet, node=node), bag=b)
    tmap = ER.load_map(CE.PKG / "config" / "eval" / "track_map.npz")
    r, _ = ER.make_runner(p, node, tmap)
    gl = ER.Glue(r, node)
    T, Vv, Nl = [], [], []
    for tb, kind, i, th, val in ER.events(a, "3"):
        for o in gl.feed(tb, kind, i, th, val):
            T.append(o["stamp"])
            Vv.append(o["v"])
            Nl.append(len(r.pos.scale_log))
    T, Vv, Nl = map(np.asarray, (T, Vv, Nl))
    log = list(r.pos.scale_log)
    res = {}
    for name, kw in GRID:
        K = np.ones(len(log) + 1) if kw is None else k_series(log, tmap.scale, kw)
        _, s = EM.score_speed(dict(T=T, V=Vv * K[Nl]), a)
        if s is None:
            return b, None
        res[name] = dict(ae=float(np.sum(np.abs(s["ev"]))), e=float(np.sum(s["ev"])),
                         n=int(len(s["ev"])), k_end=float(K[-1]))
    return b, res


def study_grid(workers):
    """Перебор онлайн-масштаба колёс на train: MAE и смещение скорости по
    вагонам и датам, по вагону 30618 и по всем."""
    from concurrent.futures import ProcessPoolExecutor
    md = CV.meta()
    ids = CD.fit_ids("eval")
    with ProcessPoolExecutor(workers) as ex:
        res = {b: r for b, r in ex.map(_grid_run, ids) if r is not None}
    groups = {}
    for b in res:
        v, d = md[b]["vehicle"], md[b]["date"]
        for g in (f"{v} {d}", v, "все"):
            groups.setdefault(g, []).append(b)
    out = {"_grid": {name: kw for name, kw in GRID}, "_groups": {}}
    order = sorted(k for k in groups if " " in k) + list(CV.VEHICLES) + ["все"]
    for g in order:
        bs = groups.get(g, [])
        row = {}
        for name, _ in GRID:
            ae = sum(res[b][name]["ae"] for b in bs)
            e = sum(res[b][name]["e"] for b in bs)
            n = sum(res[b][name]["n"] for b in bs)
            row[name] = dict(mae=ae / n, bias=e / n, runs=len(bs),
                             k_end=float(np.mean([res[b][name]["k_end"] for b in bs])))
        out["_groups"][g] = row
    base = "выкл."
    print(f"{'вариант':22s}" + "".join(f"{g:>18s}" for g in order))
    for name, _ in GRID:
        cells = []
        for g in order:
            m0 = out["_groups"][g][base]["mae"]
            m = out["_groups"][g][name]["mae"]
            cells.append(f"{m:.4f} ({100 * (m / m0 - 1):+5.1f}%)")
        print(f"{name:22s}" + "".join(f"{c:>18s}" for c in cells))
    out["_runs"] = {b: {name: r[name]["k_end"] for name, _ in GRID} for b, r in res.items()}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("drive", "speed", "online", "grid"))
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    cal = json.loads(SHEET.read_text(encoding="utf-8"))["params"]
    delay, tau = cal["delay_drive"], cal["tau_drive"]
    print(f"задержка ручки {delay} с, tau {tau} с (лист ОЦЕНКИ); части: {K}")
    if a.mode == "online":
        res = study_online(a.workers)
    elif a.mode == "grid":
        res = study_grid(a.workers)
    elif a.mode == "drive":
        res = study_drive(delay, tau)
    else:
        res = study_speed(delay, tau, a.workers)
    OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / f"study_{a.mode}.json"
    f.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print("записано:", f.relative_to(ROOT).as_posix())


if __name__ == "__main__":
    main()
