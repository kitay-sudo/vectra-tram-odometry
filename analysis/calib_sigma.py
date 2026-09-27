"""Шаг 2д: калибровка публикуемой σ по остаткам подгоночных прогонов.

Фильтр знает только свою модель; на реальных данных к его σ добавляются
возраст показаний (при торможении и разгоне оценка отстаёт на |a|·возраст),
ошибка масштаба колёс, шум эталона на стоянке, ошибка карты и точки привязки.
Форма (estimator_core.Estimator._sigma_v_out, position_sigma):

    σ_v² = (sv_gain·σ_фильтра)² + пол² + (sv_age·a)² + (sv_rel·v)²,
           пол = sv_floor_stand на стоянке (режим ядра), иначе sv_floor;
    σ_s² = σ_пути² + ss_map² + (ss_rel·Δs)²,
           Δs - путь после выставки или последней привязки к остановке.

Параметры подбираются перебором по сетке на подгоночных прогонах: доля
|ошибка| <= 2σ по всем парам 94,5–96,5 % (номинал 2σ - 95 %) и не меньше 90 %
в каждой фазе (стоянка, тяга, выбег, торможение - по ручке и скорости GNSS, как
в метриках); из подходящих - с наименьшей средней σ (самая острая). Для
положения: |along| <= 2σ_s не меньше 95 % при наименьшей медиане σ_s. Запас к
целям на отложенных (90–97 % и >= 85 % по фазам, >= 85 % по положению) - на
перенос: карта для подгоночных прогонов своя, along на них оптимистичен.

Пары сохраняются в out/calib/sigma_samples_<лист>.npz; --from-samples
перепадгоняет σ без нового прогона связки.

    python3 analysis/calib_sigma.py eval   # split train, карта только из train
    python3 analysis/calib_sigma.py jury   # все уникальные записи, карта пакета

Пишет sv_*/ss_* в analysis/calib_tuned_<eval|jury>.json ("params"), откуда их
берёт calib_sheet.py. Для jury карта пакета собрана по всем данным, поэтому
along на подгоночных прогонах оптимистичен - это оговорено в отчёте.
"""

import argparse
import json
import os
import sys
import time
from itertools import product
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import calib_eval as CE                                         # noqa: E402

CM = CE.CM
SHEET = {"eval": CE.PKG / "config" / "eval" / "tram_calibration.json",
         "jury": CE.PKG / "config" / "tram_calibration.json"}
IDS = {"eval": "train_unique", "jury": "all_unique"}
MAPS = {"eval": CE.ROOT / "analysis" / "cache" / "track_map_train.npz",
        "jury": CE.PKG / "config" / "track_map.npz"}
RESET = dict(sv_gain=1.0, sv_floor=0.0, sv_floor_stand=0.0, sv_age=0.0,
             sv_rel=0.0, ss_map=0.0, ss_rel=0.0)
MAP = None
STANDSTILL = 5


def run_one(b):
    """Выходы связки с картой; пары скорости (master vel) и положения
    (master fix: along) с сырыми σ фильтра."""
    p = CE._P
    a = CE.bagio.load(b)
    if len(a["mvel"]) < 50 or len(a["mfix"]) < 50:
        return b, None
    r = CE.Runner(p, track_map=CM.TrackMap.load(MAP) if MAP else None)
    outs = CM.replay(a, [r])[0]
    if len(outs) < 10:
        return b, None
    T = CM.arr(outs, "stamp")
    jj, vg, tg = CE._pairs(T, a["mvel"])
    V = CM.arr(outs, "v")
    sp = dict(e=(V[jj] - vg).astype(np.float32),
              sf=CM.arr(outs, "sigma_v_filt")[jj].astype(np.float32),
              sp=CM.arr(outs, "sigma_v")[jj].astype(np.float32),
              a=CM.arr(outs, "a")[jj].astype(np.float32),
              v=V[jj].astype(np.float32),
              stand=(CM.arr(outs, "mode", int)[jj] == STANDSTILL),
              phase=CE.phase_of(a, tg, vg))
    m = a["mfix"]
    enu = CM.Enu(m[0, 2], m[0, 3], m[0, 4])
    ref = np.array([enu.fwd(la, lo, al) for la, lo, al in m[:, 2:5]])
    j2, ok2 = CM.E.nearest(T, m[:, 1])
    idx = np.flatnonzero(ok2)
    jp = j2[ok2]
    X = np.array([[o["x"], o["y"]] for o in outs])
    al, _, _, _ = CM.along_cross(ref[:, :2], idx, X[jp])
    ds = np.array([o.get("ds_fix", np.nan) for o in outs])[jp]
    ps = dict(al=al.astype(np.float32), ss=CM.arr(outs, "sigma_s")[jp].astype(np.float32),
              ds=ds.astype(np.float32))
    return b, dict(sp=sp, ps=ps)


def cov_by_phase(e, s, ph):
    c = np.abs(e) <= 2 * s
    return float(c.mean()), {k: float(c[ph == k].mean()) for k in range(4) if (ph == k).any()}


def fit_speed(S):
    e, sf, a, v = (S[k].astype(float) for k in ("e", "sf", "a", "v"))
    stand, ph = S["stand"], S["phase"]
    grid = dict(sv_gain=(1.0, 1.25, 1.5, 2.0),
                sv_floor=(0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08),
                sv_floor_stand=(0.01, 0.02, 0.03, 0.04, 0.06),
                sv_age=(0.0, 0.05, 0.1, 0.15, 0.2),
                sv_rel=(0.0, 0.002, 0.004, 0.006))
    ae = np.abs(e)
    best, fallback = None, None
    for g, fl, fs, age, rel in product(*grid.values()):
        floor = np.where(stand, fs, fl)
        s2 = (g * sf) ** 2 + floor ** 2 + (age * a) ** 2 + (rel * v) ** 2
        c = ae <= 2 * np.sqrt(s2)
        tot = float(c.mean())
        phc = [float(c[ph == k].mean()) for k in range(4) if (ph == k).any()]
        sharp = float(np.sqrt(s2).mean())
        row = dict(sv_gain=g, sv_floor=fl, sv_floor_stand=fs, sv_age=age, sv_rel=rel,
                   cov2=tot, cov2_phase_min=min(phc), sigma_mean=sharp)
        if 0.945 <= tot <= 0.965 and min(phc) >= 0.90:
            if best is None or sharp < best["sigma_mean"]:
                best = row
        key = (min(phc), -abs(tot - 0.95))
        if fallback is None or key > fallback[0]:
            fallback = (key, row)
    return best or fallback[1], best is not None


def fit_pos(P):
    al, ss, ds = (P[k].astype(float) for k in ("al", "ss", "ds"))
    m = np.isfinite(al) & np.isfinite(ds)
    al, ss, ds = np.abs(al[m]), ss[m], ds[m]
    best, fallback = None, None
    for smap, rel in product((0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0),
                             (0.0, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.008,
                              0.010, 0.012, 0.015, 0.020, 0.025)):
        s = np.sqrt(ss ** 2 + smap ** 2 + (rel * ds) ** 2)
        cov = float(np.mean(al <= 2 * s))
        row = dict(ss_map=smap, ss_rel=rel, cov2_along=cov, sigma_s_median=float(np.median(s)))
        if cov >= 0.95 and (best is None or row["sigma_s_median"] < best["sigma_s_median"]):
            best = row
        if fallback is None or cov > fallback["cov2_along"]:
            fallback = row
    return best or fallback, best is not None


def main():
    global MAP
    ap = argparse.ArgumentParser()
    ap.add_argument("which", choices=("eval", "jury"))
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--from-samples", action="store_true",
                    help="взять пары из out/calib/sigma_samples_<лист>.npz")
    args = ap.parse_args()
    t0 = time.time()
    smp = CE.OUT / f"sigma_samples_{args.which}.npz"
    if args.from_samples:
        z = np.load(smp)
        S = {k[3:]: z[k] for k in z.files if k.startswith("sp_")}
        P = {k[3:]: z[k] for k in z.files if k.startswith("ps_")}
        done = list(z["runs"])
    else:
        CE._P = CE.load_sheet(SHEET[args.which], RESET)
        MAP = MAPS[args.which]
        ids = CE.split_ids(IDS[args.which])
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(args.workers) as ex:
            res = dict(ex.map(run_one, ids))
        done = [b for b in ids if res.get(b) is not None]
        S = {k: np.concatenate([res[b]["sp"][k] for b in done]) for k in res[done[0]]["sp"]}
        P = {k: np.concatenate([res[b]["ps"][k] for b in done]) for k in res[done[0]]["ps"]}
        CE.OUT.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(smp, runs=np.array(done), **{f"sp_{k}": v for k, v in S.items()},
                            **{f"ps_{k}": v for k, v in P.items()})
    c0, ph0 = cov_by_phase(S["e"], S["sp"], S["phase"])
    sv, ok_v = fit_speed(S)
    ssr, ok_s = fit_pos(P)
    m = np.isfinite(P["al"])
    cov_s0 = float(np.mean(np.abs(P["al"][m]) <= 2 * P["ss"][m]))
    report = dict(
        which=args.which, runs=len(done), pairs_v=int(len(S["e"])), pairs_s=int(m.sum()),
        before=dict(cov2_v=c0, cov2_v_phase=ph0, cov2_along=cov_s0,
                    sigma_v_median=float(np.median(S["sp"])),
                    sigma_s_median=float(np.median(P["ss"]))),
        speed=dict(sv, target_met=ok_v), position=dict(ssr, target_met=ok_s),
        wall_s=time.time() - t0)
    print(json.dumps(report, ensure_ascii=False, indent=1))
    out = HERE / f"calib_tuned_{args.which}.json"
    d = json.loads(out.read_text(encoding="utf-8")) if out.exists() else {"params": {}}
    for k in RESET:
        d["params"].pop(k, None)
    d["params"].update({k: sv[k] for k in ("sv_gain", "sv_floor", "sv_floor_stand",
                                            "sv_age", "sv_rel")})
    d["params"].update({k: ssr[k] for k in ("ss_map", "ss_rel")})
    d["sigma_fit"] = report
    out.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
    print("записано:", out)


if __name__ == "__main__":
    main()
