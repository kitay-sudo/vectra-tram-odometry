#!/usr/bin/env python3
"""Юз, буксование и шум обеих тележек: быстрый стенд подбора на ОБУЧАЮЩИХ прогонах.

Зачем. `tools/eval.py` считает инъекции на трёх отложенных прогонах, и
подбирать по ним пороги нельзя (утечка). Здесь те же инъекции
(`tools/inject.py`, те же окна и зёрна) ставятся в обучающие прогоны
`tools/split.json:train`, и связка (`Runner` пакета из этого дерева, лист
ОЦЕНКИ) проигрывает только отрезок вокруг аномалии: 40 с до неё (разгон
фильтра с нуля по первым показаниям), окно и 60 с после. GNSS в связку не
подаётся (скорость от него не зависит), он — только эталон скорости.

Режимы:
  --inject   инъекции (по умолчанию skid_brake, spin_traction, noise и
             контрольные outliers, both_zero, both_stuck, front_zero): для
             модели и базы «только колесо» — MAE во время и после, доли
             флагов, ±2σ, время восстановления (как в tools/eval.py:
             |v − v_чисто| ≤ 0,1 м/с не меньше 3 с);
  --clean    чистые прогоны целиком: MAE, смещение, ±2σ по фазам, число
             срывов всех осей (ложных: инъекций нет), наибольшая оценка шума;
  --openloop ошибка разомкнутого прогноза модели (таблица привода листа) против
             GNSS через 1, 2, 4, 6 с от точки, где скорость взята из GNSS, —
             отсюда slip_sigma_a (рост СКО ошибки скорости, м/с за секунду).
Виды инъекций, кроме видов tools/inject.py, — варианты глубины и длительности
(VARIANTS: юз −15 / −50 %, буксование +15 / +60 %, по 10 с, шум ×3 / ×10):
пороги не должны быть подогнаны под ровно ±30 % и 4 с. EDGE — форма срыва
(из ревью потока): плавное начало или конец, задняя тележка на 0,3 с позже,
циклы противоюзной защиты, юз, переходящий в блокировку (нули), длинный юз с
плавным отпусканием, блокировка с падением в ноль за 0,3 / 1 с (экстренное
торможение), короткий юз 1 с, юз одной передней тележки. Эти виды считаются
только по --kinds, например
  python3 tools/slip_study.py --inject --runs train --n 27 --kinds edge
(edge — все виды EDGE; std — виды по умолчанию). В сводке «slip после» — доля
шагов с флагом срыва за 60 с после окна (признак не должен защёлкиваться),
«valid и ошибка > 1 м/с» — доля точек окна, где оценка уверенно неверна.

Запуск (из корня дерева; кэш — analysis/cache или TRAM_CACHE):
  python3 tools/slip_study.py --inject --clean --runs train --out out/slip/now.json
  python3 tools/slip_study.py --report out/slip/now.json
  python3 tools/slip_study.py --openloop --runs train
  python3 tools/slip_study.py --compare out/slip2/head.json out/slip2/new.json
`--set k=v,...` — поля листа поверх (как в tools/eval.py), для перебора.
"""

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "analysis"))

import bagio                    # noqa: E402
import eval_metrics as M        # noqa: E402
import eval_replay as R         # noqa: E402
import inject as I              # noqa: E402

KINDS = ("skid_brake", "spin_traction", "noise", "outliers", "both_zero", "both_stuck",
         "front_zero")
# Варианты глубины и длительности (проверка обобщения: пороги не должны быть
# подогнаны под ровно ±30 % и 4 с). База окна — вид tools/inject.py;
# множитель показаний (или σ шума, м/с) и длительность — свои.
VARIANTS = {
    "skid_15": ("skid_brake", 0.85, 4.0),
    "skid_50": ("skid_brake", 0.5, 4.0),
    "skid_long": ("skid_brake", 0.7, 10.0),
    "spin_15": ("spin_traction", 1.15, 4.0),
    "spin_60": ("spin_traction", 1.6, 4.0),
    "spin_long": ("spin_traction", 1.3, 10.0),
    "noise_x3": ("noise", 0.15, 20.0),
    "noise_x10": ("noise", 0.5, 20.0),
}
for _name, (_base, _f, _dur) in VARIANTS.items():
    I.KINDS.setdefault(_name, dict(I.KINDS[_base], dur=_dur, ru=f"{_base} ×{_f}, {_dur:g} с"))


# Форма срыва во времени: множитель показаний f(tr) для tr = t − t0 в окне.
def _rampout(tr, dur, lo, ramp):
    """lo сразу, в конце плавно назад к 1 за ramp с."""
    return np.where(tr < dur - ramp, lo, lo + (1 - lo) * (tr - (dur - ramp)) / ramp)


def _rampin(tr, dur, lo, ramp):
    """плавно к lo за ramp с, отпускание скачком."""
    return np.where(tr < ramp, 1 + (lo - 1) * tr / ramp, lo)


def _ramp_both(tr, dur, lo, ramp):
    a = np.clip(tr / ramp, 0, 1)
    b = np.clip((dur - tr) / ramp, 0, 1)
    return 1 + (lo - 1) * np.minimum(a, b)


def _then_lock(tr, dur, lo, t_lock):
    """частичный юз, затем блокировка (нули), отпускание скачком."""
    return np.where(tr < t_lock, lo, 0.0)


def _lock_ramp(tr, dur, lo, ramp):
    """блокировка: плавно в ноль за ramp с (экстренное торможение), ноль до
    конца окна, отпускание скачком."""
    return np.clip(1.0 - tr / ramp, 0.0, 1.0)


def _wsp(tr, dur, lo, per):
    """противоюзная защита: глубокий юз 60 % периода, почти сцепление 40 %."""
    ph = np.mod(tr, per) / per
    return np.where(ph < 0.6, lo, 1 - 0.15 * (1 - lo))


# имя: (окно вида tools/inject.py, длительность, форма, множитель, параметр
# формы, запаздывание задней тележки, с[, тележки — по умолчанию обе])
EDGE = {
    "skid_rampout1": ("skid_brake", 4.0, _rampout, 0.7, 1.0, 0.0),
    "skid_rampout2": ("skid_brake", 4.0, _rampout, 0.7, 2.0, 0.0),
    "skid_rampin05": ("skid_brake", 4.0, _rampin, 0.7, 0.5, 0.0),
    "skid_ramp_both05": ("skid_brake", 4.0, _ramp_both, 0.7, 0.5, 0.0),
    "skid_stagger03": ("skid_brake", 4.0, None, 0.7, 0.0, 0.3),
    "skid_wsp": ("skid_brake", 4.0, _wsp, 0.7, 1.0, 0.0),
    "spin_rampout1": ("spin_traction", 4.0, _rampout, 1.3, 1.0, 0.0),
    "spin_rampout2": ("spin_traction", 4.0, _rampout, 1.3, 2.0, 0.0),
    "spin_stagger03": ("spin_traction", 4.0, None, 1.3, 0.0, 0.3),
    "skid_then_lock": ("skid_brake", 5.0, _then_lock, 0.7, 2.0, 0.0),
    "skid_long_rampout": ("skid_brake", 10.0, _rampout, 0.7, 1.0, 0.0),
    "lock_ramp03": ("skid_brake", 5.0, _lock_ramp, 0.0, 0.3, 0.0),
    "lock_ramp1": ("skid_brake", 5.0, _lock_ramp, 0.0, 1.0, 0.0),
    "skid_short1": ("skid_brake", 1.0, None, 0.7, 0.0, 0.0),
    "skid_front_only": ("skid_brake", 4.0, None, 0.7, 0.0, 0.0, ("front",)),
}
for _name, (_base, _dur, *_rest) in EDGE.items():
    I.KINDS.setdefault(_name, dict(I.KINDS[_base], dur=_dur, ru=_name))
PRE_S, POST_S = 40.0, 60.0


def _apply_edge(seg, kind, t0):
    base, dur, fn, lo, arg, lag, *keys = EDGE[kind]
    b = {k: v.copy() for k, v in seg.items()}
    for key in (keys[0] if keys else ("front", "rear")):
        x = b[key]
        s0 = t0 + (lag if key == "rear" else 0.0)
        w = (x[:, 1] >= s0) & (x[:, 1] < t0 + dur)
        tr = x[w, 1] - t0
        x[w, 2] *= np.full(int(w.sum()), lo) if fn is None else fn(tr, dur, lo, arg)
    return b, dict(kind=kind, t0=t0, dur=dur)


def _apply(seg, kind, t0, dur, seed):
    """tools/inject.py для своих видов, вариантам — свой множитель."""
    if kind in EDGE:
        return _apply_edge(seg, kind, t0)
    if kind not in VARIANTS:
        return I.apply(seg, kind, t0, dur, seed=seed)
    base, f, dur = VARIANTS[kind]
    b = {k: v.copy() for k, v in seg.items()}
    rng = np.random.default_rng(seed)
    for key in ("front", "rear"):
        x = b[key]
        w = (x[:, 1] >= t0) & (x[:, 1] < t0 + dur)
        if base == "noise":
            x[w, 2] += rng.normal(0.0, f * I.KMH, int(w.sum()))
        else:
            x[w, 2] *= f
    return b, dict(kind=kind, t0=t0, dur=dur)
REC_TOL, REC_HOLD = 0.1, 3.0
PH = ("stand", "tract", "coast", "brake")


def _cut(a, lo, hi):
    out = {}
    for k, v in a.items():
        v = np.asarray(v)
        if k in ("mfix", "rfix"):
            out[k] = v[:0]
        elif len(v):
            out[k] = v[(v[:, 1] >= lo) & (v[:, 1] <= hi)]
        else:
            out[k] = v
    return out


def _run(evs, p, node, naive=False, want_runner=False):
    r = R.make_naive(p, node, None) if naive else R.make_runner(p, node, None)[0]
    peak = [0.0]
    core = getattr(r, "core", None)
    if want_runner and core is not None and hasattr(core, "_noise"):
        noise = core._noise

        def tracked(a, za, gap):          # наибольшая оценка шума за прогон
            noise(a, za, gap)
            peak[0] = max(peak[0], float(core.nz_sigma[a]))
        core._noise = tracked
    (O, crash), = R.replay(evs, [r], node)
    r.noise_peak = peak[0]
    return (O, crash, r) if want_runner else (O, crash)


def _pairs(O, a, lo, hi):
    tg, vg = M.speed_ref(a, "master")
    m = (tg >= lo) & (tg < hi)
    tg, vg = tg[m], vg[m]
    j, ok = M.nearest(O["T"], tg)
    e = np.full(len(tg), np.nan)
    e[ok] = O["V"][j[ok]] - vg[ok]
    sv = np.full(len(tg), np.nan)
    if "SV" in O:
        sv[ok] = O["SV"][j[ok]]
    return tg, vg, e, sv


def _stats(e, sv=None):
    f = np.isfinite(e)
    if not f.any():
        return dict(n=0)
    d = dict(n=int(f.sum()), mae=float(np.mean(np.abs(e[f]))), bias=float(np.mean(e[f])),
             max=float(np.max(np.abs(e[f]))))
    if sv is not None and np.isfinite(sv[f]).any():
        d["cov2s"] = float(np.mean(np.abs(e[f]) <= 2 * sv[f]))
    return d


def _recovery(Oi, Oc, t1):
    Ti, Vi = Oi["T"], Oi["V"]
    j, ok = M.nearest(Oc["T"], Ti, tol=0.026)
    good = ok & (np.abs(Vi - Oc["V"][j]) <= REC_TOL)
    start = None
    for i in np.flatnonzero(Ti >= t1):
        if good[i]:
            if start is None:
                start = i
            if Ti[i] - Ti[start] >= REC_HOLD:
                return max(0.0, float(Ti[start] - t1))
        else:
            start = None
    return None


def run_inject(task):
    bag, kinds, sheet, ov = task["bag"], task["kinds"], task["sheet"], task["ov"]
    a = bagio.load(bag)
    p = R.make_params(sheet, ov)
    node = sheet["node"]
    out = []
    for kind in kinds:
        t0, dur = I.choose_window(a, kind)
        if t0 is None:
            continue
        ev_s = I.eval_window(kind)
        t1, te = t0 + dur, t0 + ev_s
        seg = _cut(a, t0 - PRE_S, te + POST_S)
        b, info = _apply(seg, kind, t0, dur, I.seed_for(bag, kind))
        Oc, _ = _run(R.events(seg, "0"), p, node)
        Oi, crash = _run(R.events(b, "0"), p, node)
        On, _ = _run(R.events(b, "0"), p, node, naive=True)
        row = dict(bag=bag, kind=kind, t0=t0, crash=crash)
        for name, O in (("model", Oi), ("naive", On), ("clean", Oc)):
            d = {}
            for w, (lo, hi) in dict(before=(t0 - 30.0, t0), during=(t0, te),
                                    after=(te, te + POST_S)).items():
                _, _, e, sv = _pairs(O, seg, lo, hi)
                d[w] = _stats(e, sv if name != "naive" else None)
            if name == "model":
                m = (O["T"] >= t0) & (O["T"] < te)
                d["slip"] = float(O["SLIP"][m].mean()) if m.any() else None
                d["valid"] = float(O["VALID"][m].mean()) if m.any() else None
                d["amb"] = float(O["AMB"][m].mean()) if m.any() else None
                d["sv_max"] = float(np.nanmax(O["SV"][m])) if m.any() else None
                d["rec"] = _recovery(Oi, Oc, t1)
                # «уверенно неверно»: valid при ошибке больше 1 м/с против GNSS
                # (юз, перешедший в блокировку, прежде давал v = 0, valid)
                tg, _, e, _ = _pairs(O, seg, t0, te)
                j, ok = M.nearest(O["T"], tg)
                bad = ok & (np.abs(np.nan_to_num(e)) > 1.0)
                bad[ok] &= O["VALID"][j[ok]]
                d["valid_bad"] = float(bad.mean()) if len(tg) else None
                ma = (O["T"] >= te) & (O["T"] < te + POST_S)
                d["slip_after"] = float(O["SLIP"][ma].mean()) if ma.any() else None
                mb = (O["T"] >= t0 - 30.0) & (O["T"] < t0)
                d["slip_before"] = float(O["SLIP"][mb].mean()) if mb.any() else None
            if name == "clean":
                m = (O["T"] >= t0) & (O["T"] < te)
                d["slip"] = float(O["SLIP"][m].mean()) if m.any() else None
            row[name] = d
        out.append(row)
    return out


def run_clean(task):
    bag, sheet, ov = task["bag"], task["sheet"], task["ov"]
    a = bagio.load(bag)
    p = R.make_params(sheet, ov)
    node = sheet["node"]
    evs = R.events(a, "0")
    O, crash, r = _run(evs, p, node, want_runner=True)
    tg, vg, e, sv = _pairs(O, a, -np.inf, np.inf)
    ph = M.phases(a, tg, vg)
    j, ok = M.nearest(O["T"], tg)
    slip = np.zeros(len(tg), bool)
    slip[ok] = O["SLIP"][j[ok]]
    res = dict(bag=bag, crash=crash, n_out=int(len(O["T"])),
               slip_frac=float(O["SLIP"].mean()), valid_frac=float(O["VALID"].mean()),
               amb_frac=float(O["AMB"].mean()),
               n_slip_all=int(getattr(r.core, "n_slip_all", 0)),
               n_override=int(getattr(r.core, "n_override", 0)),
               noise_max=float(getattr(r, "noise_peak", 0.0)))
    f = np.isfinite(e)
    acc = {}
    for k, sel in [("all", f)] + [(PH[i], f & (ph == i)) for i in range(4)]:
        ee, ss = e[sel], sv[sel]
        acc[k] = dict(n=int(sel.sum()), sabs=float(np.sum(np.abs(ee))), s=float(np.sum(ee)),
                      s2=float(np.sum(ee * ee)), in2=int(np.sum(np.abs(ee) <= 2 * ss)),
                      slip=int(np.sum(slip[sel])))
    res["acc"] = acc
    return res


HORIZONS = (1.0, 2.0, 4.0, 6.0)


def run_openloop(task):
    """Разомкнутый прогноз скорости по модели листа из точек каждые 7 с
    (скорость GNSS > 3 м/с, 30 с от начала): привод с запаздыванием и
    инерцией по ручке, номинальное сцепление, без возмущения."""
    from tram_state_estimator import estimator_core as core
    bag, sheet, ov = task["bag"], task["sheet"], task["ov"]
    a = bagio.load(bag)
    p = R.make_params(sheet, ov)
    tg, vg = M.speed_ref(a, "master")
    c = a["cmd"][np.argsort(a["cmd"][:, 1], kind="stable")]
    out = {ph: {h: [] for h in HORIZONS} for ph in ("tract", "brake", "coast")}
    n_pre = int(round(1.0 / p.dt))
    n_run = int(round(max(HORIZONS) / p.dt))
    for t in np.arange(tg[0] + 30.0, tg[-1] - max(HORIZONS) - 1.0, 7.0):
        v = float(vg[np.searchsorted(tg, t)])
        if v < 3.0:
            continue
        tt = t - n_pre * p.dt + np.arange(n_pre + n_run + 1) * p.dt
        nh = np.nan_to_num(M.zoh(c[:, 1], c[:, 2], tt), nan=0.0)
        es = core.Estimator(p)
        for k in range(n_pre):
            es._drive_model(nh[k])
        u0 = es.u_filt
        ph = "tract" if u0 > p.T_dead else ("brake" if u0 < -p.T_dead else "coast")
        for k in range(n_pre, n_pre + n_run):
            u = es._drive_model(nh[k])
            acc = (core.body_force(u, v, 1.0, 1.0, p.mu_nominal, p)
                   - core.resistance(v, p)) / p.M_nom
            v = max(0.0, v + (0.0 if v <= 0.0 and acc < 0.0 else acc) * p.dt)
            T = (k - n_pre + 1) * p.dt
            for h in HORIZONS:
                if abs(T - h) < 1e-6:
                    j = np.searchsorted(tg, t + h)
                    if j < len(tg) and abs(tg[j] - t - h) < 0.1:
                        out[ph][h].append(v - float(vg[j]))
    return out


def report_openloop(rows):
    lines = ["разомкнутый прогноз против GNSS: фаза | горизонт | n | СКО, м/с | смещение"]
    pooled = {h: [] for h in HORIZONS}
    for ph in ("tract", "brake", "coast"):
        for h in HORIZONS:
            e = np.array([x for r in rows for x in r[ph][h]])
            pooled[h] += list(e)
            if len(e):
                lines.append(f"  {ph} | {h:g} с | {len(e)} | {np.sqrt(np.mean(e * e)):.3f} | "
                             f"{np.mean(e):+.3f}")
    rms = {h: float(np.sqrt(np.mean(np.square(pooled[h])))) for h in HORIZONS}
    # СКО(T)² = СКО(1)² + (σ_a·(T − 1))²: рост после первой секунды (в первой —
    # шум эталона и начальная ошибка)
    k = [np.sqrt(max(rms[h] ** 2 - rms[1.0] ** 2, 0.0)) / (h - 1.0) for h in HORIZONS if h > 1.0]
    lines.append("  все фазы: " + ", ".join(f"{h:g} с {rms[h]:.3f}" for h in HORIZONS)
                 + f"; рост σ_a по горизонтам 2/4/6 с: " + ", ".join(f"{x:.3f}" for x in k)
                 + " м/с²")
    return chr(10).join(lines)


def _pool(fn, tasks, workers):
    if workers <= 1:
        return [fn(t) for t in tasks]
    with ProcessPoolExecutor(workers) as ex:
        return list(ex.map(fn, tasks))


def summarize(res):
    lines = []
    inj = res.get("inject", [])
    if inj:
        lines.append("вид | n | MAE мод. во время | MAE базы | MAE чист. | после мод/база | "
                     "slip во время | slip чист. | valid | ±2σ | восст. медиана/макс, с | "
                     "slip после | наиб. ошибка | valid и ошибка > 1 м/с")
        for kind in dict.fromkeys(r["kind"] for r in inj):
            rs = [r for r in inj if r["kind"] == kind]

            def mean(get):
                xs = [get(r) for r in rs]
                xs = [x for x in xs if x is not None and np.isfinite(x)]
                return float(np.mean(xs)) if xs else float("nan")
            rec = [r["model"]["rec"] for r in rs]
            recf = [x for x in rec if x is not None]
            lines.append(
                f"{kind} | {len(rs)} | {mean(lambda r: r['model']['during'].get('mae')):.3f} | "
                f"{mean(lambda r: r['naive']['during'].get('mae')):.3f} | "
                f"{mean(lambda r: r['clean']['during'].get('mae')):.3f} | "
                f"{mean(lambda r: r['model']['after'].get('mae')):.3f}/"
                f"{mean(lambda r: r['naive']['after'].get('mae')):.3f} | "
                f"{mean(lambda r: r['model']['slip']):.0%} | {mean(lambda r: r['clean']['slip']):.0%} | "
                f"{mean(lambda r: r['model']['valid']):.0%} | "
                f"{mean(lambda r: r['model']['during'].get('cov2s')):.0%} | "
                f"{(np.median(recf) if recf else float('nan')):.1f}/{(max(recf) if recf else float('nan')):.1f}"
                f" (нет: {sum(x is None for x in rec)}) | "
                f"{mean(lambda r: r['model'].get('slip_after')):.1%} | "
                f"{max((r['model']['during'].get('max') or 0.0) for r in rs):.2f} | "
                f"{mean(lambda r: r['model'].get('valid_bad')):.1%} | "
                f"лучше базы {sum((r['model']['during'].get('mae') or 9) < (r['naive']['during'].get('mae') or 9) for r in rs)}/{len(rs)}")
    cl = res.get("clean", [])
    if cl:
        tot = {k: dict(n=0, sabs=0.0, s=0.0, s2=0.0, in2=0, slip=0) for k in ("all",) + PH}
        for r in cl:
            for k, d in r["acc"].items():
                for q in d:
                    tot[k][q] += d[q]
        lines.append("чистые: фаза | n | MAE | смещение | RMSE | ±2σ | slip")
        for k, d in tot.items():
            n = max(d["n"], 1)
            lines.append(f"  {k} | {d['n']} | {d['sabs'] / n:.4f} | {d['s'] / n:+.4f} | "
                         f"{np.sqrt(d['s2'] / n):.4f} | {d['in2'] / n:.1%} | {d['slip'] / n:.2%}")
        lines.append(f"  прогонов {len(cl)}, падений {sum(r['crash'] is not None for r in cl)}, "
                     f"срывов всех осей {sum(r.get('n_slip_all', 0) for r in cl)} "
                     f"(в {sum(r.get('n_slip_all', 0) > 0 for r in cl)} прогонах), "
                     f"доля slip {np.mean([r['slip_frac'] for r in cl]):.3%}, "
                     f"наибольшая оценка шума {max(r.get('noise_max', 0.0) for r in cl):.3f} м/с")
    return "\n".join(lines)


def compare(paths, names=None):
    """Сводка нескольких прогонов стенда рядом (до → после): по видам,
    только общие для всех файлов окна (bag, вид)."""
    res = [json.loads(Path(p).read_text(encoding="utf-8")) for p in paths]
    names = names or [Path(p).stem for p in paths]
    keys = [{(r["bag"], r["kind"]): r for r in d.get("inject", [])} for d in res]
    common = set(keys[0]).intersection(*keys[1:])
    kinds = list(dict.fromkeys(k for b, k in (x for x in
                                              ((r["bag"], r["kind"]) for r in res[0].get("inject", [])))
                               if (b, k) in common))

    def m(rows, get):
        xs = [get(r) for r in rows]
        xs = [x for x in xs if x is not None and np.isfinite(x)]
        return float(np.mean(xs)) if xs else float("nan")

    def cell(vals, fmt):
        return " → ".join(fmt.format(v) for v in vals)
    lines = ["сравнение: " + " → ".join(names),
             "вид | окон | MAE модели во время | MAE базы | MAE после окна (база) | slip во время | "
             "slip после окна | valid и ошибка > 1 м/с | ±2σ | наиб. ошибка | восст. макс, с | лучше базы"]
    for kind in kinds:
        bags = sorted(b for b, k in common if k == kind)
        rs = [[kk[(b, kind)] for b in bags] for kk in keys]
        mae = [m(r, lambda x: x["model"]["during"].get("mae")) for r in rs]
        aft = [m(r, lambda x: x["model"]["after"].get("mae")) for r in rs]
        rec = [max([x["model"]["rec"] for x in r if x["model"]["rec"] is not None] or [np.nan])
               for r in rs]
        better = [sum((x["model"]["during"].get("mae") or 9) < (x["naive"]["during"].get("mae") or 9)
                      for x in r) for r in rs]
        lines.append(" | ".join([
            kind, str(len(bags)), cell(mae, "{:.3f}"),
            f"{m(rs[0], lambda x: x['naive']['during'].get('mae')):.3f}",
            cell(aft, "{:.3f}") + f" ({m(rs[0], lambda x: x['naive']['after'].get('mae')):.3f})",
            cell([m(r, lambda x: x["model"]["slip"]) for r in rs], "{:.0%}"),
            cell([m(r, lambda x: x["model"].get("slip_after")) for r in rs], "{:.1%}"),
            cell([m(r, lambda x: x["model"].get("valid_bad")) for r in rs], "{:.1%}"),
            cell([m(r, lambda x: x["model"]["during"].get("cov2s")) for r in rs], "{:.0%}"),
            cell([max((x["model"]["during"].get("max") or 0.0) for x in r) for r in rs], "{:.2f}"),
            cell(rec, "{:.1f}"),
            cell(better, "{:d}") + f" из {len(bags)}"]))
    cl = [d.get("clean") for d in res]
    if all(cl):
        lines.append("чистые (все фазы): MAE | RMSE | ±2σ | срывов всех осей (прогонов)")
        for name, c in zip(names, cl):
            n = sum(r["acc"]["all"]["n"] for r in c)
            sabs = sum(r["acc"]["all"]["sabs"] for r in c)
            s2 = sum(r["acc"]["all"]["s2"] for r in c)
            in2 = sum(r["acc"]["all"]["in2"] for r in c)
            lines.append(f"  {name}: {sabs / n:.4f} | {np.sqrt(s2 / n):.4f} | {in2 / n:.1%} | "
                         f"{sum(r.get('n_slip_all', 0) for r in c)} "
                         f"({sum(r.get('n_slip_all', 0) > 0 for r in c)})")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", default="train", help="train | holdout | список через запятую")
    ap.add_argument("--n", type=int, default=0, help="только первые n прогонов (по порядку)")
    ap.add_argument("--kinds", default=",".join(KINDS))
    ap.add_argument("--inject", action="store_true")
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--openloop", action="store_true")
    ap.add_argument("--sheet", default="eval")
    ap.add_argument("--set", default="")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", default="")
    ap.add_argument("--report", default="")
    ap.add_argument("--compare", nargs="+", default=None,
                    help="сводки стенда рядом: до.json после.json [...]")
    args = ap.parse_args()
    if args.compare:
        print(compare(args.compare))
        return
    if args.report:
        print(summarize(json.loads(Path(args.report).read_text(encoding="utf-8"))))
        return
    split = json.loads((ROOT / "tools" / "split.json").read_text(encoding="utf-8"))
    if args.runs in ("train", "holdout"):
        ids = split["train" if args.runs == "train" else "holdout_scored"]
    else:
        ids = [b for b in args.runs.split(",") if b]
    ids = [b for b in ids if (bagio.CACHE / f"{b}.npz").exists()]
    # только прогоны с эталоном скорости
    ids = [b for b in ids if len(np.load(bagio.CACHE / f"{b}.npz")["mvel"])]
    if args.n:
        ids = ids[:args.n]
    sheet = R.resolve_sheet(args.sheet)
    ov = R.parse_overrides(args.set)
    core_ov, node_ov = R.split_overrides(ov)
    sheet["node"].update(node_ov)
    res = dict(meta=dict(runs=ids, set=args.set, sheet=sheet["label"]))
    t = time.time()
    if args.inject:
        kinds = []
        for k in args.kinds.split(","):
            kinds += (list(EDGE) if k == "edge" else list(KINDS) if k == "std"
                      else list(VARIANTS) if k == "variants" else [k])
        rows = _pool(run_inject, [dict(bag=b, kinds=kinds, sheet=sheet, ov=core_ov) for b in ids],
                     args.workers)
        res["inject"] = [x for r in rows for x in r]
    if args.clean:
        res["clean"] = _pool(run_clean, [dict(bag=b, sheet=sheet, ov=core_ov) for b in ids],
                             args.workers)
    if args.openloop:
        rows = _pool(run_openloop, [dict(bag=b, sheet=sheet, ov=core_ov) for b in ids],
                     args.workers)
        res["openloop"] = [{ph: {str(h): v for h, v in d.items()} for ph, d in r.items()}
                           for r in rows]
        print(report_openloop(rows))
    res["meta"]["wall_s"] = round(time.time() - t, 1)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(res, ensure_ascii=False, default=float), encoding="utf-8")
    print(summarize(res))
    print(f"время {res['meta']['wall_s']} с, прогонов {len(ids)}")


if __name__ == "__main__":
    main()
