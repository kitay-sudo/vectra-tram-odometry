#!/usr/bin/env python3
"""Независимая реализация метрик ядра на реальных данных: метрики «как у судьи»
и наивная база «только колесо». С ней сверяются tools/eval_selftest.py
(самотест оценки) и tools/export_replay.py (ошибка вдоль и поперёк пути);
analysis/calib_eval.py берёт из неё метрики подгонки.

Порядок событий, GNSS только первые 3 с и пары по ближайшей метке <= 0,05 с —
из analysis/bagio.py и analysis/evaluate.py; Runner, Position и TrackMap —
из пакета tram_state_estimator прямо из репозитория.

Запуск — в образе vectra/tram:dev из корня репозитория:

  # метрики (held-out / все прогоны), конфиг ядра json (как evaluate.py) или yaml (как нода)
  python3 tools/core_metrics.py metrics --set val --cfg json --map train
  python3 tools/core_metrics.py metrics --set all --cfg yaml --map train
  # стоимость шага (мс на выход), память, детерминизм
  python3 tools/core_metrics.py timing --bags 30618_3e9f4952,30639_d3c43d69
  # инъекция отказа всех датчиков в реальную запись
  python3 tools/core_metrics.py inject --bags 30618_3e9f4952
  # скачок метки времени во входе (устойчивость связки)
  python3 tools/core_metrics.py probe

Сырые результаты — out/core/*.json|csv.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import resource
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
sys.path.insert(0, str(ROOT / "analysis"))
sys.path.insert(0, str(PKG))

import bagio                                                     # noqa: E402
import evaluate as E                                             # noqa: E402
from tram_state_estimator.estimator_core import (                # noqa: E402
    LINEAR_UNITS, MODE_NAMES, STANDSTILL, Params)
from tram_state_estimator.runner import Position, Runner, Enu    # noqa: E402
from tram_state_estimator.track_map import TrackMap              # noqa: E402

OUT = ROOT / "out" / "core"
MAPS = {"train": ROOT / "analysis" / "cache" / "track_map_train.npz",
        "pkg": PKG / "config" / "track_map.npz",
        "none": None}
NODE_ONLY = {"wheel_timeout_s", "handle_timeout_s", "init_window_s", "map_file",
             "origin_lat", "origin_lon", "origin_alt", "frame_id",
             "child_frame_id"}
V_STAND_GNSS = 0.2      # м/с: ниже — фаза «стоянка» по GNSS
V_FALSE_SS = 0.5        # м/с: STANDSTILL при GNSS выше — ложная стоянка
PHASES = ("standstill", "traction", "coast", "brake")
DUP = "30618_f3b8c99b"  # дубликат 30618_21dd3af3 (MODEL.md 0.1)


# ------------------------------------------------------------------ конфиг

CFGS = ("json", "yaml", "json_nocreep", "yaml_nocreep")


def load_params(cfg):
    """json — config/tram_calibration.json (так считает analysis/evaluate.py);
    yaml — config/tram.yaml (так работает нода). *_nocreep — эксперимент:
    заготовочные c_creep, c_creep_drag обнулены (масштаб колёс уже откалиброван
    по GNSS, крип-заглушка сверху даёт систематический сдвиг)."""
    from dataclasses import replace
    base = cfg.split("_")[0]
    if base == "json":
        p = E.tram_params()                      # то, что берёт evaluate.py
    else:
        import yaml
        with open(PKG / "config" / "tram.yaml", encoding="utf-8") as fh:
            got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
        from dataclasses import fields as _fields
        core = {f.name for f in _fields(Params)}     # параметры ноды вне ядра — мимо
        p = Params.from_dict({k: v for k, v in got.items() if k in core})
    if cfg.endswith("_nocreep"):
        p = replace(p, c_creep=0.0, c_creep_drag=0.0)
    return p


def load_map(name):
    f = MAPS[name]
    return TrackMap.load(f) if f is not None and f.exists() else None


def val_ids():
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    return list(dm["val"])


def all_ids():
    return sorted(p.stem for p in (ROOT / "analysis" / "cache").glob("3*.npz"))


# ------------------------------------------------------ наивная база

class NaiveRunner:
    """«Только колесо»: скорость — среднее последних показаний обеих тележек
    (причинно, без заглядывания вперёд) × meas_scale / 3,6; путь — интеграл на
    той же сетке 50 мс по меткам сообщений; положение — ТА ЖЕ машинерия
    Position/TrackMap, та же выставка по GNSS и та же привязка к остановкам
    (стоянка = наивная скорость < v_standstill дольше 8 с)."""

    def __init__(self, p, tmap, wheel_timeout=1.0, stop_dwell=8.0):
        self.p, self.dt = p, p.dt
        self.k = p.meas_scale * LINEAR_UNITS[p.meas_units]
        self.last = [None, None]
        self.t_w = [-math.inf, -math.inf]
        self.t = None
        self.s = 0.0
        self.v = 0.0
        self.s0 = None
        self.pos = Position(tmap, None)
        self._dwell = 0.0
        self.wheel_timeout = wheel_timeout
        self.stop_dwell = stop_dwell

    def on_wheel(self, i, stamp, value):
        out = self._advance(stamp)
        self.last[i] = value
        self.t_w[i] = stamp
        return out

    def on_handle(self, stamp, position):
        return self._advance(stamp)

    def on_fix(self, stamp, antenna, lat, lon, alt):
        out = self._advance(stamp)
        moved = (self.s - self.s0) if self.s0 is not None else 0.0
        was = self.pos.ready
        self.pos.on_fix(stamp, antenna, lat, lon, alt, moved)
        if self.pos.ready and was:
            self.s0 = self.s
        return out

    def _advance(self, stamp):
        if self.t is None:
            self.t = stamp
            return []
        outs = []
        while self.t + self.dt <= stamp + 1e-9:
            self.t += self.dt
            outs.append(self._step())
        return outs

    def _step(self):
        t = self.t
        vals = [self.last[i] for i in (0, 1)
                if self.last[i] is not None and t - self.t_w[i] <= self.wheel_timeout]
        if vals:
            self.v = max(0.0, float(np.mean(vals)) * self.k)
        self.s += self.v * self.dt
        if self.pos.ready:
            if self.s0 is None:
                self.s0 = self.s
            x, y, z = self.pos.xyz(self.s - self.s0)
            standing = bool(vals) and self.v < self.p.v_standstill
            self._dwell = self._dwell + self.dt if standing else 0.0
            if self._dwell >= self.stop_dwell > self._dwell - self.dt:
                self.pos.on_stop(self.s - self.s0)
                x, y, z = self.pos.xyz(self.s - self.s0)
        else:
            x, y, z = self.s, 0.0, 0.0
        return dict(stamp=t, v=self.v, s=self.s, x=x, y=y, z=z)


# ------------------------------------------------------ прогон

def replay(a, runners):
    """Прогон событий bag (порядок записи, GNSS первые 3 с — E.events) через
    несколько связок сразу. Возвращает списки выходов каждой связки."""
    outs = [[] for _ in runners]
    for tb, kind, i, th, val in E.events(a):
        for k, r in enumerate(runners):
            if kind == 0:
                o = r.on_wheel(i, th, val)
            elif kind == 1:
                o = r.on_handle(th, val)
            else:
                o = r.on_fix(th, i, *val)
            outs[k] += o
    return outs


def arr(outs, key, dtype=float):
    return np.array([o[key] for o in outs], dtype=dtype)


# ------------------------------------------------------ геометрия

EXT = 1000.0   # м: продление траектории за концы, чтобы проекция не обрезалась


def polyline(ref_xy, min_step=1.0):
    """Траектория GNSS, прореженная до шага >= min_step (гасит дрожание на
    стоянке) и продлённая по касательной на EXT м за оба конца (иначе оценка,
    ушедшая за последнюю точку, проецируется в конец и даёт along = 0).
    kept_of[i] — индекс (в продлённом массиве) последней точки до фикса i.
    S — дуговая координата, 0 в первой точке GNSS; path — длина пути GNSS."""
    keep, last = [], None
    kept_of = np.full(len(ref_xy), -1)
    for i, (x, y) in enumerate(ref_xy):
        if math.isfinite(x) and math.isfinite(y):
            if last is None or math.hypot(x - ref_xy[last, 0], y - ref_xy[last, 1]) >= min_step:
                keep.append(i)
                last = i
        kept_of[i] = len(keep) - 1
    P = ref_xy[keep]
    if len(P) < 2:
        return P, np.zeros((0, 2)), np.zeros(0), np.zeros(1), kept_of, 0.0
    u0 = (P[1] - P[0]) / np.linalg.norm(P[1] - P[0])
    u1 = (P[-1] - P[-2]) / np.linalg.norm(P[-1] - P[-2])
    P = np.vstack([P[0] - EXT * u0, P, P[-1] + EXT * u1])
    kept_of = np.where(kept_of >= 0, kept_of + 1, -1)
    seg = np.diff(P, axis=0)
    L = np.hypot(seg[:, 0], seg[:, 1])
    S = np.concatenate([[0.0], np.cumsum(L)]) - EXT
    path = float(S[-2] - S[1])
    return P, seg, L, S, kept_of, path


def median5(xy):
    """Скользящая медиана по 5 точкам по каждой координате (края — как есть)."""
    out = xy.copy()
    if len(xy) >= 5:
        from numpy.lib.stride_tricks import sliding_window_view
        for c in range(xy.shape[1]):
            out[2:-2, c] = np.median(sliding_window_view(xy[:, c], 5), axis=1)
    return out


def along_cross(ref_xy, idx, est_xy, W=1000.0, max_d=60.0):
    """Ошибка вдоль и поперёк эталонной траектории для фиксов idx.

    s_ref — дуговая координата эталонной точки; оценка проецируется на
    отрезки траектории в окне ±W м от s_ref с тем же направлением движения
    (встречный проход по тому же месту отсекается). along = s_проекции − s_ref,
    cross — знаковое расстояние до траектории (+ слева по ходу); cross равен
    расстоянию до ближайшей из почти равноудалённых проекций, т. е. с точностью
    до 5 м — расстоянию до траектории. Оценка дальше max_d от траектории
    (ушла на чужую ветку) — along/cross не определены (NaN, считаются отдельно)."""
    # Эталон для разложения вдоль/поперёк — медиана по 5 фиксам (~0,7 с): одиночный
    # выброс GNSS иначе даёт «шип» в траектории, на который ложится проекция.
    # 3D-ошибка считается по сырому эталону (как у судьи).
    ref_xy = median5(ref_xy)
    P, seg, L, S, kept_of, path = polyline(ref_xy)
    n = len(idx)
    along = np.full(n, np.nan)
    cross = np.full(n, np.nan)
    sref = np.full(n, np.nan)
    if len(seg) < 3:
        return along, cross, sref, 0.0
    nseg = len(seg)
    U = seg / L[:, None]
    for q, i in enumerate(idx):
        k = kept_of[i]
        if k < 0:
            continue
        k = min(k, nseg - 1)
        d = ref_xy[i] - P[k]
        s_r = S[k] + float(np.clip(d @ U[k], 0.0, L[k]))
        sref[q] = s_r
        lo = max(int(np.searchsorted(S, s_r - W)) - 1, 0)
        hi = min(int(np.searchsorted(S, s_r + W)) + 1, nseg)
        cand = np.arange(lo, hi)
        e = est_xy[q]
        de = e - P[cand]
        t = np.clip(np.einsum("ij,ij->i", de, seg[cand]) / (L[cand] ** 2), 0.0, 1.0)
        c = P[cand] + t[:, None] * seg[cand]
        dist = np.hypot(e[0] - c[:, 0], e[1] - c[:, 1])
        dist[U[cand] @ U[k] <= 0.0] = np.inf       # встречное направление
        dmin = float(dist.min())
        if not math.isfinite(dmin) or dmin > max_d:
            continue
        # Кандидаты — локальные минимумы расстояния вдоль траектории (основания
        # перпендикуляров). На петле маршрута тот же участок в том же направлении
        # встречается в окне дважды: из почти равноудалённых (dmin + 5 м)
        # минимумов берётся ближайший ВДОЛЬ пути к эталонной точке.
        dl = np.concatenate([[np.inf], dist[:-1]])
        dr = np.concatenate([dist[1:], [np.inf]])
        near = np.flatnonzero((dist <= dl) & (dist <= dr) & (dist <= dmin + 5.0))
        al_c = S[cand[near]] + t[near] * L[cand[near]] - s_r
        m = int(near[int(np.argmin(np.abs(al_c)))])
        i2 = cand[m]
        along[q] = S[i2] + t[m] * L[i2] - s_r
        sgn = np.sign(seg[i2, 0] * de[m, 1] - seg[i2, 1] * de[m, 0]) or 1.0
        cross[q] = sgn * dist[m]
    return along, cross, sref, path


# ------------------------------------------------------ метрики одного прогона

def zoh(t_src, v_src, t_q):
    """Последнее значение источника на момент t_q (причинно)."""
    o = np.argsort(t_src, kind="stable")
    ts, vs = t_src[o], v_src[o]
    j = np.searchsorted(ts, t_q, side="right") - 1
    out = np.where(j >= 0, vs[np.clip(j, 0, None)], np.nan)
    return out


def run_one(args):
    b, cfg, mapname = args
    t_wall = time.time()
    p = load_params(cfg)
    a = bagio.load(b)
    if len(a["mvel"]) < 50 or len(a["mfix"]) < 50:
        return b, None
    r = Runner(p, track_map=load_map(mapname))
    nv = NaiveRunner(p, load_map(mapname))
    mo, no = replay(a, [r, nv])
    if len(mo) < 10:
        return b, None
    T = arr(mo, "stamp")
    V = arr(mo, "v")
    X = np.array([[o["x"], o["y"], o["z"]] for o in mo])
    SV = arr(mo, "sigma_v")
    SS = arr(mo, "sigma_s")
    MODE = arr(mo, "mode", int)
    VALID = arr(mo, "valid", bool)
    AMB = arr(mo, "ambiguous", bool)
    SLIP = arr(mo, "slip", bool)
    KT, KB, MU = arr(mo, "k_t"), arr(mo, "k_b"), arr(mo, "mu")
    Tn = arr(no, "stamp")
    Vn = arr(no, "v")
    Xn = np.array([[o["x"], o["y"], o["z"]] for o in no])
    assert np.allclose(T, Tn), "сетки модели и наивной базы разошлись"

    # ---------------- скорость
    g = a["mvel"]
    j, ok = E.nearest(T, g[:, 1])
    vg_all = np.hypot(g[:, 2], g[:, 3])
    jj, vg, tg = j[ok], vg_all[ok], g[ok, 1]
    ev = V[jj] - vg
    en = Vn[jj] - vg
    f, rr = a["front"], a["rear"]
    nc = 0.5 * (np.interp(tg, f[:, 1], f[:, 2]) + np.interp(tg, rr[:, 1], rr[:, 2])) / E.KMH
    enc = nc - vg                                   # как в evaluate.py: интерп., без meas_scale
    n_h = zoh(a["cmd"][:, 1], a["cmd"][:, 2], tg)
    n_h = np.nan_to_num(n_h, nan=0.0)
    phase = np.where(vg < V_STAND_GNSS, 0, np.where(n_h > 0, 1, np.where(n_h < 0, 3, 2)))
    # выбросы самого эталона: GNSS расходится с ОБЕИМИ тележками (интерп.) > 1 м/с
    fz = np.interp(tg, f[:, 1], f[:, 2]) / E.KMH * p.meas_scale
    rz = np.interp(tg, rr[:, 1], rr[:, 2]) / E.KMH * p.meas_scale
    ref_glitch = (np.abs(fz - vg) > 1.0) & (np.abs(rz - vg) > 1.0) & (np.abs(fz - rz) < 0.5)

    # ---------------- положение
    m = a["mfix"]
    enu = Enu(m[0, 2], m[0, 3], m[0, 4])
    ref = np.array([enu.fwd(la, lo, al) for la, lo, al in m[:, 2:5]])
    j2, ok2 = E.nearest(T, m[:, 1])
    idx = np.flatnonzero(ok2)
    jp = j2[ok2]
    d3 = np.linalg.norm(X[jp] - ref[idx], axis=1)
    d2 = np.linalg.norm(X[jp, :2] - ref[idx, :2], axis=1)
    d3n = np.linalg.norm(Xn[jp] - ref[idx], axis=1)
    d2n = np.linalg.norm(Xn[jp, :2] - ref[idx, :2], axis=1)
    al, cr, sref, Lpath = along_cross(ref[:, :2], idx, X[jp, :2])
    aln, crn, _, _ = along_cross(ref[:, :2], idx, Xn[jp, :2])
    ssp = SS[jp]
    fix_bad = int(np.sum(m[idx, 5] < 0))

    res = dict(
        bag=b, vehicle=b.split("_")[0], cfg=cfg, map=mapname,
        dur_s=float(T[-1] - T[0]), n_out=len(T), rate_hz=float(len(T) / max(T[-1] - T[0], 1e-9)),
        path_m=Lpath, anchors=int(r.pos.anchors), anchors_naive=int(nv.pos.anchors),
        pos_ready=bool(r.pos.ready),
        kt_min=float(KT.min()), kt_max=float(KT.max()), kt_end=float(KT[-1]),
        kb_min=float(KB.min()), kb_max=float(KB.max()), kb_end=float(KB[-1]),
        k_at_bound=float(np.mean((KT <= p.k_min + 1e-6) | (KT >= p.k_max - 1e-6)
                                 | (KB <= p.k_min + 1e-6) | (KB >= p.k_max - 1e-6))),
        mu_min=float(MU.min()),
        frac_valid=float(VALID.mean()), frac_amb=float(AMB.mean()), frac_slip=float(SLIP.mean()),
        mode_frac={MODE_NAMES[k]: float(np.mean(MODE == k)) for k in range(len(MODE_NAMES))},
        fix_status_neg=fix_bad,
        wall_s=time.time() - t_wall,
        maxrss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
    )
    samples = dict(
        ev=ev.astype(np.float32), en=en.astype(np.float32), enc=enc.astype(np.float32),
        vg=vg.astype(np.float32), sv=SV[jj].astype(np.float32), phase=phase.astype(np.int8),
        mode=MODE[jj].astype(np.int8), vn=Vn[jj].astype(np.float32),
        glitch=ref_glitch,
        d3=d3.astype(np.float32), d2=d2.astype(np.float32), d3n=d3n.astype(np.float32),
        d2n=d2n.astype(np.float32), al=al.astype(np.float32), cr=cr.astype(np.float32),
        aln=aln.astype(np.float32), crn=crn.astype(np.float32), ss=ssp.astype(np.float32),
        sref=sref.astype(np.float32),
    )
    return b, dict(info=res, s=samples)


# ------------------------------------------------------ агрегаты

def vstats(e):
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if not len(e):
        return None
    return dict(n=int(len(e)), rmse=float(np.sqrt(np.mean(e ** 2))), mae=float(np.mean(np.abs(e))),
                bias=float(np.mean(e)), p95=float(np.percentile(np.abs(e), 95)),
                max=float(np.max(np.abs(e))))


def pstats(d):
    d = np.asarray(d, float)
    d = d[np.isfinite(d)]
    if not len(d):
        return None
    return dict(n=int(len(d)), mean_abs=float(np.mean(np.abs(d))), rmse=float(np.sqrt(np.mean(d ** 2))),
                max_abs=float(np.max(np.abs(d))), median_abs=float(np.median(np.abs(d))),
                p95_abs=float(np.percentile(np.abs(d), 95)), bias=float(np.mean(d)))


def per_run_row(b, R):
    s, info = R["s"], R["info"]
    ev, en = s["ev"].astype(float), s["en"].astype(float)
    mv = s["vg"] > V_FALSE_SS
    last = lambda x: float(x[np.isfinite(x)][-1]) if np.isfinite(x).any() else float("nan")  # noqa: E731
    L = info["path_m"]
    row = dict(
        bag=b, vehicle=info["vehicle"], cfg=info["cfg"], map=info["map"],
        dur_min=info["dur_s"] / 60, path_km=L / 1000, rate_hz=info["rate_hz"],
        v_pairs=len(ev),
        v_rmse=float(np.sqrt(np.mean(ev ** 2))), v_mae=float(np.mean(np.abs(ev))),
        v_bias=float(np.mean(ev)), v_max=float(np.max(np.abs(ev))),
        naive_v_rmse=float(np.sqrt(np.mean(en ** 2))), naive_v_mae=float(np.mean(np.abs(en))),
        naive_v_bias=float(np.mean(en)),
        naiveNC_v_mae=float(np.mean(np.abs(s["enc"]))),
        cov2s_v=float(np.mean(np.abs(ev) <= 2 * s["sv"])),
        sigma_v_med=float(np.median(s["sv"])),
        false_ss_rate=float(np.mean(s["mode"][mv] == STANDSTILL)) if mv.any() else 0.0,
        naive_false_ss_rate=float(np.mean(s["vn"][mv] < 0.3)) if mv.any() else 0.0,
        p3d_mean=float(np.mean(s["d3"])), p3d_rmse=float(np.sqrt(np.mean(s["d3"].astype(float) ** 2))),
        p3d_max=float(np.max(s["d3"])), p3d_end=float(s["d3"][-1]),
        along_mean=float(np.nanmean(np.abs(s["al"]))) if np.isfinite(s["al"]).any() else float("nan"),
        along_rmse=float(np.sqrt(np.nanmean(s["al"].astype(float) ** 2))) if np.isfinite(s["al"]).any() else float("nan"),
        along_max=float(np.nanmax(np.abs(s["al"]))) if np.isfinite(s["al"]).any() else float("nan"),
        along_end=last(s["al"]),
        cross_mean=float(np.nanmean(np.abs(s["cr"]))) if np.isfinite(s["cr"]).any() else float("nan"),
        cross_max=float(np.nanmax(np.abs(s["cr"]))) if np.isfinite(s["cr"]).any() else float("nan"),
        along_nan=int(np.sum(~np.isfinite(s["al"]))),
        drift_pct_along=100 * abs(last(s["al"])) / L if L > 100 else float("nan"),
        drift_pct_3d=100 * float(s["d3"][-1]) / L if L > 100 else float("nan"),
        cov2s_s_along=float(np.nanmean((np.abs(s["al"]) <= 2 * s["ss"])[np.isfinite(s["al"])]))
        if np.isfinite(s["al"]).any() else float("nan"),
        sigma_s_med=float(np.median(s["ss"])), sigma_s_end=float(s["ss"][-1]),
        naive_p3d_mean=float(np.mean(s["d3n"])), naive_p3d_end=float(s["d3n"][-1]),
        naive_p3d_max=float(np.max(s["d3n"])),
        naive_along_rmse=float(np.sqrt(np.nanmean(s["aln"].astype(float) ** 2))) if np.isfinite(s["aln"]).any() else float("nan"),
        naive_along_end=last(s["aln"]),
        naive_drift_pct_along=100 * abs(last(s["aln"])) / L if L > 100 else float("nan"),
        anchors=info["anchors"], anchors_naive=info["anchors_naive"],
        kt_min=info["kt_min"], kt_max=info["kt_max"], kb_min=info["kb_min"], kb_max=info["kb_max"],
        k_at_bound=info["k_at_bound"], mu_min=info["mu_min"], frac_valid=info["frac_valid"],
        frac_amb=info["frac_amb"], frac_slip=info["frac_slip"],
        fix_status_neg=info["fix_status_neg"], wall_s=info["wall_s"], maxrss_mb=info["maxrss_mb"],
    )
    return row


def aggregate(results, ids):
    rs = [results[b] for b in ids if results.get(b) is not None]
    if not rs:
        return None
    cat = {k: np.concatenate([R["s"][k] for R in rs]) for k in rs[0]["s"]}
    ev, en, enc = (cat[k].astype(float) for k in ("ev", "en", "enc"))
    sv, ph, vg = cat["sv"].astype(float), cat["phase"], cat["vg"]
    clean = ~cat["glitch"]
    out = dict(runs=len(rs), bags=[R["info"]["bag"] for R in rs])
    sp = dict(model=vstats(ev), naive_causal=vstats(en), naive_noncausal_as_evaluate=vstats(enc),
              model_ref_clean=vstats(ev[clean]), naive_causal_ref_clean=vstats(en[clean]),
              ref_glitch_samples=int((~clean).sum()),
              cov1s=float(np.mean(np.abs(ev) <= sv)), cov2s=float(np.mean(np.abs(ev) <= 2 * sv)),
              cov2s_ref_clean=float(np.mean((np.abs(ev) <= 2 * sv)[clean])),
              sigma_v_median=float(np.median(sv)), sigma_v_rms=float(np.sqrt(np.mean(sv ** 2))),
              by_phase={})
    for k, name in enumerate(PHASES):
        mk = ph == k
        if not mk.any():
            continue
        sp["by_phase"][name] = dict(
            share=float(mk.mean()), model=vstats(ev[mk]), naive_causal=vstats(en[mk]),
            cov2s=float(np.mean(np.abs(ev[mk]) <= 2 * sv[mk])),
            sigma_v_median=float(np.median(sv[mk])))
    mv = vg > V_FALSE_SS
    mv1 = vg > 1.0
    sp["false_standstill"] = dict(
        rate_vg_gt_0_5=float(np.mean(cat["mode"][mv] == STANDSTILL)),
        rate_vg_gt_1_0=float(np.mean(cat["mode"][mv1] == STANDSTILL)),
        count_vg_gt_0_5=int(np.sum(cat["mode"][mv] == STANDSTILL)),
        naive_rate_vg_gt_0_5=float(np.mean(cat["vn"][mv] < 0.3)),
        moving_samples=int(mv.sum()))
    out["speed"] = sp
    # положение
    rows = [per_run_row(R["info"]["bag"], R) for R in rs]
    ss = cat["ss"].astype(float)
    alf = np.isfinite(cat["al"])
    pos = dict(
        model=dict(d3=pstats(cat["d3"]), d2=pstats(cat["d2"]), along=pstats(cat["al"]),
                   cross=pstats(cat["cr"])),
        naive=dict(d3=pstats(cat["d3n"]), d2=pstats(cat["d2n"]), along=pstats(cat["aln"]),
                   cross=pstats(cat["crn"])),
        along_undefined_samples=int((~alf).sum()),
        # along имеет смысл, пока оценка на том же пути, что эталон: |cross| <= 5 м
        on_track=dict(
            frac_model=float(np.mean(np.abs(cat["cr"][alf]) <= 5.0)),
            frac_naive=float(np.mean(np.abs(cat["crn"][np.isfinite(cat["crn"])]) <= 5.0)),
            model_along=pstats(cat["al"][alf & (np.abs(np.nan_to_num(cat["cr"], nan=1e9)) <= 5.0)]),
            naive_along=pstats(cat["aln"][np.isfinite(cat["aln"])
                                          & (np.abs(np.nan_to_num(cat["crn"], nan=1e9)) <= 5.0)])),
        cov2s_along=float(np.mean(np.abs(cat["al"][alf]) <= 2 * ss[alf])),
        cov2s_h=float(np.mean(cat["d2"] <= 2 * ss)),
        cov245s_h=float(np.mean(cat["d2"] <= 2.45 * ss)),
        sigma_s_median=float(np.median(ss)),
        per_run=dict(
            end3d_median=float(np.median([r["p3d_end"] for r in rows])),
            end3d_mean=float(np.mean([r["p3d_end"] for r in rows])),
            end3d_max=float(np.max([r["p3d_end"] for r in rows])),
            naive_end3d_median=float(np.median([r["naive_p3d_end"] for r in rows])),
            drift_pct_along_median=float(np.nanmedian([r["drift_pct_along"] for r in rows])),
            drift_pct_along_mean=float(np.nanmean([r["drift_pct_along"] for r in rows])),
            drift_pct_along_max=float(np.nanmax([r["drift_pct_along"] for r in rows])),
            naive_drift_pct_along_median=float(np.nanmedian([r["naive_drift_pct_along"] for r in rows])),
            naive_drift_pct_along_mean=float(np.nanmean([r["naive_drift_pct_along"] for r in rows])),
            drift_pct_3d_median=float(np.nanmedian([r["drift_pct_3d"] for r in rows])),
            p3d_mean_of_runs_median=float(np.median([r["p3d_mean"] for r in rows])),
        ),
        # сверка с evaluate.py: ср. 3D, взвешенное числом пар СКОРОСТИ
        evaluate_style=dict(
            v_mae_weighted=float(np.average([r["v_mae"] for r in rows], weights=[r["v_pairs"] for r in rows])),
            naive_nc_mae_weighted=float(np.average([r["naiveNC_v_mae"] for r in rows],
                                                   weights=[r["v_pairs"] for r in rows])),
            p3d_mean_weighted=float(np.average([r["p3d_mean"] for r in rows],
                                               weights=[r["v_pairs"] for r in rows])),
            end_median=float(np.median([r["p3d_end"] for r in rows]))),
    )
    out["position"] = pos
    return out


def cmd_metrics(args):
    ids = val_ids() if args.set == "val" else all_ids() if args.set == "all" else args.set.split(",")
    vset = set(val_ids())
    OUT.mkdir(parents=True, exist_ok=True)
    for cfg in (("json", "yaml") if args.cfg == "both" else (args.cfg,)):
        t0 = time.time()
        with ProcessPoolExecutor(args.workers) as ex:
            res = dict(ex.map(run_one, [(b, cfg, args.map) for b in ids]))
        wall = time.time() - t0
        tag = f"{args.set if args.set in ('val', 'all') else 'custom'}_{cfg}_{args.map}"
        done = [b for b in ids if res.get(b) is not None]
        rows = [per_run_row(b, res[b]) for b in done]
        with open(OUT / f"runs_{tag}.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for r in rows:
                w.writerow({k: (f"{v:.5g}" if isinstance(v, float) else v) for k, v in r.items()})
        groups = {"selected": done}
        groups["selected_dedup"] = [b for b in done if b != DUP]
        for veh in ("30618", "30639"):
            groups[f"selected_{veh}"] = [b for b in done if b.startswith(veh)]
        if args.set == "all":
            groups["val"] = [b for b in done if b in vset]
            groups["train"] = [b for b in done if b not in vset]
            for veh in ("30618", "30639"):
                groups[f"val_{veh}"] = [b for b in done if b in vset and b.startswith(veh)]
                groups[f"train_{veh}"] = [b for b in done if b not in vset and b.startswith(veh)]
        agg = {g: aggregate(res, bs) for g, bs in groups.items() if bs}
        if args.set != "all":           # сырьё отложенных прогонов для доп. анализа
            keys = list(res[done[0]]["s"].keys())
            np.savez_compressed(OUT / f"samples_{tag}.npz",
                                bag=np.concatenate([[b] * len(res[b]["s"]["ev"]) for b in done]),
                                bag_pos=np.concatenate([[b] * len(res[b]["s"]["d3"]) for b in done]),
                                **{k: np.concatenate([res[b]["s"][k] for b in done]) for k in keys})
        skipped = [b for b in ids if res.get(b) is None]
        meta = dict(cfg=cfg, map=args.map, map_file=str(MAPS[args.map]), set=args.set,
                    runs_scored=len(done), runs_skipped_no_gnss=skipped, wall_s=wall,
                    workers=args.workers, V_STAND_GNSS=V_STAND_GNSS, V_FALSE_SS=V_FALSE_SS,
                    params_q_v=load_params(cfg).q_v, params_dt=load_params(cfg).dt,
                    note="GNSS в связку — только первые 3 с (evaluate.events); пары <= 0,05 с; "
                         "эталон скорости |master vel| (x,y); положение — ENU от первого master fix.")
        (OUT / f"metrics_{tag}.json").write_text(
            json.dumps(dict(meta=meta, groups=agg), ensure_ascii=False, indent=1), encoding="utf-8")
        s = agg["selected"]
        print(f"[{tag}] {len(done)} прогонов за {wall:.0f} с; скорость RMSE {s['speed']['model']['rmse']:.4f} "
              f"MAE {s['speed']['model']['mae']:.4f} bias {s['speed']['model']['bias']:+.4f} "
              f"(наивно MAE {s['speed']['naive_causal']['mae']:.4f}); cov2σ {s['speed']['cov2s']:.3f}; "
              f"3D mean {s['position']['model']['d3']['mean_abs']:.2f} м; "
              f"evaluate-style {s['position']['evaluate_style']}", flush=True)


# ------------------------------------------------------ быстродействие

def cmd_timing(args):
    p = load_params(args.cfg)
    OUT.mkdir(parents=True, exist_ok=True)
    rep = dict(cfg=args.cfg, map=args.map, bags={}, note="один процесс, последовательно; "
               "на хосте параллельно работают другие контейнеры — числа предварительные")
    allstep, allcb = [], []

    def digest(outs):
        return hashlib.sha256(np.array([[o["stamp"], o["v"], o["x"], o["y"], o["z"], o["sigma_v"],
                                         o["sigma_s"]] for o in outs]).tobytes()).hexdigest()
    # 1) детерминизм и сквозное время: два одинаковых прогона без инструмента
    for b in args.bags.split(","):
        a = bagio.load(b)
        t0 = time.perf_counter()
        o1 = replay(a, [Runner(p, track_map=load_map(args.map))])[0]
        wall = time.perf_counter() - t0
        o2 = replay(a, [Runner(p, track_map=load_map(args.map))])[0]
        h1, h2 = digest(o1), digest(o2)
        rep["bags"][b] = dict(wall_s=wall, n_out=len(o1),
                              data_s=float(o1[-1]["stamp"] - o1[0]["stamp"]),
                              deterministic=(h1 == h2), sha256=h1[:16])
    # 2) замеры: отдельный прогон с инструментом на каждом шаге и колбэке
    for b in args.bags.split(","):
        a = bagio.load(b)
        r = Runner(p, track_map=load_map(args.map))
        step_ns, cb, lag = [], [], []
        orig = r._step

        def timed(orig=orig, step_ns=step_ns):
            t0 = time.perf_counter_ns()
            o = orig()
            step_ns.append(time.perf_counter_ns() - t0)
            return o
        r._step = timed
        for tb, kind, i, th, val in E.events(a):
            t0 = time.perf_counter_ns()
            if kind == 0:
                o = r.on_wheel(i, th, val)
            elif kind == 1:
                o = r.on_handle(th, val)
            else:
                o = r.on_fix(th, i, *val)
            cb.append((time.perf_counter_ns() - t0) / 1e6)
            lag.extend(th - x["stamp"] for x in o)
        st = np.array(step_ns) / 1e6
        cbv = np.array(cb)
        lg = np.array(lag)
        allstep.append(st)
        allcb.append(cbv)
        rep["bags"][b].update(
            step_ms=dict(median=float(np.median(st)), p99=float(np.percentile(st, 99)),
                         p999=float(np.percentile(st, 99.9)), max=float(st.max()), n=int(len(st))),
            callback_ms=dict(median=float(np.median(cbv)), p99=float(np.percentile(cbv, 99)),
                             max=float(cbv.max()), n=int(len(cbv))),
            stamp_lag_s=dict(median=float(np.median(lg)), p99=float(np.percentile(lg, 99)),
                             max=float(lg.max()), frac_gt_0_1=float(np.mean(lg > 0.1))),
            realtime_factor=float(rep["bags"][b]["data_s"] / max(st.sum() / 1e3, 1e-9)))
        print(b, rep["bags"][b], flush=True)
    st = np.concatenate(allstep)
    cbv = np.concatenate(allcb)
    rep["pooled_step_ms"] = dict(median=float(np.median(st)), p99=float(np.percentile(st, 99)),
                                 p999=float(np.percentile(st, 99.9)), max=float(st.max()), n=int(len(st)))
    rep["pooled_callback_ms"] = dict(median=float(np.median(cbv)), p99=float(np.percentile(cbv, 99)),
                                     max=float(cbv.max()))
    rep["maxrss_mb"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    (OUT / f"timing_{args.cfg}_{args.map}.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: rep[k] for k in ("pooled_step_ms", "pooled_callback_ms", "maxrss_mb")}, indent=1))


# ------------------------------------------------------ инъекция отказов (#8)

def inject(a, kind, t0, dur):
    """Копия прогона с отказом ОБЕИХ тележек в окне [t0, t0+dur) по header.stamp:
    zero — показания 0; drop — сообщений нет; freeze — последнее значение до окна."""
    b = {k: v.copy() for k, v in a.items()}
    for key in ("front", "rear"):
        x = b[key]
        w = (x[:, 1] >= t0) & (x[:, 1] < t0 + dur)
        if kind == "zero":
            x[w, 2] = 0.0
        elif kind == "freeze":
            before = x[x[:, 1] < t0]
            x[w, 2] = before[-1, 2] if len(before) else 0.0
        elif kind == "drop":
            x = x[~w]
        b[key] = x
    return b


def _inj_one(args):
    bag, kind, dur, cfg, mapname = args
    p = load_params(cfg)
    a = bagio.load(bag)
    g = a["mvel"]
    vg = np.hypot(g[:, 2], g[:, 3])
    t_start = g[0, 1]
    # первое окно, где GNSS-скорость > 8 м/с непрерывно dur+5 с, не раньше 120 с от начала
    t0 = None
    for i in range(len(g)):
        if g[i, 1] < t_start + 120 or vg[i] < 8.0:
            continue
        w = (g[:, 1] >= g[i, 1]) & (g[:, 1] < g[i, 1] + dur + 5)
        if vg[w].min() > 6.0:
            t0 = g[i, 1]
            break
    if t0 is None:
        return bag, kind, None
    res = {}
    for label, aa in (("clean", a), (kind, inject(a, kind, t0, dur))):
        r = Runner(p, track_map=load_map(mapname))
        nv = NaiveRunner(p, load_map(mapname))
        mo, no = replay(aa, [r, nv])
        T = arr(mo, "stamp")
        m = a["mfix"]
        enu = Enu(m[0, 2], m[0, 3], m[0, 4])
        ref = np.array([enu.fwd(la, lo, al) for la, lo, al in m[:, 2:5]])
        j2, ok2 = E.nearest(T, m[:, 1])
        idx = np.flatnonzero(ok2)
        X = np.array([[o["x"], o["y"]] for o in mo])[j2[ok2]]
        Xn = np.array([[o["x"], o["y"]] for o in no])[j2[ok2]]
        al, _, _, _ = along_cross(ref[:, :2], idx, X)
        aln, _, _, _ = along_cross(ref[:, :2], idx, Xn)
        tf = m[idx, 1]

        def at(x, t):
            k = np.searchsorted(tf, t)
            k = min(max(k, 0), len(x) - 1)
            return float(x[k])
        jv, okv = E.nearest(T, g[:, 1])
        V = arr(mo, "v")
        Vn = arr(no, "v")
        win = okv & (g[:, 1] >= t0) & (g[:, 1] < t0 + dur)
        o_w = [o for o in mo if t0 <= o["stamp"] < t0 + dur]
        o_after = [o for o in mo if t0 + dur <= o["stamp"] < t0 + dur + 30]
        res[label] = dict(
            along_at_end_of_window=at(al, t0 + dur), along_60s_after=at(al, t0 + dur + 60),
            along_end=float(al[np.isfinite(al)][-1]),
            naive_along_at_end_of_window=at(aln, t0 + dur), naive_along_end=float(aln[np.isfinite(aln)][-1]),
            v_mae_window=float(np.mean(np.abs(V[jv[win]] - vg[win]))),
            v_bias_window=float(np.mean(V[jv[win]] - vg[win])),
            naive_v_mae_window=float(np.mean(np.abs(Vn[jv[win]] - vg[win]))),
            sigma_v_end_window=float(o_w[-1]["sigma_v"]) if o_w else None,
            sigma_s_end_window=float(o_w[-1]["sigma_s"]) if o_w else None,
            frac_amb_window=float(np.mean([o["ambiguous"] for o in o_w])) if o_w else None,
            frac_valid_window=float(np.mean([o["valid"] for o in o_w])) if o_w else None,
            frac_standstill_window=float(np.mean([o["mode"] == STANDSTILL for o in o_w])) if o_w else None,
            frac_valid_30s_after=float(np.mean([o["valid"] for o in o_after])) if o_after else None,
            gnss_path_in_window=float(np.trapz(vg[(g[:, 1] >= t0) & (g[:, 1] < t0 + dur)],
                                               g[(g[:, 1] >= t0) & (g[:, 1] < t0 + dur), 1])),
        )
    res["t0_rel_s"] = float(t0 - t_start)
    res["dur_s"] = dur
    return bag, kind, res


def cmd_inject(args):
    jobs = [(b, k, args.dur, args.cfg, args.map) for b in args.bags.split(",")
            for k in args.kinds.split(",")]
    with ProcessPoolExecutor(args.workers) as ex:
        out = list(ex.map(_inj_one, jobs))
    rep = {f"{b}|{k}": r for b, k, r in out}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"inject_{args.cfg}_{args.map}.json").write_text(
        json.dumps(dict(cfg=args.cfg, map=args.map, dur=args.dur, runs=rep), ensure_ascii=False, indent=1),
        encoding="utf-8")
    for k, r in rep.items():
        if r is None:
            print(k, "нет подходящего окна")
            continue
        kind = k.split("|")[1]
        c, f = r["clean"], r[kind]
        print(f"{k}: t0={r['t0_rel_s']:.0f}s путь GNSS в окне {f['gnss_path_in_window']:.0f} м; "
              f"along к концу окна: чисто {c['along_at_end_of_window']:+.1f} / отказ {f['along_at_end_of_window']:+.1f} м "
              f"(наивно {f['naive_along_at_end_of_window']:+.1f}); +60 с {f['along_60s_after']:+.1f}; "
              f"конец {c['along_end']:+.1f}/{f['along_end']:+.1f}; v MAE в окне {f['v_mae_window']:.2f} "
              f"(наивно {f['naive_v_mae_window']:.2f}); amb {f['frac_amb_window']}, valid {f['frac_valid_window']}, "
              f"STANDSTILL {f['frac_standstill_window']}; σv {f['sigma_v_end_window']}, σs {f['sigma_s_end_window']}")


# ------------------------------------------------------ скачок метки времени

def cmd_probe(args):
    """Связка шагает циклом while до метки сообщения без ограничения числа
    шагов. Одна битая (будущая) метка -> тысячи шагов в одном колбэке."""
    p = load_params(args.cfg)
    rep = {}
    for jump in (10.0, 60.0, 600.0):
        r = Runner(p)
        t = 1000.0
        for k in range(200):
            t = 1000.0 + k * 0.05
            r.on_handle(t, 0)
            r.on_wheel(k % 2, t + 0.01, 18.0)
        t0 = time.perf_counter()
        outs = r.on_wheel(0, t + jump, 18.0)
        dt = time.perf_counter() - t0
        # после скачка нормальные сообщения со «старыми» метками шагов не дают
        after = r.on_wheel(1, t + 0.1, 18.0) + r.on_handle(t + 0.15, 0)
        rep[f"jump_{jump:g}s"] = dict(outputs_in_one_callback=len(outs), callback_s=dt,
                                      outputs_after_from_normal_msgs=len(after),
                                      extrapolated_s_for_2e9s_jump=dt / jump * 2e9)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "probe_stamp_jump.json").write_text(json.dumps(rep, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(rep, ensure_ascii=False, indent=1))


def cmd_adapt(args):
    """Срабатывает ли адаптация масштабов привода k_t, k_b на реальных данных.
    _adapt_scale (estimator_core.py) требует len(acc) >= 2 на КАЖДОМ шаге окна
    t_adapt; тележки ~9,4 Гц приходят в разные моменты, шаг 50 мс."""
    p = load_params(args.cfg)
    rep = {}
    for b in args.bags.split(","):
        a = bagio.load(b)
        r = Runner(p, track_map=None)
        c = r.core
        st = dict(steps=0, steps_tr_br_fast=0, steps_acc2=0, steps_acc1=0, adapting=0,
                  win_t_max=0.0, kt_changes=0, kb_changes=0)
        orig = r._step
        prev = [float(c.x[3]), float(c.x[4])]

        def timed(orig=orig, st=st, prev=prev):
            o = orig()
            st["steps"] += 1
            if c.mode_pre in (1, 2) and c.x[1] > p.v_adapt_min:
                st["steps_tr_br_fast"] += 1
                st["steps_acc2"] += int(o["n_accepted"] >= 2)
                st["steps_acc1"] += int(o["n_accepted"] == 1)
            st["adapting"] += int(c.adapting)
            st["win_t_max"] = max(st["win_t_max"], float(c.win_t))
            st["kt_changes"] += int(abs(float(c.x[3]) - prev[0]) > 1e-12)
            st["kb_changes"] += int(abs(float(c.x[4]) - prev[1]) > 1e-12)
            prev[0], prev[1] = float(c.x[3]), float(c.x[4])
            return o
        r._step = timed
        # Повтор логики _adapt_scale (estimator_core.py) ДО вызова оригинала —
        # только чтобы записать, чем закончилось каждое окно; поведение не меняется.
        from tram_state_estimator.estimator_core import drive_force, rated_force, resistance
        wins = []
        orig_ad = c._adapt_scale

        st["adapt_calls"] = 0
        st["t_first"] = None

        def ad(u, mode, acc, wins=wins, orig_ad=orig_ad, st=st):
            st["adapt_calls"] += 1
            active = (mode in (1, 2) and len(acc) >= 2 and c.x[1] > p.v_adapt_min and not c.sat)
            if active and mode == c.win_mode and c.win_t == 0.0:
                st["_t_open"] = r.t                     # время (по меткам) открытия окна
            if active and mode == c.win_mode and c.win_t + p.dt >= p.t_adapt and "_t_open" in st:
                st.setdefault("_real_window_s", []).append(r.t - st["_t_open"] + p.dt)
            if active and mode == c.win_mode:
                v = float(c.x[1])
                F = c.win_F + drive_force(u, v, 1.0, 1.0, p) * p.dt
                W = c.win_W + resistance(v, p) * p.dt
                T = c.win_t + p.dt
                if T >= p.t_adapt:
                    rated = rated_force(mode == 1, p)
                    passF = abs(F) >= p.adapt_f_min * rated * p.t_adapt
                    k_meas = gate = None
                    if passF:
                        k_meas = (p.M_nom * (v - c.win_v0 - c.x[2] * T) + W) / F
                        idx = 3 if mode == 1 else 4
                        S = c.P[idx, idx] + p.sigma_k_meas ** 2
                        gate = bool((k_meas - c.x[idx]) ** 2 <= p.gate_nis * S)
                    wins.append((mode, passF, k_meas, gate, abs(F) / (rated * p.t_adapt),
                                 v - c.win_v0, float(c.x[2]) * T, W / p.M_nom, F / p.M_nom, T))
            return orig_ad(u, mode, acc)
        c._adapt_scale = ad
        replay(a, [r])
        W_ = [w for w in wins]
        st["windows_closed"] = len(W_)
        st["windows_force_ok"] = sum(1 for w in W_ if w[1])
        st["windows_gate_ok"] = sum(1 for w in W_ if w[3])
        km = [w[2] for w in W_ if w[2] is not None]
        st["k_meas_median"] = float(np.median(km)) if km else None
        st["k_meas_p10_p90"] = [float(np.percentile(km, 10)), float(np.percentile(km, 90))] if km else None
        fr = [w[4] for w in W_]
        st["mean_force_frac_of_rated_median"] = float(np.median(fr)) if fr else None
        for mname, mcode in (("traction", 1), ("brake", 2)):
            ww = [w for w in W_ if w[0] == mcode and w[1]]
            if ww:
                A = np.array([w[5:10] for w in ww], float)
                st[f"{mname}_windows"] = dict(
                    n=len(ww), gate_ok=sum(1 for w in ww if w[3]),
                    k_meas_median=float(np.median([w[2] for w in ww])),
                    dv_median=float(np.median(A[:, 0])), dT_median=float(np.median(A[:, 1])),
                    intW_over_M_median=float(np.median(A[:, 2])),
                    intF_over_M_median=float(np.median(A[:, 3])))
        rw = st.pop("_real_window_s", [])
        st.pop("_t_open", None)
        st.pop("t_first", None)
        st["window_real_duration_s_median"] = float(np.median(rw)) if rw else None
        st["steps_per_adapt_call"] = st["steps"] / max(st["adapt_calls"], 1)
        st["rated_traction_N"] = rated_force(True, p)
        st["rated_brake_N"] = rated_force(False, p)
        st["k_t_end"], st["k_b_end"] = float(c.x[3]), float(c.x[4])
        st["t_adapt"] = p.t_adapt
        rep[b] = st
        print(b, st, flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"adapt_{args.cfg}.json").write_text(json.dumps(rep, ensure_ascii=False, indent=1),
                                                encoding="utf-8")


def cmd_lag(args):
    """Сдвиг по времени «тележки (header.stamp) — скорость GNSS master»:
    ошибка среднего двух тележек (линейная интерполяция в момент tg + τ)
    против |v| GNSS для τ из сетки. τ > 0 — показание тележки относится к
    более позднему моменту, чем его метка (датчик «отстаёт»)."""
    ids = val_ids() if args.set == "val" else all_ids() if args.set == "all" else args.set.split(",")
    p = load_params("yaml")
    taus = np.round(np.arange(-0.30, 0.501, 0.025), 3)
    acc = {float(t): [] for t in taus}
    ph_all = []
    for b in ids:
        a = bagio.load(b)
        g = a["mvel"]
        if len(g) < 50:
            continue
        tg, vg = g[:, 1], np.hypot(g[:, 2], g[:, 3])
        f, r = a["front"], a["rear"]
        n_h = np.nan_to_num(zoh(a["cmd"][:, 1], a["cmd"][:, 2], tg), nan=0.0)
        ph_all.append(np.where(vg < V_STAND_GNSS, 0, np.where(n_h > 0, 1, np.where(n_h < 0, 3, 2))))
        for t in taus:
            w = 0.5 * (np.interp(tg + t, f[:, 1], f[:, 2]) + np.interp(tg + t, r[:, 1], r[:, 2])) \
                / 3.6 * p.meas_scale
            acc[float(t)].append((w - vg).astype(np.float32))
    ph = np.concatenate(ph_all)
    rep = {}
    for t in taus:
        e = np.concatenate(acc[float(t)]).astype(float)
        e = e[np.abs(e) < 2.0]  # без грубых выбросов эталона
        rep[f"{t:+.3f}"] = dict(mae=float(np.mean(np.abs(e))), bias=float(np.mean(e)))
    best = min(rep, key=lambda k: rep[k]["mae"])
    # смещение по фазам при τ = 0 и при лучшем τ
    by = {}
    for key in ("+0.000", best):
        e = np.concatenate(acc[float(key)]).astype(float)
        m0 = np.abs(e) < 2.0
        by[key] = {name: dict(mae=float(np.mean(np.abs(e[(ph == k) & m0]))),
                              bias=float(np.mean(e[(ph == k) & m0])))
                   for k, name in enumerate(PHASES)}
    out = dict(set=args.set, runs=len(ph_all), best_tau_s=float(best), by_tau=rep, by_phase=by,
               note="ошибка = среднее тележек(tg+τ)·meas_scale/3,6 − |v GNSS|; |e|<2 м/с")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"lag_{args.set}.json").write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("лучший τ", best, rep[best], "τ=0:", rep["+0.000"])
    print(json.dumps(by, ensure_ascii=False, indent=1))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("metrics")
    m.add_argument("--set", default="val")
    m.add_argument("--cfg", default="both", choices=CFGS + ("both",))
    m.add_argument("--map", default="train", choices=tuple(MAPS))
    m.add_argument("--workers", type=int, default=max(1, min(10, (os.cpu_count() or 2) - 2)))
    t = sub.add_parser("timing")
    t.add_argument("--bags", default="30618_3e9f4952")
    t.add_argument("--cfg", default="yaml", choices=CFGS)
    t.add_argument("--map", default="pkg", choices=tuple(MAPS))
    i = sub.add_parser("inject")
    i.add_argument("--bags", default="30618_3e9f4952")
    i.add_argument("--kinds", default="zero,drop,freeze")
    i.add_argument("--dur", type=float, default=20.0)
    i.add_argument("--cfg", default="yaml", choices=CFGS)
    i.add_argument("--map", default="train", choices=tuple(MAPS))
    i.add_argument("--workers", type=int, default=6)
    pr = sub.add_parser("probe")
    pr.add_argument("--cfg", default="yaml", choices=CFGS)
    ad = sub.add_parser("adapt")
    ad.add_argument("--bags", default="30618_3e9f4952")
    ad.add_argument("--cfg", default="yaml", choices=CFGS)
    lg = sub.add_parser("lag")
    lg.add_argument("--set", default="val")
    args = ap.parse_args()
    {"metrics": cmd_metrics, "timing": cmd_timing, "inject": cmd_inject, "probe": cmd_probe,
     "adapt": cmd_adapt, "lag": cmd_lag}[args.cmd](args)


if __name__ == "__main__":
    main()
