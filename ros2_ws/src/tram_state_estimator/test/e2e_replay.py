"""Офлайн-проигрывание фикстуры реального прогона через Runner (без ROS).

Общий код для test_e2e_real.py и test_determinism.py. Нода собирается так же,
как в tram_node.py: лист config/tram.yaml (поля Params + параметры ноды),
карта из map_file. Сообщения подаются в порядке записи в bag (tb), как их
проигрывает `ros2 bag play`; GNSS fix — только первые init_window_s секунд
(в проверочных bag GNSS гарантирован лишь в начале, README датасета §3.2)
или весь кусок (gnss="all", проверка C2).

Эталон скорости — |GNSS master/vel| по горизонтали (основной) и rover/vel
(контрольный): «эталонной скорости нет, есть 4 источника» (организаторы,
25.09). Эталон положения — GNSS master fix, переведённый в ту систему, в
которой публикует Runner (MGRS, UTM, ENU или прежний equirect): система
определяется по самому выходу (refgeo.detect), поэтому тест не зависит от
значения параметра projection. Пары — по ближайшей метке выхода в пределах
0,05 с, как у судьи. Положение сравнивается только у выходов, которые нода
публикует в /result/position (pos_valid, как в tram_node.py); ошибка — без
вычета скачка на границе 100-км квадратов MGRS (так её увидит судья).
"""

import hashlib
import inspect
import json
import math
import os
from dataclasses import fields

import numpy as np

import refgeo

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
FIXTURE = os.path.join(HERE, "data", "e2e_30618_b95ca60a_180s.npz")
TOL = 0.05


def load_fixture(path=FIXTURE):
    z = np.load(path, allow_pickle=False)
    fx = {k: z[k] for k in z.files if k != "meta"}
    fx["meta"] = json.loads(str(z["meta"]))
    return fx


def sheet():
    """(Params, параметры ноды) из config/tram.yaml — как в tram_node.py."""
    import yaml
    from tram_state_estimator.estimator_core import Params
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    names = {f.name for f in fields(Params)}
    params = Params.from_dict({k: v for k, v in got.items() if k in names})
    node = {k: v for k, v in got.items() if k not in names}
    return params, node


def _accepts(fn, name):
    try:
        sig = inspect.signature(fn)
    except (TypeError, ValueError):
        return False
    return name in sig.parameters or any(
        p.kind is p.VAR_KEYWORD for p in sig.parameters.values())


# Параметры положения: (ключ листа / ноды, аргумент Runner/Position). Ключи
# листа — как в tram_node.py; передаются, только если есть в листе или в
# переопределениях и Runner их принимает (у ветки без потока position — нет).
POSITION_OPTS = (("init_window_s", "init_window"), ("projection", "projection"),
                 ("mgrs_grid", "mgrs_grid"), ("utm_zone", "utm_zone"),
                 ("mgrs_guard_m", "mgrs_guard_m"), ("scale_adapt", "scale_adapt"),
                 ("nomap_mode", "nomap_mode"), ("keep_offset_xy", "keep_offset_xy"),
                 ("keep_offset_z", "keep_offset_z"))


def _explicit(fn, name):
    try:
        return name in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False


def runner_accepts(name):
    """Принимает ли Runner параметр name: явно в __init__ или через **kwargs,
    которые уходят в Position (поток position: Runner(..., **position_opts))."""
    from tram_state_estimator import runner as R
    init = R.Runner.__init__
    if _explicit(init, name):
        return True
    pos = getattr(R, "Position", None)
    return (_accepts(init, name) and pos is not None
            and _explicit(pos.__init__, name))


def make_runner(use_map=True, **over):
    """Runner как в ноде. over — параметры ноды поверх листа (например
    projection="utm"); передаются, только если Runner их принимает."""
    from tram_state_estimator.runner import Runner
    from tram_state_estimator.track_map import TrackMap
    params, node = sheet()
    node.update(over)
    tmap = None
    mf = node.get("map_file", "")
    if use_map and mf:
        path = mf if os.path.isabs(mf) else os.path.join(PKG, mf)
        tmap = TrackMap.load(path)
    kw = dict(track_map=tmap,
              wheel_timeout=node.get("wheel_timeout_s", 1.0),
              handle_timeout=node.get("handle_timeout_s", 0.5))
    o = tuple(node.get(k, math.nan) for k in ("origin_lat", "origin_lon", "origin_alt"))
    if all(isinstance(v, (int, float)) and math.isfinite(v) for v in o):
        kw["origin"] = o
    for key, arg in POSITION_OPTS:
        if key in node and runner_accepts(arg):
            kw[arg] = node[key]
    r = Runner(params, **kw)
    pos = getattr(r, "pos", None)
    if pos is not None and hasattr(pos, "init_window") and "init_window" not in kw:
        pos.init_window = node.get("init_window_s", 3.0)
    return r, node


def events(fx, gnss="window", init_s=3.0):
    """Сообщения в порядке записи. gnss: "window" — fix только первые init_s
    секунд от первого master fix (по времени записи); "all" — все; "none"."""
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in fx[key]:
            ev.append((tb, 0, i, th, float(v)))
    for tb, th, n in fx["cmd"]:
        ev.append((tb, 1, 0, th, int(n)))
    if gnss != "none" and len(fx["mfix"]):
        t_end = fx["mfix"][0, 0] + init_s if gnss == "window" else math.inf
        for key, ant in (("mfix", "master"), ("rfix", "rover")):
            for row in fx[key]:
                if row[0] <= t_end:
                    st = int(row[5]) if len(row) > 5 else 0
                    ev.append((row[0], 2, ant, row[1], (row[2], row[3], row[4], st)))
    ev.sort(key=lambda e: (e[0], e[1], str(e[2])))
    return ev


def replay(fx, use_map=True, gnss="window", **over):
    r, node = make_runner(use_map, **over)
    status_ok = _explicit(r.on_fix, "status")      # NavSatFix.status, как в ноде
    outs = []
    for _tb, kind, i, th, val in events(fx, gnss, node.get("init_window_s", 3.0)):
        if kind == 0:
            outs += r.on_wheel(i, th, val)
        elif kind == 1:
            outs += r.on_handle(th, val)
        elif status_ok:
            outs += r.on_fix(th, i, *val[:3], status=val[3])
        else:
            outs += r.on_fix(th, i, *val[:3])
    return outs


def _nearest(t_out, t_ref):
    j = np.clip(np.searchsorted(t_out, t_ref), 1, len(t_out) - 1)
    j = np.where(np.abs(t_out[j - 1] - t_ref) < np.abs(t_out[j] - t_ref), j - 1, j)
    return j, np.abs(t_out[j] - t_ref) <= TOL


def _speed(T, V, g):
    j, ok = _nearest(T, g[:, 1])
    e = V[j[ok]] - np.hypot(g[ok, 2], g[ok, 3])
    return int(ok.sum()), e


def metrics(outs, fx, frame=None):
    """Метрики выхода. frame — система эталона положения (имя из refgeo.frames);
    None — определить по самому выходу (refgeo.detect)."""
    T = np.array([o["stamp"] for o in outs])
    V = np.array([o["v"] for o in outs])
    X = np.array([[o["x"], o["y"], o["z"]] for o in outs])
    S = np.array([[o["sigma_v"], o["sigma_s"]] for o in outs])
    res = {"n_out": len(T), "span_s": float(T[-1] - T[0])}
    res["rate_hz"] = (len(T) - 1) / res["span_s"]
    dT = np.diff(T)
    res["stamp_step_min"], res["stamp_step_max"] = float(dT.min()), float(dT.max())
    res["nonfinite"] = int((~np.isfinite(V)).sum() + (~np.isfinite(X)).sum()
                           + (~np.isfinite(S)).sum())
    n, e = _speed(T, V, fx["mvel"])
    res.update(v_pairs=n, v_mae=float(np.abs(e).mean()),
               v_rmse=float(np.sqrt((e ** 2).mean())), v_bias=float(e.mean()))
    if "rvel" in fx:
        n, e = _speed(T, V, fx["rvel"])
        res.update(v_pairs_rover=n, v_mae_rover=float(np.abs(e).mean()))
    # Положение — только выходы, которые нода публикует в /result/position:
    # tram_node.py не публикует выход с pos_valid=False (нет якоря — нет
    # положения). У Runner без этого поля публикуется всё, включая заглушку
    # (s, 0, 0) до выставки.
    pub = [k for k, o in enumerate(outs) if o.get("pos_valid", True)]
    res["n_pos_published"] = len(pub)
    res["squares"] = refgeo.mgrs_squares(fx["mfix"][:, 2], fx["mfix"][:, 3])
    m = fx["mfix"]
    if len(pub) < 2:            # положения нет совсем (нет GNSS — нет якоря)
        res.update(p_frame=frame or "none", p_pairs=0)
        return _path(res, m)
    Tp, Xall = T[pub], X[pub]
    ready_all = np.array([bool(outs[k].get("pos_ready", True)) for k in pub])
    j2, ok2 = _nearest(Tp, m[:, 1])
    if not ok2.any():
        res.update(p_frame=frame or "none", p_pairs=0)
        return _path(res, m)
    fr = refgeo.frames(m[ok2, 2], m[ok2, 3], m[ok2, 4], origin=tuple(m[0, 2:5]))
    Xp, ready = Xall[j2[ok2]], ready_all[j2[ok2]]
    if frame is None:
        frame, _ = refgeo.detect(Xp, fr)
    # основная ошибка — как у судьи, без развёртки; развёрнутая — справочно
    d3, d3u, mism = refgeo.errors(Xp, fr[frame], frame)
    d = Xp - fr[frame]
    res.update(p_frame=frame, p_pairs=int(ok2.sum()), p_mean3d=float(d3.mean()),
               p_max3d=float(d3.max()), p_end3d=float(d3[-1]),
               p_median3d=float(np.median(d3)),
               p_mean3d_unwrapped=float(d3u.mean()), p_square_mismatch=mism,
               p_pairs_unaligned=int((~ready).sum()),
               p_max3d_unaligned=float(d3[~ready].max()) if (~ready).any() else 0.0,
               p_mean3d_aligned=float(d3[ready].mean()) if ready.any() else float("nan"),
               p_mean_xy=float(np.hypot(d[:, 0], d[:, 1]).mean()),
               p_mean_dz=float(np.abs(d[:, 2]).mean()))
    return _path(res, m)


def _path(res, m):
    enu = refgeo.enu(m[:, 2], m[:, 3], m[:, 4], *m[0, 2:5])
    res["path_m"] = float(np.sum(np.linalg.norm(np.diff(enu[:, :2], axis=0), axis=1)))
    return res


def digest(outs):
    """sha256 выходов: метка, скорость, положение, σ, режим — побайтно."""
    keys = ("stamp", "v", "x", "y", "z", "sigma_v", "sigma_s", "a")
    arr = np.array([[float(o[k]) for k in keys] + [float(o["mode"])] for o in outs],
                   dtype="<f8")
    return hashlib.sha256(arr.tobytes()).hexdigest()
