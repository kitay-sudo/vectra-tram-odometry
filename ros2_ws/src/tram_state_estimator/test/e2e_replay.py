"""Офлайн-проигрывание фикстуры реального прогона через Runner (без ROS).

Общий код для test_e2e_real.py и test_determinism.py. Нода собирается так же,
как в tram_node.py: лист config/tram.yaml (поля Params + параметры ноды),
карта из map_file. Сообщения подаются в порядке записи в bag (tb), как их
проигрывает `ros2 bag play`; GNSS fix - только первые init_window_s секунд
(в проверочных bag GNSS гарантирован лишь в начале, README датасета §3.2)
или весь кусок (gnss="all": GNSS после окна не двигает сетку и скорость).

Эталон скорости - |GNSS master/vel| по горизонтали (основной) и rover/vel
(контрольный): «эталонной скорости нет, есть 4 источника» (организаторы,
25.09). Эталон положения - точка base_link по tf антенн (организаторы, 25.09:
base_link = master + 9,873/12,436 · (rover − master), z − 3,0; пары master и
rover одной эпохи ±0,05 с), переведённая в ту систему, в которой публикует
Runner (MGRS, UTM, ENU или прежний equirect): система определяется по самому
выходу (refgeo.detect), поэтому тест не зависит от значения параметра
projection. point="master" - прежний эталон (антенна master). Пары - по ближайшей метке выхода в пределах
0,05 с, как у судьи. Положение сравнивается только у выходов, которые нода
публикует в /result/position (pos_valid, как в tram_node.py); ошибка - без
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


def sheet(over=None):
    """(Params, параметры ноды) из config/tram.yaml - как в tram_node.py
    (с масштабом колёс вагона из параметра vehicle, если он есть). over -
    параметры ноды поверх листа (до выбора вагона: vehicle="auto" и т. п.)."""
    import yaml
    from tram_state_estimator.estimator_core import Params
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    names = {f.name for f in fields(Params)}
    params = Params.from_dict({k: v for k, v in got.items() if k in names})
    node = {k: v for k, v in got.items() if k not in names}
    node.update(over or {})
    from tram_state_estimator import vehicle
    return vehicle.apply_node(params, node)[0], node


def _accepts(fn, name):
    try:
        sig = inspect.signature(fn)
    except (TypeError, ValueError):
        return False
    return name in sig.parameters or any(
        p.kind is p.VAR_KEYWORD for p in sig.parameters.values())


# Параметры положения: (ключ листа / ноды, аргумент Runner/Position). Ключи
# листа - как в tram_node.py; передаются, только если есть в листе или в
# переопределениях и Runner их принимает.
POSITION_OPTS = (("init_window_s", "init_window"), ("projection", "projection"),
                 ("mgrs_grid", "mgrs_grid"), ("utm_zone", "utm_zone"),
                 ("mgrs_guard_m", "mgrs_guard_m"), ("scale_adapt", "scale_adapt"),
                 ("nomap_mode", "nomap_mode"), ("keep_offset_xy", "keep_offset_xy"),
                 ("keep_offset_z", "keep_offset_z"), ("output_point", "output_point"),
                 ("antenna_master_x", "antenna_master_x"),
                 ("antenna_rover_x", "antenna_rover_x"), ("antenna_z", "antenna_z"),
                 # коррекция по GNSS после окна: имена - как у ноды
                 ("gnss_correction", "gnss_correction"),
                 ("gnss_sigma_rtk_m", "gnss_sigma_rtk_m"),
                 ("gnss_sigma_sbas_m", "gnss_sigma_sbas_m"),
                 ("gnss_sigma_fix_m", "gnss_sigma_fix_m"), ("gnss_gate", "gnss_gate"),
                 ("gnss_jump_m", "gnss_jump_m"), ("gnss_confirm_n", "gnss_confirm_n"),
                 ("gnss_min_interval_s", "gnss_min_interval_s"),
                 ("gnss_max_skew_s", "gnss_max_skew_s"),
                 ("gnss_prior_rel", "gnss_prior_rel"),
                 ("gnss_scale_adapt", "gnss_scale_adapt"),
                 ("gnss_stop_skip_m", "gnss_stop_skip_m"),
                 ("gnss_persist_s", "gnss_persist_s"))
# tf антенн в base_link (организаторы 25.09); независимо от body.py пакета
MASTER_X, ROVER_X, ANTENNA_Z = -9.873, 2.563, 3.0


def _explicit(fn, name):
    try:
        return name in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False


def runner_accepts(name):
    """Принимает ли Runner параметр name: явно в __init__ или через **kwargs,
    которые уходят в Position (Runner(..., **position_opts))."""
    from tram_state_estimator import runner as R
    init = R.Runner.__init__
    if _explicit(init, name):
        return True
    pos = getattr(R, "Position", None)
    return (_accepts(init, name) and pos is not None
            and _explicit(pos.__init__, name))


def make_runner(use_map=True, **over):
    """Runner как в ноде. over - параметры ноды поверх листа (например
    projection="utm"); передаются, только если Runner их принимает."""
    from tram_state_estimator.runner import Runner
    from tram_state_estimator.track_map import TrackMap
    params, node = sheet(over)
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
    if node.get("wheel_scale_online"):
        # онлайн-масштаб колёс - как в ноде (tram_node.py: wheel_scale_hook)
        try:
            from tram_state_estimator import vehicle as V
        except ImportError:
            V = None
        if V is not None:
            V.wheel_scale_hook(r, True)
    return r, node


def events(fx, gnss="window", init_s=3.0):
    """Сообщения в порядке записи. gnss: "window" - fix только первые init_s
    секунд от первого master fix (по времени записи); "all" - все; "none"."""
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


def reference(fx, point="base_link", origin=None):
    """Эталон положения: (метки, {система: N×3}) - точка point вагона по GNSS.
    base_link - по парам master+rover одной эпохи (±0,05 с) в каждой системе:
    master + 9,873/12,436 · (rover − master), z − 3,0 (фиксы master без пары
    не входят); master - сами фиксы master."""
    m = fx["mfix"]
    o = origin or tuple(m[0, 2:5])
    if point == "master":
        return m[:, 1], refgeo.frames(m[:, 2], m[:, 3], m[:, 4], origin=o)
    r = fx["rfix"]
    j, ok = _nearest(r[:, 1], m[:, 1])
    fm = refgeo.frames(m[ok, 2], m[ok, 3], m[ok, 4], origin=o)
    fr = refgeo.frames(r[j[ok], 2], r[j[ok], 3], r[j[ok], 4], origin=o)
    f = -MASTER_X / (ROVER_X - MASTER_X)
    out = {}
    for name in fm:
        if name not in fr:
            continue
        d = fr[name] - fm[name]
        d[:, :2] -= np.round(d[:, :2] / 1e5) * 1e5      # пара по разные стороны границы квадратов
        out[name] = fm[name] + f * d
        out[name][:, 2] -= ANTENNA_Z
    return m[ok, 1], out


def metrics(outs, fx, frame=None, point="base_link"):
    """Метрики выхода. frame - система эталона положения (имя из refgeo.frames);
    None - определить по самому выходу (refgeo.detect). point - точка
    эталона: base_link (по умолчанию, как у судьи) или master."""
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
    # Положение - только выходы, которые нода публикует в /result/position:
    # tram_node.py не публикует выход с pos_valid=False (нет якоря - нет
    # положения). У Runner без этого поля публикуется всё, включая заглушку
    # (s, 0, 0) до выставки.
    pub = [k for k, o in enumerate(outs) if o.get("pos_valid", True)]
    res["n_pos_published"] = len(pub)
    # шагов без опубликованного положения после первого опубликованного
    res["n_pos_gaps"] = (len(outs) - pub[0] - len(pub)) if pub else 0
    res["squares"] = refgeo.mgrs_squares(fx["mfix"][:, 2], fx["mfix"][:, 3])
    m = fx["mfix"]
    if len(pub) < 2:            # положения нет совсем (нет GNSS - нет якоря)
        res.update(p_frame=frame or "none", p_pairs=0)
        return _path(res, m)
    Tp, Xall = T[pub], X[pub]
    ready_all = np.array([bool(outs[k].get("pos_ready", True)) for k in pub])
    t_ref, fr_all = reference(fx, point)
    j2, ok2 = _nearest(Tp, t_ref)
    if not ok2.any():
        res.update(p_frame=frame or "none", p_pairs=0)
        return _path(res, m)
    fr = {k: v[ok2] for k, v in fr_all.items()}
    Xp, ready = Xall[j2[ok2]], ready_all[j2[ok2]]
    if frame is None:
        frame, _ = refgeo.detect(Xp, fr)
    # основная ошибка - как у судьи, без развёртки; развёрнутая - справочно
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
    """sha256 выходов: метка, скорость, положение, σ, режим - побайтно."""
    keys = ("stamp", "v", "x", "y", "z", "sigma_v", "sigma_s", "a")
    arr = np.array([[float(o[k]) for k in keys] + [float(o["mode"])] for o in outs],
                   dtype="<f8")
    return hashlib.sha256(arr.tobytes()).hexdigest()
