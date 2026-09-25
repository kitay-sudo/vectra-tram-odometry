"""Шаг 6: оценка точности «как у судьи», без ROS.

Прогон воспроизводится в порядке записи в bag через ту же связку Runner, что
в ноде. GNSS подаётся в связку только первые INIT_S секунд — как в проверочных
прогонах. Эталон — GNSS master: скорость |vel|, положение fix, посчитанное
НЕЗАВИСИМО от кода ноды (georef.py: UTM по Снайдеру). Выход ноды (по умолчанию
MGRS) сравнивается в непрерывных координатах UTM: MGRS с переносом по 100-км
квадратам разворачивается к квадрату эталона. Пары «выход — эталон» — по
ближайшей метке времени в пределах 0,05 с; выходы без положения (pos_valid)
в пары не входят. Подробная оценка положения — tools/position_eval.py.

    py -3 evaluate.py                # отложенные прогоны
    py -3 evaluate.py all            # все прогоны с GNSS
    py -3 evaluate.py 30618_0e41eac3 # один прогон
"""

import json
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import bagio
import georef

sys.path.insert(0, str(bagio.ROOT / "ros2_ws" / "src" / "tram_state_estimator"))
from tram_state_estimator.estimator_core import Params          # noqa: E402
from tram_state_estimator.runner import Runner                   # noqa: E402

INIT_S = 3.0
TOL = 0.05
KMH = 3.6
CALIB = bagio.ROOT / "ros2_ws" / "src" / "tram_state_estimator" / "config" / "tram_calibration.json"


def tram_params():
    d = json.loads(CALIB.read_text(encoding="utf-8"))["params"]
    return Params.from_dict(d)


def track_map():
    """Карта: TRAM_MAP=<файл>, иначе оценочная карта из обучающих прогонов
    config/eval/track_map.npz (build_map.py eval). Боевая карта жюри
    (config/track_map.npz, все прогоны) для чисел на отложенных не годится."""
    import os
    try:
        from tram_state_estimator.track_map import TrackMap
        f = os.environ.get("TRAM_MAP") or (
            bagio.ROOT / "ros2_ws" / "src" / "tram_state_estimator" / "config" / "eval" / "track_map.npz")
        from pathlib import Path
        f = Path(f)
        return TrackMap.load(f) if f.exists() else None
    except ImportError:
        return None


def events(a):
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in a[key]:
            ev.append((tb, 0, i, th, v))
    for tb, th, n in a["cmd"]:
        ev.append((tb, 1, 0, th, n))
    t_end = a["mfix"][0, 0] + INIT_S if len(a["mfix"]) else -1
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        for row in a[key]:
            if row[0] <= t_end:
                ev.append((row[0], 2, ant, row[1], (row[2], row[3], row[4], int(row[5]))))
    ev.sort(key=lambda e: e[0])
    return ev


def run_bag(b, params=None, tmap=None):
    a = bagio.load(b)
    r = Runner(params or tram_params(), track_map=tmap)
    outs = []
    for tb, kind, i, th, val in events(a):
        if kind == 0:
            outs += r.on_wheel(i, th, val)
        elif kind == 1:
            outs += r.on_handle(th, val)
        else:
            outs += r.on_fix(th, i, *val)
    return a, outs


def nearest(t_out, t_ref):
    j = np.clip(np.searchsorted(t_out, t_ref), 1, len(t_out) - 1)
    j = np.where(np.abs(t_out[j - 1] - t_ref) < np.abs(t_out[j] - t_ref), j - 1, j)
    return j, np.abs(t_out[j] - t_ref) <= TOL


def score(b):
    tmap = track_map()
    a, outs = run_bag(b, tmap=tmap)
    if not outs or len(a["mvel"]) < 50:
        return b, None
    T = np.array([o["stamp"] for o in outs])
    V = np.array([o["v"] for o in outs])
    X = np.array([[o["x"], o["y"], o["z"]] for o in outs])
    # скорость
    g = a["mvel"]
    j, ok = nearest(T, g[:, 1])
    vg = np.hypot(g[:, 2], g[:, 3])
    ev = V[j[ok]] - vg[ok]
    # наивная оценка: среднее тележек / 3,6
    f, r = a["front"], a["rear"]
    naive = 0.5 * (np.interp(g[:, 1], f[:, 1], f[:, 2]) + np.interp(g[:, 1], r[:, 1], r[:, 2])) / KMH
    en = naive[ok] - vg[ok]
    # положение: эталон независимо от ноды, сравнение в непрерывных UTM
    valid = np.array([bool(o.get("pos_valid", True)) for o in outs])
    t_ref, ref = georef.reference(a["mfix"], "utm")
    j2, ok2 = nearest(T, t_ref)
    ok2 &= valid[j2]
    Xk = X[j2[ok2]].copy()
    if len(Xk) and np.abs(Xk[:, 0]).max() < 2e5:        # MGRS: в квадрате
        for k in (0, 1):
            Xk[:, k] += 1e5 * np.round((ref[ok2, k] - Xk[:, k]) / 1e5)
    d3 = np.linalg.norm(Xk - ref[ok2], axis=1)
    d2 = np.linalg.norm(Xk[:, :2] - ref[ok2, :2], axis=1)
    dur = T[-1] - T[0]
    path = float(np.sum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1)))
    return b, dict(dur=dur, path=path, v_mae=float(np.mean(np.abs(ev))),
                   v_rmse=float(np.sqrt(np.mean(ev ** 2))),
                   v_max=float(np.max(np.abs(ev))),
                   naive_mae=float(np.mean(np.abs(en))),
                   p_mean=float(np.mean(d3)), p_end=float(d3[-1]),
                   p_max=float(np.max(d3)), h_mean=float(np.mean(d2)),
                   pairs=int(ok.sum()), rate=len(T) / max(dur, 1e-9))


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "val"
    if arg == "val":
        dm = json.loads((bagio.CACHE.parent / "drive_model.json").read_text(encoding="utf-8"))
        ids = dm["val"]
    elif arg == "all":
        ids = bagio.bag_ids()
    else:
        ids = [arg]
    with ProcessPoolExecutor() as ex:
        res = [(b, s) for b, s in ex.map(score, ids) if s is not None]
    print(f"{'прогон':<16}{'мин':>5}{'путь,км':>8}{'|ош v|':>8}{'наивн.':>8}{'макс v':>8}"
          f"{'ср.3D,м':>9}{'конец,м':>9}{'макс,м':>9}{'Гц':>6}")
    for b, s in sorted(res):
        print(f"{b:<16}{s['dur'] / 60:5.1f}{s['path'] / 1000:8.2f}{s['v_mae']:8.3f}"
              f"{s['naive_mae']:8.3f}{s['v_max']:8.2f}{s['p_mean']:9.1f}{s['p_end']:9.1f}"
              f"{s['p_max']:9.1f}{s['rate']:6.1f}")
    if res:
        S = [s for _, s in res]
        w = np.array([s["pairs"] for s in S], float)
        print(f"ИТОГО по {len(S)} прогонам: |ош v| {np.average([s['v_mae'] for s in S], weights=w):.3f} м/с "
              f"(наивно {np.average([s['naive_mae'] for s in S], weights=w):.3f}), "
              f"ср. 3D {np.average([s['p_mean'] for s in S], weights=w):.1f} м, "
              f"медиана конца {np.median([s['p_end'] for s in S]):.1f} м")


if __name__ == "__main__":
    main()
