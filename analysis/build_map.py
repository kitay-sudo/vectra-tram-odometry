"""Сборка карты путей из GNSS прогонов (офлайн).

Точки GNSS master на ходу (скорость > 1 м/с) агрегируются по клеткам 1 м и
секторам курса 15°: средние широта, долгота, высота, истинный курс; вес — число
разных прогонов, прошедших через клетку в этом направлении. Точки остановок —
устойчивые стоянки не меньше 3 разных прогонов. Множитель пути (метры карты на
метр пути колёс) калибруется во ВНУТРЕННЕЙ системе ноды (UTM со сдвигом в
начало прогона, geodesy.Frame): scale_frame = "utm".

Два набора (docs/POSITION_FRAME.md):
    EVAL — только обучающие прогоны разбиения (tools/split.json: train),
           для всех чисел оценки на holdout_scored;
    JURY — все прогоны, уходит жюри в пакете (config/track_map.npz).

    python3 analysis/build_map.py eval   # -> config/eval/track_map.npz
    python3 analysis/build_map.py jury   # -> config/track_map.npz
    python3 analysis/build_map.py --set train --split tools/split.json --out F
    опции: --calib <tram_calibration.json> (meas_scale колёс), --n-runs 30

Множитель зависит от meas_scale листа: после смены листа карты пересобрать.
"""

import argparse
import json
import math
import sys

import numpy as np

import bagio

PKG = bagio.ROOT / "ros2_ws" / "src" / "tram_state_estimator"
sys.path.insert(0, str(PKG))
from tram_state_estimator.geodesy import Frame  # noqa: E402
from tram_state_estimator.estimator_core import Params  # noqa: E402
from tram_state_estimator.track_map import TrackMap  # noqa: E402

LAT0, LON0 = 55.80484, 37.42050    # только для сетки агрегации (клетки 1 м)
R = 6378137.0
OUT_JURY = PKG / "config" / "track_map.npz"
OUT_EVAL = PKG / "config" / "eval" / "track_map.npz"
SPLIT = bagio.ROOT / "tools" / "split.json"
CALIB = PKG / "config" / "tram_calibration.json"


def all_ids():
    if bagio.DATA.exists() and any(bagio.DATA.iterdir()):
        return bagio.bag_ids()
    return sorted(p.stem for p in bagio.CACHE.glob("30*.npz"))


def select_ids(which, split=SPLIT):
    if which == "all":
        return all_ids()
    sp = json.loads(split.read_text(encoding="utf-8"))
    return list(sp[which])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("preset", nargs="?", choices=["eval", "jury"])
    ap.add_argument("--set", default=None, help="train | all | ключ split-файла")
    ap.add_argument("--split", default=str(SPLIT))
    ap.add_argument("--out")
    ap.add_argument("--calib", default=str(CALIB))
    ap.add_argument("--n-runs", type=int, default=30)
    a = ap.parse_args()
    which = a.set or {"eval": "train", "jury": "all", None: "train"}[a.preset]
    out = a.out or (OUT_JURY if (a.preset == "jury" or which == "all") else OUT_EVAL)
    ids = select_ids(which, bagio.Path(a.split))
    ms = Params.from_dict(json.loads(bagio.Path(a.calib).read_text(encoding="utf-8"))
                          ["params"]).meas_scale
    tm = build(ids, which == "all" or len(ids) > 20)
    tm.stops = find_stops(ids)
    tm.scale = calibrate_scale(tm, ids, ms, a.n_runs)
    tm.scale_frame = "utm"
    bagio.Path(out).parent.mkdir(parents=True, exist_ok=True)
    tm.save(out, source=np.array(f"{which}: {len(ids)} прогонов; split {bagio.Path(a.split).name}"),
            meas_scale=np.array(ms))
    print(f"набор {which}: прогонов {len(ids)}, точек карты {len(tm.lat)}, "
          f"остановок {len(tm.stops)}, множитель пути {tm.scale:.5f} (UTM), записано {out}")


def build(ids, min2):
    k = np.cos(np.radians(LAT0))
    acc = {}
    for rid, b in enumerate(ids):
        a = bagio.load(b)
        m, v = a["mfix"], a["mvel"]
        if len(m) < 100 or len(v) < 100:
            continue
        ok = m[:, 5] >= 0
        m = m[ok]
        ve = np.interp(m[:, 1], v[:, 1], v[:, 2])
        vn = np.interp(m[:, 1], v[:, 1], v[:, 3])
        mv = np.hypot(ve, vn) > 1.0
        x = np.radians(m[:, 3] - LON0) * R * k
        y = np.radians(m[:, 2] - LAT0) * R
        jump = np.r_[False, np.hypot(np.diff(x), np.diff(y)) > 5.0]
        sel = mv & ~jump
        h = np.arctan2(ve, vn)
        cx = np.floor(x / 1.0).astype(np.int64)
        cy = np.floor(y / 1.0).astype(np.int64)
        hb = np.floor((h + np.pi) / np.radians(15.0)).astype(np.int64) % 24
        for i in np.flatnonzero(sel):
            key = (cx[i], cy[i], hb[i])
            e = acc.get(key)
            if e is None:
                e = acc[key] = [0.0, 0.0, 0.0, 0.0, 0.0, 0, set()]
            e[0] += m[i, 2]; e[1] += m[i, 3]; e[2] += m[i, 4]
            e[3] += np.sin(h[i]); e[4] += np.cos(h[i]); e[5] += 1
            e[6].add(rid)
    rows = [(e[0] / e[5], e[1] / e[5], e[2] / e[5], np.arctan2(e[3], e[4]), len(e[6]))
            for e in acc.values()]
    R_ = np.array(rows)
    # одиночные случайные клетки (выбросы) — вон; путь, пройденный хоть одним
    # прогоном дважды в разные дни, остаётся
    R_ = R_[R_[:, 4] >= (2 if min2 else 1)]
    return TrackMap(R_[:, 0], R_[:, 1], R_[:, 2], R_[:, 3], R_[:, 4].astype(float),
                    1.0, None, "utm")


def find_stops(ids, dwell=8.0, join_r=8.0, min_runs=3, max_spread=4.0):
    """Устойчивые точки остановок: платформы и стоп-линии.

    Стоянка — скорость GNSS ниже 0,2 м/с дольше dwell. Её точка — медиана
    положения, курс — последний истинный курс на ходу перед ней. Стоянки
    ближе join_r с тем же курсом объединяются; остаются точки, где стояли не
    меньше min_runs разных прогонов с разбросом вдоль пути не больше max_spread.
    """
    k = np.cos(np.radians(LAT0))
    ev = []
    for rid, b in enumerate(ids):
        a = bagio.load(b)
        m, v = a["mfix"], a["mvel"]
        if len(m) < 100 or len(v) < 100:
            continue
        m = m[m[:, 5] >= 0]
        sp = np.interp(m[:, 1], v[:, 1], np.hypot(v[:, 2], v[:, 3]))
        hd = np.arctan2(np.interp(m[:, 1], v[:, 1], v[:, 2]),
                        np.interp(m[:, 1], v[:, 1], v[:, 3]))
        still = sp < 0.2
        i = 0
        while i < len(m):
            if not still[i]:
                i += 1
                continue
            j = i
            while j + 1 < len(m) and still[j + 1]:
                j += 1
            mv = np.flatnonzero(sp[:i] > 1.0)
            if m[j, 1] - m[i, 1] >= dwell and len(mv):
                ev.append((float(np.median(m[i:j + 1, 2])), float(np.median(m[i:j + 1, 3])),
                           float(hd[mv[-1]]), rid))
            i = j + 1
    E = np.array(ev)
    x = np.radians(E[:, 1] - LON0) * R * k
    y = np.radians(E[:, 0] - LAT0) * R
    used = np.zeros(len(E), bool)
    out = []
    for i in range(len(E)):
        if used[i]:
            continue
        dh = np.abs(np.angle(np.exp(1j * (E[:, 2] - E[i, 2]))))
        grp = (~used) & (np.hypot(x - x[i], y - y[i]) <= join_r) & (dh < np.radians(35))
        used |= grp
        g = np.flatnonzero(grp)
        h = float(np.arctan2(np.sin(E[g, 2]).sum(), np.cos(E[g, 2]).sum()))
        cx, cy = x[g].mean(), y[g].mean()
        along = (x[g] - cx) * np.sin(h) + (y[g] - cy) * np.cos(h)
        runs = len(set(E[g, 3].astype(int)))
        spread = float(np.std(along))
        if runs >= min_runs and spread <= max_spread:
            out.append((float(E[g, 0].mean()), float(E[g, 1].mean()), h, max(spread, 1.0)))
    print(f"стоянок {len(E)}, устойчивых точек остановки {len(out)}")
    return np.array(out) if out else np.zeros((0, 4))


def calibrate_scale(tm, ids, meas_scale, n_runs=30):
    """Сколько метров карты (во внутренней системе UTM) приходится на метр
    пути колёс.

    Курсор идёт по карте на путь колёс (среднее тележек × meas_scale) от
    истинной точки старта; опережение вдоль пути относительно master делится
    на пройденный путь. Участки, где курсор ушёл с эталона вбок больше чем
    на 3 м (другая ветка), не учитываются.
    """
    tm.scale, tm.scale_frame = 1.0, "utm"
    runs = sorted(ids, key=lambda b: -len(bagio.load(b)["mfix"]))[:n_runs]
    fr = []
    for b in runs:
        a = bagio.load(b)
        m, g, f, r = a["mfix"], a["mvel"], a["front"], a["rear"]
        if len(m) < 1000:
            continue
        fr_ = Frame(m[0, 2], m[0, 3], m[0, 4], "utm")
        ref = fr_.fwd_arr(m[:, 2], m[:, 3], m[:, 4])
        tm.bind(fr_)
        sp = np.hypot(g[:, 2], g[:, 3])
        vg = np.interp(m[:, 1], g[:, 1], sp)
        k0 = int(np.argmax(vg > 1.0))
        j0 = np.searchsorted(g[:, 1], m[k0, 1])
        h_true = math.atan2(g[j0, 2], g[j0, 3])
        h_grid = float(fr_.scale_heading(m[k0, 2], m[k0, 3], h_true)[1][0])
        c = tm.locate(ref[k0], h_grid)
        grid = np.arange(m[k0, 1], m[-1, 1], 0.05)
        w = 0.5 * (np.interp(grid, f[:, 1], f[:, 2]) + np.interp(grid, r[:, 1], r[:, 2])) / 3.6 * meas_scale
        sw = np.interp(m[:, 1], grid, np.r_[0, np.cumsum(w[:-1] * 0.05)])
        for q in range(k0 + 1, len(m)):
            tm.advance(c, sw[q] - sw[q - 1])
            if q % 50 or vg[q] < 2 or sw[q] < 1000:
                continue
            # направление движения в осях сетки
            he = math.atan2(np.interp(m[q, 1], g[:, 1], g[:, 2]),
                            np.interp(m[q, 1], g[:, 1], g[:, 3]))
            hg = float(fr_.scale_heading(m[q, 2], m[q, 3], he)[1][0])
            ve, vn = math.sin(hg), math.cos(hg)
            dx, dy = c["x"] - ref[q, 0], c["y"] - ref[q, 1]
            if abs(-dx * vn + dy * ve) < 3.0:
                fr.append((dx * ve + dy * vn) / sw[q])
    f = float(np.median(fr))
    print(f"опережение курсора по карте: медиана {100 * f:+.3f} % пути "
          f"({len(fr)} отсчётов)")
    return 1.0 / (1.0 + f)


if __name__ == "__main__":
    main()
