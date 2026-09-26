"""Сборка карты путей (офлайн): из GNSS прогонов, из pathgraph организаторов
или гибрид.

Точка вагона (--point, по умолчанию base_link). С 26.09 эталон судьи и карта
организаторов — точка base_link (ось поворота передней тележки на уровне
рельса, tf антенн: master (−9,873; 0; 3,0), rover (2,563; 0; 3,0)). Карта
строится по траектории base_link: пара master+rover одной эпохи (±0,05 с) —
base_link = master + 9,873/12,436 · (rover − master), z — по той же доле
между высотами антенн минус 3,0; без пары — master + 9,873 м вдоль курса
движения (скорость GNSS > 1 м/с), z − 3,0. Курс точки — касательная самой
траектории base_link (±1 с). --point master — прежняя карта по антенне.

Источник (--source; по умолчанию auto — hybrid, если есть pathgraph, иначе gnss):
  gnss      точки траектории на ходу (скорость > 1 м/с)
            агрегируются по клеткам 1 м и секторам курса 15°: средние широта,
            долгота, высота, истинный курс; вес — число разных прогонов,
            прошедших через клетку в этом направлении;
  pathgraph ось пути из pathgraph организаторов (_incoming/pathgraph, два
            направления, точки через 1 м, z — уровень рельса); только
            base_link; вес 1;
  hybrid    pathgraph там, где он есть, плюс точки GNSS-карты дальше
            --join-r м от pathgraph своего направления (концы линии за
            пределами pathgraph, развороты, боковые пути). Вес точки pathgraph —
            сумма весов замещённых ею точек GNSS-карты (не меньше медианы
            весов): на стрелке выбор ветки по «езженности», как в GNSS-карте.
Остановки и конечные — всегда из GNSS (стоянки и концы записей тех же
прогонов, в той же точке вагона). Множитель пути (метры карты на метр пути
колёс) калибруется во ВНУТРЕННЕЙ системе ноды (UTM со сдвигом в начало
прогона, geodesy.Frame) против той же точки вагона: scale_frame = "utm".

Два набора (docs/POSITION_FRAME.md):
    EVAL — только обучающие прогоны разбиения (tools/split.json: train),
           для всех чисел оценки на holdout_scored;
    JURY — все прогоны, уходит жюри в пакете (config/track_map.npz).

    python3 analysis/build_map.py eval   # -> config/eval/track_map.npz
    python3 analysis/build_map.py jury   # -> config/track_map.npz
    python3 analysis/build_map.py --set train --split tools/split.json --out F
    опции: --source auto|gnss|pathgraph|hybrid, --pathgraph <каталог>, --join-r 2.0,
           --point base_link|master, --calib <tram_calibration.json>
           (meas_scale колёс; по умолчанию лист своего набора: EVAL —
           config/eval/, JURY — config/), --n-runs 30, --e2e-mult 0.999
           (поправка множителя по сквозной оценке, см. ниже)

Поправка --e2e-mult выбрана ТОЛЬКО по обучающим прогонам: полная связка
(выставка, карта, привязки, онлайн-масштаб) на 68 обучающих прогонах с GNSS,
множитель карты × 0,997 / 0,998 / 0,999 / 1,000 / 1,001 даёт ср. 3D
3,95 / 3,84 / 3,69 / 3,81 / 4,75 м (tools/position_eval.py, теги tr_*; карта
по master, 25.09). Перебег курсора вреднее недобега: привязка к остановке
ловит остановку в окне впереди хуже, чем позади.

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
from tram_state_estimator.body import Body  # noqa: E402
from tram_state_estimator.geodesy import Frame, utm_fwd, utm_inv  # noqa: E402
from tram_state_estimator.estimator_core import Params  # noqa: E402
from tram_state_estimator.track_map import TrackMap  # noqa: E402

LAT0, LON0 = 55.80484, 37.42050    # только для сетки агрегации (клетки 1 м)
R = 6378137.0
ZONE = 37                          # зона UTM линии
OUT_JURY = PKG / "config" / "track_map.npz"
OUT_EVAL = PKG / "config" / "eval" / "track_map.npz"
SPLIT = bagio.ROOT / "tools" / "split.json"
PATHGRAPH = bagio.ROOT / "_incoming" / "pathgraph"
CALIB = PKG / "config" / "tram_calibration.json"            # лист ЖЮРИ (все данные)
CALIB_EVAL = PKG / "config" / "eval" / "tram_calibration.json"  # лист ОЦЕНКИ (train)
PAIR_TOL = 0.05                    # с: master и rover одной эпохи
BASE_OK = (5.0, 25.0)              # м: годная база пары
BODY = Body()


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
    ap.add_argument("--calib", default=None,
                    help="калибровка (meas_scale колёс); по умолчанию лист своего "
                         "набора: EVAL — config/eval/tram_calibration.json, JURY — "
                         "config/tram_calibration.json")
    ap.add_argument("--n-runs", type=int, default=30)
    ap.add_argument("--e2e-mult", type=float, default=0.999)
    ap.add_argument("--point", default="base_link", choices=["base_link", "master"],
                    help="точка вагона, по траектории которой строится карта")
    ap.add_argument("--source", default="auto", choices=["auto", "gnss", "pathgraph", "hybrid"],
                    help="auto (по умолчанию) — hybrid, если есть pathgraph (--pathgraph), иначе "
                         "gnss (выбор на train, docs/POSITION_FRAME.md §3)")
    ap.add_argument("--pathgraph", default=str(PATHGRAPH),
                    help="каталог pathgraph организаторов (или файл, или список через «;»)")
    ap.add_argument("--join-r", type=float, default=2.0,
                    help="hybrid: точки GNSS-карты ближе этого к pathgraph своего "
                         "направления заменяются pathgraph, м")
    a = ap.parse_args()
    which = a.set or {"eval": "train", "jury": "all", None: "train"}[a.preset]
    out = a.out or (OUT_JURY if (a.preset == "jury" or which == "all") else OUT_EVAL)
    ids = select_ids(which, bagio.Path(a.split))
    if a.calib is None:
        # карта ОЦЕНКИ — с масштабом колёс листа ОЦЕНКИ (только train), иначе
        # множитель карты подогнан к масштабу листа жюри (утечка через масштаб)
        a.calib = str(CALIB if which == "all" or not CALIB_EVAL.exists() else CALIB_EVAL)
    if a.source == "auto":
        from tram_state_estimator.track_map import _pathgraph_files
        have = all(f.exists() for f in _pathgraph_files(a.pathgraph)) and \
            bool(_pathgraph_files(a.pathgraph))
        a.source = "hybrid" if have and a.point == "base_link" else "gnss"
        if a.source == "gnss":
            print(f"pathgraph {a.pathgraph} не найден: карта только из GNSS (--source gnss)")
    if a.source != "gnss" and a.point != "base_link":
        sys.exit("pathgraph — ось пути точки base_link: --point base_link")
    ms = Params.from_dict(json.loads(bagio.Path(a.calib).read_text(encoding="utf-8"))
                          ["params"]).meas_scale
    tracks = {b: track(bagio.load(b), a.point) for b in ids}
    min2 = which == "all" or len(ids) > 20
    tm = make_map(a.source, tracks, min2, a.pathgraph, a.join_r, a.point)
    tm.stops = find_stops(tracks)
    tm.terminals = find_terminals(tracks)
    # множитель пути — свойство колёс, не карты: для pathgraph без продолжения
    # за его концами калибровка курсором от начала записи (оно за концом
    # pathgraph) не работает — калибруется по гибриду (та же геометрия там,
    # где pathgraph есть)
    cal = (make_map("hybrid", tracks, min2, a.pathgraph, a.join_r, a.point)
           if a.source == "pathgraph" else tm)
    tm.scale = calibrate_scale(cal, tracks, ms, a.n_runs) * a.e2e_mult
    tm.scale_frame = "utm"
    bagio.Path(out).parent.mkdir(parents=True, exist_ok=True)
    src = f"{which}: {len(ids)} прогонов; split {bagio.Path(a.split).name}; " \
          f"точка {a.point}; источник {a.source}"
    tm.save(out, source=np.array(src), meas_scale=np.array(ms), e2e_mult=np.array(a.e2e_mult),
            map_source=np.array(a.source))
    print(f"набор {which}: прогонов {len(ids)}, точка {a.point}, источник {a.source}: "
          f"точек карты {len(tm.lat)}, остановок {len(tm.stops)}, конечных "
          f"{len(tm.terminals)}, множитель пути {tm.scale:.5f} (UTM, с поправкой "
          f"×{a.e2e_mult}), записано {out}")


# ------------------------------------------------------------------ траектория точки вагона

def track(a, point="base_link"):
    """Траектория точки вагона по GNSS прогона: dict(t, lat, lon, alt, head,
    speed, E, N) — по годным фиксам master (status ≥ 0). head — истинный курс
    (рад от севера по часовой): для base_link — касательная её траектории за
    ±1 с (при смещении ≥ 1,5 м), иначе курс скорости GNSS master; speed —
    |v| GNSS master. Пустой dict, если фиксов < 100."""
    m, v = a["mfix"], a["mvel"]
    if len(m) < 100 or len(v) < 100:
        return {}
    m = m[(m[:, 5] >= 0) & np.isfinite(m[:, 2]) & np.isfinite(m[:, 3]) & np.isfinite(m[:, 4])]
    if len(m) < 100:
        return {}
    m = m[np.argsort(m[:, 1], kind="stable")]              # по меткам (стартовый всплеск)
    v = v[np.argsort(v[:, 1], kind="stable")]
    t = m[:, 1]
    ve = np.interp(t, v[:, 1], v[:, 2])
    vn = np.interp(t, v[:, 1], v[:, 3])
    speed = np.hypot(ve, vn)
    h_vel = np.arctan2(ve, vn)                              # истинный курс скорости
    Em, Nm = (np.asarray(x, float) for x in utm_fwd(m[:, 2], m[:, 3], ZONE))
    if point == "master":
        return dict(t=t, lat=m[:, 2], lon=m[:, 3], alt=m[:, 4], head=h_vel, speed=speed,
                    E=Em, N=Nm, raw=m)
    fr = Frame(float(m[0, 2]), float(m[0, 3]), 0.0, "utm", utm_zone=ZONE)
    conv = fr.scale_heading(m[:, 2], m[:, 3], np.zeros(len(m)))[1]   # курс сетки истинного севера
    r = a["rfix"]
    paired = np.zeros(len(m), bool)
    E, N, z = Em.copy(), Nm.copy(), m[:, 4].copy()
    if len(r) >= 2:
        r = r[(r[:, 5] >= 0) & np.isfinite(r[:, 2]) & np.isfinite(r[:, 3]) & np.isfinite(r[:, 4])]
        r = r[np.argsort(r[:, 1], kind="stable")]
    if len(r) >= 2:
        j = np.clip(np.searchsorted(r[:, 1], t), 1, len(r) - 1)
        j = np.where(np.abs(r[j - 1, 1] - t) < np.abs(r[j, 1] - t), j - 1, j)
        Er, Nr = (np.asarray(x, float) for x in utm_fwd(r[j, 2], r[j, 3], ZONE))
        b = np.hypot(Er - Em, Nr - Nm)
        paired = (np.abs(r[j, 1] - t) <= PAIR_TOL) & (b >= BASE_OK[0]) & (b <= BASE_OK[1])
        f = BODY.frac("base_link")
        E = np.where(paired, Em + f * (Er - Em), E)
        N = np.where(paired, Nm + f * (Nr - Nm), N)
        z = np.where(paired, z + f * (r[j, 4] - z), z)
    # без пары: вперёд по курсу движения (курс сетки = истинный + сближение)
    hg = h_vel + conv
    lone = ~paired & (speed > 1.0)
    d = -BODY.master_x
    E = np.where(lone, Em + d * np.sin(hg), E)
    N = np.where(lone, Nm + d * np.cos(hg), N)
    keep = paired | lone
    z = z - BODY.antenna_z
    t, E, N, z, speed, hg, conv = (x[keep] for x in (t, E, N, z, speed, hg, conv))
    if len(t) < 100:
        return {}
    # курс — касательная траектории base_link за ±1 с (кривая: ось тележки)
    dE = np.interp(t + 1.0, t, E) - np.interp(t - 1.0, t, E)
    dN = np.interp(t + 1.0, t, N) - np.interp(t - 1.0, t, N)
    tan_ok = np.hypot(dE, dN) >= 1.5
    head = np.where(tan_ok, np.arctan2(dE, dN), hg) - conv   # истинный курс
    lat, lon = utm_inv(E, N, ZONE)
    return dict(t=t, lat=np.asarray(lat, float), lon=np.asarray(lon, float), alt=z,
                head=np.angle(np.exp(1j * head)), speed=speed, E=E, N=N)


# ------------------------------------------------------------------ карты

def make_map(source, tracks, min2, pg_path, join_r, point):
    if source == "gnss":
        return build(tracks, min2, point)
    pg = TrackMap.from_pathgraph(pg_path)
    pg.scale_frame = "utm"
    if source == "pathgraph":
        print(f"pathgraph: точек {len(pg.lat)}")
        return pg
    return hybrid(pg, build(tracks, min2, point), join_r)


def build(tracks, min2, point="base_link"):
    k = np.cos(np.radians(LAT0))
    acc = {}
    for rid, (b, tr) in enumerate(tracks.items()):
        if not tr:
            continue
        x = np.radians(tr["lon"] - LON0) * R * k
        y = np.radians(tr["lat"] - LAT0) * R
        mv = tr["speed"] > 1.0
        jump = np.r_[False, np.hypot(np.diff(x), np.diff(y)) > 5.0]
        sel = mv & ~jump
        h = tr["head"]
        cx = np.floor(x / 1.0).astype(np.int64)
        cy = np.floor(y / 1.0).astype(np.int64)
        hb = np.floor((h + np.pi) / np.radians(15.0)).astype(np.int64) % 24
        for i in np.flatnonzero(sel):
            key = (cx[i], cy[i], hb[i])
            e = acc.get(key)
            if e is None:
                e = acc[key] = [0.0, 0.0, 0.0, 0.0, 0.0, 0, set()]
            e[0] += tr["lat"][i]; e[1] += tr["lon"][i]; e[2] += tr["alt"][i]
            e[3] += np.sin(h[i]); e[4] += np.cos(h[i]); e[5] += 1
            e[6].add(rid)
    rows = [(e[0] / e[5], e[1] / e[5], e[2] / e[5], np.arctan2(e[3], e[4]), len(e[6]))
            for e in acc.values()]
    R_ = np.array(rows)
    # одиночные случайные клетки (выбросы) — вон; путь, пройденный хоть одним
    # прогоном дважды в разные дни, остаётся
    R_ = R_[R_[:, 4] >= (2 if min2 else 1)]
    return TrackMap(R_[:, 0], R_[:, 1], R_[:, 2], R_[:, 3], R_[:, 4].astype(float),
                    1.0, None, "utm", point=point)


def hybrid(pg, gm, join_r=2.0, max_dh=math.radians(35.0)):
    """pathgraph + точки GNSS-карты gm дальше join_r от pathgraph своего
    направления. Вес точки pathgraph — сумма весов замещённых ею точек gm, не
    меньше медианы весов gm."""
    Ep, Np = (np.asarray(x, float) for x in utm_fwd(pg.lat, pg.lon, ZONE))
    Eg, Ng = (np.asarray(x, float) for x in utm_fwd(gm.lat, gm.lon, ZONE))
    near = np.full(len(Eg), -1)
    dmin = np.full(len(Eg), np.inf)
    for a in range(0, len(Eg), 500):
        d2 = (Eg[a:a + 500, None] - Ep[None, :]) ** 2 + (Ng[a:a + 500, None] - Np[None, :]) ** 2
        dh = np.abs(np.angle(np.exp(1j * (gm.head[a:a + 500, None] - pg.head[None, :]))))
        d2 = np.where(dh <= max_dh, d2, np.inf)
        i = np.argmin(d2, axis=1)
        near[a:a + 500] = i
        dmin[a:a + 500] = np.sqrt(d2[np.arange(len(i)), i])
    drop = dmin <= join_r
    w = np.zeros(len(pg.lat))
    np.add.at(w, near[drop], gm.weight[drop])
    w = np.maximum(w, float(np.median(gm.weight)))
    keep = ~drop
    print(f"гибрид: pathgraph {len(pg.lat)} точек; GNSS-карта {len(gm.lat)}, заменено "
          f"{int(drop.sum())} (ближе {join_r} м), оставлено {int(keep.sum())}; расстояние "
          f"точек GNSS-карты до pathgraph своего направления: медиана "
          f"{np.median(dmin[np.isfinite(dmin)]):.2f} м, 95 % {np.percentile(dmin[np.isfinite(dmin)], 95):.2f} м")
    return TrackMap(np.r_[pg.lat, gm.lat[keep]], np.r_[pg.lon, gm.lon[keep]],
                    np.r_[pg.alt, gm.alt[keep]], np.r_[pg.head, gm.head[keep]],
                    np.r_[w, gm.weight[keep]], 1.0, None, "utm", point="base_link")


def find_stops(tracks, dwell=8.0, join_r=8.0, min_runs=3, max_spread=4.0):
    """Устойчивые точки остановок: платформы и стоп-линии.

    Стоянка — скорость GNSS ниже 0,2 м/с дольше dwell. Её точка — медиана
    положения (точки вагона карты), курс — последний истинный курс на ходу
    перед ней. Стоянки ближе join_r с тем же курсом объединяются; остаются
    точки, где стояли не меньше min_runs разных прогонов с разбросом вдоль
    пути не больше max_spread.
    """
    k = np.cos(np.radians(LAT0))
    ev = []
    for rid, (b, tr) in enumerate(tracks.items()):
        if not tr:
            continue
        sp, t, hd = tr["speed"], tr["t"], tr["head"]
        still = sp < 0.2
        i = 0
        n = len(t)
        while i < n:
            if not still[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and still[j + 1]:
                j += 1
            mv = np.flatnonzero(sp[:i] > 1.0)
            if t[j] - t[i] >= dwell and len(mv):
                ev.append((float(np.median(tr["lat"][i:j + 1])),
                           float(np.median(tr["lon"][i:j + 1])), float(hd[mv[-1]]), rid))
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


def find_terminals(tracks, join_r=50.0, min_runs=3):
    """Известные конечные: места, где начинались или кончались записи не
    меньше min_runs разных прогонов (первая и последняя точка траектории).
    Дубликаты прогонов (та же точка до 1e-7°) считаются один раз. Тупик карты
    рядом с конечной — настоящий тупик (удержание курсора, WP11), в других
    местах — разрыв карты (track_map.py)."""
    k = np.cos(np.radians(LAT0))
    pts = {}
    for b, tr in tracks.items():
        if not tr:
            continue
        for i in (0, -1):
            pts[(round(float(tr["lat"][i]), 7), round(float(tr["lon"][i]), 7))] = b
    P = np.array(list(pts.keys()))
    run = np.array(list(pts.values()))
    x = np.radians(P[:, 1] - LON0) * R * k
    y = np.radians(P[:, 0] - LAT0) * R
    used = np.zeros(len(P), bool)
    out = []
    for i in np.argsort(x):
        if used[i]:
            continue
        grp = (~used) & (np.hypot(x - x[i], y - y[i]) <= join_r)
        used |= grp
        if len(set(run[grp])) >= min_runs:
            out.append((float(P[grp, 0].mean()), float(P[grp, 1].mean())))
    print(f"концов записей {len(P)}, известных конечных {len(out)}: "
          + ", ".join(f"({la:.5f}, {lo:.5f})" for la, lo in out))
    return np.array(out) if out else np.zeros((0, 2))


def calibrate_scale(tm, tracks, meas_scale, n_runs=30):
    """Сколько метров карты (во внутренней системе UTM) приходится на метр
    пути колёс.

    Курсор идёт по карте на путь колёс (среднее тележек × meas_scale) от
    истинной точки старта (та же точка вагона, что у карты); опережение вдоль
    пути относительно траектории этой точки по GNSS делится на пройденный
    путь. Участки, где курсор ушёл с эталона вбок больше чем на 3 м (другая
    ветка), не учитываются.
    """
    tm.scale, tm.scale_frame = 1.0, "utm"
    runs = sorted((b for b, tr in tracks.items() if tr),
                  key=lambda b: -len(tracks[b]["t"]))[:n_runs]
    fr = []
    for b in runs:
        tr = tracks[b]
        if len(tr["t"]) < 1000:
            continue
        a = bagio.load(b)
        f, r = a["front"], a["rear"]
        fr_ = Frame(tr["lat"][0], tr["lon"][0], tr["alt"][0], "utm")
        ref = fr_.fwd_arr(tr["lat"], tr["lon"], tr["alt"])
        tm.bind(fr_)
        vg = tr["speed"]
        t = tr["t"]
        k0 = int(np.argmax(vg > 1.0))
        h_grid = float(fr_.scale_heading(tr["lat"][k0], tr["lon"][k0], tr["head"][k0])[1][0])
        c = tm.locate(ref[k0], h_grid)
        grid = np.arange(t[k0], t[-1], 0.05)
        w = 0.5 * (np.interp(grid, f[:, 1], f[:, 2]) + np.interp(grid, r[:, 1], r[:, 2])) / 3.6 * meas_scale
        sw = np.interp(t, grid, np.r_[0, np.cumsum(w[:-1] * 0.05)])
        for q in range(k0 + 1, len(t)):
            tm.advance(c, sw[q] - sw[q - 1])
            if q % 50 or vg[q] < 2 or sw[q] < 1000:
                continue
            # направление движения в осях сетки
            hg = float(fr_.scale_heading(tr["lat"][q], tr["lon"][q], tr["head"][q])[1][0])
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
