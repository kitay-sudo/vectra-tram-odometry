"""Метрики оценки «как у судьи» (для tools/eval.py). Только numpy.

Пары «выход — эталон» — по ближайшей метке времени в пределах TOL = 0,05 с
(README датасета, раздел 5.1). Эталон скорости — |v| GNSS master по (x, y)
(основной) и rover (дополнительный): официального эталона скорости нет,
источников четыре — 2 тележки и 2 GNSS. Эталон положения — точка base_link
по GNSS (организаторы 25.09: ось поворота передней тележки на уровне рельса;
антенны в base_link: master (−9,873; 0; 3,0), rover (2,563; 0; 3,0)):
пара master+rover одной эпохи (±0,05 с) — base_link = master + 9,873/12,436 ·
(rover − master), z − 3,0; без пары — master + 9,873 м вдоль курса эталонной
траектории, z = высота master − 3,0 (reference). Система судьи — плоские MGRS
от угла квадрата 37UCB непрерывно (как pathgraph организаторов). Прежний
эталон (антенна master) — ref_point="master".

Разложение вдоль/поперёк пути (along_cross) перенесено без изменений из
tools/core_metrics.py (независимая реализация метрик), чтобы числа
совпадали с ней.
"""

import math

import numpy as np

import eval_geo as G
import eval_pathgraph as PGM

TOL = 0.05
V_STAND_GNSS = 0.2      # м/с: ниже — фаза «стоянка» по GNSS
V_FALSE_SS = 0.5        # м/с: стоянка оценки при GNSS выше — ложная стоянка
PHASES = ("standstill", "traction", "coast", "brake")
PHASES_RU = dict(standstill="стоянка", traction="тяга", coast="выбег", brake="торможение")
STANDSTILL = 5          # estimator_core.STANDSTILL (сверяется в eval_replay)
FRAMES = ("mgrs", "utm", "enu", "equirect")
FRAMES_RU = {
    "mgrs": "MGRS (непрерывные UTM зоны начала; x — восток, y — север, z — высота)",
    "utm": "UTM зоны начала (то же, что mgrs, без учёта квадратов)",
    "enu": "строгий ENU WGS84 от первой точки master",
    "equirect": "прежний equirect от первой точки master (как core_metrics)",
}


# ------------------------------------------------------------------ системы координат

class Frame:
    """Система, в которой считаются ошибки. Для mgrs/utm — непрерывные UTM
    зоны начала минус (E, N) первой точки master (метры, без потери точности
    в float); z — абсолютная высота. Для enu/equirect — от первой точки master."""

    def __init__(self, name, origin):
        self.name = name
        self.origin = tuple(float(v) for v in origin)     # (lat, lon, alt) первой точки master
        self.zone = G.utm_zone(origin[1], origin[0])
        E0, N0 = G.utm_fwd(origin[0], origin[1], self.zone)
        self.E0, self.N0 = float(E0), float(N0)

    def fwd(self, lat, lon, alt):
        if self.name in ("mgrs", "utm"):
            E, N = G.utm_fwd(lat, lon, self.zone)
            return np.stack([np.asarray(E) - self.E0, np.asarray(N) - self.N0,
                             np.asarray(alt, float)], axis=-1)
        if self.name == "enu":
            return G.enu_fwd(lat, lon, alt, self.origin)
        if self.name == "equirect":
            return G.equirect_fwd(lat, lon, alt, self.origin)
        raise ValueError(self.name)

    def utm(self, lat, lon):
        E, N = G.utm_fwd(lat, lon, self.zone)
        return np.asarray(E, float), np.asarray(N, float)


# ------------------------------------------------------------------ пары

def nearest(t_out, t_ref, tol=TOL):
    """Индекс ближайшего выхода для каждой метки эталона и признак |Δt| ≤ tol.
    t_out — по возрастанию."""
    t_ref = np.asarray(t_ref, float)
    if len(t_out) == 0:
        return np.zeros(len(t_ref), int), np.zeros(len(t_ref), bool)
    if len(t_out) == 1:
        j = np.zeros(len(t_ref), int)
        return j, np.abs(t_out[0] - t_ref) <= tol
    j = np.clip(np.searchsorted(t_out, t_ref), 1, len(t_out) - 1)
    j = np.where(np.abs(t_out[j - 1] - t_ref) < np.abs(t_out[j] - t_ref), j - 1, j)
    return j, np.abs(t_out[j] - t_ref) <= tol


def zoh(t_src, v_src, t_q):
    """Последнее значение источника на момент t_q (причинно)."""
    o = np.argsort(t_src, kind="stable")
    ts, vs = t_src[o], v_src[o]
    j = np.searchsorted(ts, t_q, side="right") - 1
    return np.where(j >= 0, vs[np.clip(j, 0, None)], np.nan)


def phases(a, tg, vg):
    """0 стоянка (|v| GNSS < 0,2), иначе 1 тяга / 2 выбег / 3 торможение по ручке."""
    if len(a["cmd"]):
        n_h = np.nan_to_num(zoh(a["cmd"][:, 1], a["cmd"][:, 2], tg), nan=0.0)
    else:
        n_h = np.zeros(len(tg))
    return np.where(vg < V_STAND_GNSS, 0,
                    np.where(n_h > 0, 1, np.where(n_h < 0, 3, 2))).astype(np.int8)


def _rows(x, ncol):
    """Массив прогона как (n, ≥ncol); пустой или неполный — (0, ncol)."""
    x = np.asarray(x, float)
    if x.ndim != 2 or x.shape[1] < ncol:
        return np.zeros((0, ncol))
    return x


def speed_ref(a, which="master"):
    g = _rows(a["mvel" if which == "master" else "rvel"], 4)
    if len(g) == 0:
        return np.zeros(0), np.zeros(0)
    return g[:, 1], np.hypot(g[:, 2], g[:, 3])


def ref_glitch(a, tg, vg, meas_scale):
    """Выбросы самого эталона: GNSS расходится с ОБЕИМИ согласными тележками > 1 м/с."""
    f, r = a["front"], a["rear"]
    if len(f) < 2 or len(r) < 2:
        return np.zeros(len(tg), bool)
    fz = np.interp(tg, f[:, 1], f[:, 2]) / 3.6 * meas_scale
    rz = np.interp(tg, r[:, 1], r[:, 2]) / 3.6 * meas_scale
    return (np.abs(fz - vg) > 1.0) & (np.abs(rz - vg) > 1.0) & (np.abs(fz - rz) < 0.5)


# ------------------------------------------------------------------ вдоль / поперёк

EXT = 1000.0   # м: продление траектории за концы, чтобы проекция не обрезалась


def polyline(ref_xy, min_step=1.0):
    """Траектория GNSS, прореженная до шага >= min_step и продлённая на EXT м
    за оба конца. kept_of[i] — индекс последней точки до фикса i. S — дуговая
    координата (0 в первой точке GNSS); path — длина пути GNSS."""
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
    out = xy.copy()
    if len(xy) >= 5:
        from numpy.lib.stride_tricks import sliding_window_view
        for c in range(xy.shape[1]):
            out[2:-2, c] = np.median(sliding_window_view(xy[:, c], 5), axis=1)
    return out


def along_cross(ref_xy, idx, est_xy, W=1000.0, max_d=60.0):
    """Ошибка вдоль и поперёк эталонной траектории для фиксов idx.

    Эталон сглажен медианой по 5 фиксам, прорежен до 1 м и продлён за концы.
    Оценка проецируется на отрезки в окне ±W м от эталонной точки с тем же
    направлением движения; из почти равноудалённых оснований перпендикуляров
    берётся ближайшее вдоль пути. along = s_проекции − s_эталона (+ — оценка
    впереди), cross — знаковое расстояние (+ слева по ходу). Дальше max_d от
    траектории (чужая ветка) — NaN. Возвращает along, cross, s_ref, path."""
    ref_xy = median5(np.asarray(ref_xy, float))
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
        e = est_xy[q]
        if not (math.isfinite(e[0]) and math.isfinite(e[1])):
            continue
        lo = max(int(np.searchsorted(S, s_r - W)) - 1, 0)
        hi = min(int(np.searchsorted(S, s_r + W)) + 1, nseg)
        cand = np.arange(lo, hi)
        de = e - P[cand]
        t = np.clip(np.einsum("ij,ij->i", de, seg[cand]) / (L[cand] ** 2), 0.0, 1.0)
        c = P[cand] + t[:, None] * seg[cand]
        dist = np.hypot(e[0] - c[:, 0], e[1] - c[:, 1])
        dist[U[cand] @ U[k] <= 0.0] = np.inf       # встречное направление
        dmin = float(dist.min())
        if not math.isfinite(dmin) or dmin > max_d:
            continue
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


# ------------------------------------------------------------------ статистики

def _fin(x):
    x = np.asarray(x, float)
    return x[np.isfinite(x)]


def vstats(e):
    e = _fin(e)
    if not len(e):
        return None
    return dict(n=int(len(e)), rmse=float(np.sqrt(np.mean(e ** 2))), mae=float(np.mean(np.abs(e))),
                bias=float(np.mean(e)), p95=float(np.percentile(np.abs(e), 95)),
                max=float(np.max(np.abs(e))))


def _nanstat(fn, x):
    x = _fin(x)
    return float(fn(x)) if len(x) else float("nan")


def _last(x):
    x = _fin(x)
    return float(x[-1]) if len(x) else float("nan")


# ------------------------------------------------------------------ одна оценка

def score_speed(O, a, which="master", v_standstill=0.3, meas_scale=1.0):
    """Скорость одной оценки O (T, V, [SV], [MODE]) против GNSS which.
    Возвращает (row, samples)."""
    T, V = O["T"], O["V"]
    tg, vg = speed_ref(a, which)
    row = dict(v_ref=int(len(tg)), v_out_nan=int(np.sum(~np.isfinite(V))))
    if not len(tg):
        return dict(row, v_pairs=0), None
    j, ok = nearest(T, tg)
    ev_all = np.full(len(tg), np.nan)
    ev_all[ok] = V[j[ok]] - vg[ok]
    fin = np.isfinite(ev_all)
    ev = ev_all[fin]
    jj = j[fin]
    # v_nan: эталон нашёл выход в пределах 0,05 с, но скорость в нём не число
    row.update(v_pairs=int(fin.sum()), v_pair_frac=float(fin.mean()),
               v_unpaired=int(len(tg) - ok.sum()), v_nan=int(ok.sum() - fin.sum()))
    vs = vstats(ev)
    for k in ("rmse", "mae", "bias", "max", "p95"):
        row[f"v_{k}"] = vs[k] if vs else float("nan")
    s = dict(ev=ev, phase=phases(a, tg, vg)[fin], vg=vg[fin], tg=tg[fin])
    if "SV" in O and np.isfinite(O["SV"]).any():
        sv = O["SV"][jj]
        s["sv"] = sv
        row["cov2s_v"] = float(np.mean(np.abs(ev) <= 2 * sv)) if len(ev) else float("nan")
        row["cov1s_v"] = float(np.mean(np.abs(ev) <= sv)) if len(ev) else float("nan")
        row["sigma_v_med"] = float(np.median(sv)) if len(sv) else float("nan")
    mv = vg[fin] > V_FALSE_SS
    if "MODE" in O and O.get("has_mode", True):
        ss = O["MODE"][jj] == STANDSTILL
    else:
        ss = V[jj] < v_standstill
    s["false_ss"] = ss & mv
    s["moving"] = mv
    row["false_ss_n"] = int(np.sum(ss & mv))
    row["moving_n"] = int(mv.sum())
    row["false_ss_rate"] = float(np.mean(ss[mv])) if mv.any() else 0.0
    s["glitch"] = ref_glitch(a, s["tg"], s["vg"], meas_scale)
    return row, s


# tf антенн в base_link (организаторы 25.09), м; независимо от body.py пакета
MASTER_X, ROVER_X, ANTENNA_Z = -9.873, 2.563, 3.0
REF_POINTS = ("base_link", "master")
PAIR_TOL = 0.05            # с: master и rover одной эпохи
BASE_OK = (5.0, 25.0)      # м: годная база пары (≈ 12,44)
TAN_DT, TAN_MIN = 1.0, 1.0  # с, м: касательная траектории по ±1 с при смещении ≥ 1 м


def reference(a, point="base_link"):
    """Эталон положения: dict(t, lat, lon, alt, yaw, paired, zone).

    Строки — фиксы master (как в core_metrics: без фильтрации по status,
    неконечные отброшены). point "master" — сама антенна. point "base_link":
    с парой rover (ближайшая метка ±0,05 с, база 5–25 м) — по tf на отрезке
    master→rover (кузов жёсткий, base_link на оси тележки и на кривой),
    z — по той же доле между высотами антенн минус 3,0; без пары — master +
    9,873 м вдоль курса (касательная траектории master за ±1 с, на стоянке —
    курс ближайшей пары), z = высота master − 3,0; без курса строка
    отбрасывается. yaw — курс (рад от оси x против часовой, UTM) для выбора
    пути pathgraph своего направления."""
    m = _rows(a["mfix"], 5)
    m = m[np.isfinite(m[:, 2]) & np.isfinite(m[:, 3]) & np.isfinite(m[:, 4])]
    n = len(m)
    empty = np.zeros(0)
    if n == 0:
        return dict(t=empty, lat=empty, lon=empty, alt=empty, yaw=empty,
                    paired=np.zeros(0, bool), zone=None)
    t, lat, lon, alt = m[:, 1], m[:, 2], m[:, 3], m[:, 4]
    zone = G.utm_zone(lon[0], lat[0])
    Em, Nm = (np.asarray(v, float) for v in G.utm_fwd(lat, lon, zone))
    r = _rows(a["rfix"], 5)
    r = r[np.isfinite(r[:, 2]) & np.isfinite(r[:, 3]) & np.isfinite(r[:, 4])]
    paired = np.zeros(n, bool)
    Er = Nr = zr = np.full(n, np.nan)
    if len(r) >= 1:
        r = r[np.argsort(r[:, 1], kind="stable")]
        j, ok = nearest(r[:, 1], t, tol=PAIR_TOL)
        E_, N_ = (np.asarray(v, float) for v in G.utm_fwd(r[j, 2], r[j, 3], zone))
        b = np.hypot(E_ - Em, N_ - Nm)
        paired = ok & (b >= BASE_OK[0]) & (b <= BASE_OK[1])
        Er, Nr, zr = E_, N_, r[j, 4]
    yaw_pair = np.where(paired, np.arctan2(Nr - Nm, Er - Em), np.nan)
    # касательная траектории master: положение за ±TAN_DT по времени
    o = np.argsort(t, kind="stable")
    ts = t[o]
    Ea = np.interp(t + TAN_DT, ts, Em[o]) - np.interp(t - TAN_DT, ts, Em[o])
    Na = np.interp(t + TAN_DT, ts, Nm[o]) - np.interp(t - TAN_DT, ts, Nm[o])
    yaw_tan = np.where(np.hypot(Ea, Na) >= TAN_MIN, np.arctan2(Na, Ea), np.nan)
    yaw = np.where(paired, yaw_pair, yaw_tan)
    if (~np.isfinite(yaw)).any() and np.isfinite(yaw_pair).any():
        # стоянка без пары: курс ближайшей по времени пары
        k = np.flatnonzero(np.isfinite(yaw_pair))
        tk = t[k]
        oo = np.argsort(tk)
        jj, _ = nearest(tk[oo], t, tol=np.inf)
        yaw = np.where(np.isfinite(yaw), yaw, yaw_pair[k[oo][jj]])
    if point == "master":
        E, N, z = Em, Nm, alt
        keep = np.ones(n, bool)
    elif point == "base_link":
        f = -MASTER_X / (ROVER_X - MASTER_X)
        c, s_ = np.cos(yaw), np.sin(yaw)
        E = np.where(paired, Em + f * (Er - Em), Em - MASTER_X * c)
        N = np.where(paired, Nm + f * (Nr - Nm), Nm - MASTER_X * s_)
        z = np.where(paired, alt + f * (zr - alt), alt) - ANTENNA_Z
        keep = paired | np.isfinite(yaw)
    else:
        raise ValueError(f"точка эталона {point!r}: {REF_POINTS}")
    if point == "master":                       # сами фиксы, без пересчёта (как прежде)
        return dict(t=t, lat=lat, lon=lon, alt=alt, yaw=yaw, paired=paired, zone=zone)
    la, lo = G.utm_inv(E[keep], N[keep], zone)
    return dict(t=t[keep], lat=np.asarray(la, float), lon=np.asarray(lo, float),
                alt=np.asarray(z, float)[keep], yaw=yaw[keep], paired=paired[keep], zone=zone)


def reference_geo(a, point="master"):
    """Эталон положения: метки и (lat, lon, alt) точки point (по умолчанию —
    master fix, как прежде; оценка берёт base_link через reference)."""
    r = reference(a, point)
    return r["t"], r["lat"], r["lon"], r["alt"]


BOUNDARY_GRID = "37UDB"     # «непрерывно от квадрата» для матрицы соглашений на границе
JUDGE_GRID = "37UCB"        # соглашение судьи: от угла 37UCB непрерывно (pathgraph, 26.09)


def _conv(E, N, conv):
    """Плоские координаты MGRS по соглашению conv: "" — перенос по точке,
    код квадрата — непрерывно от его юго-западного угла."""
    if not conv:
        return G.wrap(E, N)
    _, gE, gN = G.grid_origin(conv)
    return np.asarray(E, float) - gE, np.asarray(N, float) - gN


def published(O):
    """Строки выхода, у которых положение опубликовано (нода не публикует
    /result/position при pos_valid = False) и конечно."""
    n = len(O["T"])
    PV = O.get("PV")
    if PV is None:
        PV = np.isfinite(O["XYZ"]).all(axis=1) if n else np.zeros(0, bool)
    return np.asarray(PV, bool)


def score_position(O, a, frame, judge_grid=JUDGE_GRID, full=True, boundary_grid=BOUNDARY_GRID,
                   ref_point="base_link", pg=None):
    """Положение оценки против эталона ref_point (reference: base_link по
    GNSS, как у судьи, или антенна master). O["GEO"] — (lat, lon, alt) выхода
    (переведённые из системы Runner'а), O["XYZ"] — сырые x, y, z выхода (для
    «взгляда судьи»), O["PV"] — положение опубликовано. Фикс сопоставляется
    с ближайшим ОПУБЛИКОВАННЫМ положением в пределах 0,05 с; без него фикс
    непарный (p_unpaired). frame — Frame. pg — eval_pathgraph.Pathgraph (или
    None): поперечная ошибка и путь вдоль pathgraph. p_gap_steps — шагов
    выхода без опубликованного положения после первого опубликованного.
    Возвращает (row, samples)."""
    PV = published(O)
    T = O["T"][PV]
    ref = reference(a, ref_point)
    tr, la, lo, al = ref["t"], ref["lat"], ref["lon"], ref["alt"]
    first = int(np.argmax(PV)) if PV.any() else len(PV)
    row = dict(p_ref=int(len(tr)), p_out=int(len(O["T"])),
               p_out_invalid=int((~O.get("POSV", PV)).sum()) if len(O["T"]) else 0,
               p_nan=int((O.get("POSV", PV) & ~PV).sum()) if len(O["T"]) else 0,
               p_gap_steps=int((~PV[first:]).sum()))
    if len(tr) < 2 or not len(T):
        return dict(row, p_pairs=0, p_unpaired=int(len(tr)), p_pair_frac=0.0), None
    P = frame.fwd(la, lo, al)
    lat_e, lon_e, alt_e = (np.asarray(g_, float)[PV] for g_ in O["GEO"])
    XYZ_raw = O["XYZ"][PV]
    X = frame.fwd(lat_e, lon_e, alt_e)
    j2, ok2 = nearest(T, tr)
    idx = np.flatnonzero(ok2)
    jp = j2[ok2]
    Xp = X[jp]
    d3 = np.linalg.norm(Xp - P[idx], axis=1)
    d2 = np.linalg.norm(Xp[:, :2] - P[idx, :2], axis=1)
    dz = Xp[:, 2] - P[idx, 2]
    row["p_pairs"] = int(np.isfinite(d3).sum())
    row["p_unpaired"] = int(len(tr) - row["p_pairs"])
    row["p_pair_frac"] = float(row["p_pairs"] / len(tr))
    row["p3d_mean"] = _nanstat(np.mean, d3)
    row["p3d_rmse"] = float(np.sqrt(_nanstat(np.mean, d3 ** 2)))
    row["p3d_max"] = _nanstat(np.max, d3)
    row["p3d_end"] = _last(d3)
    row["p2d_mean"] = _nanstat(np.mean, d2)
    row["pz_mean"] = _nanstat(np.mean, np.abs(dz))
    row["pz_bias"] = _nanstat(np.mean, dz)
    s = dict(d3=d3, d2=d2, tp=tr[idx])
    if pg is not None and len(idx):
        Er, Nr = frame.utm(la[idx], lo[idx])
        Ee, Ne = frame.utm(lat_e[jp], lon_e[jp])
        rpg, spg = PGM.score(pg, Er, Nr, al[idx], ref["yaw"][idx], Ee, Ne)
        row.update(rpg)
        s.update(spg)
        on = spg["pg_ok"]           # эталон на pathgraph (не за его концами)
        row["p3d_on_pg"] = _nanstat(np.mean, d3[on])
        row["p3d_off_pg"] = _nanstat(np.mean, d3[~on])
        row["p_pairs_off_pg"] = int((~on & np.isfinite(d3)).sum())
    # «взгляд судьи»: квадраты 100 км MGRS при переносе по точке
    if frame.name == "mgrs" and len(idx):
        Er, Nr = frame.utm(la[idx], lo[idx])
        Ee, Ne = frame.utm(lat_e[jp], lon_e[jp])
        fe = np.isfinite(Ee) & np.isfinite(Ne)
        sr = G.square_index(Er, Nr)
        se = G.square_index(np.where(fe, Ee, 0.0), np.where(fe, Ne, 0.0))
        mism = fe & ((sr[0] != se[0]) | (sr[1] != se[1]))
        row["sq_mismatch"] = int(mism.sum())
        row["ref_squares"] = sorted({G.square_letters(e, n, frame.zone)
                                     for e, n in zip(Er[::50], Nr[::50])} |
                                    {G.square_letters(float(Er[-1]), float(Nr[-1]), frame.zone)})
        # матрица соглашений на границе квадратов: наш выход (перенос по точке
        # или непрерывно от boundary_grid) × эталон судьи (то же); координаты
        # выхода — непрерывная оценка, переведённая по нашему соглашению
        dzp = Xp[:, 2] - P[idx, 2]
        for on, oc in (("wrap", ""), ("grid", boundary_grid)):
            xo, yo = _conv(Ee, Ne, oc)
            for jn, jc in (("wrap", ""), ("grid", boundary_grid)):
                xj, yj = _conv(Er, Nr, jc)
                dd = np.sqrt((xo - xj) ** 2 + (yo - yj) ** 2 + dzp ** 2)
                row[f"bx_{on}_{jn}_3d_mean"] = _nanstat(np.mean, dd)
                row[f"bx_{on}_{jn}_km"] = int(np.sum(dd > 1000.0))
        # если выход — правильный MGRS с переносом по точке, а судья тоже переносит
        # по точке: пары с другим квадратом дают ~100 км
        wx_r, wy_r = G.wrap(Er, Nr)
        wx_e, wy_e = G.wrap(Ee, Ne)
        dw = np.sqrt((wx_e - wx_r) ** 2 + (wy_e - wy_r) ** 2 + dzp ** 2)
        row["wrap_3d_mean"] = _nanstat(np.mean, dw)
        row["wrap_3d_max"] = _nanstat(np.max, dw)
        # сырые x, y, z выхода против эталона в соглашении судьи judge_grid
        jx, jy = _conv(Er, Nr, judge_grid)
        J = np.c_[jx, jy, al[idx]]
        dj = np.linalg.norm(XYZ_raw[jp] - J, axis=1)
        row["judge_raw_3d_mean"] = _nanstat(np.mean, dj)
        row["judge_raw_3d_max"] = _nanstat(np.max, dj)
        row["judge_raw_km"] = int(np.sum(dj > 1000.0))
    if not full:
        return row, s
    alg, cr, sref, L = along_cross(P[:, :2], idx, Xp[:, :2])
    row["path_m"] = float(L)
    row["along_mean"] = _nanstat(lambda x: np.mean(np.abs(x)), alg)
    row["along_rmse"] = float(np.sqrt(_nanstat(np.mean, alg ** 2)))
    row["along_max"] = _nanstat(lambda x: np.max(np.abs(x)), alg)
    row["along_bias"] = _nanstat(np.mean, alg)
    row["along_end"] = _last(alg)
    row["cross_mean"] = _nanstat(lambda x: np.mean(np.abs(x)), cr)
    row["cross_max"] = _nanstat(lambda x: np.max(np.abs(x)), cr)
    row["along_undef"] = int(np.sum(~np.isfinite(alg)))
    row["drift_pct_3d"] = 100.0 * row["p3d_end"] / L if L > 100 else float("nan")
    row["drift_pct_along"] = 100.0 * abs(row["along_end"]) / L if L > 100 else float("nan")
    if "SS" in O and np.isfinite(O["SS"]).any():
        ssp = O["SS"][PV][jp]
        m = np.isfinite(alg)
        row["cov2s_along"] = float(np.mean(np.abs(alg[m]) <= 2 * ssp[m])) if m.any() else float("nan")
        row["sigma_s_med"] = float(np.median(ssp)) if len(ssp) else float("nan")
    s.update(al=alg, cr=cr, sref=sref)
    return row, s


# ------------------------------------------------------------------ агрегирование

BX = tuple(f"bx_{o}_{j}" for o in ("wrap", "grid") for j in ("wrap", "grid"))
W_MEAN = ("v_mae", "v_bias", "cov2s_v", "cov1s_v", "p3d_mean", "p2d_mean", "pz_mean", "pz_bias",
          "along_mean", "along_bias", "cross_mean", "cov2s_along",
          "judge_raw_3d_mean", "wrap_3d_mean",
          "pg_frac", "pg_cross_mean", "pg_cross_p95", "pg_along_mean", "pg_along_bias",
          "p3d_on_pg", "p3d_off_pg",
          "pg_ref_lat_med", "pg_ref_lat_signed_med", "pg_ref_lat_p95",
          "pg_ref_dz_med") + tuple(b + "_3d_mean" for b in BX)
W_RMS = ("v_rmse", "p3d_rmse", "along_rmse", "pg_along_rmse")
MAXES = ("v_max", "p3d_max", "along_max", "cross_max", "judge_raw_3d_max", "wrap_3d_max",
         "pg_cross_max", "pg_along_max")
SUMS = ("v_ref", "v_pairs", "v_unpaired", "v_nan", "v_out_nan", "p_ref", "p_pairs", "p_unpaired",
        "p_out", "p_out_invalid", "p_nan", "p_gap_steps", "sq_mismatch", "judge_raw_km",
        "false_ss_n", "moving_n", "along_undef", "pg_pairs", "p_pairs_off_pg") + tuple(
            b + "_km" for b in BX)


def totals(rows):
    """Итог по прогонам. Средние взвешены числом пар скорости (v_pairs): для
    MAE, смещения и RMSE это совпадает с пулом всех пар; для средней 3D — «средняя по прогонам с весом пар скорости».
    Максимумы — по всем прогонам; дрейф — по прогонам длиннее 100 м. Доли
    пар — по суммам (пар / меток эталона). Прогоны без пар скорости (упал
    на старте, нет выхода) в средние не входят — их число runs_empty."""
    n_in = len([r for r in rows if r is not None])
    rows = [r for r in rows if r and r.get("v_pairs", 0) > 0]
    if not rows:
        return None
    w = np.array([r["v_pairs"] for r in rows], float)
    out = dict(runs=len(rows), runs_empty=n_in - len(rows))

    def wavg(k, sq=False):
        x = np.array([r.get(k, np.nan) for r in rows], float)
        m = np.isfinite(x)
        if not m.any():
            return float("nan")
        return float(np.sqrt(np.average(x[m] ** 2, weights=w[m]))) if sq \
            else float(np.average(x[m], weights=w[m]))
    for k in W_MEAN:
        if any(k in r for r in rows):
            out[k] = wavg(k)
    for k in W_RMS:
        if any(k in r for r in rows):
            out[k] = wavg(k, sq=True)
    for k in MAXES:
        x = [r[k] for r in rows if k in r and np.isfinite(r[k])]
        if x:
            out[k] = float(max(x))
    for k in SUMS:
        if any(k in r for r in rows):
            out[k] = int(sum(r.get(k, 0) for r in rows))
    for k in ("drift_pct_3d", "drift_pct_along", "p3d_end"):
        x = np.array([r.get(k, np.nan) for r in rows], float)
        x = x[np.isfinite(x)]
        if len(x):
            out[k + "_mean"] = float(np.mean(x))
            out[k + "_median"] = float(np.median(x))
            out[k + "_max"] = float(np.max(x))
    mv = out.get("moving_n", 0)
    out["false_ss_rate"] = out.get("false_ss_n", 0) / mv if mv else 0.0
    for p in ("v", "p"):
        if out.get(f"{p}_ref"):
            out[f"{p}_pair_frac"] = out.get(f"{p}_pairs", 0) / out[f"{p}_ref"]
    return out


def by_phase(samples_list):
    """Скорость по фазам (пулом по всем парам)."""
    samples_list = [s for s in samples_list if s is not None]
    if not samples_list:
        return {}
    ev = np.concatenate([s["ev"] for s in samples_list])
    ph = np.concatenate([s["phase"] for s in samples_list])
    sv = np.concatenate([s["sv"] for s in samples_list]) \
        if all("sv" in s for s in samples_list) else None
    out = {}
    for k, name in enumerate(PHASES):
        m = ph == k
        if not m.any():
            continue
        d = dict(share=float(m.mean()), **(vstats(ev[m]) or {}))
        if sv is not None:
            d["cov2s"] = float(np.mean(np.abs(ev[m]) <= 2 * sv[m]))
        out[name] = d
    return out


def clean_ref(samples_list):
    """Скорость без выбросов эталона (GNSS против обеих согласных тележек > 1 м/с)."""
    samples_list = [s for s in samples_list if s is not None]
    if not samples_list:
        return {}
    ev = np.concatenate([s["ev"] for s in samples_list])
    g = np.concatenate([s["glitch"] for s in samples_list])
    return dict(glitch_samples=int(g.sum()), **(vstats(ev[~g]) or {}))


def pooled_position(samples_list):
    samples_list = [s for s in samples_list if s is not None]
    if not samples_list:
        return {}
    out = {}
    for k in ("d3", "al", "cr", "pg_along", "pg_cross"):
        if all(k in s for s in samples_list):
            x = _fin(np.concatenate([s[k] for s in samples_list]))
            if len(x):
                out[k] = dict(n=int(len(x)), mean_abs=float(np.mean(np.abs(x))),
                              rmse=float(np.sqrt(np.mean(x ** 2))), max_abs=float(np.max(np.abs(x))),
                              p95_abs=float(np.percentile(np.abs(x), 95)))
    return out
