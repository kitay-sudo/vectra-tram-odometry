"""Эталонная геодезия для тестов и пробы: WGS84 -> плоские системы выхода.

Независимая от пакета реализация (пакет считает свою в geodesy.py потока
«положение»). Нужна, чтобы e2e-тест и tools/ros_probe.py сравнивали выход
ноды с GNSS в той же системе, в которой нода публикует, какой бы она ни была:

    mgrs           абсолютные плоские MGRS, x — easting, y — northing внутри
                   100-км квадрата КАЖДОЙ точки (соглашение Autoware
                   gnss_poser / lanelet2 MGRSProjector: на границе квадратов
                   x скачет на 100 км); z — высота NavSatFix
    mgrs:37UDB     непрерывно от одного квадрата (западнее границы x < 0)
    utm            абсолютные UTM E/N зоны начала, z — высота
    enu            строгая касательная плоскость WGS84 в первой точке master
    equirect       прежняя формула пакета (сфера R = a, масштаб cos φ0)

Оси по REP 103: x — восток, y — север, z — вверх. Все функции векторные.
UTM — ряд Крюгера до n^4 (Karney 2011, точность лучше 1 мм в зоне); для
самопроверки есть независимый ряд Снайдера (USGS PP 1395), расхождение на
нашей линии < 1 см (test_e2e_real.py::test_refgeo_self_check).
"""

import math

import numpy as np

A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)
K0 = 0.9996
MGRS_SQUARE = 100000.0

_COLS = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")
_ROWS = "ABCDEFGHJKLMNPQRSTUV"
_BANDS = "CDEFGHJKLMNPQRSTUVWX"


def utm_zone(lon):
    return int(math.floor((float(lon) + 180.0) / 6.0)) % 60 + 1


def _arr(*v):
    return np.broadcast_arrays(*(np.asarray(x, float) for x in v))


# ------------------------------------------------------------------ UTM
_N = F / (2.0 - F)
_AR = A / (1.0 + _N) * (1.0 + _N ** 2 / 4.0 + _N ** 4 / 64.0)
_ALPHA = (_N / 2.0 - 2.0 * _N ** 2 / 3.0 + 5.0 * _N ** 3 / 16.0 + 41.0 * _N ** 4 / 180.0,
          13.0 * _N ** 2 / 48.0 - 3.0 * _N ** 3 / 5.0 + 557.0 * _N ** 4 / 1440.0,
          61.0 * _N ** 3 / 240.0 - 103.0 * _N ** 4 / 140.0,
          49561.0 * _N ** 4 / 161280.0)


def utm(lat, lon, zone=None):
    """(E, N, zone): UTM северного полушария, ряд Крюгера. zone=None — по
    долготе первой точки (вся траектория в одной зоне)."""
    lat, lon = _arr(lat, lon)
    if zone is None:
        zone = utm_zone(np.ravel(lon)[0])
    e = math.sqrt(E2)
    phi = np.radians(lat)
    lam = np.radians(lon - (6.0 * zone - 183.0))
    s = np.sin(phi)
    t = np.sinh(np.arctanh(s) - e * np.arctanh(e * s))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1.0 + t * t))
    x, y = eta.copy(), xi.copy()
    for j, a in enumerate(_ALPHA, 1):
        x = x + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
        y = y + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
    return 500000.0 + K0 * _AR * x, K0 * _AR * y, zone


def utm_snyder(lat, lon, zone):
    """Тот же UTM рядом Снайдера (USGS PP 1395, с. 61) — только для самопроверки."""
    lat, lon = _arr(lat, lon)
    ep2 = E2 / (1.0 - E2)
    phi = np.radians(lat)
    lam = np.radians(lon - (6.0 * zone - 183.0))
    sp, cp = np.sin(phi), np.cos(phi)
    Nr = A / np.sqrt(1.0 - E2 * sp * sp)
    T = np.tan(phi) ** 2
    C = ep2 * cp * cp
    Aa = lam * cp
    e4, e6 = E2 * E2, E2 * E2 * E2
    M = A * ((1 - E2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
             - (3 * E2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * np.sin(2 * phi)
             + (15 * e4 / 256 + 45 * e6 / 1024) * np.sin(4 * phi)
             - (35 * e6 / 3072) * np.sin(6 * phi))
    x = K0 * Nr * (Aa + (1 - T + C) * Aa ** 3 / 6
                   + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * Aa ** 5 / 120)
    y = K0 * (M + Nr * np.tan(phi) * (Aa ** 2 / 2 + (5 - T + 9 * C + 4 * C * C) * Aa ** 4 / 24
                                      + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * Aa ** 6 / 720))
    return 500000.0 + x, y


# ------------------------------------------------------------------ MGRS
def mgrs_square(E, N, zone, lat):
    """Код 100-км квадрата точки: '37UDB'."""
    band = _BANDS[int(math.floor((float(lat) + 80.0) / 8.0))]
    col = _COLS[(zone - 1) % 3][int(E // MGRS_SQUARE) - 1]
    row = _ROWS[(int(N // MGRS_SQUARE) + (5 if zone % 2 == 0 else 0)) % 20]
    return f"{zone}{band}{col}{row}"


def mgrs_squares(lat, lon):
    """Множество кодов квадратов, через которые проходит траектория."""
    E, N, z = utm(lat, lon)
    lat = np.ravel(np.asarray(lat, float))
    return sorted({mgrs_square(e, n, z, la) for e, n, la in
                   zip(np.ravel(E), np.ravel(N), lat)})


def mgrs_grid_base(code, n_hint):
    """(зона, E0, N0) квадрата с кодом code: '37UDB'. Строки букв повторяются
    каждые 2000 км — берётся ближайшая к northing n_hint."""
    code = code.replace(" ", "").upper()
    zone = int(code[:-3])
    col, row = code[-2], code[-1]
    E0 = (_COLS[(zone - 1) % 3].index(col) + 1) * MGRS_SQUARE
    off = 5 if zone % 2 == 0 else 0
    k0 = int(n_hint // MGRS_SQUARE)
    ks = [k for k in range(k0 - 20, k0 + 21) if _ROWS[(k + off) % 20] == row]
    k = min(ks, key=lambda k: abs(k - k0))
    return zone, E0, k * MGRS_SQUARE


def mgrs_xy(lat, lon, grid=""):
    """Плоские MGRS: grid="" — внутри квадрата каждой точки; иначе от квадрата grid."""
    lat, lon = _arr(lat, lon)
    if grid:
        n_hint = float(np.ravel(utm(lat, lon)[1])[0])
        zone, E0, N0 = mgrs_grid_base(grid, n_hint)
        E, N, _ = utm(lat, lon, zone)
        return E - E0, N - N0
    E, N, _ = utm(lat, lon)
    return np.mod(E, MGRS_SQUARE), np.mod(N, MGRS_SQUARE)


# ------------------------------------------------------------------ ENU
def _ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon)
    n = A / np.sqrt(1.0 - E2 * np.sin(la) ** 2)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo),
                     (n + h) * np.cos(la) * np.sin(lo),
                     (n * (1.0 - E2) + h) * np.sin(la)], -1)


def enu(lat, lon, alt, lat0, lon0, alt0):
    lat, lon, alt = _arr(lat, lon, alt)
    d = _ecef(lat, lon, alt) - _ecef(lat0, lon0, alt0)
    la0, lo0 = math.radians(lat0), math.radians(lon0)
    R = np.array([[-math.sin(lo0), math.cos(lo0), 0.0],
                  [-math.sin(la0) * math.cos(lo0), -math.sin(la0) * math.sin(lo0), math.cos(la0)],
                  [math.cos(la0) * math.cos(lo0), math.cos(la0) * math.sin(lo0), math.sin(la0)]])
    return d @ R.T


def equirect(lat, lon, alt, lat0, lon0, alt0):
    lat, lon, alt = _arr(lat, lon, alt)
    k = math.cos(math.radians(lat0))
    return np.stack([np.radians(lon - lon0) * A * k, np.radians(lat - lat0) * A,
                     alt - alt0], -1)


# ------------------------------------------------------------------ системы выхода
def frames(lat, lon, alt, origin=None):
    """Эталонные траектории во всех системах-кандидатах: {имя: N×3}.

    origin — (lat0, lon0, alt0) для относительных систем (enu, equirect);
    по умолчанию первая точка."""
    lat, lon, alt = (np.ravel(np.asarray(v, float)) for v in (lat, lon, alt))
    o = origin or (lat[0], lon[0], alt[0])
    out = {"enu": enu(lat, lon, alt, *o), "equirect": equirect(lat, lon, alt, *o)}
    E, N, _ = utm(lat, lon)
    out["utm"] = np.stack([E, N, alt], -1)
    out["mgrs"] = np.stack([*mgrs_xy(lat, lon, ""), alt], -1)
    for code in grid_candidates(lat, lon):
        out[f"mgrs:{code}"] = np.stack([*mgrs_xy(lat, lon, code), alt], -1)
    return out


def grid_candidates(lat, lon):
    """Квадраты траектории и их соседи по востоку/западу в той же строке:
    параметр mgrs_grid ноды может быть задан квадратом, которого кусок
    траектории не касается (например 37UDB для западного конца линии)."""
    out = set()
    for code in mgrs_squares(lat, lon):
        cols = _COLS[(int(code[:-3]) - 1) % 3]
        i = cols.index(code[-2])
        for k in (i - 1, i, i + 1):
            if 0 <= k < len(cols):
                out.add(f"{code[:-2]}{cols[k]}{code[-1]}")
    return sorted(out)


def err3d(X, ref, wrap=False):
    """Евклидова ошибка по строкам. wrap — разность x/y по модулю 100 км
    (выход и эталон в соглашении «квадрат каждой точки» могут оказаться по
    разные стороны границы квадрата)."""
    d = np.asarray(X, float) - np.asarray(ref, float)
    if wrap:
        d[:, :2] = (d[:, :2] + MGRS_SQUARE / 2) % MGRS_SQUARE - MGRS_SQUARE / 2
    return np.linalg.norm(d, axis=1)


def detect(X, fr):
    """Система, в которой выход X (N×3) ближе всего к эталону: (имя, ошибки N).
    fr — словарь frames() на тех же N точках."""
    best = None
    for name, ref in fr.items():
        e = err3d(X, ref, wrap=(name == "mgrs"))
        if best is None or e.mean() < best[1].mean():
            best = (name, e)
    return best
