"""Эталон положения для оценки — НЕЗАВИСИМО от кода ноды.

Эталон судьи — GNSS master, «плоские координаты MGRS» (ответ организаторов
25.09): x — восток, y — север, z — высота (REP-103). Здесь соглашения
написаны отдельно от tram_state_estimator/geodesy.py другими алгоритмами,
чтобы ошибка в проекции ноды была видна в метриках, а не сокращалась:

    utm       UTM по рядам Снайдера (USGS PP 1395, с. 61), абсолютные E, N,
              z — высота; зона — по первой точке (для MGRS-сравнения
              используются непрерывные E, N);
    mgrs      то же, но каждая точка в своём 100-км квадрате (E mod 100 км);
    enu       строгий ENU WGS84 от первой точки: явные формулы ECEF и поворота;
    equirect  x = Δλ·a·cos φ0, y = Δφ·a, z = Δh (прежняя формула напарника).

Совпадение с PROJ: test/test_position.py (Снайдер против Крюгера < 1 мм
на линии).
"""

import math

import numpy as np

A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)
EP2 = E2 / (1.0 - E2)
K0 = 0.9996


def _ecef(lat, lon, h):
    phi, lam = np.radians(lat), np.radians(lon)
    n = A / np.sqrt(1.0 - E2 * np.sin(phi) ** 2)
    return ((n + h) * np.cos(phi) * np.cos(lam), (n + h) * np.cos(phi) * np.sin(lam),
            (n * (1.0 - E2) + h) * np.sin(phi))


def enu(lat, lon, alt, o):
    lat, lon, alt = (np.asarray(v, float) for v in (lat, lon, alt))
    x, y, z = _ecef(lat, lon, alt)
    x0, y0, z0 = _ecef(np.float64(o[0]), np.float64(o[1]), np.float64(o[2]))
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = math.radians(o[0]), math.radians(o[1])
    e = -math.sin(lam) * dx + math.cos(lam) * dy
    n = (-math.sin(phi) * math.cos(lam) * dx - math.sin(phi) * math.sin(lam) * dy
         + math.cos(phi) * dz)
    u = (math.cos(phi) * math.cos(lam) * dx + math.cos(phi) * math.sin(lam) * dy
         + math.sin(phi) * dz)
    return np.c_[e, n, u]


def equirect(lat, lon, alt, o):
    k = math.cos(math.radians(o[0]))
    return np.c_[np.radians(np.asarray(lon, float) - o[1]) * A * k,
                 np.radians(np.asarray(lat, float) - o[0]) * A,
                 np.asarray(alt, float) - o[2]]


def zone_of(lon):
    return int(math.floor((float(lon) + 180.0) / 6.0)) % 60 + 1


def utm_snyder(lat, lon, zone=None):
    """UTM (E, N) по Снайдеру; zone=None — по первой долготе."""
    lat, lon = np.asarray(lat, float), np.asarray(lon, float)
    if zone is None:
        zone = zone_of(np.ravel(lon)[0])
    lam0 = math.radians(zone * 6.0 - 183.0)
    phi, lam = np.radians(lat), np.radians(lon)
    n = A / np.sqrt(1.0 - E2 * np.sin(phi) ** 2)
    t = np.tan(phi) ** 2
    c = EP2 * np.cos(phi) ** 2
    a = (lam - lam0) * np.cos(phi)
    e4, e6 = E2 * E2, E2 ** 3
    m = A * ((1 - E2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
             - (3 * E2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * np.sin(2 * phi)
             + (15 * e4 / 256 + 45 * e6 / 1024) * np.sin(4 * phi)
             - (35 * e6 / 3072) * np.sin(6 * phi))
    x = K0 * n * (a + (1 - t + c) * a ** 3 / 6
                  + (5 - 18 * t + t * t + 72 * c - 58 * EP2) * a ** 5 / 120)
    y = K0 * (m + n * np.tan(phi) * (a * a / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24
                                     + (61 - 58 * t + t * t + 600 * c - 330 * EP2) * a ** 6 / 720))
    y = np.where(lat < 0, y + 10000000.0, y)
    return 500000.0 + x, y


def utm_abs(lat, lon, alt, o):
    E, N = utm_snyder(lat, lon, zone_of(o[1]))
    return np.c_[E, N, np.asarray(alt, float)]


def mgrs_wrap(lat, lon, alt, o):
    P = utm_abs(lat, lon, alt, o)
    return np.c_[np.mod(P[:, 0], 1e5), np.mod(P[:, 1], 1e5), P[:, 2]]


def square_code(E, N, zone, lat):
    """Код 100-км квадрата — отдельная запись той же схемы AA (WGS84)."""
    cols = ["ABCDEFGH", "JKLMNPQR", "STUVWXYZ"][(zone - 1) % 3]
    rows = "ABCDEFGHJKLMNPQRSTUV"
    band = "CDEFGHJKLMNPQRSTUVWX"[min(int((lat + 80.0) // 8), 19)]
    col = cols[int(E // 1e5) - 1]
    row = rows[(int(N // 1e5) + (5 if zone % 2 == 0 else 0)) % 20]
    return f"{zone}{band}{col}{row}"


def grid_origin(code, N_hint):
    """Юго-западный угол квадрата вида '37UDB' (строка — ближайший к N_hint повтор)."""
    zone = int("".join(ch for ch in code if ch.isdigit()))
    col, row = code[-2], code[-1]
    cols = ["ABCDEFGH", "JKLMNPQR", "STUVWXYZ"][(zone - 1) % 3]
    rows = "ABCDEFGHJKLMNPQRSTUV"
    E0 = 1e5 * (cols.index(col) + 1)
    r = (rows.index(row) - (5 if zone % 2 == 0 else 0)) % 20
    cands = [1e5 * r + 2e6 * k for k in range(-1, 6)]
    N0 = min(cands, key=lambda n: abs(n + 5e4 - N_hint))
    return E0, N0


REF = {"enu": enu, "equirect": equirect, "utm": utm_abs, "mgrs": mgrs_wrap}


def reference(mfix, convention="utm"):
    """Эталон по массиву master fix (bagio: tb, th, lat, lon, alt, status, ...):
    начало/зона — первая годная точка; возвращает (метки th, N×3)."""
    m = mfix[np.isfinite(mfix[:, 2]) & np.isfinite(mfix[:, 3]) & np.isfinite(mfix[:, 4])
             & (mfix[:, 5] >= 0)]
    o = (float(m[0, 2]), float(m[0, 3]), float(m[0, 4]))
    return m[:, 1], REF[convention](m[:, 2], m[:, 3], m[:, 4], o)
