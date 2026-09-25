"""Геодезия для оценки (tools/eval.py): WGS84, UTM, MGRS, строгий ENU, equirect.

Только numpy, без pyproj: оценка должна идти в чистом образе без сети.
Модуль отдельный от геодезии пакета (поток «положение») намеренно: эталон
судьи не должен зависеть от кода, который он проверяет. После слияния
интегратор может сравнить обе реализации тестом tools/eval_selftest.py.

Система судьи (ответ организаторов 25.09): плоские координаты MGRS,
x — восток (easting), y — север (northing), z — высота NavSatFix (абсолютная),
оси по REP-103. Линия пересекает границу квадратов 100 км (37U CB | 37U DB
на E = 400 км), соглашение на границе не известно:
  * grid ""      — «перенос по точке»: координаты внутри квадрата, где лежит
                   точка (так делают Autoware gnss_poser и lanelet2 MGRSProjector);
                   на границе x скачет на 100 км;
  * grid "37UDB" — непрерывные координаты от одного квадрата (западная часть
                   получает отрицательный x).
Ошибки оценки считаются в непрерывных координатах UTM; отдельно считается,
сколько точек оценки при «переносе по точке» попали бы в другой квадрат,
чем эталон.

UTM — ряд Крюгера до n^6 (Karney 2011), погрешность < 1 мм в пределах зоны.
Проверено против pyproj 3.7 (PROJ) в tools/eval_selftest.py.
"""

import math

import numpy as np

A = 6378137.0                       # WGS84, большая полуось
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)
K0 = 0.9996
N_ = F / (2.0 - F)
AB = A / (1.0 + N_) * (1.0 + N_ ** 2 / 4.0 + N_ ** 4 / 64.0 + N_ ** 6 / 256.0)
_n = N_
ALPHA = (
    _n / 2 - 2 * _n ** 2 / 3 + 5 * _n ** 3 / 16 + 41 * _n ** 4 / 180 - 127 * _n ** 5 / 288
    + 7891 * _n ** 6 / 37800,
    13 * _n ** 2 / 48 - 3 * _n ** 3 / 5 + 557 * _n ** 4 / 1440 + 281 * _n ** 5 / 630
    - 1983433 * _n ** 6 / 1935360,
    61 * _n ** 3 / 240 - 103 * _n ** 4 / 140 + 15061 * _n ** 5 / 26880
    + 167603 * _n ** 6 / 181440,
    49561 * _n ** 4 / 161280 - 179 * _n ** 5 / 168 + 6601661 * _n ** 6 / 7257600,
    34729 * _n ** 5 / 80640 - 3418889 * _n ** 6 / 1995840,
    212378941 * _n ** 6 / 319334400,
)
BETA = (
    _n / 2 - 2 * _n ** 2 / 3 + 37 * _n ** 3 / 96 - _n ** 4 / 360 - 81 * _n ** 5 / 512
    + 96199 * _n ** 6 / 604800,
    _n ** 2 / 48 + _n ** 3 / 15 - 437 * _n ** 4 / 1440 + 46 * _n ** 5 / 105
    - 1118711 * _n ** 6 / 3870720,
    17 * _n ** 3 / 480 - 37 * _n ** 4 / 840 - 209 * _n ** 5 / 4480 + 5569 * _n ** 6 / 90720,
    4397 * _n ** 4 / 161280 - 11 * _n ** 5 / 504 - 830251 * _n ** 6 / 7257600,
    4583 * _n ** 5 / 161280 - 108847 * _n ** 6 / 3991680,
    20648693 * _n ** 6 / 638668800,
)
_E = math.sqrt(E2)


def _arr(x):
    return np.asarray(x, dtype=float)


# ------------------------------------------------------------------ UTM

def utm_zone(lon, lat=0.0):
    """Номер зоны UTM (с исключениями Норвегии и Шпицбергена)."""
    lon = float(lon)
    lat = float(lat)
    z = int(math.floor((lon + 180.0) / 6.0)) + 1
    if 56.0 <= lat < 64.0 and 3.0 <= lon < 12.0:
        z = 32
    if 72.0 <= lat < 84.0 and 0.0 <= lon < 42.0:
        z = 31 if lon < 9 else 33 if lon < 21 else 35 if lon < 33 else 37
    return min(max(z, 1), 60)


def utm_fwd(lat, lon, zone):
    """(E, N) UTM северного полушария в зоне zone, м. Векторно."""
    phi = np.radians(_arr(lat))
    lam = np.radians(_arr(lon) - (zone * 6 - 183))
    t = np.sinh(np.arctanh(np.sin(phi)) - _E * np.arctanh(_E * np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1.0 + t * t))
    x, y = xi.copy(), eta.copy()
    for j, a in enumerate(ALPHA, 1):
        x = x + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
        y = y + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
    return 500000.0 + K0 * AB * y, K0 * AB * x


def utm_inv(E, N, zone):
    """(lat, lon), градусы, из UTM северного полушария. Векторно."""
    xi = _arr(N) / (K0 * AB)
    eta = (_arr(E) - 500000.0) / (K0 * AB)
    x, y = xi.copy(), eta.copy()
    for j, b in enumerate(BETA, 1):
        x = x - b * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
        y = y - b * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
    chi = np.arcsin(np.sin(x) / np.cosh(y))
    tau0 = np.tan(chi)
    tau = tau0.copy()
    for _ in range(6):              # Ньютон для конформной широты (Karney 2011, 7-9)
        s = np.sinh(_E * np.arctanh(_E * tau / np.sqrt(1 + tau * tau)))
        tp = tau * np.sqrt(1 + s * s) - s * np.sqrt(1 + tau * tau)
        d = (tau0 - tp) / np.sqrt(1 + tp * tp) * (1 + (1 - E2) * tau * tau) / (
            (1 - E2) * np.sqrt(1 + tau * tau))
        tau = tau + d
    lat = np.degrees(np.arctan(tau))
    lon = (zone * 6 - 183) + np.degrees(np.arctan2(np.sinh(y), np.cos(x)))
    return lat, lon


# ------------------------------------------------------------------ MGRS

BANDS = "CDEFGHJKLMNPQRSTUVWX"
COLS = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")
ROWS = "ABCDEFGHJKLMNPQRSTUV"


def band_letter(lat):
    i = int(math.floor((float(lat) + 80.0) / 8.0))
    return BANDS[min(max(i, 0), len(BANDS) - 1)]


def square_letters(E, N, zone):
    """Буквы квадрата 100 км MGRS (схема AA, WGS84) для точки UTM."""
    col = COLS[(zone - 1) % 3][int(math.floor(E / 1e5)) - 1]
    row = ROWS[(int(math.floor(N / 1e5)) + (5 if zone % 2 == 0 else 0)) % 20]
    return col + row


def mgrs_square(lat, lon):
    """Код квадрата 100 км, например «37UDB», для точки WGS84."""
    z = utm_zone(lon, lat)
    E, N = utm_fwd(lat, lon, z)
    return f"{z}{band_letter(lat)}{square_letters(float(E), float(N), z)}"


def grid_origin(code):
    """(зона, E0, N0) — юго-западный угол квадрата MGRS «37UDB» в UTM, м."""
    code = code.strip().upper()
    i = 0
    while i < len(code) and code[i].isdigit():
        i += 1
    zone = int(code[:i])
    band, col, row = code[i], code[i + 1], code[i + 2]
    e_idx = COLS[(zone - 1) % 3].index(col) + 1
    r_idx = ROWS.index(row)
    if zone % 2 == 0:
        r_idx = (r_idx - 5) % 20
    # сотня км по северу: ближайшая с нужным остатком mod 20 к нижнему краю полосы
    lat_lo = -80.0 + 8.0 * BANDS.index(band)
    n_lo = float(utm_fwd(lat_lo, zone * 6 - 183, zone)[1]) if lat_lo >= 0 else 0.0
    k = int(math.floor(n_lo / 1e5))
    k0 = k - 1
    while k0 % 20 != r_idx:
        k0 += 1
    return zone, e_idx * 1e5, k0 * 1e5


def wrap(E, N):
    """«Перенос по точке»: координаты внутри своего квадрата 100 км."""
    return np.mod(_arr(E), 1e5), np.mod(_arr(N), 1e5)


def square_index(E, N):
    """Индексы квадрата 100 км (для сравнения, в каком квадрате точка)."""
    return np.floor(_arr(E) / 1e5).astype(np.int64), np.floor(_arr(N) / 1e5).astype(np.int64)


def unwrap(x, start):
    """Непрерывная координата из координаты внутри квадрата (mod 1e5):
    каждое значение переносится на целое число по 100 км к предыдущему,
    первое — к start (непрерывная координата начала выставки)."""
    x = _arr(x)
    out = np.empty_like(x)
    prev = float(start)
    for i, v in enumerate(x):
        if not math.isfinite(v):
            out[i] = np.nan
            continue
        k = round((prev - v) / 1e5)
        out[i] = v + k * 1e5
        prev = out[i]
    return out


# ------------------------------------------------------------------ ENU, equirect

def ecef(lat, lon, alt):
    la, lo = np.radians(_arr(lat)), np.radians(_arr(lon))
    n = A / np.sqrt(1.0 - E2 * np.sin(la) ** 2)
    h = _arr(alt)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo),
                     (n * (1.0 - E2) + h) * np.sin(la)], axis=-1)


def _rot(lat0, lon0):
    la, lo = math.radians(lat0), math.radians(lon0)
    return np.array([[-math.sin(lo), math.cos(lo), 0.0],
                     [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def enu_fwd(lat, lon, alt, o):
    """Строгий ENU WGS84 от начала o = (lat0, lon0, alt0): N×3."""
    d = ecef(lat, lon, alt) - ecef(o[0], o[1], o[2])
    return d @ _rot(o[0], o[1]).T


def ecef_to_geo(X):
    """ECEF N×3 -> (lat, lon, alt), итерации Боуринга (< 1e-9 м на Земле)."""
    X = np.atleast_2d(_arr(X))
    x, y, z = X[:, 0], X[:, 1], X[:, 2]
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - E2))
    for _ in range(8):
        n = A / np.sqrt(1.0 - E2 * np.sin(lat) ** 2)
        alt = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1.0 - E2 * n / (n + alt)))
    n = A / np.sqrt(1.0 - E2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - n
    return np.degrees(lat), np.degrees(lon), alt


def enu_inv(x, y, z, o):
    d = np.stack([_arr(x), _arr(y), _arr(z)], axis=-1) @ _rot(o[0], o[1])
    return ecef_to_geo(d + ecef(o[0], o[1], o[2]))


def equirect_fwd(lat, lon, alt, o):
    """Формула напарника (runner.Enu до правок): сфера R = a, x = Δλ·a·cos φ0."""
    k = math.cos(math.radians(o[0]))
    return np.stack([np.radians(_arr(lon) - o[1]) * A * k, np.radians(_arr(lat) - o[0]) * A,
                     _arr(alt) - o[2]], axis=-1)


def equirect_inv(x, y, z, o):
    k = math.cos(math.radians(o[0]))
    return (o[0] + np.degrees(_arr(y) / A), o[1] + np.degrees(_arr(x) / (A * k)),
            _arr(z) + o[2])
