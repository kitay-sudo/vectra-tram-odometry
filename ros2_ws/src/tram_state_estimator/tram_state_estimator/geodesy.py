"""Геодезия WGS84 без внешних библиотек: UTM (ряды Крюгера до n^6, Karney
2011), MGRS (зона, пояс, буквы 100-км квадрата, координаты в квадрате), ENU
(геодезические -> ECEF -> касательная плоскость) и прежняя плоская формула
(equirect).

Системы координат ноды (docs/POSITION_FRAME.md):

* ВНУТРЕННЯЯ — непрерывная метрическая: UTM выбранной зоны со сдвигом в точку
  выставки, x = E − E0, y = N − N0, z — абсолютная высота (м). В ней идут
  движение по карте, счисление пути и привязки. Разрывов 100-км квадратов в
  ней нет.
* ВЫХОДНАЯ (/result/position) — параметр `projection`, перевод только при
  публикации (Frame.out):

      mgrs      (по умолчанию) плоские координаты MGRS, как ответили
                организаторы 25.09: x — восток, y — север в 100-км квадрате
                (REP-103), z — абсолютная высота. mgrs_grid "" — каждая точка
                в своём квадрате (соглашение Autoware gnss_poser / lanelet2
                MGRSProjector: на границе x прыгает на 100 км); mgrs_grid
                "37UDB" — непрерывно от угла этого квадрата (к западу от
                него x < 0).
      utm       абсолютные E, N зоны; z — абсолютная высота.
      enu       строгий ENU от точки выставки (первая точка master): x, y, z
                — восток, север, вверх относительно неё.
      equirect  прежняя формула: x = Δλ·a·cos φ0, y = Δφ·a, z = Δh.

Все формулы проверены против PROJ (pyproj 3.7.1) и пакета `mgrs` (NGA
GeoTrans) в test/test_position.py.
"""

import math

import numpy as np

A_WGS = 6378137.0
F_WGS = 1.0 / 298.257223563
E2_WGS = F_WGS * (2.0 - F_WGS)
_E = math.sqrt(E2_WGS)
UTM_K0 = 0.9996
UTM_E0 = 500000.0
UTM_N0_SOUTH = 10000000.0

PROJECTIONS = ("mgrs", "utm", "enu", "equirect")
_ALIASES = {"enu_wgs84": "enu", "ltp": "enu", "equirectangular": "equirect",
            "flat": "equirect", "legacy": "equirect", "": "mgrs"}


def projection_name(name):
    """Каноническое имя выходной проекции; неизвестное -> ValueError."""
    key = str(name if name is not None else "mgrs").strip().lower()
    key = _ALIASES.get(key, key)
    if key not in PROJECTIONS:
        raise ValueError(f"неизвестная проекция {name!r}: ожидается "
                         f"{' | '.join(PROJECTIONS)}")
    return key


# ------------------------------------------------------------------ ECEF / ENU
def ecef(lat, lon, alt):
    """Геодезические WGS84 (градусы, м) -> ECEF (м), массив N×3."""
    la = np.radians(np.asarray(lat, float))
    lo = np.radians(np.asarray(lon, float))
    h = np.asarray(alt, float)
    sl = np.sin(la)
    n = A_WGS / np.sqrt(1.0 - E2_WGS * sl * sl)
    cl = np.cos(la)
    return np.stack(np.broadcast_arrays((n + h) * cl * np.cos(lo),
                                        (n + h) * cl * np.sin(lo),
                                        (n * (1.0 - E2_WGS) + h) * sl), axis=-1)


class Enu:
    """Строгий ENU WGS84 с началом (lat0, lon0, alt0)."""

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = float(lat0), float(lon0), float(alt0)
        self._o = ecef(self.lat0, self.lon0, self.alt0).reshape(3)
        la, lo = math.radians(self.lat0), math.radians(self.lon0)
        sla, cla, slo, clo = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        self._R = np.array([[-slo, clo, 0.0],
                            [-sla * clo, -sla * slo, cla],
                            [cla * clo, cla * slo, sla]])

    def fwd_arr(self, lat, lon, alt):
        d = ecef(lat, lon, alt).reshape(-1, 3) - self._o
        return d @ self._R.T

    def fwd(self, lat, lon, alt):
        p = self.fwd_arr(lat, lon, alt)[0]
        return float(p[0]), float(p[1]), float(p[2])


class Equirect:
    """Прежняя плоская формула (сфера R = a, масштаб cos φ0)."""

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = float(lat0), float(lon0), float(alt0)
        self.k = math.cos(math.radians(self.lat0))

    def fwd_arr(self, lat, lon, alt):
        lat, lon, alt = np.broadcast_arrays(*(np.asarray(v, float).reshape(-1)
                                              for v in (lat, lon, alt)))
        return np.c_[np.radians(lon - self.lon0) * A_WGS * self.k,
                     np.radians(lat - self.lat0) * A_WGS, alt - self.alt0]

    def fwd(self, lat, lon, alt):
        p = self.fwd_arr(lat, lon, alt)[0]
        return float(p[0]), float(p[1]), float(p[2])


# ------------------------------------------------------------------ UTM
_N = F_WGS / (2.0 - F_WGS)
_A_RECT = A_WGS / (1.0 + _N) * (1.0 + _N ** 2 / 4.0 + _N ** 4 / 64.0 + _N ** 6 / 256.0)
_n = _N
_ALPHA = (
    _n / 2 - 2 * _n ** 2 / 3 + 5 * _n ** 3 / 16 + 41 * _n ** 4 / 180
    - 127 * _n ** 5 / 288 + 7891 * _n ** 6 / 37800,
    13 * _n ** 2 / 48 - 3 * _n ** 3 / 5 + 557 * _n ** 4 / 1440
    + 281 * _n ** 5 / 630 - 1983433 * _n ** 6 / 1935360,
    61 * _n ** 3 / 240 - 103 * _n ** 4 / 140 + 15061 * _n ** 5 / 26880
    + 167603 * _n ** 6 / 181440,
    49561 * _n ** 4 / 161280 - 179 * _n ** 5 / 168 + 6601661 * _n ** 6 / 7257600,
    34729 * _n ** 5 / 80640 - 3418889 * _n ** 6 / 1995840,
    212378941 * _n ** 6 / 319334400,
)
_BETA = (
    _n / 2 - 2 * _n ** 2 / 3 + 37 * _n ** 3 / 96 - _n ** 4 / 360
    - 81 * _n ** 5 / 512 + 96199 * _n ** 6 / 604800,
    _n ** 2 / 48 + _n ** 3 / 15 - 437 * _n ** 4 / 1440 + 46 * _n ** 5 / 105
    - 1118711 * _n ** 6 / 3870720,
    17 * _n ** 3 / 480 - 37 * _n ** 4 / 840 - 209 * _n ** 5 / 4480
    + 5569 * _n ** 6 / 90720,
    4397 * _n ** 4 / 161280 - 11 * _n ** 5 / 504 - 830251 * _n ** 6 / 7257600,
    4583 * _n ** 5 / 161280 - 108847 * _n ** 6 / 3991680,
    20648693 * _n ** 6 / 638668800,
)
del _n


def utm_zone(lon, lat=0.0):
    """Номер зоны UTM по долготе, с исключениями Норвегии и Шпицбергена."""
    return _utm_zone(lon, lat)


def _utm_zone(lon, lat=0.0):
    lon = (float(lon) + 180.0) % 360.0 - 180.0
    z = int(math.floor((lon + 180.0) / 6.0)) % 60 + 1
    lat = float(lat)
    if 56.0 <= lat < 64.0 and 3.0 <= lon < 12.0:
        return 32
    if 72.0 <= lat < 84.0 and lon >= 0.0:
        if lon < 9.0:
            return 31
        if lon < 21.0:
            return 33
        if lon < 33.0:
            return 35
        if lon < 42.0:
            return 37
    return z


def utm_cm(zone):
    """Долгота осевого меридиана зоны, градусы."""
    return zone * 6.0 - 183.0


def utm_fwd(lat, lon, zone, north=None):
    """Геодезические WGS84 -> UTM (E, N), м, в заданной зоне (можно вне её).
    north=None — полушарие по знаку широты (для юга N + 10 000 км)."""
    lat_a = np.asarray(lat, float)
    phi = np.radians(lat_a)
    dl = np.radians(np.asarray(lon, float) - utm_cm(zone))
    dl = (dl + np.pi) % (2.0 * np.pi) - np.pi
    sp = np.sin(phi)
    t = np.sinh(np.arctanh(sp) - _E * np.arctanh(_E * sp))   # tg конформной широты
    xi = np.arctan2(t, np.cos(dl))
    eta = np.arctanh(np.sin(dl) / np.sqrt(1.0 + t * t))
    E, N = eta.copy(), xi.copy()
    for j, a in enumerate(_ALPHA, 1):
        E = E + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
        N = N + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
    E = UTM_E0 + UTM_K0 * _A_RECT * E
    N = UTM_K0 * _A_RECT * N
    south = (lat_a < 0.0) if north is None else (not north)
    N = np.where(south, N + UTM_N0_SOUTH, N)
    return E, N


def utm_inv(E, N, zone, north=True):
    """UTM (E, N) зоны -> геодезические WGS84 (lat, lon), градусы."""
    xi = (np.asarray(N, float) - (0.0 if north else UTM_N0_SOUTH)) / (UTM_K0 * _A_RECT)
    eta = (np.asarray(E, float) - UTM_E0) / (UTM_K0 * _A_RECT)
    xi_, eta_ = xi.copy(), eta.copy()
    for j, b in enumerate(_BETA, 1):
        xi_ = xi_ - b * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
        eta_ = eta_ - b * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
    she, cxi = np.sinh(eta_), np.cos(xi_)
    tp = np.sin(xi_) / np.sqrt(she * she + cxi * cxi)      # tg конформной широты
    lam = np.arctan2(she, cxi)
    # tg геодезической широты по tg конформной: Ньютон (Karney 2011, ур. 19-21)
    tau = tp.copy()
    for _ in range(6):
        sig = np.sinh(_E * np.arctanh(_E * tau / np.sqrt(1.0 + tau * tau)))
        tpi = tau * np.sqrt(1.0 + sig * sig) - sig * np.sqrt(1.0 + tau * tau)
        dtau = ((tp - tpi) / np.sqrt(1.0 + tpi * tpi)
                * (1.0 + (1.0 - E2_WGS) * tau * tau)
                / ((1.0 - E2_WGS) * np.sqrt(1.0 + tau * tau)))
        tau = tau + dtau
    lat = np.degrees(np.arctan(tau))
    lon = utm_cm(zone) + np.degrees(lam)
    return lat, lon


# ------------------------------------------------------------------ MGRS
MGRS_BANDS = "CDEFGHJKLMNPQRSTUVWX"
MGRS_COLS = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")
MGRS_ROWS = "ABCDEFGHJKLMNPQRSTUV"


def mgrs_band(lat):
    """Буква широтного пояса (8°, X — 72…84°)."""
    i = int(math.floor((float(lat) + 80.0) / 8.0))
    return MGRS_BANDS[min(max(i, 0), 19)]


def mgrs_square(E, N, zone):
    """Буквы 100-км квадрата (столбец, строка) для UTM (E, N) зоны
    (схема AA, WGS84): столбцы A–H / J–R / S–Z по (zone−1) mod 3 от E = 100 км;
    строки A–V (без I, O) с периодом 2000 км, у чётных зон сдвиг на 5 букв."""
    col = int(math.floor(float(E) / 100000.0))
    row = int(math.floor(float(N) / 100000.0))
    letters = MGRS_COLS[(zone - 1) % 3]
    if not 1 <= col <= 8:
        raise ValueError(f"E = {E:.0f} м вне столбцов MGRS зоны {zone}")
    off = 5 if zone % 2 == 0 else 0
    return letters[col - 1], MGRS_ROWS[(row + off) % 20]


def mgrs_fwd(lat, lon, zone=None):
    """lat/lon -> (зона, пояс, 'квадрат', e, n): e, n — метры в квадрате."""
    zone = zone or _utm_zone(lon, lat)
    E, N = utm_fwd(lat, lon, zone)
    E, N = float(E), float(N)
    c, r = mgrs_square(E, N, zone)
    return zone, mgrs_band(lat), c + r, E % 100000.0, N % 100000.0


def mgrs_string(lat, lon, digits=5, zone=None):
    """Строка MGRS; координаты усекаются (не округляются), как в GeoTrans."""
    z, b, sq, e, n = mgrs_fwd(lat, lon, zone)
    q = 10 ** (5 - digits)
    return f"{z:02d}{b}{sq}{int(e // q):0{digits}d}{int(n // q):0{digits}d}"


def parse_mgrs_grid(code):
    """'37UDB' -> (37, 'U', 'D', 'B'). Пояс можно опустить: '37DB'."""
    s = str(code).strip().upper().replace(" ", "")
    i = 0
    while i < len(s) and s[i].isdigit():
        i += 1
    if i == 0 or len(s) - i not in (2, 3):
        raise ValueError(f"код 100-км квадрата MGRS {code!r}: ожидается вида 37UDB")
    zone = int(s[:i])
    band = s[i] if len(s) - i == 3 else ""
    col, row = s[-2], s[-1]
    if not 1 <= zone <= 60 or col not in MGRS_COLS[(zone - 1) % 3] or row not in MGRS_ROWS:
        raise ValueError(f"код 100-км квадрата MGRS {code!r} не существует")
    return zone, band, col, row


def mgrs_square_origin(code, N_hint):
    """Юго-западный угол квадрата (E, N) в UTM его зоны. Строки повторяются
    каждые 2000 км; берётся повтор, ближайший к N_hint (например, к точке
    выставки)."""
    zone, _, col, row = parse_mgrs_grid(code)
    E = 100000.0 * (MGRS_COLS[(zone - 1) % 3].index(col) + 1)
    off = 5 if zone % 2 == 0 else 0
    r = (MGRS_ROWS.index(row) - off) % 20
    k = round((float(N_hint) - 50000.0 - 100000.0 * r) / 2000000.0)
    return E, 100000.0 * r + 2000000.0 * k


def mgrs_inv(code, e, n, N_hint=None, north=True):
    """Квадрат + координаты в нём -> lat, lon. N_hint — примерная северная
    координата UTM (по умолчанию центр пояса из кода)."""
    zone, band, _, _ = parse_mgrs_grid(code)
    if N_hint is None:
        if not band:
            raise ValueError("без пояса нужен N_hint")
        lat_c = -80.0 + 8.0 * MGRS_BANDS.index(band) + 4.0
        north = lat_c >= 0.0
        N_hint = float(utm_fwd(lat_c, utm_cm(zone), zone)[1])
    E0, N0 = mgrs_square_origin(code, N_hint)
    lat, lon = utm_inv(E0 + np.asarray(e, float), N0 + np.asarray(n, float), zone, north)
    return lat, lon


# ------------------------------------------------------------------ система прогона
class Frame:
    """Внутренняя непрерывная система прогона и перевод в выходную.

    Внутренняя: UTM зоны `zone` со сдвигом в начало (lat0, lon0):
    x = E − E0, y = N − N0, z — абсолютная высота. Выходная — по `projection`
    (см. докстринг модуля).
    """

    def __init__(self, lat0, lon0, alt0, projection="mgrs", mgrs_grid="", utm_zone=0):
        self.lat0, self.lon0, self.alt0 = float(lat0), float(lon0), float(alt0)
        self.projection = projection_name(projection)
        self.grid = str(mgrs_grid or "").strip().upper()
        zone = int(utm_zone or 0)
        if self.projection == "mgrs" and self.grid:
            gz = parse_mgrs_grid(self.grid)[0]
            zone = gz                       # зона задана кодом квадрата
        self.zone = zone or _utm_zone(self.lon0, self.lat0)
        self.north = self.lat0 >= 0.0
        E0, N0 = utm_fwd(self.lat0, self.lon0, self.zone, self.north)
        self.E0, self.N0 = float(E0), float(N0)
        self.grid_EN = (mgrs_square_origin(self.grid, self.N0)
                        if self.projection == "mgrs" and self.grid else None)
        self._enu = Enu(self.lat0, self.lon0, self.alt0) if self.projection == "enu" else None
        self._eqr = (Equirect(self.lat0, self.lon0, self.alt0)
                     if self.projection == "equirect" else None)
        # масштаб UTM в начале: путь по карте без карты (прямая) в метрах сетки
        self.k0 = float(self.scale_heading(self.lat0, self.lon0, 0.0)[0][0])

    # --- внутренняя система
    def fwd_arr(self, lat, lon, alt):
        lat, lon, alt = np.broadcast_arrays(*(np.asarray(v, float).reshape(-1)
                                              for v in (lat, lon, alt)))
        E, N = utm_fwd(lat, lon, self.zone, self.north)
        return np.c_[E - self.E0, N - self.N0, alt]

    def fwd(self, lat, lon, alt):
        E, N = utm_fwd(float(lat), float(lon), self.zone, self.north)
        return float(E) - self.E0, float(N) - self.N0, float(alt)

    def geodetic(self, x, y):
        lat, lon = utm_inv(np.asarray(x, float) + self.E0, np.asarray(y, float) + self.N0,
                           self.zone, self.north)
        return lat, lon

    def utm(self, x, y):
        """Абсолютные E, N (непрерывные) для точки внутренней системы."""
        return x + self.E0, y + self.N0

    def scale_heading(self, lat, lon, head, d=1.0):
        """Масштаб UTM и курс сетки для направления head (азимут от
        истинного севера, рад) в точке (lat, lon). Для зоны 37 здесь масштаб
        ≈ 0,99972, сближение меридианов ≈ −1,3°. Центральная разность ±d/2."""
        lat, lon, head = np.broadcast_arrays(*(np.asarray(v, float).reshape(-1)
                                               for v in (lat, lon, head)))
        phi = np.radians(lat)
        w = np.sqrt(1.0 - E2_WGS * np.sin(phi) ** 2)
        M = A_WGS * (1.0 - E2_WGS) / w ** 3          # радиус кривизны меридиана
        Nr = A_WGS / w                                # радиус первого вертикала
        dlat = np.degrees(0.5 * d * np.cos(head) / M)
        dlon = np.degrees(0.5 * d * np.sin(head) / (Nr * np.cos(phi)))
        E1, N1 = utm_fwd(lat - dlat, lon - dlon, self.zone, self.north)
        E2, N2 = utm_fwd(lat + dlat, lon + dlon, self.zone, self.north)
        return np.hypot(E2 - E1, N2 - N1) / d, np.arctan2(E2 - E1, N2 - N1)

    # --- выходная система
    def out(self, x, y, z):
        """Точка внутренней системы -> публикуемые (x, y, z)."""
        p = self.projection
        if p == "mgrs":
            E, N = x + self.E0, y + self.N0
            if self.grid_EN is not None:
                return E - self.grid_EN[0], N - self.grid_EN[1], z
            return E % 100000.0, N % 100000.0, z
        if p == "utm":
            return x + self.E0, y + self.N0, z
        lat, lon = self.geodetic(x, y)
        q = (self._enu or self._eqr).fwd(float(lat), float(lon), z)
        return q

    def out_yaw(self, x, y, az):
        """Курс (азимут сетки UTM, рад от севера по часовой) -> угол рыскания
        REP-103 в выходной системе (от оси x против часовой)."""
        if self.projection in ("mgrs", "utm"):
            return math.pi / 2.0 - az
        p0 = self.out(x, y, 0.0)
        p1 = self.out(x + math.sin(az), y + math.cos(az), 0.0)
        return math.atan2(p1[1] - p0[1], p1[0] - p0[0])

    def square(self, x, y):
        """Код 100-км квадрата точки внутренней системы, например '37UDB'."""
        E, N = x + self.E0, y + self.N0
        lat = float(self.geodetic(x, y)[0])
        c, r = mgrs_square(E, N, self.zone)
        return f"{self.zone:02d}{mgrs_band(lat)}{c}{r}"

    def describe(self):
        if self.projection == "mgrs":
            conv = (f"непрерывно от квадрата {self.grid}" if self.grid
                    else "каждая точка в своём 100-км квадрате")
            return (f"MGRS зона {self.zone}, {conv}; начало выставки "
                    f"{self.square(0.0, 0.0)} E={self.E0:.1f} N={self.N0:.1f}")
        return f"{self.projection}, зона UTM {self.zone}, начало ({self.lat0:.7f}, {self.lon0:.7f})"
