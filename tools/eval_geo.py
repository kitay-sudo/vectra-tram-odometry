"""Геодезия для оценки (tools/eval.py): тонкая обёртка над геодезией пакета.

В проекте одна реализация WGS84 / UTM / MGRS / ENU -
`tram_state_estimator/geodesy.py`. Её используют нода (Runner), оценка (этот
модуль) и экспорт прогонов симулятора (tools/export_replay.py). Прежняя
собственная копия оценки совпадала с ней лучше 1 мм и заменена этой
обёрткой; имена функций оставлены прежними.

Независимость эталона обеспечивают не копии кода, а проверки: геодезия пакета
сверена с PROJ (pyproj) и NGA GeoTrans (test/test_position.py), а оценка -
с независимым рядом Снайдера (analysis/georef.py) в tools/eval_selftest.py.

Система судьи (ответ организаторов 25.09): плоские координаты MGRS,
x - восток (easting), y - север (northing), z - высота NavSatFix (абсолютная),
оси по REP-103. Линия пересекает границу квадратов 100 км (37U CB | 37U DB
на E = 400 км), соглашение на границе не известно:
  * grid ""      - «перенос по точке»: координаты внутри квадрата, где лежит
                   точка (так делают Autoware gnss_poser и lanelet2 MGRSProjector);
                   на границе x скачет на 100 км;
  * grid "37UDB" - непрерывные координаты от одного квадрата (западная часть
                   получает отрицательный x).
Ошибки оценки считаются в непрерывных координатах UTM; отдельно считается,
сколько точек оценки при «переносе по точке» попали бы в другой квадрат,
чем эталон.

Обратные преобразования ENU и equirect (нужны только оценке, нода их не
делает) остаются здесь, но на константах и ECEF пакета.
"""

import math
import sys
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "tram_state_estimator"
# пакет из этого дерева - первым в пути (в образе PYTHONPATH ведёт на /ws со
# старой сборкой, и без этого молча взялась бы она)
if sys.path[:1] != [str(_PKG)]:
    if str(_PKG) in sys.path:
        sys.path.remove(str(_PKG))
    sys.path.insert(0, str(_PKG))

from tram_state_estimator import geodesy as GD      # noqa: E402

if not Path(GD.__file__).resolve().is_relative_to(_PKG):
    raise ImportError(f"геодезия взята не из этого дерева: {GD.__file__} "
                      f"(tram_state_estimator уже импортирован из другого места)")

A = GD.A_WGS
F = GD.F_WGS
E2 = GD.E2_WGS
K0 = GD.UTM_K0

BANDS = GD.MGRS_BANDS
COLS = GD.MGRS_COLS
ROWS = GD.MGRS_ROWS


def _arr(x):
    return np.asarray(x, dtype=float)


# ------------------------------------------------------------------ UTM

def utm_zone(lon, lat=0.0):
    """Номер зоны UTM (с исключениями Норвегии и Шпицбергена)."""
    return GD.utm_zone(lon, lat)


def utm_fwd(lat, lon, zone):
    """(E, N) UTM северного полушария в зоне zone, м. Векторно."""
    return GD.utm_fwd(lat, lon, zone, north=True)


def utm_inv(E, N, zone):
    """(lat, lon), градусы, из UTM северного полушария. Векторно."""
    return GD.utm_inv(E, N, zone, north=True)


# ------------------------------------------------------------------ MGRS

def band_letter(lat):
    return GD.mgrs_band(lat)


def square_letters(E, N, zone):
    """Буквы квадрата 100 км MGRS (схема AA, WGS84) для точки UTM."""
    return "".join(GD.mgrs_square(E, N, zone))


def mgrs_square(lat, lon):
    """Код квадрата 100 км, например «37UDB», для точки WGS84."""
    z, b, sq, _, _ = GD.mgrs_fwd(float(lat), float(lon))
    return f"{z}{b}{sq}"


def grid_origin(code):
    """(зона, E0, N0) - юго-западный угол квадрата MGRS «37UDB» в UTM, м.
    Повтор строки (период 2000 км) - ближайший к центру пояса из кода."""
    zone, band, _, _ = GD.parse_mgrs_grid(code)
    if not band:
        raise ValueError(f"код квадрата {code!r}: нужен пояс (например 37UDB)")
    lat_c = -80.0 + 8.0 * BANDS.index(band) + 4.0
    n_hint = float(GD.utm_fwd(lat_c, GD.utm_cm(zone), zone, north=True)[1])
    E0, N0 = GD.mgrs_square_origin(code, n_hint)
    return zone, float(E0), float(N0)


def wrap(E, N):
    """«Перенос по точке»: координаты внутри своего квадрата 100 км."""
    return np.mod(_arr(E), 1e5), np.mod(_arr(N), 1e5)


def square_index(E, N):
    """Индексы квадрата 100 км (для сравнения, в каком квадрате точка)."""
    return np.floor(_arr(E) / 1e5).astype(np.int64), np.floor(_arr(N) / 1e5).astype(np.int64)


def unwrap(x, start):
    """Непрерывная координата из координаты внутри квадрата (mod 1e5):
    каждое значение переносится на целое число по 100 км к предыдущему,
    первое - к start (непрерывная координата начала выставки)."""
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
    return GD.ecef(lat, lon, alt)


def _rot(lat0, lon0):
    return GD.Enu(lat0, lon0, 0.0)._R


def _shape3(lat, lon, alt):
    return np.broadcast(_arr(lat), _arr(lon), _arr(alt)).shape + (3,)


def enu_fwd(lat, lon, alt, o):
    """Строгий ENU WGS84 от начала o = (lat0, lon0, alt0): N×3."""
    return GD.Enu(*o).fwd_arr(lat, lon, alt).reshape(_shape3(lat, lon, alt))


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
    return ecef_to_geo(d + ecef(o[0], o[1], o[2]).reshape(3))


def equirect_fwd(lat, lon, alt, o):
    """Прежняя плоская формула (runner.Enu): сфера R = a, x = Δλ·a·cos φ0."""
    return GD.Equirect(*o).fwd_arr(lat, lon, alt).reshape(_shape3(lat, lon, alt))


def equirect_inv(x, y, z, o):
    k = math.cos(math.radians(o[0]))
    return (o[0] + np.degrees(_arr(y) / A), o[1] + np.degrees(_arr(x) / (A * k)),
            _arr(z) + o[2])
