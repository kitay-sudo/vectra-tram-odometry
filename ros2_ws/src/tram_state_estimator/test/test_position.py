"""Тесты положения: геодезия (UTM/MGRS/ENU против PROJ и GeoTrans), выходная
система MGRS (перенос по квадратам и фиксированный квадрат), GNSS только в
окне выставки (C2), запасная выставка (нет rover, старт на ходу), проверка
точек GNSS, тупики карты, онлайн-масштаб пути. ROS не требуется.

Эталонные числа получены 25.09.2026 независимыми реализациями: pyproj 3.7.1
(PROJ 9.5.1, etmerc и topocentric) и пакет `mgrs` (NGA GeoTrans), в
контейнере vectra/tram:dev с pip install pyproj mgrs.
"""

import math
import os
import sys

import numpy as np
import pytest

from tram_state_estimator import geodesy as g
from tram_state_estimator.estimator_core import Params
from tram_state_estimator.runner import Position, Runner
from tram_state_estimator.track_map import TrackMap

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PKG)))

# (широта, долгота, зона, E, N, MGRS 1 м, сближение меридианов PROJ °, масштаб)
UTM_VECTORS = [
    (55.80484, 37.4205, 37, 400998.8106, 6185487.9754, "37UDB0099885487", -1.306554, 0.99972022),
    (55.8103997, 37.4622972, 37, 403632.1125, 6186047.6983, "37UDB0363286047", -1.272058, 0.99971391),
    (55.8, 37.389, 37, 399012.0889, 6184994.8680, "37UCB9901284994", -1.332538, 0.99972509),
    (55.805, 37.402, 37, 399839.7733, 6185532.3790, "37UCB9983985532", -1.321862, 0.99972305),
    (0.0, 39.0, 37, 500000.0000, 0.0000, "37NEA0000000000", -0.000000, 0.99960000),
    (-33.8688, 151.2093, 56, 334368.6336, 6250948.3454, "56HLH3436850948", 0.998172, 0.99993820),
    (60.0, 5.0, 32, 276979.9264, 6658157.2024, "32VKM7697958157", -3.465515, 1.00020958),
    (78.2, 15.6, 33, 513696.9454, 8680760.0532, "33XWG1369680760", 0.587321, 0.99960229),
    (40.7128, -74.006, 18, 583959.3723, 4507350.9982, "18TWL8395907350", 0.648392, 0.99968676),
    (38.8895, -77.0352, 18, 323486.7372, 4306483.0483, "18SUJ2348606483", -1.278070, 0.99998367),
    (-45.0, -70.0, 19, 421184.6971, 5016563.2317, "19GDL2118416563", 0.707143, 0.99967638),
    (55.75, 37.62, 37, 413380.7203, 6179118.1468, "37UDB1338079118", -1.140764, 0.99969203),
    (83.9, 20.0, 33, 559245.7222, 9319502.2688, "33XWP5924519502", 4.971832, 0.99964288),
    (-79.9, 0.5, 31, 451070.7149, 1128524.5719, "31CDM5107028524", 2.461306, 0.99962925),
]
# PROJ topocentric от (55.8103997, 37.4622972, 168.36): (lat, lon, h, e, n, u)
ENU_ORIGIN = (55.8103997, 37.4622972, 168.36)
ENU_VECTORS = [
    (55.8, 37.389, 150.0, -4596.9110, -1155.4797, -20.1174),
    (55.805, 37.402, 190.0, -3781.1417, -599.5657, 20.4936),
    (55.8104, 37.4623, 168.36, 0.1756, 0.0334, -0.0000),
]


# ================================================================ геодезия

@pytest.mark.parametrize("lat,lon,zone,E,N,code,conv,k", UTM_VECTORS)
def test_utm_and_mgrs_match_proj_and_geotrans(lat, lon, zone, E, N, code, conv, k):
    assert g.utm_zone(lon, lat) == zone
    e, n = g.utm_fwd(lat, lon, zone)
    assert float(e) == pytest.approx(E, abs=1e-3)
    assert float(n) == pytest.approx(N, abs=1e-3)
    la, lo = g.utm_inv(E, N, zone, lat >= 0)
    # эталон E, N дан с точностью 0,1 мм: допуск обратного — 0,2 мм
    assert float(la) == pytest.approx(lat, abs=2e-4 / 111320.0)
    assert float(lo) == pytest.approx(lon, abs=2e-4 / (111320.0 * math.cos(math.radians(lat))))
    assert g.mgrs_string(lat, lon) == code
    kk, h = g.Frame(lat, lon, 0.0, "utm", utm_zone=zone).scale_heading(lat, lon, 0.0)
    # курс истинного севера в сетке = −сближение меридианов PROJ
    assert -math.degrees(float(h[0])) == pytest.approx(conv, abs=1e-5)
    assert float(kk[0]) == pytest.approx(k, abs=1e-7)


@pytest.mark.parametrize("lat,lon,h,e,n,u", ENU_VECTORS)
def test_enu_matches_proj_topocentric(lat, lon, h, e, n, u):
    p = g.Enu(*ENU_ORIGIN).fwd(lat, lon, h)
    assert p == pytest.approx((e, n, u), abs=1e-3)


def test_mgrs_squares_of_our_line():
    """Линия 30618/30639: E 398,8…403,6 км зоны 37 — квадраты 37UCB и 37UDB.
    Столбцы зоны 37 (набор 1): 300–400 км — C, 400–500 км — D; строка для
    N 6,1–6,2 Мм — B (нечётная зона, без сдвига)."""
    assert g.mgrs_square(399999.0, 6185000.0, 37) == ("C", "B")
    assert g.mgrs_square(400000.0, 6185000.0, 37) == ("D", "B")
    assert g.mgrs_band(55.8) == "U"
    assert g.mgrs_square_origin("37UDB", 6185000.0) == (400000.0, 6100000.0)
    assert g.mgrs_square_origin("37UCB", 6185000.0) == (300000.0, 6100000.0)
    assert g.mgrs_square_origin("37DB", 6185000.0) == (400000.0, 6100000.0)
    with pytest.raises(ValueError):
        g.parse_mgrs_grid("37UIB")        # I в строках нет
    with pytest.raises(ValueError):
        g.Frame(55.8, 37.4, 150.0, "mgrs", "junk")
    la, lo = g.mgrs_inv("37UCB", 99012.0889, 84994.8680)
    assert float(la) == pytest.approx(55.8, abs=1e-9)
    assert float(lo) == pytest.approx(37.389, abs=1e-9)


def test_frame_output_conventions():
    """Одна точка западнее E = 400 км во всех выходных системах."""
    o = (55.8103997, 37.4622972, 168.36)                 # 37UDB
    lat, lon, h = 55.8, 37.389, 150.0                    # 37UCB, E = 399012,09
    fr = {p: g.Frame(*o, projection=p) for p in g.PROJECTIONS}
    fixed = g.Frame(*o, projection="mgrs", mgrs_grid="37UDB")
    x, y, z = fr["mgrs"].fwd(lat, lon, h)                # внутренняя: непрерывная
    assert (x, y) == pytest.approx((399012.0889 - 403632.1125, 6184994.8680 - 6186047.6983), abs=1e-3)
    assert fr["mgrs"].out(x, y, z) == pytest.approx((99012.0889, 84994.8680, 150.0), abs=1e-3)
    assert fixed.out(x, y, z) == pytest.approx((-987.9111, 84994.8680, 150.0), abs=1e-3)
    assert fr["utm"].out(x, y, z) == pytest.approx((399012.0889, 6184994.8680, 150.0), abs=1e-3)
    assert fr["enu"].out(x, y, z) == pytest.approx(ENU_VECTORS[0][3:], abs=2e-3)
    q = g.Equirect(*o).fwd(lat, lon, h)
    assert fr["equirect"].out(x, y, z) == pytest.approx(q, abs=2e-3)
    assert fr["mgrs"].square(x, y) == "37UCB" and fr["mgrs"].square(0.0, 0.0) == "37UDB"
    # курс: в MGRS/UTM — курс сетки; в ENU повёрнут на сближение меридианов
    assert fr["utm"].out_yaw(x, y, 0.0) == pytest.approx(math.pi / 2)
    # ENU: север сетки повёрнут относительно осей ENU начала на сближение
    # меридианов в начале (PROJ, точка 2: −1,272058°), с точностью до кривизны
    conv0 = math.radians(-1.272058)
    assert fr["enu"].out_yaw(x, y, 0.0) == pytest.approx(math.pi / 2 - conv0, abs=2e-4)


def test_independent_eval_reference_agrees():
    """Эталон оценки (analysis/georef.py, ряды Снайдера) совпадает с узлом
    (ряды Крюгера) на линии до миллиметра, коды квадратов тоже."""
    sys.path.insert(0, os.path.join(ROOT, "analysis"))
    georef = pytest.importorskip("georef")
    lat = np.array([55.79892, 55.8, 55.805, 55.8107])
    lon = np.array([37.38883, 37.389, 37.402, 37.46233])
    E, N = georef.utm_snyder(lat, lon, 37)
    e, n = g.utm_fwd(lat, lon, 37)
    assert np.abs(E - e).max() < 1e-3 and np.abs(N - n).max() < 1e-3
    for a, b, la in zip(E, N, lat):
        assert georef.square_code(a, b, 37, la) == g.Frame(la, 37.4, 0, "mgrs").square(
            a - g.Frame(la, 37.4, 0, "mgrs").E0, b - g.Frame(la, 37.4, 0, "mgrs").N0)
    assert georef.grid_origin("37UDB", 6185000.0) == (400000.0, 6100000.0)


# ================================================================ синтетика

LAT_B, E_B, N_B = 55.8, 399800.0, 6185000.0       # старт западнее границы 400 км


def _tram():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    names = set(Params.__dataclass_fields__)
    return Params.from_dict({k: v for k, v in got.items() if k in names})


class Route:
    """Путь в UTM зоны 37: восток L1 м, дуга R = 50 м, север L2 м."""

    def __init__(self, L1=260.0, L2=200.0, R=50.0, E0=E_B, N0=N_B):
        pts = [(E0 + x, N0) for x in np.arange(0.0, L1, 1.0)]
        for a in np.linspace(0, np.pi / 2, 80)[1:]:
            pts.append((E0 + L1 + R * np.sin(a), N0 + R - R * np.cos(a)))
        pts += [(E0 + L1 + R, N0 + R + y) for y in np.arange(1.0, L2, 1.0)]
        self.P = np.array(pts)
        seg = np.hypot(*np.diff(self.P, axis=0).T)
        self.s = np.r_[0.0, np.cumsum(seg)]

    def en(self, s):
        """E, N точки на пути s (м по сетке)."""
        return (float(np.interp(s, self.s, self.P[:, 0])),
                float(np.interp(s, self.s, self.P[:, 1])))

    def latlon(self, s):
        e, n = self.en(s)
        la, lo = g.utm_inv(e, n, 37)
        return float(la), float(lo)

    def track_map(self, end=None, terminals=False):
        P = self.P if end is None else self.P[self.s <= end]
        return TrackMap.from_polylines([np.c_[P, np.full(len(P), 150.0)]], crs="utm",
                                       zone=37, bidirectional=False,
                                       find_terminals=terminals)


def _feed(r, t0, t_end, v_of_t, route=None, s_of_t=None, gnss_until=2.0,
          rover=True, late=None, fix_status=0, late_after=3.2):
    """Поток как в bag: тележки ~9,4 Гц, ручка 20 Гц, GNSS 10 Гц до gnss_until.
    late(t) -> (lat, lon) — «поздний» GNSS после late_after с (окно выставки
    3 с от первой точки уже закрыто), или None."""
    ev = []
    for k in range(int((t_end - t0) * 9.4)):
        t = t0 + k / 9.4
        ev += [(t, 0), (t + 0.037, 1)]
    ev += [(t0 + k * 0.05, 2) for k in range(int((t_end - t0) / 0.05))]
    ev += [(t0 + k * 0.1 + 0.013, 3) for k in range(int((t_end - t0) / 0.1))]
    outs = []
    for t, kind in sorted(ev):
        if kind == 2:
            outs += r.on_handle(t, 0)
        elif kind < 2:
            outs += r.on_wheel(kind, t, v_of_t(t) * 3.6)
        elif t - t0 <= gnss_until:
            s = s_of_t(t)
            la, lo = route.latlon(s)
            outs += r.on_fix(t, "master", la, lo, 150.0, fix_status)
            if rover:
                la, lo = route.latlon(s + 12.4)
                outs += r.on_fix(t, "rover", la, lo, 150.0, fix_status)
        elif late is not None and t - t0 > late_after:
            la, lo = late(t)
            outs += r.on_fix(t, "master", la, lo, 150.0, fix_status)
            outs += r.on_fix(t, "rover", la + 1e-4, lo, 150.0, fix_status)
    return outs


def _profile(v, t_start):
    """Скорость v после t_start (до — стоянка) и путь по ней."""
    return (lambda t: v if t > t_start else 0.0,
            lambda t: v * max(0.0, t - t_start))


def _continuous(o, fr):
    """Выход MGRS (перенос по квадратам) -> непрерывные E, N рядом с началом."""
    E = o["x"] + 1e5 * round((fr.E0 - o["x"]) / 1e5)
    N = o["y"] + 1e5 * round((fr.N0 - o["y"]) / 1e5)
    return E, N


def test_position_follows_map_in_mgrs():
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    r = Runner(_tram(), track_map=route.track_map())
    outs = _feed(r, 0.0, 70.0, v_of_t, route, s_of_t)
    fr = r.pos.frame
    assert r.pos.ready and fr.projection == "mgrs" and fr.zone == 37
    o = outs[-1]
    s_true = s_of_t(o["stamp"])
    e, n = route.en(s_true)
    E, N = _continuous(o, fr)
    assert math.hypot(E - e, N - n) < 0.02 * s_true + 3.0
    assert o["z"] == pytest.approx(150.0, abs=0.5)                   # абсолютная высота
    assert 0.0 <= o["x"] < 1e5 and 0.0 <= o["y"] < 1e5
    # положение есть всегда, кроме полосы mgrs_guard_m = 5 м у E = 400 км
    for q in outs:
        if q["stamp"] > 0.2 and not q["pos_valid"]:
            e, _ = route.en(s_of_t(q["stamp"]))
            assert abs(e - 400000.0) < 12.0          # 5 м + ошибка оценки


def test_whole_run_gnss_does_not_freeze_or_shift_outputs():
    """C2: GNSS весь прогон (здесь — заведомо ложный, в 500 м) не меняет ни
    положение, ни скорость, ни сетку шагов: выход бит-в-бит как при GNSS
    только в окне, и положение не стоит в точке выставки."""
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)

    def run(late):
        r = Runner(_tram(), track_map=route.track_map())
        return r, _feed(r, 0.0, 60.0, v_of_t, route, s_of_t, late=late)

    r1, a = run(None)
    r2, b = run(lambda t: route.latlon(s_of_t(t) + 500.0))
    assert [o["stamp"] for o in a] == [o["stamp"] for o in b]
    assert max(abs(p["v"] - q["v"]) for p, q in zip(a, b)) == 0.0
    assert max(math.hypot(p["x"] - q["x"], p["y"] - q["y"]) for p, q in zip(a, b)) == 0.0
    assert r2.pos.n_used == r1.pos.n_used
    # и не замёрзло: последнее положение ≈ истинное, далеко от выставки
    E, N = _continuous(b[-1], r2.pos.frame)
    e, n = route.en(s_of_t(b[-1]["stamp"]))
    assert math.hypot(E - e, N - n) < 10.0
    assert math.hypot(E - E_B, N - N_B) > 200.0


def test_late_fix_does_not_step_the_grid():
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    r = Runner(_tram(), track_map=route.track_map())
    _feed(r, 0.0, 20.0, v_of_t, route, s_of_t)
    t, anchor, s_ref = r.t, r.pos.xyz0, r.pos.s_ref
    la, lo = route.latlon(0.0)
    assert r.on_fix(t + 5.0, "master", la, lo, 150.0) == []
    assert r.t == t and r.pos.xyz0 == anchor and r.pos.s_ref == s_ref


def test_invalid_fixes_are_rejected():
    """NaN, (0, 0), статус «нет решения» — не выставка; высота NaN
    заменяется высотой карты."""
    route = Route()
    r = Runner(_tram(), track_map=route.track_map())
    la, lo = route.latlon(0.0)
    assert r.on_fix(0.1, "master", float("nan"), lo, 150.0) == []
    assert r.on_fix(0.2, "master", 0.0, 0.0, 0.0) == []
    assert r.on_fix(0.3, "master", la, lo, 150.0, status=-1) == []
    assert r.pos.frame is None and r.pos.n_rejected == 3
    assert r.on_fix(0.4, "master", la, lo, float("nan")) == []
    assert not r.pos.fixed                     # в очереди: ждёт шага сетки
    r.on_wheel(0, 0.0, 0.0)
    r.on_wheel(0, 0.6, 0.0)                    # сетка прошла метку 0,4
    assert r.pos.fixed and r.pos.frame.lat0 == pytest.approx(la)
    assert r.pos.xyz0[2] == pytest.approx(150.0, abs=0.5)           # высота карты


def test_gnss_never_steps_the_grid_and_speed_ignores_it():
    """Скорость не зависит от GNSS вовсе: с GNSS в окне, весь прогон и без
    GNSS выход скорости и метки шагов одинаковы бит-в-бит; сама точка GNSS
    выходов не порождает."""
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    runs = []
    for until, late in ((2.0, None), (-1.0, None), (2.0, lambda t: route.latlon(s_of_t(t)))):
        r = Runner(_tram(), track_map=route.track_map())
        runs.append(_feed(r, 0.0, 30.0, v_of_t, route, s_of_t, gnss_until=until, late=late))
    for o in runs[1:]:
        assert [q["stamp"] for q in o] == [q["stamp"] for q in runs[0]]
        assert [q["v"] for q in o] == [q["v"] for q in runs[0]]
    r = Runner(_tram())
    r.on_wheel(0, 0.0, 0.0)
    la, lo = route.latlon(0.0)
    assert r.on_fix(5.0, "master", la, lo, 150.0) == [] and r.t == 0.0


def test_no_output_position_before_first_fix():
    """До первой годной точки master положения нет: pos_valid = False
    (нода /result/position не публикует, а не выдаёт (s, 0, 0) в MGRS)."""
    route = Route()
    r = Runner(_tram(), track_map=route.track_map())
    outs = _feed(r, 0.0, 3.0, lambda t: 0.0, route, lambda t: 0.0, gnss_until=-1.0)
    assert outs and not any(o["pos_valid"] for o in outs)


def test_no_rover_heading_from_map():
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    r = Runner(_tram(), track_map=route.track_map())
    outs = _feed(r, 0.0, 70.0, v_of_t, route, s_of_t, rover=False)
    assert r.pos.ready
    E, N = _continuous(outs[-1], r.pos.frame)
    e, n = route.en(s_of_t(outs[-1]["stamp"]))
    assert math.hypot(E - e, N - n) < 10.0


def test_no_rover_no_map_holds_anchor():
    route = Route()
    r = Runner(_tram())
    outs = _feed(r, 0.0, 30.0, _profile(5.0, 3.0)[0], route, lambda t: 0.0, rover=False)
    assert r.pos.fixed and not r.pos.ready
    E, N = _continuous(outs[-1], r.pos.frame)
    assert math.hypot(E - E_B, N - N_B) < 0.5 and outs[-1]["pos_valid"]


@pytest.mark.parametrize("rover", [True, False])
def test_start_on_the_move(rover):
    """Запись начинается на ходу (8 м/с): якорь — последняя точка master окна с
    путём колёс на её метку; без rover — курс по смещению master."""
    route = Route(L1=400.0)
    v = 8.0
    r = Runner(_tram(), track_map=route.track_map())
    outs = _feed(r, 100.0, 140.0, lambda t: v, route, lambda t: 20.0 + v * (t - 100.0),
                 rover=rover)
    assert r.pos.ready
    err = []
    for o in outs[-200:]:
        E, N = _continuous(o, r.pos.frame)
        e, n = route.en(20.0 + v * (o["stamp"] - 100.0))
        err.append(math.hypot(E - e, N - n))
    assert max(err) < 5.0


def test_mgrs_wrap_vs_fixed_grid_across_boundary():
    """Путь пересекает E = 400 км (37UCB -> 37UDB). Внутри всё непрерывно;
    «перенос» даёт скачок x на 100 км на границе, «37UDB» — непрерывно (x < 0
    западнее границы). y и z одинаковы."""
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    res = {}
    for grid in ("", "37UDB"):
        r = Runner(_tram(), track_map=route.track_map(), mgrs_grid=grid, mgrs_guard_m=0.0)
        res[grid] = (r, _feed(r, 0.0, 70.0, v_of_t, route, s_of_t))
    (ra, a), (rb, b) = res[""], res["37UDB"]
    xa = np.array([o["x"] for o in a if o["pos_valid"]])
    xb = np.array([o["x"] for o in b if o["pos_valid"]])
    assert xb.min() < 0.0 < xb.max()                 # непрерывно от угла 37UDB
    assert np.abs(np.diff(xb)).max() < 1.0
    assert np.abs(np.diff(xa)).max() > 99000.0       # скачок на границе
    d = xa - xb
    assert np.all(np.isclose(d, 1e5, atol=1e-6) | np.isclose(d, 0.0, atol=1e-6))
    assert np.isclose(d, 1e5, atol=1e-6).any() and np.isclose(d, 0.0, atol=1e-6).any()
    assert [o["y"] for o in a] == pytest.approx([o["y"] for o in b], abs=1e-9)
    assert ra.pos.frame.square(0.0, 0.0) == "37UCB"


def test_terminal_hold_at_dead_end():
    """Карта кончается у известной конечной (тупик), а вагон едет ещё 60 м:
    курсор стоит в последней точке пути, а не уходит по прямой за конец. Без
    известной конечной там же (или terminal_hold off) — уходит по прямой."""
    route = Route(L1=400.0)
    end = 150.0
    v_of_t, s_of_t = _profile(5.0, 3.0)
    e_end, n_end = route.en(end)
    t_end = 3.0 + (end + 60.0) / 5.0

    def last(tm, **kw):
        r = Runner(_tram(), track_map=tm, **kw)
        o = [q for q in _feed(r, 0.0, t_end, v_of_t, route, s_of_t) if q["pos_valid"]][-1]
        E, N = _continuous(o, r.pos.frame)
        return math.hypot(E - e_end, N - n_end)

    tm = route.track_map(end=end, terminals=True)
    assert len(tm.terminals) == 2                    # оба конца ломаной
    assert Runner(_tram(), track_map=tm).pos.map.terminal_hold == "terminals"
    assert last(tm) < 3.0
    assert last(route.track_map(end=end, terminals=True), terminal_hold="off") > 40.0
    assert last(route.track_map(end=end)) > 40.0     # конечных нет — не тупик


def test_map_gap_is_not_a_dead_end():
    """Разрыв карты 25 м посреди пути — не тупик: курсор проходит дальше."""
    route = Route(L1=500.0)
    tm_full = route.track_map()
    keep = ~((route.s > 150.0) & (route.s < 175.0))
    P = route.P[keep]
    parts = [P[route.s[keep] <= 150.0], P[route.s[keep] >= 175.0]]
    tm = TrackMap.from_polylines([np.c_[q, np.full(len(q), 150.0)] for q in parts],
                                 crs="utm", zone=37, bidirectional=False)
    assert len(tm.lat) < len(tm_full.lat)
    v_of_t, s_of_t = _profile(5.0, 3.0)
    r = Runner(_tram(), track_map=tm)
    outs = _feed(r, 0.0, 3.0 + 300.0 / 5.0, v_of_t, route, s_of_t)
    E, N = _continuous(outs[-1], r.pos.frame)
    e, n = route.en(s_of_t(outs[-1]["stamp"]))
    assert math.hypot(E - e, N - n) < 6.0


def test_map_gap_curved():
    """Разрыв карты 300 м на кривой (R = 1000 м) далеко от конечных — такой
    бывает, если по участку прошёл только один обучающий прогон. Курсор не
    должен встать на краю разрыва: идёт по прямой, находит путь снова и
    дальше идёт по карте. Удержание «где угодно» (any) здесь встаёт до конца
    прогона — поэтому по умолчанию только у известных конечных."""
    Rc, L1, arc, L2 = 1000.0, 300.0, 700.0, 400.0
    pts = [(E_B + x, N_B) for x in np.arange(0.0, L1, 1.0)]
    for a in np.arange(0.0, arc, 1.0) / Rc:           # поворот налево на 40°
        pts.append((E_B + L1 + Rc * np.sin(a), N_B + Rc - Rc * np.cos(a)))
    a1 = arc / Rc
    ex, ey = pts[-1]
    pts += [(ex + d * np.cos(a1), ey + d * np.sin(a1)) for d in np.arange(1.0, L2, 1.0)]
    P = np.array(pts)
    sg = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]

    class Curved(Route):
        def __init__(self):
            self.P, self.s = P, sg

    route = Curved()
    gap = (sg > L1 + 100.0) & (sg < L1 + 400.0)       # 300 м посреди дуги
    parts = [P[(sg <= L1 + 100.0)], P[sg >= L1 + 400.0]]
    v, t0 = 10.0, 3.0
    v_of_t, s_of_t = _profile(v, t0)
    t_end = t0 + (sg[-1] - 50.0) / v
    res = {}
    for mode in ("terminals", "any"):
        tm = TrackMap.from_polylines([np.c_[q, np.full(len(q), 150.0)] for q in parts],
                                     crs="utm", zone=37, bidirectional=False)
        la, lo = zip(*(route.latlon(s) for s in (0.0, sg[-1])))
        tm.terminals = np.c_[la, lo]                  # конечные — концы линии
        assert gap.sum() > 250 and len(tm.lat) < len(P)
        r = Runner(_tram(), track_map=tm, terminal_hold=mode)
        o = _feed(r, 0.0, t_end, v_of_t, route, s_of_t)[-1]
        E, N = _continuous(o, r.pos.frame)
        e, n = route.en(s_of_t(o["stamp"]))
        res[mode] = (math.hypot(E - e, N - n), r.pos._cursor["on_map"])
    # прошёл разрыв по прямой и снова на карте (ошибка вдоль пути — от
    # точки возврата на карту: ~25 м), а не стоит у края разрыва
    assert res["terminals"][0] < 40.0 and res["terminals"][1]
    assert res["any"][0] > 200.0                      # «где угодно» — встал у разрыва


def test_online_scale_is_slow_bounded_and_self_disabling():
    tm = Route().track_map()
    tm.scale = 0.999

    def drive(p, true_mult, n, L=800.0, noise=()):
        for k in range(n):
            # курсор прошёл L·s0·mult, вагон — L·s0·true: сдвиг привязки
            d = L * tm.scale * (true_mult - p.mult) + (noise[k] if k < len(noise) else 0.0)
            p._adapt_scale(L, d)

    p = Position(tm)
    drive(p, 1.005, 3)                               # колёса занижают путь на 0,5 %
    assert 1.0 < p.mult < 1.005 and p.scale_adapt    # медленно, в сторону правды
    drive(p, 1.005, 40, noise=[1.5, -1.5] * 20)
    assert p.mult == pytest.approx(1.005, abs=5e-4) and p.scale_adapt
    q = Position(tm)
    drive(q, 1.03, 5)                                # 3 % — за гейтом: не учитывать
    assert q.mult == 1.0 and not q._segs
    w = Position(tm)
    drive(w, 1.008, 1)
    drive(w, 0.994, 1)                               # привязки противоречат
    assert not w.scale_adapt and w.mult == 1.0
    drive(w, 1.008, 3)
    assert w.mult == 1.0


def test_external_map_loaders_agree(tmp_path):
    """Карта организаторов: одна и та же ломаная в широте/долготе, UTM и MGRS
    (в квадрате 37UDB) даёт одинаковые точки и истинные курсы."""
    E = np.arange(400100.0, 400300.0, 10.0)
    N = np.full(len(E), 6185000.0)
    la, lo = g.utm_inv(E, N, 37)
    a = TrackMap.from_polylines([np.c_[la, lo]], "latlon", bidirectional=False)
    b = TrackMap.from_polylines([np.c_[E, N]], "utm", zone=37, bidirectional=False)
    c = TrackMap.from_polylines([np.c_[E - 4e5, N - 6.1e6]], "mgrs", grid="37UDB",
                                bidirectional=False)
    for m in (b, c):
        assert np.abs(m.lat - a.lat).max() < 1e-9 and np.abs(m.lon - a.lon).max() < 1e-9
    # восток по сетке = истинный курс 90° + сближение меридианов (≈ −1,3°)
    assert np.degrees(a.head[0]) == pytest.approx(90.0 - 1.30, abs=0.05)
    f = tmp_path / "m.geojson"
    f.write_text('{"type": "FeatureCollection", "features": [{"type": "Feature", '
                 '"geometry": {"type": "LineString", "coordinates": '
                 + str([[float(x), float(y)] for x, y in zip(lo, la)]) + '}}]}',
                 encoding="utf-8")
    d = TrackMap.load(f)
    assert len(d.lat) == 2 * len(a.lat)              # обе стороны по умолчанию
    d.bind(g.Frame(float(la[0]), float(lo[0]), 0.0))
    assert d.heading_at((50.0, 0.0)) is not None


@pytest.mark.parametrize("status,max_status,kept", [
    (0, 1, True),        # без RTK (30639): сдвиг сохраняется (по умолчанию)
    (2, 1, False),       # RTK/GBAS (30618): сдвиг — шум окна, не переносится
    (2, 2, True),        # keep_offset_max_status 2 — всегда
])
def test_window_offset_gnss_minus_map_kept_only_without_rtk(status, max_status, kept):
    """GNSS окна сдвинут от оси пути на 1,5 м вбок. Эталон судьи — тот же
    GNSS: у решения без RTK смещение держится весь прогон, и сдвиг сохраняется
    в выходе; у RTK (статус 2) выход — ось пути. Правило выбрано по train."""
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)

    class Shifted(Route):
        def latlon(self, s):
            e, n = self.en(s)
            la, lo = g.utm_inv(e, n + 1.5, 37)            # на север, поперёк пути
            return float(la), float(lo)

    r = Runner(_tram(), track_map=route.track_map(), keep_offset_max_status=max_status)
    o = _feed(r, 0.0, 20.0, v_of_t, Shifted(), s_of_t, fix_status=status)[-1]
    assert r.pos.window_status == status
    E, N = _continuous(o, r.pos.frame)
    e, n = route.en(s_of_t(o["stamp"]))
    assert N - n == pytest.approx(1.5 if kept else 0.0, abs=0.3)
    r = Runner(_tram(), track_map=route.track_map(), keep_offset_xy=False,
               keep_offset_z=False, keep_offset_max_status=max_status)
    o = _feed(r, 0.0, 20.0, v_of_t, Shifted(), s_of_t, fix_status=status)[-1]
    E, N = _continuous(o, r.pos.frame)
    assert N - n == pytest.approx(0.0, abs=0.3)


def test_degenerate_window_gives_finite_anchor():
    """Стоим, в окне две точки master в 50 м друг от друга (или два облака):
    медианный фильтр выбросов пуст — якорь не NaN, а середина; положение
    конечно на каждом выходе."""
    lat0, lon0 = 55.8, 37.45
    dlon = math.degrees(50.0 / (g.A_WGS * math.cos(math.radians(lat0))))
    for fixes in ([(1.0, lon0), (1.1, lon0 + dlon)],
                  [(1.0 + 0.1 * k, lon0 + (dlon if k % 2 else 0.0)) for k in range(4)]):
        r = Runner(_tram())
        outs = []
        ev = [(k / 9.4, 0) for k in range(94)] + [(k / 20 + 0.02, 1) for k in range(200)]
        ev += [(t, 2, lo) for t, lo in fixes]
        for e in sorted(ev):
            if e[1] == 0:
                outs += r.on_wheel(0, e[0], 0.0) + r.on_wheel(1, e[0] + 0.01, 0.0)
            elif e[1] == 1:
                outs += r.on_handle(e[0], 0)
            else:
                outs += r.on_fix(e[0], "master", lat0, e[2], 150.0)
        ok = [o for o in outs if o["pos_valid"]]
        assert ok and all(math.isfinite(o["x"]) and math.isfinite(o["y"]) for o in ok)
        assert all(math.isfinite(v) for v in r.pos.xyz0)


@pytest.mark.parametrize("bad", [1e10, 0.0])
def test_skewed_fix_stamp_does_not_block_alignment(bad):
    """Первая точка GNSS со сбитой меткой (0 или 1e10) приходит до тележек:
    она не должна открыть окно выставки (иначе положения нет весь прогон или
    выставка по одной точке). Та же точка при идущей сетке отбрасывается."""
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    la, lo = route.latlon(0.0)
    r = Runner(_tram(), track_map=route.track_map())
    r.on_fix(bad, "master", la, lo, 150.0)            # сетки ещё нет
    outs = _feed(r, 100.0, 130.0, lambda t: v_of_t(t - 100.0), route,
                 lambda t: s_of_t(t - 100.0), gnss_until=2.0)
    assert r.pos.fixed and r.pos.ready and r.pos.n_used > 20
    assert sum(o["pos_valid"] for o in outs) > 0.9 * len(outs)
    n = r.pos.n_rejected
    assert r.on_fix(bad, "master", la, lo, 150.0) == [] and r.pos.n_rejected == n + 1


def test_rover_only_window():
    """В окне нет master, есть rover: якорь — rover, сдвинутый назад по курсу
    на базу 12,42 м (rover впереди); курс — по смещению rover или по карте."""
    route = Route(L1=400.0)
    for v, t_start in ((0.0, 99.0), (8.0, -1.0)):     # стоим / едем с начала
        v_of_t = (lambda t, v=v: v)
        s_of_t = (lambda t, v=v: 20.0 + v * t)
        r = Runner(_tram(), track_map=route.track_map())
        ev = []
        for k in range(int(40 * 9.4)):
            t = k / 9.4
            ev += [(t, 0), (t + 0.037, 1)]
        ev += [(k * 0.05, 2) for k in range(800)]
        ev += [(k * 0.1 + 0.013, 3) for k in range(25)]
        outs = []
        for t, kind in sorted(ev):
            if kind == 2:
                outs += r.on_handle(t, 0)
            elif kind < 2:
                outs += r.on_wheel(kind, t, v_of_t(t) * 3.6)
            else:
                la, lo = route.latlon(s_of_t(t) + 12.42)
                outs += r.on_fix(t, "rover", la, lo, 150.0)
        assert r.pos.fixed and r.pos.ready and r.pos._frame_rover
        err = []
        for o in outs[-100:]:
            E, N = _continuous(o, r.pos.frame)
            e, n = route.en(s_of_t(o["stamp"]))
            err.append(math.hypot(E - e, N - n))
        assert max(err) < 4.0


def test_grid_nodes_are_multiples_of_dt():
    """WP23: узлы сетки кратны dt (совпадают с метками GNSS, кратными 0,1 с),
    какое бы сообщение ни пришло первым."""
    r = Runner(_tram())
    outs = []
    for k in range(40):
        t = 100.037 + k * 0.1063
        outs += r.on_wheel(0, t, 0.0) + r.on_handle(t + 0.011, 0)
    T = np.array([o["stamp"] for o in outs])
    assert len(T) > 50
    assert np.abs(T / 0.05 - np.round(T / 0.05)).max() < 1e-6


def test_mgrs_guard_band_suppresses_position_near_square_edge():
    """mgrs_guard_m: ближе g к краю 100-км квадрата положение не публикуется
    (pos_valid = False), чтобы выход и эталон не оказались в разных квадратах."""
    route = Route()
    v_of_t, s_of_t = _profile(5.0, 3.0)
    r = Runner(_tram(), track_map=route.track_map())       # по умолчанию 5 м
    assert r.pos.mgrs_guard_m == 5.0
    outs = _feed(r, 0.0, 70.0, v_of_t, route, s_of_t)
    bad = [o for o in outs if not o["pos_valid"] and o["stamp"] > 1.0]
    assert 10 <= len(bad) <= 60                     # ~10 м при 5 м/с — около 2 с
    for o in outs:
        if o["pos_valid"] and o["stamp"] > 1.0:
            assert 5.0 <= o["x"] <= 1e5 - 5.0
    # защита только для MGRS с переносом: от фиксированного квадрата — нет
    r = Runner(_tram(), track_map=route.track_map(), mgrs_grid="37UDB")
    outs = _feed(r, 0.0, 70.0, v_of_t, route, s_of_t)
    assert all(o["pos_valid"] for o in outs if o["stamp"] > 1.0)


@pytest.mark.parametrize("mode", ["hold", "line"])
def test_no_map_fallback_modes(mode):
    """Без карты (map_file ""): hold (по умолчанию) — выход стоит в якоре;
    line — прямая вдоль курса выставки (здесь курс на восток, путь 100 м)."""
    route = Route(L1=400.0)
    v_of_t, s_of_t = _profile(5.0, 3.0)
    r = Runner(_tram(), nomap_mode=mode)
    assert Runner(_tram()).pos.nomap_mode == "hold"
    outs = _feed(r, 0.0, 23.0, v_of_t, route, s_of_t)
    assert r.pos.ready and outs[-1]["pos_valid"]
    E, N = _continuous(outs[-1], r.pos.frame)
    moved = math.hypot(E - E_B, N - N_B)
    if mode == "hold":
        assert moved < 0.5
    else:
        assert moved == pytest.approx(s_of_t(outs[-1]["stamp"]), abs=5.0)
