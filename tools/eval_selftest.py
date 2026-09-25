"""Тесты инструментов оценки: геодезия, инъекции, метрики, детерминизм.

    python3 -m pytest tools/eval_selftest.py -q        (в образе vectra/tram:dev)

Эталонные значения UTM/MGRS/ENU получены pyproj 3.7.1 (PROJ) и пакетом mgrs
в одноразовом контейнере 25.09 (pip install pyproj mgrs); в тест вписаны
числа, чтобы тест не зависел от сети.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import eval_geo as G                    # noqa: E402
import eval_metrics as M                # noqa: E402
import inject                           # noqa: E402

# (lat, lon) -> (E, N) UTM 37N и MGRS 1 м по pyproj 3.7.1 / mgrs
PYPROJ_UTM37 = [
    ((55.80484, 37.42050), 400998.8106194843, 6185487.975361051, "37UDB0099885487"),
    ((55.8120, 37.3900), 399105.81387045217, 6186328.741651115, "37UCB9910586328"),
    ((55.7990, 37.4800), 404713.609125147, 6184754.656024665, "37UDB0471384754"),
    ((55.8065, 37.4120), 400470.32651829405, 6185684.879679033, "37UDB0047085684"),
    ((55.80, 37.30), 393433.5788920503, 6185128.218264384, "37UCB9343385128"),
    ((55.81, 37.60), 412260.3894632523, 6185820.189440803, "37UDB1226085820"),
    ((56.2, 38.9), 493795.46631646884, 6228343.907990631, "37VDC9379528343"),
    ((55.5, 36.1), 316833.2491472647, 6154254.757630054, "37UCB1683354254"),
    ((0.0, 39.0), 500000.0000000007, 0.0, "37NEA0000000000"),
    ((55.8, 36.0001), 311969.3435990284, 6187893.292083722, "37UCB1196987893"),
]
# строгий ENU (pyproj: cart + topocentric), начало (55.80484, 37.42050, 150)
PYPROJ_ENU = [
    ((55.8120, 37.3900, 160.0), (-1912.2546985407753, 797.6235027622538, 9.664140029624264)),
    ((55.7990, 37.4800, 140.0), (3731.695195971481, -648.6268251364489, -11.122111497431604)),
]
ENU_O = (55.80484, 37.42050, 150.0)


@pytest.mark.parametrize("ll,E,N,code", PYPROJ_UTM37)
def test_utm_matches_pyproj(ll, E, N, code):
    e, n = G.utm_fwd(ll[0], ll[1], 37)
    assert abs(float(e) - E) < 1e-3 and abs(float(n) - N) < 1e-3
    assert G.mgrs_square(*ll) == code[:5]
    # «перенос по точке» даёт те же цифры, что MGRS 1 м
    x, y = G.wrap(e, n)
    assert int(math.floor(float(x))) == int(code[5:10]) and int(math.floor(float(y))) == int(code[10:])


def test_one_geodesy_and_independent_series():
    """Интеграция: eval_geo — обёртка над геодезией пакета (одна реализация
    на ноду, оценку и экспорт). Независимая проверка — ряд Снайдера
    (analysis/georef.py, поток «положение») на линии и вокруг: < 1 см."""
    sys.path.insert(0, str(ROOT / "analysis"))
    import georef
    from tram_state_estimator import geodesy as GD
    assert G.GD is GD
    rng = np.random.default_rng(0)
    lat = rng.uniform(55.70, 55.90, 5000)
    lon = rng.uniform(37.20, 37.70, 5000)
    E, N = G.utm_fwd(lat, lon, 37)
    Es, Ns = georef.utm_snyder(lat, lon, 37)
    assert np.abs(E - Es).max() < 0.01 and np.abs(N - Ns).max() < 0.01


def test_utm_inverse_roundtrip():
    lat = np.array([p[0][0] for p in PYPROJ_UTM37])
    lon = np.array([p[0][1] for p in PYPROJ_UTM37])
    E, N = G.utm_fwd(lat, lon, 37)
    la, lo = G.utm_inv(E, N, 37)
    assert np.max(np.abs(la - lat)) < 1e-9 and np.max(np.abs(lo - lon)) < 1e-9
    la, lo = G.utm_inv(400000.0, 6185000.0, 37)      # pyproj: 37.4047507783766, 55.8002514861167
    assert abs(float(la) - 55.8002514861167) < 1e-9 and abs(float(lo) - 37.4047507783766) < 1e-9


def test_grid_origin_and_squares():
    assert G.grid_origin("37UDB") == (37, 400000.0, 6100000.0)
    assert G.grid_origin("37UCB") == (37, 300000.0, 6100000.0)
    assert G.grid_origin("37VDC") == (37, 400000.0, 6200000.0)
    assert G.utm_zone(37.42, 55.8) == 37 and G.utm_zone(5.0, 60.0) == 32


def test_unwrap_continuity():
    E = np.array([400020.0, 400005.0, 399990.0, 399970.0, 400010.0])
    x, _ = G.wrap(E, E)
    assert np.allclose(G.unwrap(x, 400050.0), E)


@pytest.mark.parametrize("llh,enu", PYPROJ_ENU)
def test_enu_matches_pyproj(llh, enu):
    got = G.enu_fwd([llh[0]], [llh[1]], [llh[2]], ENU_O)[0]
    assert np.max(np.abs(got - np.array(enu))) < 1e-6
    la, lo, al = G.enu_inv([got[0]], [got[1]], [got[2]], ENU_O)
    assert abs(la[0] - llh[0]) < 1e-10 and abs(lo[0] - llh[1]) < 1e-10 and abs(al[0] - llh[2]) < 1e-6


def test_equirect_roundtrip_matches_partner_formula():
    o = (55.80484, 37.42050, 150.0)
    lat, lon, alt = 55.8120, 37.3900, 160.0
    x, y, z = G.equirect_fwd([lat], [lon], [alt], o)[0]
    k = math.cos(math.radians(o[0]))
    assert abs(x - math.radians(lon - o[1]) * 6378137.0 * k) < 1e-9
    assert abs(y - math.radians(lat - o[0]) * 6378137.0) < 1e-9
    la, lo, al = G.equirect_inv(x, y, z, o)
    assert abs(la - lat) < 1e-12 and abs(lo - lon) < 1e-12 and abs(al - alt) < 1e-9


# ------------------------------------------------------------ инъекции

def _fake_run(T=400.0, v_kmh=30.0):
    t = np.arange(0.0, T, 0.1)
    f = np.c_[t + 0.05, t, np.full_like(t, v_kmh)]
    r = np.c_[t + 0.06, t + 0.01, np.full_like(t, v_kmh)]
    h = np.where((t % 60) < 20, 5.0, np.where((t % 60) < 40, -5.0, 0.0))
    c = np.c_[t + 0.04, t + 0.02, h]
    z = np.zeros((0, 9))
    return dict(front=f, rear=r, cmd=c, mfix=z, rfix=z, mvel=np.zeros((0, 5)), rvel=np.zeros((0, 5)))


@pytest.mark.parametrize("kind", inject.ORDER)
def test_inject_every_kind(kind):
    a = _fake_run()
    t0, dur = inject.choose_window(a, kind)
    assert t0 is not None, kind
    b, info = inject.apply(a, kind, t0, dur, seed=inject.seed_for("x", kind))
    assert info["kind"] == kind and sum(info["touched"].values()) > 0
    # оригинал не тронут, GNSS не тронут
    assert np.array_equal(a["front"], _fake_run()["front"])
    assert b["mfix"] is not a["mfix"] and np.array_equal(b["mfix"], a["mfix"])
    b2, _ = inject.apply(a, kind, t0, dur, seed=inject.seed_for("x", kind))
    for k in b:
        assert np.array_equal(b[k], b2[k], equal_nan=True)


def test_inject_values():
    a = _fake_run()
    b, _ = inject.apply(a, "both_zero", 100.0, 20.0)
    w = (b["front"][:, 1] >= 100.0) & (b["front"][:, 1] < 120.0)
    assert np.all(b["front"][w, 2] == 0) and np.all(b["front"][~w, 2] == 30.0)
    b, _ = inject.apply(a, "dropout", 100.0, 2.0)
    assert len(b["front"]) == len(a["front"]) - 20 and len(b["cmd"]) == len(a["cmd"])
    b, _ = inject.apply(a, "stamp_jump", 100.0)
    d = b["front"][:, 1] - a["front"][:, 1]
    assert np.count_nonzero(d) == 1 and d.max() == pytest.approx(30.0)
    b, _ = inject.apply(a, "nan", 100.0)
    assert np.isnan(b["front"][:, 2]).sum() == 1


# ------------------------------------------------------------ метрики

def test_nearest_pairs_tolerance():
    t_out = np.arange(0.0, 10.0, 0.05)
    t_ref = np.array([0.0, 1.02, 5.049, 20.0])
    j, ok = M.nearest(t_out, t_ref)
    assert list(ok) == [True, True, True, False]
    assert t_out[j[1]] == pytest.approx(1.0)


def test_along_cross_straight_line():
    ref = np.c_[np.linspace(0, 1000, 1001), np.zeros(1001)]
    idx = np.arange(0, 1001, 10)
    est = ref[idx] + np.array([3.0, 1.5])          # на 3 м впереди, на 1,5 м слева
    al, cr, sref, path = M.along_cross(ref, idx, est)
    ok = np.isfinite(al)
    assert ok.sum() >= 99
    assert np.allclose(al[ok], 3.0, atol=1e-6) and np.allclose(cr[ok], 1.5, atol=1e-6)
    assert path == pytest.approx(1000.0)


# ------------------------------------------------------------ связка и база

def _synthetic_run():
    """Стоянка 5 с, разгон до 8 м/с, выбег, торможение; GNSS master/rover 3 с."""
    t = np.arange(0.0, 120.0, 0.1)
    v = np.clip(np.minimum(0.8 * (t - 5.0), 8.0), 0, None)
    v = np.where(t > 80, np.clip(8.0 - 1.0 * (t - 80), 0, None), v)
    kmh = v * 3.6
    f = np.c_[t + 0.05, t, kmh]
    r = np.c_[t + 0.06, t + 0.013, kmh * 1.001]
    h = np.where((t > 5) & (t < 15), 6.0, np.where(t > 80, -6.0, 0.0))
    c = np.c_[t + 0.04, t + 0.021, h]
    tf = np.arange(0.0, 4.0, 0.1)
    lat0, lon0 = 55.80484, 37.42050
    dlat = 12.4 / 6378137.0 * 180 / math.pi
    mfix = np.c_[tf + 0.08, tf, np.full_like(tf, lat0), np.full_like(tf, lon0), np.full_like(tf, 150.0),
                 np.ones_like(tf), np.zeros((len(tf), 3))]
    rfix = mfix.copy()
    rfix[:, 2] += dlat
    rfix[:, 0] += 0.01
    vel = np.c_[t + 0.08, t, np.zeros_like(t), v, np.zeros_like(t)]
    return dict(front=f, rear=r, cmd=c, mfix=mfix, rfix=rfix, mvel=vel, rvel=vel)


def test_naive_speed_is_mean_of_fresh_bogies():
    """База для любой версии Runner: при постоянных показаниях v = среднее ×
    meas_scale / 3,6, путь — интеграл v·dt на сетке."""
    import eval_replay as R
    a = _synthetic_run()
    a["front"][:, 2] = 36.0
    a["rear"][:, 2] = 43.2
    sheet = R.resolve_sheet("json")
    p = R.make_params(sheet)
    nv = R.make_naive(p, sheet["node"], None)
    (O, crash), = R.replay(R.events(a, "3"), [nv], sheet["node"])
    assert crash is None and len(O["T"]) > 2000
    v_exp = 0.5 * (36.0 + 43.2) / 3.6 * p.meas_scale
    assert np.allclose(O["V"][5:], v_exp, atol=1e-12)
    assert float(nv.core.x[R.core.IS]) == pytest.approx(float(np.sum(O["V"])) * p.dt, rel=1e-9)


def test_naive_matches_core_metrics_baseline():
    """База на Runner (eval_replay.make_naive) = NaiveRunner аудита (core_metrics).
    Только для связки с той же сеткой, что у аудита (до WP23: узлы от первой
    метки); после WP23 узлы кратны dt — проверяет test_naive_speed_is_mean_of_fresh_bogies."""
    sys.path.insert(0, str(ROOT / "tools" / "audit"))
    import eval_replay as R
    import core_metrics as CM
    a = _synthetic_run()
    sheet = R.resolve_sheet("json")
    p = R.make_params(sheet)
    evs = R.events(a, "3")
    nv_new = R.make_naive(p, sheet["node"], None)
    nv_old = CM.NaiveRunner(p, None)
    (o_new, crash), = R.replay(evs, [nv_new])
    assert crash is None
    old = []
    for tb, kind, i, th, val in evs:
        old += (nv_old.on_wheel(i, th, val) if kind == 0 else nv_old.on_handle(th, val)
                if kind == 1 else nv_old.on_fix(th, i, *val[:3]))
    To = np.array([o["stamp"] for o in old])
    Vo = np.array([o["v"] for o in old])
    Xo = np.array([[o["x"], o["y"], o["z"]] for o in old])
    if len(To) != len(o_new["T"]) or To[0] != o_new["T"][0]:
        pytest.skip("сетка Runner отличается от аудита (WP23: узлы кратны dt)")
    assert np.array_equal(To, o_new["T"])
    assert np.array_equal(Vo, o_new["V"])
    assert np.allclose(Xo, o_new["XYZ"], atol=1e-9)


def test_runner_like_node_and_frame_roundtrip():
    """Связка строится из листа, выход equirect переводится в UTM без потерь."""
    import eval_replay as R
    a = _synthetic_run()
    sheet = R.resolve_sheet("jury")
    p = R.make_params(sheet)
    r, unused = R.make_runner(p, sheet["node"], None)
    assert r.pos.init_window == sheet["node"]["init_window_s"]
    assert r.wheel_timeout == sheet["node"]["wheel_timeout_s"]
    (O, crash), = R.replay(R.events(a, "3"), [r], sheet["node"])
    assert crash is None and len(O["T"]) > 2000
    PV = O["PV"]
    assert PV.sum() > 2000
    XYZ = O["XYZ"][PV]
    fr, grid, note = R.detect_frame(sheet["node"], "auto", XYZ)
    origin = R.runner_origin(r)
    assert origin is not None
    lat, lon, alt = R.to_geo(XYZ, fr, grid, origin, 37)
    # обратно в систему выхода той же формулой: перевод без потерь (любая версия Runner)
    if fr == "equirect":
        back = G.equirect_fwd(lat, lon, alt, origin)
    elif fr == "enu":
        back = G.enu_fwd(lat, lon, alt, origin)
    else:
        E_, N_ = G.utm_fwd(lat, lon, 37)
        if fr == "mgrs":
            x_, y_ = M._conv(E_, N_, grid)
        elif np.median(np.abs(XYZ[:, 0])) > 1e5:
            x_, y_ = E_, N_
        else:
            E0, N0 = G.utm_fwd(origin[0], origin[1], 37)
            x_, y_ = E_ - E0, N_ - N0
        back = np.c_[x_, y_, alt if fr == "mgrs" or np.median(np.abs(XYZ[:, 0])) > 1e5
                     else alt - origin[2]]
    assert np.max(np.abs(back - XYZ)) < 1e-6, fr
    # MGRS «перенос по точке» и обратно: непрерывность через границу квадрата
    E, N = G.utm_fwd(lat, lon, 37)
    x, y = G.wrap(E, N)
    la2, lo2, al2 = R.to_geo(np.c_[x, y, alt], "mgrs", "", origin, 37)
    assert np.max(np.abs(la2 - lat)) < 1e-9 and np.max(np.abs(lo2 - lon)) < 1e-9


# ------------------------------------------------------------ опубликованное положение (pos_valid)

def _line_ref(n=600, E0=399700.0, N0=6185000.0, dE=1.0, zone=37):
    """Эталон: прямая на восток через E = 400 км, фиксы 10 Гц."""
    t = np.arange(n) * 0.1
    E = E0 + dE * np.arange(n)
    N = np.full(n, N0)
    lat, lon = G.utm_inv(E, N, zone)
    alt = np.full(n, 150.0)
    mfix = np.c_[t + 0.05, t, lat, lon, alt, np.zeros(n), np.zeros((n, 3))]
    vel = np.c_[t + 0.05, t, np.full(n, 10.0), np.zeros(n), np.zeros(n)]
    a = dict(front=np.zeros((0, 3)), rear=np.zeros((0, 3)), cmd=np.zeros((0, 3)), mfix=mfix,
             rfix=np.zeros((0, 3)), mvel=vel, rvel=vel)
    return a, t, E, N, lat, lon, alt


def _out(t, lat, lon, alt, pv):
    """Выход «как у Runner»: GEO (для опубликованных), сырые XYZ — MGRS с переносом."""
    E, N = G.utm_fwd(lat, lon, 37)
    x, y = G.wrap(E, N)
    XYZ = np.c_[x, y, alt]
    k = int((~pv).sum())
    XYZ[~pv] = np.c_[np.arange(k) * 0.5, np.zeros(k), np.zeros(k)]
    geo = tuple(np.where(pv, g_, np.nan) for g_ in (lat, lon, alt))
    return dict(T=t.copy(), V=np.full(len(t), 10.0), XYZ=XYZ, SV=np.full(len(t), 0.1),
                SS=np.full(len(t), 1.0), GEO=geo, POSV=pv.copy(), PV=pv.copy())


def test_pos_valid_rows_are_not_paired():
    """Строки без опубликованного положения (pos_valid = False, «s, 0, 0») не
    входят в пары: средняя 3D та же, непарных фиксов больше."""
    a, t, E, N, lat, lon, alt = _line_ref()
    fr = M.Frame("mgrs", (lat[0], lon[0], alt[0]))
    lat_e = lat + 1e-6                                   # оценка ~0,11 м севернее
    pv_all = np.ones(len(t), bool)
    pv_late = pv_all.copy()
    pv_late[:50] = False                                 # первые 5 с — без якоря
    r_all, _ = M.score_position(_out(t, lat_e, lon, alt, pv_all), a, fr, full=False)
    r_late, _ = M.score_position(_out(t, lat_e, lon, alt, pv_late), a, fr, full=False)
    assert r_all["p_unpaired"] == 0 and r_late["p_unpaired"] == 50
    assert r_late["p_pairs"] == r_all["p_pairs"] - 50 and r_late["p_out_invalid"] == 50
    assert r_late["p3d_mean"] == pytest.approx(r_all["p3d_mean"], rel=1e-6)
    assert r_late["p3d_max"] < 1.0 and r_late["judge_raw_3d_max"] < 1.0
    # опубликованный NaN: не пара, отдельный счёт
    o = _out(t, lat_e, lon, alt, pv_all)
    o["XYZ"][100] = np.nan
    o["PV"][100] = False
    r_nan, _ = M.score_position(o, a, fr, full=False)
    assert r_nan["p_nan"] == 1 and r_nan["p_unpaired"] == 1


def test_replay_marks_pos_valid():
    """Связка-заглушка: первые шаги с pos_valid = False -> PV ложь, NaN -> PV ложь."""
    import eval_replay as R

    class Stub:
        def __init__(self):
            self.n = 0

        def _o(self, t):
            self.n += 1
            return [dict(stamp=t, v=1.0, x=float("nan") if self.n == 6 else 1.0, y=2.0, z=3.0,
                         pos_valid=self.n > 3)]

        def on_wheel(self, i, stamp, value):
            return self._o(stamp)

        def on_handle(self, stamp, pos):
            return []

        def on_fix(self, stamp, antenna, lat, lon, alt):
            return []

    evs = [(0.1 * k, 0, 0, 0.1 * k, 1.0) for k in range(10)]
    (O, crash), = R.replay(evs, [Stub()])
    assert crash is None
    assert list(O["POSV"]) == [False] * 3 + [True] * 7
    assert list(O["PV"]) == [False] * 3 + [True, True, False] + [True] * 4


def test_totals_pair_fractions_and_empty_runs():
    rows = [dict(v_pairs=90, v_ref=100, p_pairs=40, p_ref=50, v_mae=0.1, p3d_mean=2.0, p_out_invalid=3),
            dict(v_pairs=10, v_ref=100, p_pairs=10, p_ref=50, v_mae=0.3, p3d_mean=4.0, p_out_invalid=0),
            dict(v_pairs=0, v_ref=100)]
    t = M.totals(rows)
    assert t["runs"] == 2 and t["runs_empty"] == 1
    assert t["v_pair_frac"] == pytest.approx(100 / 200) and t["p_pair_frac"] == pytest.approx(50 / 100)
    assert t["v_mae"] == pytest.approx(0.12) and t["p_out_invalid"] == 3


def test_boundary_matrix():
    """Эталон через E = 400 км, оценка = эталон: совпавшие соглашения — 0 м,
    разные — 100 км у каждой пары западнее границы."""
    a, t, E, N, lat, lon, alt = _line_ref()
    fr = M.Frame("mgrs", (lat[0], lon[0], alt[0]))
    r, _ = M.score_position(_out(t, lat, lon, alt, np.ones(len(t), bool)), a, fr, full=False)
    west = int(np.sum(E < 4e5))
    assert 0 < west < len(t)
    assert r["bx_wrap_wrap_3d_mean"] < 1e-6 and r["bx_grid_grid_3d_mean"] < 1e-6
    assert r["bx_wrap_grid_km"] == west and r["bx_grid_wrap_km"] == west
    assert r["bx_wrap_grid_3d_mean"] == pytest.approx(1e5 * west / len(t), rel=1e-6)
    assert r["sq_mismatch"] == 0


def test_no_master_fix_does_not_crash():
    import eval_replay as R
    a = _fake_run()
    a["mfix"] = np.zeros((0, 3))                          # так bagio пишет пустой топик
    a["rfix"] = np.zeros((0, 3))
    assert R.first_master(a) is None
    tr, la, lo, al = M.reference_geo(a)
    assert len(tr) == 0
    assert all(e[1] != 2 for e in R.events(a, "3"))


# ------------------------------------------------------------ связка как в ноде

def test_node_declared_parses_defaults(tmp_path):
    import eval_replay as R
    src = tmp_path / "node.py"
    src.write_text("class N:\n    def __init__(self):\n        P = self.declare_parameter\n"
                   "        P('init_window_s', 3.0)\n        P('origin_lat', float('nan'))\n"
                   "        P('mgrs_grid', '')\n        P('utm_zone', 0)\n"
                   "        self.declare_parameter('pulse_horizon_s', 2.0)\n"
                   "        P('dt', 0.05)\n", encoding="utf-8")
    d = R.node_declared(src)
    assert d["init_window_s"] == 3.0 and math.isnan(d["origin_lat"]) and d["mgrs_grid"] == ""
    assert d["utm_zone"] == 0 and d["pulse_horizon_s"] == 2.0
    assert "dt" not in d                                  # поле Params — не параметр ноды


def test_make_runner_passes_position_kwargs(monkeypatch):
    """Runner(…, **position_opts): параметры ноды доходят до Position по имени."""
    import types
    import eval_replay as R

    class Position:
        def __init__(self, track_map=None, origin=None, init_window=3.0, projection="mgrs",
                     mgrs_grid="", nomap_mode="hold", scale_adapt=True):
            self.kw = dict(init_window=init_window, projection=projection, mgrs_grid=mgrs_grid,
                           nomap_mode=nomap_mode, scale_adapt=scale_adapt)

    class Runner:
        def __init__(self, params, track_map=None, origin=None, wheel_timeout=1.0,
                     handle_timeout=0.5, stop_dwell=8.0, **position_opts):
            self.wheel_timeout = wheel_timeout
            self.pos = Position(track_map, origin, **position_opts)

    monkeypatch.setattr(R, "runner_mod", types.SimpleNamespace(Runner=Runner, Position=Position))
    node = dict(wheel_timeout_s=2.0, handle_timeout_s=0.5, init_window_s=4.0, projection="utm",
                mgrs_grid="37UDB", nomap_mode="line", scale_adapt=False, map_file="",
                frame_id="map", start_sort_s=0.1, bogus=1)
    r, unused = R.make_runner(None, node, None)
    assert r.wheel_timeout == 2.0
    assert r.pos.kw == dict(init_window=4.0, projection="utm", mgrs_grid="37UDB", nomap_mode="line",
                            scale_adapt=False)
    assert unused == ["bogus"]


def test_glue_status_and_start_sorter(monkeypatch):
    """Статус NavSatFix — если on_fix его принимает; стартовый всплеск — по меткам."""
    import types
    import eval_replay as R

    class Sorter:                      # семантика StartSorter (WP24)
        def __init__(self, window):
            self.window, self.done, self._t0, self._buf = window, window <= 0, None, []

        def push(self, now, stamp, item):
            if self.done:
                return [item]
            if self._t0 is None:
                self._t0 = now
            self._buf.append((stamp, len(self._buf), item))
            return self.poll(now)

        def poll(self, now):
            if self.done or self._t0 is None or now - self._t0 < self.window:
                return []
            self.done = True
            out = [it for _, _, it in sorted(self._buf, key=lambda e: e[:2])]
            self._buf = []
            return out

    seen = []

    class Rn:
        def on_wheel(self, i, stamp, value):
            seen.append(("w", stamp))
            return []

        def on_handle(self, stamp, pos):
            seen.append(("h", stamp))
            return []

        def on_fix(self, stamp, antenna, lat, lon, alt, status=0):
            seen.append(("f", stamp, status))
            return []

    monkeypatch.setattr(R, "runner_mod", types.SimpleNamespace(StartSorter=Sorter))
    gl = R.Glue(Rn(), dict(start_sort_s=0.1))
    assert gl.fix_status and gl.sort_s == 0.1
    evs = [(10.00, 0, 0, 9.0, 1.0), (10.01, 1, 0, 7.5, 0.0),
           (10.02, 2, "master", 8.0, (55.8, 37.4, 150.0, 2)), (10.50, 0, 0, 10.4, 1.0)]
    for e in evs:
        gl.feed(*e)
    gl.flush()
    assert seen == [("h", 7.5), ("f", 8.0, 2), ("w", 9.0), ("w", 10.4)]


def _long_run(T=260.0):
    """Периодический профиль (разгон, ход, торможение, стоянка) с GNSS в начале."""
    t = np.arange(0.0, T, 0.1)
    ph = t % 60.0
    v = np.where(ph < 10, 0.8 * ph, np.where(ph < 35, 8.0, np.where(ph < 43, 8.0 - (ph - 35), 0.0)))
    kmh = v * 3.6
    f = np.c_[t + 0.05, t, kmh]
    r = np.c_[t + 0.06, t + 0.013, kmh * 1.001]
    h = np.where(ph < 10, 6.0, np.where(ph < 35, 0.0, np.where(ph < 43, -6.0, 0.0)))
    c = np.c_[t + 0.04, t + 0.021, h]
    tf = np.arange(0.0, 4.0, 0.1)
    lat0, lon0 = 55.80484, 37.42050
    dlat = 12.4 / 6378137.0 * 180 / math.pi
    mfix = np.c_[tf + 0.08, tf, np.full_like(tf, lat0), np.full_like(tf, lon0), np.full_like(tf, 150.0),
                 np.ones_like(tf), np.zeros((len(tf), 3))]
    rfix = mfix.copy()
    rfix[:, 2] += dlat
    rfix[:, 0] += 0.01
    vel = np.c_[t + 0.08, t, np.zeros_like(t), v, np.zeros_like(t)]
    return dict(front=f, rear=r, cmd=c, mfix=mfix, rfix=rfix, mvel=vel, rvel=vel)


@pytest.mark.parametrize("kind", ["both_zero", "stamp_jump", "rear_drop", "nan"])
def test_injection_from_snapshot_equals_full_replay(kind):
    """Копия чистой связки перед аномалией + поток с инъекцией = прогон с нуля."""
    import eval_replay as R
    a = _long_run()
    sheet = R.resolve_sheet("json")
    p = R.make_params(sheet)
    node = sheet["node"]
    t0, dur = inject.choose_window(a, kind)
    assert t0 is not None
    b, _ = inject.apply(a, kind, t0, dur, seed=inject.seed_for("x", kind))
    b = R.cut_stamp(b, t0 + inject.eval_window(kind) + 20.0)
    evs, clean = R.events(b, "3"), R.events(a, "3")
    t_snap = t0 - 35.0

    def runners():
        return [R.make_runner(p, node, None)[0], R.make_naive(p, node, None)]
    full = R.replay(evs, runners(), node)
    pos = int(np.searchsorted([e[0] for e in clean], t_snap))
    assert pos > 100 and all(x[:4] == y[:4] for x, y in zip(evs[:pos], clean[:pos]))
    rp = R.Replay(runners(), node).feed(clean[:pos])
    snap = rp.fork().feed(evs[pos:]).finish()
    rest = rp.feed(clean[pos:]).finish()                  # оригинал после копии не испорчен
    assert len(rest[0][0]["T"]) > len(snap[0][0]["T"])
    for (Of, cf), (Os, cs) in zip(full, snap):
        assert (cf is None) == (cs is None)
        for k in ("T", "V", "XYZ", "SV", "PV"):
            assert np.array_equal(Of[k], Os[k], equal_nan=k != "PV"), (kind, k)
