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


def test_naive_matches_core_metrics_baseline():
    """База на Runner (eval_replay.make_naive) = NaiveRunner аудита (core_metrics)."""
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
                if kind == 1 else nv_old.on_fix(th, i, *val))
    To = np.array([o["stamp"] for o in old])
    Vo = np.array([o["v"] for o in old])
    Xo = np.array([[o["x"], o["y"], o["z"]] for o in old])
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
    (O, crash), = R.replay(R.events(a, "3"), [r])
    assert crash is None and len(O["T"]) > 2000
    fr, grid, note = R.detect_frame(sheet["node"], "auto", O["XYZ"])
    origin = R.runner_origin(r)
    assert origin is not None
    lat, lon, alt = R.to_geo(O["XYZ"], fr, grid, origin, 37)
    back = G.equirect_fwd(lat, lon, alt, origin)
    assert np.max(np.abs(back - O["XYZ"])) < 1e-6
    # MGRS «перенос по точке» и обратно: непрерывность через границу квадрата
    E, N = G.utm_fwd(lat, lon, 37)
    x, y = G.wrap(E, N)
    la2, lo2, al2 = R.to_geo(np.c_[x, y, alt], "mgrs", "", origin, 37)
    assert np.max(np.abs(la2 - lat)) < 1e-9 and np.max(np.abs(lo2 - lon)) < 1e-9
