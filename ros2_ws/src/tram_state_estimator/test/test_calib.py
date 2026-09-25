"""Калибровка по данным вагона: окно адаптации, выходная σ, залипание всех
датчиков (WP15a, WP12, WP15). ROS не требуется."""

import os
from dataclasses import fields, replace

import numpy as np
import pytest

from tram_state_estimator.estimator_core import (
    DEFAULT, DEGRADED, IKB, IKT, Estimator, Params, position_sigma)
from tram_state_estimator.plant import Plant, Track

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE = {f.name for f in fields(Params)}
DT = 0.001


def _tram():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    return Params.from_dict({k: v for k, v in got.items() if k in CORE})


def _feed(r, t_end, v_of_t, notch_of_t, wheel=None, t0=0.0):
    """Поток как в bag: тележки ~9,4 Гц со сдвигом, ручка 20 Гц.
    wheel(i, t, v_kmh) может подменить показание."""
    ev = []
    for k in range(int((t_end - t0) * 9.4)):
        t = t0 + k / 9.4
        ev += [(t, 0), (t + 0.037, 1)]
    ev += [(t0 + k * 0.05, 2) for k in range(int((t_end - t0) / 0.05))]
    outs = []
    for t, kind in sorted(ev):
        if kind == 2:
            outs += r.on_handle(t, notch_of_t(t))
        else:
            val = v_of_t(t) * 3.6
            if wheel is not None:
                val = wheel(kind, t, val)
            outs += r.on_wheel(kind, t, val)
    return outs


# ------------------------------------------------------------ WP15a

def _drive_slow_sensors(params, t_end=40.0):
    """Имитатор: показания приходят раз в два шага фильтра (как тележки
    9,4 Гц при цикле 20 Гц), тяга 20 с, затем выбег и торможение."""
    pl = Plant(Track(), dt=DT, seed=0)
    es = Estimator(params)
    sub = int(round(params.dt / DT))
    m, fresh = None, False
    for k in range(int(t_end / DT)):
        t = k * DT
        notch = 4 if t < 20 else (0 if t < 30 else -3)
        pl.step(notch)
        raw = pl.measure()          # окно энкодера ведётся на каждом шаге имитатора
        if k % (2 * sub) == 0:
            m, fresh = raw, True
        if k % sub == 0:
            es.step(notch, m, fresh=fresh)
            fresh = False
    return es


def test_adapt_window_is_real_time_with_slow_sensors():
    """Окно адаптации копит реальное прошедшее время. Прежде оно копило p.dt
    на вызов, вызов шёл раз в два шага: «3 с» длились 6 с, k_изм выходил
    около 2, и ворота отвергали почти все окна."""
    es = _drive_slow_sensors(DEFAULT)
    st = es.adapt_stats
    assert st["windows"] >= 3
    win = st["win_sum"] / st["windows"]
    assert DEFAULT.t_adapt <= win <= DEFAULT.t_adapt + 3 * DEFAULT.dt
    assert st["force_ok"] >= 2
    k_mean = st["k_sum"] / st["force_ok"]
    assert 0.6 < k_mean < 1.4, f"k_изм {k_mean:.2f}: окно не по реальному времени"
    assert st["gate_ok"] / st["force_ok"] > 0.5
    assert DEFAULT.k_min + 0.05 < es.x[IKT] < DEFAULT.k_max - 0.05


def test_adaptation_can_be_switched_off():
    es = _drive_slow_sensors(replace(DEFAULT, adapt_on=False), t_end=30.0)
    assert es.x[IKT] == pytest.approx(1.0, abs=1e-9)
    assert es.x[IKB] == pytest.approx(1.0, abs=1e-9)
    assert es.adapt_stats["windows"] == 0


def test_adapt_on_accepts_yaml_booleans():
    assert Params.from_dict({"adapt_on": False}).adapt_on is False
    with pytest.raises(ValueError):
        Params.from_dict({"adapt_on": "yes"})


# ------------------------------------------------------------ WP12

def _run_pair(p0, p1):
    from tram_state_estimator.runner import Runner
    outs = []
    for p in (p0, p1):
        r = Runner(p)
        outs.append(_feed(r, 60.0, lambda t: min(t, 20.0) * 0.5 if t < 40
                          else max(0.0, 10.0 - (t - 40) * 1.0),
                          lambda t: 6 if t < 20 else (0 if t < 40 else -8)))
    return outs


def test_output_sigma_defaults_equal_filter_sigma():
    e = Estimator()
    for _ in range(50):
        o = e.step(2.0, np.full(e.nw, 5.0 / DEFAULT.r_nom))
    assert o["sigma_v"] == pytest.approx(o["sigma_v_filt"], rel=1e-12)


def test_output_sigma_calibration_changes_sigma_not_estimate():
    """Выходная σ — только публикация: оценка скорости и пути та же."""
    p0 = _tram()
    p1 = replace(p0, sv_gain=1.5, sv_floor=0.03, sv_floor_stand=0.02,
                 sv_age=0.1, sv_rel=0.004, ss_map=2.0, ss_rel=0.003)
    a, b = _run_pair(p0, p1)
    assert [o["v"] for o in a] == [o["v"] for o in b]
    assert [o["s"] for o in a] == [o["s"] for o in b]
    for o in b[40:]:
        sf, v, acc = o["sigma_v_filt"], o["v"], o["a"]
        floor = p1.sv_floor_stand if o["mode"] == 5 else p1.sv_floor
        want = np.sqrt((1.5 * sf) ** 2 + floor ** 2 + (0.1 * acc) ** 2 + (0.004 * v) ** 2)
        if not o["ambiguous"]:
            assert o["sigma_v"] == pytest.approx(want, rel=1e-6)


def test_position_sigma_grows_with_path_after_fix():
    assert position_sigma(1.0, 0.0, DEFAULT) == pytest.approx(1.0)
    p = replace(DEFAULT, ss_map=2.0, ss_rel=0.003)
    assert position_sigma(1.0, 1000.0, p) == pytest.approx(np.sqrt(1 + 4 + 9))


def test_runner_position_sigma_resets_at_stop_anchor():
    """σ положения растёт с путём после выставки и сбрасывается привязкой к
    остановке на карте."""
    from tram_state_estimator.runner import Runner
    from tram_state_estimator.track_map import TrackMap
    lat0, lon0 = 55.8, 37.4
    k = np.cos(np.radians(lat0))
    xs = np.arange(-20.0, 900.0, 1.0)
    lat = np.full(len(xs), lat0)
    lon = lon0 + np.degrees(xs / (6378137.0 * k))
    stop_x = 375.0                  # разгон 12,5 + 350 + торможение 12,5 м
    stops = [(lat0, lon0 + np.degrees(stop_x / (6378137.0 * k)), np.pi / 2, 1.0)]
    tm = TrackMap(lat, lon, np.zeros(len(xs)), np.full(len(xs), np.pi / 2),
                  np.ones(len(xs)), stops=stops)
    p = replace(_tram(), ss_map=1.0, ss_rel=0.01)
    r = Runner(p, track_map=tm)
    for t in np.arange(0, 2.0, 0.1):
        r.on_fix(t, "master", lat0, lon0, 0.0)
        r.on_fix(t, "rover", lat0, lon0 + np.degrees(12.0 / (6378137.0 * k)), 0.0)

    def v_of_t(t):                  # разгон 1 м/с², 5 м/с, торможение, стоянка 12 с
        if t < 2.0:
            return 0.0
        if t < 77.0:
            return min(5.0, t - 2.0)
        if t < 94.0:
            return max(0.0, 5.0 - (t - 77.0))
        return min(5.0, t - 94.0)

    def notch(t):
        return 3 if (2 <= t < 7 or 94 <= t < 99) else (-3 if 77 <= t < 94 else 0)

    outs = _feed(r, 110.0, v_of_t, notch)
    before = [o for o in outs if 70 < o["stamp"] < 76]
    after = [o for o in outs if 95 < o["stamp"] < 97]
    assert r.pos.anchors >= 1
    assert before[-1]["ds_fix"] > 300 and before[-1]["sigma_s"] > 3.0
    assert after[0]["ds_fix"] < 20
    assert after[0]["sigma_s"] < 0.5 * before[-1]["sigma_s"]


# ------------------------------------------------------------ WP15

def _frozen_run(freeze, t_end=60.0, notch=lambda t: 5 if t < 30 else -6,
                v_true=lambda t: 0.6 * t if t < 30 else max(0.0, 18.0 - (t - 30))):
    from tram_state_estimator.runner import Runner
    r = Runner(_tram())
    held = {}

    def wheel(i, t, val):
        if freeze is not None and freeze[0] <= t < freeze[1]:
            return held.setdefault(i, val)
        return val
    return _feed(r, t_end, v_true, notch, wheel), v_true


def test_all_bogies_frozen_are_flagged_and_uncertain():
    """Обе тележки залипли разом на 20 с при тяге и торможении: флаг,
    valid = false, σ растёт; после оттаивания — снова valid и точная оценка."""
    outs, v_true = _frozen_run((15.0, 35.0))
    T = np.array([o["stamp"] for o in outs])
    fr = np.array([o["frozen"] for o in outs])
    first = T[fr][0]
    assert 15.0 < first < 15.0 + 3.0                # ~stuck_n показаний
    win = [o for o in outs if first + 0.1 < o["stamp"] < 35.0]
    assert all(o["frozen"] and not o["valid"] and o["mode"] == DEGRADED for o in win)
    assert win[-1]["sigma_v"] > 1.0
    late = [o for o in outs if 40.0 < o["stamp"] < 45.0]
    assert all(o["valid"] and not o["frozen"] for o in late)
    assert max(abs(o["v"] - v_true(o["stamp"])) for o in late) < 0.3


def test_normal_stream_and_standstill_are_not_frozen():
    outs, _ = _frozen_run(None)
    assert not any(o["frozen"] for o in outs)
    # стоянка под тормозом и под тягой: нули бит в бит — это не залипание
    for n in (-6, 5):
        outs, _ = _frozen_run(None, t_end=20.0, notch=lambda t: n,
                              v_true=lambda t: 0.0)
        assert not any(o["frozen"] for o in outs)
