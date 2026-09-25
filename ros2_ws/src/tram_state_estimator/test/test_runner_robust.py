"""Устойчивость связки Runner к битым входам и разрывам времени (без ROS).

WP3 (проверка входов), WP4 (разрывы времени, сброс, предел шагов), WP23
(фаза сетки), WP6 (приведение показаний к шагу), WP16 (копия для пульса),
WP24 (сортировка старта). Синтетика с метками эпохи ~1,8e9 с, как в bag.

Главный приём: отброшенное сообщение не должно менять ничего, поэтому выход
сравнивается бит-в-бит с прогоном, где этого сообщения не было; после сброса
связка должна вести себя в точности как новая связка на том же потоке.
"""

import gc
import math
import os
import time
import tracemalloc
from dataclasses import fields, replace

import numpy as np
import pytest

from tram_state_estimator.estimator_core import Params
from tram_state_estimator.runner import Runner, StartSorter

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T0 = 1786387757.337           # эпоха bag; не кратна шагу сетки
LAT0, LON0 = 55.75, 37.30
KMH = 3.6


def _tram():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    names = {f.name for f in fields(Params)}
    return Params.from_dict({k: v for k, v in got.items() if k in names})


P = _tram()


def v_true(t):
    """Разгон 1 м/с² 10 с, ход 20 с, торможение 1 м/с² 10 с, стоянка."""
    if t < 5:
        return 0.0
    if t < 15:
        return t - 5.0
    if t < 35:
        return 10.0
    if t < 45:
        return 10.0 - (t - 35.0)
    return 0.0


def notch(t):
    return 8 if 5 <= t < 15 else (-8 if 35 <= t < 45 else 0)


def gnss(t_rel, lat0=LAT0, lon0=LON0, az=math.radians(60.0), dur=3.0):
    """Окно выставки: вагон стоит, rover на 12 м впереди по курсу az."""
    k = math.cos(math.radians(lat0))
    dlat = math.degrees(12.0 * math.cos(az) / 6378137.0)
    dlon = math.degrees(12.0 * math.sin(az) / (6378137.0 * k))
    ev = []
    for i in range(int(dur / 0.1)):
        t = t_rel + 0.1 * i
        ev.append((t, "on_fix", (t, "master", lat0, lon0, 150.0)))
        ev.append((t + 0.01, "on_fix", (t, "rover", lat0 + dlat, lon0 + dlon, 150.0)))
    return ev


def stream(dur=60.0, t0=T0, lag=0.0, handle=True, wheels=True, with_gnss=True,
           gnss_kw=None):
    """События (время прихода, метод, аргументы) в порядке прихода, как
    ros2 bag play. Тележки ~9,4 Гц со сдвигом; их метка на lag раньше
    прихода (в данных ~50 мс). Ручка 20 Гц, метка = приход."""
    ev = []
    if wheels:
        for k in range(int(dur * 9.4)):
            ta = k / 9.4 + 0.013
            for i, d in ((0, 0.0), (1, 0.037)):
                ts = ta + d - lag
                if ts >= 0:
                    ev.append((ta + d, "on_wheel", (i, t0 + ts, v_true(ts) * KMH)))
    if handle:
        for k in range(int(dur / 0.05)):
            t = k * 0.05 + 0.021
            ev.append((t, "on_handle", (t0 + t, notch(t))))
    if with_gnss:
        ev += [(t, m, (t0 + a[0],) + a[1:])
               for t, m, a in gnss(0.2, **(gnss_kw or {}))]
    ev.sort(key=lambda e: e[0])
    return ev


def run(r, ev):
    """Прогон; возвращает выходы и наибольшее число выходов за вызов."""
    outs, most = [], 0
    for _, m, a in ev:
        o = getattr(r, m)(*a)
        most = max(most, len(o))
        outs += o
    return outs, most


def arr(outs, k):
    return np.array([o[k] for o in outs], dtype=float)


def finite(outs):
    return all(math.isfinite(o[k]) for o in outs
               for k in ("v", "s", "x", "y", "z", "sigma_v", "sigma_s", "a"))


def same(a, b):
    """Бит-в-бит одинаковые выходы (метка, скорость, путь, положение)."""
    if len(a) != len(b):
        return False
    keys = ("stamp", "v", "s", "x", "y", "z", "sigma_v")
    return all(x[k] == y[k] for x, y in zip(a, b) for k in keys)


def verr(outs, t0=T0, t_from=2.0):
    T = arr(outs, "stamp") - t0
    V = arr(outs, "v")
    m = T > t_from
    return np.abs(V[m] - np.array([v_true(t) for t in T[m]]))


@pytest.fixture(scope="module")
def clean():
    ev = stream()
    outs, most = run(Runner(P), ev)
    return ev, outs


# ------------------------------------------------------------ WP3: значения

@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, 1e9, -1e9,
                                 "мусор", None])
def test_bad_wheel_value_is_dropped_and_changes_nothing(clean, bad):
    ev, ref = clean
    k = next(i for i, e in enumerate(ev) if e[1] == "on_wheel" and e[0] > 20)
    i, st, _ = ev[k][2]
    bad_ev = ev[:k + 1] + [(ev[k][0], "on_wheel", (i, st, bad))] + ev[k + 1:]
    r = Runner(P)
    outs, _ = run(r, bad_ev)
    assert finite(outs)
    assert r.rejected_values == 1 and r.core_resets == 0 and r.resets == 0
    assert same(outs, ref)


def test_nan_both_bogies_5s_then_recovery(clean):
    ev, ref = clean
    ev2 = [(t, m, (a[0], a[1], math.nan)) if m == "on_wheel" and 20 < t < 25
           else (t, m, a) for t, m, a in ev]
    r = Runner(P)
    outs, _ = run(r, ev2)
    assert finite(outs) and r.core_resets == 0
    assert r.rejected_values > 80
    T = arr(outs, "stamp") - T0
    e = verr(outs)
    Tm = T[T > 2.0]
    assert e[Tm > 30].max() < 0.3            # восстановилась
    assert len(outs) == len(ref)             # выход шёл и во время отказа


def test_huge_first_wheel_value_does_not_poison_start(clean):
    ev, ref = clean
    k = next(i for i, e in enumerate(ev) if e[1] == "on_wheel")
    i, st, _ = ev[k][2]
    ev2 = ev[:k] + [(ev[k][0], "on_wheel", (i, st, 1e6))] + ev[k + 1:]
    r = Runner(P)
    outs, _ = run(r, ev2)
    assert finite(outs) and r.rejected_values == 1
    assert verr(outs).max() < 0.5


@pytest.mark.parametrize("bad,expect", [(127, 15.0), (-128, -15.0),
                                        (math.nan, None), (math.inf, None)])
def test_handle_clipped_or_dropped(bad, expect):
    r = Runner(P)
    r.on_handle(T0, 3)
    r.on_handle(T0 + 0.05, bad)
    if expect is None:
        assert r.notch == 3 and r.rejected_values == 1
    else:
        assert r.notch == expect and r.rejected_values == 0


def test_limit_is_in_sensor_units():
    """Предел 1,5·v_max_line: 20 м/с -> 108 км/ч в листе km_h."""
    r = Runner(P)
    assert r._v_limit() == pytest.approx(1.5 * P.v_max_line * 3.6 / P.meas_scale)
    r.on_wheel(0, T0, 100.0)
    r.on_wheel(0, T0 + 0.1, 110.0)
    assert r.rejected_values == 1


def test_core_reset_on_nonfinite_state_keeps_path():
    ev = stream(30.0)
    r = Runner(P)
    outs, _ = run(r, [e for e in ev if e[0] < 20])
    s_before = outs[-1]["s"]
    r.core.x[1] = math.nan                   # как если бы NaN прошёл в ядро
    outs2, _ = run(r, [e for e in ev if e[0] >= 20])
    assert r.core_resets == 1 and finite(outs2)
    assert outs2[0]["s"] == pytest.approx(s_before, abs=0.6)
    assert verr(outs2, t_from=24).max() < 0.5


# ------------------------------------------------------------ WP4: время

@pytest.mark.parametrize("stamp", [0.0, -5.0, math.nan, math.inf, "x"])
def test_bad_stamp_first_message_is_dropped(clean, stamp):
    ev, ref = clean
    r = Runner(P)
    r.on_wheel(0, stamp, 10.0)
    r.on_handle(stamp, 5)
    outs, _ = run(r, ev)
    assert r.rejected_stamps == 2 and r.resets == 0
    assert same(outs, ref)


@pytest.mark.parametrize("jump", [3600.0, -3600.0, 86400.0, 1e5, -T0 + 1.0])
def test_single_stamp_outlier_is_ignored(clean, jump):
    ev, ref = clean
    k = next(i for i, e in enumerate(ev) if e[0] > 20)
    t, m, a = ev[k]
    bad = (t, "on_handle", (T0 + 20.0 + jump, 5))
    r = Runner(P)
    t_call = time.perf_counter()
    outs, most = run(r, ev[:k] + [bad] + ev[k:])
    assert time.perf_counter() - t_call < 60
    assert r.resets == 0 and r.rejected_stamps == 1
    assert r.t_notch < T0 + 100                 # выброс не записан
    assert same(outs, ref)
    assert most <= 20


def test_start_burst_going_back_2_7s_does_not_reset():
    """Хвост буфера записи: в начале bag метки идут назад на 1,1–2,7 с."""
    ev = stream(30.0)
    head = [(0.0, "on_handle", (T0 + 2.7, 0)), (0.0, "on_wheel", (0, T0 + 2.6, 0.0))]
    r = Runner(P)
    outs, _ = run(r, head + ev)
    assert r.resets == 0 and finite(outs)


def _bag2(shift, **kw):
    """Второй прогон: метки сдвинуты на shift, выставка в другом месте."""
    return stream(40.0, t0=T0 + shift,
                  gnss_kw=dict(lat0=LAT0 + 0.01, lon0=LON0 - 0.02,
                               az=math.radians(200.0)), **kw)


@pytest.mark.parametrize("shift", [-139.0, 86400.0, -16 * 86400.0, 3600.0])
def test_second_bag_resets_and_realigns_like_a_fresh_runner(clean, shift):
    ev, ref = clean
    ev2 = _bag2(shift)
    r = Runner(P)
    r.pos.init_window = 3.0
    out1, _ = run(r, ev)
    t_call = time.perf_counter()
    out2, most = run(r, ev2)
    assert time.perf_counter() - t_call < 60
    assert r.resets == 1 and "разрыв" in r.reset_reason
    assert most <= Runner.MAX_STEPS
    # первое сообщение нового прогона — кандидат разрыва, дальше как новая связка
    fresh = Runner(P)
    ref2, _ = run(fresh, ev2[1:])
    assert same(out2, ref2) and len(out2) > 700
    assert r.pos.ready and r.pos.init_window == 3.0
    T = arr(out2, "stamp")
    assert T.min() >= T0 + shift and np.all(np.diff(T) > 0)


def test_loop_replay_of_same_bag(clean):
    """ros2 bag play --loop: тот же поток снова, метки назад на весь прогон."""
    ev, ref = clean
    r = Runner(P)
    run(r, ev)
    again, _ = run(r, ev)
    ref2, _ = run(Runner(P), ev[1:])
    assert r.resets == 1 and same(again, ref2)


def test_bounded_work_per_call_and_skipped_steps():
    """Не больше MAX_STEPS шагов за вызов, даже при мелком шаге сетки."""
    p = replace(P, dt=0.01)
    r = Runner(p)
    r.on_handle(T0, 0)
    out = r.on_handle(T0 + 9.9, 0)            # 990 узлов при dt = 10 мс
    assert len(out) == Runner.MAX_STEPS and r.skipped_steps == 790
    assert r.resets == 0


def test_duplicates_and_swapped_neighbours():
    ev = stream(50.0)
    dup = [e for e in ev for _ in (0, 1)]
    sw = list(ev)
    for k in range(0, len(sw) - 1, 7):
        sw[k], sw[k + 1] = sw[k + 1], sw[k]
    for variant in (dup, sw):
        r = Runner(P)
        outs, _ = run(r, variant)
        T = arr(outs, "stamp")
        assert finite(outs) and r.resets == 0 and np.all(np.diff(T) > 0)
        assert verr(outs).mean() < 0.1


def test_wheels_only_and_handle_only():
    r = Runner(P)
    outs, _ = run(r, stream(50.0, handle=False))
    T = arr(outs, "stamp")
    assert finite(outs) and 19.5 < len(T) / (T[-1] - T[0]) < 20.5
    assert verr(outs).mean() < 0.15
    r = Runner(P)
    outs, _ = run(r, stream(50.0, wheels=False))
    V = arr(outs, "v")
    assert finite(outs) and len(outs) > 900
    assert V.min() >= 0.0 and V.max() <= P.v_max_line
    assert not any(o["valid"] for o in outs[-100:])


def test_long_run_memory_is_bounded():
    """Память связки после прогрева не растёт, внутренние списки ограничены.
    Утечка «копим выходы» дала бы ~1,6 КБ на шаг (≈3 МБ за 2 мин)."""
    def chunk(k):
        return stream(30.0, t0=T0 + 30.0 * k, with_gnss=(k == 0))
    r = Runner(P)
    for k in range(3):
        run(r, chunk(k))
    gc.collect()
    tracemalloc.start()
    run(r, chunk(3))
    gc.collect()
    m1 = tracemalloc.get_traced_memory()[0]
    for k in range(4, 7):
        run(r, chunk(k))
    gc.collect()
    grow = tracemalloc.get_traced_memory()[0] - m1
    tracemalloc.stop()
    assert r.resets == 0
    assert grow < 100_000, f"рост памяти {grow} байт за 1,5 мин"
    assert len(getattr(r.pos, "_acc", ())) < 100
    assert len(r.core.notch_hist) < 10


# ------------------------------------------------------------ WP23: сетка

def test_grid_nodes_are_multiples_of_dt_without_drift():
    r = Runner(P)
    outs = []
    for k in range(5):                        # 5 мин: прежняя сетка t += dt
                                              # ушла бы уже на ~0,3 мс
        o, _ = run(r, stream(60.0, t0=T0 + 60.0 * k, with_gnss=(k == 0)))
        outs += o
    T = arr(outs, "stamp")
    n = np.round(T / P.dt)
    assert np.abs(T - n * P.dt).max() < 1e-6
    assert np.all(np.diff(n) == 1)
    first = T0 + 0.013                        # метка первого сообщения
    assert T[0] - P.dt - 1e-6 <= first < T[0]


def test_gnss_stamps_fall_on_grid_nodes():
    r = Runner(P)
    outs, _ = run(r, stream(10.0))
    T = arr(outs, "stamp")
    g = np.array([T0 + 0.2 + 0.1 * i for i in range(30)])
    g = np.round(g * 10) / 10                  # метки GNSS кратны 0,1 с
    j = np.searchsorted(T, g)
    ok = (j > 0) & (j < len(T))
    d = np.minimum(np.abs(T[np.clip(j, 0, len(T) - 1)] - g),
                   np.abs(T[np.clip(j - 1, 0, len(T) - 1)] - g))
    assert d[ok].max() < 1e-6


# ------------------------------------------------------------ WP6: возраст

def test_age_compensation_reduces_lag_bias():
    """Метки тележек на 60 мс раньше прихода, как в bag: без приведения
    выход на торможении завышен, на разгоне занижен."""
    ev = stream(60.0, lag=0.06)
    res = {}
    for comp in (False, True):
        r = Runner(P)
        r.age_comp = comp
        outs, _ = run(r, ev)
        T = arr(outs, "stamp") - T0
        e = arr(outs, "v") - np.array([v_true(t) for t in T])
        acc, brk = (T > 7) & (T < 14), (T > 37) & (T < 44)
        res[comp] = (e[acc].mean(), e[brk].mean(), np.abs(e[T > 2]).mean())
    (a0, b0, m0), (a1, b1, m1) = res[False], res[True]
    assert abs(b1) < abs(b0) and abs(a1) < abs(a0)
    assert m1 < m0


def test_age_compensation_skips_stuck_sensor():
    """Залипшее показание (то же значение) не приводится: иначе диагностика
    ядра не увидела бы залипания."""
    r = Runner(P)
    r.on_handle(T0, 8)
    for k in range(40):
        r.on_wheel(0, T0 + 0.1 * k, 20.0 + k)
        r.on_wheel(1, T0 + 0.1 * k, 20.0)       # задняя «залипла»
        r.on_handle(T0 + 0.1 * k + 0.08, 8)
    r.last = dict(r.last, a=1.0)
    r.fresh[:] = True
    r.t_wheel[:] = r.t - 0.05
    m = r._meas_at_step()
    assert m[1] == r.meas[1] and m[0] > r.meas[0]


# ------------------------------------------------------------ WP16, WP24

def test_fork_is_independent(clean):
    ev, ref = clean
    k = len(ev) // 2
    r = Runner(P)
    run(r, ev[:k])
    f = r.fork()
    spec = f.tick(r.t + 2.0)                  # прогноз на 2 с вперёд
    assert len(spec) == 40 and finite(spec)
    assert f.pos.map is r.pos.map and f.p is r.p
    rest, _ = run(r, ev[k:])
    first, _ = run(Runner(P), ev[:k])
    assert same(first + rest, ref)            # связка не заметила прогноза


def test_start_sorter_orders_by_stamp_then_passes_through():
    s = StartSorter(0.3)
    assert s.push(0.00, 5.0, "a") == []
    assert s.push(0.10, 3.0, "b") == []
    assert s.push(0.20, math.nan, "c") == []
    assert s.push(0.25, 4.0, "d") == []
    assert s.poll(0.29) == []
    assert s.poll(0.31) == ["b", "d", "a", "c"]
    assert s.push(0.40, 1.0, "e") == ["e"]
    assert StartSorter(0.0).push(0.0, 1.0, "x") == ["x"]


def test_sorted_start_begins_grid_at_earliest_stamp():
    ev = stream(10.0)
    burst = [(0.0, "on_wheel", (0, T0 + 0.9, 0.0)), (0.0, "on_handle", (T0 + 0.5, 0))]
    s = StartSorter(0.3)
    r = Runner(P)
    outs = []
    for t, m, a in burst + ev:
        stamp = a[1] if m == "on_wheel" else a[0]
        for mm, aa in s.push(t, stamp, (m, a)):
            outs += getattr(r, mm)(*aa)
    assert outs[0]["stamp"] - T0 < 0.1       # сетка от самой ранней метки
