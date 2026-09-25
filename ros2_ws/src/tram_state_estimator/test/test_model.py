"""Тесты модели и фильтра. ROS не требуется.

Проверяется структура модели, поведение при срывах и отказах, связь с листом
параметров. Константы — заглушки; тесты не подгоняются под конкретные числа
вагона, а проверяют физически осмысленные свойства.
"""

import os
import re
from dataclasses import fields

import numpy as np
import pytest

from tram_state_estimator import estimator_core as core
from tram_state_estimator.estimator_core import (
    Params, DEFAULT, Estimator, body_force, drive_force, creep, resistance,
    shape_tract, shape_brake, f_process, h_axles, axle_load,
    IS, IV, ID, IKT, IKB, STANDSTILL, DEGRADED)
from tram_state_estimator.plant import Plant, Track

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DT = 0.001
SUB = int(round(DEFAULT.dt / DT))


# ================================================================ модель

def test_shape_tract_is_field_weakening():
    assert shape_tract(0.5 * DEFAULT.v_base) == pytest.approx(1.0)
    assert shape_tract(2.0 * DEFAULT.v_base) == pytest.approx(0.5)


@pytest.mark.parametrize("hold", [1.0, 0.5, 0.0])
def test_shape_brake_hold_below_ed_fade(hold):
    """Ниже v_ed_fade ЭД-тормоз гаснет, механический подхватывает долю
    brake_hold_frac. Прежде модель гасила тормоз в ноль вопреки документу."""
    p = Params.from_dict({"brake_hold_frac": hold})
    assert shape_brake(0.0, p) == pytest.approx(hold)
    assert shape_brake(10.0 * p.v_ed_fade, p) == pytest.approx(1.0)
    assert drive_force(-1.0, 0.5, 1.0, 1.0, p) == pytest.approx(
        -p.F_brake * shape_brake(0.5, p))


def test_brake_hold_frac_is_validated():
    with pytest.raises(ValueError):
        Params.from_dict({"brake_hold_frac": 1.5})


def test_drive_force_signs():
    assert drive_force(1.0, 5.0, 1.0, 1.0) > 0
    assert drive_force(-1.0, 5.0, 1.0, 1.0) < 0
    assert drive_force(0.0, 5.0, 1.0, 1.0) == 0


def test_drive_force_is_linear_in_scale():
    """Аппроксиматор линеен по адаптируемым масштабам."""
    assert drive_force(1.0, 5.0, 0.8, 1.0) == pytest.approx(
        0.8 * drive_force(1.0, 5.0, 1.0, 1.0))


def test_resistance_grows_with_speed():
    assert resistance(0.0) == 0.0
    assert resistance(5.0) < resistance(10.0)


def test_body_force_is_capped_by_adhesion():
    """На плохом рельсе полная тяга не реализуется: сила ограничена μ·N."""
    p = DEFAULT
    dry = body_force(1.0, 5.0, 1.0, 1.0, 0.25, p)
    ice = body_force(1.0, 5.0, 1.0, 1.0, 0.02, p)
    cap = 0.02 * axle_load(p) * sum(p.driven)
    assert ice == pytest.approx(cap)
    assert ice < dry
    ice_b = body_force(-1.0, 5.0, 1.0, 1.0, 0.02, p)
    assert ice_b == pytest.approx(-0.02 * axle_load(p) * sum(p.braked))


def test_creep_sign_follows_force():
    """Под тягой колесо опережает корпус, под торможением отстаёт."""
    p = DEFAULT
    xi_t = creep(body_force(1.0, 5.0, 1.0, 1.0, 0.25, p), p)
    xi_b = creep(body_force(-1.0, 5.0, 1.0, 1.0, 0.25, p), p)
    driven = [i for i, d in enumerate(p.driven) if d]
    idle = [i for i, d in enumerate(p.driven) if not d]
    assert all(xi_t[i] > 0 for i in driven)
    assert all(xi_b[i] < 0 for i in driven)
    assert all(xi_t[i] < 0 for i in idle)      # холостая ось чуть отстаёт


def test_measurement_model_predicts_odometry_bias():
    x = np.array([0.0, 10.0, 0.0, 1.0, 1.0])
    z = h_axles(x, 1.0, 0.25)
    driven = [i for i, d in enumerate(DEFAULT.driven) if d]
    assert all(z[i] > x[IV] for i in driven)


def test_batch_measurement_model_matches_scalar():
    """Векторный вариант обязан давать те же числа, что и скалярный."""
    from tram_state_estimator.estimator_core import h_axles_batch
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(0, 5, 11), rng.uniform(0, 15, 11),
                           np.zeros(11), np.ones(11), np.ones(11)])
    for u in (-1.0, -0.3, 0.0, 0.4, 1.0):
        for mu in (0.02, 0.25):
            batch = h_axles_batch(pts, u, mu)
            for i, pt in enumerate(pts):
                assert batch[i] == pytest.approx(h_axles(pt, u, mu), abs=1e-9)


def test_process_integrates_speed():
    x = np.array([0.0, 5.0, 0.0, 1.0, 1.0])
    y = f_process(x, 0.0, 0.25)
    assert y[IS] == pytest.approx(x[IS] + x[IV] * DEFAULT.dt)


def test_car_does_not_roll_backwards():
    x = np.array([0.0, 0.0, 0.0, 1.0, 1.0])
    assert f_process(x, 0.0, 0.25)[IV] >= 0.0


# ============================================== лист параметров и yaml

def test_from_dict_rejects_unknown_key():
    """Опечатка в yaml не должна молча оставлять заглушку."""
    with pytest.raises(KeyError):
        Params.from_dict({"M_nomm": 1.0})


def test_hand_written_yaml_types_are_accepted():
    """При подстановке ТЗ в yaml пишут «30000» и «[1, 1, 0, 0]». Это должно
    приниматься, а не ронять ноду."""
    p = Params.from_dict({"M_nom": 30000, "driven": [1, 1, 0, 0],
                          "n_axles": 4.0})
    assert isinstance(p.M_nom, float) and p.M_nom == 30000.0
    assert p.driven == (True, True, False, False)
    assert isinstance(p.n_axles, int)


def test_inconsistent_sheet_is_rejected_with_clear_message():
    with pytest.raises(ValueError, match="driven: 4 значений при n_axles = 6"):
        Params.from_dict({"n_axles": 6})
    with pytest.raises(ValueError, match="n_axles: ожидается int"):
        Params.from_dict({"n_axles": 4.5})
    with pytest.raises(ValueError, match="M_nom должен быть > 0"):
        Params.from_dict({"M_nom": 0})
    with pytest.raises(ValueError, match="mu_min <= mu_nominal"):
        Params.from_dict({"mu_min": 0.5})


def test_six_axle_sheet_runs():
    """Другая компоновка вагона подставляется листом, без правки кода."""
    p = Params.from_dict({"n_axles": 6,
                          "driven": [1, 1, 0, 0, 1, 1],
                          "braked": [1] * 6,
                          "sensor_axles": [1] * 6})
    es = Estimator(p)
    out = None
    for _ in range(200):
        out = es.step(4.0, np.full(12, 5.0 / p.r_nom))
    assert np.isfinite(out["v"]) and len(out["healthy"]) == 12


def test_params_change_behaviour():
    """Изменение параметра меняет поведение модели — константы не зашиты."""
    heavy = Params.from_dict({"M_nom": 2 * DEFAULT.M_nom})
    x = np.array([0.0, 5.0, 0.0, 1.0, 1.0])
    a0 = f_process(x, 1.0, 0.25, DEFAULT)[IV] - x[IV]
    a1 = f_process(x, 1.0, 0.25, heavy)[IV] - x[IV]
    assert a1 < a0
    stronger = Params.from_dict({"F_notch": 2 * DEFAULT.F_notch})
    assert drive_force(1.0, 1.0, 1.0, 1.0, stronger) == pytest.approx(
        2 * drive_force(1.0, 1.0, 1.0, 1.0, DEFAULT))


def test_every_parameter_is_used_by_the_core():
    """Параметр, который ядро не читает, — обман: он есть в листе, но ничего
    не меняет. Такой параметр надо удалить или подключить."""
    src = open(os.path.join(PKG, "tram_state_estimator",
                            "estimator_core.py"), encoding="utf-8").read()
    tail = src.split("DEFAULT = Params()", 1)[1]
    unused = [f.name for f in fields(Params)
              if not re.search(rf"\b\w+\.{f.name}\b", tail)]
    assert not unused, f"параметры не используются ядром: {unused}"


def test_params_mirror_yaml():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "params.yaml"), encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    got = doc["/tram_state_estimator"]["ros__parameters"]
    node_only = {"rate_hz", "input_timeout_s", "frame_id"}
    # пустые списки в yaml не пишутся (парсер ROS 2 их не принимает)
    want = {f.name for f in fields(Params)
            if getattr(DEFAULT, f.name) != ()} - {"dt"}
    assert set(got) - node_only == want, "yaml и Params разошлись; " \
        "запустите tools/gen_params.py"
    for name in want:
        d = getattr(DEFAULT, name)
        v = tuple(got[name]) if isinstance(d, tuple) else got[name]
        assert v == pytest.approx(d) if isinstance(d, float) else v == d, name


def test_yaml_values_build_valid_params():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "params.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    vals = {k: v for k, v in got.items()
            if k not in ("rate_hz", "input_timeout_s", "frame_id")}
    assert Params.from_dict(vals) == DEFAULT


# ================================================================ фильтр

def _drive(track, driver, t_end, faults=None, seed=0, params=None,
           meas_map=None, fault_fn=None, hold_every=1, t_start=0.0):
    """meas_map переводит показания имитатора (8 колёс, рад/с на колесе) в
    то, что выдал бы датчик по листу params. fault_fn(m, t) искажает
    показания во времени. hold_every — новые показания раз в столько шагов
    имитатора (датчики медленнее цикла). t_start — оценщик включается на ходу."""
    pl = Plant(track, dt=DT, seed=seed)
    es = None
    if faults:
        pl.faults = faults
    log = dict(t=[], v=[], vh=[], mode=[], valid=[], s=[], sh=[], slip=[],
               sig=[], amb=[])
    m, fresh = None, False
    for k in range(int(t_end / DT)):
        t = k * DT
        notch = driver(t)
        pl.step(notch)
        raw = pl.measure()
        if m is None or k % hold_every == 0:
            m = raw if meas_map is None else meas_map(raw)
            if fault_fn is not None:
                m = fault_fn(m, t)
            fresh = True
        if es is None and t >= t_start:
            es = Estimator(params)
        if es is not None and k % SUB == 0:
            o = es.step(notch, m, fresh=fresh)
            fresh = False
            log["t"].append(t)
            log["v"].append(pl.v)
            log["vh"].append(o["v"])
            log["mode"].append(o["mode"])
            log["valid"].append(o["valid"])
            log["s"].append(pl.s)
            log["sh"].append(o["s"])
            log["slip"].append(o["slip"])
            log["sig"].append(o["sigma_v"])
            log["amb"].append(o["ambiguous"])
    return {k: np.array(v) for k, v in log.items()}, es


def accel_brake(t):
    return 4 if t < 20 else (0 if t < 35 else -3)


def stop_and_go(t):
    c = t % 45
    return 4 if c < 18 else (0 if c < 28 else -3)


def ice_brake(t):
    return 4 if t < 18 else -4


def test_filter_finite_and_covariance_positive_definite():
    lg, es = _drive(Track(), accel_brake, 55.0)
    assert np.all(np.isfinite(lg["vh"]))
    w = np.linalg.eigvalsh(0.5 * (es.P + es.P.T))
    assert w.min() > -1e-9, "ковариация потеряла положительную определённость"


def test_speed_accuracy_on_dry_track():
    lg, _ = _drive(Track(), accel_brake, 55.0)
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.3


def test_position_over_route():
    """Остановки сбрасывают ошибку скорости: дрейф положения мал."""
    lg, _ = _drive(Track(), stop_and_go, 135.0)
    assert abs(lg["sh"][-1] - lg["s"][-1]) < 0.05 * lg["s"][-1]


def test_single_dead_sensor_is_isolated():
    lg, es = _drive(Track(), accel_brake, 55.0, faults={3: (10.0, "zero")})
    assert not es.healthy[3]
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.3


def test_stuck_sensor_is_isolated():
    _, es = _drive(Track(), accel_brake, 55.0, faults={5: (12.0, "stuck")})
    assert not es.healthy[5]


def test_equal_readings_at_constant_speed_are_not_stuck():
    """Реальный энкодер на постоянной скорости выдаёт одинаковые отсчёты.
    Одинаковость сама по себе не залипание, если не меняются и остальные."""
    es = Estimator()
    for _ in range(500):
        es.step(0.0, np.full(es.nw, 10.0 / DEFAULT.r_nom))
    assert es.healthy.all()


def test_stuck_detected_when_others_change():
    es = Estimator()
    for k in range(400):
        meas = np.full(es.nw, (5.0 + 0.01 * k) / DEFAULT.r_nom)
        meas[2] = 5.0 / DEFAULT.r_nom
        es.step(2.0, meas)
    assert not es.healthy[2]
    assert es.healthy[[0, 1, 3, 4, 5, 6, 7]].all()


def test_idle_sensors_kept_during_wheelspin_at_standstill():
    """Буксование на месте: моторные колёса крутятся, вагон стоит, холостые
    показывают ровно ноль. Прежде медиану давали буксующие, и исправные
    датчики холостых осей исключались как оборванные."""
    es = Estimator()
    for _ in range(300):
        meas = np.zeros(es.nw)
        meas[:4] = 50.0 / DEFAULT.r_nom
        es.step(4.0, meas)
    assert es.healthy[4:].all()
    assert abs(es.x[IV]) < 0.5


def test_sensor_returns_after_short_lock():
    """Пятно льда: ось заблокирована 0,5 с при торможении. Прежде её датчики
    исключались навсегда; теперь возвращаются после t_recover согласия."""
    def lock(m, t):
        m = m.copy()
        if 40.0 <= t < 40.5:
            m[0:2] = 0.0
        return m
    _, es = _drive(Track(), accel_brake, 55.0, fault_fn=lock)
    assert es.healthy.all()


def test_histories_are_bounded():
    """Прежде история ручки росла на элемент за шаг: 8,6 млн в сутки."""
    es = Estimator()
    pl = Plant(Track(), dt=DT)
    for _ in range(3000):
        es.step(1.0, np.full(es.nw, 10.0))
        pl.step(1.0)
        pl.measure()
    assert len(es.notch_hist) <= int(round(DEFAULT.delay_drive / DEFAULT.dt)) + 1
    assert max(len(pl.notch_hist), len(pl.angle_hist), len(pl.meas_hist)) < 1000


def test_slow_sensors_give_no_false_slip():
    """Датчики 20 Гц при цикле 100 Гц. Прежде одно показание учитывалось
    многократно, производная колеса скакала: 37 ложных срывов на прогоне."""
    lg, _ = _drive(Track(), accel_brake, 55.0, hold_every=50)
    assert not lg["slip"].any()
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.15
    assert abs(lg["sh"][-1] - lg["s"][-1]) < 3.0


def test_restart_on_the_move():
    """Оценщик включён на 12 м/с. Прежде до выхода на точность уходило ~7 с."""
    lg, _ = _drive(Track(), accel_brake, 20.0, t_start=15.0)
    first = lg["t"] < 15.5
    assert np.all(np.abs(lg["vh"][first] - lg["v"][first]) < 0.5)
    assert lg["valid"][first][-1]


# ------------------------------------------------------- срывы и отказы

def test_traction_slip_is_survived_by_idle_axles():
    """Буксование на мокром рельсе: моторные оси срываются, холостые держат
    скорость. Раньше отбрасывались все оси сразу, и ошибка пути доходила до
    километров."""
    lg, es = _drive(Track(mu=lambda s, t: 0.12), accel_brake, 55.0)
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.5
    assert abs(lg["sh"][-1] - lg["s"][-1]) < 20.0


def test_adhesion_estimate_drops_on_lock_and_stays_bounded():
    lg, es = _drive(Track(mu=lambda s, t: 0.25 if t < 18 else 0.03),
                    ice_brake, 30.0)
    p = DEFAULT
    assert p.mu_min - 1e-9 <= es.mu <= p.mu_nominal + 1e-9
    assert es.mu < 0.5 * p.mu_nominal, "сцепление не снизилось при блокировке"


def test_lock_on_ice_is_not_declared_standstill_early():
    """Нули заблокированных колёс не должны сразу приниматься за остановку."""
    lg, _ = _drive(Track(mu=lambda s, t: 0.25 if t < 18 else 0.03),
                   ice_brake, 40.0)
    window = (lg["t"] > 20.0) & (lg["t"] < 33.0) & (lg["v"] > 5.0)
    assert not np.any(lg["mode"][window] == STANDSTILL)


def ice_lock_then_go(t):
    return 4 if t < 18 else (-4 if t < 26 else 4)


def test_ice_lock_is_flagged_ambiguous():
    """Все колёса заблокированы на льду: по колёсам скольжение не отличить от
    остановки. Прежде через ~25 с нули принимались: v = 0, σ почти ноль,
    valid = true, пока вагон ещё скользил."""
    lg, _ = _drive(Track(mu=lambda s, t: 0.02 if t > 18 else 0.25),
                   ice_brake, 60.0)
    moving = lg["v"] > 0.5
    err = np.abs(lg["vh"] - lg["v"])
    assert lg["amb"].any()
    assert not np.any(lg["mode"][moving] == STANDSTILL)
    assert not np.any(lg["valid"][moving] & (err[moving] > 1.0))
    big = moving & (err > 1.0)
    assert np.all(err[big] <= 2.0 * lg["sig"][big]), "σ не покрывает ошибку"


def test_ambiguity_clears_when_wheels_roll():
    lg, _ = _drive(Track(mu=lambda s, t: 0.02 if 18 < t < 26 else 0.25),
                   ice_lock_then_go, 45.0)
    assert lg["amb"].any()
    assert not lg["amb"][-1]
    assert lg["valid"][-1]
    assert abs(lg["vh"][-1] - lg["v"][-1]) < 0.3


def test_stale_handle_freezes_parameters():
    """Ручка устарела: скорость корректируется по колёсам, а параметры модели
    по неверной команде не адаптируются."""
    lg, es = _drive(Track(), accel_brake, 30.0)
    x0 = es.x.copy()
    w = lg["v"][-1] / DEFAULT.r_nom
    for _ in range(300):
        es.step(4.0, np.full(es.nw, w), handle_ok=False)
    # равенство с точностью до округления усреднения сигма-точек
    assert es.x[ID] == pytest.approx(x0[ID], abs=1e-9)
    assert es.x[IKT] == pytest.approx(x0[IKT], abs=1e-9)
    assert es.x[IKB] == pytest.approx(x0[IKB], abs=1e-9)


def test_all_sensors_zero_is_not_taken_for_standstill():
    """Общий отказ: показания разом ушли в ноль при 13 м/с. Раньше оценщик
    через 3 с объявлял остановку."""
    faults = {i: (25.0, "zero") for i in range(8)}
    lg, _ = _drive(Track(), accel_brake, 55.0, faults=faults)
    win = (lg["t"] > 26.0) & (lg["t"] < 33.0)
    assert float(np.max(np.abs(lg["vh"][win] - lg["v"][win]))) < 2.0
    assert not np.any(lg["mode"][win] == STANDSTILL)
    assert not np.any(lg["valid"][win]), "оценка должна быть помечена недостоверной"


def test_open_loop_step_keeps_running_and_grows_uncertainty():
    es = Estimator()
    for _ in range(300):
        es.step(4.0, np.full(8, 4.0 / DEFAULT.r_nom))
    s0 = es.step(0.0, np.full(8, 4.0 / DEFAULT.r_nom))["sigma_v"]
    for _ in range(300):
        out = es.step_open_loop(0.0)
    assert np.isfinite(out["v"])
    assert not out["valid"]
    assert out["sigma_v"] > s0


# ------------------------------------------------------------ адаптация

def test_scale_adapts_and_does_not_rail():
    """Масштаб тяги уточняется (уходит от заглушки, неопределённость падает) и
    не упирается в границу."""
    lg, es = _drive(Track(), accel_brake, 40.0)
    p = DEFAULT
    assert p.k_min + 0.05 < es.x[IKT] < p.k_max - 0.05
    assert abs(es.x[IKT] - 1.0) > 0.02, "адаптации не было"
    assert np.sqrt(es.P[IKT, IKT]) < p.p0_k, "неопределённость масштаба не упала"


def test_adaptation_is_frozen_during_slip():
    """При буксовании масштаб тяги не должен уходить: срыв — не изменение
    силы привода."""
    _, es = _drive(Track(mu=lambda s, t: 0.12), accel_brake, 30.0)
    assert abs(es.x[IKT] - 1.0) < 0.15


# ------------------------------------------- ручка и датчики по листу ТЗ

def test_notch_tables_asymmetric_and_nonlinear():
    from tram_state_estimator.estimator_core import notch_to_u
    p = Params.from_dict({"notch_tract": [0.1, 0.3, 1.0],
                          "notch_brake": [0.5, 1.0]})
    assert [notch_to_u(n, p) for n in (1, 2, 3)] == pytest.approx([0.1, 0.3, 1.0])
    assert [notch_to_u(n, p) for n in (-1, -2)] == pytest.approx([-0.5, -1.0])
    assert notch_to_u(7, p) == pytest.approx(1.0)          # за таблицей — полный
    assert notch_to_u(1.5, p) == pytest.approx(0.2)        # непрерывная ручка
    assert notch_to_u(0, p) == 0.0


def test_default_notch_table_matches_old_linear_mapping():
    from tram_state_estimator.estimator_core import notch_to_u
    for n in range(-4, 5):
        assert notch_to_u(n) == pytest.approx(n / 4.0)


@pytest.mark.parametrize("unit,factor", [
    ("rad_s", 1.0), ("rpm", 60.0 / (2 * np.pi)), ("hz", 1.0 / (2 * np.pi)),
    ("pulses_s", DEFAULT.ppr / (2 * np.pi))])
def test_angular_units_give_same_estimate(unit, factor):
    """Та же поездка в других единицах датчика даёт ту же оценку."""
    p = Params.from_dict({"meas_units": unit})
    lg, _ = _drive(Track(), accel_brake, 30.0, params=p,
                   meas_map=lambda m: m * factor)
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.3


def test_linear_units_give_same_estimate():
    p = Params.from_dict({"meas_units": "km_h"})
    lg, _ = _drive(Track(), accel_brake, 30.0, params=p,
                   meas_map=lambda m: m * DEFAULT.r_nom * 3.6)
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.3


def test_single_sensor_per_axle():
    """Датчик на одном борту: 4 показания вместо 8."""
    p = Params.from_dict({"sensors_per_axle": 1})
    lg, es = _drive(Track(), accel_brake, 40.0, params=p,
                    meas_map=lambda m: m[0::2])
    assert es.nw == 4
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.3


def test_sensors_on_motor_shaft():
    """Датчики на валах двигателей: только моторные оси, передаточное число 6."""
    p = Params.from_dict({"sensor_axles": [1, 1, 0, 0], "sensor_ratio": 6.0})
    lg, es = _drive(Track(), accel_brake, 40.0, params=p,
                    meas_map=lambda m: m[:4] * 6.0)
    assert es.nw == 4
    assert float(np.mean(np.abs(lg["vh"] - lg["v"]))) < 0.3


def test_wrong_number_of_readings_is_an_error():
    es = Estimator()
    with pytest.raises(ValueError, match="ожидается 8 показаний"):
        es.step(1.0, np.zeros(6))


def test_new_sheet_fields_are_validated():
    with pytest.raises(ValueError, match="meas_units"):
        Params.from_dict({"meas_units": "mph"})
    with pytest.raises(ValueError, match="notch_tract"):
        Params.from_dict({"notch_tract": [0.5, 0.3]})
    with pytest.raises(ValueError, match="notch_brake"):
        Params.from_dict({"notch_brake": [1.5]})
    with pytest.raises(ValueError, match="sensors_per_axle"):
        Params.from_dict({"sensors_per_axle": 3})
    with pytest.raises(ValueError, match="sensor_axles: нет ни одной"):
        Params.from_dict({"sensor_axles": [0, 0, 0, 0]})
    with pytest.raises(ValueError, match="sensor_ratio должен быть > 0"):
        Params.from_dict({"sensor_ratio": 0})


# ------------------------------------------------------------ опознавание

def test_identification_is_accepted_and_recovers_resistance():
    """Режим опознавания на специальном профиле принимается и находит
    сопротивление объекта, а не заглушку листа. Раньше он отказывал — из-за
    дефекта имитатора (привод набирал момент ~20 с)."""
    from tram_state_estimator.identification import (
        DriveIdentifier, identification_profile)
    from tram_state_estimator import plant as pm
    pl, es = Plant(Track(), dt=DT, seed=0), Estimator()
    ident = DriveIdentifier(mass=DEFAULT.M_nom, params=DEFAULT)
    for k in range(int(220.0 / DT)):
        n = identification_profile(k * DT)
        pl.step(n)
        m = pl.measure()
        if k % SUB == 0:
            o = es.step(n, m)
            good = (not o["slip"]) and o["n_accepted"] >= 2 and o["valid"]
            ident.add(es.u_filt, float(np.mean(m)) * DEFAULT.r_nom,
                      adhesion_ok=good)
    w_t, w_b, rep = ident.solve()               # не бросает: принято
    truth = pm.P.res_B + 2 * pm.P.n_axles * pm.P.B_wheel / pm.P.r ** 2
    res_b = rep["параметры"]["res_B"][0]
    assert abs(res_b - truth) < 0.35 * truth, (res_b, truth)
    assert w_t > 0 and w_b > 0


# ------------------------------------------------------------ быстродействие

def test_step_time_within_budget():
    import time
    es = Estimator()
    meas = np.linspace(5.0, 6.0, 8)
    for _ in range(300):
        es.step(4.0, meas)
    t = []
    for _ in range(3000):
        t0 = time.perf_counter_ns()
        es.step(4.0, meas)
        t.append(time.perf_counter_ns() - t0)
    p999 = np.percentile(t, 99.9) / 1000.0
    assert p999 < DEFAULT.dt * 1e6 / 2, \
        f"p99.9 = {p999:.0f} мкс при бюджете {DEFAULT.dt * 1e6:.0f} мкс"


# ============================================================ лист вагона и связка

TRAM_NODE_ONLY = {"wheel_timeout_s", "handle_timeout_s", "init_window_s",
                  "map_file", "origin_lat", "origin_lon", "origin_alt",
                  "frame_id", "child_frame_id"}


def _tram():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    return Params.from_dict({k: v for k, v in got.items()
                             if k not in TRAM_NODE_ONLY})


def test_tram_sheet_is_valid_and_tabular():
    p = _tram()
    assert p.n_axles == 2 and p.meas_units == "km_h"
    assert len(p.acc_table) == len(p.acc_u) * len(p.acc_v)
    assert core._table(p) is not None


def test_table_drive_force_and_resistance():
    p = _tram()
    U, V, T = core._table(p)
    i, j = len(U) - 1, 4                         # полная тяга, узел сетки
    assert core.table_acc(U[i], V[j], p) == pytest.approx(T[i, j])
    assert drive_force(0.0, 5.0, 1.0, 1.0, p) == 0.0
    assert drive_force(1.0, 5.0, 1.0, 1.0, p) > 0
    assert drive_force(-1.0, 5.0, 1.0, 1.0, p) < 0
    assert all(resistance(v, p) >= 0 for v in (0.5, 3.0, 8.0, 15.0))


def _feed(r, t_end, v_of_t, notch_of_t, v_scale=3.6):
    """Синтетический поток как в bag: тележки ~9,4 Гц со сдвигом, ручка 20 Гц."""
    from tram_state_estimator.runner import Runner  # noqa: F401
    ev = []
    for k in range(int(t_end * 9.4)):
        t = k / 9.4
        ev += [(t, 0), (t + 0.037, 1)]
    ev += [(k * 0.05, 2) for k in range(int(t_end / 0.05))]
    outs = []
    for t, kind in sorted(ev):
        if kind == 2:
            outs += r.on_handle(t, notch_of_t(t))
        else:
            outs += r.on_wheel(kind, t, v_of_t(t) * v_scale)
    return outs


def test_runner_steps_on_message_time():
    from tram_state_estimator.runner import Runner
    r = Runner(_tram())
    outs = _feed(r, 30.0, lambda t: min(t, 10.0) * 0.8, lambda t: 8 if t < 10 else 0)
    T = np.array([o["stamp"] for o in outs])
    assert np.all(np.diff(T) > 0)
    assert 19.0 < len(T) / (T[-1] - T[0]) < 21.0      # 20 Гц, не меньше 10 Гц
    v = np.array([o["v"] for o in outs])
    truth = np.minimum(T, 10.0) * 0.8
    assert float(np.max(np.abs(v - truth)[T > 2])) < 0.3


def test_agreeing_bogies_outweigh_wrong_handle():
    """В данных вагон разгоняется при «тормозной» позиции ручки. Прежде фильтр
    держал скорость у нуля, пока обе тележки показывали разгон."""
    from tram_state_estimator.runner import Runner
    r = Runner(_tram())
    outs = _feed(r, 20.0, lambda t: max(0.0, t - 5.0) * 1.0,
                 lambda t: -10)
    v = np.array([o["v"] for o in outs])
    T = np.array([o["stamp"] for o in outs])
    truth = np.maximum(0.0, T - 5.0)
    assert float(np.max(np.abs(v - truth)[T > 8])) < 0.5


def test_position_follows_map_and_ignores_late_gnss():
    """Выставка по двум антеннам, движение по карте; GNSS после окна
    выставки не используется."""
    from tram_state_estimator.runner import Runner
    from tram_state_estimator.track_map import TrackMap
    lat0, lon0 = 55.8, 37.4
    # карта: путь на восток, затем поворот на север по дуге R = 50 м
    pts = [(0.0, float(x), np.pi / 2) for x in np.arange(-20, 200, 1.0)]
    for a in np.linspace(0, np.pi / 2, 80)[1:]:
        pts.append((50 - 50 * np.cos(a), 200 + 50 * np.sin(a), np.pi / 2 - a))
    pts += [(50.0 + y, 250.0, 0.0) for y in np.arange(1, 150, 1.0)]
    k = np.cos(np.radians(lat0))
    P = np.array(pts)
    lat = lat0 + np.degrees(P[:, 0] / 6378137.0)
    lon = lon0 + np.degrees(P[:, 1] / (6378137.0 * k))
    tm = TrackMap(lat, lon, np.zeros(len(P)), P[:, 2], np.ones(len(P)))
    r = Runner(_tram(), track_map=tm)
    for t in np.arange(0, 2.0, 0.1):                 # выставка: стоим в (0, 0)
        r.on_fix(t, "master", lat0, lon0, 0.0)
        r.on_fix(t, "rover", lat0, lon0 + np.degrees(12.0 / (6378137.0 * k)), 0.0)
    v = 5.0
    outs = _feed(r, 80.0, lambda t: v if t > 2 else 0.0, lambda t: 0)
    # поздний «GNSS» с другой точкой не должен сдвинуть оценку
    r.on_fix(80.0, "master", lat0 + 0.01, lon0, 0.0)
    o = outs[-1]
    s = v * (o["stamp"] - 2.0)                        # пройденный путь
    assert s > 300
    arc = 200 + np.pi / 2 * 50
    y_true = 50.0 + (s - arc)                         # вышли на северный участок
    assert o["x"] == pytest.approx(250.0, abs=3.0)
    assert o["y"] == pytest.approx(y_true, abs=0.01 * s + 3.0)
