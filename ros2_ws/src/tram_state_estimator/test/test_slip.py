"""Срыв обеих тележек и шум показаний (поток slip, 26.09).

Синтетический вагон с листом config/tram.yaml: истинная скорость — та же
модель (таблица привода, номинальное сцепление), поэтому ошибка оценки во
время срыва — только ошибка самого механизма, а не модели. Показания — как в
bag: тележки ~9,4 Гц со сдвигом, ручка 20 Гц, км/ч. Аномалии — как в
tools/inject.py: юз −30 % и буксование +30 % обеих тележек на 4 с, шум ×5
(σ = 0,25 м/с), одиночный выброс ×3, залипание обеих, скачок с нуля.
"""

import os
from dataclasses import fields, replace

import numpy as np
import pytest

from tram_state_estimator.estimator_core import (Estimator, Params, SLIP,
                                                 f_process)
from tram_state_estimator.runner import Runner

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE_FIELDS = {f.name for f in fields(Params)}
HZ_BOGIE, OFFSET, HZ_HANDLE = 9.4, 0.037, 20.0


def _tram(**kw):
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    d = {k: v for k, v in got.items() if k in CORE_FIELDS}
    d.update(kw)
    return Params.from_dict(d)


def _truth(p, notch_of_t, t_end, h=0.01):
    """Истинная скорость по модели листа: привод с запаздыванием, таблица,
    номинальное сцепление, без возмущения."""
    q = replace(p, dt=h)
    drv = Estimator(q)                       # только модель привода
    x = np.array([0.0, 0.0, 0.0, 1.0, 1.0])
    T, V = [0.0], [0.0]
    for k in range(int(round(t_end / h))):
        u = drv._drive_model(notch_of_t(k * h))
        x = f_process(x, u, q.mu_nominal, q)
        x[1] = max(0.0, x[1])
        T.append((k + 1) * h)
        V.append(x[1])
    return np.array(T), np.array(V)


def _run(p, notch_of_t, t_end, anomaly=None, seed=0):
    """Поток в Runner. anomaly(i, t, z) -> показание тележки i (м/с) с
    аномалией. Возвращает выходы, истину на метках выходов и «голые колёса»
    (среднее последних показаний) на тех же метках."""
    T, V = _truth(p, notch_of_t, t_end)
    rng = np.random.default_rng(seed)
    ev = []
    for k in range(int(t_end * HZ_BOGIE)):
        t = 0.05 + k / HZ_BOGIE
        ev += [(t, 0), (t + OFFSET, 1)]
    ev += [(0.05 + k / HZ_HANDLE, 2) for k in range(int(t_end * HZ_HANDLE))]
    r = Runner(p)
    outs, last = [], [0.0, 0.0]
    naive = []
    for t, kind in sorted(ev):
        if t >= t_end:
            break
        if kind == 2:
            got = r.on_handle(t, notch_of_t(t))
        else:
            z = float(np.interp(t, T, V))
            if anomaly is not None:
                z = anomaly(kind, t, z, rng)
            last[kind] = z
            got = r.on_wheel(kind, t, z * 3.6 / p.meas_scale)
        for o in got:
            outs.append(o)
            naive.append(0.5 * (last[0] + last[1]))
    st = np.array([o["stamp"] for o in outs])
    return outs, st, np.interp(st, T, V), np.array(naive)


def _arr(outs, key):
    return np.array([float(o[key]) for o in outs])


# ------------------------------------------------------------ сценарии ручки

def brake_from_speed(t):
    """Разгон до ~11 м/с, выбег, торможение с 30 с."""
    return 10 if t < 18 else (0 if t < 30 else -10)


def traction_long(t):
    return 0 if t < 2 else 6


T_SKID, T_SPIN, DUR = 32.0, 8.0, 4.0


def skid(i, t, z, rng):
    return z * 0.7 if T_SKID <= t < T_SKID + DUR else z


def spin(i, t, z, rng):
    return z * 1.3 if T_SPIN <= t < T_SPIN + DUR else z


# ------------------------------------------------------------------ тесты

def _check_slip_window(outs, st, truth, naive, t0):
    win = (st >= t0 + 0.3) & (st < t0 + DUR)
    after = (st >= t0 + DUR + 1.0) & (st < t0 + DUR + 6.0)
    v = _arr(outs, "v")
    slip = _arr(outs, "slip") > 0
    sv = _arr(outs, "sigma_v")
    err = np.abs(v - truth)
    err_naive = np.abs(naive - truth)
    assert slip[win].mean() > 0.9, "срыв обеих тележек не помечен"
    assert all(outs[i]["mode"] == SLIP for i in np.flatnonzero(win))
    # скорость ведёт модель: ошибка много меньше, чем у голых колёс
    assert err[win].mean() < 0.2 * err_naive[win].mean()
    # σ честная: ошибка внутри 2σ
    assert np.all(err[win] <= 2.0 * sv[win] + 0.05)
    # после срыва — быстро назад к колёсам, флаг снят
    assert err[after].max() < 0.15
    assert not slip[after].any()
    assert not any(o.get("slip_all") for o in outs if o["stamp"] < t0)


def test_both_bogie_skid_on_braking_is_flagged_and_model_holds_speed():
    """Юз обеих тележек −30 % на торможении. Прежде обе тележки согласны
    друг с другом, и правило «согласие осей сильнее модели» принимало юз за
    торможение: ошибка как у голых колёс (1,8 м/с), флага нет."""
    p = _tram()
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, skid)
    assert truth[np.searchsorted(st, T_SKID)] > 5.0
    _check_slip_window(outs, st, truth, naive, T_SKID)
    assert any(o.get("slip_all") == -1 for o in outs)


def test_both_bogie_spin_on_traction_is_flagged_and_model_holds_speed():
    """Буксование обеих тележек +30 % на разгоне: признак прежде срабатывал
    частично, а снижение μ без ненагруженных осей останавливало модель, и
    оценка отставала от вагона до 20 с после срыва."""
    p = _tram()
    outs, st, truth, naive = _run(p, traction_long, 20.0, spin)
    assert truth[np.searchsorted(st, T_SPIN)] > 2.5
    _check_slip_window(outs, st, truth, naive, T_SPIN)
    assert any(o.get("slip_all") == 1 for o in outs)
    # сцепление по срыву всех осей не оценивается — модель не урезана
    assert min(_arr(outs, "mu")) > 0.8 * p.mu_nominal


def test_both_bogie_skid_off_switch_restores_old_behaviour():
    """slip_all_on = false — прежние признаки: юз обеих тележек не помечен."""
    p = _tram(slip_all_on=False)
    outs, st, truth, naive = _run(p, brake_from_speed, 40.0, skid)
    win = (st >= T_SKID + 0.3) & (st < T_SKID + DUR)
    assert not any(o.get("slip_all") for o in outs)
    err = np.abs(_arr(outs, "v") - truth)
    assert err[win].mean() > 0.5


def test_noise_x5_is_noise_not_slip():
    """Шум ×5 (σ = 0,25 м/с) на обеих тележках: «аномально высокая дисперсия»
    — это шум. Оценка шума растёт, дисперсия измерения тоже, срыва нет, и
    фильтр усредняет лучше голых колёс. Прежде шум давал ложные срывы, μ
    падало до минимума, и оценка отставала на 3–5 м/с."""
    p = _tram()
    t0, t1 = 8.0, 28.0

    def noise(i, t, z, rng):
        return z + rng.normal(0.0, 0.25) if t0 <= t < t1 else z
    outs, st, truth, naive = _run(p, traction_long, 32.0, noise, seed=3)
    win = (st >= t0 + 2.0) & (st < t1)
    v = _arr(outs, "v")
    assert _arr(outs, "meas_noise")[win].min() > 2.0 * p.sigma_meas
    assert not any(outs[i].get("slip_all") for i in np.flatnonzero(win))
    assert _arr(outs, "slip")[win].mean() < 0.05
    err = np.abs(v - truth)[win].mean()
    assert err < 0.8 * np.abs(naive - truth)[win].mean()
    # пока оценка шума набирается (~1 с), отдельные показания ещё дают
    # признак срыва и μ немного падает, но не до mu_min, как прежде
    assert min(_arr(outs, "mu")) > 0.3 * p.mu_nominal
    # после шума оценка шума возвращается к паспортной
    assert _arr(outs, "meas_noise")[-1] < p.sigma_meas


def test_single_outlier_does_not_collapse_adhesion():
    """Одиночный выброс ×3 одной тележки под тягой. Прежде сглаженная
    производная держалась за пределом ещё несколько показаний, каждое
    снижало μ, модель переставала разгонять вагон — и признак поддерживал
    сам себя (+3 м/с на 18 с на обучающем прогоне)."""
    p = _tram()
    hit = []

    def outlier(i, t, z, rng):
        if i == 0 and t >= 8.0 and not hit:
            hit.append(t)
            return 3.0 * z
        return z
    outs, st, truth, naive = _run(p, traction_long, 20.0, outlier)
    after = st >= 9.5
    err = np.abs(_arr(outs, "v") - truth)
    assert err[after].max() < 0.15
    assert min(_arr(outs, "mu")[after]) > 0.8 * p.mu_nominal
    assert not any(o.get("slip_all") for o in outs)


def test_start_from_standstill_jump_is_not_slip():
    """Скачок обеих тележек с нуля (начало потока на ходу, трогание) — не
    срыв: признаки срыва оцениваются только на ходу."""
    p = _tram()

    def start(i, t, z, rng):
        return 5.0 if t > 2.0 else 0.0
    outs, st, truth, naive = _run(p, lambda t: 0, 12.0, start)
    assert not any(o.get("slip_all") for o in outs)
    v = _arr(outs, "v")
    assert abs(v[-1] - 5.0) < 0.1


def test_both_bogies_stuck_then_released_is_not_slip():
    """Залипание обеих тележек, затем показания возвращаются скачком к
    истинной скорости: это конец отказа, а не срыв (скачки после
    неподтверждённого состояния не считаются)."""
    p = _tram()
    held = {}

    def stuck(i, t, z, rng):
        if 8.0 <= t < 14.0:
            return held.setdefault(i, z)
        return z
    outs, st, truth, naive = _run(p, traction_long, 22.0, stuck)
    assert not any(o.get("slip_all") for o in outs)
    err = np.abs(_arr(outs, "v") - truth)
    assert err[st >= 16.0].max() < 0.15


def test_clean_stop_and_go_has_no_slip():
    p = _tram()

    def stop_go(t):
        c = t % 40
        return 8 if c < 15 else (0 if c < 22 else (-10 if c < 32 else 0))
    outs, st, truth, naive = _run(p, stop_go, 120.0)
    assert not any(o["slip"] for o in outs)
    assert _arr(outs, "meas_noise").max() < p.sigma_meas
    assert np.abs(_arr(outs, "v") - truth).mean() < 0.05


def test_slip_longer_than_limit_releases_and_return_is_not_a_new_slip():
    """Буксование обеих тележек дольше slip_t_max: после предела колёсам
    снова верят (защита от защёлки), а скачок обратно, когда колёса схватят,
    — возвращение, а не новый юз. Прежде (до правила) он открывал новый
    срыв, и модель 8 с держала скорость скользивших колёс."""
    p = _tram(slip_t_max=3.0)
    t0, t1 = 8.0, 13.0

    def spin_long(i, t, z, rng):
        return z * 1.3 if t0 <= t < t1 else z
    outs, st, truth, naive = _run(p, traction_long, 25.0, spin_long)
    ep = _arr(outs, "slip_all")
    assert (ep[(st >= t0 + 0.3) & (st < t0 + 3.0)] == 1).all()
    assert not (ep[st >= t0 + 3.2] != 0).any(), "после предела и при возвращении срыва нет"
    err = np.abs(_arr(outs, "v") - truth)
    assert err[st >= t1 + 1.0].max() < 0.15
