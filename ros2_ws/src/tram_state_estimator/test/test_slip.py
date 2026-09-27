"""Срыв обеих тележек и шум показаний.

Синтетический вагон с листом config/tram.yaml: истинная скорость — та же
модель (таблица привода, номинальное сцепление), поэтому ошибка оценки во
время срыва — только ошибка самого механизма, а не модели. Показания — как в
bag: тележки ~9,4 Гц со сдвигом, ручка 20 Гц, км/ч. Аномалии — как в
tools/inject.py: юз −30 % и буксование +30 % обеих тележек на 4 с, шум ×5
(σ = 0,25 м/с), одиночный выброс ×3, залипание обеих, скачок с нуля. Формы
срыва на границах (tools/slip_study.py, виды EDGE): юз, перешедший в
блокировку, задняя тележка на 0,3 с позже, плавное начало, плавный конец.
"""

import os
from dataclasses import fields, replace

import numpy as np
import pytest

from tram_state_estimator.estimator_core import (STANDSTILL, Estimator,
                                                 Params, SLIP, f_process)
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


def _truth(p, notch_of_t, t_end, h=0.01, kb=1.0):
    """Истинная скорость по модели листа: привод с запаздыванием, таблица,
    номинальное сцепление, без возмущения. kb — масштаб торможения вагона
    против таблицы (ошибка модели: kb > 1 — вагон тормозит сильнее)."""
    q = replace(p, dt=h)
    drv = Estimator(q)                       # только модель привода
    x = np.array([0.0, 0.0, 0.0, 1.0, kb])
    T, V = [0.0], [0.0]
    for k in range(int(round(t_end / h))):
        u = drv._drive_model(notch_of_t(k * h))
        x = f_process(x, u, q.mu_nominal, q)
        x[1] = max(0.0, x[1])
        T.append((k + 1) * h)
        V.append(x[1])
    return np.array(T), np.array(V)


def _run(p, notch_of_t, t_end, anomaly=None, seed=0, offset=OFFSET, kb=1.0):
    """Поток в Runner. anomaly(i, t, z) -> показание тележки i (м/с) с
    аномалией. Возвращает выходы, истину на метках выходов и «голые колёса»
    (среднее последних показаний) на тех же метках. offset — сдвиг задней
    тележки; 0 — обе приходят на одном шаге фильтра, как чаще всего в bag."""
    T, V = _truth(p, notch_of_t, t_end, kb=kb)
    rng = np.random.default_rng(seed)
    ev = []
    for k in range(int(t_end * HZ_BOGIE)):
        t = 0.05 + k / HZ_BOGIE
        ev += [(t, 0), (t + offset, 1)]
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
    # признак срыва и μ немного падает, но не до mu_min
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


# ------------------------------------------------ формы срыва на границах

T_LOCK0, T_LOCK1, T_LOCK2 = 30.6, 32.6, 35.6


@pytest.mark.parametrize("offset", [0.0, OFFSET])
def test_skid_turning_into_lock_is_not_a_stop(offset):
    """Юз обеих тележек −30 % 2 с, затем блокировка (нули) 3 с, затем
    колёса снова катятся. В bag обе тележки чаще приходят на одном шаге.
    Прежде срыв всех осей снимался нулями вместе с признаком скачка этого же
    шага, нули проходили правило согласия осей как остановка: v падала с
    13 м/с в ноль с σ 0,02 и valid = true (обучающий прогон 30618_2dbce472).
    Теперь — неоднозначность «стоим или скользим»: оценка держится, valid
    снят, σ широкая."""
    p = _tram()

    def lock(i, t, z, rng):
        if T_LOCK0 <= t < T_LOCK1:
            return 0.7 * z
        return 0.0 if T_LOCK1 <= t < T_LOCK2 else z
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, lock, offset=offset)
    v = _arr(outs, "v")
    err = np.abs(v - truth)
    valid = _arr(outs, "valid") > 0
    amb = _arr(outs, "ambiguous") > 0
    assert truth[np.searchsorted(st, T_LOCK1)] > 10.0
    assert not any(o["mode"] == STANDSTILL and tv > 2.0
                   for o, tv in zip(outs, truth)), "остановка на ходу"
    lk = (st >= T_LOCK1 + 0.2) & (st < T_LOCK2)
    assert amb[lk].all()
    assert (v[lk] > 0.5 * truth[lk]).all()
    win = (st >= T_LOCK0) & (st < T_LOCK2 + 3.0)
    assert not (valid & (err > 1.0))[win].any(), "уверенно неверная скорость"
    sv = _arr(outs, "sigma_v")
    assert np.all(err[lk] <= 2.0 * sv[lk])
    after = (st >= T_LOCK2 + 3.0) & (st < T_LOCK2 + 8.0)
    assert err[after].max() < 0.15


def test_staggered_skid_pairs_into_one_slip():
    """Задняя тележка срывается на 0,3 с позже передней (slip_pair_s =
    0,5 с). Прежде ровное показание после скачка давало «скачок» обратного
    знака (сглаженная производная вобрала скачок), он затирал знак
    передней, пара не складывалась, срыв не помечался, а отпускание юза
    открывало ложное буксование на 12 с."""
    p = _tram()

    def stagger(i, t, z, rng):
        s0 = T_SKID + (0.3 if i == 1 else 0.0)
        return 0.7 * z if s0 <= t < T_SKID + DUR else z
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, stagger)
    ep = _arr(outs, "slip_all")
    win = (st >= T_SKID + 0.5) & (st < T_SKID + DUR)
    assert (ep[win] == -1).mean() > 0.9
    assert not (ep > 0).any()
    err = np.abs(_arr(outs, "v") - truth)
    assert err[win].mean() < 0.2 * np.abs(naive - truth)[win].mean()
    after = (st >= T_SKID + DUR + 1.0) & (st < T_SKID + DUR + 12.0)
    assert err[after].max() < 0.15
    assert not (_arr(outs, "slip")[after] > 0).any()


def test_unflagged_ramp_in_skid_release_is_not_spin():
    """Юз, начавшийся плавно (0,5 с, без скачка), не распознаётся — это
    ограничение (MODEL.md §9). Но его отпускание скачком вверх при ручке в
    торможении — не буксование: прежде оно открывало срыв всех осей «+1»,
    и модель 12 с держала скорость после юза (ошибка до 1,8 м/с)."""
    p = _tram()

    def rampin(i, t, z, rng):
        if T_SKID <= t < T_SKID + 0.5:
            return z * (1.0 - 0.3 * (t - T_SKID) / 0.5)
        return 0.7 * z if T_SKID + 0.5 <= t < T_SKID + DUR else z
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, rampin)
    assert not (_arr(outs, "slip_all") > 0).any()
    err = np.abs(_arr(outs, "v") - truth)
    after = (st >= T_SKID + DUR + 1.0) & (st < T_SKID + DUR + 8.0)
    assert err[after].max() < 0.15
    assert not (_arr(outs, "slip")[after] > 0).any()


@pytest.mark.parametrize("kind", ["skid", "spin"])
def test_gradual_release_ends_slip(kind):
    """Срыв начался скачком, а отпускается плавно (1 с, без скачка
    обратно). Прежде признак держался до slip_t_max (ещё ~8 с после того,
    как колёса вернулись). Теперь срыв снимается, когда все новые
    показания принимаются подряд slip_ok_s."""
    p = _tram()
    lo, t0, hnd, te = ((0.7, T_SKID, brake_from_speed, 45.0) if kind == "skid"
                       else (1.3, T_SPIN, traction_long, 25.0))

    def rampout(i, t, z, rng):
        if t0 <= t < t0 + DUR - 1.0:
            return lo * z
        if t0 + DUR - 1.0 <= t < t0 + DUR:
            return z * (lo + (1.0 - lo) * (t - (t0 + DUR - 1.0)))
        return z
    outs, st, truth, naive = _run(p, hnd, te, rampout)
    ep = _arr(outs, "slip_all")
    sign = -1 if kind == "skid" else 1
    win = (st >= t0 + 0.3) & (st < t0 + DUR - 1.0)
    assert (ep[win] == sign).all()
    t_end = st[np.flatnonzero(ep != 0)].max()
    assert t_end < t0 + DUR + p.slip_ok_s + 1.0, f"срыв снят только в {t_end:.1f} с"
    err = np.abs(_arr(outs, "v") - truth)
    assert err[(st >= t0) & (st < t0 + DUR)].mean() < 0.2 * np.abs(
        naive - truth)[(st >= t0) & (st < t0 + DUR)].mean()
    after = (st >= t0 + DUR + 1.0) & (st < t0 + DUR + 8.0)
    assert err[after].max() < 0.15


T_LOCK_FAST = 36.0      # истинная скорость ≈ 6 м/с


@pytest.mark.parametrize("offset", [0.0, OFFSET])
@pytest.mark.parametrize("seq", [(0.82, 0.10), (0.82, 0.48, 0.10), "ramp03"])
def test_fast_lock_to_zero_is_ambiguous_not_a_stop(seq, offset):
    """Блокировка с 6 м/с: обе тележки падают в ноль за 0,2–0,3 с (два-три
    показания при 10 Гц) и 5 с стоят на нуле, как при экстренном торможении.
    Срыв всех осей начинается скачком вниз и снимается нулями. Если
    последний шаг к нулю меньше порога скачка (0,6 → 0 м/с), прежде нули
    проходили согласие осей как остановка: v = 0, σ 0,02, valid = true на
    ходу 6 м/с (холдаут 30618_a53d5f6f, 5,25 → 3,09 → 0,64 → 0). Теперь —
    неоднозначность «стоим или скользим»: valid снят, оценка держится."""
    p = _tram()
    t0, dur = T_LOCK_FAST, 5.0

    def lock(i, t, z, rng):
        tr = t - t0
        if not 0.0 <= tr < dur:
            return z
        if seq == "ramp03":
            return z * max(0.0, 1.0 - tr / 0.3)
        k = int(tr * HZ_BOGIE)
        return z * (seq[k] if k < len(seq) else 0.0)
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, lock, offset=offset)
    v = _arr(outs, "v")
    err = np.abs(v - truth)
    valid = _arr(outs, "valid") > 0
    amb = _arr(outs, "ambiguous") > 0
    assert truth[np.searchsorted(st, t0)] > 5.5
    win = (st >= t0) & (st < t0 + dur)
    assert not (valid & (err > 1.0))[win].any(), "уверенно неверная скорость"
    lk = (st >= t0 + 0.5) & (st < t0 + dur)
    assert amb[lk].all() and not valid[lk].any()
    moving = win & (truth > 2.0)
    assert not any(outs[i]["mode"] == STANDSTILL for i in np.flatnonzero(moving))
    assert (v[moving] > 1.0).all(), "оценка упала в ноль на ходу"


@pytest.mark.parametrize("ramp", [1.0, 2.0])
@pytest.mark.parametrize("kb", [1.0, 1.1, 1.3])
def test_gradual_release_with_model_error_returns_to_wheels(kb, ramp):
    """Юз обеих тележек −30 %, отпускание плавное (1–2 с), а таблица
    привода тормозит слабее вагона (kb = 1,1–1,3: за 4 с срыва модель уходит
    от вагона на 0,4–1,6 м/с). Вернувшиеся колёса оставались «в сторону
    срыва» от ушедшей модели: срыв держался до нулей или slip_t_max, модель
    вела скорость ещё до 5–8 с (на обучающих прогонах — до 8 с). Теперь
    возвращение видно по производной колеса против ускорения модели."""
    p = _tram()
    t0 = 31.0

    def rampout(i, t, z, rng):
        if t0 <= t < t0 + DUR - ramp:
            return 0.7 * z
        if t0 + DUR - ramp <= t < t0 + DUR:
            return z * (0.7 + 0.3 * (t - (t0 + DUR - ramp)) / ramp)
        return z
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, rampout, kb=kb)
    ep = _arr(outs, "slip_all")
    assert (ep[(st >= t0 + 0.3) & (st < t0 + DUR - ramp)] == -1).all()
    assert not (ep[st >= t0 + DUR + 1.0] != 0).any(), "срыв не снят после отпускания"
    err = np.abs(_arr(outs, "v") - truth)
    after = (st >= t0 + DUR + 1.0) & (st < t0 + DUR + 8.0)
    assert err[after].max() < 0.15


@pytest.mark.parametrize("depth", [0.7, 0.55])
@pytest.mark.parametrize("kb", [0.85, 1.0, 1.1])
def test_steady_skid_with_model_error_is_not_a_return(kb, depth):
    """Ровный юз −30 % и −45 % 4 с при ошибке таблицы торможения −15…+10 %:
    колесо — доля ρ скорости вагона, его производная — ρ·a, модель —
    быстрее или медленнее вагона. Это не возвращение колёс: срыв держится
    всё окно. Прежде (без ρ) в глубоком юзе ровное колесо отходило от
    ускорения модели на (1 − ρ)·|a| (0,6–0,7 м/с²), и срыв ложно снимался
    (юз −50 % на обучающих прогонах: 0,435 → 0,579 м/с); при пороге
    slip_ret_a = 0,5 так было уже в юзе −30 % при kb = 0,85. Юз −50 % в
    этой синтетике — ровно порог блокировки lock_ratio (другая ветка),
    поэтому здесь −45 %."""
    p = _tram()

    def deep(i, t, z, rng):
        return z * depth if T_SKID <= t < T_SKID + DUR else z
    outs, st, truth, naive = _run(p, brake_from_speed, 45.0, deep, kb=kb)
    ep = _arr(outs, "slip_all")
    assert (ep[(st >= T_SKID + 0.3) & (st < T_SKID + DUR)] == -1).all()
