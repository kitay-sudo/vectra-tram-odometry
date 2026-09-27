"""Коррекция положения по GNSS в середине маршрута (gnss_correction, 26.09).

Организаторы 26.09 18:05: «в середине маршрута могут быть ещё сообщения
[GNSS], которые можно использовать для коррекции». Проверяется:

  * при gnss_correction: false выход бит-в-бит как в main (GNSS только для
    выставки), при GNSS только в окне — бит-в-бит и с коррекцией;
  * GNSS не двигает сетку и не влияет на скорость;
  * скачки GNSS (одиночные и по нескольку эпох, по статусу 2 и 0) отбрасываются,
    метки не по часам входов — тоже; устойчивая невязка после пропуска GNSS
    исправляется; вес точки — по статусу;
  * одна антенна, выставка по GNSS посреди прогона, σ после поправки;
  * мусор на входе GNSS не роняет связку.
ROS не требуется.
"""

import math
import os

import numpy as np
import pytest

from tram_state_estimator import geodesy as g
from tram_state_estimator.body import MASTER_X, ROVER_X
from tram_state_estimator.estimator_core import Params
from tram_state_estimator.runner import Runner
from tram_state_estimator.track_map import TrackMap

import e2e_replay as E

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = ROVER_X - MASTER_X            # 12,436 м
BL = -MASTER_X                       # 9,873 м: base_link впереди master
E0, N0 = 401000.0, 6185000.0         # восточнее границы 400 км: без сюрпризов квадратов
ALT_RAIL = 150.0
ALT_ANT = ALT_RAIL + 3.0
V = 10.0                             # м/с
T_START = 3.0                        # с: стоим, потом разгон ACC до V
ACC = 1.0                            # м/с²
# Выход main (7d1855b, GNSS только для выставки) на фикстуре
# e2e_30618_b95ca60a_180s: sha256 e2e_replay.digest при GNSS в окне и весь кусок
# (одинаковые), с картой пакета и без. Записано в образе vectra/tram:integration
# (numpy 1.21.5) с кодом main из дерева; в другом окружении числа могут
# отличаться в последнем бите — тогда сверка с записью пропускается.
MAIN_DIGEST = {True: "815fe46efa96989fd2436026fe490b4a7612f0dad1dd3419f7a3801c643a5999",
               False: "e885a6c6f7ad110457afcc5358ead3aafd98a543183126e8b635d5a5c673a7fd"}
MAIN_NUMPY = "1.21.5"


def _tram():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, "config", "tram.yaml"), encoding="utf-8") as fh:
        got = yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]
    names = set(Params.__dataclass_fields__)
    return Params.from_dict({k: v for k, v in got.items() if k in names})


def _map(L=3000.0):
    """Прямой путь на восток длиной L (ось base_link, уровень рельса)."""
    x = np.arange(-50.0, L, 1.0)
    P = np.c_[E0 + x, np.full(len(x), N0), np.full(len(x), ALT_RAIL)]
    return TrackMap.from_polylines([P], crs="utm", zone=37, bidirectional=False)


def _latlon(e, n):
    la, lo = g.utm_inv(e, n, 37)
    return float(la), float(lo)


def v_true(t):
    return min(V, ACC * max(0.0, t - T_START))


def s_true(t):
    """Путь base_link по пути от начала карты (стоим до T_START, разгон)."""
    u = max(0.0, t - T_START)
    ta = V / ACC
    return 0.5 * ACC * u * u if u <= ta else 0.5 * V * ta + V * (u - ta)


def feed(r, t0, t1, wheel_gain=1.0, gnss=lambda t: t <= 2.0, fix=None, rover=True,
         status=2, extra=None, prof=None):
    """Поток как в bag: тележки ~9,4 Гц, ручка 20 Гц, GNSS эпохи 10 Гц, пока
    gnss(t). fix(t) -> (dE, dN, status, stamp_shift) — искажение эпохи (по
    умолчанию нет). extra(t, r) — вызывается на каждой эпохе (мусор и т. п.).
    Колёса показывают V·wheel_gain. prof — (v(t), s(t), ручка(t)) вместо
    разгона до V (например, stop_profile)."""
    v_true_, s_true_, notch_ = prof or (
        v_true, s_true, lambda t: 3 if T_START < t < T_START + V / ACC else 0)
    ev = []
    for k in range(int((t1 - t0) * 9.4)):
        t = t0 + k / 9.4
        ev += [(t, 0), (t + 0.037, 1)]
    ev += [(t0 + k * 0.05, 2) for k in range(int((t1 - t0) / 0.05))]
    ev += [(t0 + k * 0.1 + 0.013, 3) for k in range(int((t1 - t0) / 0.1))]
    outs = []
    for t, kind in sorted(ev):
        if kind == 2:
            outs += r.on_handle(t, notch_(t))
        elif kind < 2:
            outs += r.on_wheel(kind, t, v_true_(t) * wheel_gain * 3.6)
        elif gnss(t - t0):
            dE, dN, st, sh = fix(t - t0) if fix is not None else (0.0, 0.0, status, 0.0)
            s = s_true_(t)
            e_m = E0 + s - BL                              # master позади base_link
            outs += r.on_fix(t + sh, "master", *_latlon(e_m + dE, N0 + dN), ALT_ANT, st)
            if rover:
                outs += r.on_fix(t + sh, "rover", *_latlon(e_m + BASE + dE, N0 + dN),
                                 ALT_ANT, st)
            if extra is not None:
                outs += extra(t, r)
    return outs


def along_err(o, r, s_fn=None):
    """Ошибка вдоль пути (м, + — впереди) выхода o против истины: путь на
    восток, выход в MGRS от 37UCB (x = E − 300 000)."""
    return o["x"] + 300000.0 - (E0 + (s_fn or s_true)(o["stamp"]))


def runner(**kw):
    return Runner(_tram(), track_map=_map(), **kw)


def stop_profile(x_stop):
    """Разгон до V, ход, торможение ACC до остановки ровно на x_stop м пути
    и стоянка -> (v(t), s(t), ручка(t), момент остановки)."""
    ta = V / ACC
    t_dec = T_START + ta + (x_stop - V * ta) / V
    t_halt = t_dec + ta

    def v(t):
        if t <= T_START:
            return 0.0
        if t <= T_START + ta:
            return ACC * (t - T_START)
        if t <= t_dec:
            return V
        return max(0.0, V - ACC * (t - t_dec))

    def s(t):
        u = max(0.0, t - T_START)
        if u <= ta:
            return 0.5 * ACC * u * u
        s1 = 0.5 * V * ta + V * (min(t, t_dec) - T_START - ta)
        w = min(max(t - t_dec, 0.0), ta)
        return s1 + V * w - 0.5 * ACC * w * w

    def notch(t):
        if T_START < t < T_START + ta:
            return 3
        return -3 if t_dec < t < t_halt + 1.0 else 0

    return (v, s, notch), t_halt


def final_err(outs, r):
    return along_err(outs[-1], r)


# ------------------------------------------------------------ бит-в-бит

# Как было в main до потока vehicle: общий лист (вагон auto), без онлайн-масштаба
# колёс. Карта пакета с 27.09 другая (поток loops: ветки за тупиком у западной
# конечной, облако там без проходов-веток), поэтому с main сверяется выход без
# карты; с картой main (файл из main 56da11b) интеграция 27.09 дала ровно
# MAIN_DIGEST[True] (out/int3/dbg/digest.py).
MAIN_CFG = dict(vehicle="auto", wheel_scale_online=False)


def test_flag_off_is_bit_identical_to_main():
    """gnss_correction: false — выход как в main: GNSS весь кусок = GNSS в
    окне (C2), и (в окружении записи, без карты) sha256 равен выходу main."""
    fx = E.load_fixture()
    for use_map in (True, False):
        a = E.digest(E.replay(fx, use_map=use_map, gnss="all", gnss_correction=False,
                              **MAIN_CFG))
        b = E.digest(E.replay(fx, use_map=use_map, gnss="window", gnss_correction=False,
                              **MAIN_CFG))
        assert a == b
        if np.__version__ == MAIN_NUMPY and not use_map:
            assert a == MAIN_DIGEST[use_map]


def test_first3_with_correction_is_identical():
    """GNSS только в окне выставки (метки точек в пределах окна): коррекция
    ничего не меняет — бит-в-бит с выключенной."""
    outs = {}
    for flag in (True, False):
        r = runner(gnss_correction=flag)
        outs[flag] = feed(r, 0.0, 60.0, wheel_gain=1.02)
    keys = ("stamp", "v", "x", "y", "z", "sigma_s", "sigma_v", "mode", "pos_valid")
    assert [[o[k] for k in keys] for o in outs[True]] == \
        [[o[k] for k in keys] for o in outs[False]]


def test_first3_by_record_time_on_real_fixture_is_tiny():
    """Реальная фикстура, GNSS первые 3 с по времени записи (как в оценке):
    первая точка пришла из стартового всплеска bag с меткой на 2,4 с раньше,
    и 23 точки этих 3 с имеют метку уже после окна выставки — их берёт
    коррекция. Разница — сантиметры; сетка и скорость те же."""
    fx = E.load_fixture()
    a = E.replay(fx, use_map=True, gnss="window", gnss_correction=False)
    b = E.replay(fx, use_map=True, gnss="window", gnss_correction=True)
    assert [o["stamp"] for o in a] == [o["stamp"] for o in b]
    assert [o["v"] for o in a] == [o["v"] for o in b]
    d = max(math.hypot(p["x"] - q["x"], p["y"] - q["y"]) for p, q in zip(a, b))
    assert d < 0.1, d


def test_whole_slice_gnss_real_fixture_speed_same_position_closer():
    """Реальная фикстура, GNSS весь кусок: сетка и скорость те же, что без
    коррекции; положение ближе к эталону base_link."""
    fx = E.load_fixture()
    off = E.replay(fx, use_map=True, gnss="window", gnss_correction=True)
    on = E.replay(fx, use_map=True, gnss="all", gnss_correction=True)
    assert [o["stamp"] for o in on] == [o["stamp"] for o in off]
    assert [o["v"] for o in on] == [o["v"] for o in off]
    m_off, m_on = E.metrics(off, fx), E.metrics(on, fx)
    assert m_on["p_mean3d"] < m_off["p_mean3d"]
    assert m_on["v_mae"] == m_off["v_mae"]


# ------------------------------------------------------------ сетка и скорость

def test_gnss_never_moves_the_grid():
    """Поздняя и ранняя (в будущем) точка GNSS после окна не двигают сетку и
    не порождают выходов; метки и скорость с GNSS весь прогон и без него
    одинаковы бит-в-бит."""
    r = runner()
    feed(r, 0.0, 20.0)
    t = r.t
    la, lo = _latlon(E0, N0)
    for st in (t - 1.0, t + 0.2, t + 5.0):
        assert r.on_fix(st, "master", la, lo, ALT_ANT, 2) == []
        assert r.t == t
    a = feed(runner(), 0.0, 40.0, gnss=lambda t: True)
    b = feed(runner(), 0.0, 40.0)
    assert [o["stamp"] for o in a] == [o["stamp"] for o in b]
    assert [o["v"] for o in a] == [o["v"] for o in b]


# ------------------------------------------------------------ поправка и отбраковка

def test_drift_is_corrected_by_mid_route_burst():
    """Колёса врут на +1 % (путь убегает на 1 м за 100 м); пачка GNSS на 6 с
    через минуту: невязка ~5 м в воротах — после 3 согласных эпох положение
    возвращается к истине, σ после поправки меньше."""
    r = runner()
    outs = feed(r, 0.0, 75.0, wheel_gain=1.01, gnss=lambda t: t <= 2.0 or 62.0 <= t <= 68.0)
    before = [o for o in outs if 61.0 <= o["stamp"] <= 61.9]
    assert abs(along_err(before[-1], r)) > 4.0             # ~1 % от 530 м
    right_after = [o for o in outs if 68.2 <= o["stamp"] <= 68.3]
    assert abs(along_err(right_after[0], r)) < 1.0
    assert abs(final_err(outs, r)) < 1.5                   # и дальше — тот же 1 % от 70 м
    assert r.pos.n_corr > 0
    s_before = before[-1]["sigma_s"]
    after = [o for o in outs if 68.5 <= o["stamp"] <= 69.0]
    assert after[0]["sigma_s"] < 0.5 * s_before


def test_large_drift_needs_lasting_rtk():
    """Колёса врут на +2 %: через минуту невязка ~11 м — вне ворот (для
    колёс это невероятно). Пачка на 6 с — короче gnss_persist_s: не верим
    (так же выглядит сбой GNSS). Пачка на 15 с — несогласие держится,
    оценка ставится по GNSS."""
    short = runner()
    outs = feed(short, 0.0, 75.0, wheel_gain=1.02, gnss=lambda t: t <= 2.0 or 62.0 <= t <= 68.0)
    assert abs(final_err(outs, short)) > 10.0
    r = runner()
    outs = feed(r, 0.0, 85.0, wheel_gain=1.02, gnss=lambda t: t <= 2.0 or 62.0 <= t <= 77.0)
    right_after = [o for o in outs if 77.2 <= o["stamp"] <= 77.3]
    assert abs(along_err(right_after[0], r)) < 1.0
    assert r.pos.n_corr_big >= 1


def test_off_flag_ignores_mid_route_burst():
    r = runner(gnss_correction=False)
    outs = feed(r, 0.0, 75.0, wheel_gain=1.02, gnss=lambda t: t <= 2.0 or 62.0 <= t <= 68.0)
    assert abs(final_err(outs, r)) > 10.0 and r.pos.n_corr == 0


@pytest.mark.parametrize("n_epochs,status", [(1, 2), (3, 2), (5, 2), (5, 0), (20, 0)])
def test_gnss_jump_is_rejected(n_epochs, status):
    """GNSS весь прогон; на n_epochs эпох он прыгает на 40 м (вбок и вперёд).
    Оценка по колёсам непрерывна: скачок — сбой GNSS, положение не уходит."""
    r = runner()
    j0 = 40.0

    def fix(t):
        if j0 <= t < j0 + 0.1 * n_epochs - 1e-6:
            return 30.0, 25.0, status, 0.0
        return 0.0, 0.0, 2, 0.0

    outs = feed(r, 0.0, 50.0, gnss=lambda t: True, fix=fix)
    err = [abs(along_err(o, r)) for o in outs if o["stamp"] >= 10.0]
    assert max(err) < 1.5, max(err)
    assert r.pos.n_corr_gated >= 1


def test_long_rtk_jump_is_not_followed():
    """RTK прыгает вперёд на 12 м и держится 8 с (так в 30618_b95ca60a):
    сразу после согласной эпохи — это скачок GNSS, не переставляем."""
    r = runner()
    outs = feed(r, 0.0, 60.0, gnss=lambda t: True,
                fix=lambda t: (12.0, 0.0, 2, 0.0) if 30.0 <= t < 38.0 else (0.0, 0.0, 2, 0.0))
    err = [abs(along_err(o, r)) for o in outs if o["stamp"] >= 10.0]
    assert max(err) < 1.5, max(err)


def test_stamp_off_input_clock_is_rejected():
    """Метка GNSS на 1 с не по часам тележек (в данных — участки по минутам):
    точка не берётся (иначе ошибка v·1 с = 10 м)."""
    r = runner()
    outs = feed(r, 0.0, 60.0, gnss=lambda t: t <= 2.0 or 30.0 <= t <= 40.0,
                fix=lambda t: (0.0, 0.0, 2, 1.0 if t > 3.0 else 0.0))
    assert r.pos.n_corr == 0 and r.pos.n_corr_skew > 0
    err = [abs(along_err(o, r)) for o in outs if o["stamp"] >= 10.0]
    assert max(err) < 1.0


def test_status_weighting():
    """Одна и та же невязка 3 м: RTK (статус 2) тянет почти целиком, без
    поправок (статус 0) — едва."""
    shifts = {}
    for st in (2, 0):
        r = runner()
        outs = feed(r, 0.0, 45.0, gnss=lambda t: t <= 2.0 or 40.0 <= t <= 40.35,
                    fix=lambda t, st=st: (3.0, 0.0, st, 0.0) if t > 3.0 else (0.0, 0.0, 2, 0.0))
        shifts[st] = along_err(outs[-1], r)
    assert shifts[2] > 2.0
    assert abs(shifts[0]) < 1.0


def test_single_antenna_fix():
    """После окна идёт только master: base_link — перенос вдоль курса карты
    на 9,87 м; дрейф исправляется."""
    r = runner()
    outs = feed(r, 0.0, 75.0, wheel_gain=1.01, rover=False,
                gnss=lambda t: t <= 2.0 or 62.0 <= t <= 68.0)
    right_after = [o for o in outs if 68.2 <= o["stamp"] <= 68.3]
    assert abs(along_err(right_after[0], r)) < 1.5
    assert r.pos.n_corr > 0


def test_alignment_mid_route_when_no_gnss_at_start():
    """В начале GNSS нет (узел запущен поздно, старт с середины): первая
    годная пара посреди прогона выставляет положение."""
    r = runner()
    outs = feed(r, 0.0, 40.0, gnss=lambda t: 25.0 <= t <= 26.0)
    assert not any(o["pos_valid"] for o in outs if o["stamp"] < 25.0)
    assert r.pos.ready
    assert abs(final_err(outs, r)) < 1.0


def test_realign_when_window_had_no_heading():
    """Окно выставки: только master и стоим вне карты — курса нет. Пара
    после окна (gnss_correction) выставляет заново: курс есть, положение
    идёт по пути."""
    r = Runner(_tram())                                   # без карты
    feed(r, 0.0, 10.0, rover=False)
    assert r.pos.fixed and not r.pos.ready
    outs = feed(r, 10.0, 30.0, gnss=lambda t: True)
    assert r.pos.ready and r.pos.n_realign >= 1
    assert outs[-1]["pos_valid"]


def test_garbage_gnss_after_window_does_not_crash():
    """NaN, бесконечность, строки, (0, 0), широта за 90°, статус −1, чужая
    антенна — связка жива, положение не портится."""
    bad = [(math.nan, 37.0, 150.0, 2), (math.inf, 37.0, 150.0, 2), ("x", "y", "z", 2),
           (0.0, 0.0, 0.0, 2), (95.0, 37.0, 150.0, 2), (55.8, 37.6, 150.0, -1),
           (55.8, 37.6, None, 2), (-55.8, 200.0, 1e9, 2), (80.0, -170.0, 150.0, 2)]

    def extra(t, r):
        if t < 5.0:
            return []                   # окно выставки — не про коррекцию
        out = []
        for i, (la, lo, al, st) in enumerate(bad):
            out += r.on_fix(t + 0.001 * i, ("master", "rover", "foo")[i % 3], la, lo, al, st)
        out += r.on_fix(float("nan"), "master", 55.8, 37.6, 150.0, 2)
        out += r.on_fix(t, "master", 55.8, 37.6, 150.0, 2, cov=float("nan"))
        return out

    r = runner()
    outs = feed(r, 0.0, 40.0, gnss=lambda t: True, extra=extra)
    assert all(math.isfinite(o["x"]) and math.isfinite(o["v"]) for o in outs)
    err = [abs(along_err(o, r)) for o in outs if o["stamp"] >= 10.0]
    assert max(err) < 1.5, max(err)


def test_queue_is_bounded_when_wheels_stop():
    """Тележки молчат, GNSS идёт: сетка стоит, очередь коррекции ограничена."""
    r = runner()
    feed(r, 0.0, 10.0)
    la, lo = _latlon(E0, N0)
    for k in range(3000):
        r.on_fix(r.stamp_max + 0.01, "master", la, lo, ALT_ANT, 2)
    assert len(r.pos._cq) <= r.pos.CORR_MAX_Q


def test_declared_covariance_is_used():
    """Заявленная ковариация NavSatFix (если есть) задаёт вес точки: с
    дисперсией 100 м² поправка на 3 м почти не тянет даже при статусе 2."""
    class Cov(Runner):
        def on_fix(self, *a, **k):
            k.setdefault("cov", 100.0)
            return super().on_fix(*a, **k)

    r = Cov(_tram(), track_map=_map())
    outs = feed(r, 0.0, 45.0, gnss=lambda t: t <= 2.0 or 40.0 <= t <= 40.35,
                fix=lambda t: (3.0, 0.0, 2, 0.0) if t > 3.0 else (0.0, 0.0, 2, 0.0))
    assert abs(along_err(outs[-1], r)) < 1.0


# ------------------------------------------------------------ ветка, встречный путь, вне карты

def _map_two_ways(L=3000.0, gap=4.0):
    """Путь на восток и встречный (на запад) в gap м севернее."""
    x = np.arange(-50.0, L, 1.0)
    east = np.c_[E0 + x, np.full(len(x), N0), np.full(len(x), ALT_RAIL)]
    west = np.c_[E0 + x[::-1], np.full(len(x), N0 + gap), np.full(len(x), ALT_RAIL)]
    return TrackMap.from_polylines([east, west], crs="utm", zone=37, bidirectional=False)


def _map_turn(x_turn=400.0, R_c=50.0, L_north=600.0):
    """Путь на восток до x_turn, затем поворот налево по дуге R_c и на север:
    трамвай на самом деле едет прямо (тупик/ветка, которой нет в карте)."""
    x = np.arange(-50.0, x_turn, 1.0)
    pts = [np.c_[E0 + x, np.full(len(x), N0)]]
    a = np.arange(0.0, math.pi / 2, 1.0 / R_c)
    pts.append(np.c_[E0 + x_turn + R_c * np.sin(a), N0 + R_c * (1 - np.cos(a))])
    y = np.arange(0.0, L_north, 1.0)
    pts.append(np.c_[np.full(len(y), E0 + x_turn + R_c), N0 + R_c + y])
    P = np.vstack(pts)
    return TrackMap.from_polylines([np.c_[P, np.full(len(P), ALT_RAIL)]], crs="utm", zone=37,
                                   bidirectional=False)


def test_cursor_on_opposite_track_is_relocated_by_rtk_pair():
    """Курсор оказался на встречном пути (курс против движения): чистая пара
    RTK смотрит против курса пути — вдоль не поправляем, а после
    gnss_persist_s согласных эпох курсор переставляется в точку GNSS на путь
    с курсом пары."""
    def run(correction):
        r = Runner(_tram(), track_map=_map_two_ways(), gnss_correction=correction)
        feed(r, 0.0, 20.0)
        c = r.pos._cursor
        r.pos._cursor = r.pos.map.locate((c["x"], c["y"] + 4.0, c["z"]), c["h"] + math.pi)
        assert abs(math.remainder(r.pos._cursor["h"] - c["h"], 2 * math.pi)) > 3.0
        return r, feed(r, 20.0, 50.0, gnss=lambda t: t >= 5.0)
    r, outs = run(True)
    assert r.pos.n_corr_reloc >= 1
    assert abs(along_err(outs[-1], r)) < 1.5 and abs(outs[-1]["y"] + 6100000.0 - N0) < 1.0
    r0, outs0 = run(False)
    assert abs(along_err(outs0[-1], r0)) > 100.0


def test_off_map_branch_is_followed_by_rtk_pair():
    """Карта сворачивает налево, а трамвай едет прямо (пути нет в карте):
    без GNSS курсор уходит по карте на север; с GNSS весь прогон пара RTK
    держится против курса курсора gnss_persist_s — курсор ставится в точку
    GNSS вне карты с курсом пары и едет прямо."""
    def run(correction):
        r = Runner(_tram(), track_map=_map_turn(), gnss_correction=correction)
        return r, feed(r, 0.0, 90.0, gnss=lambda t: True)
    r, outs = run(True)
    e = outs[-1]
    assert r.pos.n_corr_reloc >= 1
    assert math.hypot(along_err(e, r), e["y"] + 6100000.0 - N0) < 2.0
    r0, outs0 = run(False)
    e0 = outs0[-1]
    assert math.hypot(along_err(e0, r0), e0["y"] + 6100000.0 - N0) > 100.0


def test_short_rtk_offset_in_a_burst_is_not_followed():
    """Пачка RTK на 5 с со сдвигом 15 м вдоль пути (сбой GNSS, как
    30618_49fe4c54 на 320 с), а колёса точны: невязка вне ворот держится
    меньше gnss_persist_s — не верим; положение не уходит."""
    r = runner()
    outs = feed(r, 0.0, 75.0, gnss=lambda t: t <= 2.0 or 60.0 <= t <= 65.0,
                fix=lambda t: (15.0, 0.0, 2, 0.0) if t > 3.0 else (0.0, 0.0, 2, 0.0))
    err = [abs(along_err(o, r)) for o in outs if o["stamp"] >= 10.0]
    assert max(err) < 1.5, max(err)


def test_broken_pair_is_not_used():
    """База пары 40 м (одна антенна сбита, неизвестно какая): эпоха не
    берётся вовсе — ни парой, ни одной антенной (30618_616ec56b, 570 с)."""
    r = runner()
    # rover сбит на 28 м вперёд, master — на 4 м назад вдоль пути
    orig = r.on_fix

    def on_fix(stamp, antenna, lat, lon, alt, status=0, cov=None):
        if 40.0 <= stamp <= 50.0:
            e, n = g.utm_fwd(lat, lon, 37)
            e = float(e) + (28.0 if antenna == "rover" else -4.0)
            lat, lon = _latlon(e, float(n))
        return orig(stamp, antenna, lat, lon, alt, status, cov)
    r.on_fix = on_fix
    outs = feed(r, 0.0, 60.0, gnss=lambda t: True)
    err = [abs(along_err(o, r)) for o in outs if o["stamp"] >= 10.0]
    assert max(err) < 1.0, max(err)
    assert r.pos.n_corr_geom > 50


def test_dead_end_hold_is_released_by_gnss_track():
    """Карта кончается тупиком (удержание курсора в тупике, terminal_hold
    any), а трамвай едет дальше по пути, которого нет в карте. Невязка
    растёт со скоростью вагона, но точки GNSS движутся ровно так, как
    говорят колёса: после gnss_persist_s курсор ставится в точку GNSS вне
    карты и едет дальше."""
    def run(correction):
        x = np.arange(-50.0, 400.0, 1.0)
        tm = TrackMap.from_polylines(
            [np.c_[E0 + x, np.full(len(x), N0), np.full(len(x), ALT_RAIL)]],
            crs="utm", zone=37, bidirectional=False)
        r = Runner(_tram(), track_map=tm, terminal_hold="any", gnss_correction=correction)
        return r, feed(r, 0.0, 75.0, gnss=lambda t: True)
    r, outs = run(True)
    assert r.pos.n_corr_reloc >= 1
    assert abs(along_err(outs[-1], r)) < 2.0
    r0, outs0 = run(False)
    assert abs(along_err(outs0[-1], r0)) > 100.0


# ------------------------------------------------------------ ревью раунда 2

def _stop_map(x_stop, L=3000.0):
    """Прямой путь на восток с точкой остановки на x_stop (base_link)."""
    x = np.arange(-50.0, L, 1.0)
    P = np.c_[E0 + x, np.full(len(x), N0), np.full(len(x), ALT_RAIL)]
    la, lo = _latlon(E0 + x_stop, N0)
    return TrackMap.from_polylines([P], crs="utm", zone=37, bidirectional=False,
                                   stops=[(la, lo, math.pi / 2, 1.0)])


def test_weak_correction_keeps_stop_anchoring():
    """Колёса врут на +1,2 %; на 400 м пути пачка GNSS без RTK (статус 0):
    малая поправка (≈1 м из ~5 м невязки). На остановке у 1600 м ошибка
    ~18 м — привязка к остановке должна случиться, как и без коррекции:
    слабая поправка окно привязки не сужает (раньше отсчёт окна начинался
    от поправки: окно ±17 м, привязки нет, ошибка оставалась)."""
    x_stop = 1600.0
    prof, t_halt = stop_profile(x_stop)
    res = {}
    for flag in (False, True):
        r = Runner(_tram(), track_map=_stop_map(x_stop), gnss_correction=flag)
        outs = feed(r, 0.0, t_halt + 15.0, wheel_gain=1.012, prof=prof,
                    gnss=lambda t: t <= 2.0 or 48.0 <= t <= 54.0,
                    fix=lambda t: (0.0, 0.0, 0 if t > 3.0 else 2, 0.0))
        before = [o for o in outs if t_halt - 1.0 <= o["stamp"] <= t_halt]
        res[flag] = (r, outs, along_err(before[-1], r, prof[1]))
    r0, _, e0 = res[False]
    r1, outs1, e1 = res[True]
    assert e0 > 15.0 and e1 > 15.0, (e0, e1)          # до остановки — дрейф колёс
    assert r1.pos.n_corr >= 1 and abs(e0 - e1) > 0.3  # поправка без RTK была
    assert r0.pos.anchors == 1
    assert r1.pos.anchors == 1                         # и привязка — тоже
    assert abs(along_err(outs1[-1], r1, prof[1])) < 3.0


@pytest.mark.parametrize("status", [0, 1])
@pytest.mark.parametrize("bias", [6.0, 10.0, 15.0])
def test_nonrtk_bias_burst_is_not_followed(status, bias):
    """Пачка без RTK (статус 0 или 1) на 8 с со сдвигом 6–15 м вдоль пути
    через 2,4 км точных колёс: σ_s уже большая, невязка в воротах, но без
    RTK большая поправка не делается (смещение таких точек держится
    минутами) — ошибка как без коррекции (раньше: 5–15 м)."""
    fix = (lambda t: (bias, 0.0, status, 0.0) if t >= 250.0 else (0.0, 0.0, 2, 0.0))
    gnss = (lambda t: t <= 2.0 or 250.0 <= t <= 258.0)
    r0 = runner(gnss_correction=False)
    e0 = final_err(feed(r0, 0.0, 300.0, gnss=gnss, fix=fix), r0)
    r1 = runner()
    e1 = final_err(feed(r1, 0.0, 300.0, gnss=gnss, fix=fix), r1)
    assert abs(e1 - e0) < 0.5, (e0, e1)
    assert r1.pos.n_corr_big == 0 and r1.pos.n_corr_nonrtk > 0


def test_rover_only_epoch_takes_master_status():
    """Rover меряется от master (подвижная база): у rover статус 2, у master
    0 — эпоха только с rover не RTK. Пачка пар (master статус 0), через 5 с
    — пачка, где master пропал, а rover со сдвигом 10 м и статусом 2:
    поправки больше gnss_jump_m нет (раньше такая эпоха считалась RTK и
    после 3 эпох ставила положение на 10 м вперёд)."""
    r = runner()
    orig = r.on_fix

    def on_fix(stamp, antenna, lat, lon, alt, status=0, cov=None):
        if stamp > 3.5:
            if antenna == "master":
                if stamp >= 250.0:
                    return []                   # master пропал
                status = 0                      # master весь прогон без RTK
            elif stamp >= 250.0:
                e, n = g.utm_fwd(lat, lon, 37)
                lat, lon = _latlon(float(e) + 10.0, float(n))
        return orig(stamp, antenna, lat, lon, alt, status, cov)
    r.on_fix = on_fix
    gnss = (lambda t: t <= 2.0 or 240.0 <= t <= 245.0 or 250.0 <= t <= 258.0)
    outs = feed(r, 0.0, 300.0, gnss=gnss)
    r0 = runner(gnss_correction=False)
    e0 = final_err(feed(r0, 0.0, 300.0, gnss=gnss), r0)
    assert abs(final_err(outs, r) - e0) < 3.0
    assert r.pos.n_corr_big == 0


# ------------------------------------------------ онлайн-масштаб колёс (поток vehicle)

def _multi_stop_profile(stops, dwell=12.0):
    """Разгон до V, ход, торможение ACC до остановки ровно на каждой из
    stops (м пути) и стоянка dwell с -> (v(t), s(t), ручка(t), конец)."""
    ph = [(T_START, 0.0)]                          # (длительность, ускорение)
    x = 0.0
    da = V * V / (2.0 * ACC)
    for xs in stops:
        cruise = (xs - x - 2.0 * da) / V
        assert cruise > 0.0
        ph += [(V / ACC, ACC), (cruise, 0.0), (V / ACC, -ACC), (dwell, 0.0)]
        x = xs
    t0 = np.cumsum([0.0] + [d for d, _ in ph[:-1]])
    v0, s0 = [0.0], [0.0]
    for (d, a) in ph[:-1]:
        s0.append(s0[-1] + v0[-1] * d + 0.5 * a * d * d)
        v0.append(max(0.0, v0[-1] + a * d))

    def _k(t):
        return max(0, int(np.searchsorted(t0, t, side="right")) - 1)

    def v(t):
        k = _k(t)
        return max(0.0, v0[k] + ph[k][1] * (t - t0[k]))

    def s(t):
        k = _k(t)
        u = t - t0[k]
        return s0[k] + v0[k] * u + 0.5 * ph[k][1] * u * u

    def notch(t):
        a = ph[_k(t)][1]
        return 3 if a > 0 else (-3 if a < 0 else 0)

    return (v, s, notch), float(t0[-1] + ph[-1][0])


def test_online_wheel_scale_ignores_mid_route_gnss():
    """Онлайн-масштаб колёс (vehicle.OnlineWheelScale, wheel_scale_online)
    учится по привязкам к остановкам. Поправки GNSS двигают курсор, и без
    защиты сдвиг привязки после них уже не ошибка колёс, а скорость начинает
    зависеть от GNSS. Колёса врут на +1,2 %, три остановки через 700 м:
    при GNSS только в окне масштаб включается (≈ 0,988); при GNSS весь
    прогон (RTK) положение поправляется, а скорость на каждом шаге та же,
    что при GNSS в окне, — масштаб учится по копии положения без поправок."""
    from tram_state_estimator import vehicle as VH
    stops = [700.0, 1400.0, 2100.0]
    prof, t_end = _multi_stop_profile(stops)
    x = np.arange(-50.0, 3000.0, 1.0)
    P = np.c_[E0 + x, np.full(len(x), N0), np.full(len(x), ALT_RAIL)]
    st = [(*_latlon(E0 + xs, N0), math.pi / 2, 1.0) for xs in stops]
    res = {}
    # «first3» — ровно точки окна выставки (±3 с от первой: эпохи до 3,013 с),
    # «full» — те же и все после окна
    for name, gnss in (("first3", lambda t: t <= 3.05), ("full", lambda t: True)):
        tm = TrackMap.from_polylines([P], crs="utm", zone=37, bidirectional=False, stops=st)
        r = VH.wheel_scale_hook(Runner(_tram(), track_map=tm), True)
        res[name] = (r, feed(r, 0.0, t_end, wheel_gain=1.012, prof=prof, gnss=gnss))
    (r3, o3), (rf, of) = res["first3"], res["full"]
    assert r3.pos.anchors >= 2 and r3._ws_pos is None      # без GNSS после окна копии нет
    assert r3.wheel_scale.k < 0.995                        # масштаб колёс включился
    assert rf.pos.n_corr > 0 and rf._ws_pos is not None
    assert max(abs(a["x"] - b["x"]) for a, b in zip(o3, of)) > 1.0   # положение — за GNSS
    assert [o["stamp"] for o in o3] == [o["stamp"] for o in of]
    assert [o["v"] for o in o3] == [o["v"] for o in of]              # скорость — нет
    assert rf.wheel_scale.k == r3.wheel_scale.k
    # и после сброса (новый прогон) копии нет, пока нет GNSS после окна
    rf.reset("тест")
    assert rf._ws_pos is None
    f = rf.fork()
    assert f._ws_pos is None
