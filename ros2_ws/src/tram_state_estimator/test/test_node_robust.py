"""Нода tram_estimator без DDS-трафика: пульс (WP16), подавление повторов,
темп проигрывания, сброс, ковариации (WP12a), лист пакета (WP22).

Входы подаются прямо в node._input по времени прихода, таймер пульса
вызывается каждые 10 мс поддельных монотонных часов (модуль time ноды
подменён), публикация перехвачена. Связка — Runner на листе тестов
test_runner_robust (config/tram.yaml пакета), без карты.
"""

import math
import os
import re

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("tram_msgs.msg")
pytest.importorskip("tram_vehicle_msgs.msg")

from rclpy.parameter import Parameter                           # noqa: E402

import test_runner_robust as trt                                # noqa: E402
from tram_state_estimator import tram_node as TN                # noqa: E402
from tram_state_estimator.runner import Runner                  # noqa: E402

SHEET = os.path.join(trt.PKG, "config", "tram.yaml")


class FakeTime:
    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def perf_counter_ns(self):
        return int(self.now * 1e9)


@pytest.fixture(scope="module")
def ros():
    mine = False
    try:
        rclpy.init(args=["--ros-args", "-p", "sheet:=none"])
        mine = True
    except RuntimeError:            # контекст уже поднят другим тестом
        pass
    yield
    if mine:
        rclpy.try_shutdown()


def offline(ev):
    r = Runner(trt.P)
    out = {}
    for _, m, a in ev:
        for o in getattr(r, m)(*a):
            out[round(o["stamp"], 4)] = o["v"]
    return out


def run_node(ev, monkeypatch, rate=1.0, pause=None, tail=3.0):
    """Прогон ноды; возвращает массивы (метка, v, стенное время, от пульса)."""
    ft = FakeTime()
    monkeypatch.setattr(TN, "time", ft)
    node = TN.TramEstimatorNode()
    node.runner = Runner(trt.P)
    node.runner.pos.init_window = 3.0
    pubs, state = [], {"pulse": False}
    orig = node._extrapolate

    def extrap(now):
        state["pulse"] = True
        try:
            orig(now)
        finally:
            state["pulse"] = False

    node._extrapolate = extrap
    node._publish = lambda o, us: pubs.append((o["stamp"], o["v"], ft.now,
                                               state["pulse"]))
    t_start, k, tw = ft.now, 0, 0.0
    evs = sorted(ev, key=lambda e: e[0])

    def arrive(e):
        return e[0] / rate + (pause[1] if pause and e[0] > pause[0] else 0.0)

    t_end = arrive(evs[-1]) + tail
    while tw < t_end:
        ft.now = t_start + tw
        while k < len(evs) and arrive(evs[k]) <= tw:
            _, m, a = evs[k]
            k += 1
            node._input(m, a)
        node._pulse()
        tw += 0.01
    summary = node.summary()
    node.destroy_node()
    T, V, W, Pu = (np.array([p[i] for p in pubs]) for i in range(4))
    return T, V, W - t_start, Pu.astype(bool), summary


def dv_offline(ev, T, V):
    off = offline(ev)
    return np.array([abs(v - off[round(t, 4)]) for t, v in zip(T, V)
                     if round(t, 4) in off])


def test_no_handle_1x_pulse_fills_between_bogie_messages(ros, monkeypatch):
    """Без ручки сетку двигают только тележки ~10 Гц: пульс выдаёт узлы
    между ними. Разрывов по стенным часам > 0,1 с нет, значения = офлайн."""
    ev = trt.stream(60.0, handle=False, lag=0.05)
    T, V, W, Pu, _ = run_node(ev, monkeypatch)
    assert np.all(np.diff(T) > 0)
    assert Pu.sum() > 200
    assert np.diff(W).max() <= 0.1 + 1e-9
    assert dv_offline(ev, T, V).max() < 1e-6


def test_slow_playback_does_not_turn_outputs_into_forecasts(ros, monkeypatch):
    """play -r 0.5: темп оценивается по приросту меток, пульс не опережает
    входы (раньше 1176 из 1236 выходов были прогнозом, |dv| до 0,07)."""
    ev = trt.stream(60.0, handle=False, lag=0.05)
    T, V, W, Pu, summ = run_node(ev, monkeypatch, rate=0.5, tail=6.0)
    assert np.all(np.diff(T) > 0)
    during = T <= max(a[1] for _, m, a in ev if m == "on_wheel")
    assert Pu[during].mean() < 0.6, summ
    assert dv_offline(ev, T, V).max() < 0.02
    rate = float(re.search(r"темп ([0-9.]+)", summ).group(1))
    assert 0.45 <= rate <= 0.55


def test_player_pause_forecast_within_horizon_and_monotonic(ros, monkeypatch):
    """Пауза плеера 7,6 с: первые pulse_horizon_s заполнены прогнозом с
    шагом ≤ 0,13 с по стенным часам; метки не идут назад после паузы."""
    ev = trt.stream(60.0, lag=0.05)
    T, V, W, Pu, _ = run_node(ev, monkeypatch, pause=(20.0, 7.6))
    assert np.all(np.diff(T) > 0)
    in_pause = (W > 20.0) & (W < 27.6)
    assert Pu[in_pause].sum() >= 39
    Wp = W[(W > 20.0) & (W < 22.0)]
    assert len(Wp) >= 30 and np.diff(Wp).max() < 0.13
    assert dv_offline(ev, T, V).max() < 0.01


def test_second_bag_is_published_after_reset(ros, monkeypatch):
    """Второй bag с метками на 139 с раньше: сброс, выходы второго bag
    публикуются (подавление повторов обнулено), внутри каждого — по порядку."""
    ev1 = trt.stream(20.0)
    ev2 = [(t + 21.0, m, a) for t, m, a in trt._bag2(-139.0 - 20.0)
           if t < 20.0]
    T, V, W, Pu, summ = run_node(ev1 + ev2, monkeypatch)
    split = int(np.argmax(np.diff(T) < 0)) + 1
    assert split > 1 and "сбросов связки 1" in summ
    assert np.all(np.diff(T[:split]) > 0) and np.all(np.diff(T[split:]) > 0)
    n2 = len(offline(ev2))
    assert len(T) - split >= n2 - 2


def test_odometry_covariances_have_no_zero_diagonal(ros, monkeypatch):
    ft = FakeTime()
    monkeypatch.setattr(TN, "time", ft)
    node = TN.TramEstimatorNode()
    node.runner = Runner(trt.P)
    got = []
    node.pub_p.publish = got.append
    ev = trt.stream(10.0)
    for t, m, a in ev:
        ft.now = 1000.0 + t
        node._input(m, a)
    node.destroy_node()
    assert len(got) > 150
    ready = [o for o in got if o.pose.covariance[0] < 1e5]
    assert ready and len(ready) < len(got)      # и до выставки, и после
    for od in got:
        for cov in (od.pose.covariance, od.twist.covariance):
            d = [cov[i] for i in (0, 7, 14, 21, 28, 35)]
            assert all(math.isfinite(x) and x > 0.0 for x in d)


def test_fallback_alignment_widens_pose_covariance(ros, monkeypatch):
    """После сброса, пока новый прогон не выставился, положение идёт по
    запасной выставке: σ положения не меньше FALLBACK_SD."""
    ft = FakeTime()
    monkeypatch.setattr(TN, "time", ft)
    node = TN.TramEstimatorNode()
    node.runner = Runner(trt.P)
    got = []
    node.pub_p.publish = got.append
    ev1 = trt.stream(20.0)
    ev2 = [(t + 21.0, m, a) for t, m, a in trt._bag2(-159.0) if t < 5.0]
    for t, m, a in ev1 + ev2:
        ft.now = 1000.0 + t
        node._input(m, a)
    node.destroy_node()
    T = np.array([to_s(od.header.stamp) for od in got])
    k = int(np.argmax(np.diff(T) < 0)) + 1      # первый выход второго bag
    fb = [od.pose.covariance[0] for od in got[k:k + 3]]
    assert all(c >= TN.FALLBACK_SD ** 2 for c in fb)
    assert got[-1].pose.covariance[0] < TN.FALLBACK_SD ** 2   # выставился


def to_s(st):
    return st.sec + st.nanosec * 1e-9


# ------------------------------------------------------------ лист пакета

class _Log:
    def warn(self, *a, **k):
        pass


class StubNode:
    def __init__(self, ov, name="tram_state_estimator", ns="/"):
        self._parameter_overrides = ov
        self._name, self._ns = name, ns

    def get_name(self):
        return self._name

    def get_fully_qualified_name(self):
        return (self._ns.rstrip("/") + "/" + self._name)

    def get_logger(self):
        return _Log()


def _ov(**kw):
    return {k: Parameter(k, value=v) for k, v in kw.items()}


def test_package_sheet_without_params_file():
    ov = _ov(sheet=SHEET)
    msg = TN.use_package_sheet(StubNode(ov))
    assert "без --params-file" in msg
    assert ov["meas_units"].value == "km_h" and ov["map_file"].value


def test_package_sheet_stays_under_single_override():
    """Один -p параметра ядра больше не превращает остальные в заглушки."""
    ov = _ov(sheet=SHEET, q_v=0.123)
    msg = TN.use_package_sheet(StubNode(ov))
    assert "под параметрами запуска" in msg
    assert ov["q_v"].value == 0.123 and ov["meas_units"].value == "km_h"


def test_package_sheet_found_for_renamed_node():
    ov = _ov(sheet=SHEET)
    msg = TN.use_package_sheet(StubNode(ov, name="est2", ns="/tram"))
    assert "/tram_state_estimator" in msg and ov["meas_units"].value == "km_h"


def test_package_sheet_none_keeps_stubs():
    ov = _ov(sheet="none")
    assert "заглушки" in TN.use_package_sheet(StubNode(ov))
    assert set(ov) == {"sheet"}
