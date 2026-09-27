"""Нода tram_estimator без DDS-трафика: пульс, подавление повторов,
темп проигрывания, сброс, ковариации, лист пакета.

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


def run_node(ev, monkeypatch, rate=1.0, pause=None, tail=3.0, delay=None,
             horizon=None, log=None):
    """Прогон ноды; возвращает массивы (метка, v, стенное время, от пульса).
    delay(t, метод) — добавочное опоздание прихода события, с; horizon —
    pulse_horizon_s вместо умолчания ноды; log (dict) — сюда пишутся входы
    тележек и ручки (номер события, стенное время, метка), номера событий
    выходов и горизонт: для задержки in2out, как у tools/ros_probe.py."""
    ft = FakeTime()
    monkeypatch.setattr(TN, "time", ft)
    node = TN.TramEstimatorNode()
    node.runner = Runner(trt.P)
    node.runner.pos.init_window = 3.0
    if horizon is not None:
        node.pulse_h = horizon
    pubs, state, seq, ins = [], {"pulse": False}, [0], []
    orig = node._extrapolate

    def extrap(now):
        state["pulse"] = True
        try:
            orig(now)
        finally:
            state["pulse"] = False

    def publish(o, us):
        seq[0] += 1
        pubs.append((o["stamp"], o["v"], ft.now, state["pulse"], seq[0]))

    node._extrapolate = extrap
    node._publish = publish
    t_start, k, tw = ft.now, 0, 0.0

    def arrive(e):
        return (e[0] / rate + (pause[1] if pause and e[0] > pause[0] else 0.0)
                + (delay(e[0], e[1]) if delay else 0.0))

    evs = sorted(ev, key=arrive)
    t_end = arrive(evs[-1]) + tail
    while tw < t_end:
        ft.now = t_start + tw
        while k < len(evs) and arrive(evs[k]) <= tw:
            _, m, a = evs[k]
            k += 1
            if m in ("on_wheel", "on_handle"):
                seq[0] += 1
                ins.append((seq[0], tw, a[1] if m == "on_wheel" else a[0]))
            node._input(m, a)
        node._pulse()
        tw += 0.01
    summary = node.summary()
    if log is not None:
        log.update(inputs=np.array(ins), seq=np.array([p[4] for p in pubs]),
                   horizon=node.pulse_h)
    node.destroy_node()
    T, V, W, Pu = (np.array([p[i] for p in pubs]) for i in range(4))
    return T, V, W - t_start, Pu.astype(bool), summary


def in2out(log, T, W):
    """Задержка «вход → выход», с (как in2out у tools/ros_probe.py): от прихода
    входа тележки или ручки до первого выхода после него с меткой не раньше
    метки входа."""
    I = log["inputs"]
    k1 = np.searchsorted(log["seq"], I[:, 0], side="right")
    k2 = np.searchsorted(np.maximum.accumulate(T), I[:, 2] - 1e-6, side="left")
    k = np.maximum(k1, k2)
    ok = k < len(T)
    return W[k[ok]] - I[ok, 1]


def record_pause(t_p=20.0, gap=0.9, catch=0.11):
    """Пауза записи, как в 30618_af7496f0 на t+142 с: приход тележек и ручки
    после t_p прерывается на gap с, метки без разрыва; дальше входы идут с
    опозданием, которое убывает на catch с за секунду (запись догоняет:
    0,045 с прихода на 0,05 с меток). GNSS не задерживается."""
    def delay(t, m):
        if m not in ("on_wheel", "on_handle") or t <= t_p:
            return 0.0
        return max(0.0, gap - catch * (t - t_p))
    return delay


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


@pytest.mark.parametrize("horizon", [None, 2.0])
def test_player_pause_forecast_within_horizon_and_monotonic(ros, monkeypatch, horizon):
    """Пауза плеера 7,6 с: прогноз пульса — узлы сетки не дальше
    pulse_horizon_s от последней метки входа (умолчание ноды и прежние 2 с),
    с шагом ≤ 0,13 с по стенным часам; метки не идут назад после паузы."""
    ev = trt.stream(60.0, lag=0.05)
    log = {}
    T, V, W, Pu, _ = run_node(ev, monkeypatch, pause=(20.0, 7.6),
                              horizon=horizon, log=log)
    h = log["horizon"]
    assert np.all(np.diff(T) > 0)
    in_pause = (W > 20.0) & (W < 27.6)
    last_in = log["inputs"][log["inputs"][:, 1] <= 20.0, 2].max()
    assert np.all(T[in_pause] <= last_in + h + 1e-6)
    n = int(round(h / trt.P.dt))
    assert n - 1 <= Pu[in_pause].sum() <= n + 1
    Wp = W[in_pause]
    assert Wp.max() < 20.0 + h + 0.2 and np.diff(Wp).max() < 0.13
    assert dv_offline(ev, T, V).max() < 0.01


def test_record_pause_latency_bounded_by_horizon(ros, monkeypatch):
    """Запись с паузой входов (как 30618_af7496f0): за паузу пульс занимает
    узлы сетки прогнозом, вернувшиеся с опозданием входы с этими метками ждут
    первого нового узла. С горизонтом 2 с это до ~0,7 с (пик 767,8 мс в ROS до
    правки); с горизонтом по умолчанию задержка не больше горизонта с запасом
    на шаг сетки и остаётся меньше 250 мс. Выход без пауз и повторов меток."""
    ev = trt.stream(40.0, lag=0.05)
    lat, hor = {}, {}
    for h in (2.0, None):
        log = {}
        T, V, W, Pu, _ = run_node(ev, monkeypatch, delay=record_pause(),
                                  horizon=h, log=log)
        assert np.all(np.diff(T) > 0)
        assert dv_offline(ev, T, V).max() < 0.01
        lat[h], hor[h] = in2out(log, T, W), log["horizon"]
    assert hor[None] <= 0.25
    assert lat[2.0].max() > 0.5
    assert lat[None].max() <= hor[None] + 0.1 and lat[None].max() < 0.25
    # вне паузы то же, что с прежним горизонтом
    assert np.median(lat[None]) == pytest.approx(np.median(lat[2.0]), abs=0.011)


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
    got, vel = [], []
    node.pub_p.publish = got.append
    node.pub_v.publish = vel.append
    ev = trt.stream(10.0)
    for t, m, a in ev:
        ft.now = 1000.0 + t
        node._input(m, a)
    node.destroy_node()
    assert len(got) > 150
    # до якоря GNSS положение не публикуется (pos_valid),
    # поэтому все опубликованные — с якорем: σ x, y конечна и мала
    assert len(got) < len(vel)
    assert all(o.pose.covariance[0] < 1e5 for o in got)
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
