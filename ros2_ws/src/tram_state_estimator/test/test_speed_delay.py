"""Сдвиг скорости в выходе ноды: с меткой t выдаётся скорость на t - speed_output_delay_s,
положение не сдвигается. Эталон скорости проверки организаторов сглажен и запаздывает."""

from collections import deque
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from tram_state_estimator.tram_node import TramEstimatorNode  # noqa: E402

delayed = TramEstimatorNode._delayed_speed


def _node(delay):
    return SimpleNamespace(speed_delay=delay, _v_hist=deque(maxlen=200))


def test_zero_delay_returns_current_speed():
    n = _node(0.0)
    for k in range(10):
        assert delayed(n, 0.05 * k, 3.0 * k) == 3.0 * k


def test_ramp_is_shifted_by_delay():
    n = _node(0.08)
    out = [delayed(n, 0.05 * k, 1.0 * 0.05 * k) for k in range(41)]
    assert out[0] == 0.0
    assert out[-1] == pytest.approx(2.0 - 0.08, abs=1e-9)


def test_backward_stamp_starts_history_again():
    n = _node(0.08)
    for k in range(20):
        delayed(n, 100.0 + 0.05 * k, 5.0)
    assert delayed(n, 10.0, 1.0) == 1.0
    assert len(n._v_hist) == 1
