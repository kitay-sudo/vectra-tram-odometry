"""Взаимодействия механизмов ядра и связки, которых нет в тестах отдельных
механизмов."""

import numpy as np
import pytest

from tram_state_estimator.estimator_core import (DEFAULT, STANDSTILL, Estimator,
                                                 sensor_to_speed)


def _drive(dt_zero, v=4.0, n_move=200, dt_msg=0.1):
    """Вагон едет v м/с с показаниями тележек раз в dt_msg; затем обе
    тележки разом показывают 0 (отказ датчиков), первое нулевое показание —
    через dt_zero после последнего ненулевого. Шаг ядра 0,05 с."""
    p = DEFAULT.__class__(**{**DEFAULT.__dict__, "dt": 0.05})
    e = Estimator(p)
    per = e.nw
    t, t_msg = 0.0, 0.0
    outs = []
    # показание в единицах листа: обратный перевод через sensor_to_speed
    k = float(sensor_to_speed(1.0, p))
    z_move = v / k
    for _ in range(n_move):
        t += p.dt
        fresh = t - t_msg >= dt_msg - 1e-9
        if fresh:
            t_msg = t
        o = e.step(0.0, np.full(per, z_move), fresh=fresh)
    # пауза между последним ненулевым и первым нулевым показанием
    steps = int(round(dt_zero / p.dt))
    for _ in range(steps - 1):
        o = e.step(0.0, np.full(per, z_move), fresh=False)
    for i in range(40):                       # 2 с нулей
        o = e.step(0.0, np.zeros(per), fresh=(i % 2 == 0))
        outs.append(o)
    return outs


@pytest.mark.parametrize("dt_zero", [0.1, 0.15, 0.2, 0.25])
def test_both_bogies_drop_to_zero_is_not_a_stop_at_any_message_interval(dt_zero):
    """Обе тележки разом упали с 4 м/с в ноль на выбеге: это отказ датчиков
    (или блокировка), а не остановка — физически невозможное замедление.
    Сглаженная производная оси пересекала порог срыва только при интервале
    между показаниями < ~0,17 с: при 0,2 с (10 Гц с пропуском, фаза сетки)
    нули принимались «по согласию осей», v падала в 0 с
    valid = true (инъекция both_zero на 30639_d3c43d69: MAE 0,9 -> 9,4 м/с).
    Поэтому скачок больше физически возможного за один интервал ловится
    сразу: неоднозначность, скорость держит модель."""
    outs = _drive(dt_zero)
    assert all(o["ambiguous"] for o in outs[2:])
    assert all(o["v"] > 3.0 for o in outs)
    assert not any(o["mode"] == STANDSTILL and o["valid"] for o in outs)
