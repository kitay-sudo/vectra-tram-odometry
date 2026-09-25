"""Замер времени шага оценщика.

Для предохранительной функции медиана не значит ничего: требование
формулируется через худший случай и хвост распределения.
"""

import time
import numpy as np
from estimator import Estimator, EP

N_WARM = 2000
N = 60000          # 10 минут работы на 100 Гц


def bench():
    es = Estimator()
    rng = np.random.default_rng(0)
    # правдоподобный вход: разгон, выбег, торможение по кругу
    notches = np.concatenate([np.full(2000, 4.0), np.full(1500, 0.0),
                              np.full(1500, -3.0)])
    base = np.linspace(0.0, 37.0, N)
    meas = np.clip(base[:, None] + rng.normal(0, 0.05, (N, 8)), 0, None)

    for k in range(N_WARM):
        es.step(float(notches[k % len(notches)]), meas[k % N])

    es = Estimator()
    t = np.empty(N)
    for k in range(N):
        n = float(notches[k % len(notches)])
        m = meas[k]
        t0 = time.perf_counter_ns()
        es.step(n, m)
        t[k] = time.perf_counter_ns() - t0
    return t / 1000.0     # микросекунды


t = bench()
q = np.percentile(t, [50, 90, 99, 99.9])
print(f"шагов: {len(t)}  (эквивалент {len(t) * EP.dt / 60:.0f} мин на 100 Гц)")
print(f"{'медиана':>12}{'p90':>10}{'p99':>10}{'p99.9':>10}{'максимум':>11}")
print(f"{q[0]:>12.1f}{q[1]:>10.1f}{q[2]:>10.1f}{q[3]:>10.1f}{t.max():>11.1f}   мкс")
budget = EP.dt * 1e6
print(f"\nбюджет цикла при {1/EP.dt:.0f} Гц: {budget:.0f} мкс")
print(f"запас по максимуму: {budget / t.max():.0f}x, по p99.9: {budget / q[3]:.0f}x")
