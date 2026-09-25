"""Опознавание привода при вводе в эксплуатацию — прогон на имитаторе.

Процедура ведёт вагон по специальному профилю (разные позиции ручки с
выбегами между ними, ниже предела сцепления), собирает окна уравнения корпуса и
решает задачу наименьших квадратов на четыре параметра: тяга, торможение и две
составляющие сопротивления. Приёмка — по относительной погрешности каждого
параметра; при недостаточном возбуждении процедура отказывается, а не выдаёт
смещённые числа.

    python identify.py
"""

import os
import re
import sys

import numpy as np

_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "ros2_ws", "src", "tram_state_estimator")
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from tram_state_estimator.identification import (      # noqa: E402
    DriveIdentifier, Insufficient, identification_profile)
from plant import Plant, P                              # noqa: E402
from estimator import Estimator, EP                     # noqa: E402
from scenarios import flat, accel_cruise_brake, DT, SUB  # noqa: E402


def collect(driver, T_end, seed=0):
    """Гоняет модель и кормит идентификатор тем же, что видит оценщик."""
    pl, es = Plant(flat, dt=DT, seed=seed), Estimator()
    ident = DriveIdentifier(mass=EP.M_nom, params=EP)
    for k in range(int(T_end / DT)):
        n = driver(k * DT)
        pl.step(n)
        m = pl.measure()
        if k % SUB:
            continue
        o = es.step(n, m)
        v_meas = float(np.mean(m)) * EP.r_nom
        # опознавание ведётся только при подтверждённом сцеплении
        good = (not o["slip"]) and o["n_accepted"] >= 2 and o["valid"]
        ident.add(es.u_filt, v_meas, adhesion_ok=good)
    return ident


def truth_res_B():
    """Линейное сопротивление объекта: заданное плюс вращательное сопротивление
    всех колёс, приведённое к поступательному движению."""
    return P.res_B + 2 * P.n_axles * P.B_wheel / P.r ** 2


def report(title, ident):
    print("=" * 66)
    print(title)
    try:
        w_t, w_b, rep = ident.solve()
        ok = True
    except Insufficient as e:
        m = re.search(r"\{.*\}$", str(e))
        rep = eval(m.group(0)) if m else None
        ok = False
        print("  ОТКАЗ приёмки")
    if rep is None:
        print("  нет данных")
        return
    print(f"  окон {rep['окон']}, обусловленность "
          f"{rep['число_обусловленности']:.0f}, "
          f"невязка модели {100 * rep['относительная_невязка']:.0f} %")
    print(f"  {'параметр':<12}{'оценка':>12}{'погрешность':>14}")
    for nm, (val, rel) in rep["параметры"].items():
        print(f"  {nm:<12}{val:>12.0f}{100 * rel:>13.0f} %")
    print(f"  res_B объекта (истина): {truth_res_B():.0f}")
    print(f"  паспорт листа: F_notch {EP.F_notch:.0f} Н, F_brake {EP.F_brake:.0f} Н, "
          f"res_B {EP.res_B:.0f}")
    print("  ПРИНЯТО" if ok else "  НЕ ПРИНЯТО")


if __name__ == "__main__":
    report("ОБЫЧНОЕ ДВИЖЕНИЕ (разгон-выбег-торможение, 110 с)",
           collect(lambda t: accel_cruise_brake(t, 0.0), 110.0))
    report("РЕЖИМ ОПОЗНАВАНИЯ (специальный профиль, 220 с)",
           collect(identification_profile, 220.0))
