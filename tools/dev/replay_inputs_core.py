"""Аудит: входы оценщика, записанные в песочнице симулятора
(tools/dev/sim_headless.js с REC_INPUTS=1), прогоняются через ТЕКУЩЕЕ
Python-ядро с параметрами по умолчанию (как в песочнице). Ответ на вопрос:
починит ли перенос свежего ядра в JS то, что ломается в песочнице.

    docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev \
        bash -c "python3 tools/dev/replay_inputs_core.py out/sim/sim_inputs_seed3.json"

Строка входа: [notch, meas[8] (рад/с), v_true (м/с), s_true (м)], шаг 0,01 с.
Замечание: страница при «починке датчиков» вручную сбрасывает healthy и
раздувает P[v,v] у JS-порта (index.html:1099-1101); здесь этого нет -
текущее ядро возвращает датчики само (t_recover).
"""

import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "tram_state_estimator"))
from tram_state_estimator.estimator_core import Estimator, MODE_NAMES  # noqa: E402


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, "out", "sim", "sim_inputs_seed3.json")
    rows = json.load(open(path, encoding="utf-8"))
    es = Estimator()
    out = []
    for n, m, vt, st in rows:
        o = es.step(n, np.asarray(m))
        out.append((o["v"], vt, o["sigma_v"], o["mode"], o["valid"],
                    int(o["healthy"].sum()), o["ambiguous"], o["s"], st))
    A = np.array([(a, b, c, d, e, f, g, h, i) for a, b, c, d, e, f, g, h, i in out],
                 dtype=float)
    v, vt, sv = A[:, 0], A[:, 1], A[:, 2]
    t = np.arange(len(A)) * 0.01
    err = v - vt
    print(f"шагов {len(A)}, {t[-1]:.0f} с модели")
    print(f"RMSE v, км/ч: {3.6 * np.sqrt(np.mean(err ** 2)):.2f}; "
          f"макс |ош|, км/ч: {3.6 * np.max(np.abs(err)):.1f}; "
          f"истина в ±2σ: {100 * np.mean(np.abs(err) <= 2 * sv):.1f} %")
    print("  t, с | v ист | v ядро | σ | режим | valid | исправных | amb   (км/ч)")
    marks = list(range(0, int(t[-1]), 100)) + list(range(1340, 1720, 20))
    for tm in sorted(set(marks)):
        i = min(int(tm / 0.01), len(A) - 1)
        print(f"{tm:7.0f} | {3.6 * vt[i]:6.1f} | {3.6 * v[i]:6.1f} | {3.6 * sv[i]:5.2f} | "
              f"{MODE_NAMES[int(A[i, 3])]:<10} | {int(A[i, 4])} | {int(A[i, 5])} | {int(A[i, 6])}")
    late = t > 1600
    print(f"после 1600 с: средняя v ист {3.6 * vt[late].mean():.2f} км/ч, "
          f"средняя v ядра {3.6 * v[late].mean():.2f} км/ч, "
          f"доля valid {100 * A[late, 4].mean():.0f} %, "
          f"режимы: { {MODE_NAMES[k]: round(100 * float(np.mean(A[late, 3] == k)), 1) for k in range(7)} }")
    json.dump({"rmse_kmh": float(3.6 * np.sqrt(np.mean(err ** 2))),
               "in_2sigma_pct": float(100 * np.mean(np.abs(err) <= 2 * sv)),
               "late_v_true_kmh": float(3.6 * vt[late].mean()),
               "late_v_core_kmh": float(3.6 * v[late].mean())},
              open(path.replace(".json", "_core_result.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
