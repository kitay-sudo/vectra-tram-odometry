"""Аудит JS-порта: эталонная запись по ТЕКУЩЕМУ Python-ядру.

Аналог js-port/record.py, но:
  * не пишет в js-port/ (вывод в out/sim/ или путь из argv[1]);
  * пишет больше полей (sigma_s, k_b, ambiguous, n_acc, n_rej, valid, истина s);
  * параметры ядра — DEFAULT (заглушки), как в симуляторе и старом record.py.

Запуск (Docker, из корня репозитория):
  docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev \
      bash -c "python3 tools/dev/record_current.py out/sim/rec_current.json"

Формат строки: [notch, meas[8], v, s, mode, sigma_v, k_t, mu, d,
                v_true, s_true, sigma_s, k_b, amb, n_acc, n_rej, valid]
Первые 10 полей совпадают по смыслу с js-port/record.py (compare.js читает их).
"""

import json
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "prototype"))

import scenarios as S  # noqa: E402  (prototype/scenarios.py -> шимы на ros2_ws)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, "out", "sim", "rec_current.json")
    only = set(sys.argv[2:])
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    out, timing = {}, {}
    for name, kw in S.SCENARIOS:
        if only and name not in only:
            continue
        pl = S.Plant(kw["track"], dt=S.DT, seed=1)
        es = S.Estimator()
        if kw.get("faults"):
            pl.faults = kw["faults"]
        sand = kw.get("sand", lambda t: False)
        tb = kw.get("tbrake", lambda t: False)
        rec, t_step = [], 0.0
        for k in range(int(kw["T_end"] / S.DT)):
            t = k * S.DT
            notch = kw["driver"](t, pl.v)
            pl.step(notch, sand=sand(t), track_brake=tb(t))
            m = pl.measure()
            if k % S.SUB:
                continue
            t0 = time.perf_counter()
            o = es.step(notch, m)
            t_step += time.perf_counter() - t0
            rec.append([notch, [float(x) for x in m], o["v"], o["s"],
                        int(o["mode"]), o["sigma_v"], o["k_t"], o["mu"], o["d"],
                        float(pl.v), float(pl.s), o["sigma_s"], o["k_b"],
                        int(bool(o["ambiguous"])), int(o["n_accepted"]),
                        int(o["n_rejected"]), int(bool(o["valid"]))])
        out[name] = rec
        timing[name] = 1e6 * t_step / max(len(rec), 1)
        print(f"{name:<28} {len(rec):6d} шагов, {timing[name]:7.1f} мкс/шаг (Python)",
              flush=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f)
    with open(out_path.replace(".json", "_timing.json"), "w",
              encoding="utf-8") as f:
        json.dump(timing, f, ensure_ascii=False, indent=1)
    print("ok ->", out_path)


if __name__ == "__main__":
    main()
