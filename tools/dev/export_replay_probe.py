"""Аудит симулятора: пробный экспорт прогона для режима «Прогон данных комиссии».

Не финальный инструмент, а замер: сколько весит компактный JSON поездки,
если страница проигрывает тележки, ручку, GNSS-эталон, оценку модели
(та же связка Runner, что в ноде и в analysis/evaluate.py) и наивное колесо.

    docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev \
        bash -c "python3 tools/dev/export_replay_probe.py [bag_id]"

Без bag_id берётся прогон с длительностью, ближайшей к 20 мин, из кэша
analysis/cache. Вывод: out/sim/replay_<bag>_{20hz,10hz}.json (+ .gz) и сводка.
Время отображения — сетка шага ядра (p.dt), входы — последнее значение на
момент шага (без заглядывания вперёд), GNSS — ближайший в пределах 0,05 с.
"""

import gzip
import json
import math
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "analysis"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "tram_state_estimator"))

import numpy as np  # noqa: E402

import bagio  # noqa: E402
import evaluate as ev  # noqa: E402
from tram_state_estimator.runner import Enu  # noqa: E402

KMH = 3.6
OUT = os.path.join(ROOT, "out", "sim")


def pick_bag():
    best = None
    for b in bagio.bag_ids():
        a = bagio.load(b)
        if len(a["mfix"]) < 100 or len(a["front"]) < 100:
            continue
        dur = a["front"][-1, 1] - a["front"][0, 1]
        d = abs(dur - 1200.0)
        if best is None or d < best[0]:
            best = (d, b, dur)
    return best[1]


def hold(t_src, v_src, t_q):
    """Последнее значение источника на момент t_q (без будущего)."""
    j = np.searchsorted(t_src, t_q, side="right") - 1
    out = np.where(j >= 0, v_src[np.clip(j, 0, None)], np.nan)
    return out


def q(arr, scale):
    """Квантование в целые; NaN -> None."""
    return [None if not math.isfinite(x) else int(round(x * scale)) for x in arr]


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else pick_bag()
    os.makedirs(OUT, exist_ok=True)
    t0 = time.perf_counter()
    a, outs = ev.run_bag(bag, tmap=ev.track_map())
    t_run = time.perf_counter() - t0
    T = np.array([o["stamp"] for o in outs])
    f, r, c = a["front"], a["rear"], a["cmd"]
    front = hold(f[:, 1], f[:, 2], T)
    rear = hold(r[:, 1], r[:, 2], T)
    handle = hold(c[:, 1], c[:, 2], T)
    naive_v = 0.5 * (front + rear) / KMH
    nv0 = np.nan_to_num(naive_v)    # до первого показания тележек — 0
    naive_s = np.concatenate([[0.0], np.cumsum(0.5 * (nv0[1:] + nv0[:-1])
                                               * np.diff(T))])
    # GNSS-эталон (только для показа и метрик на странице)
    m, g = a["mfix"], a["mvel"]
    enu = Enu(m[0, 2], m[0, 3], m[0, 4])
    ref = np.array([enu.fwd(la, lo, al) for la, lo, al in m[:, 2:5]])
    jm, okm = ev.nearest(m[:, 1], T) if len(m) > 1 else (None, np.zeros(len(T), bool))
    gx = np.where(okm, ref[jm, 0], np.nan)
    gy = np.where(okm, ref[jm, 1], np.nan)
    jv, okv = ev.nearest(g[:, 1], T)
    gv = np.where(okv, np.hypot(g[jv, 2], g[jv, 3]), np.nan)
    # путь по GNSS (вдоль траектории) для сравнения пути
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1))]
    gs = np.where(okm, seg[jm], np.nan)

    O = lambda k: np.array([o[k] for o in outs], dtype=float)  # noqa: E731
    s_est = O("s") - O("s")[0]
    cols = {
        "front_kmh": q(front, 100), "rear_kmh": q(rear, 100),
        "handle": [None if not math.isfinite(x) else int(x) for x in handle],
        "gnss_v": q(gv, 100), "gnss_x": q(gx, 10), "gnss_y": q(gy, 10),
        "gnss_s": q(gs, 10),
        "v": q(O("v"), 100), "sigma_v": q(O("sigma_v"), 1000),
        "s": q(s_est, 10), "sigma_s": q(O("sigma_s"), 10),
        "x": q(O("x"), 10), "y": q(O("y"), 10),
        "mode": [int(o["mode"]) for o in outs],
        "flags": [int(o["valid"]) | int(o["slip"]) << 1 | int(o["ambiguous"]) << 2
                  | int(o["handle_ok"]) << 3 | int(o["wheels_stale"]) << 4
                  for o in outs],
        "n_acc": [int(o["n_accepted"]) for o in outs],
        "n_rej": [int(o["n_rejected"]) for o in outs],
        "k_t": q(O("k_t"), 1000), "k_b": q(O("k_b"), 1000),
        "mu": q(O("mu"), 1000), "d": q(O("d"), 1000),
        "healthy": [int(sum(int(h) << i for i, h in enumerate(o["healthy"])))
                    for o in outs],
        "naive_v": q(naive_v, 100), "naive_s": q(naive_s, 10),
    }
    scale = {"front_kmh": 0.01, "rear_kmh": 0.01, "gnss_v": 0.01, "gnss_x": 0.1,
             "gnss_y": 0.1, "gnss_s": 0.1, "v": 0.01, "sigma_v": 0.001, "s": 0.1,
             "sigma_s": 0.1, "x": 0.1, "y": 0.1, "k_t": 0.001, "k_b": 0.001,
             "mu": 0.001, "d": 0.001, "naive_v": 0.01, "naive_s": 0.1}
    dt = float(np.median(np.diff(T)))
    report = {"bag": bag, "n": len(T), "dur_s": float(T[-1] - T[0]), "dt": dt,
              "run_s": t_run, "sizes": {}}
    for tag, step in (("20hz", 1), ("10hz", 2), ("5hz", 4)):
        doc = {"meta": {"bag": bag, "t0": float(T[0]), "dt": dt * step,
                        "n": len(T[::step]), "scale": scale,
                        "flags": "bit0 valid, bit1 slip, bit2 ambiguous, "
                                 "bit3 handle_ok, bit4 wheels_stale",
                        "enu_origin": [float(m[0, 2]), float(m[0, 3]), float(m[0, 4])]},
               "cols": {k: v[::step] for k, v in cols.items()}}
        raw = json.dumps(doc, separators=(",", ":")).encode()
        gz = gzip.compress(raw, 9)
        p = os.path.join(OUT, f"replay_{bag}_{tag}.json")
        with open(p, "wb") as fh:
            fh.write(raw)
        with open(p + ".gz", "wb") as fh:
            fh.write(gz)
        report["sizes"][tag] = {"json_kb": len(raw) / 1024, "gz_kb": len(gz) / 1024}
    # метрики для подписи к экрану (грубо, как в evaluate.py)
    ok = np.isfinite(gv)
    report["v_mae_model"] = float(np.nanmean(np.abs(O("v")[ok] - gv[ok])))
    report["v_mae_naive"] = float(np.nanmean(np.abs(naive_v[ok] - gv[ok])))
    oks = np.isfinite(gs)
    report["s_end_err_model"] = float(s_est[oks][-1] - gs[oks][-1]) if oks.any() else None
    report["s_end_err_naive"] = float(naive_s[oks][-1] - gs[oks][-1]) if oks.any() else None
    okx = np.isfinite(gx)
    e2 = np.hypot(O("x")[okx] - gx[okx], O("y")[okx] - gy[okx])
    report["xy_err_model_mean_m"] = float(e2.mean()) if okx.any() else None
    report["xy_err_model_max_m"] = float(e2.max()) if okx.any() else None
    report["gnss_path_m"] = float(seg[-1])
    report["modes_pct"] = {n: round(100.0 * float(np.mean(np.array(cols["mode"]) == i)), 2)
                           for i, n in enumerate(["COAST", "TRACTION", "BRAKE", "TRANSITION",
                                                  "SLIP", "STANDSTILL", "DEGRADED"])}
    print(json.dumps(report, ensure_ascii=False, indent=1))
    with open(os.path.join(OUT, f"replay_{bag}_report.json"), "w",
              encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
