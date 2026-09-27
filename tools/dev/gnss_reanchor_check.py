#!/usr/bin/env python3
"""gnss_reanchor_check — ошибка положения, когда GNSS идёт ВЕСЬ прогон.

Гипотеза (проверка ROS 2 E2E): runner.Runner.on_fix (runner.py:149-157) после
выставки на КАЖДОМ fix делает self.s0 = s ядра, даже когда Position.on_fix
этот fix отбросил (окно init_window прошло). Тогда положение xyz(s - s0)
каждые 0,1 с возвращается к точке выставки — положение «замерзает».
analysis/evaluate.py этого не видит: подаёт GNSS только первые 3 с.
ros2 bag play полного прогона (наше демо; bag жюри, если GNSS в нём дольше
первых секунд) — подаёт GNSS весь прогон.

Три варианта на одном прогоне (офлайн, та же связка Runner, лист tram.yaml):
  A  GNSS только первые 3 с (как evaluate.py)
  B  GNSS весь прогон, Runner как есть (как нода при полном bag play)
  C  GNSS весь прогон, Runner с правкой: s0 переносится, только если fix
     действительно вошёл в выставку (подкласс, файлы пакета НЕ меняются)

    python3 tools/dev/gnss_reanchor_check.py 30618_af7496f0 30618_0e41eac3
"""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "dev"))
from ros_vs_offline import node_params, PKG                     # noqa: E402
from tram_state_estimator.estimator_core import IS               # noqa: E402
from tram_state_estimator.runner import Enu, Runner              # noqa: E402
from tram_state_estimator.track_map import TrackMap              # noqa: E402


class FixedRunner(Runner):
    """Предлагаемая правка runner.py:149-157."""

    def on_fix(self, stamp, antenna, lat, lon, alt):
        out = self._advance(stamp)
        moved = (float(self.core.x[IS]) - self.s0) if self.s0 is not None else 0.0
        was = self.pos.ready
        n = len(self.pos._acc)
        self.pos.on_fix(stamp, antenna, lat, lon, alt, moved)
        if self.pos.ready and was and len(self.pos._acc) != n:
            self.s0 = float(self.core.x[IS])
        return out


def run(a, cls, gnss_s):
    params, extra = node_params()
    tmap = TrackMap.load(PKG / extra["map_file"])
    r = cls(params, track_map=tmap, wheel_timeout=extra["wheel_timeout_s"],
            handle_timeout=extra["handle_timeout_s"])
    r.pos.init_window = extra["init_window_s"]
    ev = [(tb, 0, i, th, v) for i, k in enumerate(("front", "rear")) for tb, th, v in a[k]]
    ev += [(tb, 1, 0, th, n) for tb, th, n in a["cmd"]]
    t_end = a["mfix"][0, 0] + gnss_s
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        ev += [(row[0], 2, ant, row[1], tuple(row[2:5])) for row in a[key] if row[0] <= t_end]
    ev.sort(key=lambda e: e[0])
    outs = []
    for tb, kind, i, th, val in ev:
        if kind == 0:
            outs += r.on_wheel(i, th, val)
        elif kind == 1:
            outs += r.on_handle(th, val)
        else:
            outs += r.on_fix(th, i, *val)
    return np.array([[o["stamp"], o["v"], o["x"], o["y"], o["z"]] for o in outs])


def score(a, O):
    m = a["mfix"]
    enu = Enu(*m[0, 2:5])
    ref = np.array([enu.fwd(*row[2:5]) for row in m])
    T = O[:, 0]
    j = np.clip(np.searchsorted(T, m[:, 1]), 1, len(T) - 1)
    j = np.where(np.abs(T[j - 1] - m[:, 1]) < np.abs(T[j] - m[:, 1]), j - 1, j)
    ok = np.abs(T[j] - m[:, 1]) <= 0.05
    d3 = np.linalg.norm(O[j[ok], 2:5] - ref[ok], axis=1)
    g = a["mvel"]
    k = np.clip(np.searchsorted(T, g[:, 1]), 1, len(T) - 1)
    k = np.where(np.abs(T[k - 1] - g[:, 1]) < np.abs(T[k] - g[:, 1]), k - 1, k)
    okv = np.abs(T[k] - g[:, 1]) <= 0.05
    ev = O[k[okv], 1] - np.hypot(g[okv, 2], g[okv, 3])
    path = float(np.sum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1)))
    return {"pos3d_mean_m": round(float(d3.mean()), 1), "pos3d_max_m": round(float(d3.max()), 1),
            "pos3d_end_m": round(float(d3[-1]), 1), "v_mae": round(float(np.abs(ev).mean()), 4),
            "gnss_path_km": round(path / 1000, 2)}


def main():
    """Аргументы: id прогонов; --late S — нода «включается» через S с после
    начала bag (все сообщения раньше отбрасываются, начало ENU — первый fix
    после включения, как у ноды)."""
    args, late = [], 0.0
    it = iter(sys.argv[1:])
    for x in it:
        if x == "--late":
            late = float(next(it))
        else:
            args.append(x)
    res = {}
    for b in args or ["30618_af7496f0"]:
        a = {k: v for k, v in np.load(ROOT / "analysis" / "cache" / f"{b}.npz").items()}
        if late:
            t_join = min(a[k][0, 0] for k in ("front", "rear", "cmd") if len(a[k])) + late
            a = {k: (v[v[:, 0] >= t_join] if v.ndim == 2 and len(v) else v) for k, v in a.items()}
            b = f"{b} (late {late:g} s)"
        res[b] = {"A_gnss3s": score(a, run(a, Runner, 3.0)),
                  "B_gnss_all_asis": score(a, run(a, Runner, np.inf)),
                  "C_gnss_all_fixed": score(a, run(a, FixedRunner, np.inf))}
        print(b, json.dumps(res[b], ensure_ascii=False), flush=True)
    return res


if __name__ == "__main__":
    main()
