#!/usr/bin/env python3
"""ros_vs_offline — совпадают ли выходы ноды в ROS с офлайн-прогоном Runner.

Докстринг tram_node.py утверждает: «числа оценки и работа ноды совпадают».
Скрипт берёт сырые ряды пробы (out/ros_e2e/<tag>/raw.npz) и прогоняет ту же
связку Runner офлайн по кэшу bag (analysis/cache/<bag>.npz) в двух вариантах:
  node_view  — как видит нода при полном ros2 bag play: все front/rear/cmd и
               ВСЕ GNSS fix (нода сама отбрасывает их после окна выставки,
               но метки GNSS двигают сетку шагов);
  eval_view  — как analysis/evaluate.py: GNSS только первые 3 с.
Параметры — config/tram.yaml (как у ноды), карта — config/track_map.npz.
Сравнение — по ближайшей метке времени.

    python3 tools/dev/ros_vs_offline.py <tag> <bag_id> [--no-gnss] [--topics front,rear]
"""

import argparse
import json
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
sys.path.insert(0, str(PKG))
from tram_state_estimator.estimator_core import Params          # noqa: E402
from tram_state_estimator.runner import Runner                   # noqa: E402
from tram_state_estimator.track_map import TrackMap              # noqa: E402


def node_params():
    y = yaml.safe_load((PKG / "config" / "tram.yaml").read_text(encoding="utf-8"))
    d = y["/tram_state_estimator"]["ros__parameters"]
    names = {f.name for f in fields(Params)}
    core = {k: v for k, v in d.items() if k in names}
    return Params.from_dict(core), d


def run(a, params, extra, view, topics):
    tmap = TrackMap.load(PKG / extra["map_file"]) if extra.get("map_file") else None
    r = Runner(params, track_map=tmap, wheel_timeout=extra["wheel_timeout_s"],
               handle_timeout=extra["handle_timeout_s"])
    r.pos.init_window = extra["init_window_s"]
    ev = []
    for i, key in enumerate(("front", "rear")):
        if key in topics:
            ev += [(tb, 0, i, th, v) for tb, th, v in a[key]]
    if "cmd" in topics:
        ev += [(tb, 1, 0, th, n) for tb, th, n in a["cmd"]]
    if "gnss" in topics and len(a["mfix"]):
        t_end = a["mfix"][0, 0] + 3.0 if view == "eval_view" else np.inf
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


def compare(ros_v, ros_x, off):
    T, V = ros_v[:, 1], ros_v[:, 2]
    To = off[:, 0]
    j = np.clip(np.searchsorted(To, T), 1, len(To) - 1)
    j = np.where(np.abs(To[j - 1] - T) < np.abs(To[j] - T), j - 1, j)
    dts = T - To[j]
    ok = np.abs(dts) <= 0.026
    dv = np.abs(V[ok] - off[j[ok], 1])
    res = {"ros_n": int(len(T)), "offline_n": int(len(To)), "matched": int(ok.sum()),
           "grid_offset_ms_median": round(float(np.median(dts[ok])) * 1e3, 3) if ok.any() else None,
           "dv_max": round(float(dv.max()), 5) if len(dv) else None,
           "dv_mean": round(float(dv.mean()), 6) if len(dv) else None,
           "dv_p99": round(float(np.percentile(dv, 99)), 5) if len(dv) else None,
           "frac_dv_lt_1e-6": round(float((dv < 1e-6).mean()), 4) if len(dv) else None}
    if len(ros_x):
        Tx = ros_x[:, 1]
        k = np.clip(np.searchsorted(To, Tx), 1, len(To) - 1)
        k = np.where(np.abs(To[k - 1] - Tx) < np.abs(To[k] - Tx), k - 1, k)
        okx = np.abs(Tx - To[k]) <= 0.026
        dp = np.linalg.norm(ros_x[okx, 2:5] - off[k[okx], 2:5], axis=1)
        res["dpos_max_m"] = round(float(dp.max()), 3) if len(dp) else None
        res["dpos_mean_m"] = round(float(dp.mean()), 4) if len(dp) else None
        res["dpos_end_m"] = round(float(dp[-1]), 3) if len(dp) else None
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("bag")
    ap.add_argument("--topics", default="front,rear,cmd,gnss")
    a_ = ap.parse_args()
    topics = set(a_.topics.split(","))
    raw = np.load(ROOT / "out" / "ros_e2e" / a_.tag / "raw.npz")
    a = {k: v for k, v in np.load(ROOT / "analysis" / "cache" / f"{a_.bag}.npz").items()}
    params, extra = node_params()
    res = {"tag": a_.tag, "bag": a_.bag, "topics": sorted(topics)}
    for view in ("node_view", "eval_view"):
        off = run(a, params, extra, view, topics)
        res[view] = compare(raw["vel"], raw["odo"], off)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    out = ROOT / "out" / "ros_e2e" / a_.tag / "ros_vs_offline.json"
    try:
        out.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    except OSError:
        pass


if __name__ == "__main__":
    main()
