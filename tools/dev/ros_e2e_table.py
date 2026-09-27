#!/usr/bin/env python3
"""Сводная таблица по всем прогонам out/ros_e2e/*/summary.json (Markdown).

Дополнительно, по кэшу bag (analysis/cache/<bag>.npz): ошибка скорости выхода
против GNSS master/vel - и для прогонов, где GNSS не проигрывался; для
прогонов без выставки (y = z = 0, x = путь) - ошибка пути x против
накопленного пути GNSS с момента первого выхода.

    python3 tools/dev/ros_e2e_table.py [tag ...]
"""

import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "out" / "ros_e2e"


def g(d, *ks):
    for k in ks:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def nearest(T, t):
    j = np.clip(np.searchsorted(T, t), 1, len(T) - 1)
    j = np.where(np.abs(T[j - 1] - t) < np.abs(T[j] - t), j - 1, j)
    return j, np.abs(T[j] - t) <= 0.05


def cache_acc(tag_dir, bag):
    f = ROOT / "analysis" / "cache" / f"{bag}.npz"
    if not f.exists() or not (tag_dir / "raw.npz").exists():
        return {}
    a = np.load(f)
    r = np.load(tag_dir / "raw.npz")
    vel, odo = r["vel"], r["odo"]
    if len(vel) < 20 or len(a["mvel"]) < 20:
        return {}
    o = np.argsort(vel[:, 1])
    T, V = vel[o, 1], vel[o, 2]
    gv = a["mvel"]
    m = (gv[:, 1] >= T[0]) & (gv[:, 1] <= T[-1])
    gv = gv[m]
    j, ok = nearest(T, gv[:, 1])
    e = V[j[ok]] - np.hypot(gv[ok, 2], gv[ok, 3])
    res = {"v_mae": round(float(np.abs(e).mean()), 3), "v_bias": round(float(e.mean()), 3)} if len(e) else {}
    if len(odo) > 20 and np.mean(odo[:, 3] == 0) > 0.99:
        from math import cos, radians
        mf = a["mfix"]
        mf = mf[(mf[:, 1] >= T[0]) & (mf[:, 1] <= T[-1])]
        if len(mf) > 20:
            k = cos(radians(mf[0, 2]))
            xy = np.c_[np.radians(mf[:, 3]) * 6378137.0 * k, np.radians(mf[:, 2]) * 6378137.0]
            cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
            oo = np.argsort(odo[:, 1])
            To, X = odo[oo, 1], odo[oo, 2]
            jj, okk = nearest(To, mf[:, 1])
            d = (X[jj[okk]] - X[jj[okk]][0]) - (cum[okk] - cum[okk][0])
            res["path_err_end_m"] = round(float(d[-1]), 1)
            res["path_gnss_m"] = round(float(cum[-1]), 1)
    return res


def row(tag):
    d = OUT / tag
    R = json.loads((d / "summary.json").read_text(encoding="utf-8"))
    run = (d / "run.log").read_text(encoding="utf-8", errors="replace") if (d / "run.log").exists() else ""
    bags = re.search(r"bags=(\S+)", run)
    bag = bags.group(1) if bags else ""
    node = (d / "node.log").read_text(encoding="utf-8", errors="replace") if (d / "node.log").exists() else ""
    crashed = "process has died" in node and "user interrupted" not in node.split("process has died")[0]
    exc = re.findall(r"^\[tram_estimator-1\] (\w+Error: .*)$", node, re.M)
    v = R["outputs"]["velocity"]
    L = R.get("latency", {})
    P = R.get("node_process", {})
    return {
        "tag": tag, "bag": bag, "rate": g(R, "probe", "rate_arg"),
        "out_n": v.get("count"), "exp_n": v.get("expected_by_stamp"),
        "hz_stamp": v.get("rate_stamp_hz"), "hz_wall": v.get("rate_wall_hz"),
        "gaps>0.1": v.get("wall_gaps_over_100ms"), "max_gap_s": g(v, "wall_gap_s", "max"),
        "min_1s": v.get("min_msgs_in_1s_window"),
        "lat_p50": g(L, "in2out_vehicle_steady_ms", "p50"),
        "lat_p95": g(L, "in2out_vehicle_steady_ms", "p95"),
        "lat_p99": g(L, "in2out_vehicle_steady_ms", "p99"),
        "lat_max": g(L, "in2out_vehicle_steady_ms", "max"),
        "lat>100": L.get("in2out_vehicle_steady_over_100ms"),
        "unans": L.get("in2out_vehicle_unanswered"),
        "proc_p50": g(L, "proc_ms", "p50"), "proc_p99": g(L, "proc_ms", "p99"),
        "cpu_mean": g(P, "cpu_pct_1core", "mean"), "cpu_p95": g(P, "cpu_pct_1core", "p95"),
        "cpu_max": g(P, "cpu_pct_1core", "max"),
        "rss": P.get("rss_mb_first_last_max"), "rss_slope": P.get("rss_slope_mb_per_min_after_30s"),
        "lost": g(R, "status", "lost_at_probe_by_frame_count"),
        "crash": ("; ".join(exc) or "died") if crashed else "",
        "acc": cache_acc(d, bag.split(",")[0]) if bag else {},
    }


def main():
    tags = sys.argv[1:] or sorted(p.name for p in OUT.iterdir() if (p / "summary.json").exists())
    rows = [row(t) for t in tags]
    cols = ["tag", "rate", "out_n", "exp_n", "hz_stamp", "hz_wall", "gaps>0.1", "max_gap_s", "min_1s",
            "lat_p50", "lat_p95", "lat_p99", "lat_max", "lat>100", "unans", "proc_p50", "proc_p99",
            "cpu_mean", "cpu_p95", "cpu_max", "rss", "rss_slope", "lost", "crash", "acc"]
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for r in rows:
        print("| " + " | ".join(str(r[c]) if r[c] not in (None, "") else "-" for c in cols) + " |")
    (OUT / "table.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
