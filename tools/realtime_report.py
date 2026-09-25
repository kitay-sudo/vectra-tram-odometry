#!/usr/bin/env python3
"""realtime_report — итог замера tools/measure_realtime.sh: summary.json и таблица.

Читает из каталога замера summary_probe.json и raw.npz (tools/ros_probe.py) и
docker_stats.csv (опрос docker stats контейнера ноды), пишет summary.json и
summary.md: частота, задержка, CPU, RSS по минутам, проверка утечки и сверка
с критерием 4 ТЗ (задержка ≤ 100 мс, пик ≤ 250 мс; ≥ 10 Гц; ≤ 2 ядра;
≤ 0,5 ГБ; без утечек).

    python3 tools/realtime_report.py out/realtime/<tag> --bag ID --rate 1 ...
"""

import argparse
import csv
import json
import os
import shutil
import sys

import numpy as np


def dist(a, nd=1):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"n": 0}
    r = lambda x: round(float(x), nd)
    return {"n": int(len(a)), "mean": r(a.mean()), "p50": r(np.percentile(a, 50)),
            "p95": r(np.percentile(a, 95)), "p99": r(np.percentile(a, 99)), "max": r(a.max())}


def mib(s):
    s = s.strip()
    for unit, k in (("GiB", 1024.0), ("MiB", 1.0), ("KiB", 1 / 1024.0), ("GB", 953.674),
                    ("MB", 0.953674), ("kB", 0.000953674), ("B", 1 / 1048576.0)):
        if s.endswith(unit):
            return float(s[:-len(unit)]) * k
    return float("nan")


def docker_stats(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r["t_unix"]), float(r["cpu_pct"]), mib(r["mem_usage"]),
                             mib(r["mem_limit"])))
            except (ValueError, KeyError):
                continue
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--bag", default="")
    ap.add_argument("--rate", default="1.0")
    ap.add_argument("--cpus", default="2")
    ap.add_argument("--memory", default="512m")
    ap.add_argument("--oom", default="")
    ap.add_argument("--alive", default="")
    ap.add_argument("--stop-s", default="")
    ap.add_argument("--exit-code", default="")
    ap.add_argument("--latest", default="", help="куда положить копию summary.{json,md}")
    a = ap.parse_args()

    P = json.load(open(os.path.join(a.dir, "summary_probe.json"), encoding="utf-8"))
    z = np.load(os.path.join(a.dir, "raw.npz"))
    vel, proc = z["vel"], z["proc"]
    o = P["outputs"]["velocity"]
    L = P["latency"]

    # ресурсы ноды по psutil (1 Гц): CPU % одного ядра, RSS МБ
    res = {}
    if len(proc):
        t = proc[:, 0] - proc[0, 0]
        cpu, rss = proc[:, 1], proc[:, 2]
        res["cpu_pct_1core"] = dist(cpu)
        res["cores_mean"] = round(float(cpu.mean()) / 100.0, 3)
        res["rss_mb"] = dist(rss)
        res["rss_first_last_max_mb"] = [round(float(rss[0]), 1), round(float(rss[-1]), 1),
                                        round(float(rss.max()), 1)]
        warm = t > 60.0
        if warm.sum() >= 30:
            k, _ = np.polyfit(t[warm] / 60.0, rss[warm], 1)
            h = int(warm.sum()) // 2
            res["rss_slope_mb_per_min_after_60s"] = round(float(k), 3)
            res["rss_mean_halves_after_60s_mb"] = [round(float(rss[warm][:h].mean()), 1),
                                                   round(float(rss[warm][h:].mean()), 1)]
        per_min = []
        for m in range(int(t.max() // 60) + 1):
            s = (t >= 60 * m) & (t < 60 * (m + 1))
            if s.any():
                per_min.append({"min": m, "rss_mb": round(float(rss[s].mean()), 1),
                                "rss_max_mb": round(float(rss[s].max()), 1),
                                "cpu_pct": round(float(cpu[s].mean()), 1)})
        res["per_minute"] = per_min
        res["threads_max"] = int(proc[:, 3].max())
    ds = docker_stats(os.path.join(a.dir, "docker_stats.csv"))
    if ds:
        D = np.array(ds)
        res["docker_stats"] = {"samples": int(len(D)), "cpu_pct": dist(D[:, 1]),
                               "mem_mib": dist(D[:, 2]),
                               "mem_limit_mib": round(float(np.nanmax(D[:, 3])), 1)}

    span = float(vel[-1, 0] - vel[0, 0]) if len(vel) > 1 else 0.0
    # разрывы по стенным часам: выходы ноды и входы /vehicle/* (как их отдал плеер).
    # Если разрыв выхода совпадает с разрывом входов, пауза — в самой записи bag.
    inp = z["inp"]
    veh = np.sort(inp[np.isin(inp[:, 0], (0, 1, 2)), 1]) if len(inp) else np.zeros(0)
    gaps = {"output_top5_s": [round(float(x), 3) for x in np.sort(np.diff(vel[:, 0]))[-5:][::-1]]
            if len(vel) > 5 else [],
            "input_top5_s": [round(float(x), 3) for x in np.sort(np.diff(veh))[-5:][::-1]]
            if len(veh) > 5 else []}
    st = L.get("in2out_vehicle_steady_ms", {})
    al = L.get("in2out_vehicle_ms", {})
    slope = res.get("rss_slope_mb_per_min_after_60s")
    checks = [
        ("частота /result/velocity ≥ 10 Гц (рек. 20)", f"{o.get('rate_stamp_hz')} Гц по меткам, {o.get('rate_wall_hz')} по стенным",
         (o.get("rate_stamp_hz") or 0) >= 10 and (o.get("rate_wall_hz") or 0) >= 10),
        ("выходы от узлов сетки 50 мс", f"{o.get('count')}/{o.get('expected_by_stamp')}",
         (o.get("count") or 0) >= 0.99 * (o.get("expected_by_stamp") or 1)),
        ("задержка in2out ≤ 100 мс (p99, установившийся режим)",
         f"p50 {st.get('p50')} / p95 {st.get('p95')} / p99 {st.get('p99')} / max {st.get('max')} мс",
         st.get("n", 0) > 0 and st["p99"] <= 100),
        ("пик задержки ≤ 250 мс (установившийся режим; старт bag — справочно)",
         f"устан.: > 250 мс {L.get('in2out_vehicle_steady_over_250ms')} из {st.get('n')}; весь прогон: max {al.get('max')} мс, > 250 мс {L.get('in2out_vehicle_over_250ms')}",
         al.get("n", 0) > 0 and (L.get("in2out_vehicle_steady_over_250ms", 0) == 0)),
        ("CPU ≤ 2 ядра", f"среднее {res.get('cores_mean')} ядра, max {res.get('cpu_pct_1core', {}).get('max')} % ядра",
         res.get("cpu_pct_1core", {}).get("max", 1e9) <= 200),
        ("ОЗУ ≤ 0,5 ГБ", f"RSS max {res.get('rss_first_last_max_mb', [None] * 3)[2]} МБ; лимит контейнера {a.memory}; OOM: {a.oom}",
         (res.get("rss_first_last_max_mb", [0, 0, 1e9])[2] <= 512) and a.oom != "true"),
        # короткий bag (< ~90 с после прогрева): наклон не считается — «н/д», не FAIL
        ("без утечки памяти", f"наклон RSS после 60 с {slope} МБ/мин; половины {res.get('rss_mean_halves_after_60s_mb')}"
         if slope is not None else "н/д: после прогрева 60 с меньше 30 замеров RSS (bag короче ~90 с)",
         None if slope is None else slope < 0.5),
        ("работа без вмешательства: нода жива до конца", f"alive={a.alive}, SIGINT-останов {a.stop_s} с, код {a.exit_code}",
         a.alive == "true"),
    ]
    verdict = lambda ok: "н/д" if ok is None else ("PASS" if ok else "FAIL")
    # разрывы выхода > 0,3 с: совпадает ли каждый с разрывом входов /vehicle/*
    # (±0,1 с по времени начала) — тогда пауза в самой записи bag
    big = []
    if len(vel) > 5 and len(veh) > 5:
        dv, dveh = np.diff(vel[:, 0]), np.diff(veh)
        for k in np.where(dv > 0.3)[0]:
            t = vel[k, 0]
            m = (np.abs(veh[:-1] - t) < 0.1 + dv[k]) & (dveh > 0.5 * dv[k])
            big.append({"t_s": round(float(t - vel[0, 0]), 1), "gap_s": round(float(dv[k]), 3),
                        "inputs_gap": bool(m.any())})
    gaps["output_over_300ms"] = big
    S = {"bag": a.bag, "rate": float(a.rate), "limits": {"cpus": a.cpus, "memory": a.memory},
         "wall_span_s": round(span, 1), "outputs": o, "latency": L, "node": res,
         "oom_killed": a.oom, "alive_at_end": a.alive, "stop_s": a.stop_s,
         "exit_code": a.exit_code, "input_counts": P.get("inputs"),
         "accuracy_sanity": P.get("accuracy_sanity_vs_bag_gnss"),
         "wall_gaps": gaps,
         "checks": [{"check": c, "value": v, "pass": None if ok is None else bool(ok)}
                    for c, v, ok in checks],
         "pass": all(ok is not False for _, _, ok in checks)}
    with open(os.path.join(a.dir, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(S, fh, ensure_ascii=False, indent=1)

    md = [f"# Замер реального времени: {a.bag} x{a.rate}", "",
          f"Нода в контейнере `--cpus {a.cpus} --memory {a.memory}` (без swap); плеер и проба "
          f"в соседнем контейнере в тех же сетевом/IPC/PID пространствах. Длительность по "
          f"стенным часам {span / 60:.1f} мин. Команда: `tools/measure_realtime.sh --bag {a.bag} --rate {a.rate}`.", "",
          "| критерий ТЗ | измерено | итог |", "|---|---|---|"]
    md += [f"| {c} | {v} | {verdict(ok)} |" for c, v, ok in checks]
    if res.get("per_minute"):
        md += ["", "RSS и CPU ноды по минутам (psutil, раз в 1 с):", "",
               "| мин | RSS ср., МБ | RSS макс., МБ | CPU, % ядра |", "|---|---|---|---|"]
        md += [f"| {r['min']} | {r['rss_mb']} | {r['rss_max_mb']} | {r['cpu_pct']} |" for r in res["per_minute"]]
    if "docker_stats" in res:
        d = res["docker_stats"]
        md += ["", f"docker stats контейнера ноды ({d['samples']} замеров): CPU ср. "
               f"{d['cpu_pct'].get('mean')} %, макс {d['cpu_pct'].get('max')} %; память ср. "
               f"{d['mem_mib'].get('mean')} МиБ, макс {d['mem_mib'].get('max')} МиБ из {d['mem_limit_mib']} МиБ."]
    md += ["", f"Крупнейшие разрывы по стенным часам, с: выход ноды {gaps['output_top5_s']}; "
           f"входы /vehicle/* от плеера {gaps['input_top5_s']}."]
    if big:
        same = [g for g in big if g["inputs_gap"]]
        own = [g for g in big if not g["inputs_gap"]]
        if same:
            md += [f"Разрывы выхода > 0,3 с вместе с разрывом входов (пауза в записи bag; "
                   f"пока входы молчат, выхода нет — пульс выхода, поток robust): {same}."]
        if own:
            md += [f"Разрывы выхода > 0,3 с БЕЗ разрыва входов (задержка самой ноды): {own}."]
    else:
        md += ["Разрывов выхода > 0,3 с нет."]
    md += ["", f"Итог: **{'PASS' if S['pass'] else 'FAIL'}**", ""]
    with open(os.path.join(a.dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(md))
    print("\n".join(md))
    if a.latest:
        for f in ("summary.json", "summary.md"):
            shutil.copyfile(os.path.join(a.dir, f), os.path.join(a.latest, f))
    return 0 if S["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
