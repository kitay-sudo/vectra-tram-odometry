#!/usr/bin/env python3
"""Линия для песочницы и вида трассы: simulator/js/track.js.

Источник — pathgraph организаторов (_incoming/pathgraph/*.json, в git не
лежит): по файлу на направление, точки через 1 м, x, y — плоские MGRS от
квадрата 37U CB без переноса (x = UTM37 E − 300 000, y = UTM N − 6 100 000),
z — уровень рельса (высота base_link), tang — курс, curv — кривизна 1/м.

Пишется прореженная копия (шаг STEP м): метры от первой точки направления A
(x — восток, y — север), путь s, высота z, кривизна (максимум модуля на шаге),
и начало направления A в MGRS 37UCB — по нему страница кладёт линию на план
режима «Прогон данных комиссии» (там метры от первой точки GNSS master).

Остановки — точки остановок карты пакета (config/track_map.npz, выучены по
стоянкам обучающих прогонов), спроецированные на направление A: ближе
STOP_R м к оси, курс совпадает (±45°), слиты в пределах STOP_JOIN м.

    docker run --rm -v <repo>:/repo -v <repo>/_incoming:/repo/_incoming:ro -w /repo \
        vectra/tram:integration python3 simulator/tools/gen_track.py
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PKG = os.path.join(ROOT, "ros2_ws", "src", "tram_state_estimator")
sys.path.insert(0, PKG)
from tram_state_estimator import geodesy as GD  # noqa: E402

STEP = 5            # м: шаг прореживания
STOP_R = 8.0        # м: остановка карты не дальше этого от оси направления
STOP_JOIN = 60.0    # м: остановки ближе — одна
GRID_E, GRID_N = 300000.0, 6100000.0      # начало квадрата 37U CB (UTM зона 37)
DIRS = ("щукинская - таллинская", "таллинская - щукинская")


def load_dir(path):
    with open(path, encoding="utf-8") as fh:
        d = json.load(fh)
    pts = d["points"]
    idx = d["paths"][0]["point_indices"] if d.get("paths") else list(range(len(pts)))
    P = np.array([[pts[i]["x"], pts[i]["y"], pts[i]["z"], pts[i]["tang"], pts[i]["curv"]] for i in idx])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))])
    return P, s


def decimate(P, s, x0, y0, z0):
    n = len(s)
    keep = list(range(0, n, STEP))
    if keep[-1] != n - 1:
        keep.append(n - 1)
    k = [float(np.max(np.abs(P[max(0, i - STEP // 2):i + STEP // 2 + 1, 4]))) for i in keep]
    return dict(
        s=[round(float(s[i]), 1) for i in keep],
        x=[round(float(P[i, 0] - x0), 2) for i in keep],
        y=[round(float(P[i, 1] - y0), 2) for i in keep],
        z=[round(float(P[i, 2] - z0), 3) for i in keep],
        k=[round(v, 5) for v in k],
    )


def stops_on(P, s, tmap):
    z = np.load(tmap, allow_pickle=True)
    if "stops" not in z.files or not len(z["stops"]):
        return []
    st = np.asarray(z["stops"], float)
    E, N = GD.utm_fwd(st[:, 0], st[:, 1], 37)
    X, Y = E - GRID_E, N - GRID_N
    head_math = math.pi / 2 - st[:, 2]            # курс остановки: от севера по часовой -> от востока
    out = []
    for x, y, h in zip(X, Y, head_math):
        d = np.hypot(P[:, 0] - x, P[:, 1] - y)
        i = int(np.argmin(d))
        dh = abs((h - P[i, 3] + math.pi) % (2 * math.pi) - math.pi)
        if d[i] <= STOP_R and dh <= math.radians(45):
            out.append(float(s[i]))
    out.sort()
    merged = []
    for v in out:
        if merged and v - merged[-1][-1] <= STOP_JOIN:
            merged[-1].append(v)
        else:
            merged.append([v])
    return [round(float(np.mean(g)), 1) for g in merged]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "_incoming", "pathgraph"))
    ap.add_argument("--map", default=os.path.join(PKG, "config", "track_map.npz"))
    ap.add_argument("--out", default=os.path.join(ROOT, "simulator", "js", "track.js"))
    a = ap.parse_args()
    files = {os.path.splitext(os.path.basename(f))[0]: f for f in glob.glob(os.path.join(a.src, "*.json"))}
    missing = [d for d in DIRS if d not in files]
    if missing:
        sys.exit(f"нет файлов pathgraph: {missing} в {a.src}")
    PA, sA = load_dir(files[DIRS[0]])
    x0, y0, z0 = (float(v) for v in PA[0, :3])
    dirs = []
    for name in DIRS:
        P, s = load_dir(files[name])
        dd = decimate(P, s, x0, y0, z0)
        dd["name"] = name
        dd["length_m"] = round(float(s[-1]), 1)
        dirs.append(dd)
    stops = stops_on(PA, sA, a.map)
    doc = dict(
        source="pathgraph организаторов (_incoming/pathgraph), прорежено до %d м" % STEP,
        frame="метры от первой точки направления A; x — восток, y — север, z — уровень рельса",
        origin=dict(mgrs_grid="37UCB", x=round(x0, 3), y=round(y0, 3), z=round(z0, 3),
                    utm_zone=37, utm_e=round(x0 + GRID_E, 3), utm_n=round(y0 + GRID_N, 3)),
        stops_src="config/track_map.npz пакета (остановки, выученные по стоянкам), на направлении A",
        stops=stops, dirs=dirs,
    )
    body = json.dumps(doc, ensure_ascii=False, separators=(",", ":"))
    with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("// Сгенерировано simulator/tools/gen_track.py — не править руками.\n")
        fh.write("(function (g) {\n")
        fh.write(f"g.TV_TRACK = {body};\n")
        fh.write("if (typeof module !== 'undefined') module.exports = g.TV_TRACK;\n")
        fh.write("})(typeof window !== 'undefined' ? window : globalThis);\n")
    zA = np.array(dirs[0]["z"])
    sa = np.array(dirs[0]["s"])
    g = np.diff(zA) / np.maximum(np.diff(sa), 1e-6)
    print("ok", a.out, f"{os.path.getsize(a.out) / 1024:.0f} КБ;",
          ", ".join(f"{d['name']}: {d['length_m']} м, {len(d['s'])} точек" for d in dirs),
          f"; остановок {len(stops)}: {stops}; уклон ‰ (шаг {STEP} м) мин {1000 * g.min():.0f} макс {1000 * g.max():.0f}")


if __name__ == "__main__":
    main()
