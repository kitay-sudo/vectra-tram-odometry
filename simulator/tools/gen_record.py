#!/usr/bin/env python3
"""Настоящая запись для песочницы: simulator/replays/rec_<прогон>.js.

Песочница проигрывает запись вместо имитатора: модель (JS-порт ядра и связки,
js-port/check.sh) получает сообщения тележек и ручки в порядке записи bag, как
нода. GNSS в модель не подаётся (как в js-port/record_runner.py).

Источник - кэш прогона датасета (analysis/bagio) или папка записи (--bag),
например запись проверки организаторов _incoming/check/check-code/bags/30618_88aea4d9.

Что пишется:
  * сообщения: код (0 передняя, 1 задняя тележка, 2 ручка), время приёма и
    метка header в 0,1 мс от начала, значение (км/ч × 1000 или позиция ручки);
  * истина: скорость |v| и пройденный путь (интеграл этой скорости), по ним
    считаются ошибки. Если в записи есть эталон организаторов
    /localization/kinematic_state (base_link в MGRS 37UCB, как у проверки жюри),
    истина - он; иначе GNSS master;
  * линия записи: путь самой записи, вместе с разворотными кольцами, точки
    через TRACK_STEP_M по пройденному пути (плоскость и высота - эталон, у GNSS
    master высота рельса = высота антенны - 3,0 м по tf организаторов), и
    стоянки - где вагон стоял дольше STAND_S. По этой линии страница рисует
    сцену, карту и профиль, вагон на ней там же, где был на самом деле.
Метки сдвинуты на T0, кратное шагу сетки 0,05 с: узлы сетки связки те же.

    docker run --rm -v <repo>:/repo -w /repo vectra/tram:compose \
        python3 simulator/tools/gen_record.py [прогон] [--bag <папка>]
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_replay as X  # noqa: E402  (кэш записей, порядок событий, геодезия пакета)

GNSS_STEP_S = 0.2           # истина на странице: 5 точек в секунду, между ними линейно
DT = 0.05                   # шаг сетки связки
TRACK_STEP_M = 5.0          # шаг линии записи по пути, как у simulator/js/track.js
Z_SMOOTH_M = 50.0           # сглаживание высоты: уклон на странице берётся по ±20 м
ANTENNA_Z_M = 3.0           # антенна GNSS над уровнем рельса (tf организаторов)
STAND_V = 0.3               # м/с: ниже - вагон стоит
STAND_S = 2.5               # с: стоянка дольше - место на линии записи (платформа или светофор)
STAND_JOIN_M = 25.0         # стоянки ближе - одна (подтягивание к платформе)
KIN_TOPIC = "/localization/kinematic_state"


def track_doc():
    """TV_TRACK из simulator/js/track.js (JSON внутри обёртки)."""
    with open(os.path.join(ROOT, "simulator", "js", "track.js"), encoding="utf-8") as fh:
        src = fh.read()
    return json.loads(src[src.index("g.TV_TRACK = ") + len("g.TV_TRACK = "):src.rindex("};") + 1])


def kinematic(bag_dir):
    """Эталон организаторов из папки записи: [метка header, x, y (MGRS 37UCB), z, |v|] или None."""
    from rosbags.highlevel import AnyReader

    rows = []
    with AnyReader([Path(bag_dir)], default_typestore=X.bagio.typestore()) as rd:
        conns = [c for c in rd.connections if c.topic == KIN_TOPIC]
        for conn, _, raw in rd.messages(connections=conns):
            m = rd.deserialize(raw, conn.msgtype)
            p, lv = m.pose.pose.position, m.twist.twist.linear
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, math.hypot(lv.x, lv.y)))
    return np.array(sorted(rows)) if rows else None


def direction_name(px, py, tr):
    """Название направления pathgraph, вдоль которого идёт запись (больше прирост пути)."""
    best = None
    for d in tr["dirs"]:
        x, y, s = (np.asarray(d[k], float) for k in ("x", "y", "s"))
        i = [int(np.argmin(np.hypot(x - qx, y - qy))) for qx, qy in zip(px[::50], py[::50])]
        gain = float(np.sum(np.diff(s[i])))
        if best is None or gain > best[0]:
            best = (gain, d.get("name", ""))
    return best[1]


def stands(t, v, p):
    """Пути стоянок: v < STAND_V дольше STAND_S; близкие сливаются."""
    out, a = [], None
    for i, slow in enumerate(v < STAND_V):
        if slow and a is None:
            a = i
        if (not slow or i == len(v) - 1) and a is not None:
            if t[i] - t[a] >= STAND_S:
                s = float(np.median(p[a:i + 1]))
                if not out or s - out[-1] > STAND_JOIN_M:
                    out.append(round(s, 1))
            a = None
    return out


def line_of_record(t, p, px, py, pz, z0):
    """Линия записи через TRACK_STEP_M по пути: s, x, y, z (от z0), модуль кривизны."""
    pk = np.r_[np.arange(0.0, p[-1], TRACK_STEP_M), p[-1]]
    tk = np.interp(pk, p + 1e-9 * np.arange(len(p)), t)       # путь не убывает: время по пути
    x, y = np.interp(tk, t, px), np.interp(tk, t, py)
    w = max(1, int(Z_SMOOTH_M / TRACK_STEP_M / 2))
    zr = np.interp(tk, t, pz)
    z = np.array([zr[max(0, i - w):i + w + 1].mean() for i in range(len(zr))]) - z0
    th = np.unwrap(np.arctan2(np.diff(y), np.diff(x)))
    k = np.r_[0.0, np.abs(np.diff(th)) / np.maximum(np.diff(pk)[1:], 1e-6), 0.0]
    return {"s": [round(v, 1) for v in pk], "x": [round(v, 2) for v in x], "y": [round(v, 2) for v in y],
            "z": [round(v, 2) for v in z], "k": [round(v, 5) for v in k]}


def main():
    ap = argparse.ArgumentParser(description="Запись для песочницы: сообщения тележек и ручки, истина и линия записи")
    ap.add_argument("run", nargs="?", default=X.DEFAULT_RUN, help="прогон, например 30618_e9a34502")
    ap.add_argument("--bag", default=None, help="папка записи вместо кэша, например _incoming/check/check-code/bags/30618_88aea4d9")
    ap.add_argument("--out", default=None, help="файл вывода (по умолчанию simulator/replays/rec_<прогон>.js)")
    a = ap.parse_args()
    if a.bag:
        a.run = os.path.basename(os.path.normpath(a.bag))
    for s in (sys.stdout, sys.stderr):
        s.reconfigure(encoding="utf-8")
    print(f"Запись для песочницы: {a.run}", flush=True)

    if a.bag and not os.path.isdir(a.bag):
        raise SystemExit(f"Ошибка: нет папки записи {a.bag}")
    rec = X.bagio.extract(Path(a.bag)) if a.bag else X.bagio.load(a.run)
    kin = kinematic(a.bag) if a.bag else None
    ev = [(tb, kind, i, th, val) for tb, kind, i, th, val in X.E.events(rec) if kind != 2]
    if not ev:
        raise SystemExit(f"Ошибка: в {a.run} нет сообщений тележек и ручки")
    t0 = math.floor(min(e[3] for e in ev) / DT) * DT
    code = [int(i) if kind == 0 else 2 for _, kind, i, _, _ in ev]
    tb = [round((e[0] - t0) * 1e4) for e in ev]
    th = [round((e[3] - t0) * 1e4) for e in ev]
    val = [round(v * 1000) if kind == 0 else round(v) for _, kind, _, _, v in ev]

    # истина: эталон организаторов, если он есть в записи; иначе GNSS master (фиксы -> UTM)
    tr = track_doc()
    o = tr["origin"]
    if kin is not None:
        ref = f"эталон организаторов {KIN_TOPIC}"
        gt = mt = kin[:, 0] - t0
        gv = kin[:, 4]
        px, py, pz = kin[:, 1] - o["x"], kin[:, 2] - o["y"], kin[:, 3]
    else:
        ref = "GNSS master"
        g = rec["mvel"]
        gt = g[:, 1] - t0
        gv = np.hypot(g[:, 2], g[:, 3])
        m = rec["mfix"]
        en = X.utm_en(m[:, 2], m[:, 3], int(o["utm_zone"]))
        px, py, pz = en[:, 0] - o["utm_e"], en[:, 1] - o["utm_n"], m[:, 4] - ANTENNA_Z_M
        mt = m[:, 1] - t0
    print(f"Истина: {ref}", flush=True)
    grid = np.arange(max(gt[0], mt[0]), min(gt[-1], mt[-1]), GNSS_STEP_S)
    v_ref = np.interp(grid, gt, gv)
    p_ref = np.r_[0.0, np.cumsum(0.5 * (v_ref[1:] + v_ref[:-1]) * GNSS_STEP_S)]

    # линия записи и стоянки
    gx, gy, gz = (np.interp(grid, mt, q) for q in (px, py, pz))
    line = line_of_record(grid, p_ref, gx, gy, gz, float(o["z"]))
    line["name"] = direction_name(gx, gy, tr)
    line["stops"] = stands(grid, v_ref, p_ref)

    doc = {
        "run": a.run, "source": "кэш записи (analysis/bagio), порядок событий tools/evaluate.events",
        "t0": t0, "dur": float(max(e[0] for e in ev) - t0),
        "n": len(ev), "code": "".join(map(str, code)), "tb": tb, "th": th, "val": val,
        "g": {"t0": float(grid[0]), "dt": GNSS_STEP_S, "v": [round(x, 3) for x in v_ref],
              "p": [round(x, 2) for x in p_ref]},
        "track": line, "ref": ref, "core_sha1": X.core_sha1(),
    }
    out = a.out or os.path.join(ROOT, "simulator", "replays", f"rec_{a.run}.js")
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("// Сгенерировано simulator/tools/gen_record.py - не править руками.\n")
        fh.write("(function (g) {\ng.TV_REC = g.TV_REC || {};\n")
        fh.write(f"g.TV_REC[{json.dumps(a.run)}] = {json.dumps(doc, ensure_ascii=False, separators=(',', ':'))};\n")
        fh.write("})(typeof window !== 'undefined' ? window : globalThis);\n")
    print(f"Сообщений: {len(ev)}, длительность {doc['dur']:.0f} с, путь {p_ref[-1]:.0f} м, "
          f"стоянок {len(line['stops'])}, направление: {line['name']}", flush=True)
    print(f"Готово: {os.path.relpath(out, ROOT)} ({os.path.getsize(out) / 1024:.0f} КБ)", flush=True)


if __name__ == "__main__":
    main()
