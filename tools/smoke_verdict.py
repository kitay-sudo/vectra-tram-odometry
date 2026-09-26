#!/usr/bin/env python3
"""smoke_verdict — таблица PASS/FAIL для одного прогона tools/ros_smoke.sh.

    python3 tools/smoke_verdict.py <out>/summary.json --alive 1 --shut 1 --trace 0 \
        [--fixture test/data/e2e_*.npz --raw <out>/raw.npz --need-position]

Критерии (TODO WP9): >= 19 Гц по меткам и стенным часам; in2out p99 < 100 мс в
установившемся режиме; >= 95 % узлов сетки; /result/position — не меньше 95 %
узлов сетки и не больше, чем /result/velocity (до якоря нода положение не
публикует); 0 NaN; frame_id map/base_link; нода жива; останов по SIGINT;
положение в системе судьи — MGRS от угла 37UCB непрерывно, точка base_link:
x 98 800…103 700 м (линия E 398,8…403,7 км), z 140…180 м (уровень рельса). С
--fixture — положение против эталона base_link по GNSS фикстуры (пары
master+rover, tf антенн; e2e_replay.reference) в системе выхода (refgeo):
ошибка без вычета скачка на границе 100-км квадратов MGRS, как у судьи
(развёрнутая — справочно); в системе "mgrs" выход и эталон должны быть в
одном квадрате (фикстура целиком в 37UCB). С --need-position положение
обязательно: выставка прошла и средняя 3D < 10 м (сценарий «GNSS только
первые секунды»).

С --bag-meta (metadata.yaml bag, проигранного целиком) проверяется и обвязка:
получила ли проба все входы bag. Если проба потеряла начало bag (гонка
обнаружения DDS на старте проигрывания, риск 16), нода, скорее всего, потеряла
его тоже; такой прогон с проваленной проверкой помечается «НЕ ЗАСЧИТАН» (код 3),
а не FAIL ноды.
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "ros2_ws", "src", "tram_state_estimator", "test"))


X_RANGE = (98800.0, 103700.0)   # м: MGRS от 37UCB, линия E 398,8…403,7 км
Z_RANGE = (140.0, 180.0)         # м: высота уровня рельса на линии (pathgraph 144,8…173,6)


def position(raw, fixture):
    import e2e_replay
    import refgeo
    odo = np.load(raw)["odo"]
    fx = e2e_replay.load_fixture(fixture)
    m = fx["mfix"]
    if len(odo) < 2:
        return {"aligned": False, "pairs": 0}
    o = odo[np.argsort(odo[:, 1])]
    T, X = o[:, 1], o[:, 2:5]
    t_ref, fr_all = e2e_replay.reference(fx)            # base_link по парам антенн
    j = np.clip(np.searchsorted(T, t_ref), 1, len(T) - 1)
    j = np.where(np.abs(T[j - 1] - t_ref) < np.abs(T[j] - t_ref), j - 1, j)
    ok = np.abs(T[j] - t_ref) <= 0.05
    # /result/position публикуется только после выставки (pos_valid); признак выставки — y и z ненулевые
    aligned = bool(np.any(X[-20:, 1] != 0.0) or np.any(X[-20:, 2] != 0.0))
    if not ok.any():
        return {"aligned": aligned, "pairs": 0}
    fr = {k: v[ok] for k, v in fr_all.items()}
    frame, _ = refgeo.detect(X[j[ok]], fr)
    d3, d3u, mism = refgeo.errors(X[j[ok]], fr[frame], frame)
    return {"aligned": aligned, "frame": frame, "pairs": int(ok.sum()),
            "mean_m": round(float(d3.mean()), 2), "max_m": round(float(d3.max()), 2),
            "end_m": round(float(d3[-1]), 2),
            "mean_m_unwrapped": round(float(d3u.mean()), 2), "square_mismatch": mism,
            "squares": refgeo.mgrs_squares(m[:, 2], m[:, 3])}


PROBE_KEYS = {"/vehicle/front_bogie_velocity": "front", "/vehicle/rear_bogie_velocity": "rear",
              "/vehicle/driver_position_cmd": "cmd", "/sensing/gnss/master/fix": "mfix",
              "/sensing/gnss/rover/fix": "rfix"}


def bag_counts(meta):
    import yaml
    info = yaml.safe_load(open(meta, encoding="utf-8"))["rosbag2_bagfile_information"]
    return {PROBE_KEYS[t["topic_metadata"]["name"]]: int(t["message_count"])
            for t in info["topics_with_message_count"] if t["topic_metadata"]["name"] in PROBE_KEYS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("summary")
    ap.add_argument("--alive", type=int, default=0)
    ap.add_argument("--shut", type=int, default=0)
    ap.add_argument("--trace", type=int, default=0)
    ap.add_argument("--fixture", default="")
    ap.add_argument("--raw", default="")
    ap.add_argument("--need-position", action="store_true")
    ap.add_argument("--json", default="", help="куда записать итог (json)")
    ap.add_argument("--bag-meta", default="", help="metadata.yaml bag, проигранного целиком")
    a = ap.parse_args()

    R = json.load(open(a.summary, encoding="utf-8"))
    o = R["outputs"]["velocity"]
    p = R["outputs"]["position"]
    L = R["latency"]
    st = L.get("in2out_vehicle_steady_ms", {})
    al = L.get("in2out_vehicle_ms", {})
    bad = R.get("nonfinite_or_bad", {})
    fr = R.get("frame_ids", {})
    exp = o.get("expected_by_stamp") or 0
    rows = [
        ("выходов /result/velocity от узлов сетки", f"{o.get('count')}/{exp}",
         exp > 0 and o.get("count", 0) >= 0.95 * exp),
        ("/result/position >= 95 % узлов сетки и <= /result/velocity",
         f"{p.get('count')}/{exp} (скорость {o.get('count')})",
         exp > 0 and 0.95 * exp <= (p.get("count") or 0) <= (o.get("count") or 0)),
        ("частота по меткам >= 19 Гц", f"{o.get('rate_stamp_hz')}", (o.get("rate_stamp_hz") or 0) >= 19),
        ("частота по стенным часам >= 19 Гц", f"{o.get('rate_wall_hz')}", (o.get("rate_wall_hz") or 0) >= 19),
        ("in2out p99 < 100 мс (без первых 2 с)",
         f"p50 {st.get('p50')} / p95 {st.get('p95')} / p99 {st.get('p99')} / max {st.get('max')}",
         st.get("n", 0) > 0 and st["p99"] < 100),
        ("0 NaN/inf в выходах", json.dumps(bad), not any(bad.get(k, 0) for k in
                                                          ("vel_nonfinite", "odo_nonfinite", "cov_nonfinite"))),
        ("frame_id: map / base_link", f"{list(fr.get('position', {}))} / {list(fr.get('position_child', {}))}",
         set(fr.get("position", {})) == {"map"} and set(fr.get("position_child", {})) == {"base_link"}),
        ("нода жива до конца bag", "да" if a.alive else "НЕТ", bool(a.alive)),
        ("останов по SIGINT за 15 с", "да" if a.shut else "НЕТ", bool(a.shut)),
    ]
    if a.raw and os.path.exists(a.raw):
        odo = np.load(a.raw)["odo"]
        if len(odo):
            x, z = odo[:, 2], odo[:, 4]
            rows.append(("положение: MGRS от 37UCB, base_link (x 98 800…103 700, z 140…180 м)",
                         f"x {x.min():.1f}…{x.max():.1f}, z {z.min():.1f}…{z.max():.1f}",
                         bool(X_RANGE[0] <= x.min() and x.max() <= X_RANGE[1]
                              and Z_RANGE[0] <= z.min() and z.max() <= Z_RANGE[1])))
    pos = None
    if a.fixture and a.raw:
        pos = position(a.raw, a.fixture)
        val = json.dumps(pos, ensure_ascii=False)
        ok = pos["aligned"] and pos.get("pairs", 0) > 0 and pos.get("mean_m", 1e9) < 10.0
        if a.need_position:
            rows.append(("выставка прошла, ср. 3D < 10 м против GNSS фикстуры", val, ok))
        if pos.get("frame") == "mgrs" and len(pos.get("squares", [])) == 1:
            # кусок в одном квадрате: пара в разных квадратах — ошибка ~100 км у судьи
            rows.append(("MGRS: выход и эталон в одном 100-км квадрате",
                         f"пар в разных квадратах {pos['square_mismatch']} из {pos['pairs']}",
                         pos["square_mismatch"] == 0))
    info = [("in2out p99 с учётом старта (справочно)", f"p99 {al.get('p99')} / max {al.get('max')}"),
            ("трассировки при останове (справочно, WP3)", str(a.trace)),
            ("CPU ноды, % ядра", str(R.get("node_process", {}).get("cpu_pct_1core"))),
            ("RSS ноды, МБ (первый/последний/макс)", str(R.get("node_process", {}).get("rss_mb_first_last_max"))),
            ("точность по GNSS из bag (санити; положение — против АНТЕННЫ master: ~10 м — плечо до base_link, не ошибка)",
             json.dumps({k: v for k, v in R.get("accuracy_sanity_vs_bag_gnss", {}).items() if k != "note"},
                        ensure_ascii=False))]
    if pos is not None and not a.need_position:
        info.append(("положение против всего GNSS фикстуры (справочно)", json.dumps(pos, ensure_ascii=False)))
    print("| проверка | значение | итог |\n|---|---|---|")
    for name, val, ok in rows:
        print(f"| {name} | {val} | {'PASS' if ok else 'FAIL'} |")
    for name, val in info:
        print(f"| {name} | {val} | — |")
    passed = all(r[2] for r in rows)
    loss = {}
    if a.bag_meta and os.path.exists(a.bag_meta):
        got = R.get("inputs", {})
        loss = {k: f"{got.get(k, 0)}/{n}" for k, n in bag_counts(a.bag_meta).items()
                if got.get(k, 0) < n}
        print(f"| проба получила все входы bag (обвязка, справочно) | "
              f"{'да' if not loss else 'потеряно: ' + json.dumps(loss)} | — |")
    verdict = "PASS" if passed else ("НЕ ЗАСЧИТАН" if loss else "FAIL")
    note = (" (проба потеряла начало bag: гонка обнаружения DDS на старте проигрывания, риск 16)"
            if loss and not passed else "")
    print(f"\n[smoke] ИТОГ: {verdict}{note}")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump({"pass": passed, "verdict": verdict, "harness_loss": loss,
                       "checks": [{"check": n, "value": v, "pass": bool(k)} for n, v, k in rows],
                       "position": pos, "in2out_steady_ms": st, "in2out_all_ms": al,
                       "outputs": o.get("count"), "expected": exp}, fh, ensure_ascii=False, indent=1)
    return 0 if passed else (3 if loss else 1)


if __name__ == "__main__":
    sys.exit(main())
