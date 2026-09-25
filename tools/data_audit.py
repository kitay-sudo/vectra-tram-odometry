#!/usr/bin/env python3
"""Аудит датасета кейса «Резервная одометрия по модели» (шаг 1 плана).

Читает все rosbag2 (sqlite3) из data/ библиотекой rosbags (без ROS),
считает по каждому прогону и топику: длительность, число сообщений,
частоты, header.stamp против времени записи в bag, разрывы, GNSS
(наличие, статусы, ковариации, высота), движение на старте, отношение
«тележки / GNSS» (проверка единиц), ручку, аномалии тележек, пройденный
путь, дубликаты прогонов и перекрытия, кластеры маршрутов, утечку
отложенных прогонов партнёра (analysis/calib_drive.split) в обучение.

Запуск (в образе vectra/tram:dev, из корня репозитория):

    docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev \
        python3 tools/data_audit.py [--workers 6] [--bags ID ...]

Выход (out/data/): bags.csv (строка на прогон), topics.csv (прогон × топик),
pairs.csv (перекрывающиеся/одинаковые прогоны), summary.json (агрегаты),
bags_table.md (компактная таблица для docs/DATA.md), routes.png, handle_hist.png.
Детерминирован: порядок прогонов отсортирован, случайности нет.
"""

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import math
import os
import sqlite3
import sys
from multiprocessing import Pool
from pathlib import Path

import warnings

import numpy as np

# пустые срезы и деление на 0 в коротких прогонах дают NaN — это ожидаемо
warnings.filterwarnings("ignore", category=RuntimeWarning)
np.seterr(all="ignore")

REPO = Path(__file__).resolve().parents[1]
DATA = REPO / "data"
OUT = REPO / "out" / "data"
MSG_DIR = REPO / "ros2_ws" / "src" / "tram_vehicle_msgs" / "msg"

TOPICS = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
    "/vehicle/driver_position_cmd": "cmd",
    "/sensing/gnss/master/fix": "mfix",
    "/sensing/gnss/master/vel": "mvel",
    "/sensing/gnss/rover/fix": "rfix",
    "/sensing/gnss/rover/vel": "rvel",
}
KEYS = list(TOPICS.values())
INPUTS = ("front", "rear", "cmd")
KMH = 3.6                      # гипотеза «тележки в км/ч» проверяется ниже (ratio_*)
MOVE_MS = 0.3                  # порог «едет» по ТЗ аудита, м/с
LAT0, LON0 = 55.80484, 37.42050  # как в analysis/build_map.py
R_EARTH = 6378137.0
CELL_M = 25.0                  # клетка для сравнения маршрутов
MSK = dt.timezone(dt.timedelta(hours=3))


# ---------------------------------------------------------------- чтение

def typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for name in ("VelocitySensor", "DriverControllerCommand"):
        text = (MSG_DIR / f"{name}.msg").read_text(encoding="utf-8")
        add.update(get_types_from_msg(text, f"tram_vehicle_msgs/msg/{name}"))
    ts.register(add)
    return ts


def h64(raw):
    return int.from_bytes(hashlib.blake2b(bytes(raw), digest_size=8).digest(), "little")


def read_bag(bag_dir, ts):
    """Все сообщения 7 топиков: словарь key -> dict массивов + хэши."""
    from rosbags.rosbag2 import Reader
    rows = {k: collections.defaultdict(list) for k in KEYS}
    frames = {k: collections.Counter() for k in KEYS}
    th_topic = {k: hashlib.sha1() for k in KEYS}
    h_full = hashlib.sha1()
    front_h = []
    other_topics = collections.Counter()
    with Reader(bag_dir) as reader:
        for conn, t, raw in reader.messages():
            key = TOPICS.get(conn.topic)
            if key is None:
                other_topics[conn.topic] += 1
                continue
            raw = bytes(raw)
            h_full.update(conn.topic.encode()); h_full.update(t.to_bytes(8, "little")); h_full.update(raw)
            th_topic[key].update(raw)
            m = ts.deserialize_cdr(raw, conn.msgtype)
            r = rows[key]
            r["tb"].append(t)
            r["th"].append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
            frames[key][m.header.frame_id] += 1
            if key in ("front", "rear"):
                r["v"].append(m.velocity)
                if key == "front":
                    front_h.append(h64(raw))
            elif key == "cmd":
                r["v"].append(m.position)
            elif key in ("mfix", "rfix"):
                r["lat"].append(m.latitude); r["lon"].append(m.longitude); r["alt"].append(m.altitude)
                r["st"].append(m.status.status); r["sv"].append(m.status.service)
                r["ct"].append(m.position_covariance_type)
                r["cnz"].append(bool(np.any(np.asarray(m.position_covariance) != 0)))
            else:
                lv, av = m.twist.linear, m.twist.angular
                r["x"].append(lv.x); r["y"].append(lv.y); r["z"].append(lv.z)
                r["ang"].append(abs(av.x) + abs(av.y) + abs(av.z))
    out = {}
    for k in KEYS:
        d = {}
        for f, vals in rows[k].items():
            d[f] = np.array(vals, dtype=np.int64 if f in ("tb", "th") else float)
        if "tb" not in d:
            d["tb"] = np.zeros(0, np.int64); d["th"] = np.zeros(0, np.int64)
        out[k] = d
    inp = hashlib.sha1("".join(th_topic[k].hexdigest() for k in INPUTS).encode()).hexdigest()
    hashes = dict(full=h_full.hexdigest(), inputs=inp,
                  topics={k: th_topic[k].hexdigest() for k in KEYS})
    return out, frames, hashes, np.array(sorted(set(front_h)), dtype=np.uint64), other_topics


def sqlite_order(bag_dir, bag_id):
    """Сколько строк messages записано не по возрастанию timestamp (порядок id)."""
    db = bag_dir / f"{bag_id}_0.db3"
    c = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        n = c.execute("SELECT COUNT(*) FROM (SELECT timestamp - LAG(timestamp) OVER (ORDER BY id) AS d "
                      "FROM messages) WHERE d < 0").fetchone()[0]
    finally:
        c.close()
    return int(n)


def read_meta(bag_dir):
    import yaml
    y = yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8"))
    info = y["rosbag2_bagfile_information"]
    pubs = {}
    for tm in info["topics_with_message_count"]:
        md = tm["topic_metadata"]
        q = md.get("offered_qos_profiles") or ""
        pubs[md["name"]] = q.count("- history")
    return dict(dur_ns=int(info["duration"]["nanoseconds"]),
                t0_ns=int(info["starting_time"]["nanoseconds_since_epoch"]),
                n=int(info["message_count"]), version=info.get("version"),
                topics=sorted(pubs), publishers=pubs)


# ---------------------------------------------------------------- помощники

def enu_xy(lat, lon):
    k = math.cos(math.radians(LAT0))
    return np.radians(lon - LON0) * R_EARTH * k, np.radians(lat - LAT0) * R_EARTH


def runs_of(mask):
    """Пары (начало, конец включительно) подряд идущих True."""
    if not len(mask):
        return []
    m = np.r_[False, mask, False].astype(np.int8)
    d = np.diff(m)
    s = np.flatnonzero(d == 1)
    e = np.flatnonzero(d == -1) - 1
    return list(zip(s, e))


def fnum(x, nd=3):
    if x is None:
        return ""
    if isinstance(x, (bool, np.bool_)):
        return int(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, float) or isinstance(x, np.floating):
        if not np.isfinite(x):
            return ""
        return round(float(x), nd)
    return x


def best_lag(t_ref, y_ref, t_x, y_x, mask, lags, scale=True):
    """Лаг L, при котором y_x(t_ref + L) (после масштаба) ближе всего к y_ref.

    L > 0: одно и то же событие в x помечено на L позже, чем в ref.
    """
    best = (np.inf, np.nan, np.nan)
    if mask.sum() < 50:
        return best
    for L in lags:
        tq = t_ref[mask] + L
        ok = (tq >= t_x[0]) & (tq <= t_x[-1])
        if ok.sum() < 50:
            continue
        x = np.interp(tq[ok], t_x, y_x)
        y = y_ref[mask][ok]
        k = float(np.dot(x, y) / max(np.dot(x, x), 1e-12)) if scale else 1.0
        rms = float(np.sqrt(np.mean((k * x - y) ** 2)))
        if rms < best[0]:
            best = (rms, float(L), k)
    return best


# ---------------------------------------------------------------- анализ прогона

def topic_stats(d, t0, t_end):
    tb, th = d["tb"], d["th"]
    n = len(tb)
    s = dict(n=n)
    if n == 0:
        return s
    zero = th == 0
    s["hdr_zero"] = int(zero.sum())
    s["first_s"] = (tb[0] - t0) / 1e9
    s["tail_gap_s"] = (t_end - tb[-1]) / 1e9
    if n > 1:
        dtb = np.diff(tb) / 1e9
        dth = np.diff(th) / 1e9
        span = (tb[-1] - tb[0]) / 1e9
        s["rate_mean_hz"] = (n - 1) / span if span > 0 else np.nan
        md = float(np.median(dtb))
        s["rate_med_bag_hz"] = 1.0 / md if md > 0 else np.nan
        mh = float(np.median(dth))
        s["rate_med_hdr_hz"] = 1.0 / mh if mh > 0 else np.nan
        s["dt_p99_bag_s"] = float(np.percentile(dtb, 99))
        s["max_gap_bag_s"] = float(dtb.max())
        s["gaps05_bag"] = int((dtb > 0.5).sum())
        s["gaps1_bag"] = int((dtb > 1.0).sum())
        s["max_gap_hdr_s"] = float(dth.max())
        s["gaps05_hdr"] = int((dth > 0.5).sum())
        s["hdr_back"] = int((dth < 0).sum())
        s["hdr_back_max_s"] = float(-dth.min()) if (dth < 0).any() else 0.0
        s["hdr_dup"] = int((dth == 0).sum())
        s["bag_dup"] = int((dtb == 0).sum())
    nz = ~zero
    if nz.any():
        off = (tb[nz] - th[nz]) / 1e9
        s["off_med_s"] = float(np.median(off))
        s["off_min_s"] = float(off.min())
        s["off_p01_s"] = float(np.percentile(off, 1))
        s["off_p99_s"] = float(np.percentile(off, 99))
        s["off_max_s"] = float(off.max())
        s["off_std_s"] = float(off.std())
        big = off > 0.5
        s["off_gt05"] = int(big.sum())
        s["off_gt05_last_s"] = float((tb[nz][big].max() - t0) / 1e9) if big.any() else 0.0
        # g — отклонение смещения «запись − метка» от медианы топика:
        # g > 0,5 — метка старше обычного (хвост буфера на старте или сдвиг −1 с),
        # g < −0,5 — метка «из будущего» (сдвиг +1 с)
        g = off - np.median(off)
        trel = (tb[nz] - t0) / 1e9
        start = trel <= 2.0
        s["start_backlog_n"] = int(np.sum(start & (g > 0.5)))
        s["start_backlog_s"] = float(g[start].max()) if start.any() else 0.0
        s["glitch_ahead"] = int(np.sum(~start & (g < -0.5)))
        s["glitch_behind"] = int(np.sum(~start & (g > 0.5)))
        gl = ~start & (np.abs(g) > 0.5)
        s["glitch_first_s"] = float(trel[gl].min()) if gl.any() else np.nan
        s["glitch_last_s"] = float(trel[gl].max()) if gl.any() else np.nan
        s["glitch_absmax_s"] = float(np.abs(g[gl]).max()) if gl.any() else 0.0
        # доля меток, кратных 0,1 с (эпохи GNSS)
        s["hdr_q100ms"] = float(np.mean(th[nz] % 100_000_000 == 0))
    return s


def fresh_mask(t, t_o, max_dt=0.3):
    """True там, где у другого ряда есть отсчёт не дальше max_dt слева и справа."""
    j = np.searchsorted(t_o, t)
    left = np.where(j > 0, t - t_o[np.clip(j - 1, 0, len(t_o) - 1)], np.inf)
    right = np.where(j < len(t_o), t_o[np.clip(j, 0, len(t_o) - 1)] - t, np.inf)
    return (left <= max_dt) & (right <= max_dt)


def stuck_runs(t, v, t_o, v_o, min_s=2.0):
    """Застывание: одинаковое ненулевое показание >= min_s, пока другая тележка меняется > 2 км/ч."""
    if len(v) < 3:
        return 0, 0.0, 0.0, np.nan
    same = np.r_[False, v[1:] == v[:-1]]
    cnt, mx, tot, val = 0, 0.0, 0.0, np.nan
    for s, e in runs_of(same):
        a = s - 1                       # первое значение серии
        dur = t[e] - t[a]
        if dur < min_s or v[a] == 0:
            continue
        seg = (t_o >= t[a]) & (t_o <= t[e])
        if seg.sum() < 3:
            continue
        if np.ptp(v_o[seg]) > 2.0:
            cnt += 1
            tot += dur
            if dur > mx:
                mx, val = dur, float(v[a])
    return cnt, mx, tot, val


def zero_while_other(t, v, t_o, v_o, other_kmh=5.0):
    if len(v) < 3 or len(v_o) < 3:
        return 0.0, 0.0, 0
    vo = np.interp(t, t_o, v_o)
    m = (v == 0) & (vo > other_kmh) & fresh_mask(t, t_o)
    tot, mx, ep = 0.0, 0.0, 0
    for s, e in runs_of(m):
        # длительность по меткам, без провалов связи (каждый отсчёт ≤ 0,5 с)
        dur = float(np.sum(np.minimum(np.diff(t[s:min(e + 2, len(t))]), 0.5))) if e + 1 < len(t) else 0.1
        tot += dur
        mx = max(mx, dur)
        if dur >= 1.0:
            ep += 1
    return tot, mx, ep


def analyze(bag_id):
    ts = typestore()
    bag_dir = DATA / bag_id
    meta = read_meta(bag_dir)
    A, frames, hashes, front_h, other = read_bag(bag_dir, ts)
    unordered = sqlite_order(bag_dir, bag_id)

    all_tb = np.concatenate([A[k]["tb"] for k in KEYS if len(A[k]["tb"])])
    t0, t_end = int(all_tb.min()), int(all_tb.max())
    row = dict(bag=bag_id, vehicle=bag_id.split("_")[0])
    row["start_msk"] = dt.datetime.fromtimestamp(t0 / 1e9, MSK).strftime("%Y-%m-%d %H:%M:%S")
    row["dur_meta_s"] = meta["dur_ns"] / 1e9
    row["dur_msgs_s"] = (t_end - t0) / 1e9
    row["n_msgs"] = int(sum(len(A[k]["tb"]) for k in KEYS))
    row["n_meta"] = meta["n"]
    row["t0_meta_match"] = int(meta["t0_ns"] == t0)
    row["sqlite_unordered"] = unordered
    row["other_topics"] = ";".join(f"{k}:{v}" for k, v in sorted(other.items()))
    row["topic_set_ok"] = int(set(meta["topics"]) == set(TOPICS))
    row["cmd_publishers"] = meta["publishers"].get("/vehicle/driver_position_cmd", 0)

    trows = []
    tstat = {}
    for tp, k in TOPICS.items():
        s = topic_stats(A[k], t0, t_end)
        tstat[k] = s
        fr = ";".join(f"{f or '<empty>'}:{c}" for f, c in sorted(frames[k].items()))
        trows.append(dict(bag=bag_id, topic=tp, frame_ids=fr, sha1=hashes["topics"][k][:12],
                          **{a: fnum(b, 4) for a, b in s.items()}))
        row[f"n_{k}"] = s["n"]
    for k in KEYS:
        s = tstat[k]
        row[f"rate_{k}"] = s.get("rate_mean_hz", np.nan)
        row[f"off_{k}"] = s.get("off_med_s", np.nan)
    for k in INPUTS:
        s = tstat[k]
        row[f"maxgap_{k}"] = s.get("max_gap_bag_s", np.nan)
        row[f"gaps05_{k}"] = s.get("gaps05_bag", 0)
        row[f"hdrback_{k}"] = s.get("hdr_back", 0)
        row[f"hdrdup_{k}"] = s.get("hdr_dup", 0)
    row["hdr_zero_total"] = int(sum(tstat[k].get("hdr_zero", 0) for k in KEYS))
    row["backlog_msgs"] = int(sum(tstat[k].get("off_gt05", 0) for k in KEYS))
    row["backlog_last_s"] = float(max(tstat[k].get("off_gt05_last_s", 0.0) for k in KEYS))
    row["off_max_inputs_s"] = float(max(tstat[k].get("off_max_s", 0.0) for k in INPUTS))
    row["hdr_back_total"] = int(sum(tstat[k].get("hdr_back", 0) for k in KEYS))

    # ------------------------------------------------ задержки входов в порядке записи
    # Так их увидит нода, которая шагает по header.stamp в порядке прихода
    # (runner._advance): «опоздавшее» сообщение — метка меньше уже достигнутой,
    # «прыжок» — метка сразу на > 0,5 с впереди уже достигнутой.
    ev_tb = np.concatenate([A[k]["tb"] for k in INPUTS])
    ev_th = np.concatenate([A[k]["th"] for k in INPUTS])
    if len(ev_tb) > 10:
        o = np.argsort(ev_tb, kind="stable")
        tb_s, th_s = (ev_tb[o] - t0) / 1e9, (ev_th[o] - t0) / 1e9
        runmax = np.maximum.accumulate(th_s)
        late = runmax - th_s
        st = tb_s <= 2.0
        row["late_start_n"] = int(np.sum(st & (late > 0.05)))
        row["late_start_max_s"] = float(late[st].max()) if st.any() else 0.0
        row["late_mid_n"] = int(np.sum(~st & (late > 0.05)))
        row["late_mid_max_s"] = float(late[~st].max()) if (~st).any() else 0.0
        jump = np.diff(runmax)
        row["leap_mid_n"] = int(np.sum((jump > 0.5) & ~st[1:]))
        row["leap_mid_max_s"] = float(jump[~st[1:]].max()) if (~st[1:]).any() else 0.0
    for k in INPUTS:
        row[f"glitch_{k}"] = tstat[k].get("glitch_ahead", 0) + tstat[k].get("glitch_behind", 0)
    row["glitch_gnss"] = int(sum(tstat[k].get("glitch_ahead", 0) + tstat[k].get("glitch_behind", 0)
                                 for k in ("mfix", "mvel", "rfix", "rvel")))
    gl = [tstat[k].get("glitch_last_s", np.nan) - tstat[k].get("glitch_first_s", np.nan)
          for k in ("mfix", "mvel", "rfix", "rvel")]
    gl = [x for x in gl if np.isfinite(x)]
    row["glitch_gnss_span_s"] = float(max(gl)) if gl else 0.0
    row["start_backlog_inputs_s"] = float(max(tstat[k].get("start_backlog_s", 0.0) for k in INPUTS))
    row["start_backlog_msgs"] = int(sum(tstat[k].get("start_backlog_n", 0) for k in KEYS))

    # ------------------------------------------------ тележки и ручка
    F, Rr, C = A["front"], A["rear"], A["cmd"]
    rel = lambda a: (a - t0) / 1e9                                   # noqa: E731

    def ctime(d, *fields):
        """Исправленное время метки: сбои ±1 с после старта заменены на
        «время записи − медианное смещение»; ряд отсортирован по времени."""
        if not len(d["tb"]):
            return (np.zeros(0),) + tuple(np.zeros(0) for _ in fields)
        off = (d["tb"] - d["th"]) / 1e9
        med = np.median(off)
        tb_r, th_r = rel(d["tb"]), rel(d["th"])
        bad = (np.abs(off - med) > 0.5) & (tb_r > 2.0)
        t = np.where(bad, tb_r - med, th_r)
        o = np.argsort(t, kind="stable")
        return (t[o],) + tuple(d[f][o] for f in fields)

    tf_h, vf = ctime(F, "v") if "v" in F else (np.zeros(0), np.zeros(0))
    tr_h, vr = ctime(Rr, "v") if "v" in Rr else (np.zeros(0), np.zeros(0))
    tc_h, vc = ctime(C, "v") if "v" in C else (np.zeros(0), np.zeros(0))
    of, orr = np.argsort(F["tb"], kind="stable"), np.argsort(Rr["tb"], kind="stable")
    tf_b, tr_b = rel(F["tb"])[of], rel(Rr["tb"])[orr]
    vf_b = F["v"][of] if "v" in F else np.zeros(0)
    vr_b = Rr["v"][orr] if "v" in Rr else np.zeros(0)
    for nm, v in (("front", vf), ("rear", vr)):
        row[f"nan_{nm}"] = int(np.isnan(v).sum())
        row[f"inf_{nm}"] = int(np.isinf(v).sum())
        row[f"neg_{nm}"] = int((v < 0).sum())
        row[f"min_{nm}"] = float(np.nanmin(v)) if len(v) else np.nan
        row[f"max_{nm}_kmh"] = float(np.nanmax(v)) if len(v) else np.nan

    # движение на старте (время записи от начала bag)
    for w in (3.0, 10.0):
        vals = np.r_[vf_b[tf_b <= w], vr_b[tr_b <= w]]
        vmax = float(vals.max()) / KMH if len(vals) else np.nan
        row[f"vmax{int(w)}_ms"] = vmax
        row[f"moving{int(w)}"] = int(vmax > MOVE_MS) if np.isfinite(vmax) else ""

    # ручка
    if len(vc):
        row["handle_min"] = int(vc.min()); row["handle_max"] = int(vc.max())
        row["handle_changes"] = int((np.diff(vc) != 0).sum())
        row["handle_frac0"] = float(np.mean(vc == 0))
        row["handle_start"] = int(C["v"][np.argmin(C["tb"])])
        hist = collections.Counter(int(x) for x in vc)
    else:
        hist = collections.Counter()

    # провалы связи тележек (после исправления меток)
    for nm, t in (("front", tf_h), ("rear", tr_h)):
        d = np.diff(t) if len(t) > 1 else np.zeros(0)
        row[f"drop_{nm}_s"] = float(d[d > 0.5].sum())
        row[f"drop_{nm}_max_s"] = float(d.max()) if len(d) else np.nan

    # сетка 0,1 с по header.stamp (время ноды) для путей и противоречий
    have_b = len(vf) > 5 and len(vr) > 5
    extra = dict(handle_hist=dict(hist), front_h=front_h)
    if have_b:
        g0 = max(tf_h[0], tr_h[0]); g1 = min(tf_h[-1], tr_h[-1])
        grid = np.arange(g0, g1, 0.1)
        vfg = np.interp(grid, tf_h, vf) / KMH
        vrg = np.interp(grid, tr_h, vr) / KMH
        vb = 0.5 * (vfg + vrg)
        row["wheel_km"] = float(np.sum(vb) * 0.1 / 1000)
        # ручка против ускорения
        if len(vc) > 5 and len(grid) > 20:
            ci = np.clip(np.searchsorted(tc_h, grid, side="right") - 1, 0, len(vc) - 1)
            cg = vc[ci]
            acc = np.full_like(vb, np.nan)
            acc[5:-5] = (vb[10:] - vb[:-10]) / 1.0
            m = (cg <= -8) & (acc > 0.3) & (vb > 1.0)
            row["brake_acc_s"] = float(m.sum() * 0.1)
            ep = [(s_, e_) for s_, e_ in runs_of(m) if (e_ - s_ + 1) * 0.1 >= 2.0]
            row["brake_acc_ep2s"] = len(ep)
            row["brake_acc_ep2s_s"] = float(sum((e_ - s_ + 1) * 0.1 for s_, e_ in ep))
            m2 = (cg >= 1) & (acc < -0.5) & (vb > 1.0)
            row["tract_dec_s"] = float(m2.sum() * 0.1)
            # трогания с места: знак ручки в окне [-1; +3] с от пересечения 0,5 м/с
            pos = neg = zer = 0
            k_ = 20
            while k_ < len(vb) - 100:
                if vb[k_] >= 0.5 > vb[k_ - 1] and np.all(vb[k_ - 20:k_ - 8] < 0.2) and np.any(vb[k_:k_ + 100] > 3.0):
                    med = float(np.median(cg[k_ - 10:k_ + 30]))
                    if med > 0:
                        pos += 1
                    elif med < 0:
                        neg += 1
                    else:
                        zer += 1
                    k_ += 100
                else:
                    k_ += 1
            row["starts_pos"], row["starts_neg"], row["starts_zero"] = pos, neg, zer
        # расхождение передняя/задняя (только там, где у задней свежие отсчёты)
        vri = np.interp(tf_h, tr_h, vr)
        fr_ok = fresh_mask(tf_h, tr_h)
        dd = vf - vri
        mv = (np.maximum(vf, vri) > 5.0) & fr_ok
        if mv.sum() > 10:
            ad = np.abs(dd[mv])
            row["fr_med_abs_kmh"] = float(np.median(ad))
            row["fr_p99_abs_kmh"] = float(np.percentile(ad, 99))
            row["fr_max_abs_kmh"] = float(ad.max())
            row["fr_frac_gt2kmh"] = float(np.mean(ad > 2.0))
            row["fr_bias_kmh"] = float(np.median(dd[mv]))
        # эпизоды расхождения > 3 км/ч (юз/буксование одной тележки)
        slip = (np.abs(dd) > 3.0) & fr_ok & (np.maximum(vf, vri) > 1.0)
        ep = [(s_, e_) for s_, e_ in runs_of(slip) if tf_h[e_] - tf_h[s_] >= 0.2]
        row["slip_ep"] = len(ep)
        row["slip_s"] = float(sum(tf_h[e_] - tf_h[s_] + 0.1 for s_, e_ in ep))
        row["slip_max_kmh"] = float(np.abs(dd[slip]).max()) if slip.any() else 0.0
        # застывание и ноль при движении другой
        for nm, (t, v, to, vo) in (("front", (tf_h, vf, tr_h, vr)), ("rear", (tr_h, vr, tf_h, vf))):
            c, mx, tot, val = stuck_runs(t, v, to, vo)
            row[f"stuck_{nm}_n"] = c; row[f"stuck_{nm}_max_s"] = mx; row[f"stuck_{nm}_val"] = val
            zt, zm, ze = zero_while_other(t, v, to, vo)
            row[f"zero_{nm}_s"] = zt; row[f"zero_{nm}_max_s"] = zm
            dv = np.abs(np.diff(v)) / KMH
            dtt = np.diff(t)
            ok = dtt > 0.02
            row[f"spikes_{nm}"] = int(np.sum(dv[ok] / dtt[ok] > 3.0))
        # лаг передняя-задняя (обе шкалы)
        lags = np.round(np.arange(-3.0, 3.0001, 0.05), 3)
        for base, (a_t, a_v, b_t, b_v) in (("hdr", (tr_h, vr, tf_h, vf)), ("bag", (tr_b, vr_b, tf_b, vf_b))):
            m = a_v > 5.0
            _, L, _ = best_lag(a_t, a_v, b_t, b_v, m, lags, scale=False)
            row[f"lag_front_rear_{base}_s"] = L

    # ------------------------------------------------ GNSS
    gn = {}
    for rx, kf, kv in (("master", "mfix", "mvel"), ("rover", "rfix", "rvel")):
        fx, vl = A[kf], A[kv]
        g = dict(n_fix=len(fx["tb"]), n_vel=len(vl["tb"]))
        if len(fx["tb"]):
            tb = np.sort(rel(fx["tb"]))
            g["first_s"] = float(tb[0]); g["last_s"] = float(tb[-1]); g["span_s"] = float(tb[-1] - tb[0])
            g["status"] = ";".join(f"{int(k)}:{v}" for k, v in sorted(collections.Counter(fx["st"].astype(int)).items()))
            g["service"] = ";".join(f"{int(k)}:{v}" for k, v in sorted(collections.Counter(fx["sv"].astype(int)).items()))
            g["cov_type"] = ";".join(f"{int(k)}:{v}" for k, v in sorted(collections.Counter(fx["ct"].astype(int)).items()))
            g["cov_nonzero"] = int(fx["cnz"].sum())
            g["st2_frac"] = float(np.mean(fx["st"] == 2))
            fin = np.isfinite(fx["lat"]) & np.isfinite(fx["lon"]) & np.isfinite(fx["alt"])
            g["nonfinite"] = int((~fin).sum())
            g["alt_min"] = float(fx["alt"][fin].min()); g["alt_max"] = float(fx["alt"][fin].max())
            g["lat_min"] = float(fx["lat"][fin].min()); g["lat_max"] = float(fx["lat"][fin].max())
            g["lon_min"] = float(fx["lon"][fin].min()); g["lon_max"] = float(fx["lon"][fin].max())
            ts_, lat, lon, st_ = ctime(fx, "lat", "lon", "st")
            xs, ys = enu_xy(lat, lon)
            step = np.hypot(np.diff(xs), np.diff(ys))
            dtt = np.diff(ts_)
            jump = step > np.maximum(30.0 * np.maximum(dtt, 0.1), 5.0)
            g["jumps"] = int(jump.sum())
            g["path_km"] = float(step[~jump].sum() / 1000)
            g["x"], g["y"], g["t"], g["st"] = xs, ys, ts_, st_
        if len(vl["tb"]):
            tv, vx, vy, vz = ctime(vl, "x", "y", "z")
            sp = np.hypot(vx, vy)
            g["vmax"] = float(sp.max())
            g["vel_km"] = float(np.sum(0.5 * (sp[1:] + sp[:-1]) * np.clip(np.diff(tv), 0, 1.0)) / 1000)
            g["vz_absmax"] = float(np.abs(vz).max())
            g["ang_nonzero"] = int((vl["ang"] > 0).sum())
            ob = np.argsort(vl["tb"], kind="stable")
            g["sp"], g["tv"], g["ve"], g["vn"] = sp, tv, vx, vy
            g["tv_b"], g["sp_b"] = rel(vl["tb"])[ob], np.hypot(vl["x"], vl["y"])[ob]
        gn[rx] = g
    M = gn["master"]
    row["gnss_any"] = int(any(gn[r]["n_fix"] + gn[r]["n_vel"] > 0 for r in gn))
    for rx in ("master", "rover"):
        g = gn[rx]
        p = "m" if rx == "master" else "r"
        row[f"{p}fix_first_s"] = g.get("first_s", np.nan)
        row[f"{p}fix_span_s"] = g.get("span_s", np.nan)
        row[f"{p}fix_status"] = g.get("status", "")
        row[f"{p}fix_service"] = g.get("service", "")
        row[f"{p}fix_covtype"] = g.get("cov_type", "")
        row[f"{p}fix_cov_nonzero"] = g.get("cov_nonzero", "")
        row[f"{p}fix_jumps"] = g.get("jumps", "")
        row[f"{p}fix_nonfinite"] = g.get("nonfinite", "")
        row[f"{p}fix_st2_frac"] = g.get("st2_frac", np.nan)
    row["alt_min"] = M.get("alt_min", np.nan); row["alt_max"] = M.get("alt_max", np.nan)
    row["lat_min"] = M.get("lat_min", np.nan); row["lat_max"] = M.get("lat_max", np.nan)
    row["lon_min"] = M.get("lon_min", np.nan); row["lon_max"] = M.get("lon_max", np.nan)
    row["gnss_km"] = M.get("path_km", np.nan)
    row["gnss_vel_km"] = M.get("vel_km", np.nan)
    row["gnss_vmax_ms"] = M.get("vmax", np.nan)
    row["mvel_vz_absmax"] = M.get("vz_absmax", np.nan)
    row["mvel_ang_nonzero"] = M.get("ang_nonzero", "")

    # база master–rover
    R = gn["rover"]
    if "x" in M and "x" in R and len(M["t"]) > 10 and len(R["t"]) > 10:
        rx_ = np.interp(M["t"], R["t"], R["x"]); ry_ = np.interp(M["t"], R["t"], R["y"])
        inside = (M["t"] >= R["t"][0]) & (M["t"] <= R["t"][-1])
        b = np.hypot(rx_ - M["x"], ry_ - M["y"])[inside]
        if len(b):
            row["baseline_med_m"] = float(np.median(b))
            row["baseline_p05_m"] = float(np.percentile(b, 5))
            row["baseline_p95_m"] = float(np.percentile(b, 95))

    # тележки против GNSS: единицы, лаг, путь на отрезке GNSS
    if have_b and "sp" in M and len(M["sp"]) > 100:
        lags = np.round(np.arange(-3.0, 3.0001, 0.05), 3)
        for base, (tg, sp, tfx, vfx, trx, vrx) in (
                ("hdr", (M["tv"], M["sp"], tf_h, vf, tr_h, vr)),
                ("bag", (M["tv_b"], M["sp_b"], tf_b, vf_b, tr_b, vr_b))):
            fi = np.interp(tg, tfx, vfx); ri = np.interp(tg, trx, vrx)
            inside = (tg >= max(tfx[0], trx[0])) & (tg <= min(tfx[-1], trx[-1]))
            inside &= fresh_mask(tg, tfx) & fresh_mask(tg, trx)
            m = inside & (sp > 3.0) & (fi > 0) & (ri > 0)
            if m.sum() > 50:
                q = 0.5 * (fi[m] + ri[m]) / sp[m]
                row[f"ratio_front_{base}"] = float(np.median(fi[m] / sp[m]))
                row[f"ratio_rear_{base}"] = float(np.median(ri[m] / sp[m]))
                row[f"ratio_mean_{base}"] = float(np.median(q))
                row[f"ratio_iqr_{base}"] = float(np.subtract(*np.percentile(q, [75, 25])))
            # лаг
            tgrid = np.arange(max(tfx[0], trx[0]), min(tfx[-1], trx[-1]), 0.05)
            vbg = 0.5 * (np.interp(tgrid, tfx, vfx) + np.interp(tgrid, trx, vrx))
            m = inside & (sp > 1.0)
            rms, L, k = best_lag(tg, sp, tgrid, vbg, m, lags, scale=True)
            row[f"lag_bogie_gnss_{base}_s"] = L
            row[f"lag_rms_{base}_ms"] = rms
            rms0, _, _ = best_lag(tg, sp, tgrid, vbg, m, [0.0], scale=True)
            row[f"lag0_rms_{base}_ms"] = rms0
        # независимая проверка масштаба: путь колёс / хорда между точками fix
        # на прямых 10-секундных окнах (не зависит от доплеровской скорости GNSS)
        if "t" in M and len(M["t"]) > 200:
            cum = np.r_[0.0, np.cumsum(vb[:-1] * 0.1)] * KMH          # км/ч·с
            tfx, xs, ys, stt = M["t"], M["x"], M["y"], M["st"]
            byst = collections.defaultdict(list)
            i = 0
            while i < len(tfx):
                j = int(np.searchsorted(tfx, tfx[i] + 10.0))
                if j >= len(tfx):
                    break
                if (tfx[j] - tfx[i] > 10.3 or stt[i] != stt[j] or tfx[i] < grid[0] or tfx[j] > grid[-1]):
                    i += 10
                    continue
                path = float(np.sum(np.hypot(np.diff(xs[i:j + 1]), np.diff(ys[i:j + 1]))))
                chord = float(np.hypot(xs[j] - xs[i], ys[j] - ys[i]))
                if chord > 60.0 and path / chord < 1.002:
                    wd = np.interp(tfx[j], grid, cum) - np.interp(tfx[i], grid, cum)
                    byst[int(stt[i])].append(wd / chord)
                i = j
            for s_ in (0, 2):
                if len(byst.get(s_, [])) >= 5:
                    row[f"ratio_pos_st{s_}"] = float(np.median(byst[s_]))
                    row[f"ratio_pos_st{s_}_n"] = len(byst[s_])
        # отклонение среднего тележек (в м/с по отношению этого прогона) от GNSS
        sp, tv = M["sp"], M["tv"]
        fi = np.interp(tv, tf_h, vf); ri = np.interp(tv, tr_h, vr)
        ok = fresh_mask(tv, tf_h) & fresh_mask(tv, tr_h) & (tv >= g0) & (tv <= g1)
        kk = row.get("ratio_mean_hdr", KMH)
        dev = 0.5 * (fi + ri) / kk - sp
        mm = ok & (np.maximum(sp, 0.5 * (fi + ri) / kk) > 1.0)
        if mm.sum() > 50:
            row["dev_gnss_p99_ms"] = float(np.percentile(np.abs(dev[mm]), 99))
            row["dev_gnss_gt1_s"] = float(np.sum(np.abs(dev[mm]) > 1.0) * 0.1)
            row["dev_gnss_rmse_ms"] = float(np.sqrt(np.mean(dev[mm] ** 2)))
        # отсчёты GNSS со скоростью 0 при езде по тележкам
        row["gnss_zero_drop"] = int(np.sum(ok & (sp == 0) & (np.minimum(fi, ri) > 10.0)))
        # путь колёс на отрезке GNSS master fix
        if "t" in M and len(M["t"]) > 10:
            a, b = max(M["t"][0], g0), min(M["t"][-1], g1)
            sel = (grid >= a) & (grid <= b)
            row["wheel_km_gspan"] = float(np.sum(vb[sel]) * 0.1 / 1000)
    # GNSS-скорость на старте
    if "sp_b" in M:
        w = M["tv_b"] <= 3.0
        row["gnss_vmax3_ms"] = float(M["sp_b"][w].max()) if w.any() else np.nan

    # трек для маршрутов: клетки, прореженная линия и точки на ходу с курсом
    if "x" in M and len(M["x"]) > 50:
        x, y = M["x"], M["y"]
        cells = np.unique((np.floor(x / CELL_M).astype(np.int64) << 32) + np.floor(y / CELL_M).astype(np.int64))
        extra["cells"] = cells
        extra["track"] = np.c_[x[::20], y[::20]].astype(np.float32)
        row["start_xy"] = f"{x[0]:.0f},{y[0]:.0f}"
        row["end_xy"] = f"{x[-1]:.0f},{y[-1]:.0f}"
        dx_ = x[-1] - x[0]
        row["dir"] = "В→З" if dx_ < -2000 else ("З→В" if dx_ > 2000 else "—")
        if "sp" in M and len(M["sp"]) > 50:
            ve = np.interp(M["t"], M["tv"], M["ve"]); vn = np.interp(M["t"], M["tv"], M["vn"])
            mv = (np.hypot(ve, vn) > 2.0) & (M["st"] >= 0)
            idx = np.flatnonzero(mv)[::10]
            extra["pts"] = np.c_[x[idx], y[idx], np.arctan2(ve[idx], vn[idx])]
    row["hash_full"] = hashes["full"][:16]
    row["hash_inputs"] = hashes["inputs"][:16]
    extra["t_range"] = (t0, t_end)
    extra["th_front"] = (int(F["th"].min()), int(F["th"].max())) if len(F["th"]) else None
    return row, trows, extra


# ---------------------------------------------------------------- агрегация

def partner_split(ids):
    """Копия analysis/calib_drive.split: отложен каждый пятый, начиная с третьего."""
    val = set(ids[2::5])
    return [b for b in ids if b not in val], sorted(val)


def components(n, edges):
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)
    groups = collections.defaultdict(list)
    for i in range(n):
        groups[find(i)].append(i)
    return sorted(groups.values(), key=lambda g: (-len(g), g[0]))


def write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow({k: fnum(r.get(k), 4) for k in keys})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=min(6, os.cpu_count() or 2))
    ap.add_argument("--bags", nargs="*")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    all_ids = sorted(p.name for p in DATA.iterdir() if (p / "metadata.yaml").exists())
    ids = sorted(args.bags) if args.bags else all_ids
    print(f"прогонов: {len(ids)}, процессов: {args.workers}", flush=True)
    res = []
    with Pool(args.workers) as pool:
        for i, r in enumerate(pool.imap(analyze, ids), 1):
            res.append(r)
            if i % 10 == 0 or i == len(ids):
                print(f"  {i}/{len(ids)}", flush=True)
    rows = [r[0] for r in res]
    trows = [t for r in res for t in r[1]]
    extras = {r[0]["bag"]: r[2] for r in res}

    # --- дубликаты: одинаковые входы / одинаковое всё
    by_full = collections.defaultdict(list)
    by_inp = collections.defaultdict(list)
    for r in rows:
        by_full[r["hash_full"]].append(r["bag"])
        by_inp[r["hash_inputs"]].append(r["bag"])
    dup_groups = sorted([sorted(v) for v in by_inp.values() if len(v) > 1])
    canon = {}
    for grp in [sorted(v) for v in by_inp.values()]:
        for b in grp:
            canon[b] = grp[0]
    # --- перекрытия во времени и по содержимому (передняя тележка)
    pairs = []
    bl = [r["bag"] for r in rows]
    for i in range(len(bl)):
        for j in range(i + 1, len(bl)):
            a, b = bl[i], bl[j]
            if a.split("_")[0] != b.split("_")[0]:
                continue
            ta, tb_ = extras[a]["t_range"], extras[b]["t_range"]
            ov = (min(ta[1], tb_[1]) - max(ta[0], tb_[0])) / 1e9
            ha, hb = extras[a]["front_h"], extras[b]["front_h"]
            common = len(np.intersect1d(ha, hb, assume_unique=True)) if len(ha) and len(hb) else 0
            if ov > 0 or common > 0:
                pairs.append(dict(a=a, b=b, overlap_s=ov, front_common=common,
                                  contain=common / max(1, min(len(ha), len(hb))),
                                  same_full=int(rows[i]["hash_full"] == rows[j]["hash_full"]),
                                  same_inputs=int(rows[i]["hash_inputs"] == rows[j]["hash_inputs"])))
    near = [p for p in pairs if not p["same_inputs"]]

    # --- разбиение партнёра и утечка
    tr, va = partner_split(all_ids)
    dm_path = REPO / "analysis" / "drive_model.json"
    dm_match = None
    if dm_path.exists():
        dm = json.loads(dm_path.read_text(encoding="utf-8"))
        dm_match = (sorted(dm.get("val", [])) == va) and (sorted(dm.get("train", [])) == sorted(tr))
    va_s = set(va)
    for r in rows:
        r["split"] = "val" if r["bag"] in va_s else "train"
        r["dup_of"] = canon[r["bag"]] if canon[r["bag"]] != r["bag"] else ""
    leak = []
    for grp in dup_groups:
        sp = {b: ("val" if b in va_s else "train") for b in grp}
        if "val" in sp.values() and "train" in sp.values():
            leak.extend([b for b in grp if sp[b] == "val"])
    val_both = [g for g in dup_groups if all(b in va_s for b in g)]
    rowd = {r["bag"]: r for r in rows}
    val_gnss = [b for b in va if rowd.get(b, {}).get("n_mvel", 0) >= 50]
    leak_gnss = [b for b in leak if rowd.get(b, {}).get("n_mvel", 0) >= 50]

    # --- маршруты: клетки 25 м по уникальным записям с GNSS
    uniq = sorted({canon[b] for b in bl})
    ug = [b for b in uniq if "cells" in extras[b]]
    J = np.zeros((len(ug), len(ug)))
    for i, a in enumerate(ug):
        for j, b in enumerate(ug):
            if j < i:
                J[i, j] = J[j, i]
                continue
            ca, cb = extras[a]["cells"], extras[b]["cells"]
            inter = len(np.intersect1d(ca, cb, assume_unique=True))
            J[i, j] = inter / max(1, min(len(ca), len(cb)))          # доля меньшего трека
    comp = components(len(ug), [(i, j) for i in range(len(ug)) for j in range(i + 1, len(ug)) if J[i, j] >= 0.5])
    route_of = {}
    routes = []
    for ci, g in enumerate(comp, 1):
        members = [ug[i] for i in g]
        cells = np.unique(np.concatenate([extras[b]["cells"] for b in members]))
        routes.append(dict(route=f"R{ci}", n=len(members),
                           vehicles=dict(collections.Counter(b.split("_")[0] for b in members)),
                           cells=int(len(cells)), members=members))
        for b in members:
            route_of[b] = f"R{ci}"
    for r in rows:
        r["route"] = route_of.get(canon[r["bag"]], "")
    all_cells = np.unique(np.concatenate([extras[b]["cells"] for b in ug])) if ug else np.zeros(0)
    veh_cells = {}
    for v in ("30618", "30639"):
        cs = [extras[b]["cells"] for b in ug if b.startswith(v)]
        veh_cells[v] = np.unique(np.concatenate(cs)) if cs else np.zeros(0, np.int64)
    shared_cells = len(np.intersect1d(veh_cells["30618"], veh_cells["30639"]))

    # --- смещение трека относительно треков 30618 (без самого прогона, тот же курс ±29°)
    ref_b = [b for b in ug if b.startswith("30618") and "pts" in extras[b]]
    for b in ug:
        if "pts" not in extras[b] or len(extras[b]["pts"]) < 20:
            continue
        others = [extras[x]["pts"][::2] for x in ref_b if x != b]     # опорные точки через ~2 с
        if not others:
            continue
        Rp = np.concatenate(others)
        P = extras[b]["pts"]
        cts, des, dns = [], [], []
        for i0 in range(0, len(P), 64):
            q = P[i0:i0 + 64]
            dx = q[:, None, 0] - Rp[None, :, 0]
            dy = q[:, None, 1] - Rp[None, :, 1]
            d2 = dx * dx + dy * dy
            d2 = np.where(np.cos(Rp[None, :, 2] - q[:, None, 2]) > 0.875, d2, np.inf)
            j = np.argmin(d2, axis=1)
            ii = np.arange(len(q))
            ok = d2[ii, j] < 30.0 ** 2
            h = q[:, 2]
            ex, ny = dx[ii, j], dy[ii, j]
            cts.append((-ex * np.cos(h) + ny * np.sin(h))[ok])
            des.append(ex[ok]); dns.append(ny[ok])
        ct, de, dn = np.concatenate(cts), np.concatenate(des), np.concatenate(dns)
        if len(ct) >= 20:
            for bb in [x for x in bl if canon[x] == b]:
                r = rowd[bb]
                r["xt_left_med_m"] = float(np.median(ct))
                r["xt_abs_med_m"] = float(np.median(np.abs(ct)))
                r["xt_east_med_m"] = float(np.median(de))
                r["xt_north_med_m"] = float(np.median(dn))
                r["xt_n"] = int(len(ct))

    # --- статусы NavSatFix по всем прогонам
    st_tot = {}
    for p in ("mfix_status", "rfix_status", "mfix_covtype", "rfix_covtype", "mfix_service", "rfix_service"):
        c = collections.Counter()
        for r in rows:
            for kv in filter(None, str(r.get(p, "")).split(";")):
                k, v = kv.split(":")
                c[int(k)] += int(v)
        st_tot[p] = {int(k): v for k, v in sorted(c.items())}

    # --- ручка: гистограмма по всем и по уникальным
    hist_all, hist_uniq = collections.Counter(), collections.Counter()
    for b in bl:
        hist_all.update(extras[b]["handle_hist"])
        if canon[b] == b:
            hist_uniq.update(extras[b]["handle_hist"])

    # --- агрегаты по топикам
    tagg = {}
    for tp in TOPICS:
        tt = [t for t in trows if t["topic"] == tp and t.get("n", 0)]
        def col(k):
            v = np.array([t[k] for t in tt if t.get(k, "") != ""], float)
            return v
        agg = dict(bags_with=len(tt), msgs=int(sum(t["n"] for t in tt)))
        for k in ("rate_mean_hz", "rate_med_bag_hz", "rate_med_hdr_hz", "off_med_s", "off_min_s", "off_max_s",
                  "off_std_s", "max_gap_bag_s", "gaps05_bag", "hdr_back", "hdr_dup", "hdr_zero", "bag_dup",
                  "hdr_q100ms", "first_s", "off_gt05", "off_gt05_last_s", "off_p99_s"):
            v = col(k)
            if len(v):
                agg[k] = dict(min=float(v.min()), med=float(np.median(v)), max=float(v.max()),
                              sum=float(v.sum()))
        agg["frame_ids"] = sorted({f.split(":")[0] for t in tt for f in t["frame_ids"].split(";") if f})
        tagg[tp] = agg

    def colr(k, subset=None):
        return np.array([r[k] for r in rows if (subset is None or r["bag"] in subset)
                         and isinstance(r.get(k), (int, float, np.floating, np.integer)) and np.isfinite(r[k])], float)
    uniq_s = set(uniq)
    summary = dict(
        n_bags=len(rows), n_unique_inputs=len(by_inp), n_unique_full=len(by_full),
        dup_groups=dup_groups, near_dup_pairs=near,
        total_hours=float(colr("dur_msgs_s").sum() / 3600),
        unique_hours=float(colr("dur_msgs_s", uniq_s).sum() / 3600),
        total_msgs=int(colr("n_msgs").sum()),
        gnss_bags=int(sum(r["gnss_any"] for r in rows)),
        gnss_bags_unique=int(sum(r["gnss_any"] for r in rows if r["bag"] in uniq_s)),
        no_gnss=[r["bag"] for r in rows if not r["gnss_any"]],
        partial_gnss=[r["bag"] for r in rows if r["gnss_any"] and r["n_mfix"] < 0.8 * r["dur_msgs_s"] * 10],
        split=dict(train=len(tr), val=len(va), matches_drive_model_json=dm_match, val_ids=va,
                   val_with_train_duplicate=sorted(leak), val_gnss=len(val_gnss),
                   val_gnss_with_train_duplicate=sorted(leak_gnss), dup_groups_all_val=val_both,
                   unique_val_recordings=len({canon.get(b, b) for b in va if b in canon}),
                   val_unique_not_in_train=sorted({canon.get(b, b) for b in va if b in canon}
                                                  - {canon.get(b, b) for b in tr if b in canon})),
        routes=routes, cells_total=int(len(all_cells)), cells_per_vehicle={k: int(len(v)) for k, v in veh_cells.items()},
        cells_shared_vehicles=int(shared_cells), cell_m=CELL_M,
        handle_hist_all={int(k): v for k, v in sorted(hist_all.items())},
        handle_hist_unique={int(k): v for k, v in sorted(hist_uniq.items())},
        topics=tagg, navsat=st_tot,
        master_fix_bags=[r["bag"] for r in rows if r["n_mfix"] > 0],
        rover_only=[r["bag"] for r in rows if r["n_mfix"] == 0 and r["n_rfix"] > 0],
    )
    for k in ("ratio_mean_hdr", "ratio_mean_bag", "ratio_front_hdr", "ratio_rear_hdr", "lag_bogie_gnss_hdr_s",
              "lag_bogie_gnss_bag_s", "lag_front_rear_hdr_s", "lag_front_rear_bag_s", "lag_rms_hdr_ms",
              "lag0_rms_hdr_ms", "lag_rms_bag_ms", "lag0_rms_bag_ms", "alt_min", "alt_max", "baseline_med_m",
              "wheel_km", "gnss_km", "gnss_vel_km", "dur_msgs_s", "fr_med_abs_kmh", "fr_p99_abs_kmh",
              "backlog_last_s", "backlog_msgs", "off_max_inputs_s", "vmax3_ms", "vmax10_ms", "brake_acc_s",
              "brake_acc_ep2s", "brake_acc_ep2s_s", "starts_pos", "starts_neg", "starts_zero",
              "late_start_n", "late_start_max_s", "late_mid_n", "late_mid_max_s", "leap_mid_n", "leap_mid_max_s",
              "glitch_front", "glitch_rear", "glitch_cmd", "glitch_gnss", "glitch_gnss_span_s",
              "start_backlog_inputs_s", "start_backlog_msgs", "dev_gnss_p99_ms", "dev_gnss_rmse_ms",
              "dev_gnss_gt1_s", "slip_ep", "slip_s", "slip_max_kmh", "drop_front_s", "drop_rear_s",
              "drop_front_max_s", "drop_rear_max_s", "zero_front_s", "zero_rear_s", "spikes_front",
              "spikes_rear", "xt_left_med_m", "xt_abs_med_m", "xt_east_med_m", "xt_north_med_m",
              "rate_front", "rate_rear", "rate_cmd", "rate_mfix", "rate_mvel", "rate_rfix", "rate_rvel",
              "gnss_zero_drop", "mfix_jumps", "rfix_jumps", "handle_frac0", "wheel_km_gspan",
              "ratio_pos_st0", "ratio_pos_st2", "mfix_st2_frac", "rfix_st2_frac"):
        for nm, sub in (("all", None), ("unique", uniq_s)):
            v = colr(k, sub)
            if len(v):
                summary.setdefault("stats", {}).setdefault(k, {})[nm] = dict(
                    n=len(v), min=float(v.min()), p05=float(np.percentile(v, 5)), med=float(np.median(v)),
                    p95=float(np.percentile(v, 95)), max=float(v.max()), sum=float(v.sum()))
        for v_ in ("30618", "30639"):
            v = colr(k, {b for b in uniq_s if b.startswith(v_)})
            if len(v):
                summary["stats"][k][v_] = dict(n=len(v), min=float(v.min()), med=float(np.median(v)),
                                                max=float(v.max()))

    write_csv(out / "bags.csv", rows)
    write_csv(out / "topics.csv", trows)
    write_csv(out / "pairs.csv", pairs)
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, sort_keys=True,
                                                 default=lambda o: o.item() if hasattr(o, "item") else str(o)),
                                      encoding="utf-8")
    write_table(out / "bags_table.md", rows)
    plots(out, rows, extras, route_of, canon, hist_uniq)
    print(json.dumps({k: summary[k] for k in ("n_bags", "n_unique_inputs", "n_unique_full", "gnss_bags",
                                              "gnss_bags_unique", "total_hours", "unique_hours")}, ensure_ascii=False))
    print("записано в", out)


def write_table(path, rows):
    def f(x, nd=1):
        return "" if x is None or x == "" or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{nd}f}"
    hdr = ("| прогон | дубль | split | мин | GNSS | едет к 10 с | км/ч÷GNSS | провал F/R, с | эп. юза | "
           "сбои меток вх./GNSS | путь колёс/GNSS, км | Δh, м | напр. |")
    lines = [hdr, "|" + "---|" * 13]
    for r in rows:
        if r.get("n_mfix"):
            g = f"M {f(r.get('mfix_span_s'), 0)} с"
        elif r.get("n_rfix"):
            g = f"только R {f(r.get('rfix_span_s'), 0)} с"
        else:
            g = "нет"
        mv = "да" if r.get("moving10") == 1 else ("нет" if r.get("moving10") == 0 else "")
        ratio = f(r.get("ratio_mean_hdr"), 3)
        dr = f"{f(r.get('drop_front_s'), 0)}/{f(r.get('drop_rear_s'), 0)}"
        dr = "" if dr == "0/0" else dr
        sl = r.get("slip_ep", "")
        gi = sum(int(r.get(f"glitch_{k}", 0) or 0) for k in INPUTS)
        gg = int(r.get("glitch_gnss", 0) or 0)
        gl = "" if gi == 0 and gg == 0 else f"{gi}/{gg}"
        dist = f"{f(r.get('wheel_km'), 2)}/{f(r.get('gnss_km'), 2)}" if r.get("n_mfix") else f(r.get("wheel_km"), 2)
        dh = f(r.get("alt_max", np.nan) - r.get("alt_min", np.nan), 0) if r.get("n_mfix") else ""
        dup = r.get("dup_of", "")
        dup = f"= {dup[6:]}" if dup else ""
        lines.append(f"| {r['bag']} | {dup} | {r.get('split', '')} | {r['dur_msgs_s'] / 60:.1f} | {g} | {mv} | "
                     f"{ratio} | {dr} | {sl} | {gl} | {dist} | {dh} | {r.get('dir', '')} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plots(out, rows, extras, route_of, canon, hist):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:                                          # noqa: BLE001
        print("графики пропущены:", e)
        return
    fig, ax = plt.subplots(figsize=(14, 5.5))
    names = sorted(set(route_of.values()), key=lambda s: int(s[1:]))
    cmap = plt.get_cmap("tab10")
    for r in rows:
        b = r["bag"]
        if canon[b] != b or "track" not in extras[b]:
            continue
        tr = extras[b]["track"]
        k = names.index(route_of[b]) if b in route_of else 0
        ls = "-" if b.startswith("30618") else "--"
        ax.plot(tr[:, 0] / 1000, tr[:, 1] / 1000, ls, lw=0.8, color=cmap(k % 10), alpha=0.6)
    for k, nm in enumerate(names):
        ax.plot([], [], color=cmap(k % 10), label=nm)
    ax.plot([], [], "k-", label="30618"); ax.plot([], [], "k--", label="30639")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_xlabel("восток, км от 55.80484N 37.42050E"); ax.set_ylabel("север, км")
    ax.set_title("GNSS master, уникальные записи (цвет — кластер маршрута)")
    fig.tight_layout(); fig.savefig(out / "routes.png", dpi=110); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ks = list(range(-15, 16))
    tot = sum(hist.values()) or 1
    ax.bar(ks, [100.0 * hist.get(k, 0) / tot for k in ks], color="#4a7ab5")
    ax.set_yscale("log"); ax.set_xlabel("позиция ручки"); ax.set_ylabel("% сообщений (лог)")
    ax.set_title("Позиция ручки, уникальные записи"); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(out / "handle_hist.png", dpi=110); plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
