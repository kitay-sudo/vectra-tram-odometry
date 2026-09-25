#!/usr/bin/env python3
"""ros_probe — внешний измеритель ноды tram_estimator (аудит ROS 2 end-to-end).

Отдельная rclpy-нода. Слушает ВХОДЫ ноды (/vehicle/*, GNSS fix) и ВЫХОДЫ
(/result/velocity, /result/position, /tram/estimator_status), всё best-effort —
как судья. Для каждого сообщения запоминает момент приёма (monotonic, стенные
часы процесса) и header.stamp. По ходу прогона раз в 1 с снимает CPU% и RSS
процесса ноды (psutil). В конце пишет JSON-сводку (и, по желанию, сырые ряды в
.npz) и печатает краткий итог.

Метрики задержки (все — в стенном времени пробы, т.е. включают доставку DDS):
  in2out  для каждого входа: от приёма входа до приёма ПЕРВОГО выхода, который
          пришёл после него и имеет stamp >= stamp входа. Это выход, в котором
          показание входа уже учтено (runner шагает до stamp, потом кладёт
          показание — оно попадает в следующий шаг сетки). Главная метрика
          «вход → результат».
  age     для каждого выхода: от приёма самого нового (по stamp) входа со
          stamp <= stamp выхода, пришедшего до выхода, до приёма выхода.
  proc    для каждого выхода: от приёма последнего входа перед ним до приёма
          выхода — «чистая» обработка ноды + один лишний переход DDS.

Запуск (внутри контейнера с собранными tram_vehicle_msgs/tram_msgs):
    python3 tools/ros_probe.py --out out/ros_e2e/x/summary.json \
        --npz out/ros_e2e/x/raw.npz --rate 1.0 --idle 10
Останов: SIGINT/SIGTERM, --duration, или --idle секунд тишины после первого
сообщения.
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

try:
    from tram_msgs.msg import EstimatorStatus
except ImportError:                 # пакет статуса может отсутствовать
    EstimatorStatus = None

try:
    import psutil
except ImportError:
    psutil = None

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=200)

# код входа -> (топик, тип)
INPUTS = {
    0: ("/vehicle/front_bogie_velocity", VelocitySensor),
    1: ("/vehicle/rear_bogie_velocity", VelocitySensor),
    2: ("/vehicle/driver_position_cmd", DriverControllerCommand),
    3: ("/sensing/gnss/master/fix", NavSatFix),
    4: ("/sensing/gnss/rover/fix", NavSatFix),
}
IN_NAMES = {0: "front", 1: "rear", 2: "cmd", 3: "mfix", 4: "rfix"}
VEHICLE = (0, 1, 2)
R_EARTH = 6378137.0


def st(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


def dist(a, scale=1.0, nd=2):
    """Сводка распределения: n, mean, p50, p95, p99, max."""
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"n": 0}
    r = lambda x: round(float(x) * scale, nd)
    return {"n": int(len(a)), "mean": r(a.mean()), "p50": r(np.percentile(a, 50)),
            "p95": r(np.percentile(a, 95)), "p99": r(np.percentile(a, 99)),
            "max": r(a.max())}


class Probe(Node):
    def __init__(self, a):
        super().__init__("tram_e2e_probe")
        self.a = a
        self.t_start = time.monotonic()
        self.last_rx = None
        self.first_rx = None
        self.lock = threading.Lock()
        # сырые ряды
        self.inp = []                       # (code, wall, stamp)
        self.vel = []                       # (wall, stamp, v)
        self.odo = []                       # (wall, stamp, x, y, z, qz, qw, vx, cp0, cp7, cp14, ct0)
        self.sta = []                       # (wall, stamp, frame, valid, mode, step_us, sigma_v, slip)
        self.gvel = []                      # (stamp, |v|) эталон GNSS master/vel
        self.gfix = []                      # (stamp, lat, lon, alt) master fix
        self.frames = {"velocity": {}, "position": {}, "position_child": {}, "status": {}}
        self.cov_nz = {"pose": set(), "twist": set()}
        self.bad = {"vel_nonfinite": 0, "odo_nonfinite": 0, "cov_negative": 0,
                    "cov_nonfinite": 0, "quat_bad": 0}
        self.qos = {}
        self.proc = []                      # (wall, cpu%, rss_mb, threads, pid)
        self.proc_info = {}
        S = self.create_subscription
        for code, (topic, typ) in INPUTS.items():
            S(typ, topic, lambda m, c=code: self._in(c, m), BE)
        S(TwistStamped, "/sensing/gnss/master/vel", self._gvel, BE)
        S(VelocitySensor, "/result/velocity", self._vel, BE)
        S(Odometry, "/result/position", self._odo, BE)
        if EstimatorStatus is not None:
            S(EstimatorStatus, "/tram/estimator_status", self._sta, BE)
        self.create_timer(2.0, self._qos_poll)

    # ---------------- приём ----------------
    def _mark(self):
        now = time.monotonic()
        self.last_rx = now
        if self.first_rx is None:
            self.first_rx = now
        return now

    def _in(self, code, m):
        w = self._mark()
        self.inp.append((code, w, st(m.header.stamp)))
        if code == 3:
            self.gfix.append((st(m.header.stamp), m.latitude, m.longitude, m.altitude))

    def _gvel(self, m):
        l = m.twist.linear
        self.gvel.append((st(m.header.stamp), math.hypot(l.x, l.y)))

    def _vel(self, m):
        w = self._mark()
        v = m.velocity
        if not math.isfinite(v):
            self.bad["vel_nonfinite"] += 1
        self.vel.append((w, st(m.header.stamp), v))
        f = self.frames["velocity"]
        f[m.header.frame_id] = f.get(m.header.frame_id, 0) + 1

    def _odo(self, m):
        w = self._mark()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        pc, tc = m.pose.covariance, m.twist.covariance
        vals = (p.x, p.y, p.z, m.twist.twist.linear.x)
        if not all(math.isfinite(x) for x in vals):
            self.bad["odo_nonfinite"] += 1
        n = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        if not (abs(n - 1.0) < 1e-3):
            self.bad["quat_bad"] += 1
        for key, c in (("pose", pc), ("twist", tc)):
            for i, x in enumerate(c):
                if x != 0.0:
                    self.cov_nz[key].add(i)
                if not math.isfinite(x):
                    self.bad["cov_nonfinite"] += 1
            for i in (0, 7, 14, 21, 28, 35):
                if c[i] < 0:
                    self.bad["cov_negative"] += 1
        self.odo.append((w, st(m.header.stamp), p.x, p.y, p.z, q.z, q.w,
                         m.twist.twist.linear.x, pc[0], pc[7], pc[14], tc[0]))
        f = self.frames["position"]
        f[m.header.frame_id] = f.get(m.header.frame_id, 0) + 1
        f = self.frames["position_child"]
        f[m.child_frame_id] = f.get(m.child_frame_id, 0) + 1

    def _sta(self, m):
        w = self._mark()
        self.sta.append((w, st(m.header.stamp), m.frame_count, m.valid, m.mode,
                         m.step_time_us, m.sigma_v, m.slip))
        f = self.frames["status"]
        f[m.header.frame_id] = f.get(m.header.frame_id, 0) + 1

    def _qos_poll(self):
        for topic in ("/result/velocity", "/result/position", "/tram/estimator_status"):
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception as e:          # noqa: BLE001
                self.qos[topic] = f"error {e!r}"
                continue
            if not infos:
                continue
            out = []
            for i in infos:
                q = i.qos_profile
                out.append({"node": i.node_name, "type": i.topic_type,
                            "reliability": q.reliability.name,
                            "durability": q.durability.name,
                            "history": q.history.name, "depth": q.depth})
            self.qos[topic] = out


# ---------------- наблюдение за процессом ноды ----------------
def find_node_proc(match):
    me = os.getpid()
    for p in psutil.process_iter(["pid", "cmdline"]):
        try:
            cl = p.info["cmdline"] or []
            if p.info["pid"] != me and any(x.endswith("/" + match) or x == match
                                           for x in cl):
                return p
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return None


def proc_sampler(probe, match, stop):
    if psutil is None:
        probe.proc_info["error"] = "psutil недоступен"
        return
    p = None
    while not stop.is_set():
        if p is None or not p.is_running():
            p = find_node_proc(match)
            if p is not None:
                try:
                    p.cpu_percent(None)
                    probe.proc_info.setdefault("pids", []).append(p.pid)
                    probe.proc_info["cmdline"] = " ".join(p.cmdline())[:300]
                except psutil.NoSuchProcess:
                    p = None
            if p is None:
                stop.wait(0.2)
                continue
        stop.wait(1.0)
        try:
            with p.oneshot():
                cpu = p.cpu_percent(None)
                rss = p.memory_info().rss / 2 ** 20
                thr = p.num_threads()
                ct = p.cpu_times()
            probe.proc.append((time.monotonic(), cpu, rss, thr, p.pid))
            probe.proc_info["cpu_times_user_sys"] = [ct.user, ct.system]
        except psutil.NoSuchProcess:
            p = None


# ---------------- анализ ----------------
def analyse(pr, a):
    R = {"probe": {"rate_arg": a.rate, "tag": a.tag,
                   "wall_span_s": round(time.monotonic() - pr.t_start, 1)}}
    inp = np.array(pr.inp, float).reshape(-1, 3)
    vel = np.array(pr.vel, float).reshape(-1, 3)
    odo = np.array(pr.odo, float).reshape(-1, 12)
    sta = np.array(pr.sta, float).reshape(-1, 8)

    # --- входы ---
    R["inputs"] = {IN_NAMES[c]: int((inp[:, 0] == c).sum()) for c in INPUTS}
    veh = inp[np.isin(inp[:, 0], VEHICLE)]
    if len(veh):
        R["inputs"]["vehicle_stamp_range"] = [float(veh[:, 2].min()), float(veh[:, 2].max())]

    # --- выходы: частота, пропуски, метки ---
    out = {}
    for name, arr in (("velocity", vel), ("position", odo), ("status", sta)):
        n = len(arr)
        d = {"count": n}
        if n >= 2:
            W, T = arr[:, 0], arr[:, 1]
            dW, dT = np.diff(W), np.diff(T)
            d["rate_wall_hz"] = round((n - 1) / (W[-1] - W[0]), 2) if W[-1] > W[0] else None
            d["rate_stamp_hz"] = round((n - 1) / (T[-1] - T[0]), 2) if T[-1] != T[0] else None
            dt = float(np.median(dT))
            span = T.max() - T.min()
            d["stamp_step_median_s"] = round(dt, 4)
            d["expected_by_stamp"] = int(round(span / dt)) + 1 if dt > 0 else None
            d["unique_stamps"] = int(len(np.unique(np.round(T, 4))))
            d["stamp_zero"] = int((T == 0).sum())
            d["stamp_backward"] = int((dT < -1e-9).sum())
            d["stamp_duplicate"] = int((np.abs(dT) < 1e-9).sum())
            d["stamp_step_not_dt"] = int((np.abs(dT - dt) > 1e-3).sum())
            d["stamp_max_forward_jump_s"] = round(float(dT.max()), 3)
            d["stamp_minus_wallclock_days"] = round(float(np.median(T) - time.time()) / 86400, 1)
            d["wall_gap_s"] = dist(dW, 1.0, 4)
            d["wall_gaps_over_100ms"] = int((dW > 0.1).sum())
            d["wall_gaps_over_250ms"] = int((dW > 0.25).sum())
            d["wall_gap_top5_s"] = [round(float(x), 3) for x in np.sort(dW)[-5:][::-1]]
            # скользящее окно 1 с: минимум сообщений в секунду
            if W[-1] - W[0] > 2.0:
                j = np.searchsorted(W, W + 1.0)
                full = W + 1.0 <= W[-1]
                d["min_msgs_in_1s_window"] = int((j - np.arange(n))[full].min())
            # пачки: выходы, пришедшие <2 мс друг за другом (шаги догоняния)
            burst = np.split(np.arange(n), np.where(dW > 0.002)[0] + 1)
            sizes = np.array([len(b) for b in burst])
            d["burst_size_max"] = int(sizes.max())
            d["bursts_ge3"] = int((sizes >= 3).sum())
            if len(veh):
                lo, hi = veh[:, 2].min(), veh[:, 2].max()
                d["stamps_outside_input_range"] = int(((T < lo - 1.0) | (T > hi + 1.0)).sum())
        out[name] = d
    R["outputs"] = out
    R["frame_ids"] = pr.frames

    # --- соответствие velocity и position ---
    if len(vel) and len(odo):
        tv = dict(zip(np.round(vel[:, 1], 4), vel[:, 2]))
        diffs = [abs(tv[k] - v) for k, v in zip(np.round(odo[:, 1], 4), odo[:, 7]) if k in tv]
        R["velocity_vs_odom_twist_maxdiff"] = float(max(diffs)) if diffs else None

    # --- ковариации, конечность ---
    R["covariance"] = {"pose_nonzero_idx": sorted(pr.cov_nz["pose"]),
                       "twist_nonzero_idx": sorted(pr.cov_nz["twist"])}
    if len(odo):
        R["covariance"]["pose_var_x"] = dist(odo[:, 8], 1.0, 3)
        R["covariance"]["twist_var_vx"] = dist(odo[:, 11], 1.0, 5)
        R["covariance"]["pose_var_z_nonzero"] = int((odo[:, 10] != 0).sum())
        R["position_shape"] = {
            "frac_y_zero": round(float((odo[:, 3] == 0).mean()), 4),
            "frac_z_zero": round(float((odo[:, 4] == 0).mean()), 4),
            "frac_orientation_set": round(float(((odo[:, 5] != 0) | (odo[:, 6] != 0)).mean()), 4),
            "x_range": [round(float(odo[:, 2].min()), 2), round(float(odo[:, 2].max()), 2)],
            "y_range": [round(float(odo[:, 3].min()), 2), round(float(odo[:, 3].max()), 2)],
            "z_range": [round(float(odo[:, 4].min()), 2), round(float(odo[:, 4].max()), 2)],
            "first_xyz": [round(float(x), 2) for x in odo[0, 2:5]],
            "last_xyz": [round(float(x), 2) for x in odo[-1, 2:5]],
        }
    R["nonfinite_or_bad"] = pr.bad

    # --- статус ---
    if len(sta):
        fc = sta[:, 2]
        R["status"] = {
            "frame_count_first_last": [int(fc[0]), int(fc[-1])],
            "lost_at_probe_by_frame_count": int(np.clip(np.diff(fc) - 1, 0, None).sum()),
            "valid_frac": round(float(sta[:, 3].mean()), 4),
            "slip_frac": round(float(sta[:, 7].mean()), 4),
            "mode_hist": {int(k): int(v) for k, v in zip(*np.unique(sta[:, 4], return_counts=True))},
            "step_time_us_field": dist(sta[:, 5], 1.0, 1),
            "sigma_v": dist(sta[:, 6], 1.0, 4),
        }

    # --- задержки ---
    lat = {}
    if len(vel) and len(inp):
        W, T = vel[:, 0], vel[:, 1]
        Tm = np.maximum.accumulate(T)               # на случай немонотонных меток
        for label, sel in (("vehicle", VEHICLE), ("all", tuple(INPUTS))):
            I = inp[np.isin(inp[:, 0], sel)]
            I = I[np.argsort(I[:, 1], kind="stable")]
            Win, Sin = I[:, 1], I[:, 2]
            k1 = np.searchsorted(W, Win, side="right")      # первый выход после входа
            k2 = np.searchsorted(Tm, Sin - 1e-6, side="left")  # первый со stamp >= stamp входа
            k = np.maximum(k1, k2)
            ok = k < len(W)
            l_in2out = W[k[ok]] - Win[ok]
            lat[f"in2out_{label}_ms"] = dist(l_in2out, 1e3, 1)
            lat[f"in2out_{label}_unanswered"] = int((~ok).sum())
            # входы, пришедшие в середине прогона и оставшиеся без ответа
            mid = (~ok) & (Win < W[-1])
            lat[f"in2out_{label}_unanswered_before_last_output"] = int(mid.sum())
            lat[f"in2out_{label}_over_100ms"] = int((l_in2out > 0.1).sum())
            lat[f"in2out_{label}_over_250ms"] = int((l_in2out > 0.25).sum())
            # установившийся режим: без входов первых 2 с после первого выхода
            # (гонка обнаружения DDS на старте проигрывания)
            steady = Win[ok] >= W[0] + 2.0
            lat[f"in2out_{label}_steady_ms"] = dist(l_in2out[steady], 1e3, 1)
            lat[f"in2out_{label}_steady_over_100ms"] = int((l_in2out[steady] > 0.1).sum())
            lat[f"in2out_{label}_steady_over_250ms"] = int((l_in2out[steady] > 0.25).sum())
        # age и proc по выходам (все входы ноды)
        I = inp[np.argsort(inp[:, 1], kind="stable")]
        Win, Sin = I[:, 1], I[:, 2]
        idx = np.searchsorted(Win, W, side="right")          # входов до выхода
        has = idx > 0
        proc = W[has] - Win[idx[has] - 1]
        lat["proc_ms"] = dist(proc, 1e3, 2)
        K = 64
        win = idx[:, None] - np.arange(1, K + 1)[None, :]
        valid = win >= 0
        winc = np.clip(win, 0, None)
        S = np.where(valid & (Sin[winc] <= T[:, None] + 1e-6), Sin[winc], -np.inf)
        best = S.argmax(1)
        okb = np.isfinite(S[np.arange(len(W)), best])
        age = W[okb] - Win[winc[np.arange(len(W)), best][okb]]
        lat["age_ms"] = dist(age, 1e3, 1)
    R["latency"] = lat

    # --- ресурсы ноды ---
    P = np.array([x[:4] for x in pr.proc], float).reshape(-1, 4)
    res = dict(pr.proc_info)
    if len(P):
        res["samples"] = int(len(P))
        res["cpu_pct_1core"] = dist(P[:, 1], 1.0, 1)
        res["rss_mb_first_last_max"] = [round(float(P[0, 2]), 1), round(float(P[-1, 2]), 1),
                                        round(float(P[:, 2].max()), 1)]
        res["threads_max"] = int(P[:, 3].max())
        # утечка: наклон RSS после первых 30 с, МБ/мин
        t = (P[:, 0] - P[0, 0])
        m = t > 30
        if m.sum() >= 10:
            k, _ = np.polyfit(t[m] / 60.0, P[m, 2], 1)
            res["rss_slope_mb_per_min_after_30s"] = round(float(k), 3)
            h = len(P[m]) // 2
            res["rss_mean_first_half_vs_second_half_mb"] = [
                round(float(P[m, 2][:h].mean()), 1), round(float(P[m, 2][h:].mean()), 1)]
    R["node_process"] = res
    R["publisher_qos"] = pr.qos

    # --- контроль точности по GNSS из самого bag (санити, не официальная оценка) ---
    acc = {}
    G = np.array(pr.gvel, float).reshape(-1, 2)
    if len(G) > 20 and len(vel) > 20:
        order = np.argsort(vel[:, 1])
        T, V = vel[order, 1], vel[order, 2]
        j = np.clip(np.searchsorted(T, G[:, 0]), 1, len(T) - 1)
        j = np.where(np.abs(T[j - 1] - G[:, 0]) < np.abs(T[j] - G[:, 0]), j - 1, j)
        ok = np.abs(T[j] - G[:, 0]) <= 0.05
        e = V[j[ok]] - G[ok, 1]
        if len(e):
            acc["v_pairs"] = int(ok.sum())
            acc["v_mae"] = round(float(np.abs(e).mean()), 4)
            acc["v_rmse"] = round(float(np.sqrt((e ** 2).mean())), 4)
            acc["v_bias"] = round(float(e.mean()), 4)
    F = np.array(pr.gfix, float).reshape(-1, 4)
    if len(F) > 20 and len(odo) > 20:
        Fw = inp[inp[:, 0] == 3][:, 1]          # моменты приёма master fix
        origins = {"probe_first_fix": 0}
        # нода берёт начало ENU из ПЕРВОГО master fix, который приняла сама.
        # Если проба видела fix задолго до первого выхода ноды (поздний
        # старт ноды), берём первый fix, пришедший не раньше 0,2 с до него.
        if len(Fw) == len(F) and Fw[0] < odo[0, 0] - 1.0:
            origins["fix_near_node_start"] = int(np.searchsorted(Fw, odo[0, 0] - 0.2))
        order = np.argsort(odo[:, 1])
        T, X = odo[order, 1], odo[order, 2:5]
        j = np.clip(np.searchsorted(T, F[:, 0]), 1, len(T) - 1)
        j = np.where(np.abs(T[j - 1] - F[:, 0]) < np.abs(T[j] - F[:, 0]), j - 1, j)
        ok = np.abs(T[j] - F[:, 0]) <= 0.05
        for oname, i0 in origins.items():
            if i0 >= len(F) or not ok.sum():
                continue
            lat0, lon0, alt0 = F[i0, 1:4]
            k = math.cos(math.radians(lat0))
            ref = np.c_[np.radians(F[:, 2] - lon0) * R_EARTH * k,
                        np.radians(F[:, 1] - lat0) * R_EARTH, F[:, 3] - alt0]
            d3 = np.linalg.norm(X[j[ok]] - ref[ok], axis=1)
            acc[f"pos_{oname}"] = {"pairs": int(ok.sum()), "mean_m": round(float(d3.mean()), 2),
                                  "max_m": round(float(d3.max()), 2),
                                  "end_m": round(float(d3[-1]), 2)}
        acc["note"] = ("санити-проверка по GNSS из bag; начало ENU ноды = первый master fix, "
                       "принятый нодой; probe_first_fix верен, если нода запущена до bag")
    R["accuracy_sanity_vs_bag_gnss"] = acc
    return R


def save_npz(pr, path):
    np.savez_compressed(
        path,
        inp=np.array(pr.inp, float).reshape(-1, 3),
        vel=np.array(pr.vel, float).reshape(-1, 3),
        odo=np.array(pr.odo, float).reshape(-1, 12),
        sta=np.array(pr.sta, float).reshape(-1, 8),
        gvel=np.array(pr.gvel, float).reshape(-1, 2),
        gfix=np.array(pr.gfix, float).reshape(-1, 4),
        proc=np.array(pr.proc, float).reshape(-1, 5),
        t_start=np.array([pr.t_start]))


def brief(R):
    o = R["outputs"].get("velocity", {})
    L = R.get("latency", {})
    P = R.get("node_process", {})
    g = lambda d, k: (d or {}).get(k)
    lines = [
        f"[probe] outputs /result/velocity: n={o.get('count')} "
        f"rate_wall={o.get('rate_wall_hz')} Hz rate_stamp={o.get('rate_stamp_hz')} Hz "
        f"expected={o.get('expected_by_stamp')} gaps>100ms={o.get('wall_gaps_over_100ms')} "
        f"max_gap={g(o.get('wall_gap_s'), 'max')} s",
        f"[probe] latency in2out(vehicle) ms: p50={g(L.get('in2out_vehicle_ms'), 'p50')} "
        f"p95={g(L.get('in2out_vehicle_ms'), 'p95')} p99={g(L.get('in2out_vehicle_ms'), 'p99')} "
        f"max={g(L.get('in2out_vehicle_ms'), 'max')} unanswered={L.get('in2out_vehicle_unanswered')}; "
        f"proc p50={g(L.get('proc_ms'), 'p50')} p99={g(L.get('proc_ms'), 'p99')}",
        f"[probe] node CPU%: mean={g(P.get('cpu_pct_1core'), 'mean')} p95={g(P.get('cpu_pct_1core'), 'p95')} "
        f"max={g(P.get('cpu_pct_1core'), 'max')}; RSS first/last/max MB={P.get('rss_mb_first_last_max')} "
        f"slope={P.get('rss_slope_mb_per_min_after_30s')} MB/min",
        f"[probe] inputs={R.get('inputs')}",
        f"[probe] frame_ids={R.get('frame_ids')} accuracy={R.get('accuracy_sanity_vs_bag_gnss')}",
    ]
    return "\n".join(lines)


class _Replay:
    """Проба, восстановленная из raw.npz + прежней summary.json (--reanalyse)."""

    def __init__(self, d):
        z = np.load(os.path.join(d, "raw.npz"))
        old = json.load(open(os.path.join(d, "summary.json"), encoding="utf-8"))
        self.inp, self.vel, self.odo = z["inp"], z["vel"], z["odo"]
        self.sta, self.gvel, self.gfix = z["sta"], z["gvel"], z["gfix"]
        self.proc = [tuple(r) for r in z["proc"]]
        self.t_start = float(z["t_start"][0])
        self.frames = old.get("frame_ids", {})
        cov = old.get("covariance", {})
        self.cov_nz = {"pose": set(cov.get("pose_nonzero_idx", [])),
                       "twist": set(cov.get("twist_nonzero_idx", []))}
        self.bad = old.get("nonfinite_or_bad", {})
        self.qos = old.get("publisher_qos", {})
        np_ = old.get("node_process", {})
        self.proc_info = {k: np_[k] for k in ("pids", "cmdline", "cpu_times_user_sys") if k in np_}
        self.wall_span = old.get("probe", {}).get("wall_span_s")


def reanalyse(d, rate, tag):
    pr = _Replay(d)
    a = argparse.Namespace(rate=rate, tag=tag)
    R = analyse(pr, a)
    R["probe"]["wall_span_s"] = pr.wall_span
    R["probe"]["reanalysed"] = True
    with open(os.path.join(d, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(R, f, ensure_ascii=False, indent=1)
    print(brief(R))


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == "--reanalyse":
        # python3 tools/ros_probe.py --reanalyse out/ros_e2e/<tag> [rate]
        d = sys.argv[2]
        old = json.load(open(os.path.join(d, "summary.json"), encoding="utf-8"))
        rate = float(sys.argv[3]) if len(sys.argv) > 3 else old.get("probe", {}).get("rate_arg", 1.0)
        reanalyse(d, rate, old.get("probe", {}).get("tag", ""))
        return
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="JSON-сводка")
    ap.add_argument("--npz", default="", help="сырые ряды .npz (необязательно)")
    ap.add_argument("--rate", type=float, default=1.0, help="скорость проигрывания (для отчёта)")
    ap.add_argument("--tag", default="")
    ap.add_argument("--duration", type=float, default=0.0, help="макс. длительность, с (0 — без)")
    ap.add_argument("--idle", type=float, default=0.0,
                    help="выйти после стольких секунд тишины после первого сообщения (0 — нет)")
    ap.add_argument("--proc-match", default="tram_estimator",
                    help="имя исполняемого файла ноды для psutil")
    a = ap.parse_args()

    stop = threading.Event()

    def on_sig(*_):
        stop.set()
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    pr = Probe(a)
    th = threading.Thread(target=proc_sampler, args=(pr, a.proc_match, stop), daemon=True)
    th.start()
    print(f"[probe] started pid={os.getpid()} domain={os.environ.get('ROS_DOMAIN_ID')}", flush=True)
    try:
        while not stop.is_set():
            rclpy.spin_once(pr, timeout_sec=0.05)
            now = time.monotonic()
            if a.duration and now - pr.t_start > a.duration:
                break
            if a.idle and pr.last_rx is not None and now - pr.last_rx > a.idle:
                break
    finally:
        stop.set()
        pr._qos_poll()
        R = analyse(pr, a)
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(R, f, ensure_ascii=False, indent=1)
        if a.npz:
            save_npz(pr, a.npz)
        print(brief(R), flush=True)
        pr.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                   # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
