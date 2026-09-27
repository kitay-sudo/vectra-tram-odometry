#!/usr/bin/env python3
"""Аудит контракта жюри и устойчивости связки Runner (без ROS).

Гоняет Runner пакета (ros2_ws/src/tram_state_estimator/tram_state_estimator/
runner.py) напрямую потоками сообщений - синтетическими и собранными из
реальных прогонов (analysis/cache/*.npz), - с инъекцией аномалий. Файлы
пакета не меняются: пакет подключается через sys.path.

Запуск (из корня репозитория, в образе vectra/tram:dev):

    docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev \
        bash -c "python3 tools/dev/fuzz_runner.py all"

Подкоманды:
    static     статические проверки пакета (зависимости, лист, имена)
    data       свойства меток времени и значений в реальных прогонах
    causality  «нет данных из будущего», возраст показаний, дрейф сетки
    lag        запаздывание выхода против GNSS и смещение по фазам; правка-кандидат
    fixcheck   предлагаемые минимальные правки runner.py (подкласс) против оригинала
    summary    сводная таблица фаззинга (с картой и без) из fuzz*.json
    fuzz       синтетические сценарии аномалий (crash/hang/silence/NaN/recovery)
    real       реальные прогоны: частота, задержка, NaN; GNSS весь прогон vs окно;
               два прогона подряд без перезапуска
    resources  память и CPU на долгом прогоне (2 ч синтетики, самый длинный bag)
    all        всё по очереди

Опции: --out DIR (по умолчанию out/contract), --workers N, --quick.
Результаты: out/contract/<подкоманда>.txt и .json.

Время процессора предварительное: на хосте параллельно работают другие процессы.
"""

import argparse
import gc
import json
import math
import resource
import signal
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
CACHE = ROOT / "analysis" / "cache"
sys.path.insert(0, str(PKG))

from tram_state_estimator import estimator_core as ec          # noqa: E402
from tram_state_estimator.estimator_core import Params          # noqa: E402
from tram_state_estimator.runner import Runner, Enu              # noqa: E402
from tram_state_estimator.track_map import TrackMap              # noqa: E402

R_EARTH = 6378137.0
NODE_KEYS = ("wheel_timeout_s", "handle_timeout_s", "init_window_s", "map_file",
             "origin_lat", "origin_lon", "origin_alt", "frame_id",
             "child_frame_id")
T_EPOCH = 1787731608.0          # эпоха синтетики - как в данных (2026-08)
OUT = ROOT / "out" / "contract"


# ============================================================ общие утилиты

def load_sheet():
    """Лист вагона так, как его получает нода tram_estimator: tram.yaml."""
    y = yaml.safe_load((PKG / "config" / "tram.yaml").read_text(encoding="utf-8"))
    rp = y["/tram_state_estimator"]["ros__parameters"]
    from dataclasses import fields as _fields
    names = {f.name for f in _fields(Params)}     # параметры ноды вне ядра - мимо
    core = {k: v for k, v in rp.items() if k in names}
    node = {k: rp[k] for k in NODE_KEYS if k in rp}
    return Params.from_dict(core), node


PARAMS, NODE = load_sheet()


def load_map():
    f = PKG / "config" / "track_map.npz"
    return TrackMap.load(f) if f.exists() else None


def make_runner(use_map=True, params=None):
    r = Runner(params or PARAMS, track_map=load_map() if use_map else None, origin=None,
               wheel_timeout=NODE.get("wheel_timeout_s", 1.0),
               handle_timeout=NODE.get("handle_timeout_s", 0.5))
    r.pos.init_window = NODE.get("init_window_s", 3.0)
    return r


def rss_mb():
    try:
        for line in open("/proc/self/status"):
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return float("nan")


def hwm_mb():
    try:
        for line in open("/proc/self/status"):
            if line.startswith("VmHWM:"):
                return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


class Timeout(Exception):
    pass


def _alarm(signum, frame):
    raise Timeout()


def pct(a, q):
    a = np.asarray(a, float)
    return float(np.percentile(a, q)) if a.size else float("nan")


class Tee:
    """Печать в консоль и в файл отчёта."""

    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "w", encoding="utf-8")

    def __call__(self, *a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        self.f.write(s + "\n")
        self.f.flush()

    def close(self):
        self.f.close()


# ============================================================ прогон потока
#
# Событие: (tb, kind, arg, th, val)
#   tb - время прихода (время записи в bag; при ros2 bag play - порядок и темп)
#   kind: "w" тележка (arg = 0 передняя, 1 задняя; val - км/ч)
#         "h" ручка (val - позиция)
#         "f" GNSS fix (arg = "master"/"rover"; val = (lat, lon, alt))
#   th - header.stamp, его и получает Runner

def run_stream(runner, events, budget_s=120.0):
    """Подаёт события в Runner, как нода: каждый вызов - один callback.

    Возвращает запись: выходы (stamp, v, x, y, z, sigma_v, sigma_s, mode,
    valid, tb вызова, выпустившего выход), время вызовов, исключение, таймаут.
    """
    signal.signal(signal.SIGALRM, _alarm)
    cols = {k: [] for k in ("T", "V", "X", "Y", "Z", "SV", "SS", "MODE",
                            "VALID", "TE")}
    call_tb, call_kind, call_ms, call_nout = [], [], [], []
    rec = dict(exc=None, exc_where=None, timeout=False, n_in=0,
               n_events=len(events))
    t_call_start = None
    t_runner_before = None
    wall0 = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, budget_s)
    try:
        for tb, kind, arg, th, val in events:
            t_runner_before = runner.t
            t_call_start = time.perf_counter()
            if kind == "w":
                outs = runner.on_wheel(arg, th, val)
            elif kind == "h":
                outs = runner.on_handle(th, val)
            else:
                outs = runner.on_fix(th, arg, *val)
            ms = (time.perf_counter() - t_call_start) * 1e3
            call_tb.append(tb)
            call_kind.append(kind)
            call_ms.append(ms)
            call_nout.append(len(outs))
            for o in outs:
                cols["T"].append(o["stamp"])
                cols["V"].append(o["v"])
                cols["X"].append(o["x"])
                cols["Y"].append(o["y"])
                cols["Z"].append(o["z"])
                cols["SV"].append(o["sigma_v"])
                cols["SS"].append(o["sigma_s"])
                cols["MODE"].append(o["mode"])
                cols["VALID"].append(bool(o["valid"]) and not o["wheels_stale"])
                cols["TE"].append(tb)
            rec["n_in"] += 1
    except Timeout:
        rec["timeout"] = True
        el = time.perf_counter() - t_call_start
        rec["hang_call_elapsed_s"] = el
        if t_runner_before is not None and runner.t is not None:
            steps = (runner.t - t_runner_before) / PARAMS.dt
            rec["hang_steps_done"] = steps
            rec["hang_steps_per_s"] = steps / max(el, 1e-9)
    except Exception as e:                      # noqa: BLE001 - это и ищем
        rec["exc"] = f"{type(e).__name__}: {e}"
        tb_ = traceback.extract_tb(e.__traceback__)
        rec["exc_where"] = " <- ".join(
            f"{Path(fr.filename).name}:{fr.lineno}" for fr in reversed(tb_[-4:]))
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
    rec["wall_s"] = time.perf_counter() - wall0
    for k, v in cols.items():
        rec[k] = np.asarray(v, dtype=float)
    rec["call_tb"] = np.asarray(call_tb, float)
    rec["call_ms"] = np.asarray(call_ms, float)
    rec["call_nout"] = np.asarray(call_nout, int)
    rec["call_kind"] = np.asarray(call_kind)
    rec["runner_t_end"] = runner.t
    c = runner.core
    latent = []
    for name in ("x", "P", "axle_dot", "axle_prev", "axle_scale", "prev_meas"):
        a = np.asarray(getattr(c, name), dtype=float)
        if not np.all(np.isfinite(a)):
            latent.append(name)
    for name in ("mu", "u_filt"):
        if not math.isfinite(float(getattr(c, name))):
            latent.append(name)
    rec["core_nonfinite"] = latent
    return rec


def input_to_output_latency(rec):
    """Задержка «приход входа -> первая публикация, которая его учитывает».

    Сообщение m, пришедшее в вызове c, попадает в ядро на первом шаге, который
    сделан в вызове ПОСЛЕ c (в вызове c сначала шагает сетка, затем
    запоминается m). Задержка = tb первого следующего вызова с выходами - tb_c.
    Считается по сообщениям тележек и ручки. Время на обработку не входит.
    """
    tb, n, kind = rec["call_tb"], rec["call_nout"], rec["call_kind"]
    if tb.size == 0:
        return np.zeros(0)
    nxt = np.full(tb.size, np.nan)
    last = np.nan
    for i in range(tb.size - 1, -1, -1):
        nxt[i] = last
        if n[i] > 0:
            last = tb[i]
    m = (kind == "w") | (kind == "h")
    lat = nxt[m] - tb[m]
    return lat[np.isfinite(lat)]


def summarize(rec):
    """Сводка по одному прогону потока."""
    T, V = rec["T"], rec["V"]
    s = dict(exc=rec["exc"], exc_where=rec["exc_where"],
             core_nonfinite=rec.get("core_nonfinite", []),
             timeout=rec["timeout"], n_in=rec["n_in"],
             n_events=rec["n_events"], n_out=int(T.size),
             wall_s=rec["wall_s"])
    for k in ("hang_call_elapsed_s", "hang_steps_done", "hang_steps_per_s"):
        if k in rec:
            s[k] = rec[k]
    if T.size:
        allv = np.c_[V, rec["X"], rec["Y"], rec["Z"], rec["SV"], rec["SS"]]
        fin = np.all(np.isfinite(allv), axis=1)
        s["nonfinite_outputs"] = int((~fin).sum())
        s["nonfinite_first_stamp"] = (float(T[~fin][0] - T[0])
                                      if (~fin).any() else None)
        s["last_output_finite"] = bool(fin[-1])
        s["nonfinite_fields"] = [n for n, c in zip(
            ("v", "x", "y", "z", "sigma_v", "sigma_s"), allv.T)
            if not np.all(np.isfinite(c))]
        dT = np.diff(T)
        s["stamps_strictly_increasing"] = bool(np.all(dT > 0)) if dT.size else True
        s["stamp_span_s"] = float(T[-1] - T[0])
        s["rate_hz"] = float(T.size / max(T[-1] - T[0], 1e-9))
        s["max_stamp_gap_s"] = float(dT.max()) if dT.size else 0.0
        lagg = rec["TE"] - T
        s["stamp_lag_p50_ms"] = pct(lagg, 50) * 1e3
        s["stamp_lag_p99_ms"] = pct(lagg, 99) * 1e3
        s["stamp_lag_max_ms"] = float(lagg.max()) * 1e3
        te = np.unique(rec["TE"])
        # тишина: самый длинный промежуток прихода входов без публикаций
        ctb = rec["call_tb"]
        if ctb.size:
            emit = rec["call_nout"] > 0
            etb = ctb[emit]
            # добавляем конец входного потока: тишина до конца тоже тишина
            etb = np.r_[etb, ctb[-1]]
            s["max_silence_s"] = float(np.max(np.diff(etb))) if etb.size > 1 else 0.0
            s["silence_at_s"] = (float(etb[np.argmax(np.diff(etb))] - ctb[0])
                                 if etb.size > 1 else 0.0)
            s["input_span_s"] = float(ctb[-1] - ctb[0])
        s["max_burst"] = int(rec["call_nout"].max()) if rec["call_nout"].size else 0
        s["n_emitting_calls"] = int(te.size)
    else:
        s["nonfinite_outputs"] = 0
        s["max_silence_s"] = (float(rec["call_tb"][-1] - rec["call_tb"][0])
                              if rec["call_tb"].size else 0.0)
    cm = rec["call_ms"]
    if cm.size:
        s["call_ms_p50"] = pct(cm, 50)
        s["call_ms_p99"] = pct(cm, 99)
        s["call_ms_max"] = float(cm.max())
        steps = max(int(rec["call_nout"].sum()), 1)
        s["ms_per_step"] = float(cm.sum() / steps)
    lat = input_to_output_latency(rec)
    if lat.size:
        s["in2out_p50_ms"] = pct(lat, 50) * 1e3
        s["in2out_p99_ms"] = pct(lat, 99) * 1e3
        s["in2out_max_ms"] = float(lat.max()) * 1e3
        s["in2out_gt100ms_frac"] = float(np.mean(lat > 0.100))
        s["in2out_gt250ms_frac"] = float(np.mean(lat > 0.250))
    return s


# ============================================================ синтетика

def real_origin():
    """Начало и курс синтетики - из первого прогона с GNSS, чтобы карта
    путей работала как в жизни. Нет кэша - Москва, север."""
    for f in sorted(CACHE.glob("3*.npz")):
        z = np.load(f)
        m, r = z["mfix"], z["rfix"]
        if len(m) > 30 and len(r) > 30:
            w = m[:, 1] <= m[0, 1] + 3.0
            wr = r[:, 1] <= m[0, 1] + 3.0
            lat0, lon0, alt0 = m[0, 2], m[0, 3], m[0, 4]
            enu = Enu(lat0, lon0, alt0)
            M = np.array([enu.fwd(*row) for row in m[w][:, 2:5]])
            Rr = np.array([enu.fwd(*row) for row in r[wr][:, 2:5]])
            d = Rr[:, :2].mean(0) - M[:, :2].mean(0)
            return (lat0, lon0, alt0), math.atan2(d[0], d[1]), f.stem
    return (55.8, 37.46, 168.0), 0.0, None


ORIGIN, AZ0, ORIGIN_BAG = None, None, None


def _origin():
    global ORIGIN, AZ0, ORIGIN_BAG
    if ORIGIN is None:
        ORIGIN, AZ0, ORIGIN_BAG = real_origin()
    return ORIGIN, AZ0


def notch_schedule(tc):
    """Цикл 90 с: стоянка, тяга, выбег, торможение, стоянка."""
    if tc < 10:
        return 0
    if tc < 25:
        return 10
    if tc < 45:
        return 0
    if tc < 75:
        return -6
    return 0


def truth(duration, t_skip=0.0, dt=0.01):
    """Истинное движение по модели ядра (табличный привод листа вагона):
    так синтетика согласована с моделью, и отклонения - от аномалий, а не
    от несовпадения модели. Возвращает сетку t, v, s (от t_skip)."""
    p = PARAMS
    n = int((duration + t_skip) / dt) + 1
    t = np.arange(n) * dt
    v = np.zeros(n)
    s = np.zeros(n)
    uf, vv, ss = 0.0, 0.0, 0.0
    nd = int(round(p.delay_drive / dt))
    hist = [0.0] * (nd + 1)
    for i in range(n):
        hist.append(notch_schedule(t[i] % 90.0))
        u = ec.notch_to_u(hist.pop(0), p)
        uf += (u - uf) * dt / p.tau_drive
        a = (ec.body_force(uf, vv, 1.0, 1.0, p.mu_nominal, p)
             - ec.resistance(vv, p)) / p.M_nom
        vv = max(0.0, vv + a * dt)
        ss += vv * dt
        v[i], s[i] = vv, ss
    k0 = int(round(t_skip / dt))
    return t[k0:] - t[k0], v[k0:], s[k0:] - s[k0], t_skip


def synth(duration=300.0, seed=1, gnss="window", gnss_window=3.0,
          t_skip=0.0, noise_kmh=0.15):
    """Синтетический поток как из bag: тележки 9,4 Гц (задняя со сдвигом
    37 мс), ручка 20 Гц, GNSS 10 Гц (master и rover в 12 м впереди по курсу).
    tb = th + типичная задержка записи из данных (0,05 / 0,001 / 0,045 с).

    gnss: "window" - только первые gnss_window с; "full" - весь прогон;
          "none" - нет.
    Возвращает (events, truth dict).
    """
    rng = np.random.default_rng(seed)
    tt, vt, st, _ = truth(duration, t_skip)
    (lat0, lon0, alt0), az = _origin()
    k = math.cos(math.radians(lat0))
    ev = []
    t0 = T_EPOCH
    ms = PARAMS.meas_scale
    for i, off in ((0, 0.0), (1, 0.037)):
        for j in range(int((duration - off) * 9.4)):
            th = j / 9.4 + off
            vv = float(np.interp(th, tt, vt))
            kmh = vv * 3.6 / ms
            if vv > 0.05:
                kmh += rng.normal(0.0, noise_kmh)
            ev.append((t0 + th + 0.05, "w", i, t0 + th, round(max(kmh, 0.0), 2)))
    for j in range(int(duration / 0.05)):
        th = j * 0.05 + 0.013
        tc = (th + t_skip) % 90.0
        ev.append((t0 + th + 0.001, "h", 0, t0 + th, int(notch_schedule(tc))))
    if gnss != "none":
        t_end = gnss_window if gnss == "window" else duration
        for j in range(int(t_end / 0.1) + 1):
            th = j * 0.1
            sm = float(np.interp(th, tt, st))
            for ant, extra, dtb in (("master", 0.0, 0.045), ("rover", 12.0, 0.052)):
                d = sm + extra
                x, y = d * math.sin(az), d * math.cos(az)
                lat = lat0 + math.degrees(y / R_EARTH)
                lon = lon0 + math.degrees(x / (R_EARTH * k))
                ev.append((t0 + th + dtb, "f", ant, t0 + th, (lat, lon, alt0)))
    ev.sort(key=lambda e: e[0])
    tru = dict(t=tt + t0, v=vt, s=st, az=az, origin=(lat0, lon0, alt0))
    return ev, tru


def truth_xy(tru, T):
    """Истинное положение master в ENU от первой точки master (как эталон)."""
    s = np.interp(T, tru["t"], tru["s"])
    return np.c_[s * math.sin(tru["az"]), s * math.cos(tru["az"])]


# ---------------------------------------------------------------- мутации

def _moving_at(ev, t_rel, dur):
    t0 = T_EPOCH
    return lambda e: t0 + t_rel <= e[3] < t0 + t_rel + dur


def m_value(topic_idx, value, t_rel, dur, first_only=False):
    """Заменить показания тележки(ек) в окне на value."""
    def f(ev):
        out, done = [], False
        inwin = _moving_at(ev, t_rel, dur)
        for e in ev:
            if (e[1] == "w" and (topic_idx is None or e[2] == topic_idx)
                    and inwin(e) and not (first_only and done)):
                e = (e[0], e[1], e[2], e[3], value)
                done = True
            out.append(e)
        return out
    return f


def m_first_wheel_value(value):
    def f(ev):
        out, done = [], False
        for e in ev:
            if e[1] == "w" and not done:
                e = (e[0], e[1], e[2], e[3], value)
                done = True
            out.append(e)
        return out
    return f


def m_stamp_one(kind, t_rel, new_th):
    """Одному сообщению вида kind около t_rel поставить метку new_th(th)."""
    def f(ev):
        out, done = [], False
        for e in ev:
            if not done and e[1] == kind and e[3] >= T_EPOCH + t_rel:
                e = (e[0], e[1], e[2], new_th(e[3]), e[4])
                done = True
            out.append(e)
        return out
    return f


def m_first_stamp_zero(ev):
    e = ev[0]
    return [(e[0], e[1], e[2], 0.0, e[4])] + ev[1:]


def m_shift_after(t_rel, shift):
    """Все метки после t_rel сдвинуть на shift (скачок часов, перезапуск)."""
    def f(ev):
        return [(e[0], e[1], e[2], e[3] + shift if e[3] >= T_EPOCH + t_rel
                 else e[3], e[4]) for e in ev]
    return f


def m_second_bag(gap):
    """После прогона - ещё раз тот же прогон с метками на gap позже конца
    первого (gap < 0 - второй прогон раньше первого)."""
    def f(ev):
        span = ev[-1][3] - ev[0][3]
        sh = span + gap
        return ev + [(e[0] + span + 1.0, e[1], e[2], e[3] + sh, e[4]) for e in ev]
    return f


def m_duplicate(ev):
    out = []
    for e in ev:
        out += [e, e]
    return out


def m_delay_topic(kind, idx, delay):
    def f(ev):
        out = [(e[0] + delay, *e[1:]) if (e[1] == kind and e[2] == idx) else e
               for e in ev]
        out.sort(key=lambda e: e[0])
        return out
    return f


def m_jitter(maxj, seed=3):
    def f(ev):
        rng = np.random.default_rng(seed)
        out = [(e[0] + rng.uniform(0, maxj), *e[1:]) for e in ev]
        out.sort(key=lambda e: e[0])
        return out
    return f


def m_swap_adjacent_same_topic(ev):
    """Внутри каждого топика - перестановка соседних сообщений (порядок
    прихода нарушен, метки идут назад)."""
    idx = {}
    for i, e in enumerate(ev):
        idx.setdefault((e[1], e[2]), []).append(i)
    out = list(ev)
    for lst in idx.values():
        for a, b in zip(lst[0::2], lst[1::2]):
            ea, eb = out[a], out[b]
            out[a] = (ea[0], eb[1], eb[2], eb[3], eb[4])
            out[b] = (eb[0], ea[1], ea[2], ea[3], ea[4])
    return out


def m_gnss(fn):
    """Преобразование GNSS-события fn(e) -> e или None (удалить)."""
    def f(ev):
        out = []
        for e in ev:
            if e[1] == "f":
                e = fn(e)
                if e is None:
                    continue
            out.append(e)
        return out
    return f


def m_handle(fn):
    def f(ev):
        out = []
        for e in ev:
            if e[1] == "h":
                e = fn(e)
                if e is None:
                    continue
            out.append(e)
        return out
    return f


def m_drop(pred):
    return lambda ev: [e for e in ev if not pred(e)]


def m_stuck_both(t_rel, dur):
    """Обе тележки залипли: повторяют последнее значение до окна."""
    def f(ev):
        last = {0: 0.0, 1: 0.0}
        out = []
        for e in ev:
            if e[1] == "w":
                if T_EPOCH + t_rel <= e[3] < T_EPOCH + t_rel + dur:
                    e = (e[0], e[1], e[2], e[3], last[e[2]])
                else:
                    last[e[2]] = e[4]
            out.append(e)
        return out
    return f


def m_rate_drop(t_rel, dur, hz):
    def f(ev):
        out, lastt = [], {0: -1e18, 1: -1e18}
        for e in ev:
            if e[1] == "w" and T_EPOCH + t_rel <= e[3] < T_EPOCH + t_rel + dur:
                if e[3] - lastt[e[2]] < 1.0 / hz:
                    continue
                lastt[e[2]] = e[3]
            out.append(e)
        return out
    return f


def win(t_rel, dur, kinds=("w", "h", "f")):
    return lambda e: e[1] in kinds and T_EPOCH + t_rel <= e[3] < T_EPOCH + t_rel + dur


# --------------------------------------------------------------- сценарии

TA = 30.0      # момент аномалии: выбег на ~10 м/с (цикл 90 с)


def scenarios():
    """(имя, описание, мутация, окно аномалии (t_rel, dur) или None, opts)."""
    nan, inf = float("nan"), float("inf")
    S = [
        ("base", "без аномалий (эталон для сравнения)", None, None, {}),
        ("nan_front_1", "одно показание передней тележки = NaN",
         m_value(0, nan, TA, 1.0, first_only=True), (TA, 0.2), {}),
        ("nan_both_5s", "обе тележки NaN 5 с", m_value(None, nan, TA, 5.0), (TA, 5.0), {}),
        ("nan_first_wheel", "самое первое показание тележки NaN",
         m_first_wheel_value(nan), (0.0, 0.2), {}),
        ("inf_front_1", "одно показание +inf", m_value(0, inf, TA, 1.0, True), (TA, 0.2), {}),
        ("neginf_rear_1", "одно показание -inf", m_value(1, -inf, TA, 1.0, True), (TA, 0.2), {}),
        ("negative_front_5s", "передняя = -30 км/ч 5 с", m_value(0, -30.0, TA, 5.0), (TA, 5.0), {}),
        ("huge_front_1", "одно показание 1e9 км/ч", m_value(0, 1e9, TA, 1.0, True), (TA, 0.2), {}),
        ("huge_both_5s", "обе 1e6 км/ч 5 с", m_value(None, 1e6, TA, 5.0), (TA, 5.0), {}),
        ("huge_first_wheel", "первое показание 1e6 км/ч",
         m_first_wheel_value(1e6), (0.0, 0.2), {}),
        ("stamp0_wheel_mid", "у одного сообщения тележки stamp = 0",
         m_stamp_one("w", TA, lambda th: 0.0), (TA, 0.2), {}),
        ("stamp0_handle_mid", "у одного сообщения ручки stamp = 0",
         m_stamp_one("h", TA, lambda th: 0.0), (TA, 0.2), {}),
        ("stamp0_first_msg", "у ПЕРВОГО сообщения stamp = 0", m_first_stamp_zero,
         (0.0, 0.2), dict(budget=60.0)),
        ("stamp_future_1h_one", "одно сообщение тележки с меткой +1 ч (сбой часов)",
         m_stamp_one("w", TA, lambda th: th + 3600.0), (TA, 0.2), dict(budget=240.0)),
        ("stamp_future_10s_one", "одно сообщение ручки с меткой +10 с",
         m_stamp_one("h", TA, lambda th: th + 10.0), (TA, 0.2), {}),
        ("stamp_back_100s_one", "одно сообщение тележки с меткой -100 с",
         m_stamp_one("w", TA, lambda th: th - 100.0), (TA, 0.2), {}),
        ("clock_back_100s", "все метки после 150 с сдвинуты на -100 с (перезапуск/скачок назад)",
         m_shift_after(150.0, -100.0), (150.0, 150.0), {}),
        ("clock_fwd_1s", "все метки после 150 с сдвинуты на +1 с (скачок часов, как в 8 прогонах)",
         m_shift_after(150.0, 1.0), (150.0, 1.0), {}),
        ("second_bag_earlier", "второй прогон подряд без перезапуска, метки на 1 сут раньше",
         m_second_bag(-86400.0 - 600.0), None, dict(second=True)),
        ("second_bag_later_10min", "второй прогон подряд, через 10 мин по меткам",
         m_second_bag(600.0), None, dict(second=True, budget=240.0)),
        ("second_bag_later_1h", "второй прогон подряд, через 1 ч по меткам",
         m_second_bag(3600.0), None, dict(second=True, budget=240.0)),
        ("second_bag_later_1d", "второй прогон подряд, через 1 сут по меткам",
         m_second_bag(86400.0), None, dict(second=True, budget=60.0)),
        ("duplicate_all", "каждое сообщение дважды", m_duplicate, None, {}),
        ("rear_arrives_300ms_late", "задняя тележка приходит на 300 мс позже (порядок между топиками)",
         m_delay_topic("w", 1, 0.300), None, {}),
        ("jitter_150ms", "случайный сдвиг прихода 0..150 мс у всех сообщений",
         m_jitter(0.150), None, {}),
        ("swap_adjacent", "соседние сообщения каждого топика переставлены (метки назад)",
         m_swap_adjacent_same_topic, None, {}),
        ("gnss_nan_latlon", "GNSS в окне: lat/lon = NaN",
         m_gnss(lambda e: (e[0], e[1], e[2], e[3], (float("nan"), float("nan"), e[4][2]))),
         None, {}),
        ("gnss_nan_alt", "GNSS в окне: alt = NaN (lat/lon в порядке)",
         m_gnss(lambda e: (e[0], e[1], e[2], e[3], (e[4][0], e[4][1], float("nan")))),
         None, {}),
        ("gnss_nofix_zero_first", "первые 5 точек GNSS: lat = lon = 0 (нет решения)",
         None, None, dict(nofix=True)),
        ("gnss_rover_only", "GNSS только rover", m_gnss(lambda e: e if e[2] == "rover" else None),
         None, {}),
        ("gnss_master_only", "GNSS только master", m_gnss(lambda e: e if e[2] == "master" else None),
         None, {}),
        ("gnss_never", "GNSS нет совсем", None, None, dict(gnss="none")),
        ("gnss_full_run", "GNSS весь прогон (как при ros2 bag play обучающего прогона)",
         None, None, dict(gnss="full")),
        ("gnss_start_moving", "старт на ходу (~9 м/с): GNSS первые 3 с во время движения",
         None, None, dict(t_skip=20.0)),
        ("gnss_future_stamp", "одна точка GNSS в окне с меткой +3600 с",
         m_stamp_one("f", 1.0, lambda th: th + 3600.0), None, dict(budget=240.0)),
        ("handle_out_of_range", "ручка +100 / -128 по 5 с",
         m_handle(lambda e: (e[0], e[1], e[2], e[3],
                             100 if T_EPOCH + TA <= e[3] < T_EPOCH + TA + 5 else
                             (-128 if T_EPOCH + TA + 5 <= e[3] < T_EPOCH + TA + 10 else e[4]))),
         (TA, 10.0), {}),
        ("handle_missing", "ручки нет совсем", m_handle(lambda e: None), None, {}),
        ("handle_stall_10s", "ручка молчит 10 с", m_drop(win(TA, 10.0, ("h",))), (TA, 10.0), {}),
        ("one_bogie_only", "задней тележки нет весь прогон", m_drop(lambda e: e[1] == "w" and e[2] == 1),
         None, {}),
        ("both_stuck_30s", "обе тележки залипли на последнем значении 30 с (торможение и стоянка)",
         m_stuck_both(50.0, 30.0), (50.0, 30.0), {}),
        ("both_zero_10s", "обе тележки = 0 на ходу 10 с", m_value(None, 0.0, TA, 10.0), (TA, 10.0), {}),
        ("bogies_stall_5s", "тележки молчат 5 с (ручка идёт)", m_drop(win(TA, 5.0, ("w",))), (TA, 5.0), {}),
        ("all_inputs_stall_5s", "все входы молчат 5 с", m_drop(win(TA, 5.0)), (TA, 5.0), {}),
        ("bogie_rate_1hz_60s", "тележки 1 Гц на 60 с", m_rate_drop(TA, 60.0, 1.0), (TA, 60.0), {}),
    ]
    return S


def nofix_mut(ev):
    out, n = [], 0
    for e in ev:
        if e[1] == "f" and n < 5:
            e = (e[0], e[1], e[2], e[3], (0.0, 0.0, 0.0))
            n += 1
        out.append(e)
    return out


def compare(rec, base, t_end_anom):
    """Отклонение от эталонного прогона (та же сетка) и время восстановления."""
    T, B = rec["T"], base["T"]
    if T.size == 0 or B.size == 0:
        return {}
    j = np.clip(np.searchsorted(B, T), 0, B.size - 1)
    j2 = np.clip(j - 1, 0, B.size - 1)
    j = np.where(np.abs(B[j2] - T) < np.abs(B[j] - T), j2, j)
    ok = np.abs(B[j] - T) < 1e-6
    if not ok.any():
        return dict(common_stamps=0)
    Tc = T[ok]
    dv = np.abs(rec["V"][ok] - base["V"][j[ok]])
    dp = np.hypot(rec["X"][ok] - base["X"][j[ok]], rec["Y"][ok] - base["Y"][j[ok]])
    out = dict(common_stamps=int(ok.sum()))
    with np.errstate(invalid="ignore"):
        out["max_dv"] = float(np.nanmax(dv)) if np.isfinite(dv).any() else float("nan")
        out["end_dv"] = float(dv[-1])
        out["end_dp"] = float(dp[-1])
        out["max_dp"] = float(np.nanmax(dp)) if np.isfinite(dp).any() else float("nan")
    if t_end_anom is not None:
        te = T_EPOCH + t_end_anom
        after = Tc >= te
        bad = ~(dv <= 0.1)          # NaN - тоже плохо
        idx = np.flatnonzero(after & bad)
        if not after.any():
            out["recovery_s"] = None
        elif idx.size == 0:
            out["recovery_s"] = 0.0
        elif idx[-1] == Tc.size - 1:
            out["recovery_s"] = float("inf")        # не восстановилась к концу
        else:
            out["recovery_s"] = float(Tc[idx[-1] + 1] - te)
        out["max_dv_after"] = (float(np.nanmax(dv[after])) if after.any()
                               and np.isfinite(dv[after]).any() else float("nan"))
    return out


def vs_truth(rec, tru):
    T = rec["T"]
    if T.size == 0:
        return {}
    vt = np.interp(T, tru["t"], tru["v"])
    xy = truth_xy(tru, T)
    ev = rec["V"] - vt
    ep = np.hypot(rec["X"] - xy[:, 0], rec["Y"] - xy[:, 1])
    m = T > T[0] + 5.0
    with np.errstate(invalid="ignore"):
        return dict(v_mae=float(np.nanmean(np.abs(ev[m]))) if m.any() else None,
                    pos_err_end=float(ep[-1]),
                    pos_err_mean=float(np.nanmean(ep[m])) if m.any() else None,
                    pos_err_max=float(np.nanmax(ep[m])) if m.any() and np.isfinite(ep[m]).any() else None)


def verdict(s, cmp_):
    flags = []
    if s.get("exc"):
        flags.append("CRASH")
    if s.get("timeout"):
        flags.append("HANG")
    if s.get("nonfinite_outputs"):
        flags.append("NaN-perm" if not s.get("last_output_finite", True) else "NaN")
    lat = [n for n in s.get("core_nonfinite", []) if n not in ("x", "P")]
    if lat and not s.get("nonfinite_outputs"):
        flags.append("LATENT-NaN " + "/".join(lat))
    if s.get("max_silence_s", 0) > 1.0:
        flags.append(f"SILENT {s['max_silence_s']:.0f}s")
    if s.get("max_burst", 0) > 20:
        flags.append(f"BURST {s['max_burst']}")
    dp = cmp_.get("end_dp")
    if dp is not None and math.isfinite(dp) and dp > 50.0:
        flags.append(f"POS-DELTA {dp:.0f}m")
    rs = cmp_.get("recovery_s")
    if rs == float("inf"):
        flags.append("NO-RECOVERY")
    elif rs is not None and rs > 10:
        flags.append(f"slow-recovery {rs:.0f}s")
    return ", ".join(flags) if flags else "OK"


def cmd_fuzz(args):
    suffix = "" if args.map else "_nomap"
    out = Tee(args.out / f"fuzz{suffix}.txt")
    (lat0, lon0, _), az = _origin()
    out(f"# Фаззинг Runner (без ROS). Лист: tram.yaml, карта: {'config/track_map.npz' if args.map else 'нет'}")
    out(f"# Синтетика: 300 с, цикл 90 с (стоянка/тяга 10/выбег/тормоз -6/стоянка), "
        f"тележки 9,4 Гц, ручка 20 Гц, GNSS 10 Гц первые 3 с; начало {lat0:.6f},{lon0:.6f} "
        f"(из {ORIGIN_BAG}), курс {math.degrees(az):.1f}°")
    out(f"# Аномалия по умолчанию в t = {TA} с (выбег ~10 м/с). dt ядра = {PARAMS.dt} с.")
    out("# Время CPU предварительное (параллельно работают другие контейнеры).")
    out("")
    results = {}
    base_cache = {}
    for name, desc, mut, anom, opts in scenarios():
        if args.only and name not in args.only:
            continue
        dur = opts.get("duration", 300.0)
        gnss = opts.get("gnss", "window")
        t_skip = opts.get("t_skip", 0.0)
        key = (gnss, t_skip)
        ev, tru = synth(dur, gnss=gnss, t_skip=t_skip)
        if opts.get("nofix"):
            ev = nofix_mut(ev)
        if mut is not None:
            ev = mut(ev)
        if key not in base_cache:
            bev, _ = synth(dur, gnss="window", t_skip=t_skip)
            base_cache[key] = run_stream(make_runner(args.map), bev)
        base = base_cache[key]
        gc.collect()
        rss0 = rss_mb()
        rec = run_stream(make_runner(args.map), ev, budget_s=opts.get("budget", 120.0))
        rss1 = rss_mb()
        s = summarize(rec)
        s["rss_delta_mb"] = rss1 - rss0
        t_end = (anom[0] + anom[1]) if anom else None
        c = compare(rec, base, t_end) if not opts.get("second") else {}
        tr = vs_truth(rec, tru) if not opts.get("second") else {}
        if args.map:        # с картой прямая синтетика не совпадает с путями
            for k in ("pos_err_end", "pos_err_mean", "pos_err_max"):
                tr.pop(k, None)
        v = verdict(s, c)
        results[name] = dict(desc=desc, summary=s, vs_base=c, vs_truth=tr, verdict=v)
        out(f"## {name}: {desc}")
        out(f"   ВЕРДИКТ: {v}")
        if s.get("exc"):
            out(f"   исключение: {s['exc']}  @ {s['exc_where']}")
        if s.get("core_nonfinite"):
            out(f"   нечисла во внутреннем состоянии ядра после прогона: {s['core_nonfinite']}")
        if s.get("timeout"):
            sp = s.get("hang_steps_per_s")
            out(f"   таймаут {opts.get('budget', 120.0):.0f} с: в зависшем вызове сделано "
                f"{s.get('hang_steps_done', 0):.0f} шагов, {sp:.0f} шаг/с"
                if sp else "   таймаут")
        out(f"   входов {s['n_in']}/{s['n_events']}, выходов {s['n_out']}, "
            f"частота {s.get('rate_hz', 0):.2f} Гц, NaN/inf выходов {s['nonfinite_outputs']}"
            + (f" (поля {s['nonfinite_fields']}, с {s['nonfinite_first_stamp']:.1f} с, "
               f"последний конечен: {s['last_output_finite']})" if s['nonfinite_outputs'] else ""))
        out(f"   тишина (макс. интервал прихода входов без публикаций) {s.get('max_silence_s', 0):.2f} с"
            f" (в t={s.get('silence_at_s', 0):.1f} с), макс. пачка {s.get('max_burst', 0)} выходов/вызов, "
            f"макс. вызов {s.get('call_ms_max', 0):.1f} мс, RSS +{s['rss_delta_mb']:.1f} МБ, "
            f"стена {s['wall_s']:.1f} с")
        if c:
            out(f"   против эталона: общих меток {c.get('common_stamps')}, max|dv| "
                f"{c.get('max_dv', float('nan')):.3f} м/с, в конце |dv| {c.get('end_dv', float('nan')):.3f}, "
                f"|dp| {c.get('end_dp', float('nan')):.2f} м (макс {c.get('max_dp', float('nan')):.2f})"
                + (f", восстановление {c['recovery_s']:.2f} с после окна" if c.get('recovery_s') is not None else ""))
        if tr:
            out(f"   против истины: |ош v| {tr.get('v_mae') or float('nan'):.3f} м/с"
                + (f", ош. положения ср {tr.get('pos_err_mean') or float('nan'):.2f} м, "
                   f"конец {tr.get('pos_err_end', float('nan')):.2f} м" if "pos_err_end" in tr
                   else " (положение против истины не считается: карта)"))
        out("")
    (args.out / f"fuzz{suffix}.json").write_text(json.dumps(results, ensure_ascii=False, indent=1,
                                                   default=float), encoding="utf-8")
    out("# Сводка")
    out(f"{'сценарий':<26}{'вердикт':<40}{'NaN':>6}{'тишина,с':>10}{'восст.,с':>10}{'макс вызов,мс':>15}")
    for n, r in results.items():
        s, c = r["summary"], r["vs_base"]
        rs = c.get("recovery_s")
        rs = "-" if rs is None else ("нет" if rs == float("inf") else f"{rs:.1f}")
        out(f"{n:<26}{r['verdict'][:39]:<40}{s['nonfinite_outputs']:>6}"
            f"{s.get('max_silence_s', 0):>10.2f}{rs:>10}{s.get('call_ms_max', 0):>15.1f}")
    out.close()


# ============================================================ реальные прогоны

def bag_events(bag, gnss="window", init_s=3.0):
    """События прогона в порядке записи (как ros2 bag play)."""
    z = np.load(CACHE / f"{bag}.npz")
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in z[key]:
            ev.append((tb, "w", i, th, float(v)))
    for tb, th, n in z["cmd"]:
        ev.append((tb, "h", 0, th, int(n)))
    if gnss != "none" and len(z["mfix"]):
        t_end = z["mfix"][0, 0] + init_s if gnss == "window" else float("inf")
        for key, ant in (("mfix", "master"), ("rfix", "rover")):
            for row in z[key]:
                if row[0] <= t_end:
                    ev.append((row[0], "f", ant, row[1], (row[2], row[3], row[4])))
    ev.sort(key=lambda e: e[0])
    return ev, z


def gnss_metrics(rec, z):
    """Ошибки против GNSS master, как в analysis/evaluate.py (TOL 0,05 с)."""
    T = rec["T"]
    if T.size < 2 or len(z["mfix"]) < 10 or len(z["mvel"]) < 10:
        return {}

    def nearest(t_out, t_ref):
        j = np.clip(np.searchsorted(t_out, t_ref), 1, len(t_out) - 1)
        j = np.where(np.abs(t_out[j - 1] - t_ref) < np.abs(t_out[j] - t_ref), j - 1, j)
        return j, np.abs(t_out[j] - t_ref) <= 0.05
    g = z["mvel"]
    j, ok = nearest(T, g[:, 1])
    vg = np.hypot(g[:, 2], g[:, 3])
    ev = rec["V"][j[ok]] - vg[ok]
    m = z["mfix"]
    enu = Enu(m[0, 2], m[0, 3], m[0, 4])
    ref = np.array([enu.fwd(a, b, c) for a, b, c in m[:, 2:5]])
    j2, ok2 = nearest(T, m[:, 1])
    X = np.c_[rec["X"], rec["Y"], rec["Z"]]
    d3 = np.linalg.norm(X[j2[ok2]] - ref[ok2], axis=1)
    return dict(v_mae=float(np.mean(np.abs(ev))), v_pairs=int(ok.sum()),
                p_mean=float(np.mean(d3)), p_end=float(d3[-1]),
                p_max=float(np.max(d3)), p_pairs=int(ok2.sum()),
                ref_path_m=float(np.sum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1))))


def calib_params():
    d = json.loads((PKG / "config" / "tram_calibration.json").read_text(encoding="utf-8"))["params"]
    return Params.from_dict(d)


def _real_one(args):
    bag, gnss, use_map = args[:3]
    sheet = args[3] if len(args) > 3 else "yaml"
    inject = args[4] if len(args) > 4 else None
    ev, z = bag_events(bag, gnss)
    if inject == "nan_front_mid":
        ws = [i for i, e in enumerate(ev) if e[1] == "w" and e[2] == 0]
        i = ws[len(ws) // 2]
        e = ev[i]
        ev[i] = (e[0], e[1], e[2], e[3], float("nan"))
    rec = run_stream(make_runner(use_map, calib_params() if sheet == "calib" else None),
                     ev, budget_s=900.0)
    s = summarize(rec)
    s["gnss"] = gnss_metrics(rec, z)
    s["bag"] = bag
    return s


def unique_bags():
    import hashlib
    seen, out, dup = {}, [], []
    for f in sorted(CACHE.glob("3*.npz")):
        if f.stem.startswith("track_map"):
            continue
        z = np.load(f)
        h = hashlib.md5(z["front"].tobytes() + z["cmd"].tobytes()).hexdigest()
        if h in seen:
            dup.append((seen[h], f.stem))
            continue
        seen[h] = f.stem
        out.append(f.stem)
    return out, dup


def cmd_real(args):
    from concurrent.futures import ProcessPoolExecutor
    out = Tee(args.out / "real.txt")
    bags, dup = unique_bags()
    if args.quick:
        bags = bags[:12]
    out("# Реальные прогоны через Runner (без ROS), порядок по времени записи (как ros2 bag play).")
    out(f"# Уникальных прогонов: {len(bags)} (дубликатов по содержимому: {len(dup)})")
    out("# GNSS подаётся первые 3 с (как у судьи). Лист tram.yaml, карта config/track_map.npz.")
    out("# CPU предварительно: параллельно работают другие контейнеры.")
    t0 = time.time()
    with ProcessPoolExecutor(args.workers) as ex:
        res = list(ex.map(_real_one, [(b, "window", True) for b in bags]))
    out(f"# стена {time.time() - t0:.0f} с, воркеров {args.workers}")
    out("")
    out(f"{'прогон':<16}{'мин':>6}{'выход':>8}{'Гц':>6}{'NaN':>5}{'exc':>4}{'тишина,с':>9}"
        f"{'пачка':>6}{'лаг p99,мс':>11}{'лаг max':>9}{'вх→вых p50':>11}{'p99':>6}{'max':>7}"
        f"{'>100мс,%':>9}{'выз p99,мс':>11}{'выз max':>8}{'|ош v|':>7}{'3D ср,м':>8}")
    agg = {k: [] for k in ("rate_hz", "max_silence_s", "max_burst", "stamp_lag_p99_ms",
                           "stamp_lag_max_ms", "in2out_p50_ms", "in2out_p99_ms",
                           "in2out_max_ms", "in2out_gt100ms_frac", "in2out_gt250ms_frac",
                           "call_ms_p99", "call_ms_max", "ms_per_step")}
    n_nan = n_exc = 0
    for s in sorted(res, key=lambda s: s["bag"]):
        g = s["gnss"]
        for k in agg:
            if k in s:
                agg[k].append(s[k])
        n_nan += s["nonfinite_outputs"] > 0
        n_exc += bool(s["exc"])
        out(f"{s['bag']:<16}{s.get('stamp_span_s', 0) / 60:6.1f}{s['n_out']:8d}{s.get('rate_hz', 0):6.1f}"
            f"{s['nonfinite_outputs']:5d}{'Y' if s['exc'] else '-':>4}{s.get('max_silence_s', 0):9.2f}"
            f"{s.get('max_burst', 0):6d}{s.get('stamp_lag_p99_ms', 0):11.0f}{s.get('stamp_lag_max_ms', 0):9.0f}"
            f"{s.get('in2out_p50_ms', 0):11.0f}{s.get('in2out_p99_ms', 0):6.0f}{s.get('in2out_max_ms', 0):7.0f}"
            f"{100 * s.get('in2out_gt100ms_frac', 0):9.2f}{s.get('call_ms_p99', 0):11.2f}{s.get('call_ms_max', 0):8.1f}"
            f"{g.get('v_mae', float('nan')):7.3f}{g.get('p_mean', float('nan')):8.1f}")
    out("")
    out(f"# ИТОГО по {len(res)} прогонам: исключений {n_exc}, прогонов с NaN/inf на выходе {n_nan}")
    for k, v in agg.items():
        if v:
            out(f"   {k:<22} медиана {np.median(v):10.3f}   макс {np.max(v):10.3f}   мин {np.min(v):10.3f}")
    worst = sorted(res, key=lambda s: -s.get("max_silence_s", 0))[:5]
    out("# самые длинные «тишины» (вход идёт, публикаций нет):")
    for s in worst:
        out(f"   {s['bag']}: {s.get('max_silence_s', 0):.2f} с в t={s.get('silence_at_s', 0):.1f} с от начала, "
            f"пачка {s.get('max_burst', 0)}")
    (args.out / "real.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float),
                                        encoding="utf-8")
    out.close()
    cmd_real_gnss(args)
    cmd_real_twobags(args)
    cmd_real_sheet(args)


def cmd_real_sheet(args):
    """Лист ноды (tram.yaml) против листа оценки (tram_calibration.json) и
    инъекция NaN в реальный прогон."""
    from concurrent.futures import ProcessPoolExecutor
    out = Tee(args.out / "real_sheet_and_nan.txt")
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    val = [b for b in dm["val"] if len(np.load(CACHE / f"{b}.npz")["mfix"]) > 100]
    if args.quick:
        val = val[:4]
    jobs = [(b, "window", True, sh) for b in val for sh in ("yaml", "calib")]
    with ProcessPoolExecutor(args.workers) as ex:
        res = list(ex.map(_real_one, jobs))
    by = {(s["bag"], i % 2): s for i, s in enumerate(res)}
    out("# Отложенные прогоны с GNSS, GNSS 3 с, карта. Лист ноды tram.yaml (q_v=%.2f) против листа"
        % PARAMS.q_v)
    out("# analysis/evaluate.py = config/tram_calibration.json (q_v=%.2f)." % calib_params().q_v)
    out(f"{'прогон':<16}{'|ош v| yaml':>12}{'|ош v| calib':>13}{'3D ср yaml':>11}{'3D ср calib':>12}")
    W = []
    for b in val:
        y, c = by[(b, 0)]["gnss"], by[(b, 1)]["gnss"]
        W.append((y, c))
        out(f"{b:<16}{y.get('v_mae', float('nan')):12.4f}{c.get('v_mae', float('nan')):13.4f}"
            f"{y.get('p_mean', float('nan')):11.1f}{c.get('p_mean', float('nan')):12.1f}")
    if W:
        out(f"# среднее: |ош v| yaml {np.mean([y['v_mae'] for y, _ in W]):.4f}, calib "
            f"{np.mean([c['v_mae'] for _, c in W]):.4f} м/с; 3D yaml {np.mean([y['p_mean'] for y, _ in W]):.2f}, "
            f"calib {np.mean([c['p_mean'] for _, c in W]):.2f} м")
    # NaN в реальный прогон
    b = val[0]
    for use_map in (True, False):
        s = _real_one((b, "window", use_map, "yaml", "nan_front_mid"))
        out(f"# {b}: одно показание передней тележки NaN в середине прогона, карта={use_map}: "
            f"исключение={s['exc']} @ {s['exc_where']}; обработано входов {s['n_in']}/{s['n_events']}; "
            f"NaN-выходов {s['nonfinite_outputs']} (поля {s.get('nonfinite_fields')}), последний конечен "
            f"{s.get('last_output_finite')}")
    out.close()


def cmd_real_gnss(args):
    """GNSS весь прогон (так нода получает обучающие прогоны при bag play)
    против GNSS только первые 3 с (как у судьи)."""
    from concurrent.futures import ProcessPoolExecutor
    out = Tee(args.out / "real_gnss_full_vs_window.txt")
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    val = [b for b in dm["val"] if len(np.load(CACHE / f"{b}.npz")["mfix"]) > 100]
    if args.quick:
        val = val[:4]
    jobs = [(b, g, True) for b in val for g in ("window", "full")]
    with ProcessPoolExecutor(args.workers) as ex:
        res = list(ex.map(_real_one, jobs))
    by = {(s["bag"], i % 2): s for i, s in enumerate(res)}
    out("# Отложенные прогоны с GNSS: GNSS подаётся в Runner только первые 3 с (как у судьи)")
    out("# против GNSS весь прогон (как нода получит его из подписки на /sensing/gnss/*/fix,")
    out("# если GNSS в проверочном bag есть дольше окна или при показе на обучающих прогонах).")
    out(f"{'прогон':<16}{'путь,км':>8}{'|ош v| окно':>12}{'|ош v| весь':>12}{'3D ср окно':>11}"
        f"{'3D ср весь':>11}{'3D конец окно':>14}{'3D конец весь':>14}")
    rows = []
    for b in val:
        w, f = by[(b, 0)]["gnss"], by[(b, 1)]["gnss"]
        rows.append((w, f))
        out(f"{b:<16}{w.get('ref_path_m', 0) / 1000:8.2f}{w.get('v_mae', float('nan')):12.3f}"
            f"{f.get('v_mae', float('nan')):12.3f}{w.get('p_mean', float('nan')):11.1f}"
            f"{f.get('p_mean', float('nan')):11.1f}{w.get('p_end', float('nan')):14.1f}"
            f"{f.get('p_end', float('nan')):14.1f}")
    if rows:
        out(f"# среднее 3D: окно {np.mean([w['p_mean'] for w, _ in rows]):.1f} м, "
            f"весь GNSS {np.mean([f['p_mean'] for _, f in rows]):.1f} м; "
            f"|ош v|: {np.mean([w['v_mae'] for w, _ in rows]):.4f} vs {np.mean([f['v_mae'] for _, f in rows]):.4f} м/с")
    # скорость: влияет ли GNSS после окна на скорость (через шаги сетки)?
    b = val[0]
    recs = {}
    for g in ("window", "full"):
        ev, z = bag_events(b, g)
        recs[g] = run_stream(make_runner(True), ev, budget_s=900.0)
    c = compare(recs["full"], recs["window"], None)
    out(f"# {b}: max|v_full - v_window| = {c.get('max_dv', float('nan')):.4f} м/с на "
        f"{c.get('common_stamps')} общих метках; max|dp| = {c.get('max_dp', float('nan')):.1f} м")
    out.close()


def cmd_real_twobags(args):
    """Два прогона подряд без перезапуска ноды (судья проигрывает несколько
    bag в одну ноду?)."""
    out = Tee(args.out / "real_two_bags.txt")
    starts = []
    for f in sorted(CACHE.glob("3*.npz")):
        z = np.load(f)
        if len(z["front"]):
            starts.append((float(z["front"][0, 1]), float(z["front"][-1, 1]), f.stem))
    starts.sort()
    gaps = np.array([b[0] - a[1] for a, b in zip(starts, starts[1:])])
    out(f"# Прогонов {len(starts)}; промежуток «конец одного -> начало следующего по времени»:")
    out(f"#   мин {gaps.min():.0f} с, медиана {np.median(gaps):.0f} с, макс {gaps.max():.0f} с;"
        f" отрицательных (перекрытие) {int((gaps < 0).sum())}")
    out(f"#   шагов сетки на медианный промежуток: {np.median(gaps) / PARAMS.dt:.0f}")
    # реальная пара: A, затем B, где B позже A (типичный порядок по имени - случайный)
    bags = sorted(p.stem for p in CACHE.glob("3*.npz"))
    a, b = bags[0], bags[1]
    ea, _ = bag_events(a, "window")
    eb, _ = bag_events(b, "window")
    ta = (ea[0][3], ea[-1][3])
    tb_ = (eb[0][3], eb[-1][3])
    out(f"# пара по имени: {a} [{ta[0]:.0f}..{ta[1]:.0f}] затем {b} [{tb_[0]:.0f}..{tb_[1]:.0f}]; "
        f"промежуток {tb_[0] - ta[1]:.0f} с = {(tb_[0] - ta[1]) / PARAMS.dt:.0f} шагов")
    shift = ea[-1][0] - eb[0][0] + 1.0
    ev = ea + [(e[0] + shift, *e[1:]) for e in eb]
    r = make_runner(True)
    rec = run_stream(r, ev, budget_s=120.0)
    s = summarize(rec)
    na = sum(1 for _ in ea)
    out(f"# результат: исключение={s['exc']}, таймаут={s['timeout']}, выходов {s['n_out']}, "
        f"входов обработано {s['n_in']}/{len(ev)} (первый прогон {na}), тишина {s.get('max_silence_s', 0):.0f} с")
    if s["timeout"]:
        sp = s.get("hang_steps_per_s") or float("nan")
        gap = tb_[0] - ta[1]
        out(f"#   зависание в одном вызове: {sp:.0f} шагов/с -> на весь промежуток "
            f"{gap / PARAMS.dt / sp / 60:.0f} мин; выходов в одной пачке было бы {gap / PARAMS.dt:.0f}")
    out.close()


# ============================================================ ресурсы

def cmd_resources(args):
    import tracemalloc
    out = Tee(args.out / "resources.txt")
    out("# Ресурсы. CPU предварительно: параллельно работают другие контейнеры.")
    # 1) CPU на реальном самом длинном прогоне, с картой
    longest = None
    for f in CACHE.glob("3*.npz"):
        z = np.load(f)
        if len(z["mfix"]) > 100:
            d = z["cmd"][-1, 1] - z["cmd"][0, 1]
            if longest is None or d > longest[0]:
                longest = (d, f.stem)
    ev, z = bag_events(longest[1], "window")
    gc.collect()
    rss0 = rss_mb()
    t0p = time.process_time()
    rec = run_stream(make_runner(True), ev, budget_s=1200.0)
    cpu = time.process_time() - t0p
    s = summarize(rec)
    steps = int(rec["call_nout"].sum())
    out(f"## самый длинный прогон с GNSS: {longest[1]}, {longest[0] / 60:.1f} мин, событий {len(ev)}, шагов {steps}")
    out(f"   CPU {cpu:.1f} с на {longest[0]:.0f} с данных = {100 * cpu / longest[0]:.1f} % одного ядра; "
        f"{1e3 * cpu / max(steps, 1):.3f} мс CPU на шаг")
    out(f"   время вызова (callback): p50 {s['call_ms_p50']:.3f} мс, p99 {s['call_ms_p99']:.3f} мс, "
        f"макс {s['call_ms_max']:.2f} мс; RSS {rss0:.0f} -> {rss_mb():.0f} МБ")
    # 2) 2 часа синтетики без трассировки: RSS, число объектов, CPU по ходу
    hours = 0.5 if args.quick else 2.0
    evs, _ = synth(hours * 3600.0, gnss="window")

    def feed_blocks(r, evs, block, row_fn):
        t_next = evs[0][3] + block
        i, n = 0, len(evs)
        while i < n:
            j = i
            while j < n and evs[j][3] < t_next:
                j += 1
            st = 0
            c0 = time.process_time()
            for tb, kind, arg, th, val in evs[i:j]:
                if kind == "w":
                    o = r.on_wheel(arg, th, val)
                elif kind == "h":
                    o = r.on_handle(th, val)
                else:
                    o = r.on_fix(th, arg, *val)
                st += len(o)
            cpu = time.process_time() - c0
            row_fn((t_next - evs[0][3]) / 60, cpu, st, j - i)
            i = j
            t_next += block

    r = make_runner(True)
    gc.collect()
    out(f"## синтетика {hours:.1f} ч ({len(evs)} событий), карта, БЕЗ трассировки (RSS и CPU честные)")
    out(f"{'t, мин':>8}{'RSS, МБ':>9}{'len(_acc)':>10}{'notch_hist':>11}{'_TABLES':>8}"
        f"{'cursor keys':>12}{'gc объектов':>12}{'мс CPU/шаг':>11}{'% ядра':>8}")
    rows = []

    def row_a(tmin, cpu, st, nev):
        gc.collect()
        row = (tmin, rss_mb(), len(r.pos._acc), len(r.core.notch_hist), len(ec._TABLES),
               len(r.pos._cursor) if r.pos._cursor else 0, len(gc.get_objects()),
               1e3 * cpu / max(st, 1), 100 * cpu / 600.0)
        rows.append(row)
        out(f"{row[0]:8.0f}{row[1]:9.1f}{row[2]:10d}{row[3]:11d}{row[4]:8d}{row[5]:12d}{row[6]:12d}"
            f"{row[7]:11.3f}{row[8]:8.2f}")
    feed_blocks(r, evs, 600.0, row_a)
    if len(rows) >= 3:
        a = np.array(rows)
        out(f"   RSS: после 10 мин {a[0, 1]:.1f} МБ, в конце {a[-1, 1]:.1f} МБ, наклон "
            f"{np.polyfit(a[:, 0], a[:, 1], 1)[0] * 60:+.2f} МБ/ч; объектов gc: {int(a[0, 6])} -> {int(a[-1, 6])}")
    # 2б) 20 мин с tracemalloc: где растёт память (по строкам)
    evs_b, _ = synth(1200.0, gnss="window")
    r = make_runner(True)
    gc.collect()
    tracemalloc.start(1)
    snap0 = tracemalloc.take_snapshot()
    trows = []

    def row_b(tmin, cpu, st, nev):
        cur, peak = tracemalloc.get_traced_memory()
        trows.append((tmin, cur / 2 ** 20, peak / 2 ** 20))
    feed_blocks(r, evs_b, 300.0, row_b)
    snap1 = tracemalloc.take_snapshot()
    tracemalloc.stop()
    out("## 20 мин с tracemalloc: traced-память (МБ) каждые 5 мин: "
        + ", ".join(f"{t:.0f} мин {c:.2f} (пик {pk:.2f})" for t, c, pk in trows))
    out("   крупнейшие приросты за 20 мин (по строкам):")
    for st_ in snap1.compare_to(snap0, "lineno")[:6]:
        fr = st_.traceback[0]
        out(f"     {Path(fr.filename).name}:{fr.lineno}  {st_.size_diff / 1024:+.1f} КБ  ({st_.count_diff:+d} блоков)")
    # 3) пачка выходов при скачке времени: память списка выходов
    out("## память пачки при скачке меток (Runner копит все выходы вызова в список)")
    for gap in (60.0, 600.0):
        r = make_runner(False)
        evs2, _ = synth(20.0)
        for tb, kind, arg, th, val in evs2:
            (r.on_wheel(arg, th, val) if kind == "w" else r.on_handle(th, val)
             if kind == "h" else r.on_fix(th, arg, *val))
        gc.collect()
        tracemalloc.start()
        c0 = time.perf_counter()
        outs = r.on_handle(r.t + gap, 0)
        el = time.perf_counter() - c0
        cur, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        out(f"   скачок {gap:.0f} с: {len(outs)} выходов, {el:.2f} с в одном вызове (с tracemalloc), "
            f"память списка {cur / 2 ** 20:.1f} МБ ({cur / max(len(outs), 1):.0f} Б/выход) -> "
            f"1 ч = {cur / max(len(outs), 1) * 72000 / 2 ** 20:.0f} МБ, 1 сут = "
            f"{cur / max(len(outs), 1) * 1728000 / 2 ** 30:.1f} ГБ")
        del outs
    out(f"## пик RSS процесса (VmHWM): {hwm_mb():.0f} МБ")
    out.close()


# ============================================================ данные

def cmd_data(args):
    out = Tee(args.out / "data.txt")
    fs = sorted(f for f in CACHE.glob("3*.npz"))
    out(f"# Свойства меток и значений в {len(fs)} прогонах (analysis/cache, tb - время записи, th - header.stamp)")
    agg = {}
    ratio, backs = [], []
    for f in fs:
        z = np.load(f)
        for k in ("front", "rear", "cmd", "mfix", "rfix"):
            a = z[k]
            if len(a) < 2:
                continue
            off = a[:, 1] - a[:, 0]
            dth = np.diff(a[:, 1])
            d = agg.setdefault(k, dict(n=0, bags=0, back=0, dup=0, zero=0, gap=[], rate=[],
                                        off_med=[], off_min=[], off_max=[], vmin=[], vmax=[],
                                        nan=0, ahead02=0))
            d["n"] += len(a)
            d["bags"] += 1
            d["back"] += int((dth < 0).sum())
            d["dup"] += int((dth == 0).sum())
            d["zero"] += int((a[:, 1] <= 0).sum())
            d["gap"].append(float(dth.max()))
            d["rate"].append(len(a) / max(a[-1, 1] - a[0, 1], 1e-9))
            d["off_med"].append(float(np.median(off)))
            d["off_min"].append(float(off.min()))
            d["off_max"].append(float(off.max()))
            d["ahead02"] += int((off > 0.2).sum())
            if k in ("front", "rear", "cmd"):
                d["vmin"].append(float(np.nanmin(a[:, 2])))
                d["vmax"].append(float(np.nanmax(a[:, 2])))
                d["nan"] += int((~np.isfinite(a[:, 2])).sum())
            backs += list(-dth[dth < 0])
        g = z["mvel"]
        if len(g) > 100:
            vg = np.hypot(g[:, 2], g[:, 3])
            fr = np.interp(g[:, 1], z["front"][:, 1], z["front"][:, 2])
            m = vg > 3
            if m.sum() > 50:
                ratio.append(float(np.median(fr[m] / vg[m])))
    out(f"{'топик':<7}{'сообщ.':>9}{'прог.':>6}{'Гц мед':>8}{'макс пропуск th,с':>18}{'назад':>7}"
        f"{'дубл.':>6}{'stamp<=0':>9}{'th-tb мед,с':>12}{'th-tb мин':>10}{'th-tb макс':>11}{'th>tb+0.2':>10}"
        f"{'min':>8}{'max':>8}{'NaN':>5}")
    for k, d in agg.items():
        out(f"{k:<7}{d['n']:9d}{d['bags']:6d}{np.median(d['rate']):8.2f}{max(d['gap']):18.2f}{d['back']:7d}"
            f"{d['dup']:6d}{d['zero']:9d}{np.median(d['off_med']):12.3f}{min(d['off_min']):10.2f}"
            f"{max(d['off_max']):11.2f}{d['ahead02']:10d}"
            + (f"{min(d['vmin']):8.2f}{max(d['vmax']):8.2f}{d['nan']:5d}" if d['vmin'] else ""))
    r = np.array(ratio)
    out(f"# единицы тележек: медиана (тележка / |v GNSS|) при v>3 м/с по {r.size} прогонам = "
        f"{np.median(r):.4f} (p5 {np.percentile(r, 5):.4f}, p95 {np.percentile(r, 95):.4f}) -> км/ч, не м/с")
    b = np.array(backs)
    out(f"# шаги меток назад внутри топика: {b.size}, медиана {np.median(b):.3f} с, макс {b.max():.3f} с")
    bags, dup = unique_bags()
    out(f"# дубликаты прогонов (совпадают front и cmd): {len(dup)} пар; уникальных {len(bags)}")
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    tr, va = set(dm["train"]), set(dm["val"])
    leak = [(a, b) for a, b in dup if (a in tr and b in va) or (a in va and b in tr)]
    both_val = [(a, b) for a, b in dup if a in va and b in va]
    out(f"# утечка train/val в analysis/drive_model.json: {len(leak)} пар «копия в train, копия в val»: {leak}")
    out(f"#   пары, где обе копии в val (двойной вес в среднем): {both_val}")
    nog = [f.stem for f in fs if len(np.load(f)["mfix"]) == 0]
    out(f"# прогонов без GNSS master: {len(nog)}")
    mv = []
    for f in fs:
        g = np.load(f)["mvel"]
        if len(g):
            w = g[:, 1] <= g[0, 1] + 3.0
            mv.append(float(np.hypot(g[w, 2], g[w, 3]).max()))
    mv = np.array(mv)
    out(f"# вагон в окне выставки (первые 3 с GNSS): прогонов с GNSS {mv.size}, из них max|v| > 0,5 м/с: "
        f"{int((mv > 0.5).sum())}, > 3 м/с: {int((mv > 3).sum())}")
    out.close()


# ============================================================ статика

def cmd_static(args):
    import re
    out = Tee(args.out / "static.txt")
    ok = lambda c: "OK " if c else "FAIL"          # noqa: E731
    # 1) лист ноды = калибровка оценки
    calib = json.loads((PKG / "config" / "tram_calibration.json").read_text(encoding="utf-8"))["params"]
    pc = Params.from_dict(calib)
    diffs = []
    for f in ec.fields(Params):
        a, b = getattr(PARAMS, f.name), getattr(pc, f.name)
        if isinstance(a, tuple):
            same = len(a) == len(b) and all(
                (x == y) if isinstance(x, (bool, str)) else abs(float(x) - float(y)) <= 1e-6 * max(1, abs(float(y)))
                for x, y in zip(a, b))
        elif isinstance(a, str):
            same = a == b
        else:
            same = abs(float(a) - float(b)) <= 1e-6 * max(1.0, abs(float(b)))
        if not same:
            diffs.append((f.name, a if not isinstance(a, tuple) else f"{len(a)} знач.",
                          b if not isinstance(b, tuple) else f"{len(b)} знач."))
    out(f"[{ok(not diffs)}] tram.yaml (лист ноды) == tram_calibration.json (лист evaluate.py); расхождений: {len(diffs)}")
    for d in diffs[:20]:
        out(f"       {d[0]}: yaml={d[1]}  calib={d[2]}")
    # 2) имена
    y = yaml.safe_load((PKG / "config" / "tram.yaml").read_text(encoding="utf-8"))
    ns = list(y.keys())
    node_src = (PKG / "tram_state_estimator" / "tram_node.py").read_text(encoding="utf-8")
    node_name = re.search(r'super\(\).__init__\("([^"]+)"\)', node_src).group(1)
    launch = (PKG / "launch" / "tram.launch.py").read_text(encoding="utf-8")
    lname = re.search(r'executable="tram_estimator",\s*name="([^"]+)"', launch)
    out(f"[{ok(ns == ['/' + node_name])}] пространство имён yaml {ns} vs имя ноды '{node_name}'"
        f"; имя в tram.launch.py: {lname.group(1) if lname else '?'}")
    # 3) топики и типы
    topics = re.findall(r'(VelocitySensor|Odometry|DriverControllerCommand|NavSatFix|AccelStamped|EstimatorStatus),\s*\n?\s*f?"([^"]+)"', node_src)
    want = {("VelocitySensor", "/result/velocity"), ("Odometry", "/result/position"),
            ("VelocitySensor", "/vehicle/front_bogie_velocity"),
            ("VelocitySensor", "/vehicle/rear_bogie_velocity"),
            ("DriverControllerCommand", "/vehicle/driver_position_cmd")}
    got = set(topics)
    out(f"[{ok(want <= got)}] топики/типы в tram_node.py: {sorted(got)}")
    # 4) зависимости
    pxml = (PKG / "package.xml").read_text(encoding="utf-8")
    deps = set(re.findall(r"<(?:exec_)?depend>([^<]+)</", pxml))
    imports = set()
    for f in list((PKG / "tram_state_estimator").glob("*.py")) + list((PKG / "launch").glob("*.py")):
        for m in re.finditer(r"^\s*(?:from|import)\s+([a-zA-Z_][\w]*)", f.read_text(encoding="utf-8"), re.M):
            imports.add(m.group(1))
    stdlib = {"bisect", "collections", "dataclasses", "json", "math", "os", "sys", "time",
              "tram_state_estimator", "__future__"}
    mapping = {"numpy": "python3-numpy"}
    missing = sorted(i for i in imports - stdlib if mapping.get(i, i) not in deps)
    out(f"[{ok(not missing)}] импорты без <depend>/<exec_depend> в package.xml: {missing}")
    # 5) установка файлов
    setup = (PKG / "setup.py").read_text(encoding="utf-8")
    need = ["config/tram.yaml", "config/track_map.npz", "launch/tram.launch.py"]
    miss = [n for n in need if n not in setup]
    out(f"[{ok(not miss)}] setup.py data_files ставит {need}: нет {miss}")
    out(f"[{ok('map_file' in y['/tram_state_estimator']['ros__parameters'])}] map_file = "
        f"{y['/tram_state_estimator']['ros__parameters'].get('map_file')!r} (относительный -> share пакета)")
    # 6) единицы
    out(f"[INFO] meas_units = {PARAMS.meas_units}, meas_scale = {PARAMS.meas_scale}, dt = {PARAMS.dt} с "
        f"(сетка {1 / PARAMS.dt:.0f} Гц)")
    # 7) числа в Odometry
    out("[INFO] Odometry: frame_id=map, child=base_link, pose.cov[0]=[7]=sigma_s^2 (вдоль пути, "
        "изотропно по x/y), cov[14] (z) = 0, ориентация: cov 0; до выставки quaternion = (0,0,0,0)")
    out.close()


# ============================================================ причинность

class CausalRunner(Runner):
    """Runner с проверкой: на шаге t используются только показания с меткой
    <= t (нет данных «из будущего»), и сколько лет «свежим» показаниям."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.viol_w = 0
        self.viol_h = 0
        self.max_future = 0.0
        self.ages = []
        self.n_steps = 0
        self.t_first = None

    def _step(self):
        t = self.t
        if self.t_first is None:
            self.t_first = t - self.p.dt
        self.n_steps += 1
        fr = self.fresh.copy()
        if fr.any():
            d = self.t_wheel[fr] - t
            self.viol_w += int((d > 1e-9).sum())
            self.max_future = max(self.max_future, float(d.max()))
            self.ages += list(-d)
        if np.isfinite(self.t_notch) and self.t_notch > t + 1e-9:
            self.viol_h += 1
            self.max_future = max(self.max_future, self.t_notch - t)
        return super()._step()


def cmd_causality(args):
    out = Tee(args.out / "causality.txt")
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    bags = ["30618_2255aade"] + [b for b in dm["val"][:4]]
    out("# «Нет данных из будущего»: на каждом шаге сетки t проверяется, что метки")
    out("# использованных показаний тележек и ручки <= t. Плюс «возраст» свежих показаний")
    out("# (t - метка) и дрейф сетки (накопление t += dt на эпохе ~1.8e9 с).")
    for b in bags:
        ev, _ = bag_events(b, "window")
        r = CausalRunner(PARAMS, track_map=load_map(), origin=None,
                         wheel_timeout=NODE.get("wheel_timeout_s", 1.0),
                         handle_timeout=NODE.get("handle_timeout_s", 0.5))
        r.pos.init_window = NODE.get("init_window_s", 3.0)
        rec = run_stream(r, ev, budget_s=900.0)
        a = np.array(r.ages)
        drift = r.t - (r.t_first + r.n_steps * PARAMS.dt)
        out(f"{b}: шагов {r.n_steps}; нарушений тележки {r.viol_w}, ручка {r.viol_h}, "
            f"макс опережение {r.max_future * 1e3:.3f} мс; возраст свежих показаний: p50 "
            f"{pct(a, 50) * 1e3:.0f} мс, p99 {pct(a, 99) * 1e3:.0f} мс, макс {a.max() * 1e3:.0f} мс, "
            f"> 0,5 с: {int((a > 0.5).sum())}; дрейф сетки за прогон {drift * 1e6:.1f} мкс; exc={rec['exc']}")
    out.close()


# ============================================================ запаздывание

class LagCompRunner(Runner):
    """Предлагаемая правка (НЕ в файлах пакета): показания тележек
    приводятся к моменту шага сетки, z + a·(t − stamp), a - ускорение
    прошлого выхода. Метка показания известна Runner (t_wheel)."""

    def _step(self):
        if self.last is not None and self.fresh.any():
            age = np.clip(self.t - self.t_wheel, 0.0, 0.3)
            saved = self.meas.copy()
            k = 3.6 / self.p.meas_scale if self.p.meas_units == "km_h" else 1.0
            self.meas = np.where(self.meas > 0, self.meas + self.last["a"] * age * k, self.meas)
            try:
                return super()._step()
            finally:
                self.meas = saved
        return super()._step()


def _lag_one(args):
    b, variant = args
    ev, z = bag_events(b, "window")
    cls = LagCompRunner if variant == "comp" else Runner
    r = cls(PARAMS, track_map=None, origin=None,
            wheel_timeout=NODE.get("wheel_timeout_s", 1.0),
            handle_timeout=NODE.get("handle_timeout_s", 0.5))
    r.pos.init_window = NODE.get("init_window_s", 3.0)
    rec = run_stream(r, ev, budget_s=900.0)
    T, V = rec["T"], rec["V"]
    g = z["mvel"]
    tg, vg = g[:, 1], np.hypot(g[:, 2], g[:, 3])
    acc = np.gradient(vg, tg)
    m = (tg > T[0] + 5) & (tg < T[-1] - 1)
    e = np.interp(tg[m], T, V) - vg[m]
    ok = np.abs(e) < 2.0            # без выбросов самого эталона
    shifts = np.arange(-0.2, 0.2001, 0.025)
    maes = [float(np.mean(np.abs(np.interp(tg[m] + sh, T, V) - vg[m])[ok])) for sh in shifts]
    a = acc[m]

    def bias(msk):
        msk = msk & ok
        return float(np.mean(e[msk])) if msk.sum() > 50 else float("nan")
    return dict(bag=b, variant=variant, mae=float(np.mean(np.abs(e[ok]))),
                best_shift=float(shifts[int(np.argmin(maes))]),
                b_acc=bias(a > 0.3), b_brk=bias(a < -0.3),
                b_steady=bias((np.abs(a) < 0.1) & (vg[m] > 1.0)),
                b_stop=bias(vg[m] < 0.2))


def cmd_lag(args):
    from concurrent.futures import ProcessPoolExecutor
    out = Tee(args.out / "lag.txt")
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    val = [b for b in dm["val"] if len(np.load(CACHE / f"{b}.npz")["mfix"]) > 100]
    if args.quick:
        val = val[:4]
    jobs = [(b, v) for b in val for v in ("orig", "comp")]
    with ProcessPoolExecutor(args.workers) as ex:
        res = list(ex.map(_lag_one, jobs))
    out("# Запаздывание выхода относительно эталона GNSS (|v| master, метки header).")
    out("# best_shift: сдвиг s, при котором v_out(t+s) лучше всего совпадает с v_GNSS(t);")
    out("# > 0 - выход отстаёт. Смещение по фазам: a>0,3 разгон, a<-0,3 торможение,")
    out("# |a|<0,1 и v>1 ход, v<0,2 стоянка. Без карты, лист tram.yaml, GNSS 3 с.")
    out("# orig - Runner пакета; comp - та же связка с приведением показаний к моменту шага.")
    out(f"{'прогон':<16}{'вариант':<8}{'|ош v|':>8}{'сдвиг,с':>9}{'разгон':>9}{'тормоз':>9}{'ход':>9}{'стоянка':>9}")
    for r in sorted(res, key=lambda r: (r["bag"], r["variant"] != "orig")):
        out(f"{r['bag']:<16}{r['variant']:<8}{r['mae']:8.4f}{r['best_shift']:+9.3f}{r['b_acc']:+9.4f}"
            f"{r['b_brk']:+9.4f}{r['b_steady']:+9.4f}{r['b_stop']:+9.4f}")
    for v in ("orig", "comp"):
        rr = [r for r in res if r["variant"] == v]
        out(f"# {v}: среднее |ош v| {np.mean([r['mae'] for r in rr]):.4f} м/с, сдвиг {np.mean([r['best_shift'] for r in rr]):+.3f} с, "
            f"смещение разгон {np.nanmean([r['b_acc'] for r in rr]):+.4f}, тормоз {np.nanmean([r['b_brk'] for r in rr]):+.4f}, "
            f"ход {np.nanmean([r['b_steady'] for r in rr]):+.4f}, стоянка {np.nanmean([r['b_stop'] for r in rr]):+.4f} м/с")
    out.close()


# ============================================================ проверка правок

class PatchedRunner(Runner):
    """ПРЕДЛАГАЕМЫЕ минимальные правки runner.py (здесь - подклассом, файлы
    пакета не меняются). Каждая правка помечена строкой runner.py.

    1. on_wheel/on_handle/on_fix: метка и значение должны быть конечны,
       метка > 0; |тележка| <= 1,5 * v_max_line (в единицах листа); ручка
       ограничивается ±15; GNSS: конечные lat/lon/alt, не (0, 0).
    2. _advance: разрыв времени > MAX_JUMP вперёд или назад. Одиночное
       сообщение с такой меткой отбрасывается; если следующее сообщение
       подтверждает разрыв (новый bag, перезапуск проигрывания) - связка
       перезапускается (ядро и выставка заново, карта та же).
    3. on_fix: после окна выставки GNSS не вызывает _advance и ничего не
       меняет; s0 переустанавливается, только когда выставка реально
       уточнилась (добавлена пара master/rover).
    """
    MAX_JUMP = 10.0

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self._init_args = (a, k)
        self._pend = None
        self.resets = 0
        self.rejected = 0

    def _reset(self):
        a, k = self._init_args
        iw, resets, rej = self.pos.init_window, self.resets, self.rejected
        Runner.__init__(self, *a, **k)
        self.pos.init_window = iw
        self._pend, self.resets, self.rejected = None, resets + 1, rej

    def _advance(self, stamp):                              # runner.py:161
        if not math.isfinite(stamp) or stamp <= 0.0:
            self.rejected += 1
            return None
        if self.t is not None and abs(stamp - self.t) > self.MAX_JUMP:
            if self._pend is not None and abs(stamp - self._pend) <= self.MAX_JUMP:
                self._reset()                               # разрыв подтверждён
                self.t = stamp
                return []
            self._pend = stamp                              # одиночный выброс
            self.rejected += 1
            return None
        self._pend = None
        return super()._advance(stamp)

    def on_wheel(self, i, stamp, value):                    # runner.py:136
        lim = 1.5 * self.p.v_max_line * (3.6 if self.p.meas_units == "km_h" else 1.0)
        out = self._advance(stamp)
        if out is None:
            return []
        if not (math.isfinite(value) and abs(value) <= lim):
            self.rejected += 1
            return out
        self.meas[i] = value
        self.fresh[i] = True
        self.t_wheel[i] = stamp
        return out

    def on_handle(self, stamp, position):                   # runner.py:143
        out = self._advance(stamp)
        if out is None:
            return []
        self.notch = float(min(15, max(-15, position)))
        self.t_notch = stamp
        return out

    def on_fix(self, stamp, antenna, lat, lon, alt):        # runner.py:149
        pos = self.pos
        vals = (stamp, lat, lon, alt)
        if not all(math.isfinite(x) for x in vals) or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
            return []
        if pos._t0 is not None and stamp - pos._t0 > pos.init_window:
            return []                                       # окно закрыто: GNSS ни на что не влияет
        out = self._advance(stamp)
        if out is None:
            return []
        moved = (float(self.core.x[ec.IS]) - self.s0) if self.s0 is not None else 0.0
        was, n0 = pos.ready, len(pos._acc)
        pos.on_fix(stamp, antenna, lat, lon, alt, moved)
        if pos.ready and was and len(pos._acc) > n0:        # runner.py:155
            self.s0 = float(self.core.x[ec.IS])
        return out


FIX_NAMES = ["base", "nan_front_1", "nan_first_wheel", "inf_front_1", "huge_first_wheel",
             "stamp0_first_msg", "stamp_future_1h_one", "stamp_future_10s_one", "clock_back_100s",
             "second_bag_earlier", "second_bag_later_1h", "second_bag_later_1d",
             "gnss_nan_alt", "gnss_nofix_zero_first", "gnss_full_run", "gnss_start_moving",
             "gnss_future_stamp", "handle_out_of_range"]


def _mk(cls, use_map=False):
    r = cls(PARAMS, track_map=load_map() if use_map else None, origin=None,
            wheel_timeout=NODE.get("wheel_timeout_s", 1.0),
            handle_timeout=NODE.get("handle_timeout_s", 0.5))
    r.pos.init_window = NODE.get("init_window_s", 3.0)
    return r


def cmd_fixcheck(args):
    from concurrent.futures import ProcessPoolExecutor
    out = Tee(args.out / "fixcheck.txt")
    out("# Проверка предлагаемых минимальных правок runner.py (PatchedRunner в этом файле).")
    out("# Синтетика без карты (положение против истины осмысленно): оригинал против правки.")
    sc = {n: (d, m, a, o) for n, d, m, a, o in scenarios()}
    out(f"{'сценарий':<24}{'вариант':<9}{'вердикт':<44}{'NaN':>6}{'тишина,с':>9}{'|dv| конец':>11}"
        f"{'|dp| конец,м':>13}{'ош.пол. кон,м':>14}{'сбросов':>8}")
    for n in FIX_NAMES:
        desc, mut, anom, opts = sc[n]
        ev, tru = synth(300.0, gnss=opts.get("gnss", "window"), t_skip=opts.get("t_skip", 0.0))
        if opts.get("nofix"):
            ev = nofix_mut(ev)
        if mut is not None:
            ev = mut(ev)
        bev, _ = synth(300.0, gnss="window", t_skip=opts.get("t_skip", 0.0))
        for var, cls in (("orig", Runner), ("patched", PatchedRunner)):
            base = run_stream(_mk(cls), bev)
            r = _mk(cls)
            rec = run_stream(r, ev, budget_s=min(opts.get("budget", 120.0), 60.0))
            s = summarize(rec)
            c = compare(rec, base, (anom[0] + anom[1]) if anom else None) if not opts.get("second") else {}
            tr = vs_truth(rec, tru) if not opts.get("second") else {}
            out(f"{n:<24}{var:<9}{verdict(s, c)[:43]:<44}{s['nonfinite_outputs']:>6}"
                f"{s.get('max_silence_s', 0):>9.1f}{c.get('end_dv', float('nan')):>11.3f}"
                f"{c.get('end_dp', float('nan')):>13.1f}{tr.get('pos_err_end', float('nan')):>14.1f}"
                f"{getattr(r, 'resets', 0):>8d}")
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    val = [b for b in dm["val"] if len(np.load(CACHE / f"{b}.npz")["mfix"]) > 100]
    extra = ["30618_2255aade", "30618_28538acf", "30618_40ffd323", "30618_af7496f0"]
    bags = (val[:4] if args.quick else val) + [b for b in extra if b not in val]
    with ProcessPoolExecutor(args.workers) as ex:
        res = list(ex.map(_fix_real_one, bags))
    out("")
    out("# Реальные прогоны с картой: оригинал vs правка. max|dv|, max|dp| - расхождение правки")
    out("# с оригиналом при GNSS 3 с (ожидается 0: правки не должны менять штатный режим).")
    out("# 3D ср - средняя 3D-ошибка против GNSS master при GNSS весь прогон и 3 с.")
    out(f"{'прогон':<16}{'max|dv| окно':>13}{'max|dp| окно':>13}{'сбросов':>8}{'отброшено':>10}"
        f"{'3D ориг весь':>13}{'3D правка весь':>15}{'3D ориг окно':>13}")
    for r in res:
        out(f"{r['bag']:<16}{r['dv']:13.2e}{r['dp']:13.2e}{r['resets']:8d}{r['rejected']:10d}"
            f"{r['p_orig_full']:13.1f}{r['p_fix_full']:15.1f}{r['p_orig_win']:13.1f}")
    out.close()


def _fix_real_one(b):
    res = dict(bag=b)
    recs = {}
    for g in ("window", "full"):
        for var, cls in (("orig", Runner), ("patched", PatchedRunner)):
            ev, z = bag_events(b, g)
            r = _mk(cls, use_map=True)
            rec = run_stream(r, ev, budget_s=900.0)
            rec["gm"] = gnss_metrics(rec, z)
            recs[(g, var)] = rec
            if var == "patched" and g == "window":
                res["resets"], res["rejected"] = r.resets, r.rejected
    c = compare(recs[("window", "patched")], recs[("window", "orig")], None)
    res["dv"], res["dp"] = c.get("max_dv", float("nan")), c.get("max_dp", float("nan"))
    res["p_orig_full"] = recs[("full", "orig")]["gm"].get("p_mean", float("nan"))
    res["p_fix_full"] = recs[("full", "patched")]["gm"].get("p_mean", float("nan"))
    res["p_orig_win"] = recs[("window", "orig")]["gm"].get("p_mean", float("nan"))
    return res


# ============================================================ сводка

def cmd_summary(args):
    """Сводная таблица фаззинга из fuzz.json (с картой) и fuzz_nomap.json."""
    out = Tee(args.out / "fuzz_summary.txt")
    J = {}
    for tag, fn in (("map", "fuzz.json"), ("nomap", "fuzz_nomap.json")):
        f = args.out / fn
        J[tag] = json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}
    names = list(J["nomap"] or J["map"])
    out("# Сводка фаззинга. «с картой» = конфигурация ноды (tram.yaml + track_map.npz);")
    out("# «без карты» - та же связка без карты: видно поведение ядра и положение против истины.")
    out("# dp - |положение − эталонный прогон без аномалии| в конце, м; ош.пол. - против истины (без карты).")
    out(f"{'сценарий':<24}{'с картой':<30}{'без карты':<30}{'NaN вых':>8}{'тишина,с':>9}"
        f"{'восст.,с':>9}{'dp конец':>10}{'ош.пол.':>9}{'|ош v|':>8}{'макс.вызов,мс':>14}")

    def fmt(x, f="{:.1f}"):
        if x is None:
            return "-"
        if isinstance(x, float) and math.isinf(x):
            return "нет"
        if isinstance(x, float) and math.isnan(x):
            return "nan"
        return f.format(x)
    for n in names:
        a, b = J["map"].get(n, {}), J["nomap"].get(n, {})

        def vd(r):
            if not r:
                return "-"
            v = r["verdict"]
            c = r.get("vs_base", {})
            dp = c.get("end_dp")
            if dp is not None and isinstance(dp, float) and math.isfinite(dp) and dp > 50 and "POS" not in v:
                v = (v + f", POS-DELTA {dp:.0f}m") if v != "OK" else f"POS-DELTA {dp:.0f}m"
            return v[:29]
        sb = b.get("summary", {})
        cb = b.get("vs_base", {})
        tb = b.get("vs_truth", {})
        sa = a.get("summary", {})
        out(f"{n:<24}{vd(a):<30}{vd(b):<30}{sb.get('nonfinite_outputs', 0):>8}"
            f"{fmt(sb.get('max_silence_s')):>9}{fmt(cb.get('recovery_s')):>9}"
            f"{fmt(cb.get('end_dp')):>10}{fmt(tb.get('pos_err_end')):>9}{fmt(tb.get('v_mae'), '{:.3f}'):>8}"
            f"{fmt(sa.get('call_ms_max') or sb.get('call_ms_max'), '{:.0f}'):>14}")
    out.close()


# ============================================================ main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["static", "data", "causality", "lag", "fuzz", "real",
                                    "resources", "fixcheck", "summary", "all"])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", type=lambda s: s.split(","), default=None)
    ap.add_argument("--no-map", dest="map", action="store_false")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    cmds = (["static", "data", "causality", "lag", "fuzz", "real", "resources", "fixcheck", "summary"]
            if args.cmd == "all" else [args.cmd])
    for c in cmds:
        if c == "fuzz" and args.cmd == "all":       # обе конфигурации
            for m in (True, False):
                args.map = m
                cmd_fuzz(args)
            args.map = True
        else:
            globals()[f"cmd_{c}"](args)


if __name__ == "__main__":
    main()
