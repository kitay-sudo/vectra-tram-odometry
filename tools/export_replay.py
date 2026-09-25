#!/usr/bin/env python3
"""Экспорт прогона для режима «Прогон данных комиссии» (simulator/index.html).

Оценка считается той же связкой, что нода ROS 2 (tram_node.py): Runner из
ros2_ws/src/tram_state_estimator, лист параметров вагона, карта путей;
параметры ноды передаются в Runner так же, как их передаёт tram_node.py
(включая параметры выставки и системы выхода после WP потока «положение»).
Сообщения подаются в порядке записи bag, GNSS — только первые 3 с
(analysis/evaluate.events, как tools/eval.py). Рядом считаются:
  * причинная база «только колесо»: тот же Runner (сетка, выставка, карта,
    привязка к остановкам, система выхода — код пакета без копий), но на
    месте ядра — среднее свежих (не старше wheel_timeout) показаний тележек в
    м/с, путь — интеграл на сетке (как NaiveCore в tools/eval_replay.py);
  * эталон GNSS master (скорость |vel|, положение fix), rover vel — справочно;
  * входы (тележки, км/ч; ручка) и диагностика оценщика: поля
    EstimatorStatus и состояние каждой тележки (принята / нет новых данных /
    отвергнута / исключена / поток прерван).

Варианты с аномалиями строятся здесь же маленькими функциями inject_*
(та же семантика и выбор окна, что в tools/inject.py потока оценки; если
tools/inject.py есть — берётся он).

Честность (решение 7): прогон по умолчанию — отложенный (holdout_scored в
tools/split.json); карта — только из обучающих: config/eval/track_map.npz
пакета, если есть, иначе analysis/cache/track_map_train.npz; лист — по тем же
правилам, что tools/eval.py --sheet eval: оценочный config/eval/{tram_eval.yaml,
tram.yaml, tram_calibration.json}, а пока его нет — config/tram_calibration.json
с пометкой «УТЕЧКА» (она пишется в файл и показывается на странице; числа с
такой пометкой не отчётные).

Система координат на экране: система выхода модели (MGRS/UTM, ENU или
equirect — берётся из настроек Runner и проверяется по данным), сдвинутая в
первую точку GNSS master. Разрыв MGRS на границе квадратов 100 км снимается.
Судья считает в MGRS; ошибки в плане от сдвига начала не зависят.

Выход:
  simulator/replays/<run>_<variant>.js  — `TV_REPLAYS["<run>_<variant>"] = {...}`
                                          (грузится <script src> и с file://)
  simulator/replays/index.js            — список прогонов с итоговыми метриками

  docker run --rm --cpus 2 -v <worktree>:/repo \
      -v E:/MY-PROJECT/TrackVector/data:/repo/data:ro \
      -v E:/MY-PROJECT/TrackVector/analysis/cache:/repo/analysis/cache:ro \
      -w /repo vectra/tram:dev python3 tools/export_replay.py \
      [--run 30618_e9a34502] [--variants clean,front_zero,both_zero,dropout,skid_brake]
"""

import argparse
import dataclasses
import gzip
import hashlib
import inspect
import json
import math
import os
import re
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG = os.path.join(ROOT, "ros2_ws", "src", "tram_state_estimator")
for p in (os.path.join(ROOT, "analysis"), PKG, os.path.join(ROOT, "tools"),
          os.path.join(ROOT, "tools", "audit")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import bagio  # noqa: E402
import evaluate as E  # noqa: E402
from tram_state_estimator import estimator_core as EC  # noqa: E402
from tram_state_estimator import runner as RM  # noqa: E402
from tram_state_estimator.runner import Runner  # noqa: E402
from tram_state_estimator.track_map import TrackMap  # noqa: E402

KMH = 3.6
OUT_DIR = os.path.join(ROOT, "simulator", "replays")
DEFAULT_RUN = "30618_e9a34502"
CFG = os.path.join(PKG, "config")
# карта только из обучающих прогонов: пакетная EVAL-карта (поток «положение»,
# analysis/build_map.py eval), иначе кэш напарника (build_map по train)
MAP_CANDIDATES = (os.path.join(CFG, "eval", "track_map.npz"),
                  os.path.join(ROOT, "analysis", "cache", "track_map_train.npz"))
# оценочный лист — те же кандидаты и тот же запасной, что tools/eval_replay.py
EVAL_SHEETS = tuple(os.path.join(CFG, "eval", n)
                    for n in ("tram_eval.yaml", "tram.yaml", "tram_calibration.json"))
LEAK = ("УТЕЧКА: лист подогнан по всем записям, включая отложенные "
        "(оценочного листа config/eval/ ещё нет) — числа не отчётные")
VARIANTS = ("clean", "front_zero", "both_zero", "dropout", "skid_brake")
TOL = 0.05          # с: пара «выход — эталон» по ближайшей метке (README §5.1)
DECIM_DT = 0.1      # с: шаг строк экспорта (10 Гц)
T_SKIP = 60.0       # с от начала входов: окно аномалии не раньше
AFTER = 120.0       # с после окна должно остаться до конца прогона

# ------------------------------------------------------------------ аномалии
# Та же семантика, что tools/inject.py (поток оценки): окно по header.stamp,
# выбор окна детерминирован и только по входам решения (тележки, ручка).
KINDS = {
    "clean": dict(ru="Чистый прогон", dur=0.0, where=None),
    "front_zero": dict(ru="Отказ передней тележки: 0 км/ч 60 с", dur=60.0, where="moving_start"),
    "both_zero": dict(ru="Обе тележки 0 км/ч 20 с", dur=20.0, where="moving"),
    "dropout": dict(ru="Пропуск сообщений обеих тележек 2 с", dur=2.0, eval=5.0, where="moving"),
    "skid_brake": dict(ru="Юз: обе тележки −30 % на торможении 4 с", dur=4.0, where="brake"),
}


def _zoh(t_src, v_src, t_q):
    """Последнее значение источника на момент t_q (причинно)."""
    o = np.argsort(t_src, kind="stable")
    ts, vs = t_src[o], v_src[o]
    j = np.searchsorted(ts, t_q, side="right") - 1
    return np.where(j >= 0, vs[np.clip(j, 0, None)], np.nan)


def input_grid(a, step=0.1):
    """Сетка по меткам входов: t, скорость тележек (м/с), ручка."""
    f, r, c = a["front"], a["rear"], a["cmd"]
    ts = [x[:, 1] for x in (f, r, c) if len(x)]
    t = np.arange(min(float(x.min()) for x in ts), max(float(x.max()) for x in ts), step)
    ws = []
    for x in (f, r):
        if len(x) >= 2:
            o = np.argsort(x[:, 1], kind="stable")
            ws.append(np.interp(t, x[o, 1], x[o, 2]))
    w = np.nanmean(np.vstack(ws), axis=0) / KMH if ws else np.zeros_like(t)
    h = np.nan_to_num(_zoh(c[:, 1], c[:, 2], t), nan=0.0) if len(c) else np.zeros_like(t)
    return t, w, h


def eval_window(kind):
    k = KINDS[kind]
    return float(k.get("eval", k["dur"]))


def choose_window(a, kind):
    """Начало окна t0 (header.stamp): первое после T_SKIP, где выполнено
    условие where (см. tools/inject.py choose_window)."""
    k = KINDS[kind]
    dur = float(k["dur"])
    span = max(dur, eval_window(kind))
    t, w, h = input_grid(a)
    n = max(1, int(round(span / (t[1] - t[0]))))
    t_end = t[-1] - (AFTER if t[-1] - t[0] > T_SKIP + span + AFTER else 0.5 * (t[-1] - t[0]))
    for i in np.flatnonzero(t >= t[0] + T_SKIP):
        if t[i] + span > t_end:
            break
        seg = slice(i, min(i + n + 1, len(t)))
        ws, hs = w[seg], h[seg]
        where = k["where"]
        ok = (w[i] >= 5.0 if where == "moving_start" else
              ws.min() >= 4.0 if where == "moving" else
              (hs.max() <= -1 and w[i] >= 5.0 and ws.min() >= 1.5) if where == "brake" else False)
        if ok:
            return float(t[i]), dur
    return None, dur


def inject(a, kind, t0, dur):
    """Копия массивов прогона с аномалией в окне [t0, t0+dur) по header.stamp."""
    b = {key: v.copy() for key, v in a.items()}

    def win(x):
        return (x[:, 1] >= t0) & (x[:, 1] < t0 + dur)
    if kind == "front_zero":
        b["front"][win(b["front"]), 2] = 0.0
    elif kind == "both_zero":
        for key in ("front", "rear"):
            b[key][win(b[key]), 2] = 0.0
    elif kind == "skid_brake":
        for key in ("front", "rear"):
            b[key][win(b[key]), 2] *= 0.7
    elif kind == "dropout":
        for key in ("front", "rear"):
            b[key] = b[key][~win(b[key])]
    elif kind != "clean":
        raise ValueError(kind)
    return b


def make_variant(a, bag, kind):
    """(массивы, info). tools/inject.py потока оценки — если он есть."""
    if kind == "clean":
        return a, dict(kind=kind, ru=KINDS[kind]["ru"], t0=None, dur=0.0, eval=0.0, src="—")
    try:
        import inject as shared      # tools/inject.py (поток оценки)
        if kind in shared.KINDS:
            t0, dur = shared.choose_window(a, kind)
            if t0 is None:
                return None, None
            b, _ = shared.apply(a, kind, t0, dur, seed=shared.seed_for(bag, kind))
            return b, dict(kind=kind, ru=KINDS[kind]["ru"], t0=t0, dur=dur,
                           eval=float(shared.eval_window(kind)), src="tools/inject.py")
    except ImportError:
        pass
    t0, dur = choose_window(a, kind)
    if t0 is None:
        return None, None
    return inject(a, kind, t0, dur), dict(kind=kind, ru=KINDS[kind]["ru"], t0=t0, dur=dur,
                                           eval=eval_window(kind), src="tools/export_replay.py")


# ------------------------------------------------------------------ лист и связка

def _yaml_params(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        d = yaml.safe_load(fh)
    for v in d.values():
        if isinstance(v, dict) and "ros__parameters" in v:
            return v["ros__parameters"]
    raise SystemExit(f"{path}: нет ros__parameters")


def _rel(path):
    return os.path.relpath(path, ROOT).replace(os.sep, "/")


def resolve_sheet(spec=None):
    """Лист по правилам tools/eval.py --sheet (если tools/eval_replay.py есть —
    буквально его resolve_sheet, иначе те же правила здесь):
      None/'eval' — оценочный config/eval/{tram_eval.yaml, tram.yaml,
                    tram_calibration.json}; без него — config/tram_calibration.json
                    + параметры ноды из config/tram.yaml, с пометкой УТЕЧКА;
      'jury'      — боевой config/tram.yaml (УТЕЧКА на отложенных);
      путь        — yaml ноды или json калибровки.
    Возвращает (Params, параметры ноды, сведения о листе)."""
    spec = spec or "eval"
    names = {f.name for f in dataclasses.fields(EC.Params)}
    try:
        import eval_replay as ER                  # tools/eval_replay.py (поток оценки)
        sh = ER.resolve_sheet(spec)
        corep, node = dict(sh["core"]), dict(sh["node"])
        path, label, leak, src = str(sh["path"]), sh["label"], sh.get("leak"), "tools/eval_replay.py"
    except (ImportError, AttributeError):
        jury = os.path.join(CFG, "tram.yaml")
        node_defaults = {k: v for k, v in _yaml_params(jury).items() if k not in names}
        leak = None
        if spec == "eval":
            path = next((q for q in EVAL_SHEETS if os.path.exists(q)), None)
            if path is None:
                path, label, leak = (os.path.join(CFG, "tram_calibration.json"),
                                     "json (запасной: оценочного листа config/eval/ нет)", LEAK)
            else:
                label = f"оценочный ({_rel(path)}, только train)"
        elif spec == "jury":
            path, label = jury, "боевой config/tram.yaml (все данные)"
            leak = "УТЕЧКА: боевой лист подогнан по всем данным — числа на отложенных не отчётные"
        else:
            path = os.path.abspath(spec)
            label = _rel(path)
        if path.endswith(".json"):
            with open(path, encoding="utf-8") as fh:
                d = json.load(fh)
            d = d.get("params", d)
            corep = {k: v for k, v in d.items() if k in names}
            node = dict(node_defaults)
            node.update({k: v for k, v in d.items() if k not in names and not k.startswith("_")})
        else:
            d = _yaml_params(path)
            corep = {k: v for k, v in d.items() if k in names}
            node = {k: v for k, v in d.items() if k not in names}
        path, src = _rel(path), "tools/export_replay.py"
    # лист не из config/eval/ на отложенном прогоне — всегда пометка
    if not leak and "/config/eval/" not in "/" + path.replace(os.sep, "/"):
        leak = "лист не оценочный (не из config/eval/): на отложенных числа не отчётные"
    params = EC.Params.from_dict(corep)
    full = path if os.path.isabs(path) else os.path.join(ROOT, path)
    return params, node, dict(path=path, label=label, leak=leak or None, resolver=src,
                              sha1=sha1(full))


# Параметры ноды -> аргументы Runner так же, как tram_node.py: явные
# соответствия, остальное — по имени аргумента (x или x_s). Если у Runner есть
# **kwargs (поток «положение»: они уходят в Position), принимаются и
# аргументы Position.__init__ (projection, mgrs_grid, utm_zone, ...).
NODE_TO_ARG = {"wheel_timeout": "wheel_timeout_s", "handle_timeout": "handle_timeout_s",
               "init_window": "init_window_s"}
NOT_NODE = ("self", "params", "track_map", "origin")


def runner_arg_names():
    def named(fn):
        return [n for n, q in inspect.signature(fn).parameters.items()
                if q.kind not in (q.VAR_KEYWORD, q.VAR_POSITIONAL) and n not in NOT_NODE]
    sig = inspect.signature(Runner.__init__).parameters
    names = named(Runner.__init__)
    if any(q.kind == q.VAR_KEYWORD for q in sig.values()) and hasattr(RM, "Position"):
        names += [n for n in named(RM.Position.__init__) if n not in names]
    return names


def runner_kwargs(node, tmap):
    """(аргументы Runner, параметры ноды, не ушедшие в Runner)."""
    kw = dict(track_map=tmap)
    o = tuple(float(node.get(k, float("nan"))) for k in ("origin_lat", "origin_lon", "origin_alt"))
    kw["origin"] = o if all(math.isfinite(x) for x in o) else None
    # init_window_s: аргумент Runner/Position либо (код до правок) pos.init_window в make_runner
    used = {"origin_lat", "origin_lon", "origin_alt", "map_file", "frame_id", "child_frame_id",
            "init_window_s"}
    for name in runner_arg_names():
        key = NODE_TO_ARG.get(name) or (name if name in node else name + "_s")
        if key in node:
            kw[name] = node[key]
            used.add(key)
    return kw, sorted(k for k in node if k not in used)


def make_runner(params, node, tmap, cls=Runner):
    kw, _ = runner_kwargs(node, tmap)
    r = cls(params, **kw)
    # связка до WP потока «положение»: окно выставки задаётся после __init__
    # (как tram_node.py до правок)
    if "init_window" not in kw and "init_window_s" in node and hasattr(r, "pos") \
            and hasattr(r.pos, "init_window"):
        r.pos.init_window = float(node["init_window_s"])
    return r


def instrument_core():
    """Диагностика по тележкам без изменения поведения: обёртки методов
    Estimator запоминают маску свежих показаний, принятые оси и исправность;
    к выходу шага добавляется поле _dg."""
    C = EC.Estimator
    if getattr(C, "_tv_instrumented", False):
        return
    if not all(hasattr(C, n) for n in ("_axle_speeds", "_correct", "step", "step_open_loop")):
        print("внимание: у Estimator нет ожидаемых методов — состояние тележек не экспортируется")
        C._tv_instrumented = True
        return
    o_axle, o_corr, o_step, o_ol = C._axle_speeds, C._correct, C.step, C.step_open_loop

    def _axle_speeds(self, meas, fm):
        z, ok = o_axle(self, meas, fm)
        self._dg_fm = np.array(fm, dtype=bool).copy()
        self._dg_z = np.array(z, dtype=float).copy()
        return z, ok

    def _correct(self, z, ok, *a, **k):
        acc = o_corr(self, z, ok, *a, **k)
        self._dg_acc = [int(x) for x in acc]
        return acc

    def diag(self, open_loop):
        if not hasattr(self, "slots") or not hasattr(self, "healthy"):
            return dict(state=[9] * int(getattr(self, "nw", 2)))
        st = []
        for i in range(self.nw):
            ax = next((a for a, sl in enumerate(self.slots) if i in list(sl)), i)
            if open_loop:
                code = 4                                  # поток прерван
            elif not bool(self.healthy[i]):
                code = 3                                  # исключена
            elif self._dg_fm is None or not bool(self._dg_fm[i]):
                code = 1                                  # нет новых данных
            elif ax in self._dg_acc:
                code = 0                                  # принята
            else:
                code = 2                                  # отвергнута
            st.append(code)
        return dict(state=st, u=float(getattr(self, "u_filt", 0.0)))

    def step(self, *a, **k):
        self._dg_fm, self._dg_acc = None, []
        o = o_step(self, *a, **k)
        o["_dg"] = diag(self, False)
        return o

    def step_open_loop(self, *a, **k):
        self._dg_fm, self._dg_acc = None, []
        o = o_ol(self, *a, **k)
        o["_dg"] = diag(self, True)
        return o

    C._axle_speeds, C._correct, C.step, C.step_open_loop = _axle_speeds, _correct, step, step_open_loop
    C._tv_instrumented = True


class _NaiveCore(EC.Estimator):
    """База «только колесо» на месте ядра (семантика NaiveCore из
    tools/eval_replay.py): скорость — среднее свежих (не старше wheel_timeout
    связки) конечных показаний тележек в м/с, иначе прежняя; путь — интеграл
    на сетке p.dt. Остальные поля выхода нейтральные."""

    def __init__(self, params, runner):
        super().__init__(params)
        self._r = runner
        self._v = 0.0
        self._k = float(np.asarray(EC.sensor_to_speed(1.0, params)))

    def _naive(self):
        r, p = self._r, self.p
        vals = [float(r.meas[i]) for i in range(self.nw)
                if r.t - r.t_wheel[i] <= r.wheel_timeout and math.isfinite(float(r.meas[i]))]
        if vals:
            self._v = max(0.0, float(np.mean(vals)) * self._k)
        self.x[EC.IV] = self._v
        self.x[EC.IS] += self._v * p.dt
        stand = bool(vals) and self._v < p.v_standstill
        return dict(v=self._v, s=float(self.x[EC.IS]), d=0.0, k_t=1.0, k_b=1.0,
                    mu=float(self.mu), sigma_v=0.0, sigma_s=0.0,
                    mode=EC.STANDSTILL if stand else EC.COAST,
                    healthy=np.ones(self.nw, dtype=bool), slip=False, ambiguous=False,
                    n_accepted=len(vals), n_rejected=0, odometry_used=bool(vals),
                    valid=bool(vals))

    def step(self, notch, meas, fresh=True, handle_ok=True):
        return self._naive()

    def step_open_loop(self, notch):
        return self._naive()


class NaiveRunner(Runner):
    """Причинная база «только колесо» внутри той же связки: Runner пакета
    (сетка, выставка по GNSS окна, карта, привязка к остановкам, система
    выхода — без копий, при любом их API), но ядро — _NaiveCore. Ядро
    подменяется при КАЖДОМ присваивании self.core (в __init__, при сбросе
    связки по разрыву времени и при пересоздании ядра — WP3/WP4 потока
    robust)."""

    age_comp = False    # WP6 robust: показания не приводятся по ускорению модели

    @property
    def core(self):
        return self.__dict__.get("_naive_core")

    @core.setter
    def core(self, c):
        self.__dict__["_naive_core"] = c if isinstance(c, _NaiveCore) else _NaiveCore(c.p, self)


def replay(a, runners):
    """События bag в порядке записи (GNSS — первые 3 с, analysis/evaluate.events)."""
    outs = [[] for _ in runners]
    for tb, kind, i, th, val in E.events(a):
        for k, r in enumerate(runners):
            if kind == 0:
                outs[k] += r.on_wheel(i, th, val)
            elif kind == 1:
                outs[k] += r.on_handle(th, val)
            else:
                outs[k] += r.on_fix(th, i, *val)
    return outs


# ------------------------------------------------------------------ геодезия

# Геодезия — одна на проект: tram_state_estimator/geodesy.py (нода, оценка,
# экспорт). Прежние свои формулы экспортёра совпадали с ней лучше 1 мм
# (docs/audit/INTEGRATION.md) и заменены вызовами пакета.
from tram_state_estimator import geodesy as GD  # noqa: E402


def proj_equirect(lat, lon, alt, o):
    """Формула runner.Enu до правок (сфера R = a)."""
    return GD.Equirect(*o).fwd_arr(lat, lon, alt)


def proj_enu(lat, lon, alt, o):
    """Строгий ENU WGS84 от точки o."""
    return GD.Enu(*o).fwd_arr(lat, lon, alt)


def utm_zone(lon):
    return GD.utm_zone(lon)


def utm_en(lat, lon, zone):
    """UTM, северное полушарие (геодезия пакета, ряд Крюгера до n^6)."""
    E, N = GD.utm_fwd(np.asarray(lat, float), np.asarray(lon, float), zone, north=True)
    return np.c_[np.ravel(E), np.ravel(N)]


def unwrap100k(a):
    """Снять скачки ±100 км (граница квадрата MGRS при покадровом переносе)."""
    a = np.asarray(a, float)
    if len(a) < 2:
        return a.copy()
    k = np.round(np.diff(a) / 1e5)
    return a - np.r_[0.0, np.cumsum(k)] * 1e5


def frame_hint(r):
    """Система выхода по настройкам связки: Position.projection (поток
    «положение»: mgrs | utm | enu | equirect) и mgrs_grid; без них — формула
    equirect (код до правок). -> (кандидат detect_frame, projection, grid)."""
    pos = getattr(r, "pos", None)
    proj = str(getattr(pos, "projection", "") or "equirect").lower()
    grid = str(getattr(pos, "mgrs_grid", "") or "")
    cand = {"mgrs": "utm_abs", "utm": "utm_abs", "enu": "enu"}.get(proj, "equirect")
    return cand, proj, grid


def detect_frame(xyz, T, ready, m, hint=None):
    """Система выхода: кандидат из настроек связки (hint), проверенный по
    данным. Для каждого кандидата — эталон master fix в этой системе и
    медианная ошибка в плане; настройка берётся, если её медиана не хуже
    лучшей больше чем на max(0,5 м, 10 %) (на стоящем прогоне все кандидаты
    равны), иначе — лучший по данным (с предупреждением).
    Возвращает dict: имя, подпись, xy на экране, эталон, z, медианы."""
    lat, lon, alt = m[:, 2], m[:, 3], m[:, 4]
    o = (float(lat[0]), float(lon[0]), float(alt[0]))
    zone = utm_zone(o[1])
    U = utm_en(lat, lon, zone)
    U0 = utm_en([o[0]], [o[1]], zone)[0]
    j, ok = E.nearest(T, m[:, 1])
    ok = ok & ready[j]
    if ok.sum() < 5:
        return None
    X = xyz[:, :2]
    cands = {}
    cands["equirect"] = (X, proj_equirect(lat, lon, alt, o)[:, :2], np.zeros(2))
    cands["enu"] = (X, proj_enu(lat, lon, alt, o)[:, :2], np.zeros(2))
    cands["utm_rel"] = (X, U - U0, np.zeros(2))
    # абсолютные UTM / MGRS: снять перенос 100 км, сдвиг кратен 100 км
    Xu = np.c_[unwrap100k(np.where(ready, X[:, 0], np.nan)[ready]),
               unwrap100k(np.where(ready, X[:, 1], np.nan)[ready])]
    Xa = X.copy()
    Xa[ready] = Xu
    K = np.round(np.median(U[ok] - Xa[j[ok]], axis=0) / 1e5) * 1e5
    cands["utm_abs"] = (Xa + K, U, K)
    med = {}
    for name, (xm, xr, _) in cands.items():
        e = np.hypot(*(xm[j[ok]] - xr[ok]).T)
        med[name] = float(np.median(e))
    best = min(med, key=med.get)
    if hint is not None and hint[0] in med:
        if med[hint[0]] <= med[best] + max(0.5, 0.1 * med[best]):
            best = hint[0]
        else:
            print(f"  внимание: настройка связки {hint[1]} ({hint[0]}), по данным ближе {best}: "
                  f"медианы {', '.join(f'{k} {v:.2f} м' for k, v in med.items())}")
    xm, xr, K = cands[best]
    if best == "utm_abs":
        disp, ref = xm - U0, U - U0
        wrap = bool(np.any(np.abs(np.diff(X[ready, 0])) > 5e4)
                    or np.any(np.abs(np.diff(X[ready, 1])) > 5e4))
        mgrs = hint is not None and hint[1] == "mgrs"
        grid = hint[2] if mgrs else ""
        label = (f"MGRS от квадрата {grid}" if grid else
                 "MGRS, покадровый квадрат (разрыв 100 км снят)" if wrap else
                 "MGRS, координаты в квадрате 100 км" if mgrs else
                 "UTM абсолютные" if not K.any() else f"MGRS/UTM со сдвигом {K[0]:.0f}, {K[1]:.0f} м")
    else:
        disp, ref = xm, xr
        label = {"equirect": "equirect от первой точки master (прежняя формула)",
                 "enu": "ENU WGS84 от первой точки master",
                 "utm_rel": "UTM от первой точки master"}[best]
    # z: абсолютная высота или разность высот
    zm = xyz[:, 2]
    dz_abs = float(np.median(np.abs(zm[j[ok]] - alt[ok])))
    dz_rel = float(np.median(np.abs(zm[j[ok]] - (alt[ok] - o[2]))))
    z_abs = dz_abs < dz_rel
    ref_z = alt if z_abs else alt - o[2]
    return dict(name=best, label=label, disp=disp, ref=ref, ref_z=ref_z, z_abs=z_abs,
                zone=zone, origin=o, medians=med, U0=U0)


def selftest(bag):
    """Проверка определения системы выхода на синтетике из GNSS прогона: выход
    модели = эталон + шум 0,5 м в разных системах (MGRS с переносом квадрата,
    MGRS от 37UDB, UTM, ENU, equirect). После перевода на экран ошибка в плане
    должна остаться ~шумом, а система — определиться верно."""
    a = bagio.load(bag)
    m = a["mfix"]
    T = m[:, 1]
    lat, lon, alt = m[:, 2], m[:, 3], m[:, 4]
    o = (lat[0], lon[0], alt[0])
    zone = utm_zone(o[1])
    U = utm_en(lat, lon, zone)
    rng = np.random.default_rng(0)
    nz = rng.normal(0.0, 0.5, U.shape)
    cases = {
        "mgrs_wrap": (np.c_[np.mod(U + nz, 1e5), alt], "utm_abs"),
        "mgrs_37UDB": (np.c_[U + nz - (4e5, 61e5), alt], "utm_abs"),
        "utm_abs": (np.c_[U + nz, alt], "utm_abs"),
        "utm_rel": (np.c_[U + nz - U[0], alt - o[2]], "utm_rel"),
        "enu": (proj_enu(lat, lon, alt, o) + np.c_[nz, np.zeros(len(nz))], "enu"),
        "equirect": (proj_equirect(lat, lon, alt, o) + np.c_[nz, np.zeros(len(nz))], "equirect"),
    }
    wraps = int(np.sum(np.abs(np.diff(np.mod(U[:, 0], 1e5))) > 5e4))
    ok_all = True
    for name, (X, want) in cases.items():
        fr = detect_frame(X, T, np.ones(len(T), bool), m)
        e = np.hypot(*(fr["disp"] - fr["ref"]).T)
        ok = fr["name"] == want and float(np.median(e)) < 1.0 and float(np.max(e)) < 5.0
        ok_all &= ok
        print(f"{'OK  ' if ok else 'FAIL'} {name:<10} -> {fr['name']:<9} ({fr['label']}); "
              f"ошибка на экране: медиана {np.median(e):.2f} м, макс {np.max(e):.2f} м")
    print(f"переходов через границу квадрата 100 км в прогоне: {wraps}")
    return ok_all


# ------------------------------------------------------------------ метрики

def along_cross(ref_xy, idx, est_xy):
    """Ошибка вдоль/поперёк траектории эталона — алгоритм аудита
    (tools/audit/core_metrics.py; tools/eval_metrics.py — без изменений)."""
    try:
        from eval_metrics import along_cross as ac      # tools/eval_metrics.py
    except ImportError:
        from core_metrics import along_cross as ac      # tools/audit/core_metrics.py
    return ac(ref_xy, idx, est_xy)


def q(arr, scale):
    """Квантование в целые; NaN -> None."""
    a = np.asarray(arr, float)
    return [None if not math.isfinite(x) else int(round(x * scale)) for x in a]


def delta(lst):
    """Разности целых относительно прошлого непустого значения; None остаётся
    None (страница: acc += d, значение = acc)."""
    out, prev = [], 0
    for x in lst:
        if x is None:
            out.append(None)
        else:
            out.append(x - prev)
            prev = x
    return out


def run_variant(a, params, node, map_path, bag, kind, info):
    instrument_core()
    tm = TrackMap.load(map_path) if map_path else None
    tn = TrackMap.load(map_path) if map_path else None
    r = make_runner(params, node, tm)
    nv = make_runner(params, node, tn, cls=NaiveRunner)
    assert isinstance(nv.core, _NaiveCore) and not isinstance(r.core, _NaiveCore)
    t_run = time.perf_counter()
    mo, no = replay(a, [r, nv])
    t_run = time.perf_counter() - t_run
    T = np.array([o["stamp"] for o in mo])
    Tn = np.array([o["stamp"] for o in no])
    if len(T) != len(Tn) or not np.allclose(T, Tn):
        # сетка модели могла сброситься (разрыв времени) — база по ближайшей метке
        jn = np.clip(np.searchsorted(Tn, T), 0, len(Tn) - 1)
        no = [no[i] for i in jn]

    def O(k, src=mo, dtype=float):
        return np.array([o.get(k, np.nan) if o.get(k) is not None else np.nan for o in src], dtype=dtype)
    V, SV, S, SS = O("v"), O("sigma_v"), O("s"), O("sigma_s")
    XYZ = np.c_[O("x"), O("y"), O("z")]
    # положение есть: выставка прошла и (поток «положение») pos_valid — у края
    # квадрата MGRS при mgrs_guard_m нода /result/position не публикует
    def pos_ok(src):
        return np.array([bool(o.get("pos_ready", True)) and bool(o.get("pos_valid", True))
                         for o in src])
    ready = pos_ok(mo)
    Vn, Sn = O("v", no), O("s", no)
    XYZn = np.c_[O("x", no), O("y", no), O("z", no)]
    ready_n = pos_ok(no)

    m, g = a["mfix"], a["mvel"]
    hint = frame_hint(r)
    fr = detect_frame(XYZ, T, ready, m, hint)
    frn = detect_frame(XYZn, T, ready_n, m, hint)
    if fr is None:
        raise SystemExit(f"{bag}: нет пар положения с эталоном — выставка не прошла?")

    # ---- пары скорости (эталон — |v| GNSS master; rover справочно)
    j, ok = E.nearest(T, g[:, 1])
    vg = np.hypot(g[:, 2], g[:, 3])
    jj = j[ok]
    em, en = V[jj] - vg[ok], Vn[jj] - vg[ok]
    cov = np.abs(em) <= 2.0 * SV[jj]
    rv = a.get("rvel", np.zeros((0, 5)))
    if len(rv):
        jr, okr = E.nearest(T, rv[:, 1])
        er = V[jr[okr]] - np.hypot(rv[okr, 2], rv[okr, 3])
    else:
        er = np.zeros(0)
    # ---- пары положения (эталон — master fix в системе выхода)
    j2, ok2 = E.nearest(T, m[:, 1])
    ok2 = ok2 & ready[j2]
    idx = np.flatnonzero(ok2)
    jp = j2[ok2]
    ref, disp = fr["ref"], fr["disp"]
    al, cr, sref, path = along_cross(ref, idx, disp[jp])
    d2 = np.hypot(*(disp[jp] - ref[idx]).T)
    d3 = np.sqrt(d2 ** 2 + (XYZ[jp, 2] - fr["ref_z"][idx]) ** 2)
    if frn is not None:
        refn = frn["ref"]
        aln, _, _, _ = along_cross(refn, idx, frn["disp"][jp])
        d2n = np.hypot(*(frn["disp"][jp] - refn[idx]).T)
    else:
        aln = d2n = np.full(len(idx), np.nan)

    t0 = float(T[0])
    fin = np.isfinite(al)
    finn = np.isfinite(aln)
    last = lambda x: float(x[np.isfinite(x)][-1]) if np.isfinite(x).any() else float("nan")  # noqa: E731
    end_err = lambda a_, h_: float(abs(a_[-1])) if math.isfinite(a_[-1]) else float(h_[-1])  # noqa: E731
    summ = dict(
        pairs_v=int(ok.sum()), pairs_p=int(len(idx)),
        v_mae=float(np.mean(np.abs(em))), v_rmse=float(np.sqrt(np.mean(em ** 2))),
        v_bias=float(np.mean(em)), v_max=float(np.max(np.abs(em))),
        naive_v_mae=float(np.mean(np.abs(en))), naive_v_rmse=float(np.sqrt(np.mean(en ** 2))),
        naive_v_bias=float(np.mean(en)),
        v_mae_vs_rover=float(np.mean(np.abs(er))) if len(er) else None,
        cov2s_v=float(np.mean(cov)),
        path_m=float(path),
        p2d_mean=float(np.mean(d2)), p3d_mean=float(np.mean(d3)), p2d_max=float(np.max(d2)),
        p2d_end=float(d2[-1]), naive_p2d_mean=float(np.nanmean(d2n)), naive_p2d_end=float(d2n[-1]),
        along_end=last(al), naive_along_end=last(aln),
        along_mean_abs=float(np.nanmean(np.abs(al))) if fin.any() else None,
        # «накопленный дрейф» по ТЗ: ошибка положения в конце прогона / пройденный путь.
        # Ошибка — вдоль трассы эталона на последней паре; если оценка там ушла с
        # трассы (вдоль не определено) — ошибка в плане. Путь — дуговая координата
        # эталона sref последней пары (≈ path).
        drift_pct=100.0 * end_err(al, d2) / sref[-1] if len(sref) and sref[-1] > 100 else None,
        naive_drift_pct=100.0 * end_err(aln, d2n) / sref[-1] if len(sref) and sref[-1] > 100 else None,
        modes_pct={n_: round(100.0 * float(np.mean(O("mode", dtype=int) == i)), 2)
                   for i, n_ in enumerate(EC.MODE_NAMES)},
        frac_valid=float(np.mean([bool(o["valid"]) and not o.get("wheels_stale", False) for o in mo])),
        run_s=round(t_run, 1),
    )
    if info.get("t0") is not None:
        w0, w1 = info["t0"], info["t0"] + max(info["dur"], info.get("eval", 0.0))
        tg = g[ok, 1]
        ww = (tg >= w0) & (tg < w1)
        tf = m[idx, 1]

        def at(x, t):
            k = int(np.clip(np.searchsorted(tf, t), 0, len(x) - 1))
            return float(x[k])
        ow = [o for o in mo if w0 <= o["stamp"] < w1]
        summ["window"] = dict(
            t0_rel=w0 - t0, dur=info["dur"], eval=w1 - w0,
            v_mae=float(np.mean(np.abs(em[ww]))) if ww.any() else None,
            naive_v_mae=float(np.mean(np.abs(en[ww]))) if ww.any() else None,
            along_end=at(al, w1), naive_along_end=at(aln, w1),
            along_60s=at(al, w1 + 60.0), naive_along_60s=at(aln, w1 + 60.0),
            frac_valid=float(np.mean([bool(o["valid"]) and not o.get("wheels_stale", False) for o in ow])) if ow else None,
            sigma_v_end=float(ow[-1]["sigma_v"]) if ow else None,
            sigma_s_end=float(ow[-1]["sigma_s"]) if ow else None)

    # ---- строки 10 Гц (каждый второй шаг сетки 50 мс; входы — последнее значение)
    step = max(1, int(round(DECIM_DT / max(np.median(np.diff(T)), 1e-3))))
    R = np.arange(0, len(T), step)
    TR = T[R]
    front = _zoh(a["front"][:, 1], a["front"][:, 2], TR)
    rear = _zoh(a["rear"][:, 1], a["rear"][:, 2], TR)
    t_fr = _zoh(a["front"][:, 1], a["front"][:, 1], TR)
    t_rr = _zoh(a["rear"][:, 1], a["rear"][:, 1], TR)
    front = np.where(TR - t_fr <= 1.0, front, np.nan)     # нет сообщений > 1 с — пусто
    rear = np.where(TR - t_rr <= 1.0, rear, np.nan)
    handle = _zoh(a["cmd"][:, 1], a["cmd"][:, 2], TR)
    # эталон на строке — ближайший отсчёт в пределах 0,06 с
    jg = np.clip(np.searchsorted(g[:, 1], TR), 1, len(g) - 1)
    jg = np.where(np.abs(g[jg - 1, 1] - TR) < np.abs(g[jg, 1] - TR), jg - 1, jg)
    gv = np.where(np.abs(g[jg, 1] - TR) <= 0.06, vg[jg], np.nan)
    jf = np.clip(np.searchsorted(m[:, 1], TR), 1, len(m) - 1)
    jf = np.where(np.abs(m[jf - 1, 1] - TR) < np.abs(m[jf, 1] - TR), jf - 1, jf)
    okf = np.abs(m[jf, 1] - TR) <= 0.06
    gx = np.where(okf, ref[jf, 0], np.nan)
    gy = np.where(okf, ref[jf, 1], np.nan)
    # путь по эталону (дуговая координата траектории GNSS) — для дрейфа %
    s_fix = np.full(len(m), np.nan)
    s_fix[idx] = sref
    gs = np.where(okf, s_fix[jf], np.nan)
    # состояние тележки на строке — последнее решение по ней (показания
    # приходят ~10 Гц, шаг 20 Гц: «нет новых данных» не дольше 0,5 с не показываем)
    st_all = np.array([o.get("_dg", {}).get("state", [9, 9]) for o in mo])
    bst_all = st_all.copy()
    for b_ in range(st_all.shape[1]):
        lastc, last_t = 9, -np.inf
        for i in range(len(T)):
            c_ = st_all[i, b_]
            if c_ != 1:
                lastc, last_t = c_, T[i]
            elif T[i] - last_t <= 0.5 and lastc in (0, 2):
                bst_all[i, b_] = lastc
    bst = bst_all[R]
    flags = [int(bool(mo[i]["valid"]) and not mo[i].get("wheels_stale", False))
             | int(bool(mo[i]["slip"])) << 1 | int(bool(mo[i]["ambiguous"])) << 2
             | int(bool(mo[i].get("handle_ok", True))) << 3
             | int(bool(mo[i].get("wheels_stale", False))) << 4
             | int(bool(ready[i])) << 5 | int(bool(mo[i].get("odometry_used", False))) << 6
             for i in R]
    inj = np.zeros(len(TR), int)
    if info.get("t0") is not None:
        inj = ((TR >= info["t0"]) & (TR < info["t0"] + info["dur"])).astype(int)
    xd = np.where(ready[R], disp[R, 0], np.nan)
    yd = np.where(ready[R], disp[R, 1], np.nan)
    if frn is not None:
        xn = np.where(ready_n[R], frn["disp"][R, 0], np.nan)
        yn = np.where(ready_n[R], frn["disp"][R, 1], np.nan)
    else:
        xn = yn = np.full(len(R), np.nan)
    s_rel = S - S[0]
    sn_rel = Sn - Sn[0]
    cols = {
        "t": q(TR - t0, 100),
        "v": q(V[R], 100), "sv": q(SV[R], 1000), "s": q(s_rel[R], 10), "ss": q(SS[R], 10),
        "x": q(xd, 10), "y": q(yd, 10),
        "mode": [int(mo[i]["mode"]) for i in R], "fl": flags,
        "na": [int(mo[i]["n_accepted"]) for i in R], "nr": [int(mo[i]["n_rejected"]) for i in R],
        "b0": [int(x) for x in bst[:, 0]], "b1": [int(x) for x in bst[:, 1]],
        "kt": q(O("k_t")[R], 1000), "kb": q(O("k_b")[R], 1000),
        "mu": q(O("mu")[R], 1000), "d": q(O("d")[R], 1000),
        "fr": q(front, 10), "rr": q(rear, 10),
        "h": [None if not math.isfinite(x) else int(x) for x in handle],
        "nv": q(Vn[R], 100), "ns": q(sn_rel[R], 10), "nx": q(xn, 10), "ny": q(yn, 10),
        "gv": q(gv, 100), "gx": q(gx, 10), "gy": q(gy, 10), "gs": q(gs, 10),
        "inj": [int(x) for x in inj],
    }
    scale = {"t": 0.01, "v": 0.01, "sv": 0.001, "s": 0.1, "ss": 0.1, "x": 0.1, "y": 0.1,
             "kt": 0.001, "kb": 0.001, "mu": 0.001, "d": 0.001, "fr": 0.1, "rr": 0.1,
             "nv": 0.01, "ns": 0.1, "nx": 0.1, "ny": 0.1, "gv": 0.01, "gx": 0.1, "gy": 0.1,
             "gs": 0.1}
    # полный эталон трассы для карты (прорежен до ~2 м)
    keep, lastp = [], None
    for i in range(len(ref)):
        if lastp is None or math.hypot(*(ref[i] - ref[lastp])) >= 2.0:
            keep.append(i)
            lastp = i
    pairs = {
        "v": {"t": q(g[ok, 1] - t0, 100), "em": q(em, 10000), "en": q(en, 10000),
              "c2": [int(x) for x in cov]},
        # 1 мм: на стоянках ошибка постоянна тысячи пар подряд, и грубое округление
        # давало бы смещение среднего (страница считает метрики по этим парам)
        "p": {"t": q(m[idx, 1] - t0, 100), "al": q(al, 1000), "aln": q(aln, 1000),
              "h": q(d2, 1000), "hn": q(d2n, 1000), "sr": q(sref, 10)},
    }
    # плавные колонки — разностями (страница восстанавливает накопленной суммой)
    dcols = ("t", "s", "ss", "x", "y", "ns", "nx", "ny", "gx", "gy", "gs", "fr", "rr", "v", "nv", "gv",
             "kt", "kb", "mu", "d", "sv", "h")
    for k_ in dcols:
        cols[k_] = delta(cols[k_])
    for grp, k_ in (("v", "t"), ("p", "t"), ("p", "sr")):
        pairs[grp][k_] = delta(pairs[grp][k_])
    meta = dict(
        run=bag, variant=kind, variant_ru=info["ru"], inject=info,
        delta=dict(cols=list(dcols), v=["t"], p=["t", "sr"]),
        pair_scale=dict(v=dict(t=0.01, em=1e-4, en=1e-4, c2=1),
                        p=dict(t=0.01, al=0.001, aln=0.001, h=0.001, hn=0.001, sr=0.1)),
        t0=t0, dur=float(T[-1] - T[0]), n=len(TR), dt=float(np.median(np.diff(TR))), scale=scale,
        frame=dict(model=fr["name"], label=fr["label"], naive=frn["name"] if frn else None,
                   zone=fr["zone"], z_abs=fr["z_abs"], origin=list(fr["origin"]),
                   medians_m={k_: round(v_, 3) for k_, v_ in fr["medians"].items()},
                   runner_projection=hint[1], runner_grid=hint[2],
                   note="экран: система выхода модели, сдвинутая в первую точку GNSS master; "
                        "судья считает в MGRS"),
        flags="bit0 valid (и нет разрыва входов), bit1 slip, bit2 ambiguous, bit3 handle_ok, "
              "bit4 wheels_stale, bit5 pos_ready, bit6 odometry_used",
        bogie_state="0 принята, 1 нет новых данных, 2 отвергнута, 3 исключена (неисправна), "
                    "4 поток прерван (разомкнутый режим), 9 нет данных",
        modes=list(EC.MODE_NAMES),
        track={"x": q(ref[keep, 0], 10), "y": q(ref[keep, 1], 10)},
    )
    return dict(meta=meta, summary=summ, cols=cols, pairs=pairs)


# ------------------------------------------------------------------ запись

def git_rev():
    try:
        import subprocess
        return subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], capture_output=True,
                              text=True, timeout=10).stdout.strip() or None
    except Exception:  # noqa: BLE001
        return None


def sha1(path):
    with open(path, "rb") as fh:
        return hashlib.sha1(fh.read()).hexdigest()[:12]


def core_sha1():
    """Отпечаток estimator_core.py, с которым посчитан прогон (переводы строк
    приведены к LF: одинаков в клоне Windows и Linux). Страница сравнивает его
    с отпечатком ядра, с которым сверен JS-порт песочницы (EST_PORT в est.js)."""
    with open(EC.__file__, "rb") as fh:
        return hashlib.sha1(fh.read().replace(b"\r\n", b"\n")).hexdigest()[:12]


def rounded(x, nd=6):
    if isinstance(x, float):
        return None if not math.isfinite(x) else round(x, nd)
    if isinstance(x, dict):
        return {k: rounded(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [rounded(v, nd) for v in x]
    return x


def write_js(doc, key):
    os.makedirs(OUT_DIR, exist_ok=True)
    body = json.dumps(rounded(doc), ensure_ascii=False, separators=(",", ":"))
    js = (f"// Сгенерировано tools/export_replay.py — не править руками.\n"
          f"(window.TV_REPLAYS = window.TV_REPLAYS || {{}})[{json.dumps(key)}] = {body};\n")
    p = os.path.join(OUT_DIR, f"{key}.js")
    with open(p, "w", encoding="utf-8") as fh:
        fh.write(js)
    raw = js.encode("utf-8")
    return p, len(raw), len(gzip.compress(raw, 9))


def update_index(entries):
    p = os.path.join(OUT_DIR, "index.js")
    old = []
    if os.path.exists(p):
        txt = open(p, encoding="utf-8").read()
        mm = re.search(r"window\.TV_REPLAY_INDEX\s*=\s*(\[.*\]);", txt, re.S)
        if mm:
            old = json.loads(mm.group(1))
    keys = {e["key"] for e in entries}
    allv = [e for e in old if e["key"] not in keys] + entries
    order = {v: i for i, v in enumerate(VARIANTS)}
    allv.sort(key=lambda e: (e["run"], order.get(e["variant"], 99)))
    body = json.dumps(allv, ensure_ascii=False, indent=1)
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("// Сгенерировано tools/export_replay.py — список прогонов для режима "
                 "«Прогон данных комиссии».\n")
        fh.write(f"window.TV_REPLAY_INDEX = {body};\n")
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", default=DEFAULT_RUN, help="прогон (по умолчанию отложенный 30618_e9a34502)")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--map", default=None,
                    help="карта путей; по умолчанию config/eval/track_map.npz пакета, если есть, "
                         "иначе analysis/cache/track_map_train.npz (обе — только train); '' — без карты")
    ap.add_argument("--sheet", default=None,
                    help="лист: eval (по умолчанию: оценочный config/eval/, без него — запасной json "
                         "с пометкой УТЕЧКА, как tools/eval.py) | jury | путь к yaml/json")
    ap.add_argument("--allow-train", action="store_true", help="разрешить прогон не из holdout_scored")
    ap.add_argument("--out", default=None, help="каталог вывода (по умолчанию simulator/replays)")
    ap.add_argument("--selftest", action="store_true", help="проверить определение системы выхода и выйти")
    args = ap.parse_args()
    global OUT_DIR
    if args.out:
        OUT_DIR = os.path.abspath(args.out)
    if args.selftest:
        raise SystemExit(0 if selftest(args.run) else 1)

    split = json.load(open(os.path.join(ROOT, "tools", "split.json"), encoding="utf-8"))
    holdout = args.run in split["holdout_scored"]
    if not holdout and not args.allow_train:
        raise SystemExit(f"{args.run} не из holdout_scored (tools/split.json); --allow-train, "
                         "если это осознанно")
    params, node, sheet = resolve_sheet(args.sheet)
    if not holdout:
        sheet["leak"] = None          # обучающий прогон: утечки по определению нет, числа не отчётные
    if args.map is None:
        map_path = next((q for q in MAP_CANDIDATES if os.path.exists(q)), None)
        if map_path is None:
            raise SystemExit("нет карты только из обучающих: " + ", ".join(_rel(q) for q in MAP_CANDIDATES)
                             + " (--map '' — без карты)")
    else:
        map_path = args.map or None
    if map_path and not os.path.exists(map_path):
        raise SystemExit(f"нет карты {map_path}")
    kw, unused = runner_kwargs(node, None)
    a = bagio.load(args.run)
    entries = []
    csha = core_sha1()
    print(f"прогон {args.run}; лист {sheet['path']} — {sheet['label']} (правила {sheet['resolver']}); "
          f"карта {_rel(map_path) if map_path else 'нет'}; ядро {_rel(EC.__file__)} sha1 {csha}")
    print(f"  в Runner: {', '.join(f'{k}={v!r}' for k, v in kw.items() if k != 'track_map')}"
          + (f"; параметры ноды не для Runner: {', '.join(unused)}" if unused else ""))
    if sheet["leak"]:
        print(f"  ВНИМАНИЕ: {sheet['leak']}")
    for kind in args.variants.split(","):
        b, info = make_variant(a, args.run, kind)
        if b is None:
            print(f"  {kind}: нет подходящего окна — пропуск")
            continue
        doc = run_variant(b, params, node, map_path, args.run, kind, info)
        doc["meta"].update(
            sheet=sheet["path"], sheet_label=sheet["label"], sheet_sha1=sheet["sha1"],
            sheet_leak=sheet["leak"], sheet_rules=sheet["resolver"],
            map=_rel(map_path) if map_path else None,
            map_sha1=sha1(map_path) if map_path else None,
            core_sha1=csha, split="holdout_scored" if holdout else "train",
            git=git_rev(), made=time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime()),
            gnss_in_runner="первые 3 с по времени записи (analysis/evaluate.events)")
        key = f"{args.run}_{kind}"
        path, nraw, ngz = write_js(doc, key)
        s = doc["summary"]
        entries.append(dict(key=key, run=args.run, variant=kind, ru=info["ru"],
                            file=os.path.basename(path), kb=round(nraw / 1024), gz_kb=round(ngz / 1024),
                            sheet=sheet["path"], sheet_leak=sheet["leak"], map=doc["meta"]["map"],
                            split=doc["meta"]["split"], core_sha1=csha,
                            summary=rounded({k: s[k] for k in ("v_mae", "naive_v_mae", "v_mae_vs_rover",
                                                               "drift_pct", "naive_drift_pct",
                                                               "p2d_mean", "naive_p2d_mean", "p3d_mean",
                                                               "path_m", "cov2s_v", "pairs_v")}
                                            | ({"window": s["window"]} if "window" in s else {}))))
        w = s.get("window")
        print(f"  {kind:<11} {nraw / 1024:7.0f} КБ (gzip {ngz / 1024:4.0f}) | "
              f"MAE v {s['v_mae']:.4f} / база {s['naive_v_mae']:.4f} м/с (rover {s['v_mae_vs_rover'] or 0:.4f}) | "
              f"дрейф {s['drift_pct'] if s['drift_pct'] is not None else float('nan'):.3f} % / база "
              f"{s['naive_drift_pct'] if s['naive_drift_pct'] is not None else float('nan'):.3f} % | "
              f"ср. план {s['p2d_mean']:.2f} / {s['naive_p2d_mean']:.2f} м, 3D {s['p3d_mean']:.2f} м | "
              f"±2σ {100 * s['cov2s_v']:.1f} % | пар v {s['pairs_v']} | "
              f"система {doc['meta']['frame']['model']} ({doc['meta']['frame']['label']})"
              + (f" | окно t0+{w['t0_rel']:.0f} с: MAE {w['v_mae']:.3f}/{w['naive_v_mae']:.3f}, "
                 f"вдоль в конце окна {w['along_end']:+.1f}/{w['naive_along_end']:+.1f} м" if w else ""))
    p = update_index(entries)
    print("->", _rel(p))


if __name__ == "__main__":
    main()
