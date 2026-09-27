"""Прогон записи через связку Runner так же, как это делает нода (для tools/eval.py).

* Параметры ноды: значения по умолчанию из объявлений tram_node.py
  (`P("имя", значение)` / `declare_parameter`), поверх - лист. Лист: yaml
  (`/tram_state_estimator: ros__parameters`, как у ноды) или json
  (`{"params": {...}}`, как tram_calibration.json). Поля Params - ядру, всё
  остальное - параметры ноды (map_file, init_window_s, таймауты, начало,
  projection, mgrs_grid и т. п.). Для json-листа параметры ноды берутся
  из боевого config/tram.yaml.
* Связка - как в tram_node.py: Runner(params, track_map, origin, ...).
  Аргументы Runner подставляются из параметров ноды по имени (`x` или
  `x_s`); если у Runner есть **kwargs (настройки Position) -
  то же для аргументов Position.__init__. Что не подошло (кроме параметров
  самой ноды: map_file, начало, frame_id, пульс, sheet) - пишется в
  meta["node_params_unused"] и в шапку EVAL.md.
* Порядок событий - по времени записи в bag (как `ros2 bag play`); GNSS -
  первые N с записи от первой точки master (как analysis/evaluate.events)
  или весь прогон (--gnss full: так выглядит bag жюри с полным GNSS).
  Статус NavSatFix передаётся в on_fix, если Runner его принимает. Если в
  runner.py есть StartSorter, стартовый всплеск сортируется так же, как в
  ноде: часы - время записи в bag, окно - start_sort_s.
  Пульс ноды не эмулируется: он публикует те же узлы сетки, что
  связка выдаёт при следующем сообщении (прогноз на копии тем же кодом),
  с теми же значениями - меняется только момент публикации.
* Выход: скорость публикуется всегда; положение - только при pos_valid
  (нода не публикует /result/position без якоря или у края
  квадрата MGRS) и конечных x, y, z - поле PV.
* Исключение в связке = падение ноды: дальше у этой связки выходов нет
  (в ROS 2 исключение в колбэке валит rclpy.spin).
* База «только колесо»: тот же Runner (сетка, выставка, карта, привязка к
  остановкам), но вместо ядра - среднее свежих показаний тележек
  × meas_scale / 3,6 (без заглядывания вперёд), путь - интеграл на сетке.
  Совпадает с NaiveRunner из tools/core_metrics.py.
"""

import ast
import copy
import hashlib
import inspect
import json
import math
import sys
from dataclasses import fields, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
CFG = PKG / "config"
# пакет из этого дерева - первым в пути (в образе PYTHONPATH ведёт и на /ws со
# старой сборкой: при PKG не на первом месте молча взялась бы она)
if sys.path[:1] != [str(PKG)]:
    if str(PKG) in sys.path:
        sys.path.remove(str(PKG))
    sys.path.insert(0, str(PKG))
sys.path.insert(1, str(ROOT / "tools"))

from tram_state_estimator import estimator_core as core           # noqa: E402
from tram_state_estimator import runner as runner_mod             # noqa: E402
from tram_state_estimator.track_map import TrackMap               # noqa: E402

import eval_geo as G                                              # noqa: E402
import eval_metrics as M                                          # noqa: E402

assert core.STANDSTILL == M.STANDSTILL, "estimator_core.STANDSTILL сменился"

PARAM_NAMES = {f.name for f in fields(core.Params)}
# аргументы Runner, которые нода задаёт явно (tram_node.py)
RUNNER_KW = {"wheel_timeout": "wheel_timeout_s", "handle_timeout": "handle_timeout_s",
             "init_window": "init_window_s"}
# параметры самой ноды (не Runner): карта, начало, имена систем, пульс и
# сортировка всплеска, выбор листа
NODE_ONLY = {"map_file", "origin_lat", "origin_lon", "origin_alt", "frame_id", "child_frame_id",
             "sheet", "pulse_horizon_s", "pulse_margin_s", "pulse_margin_nohandle_s",
             "pulse_period_s", "start_sort_s",
             # вагон: меняет Params (meas_scale) до Runner - make_params
             "vehicle", "vehicle_ids", "vehicle_meas_scale",
             # онлайн-масштаб колёс: задаётся Runner после __init__ - make_runner
             "wheel_scale_online"}
# --set vehicle=match - только для оценки: вагон по имени прогона (30618_…)
VEHICLE_MATCH = "match"
START_SORT_DEFAULT = 0.1     # с: start_sort_s ноды, если его нет в листе
NODE_PY = PKG / "tram_state_estimator" / "tram_node.py"


def sha(path):
    """Отпечаток файла; для .npz - по содержимому массивов (в zip-архиве
    numpy есть время записи, и одинаковая карта давала бы разный хэш)."""
    path = Path(path)
    if path.suffix == ".npz":
        h = hashlib.sha256()
        with np.load(path) as z:
            for k in sorted(z.files):
                v = np.ascontiguousarray(z[k])
                h.update(k.encode())
                h.update(str(v.dtype).encode() + str(v.shape).encode())
                h.update(v.tobytes())
        return h.hexdigest()[:16]
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


# ------------------------------------------------------------------ листы

def _read_yaml_sheet(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        d = yaml.safe_load(fh)
    for v in d.values():
        if isinstance(v, dict) and "ros__parameters" in v:
            return v["ros__parameters"]
    raise ValueError(f"{path}: нет ros__parameters")


def eval_sheet_candidates():
    e = CFG / "eval"
    return [e / "tram_eval.yaml", e / "tram.yaml", e / "tram_calibration.json"]


def _literal(n):
    """Значение по умолчанию из объявления параметра (литерал или float("nan"))."""
    try:
        return ast.literal_eval(n)
    except ValueError:
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "float"
                and len(n.args) == 1 and isinstance(n.args[0], ast.Constant)):
            return float(n.args[0].value)
        raise


def node_declared(path=NODE_PY):
    """Параметры, которые нода объявляет сама, с их значениями по умолчанию:
    вызовы P("имя", значение) и *.declare_parameter("имя", значение) в
    tram_node.py (параметры ядра объявляются отдельно и сюда не входят)."""
    try:
        tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    except (OSError, SyntaxError):
        return {}
    out = {}
    for n in ast.walk(tree):
        if not (isinstance(n, ast.Call) and len(n.args) >= 2):
            continue
        f = n.func
        name = f.id if isinstance(f, ast.Name) else f.attr if isinstance(f, ast.Attribute) else ""
        if name not in ("P", "declare_parameter"):
            continue
        a0 = n.args[0]
        if not (isinstance(a0, ast.Constant) and isinstance(a0.value, str)):
            continue
        try:
            out.setdefault(a0.value, _literal(n.args[1]))
        except (ValueError, TypeError, SyntaxError):
            continue
    return {k: v for k, v in out.items() if k not in PARAM_NAMES}


def resolve_sheet(spec):
    """spec: eval | jury | json | путь. Возвращает dict(label, path, kind,
    core, node, leak). node - объявления tram_node.py, поверх - лист."""
    jury_yaml = CFG / "tram.yaml"
    declared = node_declared()
    node_defaults = dict(declared)
    node_defaults.update({k: v for k, v in _read_yaml_sheet(jury_yaml).items()
                          if k not in PARAM_NAMES})
    leak = None
    if spec == "eval":
        path = next((p for p in eval_sheet_candidates() if p.exists()), None)
        if path is None:
            path = CFG / "tram_calibration.json"
            label = "json (запасной: оценочного листа config/eval/ нет)"
            leak = ("УТЕЧКА: таблица привода и масштаб подогнаны по всем 122 bag, "
                    "включая отложенные; q_v, delay, tau подбирались на отложенных")
        else:
            label = f"оценочный ({path.relative_to(ROOT).as_posix()}, только train)"
    elif spec == "jury":
        path, label = jury_yaml, "боевой config/tram.yaml (все данные)"
        leak = "УТЕЧКА: боевой лист подогнан по всем данным - числа на отложенных не отчётные"
    elif spec == "json":
        path, label = CFG / "tram_calibration.json", "json config/tram_calibration.json"
        leak = ("УТЕЧКА: таблица привода и масштаб подогнаны по всем 122 bag, "
                "включая отложенные; q_v, delay, tau подбирались на отложенных")
    else:
        path = Path(spec)
        if not path.is_absolute():
            path = (ROOT / spec) if (ROOT / spec).exists() else path.resolve()
        label = path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)
    if path.suffix == ".json":
        d = json.loads(path.read_text(encoding="utf-8"))
        d = d.get("params", d)
        corep = {k: v for k, v in d.items() if k in PARAM_NAMES}
        node = dict(node_defaults)
        node.update({k: v for k, v in d.items() if k not in PARAM_NAMES and not k.startswith("_")})
        kind = "json"
    else:
        d = _read_yaml_sheet(path)
        corep = {k: v for k, v in d.items() if k in PARAM_NAMES}
        node = dict(declared)
        node.update({k: v for k, v in d.items() if k not in PARAM_NAMES})
        kind = "yaml"
    rel = path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)
    return dict(spec=spec, label=label, path=rel, kind=kind, sha=sha(path), core=corep,
                node=node, leak=leak, node_declared=sorted(declared))


def split_overrides(ov):
    """--set: поля Params - ядру, остальное - параметрам ноды."""
    return ({k: v for k, v in ov.items() if k in PARAM_NAMES},
            {k: v for k, v in ov.items() if k not in PARAM_NAMES})


def parse_overrides(text):
    """«c_creep=0,c_creep_drag=0» -> dict."""
    out = {}
    for part in (text or "").split(","):
        if not part.strip():
            continue
        k, v = part.split("=", 1)
        v = v.strip()
        try:
            v = json.loads(v)
        except ValueError:
            pass
        out[k.strip()] = v
    return out


def make_params(sheet, overrides=None, bag=None):
    """Params листа (+ --set) с масштабом колёс вагона, как в tram_node.py
    (tram_state_estimator/vehicle.py). vehicle=match - вагон прогона bag."""
    d = dict(sheet["core"])
    d.update({k: v for k, v in (overrides or {}).items() if k in PARAM_NAMES})
    p = core.Params.from_dict(d)
    return apply_vehicle(p, sheet["node"], bag)[0]


def apply_vehicle(p, node, bag=None):
    """-> (Params, сведения о вагоне). Старый код пакета без vehicle.py - как есть."""
    try:
        from tram_state_estimator import vehicle as V
    except ImportError:
        return p, None
    node = dict(node or {})
    if str(node.get("vehicle", "")).strip().lower() == VEHICLE_MATCH:
        node["vehicle"] = str(bag).split("_")[0] if bag else "auto"
    return V.apply_node(p, node)


# ------------------------------------------------------------------ карты

def load_map(path):
    return TrackMap.load(str(path)) if path else None


# ------------------------------------------------------------------ связка

def _named(fn):
    return [q.name for q in inspect.signature(fn).parameters.values()
            if q.kind not in (q.VAR_KEYWORD, q.VAR_POSITIONAL)
            and q.name not in ("self", "params", "track_map", "origin")]


def runner_arg_names(cls=None):
    """Имена аргументов Runner, которые берутся из параметров ноды. При
    **kwargs у Runner (настройки Position) - плюс аргументы
    Position.__init__."""
    cls = cls or runner_mod.Runner
    names = _named(cls.__init__)
    sig = inspect.signature(cls.__init__).parameters
    if any(q.kind == q.VAR_KEYWORD for q in sig.values()) and hasattr(runner_mod, "Position"):
        names += [n for n in _named(runner_mod.Position.__init__) if n not in names]
    return names


def make_runner(params, node, tmap, cls=None):
    """Runner так же, как в tram_node.py. Возвращает (runner, неиспользованные
    параметры ноды)."""
    cls = cls or runner_mod.Runner
    o = (node.get("origin_lat", float("nan")), node.get("origin_lon", float("nan")),
         node.get("origin_alt", float("nan")))
    o = tuple(float(x) for x in o)
    origin = o if all(math.isfinite(x) for x in o) else None
    kw = {}
    used = set(NODE_ONLY) | {"init_window_s"}
    for name in runner_arg_names(cls):
        key = RUNNER_KW.get(name)
        if key is None or key not in node:
            key = name if name in node else (name + "_s" if name + "_s" in node else None)
        if key is not None and key in node:
            kw[name] = node[key]
            used.add(key)
    r = cls(params, track_map=tmap, origin=origin, **kw)
    # старая версия пакета: окно выставки задаётся после __init__
    if ("init_window" not in kw and "init_window_s" in node and hasattr(r, "pos")
            and hasattr(r.pos, "init_window")):
        r.pos.init_window = float(node["init_window_s"])
    if cls is runner_mod.Runner and node.get("wheel_scale_online"):
        try:
            from tram_state_estimator import vehicle as V
        except ImportError:         # старый код пакета без vehicle.py
            V = None
        if V is not None and hasattr(V, "wheel_scale_hook"):
            V.wheel_scale_hook(r, True)
    unused = sorted(k for k in node if k not in used)
    return r, unused


class NaiveCore(core.Estimator):
    """База «только колесо» на месте ядра (см. docstring модуля)."""

    def __init__(self, p, runner):
        super().__init__(p)
        if p.meas_units not in core.LINEAR_UNITS:
            raise ValueError("база «только колесо» - только для линейных единиц (km_h, m_s)")
        self.r = runner
        self.kv = p.meas_scale * core.LINEAR_UNITS[p.meas_units]
        self.v = 0.0

    def _naive(self):
        r, p = self.r, self.p
        t = r.t
        vals = [float(r.meas[i]) for i in range(self.nw)
                if t - r.t_wheel[i] <= r.wheel_timeout and math.isfinite(float(r.meas[i]))]
        if vals:
            self.v = max(0.0, float(np.mean(vals)) * self.kv)
        self.x[core.IS] += self.v * p.dt
        self.x[core.IV] = self.v
        stand = bool(vals) and self.v < p.v_standstill
        return dict(v=self.v, s=float(self.x[core.IS]), d=0.0, k_t=1.0, k_b=1.0,
                    mu=float(self.mu), sigma_v=float("nan"), sigma_s=float("nan"),
                    mode=core.STANDSTILL if stand else core.COAST,
                    healthy=np.ones(self.nw, dtype=bool), slip=False, ambiguous=False,
                    n_accepted=len(vals), n_rejected=0, odometry_used=bool(vals),
                    valid=bool(vals))

    def step(self, notch, meas, fresh=True, handle_ok=True):
        return self._naive()

    def step_open_loop(self, notch):
        return self._naive()


class NaiveRunner(runner_mod.Runner):
    """Runner с базой вместо ядра. Сброс связки (пересоздаёт ядро
    через __init__) снова ставит базу. Подкласс, а не подмена метода у
    экземпляра: копия связки (deepcopy для инъекций) остаётся независимой."""

    def reset(self, *a, **k):
        res = super().reset(*a, **k)
        self.core = NaiveCore(self.p, self)
        return res


def make_naive(params, node, tmap):
    r, _ = make_runner(params, node, tmap, cls=NaiveRunner)
    r.core = NaiveCore(params, r)
    return r


# ------------------------------------------------------------------ события

def _rows(x, ncol):
    """Массив прогона как (n, ≥ncol); пустой - (0, ncol) (bagio пишет пустые (0, 3))."""
    x = np.asarray(x, float)
    if x.ndim != 2 or x.shape[1] < ncol:
        return np.zeros((0, ncol))
    return x


def gnss_spec(gnss):
    """Имя сценария GNSS: число секунд -> "first<N>" для отчёта; full и
    сценарии inject.GNSS_SCENARIOS - как есть."""
    g = str(gnss)
    try:
        return f"first{float(g):g}"
    except ValueError:
        return g


def events(a, gnss="3", seed=0):
    """События в порядке записи в bag: (tb, вид, индекс, th, значение).
    gnss: число секунд от первой записи master fix (как evaluate.events при 3),
    "full" или сценарий доступности inject.GNSS_SCENARIOS (sparse, bursts,
    nostart, glitchy, none; first3 - то же, что 3; midstart - обрезка записи
    inject.cut_start, затем 3). seed - зерно сценария (inject.seed_for).
    Значение GNSS - (lat, lon, alt, status)."""
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in _rows(a[key], 3)[:, :3]:
            ev.append((tb, 0, i, th, v))
    for tb, th, n in _rows(a["cmd"], 3)[:, :3]:
        ev.append((tb, 1, 0, th, n))
    g = str(gnss)
    if g in ("first3", "midstart"):
        g = "3"
    if g not in ("full",) and not _is_number(g):
        import inject as I
        for tb, ant, th, lat, lon, alt, st in I.gnss_scenario(a, g, seed)[0]:
            ev.append((tb, 2, ant, th, (lat, lon, alt, st)))
        ev.sort(key=lambda e: e[0])
        return ev
    mfix = _rows(a["mfix"], 5)
    if g == "full":
        t_end = math.inf
    else:
        t_end = mfix[0, 0] + float(g) if len(mfix) else -1
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        x = _rows(a[key], 5)
        for row in x:
            if row[0] <= t_end:
                st = int(row[5]) if len(row) > 5 and math.isfinite(row[5]) else 0
                ev.append((row[0], 2, ant, row[1], (row[2], row[3], row[4], st)))
    ev.sort(key=lambda e: e[0])
    return ev


def _is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def truncate(a, seconds):
    """Первые seconds с записи (для --quick)."""
    t0 = min(float(a[k][0, 0]) for k in ("front", "rear", "cmd") if len(a[k]))
    return {k: (v[v[:, 0] <= t0 + seconds] if len(v) else v) for k, v in a.items()}


def cut_stamp(a, t_end):
    """Только сообщения с header.stamp <= t_end (все топики)."""
    return {k: (v[v[:, 1] <= t_end] if len(v) else v) for k, v in a.items()}


KEYS = ("stamp", "v", "x", "y", "z", "sigma_v", "sigma_s", "mode", "valid", "slip", "ambiguous",
        "wheels_stale", "pos_ready", "pos_valid")
SORT_TIMER_S = 0.01     # с: период таймера ноды, который отпускает буфер StartSorter


def _num(x, default=np.nan):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _pack(o):
    return (_num(o["stamp"]), _num(o["v"]), _num(o["x"]), _num(o["y"]), _num(o["z"]),
            _num(o.get("sigma_v", np.nan)), _num(o.get("sigma_s", np.nan)),
            int(o.get("mode", -1)), bool(o.get("valid", True)), bool(o.get("slip", False)),
            bool(o.get("ambiguous", False)), bool(o.get("wheels_stale", False)),
            bool(o.get("pos_ready", False)), bool(o.get("pos_valid", True)))


class Glue:
    """Как нода подаёт сообщения в связку: статус NavSatFix - если on_fix его
    принимает; сортировка стартового всплеска - если в runner.py есть
    StartSorter (часы - время записи в bag)."""

    def __init__(self, r, node=None):
        node = node or {}
        self.r = r
        self.fix_status = "status" in inspect.signature(r.on_fix).parameters
        S = getattr(runner_mod, "StartSorter", None)
        self.sort_s = float(node.get("start_sort_s", START_SORT_DEFAULT)) if S else None
        self.sorter = S(self.sort_s) if S else None

    def _dispatch(self, m):
        kind, i, th, val = m
        r = self.r
        if kind == 0:
            return r.on_wheel(i, th, val)
        if kind == 1:
            return r.on_handle(th, val)
        if self.fix_status:
            return r.on_fix(th, i, *val)
        return r.on_fix(th, i, *val[:3])

    def feed(self, tb, kind, i, th, val):
        m = (kind, i, th, val)
        if self.sorter is None:
            return self._dispatch(m)
        out = []
        for x in self.sorter.poll(tb - SORT_TIMER_S):      # таймер сработал до сообщения
            out += self._dispatch(x)
        for x in self.sorter.push(tb, th, m):
            out += self._dispatch(x)
        return out

    def flush(self):
        out = []
        if self.sorter is not None:
            for x in self.sorter.poll(math.inf):
                out += self._dispatch(x)
        return out


def arrays(R):
    """Строки _pack -> dict массивов выхода. PV - положение опубликовано
    (pos_valid и конечные x, y, z)."""
    A = np.array(R, dtype=float) if R else np.zeros((0, len(KEYS)))
    fin = np.isfinite(A[:, 2:5]).all(axis=1)
    return dict(T=A[:, 0], V=A[:, 1], XYZ=A[:, 2:5], SV=A[:, 5], SS=A[:, 6],
                MODE=A[:, 7].astype(int), VALID=(A[:, 8] > 0) & ~(A[:, 11] > 0),
                SLIP=A[:, 9] > 0, AMB=A[:, 10] > 0, STALE=A[:, 11] > 0, READY=A[:, 12] > 0,
                POSV=A[:, 13] > 0, PV=(A[:, 13] > 0) & fin)


class Replay:
    """Прогон событий через несколько связок сразу; fork() - независимая
    копия всего состояния (связки, выходы) для продолжения другим потоком
    событий (инъекции с общего чистого начала)."""

    def __init__(self, runners, node=None):
        self.glues = [Glue(r, node) for r in runners]
        self.rows = [[] for _ in runners]
        self.crash = [None] * len(runners)

    def feed(self, evs):
        for ev in evs:
            for k, gl in enumerate(self.glues):
                if self.crash[k] is not None:
                    continue
                try:
                    self.rows[k].extend(_pack(x) for x in gl.feed(*ev))
                except Exception as e:           # noqa: BLE001 - падение ноды фиксируется
                    self.crash[k] = dict(stamp=float(ev[3]), error=f"{type(e).__name__}: {e}"[:200])
        return self

    def fork(self):
        return copy.deepcopy(self)

    def finish(self):
        for k, gl in enumerate(self.glues):
            if self.crash[k] is None:
                try:
                    self.rows[k].extend(_pack(x) for x in gl.flush())
                except Exception as e:           # noqa: BLE001
                    self.crash[k] = dict(stamp=math.inf, error=f"{type(e).__name__}: {e}"[:200])
        return [(arrays(self.rows[k]), self.crash[k]) for k in range(len(self.glues))]


def replay(evs, runners, node=None):
    """Прогон событий через несколько связок сразу. Возвращает для каждой
    dict массивов и сведения о падении (или None)."""
    return Replay(runners, node).feed(evs).finish()


# ------------------------------------------------------------------ система выхода Runner'а

RUNNER_FRAMES = ("auto", "equirect", "enu", "utm", "mgrs")


def runner_origin(r):
    """(lat0, lon0, alt0) начала локальной системы Runner'а, если есть."""
    pos = getattr(r, "pos", None)
    for name in ("frame", "enu", "proj", "projection", "geo"):
        obj = getattr(pos, name, None)
        if obj is not None and all(hasattr(obj, k) for k in ("lat0", "lon0", "alt0")):
            return (float(obj.lat0), float(obj.lon0), float(obj.alt0))
    return None


def detect_frame(node, requested, XYZ):
    """Система координат выхода Runner'а. auto: параметр листа projection
    (если он есть), иначе equirect (старая версия пакета); затем проверка по
    величине |y|: > 1000 км - UTM, > 20 км - MGRS внутри квадрата."""
    note = []
    grid = str(node.get("mgrs_grid", "") or "")
    if requested != "auto":
        return requested, grid, note
    fr = str(node.get("projection", "") or "").lower() or "equirect"
    if fr not in RUNNER_FRAMES:
        note.append(f"неизвестная projection={fr!r} в листе, беру по величине")
        fr = "equirect"
    y = np.abs(XYZ[:, 1][np.isfinite(XYZ[:, 1])]) if len(XYZ) else np.zeros(0)
    if len(y):
        med = float(np.median(y))
        by_mag = "utm" if med > 1e6 else "mgrs" if med > 2e4 else None
        if by_mag and fr in ("equirect", "enu"):
            note.append(f"|y| медиана {med:.0f} м - выход не локальный, считаю {by_mag}")
            fr = by_mag
        elif by_mag is None and fr in ("utm", "mgrs") and med < 2e4:
            if fr == "utm":
                note.append("utm с |y| < 20 км - считаю UTM относительно начала")
            else:
                note.append(f"mgrs с |y| медиана {med:.0f} м - подозрительно")
    return fr, grid, note


def to_geo(XYZ, frame_name, grid, origin, zone):
    """Выход Runner'а (x, y, z в его системе) -> (lat, lon, alt).
    origin - начало Runner'а (для equirect/enu/utm-относительного)."""
    x, y, z = XYZ[:, 0], XYZ[:, 1], XYZ[:, 2]
    if frame_name == "equirect":
        return G.equirect_inv(x, y, z, origin)
    if frame_name == "enu":
        return G.enu_inv(x, y, z, origin)
    E0, N0 = (float(v) for v in G.utm_fwd(origin[0], origin[1], zone))
    if frame_name == "utm":
        fin = np.isfinite(x)
        if fin.any() and np.median(np.abs(x[fin])) > 1e5:       # абсолютные E/N
            E, N, alt = x, y, z
        else:                                                   # от начала, z - от высоты начала
            E, N, alt = x + E0, y + N0, z + origin[2]
    elif frame_name == "mgrs":
        if grid:
            gz, gE, gN = G.grid_origin(grid)
            if gz != zone:
                raise ValueError(f"mgrs_grid {grid}: зона {gz} ≠ зоне начала {zone}")
            E, N = x + gE, y + gN
        else:
            E, N = G.unwrap(x, E0), G.unwrap(y, N0)
        alt = z
    else:
        raise ValueError(frame_name)
    lat, lon = G.utm_inv(E, N, zone)
    return lat, lon, np.asarray(alt, float)


def first_master(a):
    m = _rows(a["mfix"], 5)
    m = m[np.isfinite(m[:, 2]) & np.isfinite(m[:, 3]) & np.isfinite(m[:, 4])]
    return (float(m[0, 2]), float(m[0, 3]), float(m[0, 4])) if len(m) else None


def replace_params(p, **kw):
    return replace(p, **kw)
