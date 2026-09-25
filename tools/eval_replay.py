"""Прогон записи через связку Runner так же, как это делает нода (для tools/eval.py).

* Лист: yaml (`/tram_state_estimator: ros__parameters`, как у ноды) или json
  (`{"params": {...}}`, как tram_calibration.json). Поля Params — ядру, всё
  остальное — параметры ноды (map_file, init_window_s, таймауты, начало,
  после слияний — projection, mgrs_grid и т. п.). Для json-листа параметры
  ноды берутся из боевого config/tram.yaml.
* Связка — как в tram_node.py: Runner(params, track_map, origin,
  wheel_timeout, handle_timeout), затем pos.init_window. Новые аргументы
  Runner (после слияний) подставляются из параметров ноды по имени
  (`x` или `x_s`); что не подошло — пишется в meta["node_params_unused"].
* Порядок событий — по времени записи в bag (как `ros2 bag play`); GNSS —
  первые N с записи от первой точки master (как analysis/evaluate.events)
  или весь прогон (--gnss full: так выглядит bag жюри с полным GNSS).
* Исключение в связке = падение ноды: дальше у этой связки выходов нет
  (в ROS 2 исключение в колбэке валит rclpy.spin).
* База «только колесо»: тот же Runner (сетка, выставка, карта, привязка к
  остановкам), но вместо ядра — среднее свежих показаний тележек
  × meas_scale / 3,6 (без заглядывания вперёд), путь — интеграл на сетке.
  Совпадает с NaiveRunner из tools/audit/core_metrics.py.
"""

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
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))
sys.path.insert(0, str(ROOT / "tools"))

from tram_state_estimator import estimator_core as core           # noqa: E402
from tram_state_estimator import runner as runner_mod             # noqa: E402
from tram_state_estimator.track_map import TrackMap               # noqa: E402

import eval_geo as G                                              # noqa: E402
import eval_metrics as M                                          # noqa: E402

assert core.STANDSTILL == M.STANDSTILL, "estimator_core.STANDSTILL сменился"

PARAM_NAMES = {f.name for f in fields(core.Params)}
# аргументы Runner, которые нода задаёт явно (tram_node.py)
RUNNER_KW = {"wheel_timeout": "wheel_timeout_s", "handle_timeout": "handle_timeout_s"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


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


def resolve_sheet(spec):
    """spec: eval | jury | json | путь. Возвращает dict(label, path, kind,
    core, node, leak)."""
    jury_yaml = CFG / "tram.yaml"
    node_defaults = {k: v for k, v in _read_yaml_sheet(jury_yaml).items()
                     if k not in PARAM_NAMES}
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
        leak = "УТЕЧКА: боевой лист подогнан по всем данным — числа на отложенных не отчётные"
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
        node = {k: v for k, v in d.items() if k not in PARAM_NAMES}
        kind = "yaml"
    rel = path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)
    return dict(spec=spec, label=label, path=rel, kind=kind, sha=sha(path), core=corep,
                node=node, leak=leak)


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


def make_params(sheet, overrides=None):
    d = dict(sheet["core"])
    d.update(overrides or {})
    return core.Params.from_dict(d)


# ------------------------------------------------------------------ карты

def load_map(path):
    return TrackMap.load(str(path)) if path else None


# ------------------------------------------------------------------ связка

def make_runner(params, node, tmap):
    """Runner так же, как в tram_node.py. Возвращает (runner, неиспользованные
    параметры ноды)."""
    sig = inspect.signature(runner_mod.Runner.__init__)
    o = (node.get("origin_lat", float("nan")), node.get("origin_lon", float("nan")),
         node.get("origin_alt", float("nan")))
    o = tuple(float(x) for x in o)
    origin = o if all(math.isfinite(x) for x in o) else None
    kw = {}
    used = {"map_file", "origin_lat", "origin_lon", "origin_alt", "init_window_s",
            "frame_id", "child_frame_id"}
    for name in sig.parameters:
        if name in ("self", "params", "track_map", "origin"):
            continue
        key = RUNNER_KW.get(name)
        if key is None:
            key = name if name in node else (name + "_s" if name + "_s" in node else None)
        if key is not None and key in node:
            kw[name] = node[key]
            used.add(key)
    r = runner_mod.Runner(params, track_map=tmap, origin=origin, **kw)
    if "init_window_s" in node and hasattr(r, "pos") and hasattr(r.pos, "init_window"):
        r.pos.init_window = float(node["init_window_s"])
    unused = sorted(k for k in node if k not in used)
    return r, unused


class NaiveCore(core.Estimator):
    """База «только колесо» на месте ядра (см. docstring модуля)."""

    def __init__(self, p, runner):
        super().__init__(p)
        if p.meas_units not in core.LINEAR_UNITS:
            raise ValueError("база «только колесо» — только для линейных единиц (km_h, m_s)")
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


def make_naive(params, node, tmap):
    r, _ = make_runner(params, node, tmap)
    r.core = NaiveCore(params, r)
    if hasattr(r, "reset"):             # сброс связки (после WP4) не должен вернуть ядро
        orig = r.reset

        def reset(*a, **k):
            res = orig(*a, **k)
            r.core = NaiveCore(params, r)
            return res
        r.reset = reset
    return r


# ------------------------------------------------------------------ события

def events(a, gnss="3"):
    """События в порядке записи в bag: (tb, вид, индекс, th, значение).
    gnss: число секунд от первой записи master fix (как evaluate.events при 3)
    или "full"."""
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in a[key][:, :3]:
            ev.append((tb, 0, i, th, v))
    for tb, th, n in a["cmd"][:, :3]:
        ev.append((tb, 1, 0, th, n))
    if gnss == "full":
        t_end = math.inf
    else:
        t_end = a["mfix"][0, 0] + float(gnss) if len(a["mfix"]) else -1
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        for row in a[key]:
            if row[0] <= t_end:
                ev.append((row[0], 2, ant, row[1], (row[2], row[3], row[4])))
    ev.sort(key=lambda e: e[0])
    return ev


def truncate(a, seconds):
    """Первые seconds с записи (для --quick)."""
    t0 = min(float(a[k][0, 0]) for k in ("front", "rear", "cmd") if len(a[k]))
    return {k: (v[v[:, 0] <= t0 + seconds] if len(v) else v) for k, v in a.items()}


KEYS = ("stamp", "v", "x", "y", "z", "sigma_v", "sigma_s", "mode", "valid", "slip", "ambiguous",
        "wheels_stale", "pos_ready")


def _pack(o):
    return (float(o["stamp"]), float(o["v"]), float(o["x"]), float(o["y"]), float(o["z"]),
            float(o.get("sigma_v", np.nan)), float(o.get("sigma_s", np.nan)),
            int(o.get("mode", -1)), bool(o.get("valid", True)), bool(o.get("slip", False)),
            bool(o.get("ambiguous", False)), bool(o.get("wheels_stale", False)),
            bool(o.get("pos_ready", False)))


def replay(evs, runners):
    """Прогон событий через несколько связок сразу. Возвращает для каждой
    dict массивов и сведения о падении (или None)."""
    rows = [[] for _ in runners]
    crash = [None] * len(runners)
    for tb, kind, i, th, val in evs:
        for k, r in enumerate(runners):
            if crash[k] is not None:
                continue
            try:
                if kind == 0:
                    o = r.on_wheel(i, th, val)
                elif kind == 1:
                    o = r.on_handle(th, val)
                else:
                    o = r.on_fix(th, i, *val)
                rows[k].extend(_pack(x) for x in o)
            except Exception as e:           # noqa: BLE001 — падение ноды фиксируется
                crash[k] = dict(stamp=float(th), error=f"{type(e).__name__}: {e}"[:200])
    outs = []
    for k in range(len(runners)):
        R = rows[k]
        if R:
            A = np.array(R, dtype=float)
        else:
            A = np.zeros((0, len(KEYS)))
        O = dict(T=A[:, 0], V=A[:, 1], XYZ=A[:, 2:5], SV=A[:, 5], SS=A[:, 6],
                 MODE=A[:, 7].astype(int), VALID=(A[:, 8] > 0) & ~(A[:, 11] > 0),
                 SLIP=A[:, 9] > 0, AMB=A[:, 10] > 0, STALE=A[:, 11] > 0, READY=A[:, 12] > 0)
        outs.append((O, crash[k]))
    return outs


# ------------------------------------------------------------------ система выхода Runner'а

RUNNER_FRAMES = ("auto", "equirect", "enu", "utm", "mgrs")


def runner_origin(r):
    """(lat0, lon0, alt0) начала локальной системы Runner'а, если есть."""
    pos = getattr(r, "pos", None)
    for name in ("enu", "proj", "projection", "geo"):
        obj = getattr(pos, name, None)
        if obj is not None and all(hasattr(obj, k) for k in ("lat0", "lon0", "alt0")):
            return (float(obj.lat0), float(obj.lon0), float(obj.alt0))
    return None


def detect_frame(node, requested, XYZ):
    """Система координат выхода Runner'а. auto: параметр листа projection
    (после слияния WP10), иначе equirect (код до правок); затем проверка по
    величине |y|: > 1000 км — UTM, > 20 км — MGRS внутри квадрата."""
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
            note.append(f"|y| медиана {med:.0f} м — выход не локальный, считаю {by_mag}")
            fr = by_mag
        elif by_mag is None and fr in ("utm", "mgrs") and med < 2e4:
            if fr == "utm":
                note.append("utm с |y| < 20 км — считаю UTM относительно начала")
            else:
                note.append(f"mgrs с |y| медиана {med:.0f} м — подозрительно")
    return fr, grid, note


def to_geo(XYZ, frame_name, grid, origin, zone):
    """Выход Runner'а (x, y, z в его системе) -> (lat, lon, alt).
    origin — начало Runner'а (для equirect/enu/utm-относительного)."""
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
        else:                                                   # от начала, z — от высоты начала
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
    m = a["mfix"]
    m = m[np.isfinite(m[:, 2]) & np.isfinite(m[:, 3]) & np.isfinite(m[:, 4])]
    return (float(m[0, 2]), float(m[0, 3]), float(m[0, 4])) if len(m) else None


def replace_params(p, **kw):
    return replace(p, **kw)
