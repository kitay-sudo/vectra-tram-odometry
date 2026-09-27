"""Учёт различий трамваев: параметр ноды vehicle.

Организаторы 26.09: проверка идёт только на вагоне 30618, учёт различий
трамваев засчитывается в плюс. Вагоны различаются масштабом колёс: скорость
тележек против скорости GNSS у 30618 и 30639 расходится на 0,4 % (по всем
данным), а по датам у одного вагона — до 1,2 % (docs/VEHICLES.md).

Лист вагона (config/tram.yaml) несёт:
  * vehicle             "30618" | "30639" | "auto" — какой вагон;
  * vehicle_ids         вагоны, для которых есть своя калибровка;
  * vehicle_meas_scale  масштаб колёс каждого из них (по тем же данным, что
                        лист: ЖЮРИ — все записи вагона, ОЦЕНКА — только train).

Известный вагон — масштаб показаний ядра (Params.meas_scale) берётся его.
"auto" — общий лист как есть (калибровка по обоим вагонам) плюс онлайн-оценки
по привязкам к остановкам: масштаба пути (Position, scale_adapt) и масштаба
колёс для скорости (OnlineWheelScale, параметр wheel_scale_online); обе
работают при любом значении vehicle. Незнакомое значение — «auto» с
предупреждением.

Модуль без ROS: его используют нода (tram_node.py) и оценка (tools/eval_replay.py),
поэтому числа оценки и работа ноды совпадают.
"""

import math
from dataclasses import replace

AUTO = "auto"
# пределы правдоподобного масштаба колёс: по данным 0,992…1,013
SCALE_MIN, SCALE_MAX = 0.9, 1.1


def normalize(value):
    """Значение параметра -> строка: 30618 (целое из `-p vehicle:=30618` или
    launch), " 30618 ", "AUTO" -> "30618", "auto"."""
    if value is None:
        return ""
    if isinstance(value, float) and value.is_integer():
        value = int(value)
    return str(value).strip().lower()


def table(ids, scales):
    """Таблица вагонов листа -> ({вагон: масштаб}, ошибка или None)."""
    try:
        ids = [normalize(x) for x in (ids or ())]
    except TypeError:           # скаляр вместо списка (-p vehicle_ids:=30618)
        return {}, "vehicle_ids: не список"
    try:
        scales = [float(x) for x in (scales or ())]
    except (TypeError, ValueError):
        return {}, "vehicle_meas_scale: не числа"
    if len(ids) != len(scales):
        return {}, (f"vehicle_ids ({len(ids)}) и vehicle_meas_scale ({len(scales)}) "
                    "разной длины")
    out = {}
    for k, s in zip(ids, scales):
        if not k or k == AUTO:
            continue
        if not (math.isfinite(s) and SCALE_MIN <= s <= SCALE_MAX):
            return {}, f"vehicle_meas_scale вагона {k}: {s} вне {SCALE_MIN}…{SCALE_MAX}"
        out[k] = s
    return out, None


def resolve(vehicle, ids, scales):
    """-> dict(requested, used, meas_scale или None, warning или None).
    used — ключ вагона из таблицы или "auto"."""
    req = normalize(vehicle)
    tab, err = table(ids, scales)
    info = dict(requested=req, used=AUTO, meas_scale=None, warning=None,
                known=sorted(tab))
    if req in ("", AUTO):
        if err:
            info["warning"] = f"таблица вагонов листа негодна ({err})"
        return info
    if err:
        info["warning"] = (f"вагон {req!r}: таблица вагонов листа негодна ({err}); "
                           "работаю как auto — общий лист и онлайн-масштаб пути")
        return info
    if req not in tab:
        info["warning"] = (f"вагон {req!r} неизвестен (есть: {', '.join(sorted(tab)) or 'нет'}"
                           "); работаю как auto — общий лист и онлайн-масштаб пути")
        return info
    info.update(used=req, meas_scale=tab[req])
    return info


def apply(params, vehicle, ids, scales):
    """Params с масштабом колёс выбранного вагона -> (Params, сведения).
    Для auto и незнакомого вагона Params возвращаются как есть."""
    info = resolve(vehicle, ids, scales)
    info["sheet_meas_scale"] = float(params.meas_scale)
    if info["meas_scale"] is not None:
        params = replace(params, meas_scale=info["meas_scale"])
    return params, info


def describe(info, online=None):
    """Строка для лога ноды (online — параметр wheel_scale_online)."""
    if info["used"] == AUTO:
        out = (f"вагон auto: общий лист, meas_scale {info['sheet_meas_scale']:.6f}, "
               "масштаб пути — онлайн по остановкам")
    else:
        out = (f"вагон {info['used']}: meas_scale {info['meas_scale']:.6f} "
               f"(общий лист {info['sheet_meas_scale']:.6f})")
    if online is not None:
        out += f", онлайн-масштаб колёс {'вкл.' if online else 'выкл.'}"
    return out


class OnlineWheelScale:
    """Онлайн-масштаб колёс по привязкам к остановкам -> множитель скорости.

    Масштаб колёс меняется по датам у того же вагона до 1,2 % (docs/VEHICLES.md
    §2), а дату нода не знает. Привязка к остановке (Position.on_stop) даёт на
    каждом отрезке между остановками отношение пути по карте к пути колёс:
    k = (L·s0·m + δ) / (L·s0), где L — путь ядра по отрезку, δ — сдвиг
    привязки вдоль пути, m — множитель пути Position на этом отрезке, s0 —
    множитель пути карты (при масштабе листа). Отрезки берутся не короче
    min_l и с |k − 1| < max_dev (дальше — «не та остановка»). Оценка —
    Σ L·k / Σ L, то есть весь путь по карте на весь путь колёс принятых
    отрезков (weighted; медиана k — weighted=False), когда таких отрезков не
    меньше min_n; не больше ±cap. Длинный отрезок весит больше: ошибка
    привязки — метры, на длинном отрезке она меньше в долях. Поправка
    скорости — эта оценка, если |оценка − 1| > deadband, иначе 1 (на
    «обычных» датах шум оценки дороже поправки). Всё причинно: только
    законченные отрезки. Выбор настроек — на train (docs/VEHICLES.md §6).

    Журнал привязок — Position.scale_log: (путь, δ, L, m после привязки).
    Новый журнал (сброс Runner — новый прогон) — оценка заново."""

    MIN_L = 300.0       # м
    MAX_DEV = 0.03
    MIN_N = 2
    CAP = 0.02
    DEADBAND = 0.005

    SKIP_FIRST = False  # не брать первый отрезок (от выставки по GNSS): на train хуже
    WEIGHTED = True     # Σ L·k / Σ L; False — медиана k (на train хуже)

    def __init__(self, min_l=MIN_L, max_dev=MAX_DEV, min_n=MIN_N, cap=CAP,
                 deadband=DEADBAND, skip_first=SKIP_FIRST, weighted=WEIGHTED):
        self.min_l, self.max_dev, self.min_n = float(min_l), float(max_dev), int(min_n)
        self.cap, self.deadband = float(cap), float(deadband)
        self.skip_first, self.weighted = bool(skip_first), bool(weighted)
        self._restart(None)

    def _restart(self, log):
        self._log = log
        self._n = 0             # записей журнала учтено
        self._used = 1.0        # множитель пути Position на текущем отрезке
        self.ks = []            # k принятых отрезков
        self.ls = []            # и их длины L
        self.estimate = 1.0     # оценка масштаба (без мёртвой зоны)
        self.k = 1.0            # поправка скорости

    def factor(self, log, s0):
        """Журнал привязок Position и множитель пути карты -> поправка скорости."""
        if log is not self._log:
            self._restart(log)
        if log is None or not isinstance(s0, (int, float)) or not math.isfinite(s0) \
                or s0 <= 0.0:
            return self.k
        while self._n < len(log):
            _ds, d, L, mult = log[self._n]
            first = self._n == 0
            self._n += 1
            try:
                k = (L * s0 * self._used + d) / (L * s0) if L >= self.min_l else math.nan
            except (TypeError, ZeroDivisionError):
                k = math.nan
            if (math.isfinite(k) and abs(k - 1.0) < self.max_dev
                    and not (first and self.skip_first)):
                self.ks.append(k)
                self.ls.append(float(L))
            if isinstance(mult, (int, float)) and math.isfinite(mult) and mult > 0:
                self._used = float(mult)
            self._update()
        return self.k

    def _update(self):
        if len(self.ks) < self.min_n:
            return
        if self.weighted:
            est = sum(k * L for k, L in zip(self.ks, self.ls)) / sum(self.ls)
        else:
            x = sorted(self.ks)
            n = len(x)
            est = x[n // 2] if n % 2 else 0.5 * (x[n // 2 - 1] + x[n // 2])
        self.estimate = min(max(est, 1.0 - self.cap), 1.0 + self.cap)
        self.k = self.estimate if abs(self.estimate - 1.0) > self.deadband else 1.0


def wheel_scale_hook(runner, enabled, **kw):
    """Включить онлайн-масштаб колёс у Runner (нода и оценка — одинаково):
    runner.wheel_scale переживает reset() (его __init__ атрибут не трогает),
    журнал привязок у нового прогона новый — оценка начинается заново."""
    runner.wheel_scale = OnlineWheelScale(**kw) if enabled else None
    return runner


def apply_node(params, node):
    """Для оценки: параметры ноды (dict) -> (Params, сведения). Если в листе
    нет параметров вагона (лист до 26.09), Params не меняются."""
    if "vehicle" not in node:
        return params, dict(requested="", used=AUTO, meas_scale=None, warning=None,
                            known=[], sheet_meas_scale=float(params.meas_scale))
    return apply(params, node.get("vehicle"), node.get("vehicle_ids"),
                 node.get("vehicle_meas_scale"))
