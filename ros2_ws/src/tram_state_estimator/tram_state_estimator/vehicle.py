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
"auto" — общий лист как есть (калибровка по обоим вагонам) плюс онлайн-оценка
масштаба пути по привязкам к остановкам (Position, scale_adapt; она работает
при любом значении vehicle). Незнакомое значение — «auto» с предупреждением.

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
    ids = [normalize(x) for x in (ids or ())]
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


def describe(info):
    """Строка для лога ноды."""
    if info["used"] == AUTO:
        return (f"вагон auto: общий лист, meas_scale {info['sheet_meas_scale']:.6f}, "
                "масштаб пути — онлайн по остановкам")
    return (f"вагон {info['used']}: meas_scale {info['meas_scale']:.6f} "
            f"(общий лист {info['sheet_meas_scale']:.6f})")


def apply_node(params, node):
    """Для оценки: параметры ноды (dict) -> (Params, сведения). Если в листе
    нет параметров вагона (лист до 26.09), Params не меняются."""
    if "vehicle" not in node:
        return params, dict(requested="", used=AUTO, meas_scale=None, warning=None,
                            known=[], sheet_meas_scale=float(params.meas_scale))
    return apply(params, node.get("vehicle"), node.get("vehicle_ids"),
                 node.get("vehicle_meas_scale"))
