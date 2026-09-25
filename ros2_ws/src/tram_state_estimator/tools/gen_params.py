"""Генерация params.yaml и таблицы параметров MODEL.md из описания Params.

Лист параметров существует в ОДНОМ экземпляре — в классе `Params`
(estimator_core.py). Всё остальное получается из него:

    python tools/gen_params.py

  * config/params.yaml            — лист имитатора (значения Params);
  * config/tram.yaml              — лист вагона: Params + калибровка по данным
                                    (config/tram_calibration.json);
  * MODEL.md (между маркерами)    — таблицы «Лист параметров».

После правки значений или добавления параметра достаточно перезапустить
скрипт; тест test_params_mirror_yaml следит, чтобы yaml не разошёлся с кодом.
"""

import json
import os
import sys
from dataclasses import fields, replace

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
sys.path.insert(0, PKG)

from tram_state_estimator.estimator_core import Params, DEFAULT  # noqa: E402

BEGIN = "<!-- PARAMS:BEGIN -->"
END = "<!-- PARAMS:END -->"
TRAM_NODE_ONLY = [
    ("wheel_timeout_s", 1.0, "с", "нет показаний тележек дольше — разомкнутый режим"),
    ("handle_timeout_s", 0.5, "с", "нет ручки дольше — параметры не адаптируются"),
    ("init_window_s", 3.0, "с", "окно начальной выставки по GNSS от первой точки"),
    ("map_file", '"config/track_map.npz"', "—", "карта путей; пусто — без карты"),
    ("origin_lat", ".nan", "°", "начало ENU; NaN — первая точка GNSS master"),
    ("origin_lon", ".nan", "°", "начало ENU; NaN — первая точка GNSS master"),
    ("origin_alt", ".nan", "м", "начало ENU; NaN — первая точка GNSS master"),
    ("frame_id", '"map"', "—", "система координат положения"),
    ("child_frame_id", '"base_link"', "—", "система вагона"),
]

NODE_ONLY = [
    ("rate_hz", 100.0, "Гц", "частота цикла; из неё получается шаг фильтра dt"),
    ("input_timeout_s", 0.2, "с", "устаревшие входы = отказ одометрии"),
    ("frame_id", '"tram_base"', "—", "система координат в сообщениях"),
]


def fmt(v):
    """Значение в синтаксисе YAML. Числа с порядком, но без точки, PyYAML
    читает как строку, поэтому мантисса всегда с точкой."""
    if isinstance(v, str):
        return f'"{v}"'
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (tuple, list)):
        return "[" + ", ".join(fmt(x) for x in v) + "]"
    if isinstance(v, int):
        return str(v)
    s = format(float(v), ".10g")
    if "e" in s and "." not in s:
        m, e = s.split("e")
        s = f"{m}.0e{e}"
    elif "e" not in s and "." not in s:
        s += ".0"
    return s


def groups():
    order, table = [], {}
    for f in fields(Params):
        g = f.metadata["group"]
        if g not in table:
            table[g] = []
            order.append(g)
        table[g].append(f)
    return order, table


def tram_params():
    """Лист вагона: Params с подстановкой калибровки по данным."""
    path = os.path.join(PKG, "config", "tram_calibration.json")
    with open(path, encoding="utf-8") as fh:
        over = json.load(fh)["params"]
    return Params.from_dict(over)


def core_rows(p, skip_dt=True):
    """Строки yaml по разделам листа. Пустые списки не пишутся: парсер
    параметров ROS 2 их не принимает, а значение по умолчанию и так пусто."""
    order, table = groups()
    out = []
    for g in order:
        rows = [f for f in table[g] if not (skip_dt and f.name == "dt")
                and getattr(p, f.name) != ()]
        if not rows:
            continue
        out += ["", f"    # ---------- {g} ----------"]
        for f in rows:
            md = f.metadata
            out.append(f"    {f.name}: {fmt(getattr(p, f.name))}    # {md['unit']}"
                       f" · {md['src']} · {md['doc']}")
    return out


def gen_tram_yaml():
    p = tram_params()
    out = [
        "# Лист параметров ВАГОНА для ноды tram_estimator (контракт ТЗ).",
        "#",
        "# ФАЙЛ СГЕНЕРИРОВАН tools/gen_params.py: значения Params, заменённые",
        "# калибровкой по данным (config/tram_calibration.json, её делает",
        "# analysis/calib_sheet.py). Правьте калибровку или Params и перезапускайте.",
        "",
        "/tram_state_estimator:",
        "  ros__parameters:",
    ]
    for name, val, unit, doc in TRAM_NODE_ONLY:
        out.append(f"    {name}: {val if isinstance(val, str) else fmt(val)}"
                   f"    # {unit} · {doc}")
    out += core_rows(p, skip_dt=False)
    out.append("")
    path = os.path.join(PKG, "config", "tram.yaml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out))
    return path


def gen_yaml():
    order, table = groups()
    out = [
        "# Лист параметров резервного оценщика скорости и положения трамвая.",
        "#",
        "# ФАЙЛ СГЕНЕРИРОВАН из Params (estimator_core.py) скриптом",
        "# tools/gen_params.py. Правьте ЗНАЧЕНИЯ здесь или в Params и перезапускайте",
        "# скрипт. Значения по умолчанию — заглушки для имитатора; при получении ТЗ",
        "# заменяются данными вагона и линии.",
        "#",
        "# Источник: паспорт — документация вагона/датчика/линии;",
        "#           измерение — определяется опознаванием при вводе;",
        "#           настройка — параметр фильтра.",
        "",
        "/tram_state_estimator:",
        "  ros__parameters:",
    ]
    for name, val, unit, doc in NODE_ONLY:
        out.append(f"    {name}: {fmt(val) if name != 'frame_id' else val}"
                   f"    # {unit} · {doc}")
    out += core_rows(DEFAULT)
    out += [
        "",
        "/tram_simulator:",
        "  ros__parameters:",
        "    rate_hz: 100.0",
        "    mu: 0.25                # сцепление рельса в имитаторе",
        "    grade: 0.0              # уклон, рад",
        "    seed: 0",
        "",
    ]
    path = os.path.join(PKG, "config", "params.yaml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out))
    return path


def gen_table():
    order, table = groups()
    lines = []
    for g in order:
        rows = [f for f in table[g] if f.name != "dt"]
        if not rows:
            continue
        lines += [f"#### {g}", "",
                  "| Параметр | Единицы | Источник | Смысл | Заглушка |",
                  "| --- | --- | --- | --- | --- |"]
        for f in rows:
            md = f.metadata
            v = getattr(DEFAULT, f.name)
            shown = "; ".join(fmt(x) for x in v) if isinstance(v, tuple) \
                else fmt(v)
            lines.append(f"| `{f.name}` | {md['unit']} | {md['src']} | "
                         f"{md['doc']} | {shown} |")
        lines.append("")
    return "\n".join(lines)


def patch_model_md():
    path = os.path.join(REPO, "MODEL.md")
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    if BEGIN not in text or END not in text:
        return None
    head, rest = text.split(BEGIN, 1)
    _, tail = rest.split(END, 1)
    new = head + BEGIN + "\n" + gen_table() + "\n" + END + tail
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(new)
    return path


if __name__ == "__main__":
    print("yaml :", gen_yaml())
    print("tram :", gen_tram_yaml())
    print("model:", patch_model_md() or "маркеры в MODEL.md не найдены")
