"""Генерация params.yaml и таблицы параметров MODEL.md из описания Params.

Лист параметров существует в ОДНОМ экземпляре — в классе `Params`
(estimator_core.py). Всё остальное получается из него:

    python tools/gen_params.py

  * config/params.yaml            — лист имитатора (значения Params);
  * config/tram.yaml              — лист вагона ДЛЯ ЖЮРИ: Params + калибровка
                                    по всем данным (config/tram_calibration.json);
  * config/eval/tram.yaml         — лист вагона ДЛЯ ОЦЕНКИ: Params + калибровка
                                    только по split train
                                    (config/eval/tram_calibration.json);
  * MODEL.md (между маркерами)    — таблицы «Лист параметров».

Калибровку (json) делают analysis/calib_drive.py, calib_sheet.py,
calib_tune.py и calib_sigma.py (см. docs в их заголовках). Лист ноды — всегда
производная калибровки: руками tram.yaml не правится.

После правки значений или добавления параметра достаточно перезапустить
скрипт; тесты test_params_mirror_yaml и test_sheets.py следят, чтобы листы не
разошлись с кодом и калибровкой.
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
# Параметры ноды tram_estimator вне ядра (Params): (имя, значение в YAML,
# единицы, смысл). Значение — строка YAML как есть.
TRAM_NODE_ONLY = [
    ("wheel_timeout_s", "1.0", "с", "нет показаний тележек дольше — разомкнутый режим"),
    ("handle_timeout_s", "0.5", "с", "нет ручки дольше — параметры не адаптируются"),
    ("init_window_s", "3.0", "с", "окно начальной выставки по GNSS от первой точки"),
    ("map_file", '"config/track_map.npz"', "—", "карта путей; пусто — без карты"),
    ("projection", '"mgrs"', "—",
     "система выхода /result/position: mgrs (плоские MGRS, WGS84; так считает "
     "судья по ответу организаторов 25.09) | utm (абсолютные UTM E/N) | enu "
     "(касательная плоскость от первой точки master) | equirect (прежняя); "
     "z — абсолютная высота"),
    ("mgrs_grid", '""', "—",
     "пусто — координаты внутри квадрата 100 км, где лежит точка (как Autoware); "
     "код квадрата, например \"37UDB\", — непрерывно от этого квадрата. Линия "
     "пересекает границу 37U CB | DB (E = 400 км): соглашение судьи ждём с картой"),
    ("utm_zone", "0", "—", "зона UTM; 0 — по точке выставки (здесь 37)"),
    ("origin_lat", ".nan", "°", "начало для projection enu; NaN — первая точка GNSS master"),
    ("origin_lon", ".nan", "°", "начало для projection enu; NaN — первая точка GNSS master"),
    ("origin_alt", ".nan", "м", "начало для projection enu; NaN — первая точка GNSS master"),
    ("frame_id", '"map"', "—", "система координат положения"),
    ("child_frame_id", '"base_link"', "—", "система вагона"),
]

# Листы вагона: (калибровка, лист ноды, карта, заголовок).
SHEETS = {
    "jury": ("config/tram_calibration.json", "config/tram.yaml", None, [
        "# Лист параметров ВАГОНА для ноды tram_estimator (контракт ТЗ). ЛИСТ ЖЮРИ:",
        "# калибровка по ВСЕМ данным (train + holdout), уходит в пакет.",
    ]),
    "eval": ("config/eval/tram_calibration.json", "config/eval/tram.yaml", '""', [
        "# Лист параметров ВАГОНА — ЛИСТ ОЦЕНКИ: калибровка только по split train",
        "# (tools/split.json). По нему считаются все наши числа на holdout_scored.",
        "# Карта здесь пустая: инструмент оценки подаёт карту, собранную только из",
        "# обучающих прогонов (analysis/build_map.py train), параметром map_file.",
        "# Жюри этот лист НЕ отдаётся.",
    ]),
}

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


def tram_params(which="jury"):
    """Лист вагона: Params с подстановкой калибровки по данным."""
    path = os.path.join(PKG, SHEETS[which][0])
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


def tram_yaml_text(which="jury"):
    """Текст листа вагона: параметры ноды + Params с калибровкой."""
    calib, _, map_file, head = SHEETS[which]
    p = tram_params(which)
    out = head + [
        "#",
        "# ФАЙЛ СГЕНЕРИРОВАН tools/gen_params.py: значения Params, заменённые",
        f"# калибровкой по данным ({calib}; её делают analysis/calib_*.py).",
        "# Правьте калибровку или Params и перезапускайте; руками не править.",
        "",
        "/tram_state_estimator:",
        "  ros__parameters:",
    ]
    for name, val, unit, doc in TRAM_NODE_ONLY:
        if name == "map_file" and map_file is not None:
            val = map_file
        out.append(f"    {name}: {val}    # {unit} · {doc}")
    out += core_rows(p, skip_dt=False)
    out.append("")
    return "\n".join(out)


def gen_tram_yaml(which="jury"):
    path = os.path.join(PKG, SHEETS[which][1])
    if not os.path.exists(os.path.join(PKG, SHEETS[which][0])):
        return None
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(tram_yaml_text(which))
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
    print("tram :", gen_tram_yaml("jury"))
    print("eval :", gen_tram_yaml("eval") or "нет config/eval/tram_calibration.json")
    print("model:", patch_model_md() or "маркеры в MODEL.md не найдены")
