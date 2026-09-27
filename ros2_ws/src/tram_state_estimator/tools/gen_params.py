"""Генерация params.yaml и таблицы параметров MODEL.md из описания Params.

Лист параметров существует в ОДНОМ экземпляре - в классе `Params`
(estimator_core.py). Всё остальное получается из него:

    python tools/gen_params.py

  * config/params.yaml            - лист имитатора (значения Params);
  * config/tram.yaml              - лист вагона ДЛЯ ЖЮРИ: Params + калибровка
                                    по всем данным (config/tram_calibration.json);
  * config/eval/tram.yaml         - лист вагона ДЛЯ ОЦЕНКИ: Params + калибровка
                                    только по split train
                                    (config/eval/tram_calibration.json);
  * MODEL.md (между маркерами)    - таблицы «Лист параметров».

Калибровку (json) делают analysis/calib_drive.py, calib_sheet.py,
calib_tune.py и calib_sigma.py (см. docs в их заголовках). Лист ноды - всегда
производная калибровки: руками tram.yaml не правится.

После правки значений или добавления параметра достаточно перезапустить
скрипт; тесты test_params_mirror_yaml и test_sheets.py следят, чтобы листы не
разошлись с кодом и калибровкой.
"""

import json
import os
import sys
import textwrap
from dataclasses import fields

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
sys.path.insert(0, PKG)

from tram_state_estimator.estimator_core import Params, DEFAULT  # noqa: E402

BEGIN = "<!-- PARAMS:BEGIN -->"
END = "<!-- PARAMS:END -->"
# Параметры ноды tram_estimator вне ядра (Params): (имя, значение в YAML,
# единицы, смысл). Значение - строка YAML как есть.
TRAM_NODE_ONLY = [
    ("wheel_timeout_s", "1.0", "с", "нет показаний тележек дольше - разомкнутый режим"),
    ("handle_timeout_s", "0.5", "с", "нет ручки дольше - параметры не адаптируются"),
    ("init_window_s", "3.0", "с", "окно начальной выставки по GNSS от первой точки"),
    ("map_file", '"config/track_map.npz"', "-", "карта путей; пусто - без карты"),
    ("projection", '"mgrs"', "-",
     "система выхода /result/position: mgrs (плоские MGRS, WGS84; так считает "
     "судья по ответу организаторов 25.09) | utm (абсолютные UTM E/N) | enu "
     "(касательная плоскость от первой точки master) | equirect (прежняя); "
     "z - высота (NavSatFix) точки выхода"),
    ("mgrs_grid", '"37UCB"', "-",
     "код квадрата - координаты непрерывно от его юго-западного угла; \"37UCB\" - "
     "как в карте организаторов pathgraph (x = E − 300 000, y = N − 6 100 000, "
     "через границу E = 400 км без скачка, x 99 156…103 347). Пусто - каждая "
     "точка в своём 100-км квадрате (Autoware; на границе скачок на 100 км)"),
    ("utm_zone", "0", "-", "зона UTM; 0 - по точке выставки (здесь 37)"),
    ("mgrs_guard_m", "0.0", "м",
     "только при mgrs_grid \"\": ближе к краю 100-км квадрата положение не "
     "публикуется; 0 - выкл. При фиксированном квадрате (по умолчанию) не "
     "действует: положение публикуется на каждом шаге после выставки"),
    ("output_point", '"base_link"', "-",
     "точка вагона на выходе: base_link - ось поворота передней тележки на "
     "уровне рельса (эталон судьи и pathgraph, tf организаторов 25.09) | master "
     "- антенна master (как до 26.09; вдоль пути на 9,87 м позади и на 3 м выше)"),
    ("antenna_master_x", "-9.873", "м", "антенна master в base_link, x (вперёд); y = 0"),
    ("antenna_rover_x", "2.563", "м", "антенна rover в base_link, x (вперёд); y = 0"),
    ("antenna_z", "3.0", "м", "высота обеих антенн над base_link (уровень рельса)"),
    ("scale_adapt", "true", "-", "онлайн-подстройка масштаба пути по привязкам к остановкам (±1 %)"),
    ("vehicle", None, "-",
     "вагон (docs/VEHICLES.md): 30618 (по умолчанию: организаторы проверяют только "
     "30618) | 30639 - meas_scale этого вагона из vehicle_meas_scale; auto - общий "
     "лист (оба вагона); онлайн-масштабы по остановкам (scale_adapt, "
     "wheel_scale_online) работают при любом значении; незнакомое значение - "
     "auto с предупреждением в логе"),
    ("vehicle_ids", None, "-", "вагоны со своей калибровкой, в порядке vehicle_meas_scale"),
    ("vehicle_meas_scale", None, "-",
     "масштаб колёс каждого вагона (GNSS / тележки на установившемся ходу, записи "
     "вагона из тех же данных, что лист); заменяет meas_scale, если vehicle - из "
     "vehicle_ids"),
    ("wheel_scale_online", "true", "-",
     "онлайн-масштаб колёс по привязкам к остановкам: скорость × «путь по "
     "карте / путь колёс» на отрезках между остановками (не короче 300 м, не "
     "меньше двух), если он отходит от 1 больше чем на 0,5 % (не больше ±2 %); "
     "для смены масштаба колёс по датам (docs/VEHICLES.md §6)"),
    ("nomap_mode", '"hold"', "-",
     "без карты (map_file пуст): hold - стоять в якоре выставки (выбрано на "
     "train), line - по прямой вдоль курса выставки"),
    ("keep_offset_xy", "true", "-", "сдвиг «GNSS окна − карта» по горизонтали, если его разрешает keep_offset_max_status"),
    ("keep_offset_z", "true", "-", "то же по высоте (не больше 30 м)"),
    ("keep_offset_max_status", "-1", "-",
     "сдвиг сохраняется, если медиана NavSatFix.status окна не больше этого: "
     "−1 - никогда (на train выигрыша нет), 1 - только без RTK, 2 - всегда"),
    ("terminal_hold", '"terminals"', "-",
     "тупик карты: terminals - держать курсор только у известных конечных "
     "(≥3 обучающих прогона), off - ехать прямо, any - держать везде"),
    # коррекция по GNSS в середине маршрута (организаторы 26.09 18:05)
    ("gnss_correction", "true", "-",
     "точки GNSS после окна выставки поправляют положение вдоль пути и ветку "
     "карты (с отбраковкой выбросов); false - GNSS только для выставки, как до 26.09"),
    ("gnss_sigma_rtk_m", "0.5", "м", "σ точки GNSS при NavSatFix.status 2 (RTK)"),
    ("gnss_sigma_sbas_m", "1.5", "м", "σ точки при status 1"),
    ("gnss_sigma_fix_m", "5.0", "м", "σ точки при status 0 (без поправок: смещение до ~16 м)"),
    ("gnss_gate", "3.0", "σ", "ворота невязки; дальше - только после подтверждения"),
    ("gnss_jump_m", "3.0", "м", "поправка больше - только после подтверждения несколькими эпохами"),
    ("gnss_confirm_n", "3", "-", "эпох RTK подряд, согласных между собой, для большой поправки (без RTK больших поправок нет)"),
    ("gnss_min_interval_s", "1.0", "с", "поправки по RTK не чаще (без RTK - раз в 20 с)"),
    ("gnss_max_skew_s", "0.3", "с",
     "метка GNSS не дальше этого от метки последнего входа на момент прихода (иначе "
     "часы GNSS сбиты: в данных - участки по минутам со сдвигом ±1 с)"),
    ("gnss_prior_rel", "0.003", "-",
     "априори поправки: рост σ положения на метр пути (типичный дрейф, не запас ss_rel)"),
    ("gnss_scale_adapt", "false", "-", "подстройка масштаба пути по отрезкам между поправками RTK"),
    ("gnss_stop_skip_m", "0.0", "м", "после поправки GNSS столько м пути без привязки к остановке"),
    ("gnss_persist_s", "10.0", "с",
     "невязка RTK вне ворот (или поперёк пути, пара против курса) держится столько - оценка "
     "сбилась, ставим по GNSS; короче - скачок GNSS, не верим"),
    ("origin_lat", ".nan", "°", "начало для projection enu; NaN - первая точка GNSS master"),
    ("origin_lon", ".nan", "°", "начало для projection enu; NaN - первая точка GNSS master"),
    ("origin_alt", ".nan", "м", "начало для projection enu; NaN - первая точка GNSS master"),
    ("frame_id", '"map"', "-", "имя системы положения: плоские MGRS от угла 37UCB (см. projection, mgrs_grid)"),
    ("child_frame_id", '"base_link"', "-", "система вагона: ось поворота передней тележки, уровень рельса"),
    ("sheet", '"auto"', "-",
     "auto - лист пакета config/tram.yaml подкладывается под параметры запуска "
     "(ros2 run без --params-file работает по нему); путь к листу; none - "
     "заглушки Params (имитатор)"),
    ("pulse_horizon_s", "0.2", "с",
     "пульс: входы молчат - прогноз на копии связки не дальше этого от "
     "последней метки; 0 - без пульса. Короткий: узлы прогноза вернувшиеся после "
     "паузы входы повторно не публикуют, и их выход ждёт до горизонта"),
    ("pulse_margin_s", "0.1", "с", "пульс: узел просрочен на столько (ручка есть) - прогноз"),
    ("pulse_margin_nohandle_s", "0.03", "с", "то же без ручки (сетку двигают только тележки)"),
    ("pulse_period_s", "0.01", "с", "период таймера пульса по монотонным часам"),
    ("speed_output_delay_s", "0.08", "с",
     "скорость в выходе на столько раньше по времени: эталон скорости проверки "
     "(/localization/kinematic_state) сглажен и запаздывает относительно тележек "
     "около 0,1 с; 0 - без сдвига; положение не сдвигается"),
    ("start_sort_s", "0.1", "с",
     "первые столько секунд по часам прихода сообщения сортируются по метке "
     "(стартовый всплеск bag), сетка - с самой ранней"),
]

# Листы вагона: (калибровка, лист ноды, карта, заголовок).
SHEETS = {
    "jury": ("config/tram_calibration.json", "config/tram.yaml", None, [
        "# Лист параметров ВАГОНА для ноды tram_estimator (контракт ТЗ). ЛИСТ ЖЮРИ:",
        "# калибровка по ВСЕМ данным (train + holdout), уходит в пакет.",
    ]),
    "eval": ("config/eval/tram_calibration.json", "config/eval/tram.yaml", '""', [
        "# Лист параметров ВАГОНА - ЛИСТ ОЦЕНКИ: калибровка только по split train",
        "# (tools/split.json). По нему считаются все наши числа на holdout_scored.",
        "# Карта здесь пустая: инструмент оценки подаёт карту, собранную только из",
        "# обучающих прогонов (config/eval/track_map.npz, analysis/build_map.py eval).",
        "# Жюри этот лист НЕ отдаётся.",
    ]),
}

NODE_ONLY = [
    ("rate_hz", 100.0, "Гц", "частота цикла; из неё получается шаг фильтра dt"),
    ("input_timeout_s", 0.2, "с", "устаревшие входы = отказ одометрии"),
    ("frame_id", '"tram_base"', "-", "система координат в сообщениях"),
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


def _calibration(which):
    with open(os.path.join(PKG, SHEETS[which][0]), encoding="utf-8") as fh:
        return json.load(fh)


def tram_params(which="jury"):
    """Лист вагона: Params с подстановкой калибровки по данным."""
    return Params.from_dict(_calibration(which)["params"])


def vehicle_values(which="jury"):
    """Параметры ноды vehicle, vehicle_ids, vehicle_meas_scale из блока
    "vehicles" калибровки (analysis/calib_vehicle.py): {имя: строка YAML}."""
    blk = _calibration(which).get("vehicles")
    if not blk:
        raise SystemExit(f"{SHEETS[which][0]}: нет блока vehicles - "
                         "python3 analysis/calib_vehicle.py --write")
    ids = sorted(k for k in blk if not k.startswith("_"))
    return {"vehicle": fmt(str(blk["_default"])),
            "vehicle_ids": fmt([str(k) for k in ids]),
            "vehicle_meas_scale": fmt([float(blk[k]["meas_scale"]) for k in ids])}


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
    veh = vehicle_values(which)
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
        if name in veh:
            val = veh[name]
        out.append(f"    {name}: {val}    # {unit} · {doc}")
    out += core_rows(p, skip_dt=False)
    out.append("")
    return plain_dash("\n".join(out))


def plain_dash(text):
    """Текст с дефисом вместо длинного тире: описания Params в ядре пишутся с тире,
    а в листах и таблице MODEL.md длинное тире не используется."""
    return text.replace(" \u2014 ", " - ").replace("\u2014", "-")


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
        "# скрипт. Значения по умолчанию - заглушки для имитатора; при получении ТЗ",
        "# заменяются данными вагона и линии.",
        "#",
        "# Источник: паспорт - документация вагона/датчика/линии;",
        "#           измерение - определяется опознаванием при вводе;",
        "#           настройка - параметр фильтра.",
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
        fh.write(plain_dash("\n".join(out)))
    return path


# знаков: длинная ячейка делает таблицу на GitHub нечитаемой, поэтому длинное описание
# в таблице обрезается до первого разделителя, а полностью идёт списком под таблицей
DOC_CELL_MAX = 110


def doc_cell(doc):
    """Первая часть описания для ячейки таблицы: до первого разделителя, если описание длинное."""
    if len(doc) <= DOC_CELL_MAX:
        return doc
    # chr(0x2014) - длинное тире: в описаниях Params оно чаще отделяет сказуемое, чем
    # пояснение («скачки ... в этом окне - один срыв»), поэтому режем по нему в последнюю очередь
    for seps in ((": ", "; ", " ("), (", ", f" {chr(0x2014)} ")):
        cuts = [i for i in (doc.find(s, 20) for s in seps) if 0 < i <= DOC_CELL_MAX]
        if cuts:
            return doc[:min(cuts)]
    return doc


def sentence(text):
    """Описание в виде предложения: с прописной буквы и с точкой в конце."""
    if text[:1].islower():
        text = text[:1].upper() + text[1:]
    return text if text.endswith((".", "!", "?")) else text + "."


def list_item(text):
    """Пункт списка Markdown, перенесённый по 100 знакам.

    Дефис-тире приклеен к предыдущему слову: в начале строки он открыл бы новый пункт."""
    glued = plain_dash(text).replace(" - ", "\0- ")
    return textwrap.fill(glued, width=100, subsequent_indent="  ", break_long_words=False,
                         break_on_hyphens=False).replace("\0", " ")


def gen_table():
    order, table = groups()
    lines = []
    for g in order:
        rows = [f for f in table[g] if f.name != "dt"]
        if not rows:
            continue
        lines += [f"#### {g}", "",
                  "| Параметр | Единицы | Источник | Смысл | Заглушка |",
                  "| :--- | :--- | :--- | :--- | ---: |"]
        details = []
        for f in rows:
            md = f.metadata
            v = getattr(DEFAULT, f.name)
            shown = "; ".join(fmt(x) for x in v) if isinstance(v, tuple) \
                else fmt(v)
            cell = doc_cell(md["doc"])
            if cell != md["doc"]:
                details.append(list_item(f"- **`{f.name}`.** {sentence(md['doc'])}"))
            # вертикальная черта в описании (|a|) иначе делит ячейку таблицы
            cell = cell.replace("|", "\\|")
            lines.append(f"| `{f.name}` | {md['unit']} | {md['src']} | "
                         f"{cell} | {shown} |")
        lines.append("")
        if details:
            lines += ["Подробнее:", ""] + details + [""]
    return plain_dash("\n".join(lines))


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
