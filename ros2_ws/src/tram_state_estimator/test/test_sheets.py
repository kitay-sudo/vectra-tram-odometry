"""Один источник правды для листов вагона (WP2).

Нода читает config/tram.yaml, оценка — config/tram_calibration.json. Прежде
yaml не перегенерировали после смены q_v, и нода считала другим листом, чем
оценка (MAE 0,0515 против 0,0485). Здесь проверяется:

  * yaml — ровно вывод tools/gen_params.py из своей калибровки (руками не
    правится);
  * все поля ядра в yaml равны калибровке, а поля, которых в калибровке нет,
    равны значениям Params по умолчанию;
  * yaml несёт параметры ноды вне ядра (карта, проекция, сетка MGRS, кадры,
    таймауты);
  * лист оценки подогнан только по split train, лист жюри — по всем данным.
"""

import importlib.util
import json
import os
from dataclasses import fields

import pytest

from tram_state_estimator.estimator_core import Params, DEFAULT

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
SPLIT = os.path.join(REPO, "tools", "split.json")
CORE = {f.name for f in fields(Params)}


def _gen():
    spec = importlib.util.spec_from_file_location(
        "gen_params", os.path.join(PKG, "tools", "gen_params.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


GEN = _gen()
SHEETS = [w for w in GEN.SHEETS
          if os.path.exists(os.path.join(PKG, GEN.SHEETS[w][0]))]


def _yaml(which):
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, GEN.SHEETS[which][1]), encoding="utf-8") as fh:
        return yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]


def _calib(which):
    with open(os.path.join(PKG, GEN.SHEETS[which][0]), encoding="utf-8") as fh:
        return json.load(fh)


def _same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        return a == pytest.approx(b, rel=1e-9, abs=1e-12)
    return a == b


def test_jury_sheet_exists():
    assert "jury" in SHEETS


@pytest.mark.parametrize("which", SHEETS)
def test_yaml_is_generated_from_calibration(which):
    """Файл совпадает с выводом генератора байт в байт."""
    with open(os.path.join(PKG, GEN.SHEETS[which][1]), encoding="utf-8") as fh:
        text = fh.read()
    assert text == GEN.tram_yaml_text(which), \
        f"{GEN.SHEETS[which][1]} устарел: запустите tools/gen_params.py"


@pytest.mark.parametrize("which", SHEETS)
def test_every_core_field_equals_calibration(which):
    got = _yaml(which)
    over = _calib(which)["params"]
    assert set(over) <= CORE, f"в калибровке неизвестные поля: {set(over) - CORE}"
    p_yaml = Params.from_dict({k: v for k, v in got.items() if k in CORE})
    p_json = Params.from_dict(over)
    diff = [f.name for f in fields(Params)
            if not _same(getattr(p_yaml, f.name), getattr(p_json, f.name))]
    assert not diff, f"yaml и калибровка разошлись: {diff}"
    # всё, что задано калибровкой, записано в yaml явно (пустые списки yaml
    # не пишет: парсер ROS 2 их не принимает)
    missing = [k for k, v in over.items() if k not in got and v != []]
    assert not missing, f"поля калибровки не записаны в yaml: {missing}"
    # поля без калибровки — значения Params по умолчанию
    for f in fields(Params):
        if f.name not in over and getattr(DEFAULT, f.name) != ():
            assert _same(getattr(p_yaml, f.name), getattr(DEFAULT, f.name)), f.name


@pytest.mark.parametrize("which", SHEETS)
def test_node_params_are_in_sheet(which):
    got = _yaml(which)
    node = {n for n, *_ in GEN.TRAM_NODE_ONLY}
    assert {"map_file", "projection", "mgrs_grid", "utm_zone", "frame_id",
            "child_frame_id", "wheel_timeout_s", "handle_timeout_s",
            "init_window_s"} <= node
    assert set(got) - CORE == node
    assert got["projection"] == "mgrs" and got["mgrs_grid"] == "" \
        and got["utm_zone"] == 0
    want = {f.name for f in fields(Params) if getattr(DEFAULT, f.name) != ()}
    assert want <= set(got), f"в листе нет полей ядра: {want - set(got)}"


def test_jury_sheet_ships_the_map_and_eval_sheet_does_not():
    assert _yaml("jury")["map_file"] == "config/track_map.npz"
    assert os.path.exists(os.path.join(PKG, "config", "track_map.npz"))
    if "eval" in SHEETS:
        assert _yaml("eval")["map_file"] == ""


@pytest.mark.skipif(not os.path.exists(SPLIT), reason="нет tools/split.json")
def test_eval_sheet_is_fit_on_train_only_and_jury_on_all():
    """Дисциплина утечки: лист оценки — только split train; лист жюри — все
    уникальные записи."""
    with open(SPLIT, encoding="utf-8") as fh:
        split = json.load(fh)
    hold = set(split["holdout"]) | set(split["holdout_scored"])
    if "eval" in SHEETS:
        ids = set(_calib("eval")["_fit_ids"])
        assert ids and ids <= set(split["train"]) and not ids & hold
    ids = set(_calib("jury").get("_fit_ids", []))
    if ids:
        assert set(split["holdout_scored"]) <= ids
