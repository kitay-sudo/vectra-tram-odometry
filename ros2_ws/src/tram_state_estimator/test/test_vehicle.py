"""Учёт различий трамваев: параметр vehicle (tram_state_estimator/vehicle.py).

Организаторы 26.09: проверка — только вагон 30618, учёт различий трамваев в
плюс. Проверяется:

  * листы ЖЮРИ и ОЦЕНКИ несут vehicle (по умолчанию 30618), vehicle_ids и
    vehicle_meas_scale — ровно из блока "vehicles" своей калибровки;
  * масштаб вагонов листа ОЦЕНКИ подогнан только по train, ЖЮРИ — по всем
    записям своего вагона; «общий» масштаб блока равен meas_scale листа;
  * выбор вагона: известный — его meas_scale; auto — лист как есть;
    незнакомое значение или негодная таблица — auto с предупреждением;
  * нода и оценка применяют вагон одинаково (tram_node.py, tools/eval_replay.py);
  * вагон меняет только масштаб колёс: скорость ядра на тех же показаниях
    масштабируется ровно на отношение масштабов.
"""

import ast
import importlib.util
import json
import math
import os
import sys
from dataclasses import fields, replace

import numpy as np
import pytest

from tram_state_estimator import vehicle as V
from tram_state_estimator.estimator_core import IS, Params

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
SHEETS = [w for w in GEN.SHEETS if os.path.exists(os.path.join(PKG, GEN.SHEETS[w][0]))]


def _yaml(which):
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(PKG, GEN.SHEETS[which][1]), encoding="utf-8") as fh:
        return yaml.safe_load(fh)["/tram_state_estimator"]["ros__parameters"]


def _calib(which):
    with open(os.path.join(PKG, GEN.SHEETS[which][0]), encoding="utf-8") as fh:
        return json.load(fh)


def _params(which):
    got = _yaml(which)
    return Params.from_dict({k: v for k, v in got.items() if k in CORE}), \
        {k: v for k, v in got.items() if k not in CORE}


# ---------------------------------------------------------------- листы

@pytest.mark.parametrize("which", SHEETS)
def test_sheet_carries_vehicle_table_from_calibration(which):
    node = _params(which)[1]
    blk = _calib(which)["vehicles"]
    ids = sorted(k for k in blk if not k.startswith("_"))
    assert ids == ["30618", "30639"]
    assert node["vehicle"] == "30618" == blk["_default"]      # проверка — только 30618
    assert node["vehicle_ids"] == ids
    assert node["vehicle_meas_scale"] == [blk[k]["meas_scale"] for k in ids]
    assert all(isinstance(x, float) for x in node["vehicle_meas_scale"])


@pytest.mark.parametrize("which", SHEETS)
def test_mixed_scale_of_vehicle_block_is_the_sheet_scale(which):
    """Блок вагонов посчитан тем же способом, что общий meas_scale листа."""
    d = _calib(which)
    assert d["vehicles"]["_mixed_meas_scale"] == pytest.approx(
        d["params"]["meas_scale"], abs=2e-6)
    for v in ("30618", "30639"):
        s = d["vehicles"][v]["meas_scale"]
        assert V.SCALE_MIN < s < V.SCALE_MAX
        # вагоны различаются меньше чем на 1 % (по данным 0,1…0,4 %)
        assert abs(s / d["params"]["meas_scale"] - 1.0) < 0.01


@pytest.mark.skipif(not os.path.exists(SPLIT), reason="нет tools/split.json")
def test_vehicle_scales_fit_on_own_vehicle_and_right_split():
    with open(SPLIT, encoding="utf-8") as fh:
        split = json.load(fh)
    hold = set(split["holdout"]) | set(split["holdout_scored"])
    for which in SHEETS:
        blk = _calib(which)["vehicles"]
        for v in ("30618", "30639"):
            ids = set(blk[v]["_fit_ids"])
            assert ids and all(b.startswith(v + "_") for b in ids)
            assert blk[v]["_fit_runs_gnss"] >= 10
            if which == "eval":
                assert ids <= set(split["train"]) and not ids & hold
            else:
                assert {b for b in split["holdout_scored"] if b.startswith(v)} <= ids


def test_node_defaults_equal_jury_sheet():
    """tram_node.py объявляет vehicle и таблицу с умолчаниями листа ЖЮРИ
    (ros2 run без листа ведёт себя как лист пакета) и применяет вагон к
    Params до Runner."""
    src = os.path.join(PKG, "tram_state_estimator", "tram_node.py")
    with open(src, encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    decl = {}
    calls = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "P" \
                and len(n.args) == 2 and isinstance(n.args[0], ast.Constant):
            try:
                decl[n.args[0].value] = ast.literal_eval(n.args[1])
            except ValueError:
                pass
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and isinstance(n.func.value, ast.Name) and n.func.value.id == "vehicle_sheet":
            calls.add(n.func.attr)
    node = _params("jury")[1]
    for k in ("vehicle", "vehicle_ids", "vehicle_meas_scale"):
        assert decl[k] == node[k], k
    assert "apply" in calls


def test_jury_default_is_30618_scale():
    p, node = _params("jury")
    q, info = V.apply_node(p, node)
    blk = _calib("jury")["vehicles"]
    assert info["used"] == "30618" and info["warning"] is None
    assert q.meas_scale == blk["30618"]["meas_scale"]
    assert p.meas_scale == _calib("jury")["params"]["meas_scale"]   # общий в листе
    # остальное ядро не меняется
    assert replace(q, meas_scale=p.meas_scale) == p


# ---------------------------------------------------------------- выбор вагона

IDS, SCALES = ["30618", "30639"], [1.0014, 0.9976]
BASE = Params(meas_units="km_h", meas_scale=1.0011)


@pytest.mark.parametrize("value, want", [
    ("30618", 1.0014), ("30639", 0.9976), (30639, 0.9976), (30639.0, 0.9976),
    (" 30618 ", 1.0014), ("auto", 1.0011), ("AUTO", 1.0011), ("", 1.0011)])
def test_known_vehicle_and_auto(value, want):
    p, info = V.apply(BASE, value, IDS, SCALES)
    assert p.meas_scale == want
    assert info["warning"] is None
    assert info["used"] == (V.normalize(value) if want != 1.0011 else "auto")
    assert info["sheet_meas_scale"] == 1.0011
    assert V.describe(info)


@pytest.mark.parametrize("value", ["31000", "30618x", "none", 0, "3061"])
def test_unknown_vehicle_falls_back_to_auto_with_warning(value):
    p, info = V.apply(BASE, value, IDS, SCALES)
    assert p == BASE
    assert info["used"] == "auto"
    assert info["warning"] and "auto" in info["warning"] and "неизвестен" in info["warning"]


@pytest.mark.parametrize("ids, scales", [
    (["30618", "30639"], [1.0]),                    # разной длины
    (["30618"], [float("nan")]),                    # не число
    (["30618"], [1.5]),                             # вне 0,9…1,1
    (["30618"], ["abc"]),
    (["30618"], [-1.0])])
def test_bad_table_falls_back_to_auto_with_warning(ids, scales):
    p, info = V.apply(BASE, "30618", ids, scales)
    assert p == BASE and info["used"] == "auto" and info["warning"]
    p, info = V.apply(BASE, "auto", ids, scales)      # auto: предупреждение, лист как есть
    assert p == BASE and info["warning"]


def test_sheet_without_vehicle_params_is_unchanged():
    """Лист до 26.09 (без vehicle): Params как есть, без предупреждения."""
    p, info = V.apply_node(BASE, {"map_file": ""})
    assert p == BASE and info["used"] == "auto" and info["warning"] is None


def test_vehicle_only_rescales_wheel_speed():
    """Вагон — только масштаб колёс: та же запись (фикстура реального прогона
    30618, 180 с) с масштабом другого вагона даёт на ходу скорость и путь ядра
    в отношение масштабов больше или меньше."""
    import e2e_replay as E
    from tram_state_estimator.runner import Runner
    fx = E.load_fixture()
    p, node = _params("jury")
    ps = [V.apply(p, v, node["vehicle_ids"], node["vehicle_meas_scale"])[0]
          for v in ("30618", "30639")]
    k = ps[1].meas_scale / ps[0].meas_scale
    assert abs(k - 1.0) > 1e-3
    res = []
    for q in ps:
        r = Runner(q)
        outs = []
        for _tb, kind, i, th, val in E.events(fx, gnss="none"):
            outs += r.on_wheel(i, th, val) if kind == 0 else r.on_handle(th, val)
        res.append((np.array([o["v"] for o in outs]), float(r.core.x[IS])))
    (va, sa), (vb, sb) = res
    assert len(va) == len(vb) > 3000
    mv = va > 3.0
    assert mv.sum() > 500
    assert np.median(vb[mv] / va[mv]) == pytest.approx(k, rel=3e-4)
    assert sa > 300.0 and sb / sa == pytest.approx(k, rel=2e-3)


# ---------------------------------------------------------------- оценка = нода

def _eval_replay():
    tools = os.path.join(REPO, "tools")
    if not os.path.exists(os.path.join(tools, "eval_replay.py")):
        pytest.skip("нет tools/eval_replay.py")
    pytest.importorskip("yaml")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import eval_replay
    return eval_replay


def test_eval_applies_vehicle_like_the_node():
    R = _eval_replay()
    sheet = R.resolve_sheet("jury")
    p = R.make_params(sheet)
    want, _ = V.apply_node(R.make_params(dict(sheet, node={})), sheet["node"])
    assert p == want and p.meas_scale == _calib("jury")["vehicles"]["30618"]["meas_scale"]
    # vehicle=match (только оценка): вагон по имени прогона
    s2 = dict(sheet, node=dict(sheet["node"], vehicle="match"))
    assert R.make_params(s2, bag="30639_3b3d9eb8").meas_scale == \
        _calib("jury")["vehicles"]["30639"]["meas_scale"]
    assert R.make_params(s2, bag="30618_3e9f4952").meas_scale == \
        _calib("jury")["vehicles"]["30618"]["meas_scale"]
    s3 = dict(sheet, node=dict(sheet["node"], vehicle="auto"))
    assert R.make_params(s3).meas_scale == _calib("jury")["params"]["meas_scale"]
    # параметры вагона не считаются «не дошедшими до Runner»
    _, unused = R.make_runner(p, sheet["node"], None)
    assert not {"vehicle", "vehicle_ids", "vehicle_meas_scale"} & set(unused)
    assert math.isfinite(p.meas_scale)
