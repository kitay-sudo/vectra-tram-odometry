"""e2e на куске реальной записи (WP9): первые 180 с отложенного прогона
30618_b95ca60a (holdout_scored, tools/split.json) через Runner, как в ноде.

Фикстура: test/data/e2e_30618_b95ca60a_180s.npz (~240 КБ), собрана
`tools/make_e2e_fixture.py 30618_b95ca60a --seconds 180`. Кусок целиком
лежит в квадрате MGRS 37U CB (UTM E 399,0…399,6 км, западнее границы
квадратов 400 км), поэтому на нём видно соглашение о границе квадратов.

Пороги — санитарные (TODO WP9): MAE скорости < 0,08 м/с, средняя 3D < 10 м,
20 Гц, ни одного NaN. Положение сравнивается в той системе, в которой
публикует Runner (refgeo.detect): тест не зависит от параметра projection.
Это не отчётные числа точности: лист и карта здесь боевые (config/, построены
по всем данным, этот прогон в них входит). Отчётные числа считает
tools/eval.py на оценочных листе и карте.
"""

import numpy as np
import pytest

import e2e_replay as E
import refgeo

MAE_MAX = 0.08          # м/с
MEAN3D_MAX = 10.0       # м
RATE_HZ = 20.0


@pytest.fixture(scope="module")
def fx():
    return E.load_fixture()


@pytest.fixture(scope="module")
def run_map(fx):
    outs = E.replay(fx, use_map=True, gnss="window")
    return outs, E.metrics(outs, fx)


def _has_projection_param():
    from tram_state_estimator.runner import Runner
    return E._accepts(Runner.__init__, "projection")


def test_fixture_is_holdout_slice(fx):
    import json
    import os
    meta = fx["meta"]
    assert meta["split"] == "holdout_scored"
    assert 120.0 <= meta["seconds"] <= 180.0
    root = os.path.dirname(os.path.dirname(os.path.dirname(E.PKG)))   # корень репо
    split = os.path.join(root, "tools", "split.json")
    if os.path.exists(split):                       # есть только в полном репозитории
        with open(split, encoding="utf-8") as fh:
            assert meta["bag"] in json.load(fh)["holdout_scored"]
    assert os.path.getsize(E.FIXTURE) < 1024 * 1024
    for key in ("front", "rear", "cmd", "mfix", "rfix", "mvel", "rvel"):
        assert len(fx[key]) > 100, key


def test_refgeo_self_check(fx):
    """Эталонная геодезия: два независимых ряда UTM сходятся, квадраты MGRS
    по обе стороны границы 400 км определяются верно."""
    m = fx["mfix"]
    e1, n1, zone = refgeo.utm(m[:, 2], m[:, 3])
    e2, n2 = refgeo.utm_snyder(m[:, 2], m[:, 3], zone)
    assert zone == 37
    assert np.abs(e1 - e2).max() < 0.01 and np.abs(n1 - n2).max() < 0.01
    assert refgeo.mgrs_squares(m[:, 2], m[:, 3]) == ["37UCB"]
    assert refgeo.mgrs_squares([55.80], [37.46]) == ["37UDB"]
    x_wrap, _ = refgeo.mgrs_xy(m[:, 2], m[:, 3], "")
    x_db, _ = refgeo.mgrs_xy(m[:, 2], m[:, 3], "37UDB")
    assert np.all((x_wrap > 99000) & (x_wrap < 100000))
    assert np.allclose(x_wrap - x_db, 100000.0)


def test_rate_20hz_monotonic_stamps(run_map):
    outs, m = run_map
    assert m["rate_hz"] == pytest.approx(RATE_HZ, abs=0.1)
    assert m["stamp_step_min"] > 0.049 and m["stamp_step_max"] < 0.051
    assert m["span_s"] > 170.0


def test_no_nan(run_map):
    outs, m = run_map
    assert m["nonfinite"] == 0
    for o in outs:
        assert np.isfinite([o["a"], o["s"]]).all()


def test_speed_mae(run_map):
    outs, m = run_map
    assert m["v_pairs"] > 1500
    assert m["v_mae"] < MAE_MAX, m
    assert m["v_pairs_rover"] > 1500
    assert m["v_mae_rover"] < MAE_MAX, m      # контрольный источник: rover/vel


def test_position_mean_3d(run_map):
    outs, m = run_map
    assert m["p_pairs"] > 1500
    assert m["p_mean3d"] < MEAN3D_MAX, m
    assert outs[-1]["pos_ready"], "выставка по GNSS не прошла"


def test_no_placeholder_position_in_reference_pairs(run_map):
    """Каждое положение, которое судья сопоставит с GNSS (±0,05 с), —
    правдоподобное. До выставки Runner публикует заглушку (s, 0, 0). В
    относительной системе это рядом с началом, а в абсолютной (MGRS, UTM) —
    за 100+ км от эталона: на этой фикстуре первая точка master (метка 853,60)
    попадает на выход 853,598 до выставки, и одна такая пара добавляет ~70 м к
    средней 3D за 3 мин. Нужно не публиковать положение до выставки или
    публиковать первую точку master."""
    outs, m = run_map
    assert m["p_max3d"] < 100.0, (
        f"пар с выходами до выставки: {m['p_pairs_unaligned']}, их макс. ошибка "
        f"{m['p_max3d_unaligned']:.0f} м в системе {m['p_frame']}", m)


def test_output_frame_is_mgrs_by_default(run_map):
    """Организаторы 25.09: «плоские координаты именно в MGRS». По умолчанию
    нода публикует абсолютные MGRS: x — easting, y — northing, z — высота."""
    outs, m = run_map
    if not m["p_frame"].startswith("mgrs") and not _has_projection_param():
        pytest.xfail(f"проекция MGRS ещё не влита (поток position, WP10): "
                     f"выход в {m['p_frame']}")
    assert m["p_frame"].startswith("mgrs"), m
    x = np.array([o["x"] for o in outs[-100:]])
    z = np.array([o["z"] for o in outs[-100:]])
    assert np.all(np.abs(x) < 100000.0)
    assert np.all(z > 100.0), "z — абсолютная высота (здесь около 150 м)"


# Кусок фикстуры целиком западнее границы квадратов (UTM E 399,0…399,6 км):
# по диапазону x видно, какое соглашение реально применила нода.
@pytest.mark.parametrize("over,expect,xr", [
    ({"projection": "mgrs", "mgrs_grid": ""}, ("mgrs", "mgrs:37UCB"), (99000.0, 100000.0)),
    ({"projection": "mgrs", "mgrs_grid": "37UDB"}, ("mgrs:37UDB",), (-1100.0, 0.0)),
    ({"projection": "utm"}, ("utm",), (398000.0, 401000.0)),
    ({"projection": "enu"}, ("enu",), (-100.0, 1000.0)),
], ids=["mgrs-wrap", "mgrs-37UDB", "utm", "enu"])
def test_projection_switch(fx, over, expect, xr):
    """Параметр projection/mgrs_grid переключает систему без правки кода; в
    каждой системе точность та же. Эталон — независимая refgeo. Средняя 3D —
    по выходам после выставки (заглушку до неё ловит отдельный тест)."""
    if not _has_projection_param():
        pytest.xfail("параметра projection у Runner ещё нет (поток position, WP10)")
    outs = E.replay(fx, use_map=True, gnss="window", **over)
    m = E.metrics(outs, fx, frame=expect[0])        # эталон в заказанной системе
    auto = E.metrics(outs, fx)                        # система, ближайшая к выходу
    assert m["p_mean3d_aligned"] < MEAN3D_MAX, m
    # enu и equirect на куске 0,7 км расходятся меньше метра: достаточно, чтобы
    # заказанная система была не хуже лучшей больше чем на 0,5 м
    assert auto["p_frame"] in expect or m["p_mean3d_aligned"] - auto["p_mean3d_aligned"] < 0.5, (m, auto)
    x = np.array([o["x"] for o in outs if o.get("pos_ready")])
    assert len(x) and xr[0] < x.min() and x.max() < xr[1], (over, x.min(), x.max())
    assert m["nonfinite"] == 0


def test_map_free_fallback_keeps_working(fx, run_map):
    """map_file: "" — положение без карты (по прямой вдоль начального курса).
    Скорость от карты не зависит; длина пройденного пути совпадает с GNSS."""
    outs = E.replay(fx, use_map=False, gnss="window")
    m = E.metrics(outs, fx)
    assert m["nonfinite"] == 0
    assert m["rate_hz"] == pytest.approx(RATE_HZ, abs=0.1)
    assert m["v_mae"] == pytest.approx(run_map[1]["v_mae"], abs=1e-9)
    X = np.array([[o["x"], o["y"]] for o in outs if o.get("pos_ready")])    # после выставки
    path = float(np.sum(np.linalg.norm(np.diff(X, axis=0), axis=1)))
    assert path == pytest.approx(m["path_m"], rel=0.03), (path, m["path_m"])


@pytest.mark.xfail(reason="C2: GNSS после окна выставки двигает положение "
                          "(runner.py Runner.on_fix). Снимается WP1 (поток position); "
                          "после слияния убрать xfail", strict=False)
def test_gnss_whole_slice_does_not_change_position(fx, run_map):
    """GNSS идёт весь кусок (как в демо и, возможно, у жюри): после окна
    выставки он не должен влиять ни на что."""
    outs = E.replay(fx, use_map=True, gnss="all")
    m = E.metrics(outs, fx)
    assert m["v_mae"] < MAE_MAX
    assert m["p_mean3d"] < MEAN3D_MAX, m
    assert E.digest(outs) == E.digest(run_map[0])
