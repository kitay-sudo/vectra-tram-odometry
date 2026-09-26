"""e2e на куске реальной записи (WP9): первые 180 с отложенного прогона
30618_b95ca60a (holdout_scored, tools/split.json) через Runner, как в ноде.

Фикстура: test/data/e2e_30618_b95ca60a_180s.npz (~240 КБ), собрана
`tools/make_e2e_fixture.py 30618_b95ca60a --seconds 180`. Кусок целиком
лежит в квадрате MGRS 37U CB (UTM E 399,0…399,6 км, западнее границы
квадратов 400 км), поэтому на нём видно соглашение о границе квадратов.

Пороги — санитарные (TODO WP9): MAE скорости < 0,08 м/с, средняя 3D < 10 м,
20 Гц, ни одного NaN. Положение сравнивается в той системе, в которой
публикует Runner (refgeo.detect): тест не зависит от параметра projection.
Эталон — точка base_link по tf антенн (как у судьи с 26.09); выход в точке
master от него на 9,87 м вдоль пути и на 3 м выше (test_output_point_is_base_link).
В пары идут только выходы, которые нода публикует в /result/position
(pos_valid), ошибка — без вычета скачка на границе квадратов MGRS, как у судьи.
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
    return E.runner_accepts("projection")


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


def test_refgeo_error_is_judge_like():
    """Основная ошибка — полная разность, как у судьи; скачок ±100 км на
    границе квадратов вычитается только в справочной ошибке, только по x и
    только когда оба — выход и эталон — у края квадрата."""
    ref = np.array([[99999.0, 84941.0, 174.0],     # эталон у границы, квадрат CB
                    [99007.7, 84941.4, 174.2],     # эталон в 1 км от границы
                    [50000.0, 84941.0, 174.0]])
    X = np.array([[1.0, 84941.0, 174.0],           # выход по ту сторону границы (DB)
                  [0.0, 0.0, 0.0],                 # заглушка до выставки
                  [50000.0, 24941.0, 174.0]])      # ошибка 60 км по y
    raw, unw, mism = refgeo.errors(X, ref, "mgrs")
    assert raw[0] == pytest.approx(99998.0) and unw[0] == pytest.approx(2.0)
    assert raw[1] == pytest.approx(130452.0, abs=1.0)   # 3D, не 15 км
    assert unw[1] > 80000.0                              # y не разворачивается
    assert raw[2] == pytest.approx(60000.0) and unw[2] == pytest.approx(60000.0)
    assert mism == 2
    name, d3 = refgeo.detect(X[:1], {"mgrs": ref[:1]})
    assert name == "mgrs" and d3[0] == pytest.approx(99998.0)   # detect -> без развёртки


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


def test_position_mean_3d(run_map, fx):
    outs, m = run_map
    assert m["p_pairs"] > 1500
    assert m["p_mean3d"] < MEAN3D_MAX, m
    assert outs[-1]["pos_ready"], "выставка по GNSS не прошла"
    # положение публикуется на каждом шаге после первого опубликованного
    # (37UCB непрерывно: полосы у края квадрата нет)
    assert m["n_pos_gaps"] == 0, m
    t0 = fx["mfix"][0, 1]
    after = [o for o in outs if o["stamp"] > t0 + 1.0]
    assert all(bool(o.get("pos_valid", True)) for o in after)
    # фикстура целиком в 37UCB: выход и эталон в одном 100-км квадрате
    assert m["squares"] == ["37UCB"]
    assert m["p_square_mismatch"] == 0, m


def test_no_placeholder_position_in_reference_pairs(run_map):
    """Каждое ОПУБЛИКОВАННОЕ положение, которое судья сопоставит с GNSS
    (±0,05 с), — правдоподобное. Прежний Runner до выставки публиковал
    заглушку (s, 0, 0): в относительной системе это рядом с началом, а в
    абсолютной MGRS — 130 км от эталона (на этой фикстуре первая точка master,
    метка 853,60, попадает на выход 853,598 до выставки; одна такая пара из
    1824 добавляет к средней 3D за 3 мин 71,5 м). Поток position: без якоря
    pos_valid=False, и нода /result/position не публикует."""
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
    ({"projection": "mgrs", "mgrs_grid": "37UCB"}, ("mgrs:37UCB", "mgrs"), (99000.0, 100000.0)),
    ({"projection": "mgrs", "mgrs_grid": ""}, ("mgrs", "mgrs:37UCB"), (99000.0, 100000.0)),
    ({"projection": "mgrs", "mgrs_grid": "37UDB"}, ("mgrs:37UDB",), (-1100.0, 0.0)),
    ({"projection": "utm"}, ("utm",), (398000.0, 401000.0)),
    ({"projection": "enu"}, ("enu",), (-100.0, 1000.0)),
], ids=["mgrs-37UCB", "mgrs-wrap", "mgrs-37UDB", "utm", "enu"])
def test_projection_switch(fx, over, expect, xr):
    """Параметр projection/mgrs_grid переключает систему без правки кода; в
    каждой системе точность та же. Эталон — независимая refgeo. Средняя 3D —
    по опубликованным положениям, без развёртки на границе квадратов (как у
    судьи): выход в чужом соглашении о квадратах дал бы ~100 км."""
    if not _has_projection_param():
        pytest.xfail("параметра projection у Runner ещё нет (поток position, WP10)")
    outs = E.replay(fx, use_map=True, gnss="window", **over)
    m = E.metrics(outs, fx, frame=expect[0])        # эталон в заказанной системе
    auto = E.metrics(outs, fx)                        # система, ближайшая к выходу
    assert m["p_mean3d"] < MEAN3D_MAX, m
    assert m["p_square_mismatch"] == 0, m
    # enu и equirect на куске 0,7 км расходятся меньше метра: достаточно, чтобы
    # заказанная система была не хуже лучшей больше чем на 0,5 м
    assert auto["p_frame"] in expect or m["p_mean3d"] - auto["p_mean3d"] < 0.5, (m, auto)
    x = np.array([o["x"] for o in outs if o.get("pos_ready") and o.get("pos_valid", True)])
    assert len(x) and xr[0] < x.min() and x.max() < xr[1], (over, x.min(), x.max())
    assert m["nonfinite"] == 0


def test_map_free_fallback_line(fx, run_map):
    """map_file: "" и nomap_mode "line" (у Runner без параметра nomap_mode —
    единственный режим): положение по прямой вдоль начального курса. Скорость
    от карты не зависит; длина пройденного пути совпадает с GNSS."""
    over = {"nomap_mode": "line"} if E.runner_accepts("nomap_mode") else {}
    outs = E.replay(fx, use_map=False, gnss="window", **over)
    m = E.metrics(outs, fx)
    assert m["nonfinite"] == 0
    assert m["rate_hz"] == pytest.approx(RATE_HZ, abs=0.1)
    assert m["v_mae"] == pytest.approx(run_map[1]["v_mae"], abs=1e-9)
    X = np.array([[o["x"], o["y"]] for o in outs
                  if o.get("pos_ready") and o.get("pos_valid", True)])   # после выставки
    path = float(np.sum(np.linalg.norm(np.diff(X, axis=0), axis=1)))
    assert path == pytest.approx(m["path_m"], rel=0.03), (path, m["path_m"])


def test_map_free_fallback_default(fx, run_map):
    """map_file: "" с режимом по умолчанию. У потока position это "hold":
    после окна выставки положение стоит в якоре. Проверка: выходы конечны,
    20 Гц, положение публикуется, после окна не меняется, якорь — у GNSS окна."""
    if not E.runner_accepts("nomap_mode"):
        pytest.skip("режима nomap_mode нет (без потока position): проверен test_map_free_fallback_line")
    outs = E.replay(fx, use_map=False, gnss="window")
    m = E.metrics(outs, fx)
    _, node = E.sheet()
    assert m["nonfinite"] == 0
    assert m["rate_hz"] == pytest.approx(RATE_HZ, abs=0.1)
    assert m["v_mae"] == pytest.approx(run_map[1]["v_mae"], abs=1e-9)
    assert m["p_pairs"] > 1500, m                    # положение публикуется
    t_end = fx["mfix"][0, 1] + node.get("init_window_s", 3.0) + 0.5
    X = np.array([[o["x"], o["y"], o["z"]] for o in outs
                  if o.get("pos_valid", True) and o.get("pos_ready") and o["stamp"] > t_end])
    assert len(X) > 1500
    assert np.ptp(X, axis=0).max() < 1e-6, "без карты (hold) положение стоит в якоре"
    t_ref, fr = E.reference(fx)                      # base_link по парам антенн
    w = t_ref <= t_end                               # окно выставки
    assert np.linalg.norm(fr[m["p_frame"]][w] - X[-1], axis=1).min() < 5.0, m


def test_output_point_is_base_link(fx, run_map):
    """Выход — base_link (ось передней тележки, уровень рельса), как эталон
    судьи и pathgraph. Тот же прогон с output_point master ближе к антенне
    master, чем к base_link; разница — ~9,87 м вдоль пути и 3 м по высоте."""
    outs, m = run_map
    assert m["p_mean3d"] < MEAN3D_MAX
    mm = E.metrics(outs, fx, point="master")
    assert mm["p_mean3d"] > m["p_mean3d"] + 5.0, (m, mm)       # к антенне — дальше
    om = E.replay(fx, use_map=True, gnss="window", output_point="master")
    a, b = E.metrics(om, fx, point="master"), E.metrics(om, fx)
    assert a["p_mean3d"] < MEAN3D_MAX and b["p_mean3d"] > a["p_mean3d"] + 5.0, (a, b)
    X = np.array([[o["x"], o["y"], o["z"]] for o in outs[-200:]])
    Y = np.array([[o["x"], o["y"], o["z"]] for o in om[-200:]])
    d = X - Y
    assert np.median(np.hypot(d[:, 0], d[:, 1])) == pytest.approx(9.873, abs=0.3)
    assert np.median(d[:, 2]) == pytest.approx(-3.0, abs=0.3)


@pytest.mark.xfail(condition=not E.runner_accepts("projection"),
                   reason="C2: GNSS после окна выставки двигает положение "
                          "(runner.py Runner.on_fix). Исправлено в потоке position "
                          "(WP1): с ним маркер не действует, тест обязан пройти",
                   strict=True)
def test_gnss_whole_slice_does_not_change_position(fx, run_map):
    """GNSS идёт весь кусок (как в демо и, возможно, у жюри): при
    gnss_correction: false после окна выставки он не влияет ни на что. С
    коррекцией (по умолчанию с 26.09) — test_gnss_correction.py: сетка и
    скорость те же, положение ближе к GNSS."""
    outs = E.replay(fx, use_map=True, gnss="all", gnss_correction=False)
    m = E.metrics(outs, fx)
    assert m["v_mae"] < MAE_MAX
    assert m["p_mean3d"] < MEAN3D_MAX, m
    assert E.digest(outs) == E.digest(run_map[0])
