"""Пример координат от организаторов (чат хакатона, 26.09.2026 17:02).

«MGRS код 37UCB035858. Пример координат xyz: x 103501.6309, y 85876.1201,
z 167.4109 для координат lat 55.8088325462547, lon 37.4602768500852.
/result/position — это положение base_link.»

Точка лежит восточнее границы квадратов CB|DB (UTM easting 403 501 м), поэтому
пример заодно подтверждает соглашение: x считается от квадрата 37UCB
непрерывно, без переноса на границе. z здесь не проверяем: в примере это
высота base_link, а широта и долгота даны без высоты антенны.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tram_state_estimator.geodesy import Frame  # noqa: E402

LAT, LON = 55.8088325462547, 37.4602768500852
X_ORG, Y_ORG = 103501.6309, 85876.1201


def test_organizer_example_37ucb_continuous():
    # начало кадра — любая точка линии (здесь западнее границы, в квадрате CB)
    f = Frame(55.80, 37.39, 150.0, projection="mgrs", mgrs_grid="37UCB")
    x, y, _ = f.out(*f.fwd(LAT, LON, 170.0))
    assert abs(x - X_ORG) < 1e-3
    assert abs(y - Y_ORG) < 1e-3


def test_organizer_example_default_grid_is_37ucb():
    """Умолчание в коде и в листе жюри — квадрат 37UCB без переноса."""
    import inspect
    import re
    from tram_state_estimator.runner import Position
    assert inspect.signature(Position.__init__).parameters["mgrs_grid"].default == "37UCB"
    sheet = os.path.join(os.path.dirname(__file__), "..", "config", "tram.yaml")
    text = open(sheet, encoding="utf-8").read()
    assert re.search(r'^\s*projection:\s*"mgrs"', text, re.M)
    assert re.search(r'^\s*mgrs_grid:\s*"37UCB"', text, re.M)
