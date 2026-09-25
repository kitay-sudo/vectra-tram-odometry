"""Общие настройки pytest для тестов пакета.

Маркер `slow` — самые долгие тесты имитатора (> 10 с каждый при --cpus 2,
замер 25.09: полный набор 288 с). Быстрый прогон без них:

    python3 -m pytest test/ -m "not slow"         # или FAST=1 docker compose run --rm test

Список ведётся здесь, чтобы не править test_model.py напарника.
"""

import pytest

SLOW = {
    "test_identification_is_accepted_and_recovers_resistance",   # 54 с
    "test_position_over_route",                                  # 20 с
    "test_ice_lock_is_flagged_ambiguous",                        # 14 с
    "test_all_sensors_zero_is_not_taken_for_standstill",         # 12 с
}


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: долгий тест (> 10 с), пропускается в FAST=1")


def pytest_collection_modifyitems(config, items):
    for item in items:
        if getattr(item, "originalname", item.name) in SLOW:
            item.add_marker(pytest.mark.slow)
