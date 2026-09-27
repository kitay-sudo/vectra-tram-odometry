"""Общие настройки pytest для тестов пакета.

Маркеры (списки ведутся здесь, а не в самих тестовых файлах):

- `slow` — самые долгие тесты имитатора (> 10 с каждый при --cpus 2). Быстрый
  прогон: `-m "not slow"` или
  `FAST=1 docker compose run --rm test`.
- `timing` — проверки времени шага по стенным часам. При нехватке CPU (другая
  нагрузка рядом, облачный CI) могут «мигать»: параллельно с ROS-смоуком
  `test_step_time_within_budget` падал на p99. В CI идут отдельным
  шагом, который не валит сборку; `NO_TIMING=1` их пропускает.
"""

import pytest

SLOW = {
    "test_identification_is_accepted_and_recovers_resistance",   # 54 с
    "test_position_over_route",                                  # 20 с
    "test_ice_lock_is_flagged_ambiguous",                        # 14 с
    "test_all_sensors_zero_is_not_taken_for_standstill",         # 12 с
}
TIMING = {
    "test_step_time_within_budget",
}


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: долгий тест (> 10 с), пропускается при FAST=1")
    config.addinivalue_line("markers", "timing: проверка времени по стенным часам, зависит от загрузки CPU")


def pytest_collection_modifyitems(config, items):
    for item in items:
        name = getattr(item, "originalname", item.name)
        if name in SLOW:
            item.add_marker(pytest.mark.slow)
        if name in TIMING:
            item.add_marker(pytest.mark.timing)
