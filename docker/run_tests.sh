#!/usr/bin/env bash
# run_tests.sh - сборка пакетов и тесты (compose: сервис test; CI; любой Humble).
#
#   docker compose run --rm test                  # colcon build + pytest (кроме маркера timing)
#   docker compose run --rm test -k e2e           # аргументы уходят в pytest
#   FAST=1 docker compose run --rm test           # без 4 самых долгих тестов (маркер slow)
#   docker compose run --rm test -m timing        # только timing (в compose по умолчанию NO_TIMING=1)
#   NO_TIMING=1 ...                               # без тестов на время шага (маркер timing)
#   SRC=$PWD/ros2_ws/src bash docker/run_tests.sh # вне Docker (CI): исходники из рабочего дерева
#   SKIP_BUILD=1 ...                              # не пересобирать, если /tmp/ws уже есть
#
# Исходники берутся из смонтированного репозитория (/repo, только чтение) и
# собираются во временный workspace /tmp/ws, так что тестируется ровно то, что
# лежит в рабочем дереве. Сервис compose идёт с network_mode: none - заодно
# проверка, что colcon build и тесты не ходят в сеть.
#
# Маркер timing (test/conftest.py): p99 времени шага ядра < 5 мс; при нехватке CPU
# (--cpus 2 рядом с другой нагрузкой, облачный CI) может «мигать». В CI он идёт
# отдельным шагом, который не валит сборку.
set -eo pipefail
SRC=${SRC:-/repo/ros2_ws/src}
WS=/tmp/ws
source /opt/ros/humble/setup.bash

nif=$(ls /sys/class/net 2>/dev/null | grep -vc '^lo$' || true)
if [ "${SKIP_BUILD:-0}" = "1" ] && [ -f "$WS/install/setup.bash" ]; then
  echo "[test] colcon build пропущен (SKIP_BUILD=1, $WS уже собран)"
else
  echo "[test] colcon build: $SRC -> $WS (сетевых интерфейсов кроме lo: $nif)"
  t0=$(date +%s)
  mkdir -p "$WS"
  colcon --log-base "$WS/log" build --base-paths "$SRC" --build-base "$WS/build" \
    --install-base "$WS/install" --event-handlers console_cohesion- summary+ >"$WS/build.log" 2>&1 \
    || { cat "$WS/build.log"; echo "[test] colcon build: ОШИБКА"; exit 1; }
  echo "[test] colcon build: $(grep -m1 '^Summary:' "$WS/build.log" | sed 's/^Summary: //'), $(( $(date +%s) - t0 )) с"
  # предупреждения setuptools о выключенной байт-компиляции (PYTHONDONTWRITEBYTECODE
  # образа) безвредны; остальное из stderr сборки показываем
  extra=$(cat "$WS"/log/latest_build/*/stderr.log 2>/dev/null \
          | grep -v -e "byte-compiling is disabled" -e '^[[:space:]]*$' || true)
  if [ -n "$extra" ]; then echo "[test] colcon build, stderr:"; echo "$extra" | head -20; fi
fi
source "$WS/install/setup.bash"

cd "$SRC/tram_state_estimator"
args=(-q -p no:cacheprovider -rfEsxX --durations=10)
mexpr=""
[ "${FAST:-0}" = "1" ] && mexpr="not slow"
[ "${NO_TIMING:-0}" = "1" ] && mexpr="${mexpr:+$mexpr and }not timing"
[ -n "$mexpr" ] && args+=(-m "$mexpr")
echo "[test] pytest ${args[*]} $*"
t0=$(date +%s)
# PYTHONPATH на исходники: тесты и фикстуры - из рабочего дерева
PYTHONPATH="$SRC/tram_state_estimator:$PYTHONPATH" python3 -m pytest test/ "${args[@]}" "$@" && rc=0 || rc=$?
if [ $rc = 0 ]; then verdict="PASS"; else verdict="FAIL"; fi
echo "[test] ИТОГ: $verdict (pytest, код $rc) за $(( $(date +%s) - t0 )) с"
exit $rc
