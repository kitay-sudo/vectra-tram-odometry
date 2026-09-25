#!/usr/bin/env bash
# run_tests.sh — сборка пакетов и тесты в контейнере (compose: сервис test).
#
#   docker compose run --rm test                  # colcon build + весь pytest
#   docker compose run --rm test -k e2e           # аргументы уходят в pytest
#   FAST=1 docker compose run --rm test           # без самых долгих тестов напарника
#
# Исходники берутся из смонтированного репозитория (/repo, только чтение) и
# собираются во временный workspace /tmp/ws, так что тестируется ровно то, что
# лежит в рабочем дереве. Сервис идёт с network_mode: none — заодно проверка,
# что colcon build и тесты не ходят в сеть.
set -eo pipefail
SRC=${SRC:-/repo/ros2_ws/src}
WS=/tmp/ws
source /opt/ros/humble/setup.bash

nif=$(ls /sys/class/net 2>/dev/null | grep -vc '^lo$')
echo "[test] colcon build: $SRC -> $WS (сетевых интерфейсов кроме lo: $nif)"
t0=$(date +%s)
mkdir -p "$WS"
colcon --log-base "$WS/log" build --base-paths "$SRC" --build-base "$WS/build" \
  --install-base "$WS/install" --event-handlers console_cohesion- summary+ >"$WS/build.log" 2>&1 \
  || { cat "$WS/build.log"; echo "[test] colcon build FAILED"; exit 1; }
tail -1 "$WS/build.log"
echo "[test] colcon build: $(( $(date +%s) - t0 )) с"
source "$WS/install/setup.bash"

cd "$SRC/tram_state_estimator"
args=(-q -p no:cacheprovider -rfEsxX --durations=10)
if [ "${FAST:-0}" = "1" ]; then
  # самые долгие тесты имитатора напарника (> 10 с каждый при --cpus 2)
  args+=(-m "not slow")
fi
echo "[test] pytest ${args[*]} $*"
t0=$(date +%s)
# PYTHONPATH на исходники: тесты и фикстуры — из рабочего дерева
PYTHONPATH="$SRC/tram_state_estimator:$PYTHONPATH" python3 -m pytest test/ "${args[@]}" "$@" && rc=0 || rc=$?
echo "[test] pytest: код $rc за $(( $(date +%s) - t0 )) с"
exit $rc
