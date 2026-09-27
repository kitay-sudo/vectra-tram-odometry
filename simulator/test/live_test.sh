#!/usr/bin/env bash
# Живой ROS 2 против настоящего rosbridge: контейнер ROS (vectra/tram:dev: нода +
# rosbridge + ros2 bag play, simulator/deploy/live_demo.sh) и контейнер браузера
# (vectra/tram:sim: simulator/test/live_test.js) в одной сети docker. Посреди
# проверки ROS-контейнер останавливается и запускается снова (обрыв и переподключение).
#
#   bash simulator/test/live_test.sh [bag=30618_e9a34502]      (Git Bash / Linux, из корня)
# Переменные: DATA_DIR (<repo>/data), OUT (<repo>/out/sim/live), ROSBRIDGE_OPEN=1 — мост без
# ограничений (контроль: проверка «мост только для чтения» тогда должна упасть)
set -euo pipefail
export MSYS_NO_PATHCONV=1
ROOT="$(cd "$(dirname "$0")/../.." && (pwd -W 2>/dev/null || pwd))"
BAG="${1:-30618_e9a34502}"
DATA_DIR="${DATA_DIR:-$ROOT/data}"
OUT="${OUT:-$ROOT/out/sim/live}"
NET="tv-sim-net-$RANDOM"
ROSC="tv-sim-ros-$RANDOM"
mkdir -p "$OUT"; rm -f "$OUT"/live_phase*.done
docker network create "$NET" >/dev/null
trap 'docker rm -f "$ROSC" >/dev/null 2>&1 || true; docker network rm "$NET" >/dev/null 2>&1 || true' EXIT
start_ros() {
  docker run -d --rm --name "$ROSC" --network "$NET" --network-alias sim-ros --cpus 2 \
    -e BAG="$BAG" -e ROSBRIDGE_OPEN="${ROSBRIDGE_OPEN:-0}" -e ROS_DOMAIN_ID="$((RANDOM % 90 + 110))" \
    -v "$ROOT:/repo:ro" -v "$DATA_DIR:/data:ro" vectra/tram:dev \
    bash /repo/simulator/deploy/live_demo.sh >/dev/null
}
start_ros
docker run --rm --network "$NET" -v "$ROOT/simulator:/sim:ro" -v "$OUT:/out" vectra/tram:sim \
  node /sim/test/live_test.js /sim/index.html ws://sim-ros:9090 /out &
BR=$!
while [ ! -f "$OUT/live_phase1.done" ]; do kill -0 $BR 2>/dev/null || break; sleep 1; done
docker logs "$ROSC" >"$OUT/ros_phase1.log" 2>&1 || true
docker exec "$ROSC" bash -c 'cat /tmp/node.log | tail -5; tail -3 /tmp/rosbridge.log' >>"$OUT/ros_phase1.log" 2>&1 || true
docker rm -f "$ROSC" >/dev/null 2>&1 || true
while [ ! -f "$OUT/live_phase2.done" ]; do kill -0 $BR 2>/dev/null || break; sleep 1; done
start_ros
wait $BR && echo "live_test: OK" || { echo "live_test: FAIL"; exit 1; }
