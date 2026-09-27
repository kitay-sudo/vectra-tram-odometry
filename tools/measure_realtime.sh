#!/usr/bin/env bash
# measure_realtime.sh - замер реального времени ноды (критерий 4 ТЗ):
# полный bag x1, нода в своём контейнере с лимитами ТЗ (--cpus 2 --memory 512m),
# плеер и проба - в соседнем контейнере в тех же сетевом, IPC и PID
# пространствах (как процессы на одной машине; лимиты - только на ноду).
#
# С хоста (Git Bash / Linux), из корня репозитория:
#   tools/measure_realtime.sh --bag 30618_e9a34502                 # 20 мин, отложенный
#   tools/measure_realtime.sh --bag 30618_af7496f0 --tag short      # проверка скрипта
#   tools/measure_realtime.sh --bag 30618_0652866c --rate 1 --cpus 2 --memory 512m --build
# Опции:
#   --bag ID        прогон из DATA_DIR (обязательно)
#   --rate R        скорость проигрывания (1.0)
#   --cpus N        лимит CPU контейнера ноды (2)
#   --memory M      лимит памяти контейнера ноды, без swap (512m)
#   --tag NAME      каталог out/realtime/NAME (по умолчанию <bag>_x<rate>)
#   --build         пересобрать образ из рабочего дерева перед замером
#   --allow-stale   мерить образ, даже если его /ws/src не совпадает с ros2_ws/src
#                   (по умолчанию такой замер не запускается: мерили бы старый код)
#   --stats-every S период опроса docker stats, с (5)
# Переменные: IMAGE (vectra/tram:compose), DATA_DIR (<repo>/data).
# Результат: out/realtime/<tag>/{summary.json,summary.md,raw.npz,node.log,
# docker_stats.csv,harness.log}; копия итога - out/realtime/summary.{json,md}.

set -o pipefail
if [ -d /opt/ros/humble ] && [ "${1:-}" = "--inside" ]; then
  # ---------------- контейнер-обвязка: проба + плеер ----------------
  shift
  BAG="$1"; RATE="$2"; OUT="$3"
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
  ts() { date +%H:%M:%S.%3N; }
  # Без --report-every: промежуточная сводка считается в том же цикле, что принимает
  # сообщения, и за 20 мин растёт до ~0,3 с простоя пробы раз в 120 с - пробе входы и
  # выходы приходят с опозданием, и in2out > 250 мс ложно (замер: паузы приёма
  # 0,08 → 0,31 с ровно каждые 120 с при паузах меток входов ≤ 0,08 с).
  python3 /repo/tools/ros_probe.py --out "$OUT/summary_probe.json" --npz "$OUT/raw.npz" \
    --rate "$RATE" --tag "realtime" --idle 15 >"$OUT/probe.log" 2>&1 &
  PROBE=$!
  python3 /repo/tools/ros_wait.py --subscribers /vehicle/front_bogie_velocity:2 \
    --publishers /result/velocity:1 --timeout 120 || echo "[$(ts)] нода/проба не подписались"
  echo "[$(ts)] play /data/$BAG x$RATE"
  t0=$(date +%s)
  ros2 bag play "/data/$BAG" -d 2 -r "$RATE" --disable-keyboard-controls >"$OUT/bag.log" 2>&1
  echo "[$(ts)] bag проигран за $(( $(date +%s) - t0 )) с; жду пробу (15 с тишины)"
  wait $PROBE
  echo "[$(ts)] проба завершилась"
  exit 0
fi

# ---------------- хост ----------------
here="$(cd "$(dirname "$0")/.." && (pwd -W 2>/dev/null || pwd))"
IMAGE="${IMAGE:-vectra/tram:compose}"
DATA_DIR="${DATA_DIR:-$here/data}"
BAG=""; RATE=1.0; CPUS=2; MEM=512m; TAG=""; BUILD=0; EVERY=5; STALE_OK=0
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$2"; shift ;;
    --rate) RATE="$2"; shift ;;
    --cpus) CPUS="$2"; shift ;;
    --memory) MEM="$2"; shift ;;
    --tag) TAG="$2"; shift ;;
    --build) BUILD=1 ;;
    --allow-stale) STALE_OK=1 ;;
    --stats-every) EVERY="$2"; shift ;;
    *) echo "неизвестная опция $1"; exit 2 ;;
  esac
  shift
done
[ -n "$BAG" ] || { echo "нужен --bag <id>"; exit 2; }
[ -f "$DATA_DIR/$BAG/metadata.yaml" ] || { echo "нет $DATA_DIR/$BAG (DATA_DIR)"; exit 2; }
TAG="${TAG:-${BAG}_x${RATE}}"
OUT="$here/out/realtime/$TAG"
mkdir -p "$OUT"
export MSYS_NO_PATHCONV=1
if [ $BUILD -eq 1 ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[rt] сборка образа $IMAGE из рабочего дерева"
  docker build -q -f "$here/docker/Dockerfile" -t "$IMAGE" "$here" || exit 1
fi
# нода работает из /ws образа: исходники образа (кроме test/) должны совпадать с деревом
if ! docker run --rm -v "$here:/repo:ro" "$IMAGE" diff -r -q -x __pycache__ -x .pytest_cache -x test /ws/src /repo/ros2_ws/src >/dev/null 2>&1; then
  if [ $STALE_OK -eq 1 ]; then
    echo "[rt] ВНИМАНИЕ: /ws/src образа $IMAGE не совпадает с ros2_ws/src - мерится код образа (--allow-stale)"
  else
    echo "[rt] образ $IMAGE собран из других исходников, чем ros2_ws/src рабочего дерева:"
    echo "[rt] замер был бы по старому коду. Добавьте --build (или --allow-stale)."
    exit 2
  fi
fi
name="vectra-rt-${TAG//[^a-zA-Z0-9_.-]/_}-$RANDOM"
dom=$((RANDOM % 100 + 1))    # 1…100: вне 102…232 (эфемерные порты Linux, см. ROS 2 docs)
echo "[rt] образ $IMAGE; нода: --cpus $CPUS --memory $MEM; bag $BAG x$RATE; ROS_DOMAIN_ID=$dom"
echo "[rt] хост: $(docker info --format '{{.NCPU}} CPU, {{.MemTotal}} B, {{.OperatingSystem}} {{.ServerVersion}}')"
echo "[rt] соседние контейнеры (замер предварительный, если они есть):"
docker ps --format '  {{.Names}} ({{.Image}})' | grep -v "$name" | head -20

docker run -d --name "$name-node" --cpus "$CPUS" --memory "$MEM" --memory-swap "$MEM" \
  --ipc shareable -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=$dom \
  -e PYTHONUNBUFFERED=1 -e RCUTILS_LOGGING_BUFFERED_STREAM=0 \
  "$IMAGE" ros2 launch tram_state_estimator tram.launch.py >/dev/null || exit 1

# docker stats ноды: CPU% (100 = 1 ядро) и память контейнера (cgroup)
echo "t_unix,cpu_pct,mem_usage,mem_limit,mem_pct,pids" >"$OUT/docker_stats.csv"
(
  while docker inspect -f '{{.State.Running}}' "$name-node" 2>/dev/null | grep -q true; do
    line=$(docker stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.PIDs}}' "$name-node" 2>/dev/null)
    [ -n "$line" ] && echo "$(date +%s),$(echo "$line" | sed 's| / |,|; s|%||g')" >>"$OUT/docker_stats.csv"
    sleep "$EVERY"
  done
) &
STATS=$!

docker run --rm --name "$name-harness" --network "container:$name-node" \
  --ipc "container:$name-node" --pid "container:$name-node" \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=$dom \
  -v "$here:/repo:ro" -v "$OUT:/out" -v "$DATA_DIR:/data:ro" \
  "$IMAGE" bash /repo/tools/measure_realtime.sh --inside "$BAG" "$RATE" /out 2>&1 | tee "$OUT/harness.log"

OOM=$(docker inspect -f '{{.State.OOMKilled}}' "$name-node" 2>/dev/null)
RUNNING=$(docker inspect -f '{{.State.Running}}' "$name-node" 2>/dev/null)
t0=$(date +%s)
docker stop -s SIGINT -t 15 "$name-node" >/dev/null 2>&1
STOP_S=$(( $(date +%s) - t0 ))
EXIT=$(docker inspect -f '{{.State.ExitCode}}' "$name-node" 2>/dev/null)
docker logs "$name-node" >"$OUT/node.log" 2>&1
docker rm -f "$name-node" >/dev/null 2>&1
kill $STATS 2>/dev/null; wait $STATS 2>/dev/null

docker run --rm -v "$here:/repo:ro" -v "$OUT:/out" -v "$here/out/realtime:/rt" "$IMAGE" \
  python3 /repo/tools/realtime_report.py /out --bag "$BAG" --rate "$RATE" --cpus "$CPUS" \
  --memory "$MEM" --oom "$OOM" --alive "$RUNNING" --stop-s "$STOP_S" --exit-code "$EXIT" \
  --latest /rt
