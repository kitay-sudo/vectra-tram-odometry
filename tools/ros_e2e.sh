#!/usr/bin/env bash
# ros_e2e.sh — воспроизводимый end-to-end прогон ноды tram_estimator в ОДНОМ
# контейнере: нода (ros2 launch) + ros2 bag play + tools/ros_probe.py.
#
# С хоста (Git Bash / Linux), из корня репозитория:
#   tools/ros_e2e.sh --bag 30618_af7496f0                       # эксп. 1
#   tools/ros_e2e.sh --bag 30618_0e41eac3 --rate 5 --tag long   # эксп. 2
#   tools/ros_e2e.sh --bag A --bag B --tag seq                   # подряд в одну ноду
#   tools/ros_e2e.sh --bag X --loop 45                           # --loop 45 с
#   tools/ros_e2e.sh --bag X --late 30                           # нода через 30 с после bag
#   tools/ros_e2e.sh --bag X --topics "/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity"
#   tools/ros_e2e.sh --bag X --pause 60:5                        # пауза плеера на 5 с на 60-й с
#   tools/ros_e2e.sh --bag X --inject "zero_stamp_first"         # tools/audit/ros_inject.py
# Опции:
#   --bag ID        прогон из data/ (можно несколько — подряд в одну ноду)
#   --rate R        скорость ros2 bag play (1.0)
#   --topics "..."  проигрывать только эти топики
#   --late S        запустить ноду через S с после старта проигрывания
#   --loop S        проигрывать первый bag с --loop S секунд, затем остановить
#   --pause AT:FOR  пауза плеера (сервис /rosbag2_player/pause) на AT-й с на FOR с
#   --gap S         пауза между bag при последовательном проигрывании (2)
#   --start-offset S  начать проигрывание bag с S-й секунды
#   --inject SPEC   параллельно запустить tools/audit/ros_inject.py SPEC
#   --tag NAME      каталог результатов out/ros_e2e/NAME
#   --mem LIMIT     ограничение памяти контейнера (docker --memory), напр. 2g
#   --cpus N        ограничение CPU контейнера (docker --cpus), напр. 2
#   --timeout S     жёсткий лимит на одно проигрывание bag (сек, 0 — нет)
#   --no-shutdown-test  не проверять останов ноды по SIGINT
#   --build         собрать пакеты из /repo/ros2_ws/src во временный ws
#                   (иначе используется сборка образа /ws, если исходники совпадают)
# Переменные: IMAGE (vectra/tram:dev), DATA_DIR (<repo>/data).
# Результат: out/ros_e2e/<tag>/{summary.json,raw.npz,node.log,bag.log,probe.log,run.log}
# (без set -u: setup.bash ROS обращается к неустановленным переменным)

if [ ! -d /opt/ros/humble ]; then
  # ---------------- хост: запускаем себя внутри контейнера ----------------
  here="$(cd "$(dirname "$0")/.." && (pwd -W 2>/dev/null || pwd))"
  IMAGE="${IMAGE:-vectra/tram:dev}"
  DATA_DIR="${DATA_DIR:-$here/data}"
  tag="e2e"; mem=""; cpus=""
  args=("$@")
  for ((i = 0; i < ${#args[@]}; i++)); do
    case "${args[$i]}" in
      --tag) tag="${args[$((i + 1))]}" ;;
      --mem) mem="${args[$((i + 1))]}" ;;
      --cpus) cpus="${args[$((i + 1))]}" ;;
    esac
  done
  mkdir -p "$here/out/ros_e2e/$tag"
  extra=()
  [ -n "$mem" ] && extra+=(--memory "$mem" --memory-swap "$mem")
  [ -n "$cpus" ] && extra+=(--cpus "$cpus")
  name="vectra-e2e-${tag//[^a-zA-Z0-9_.-]/_}-$RANDOM"
  export MSYS_NO_PATHCONV=1
  exec docker run --rm --name "$name" "${extra[@]}" \
    -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$((RANDOM % 90 + 110))" \
    -v "$here:/repo:ro" -v "$here/out/ros_e2e/$tag:/repo/out/ros_e2e/$tag" \
    -v "$DATA_DIR:/data:ro" \
    "$IMAGE" bash /repo/tools/ros_e2e.sh --inside "$@"
fi

# ---------------- внутри контейнера ----------------
[ "${1:-}" = "--inside" ] && shift
BAGS=(); RATE=1.0; TOPICS=""; LATE=0; LOOP=0; PAUSE=""; GAP=2; TAG=e2e; OFFSET=""
INJECT=""; TIMEOUT=0; SHUT=1; BUILD=0
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAGS+=("$2"); shift ;;
    --rate) RATE="$2"; shift ;;
    --topics) TOPICS="$2"; shift ;;
    --late) LATE="$2"; shift ;;
    --loop) LOOP="$2"; shift ;;
    --pause) PAUSE="$2"; shift ;;
    --gap) GAP="$2"; shift ;;
    --start-offset) OFFSET="$2"; shift ;;
    --inject) INJECT="$2"; shift ;;
    --tag) TAG="$2"; shift ;;
    --timeout) TIMEOUT="$2"; shift ;;
    --mem|--cpus) shift ;;
    --no-shutdown-test) SHUT=0 ;;
    --build) BUILD=1 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
[ ${#BAGS[@]} -eq 0 ] && BAGS=(30618_af7496f0)

OUT=/repo/out/ros_e2e/$TAG
mkdir -p "$OUT"
exec > >(tee "$OUT/run.log") 2>&1
ts() { date +%H:%M:%S.%3N; }
log() { echo "[$(ts)] $*"; }

source /opt/ros/humble/setup.bash
if [ $BUILD -eq 0 ] && diff -r -q -x __pycache__ /ws/src /repo/ros2_ws/src >/dev/null 2>&1; then
  source /ws/install/setup.bash
  log "код ноды: сборка образа /ws (исходники совпадают с /repo/ros2_ws/src)"
else
  log "код ноды: собираю /repo/ros2_ws/src в /tmp/ws"
  mkdir -p /tmp/ws && cd /tmp/ws || exit 1
  colcon build --base-paths /repo/ros2_ws/src --build-base /tmp/ws/build \
    --install-base /tmp/ws/install --event-handlers console_direct- >"$OUT/build.log" 2>&1 \
    || { log "colcon build FAILED (см. build.log)"; exit 1; }
  source /tmp/ws/install/setup.bash
  cd / || exit 1
fi
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
# Job control: без него неинтерактивный bash запускает фоновые процессы с
# SIGINT=SIG_IGN, и ros2 launch не реагирует на SIGINT (проверено). С set -m у
# каждого фонового процесса своя группа — SIGINT группе = Ctrl+C в терминале.
set -m
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-} bags=${BAGS[*]} rate=$RATE late=$LATE loop=$LOOP pause=$PAUSE topics='${TOPICS}' inject='${INJECT}'"
log "CPU: $(nproc) ядер в контейнере; параллельно могут работать другие контейнеры — тайминги предварительные"

python3 /repo/tools/ros_probe.py --out "$OUT/summary.json" --npz "$OUT/raw.npz" \
  --rate "$RATE" --tag "$TAG" >"$OUT/probe.log" 2>&1 &
PROBE=$!

NODE=""
start_node() {
  ros2 launch tram_state_estimator tram.launch.py >"$OUT/node.log" 2>&1 &
  NODE=$!
  for _ in $(seq 1 150); do
    grep -q "оценщик запущен" "$OUT/node.log" 2>/dev/null && break
    sleep 0.2
  done
  log "нода: $(grep -m1 'оценщик запущен' "$OUT/node.log" || echo 'НЕТ строки запуска')"
}

[ "$LATE" = "0" ] && start_node && sleep 1
sleep 1

INJ=""
if [ -n "$INJECT" ]; then
  python3 /repo/tools/audit/ros_inject.py $INJECT >"$OUT/inject.log" 2>&1 &
  INJ=$!
  sleep 1
fi

first=1
for B in "${BAGS[@]}"; do
  args=(/data/"$B" -r "$RATE" --disable-keyboard-controls)
  [ -n "$TOPICS" ] && args+=(--topics $TOPICS)
  [ -n "$OFFSET" ] && [ $first -eq 1 ] && args+=(--start-offset "$OFFSET")
  [ "$LOOP" != "0" ] && [ $first -eq 1 ] && args+=(--loop)
  log "play $B: ros2 bag play ${args[*]}"
  echo "=== $B ===" >>"$OUT/bag.log"
  ros2 bag play "${args[@]}" >>"$OUT/bag.log" 2>&1 &
  PLAY=$!
  t0=$(date +%s)
  if [ -n "$PAUSE" ] && [ $first -eq 1 ]; then
    at=${PAUSE%%:*}; for_=${PAUSE##*:}
    ( sleep "$at"; log "pause плеера на ${for_} с"
      ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause "{}" >/dev/null 2>&1
      log "pause вызван"; sleep "$for_"
      ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}" >/dev/null 2>&1
      log "resume вызван" ) &
  fi
  if [ "$LATE" != "0" ] && [ $first -eq 1 ]; then
    sleep "$LATE"; log "поздний старт ноды (+${LATE} с)"; start_node
  fi
  if [ "$LOOP" != "0" ] && [ $first -eq 1 ]; then
    sleep "$LOOP"; log "стоп --loop через ${LOOP} с"; kill -INT $PLAY 2>/dev/null
  fi
  while kill -0 $PLAY 2>/dev/null; do
    sleep 1
    if [ "$TIMEOUT" != "0" ] && [ $(( $(date +%s) - t0 )) -ge "$TIMEOUT" ]; then
      log "timeout ${TIMEOUT} с — останавливаю плеер"; kill -INT $PLAY; sleep 2; kill -9 $PLAY 2>/dev/null
    fi
  done
  wait $PLAY 2>/dev/null
  log "play $B завершён ($(( $(date +%s) - t0 )) с)"
  first=0
  sleep "$GAP"
done
[ -n "$INJ" ] && kill -INT $INJ 2>/dev/null
sleep 3

# состояние ноды после проигрывания
if [ -n "$NODE" ]; then
  NPID=$(pgrep -f "lib/tram_state_estimator/tram_estimator" | head -1)
  if [ -n "$NPID" ]; then
    log "нода жива (pid $NPID): $(ps -o %cpu=,rss=,etime= -p "$NPID" | awk '{printf "cpu=%s%% rss=%.0fMB etime=%s", $1, $2/1024, $3}')"
  else
    log "ПРОЦЕСС НОДЫ НЕ НАЙДЕН — упала?"
  fi
fi

kill -INT $PROBE 2>/dev/null
wait $PROBE 2>/dev/null

if [ $SHUT -eq 1 ] && [ -n "$NODE" ]; then
  log "останов: SIGINT группе процессов ros2 launch (как Ctrl+C), pgid $NODE"
  t0=$(date +%s.%N)
  kill -INT -- -$NODE 2>/dev/null
  for _ in $(seq 1 150); do kill -0 $NODE 2>/dev/null || break; sleep 0.1; done
  if kill -0 $NODE 2>/dev/null; then
    log "launch не завершился за 15 с — SIGKILL"; kill -9 $NODE; pkill -9 -f tram_estimator
  fi
  wait $NODE 2>/dev/null; rc=$?
  log "launch завершился: код $rc за $(awk "BEGIN{printf \"%.2f\", $(date +%s.%N) - $t0}") с"
  if grep -n -E "Traceback|Error|exception|died|exit code" "$OUT/node.log" >/dev/null; then
    log "в node.log есть ошибки/трассировки:"; grep -n -E -A3 "Traceback|Error|exception|died|exit code" "$OUT/node.log" | tail -30
  else
    log "node.log без трассировок"
  fi
elif [ -n "$NODE" ]; then
  kill -INT $NODE 2>/dev/null; sleep 3; kill -9 $NODE 2>/dev/null
fi

log "---------------- итог ----------------"
cat "$OUT/probe.log"
log "полная сводка: out/ros_e2e/$TAG/summary.json"
