#!/usr/bin/env bash
# ros_smoke.sh — ROS-смоук ноды (WP9): ros2 launch + 30 с bag + проба в ОДНОМ
# контейнере; проверка частоты, задержки, NaN, frame_id, живости и останова.
#
# С хоста (Git Bash / Linux), из корня репозитория:
#   tools/ros_smoke.sh                           # фикстура теста -> bag (данные не нужны)
#   tools/ros_smoke.sh --bag 30618_98161270      # 30 с настоящего прогона из data/
#   tools/ros_smoke.sh --bag 30618_b95ca60a --seconds 60 --build
# Внутри ROS-окружения (CI, контейнер) — то же самое, без docker:
#   bash tools/ros_smoke.sh --ws /tmp/ws         # workspace уже собран
# Опции:
#   --bag ID      прогон из DATA_DIR (по умолчанию — bag из фикстуры e2e-теста)
#   --seconds S   сколько секунд проигрывать (30; 0 — весь bag, тогда проверяется
#                 и то, что проба получила все входы bag)
#   --build       собрать /repo/ros2_ws/src во временный workspace (иначе /ws образа,
#                 если исходники совпадают, или --ws)
#   --ws DIR      готовый workspace (DIR/install/setup.bash)
#   --tag NAME    каталог результатов out/smoke/NAME (smoke)
#   --gnss-window S  (только фикстура) GNSS в bag лишь первые S с по header.stamp —
#                 сценарий жюри «GNSS только в начале» (TODO WP9, риск 16);
#                 тогда обязательна проверка: выставка прошла, ср. 3D < 10 м
#   --repeat N    повторить N раз (новая нода каждый раз); PASS, только если все
#   пример: tools/ros_smoke.sh --gnss-window 3 --repeat 10 --tag gnss3s
# Переменные: IMAGE (vectra/tram:compose), DATA_DIR (<repo>/data).
# Критерии (TODO WP9, tools/smoke_verdict.py): >= 19 Гц по меткам и по стенным
# часам; in2out p99 < 100 мс в установившемся режиме (без первых 2 с —
# стартовый всплеск, риск 15/16); выходов >= 95 % узлов сетки; 0 NaN; frame_id
# map/base_link; нода жива до конца; останов по SIGINT за 15 с (трассировки —
# справочно до WP3). Код выхода 0 — всё PASS.

if [ ! -d /opt/ros/humble ]; then
  # ---------------- хост: запускаем себя в контейнере ----------------
  here="$(cd "$(dirname "$0")/.." && (pwd -W 2>/dev/null || pwd))"
  IMAGE="${IMAGE:-vectra/tram:compose}"
  DATA_DIR="${DATA_DIR:-$here/data}"
  tag="smoke"; args=("$@")
  for ((i = 0; i < ${#args[@]}; i++)); do
    [ "${args[$i]}" = "--tag" ] && tag="${args[$((i + 1))]}"
  done
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "[smoke] образа $IMAGE нет — собираю (docker build -f docker/Dockerfile)"
    docker build -f "$here/docker/Dockerfile" -t "$IMAGE" "$here" || exit 1
  fi
  mkdir -p "$here/out/smoke/$tag"
  export MSYS_NO_PATHCONV=1
  exec docker run --rm --name "vectra-smoke-${tag//[^a-zA-Z0-9_.-]/_}-$RANDOM" --cpus 2 \
    -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$((RANDOM % 100 + 1))" \
    -v "$here:/repo:ro" -v "$here/out/smoke/$tag:/repo/out/smoke/$tag" \
    -v "$DATA_DIR:/data:ro" \
    "$IMAGE" bash /repo/tools/ros_smoke.sh --inside "$@"
fi

# ---------------- внутри ROS-окружения ----------------
[ "${1:-}" = "--inside" ] && shift
REPO="$(cd "$(dirname "$0")/.." && pwd)"
FIXTURE="$REPO/ros2_ws/src/tram_state_estimator/test/data/e2e_30618_b95ca60a_180s.npz"
BAG=""; SECONDS_PLAY=30; BUILD=0; WS=""; TAG=smoke; GNSS_WIN=0; REPEAT=1
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$2"; shift ;;
    --seconds) SECONDS_PLAY="$2"; shift ;;
    --build) BUILD=1 ;;
    --ws) WS="$2"; shift ;;
    --tag) TAG="$2"; shift ;;
    --gnss-window) GNSS_WIN="$2"; shift ;;
    --repeat) REPEAT="$2"; shift ;;
    *) echo "[smoke] неизвестная опция $1"; exit 2 ;;
  esac
  shift
done
OUT="$REPO/out/smoke/$TAG"
mkdir -p "$OUT" 2>/dev/null || { OUT="/tmp/smoke/$TAG"; mkdir -p "$OUT"; }
exec > >(tee "$OUT/run.log") 2>&1
ts() { date +%H:%M:%S.%3N; }
log() { echo "[$(ts)] [smoke] $*"; }

source /opt/ros/humble/setup.bash
if [ -n "$WS" ]; then
  source "$WS/install/setup.bash"; log "workspace: $WS"
elif [ $BUILD -eq 0 ] && [ -f /ws/install/setup.bash ] && \
     diff -r -q -x __pycache__ -x .pytest_cache -x test /ws/src "$REPO/ros2_ws/src" >/dev/null 2>&1; then
  source /ws/install/setup.bash; log "workspace: /ws образа (исходники совпадают с репозиторием)"
else
  log "workspace: собираю $REPO/ros2_ws/src в /tmp/smoke_ws"
  mkdir -p /tmp/smoke_ws
  (cd /tmp/smoke_ws && colcon --log-base /tmp/smoke_ws/log build --base-paths "$REPO/ros2_ws/src" \
     --build-base /tmp/smoke_ws/build --install-base /tmp/smoke_ws/install \
     --event-handlers console_direct- >"$OUT/build.log" 2>&1) \
    || { log "colcon build FAILED (build.log)"; tail -30 "$OUT/build.log"; exit 1; }
  source /tmp/smoke_ws/install/setup.bash
fi
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
set -m    # у фоновых процессов своя группа: SIGINT группе = Ctrl+C

VERDICT_ARGS=()
if [ -z "$BAG" ]; then
  BAGDIR=/tmp/smoke_bag
  python3 "$REPO/tools/fixture_to_bag.py" --out "$BAGDIR" --seconds "$SECONDS_PLAY" \
    --gnss-window "$GNSS_WIN" || { log "не собрать bag из фикстуры"; exit 1; }
  SRC="фикстура e2e (30618_b95ca60a, holdout)"
  VERDICT_ARGS=(--fixture "$FIXTURE" --bag-meta "$BAGDIR/metadata.yaml")
  [ "$GNSS_WIN" != "0" ] && VERDICT_ARGS+=(--need-position) && SRC="$SRC, GNSS только ${GNSS_WIN} с по меткам"
else
  [ "$GNSS_WIN" != "0" ] && { log "--gnss-window работает только с фикстурой"; exit 2; }
  BAGDIR="/data/$BAG"
  [ -f "$BAGDIR/metadata.yaml" ] || { log "нет $BAGDIR (DATA_DIR)"; exit 2; }
  SRC="$BAG"
  [ "$SECONDS_PLAY" = "0" ] && VERDICT_ARGS=(--bag-meta "$BAGDIR/metadata.yaml")
fi
log "bag: $SRC, ${SECONDS_PLAY} с, повторов $REPEAT; ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}; CPU в контейнере: $(nproc)"

run_once() {   # $1 — каталог результатов прогона
  local D="$1" PROBE NODE NPID ALIVE=0 SHUT=1 TRACE t0
  mkdir -p "$D"
  python3 "$REPO/tools/ros_probe.py" --out "$D/summary.json" --npz "$D/raw.npz" \
    --tag "$TAG" >"$D/probe.log" 2>&1 &
  PROBE=$!
  ros2 launch tram_state_estimator tram.launch.py >"$D/node.log" 2>&1 &
  NODE=$!
  python3 "$REPO/tools/ros_wait.py" --subscribers /vehicle/front_bogie_velocity:2 \
    --publishers /result/velocity:1 --timeout 60 || log "нода или проба не подписались за 60 с"
  log "нода: $(grep -m1 -o 'оценщик запущен.*' "$D/node.log" || echo 'НЕТ строки запуска')"
  t0=$(date +%s)
  local TMO=$((${SECONDS_PLAY%.*} + 10))
  [ "$SECONDS_PLAY" = "0" ] && TMO=86400
  timeout -s INT "$TMO" \
    ros2 bag play "$BAGDIR" -d 2 --disable-keyboard-controls >"$D/bag.log" 2>&1
  log "bag проигран за $(( $(date +%s) - t0 )) с"
  sleep 2
  NPID=$(pgrep -f "lib/tram_state_estimator/tram_estimator" | head -1)
  [ -n "$NPID" ] && ALIVE=1 && log "нода жива: $(ps -o %cpu=,rss= -p "$NPID" | awk '{printf "cpu=%s%% rss=%.0f МБ", $1, $2/1024}')"
  kill -INT $PROBE 2>/dev/null; wait $PROBE 2>/dev/null
  kill -INT -- -$NODE 2>/dev/null
  for _ in $(seq 1 150); do kill -0 $NODE 2>/dev/null || break; sleep 0.1; done
  if kill -0 $NODE 2>/dev/null; then
    log "launch не остановился за 15 с — SIGKILL"; kill -9 -- -$NODE 2>/dev/null; SHUT=0
    pkill -9 -f lib/tram_state_estimator/tram_estimator 2>/dev/null
  fi
  wait $NODE 2>/dev/null
  TRACE=$(grep -c -E "Traceback|process has died" "$D/node.log")
  python3 "$REPO/tools/smoke_verdict.py" "$D/summary.json" --raw "$D/raw.npz" \
    --alive "$ALIVE" --shut "$SHUT" --trace "$TRACE" --json "$D/verdict.json" "${VERDICT_ARGS[@]}"
}

rc=0; npass=0; nlost=0
for i in $(seq 1 "$REPEAT"); do
  if [ "$REPEAT" = "1" ]; then D="$OUT"; else D="$OUT/run$i"; log "---- прогон $i из $REPEAT ----"; fi
  run_once "$D"; r=$?
  if [ $r -eq 0 ]; then npass=$((npass + 1)); else rc=1; [ $r -eq 3 ] && nlost=$((nlost + 1)); fi
  [ "$REPEAT" != "1" ] && sleep 1
done
if [ "$REPEAT" != "1" ]; then
  python3 - "$OUT" "$REPEAT" <<'PY'
import json, os, sys
out, n = sys.argv[1], int(sys.argv[2])
miss = 0
print("| прогон | итог | выходов | in2out p99 устан., мс | выставка | ср. 3D, м | потери у пробы |")
print("|---|---|---|---|---|---|---|")
for i in range(1, n + 1):
    f = os.path.join(out, f"run{i}", "verdict.json")
    if not os.path.exists(f):
        print(f"| {i} | нет verdict.json | | | | | |")
        continue
    v = json.load(open(f, encoding="utf-8"))
    p = v.get("position") or {}
    lost = json.dumps(v["harness_loss"]) if v.get("harness_loss") else "—"
    print(f"| {i} | {v.get('verdict')} | {v['outputs']}/{v['expected']} | "
          f"{v['in2out_steady_ms'].get('p99')} | {'да' if p.get('aligned') else ('нет' if p else '—')} | "
          f"{p.get('mean_m', '—')} | {lost} |")
    miss = miss + 1 if p and not p.get("aligned") else miss
if any(os.path.exists(os.path.join(out, f"run{i}", "verdict.json")) for i in range(1, n + 1)):
    # «НЕ ЗАСЧИТАН» — про обвязку; для жюри прогон без выставки — потеря баллов
    print()
    print(f"срывов выставки ноды (положения нет): {miss} из {n}")
PY
  log "ИТОГ ПОВТОРОВ из $REPEAT: PASS $npass, FAIL $((REPEAT - npass - nlost)), НЕ ЗАСЧИТАН (проба потеряла начало bag) $nlost"
fi
log "результаты: $OUT"
exit $rc
