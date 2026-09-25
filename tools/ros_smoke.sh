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
#   --seconds S   сколько секунд проигрывать (30)
#   --build       собрать /repo/ros2_ws/src во временный workspace (иначе /ws образа,
#                 если исходники совпадают, или --ws)
#   --ws DIR      готовый workspace (DIR/install/setup.bash)
#   --tag NAME    каталог результатов out/smoke/NAME (smoke)
# Переменные: IMAGE (vectra/tram:compose), DATA_DIR (<repo>/data).
# Критерии (TODO WP9): >= 19 Гц по меткам и по стенным часам; in2out p99 < 100 мс
# в установившемся режиме (без первых 2 с — стартовый всплеск, риск 15/16);
# выходов >= 95 % узлов сетки; 0 NaN; frame_id map/base_link; нода жива до
# конца; останов по SIGINT без трассировок. Код выхода 0 — всё PASS.

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
    -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$((RANDOM % 90 + 110))" \
    -v "$here:/repo:ro" -v "$here/out/smoke/$tag:/repo/out/smoke/$tag" \
    -v "$DATA_DIR:/data:ro" \
    "$IMAGE" bash /repo/tools/ros_smoke.sh --inside "$@"
fi

# ---------------- внутри ROS-окружения ----------------
[ "${1:-}" = "--inside" ] && shift
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG=""; SECONDS_PLAY=30; BUILD=0; WS=""; TAG=smoke
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$2"; shift ;;
    --seconds) SECONDS_PLAY="$2"; shift ;;
    --build) BUILD=1 ;;
    --ws) WS="$2"; shift ;;
    --tag) TAG="$2"; shift ;;
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
     diff -r -q -x __pycache__ /ws/src "$REPO/ros2_ws/src" >/dev/null 2>&1; then
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

if [ -z "$BAG" ]; then
  BAGDIR=/tmp/smoke_bag
  python3 "$REPO/tools/fixture_to_bag.py" --out "$BAGDIR" --seconds "$SECONDS_PLAY" \
    || { log "не собрать bag из фикстуры"; exit 1; }
  SRC="фикстура e2e (30618_b95ca60a, holdout)"
else
  BAGDIR="/data/$BAG"
  [ -f "$BAGDIR/metadata.yaml" ] || { log "нет $BAGDIR (DATA_DIR)"; exit 2; }
  SRC="$BAG"
fi
log "bag: $SRC, ${SECONDS_PLAY} с; ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}; CPU в контейнере: $(nproc)"

python3 "$REPO/tools/ros_probe.py" --out "$OUT/summary.json" --npz "$OUT/raw.npz" \
  --tag "$TAG" >"$OUT/probe.log" 2>&1 &
PROBE=$!
ros2 launch tram_state_estimator tram.launch.py >"$OUT/node.log" 2>&1 &
NODE=$!
python3 "$REPO/tools/ros_wait.py" --subscribers /vehicle/front_bogie_velocity:2 \
  --publishers /result/velocity:1 --timeout 60 || log "нода или проба не подписались за 60 с"
log "нода: $(grep -m1 'оценщик запущен' "$OUT/node.log" || echo 'НЕТ строки запуска')"

t0=$(date +%s)
timeout -s INT $((${SECONDS_PLAY%.*} + 10)) \
  ros2 bag play "$BAGDIR" -d 2 --disable-keyboard-controls >"$OUT/bag.log" 2>&1
log "bag проигран за $(( $(date +%s) - t0 )) с"
sleep 2

NPID=$(pgrep -f "lib/tram_state_estimator/tram_estimator" | head -1)
ALIVE=0
[ -n "$NPID" ] && ALIVE=1 && log "нода жива: $(ps -o %cpu=,rss= -p "$NPID" | awk '{printf "cpu=%s%% rss=%.0f МБ", $1, $2/1024}')"
kill -INT $PROBE 2>/dev/null; wait $PROBE 2>/dev/null

kill -INT -- -$NODE 2>/dev/null
for _ in $(seq 1 150); do kill -0 $NODE 2>/dev/null || break; sleep 0.1; done
if kill -0 $NODE 2>/dev/null; then
  log "launch не остановился за 15 с — SIGKILL"; kill -9 -- -$NODE 2>/dev/null; SHUT=0
else
  SHUT=1
fi
wait $NODE 2>/dev/null
TRACE=$(grep -c -E "Traceback|process has died" "$OUT/node.log")

python3 - "$OUT/summary.json" "$ALIVE" "$SHUT" "$TRACE" <<'EOF'
import json, sys
R = json.load(open(sys.argv[1], encoding="utf-8"))
alive, shut, trace = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
o = R["outputs"]["velocity"]; p = R["outputs"]["position"]; L = R["latency"]
st = L.get("in2out_vehicle_steady_ms", {}); al = L.get("in2out_vehicle_ms", {})
bad = R.get("nonfinite_or_bad", {})
fr = R.get("frame_ids", {})
exp = o.get("expected_by_stamp") or 0
rows = [
    ("выходов /result/velocity от узлов сетки", f"{o.get('count')}/{exp}",
     exp > 0 and o.get("count", 0) >= 0.95 * exp),
    ("/result/position столько же", f"{p.get('count')}", p.get("count") == o.get("count")),
    ("частота по меткам >= 19 Гц", f"{o.get('rate_stamp_hz')}", (o.get("rate_stamp_hz") or 0) >= 19),
    ("частота по стенным часам >= 19 Гц", f"{o.get('rate_wall_hz')}", (o.get("rate_wall_hz") or 0) >= 19),
    ("in2out p99 < 100 мс (без первых 2 с)", f"p50 {st.get('p50')} / p95 {st.get('p95')} / p99 {st.get('p99')} / max {st.get('max')}",
     st.get("n", 0) > 0 and st["p99"] < 100),
    ("0 NaN/inf в выходах", f"{bad}", not any(bad.get(k, 0) for k in ("vel_nonfinite", "odo_nonfinite", "cov_nonfinite"))),
    ("frame_id: map / base_link", f"{list(fr.get('position', {}))} / {list(fr.get('position_child', {}))}",
     set(fr.get("position", {})) == {"map"} and set(fr.get("position_child", {})) == {"base_link"}),
    ("нода жива до конца bag", "да" if alive else "НЕТ", bool(alive)),
    ("останов по SIGINT за 15 с", "да" if shut else "НЕТ", bool(shut)),
]
info = [("in2out p99 с учётом старта (справочно)", f"p99 {al.get('p99')} / max {al.get('max')}"),
        ("трассировки при останове (справочно, WP3)", str(trace)),
        ("CPU ноды, % ядра", str(R.get("node_process", {}).get("cpu_pct_1core"))),
        ("RSS ноды, МБ (первый/последний/макс)", str(R.get("node_process", {}).get("rss_mb_first_last_max"))),
        ("точность по GNSS bag (санити)", json.dumps({k: v for k, v in R.get("accuracy_sanity_vs_bag_gnss", {}).items() if k != "note"}, ensure_ascii=False))]
print("| проверка | значение | итог |\n|---|---|---|")
for name, val, ok in rows:
    print(f"| {name} | {val} | {'PASS' if ok else 'FAIL'} |")
for name, val in info:
    print(f"| {name} | {val} | — |")
ok = all(r[2] for r in rows)
print(f"\n[smoke] ИТОГ: {'PASS' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
EOF
rc=$?
log "результаты: out/smoke/$TAG/{summary.json,run.log,node.log,probe.log}"
exit $rc
