#!/usr/bin/env bash
# Повтор проверки организаторов (архив check-code): наш пакет и их нода
# hackathon_solution_checker в одной сборке, их запись проигрывается в реальном времени,
# итог - RMSE и максимум ошибки скорости и положения против /localization/kinematic_state.
#
# Запуск с хоста (Git Bash или Linux), из корня репозитория:
#   tools/judge_check.sh --check <папка check-code>
# Параметры:
#   --check DIR   распакованный архив организаторов (Dockerfile, src/, bags/)
#   --bag DIR     запись rosbag2 (по умолчанию первая папка в DIR/bags)
#   --rate R      скорость ros2 bag play (1.0; жюри проигрывает в реальном времени)
#   --msgs M      ours - наш tram_vehicle_msgs (оба типа), theirs - их пакет (только
#                 VelocitySensor): проверка работы ноды без типа ручки
#   --out DIR     папка логов (out/judge_check/<msgs>)
# Коды выхода: 0 готово, 1 сборка или прогон не удались, 2 неверные параметры.

if [ ! -d /opt/ros/humble ]; then
  # ---------------- хост: запуск внутри чистого ros:humble-ros-base ----------------
  here="$(cd "$(dirname "$0")/.." && (pwd -W 2>/dev/null || pwd))"
  check=""; bag=""; rate="1.0"; msgs="ours"; out=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --check) check="$2"; shift 2 ;;
      --bag) bag="$2"; shift 2 ;;
      --rate) rate="$2"; shift 2 ;;
      --msgs) msgs="$2"; shift 2 ;;
      --out) out="$2"; shift 2 ;;
      -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
      *) echo "Ошибка: неизвестный параметр $1; справка: tools/judge_check.sh --help"; exit 2 ;;
    esac
  done
  if [ -z "$check" ] || [ ! -d "$check/src/checker_ros" ]; then
    echo "Ошибка: не найдена папка check-code с src/checker_ros; укажите --check <папка>"
    exit 2
  fi
  if [ -z "$bag" ]; then bag="$(ls -d "$check"/bags/*/ 2>/dev/null | head -1)"; fi
  if [ -z "$bag" ] || [ ! -f "$bag/metadata.yaml" ]; then
    echo "Ошибка: не найдена запись rosbag2; укажите --bag <папка>"
    exit 2
  fi
  case "$msgs" in ours|theirs) ;; *) echo "Ошибка: --msgs ours или theirs"; exit 2 ;; esac
  out="${out:-$here/out/judge_check/$msgs}"
  mkdir -p "$out"
  # свой домен DDS: иначе ноды в соседних контейнерах на том же хосте видят чужие топики
  domain="${ROS_DOMAIN_ID:-$(( (RANDOM % 150) + 50 ))}"
  base="ros:humble-ros-base@sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138"
  MSYS_NO_PATHCONV=1 docker run --rm --cpus 2 --memory 512m \
    -v "$here:/ours:ro" -v "$(cd "$check" && (pwd -W 2>/dev/null || pwd)):/check:ro" \
    -v "$(cd "$bag" && (pwd -W 2>/dev/null || pwd)):/bag:ro" \
    -v "$(cd "$out" && (pwd -W 2>/dev/null || pwd)):/out" \
    -e ROS_DOMAIN_ID="$domain" -e RATE="$rate" -e MSGS="$msgs" "$base" bash /ours/tools/judge_check.sh
  exit $?
fi

# ---------------- контейнер ----------------
source /opt/ros/humble/setup.bash
echo "Проверка: скрипт организаторов hackathon_solution_checker"
echo "Запись: $(basename "$(dirname /bag/metadata.yaml)") ($(grep -m1 -A1 '^  duration:' /bag/metadata.yaml | awk '/nanoseconds/{printf "%.0f с", $2/1e9}'))"
echo "Пакет сообщений: $([ "$MSGS" = ours ] && echo 'наш tram_vehicle_msgs' || echo 'из check-code, только VelocitySensor')"
echo "Ограничения: 2 ядра, 512 МБ, скорость проигрывания $RATE, домен ROS $ROS_DOMAIN_ID"
mkdir -p /ws/src
cp -r /ours/ros2_ws/src/tram_msgs /ours/ros2_ws/src/tram_state_estimator /ws/src/
if [ "$MSGS" = ours ]; then
  cp -r /ours/ros2_ws/src/tram_vehicle_msgs /ws/src/
else
  cp -r /check/src/tram_vehicle_msgs /ws/src/
fi
cp -r /check/src/checker_ros /ws/src/
echo "Этап: сборка colcon"
cd /ws
if ! colcon build --event-handlers console_direct- > /out/build.log 2>&1; then
  echo "Ошибка: colcon build не прошёл; лог /out/build.log"
  exit 1
fi
source /ws/install/setup.bash
echo "Этап: запуск ноды, проверяющей ноды и записи"
ros2 launch tram_state_estimator tram.launch.py > /out/node.log 2>&1 &
node=$!
ros2 run hackathon_solution_checker metrics > /out/metrics.log 2>&1 &
metrics=$!
sleep 6
ros2 bag play /bag -d 3 -r "$RATE" > /out/play.log 2>&1
sleep 3
kill -INT "$metrics"; wait "$metrics" 2>/dev/null
kill -INT "$node"; wait "$node" 2>/dev/null
python3 - <<'EOF'
import re, sys
text = open("/out/metrics.log", encoding="utf-8", errors="replace").read()
vel = re.findall(r"velocity: RMSE=([\d.naninf]+), max=([\d.naninf]+), n=(\d+)", text)
pos = re.findall(r"distance: RMSE=([\d.naninf]+), max=([\d.naninf]+), n=(\d+)", text)
def ru(v, d):
    return f"{float(v):.{d}f}".replace(".", ",")
def cnt(n):
    return f"{int(n):,}".replace(",", " ")
if not vel or not pos:
    print("Ошибка: проверяющая нода не выдала итог; лог /out/metrics.log")
    sys.exit(1)
v, p = vel[-1], pos[-1]
print(f"Скорость, RMSE: {ru(v[0], 3)} м/с")
print(f"Скорость, максимум: {ru(v[1], 3)} м/с")
print(f"Положение 3D, RMSE: {ru(p[0], 2)} м")
print(f"Положение 3D, максимум: {ru(p[1], 2)} м")
last = [l for l in text.splitlines() if "Position metrics" in l]
if last:
    for ax, r, m in re.findall(r"([xyz]): RMSE=([\d.naninf]+), max=([\d.naninf]+)", last[-1]):
        print(f"Положение {ax}, RMSE: {ru(r, 2)} м, максимум {ru(m, 2)} м")
print(f"Пар скорость: {cnt(v[2])}")
print(f"Пар положение: {cnt(p[2])}")
print("Итог: проверка организаторов выполнена, логи в out/judge_check")
EOF
