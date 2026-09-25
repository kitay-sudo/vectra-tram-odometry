#!/usr/bin/env bash
# «Живой ROS 2» для демо: нода пакета + rosbridge_websocket + ros2 bag play --loop
# в ОДНОМ контейнере (DDS не выходит наружу, ROS_LOCALHOST_ONLY=1).
# Запускается внутри образа с ROS 2 Humble и rosbridge_server (vectra/tram:dev):
#
#   docker run --rm -p 127.0.0.1:9090:9090 -v <repo>:/repo:ro -v <data>:/data:ro \
#     -e BAG=30618_e9a34502 vectra/tram:dev bash /repo/simulator/deploy/live_demo.sh
#
# Страница: simulator/index.html?mode=live&ros=ws://localhost:9090
# На сервере с доменом — compose.demo.yml (страница и /ros за одним адресом, wss).
#
# Переменные:
#   BAG      прогон из $DATA (по умолчанию отложенный 30618_e9a34502)
#   DATA     каталог прогонов rosbag2 (/data)
#   PORT     порт rosbridge (9090); ADDRESS — адрес прослушивания (0.0.0.0)
#   RATE     скорость проигрывания (1.0)
#   BUILD    1 — собрать пакеты из /repo/ros2_ws/src во временный ws (по умолчанию);
#            0 — взять сборку образа /ws
#   THROTTLE не используется нодой: троттлинг задаёт страница в подписке (10 Гц)
#   ROSBRIDGE_OPEN  1 — мост без ограничений (отладка). По умолчанию мост ТОЛЬКО ДЛЯ
#            ЧТЕНИЯ: подписка лишь на топики страницы (/result/*, /tram/*, /vehicle/*,
#            /sensing/gnss/master/*), публикация запрещена (иначе любой с домена мог бы
#            подать ноде ложные /vehicle/* или GNSS), вызовы сервисов — только /rosapi/*
#            (без параметров: params_glob "[]"), set_parameters ноды недоступен.
#            Параметры — файлом (в launch-аргументах строка "['...']" становится списком,
#            и rosbridge 2.0 падает: InvalidParameterTypeException).
#   SUB_GLOB список подписок моста (по умолчанию — топики страницы, см. выше)
# (без set -u: setup.bash ROS обращается к неустановленным переменным)
set -eo pipefail
BAG="${BAG:-30618_e9a34502}"
DATA="${DATA:-/data}"
PORT="${PORT:-9090}"
ADDRESS="${ADDRESS:-0.0.0.0}"
RATE="${RATE:-1.0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
source /opt/ros/humble/setup.bash
if [ "${BUILD:-1}" = 1 ]; then
  mkdir -p /tmp/ws/src
  cp -r /repo/ros2_ws/src/. /tmp/ws/src/
  (cd /tmp/ws && colcon build --event-handlers console_direct- >/tmp/build.log 2>&1) \
    || { cat /tmp/build.log; exit 1; }
  source /tmp/ws/install/setup.bash
else
  source /ws/install/setup.bash
fi
[ -d "$DATA/$BAG" ] || { echo "нет прогона $DATA/$BAG"; exit 1; }

pids=()
cleanup() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

ros2 launch tram_state_estimator tram.launch.py >/tmp/node.log 2>&1 &
pids+=($!)
if [ "${ROSBRIDGE_OPEN:-0}" = 1 ]; then
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$PORT" address:="$ADDRESS" \
    >/tmp/rosbridge.log 2>&1 &
  pids+=($!)
  MODE="без ограничений (ROSBRIDGE_OPEN=1)"
else
  SUB_GLOB="${SUB_GLOB:-['/result/*', '/tram/*', '/vehicle/*', '/sensing/gnss/master/*']}"
  cat >/tmp/rosbridge_ro.yaml <<YAML
rosbridge_websocket:
  ros__parameters:
    port: $PORT
    address: "$ADDRESS"
    topics_sub_glob: "$SUB_GLOB"
    topics_pub_glob: "[]"
    services_glob: "[]"
rosapi:
  ros__parameters:
    topics_sub_glob: "$SUB_GLOB"
    topics_pub_glob: "[]"
    services_glob: "[]"
    params_glob: "[]"
YAML
  ros2 run rosbridge_server rosbridge_websocket --ros-args -r __node:=rosbridge_websocket \
    --params-file /tmp/rosbridge_ro.yaml >/tmp/rosbridge.log 2>&1 &
  pids+=($!)
  ros2 run rosapi rosapi_node --ros-args -r __node:=rosapi --params-file /tmp/rosbridge_ro.yaml \
    >/tmp/rosapi.log 2>&1 &
  pids+=($!)
  MODE="только чтение: подписка $SUB_GLOB, публикация и сервисы ноды закрыты"
fi
sleep 3
echo "rosbridge ws://$ADDRESS:$PORT ($MODE), нода tram_state_estimator, прогон $BAG по кругу (x$RATE)"
# Каждый круг — новое проигрывание: метки времени идут назад, нода переходит
# на новую выставку (сброс по разрыву времени — WP4 потока robust).
while true; do
  ros2 bag play -r "$RATE" "$DATA/$BAG" >/tmp/play.log 2>&1 || true
  sleep 1
done
