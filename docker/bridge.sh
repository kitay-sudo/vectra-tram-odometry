#!/usr/bin/env bash
# rosbridge v2 для страницы симулятора (сервис bridge в docker-compose.yml).
#
# По умолчанию мост ТОЛЬКО ДЛЯ ЧТЕНИЯ: страница на публичном домене открыта
# всем, а открытый rosbridge позволил бы любому опубликовать ноде ложные
# /vehicle/* или GNSS и вызвать set_parameters. Разрешены подписки только на
# топики страницы, публикация запрещена, сервисы — только /rosapi/* без
# параметров. Так же делает simulator/deploy/live_demo.sh (проверено на
# настоящем мосте: подделка 777 км/ч — 0 из 20 сообщений).
# Параметры задаются файлом: строка "['...']" в аргументах launch становится
# списком, и rosbridge 2.0 падает (InvalidParameterTypeException).
#
#   ROSBRIDGE_OPEN=1   мост без ограничений (отладка на своей машине)
#   BRIDGE_SUB_GLOB    список подписок (по умолчанию — топики страницы)
set -eo pipefail
source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
PORT="${PORT:-9090}"
ADDRESS="${ADDRESS:-0.0.0.0}"
if [ "${ROSBRIDGE_OPEN:-0}" = 1 ]; then
  echo "[bridge] rosbridge без ограничений (ROSBRIDGE_OPEN=1), порт $PORT"
  exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$PORT" address:="$ADDRESS"
fi
SUB_GLOB="${BRIDGE_SUB_GLOB:-['/result/*', '/tram/*', '/vehicle/*', '/sensing/gnss/master/*', '/sensing/gnss/rover/fix']}"
CFG=/tmp/rosbridge_ro.yaml
cat >"$CFG" <<YAML
rosbridge_websocket:
  ros__parameters:
    port: $PORT
    address: "$ADDRESS"
    topics_sub_glob: "$SUB_GLOB"
    topics_pub_glob: "[]"
    services_glob: "[]"
    # значения по умолчанию будущих версий rosbridge (Jazzy); с прежними мост
    # пишет при старте три WARN. Страница сервисов и действий не вызывает.
    default_call_service_timeout: 5.0
    call_services_in_new_thread: true
    send_action_goals_in_new_thread: true
rosapi:
  ros__parameters:
    topics_sub_glob: "$SUB_GLOB"
    topics_pub_glob: "[]"
    services_glob: "[]"
    params_glob: "[]"
YAML
echo "[bridge] rosbridge только для чтения: подписки $SUB_GLOB, публикация запрещена, порт $PORT"
pids=()
cleanup() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM
ros2 run rosapi rosapi_node --ros-args -r __node:=rosapi --params-file "$CFG" &
pids+=($!)
ros2 run rosbridge_server rosbridge_websocket --ros-args -r __node:=rosbridge_websocket \
  --params-file "$CFG" &
pids+=($!)
wait -n "${pids[@]}"
