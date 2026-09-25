#!/usr/bin/env bash
# play.sh — сервис player (docker compose): ждёт подписчиков и проигрывает bag.
#
# Переменные (из .env): BAG — каталог прогона в DATA_DIR (смонтирован в /data);
# PLAY_LOOP=1 — по кругу (демо-сервер); PLAY_RATE — скорость (1.0);
# PLAY_DELAY — пауза перед проигрыванием, с (2, как в инструкции жюри);
# WAIT_SUBSCRIBERS — сколько подписчиков /vehicle/front_bogie_velocity ждать
# (нода + проба = 2; не дождались за 60 с — играем всё равно).
set -o pipefail
source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

BAG=${BAG:-}
if [ -z "$BAG" ] || [ ! -f "/data/$BAG/metadata.yaml" ]; then
  echo "[player] bag '${BAG}' не найден в /data (DATA_DIR из .env)."
  echo "[player] задайте BAG=<каталог прогона> и DATA_DIR=<папка с прогонами>; доступные:"
  ls /data 2>/dev/null | head -30 | sed 's/^/[player]   /'
  exit 2
fi

python3 /repo/tools/ros_wait.py \
  --subscribers "/vehicle/front_bogie_velocity:${WAIT_SUBSCRIBERS:-2}" --timeout 60 \
  || echo "[player] подписчиков меньше ${WAIT_SUBSCRIBERS:-2} — проигрываю всё равно"

args=(-d "${PLAY_DELAY:-2}" -r "${PLAY_RATE:-1.0}" --disable-keyboard-controls)
[ "${PLAY_LOOP:-0}" = "1" ] && args+=(--loop)
echo "[player] ros2 bag play /data/$BAG ${args[*]}"
exec ros2 bag play "/data/$BAG" "${args[@]}"
