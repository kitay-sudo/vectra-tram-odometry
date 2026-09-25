#!/usr/bin/env bash
# run_eval.sh — сервис eval (docker compose): офлайн-оценка tools/eval.py.
#
#   docker compose run --rm eval                 # как задокументировано в docs/EVAL.md
#   docker compose run --rm eval --help          # аргументы уходят в tools/eval.py
#
# Репозиторий смонтирован на запись (кэш analysis/cache, результаты out/eval),
# данные — только чтение в /repo/data (DATA_DIR из .env). Сеть не нужна.
source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
cd /repo || exit 1

if [ ! -f tools/eval.py ]; then
  echo "[eval] tools/eval.py в этой версии репозитория нет (пишет поток оценки, WP7)."
  echo "[eval] ПРОПУЩЕНО: оценивать нечем. Сервис готов, запустится, когда файл появится."
  exit 0
fi
if [ -z "$(ls -A /repo/data 2>/dev/null)" ]; then
  echo "[eval] /repo/data пуст: задайте DATA_DIR в .env (папка с прогонами data/<bag_id>/)."
fi
export PYTHONPATH="/repo/ros2_ws/src/tram_state_estimator:${PYTHONPATH}"
echo "[eval] python3 tools/eval.py $*"
exec python3 tools/eval.py "$@"
