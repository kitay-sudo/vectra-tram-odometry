#!/bin/bash
# Полный пересчёт листов вагона по фиксированному разбиению tools/split.json.
#
#   eval — лист ОЦЕНКИ: только split train (train_unique). По нему считаются
#          все наши числа на holdout_scored. -> config/eval/tram_calibration.json
#   jury — лист ЖЮРИ: все уникальные записи. Уходит в пакет.
#          -> config/tram_calibration.json
#
# Затем tools/gen_params.py делает из калибровок листы ноды (config/tram.yaml,
# config/eval/tram.yaml) и таблицу MODEL.md. Запуск в образе vectra/tram:dev из
# корня репозитория: bash analysis/calib_all.sh [eval|jury]  (без аргумента — оба).
# Время: около часа на 8 ядрах (перебор calib_tune.py eval — 9 прогонов связки
# по 61 обучающей записи).
set -euo pipefail
cd "$(dirname "$0")"
W=${WORKERS:-8}
for w in ${1:-eval jury}; do
  python3 calib_drive.py "$w"               # таблица привода A(u,v), delay, tau
  python3 calib_sheet.py "$w"               # масштаб, шум, крип (регрессия) -> лист
  python3 calib_tune.py "$w" --workers "$W" # q_v, крип, адаптация (eval: перебор)
  python3 calib_sheet.py "$w"
  python3 calib_sigma.py "$w" --workers "$W" # выходная σ скорости и положения
  python3 calib_sheet.py "$w"
done
cd ../ros2_ws/src/tram_state_estimator && python3 tools/gen_params.py
