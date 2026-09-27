#!/usr/bin/env bash
# Полный набор экспериментов аудита ROS 2 E2E (docs/audit/ROS2_E2E.md).
# Каждый эксперимент — отдельный контейнер (tools/ros_e2e.sh), результаты в
# out/ros_e2e/<tag>/. Запуск из корня репозитория (Git Bash / Linux):
#   bash tools/dev/ros_e2e_suite.sh            # всё подряд (~35 мин)
#   bash tools/dev/ros_e2e_suite.sh exp1 exp4b # выборочно
cd "$(dirname "$0")/../.." || exit 1
E=tools/ros_e2e.sh
run() { local id=$1; shift; if [ $# -gt 0 ] && { [ ${#SEL[@]} -eq 0 ] || [[ " ${SEL[*]} " == *" $id "* ]]; }; then
  echo "===== $id: $*"; bash $E "$@" | tail -8; fi; }
SEL=("$@")
V="/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd"
B="/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity"

# 1. короткий прогон с GNSS, реальное время
run exp1  --bag 30618_af7496f0 --rate 1.0 --tag exp1_af7496f0_r1
# 2. длинный прогон 22 мин, x5: успевает ли нода, рост памяти
run exp2  --bag 30618_0e41eac3 --rate 5 --tag exp2_0e41eac3_r5
# 3. прогоны без GNSS
run exp3a --bag 30618_e151d6e4 --tag exp3_nognss_e151d6e4
run exp3b --bag 30618_74559c73 --tag exp3_nognss_74559c73
# 4. несколько bag подряд в одну ноду
run exp4a --bag 30618_bab2fe58 --bag 30618_98161270 --tag exp4a_seq_backward
run exp4b --bag 30618_98161270 --bag 30618_45ff6f99 --bag 30618_bab2fe58 --tag exp4b_seq_forward_small
run exp4c --bag 30618_45ff6f99 --bag 30639_0ab96c59 --mem 2g --timeout 90 --tag exp4c_seq_forward_15days
run exp4d --bag 30618_45ff6f99 --loop 45 --tag exp4d_loop
# 5. нода запущена через 30 с после начала bag
run exp5  --bag 30618_af7496f0 --late 30 --tag exp5_late30
# 6. только /vehicle/*; только тележки без ручки
run exp6a --bag 30618_af7496f0 --topics "$V" --tag exp6a_vehicle_only
run exp6b --bag 30618_af7496f0 --topics "$B" --tag exp6b_bogies_only
# 7. остановка входов: пауза плеера 5 с
run exp7  --bag 30618_bab2fe58 --pause 20:5 --tag exp7_pause5
# 8. битые данные
run exp8a --bag 30618_bab2fe58 --inject "zero_stamp_first" --mem 2g --timeout 70 --tag exp8a_zero_stamp_first
run exp8b --bag 30618_bab2fe58 --inject "future_stamp --at 15 --offset 100000" --mem 2g --timeout 70 --tag exp8b_future_stamp
run exp8c --bag 30618_bab2fe58 --inject "nan --at 15 --dur 3" --tag exp8c_nan
run exp8d --bag 30618_bab2fe58 --inject "zero_stamp_mid --at 15 --value 300" --tag exp8d_zero_stamp_mid
