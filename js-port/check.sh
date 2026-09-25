#!/usr/bin/env bash
# Сверка JS-порта (est.js) с текущим Python-ядром пакета одной командой — обязательный шаг
# интеграции после любых правок estimator_core.py (поток calib и др.).
#   bash js-port/check.sh            (Git Bash / Linux, из корня репозитория)
# 1) record.py (15 сценариев) и record_bag.py (запись 30618_e9a34502, лист вагона; варианты
#    clean, both_zero, dropout) в vectra/tram:dev; 2) compare.js / compare_bag.js в Node
#    (vectra/tram:sim). Код выхода 0 — порт совпадает с ядром ≤ 1e-6, режимы совпали и
#    отпечаток ядра равен TramEst.PORT.core_sha1; иначе 1 (что делать — печатается).
# Переменные: DATA_DIR (<repo>/data), CACHE_DIR (<repo>/analysis/cache).
set -uo pipefail
export MSYS_NO_PATHCONV=1
ROOT="$(cd "$(dirname "$0")/.." && (pwd -W 2>/dev/null || pwd))"
DATA_DIR="${DATA_DIR:-$ROOT/data}"
CACHE_DIR="${CACHE_DIR:-$ROOT/analysis/cache}"
PY=(docker run --rm --cpus 2 -v "$ROOT:/repo" -v "$DATA_DIR:/repo/data:ro" -v "$CACHE_DIR:/repo/analysis/cache:ro"
    -w /repo/js-port vectra/tram:dev)
JS=(docker run --rm --network none -v "$ROOT/js-port:/p" -w /p vectra/tram:sim)
rc=0
"${PY[@]}" python3 record.py || exit 2
"${JS[@]}" node compare.js | tail -n 3 || rc=1
for v in clean both_zero dropout; do
  "${PY[@]}" python3 record_bag.py 30618_e9a34502 "$v" || exit 2
  "${JS[@]}" node compare_bag.js || rc=1
done
[ $rc = 0 ] && echo "js-port/check: OK — порт совпадает с ядром" || echo "js-port/check: РАСХОЖДЕНИЕ — см. выше"
exit $rc
