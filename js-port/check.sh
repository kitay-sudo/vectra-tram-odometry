#!/usr/bin/env bash
# Сверка JS-порта с текущим Python-ядром и связкой пакета одной командой — обязательный шаг
# интеграции после любых правок estimator_core.py (или runner.py в части шага ядра).
#   bash js-port/check.sh            (Git Bash / Linux, из корня репозитория)
# 1) record.py: 15 сценариев prototype/scenarios.py, Params по умолчанию -> compare.js;
# 2) record_bag.py: реальная запись 30618_e9a34502 на листе жюри config/tram.yaml, входы ядра
#    так, как их подаёт Runner, варианты с аномалиями из tools/inject.py -> compare_bag.js;
# 3) record_runner.py: та же запись через Runner (сетка, свежесть, таймауты, приведение
#    показаний к шагу) и базу «только колесо» -> compare_runner.js (JS-связка runner.js);
# 4) копии для страницы simulator/js/{est,runner}.js совпадают с js-port/ байт в байт.
# Код выхода 0 — всё совпало (≤ 1e-6, режимы и флаги на каждом шаге) и отпечаток ядра равен
# TramEst.PORT.core_sha1; иначе 1 (что делать — печатается).
# Переменные: IMAGE (vectra/tram:integration — Python с numpy; код пакета берётся из этого
# дерева), DATA_DIR (<repo>/data), CACHE_DIR (<repo>/analysis/cache), BAG, QUICK=1 (только clean).
set -uo pipefail
export MSYS_NO_PATHCONV=1
ROOT="$(cd "$(dirname "$0")/.." && (pwd -W 2>/dev/null || pwd))"
DATA_DIR="${DATA_DIR:-$ROOT/data}"
CACHE_DIR="${CACHE_DIR:-$ROOT/analysis/cache}"
IMAGE="${IMAGE:-vectra/tram:integration}"
BAG="${BAG:-30618_e9a34502}"
PY=(docker run --rm --cpus 2 -v "$ROOT:/repo" -v "$DATA_DIR:/repo/data:ro" -v "$CACHE_DIR:/repo/analysis/cache:ro"
    -w /repo/js-port "$IMAGE")
if command -v node >/dev/null 2>&1; then JS=(node); cd "$(dirname "$0")"; else JS=(docker run --rm --network none -v "$ROOT/js-port:/p" -w /p vectra/tram:sim node); fi
rc=0
"${PY[@]}" python3 record.py || exit 2
"${JS[@]}" compare.js | tail -n 3 || rc=1
if [ "${QUICK:-0}" = 1 ]; then BAGV="clean"; RUNV="clean"; else
  BAGV="clean both_zero both_stuck dropout skid_brake spin_traction noise"; RUNV="clean both_stuck dropout noise"; fi
for v in $BAGV; do
  "${PY[@]}" python3 record_bag.py "$BAG" "$v" || exit 2
  "${JS[@]}" compare_bag.js || rc=1
done
for v in $RUNV; do
  "${PY[@]}" python3 record_runner.py "$BAG" "$v" || exit 2
  "${JS[@]}" compare_runner.js || rc=1
done
for f in est.js runner.js; do
  if ! cmp -s "$ROOT/js-port/$f" "$ROOT/simulator/js/$f"; then
    echo "simulator/js/$f ≠ js-port/$f: скопируйте (node js-port/sync.js)"; rc=1; fi
done
[ $rc = 0 ] && echo "js-port/check: OK — порт и связка совпадают с пакетом" || echo "js-port/check: РАСХОЖДЕНИЕ — см. выше"
exit $rc
