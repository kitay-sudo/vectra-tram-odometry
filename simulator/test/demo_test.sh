#!/usr/bin/env bash
# Проверка демо-развёртывания: compose.demo.yml (tram-live + Caddy) и браузер в той же
# сети docker открывает http://web/?mode=live без ?ros= (мост - тот же хост, /ros).
#   bash simulator/test/demo_test.sh        (Git Bash / Linux, из корня; DATA_DIR - каталог прогонов)
set -euo pipefail
export MSYS_NO_PATHCONV=1
ROOT="$(cd "$(dirname "$0")/../.." && (pwd -W 2>/dev/null || pwd))"
export DATA_DIR="${DATA_DIR:-$ROOT/data}" DOMAIN=":80" HTTP_PORT="${HTTP_PORT:-18080}" HTTPS_PORT="${HTTPS_PORT:-18443}"
OUT="${OUT:-$ROOT/out/sim/demo}"
P="tvdemo$RANDOM"
mkdir -p "$OUT"
trap 'docker compose -p "$P" -f "$ROOT/simulator/deploy/compose.demo.yml" down -v >/dev/null 2>&1 || true' EXIT
docker compose -p "$P" -f "$ROOT/simulator/deploy/compose.demo.yml" up -d
docker run --rm --network "${P}_default" -v "$ROOT/simulator:/sim:ro" -v "$OUT:/out" vectra/tram:sim \
  node /sim/test/demo_test.js http://web /out && echo "demo_test: OK" || { echo "demo_test: FAIL"; exit 1; }
