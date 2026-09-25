#!/usr/bin/env bash
# Статический CSS Tailwind v3 для simulator/index.html (страница работает с file:// без сети).
# Tailwind CLI запускается в контейнере node:22-alpine; результат вставляется в index.html
# между /* tw:begin */ и /* tw:end */. Запускать после правки классов в разметке.
#   bash simulator/css/build_css.sh
set -euo pipefail
SIM="$(cd "$(dirname "$0")/.." && pwd)"
MSYS_NO_PATHCONV=1 docker run --rm -v "$SIM":/sim -w /sim/css node:22-alpine sh -c '
  set -e
  mkdir -p /tmp/tw && cd /tmp/tw && npm init -y >/dev/null && npm i --silent --no-audit --no-fund tailwindcss@3.4.17 >/dev/null
  cd /sim/css && /tmp/tw/node_modules/.bin/tailwindcss -c tailwind.config.js -i input.css -o /tmp/tw.css --minify
  node inject.js /tmp/tw.css ../index.html'
