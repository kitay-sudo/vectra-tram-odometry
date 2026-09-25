# Образ для проверки страницы в headless Chromium (simulator/test/page_test.js, live_test.js):
#   docker build -t vectra/tram:sim simulator/test
FROM node:22-alpine
RUN apk add --no-cache chromium font-noto font-noto-cjk ttf-dejavu \
 && mkdir -p /opt/pp && cd /opt/pp && npm init -y >/dev/null && npm i --no-audit --no-fund puppeteer-core@23 >/dev/null
ENV PUPPETEER_EXECUTABLE_PATH=/usr/bin/chromium-browser NODE_PATH=/opt/pp/node_modules
