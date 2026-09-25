#!/bin/sh
# Включает HTTPS на 443, если смонтированы сертификат и ключ (PEM).
# TLS_CERT / TLS_KEY — пути внутри контейнера (по умолчанию
# /etc/nginx/certs/fullchain.pem и privkey.pem; каталог — TLS_DIR из .env).
set -e
out=/etc/nginx/conf.d/vectra-tls.conf
if [ -s "${TLS_CERT}" ] && [ -s "${TLS_KEY}" ]; then
  cat > "$out" <<CONF
server {
    listen 443 ssl;
    listen [::]:443 ssl;
    http2 on;
    server_name ${PUBLIC_HOST};
    ssl_certificate     ${TLS_CERT};
    ssl_certificate_key ${TLS_KEY};
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_session_cache shared:vectra:10m;
    include /etc/nginx/conf.d/vectra-locations.inc;
}
CONF
  echo "vectra-proxy: HTTPS включён (${TLS_CERT})"
else
  rm -f "$out"
  echo "vectra-proxy: сертификата нет (${TLS_CERT}) — только HTTP :80"
fi
