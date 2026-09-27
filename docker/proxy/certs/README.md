# Сертификаты демо-сервера

Каталог монтируется в прокси (`docker compose --profile server up`) как
`/etc/nginx/certs`. Положите сюда `fullchain.pem` и `privkey.pem` (PEM, например
от Let's Encrypt / certbot) - прокси включит HTTPS на 443. Без них работает только
HTTP на 80. Другой каталог - `TLS_DIR` в `.env`, другие имена - `TLS_CERT`/`TLS_KEY`.

Сертификаты в git не кладём (`*.pem` в `.gitignore`).
