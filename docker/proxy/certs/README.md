# Сертификаты демо-сервера

Каталог монтируется в прокси (`docker compose --profile server up`) как `/etc/nginx/certs`.
Положите сюда `fullchain.pem` и `privkey.pem` (PEM, например от certbot): прокси включит HTTPS на
443, без них работает только HTTP на 80. Другой каталог - `TLS_DIR` в `.env`, другие пути -
`TLS_CERT` и `TLS_KEY` ([docs/DEPLOY.md](../../../docs/DEPLOY.md#https)). Сертификаты в git не
попадают: `*.pem` в `.gitignore`.
