# Развёртывание демо: ноутбук и публичный сервер

Схема демо (решение команды 25.09): показываем с ноутбука. ROS 2 работает на
сервере, страница симулятора открыта по публичному домену. Всё поднимает
`docker-compose.yml` из корня репозитория, настройки — в `.env` (шаблон
`.env.example`).

## 1. Ноутбук (всё локально)

```bash
cp .env.example .env        # DATA_DIR=<папка с прогонами>, BAG=30618_e9a34502
docker compose up --build   # первый раз соберёт образ (~3 мин, нужен интернет)
```

`--build` обязателен после каждого `git pull`: без него compose возьмёт готовый
образ `vectra/tram:compose`, а в нём нода собрана из старых исходников (repo
смонтирован, но нода запускается из `/ws` образа).

| Адрес | Что |
|---|---|
| `http://localhost:8080` | страница симулятора (сервис `web`, `python3 -m http.server`) |
| `ws://localhost:9090` | rosbridge (сервис `bridge`): `/result/*`, `/tram/estimator_status` |

После сборки образа сеть не нужна: нода, мост и bag — локально. Страница
симулятора тоже без внешних зависимостей (CSS собран статически, шрифты системные).

Порты по умолчанию открыты только на `127.0.0.1`. Чтобы открыть страницу с
другого устройства в той же сети, задайте в `.env` `WEB_BIND=0.0.0.0` и
`BRIDGE_BIND=0.0.0.0`.

Прогон по кругу: `PLAY_LOOP=1`. Нода между кругами не перезапускается: разрыв
меток на новом круге она распознаёт как новый прогон и выставляется заново по его
GNSS ([ROBUST.md](ROBUST.md) §2а).

## 2. Публичный сервер

На сервере с Docker и открытыми портами 80/443:

```bash
git clone <репозиторий> vectra && cd vectra
cp .env.example .env
#   DATA_DIR=/srv/tram/data        папка с прогонами
#   BAG=30618_e9a34502  PLAY_LOOP=1
#   PUBLIC_HOST=demo.example.org   домен сервера
docker compose --profile server up -d --build
docker compose logs -f proxy estimator player
```

Профиль `server` добавляет к сервисам демо обратный прокси nginx
(`nginx:1.30-alpine` — 1.30.5 stable от 22.09.2026, закреплён по digest):

| Путь | Куда |
|---|---|
| `/` | статическая страница `simulator/` |
| `/ros`, `/ros/` | websocket rosbridge (`bridge:9090`); префикс `/ros` снимается |
| `/healthz` | `ok` — проверка живости |

Порт 9090 наружу не открывается: `BRIDGE_BIND=127.0.0.1` по умолчанию. Мост
доступен только через прокси.

### HTTPS

Положите PEM-сертификат и ключ в `docker/proxy/certs/`:
`fullchain.pem`, `privkey.pem`. Другой каталог задаётся в `TLS_DIR`, другие имена
файлов — в `TLS_CERT` / `TLS_KEY`. Например, от certbot:

```bash
certbot certonly --standalone -d demo.example.org        # до запуска прокси (порт 80 свободен)
TLS_DIR=/etc/letsencrypt/live/demo.example.org           # в .env
```

Если в `TLS_DIR` лежат символические ссылки (как у certbot в `live/`), смонтируйте
весь `/etc/letsencrypt` и укажите полные пути в `TLS_CERT` и `TLS_KEY`.

При старте прокси проверяет файлы:
- есть — включает HTTPS на 443, в логе `vectra-proxy: HTTPS включён`;
- нет — работает только HTTP на 80.

HTTP при этом тоже остаётся. Если нужен редирект на HTTPS, добавьте его в
`docker/proxy/templates/default.conf.template`.

### Адрес моста для страницы

| Страница открыта по | Адрес моста |
|---|---|
| `http://localhost:8080` (ноутбук) | `ws://localhost:9090` |
| `http://<сервер>/` (прокси) | `ws://<сервер>/ros` |
| `https://<сервер>/` (прокси с TLS) | `wss://<сервер>/ros` |
| страница на другом домене, ROS на сервере | `wss://<сервер>/ros` |

Страница должна давать задать адрес моста, например `?ws=wss://demo.example.org/ros`.
По умолчанию разумно брать `wss://` или `ws://` от `location.host` + `/ros`, а для
`localhost:8080` — `ws://localhost:9090`. Это делает поток симулятора (WP18).

### CORS, Origin, смешанный контент

- Websocket не подчиняется CORS. rosbridge (Humble) заголовок `Origin` не
  проверяет. Проверено: подключение с `Origin: https://sim.example.org` принято.
  Поэтому страница с любого домена может подключиться к `wss://<сервер>/ros`.
- Страница по `https://` может открыть только `wss://`: браузер блокирует `ws://`
  как смешанный контент. Значит, если страница на HTTPS-домене, серверу нужен
  сертификат.
- Сертификат для `wss://` должен быть доверенным: браузер молча отклоняет
  самоподписанный при подключении websocket. Самоподписанный годится только для
  проверки через `tools/ws_check.py --insecure`.
- Если нужно ограничить, кто подключается к мосту, закройте `/ros` в nginx:
  `allow`/`deny` по IP или `auth_basic`. У rosbridge своей авторизации нет.
- **Мост только для чтения (по умолчанию, интеграция 26.09).** Сервис `bridge`
  запускается `docker/bridge.sh`: подписка только на топики страницы
  (`/result/*`, `/tram/*`, `/vehicle/*`, `/sensing/gnss/master/*`, `/sensing/gnss/rover/fix` — для эталона base_link на странице), публикация,
  сервисы и параметры закрыты — посетитель домена не может подать ноде ложные
  `/vehicle/*` или GNSS и вызвать `set_parameters`. Проверено: `docker compose up`,
  20 попыток опубликовать `/vehicle/front_bogie_velocity` через мост — у топика
  по-прежнему один издатель (`rosbag2_player`); `/result/velocity` через мост —
  20,0 Гц. `ROSBRIDGE_OPEN=1` в `.env` — открытый мост для отладки на своей машине.

## 3. Проверка

```bash
curl -s http://<сервер>/healthz                                   # ok
docker compose run --rm --no-deps -T web \
  python3 /repo/tools/ws_check.py ws://proxy/ros --seconds 5      # изнутри сети compose
python3 tools/ws_check.py wss://demo.example.org/ros               # снаружи, нужен tornado
```

`ws_check.py` подписывается на `/result/velocity` и `/result/position` и печатает
частоту и последний выход. Код 0 — сообщения идут.

Проверено 25.09 на Docker Desktop:

| Проверка | Результат |
|---|---|
| `ws://bridge:9090` | 20 Гц |
| `ws://proxy/ros`, `ws://…:18081/ros/` | 20 Гц |
| `wss://proxy/ros` с тестовым самоподписанным сертификатом, `--insecure` | 20 Гц |
| тот же `wss://` без `--insecure` | отказ: `CERTIFICATE_VERIFY_FAILED` |
| страница через прокси по HTTP и HTTPS | 148 739 байт, как с диска |
| то же с `nginx:1.30-alpine` (1.30.5), `docker compose --profile server up -d` | `/healthz` ok; страница 148 739 байт по HTTP и HTTPS; `ws://proxy/ros` и `wss://proxy/ros` — 20 Гц; у `estimator` `restart=no` |

## 4. Сетевые заметки

- **DDS между контейнерами.** ROS-сервисы находятся в сети проекта compose и
  видят друг друга по multicast в одном `ROS_DOMAIN_ID`. `player` и `probe`
  работают в сетевом и IPC-пространстве `estimator` — как процессы на одной машине.
- **Изоляция.** Сеть проекта изолирована: чужие ROS 2 на сервере в тот же домен не
  попадут. Если нужен `network_mode: host`, задайте уникальный `ROS_DOMAIN_ID`.
- **Останов.** `docker compose down` посылает SIGINT (`stop_signal`), как Ctrl+C.
- **Перезапуск.** У `estimator` нет `restart`: `player` и `probe` живут в его
  сетевом, IPC и PID пространствах, а перезапущенный контейнер получает новые.
  После перезапуска `player` остаётся в старом пространстве без DDS, `probe`
  завершается (проверено ревьюером: `kill -9` ноды). Упавшую ноду перезапускает
  сам launch (`respawn`, поток robust). Если упал весь контейнер `estimator`:
  `docker compose up -d --force-recreate`.
- **Второй вариант сервера.** Есть `simulator/deploy/compose.demo.yml` (поток
  sim: caddy с автоматическим TLS от Let's Encrypt, тот же путь `/ros`, мост
  только для чтения). Он тоже занимает порты 80/443: на сервере запускать что-то
  одно. **Решение интеграции: основной — `docker compose --profile server up -d`
  из корня** (один compose на ноутбук и сервер, тесты и оценку, nginx закреплён
  по digest); `compose.demo.yml` — запасной, если на сервере нужен
  автоматический сертификат Let's Encrypt без ручной выдачи.
