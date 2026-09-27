# Развёртывание демо: ноутбук и публичный сервер

Как поднять режим «Живой ROS 2» на ноутбуке или на сервере с доменом. Всё поднимает
`docker-compose.yml` из корня репозитория, настройки - `.env` по шаблону `.env.example`. Числа
раздела 3 - замеры на Docker Desktop.

**Разделы**

1. [Ноутбук](#1-ноутбук-всё-локально)
2. [Публичный сервер](#2-публичный-сервер)
3. [Проверка](#3-проверка)
4. [Сетевые заметки](#4-сетевые-заметки)

## 1. Ноутбук (всё локально)

Команды выполняются из корня репозитория.

Создать `.env` из шаблона, чтобы задать папку прогонов `DATA_DIR` и запись `BAG`.

```bash
cp .env.example .env
```

Собрать образ и поднять ноду, проигрывание записи, мост и страницу (первый раз около 3 мин).

```bash
docker compose up --build
```

| Адрес | Что там |
|:---|:---|
| `http://localhost:8080` | страница симулятора (сервис `web`), живой режим - `/?mode=live` |
| `ws://localhost:9090` | мост rosbridge (сервис `bridge`): `/result/*`, `/tram/estimator_status` |

- **`--build` после `git pull`.** Нода запускается из `/ws` образа, без пересборки останется
  старый `vectra/tram:compose`.
- **Интернет.** Нужен только при сборке: нода, мост и запись локальны, у страницы нет внешних
  ресурсов.
- **Другие устройства.** `WEB_BIND=0.0.0.0` и `BRIDGE_BIND=0.0.0.0` в `.env` открывают порты
  для сети; по умолчанию они открыты только на `127.0.0.1`.
- **По кругу.** `PLAY_LOOP=1`. Нода между кругами не перезапускается: разрыв меток она
  распознаёт как новый прогон и выставляется заново по GNSS
  ([ROBUST.md](ROBUST.md#2а-разрывы-времени), раздел 2а).

## 2. Публичный сервер

Нужен сервер с Docker и открытыми портами 80 и 443. После `cd` команды выполняются из корня
репозитория.

Скачать репозиторий.

```bash
git clone https://github.com/kitay-sudo/vectra-tram-odometry.git ~/vectra-tram-odometry
```

Перейти в корень репозитория.

```bash
cd ~/vectra-tram-odometry
```

Создать `.env` из шаблона.

```bash
cp .env.example .env
```

Задать в `.env` папку прогонов, запись, прогон по кругу и домен.

```text
DATA_DIR=<папка с прогонами>
BAG=30618_e9a34502
PLAY_LOOP=1
PUBLIC_HOST=<домен>
```

Поднять сервисы демо вместе с обратным прокси nginx.

```bash
docker compose --profile server up -d --build
```

Посмотреть логи прокси, ноды и проигрывания.

```bash
docker compose logs -f proxy estimator player
```

Профиль `server` добавляет прокси `nginx:1.30-alpine` (1.30.5 stable, закреплён по digest).
Порт 9090 наружу не открыт (`BRIDGE_BIND=127.0.0.1`), мост доступен только через прокси.

| Путь | Куда |
|:---|:---|
| `/` | статическая страница `simulator/` |
| `/ros`, `/ros/` | websocket rosbridge (`bridge:9090`), префикс `/ros` снимается |
| `/healthz` | ответ `ok`: проверка живости |

### HTTPS

Прокси включает HTTPS на 443, если при старте находит PEM-сертификат и ключ (в логе
`vectra-proxy: HTTPS включён`), иначе работает только по HTTP на 80. По умолчанию это
`fullchain.pem` и `privkey.pem` в `docker/proxy/certs/`. HTTP остаётся и при HTTPS; если нужен
редирект на HTTPS, добавьте его в `docker/proxy/templates/default.conf.template`.

Выпустить сертификат через certbot до запуска прокси, пока порт 80 свободен.

```bash
certbot certonly --standalone -d <домен>
```

Задать в `.env` весь каталог certbot и полные пути внутри контейнера: в `live/` лежат ссылки на
`archive/`.

```text
TLS_DIR=/etc/letsencrypt
TLS_CERT=/etc/nginx/certs/live/<домен>/fullchain.pem
TLS_KEY=/etc/nginx/certs/live/<домен>/privkey.pem
```

### Адрес моста для страницы

| Страница открыта по | Адрес моста по умолчанию |
|:---|:---|
| `file://` или `http://localhost:8080` | `ws://localhost:9090` |
| `http://<домен>/` (прокси) | `ws://<домен>/ros` |
| `https://<домен>/` (прокси с TLS) | `wss://<домен>/ros` |

Другой адрес, например для страницы на другом домене, задаётся полем в панели страницы или
параметром `?ros=wss://<домен>/ros` ([simulator/README.md](../simulator/README.md#5-живой-ros-2)).

### Доступ к мосту

- **CORS.** Websocket не подчиняется CORS, а rosbridge (Humble) не проверяет `Origin`
  (подключение с `Origin: https://sim.example.org` принято): к мосту подключится страница с
  любого домена.
- **Страница по HTTPS.** Открывает только `wss://` с доверенным сертификатом: `ws://` браузер
  блокирует как смешанный контент, самоподписанный сертификат молча отклоняет. Самоподписанный
  годится только для проверки `tools/ws_check.py --insecure`.
- **Только чтение (по умолчанию).** `docker/bridge.sh` разрешает лишь подписку на топики
  страницы: `/result/*`, `/tram/*`, `/vehicle/*`, `/sensing/gnss/master/*`,
  `/sensing/gnss/rover/fix`. Публикация, сервисы и параметры закрыты, ложные входы и
  `set_parameters` не пройдут. После 20 попыток опубликовать `/vehicle/front_bogie_velocity`
  через мост у топика по-прежнему один издатель (`rosbag2_player`), `/result/velocity` идёт с
  частотой 20,0 Гц. Открытый мост для отладки на своей машине - `ROSBRIDGE_OPEN=1`.
- **Авторизация.** У rosbridge её нет. Закрыть `/ros` можно в nginx: `allow`/`deny` по IP или
  `auth_basic`.

## 3. Проверка

Команды выполняются из корня репозитория. `ws_check.py` подписывается на `/result/velocity` и
`/result/position`, печатает частоту и последний выход, код 0 - сообщения идут.

Проверить, что прокси отвечает `ok`.

```bash
curl -s http://<домен>/healthz
```

Проверить мост через прокси изнутри сети compose.

```bash
docker compose run --rm --no-deps -T web \
  python3 /repo/tools/ws_check.py ws://proxy/ros --seconds 5
```

Проверить мост снаружи (нужен Python с пакетом `tornado`).

```bash
python3 tools/ws_check.py wss://<домен>/ros
```

Результаты на Docker Desktop:

| Проверка | Результат |
|:---|:---|
| `ws://bridge:9090` | 20 Гц |
| `ws://proxy/ros`, `ws://…:18081/ros/` | 20 Гц |
| `wss://proxy/ros`, самоподписанный сертификат, `--insecure` | 20 Гц |
| тот же `wss://` без `--insecure` | отказ: `CERTIFICATE_VERIFY_FAILED` |
| страница через прокси по HTTP и HTTPS | 148 739 байт, как с диска |

Те же проверки с `nginx:1.30-alpine` (1.30.5) после `docker compose --profile server up -d`:
`/healthz` отвечает `ok`, страница 148 739 байт по HTTP и HTTPS, `ws://proxy/ros` и
`wss://proxy/ros` - 20 Гц.

## 4. Сетевые заметки

- **DDS.** Сервисы видят друг друга по multicast в сети проекта compose с одним `ROS_DOMAIN_ID`.
  `player` и `probe` работают в сетевом и IPC-пространстве `estimator`, как процессы на одной
  машине. Чужой ROS 2 в изолированную сеть проекта не попадёт; при `network_mode: host` задайте
  уникальный `ROS_DOMAIN_ID`.
- **Останов.** `docker compose down` посылает SIGINT (`stop_signal`), как Ctrl+C.
- **Запасной сервер.** `simulator/deploy/compose.demo.yml`: Caddy с автоматическим сертификатом
  Let's Encrypt, тот же `/ros`, мост только для чтения. Он занимает те же порты 80 и 443, поэтому
  запускают что-то одно.
- **Перезапуск.** Упавшую ноду перезапускает сам launch (`respawn`). У `estimator` нет `restart`:
  `player` и `probe` живут в его сетевом, IPC и PID пространствах, а перезапущенный контейнер
  получает новые. После перезапуска `player` остаётся в старом пространстве без DDS, `probe`
  завершается (проверено `kill -9` ноды).

Пересоздать сервисы, если упал весь контейнер `estimator`.

```bash
docker compose up -d --force-recreate
```
