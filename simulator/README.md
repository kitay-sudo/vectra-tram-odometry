# Симулятор Vectra (simulator/index.html)

Одна страница, работает с `file://` без сети (CSS собран статически, шрифты системные). Три режима — переключатель в шапке, либо `?mode=sandbox|replay|live`.

| Режим | Что показывает | Откуда числа |
|---|---|---|
| **Песочница** | имитатор трамвая (1 кГц, 4 оси, погода, отказы датчиков) и **JS-порт ядра пакета** | порт сверен с Python-ядром 25.09 (ядро `56933cc`, ≤ 6·10⁻¹³ м/с, `js-port/README.md`); если прогоны комиссии посчитаны другим ядром (отпечаток `core_sha1` в `replays/index.js` ≠ `TramEst.PORT`), песочница подписана «упрощённое ядро (JS-порт от 25.09)»; лист песочницы по умолчанию (4 оси, паспортный привод), имитатор упрощён — числа песочницы только для наглядности |
| **Прогон данных комиссии** | реальная отложенная запись: скорость (GNSS, модель ±2σ, база «только колесо», тележки), ошибки, план, состояние оценщика по тележкам, метрики до курсора | **сам пакет**: `tools/export_replay.py` гоняет `Runner` (как нода) по записи и пишет `replays/*.js` |
| **Живой ROS 2** | тот же экран по живому потоку ноды | rosbridge v2 (JSON поверх WebSocket, без roslib), 10 Гц на топик |

## Прогон данных комиссии

```bash
# из корня репозитория; кэш прогонов analysis/cache (в .gitignore) монтируется только на чтение
docker run --rm --cpus 2 -v <repo>:/repo -v <repo>/data:/repo/data:ro -v <repo>/analysis/cache:/repo/analysis/cache:ro \
  -w /repo vectra/tram:dev python3 tools/export_replay.py            # 30618_e9a34502, 5 вариантов, ~5 мин
#   --run <bag>              другой прогон из holdout_scored (tools/split.json); обучающий — только с --allow-train
#   --variants clean,front_zero,both_zero,dropout,skid_brake
#   --sheet eval|jury|<путь> по умолчанию eval — правила tools/eval.py: config/eval/{tram_eval.yaml,tram.yaml,
#                            tram_calibration.json}; пока их нет — config/tram_calibration.json с пометкой УТЕЧКА
#   --map <npz>              по умолчанию config/eval/track_map.npz пакета (поток «положение»), иначе
#                            analysis/cache/track_map_train.npz — обе только из обучающих; '' — без карты
```

- Связка — `Runner` пакета с параметрами ноды так же, как в `tram_node.py` (таймауты, окно выставки, после потока «положение» — `projection`, `mgrs_grid`, `utm_zone`, …). База «только колесо» — тот же `Runner` с подменённым ядром (среднее свежих показаний тележек), так что выставка, карта, привязки и система выхода у неё те же и не копируются.
- Лист и пометка утечки пишутся в файл прогона и видны на странице; числа с пометкой «УТЕЧКА» не отчётные (решение 7). После слияния калибровки (оценочный лист `config/eval/`) — перезапустить экспорт и закоммитить `replays/`.
- Сообщения подаются в порядке записи bag; GNSS в оценщик — только первые 3 с (как `analysis/evaluate.py`), дальше он только эталон.
- Варианты: чистый; отказ передней тележки (0 км/ч, 60 с); обе тележки 0 км/ч 20 с; пропуск сообщений 2 с; юз −30 % на торможении 4 с. Окно выбирается по входам так же, как в `tools/inject.py` (если он есть — берётся он).
- Плоскость на экране — система выхода модели (из настроек `Runner`: `projection`/`mgrs_grid`, с проверкой по данным: MGRS/UTM, ENU, equirect), сдвинутая в первую точку GNSS master; перенос квадрата MGRS (100 км) снимается. Судья считает в MGRS; ошибки в плане от сдвига не зависят.
- Метрики страницы считаются по парам «выход — эталон» из файла (MAE по |v| GNSS master, пары ≤ 0,05 с; дрейф — ошибка вдоль трассы / путь по эталону). В конце прогона они совпадают с итогом экспортёра (проверяет `test/page_test.js`).
- Файл прогона: 10 Гц, целые с масштабом, плавные колонки разностями; ~1,2 МБ, в gzip ~110 КБ.

## Живой ROS 2

Адрес моста: поле в панели, либо `?ros=wss://хост/ros`. По умолчанию: страница с `https://хост` — `wss://хост/ros`, с `http://хост` — `ws://хост/ros`, с `file://` и localhost — `ws://localhost:9090`. Переподключение с отсрочкой 1, 2, 4 … 15 с; при обрыве и молчании ноды — «нет связи».

| Где | Команда |
|---|---|
| ноутбук | `docker run --rm -p 127.0.0.1:9090:9090 -v <repo>:/repo:ro -v <data>:/data:ro -e BAG=30618_e9a34502 vectra/tram:dev bash /repo/simulator/deploy/live_demo.sh`, затем `index.html?mode=live` |
| сервер с доменом | `DOMAIN=demo.example.ru DATA_DIR=/srv/data docker compose -f simulator/deploy/compose.demo.yml up -d` → `https://demo.example.ru/?mode=live` (Caddy: TLS, `/ros` → rosbridge) |

Мост **только для чтения** (`deploy/live_demo.sh`, по умолчанию): подписка лишь на топики страницы, публикация запрещена (иначе любой посетитель домена мог бы подать ноде ложные `/vehicle/*` или GNSS), сервисы — только `/rosapi/*` без параметров, `set_parameters` ноды закрыт. Проверяет `test/live_test.sh` (контроль с `ROSBRIDGE_OPEN=1` — проверка падает). Отладка без ограничений: `ROSBRIDGE_OPEN=1`. Доступ к странице по паролю — при желании `basic_auth` в `deploy/Caddyfile`.
| свой прокси | `deploy/nginx.conf`: страница и `location /ros` → `tram-live:9090/` с заголовками Upgrade |
| страница и ROS на разных хостах | страница — любой статический хостинг (каталог `simulator/`); ROS-сервер — `compose.demo.yml` с `DOMAIN=ros.example.ru` (Caddy даёт `wss://ros.example.ru/ros`); адрес моста — `config.js` (`ros: 'wss://ros.example.ru/ros'`) или `?ros=` |

Порядок выбора адреса моста: `?ros=` → адрес, введённый на странице (запоминается в браузере) → `config.js` → по умолчанию. Страница по https ходит только на `wss://`. С сервера (не localhost) страница подключается сама; с `file://` — по кнопке или с `?ros=`.

Если поле адреса правят при подключении (пустое, недописанное), страница не замирает: адрес хоста берётся из соединения, циклы кадров защищены (`test/page_test.js`).

Подписки: `/result/velocity`, `/result/position`, `/tram/estimator_status`, `/vehicle/*`, `/sensing/gnss/master/{vel,fix}` (GNSS — только эталон для показа), `throttle_rate` 100 мс (fix 200 мс), `queue_length` 1. Частота ноды на странице — по `frame_count` статуса.

## Проверки

| Проверка | Команда |
|---|---|
| песочница без браузера (детерминированно, S1) | `SEED=3 node simulator/test/sim_headless.js 10` |
| JS-порт ядра против Python (**обязательно после любой правки `estimator_core.py`**) | `bash js-port/check.sh` (код выхода 0 — совпадает; см. `js-port/README.md`) |
| страница в Chromium без сети: 0 исключений, прогоны, метрики = экспортёр, подпись порта, живой режим с имитатором моста (правка адреса, переход квадрата MGRS 37U CB/DB), скриншоты | `docker build -t vectra/tram:sim simulator/test` · `docker run --rm --network none -v <repo>/simulator:/sim:ro -v <out>:/out vectra/tram:sim node /sim/test/page_test.js` |
| живой режим против настоящего rosbridge (обрыв и переподключение, мост только для чтения) | `bash simulator/test/live_test.sh` |
| пересборка CSS после правки классов Tailwind | `bash simulator/css/build_css.sh` |
