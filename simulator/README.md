# Симулятор Vectra (simulator/index.html)

Одна страница, работает с `file://` без сети (CSS собран статически, шрифты системные). Три режима — переключатель в шапке, либо `?mode=sandbox|replay|live`.

| Режим | Что показывает | Откуда числа |
|---|---|---|
| **Песочница** | имитатор трамвая (1 кГц, 4 оси, погода, отказы датчиков) и **JS-порт ядра пакета** | порт сверен с Python-ядром 25.09 (≤ 10⁻¹² м/с, `js-port/README.md`); лист песочницы по умолчанию (4 оси, паспортный привод), имитатор упрощён — числа песочницы только для наглядности |
| **Прогон данных комиссии** | реальная отложенная запись: скорость (GNSS, модель ±2σ, база «только колесо», тележки), ошибки, план, состояние оценщика по тележкам, метрики до курсора | **сам пакет**: `tools/export_replay.py` гоняет `Runner` (как нода) по записи и пишет `replays/*.js` |
| **Живой ROS 2** | тот же экран по живому потоку ноды | rosbridge v2 (JSON поверх WebSocket, без roslib), 10 Гц на топик |

## Прогон данных комиссии

```bash
# из корня репозитория; кэш прогонов analysis/cache (в .gitignore) монтируется только на чтение
docker run --rm --cpus 2 -v <repo>:/repo -v <repo>/analysis/cache:/repo/analysis/cache:ro \
  -w /repo vectra/tram:dev python3 tools/export_replay.py            # 30618_e9a34502, 5 вариантов, ~3,5 мин
#   --run <bag>              другой прогон из holdout_scored (tools/split.json); обучающий — только с --allow-train
#   --variants clean,front_zero,both_zero,dropout,skid_brake
#   --sheet <yaml>           по умолчанию config/eval/tram.yaml (оценочный), если есть, иначе config/tram.yaml
#   --map <npz>              по умолчанию analysis/cache/track_map_train.npz (только обучающие)
```

- Сообщения подаются в порядке записи bag; GNSS в оценщик — только первые 3 с (как `analysis/evaluate.py`), дальше он только эталон.
- Варианты: чистый; отказ передней тележки (0 км/ч, 60 с); обе тележки 0 км/ч 20 с; пропуск сообщений 2 с; юз −30 % на торможении 4 с. Окно выбирается по входам так же, как в `tools/inject.py` (если он есть — берётся он).
- Плоскость на экране — система выхода модели (определяется по данным: MGRS/UTM, ENU, equirect), сдвинутая в первую точку GNSS master; перенос квадрата MGRS (100 км) снимается. Судья считает в MGRS; ошибки в плане от сдвига не зависят.
- Метрики страницы считаются по парам «выход — эталон» из файла (MAE по |v| GNSS master, пары ≤ 0,05 с; дрейф — ошибка вдоль трассы / путь по эталону). В конце прогона они совпадают с итогом экспортёра (проверяет `test/page_test.js`).
- Файл прогона: 10 Гц, целые с масштабом, плавные колонки разностями; ~1,2 МБ, в gzip ~110 КБ.

## Живой ROS 2

Адрес моста: поле в панели, либо `?ros=wss://хост/ros`. По умолчанию: страница с `https://хост` — `wss://хост/ros`, с `http://хост` — `ws://хост/ros`, с `file://` и localhost — `ws://localhost:9090`. Переподключение с отсрочкой 1, 2, 4 … 15 с; при обрыве и молчании ноды — «нет связи».

| Где | Команда |
|---|---|
| ноутбук | `docker run --rm -p 127.0.0.1:9090:9090 -v <repo>:/repo:ro -v <data>:/data:ro -e BAG=30618_e9a34502 vectra/tram:dev bash /repo/simulator/deploy/live_demo.sh`, затем `index.html?mode=live` |
| сервер с доменом | `DOMAIN=demo.example.ru DATA_DIR=/srv/data docker compose -f simulator/deploy/compose.demo.yml up -d` → `https://demo.example.ru/?mode=live` (Caddy: TLS, `/ros` → rosbridge) |
| свой прокси | `deploy/nginx.conf`: страница и `location /ros` → `tram-live:9090/` с заголовками Upgrade |

Подписки: `/result/velocity`, `/result/position`, `/tram/estimator_status`, `/vehicle/*`, `/sensing/gnss/master/{vel,fix}` (GNSS — только эталон для показа), `throttle_rate` 100 мс (fix 200 мс), `queue_length` 1. Частота ноды на странице — по `frame_count` статуса.

## Проверки

| Проверка | Команда |
|---|---|
| песочница без браузера (детерминированно, S1) | `SEED=3 node simulator/test/sim_headless.js 10` |
| JS-порт ядра против Python | `js-port/README.md` (`record.py` + `compare.js`, `record_bag.py` + `compare_bag.js`) |
| страница в Chromium без сети: 0 исключений, прогоны, метрики = экспортёр, скриншоты | `docker build -t vectra/tram:sim simulator/test` · `docker run --rm --network none -v <repo>/simulator:/sim:ro -v <out>:/out vectra/tram:sim node /sim/test/page_test.js` |
| живой режим против настоящего rosbridge (обрыв и переподключение) | `bash simulator/test/live_test.sh` |
| пересборка CSS после правки классов Tailwind | `bash simulator/css/build_css.sh` |
