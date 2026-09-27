# Карта документов

Какие документы лежат в `docs/`, с чего начать проверку и какая команда пишет каждый
сгенерированный файл. Числа во всех документах взяты из файлов результатов, перечисленных
ниже; главный из них - [EVAL.md](EVAL.md), его целиком пишет `tools/eval.py`.

**Разделы:** [С чего начать](#с-чего-начать) · [Группы документов](#группы-документов) · [Какая команда пишет какой файл](#какая-команда-пишет-какой-файл)

## С чего начать

1. **[README.md](../README.md)** - что делает пакет, запуск одной командой, главные числа.
2. **[JURY.md](JURY.md)** - инструкция для жюри: сборка, запуск, топики, система
   координат, задержка, CPU и память, неполадки.
3. **[EVAL.md](EVAL.md)** - точность и устойчивость на 15 чистых отложенных записях.
4. **[TZ_COMPLIANCE.md](TZ_COMPLIANCE.md)** - где в репозитории каждый пункт ТЗ.

## Группы документов

| Группа | Документы |
|---|---|
| проверка и запуск | [JURY.md](JURY.md), [DEPLOY.md](DEPLOY.md), [DEMO.md](DEMO.md) |
| задание и соответствие | [TZ_REQUIREMENTS.md](TZ_REQUIREMENTS.md), [TZ_COMPLIANCE.md](TZ_COMPLIANCE.md), [ORGANIZER_ANSWERS.md](ORGANIZER_ANSWERS.md) |
| данные | [DATASET_README.md](DATASET_README.md) (файл организаторов), [DATA.md](DATA.md) |
| модель и результаты | [../MODEL.md](../MODEL.md), [EVAL.md](EVAL.md), [POSITION_FRAME.md](POSITION_FRAME.md), [VEHICLES.md](VEHICLES.md), [ROBUST.md](ROBUST.md) |
| симулятор | [SANDBOX.md](SANDBOX.md), [../simulator/README.md](../simulator/README.md) |

Рабочие материалы команды лежат в [internal/](internal/README.md) и к оценке решения не
относятся.

## Какая команда пишет какой файл

Команды выполняются из корня репозитория; полный порядок - раздел
[Воспроизведение по шагам](../README.md#воспроизведение-по-шагам) в README.

| Файл | Команда |
|---|---|
| [EVAL.md](EVAL.md), графики `img/eval_*.png` | `tools/eval.py` (в образе: `docker compose run --rm eval`) |
| `data/eval_gnss*/EVAL_GNSS.md` | `tools/eval_gnss.py` |
| таблица сценариев в [SANDBOX.md](SANDBOX.md) | `node simulator/test/sandbox_report.js --write` |
| `out/data/` (таблицы для [DATA.md](DATA.md)) | `tools/data_audit.py` |
| `out/realtime/<тег>/summary.{json,md}` (сводки замера реального времени) | `tools/measure_realtime.sh` |
| таблица «Лист параметров» в [../MODEL.md](../MODEL.md) и листы ноды | `ros2_ws/src/tram_state_estimator/tools/gen_params.py` |
