# tools/dev — исследовательские и проверочные скрипты разработки

Жюри эти скрипты не нужны: они не входят в пакет ROS 2, в `docker compose`
и в CI. Здесь лежат разовые исследования и проверки, по которым принимались
решения (их выводы — в `MODEL.md`, `docs/POSITION_FRAME.md`, `docs/ROBUST.md`).
Скрипты писались под свои версии кода; часть из них считает положение в старой
системе (equirect от первой точки GNSS), поэтому отчётные числа берутся только
из `tools/eval.py`.

Из основного кода используется один модуль: `core_metrics.py` — независимая
реализация метрик и базы «только колесо». С ней сверяются
`tools/eval_selftest.py` (самотест оценки) и `tools/export_replay.py` (ошибка
вдоль и поперёк траектории); `analysis/calib_eval.py` берёт из неё метрики
подгонки.

| Скрипт | Что делает |
|---|---|
| `core_metrics.py` | метрики скорости и положения, база «только колесо», инъекции |
| `critic_offline.py`, `critic_probe.py`, `critic_ros.sh` | проверки полноты: фазы движения, стартовый всплеск bag, `ros2 run` против `ros2 launch`, глубина очереди входов |
| `fuzz_runner.py` | связка `Runner` на синтетических и испорченных входах |
| `position_study.py` | исследование положения: проекции, онлайн-масштаб пути по остановкам |
| `ros_e2e_suite.sh`, `ros_e2e_table.py`, `ros_vs_offline.py` | серия прогонов ROS 2 end-to-end и сверка нода = офлайн |
| `ros_nan_check.py`, `ros_nan_check.sh` | нода на NaN и Inf во входах |
| `gnss_reanchor_check.py` | GNSS после окна выставки не двигает положение (при `gnss_correction: false`) |
| `export_replay_probe.py`, `record_current.py`, `replay_inputs_core.py`, `sim_headless.js`, `compare_port.js`, `port_variants.js`, `rosbridge_probe.js` | проверки симулятора и JS-порта |
| `adhoc_*.py` | разовые замеры: расхождение скорости с GNSS, CPU карты, медленные колбэки |
| `slip_compare.py` | сравнение двух прогонов `tools/eval.py` по инъекциям срыва |
| `map_probe.py`, `tune_q.py` | ранние пробы: карта из обучающих траекторий, перебор `q_v` и `sigma_meas` |
