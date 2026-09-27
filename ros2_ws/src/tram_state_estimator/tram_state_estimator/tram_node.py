"""Нода резервной оценки скорости и положения трамвая — по контракту ТЗ.

Входы (только они — в основном контуре):
    /vehicle/front_bogie_velocity   tram_vehicle_msgs/VelocitySensor  (км/ч)
    /vehicle/rear_bogie_velocity    tram_vehicle_msgs/VelocitySensor  (км/ч)
    /vehicle/driver_position_cmd    tram_vehicle_msgs/DriverControllerCommand
Начальная выставка (первые init_window_s секунд) и коррекция положения по
GNSS в середине маршрута (gnss_correction, по умолчанию включено; ответ
организаторов 26.09: такие сообщения можно использовать для коррекции):
    /sensing/gnss/master/fix, /sensing/gnss/rover/fix   sensor_msgs/NavSatFix
Выходы:
    /result/velocity   tram_vehicle_msgs/VelocitySensor   скорость, м/с
    /result/position   nav_msgs/Odometry                  x, y, z, м: по умолчанию
                       плоские координаты MGRS от угла квадрата 37UCB непрерывно
                       (x — восток, y — север, z — высота уровня рельса) точки
                       base_link (ось передней тележки); frame_id "map" — эта
                       система, child_frame_id "base_link"; см. параметры
                       projection, mgrs_grid, output_point
    /result/acceleration geometry_msgs/AccelStamped        ускорение, м/с²
    /tram/estimator_status tram_msgs/EstimatorStatus       состояние оценщика

Время — header.stamp входных сообщений (время из bag), не часы ноды. Ядро
шагает на сетке p.dt по этим меткам (runner.Runner); каждый шаг публикуется с
меткой своего момента. Та же связка работает в офлайн-оценке
(tools/eval_replay.py), поэтому числа оценки и работа ноды совпадают.

Устойчивость (docs/ROBUST.md):
  * лист: config/tram.yaml пакета всегда подкладывается под параметры
    запуска; без --params-file нода работает по нему целиком;
  * битые входы и разрывы времени отсекает Runner; исключение в колбэке не
    роняет ноду (лог с ограничением частоты), launch перезапускает её при
    падении (respawn);
  * пульс: если входы молчат, таймер по монотонным часам публикует прогноз
    на копии связки (состояние не меняется) не дальше pulse_horizon_s от
    последней метки (0,2 с; почему короткий — _extrapolate); метки выхода
    не идут назад; темп проигрывания (play -r) оценивается по приросту
    меток;
  * старт: первые start_sort_s с по часам прихода сообщения сортируются по
    метке, сетка начинается с самой ранней.

Консоль: при запуске — блок настроек (лист, вагон, карта, система выхода,
GNSS, шаг); во время работы — строка состояния раз в STATUS_EVERY_S с
времени bag и предупреждения только о настоящих отклонениях (с ограничением
частоты); при останове — итог (входы, выходы, отброшенное, сбросы, время
шага). Логирование на оценку не влияет.
"""

import hashlib
import math
import os
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import AccelStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from tram_msgs.msg import EstimatorStatus
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from .estimator_core import Params
from .estimator_node import declare_core_params
from .runner import Runner, StartSorter
from .track_map import TrackMap
from . import vehicle as vehicle_sheet

IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=50)
PKG = "tram_state_estimator"
SHEET = os.path.join("config", "tram.yaml")
IN_TOPICS = ("/vehicle/front_bogie_velocity", "/vehicle/rear_bogie_velocity",
             "/vehicle/driver_position_cmd", "/sensing/gnss/master/fix",
             "/sensing/gnss/rover/fix")
# Ковариации Odometry: диагонали, которые ядро не оценивает, заданы
# физически осмысленными конечными значениями, а не нулём («известно точно»).
ROLL_SD = 0.05          # рад: возвышение наружного рельса в кривой до ~3°
UNKNOWN_VAR = 1.0e6     # курс неизвестен (якорь есть, курса ещё нет)
Z_MAP_SD = 1.0          # м: высота из карты (усреднённый GNSS)
YAW_MAP_SD = 0.05       # рад: курс по оси пути карты (~3°)
LAT_V_SD = 0.05         # м/с: боковая скорость на рельсах ≈ 0
ANG_RATE_SD = 0.02      # рад/с: крен и тангаж почти не меняются
R_CURVE_MIN = 20.0      # м: наименьший радиус кривой трамвая — предел рыскания
FALLBACK_SD = 10.0      # м: запасная выставка после сброса — путь, потерянный,
                        # пока новое ядро догоняло скорость (~1 с на 10 м/с)
# параметры коррекции по GNSS: имена совпадают с аргументами runner.Position
GNSS_PARAMS = ("gnss_correction", "gnss_sigma_rtk_m", "gnss_sigma_sbas_m",
               "gnss_sigma_fix_m", "gnss_gate", "gnss_jump_m", "gnss_confirm_n",
               "gnss_min_interval_s", "gnss_max_skew_s", "gnss_prior_rel", "gnss_scale_adapt",
               "gnss_stop_skip_m", "gnss_persist_s")
# консоль
STATUS_EVERY_S = 10.0   # с времени bag между строками состояния
NO_POS_WARN_S = 10.0    # с после окна выставки без положения — предупреждение
WARN_THROTTLE_S = 5.0   # с: одно и то же предупреждение не чаще
# режимы ядра (estimator_core.MODE_NAMES) по-русски, как на странице симулятора
MODE_RU = ("выбег", "тяга", "торможение", "смена режима", "срыв сцепления",
           "стоянка", "нет данных колёс")


def fix_var(m):
    """Заявленная дисперсия положения NavSatFix по горизонтали, м²; None —
    не заявлена (COVARIANCE_TYPE_UNKNOWN или нули, как в данных кейса)."""
    try:
        if int(m.position_covariance_type) <= 0:
            return None
        c = m.position_covariance
        v = 0.5 * (float(c[0]) + float(c[4]))
        return v if math.isfinite(v) and v > 0.0 else None
    except (AttributeError, IndexError, TypeError, ValueError):
        return None


def package_info():
    """(версия из package.xml, отпечаток кода) для строки запуска. Отпечаток —
    sha256 модулей пакета по имени и содержимому, первые 16 знаков: так же
    считается «Код пакета: sha» в шапке docs/EVAL.md."""
    ver = "?"
    try:
        import xml.etree.ElementTree as ET
        from ament_index_python.packages import get_package_share_directory
        xml = os.path.join(get_package_share_directory(PKG), "package.xml")
        ver = (ET.parse(xml).getroot().findtext("version") or "?").strip()
    except Exception:                            # noqa: BLE001
        pass
    h = hashlib.sha256()
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        for name in sorted(f for f in os.listdir(here) if f.endswith(".py")):
            h.update(name.encode())
            with open(os.path.join(here, name), "rb") as fh:
                h.update(fh.read())
        sha = h.hexdigest()[:16]
    except OSError:
        sha = "?"
    return ver, sha


def _sheet_section(doc, node):
    """Секция листа для ноды: её полное имя, имя, «/**»; иначе секция имени
    ноды пакета по умолчанию (нода переименована через __node:=); иначе
    единственная секция листа. Возвращает (ключ, параметры) или (None, {})."""
    name = node.get_name()
    fqn = node.get_fully_qualified_name()
    keys = (fqn, fqn.lstrip("/"), f"/{name}", name, "/**", f"/{PKG}", PKG)
    for k in keys:
        if isinstance(doc.get(k), dict) and "ros__parameters" in doc[k]:
            return k, doc[k]["ros__parameters"] or {}
    sects = [k for k, v in doc.items()
             if isinstance(v, dict) and "ros__parameters" in v]
    if len(sects) == 1:
        return sects[0], doc[sects[0]]["ros__parameters"] or {}
    return None, {}


def use_package_sheet(node):
    """Без --params-file (ros2 run) нода молча брала бы заглушки Params:
    4 оси, rad_s, шаг 10 мс, без карты. Поэтому лист config/tram.yaml пакета
    всегда подкладывается под параметры запуска: ключи из --params-file и -p
    остаются, недостающие берутся из листа, а не из заглушек Params (один -p
    не превращает остальные параметры в заглушки). Параметр sheet: auto (так),
    путь к листу или none (заглушки Params, имитатор). Возвращает строку для
    лога: какой лист и сколько ключей из него взято."""
    ov = node._parameter_overrides      # rclpy Humble: --params-file и -p
    sheet = ov["sheet"].value if "sheet" in ov else "auto"
    core = {f for f in Params.__dataclass_fields__}
    if sheet == "none":
        return "заглушки Params (sheet: none)"
    path = SHEET if sheet == "auto" else sheet
    try:
        if not os.path.isabs(path):
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory(PKG), path)
        import yaml
        with open(path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        sect, vals = _sheet_section(doc, node)
    except Exception as e:                       # noqa: BLE001
        node.get_logger().warn(f"лист {path} не прочитан ({e}): заглушки Params")
        return f"заглушки Params (лист {path} не прочитан)"
    if sect is None:
        node.get_logger().warn(
            f"в листе {path} нет секции для ноды "
            f"{node.get_fully_qualified_name()}: заглушки Params")
        return f"заглушки Params (в листе {path} нет секции ноды)"
    given = core & set(ov)
    n = 0
    for k, v in vals.items():
        if k not in ov:
            if isinstance(v, list) and any(isinstance(x, float) for x in v):
                v = [float(x) for x in v]        # [0, 0.5] — один тип
            ov[k] = Parameter(k, value=v)
            n += 1
    if not given:
        how = "без --params-file: лист пакета"
    elif n:
        how = (f"под параметрами запуска: из листа {n} ключей, "
               f"параметров ядра из запуска {len(given)}")
    else:
        how = f"все ключи листа заданы при запуске через --params-file или -p, параметров ядра {len(given)}"
    return f"{path} [{sect}] ({how})"


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def to_msg(t):
    m = TimeMsg()
    m.sec = int(math.floor(t))
    m.nanosec = int(round((t - m.sec) * 1e9))
    if m.nanosec >= 1_000_000_000:
        m.sec, m.nanosec = m.sec + 1, m.nanosec - 1_000_000_000
    return m


def _map_line(tmap, path):
    """Строка лога о карте путей."""
    if tmap is None:
        return ("карты нет (map_file пуст): положение по пути колёс от точки "
                "выставки, nomap_mode")
    parts = [f"точек осей {len(tmap.lat)}"]
    for n, what in ((len(tmap.stops), "остановок"), (len(tmap.terminals), "конечных"),
                    (len(tmap.bi), "веток у конечных")):
        if n:
            parts.append(f"{what} {n}")
    return f"карта есть: {path} ({', '.join(parts)})"


class TramEstimatorNode(Node):
    def __init__(self):
        super().__init__("tram_state_estimator")
        self.sheet_src = use_package_sheet(self)          # до объявлений параметров
        P = self.declare_parameter
        P("wheel_timeout_s", 1.0)
        P("handle_timeout_s", 0.5)
        P("init_window_s", 3.0)
        P("map_file", "")
        P("origin_lat", float("nan"))      # NaN — начало в первой точке GNSS
        P("origin_lon", float("nan"))
        P("origin_alt", float("nan"))
        P("frame_id", "map")
        P("child_frame_id", "base_link")
        # выходная система /result/position (docs/POSITION_FRAME.md):
        # mgrs | utm | enu | equirect; mgrs_grid "37UCB" — непрерывно от угла
        # этого квадрата (так записана карта организаторов pathgraph: x
        # переходит 100 000 на E = 400 км плавно), "" — каждая точка в своём
        # 100-км квадрате (Autoware)
        P("projection", "mgrs")
        P("mgrs_grid", "37UCB")
        P("utm_zone", 0)                   # 0 — по точке выставки
        P("mgrs_guard_m", 0.0)             # только при mgrs_grid "": не публиковать у края квадрата
        # точка выхода: base_link (ось передней тележки, уровень рельса — как
        # эталон судьи и pathgraph) | master (антенна); антенны в base_link, м
        P("output_point", "base_link")
        P("antenna_master_x", -9.873)
        P("antenna_rover_x", 2.563)
        P("antenna_z", 3.0)
        P("scale_adapt", True)             # онлайн-масштаб пути по остановкам
        # вагон (docs/VEHICLES.md): 30618 | 30639 — масштаб колёс этого вагона
        # из таблицы листа; auto — общий лист; незнакомое — auto с предупреждением.
        # dynamic_typing: `-p vehicle:=30639` и launch дают целое, а не строку
        P("vehicle", "30618", descriptor=ParameterDescriptor(dynamic_typing=True))
        P("vehicle_ids", ["30618", "30639"],
          descriptor=ParameterDescriptor(dynamic_typing=True))
        P("vehicle_meas_scale", [1.001362, 0.997575],
          descriptor=ParameterDescriptor(dynamic_typing=True))
        # онлайн-масштаб колёс по привязкам к остановкам: поправка скорости
        # при смене масштаба по датам (vehicle.OnlineWheelScale)
        P("wheel_scale_online", True)
        P("nomap_mode", "hold")            # без карты: hold (стоять в якоре) | line
        P("keep_offset_xy", True)          # сдвиг GNSS окна − карта в выходе,
        P("keep_offset_z", True)           # если медиана статуса окна ≤
        P("keep_offset_max_status", -1)    # этого: −1 никогда, 1 без RTK, 2 всегда
        P("terminal_hold", "terminals")    # тупик карты: terminals | off | any
        # коррекция по GNSS после окна выставки; false — GNSS только для
        # выставки (точки после окна отбрасываются)
        P("gnss_correction", True)
        P("gnss_sigma_rtk_m", 0.5)         # σ точки при NavSatFix.status 2 (RTK)
        P("gnss_sigma_sbas_m", 1.5)        # status 1
        P("gnss_sigma_fix_m", 5.0)         # status 0 (без поправок: смещение до ~16 м)
        P("gnss_gate", 3.0)                # ворота невязки, σ
        P("gnss_jump_m", 3.0)              # поправка больше — только после подтверждения
        P("gnss_confirm_n", 3)             # эпох RTK подряд, согласных между собой (без RTK — только малые)
        P("gnss_min_interval_s", 1.0)      # поправки не чаще
        P("gnss_max_skew_s", 0.3)          # метка GNSS против метки последнего входа
        P("gnss_prior_rel", 0.003)         # априори поправки: рост σ на метр пути
        P("gnss_scale_adapt", False)       # масштаб пути по отрезкам между поправками
        P("gnss_stop_skip_m", 0.0)         # после поправки столько м без привязки к остановке
        P("gnss_persist_s", 10.0)          # неправдоподобная невязка RTK держится столько — верим
        g = lambda n: self.get_parameter(n).value

        params = declare_core_params(self, include_dt=True)
        params, self.vehicle_info = vehicle_sheet.apply(
            params, g("vehicle"), g("vehicle_ids"), g("vehicle_meas_scale"))
        if self.vehicle_info["warning"]:
            self.get_logger().warn(self.vehicle_info["warning"])
        tmap = None
        path = g("map_file")
        if path:
            if not os.path.isabs(path):
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(
                    get_package_share_directory("tram_state_estimator"), path)
            tmap = TrackMap.load(path)
        o = (g("origin_lat"), g("origin_lon"), g("origin_alt"))
        origin = o if all(math.isfinite(x) for x in o) else None
        self.runner = Runner(params, track_map=tmap, origin=origin,
                             wheel_timeout=g("wheel_timeout_s"),
                             handle_timeout=g("handle_timeout_s"),
                             init_window=g("init_window_s"),
                             projection=g("projection"), mgrs_grid=g("mgrs_grid"),
                             utm_zone=g("utm_zone"), mgrs_guard_m=g("mgrs_guard_m"),
                             scale_adapt=g("scale_adapt"), nomap_mode=g("nomap_mode"),
                             keep_offset_xy=g("keep_offset_xy"),
                             keep_offset_z=g("keep_offset_z"),
                             keep_offset_max_status=g("keep_offset_max_status"),
                             terminal_hold=g("terminal_hold"),
                             output_point=g("output_point"),
                             antenna_master_x=g("antenna_master_x"),
                             antenna_rover_x=g("antenna_rover_x"),
                             antenna_z=g("antenna_z"),
                             **{n: g(n) for n in GNSS_PARAMS})
        vehicle_sheet.wheel_scale_hook(self.runner, g("wheel_scale_online"))
        self.frame_id, self.child = g("frame_id"), g("child_frame_id")
        self.frame = 0
        self._robust_setup()
        self._console_setup()

        # Все входы идут через _input: ошибки и порядок старта.
        def sub(mtype, topic, name, args):
            self.create_subscription(
                mtype, topic, lambda m: self._input(name, args(m)), IN_QOS)
        sub(VelocitySensor, "/vehicle/front_bogie_velocity", "on_wheel",
            lambda m: (0, to_sec(m.header.stamp), m.velocity))
        sub(VelocitySensor, "/vehicle/rear_bogie_velocity", "on_wheel",
            lambda m: (1, to_sec(m.header.stamp), m.velocity))
        sub(DriverControllerCommand, "/vehicle/driver_position_cmd", "on_handle",
            lambda m: (to_sec(m.header.stamp), m.position))
        for ant in ("master", "rover"):
            sub(NavSatFix, f"/sensing/gnss/{ant}/fix", "on_fix",
                lambda m, a=ant: (to_sec(m.header.stamp), a, m.latitude,
                                  m.longitude, m.altitude, m.status.status,
                                  fix_var(m)))

        self.pub_v = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.pub_p = self.create_publisher(Odometry, "/result/position", 10)
        self.pub_a = self.create_publisher(AccelStamped, "/result/acceleration", 10)
        self.pub_s = self.create_publisher(EstimatorStatus,
                                           "/tram/estimator_status", 10)
        self._log_start(params, tmap, path, g)

    def _log_start(self, params, tmap, path, g):
        """Блок настроек при запуске. Первая строка («оценщик запущен») —
        признак готовности для скриптов и инструкции жюри."""
        ver, sha = package_info()
        grid = g("mgrs_grid")
        if g("projection") == "mgrs":
            frame = (f"mgrs от квадрата {grid} непрерывно" if grid else
                     "mgrs (MGRS: каждая точка в своём 100-км квадрате)")
        else:
            frame = str(g("projection"))
        point = g("output_point") + (f" (карта по {tmap.point})" if tmap is not None else "")
        corr = "вкл" if g("gnss_correction") else "выкл"
        lines = [
            f"оценщик запущен: {PKG} {ver}, код sha {sha}",
            f"  лист: {self.sheet_src}",
            f"  {vehicle_sheet.describe(self.vehicle_info, g('wheel_scale_online'))}",
            f"  {_map_line(tmap, path)}",
            f"  выход {frame}; точка {point}; frame_id {self.frame_id}, "
            f"child_frame_id {self.child}",
            f"  GNSS: выставка по первым {float(g('init_window_s')):.1f} с; "
            f"коррекция по GNSS после окна {corr} (только положение вдоль пути, "
            "скорость от GNSS не зависит)",
            f"  шаг {params.dt * 1000:.0f} мс ({1.0 / params.dt:.0f} Гц) по header.stamp "
            f"входов; единицы {params.meas_units}; пульс {self.pulse_h:g} с; "
            "QoS входов best-effort",
            f"  жду входы: {', '.join(IN_TOPICS)}",
        ]
        for line in lines:
            self.get_logger().info(line)

    # ---------- устойчивость ----------

    def _robust_setup(self):
        P = self.declare_parameter
        g = lambda n: self.get_parameter(n).value               # noqa: E731
        P("sheet", "auto")                  # auto | путь к листу | none
        P("pulse_horizon_s", 0.2)           # с от последней метки; 0 — без пульса
        P("pulse_margin_s", 0.1)            # с: узел просрочен — прогноз (ручка есть)
        P("pulse_margin_nohandle_s", 0.03)  # с: то же без ручки (сетка от 10 Гц)
        P("pulse_period_s", 0.01)           # с: период таймера по монотонным часам
        P("start_sort_s", 0.1)              # с: сортировка стартового всплеска
        self.pulse_h = float(g("pulse_horizon_s"))
        self.margin = float(g("pulse_margin_s"))
        self.margin_nh = float(g("pulse_margin_nohandle_s"))
        self._sorter = StartSorter(float(g("start_sort_s")))
        self._last_pub = -math.inf          # метка последнего выхода: не назад
        self._stamp_ref = None              # наибольшая принятая метка входа
        self._mono_ref = 0.0                # и когда она пришла (монотонные часы)
        self.rate = 1.0                     # темп проигрывания: метки / монотонные с
        self._rate_anchor = None            # (монотонные, метка) начала окна оценки
        self._fork = None                   # копия связки для прогноза
        self._errors = 0                    # ошибок подряд в колбэках
        self._good = (0.0, 0.0, 0.0)        # последнее конечное положение
        self.n_pulse = self.n_suppressed = self.n_errors = 0
        # Таймер по монотонным часам: с use_sim_time без /clock таймер ROS
        # не сработал бы ни разу.
        self.create_timer(max(float(g("pulse_period_s")), 1e-3), self._pulse,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _console_setup(self):
        """Счётчики для строки состояния и итога. На оценку не влияют."""
        self.n_msgs = {"on_wheel": 0, "on_handle": 0, "on_fix": 0}
        self.n_pos = 0                      # опубликовано /result/position
        self.step_us_sum = 0.0              # время шага связки: сумма и максимум
        self.step_us_max = 0.0
        self.t_first = None                 # метка первого выхода прогона
        self.t_last = None                  # метка последнего выхода
        self._st_next = None                # метка следующей строки состояния
        self._st_n = self._st_pulse = 0     # выходов и прогнозов пульса в окне
        self._st_t0 = None                  # начало окна строки состояния
        self._pos_seen = False              # было ли положение в этом прогоне
        self._no_pos_warned = False
        self._wall0 = time.monotonic()

    def _input(self, name, args):
        """Колбэк входа. Исключение не выходит в rclpy.spin: нода живёт."""
        try:
            now = time.monotonic()
            self.n_msgs[name] = self.n_msgs.get(name, 0) + 1
            stamp = args[1] if name == "on_wheel" else args[0]
            for item in self._sorter.push(now, stamp, (name, args)):
                self._dispatch(now, *item)
        except Exception as e:                   # noqa: BLE001
            self._fail(name, e)

    def _dispatch(self, now, name, args):
        r = self.runner
        n_in, resets, core_resets, gaps = r.n_in, r.resets, r.core_resets, r.gaps
        t0 = time.perf_counter_ns()
        outs = getattr(r, name)(*args)
        call_us = (time.perf_counter_ns() - t0) / 1000.0
        if r.resets != resets:
            self.get_logger().warn(f"сброс связки: {r.reset_reason}")
            self._last_pub = -math.inf           # новый прогон: метки заново
            self._stamp_ref = self._rate_anchor = None
            self._new_run()
        if r.gaps != gaps:
            self.get_logger().warn(r.gap_reason, throttle_duration_sec=WARN_THROTTLE_S)
            self._rate_anchor = None
        if r.core_resets != core_resets:
            self.get_logger().warn(r.reset_reason, throttle_duration_sec=WARN_THROTTLE_S)
        if r.n_in != n_in:                       # принято новое сообщение
            self._fork = None
            if self._stamp_ref is None or r.stamp_max > self._stamp_ref:
                self._rate_update(now, r.stamp_max)
                self._stamp_ref, self._mono_ref = r.stamp_max, now
        self._emit(outs, call_us)
        self._errors = 0

    RATE_WIN_S = 2.0        # с монотонных часов: окно оценки темпа
    RATE_HOLE_S = 1.0       # с тишины: пауза, а не темп — окно начинается заново

    def _rate_update(self, now, stamp):
        """Темп проигрывания для пульса: прирост наибольшей метки за окно
        монотонных часов, в пределах [0,05; 1]. Быстрее 1x пульс не нужен
        (входы приходят чаще), медленнее (play -r 0.5) без оценки пульс
        выдавал бы прогноз вместо почти всех выходов. Окно с паузой входов
        темп не меняет."""
        a = self._rate_anchor
        if a is None or self._stamp_ref is None \
                or now - self._mono_ref > self.RATE_HOLE_S:
            self._rate_anchor = (now, stamp)
            return
        if now - a[0] >= self.RATE_WIN_S:
            self.rate = min(1.0, max(0.05, (stamp - a[1]) / (now - a[0])))
            self._rate_anchor = (now, stamp)

    def _fail(self, where, e):
        import traceback
        self._errors += 1
        self.n_errors += 1
        tb = traceback.format_exc(limit=4).strip().splitlines()
        self.get_logger().error(
            f"ошибка в {where} ({self.n_errors} всего): {type(e).__name__}: {e}"
            f" | {' / '.join(x.strip() for x in tb[-3:])}",
            throttle_duration_sec=WARN_THROTTLE_S)
        if self._errors >= 20:
            # что-то застряло: только ядро заново (путь, выставка и сетка
            # остаются; полный сброс потерял бы выставку до конца прогона)
            self._errors = 0
            self.runner.reset_core("20 ошибок подряд в колбэках")
            self._fork = None
            self.get_logger().warn(self.runner.reset_reason)

    def _pulse(self):
        try:
            now = time.monotonic()
            for item in self._sorter.poll(now):
                self._dispatch(now, *item)
            self._extrapolate(now)
        except Exception as e:                   # noqa: BLE001
            self._fail("пульс", e)

    def _extrapolate(self, now):
        """Входы молчат: публикуются узлы сетки, просроченные на margin
        по часам (метка последнего входа + прошедшее время), не дальше
        pulse_horizon_s от этой метки. Считаются на копии связки: когда входы
        вернутся, связка продолжит со своего состояния, а её узлы, уже
        выданные прогнозом, второй раз не публикуются (метки не идут назад).
        Поэтому горизонт короткий: после паузы записи входы приходят с
        опозданием, и вход с меткой занятого прогнозом узла ждёт первого
        нового узла — до горизонта по меткам (запись 30618_af7496f0 с паузой
        входов 0,95 с, docs/EVAL.md §7)."""
        r = self.runner
        if self.pulse_h <= 0.0 or r.t is None or self._stamp_ref is None:
            return
        if r.backlog():                     # связка догоняет провал: узлы уже есть
            return
        alive = (r.t - r.t_notch) <= r.handle_timeout
        margin = self.margin if alive else self.margin_nh
        est = self._stamp_ref + self.rate * (now - self._mono_ref)
        target = min(est - margin, self._stamp_ref + self.pulse_h)
        if target + 1e-6 < max(r.next_node(), self._last_pub + r.p.dt):
            return
        t0 = time.perf_counter_ns()
        if self._fork is None:
            self._fork = r.fork()
        outs = self._fork.tick(target)
        n = self._emit(outs, (time.perf_counter_ns() - t0) / 1000.0, pulse=True)
        self.n_pulse += n

    def summary(self):
        """Итог работы ноды (печатается при останове)."""
        r = self.runner
        n = max(1, self.frame - self.n_pulse)
        span = (self.t_last - self.t_first) if self.t_first is not None else 0.0
        return "\n".join([
            f"итог: выходов {self.frame} (/result/position {self.n_pos}), "
            f"из них прогноз пульса {self.n_pulse}, подавлено повторов "
            f"{self.n_suppressed}; время bag {span:.1f} с, по часам "
            f"{time.monotonic() - self._wall0:.1f} с",
            f"  входы: тележки {self.n_msgs.get('on_wheel', 0)}, ручка "
            f"{self.n_msgs.get('on_handle', 0)}, GNSS {self.n_msgs.get('on_fix', 0)}; "
            f"отброшено меток {r.rejected_stamps}, значений {r.rejected_values}",
            f"  сбросов связки {r.resets}, ядра {r.core_resets}, провалов входов "
            f"{r.gaps}; ошибок в колбэках {self.n_errors}; темп {self.rate:.2f}",
            f"  время шага связки: среднее {self.step_us_sum / n / 1000.0:.2f} мс, "
            f"макс {self.step_us_max / 1000.0:.2f} мс",
        ])

    # ---------- консоль ----------

    def _new_run(self):
        """Сброс связки (второй bag, --loop): строка состояния и проверка
        выставки считаются заново."""
        self.t_first = self._st_next = self._st_t0 = None
        self._st_n = self._st_pulse = 0
        self._pos_seen = self._no_pos_warned = False

    def _console(self, o, pulse):
        """Строка состояния раз в STATUS_EVERY_S с времени bag и события
        выставки. Ошибка здесь не должна влиять на выход — глотается."""
        try:
            t = float(o["stamp"])
            if self.t_first is None:
                self.t_first = self._st_t0 = t
                self._st_next = t + STATUS_EVERY_S
                self.get_logger().info(
                    f"входы пошли: первый выход на метке {t:.2f}; это t+0, "
                    "дальше время bag — от неё")
            self.t_last = t
            self._st_n += 1
            self._st_pulse += int(pulse)
            if o.get("pos_valid", True) and not self._pos_seen:
                self._pos_seen = True
                pos = self.runner.pos
                how = ("запасная выставка прежнего прогона" if o.get("pos_fallback")
                       else f"выставка по GNSS: точек {pos.n_used}, курс "
                            f"{'есть' if pos.ready else 'ещё нет'}")
                self.get_logger().info(
                    f"положение есть с метки {t:.2f}, t+{t - self.t_first:.1f} с ({how}): "
                    f"x {float(o['x']):.1f}, "
                    f"y {float(o['y']):.1f}, z {float(o['z']):.1f}")
            elif (not self._pos_seen and not self._no_pos_warned
                  and t - self.t_first > self.runner.pos.init_window + NO_POS_WARN_S):
                self._no_pos_warned = True
                self.get_logger().warn(
                    f"нет выставки по GNSS за {t - self.t_first:.0f} с: /result/position "
                    "не публикуется, пока не придёт GNSS (/sensing/gnss/*/fix); "
                    "/result/velocity идёт")
            if t + 1e-6 >= self._st_next:
                self.get_logger().info(self._status_line(o, t))
                self._st_t0, self._st_next = t, t + STATUS_EVERY_S
                self._st_n = self._st_pulse = 0
        except Exception:                        # noqa: BLE001
            pass

    def _status_line(self, o, t):
        span = max(t - self._st_t0, 1e-6)
        mode = int(o["mode"])
        v = float(o["v"])
        flags = []
        if o.get("wheels_stale"):
            flags.append("колёса молчат")
        sa = int(o.get("slip_all", 0))
        if sa:
            flags.append("срыв всех осей: " + ("буксование" if sa > 0 else "юз"))
        elif o.get("slip"):
            flags.append("срыв")
        if o.get("ambiguous"):
            flags.append("стоим или скользим?")
        if not o.get("valid", True):
            flags.append("недостоверно")
        if o.get("pos_valid", True) and o.get("pos_fallback"):
            flags.append("запасная выставка")
        if self._st_pulse:
            flags.append(f"прогноз пульса {self._st_pulse}")
        na, nr = int(o.get("n_accepted", 0)), int(o.get("n_rejected", 0))
        pos = (f"x {float(o['x']):.1f} y {float(o['y']):.1f} z {float(o['z']):.1f} "
               f"±{float(o['sigma_s']):.1f} м" if o.get("pos_valid", True)
               else "положения нет")
        # режим ядра (тяга или торможение) идёт от знака ручки, поэтому ручка
        # впереди: в части записей вагон разгоняется при «тормозной» позиции
        mode_s = "режим ядра: " + (MODE_RU[mode] if 0 <= mode < len(MODE_RU) else str(mode))
        notch = getattr(self.runner, "notch", None)
        if isinstance(notch, (int, float)) and math.isfinite(notch):
            mode_s = f"ручка {notch:+.0f}, {mode_s}"
        return (f"t+{t - self.t_first:.0f} с | {self._st_n / span:.1f} Гц | "
                f"{mode_s} | "
                f"v {v:.2f} м/с ({v * 3.6:.1f} км/ч) ±{float(o['sigma_v']):.2f} | "
                f"{pos} | тележки {na}/{na + nr} | "
                f"{', '.join(flags) if flags else 'норма'}")

    # ---------- публикация ----------

    def _emit(self, outs, call_us=0.0, pulse=False):
        """Публикует шаги по порядку меток; возвращает число опубликованных.
        step_time_us — время вызова связки (шаг ядра и карты), делённое на
        число шагов этого вызова."""
        n = 0
        per_us = call_us / max(1, len(outs))
        for o in outs:
            if o["stamp"] <= self._last_pub + 1e-6:
                self.n_suppressed += 1           # уже выдан прогнозом пульса
                continue
            self._publish(o, per_us)
            self._last_pub = o["stamp"]
            n += 1
            if not pulse:
                self.step_us_sum += per_us
                self.step_us_max = max(self.step_us_max, per_us)
            self._console(o, pulse)
        return n

    def _publish(self, o, step_us):
        st = to_msg(o["stamp"])
        v_ = float(o["v"])
        xyz = tuple(float(o[k]) for k in ("x", "y", "z"))
        # без якоря GNSS (или у края квадрата MGRS) положения в выходной
        # системе нет: /result/position не публикуется, а не выдаёт мусор
        pos_ok = bool(o.get("pos_valid", True))
        if not (math.isfinite(v_) and all(math.isfinite(c) for c in xyz)):
            # последний рубеж: NaN в выход не уходит (ядро сбрасывается само)
            self.get_logger().warn("неконечный выход заменён последним конечным",
                                   throttle_duration_sec=WARN_THROTTLE_S)
            v_ = v_ if math.isfinite(v_) else 0.0
            xyz = tuple(c if math.isfinite(c) else g
                        for c, g in zip(xyz, self._good))
        if pos_ok:
            self._good = xyz
        sv2 = float(o["sigma_v"]) ** 2
        ss2 = float(o["sigma_s"]) ** 2
        th = float(self.runner.p.theta_max)

        v = VelocitySensor()
        v.header.stamp, v.header.frame_id = st, self.child
        v.velocity = v_
        self.pub_v.publish(v)

        od = Odometry()
        od.header.stamp, od.header.frame_id = st, self.frame_id
        od.child_frame_id = self.child
        (od.pose.pose.position.x, od.pose.pose.position.y,
         od.pose.pose.position.z) = xyz
        yaw = o.get("yaw")
        if yaw is not None:
            od.pose.pose.orientation.z = math.sin(yaw / 2.0)
            od.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Ни одной нулевой диагонали. x, y — σ пути ядра (изотропно);
        # z — высота карты; крен — возвышение рельса; тангаж — уклон линии
        # (theta_max); курс — ось пути карты. Положение публикуется только с
        # якорем GNSS (pos_valid), поэтому x, y, z известны всегда; без курса
        # (якорь есть, выставка не полная) неизвестен только курс.
        pc = od.pose.covariance
        pc[0] = pc[7] = ss2 + FALLBACK_SD ** 2 if o.get("pos_fallback") else ss2
        pc[14] = Z_MAP_SD ** 2
        pc[21] = ROLL_SD ** 2
        pc[28] = th ** 2
        pc[35] = YAW_MAP_SD ** 2 if yaw is not None else UNKNOWN_VAR
        od.twist.twist.linear.x = v_
        # twist в base_link: вдоль — σ_v ядра; поперёк и вверх — рельсы
        # (вверх — скорость по уклону); угловые: крен и тангаж почти
        # постоянны, рыскание не оценивается (0) и ограничено v / R_min.
        tc = od.twist.covariance
        tc[0] = sv2
        tc[7] = LAT_V_SD ** 2
        tc[14] = (v_ * th) ** 2 + LAT_V_SD ** 2
        tc[21] = tc[28] = ANG_RATE_SD ** 2
        tc[35] = (v_ / R_CURVE_MIN) ** 2 + ANG_RATE_SD ** 2
        if pos_ok:
            self.pub_p.publish(od)
            self.n_pos += 1

        ac = AccelStamped()
        ac.header.stamp, ac.header.frame_id = st, self.child
        ac.accel.linear.x = float(o["a"])
        self.pub_a.publish(ac)

        s = EstimatorStatus()
        s.header.stamp, s.header.frame_id = st, self.child
        s.mode = int(o["mode"])
        s.v, s.s, s.d = v_, float(o["s"]), float(o["d"])
        s.k_traction, s.k_brake = float(o["k_t"]), float(o["k_b"])
        s.mu = float(o["mu"])
        s.sigma_v, s.sigma_s = float(o["sigma_v"]), float(o["sigma_s"])
        s.wheel_healthy = [bool(x) for x in o["healthy"]]
        s.axle_scale = [float(x) for x in self.runner.core.axle_scale]
        s.slip, s.ambiguous = bool(o["slip"]), bool(o["ambiguous"])
        s.slip_all = int(o.get("slip_all", 0))
        s.meas_noise = float(o.get("meas_noise", 0.0))
        s.n_accepted, s.n_rejected = int(o["n_accepted"]), int(o["n_rejected"])
        s.odometry_used = bool(o["odometry_used"])
        s.valid = bool(o["valid"]) and not o["wheels_stale"]
        self.frame += 1
        s.frame_count = self.frame
        s.sensor_stamp = st
        s.step_time_us = float(step_us)
        self.pub_s.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TramEstimatorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C в терминале приходит группе процессов, launch шлёт ещё один
        # SIGINT: второй не должен прервать останов трассировкой.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            try:            # контекст уже закрыт: в /rosout не пишется, только в консоль
                for line in node.summary().splitlines():
                    print(f"[tram_state_estimator] {line}", flush=True)
            except Exception:                    # noqa: BLE001
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
