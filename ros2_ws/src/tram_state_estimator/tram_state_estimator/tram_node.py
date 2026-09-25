"""Нода резервной оценки скорости и положения трамвая — по контракту ТЗ.

Входы (только они — в основном контуре):
    /vehicle/front_bogie_velocity   tram_vehicle_msgs/VelocitySensor  (км/ч)
    /vehicle/rear_bogie_velocity    tram_vehicle_msgs/VelocitySensor  (км/ч)
    /vehicle/driver_position_cmd    tram_vehicle_msgs/DriverControllerCommand
Начальная выставка (только первые init_window секунд и пока вагон стоит):
    /sensing/gnss/master/fix, /sensing/gnss/rover/fix   sensor_msgs/NavSatFix
Выходы:
    /result/velocity   tram_vehicle_msgs/VelocitySensor   скорость, м/с
    /result/position   nav_msgs/Odometry                  x, y, z в ENU, м
    /result/acceleration geometry_msgs/AccelStamped        ускорение, м/с²
    /tram/estimator_status tram_msgs/EstimatorStatus       состояние оценщика

Время — header.stamp входных сообщений (время из bag), не часы ноды. Ядро
шагает на сетке p.dt по этим меткам (runner.Runner); каждый шаг публикуется с
меткой своего момента. Та же связка используется в офлайн-оценке
analysis/evaluate.py, поэтому числа оценки и работа ноды совпадают.

Устойчивость (docs/ROBUST.md):
  * лист: config/tram.yaml пакета всегда подкладывается под параметры
    запуска; без --params-file нода работает по нему целиком (WP22);
  * битые входы и разрывы времени отсекает Runner; исключение в колбэке не
    роняет ноду (лог с ограничением частоты), launch перезапускает её при
    падении (respawn);
  * пульс (WP16): если входы молчат, таймер по монотонным часам публикует
    прогноз на копии связки (состояние не меняется) не дальше
    pulse_horizon_s от последней метки; метки выхода не идут назад; темп
    проигрывания (play -r) оценивается по приросту меток;
  * старт (WP24): первые start_sort_s с по часам прихода сообщения
    сортируются по метке, сетка начинается с самой ранней.
"""

import math
import os
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
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

IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=50)
PKG = "tram_state_estimator"
SHEET = os.path.join("config", "tram.yaml")
# Ковариации Odometry (WP12a): диагонали, которые ядро не оценивает, заданы
# физически осмысленными конечными значениями, а не нулём («известно точно»).
ROLL_SD = 0.05          # рад: возвышение наружного рельса в кривой до ~3°
UNKNOWN_VAR = 1.0e6     # до выставки: абсолютное положение неизвестно
Z_MAP_SD = 1.0          # м: высота из карты (усреднённый GNSS)
YAW_MAP_SD = 0.05       # рад: курс по оси пути карты (~3°)
LAT_V_SD = 0.05         # м/с: боковая скорость на рельсах ≈ 0
ANG_RATE_SD = 0.02      # рад/с: крен и тангаж почти не меняются
R_CURVE_MIN = 20.0      # м: наименьший радиус кривой трамвая — предел рыскания
FALLBACK_SD = 10.0      # м: запасная выставка после сброса — путь, потерянный,
                        # пока новое ядро догоняло скорость (~1 с на 10 м/с)


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
    """WP22. Без --params-file (ros2 run) нода молча брала бы заглушки Params:
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
    how = ("без --params-file: лист пакета" if not given else
           f"под параметрами запуска: из листа {n} ключей, "
           f"параметров ядра из запуска {len(given)}")
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


class TramEstimatorNode(Node):
    def __init__(self):
        super().__init__("tram_state_estimator")
        self.sheet_src = use_package_sheet(self)          # WP22: до объявлений
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
        g = lambda n: self.get_parameter(n).value

        params = declare_core_params(self, include_dt=True)
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
                             handle_timeout=g("handle_timeout_s"))
        self.runner.pos.init_window = g("init_window_s")
        self.frame_id, self.child = g("frame_id"), g("child_frame_id")
        self.frame = 0
        self._robust_setup()

        # Все входы идут через _input: ошибки и порядок старта (WP3, WP24).
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
                                  m.longitude, m.altitude))

        self.pub_v = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.pub_p = self.create_publisher(Odometry, "/result/position", 10)
        self.pub_a = self.create_publisher(AccelStamped, "/result/acceleration", 10)
        self.pub_s = self.create_publisher(EstimatorStatus,
                                           "/tram/estimator_status", 10)
        self.get_logger().info(
            f"оценщик запущен: шаг {params.dt * 1000:.0f} мс, "
            f"карта {'есть' if tmap is not None else 'нет'}; "
            f"лист: {self.sheet_src}; карта: {path or 'нет (map_file пуст)'}; "
            f"единицы {params.meas_units}; пульс {self.pulse_h:.1f} с")

    # ---------- устойчивость ----------

    def _robust_setup(self):
        P = self.declare_parameter
        g = lambda n: self.get_parameter(n).value               # noqa: E731
        P("sheet", "auto")                  # auto | путь к листу | none (WP22)
        P("pulse_horizon_s", 2.0)           # с от последней метки; 0 — без пульса
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

    def _input(self, name, args):
        """Колбэк входа. Исключение не выходит в rclpy.spin: нода живёт."""
        try:
            now = time.monotonic()
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
        if r.gaps != gaps:
            self.get_logger().warn(r.gap_reason)
            self._rate_anchor = None
        if r.core_resets != core_resets:
            self.get_logger().warn(r.reset_reason, throttle_duration_sec=5.0)
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
            throttle_duration_sec=5.0)
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
        """WP16. Входы молчат: публикуются узлы сетки, просроченные на margin
        по часам (метка последнего входа + прошедшее время), не дальше
        pulse_horizon_s от этой метки. Считаются на копии связки: когда входы
        вернутся, связка продолжит со своего состояния, а её узлы, уже
        выданные прогнозом, второй раз не публикуются (метки не идут назад)."""
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
        self.n_pulse += self._emit(outs, (time.perf_counter_ns() - t0) / 1000.0)

    def summary(self):
        r = self.runner
        return (f"итог: выходов {self.frame}, из них прогноз пульса "
                f"{self.n_pulse}, подавлено повторов {self.n_suppressed}; "
                f"сбросов связки {r.resets}, ядра {r.core_resets}, провалов "
                f"входов {r.gaps}; темп {self.rate:.2f}; отброшено "
                f"меток {r.rejected_stamps}, значений {r.rejected_values}; "
                f"ошибок в колбэках {self.n_errors}")

    # ---------- публикация ----------

    def _emit(self, outs, call_us=0.0):
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
        return n

    def _publish(self, o, step_us):
        st = to_msg(o["stamp"])
        v_ = float(o["v"])
        xyz = tuple(float(o[k]) for k in ("x", "y", "z"))
        if not (math.isfinite(v_) and all(math.isfinite(c) for c in xyz)):
            # последний рубеж: NaN в выход не уходит (ядро сбрасывается само)
            self.get_logger().warn("неконечный выход заменён последним конечным",
                                   throttle_duration_sec=5.0)
            v_ = v_ if math.isfinite(v_) else 0.0
            xyz = tuple(c if math.isfinite(c) else g
                        for c, g in zip(xyz, self._good))
        self._good = xyz
        ready = bool(o.get("pos_ready"))
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
        # WP12a: ни одной нулевой диагонали. x, y — σ пути ядра (изотропно);
        # z — высота карты; крен — возвышение рельса; тангаж — уклон линии
        # (theta_max); курс — ось пути карты. До выставки положение и курс
        # в абсолютной системе неизвестны.
        pc = od.pose.covariance
        pc[0] = pc[7] = ((ss2 + FALLBACK_SD ** 2 if o.get("pos_fallback") else ss2)
                         if ready else UNKNOWN_VAR)
        pc[14] = Z_MAP_SD ** 2 if ready else UNKNOWN_VAR
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
        self.pub_p.publish(od)

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
                print(f"[tram_state_estimator] {node.summary()}", flush=True)
            except Exception:                    # noqa: BLE001
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
