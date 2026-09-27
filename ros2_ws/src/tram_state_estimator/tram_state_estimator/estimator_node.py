"""Нода резервного оценщика скорости и положения трамвая.

Тонкая обёртка над ядром `estimator_core`, которое не зависит от ROS.

Параметры ядра объявляются автоматически из описания `Params`: каждое поле
становится параметром ноды с тем же именем, и значение берётся из params.yaml.
Ручной копии списка параметров в ноде нет - поэтому подстановка констант из ТЗ
это правка одного yaml-файла.

Реальное время:
  * метка времени берётся у ДАТЧИКА, а не у момента публикации;
  * устаревшие входы - отказ: одометрия не подаётся ядру вообще, а не
    заменяется нулями (нули ядро приняло бы за остановку);
  * QoS best_effort глубиной 1: потеря обнаруживается, а не копится в очереди;
  * фиксированный шаг таймера, преаллокация, никакого логирования в цикле.
"""

import time
from dataclasses import fields

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

from tram_msgs.msg import EstimatorStatus

from .estimator_core import Estimator, Params, DEFAULT

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def declare_core_params(node: Node, include_dt: bool = False) -> Params:
    """Объявляет по параметру на каждое поле Params и собирает значения.

    dynamic_typing: без него ROS 2 Humble отвергает значение из yaml, если его
    тип не совпал с типом значения по умолчанию, - «M_nom: 30000» (целое) при
    объявленном 28000.0 (дробное) роняет ноду при старте. Типы приводит и
    проверяет Params.

    description: единицы, источник и смысл параметра видны в
    `ros2 param describe /tram_state_estimator <имя>`.
    """
    vals = {}
    for f in fields(Params):
        if f.name == "dt" and not include_dt:
            continue                      # задаётся частотой цикла
        default = getattr(DEFAULT, f.name)
        is_seq = isinstance(default, tuple)
        md = f.metadata
        desc = ParameterDescriptor(
            description=f"{md['doc']} [{md['unit']}; {md['src']}]",
            dynamic_typing=True)
        node.declare_parameter(f.name, list(default) if is_seq else default,
                               desc)
        v = node.get_parameter(f.name).value
        vals[f.name] = tuple(v) if is_seq else v
    try:
        return Params.from_dict(vals)
    except (KeyError, ValueError) as e:
        node.get_logger().fatal(str(e))
        raise


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("tram_state_estimator")

        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("input_timeout_s", 0.2)
        self.declare_parameter("frame_id", "tram_base")
        self.rate = float(self.get_parameter("rate_hz").value)
        self.timeout = float(self.get_parameter("input_timeout_s").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        # Имена датчиков в порядке листа (оси по порядку, по sensors_per_axle
        # на ось). Если заданы, показания берутся по msg.name, а не по
        # позиции: порядок в JointState не обязан совпадать с листом.
        # [""] - брать по позиции.
        self.declare_parameter("wheel_names", [""])
        self.wheel_names = [n for n in
                            self.get_parameter("wheel_names").value if n]

        params = declare_core_params(self).with_dt(1.0 / self.rate)
        self.core = Estimator(params)
        self.nw = self.core.nw          # длина вектора показаний по листу
        if self.wheel_names and len(self.wheel_names) != self.nw:
            msg = (f"wheel_names: {len(self.wheel_names)} имён, по листу "
                   f"ожидается {self.nw} датчиков")
            self.get_logger().fatal(msg)
            raise ValueError(msg)
        self._perm_key = None           # msg.name, для которого считана перестановка
        self._perm = None

        # преаллокация: в горячем пути ничего не выделяется
        self._meas = np.zeros(self.nw)
        self._fresh = False             # новые показания после прошлого шага
        self._notch = 0.0
        self._sensor_stamp = None
        self._last_wheels_t = None
        self._last_handle_t = None
        self._frame = 0
        self._step_us = 0.0

        self.create_subscription(JointState, "tram/wheel_speeds",
                                 self._on_wheels, SENSOR_QOS)
        self.create_subscription(Int8, "tram/handle",
                                 self._on_handle, SENSOR_QOS)

        self.pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, "tram/velocity", SENSOR_QOS)
        self.pub_odom = self.create_publisher(Odometry, "tram/odom", SENSOR_QOS)
        self.pub_status = self.create_publisher(
            EstimatorStatus, "tram/estimator_status", SENSOR_QOS)

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"оценщик запущен: {self.rate:.0f} Гц, датчиков {self.nw}, "
            f"единицы {params.meas_units}")

    # ---------- приём ----------

    def _on_wheels(self, msg: JointState):
        if len(msg.velocity) != self.nw:
            # Иначе недостающие показания остались бы нулями и выглядели бы
            # оборванными датчиками. Сообщение отбрасывается: данных нет.
            self.get_logger().error(
                f"в tram/wheel_speeds {len(msg.velocity)} показаний, по листу "
                f"ожидается {self.nw} (sensor_axles × sensors_per_axle)",
                throttle_duration_sec=5.0)
            return
        if self.wheel_names:
            if msg.name != self._perm_key:
                try:
                    idx = {n: i for i, n in enumerate(msg.name)}
                    self._perm = np.array([idx[n] for n in self.wheel_names])
                except KeyError as e:
                    self._perm = None
                    self.get_logger().error(
                        f"в tram/wheel_speeds нет датчика {e} из wheel_names",
                        throttle_duration_sec=5.0)
                self._perm_key = list(msg.name)
            if self._perm is None:
                return
            self._meas[:] = np.asarray(msg.velocity)[self._perm]
        else:
            self._meas[:] = msg.velocity
        self._fresh = True
        self._sensor_stamp = msg.header.stamp
        self._last_wheels_t = self._now()

    def _on_handle(self, msg: Int8):
        self._notch = float(msg.data)
        self._last_handle_t = self._now()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stale(self, last) -> bool:
        return last is None or (self._now() - last) > self.timeout

    # ---------- шаг ----------

    def _tick(self):
        wheels_stale = self._stale(self._last_wheels_t)
        handle_stale = self._stale(self._last_handle_t)

        t0 = time.perf_counter_ns()
        if wheels_stale:
            # Одометрии нет: коррекция пропускается, ядро работает разомкнуто.
            # Подставлять нули нельзя - это была бы «остановка».
            out = self.core.step_open_loop(self._notch)
        else:
            # Датчики могут приходить реже цикла: без нового сообщения - только
            # прогноз. При устаревшей ручке скорость по-прежнему корректируется
            # по колёсам, но параметры модели не адаптируются.
            out = self.core.step(self._notch, self._meas, fresh=self._fresh,
                                 handle_ok=not handle_stale)
        self._fresh = False
        self._step_us = (time.perf_counter_ns() - t0) / 1000.0

        self._frame += 1
        stamp = self._sensor_stamp if self._sensor_stamp is not None \
            else self.get_clock().now().to_msg()
        self._publish(out, stamp, wheels_stale or handle_stale)

    # ---------- публикация ----------

    def _publish(self, out, stamp: TimeMsg, stale: bool):
        var_v = float(out["sigma_v"]) ** 2
        var_s = float(out["sigma_s"]) ** 2

        tw = TwistWithCovarianceStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = self.frame_id
        tw.twist.twist.linear.x = float(out["v"])
        tw.twist.covariance[0] = var_v
        self.pub_vel.publish(tw)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.frame_id
        od.pose.pose.position.x = float(out["s"])
        od.pose.covariance[0] = var_s
        od.twist.twist.linear.x = float(out["v"])
        od.twist.covariance[0] = var_v
        self.pub_odom.publish(od)

        st = EstimatorStatus()
        st.header.stamp = stamp
        st.header.frame_id = self.frame_id
        st.mode = int(out["mode"])
        st.v = float(out["v"])
        st.s = float(out["s"])
        st.d = float(out["d"])
        st.k_traction = float(out["k_t"])
        st.k_brake = float(out["k_b"])
        st.mu = float(out["mu"])
        st.sigma_v = float(out["sigma_v"])
        st.sigma_s = float(out["sigma_s"])
        st.wheel_healthy = [bool(x) for x in out["healthy"]]
        st.axle_scale = [float(x) for x in self.core.axle_scale]
        st.slip = bool(out["slip"])
        st.ambiguous = bool(out["ambiguous"])
        st.n_accepted = int(out["n_accepted"])
        st.n_rejected = int(out["n_rejected"])
        st.odometry_used = bool(out["odometry_used"])
        st.valid = bool(out["valid"]) and not stale
        st.frame_count = self._frame
        st.sensor_stamp = stamp
        st.step_time_us = float(self._step_us)
        self.pub_status.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
