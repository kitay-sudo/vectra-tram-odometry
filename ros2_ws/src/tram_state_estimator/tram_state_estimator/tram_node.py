"""Нода резервной оценки скорости и положения трамвая — по контракту ТЗ.

Входы (только они — в основном контуре):
    /vehicle/front_bogie_velocity   tram_vehicle_msgs/VelocitySensor  (км/ч)
    /vehicle/rear_bogie_velocity    tram_vehicle_msgs/VelocitySensor  (км/ч)
    /vehicle/driver_position_cmd    tram_vehicle_msgs/DriverControllerCommand
Начальная выставка (только первые init_window секунд и пока вагон стоит):
    /sensing/gnss/master/fix, /sensing/gnss/rover/fix   sensor_msgs/NavSatFix
Выходы:
    /result/velocity   tram_vehicle_msgs/VelocitySensor   скорость, м/с
    /result/position   nav_msgs/Odometry                  x, y, z, м: по умолчанию
                       плоские координаты MGRS (x — восток, y — север в 100-км
                       квадрате, z — абсолютная высота), см. параметр projection
    /result/acceleration geometry_msgs/AccelStamped        ускорение, м/с²
    /tram/estimator_status tram_msgs/EstimatorStatus       состояние оценщика

Время — header.stamp входных сообщений (время из bag), не часы ноды. Ядро
шагает на сетке p.dt по этим меткам (runner.Runner); каждый шаг публикуется с
меткой своего момента. Та же связка используется в офлайн-оценке
analysis/evaluate.py, поэтому числа оценки и работа ноды совпадают.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import AccelStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from tram_msgs.msg import EstimatorStatus
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from .estimator_node import declare_core_params
from .runner import Runner
from .track_map import TrackMap

IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=50)


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
        # mgrs | utm | enu | equirect; mgrs_grid "" — каждая точка в своём
        # 100-км квадрате, "37UDB" — непрерывно от угла этого квадрата
        P("projection", "mgrs")
        P("mgrs_grid", "")
        P("utm_zone", 0)                   # 0 — по точке выставки
        P("mgrs_guard_m", 0.0)             # у края квадрата не публиковать (0 — выкл.)
        P("scale_adapt", True)             # онлайн-масштаб пути по остановкам
        P("nomap_mode", "hold")            # без карты: hold (стоять в якоре) | line
        P("keep_offset_xy", True)          # сдвиг GNSS окна − карта в выходе
        P("keep_offset_z", True)
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
                             handle_timeout=g("handle_timeout_s"),
                             init_window=g("init_window_s"),
                             projection=g("projection"), mgrs_grid=g("mgrs_grid"),
                             utm_zone=g("utm_zone"), mgrs_guard_m=g("mgrs_guard_m"),
                             scale_adapt=g("scale_adapt"), nomap_mode=g("nomap_mode"),
                             keep_offset_xy=g("keep_offset_xy"),
                             keep_offset_z=g("keep_offset_z"))
        self.frame_id, self.child = g("frame_id"), g("child_frame_id")
        self.frame = 0

        S = self.create_subscription
        S(VelocitySensor, "/vehicle/front_bogie_velocity",
          lambda m: self._emit(self.runner.on_wheel(0, to_sec(m.header.stamp),
                                                    m.velocity)), IN_QOS)
        S(VelocitySensor, "/vehicle/rear_bogie_velocity",
          lambda m: self._emit(self.runner.on_wheel(1, to_sec(m.header.stamp),
                                                    m.velocity)), IN_QOS)
        S(DriverControllerCommand, "/vehicle/driver_position_cmd",
          lambda m: self._emit(self.runner.on_handle(to_sec(m.header.stamp),
                                                     m.position)), IN_QOS)
        for ant in ("master", "rover"):
            S(NavSatFix, f"/sensing/gnss/{ant}/fix",
              lambda m, a=ant: self._emit(self.runner.on_fix(
                  to_sec(m.header.stamp), a, m.latitude, m.longitude,
                  m.altitude, m.status.status)), IN_QOS)

        self.pub_v = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.pub_p = self.create_publisher(Odometry, "/result/position", 10)
        self.pub_a = self.create_publisher(AccelStamped, "/result/acceleration", 10)
        self.pub_s = self.create_publisher(EstimatorStatus,
                                           "/tram/estimator_status", 10)
        self.get_logger().info(
            f"оценщик запущен: шаг {params.dt * 1000:.0f} мс, "
            f"карта {'есть' if tmap is not None else 'нет'}, выход "
            f"{g('projection')}"
            + (f" {g('mgrs_grid')}" if g("mgrs_grid") else
               " (MGRS: каждая точка в своём 100-км квадрате)" if g("projection") == "mgrs" else ""))

    def _emit(self, outs):
        """Публикует все шаги, сделанные по приходу сообщения."""
        for o in outs:
            t0 = time.perf_counter_ns()
            st = to_msg(o["stamp"])

            v = VelocitySensor()
            v.header.stamp, v.header.frame_id = st, self.child
            v.velocity = float(o["v"])
            self.pub_v.publish(v)

            od = Odometry()
            od.header.stamp, od.header.frame_id = st, self.frame_id
            pos_ok = o.get("pos_valid", True)
            od.child_frame_id = self.child
            od.pose.pose.position.x = float(o["x"])
            od.pose.pose.position.y = float(o["y"])
            od.pose.pose.position.z = float(o["z"])
            yaw = o.get("yaw")
            if yaw is not None:
                od.pose.pose.orientation.z = math.sin(yaw / 2.0)
                od.pose.pose.orientation.w = math.cos(yaw / 2.0)
            ss = float(o["sigma_s"]) ** 2
            od.pose.covariance[0] = od.pose.covariance[7] = ss
            od.twist.twist.linear.x = float(o["v"])
            od.twist.covariance[0] = float(o["sigma_v"]) ** 2
            if pos_ok:
                # без якоря GNSS (или у края квадрата MGRS) положения в
                # выходной системе нет: не публикуем, а не выдаём мусор
                self.pub_p.publish(od)

            ac = AccelStamped()
            ac.header.stamp, ac.header.frame_id = st, self.child
            ac.accel.linear.x = float(o["a"])
            self.pub_a.publish(ac)

            s = EstimatorStatus()
            s.header.stamp, s.header.frame_id = st, self.child
            s.mode = int(o["mode"])
            s.v, s.s, s.d = float(o["v"]), float(o["s"]), float(o["d"])
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
            s.step_time_us = (time.perf_counter_ns() - t0) / 1000.0
            self.pub_s.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = TramEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
