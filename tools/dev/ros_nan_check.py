"""Публикует в ноду tram_estimator короткий поток с выставкой по GNSS, затем
одно показание тележки NaN; считает выходы до и после."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

NAN_KIND = sys.argv[1] if len(sys.argv) > 1 else "nan"


def st(t):
    m = TimeMsg()
    m.sec = int(math.floor(t))
    m.nanosec = int(round((t - m.sec) * 1e9)) % 1_000_000_000
    return m


class P(Node):
    def __init__(self):
        super().__init__("nan_check_pub")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100)
        self.f = self.create_publisher(VelocitySensor, "/vehicle/front_bogie_velocity", q)
        self.r = self.create_publisher(VelocitySensor, "/vehicle/rear_bogie_velocity", q)
        self.h = self.create_publisher(DriverControllerCommand, "/vehicle/driver_position_cmd", q)
        self.gm = self.create_publisher(NavSatFix, "/sensing/gnss/master/fix", q)
        self.gr = self.create_publisher(NavSatFix, "/sensing/gnss/rover/fix", q)
        self.n_out = 0
        self.nan_out = 0
        self.create_subscription(Odometry, "/result/position", self.cb, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100))

    def cb(self, m):
        self.n_out += 1
        p = m.pose.pose.position
        if not all(math.isfinite(x) for x in (p.x, p.y, p.z, m.twist.twist.linear.x)):
            self.nan_out += 1


def main():
    rclpy.init()
    n = P()
    time.sleep(2.0)                      # discovery
    lat0, lon0, az = 55.810367, 37.462267, math.radians(19.2)
    k = math.cos(math.radians(lat0))
    t0 = 1787731608.0
    t = 0.0
    phase_counts = {}
    for i in range(160):                 # 8 с данных по 0,05 с, в 2 раза быстрее реального
        th = t0 + t
        vs = VelocitySensor()
        vs.header.stamp = st(th)
        vs.header.frame_id = "base_link"
        val = 0.0
        if i == 120:
            val = {"nan": float("nan"), "inf": float("inf")}[NAN_KIND]
        vs.velocity = val
        if i % 2 == 0:
            n.f.publish(vs)
            vs2 = VelocitySensor()
            vs2.header.stamp = st(th + 0.01)
            vs2.velocity = 0.0
            n.r.publish(vs2)
        hc = DriverControllerCommand()
        hc.header.stamp = st(th + 0.005)
        hc.position = 0
        n.h.publish(hc)
        if t <= 2.0 and i % 2 == 0:
            for pub, d in ((n.gm, 0.0), (n.gr, 12.0)):
                g = NavSatFix()
                g.header.stamp = st(th)
                g.latitude = lat0 + math.degrees(d * math.cos(az) / 6378137.0)
                g.longitude = lon0 + math.degrees(d * math.sin(az) / (6378137.0 * k))
                g.altitude = 168.0
                g.status.status = 0
                pub.publish(g)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.025)
        t += 0.05
        if i == 119:
            phase_counts["before"] = n.n_out
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    phase_counts["after"] = n.n_out - phase_counts["before"]
    print(f"RESULT kind={NAN_KIND} outputs_before_bad={phase_counts['before']} "
          f"outputs_after_bad={phase_counts['after']} nonfinite_outputs={n.nan_out}", flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
