#!/usr/bin/env python3
"""ros_inject - инъекция битых сообщений во входы ноды tram_estimator.

Работает рядом с ros2 bag play (tools/ros_e2e.sh --inject "MODE [опции]").
Время bag берётся из последнего принятого /vehicle/driver_position_cmd.

Режимы:
  zero_stamp_first   до старта bag: одно сообщение передней тележки со stamp=0
                     (первое сообщение, которое увидит нода)
  zero_stamp_mid     через --at с после начала bag: одно сообщение со stamp=0
  future_stamp       через --at с: одно сообщение со stamp = время bag + --offset с
  nan | inf | spike  через --at с, --dur с, 10 Гц: передняя тележка = NaN / +inf /
                     --value (км/ч), stamp = текущее время bag
  handle_garbage     через --at с, --dur с, 20 Гц: ручка = --value (int8)
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)


def stamp_of(t):
    from builtin_interfaces.msg import Time
    m = Time()
    if t <= 0:
        return m
    m.sec = int(math.floor(t))
    m.nanosec = min(int(round((t - m.sec) * 1e9)), 999_999_999)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode")
    ap.add_argument("--at", type=float, default=20.0)
    ap.add_argument("--dur", type=float, default=3.0)
    ap.add_argument("--offset", type=float, default=1.0e5)
    ap.add_argument("--value", type=float, default=300.0)
    ap.add_argument("--topic", default="/vehicle/front_bogie_velocity")
    a = ap.parse_args()

    rclpy.init()
    n = Node("tram_e2e_inject")
    pub_v = n.create_publisher(VelocitySensor, a.topic, REL)
    pub_h = n.create_publisher(DriverControllerCommand, "/vehicle/driver_position_cmd", REL)
    st = {"bag": None, "t_first": None}

    def on_cmd(m):
        if m.header.frame_id == "inject":
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t > 0:
            st["bag"] = t if st["bag"] is None else max(st["bag"], t)
            if st["t_first"] is None:
                st["t_first"] = time.monotonic()
    n.create_subscription(DriverControllerCommand, "/vehicle/driver_position_cmd", on_cmd, BE)

    def log(s):
        print(f"[inject {time.strftime('%H:%M:%S')}] {s}", flush=True)

    def send_v(t, val):
        m = VelocitySensor()
        m.header.stamp, m.header.frame_id = stamp_of(t), "base_link"
        m.velocity = float(val)
        pub_v.publish(m)

    # ждём подписчиков (нода + проба)
    t0 = time.monotonic()
    while pub_v.get_subscription_count() < 2 and time.monotonic() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    log(f"subscribers on {a.topic}: {pub_v.get_subscription_count()}")

    if a.mode == "zero_stamp_first":
        for _ in range(3):
            rclpy.spin_once(n, timeout_sec=0.1)
        send_v(0.0, 0.0)
        log("sent front velocity=0 with stamp=0 (before bag)")
    else:
        while st["t_first"] is None:
            rclpy.spin_once(n, timeout_sec=0.1)
        log(f"bag started, bag time {st['bag']:.3f}")
        while time.monotonic() - st["t_first"] < a.at:
            rclpy.spin_once(n, timeout_sec=0.05)
        if a.mode == "zero_stamp_mid":
            send_v(0.0, a.value)
            log(f"sent front velocity={a.value} with stamp=0 at bag time {st['bag']:.3f}")
        elif a.mode == "future_stamp":
            send_v(st["bag"] + a.offset, 0.0)
            log(f"sent front velocity=0 with stamp=bag+{a.offset:.0f}s ({st['bag'] + a.offset:.3f})")
        elif a.mode in ("nan", "inf", "spike"):
            val = {"nan": float("nan"), "inf": float("inf"), "spike": a.value}[a.mode]
            k, t_end = 0, time.monotonic() + a.dur
            t_start_bag = st["bag"]
            while time.monotonic() < t_end:
                rclpy.spin_once(n, timeout_sec=0.0)
                send_v(st["bag"] + 0.001, val)
                k += 1
                time.sleep(0.1)
            log(f"sent {k} front velocity={val} msgs, bag time {t_start_bag:.3f}..{st['bag']:.3f}")
        elif a.mode == "handle_garbage":
            k, t_end = 0, time.monotonic() + a.dur
            while time.monotonic() < t_end:
                rclpy.spin_once(n, timeout_sec=0.0)
                m = DriverControllerCommand()
                m.header.stamp, m.header.frame_id = stamp_of(st["bag"] + 0.001), "inject"
                m.position = int(a.value)
                pub_h.publish(m)
                k += 1
                time.sleep(0.05)
            log(f"sent {k} handle position={int(a.value)} msgs")
        else:
            log(f"unknown mode {a.mode}")
    # держим издателя живым до останова
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:       # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
