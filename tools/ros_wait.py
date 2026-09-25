#!/usr/bin/env python3
"""ros_wait — ждёт, пока в графе ROS 2 появятся подписчики или издатели.

Нужен, чтобы `ros2 bag play` не начинал проигрывание раньше, чем нода и
проба подписались на входы (иначе начало bag теряется).

    python3 tools/ros_wait.py --subscribers /vehicle/front_bogie_velocity:2 --timeout 120
    python3 tools/ros_wait.py --publishers /result/velocity:1

Код выхода: 0 — условие выполнено, 1 — вышел таймаут.
"""

import argparse
import sys
import time

import rclpy


def spec(s):
    topic, _, n = s.partition(":")
    return topic, int(n or 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--subscribers", type=spec, action="append", default=[],
                    metavar="TOPIC[:N]", help="ждать >= N подписчиков топика")
    ap.add_argument("--publishers", type=spec, action="append", default=[],
                    metavar="TOPIC[:N]", help="ждать >= N издателей топика")
    ap.add_argument("--timeout", type=float, default=60.0, help="с, 0 — без предела")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("vectra_ros_wait")
    t0 = time.monotonic()

    def state():
        return ([(t, node.count_subscribers(t), n) for t, n in a.subscribers],
                [(t, node.count_publishers(t), n) for t, n in a.publishers])

    rc = 1
    try:
        while True:
            subs, pubs = state()
            if all(c >= n for _, c, n in subs + pubs):
                rc = 0
                break
            if a.timeout and time.monotonic() - t0 > a.timeout:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        subs, pubs = state()
        desc = ", ".join([f"{t}: {c}/{n} подп." for t, c, n in subs] +
                         [f"{t}: {c}/{n} изд." for t, c, n in pubs])
        print(f"[ros_wait] {'готово' if rc == 0 else 'ТАЙМАУТ'} за "
              f"{time.monotonic() - t0:.1f} с: {desc}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown() if hasattr(rclpy, "try_shutdown") else rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
