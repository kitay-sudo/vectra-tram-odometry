#!/usr/bin/env python3
"""fixture_to_bag - rosbag2 (sqlite3) из фикстуры e2e-теста, без данных кейса.

Фикстура test/data/e2e_*.npz - кусок отложенного прогона (входы, GNSS fix и vel
обеих антенн) с временем записи tb и header.stamp th. Отсюда собирается bag с
теми же топиками, типами, метками и моментами записи, что в исходном прогоне:
его можно проигрывать `ros2 bag play` в CI и из чистого клона, где data/ нет.

    source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
    python3 tools/fixture_to_bag.py --out /tmp/smoke_bag --seconds 30

Нужны rclpy, rosbag2_py и собранный tram_vehicle_msgs.
"""

import argparse
import os
import shutil
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE = os.path.join(ROOT, "ros2_ws", "src", "tram_state_estimator", "test", "data",
                       "e2e_30618_b95ca60a_180s.npz")

TOPICS = {   # ключ фикстуры -> (топик, тип)
    "front": ("/vehicle/front_bogie_velocity", "tram_vehicle_msgs/msg/VelocitySensor"),
    "rear": ("/vehicle/rear_bogie_velocity", "tram_vehicle_msgs/msg/VelocitySensor"),
    "cmd": ("/vehicle/driver_position_cmd", "tram_vehicle_msgs/msg/DriverControllerCommand"),
    "mfix": ("/sensing/gnss/master/fix", "sensor_msgs/msg/NavSatFix"),
    "rfix": ("/sensing/gnss/rover/fix", "sensor_msgs/msg/NavSatFix"),
    "mvel": ("/sensing/gnss/master/vel", "geometry_msgs/msg/TwistStamped"),
    "rvel": ("/sensing/gnss/rover/vel", "geometry_msgs/msg/TwistStamped"),
}


def stamp(t):
    from builtin_interfaces.msg import Time
    sec = int(np.floor(t))
    ns = int(round((t - sec) * 1e9))
    if ns >= 1_000_000_000:
        sec, ns = sec + 1, ns - 1_000_000_000
    return Time(sec=sec, nanosec=ns)


def make(key, row):
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import NavSatFix
    from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor
    th = float(row[1])
    if key in ("front", "rear"):
        m = VelocitySensor()
        m.header.frame_id = "base_link"
        m.velocity = float(row[2])
    elif key == "cmd":
        m = DriverControllerCommand()
        m.position = int(row[2])
    elif key in ("mfix", "rfix"):
        m = NavSatFix()
        m.header.frame_id = "gnss_" + ("master" if key == "mfix" else "rover")
        m.latitude, m.longitude, m.altitude = float(row[2]), float(row[3]), float(row[4])
        m.status.status = int(row[5])
    else:
        m = TwistStamped()
        m.header.frame_id = "gnss_" + ("master" if key == "mvel" else "rover")
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(v) for v in row[2:5])
    m.header.stamp = stamp(th)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--fixture", default=FIXTURE)
    ap.add_argument("--out", required=True, help="каталог bag (перезаписывается)")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="только первые N с по времени записи (0 - вся фикстура)")
    ap.add_argument("--gnss-window", type=float, default=0.0,
                    help="GNSS (fix и vel) только первые S с по header.stamp от первой "
                         "точки master, как, возможно, в bag жюри (0 - весь)")
    a = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import serialize_message

    z = np.load(a.fixture, allow_pickle=False)
    rows = []
    t_first = min(float(z[k][0, 0]) for k in TOPICS if k in z.files and len(z[k]))
    th_gnss0 = float(z["mfix"][0, 1]) if "mfix" in z.files and len(z["mfix"]) else 0.0
    for key in TOPICS:
        if key not in z.files:
            continue
        gnss = key in ("mfix", "rfix", "mvel", "rvel")
        for r in z[key]:
            if a.seconds and r[0] > t_first + a.seconds:
                continue
            if gnss and a.gnss_window and r[1] > th_gnss0 + a.gnss_window:
                continue
            rows.append((float(r[0]), key, r))
    rows.sort(key=lambda x: (x[0], x[1]))

    if os.path.exists(a.out):
        shutil.rmtree(a.out)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=a.out, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for key, (topic, typ) in TOPICS.items():
        if key in z.files:
            w.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ,
                                                    serialization_format="cdr"))
    for tb, key, r in rows:
        w.write(TOPICS[key][0], serialize_message(make(key, r)), int(round(tb * 1e9)))
    del w
    span = rows[-1][0] - rows[0][0] if rows else 0.0
    g = [tb for tb, key, _ in rows if key in ("mfix", "rfix")]
    gi = (f"; GNSS fix: {len(g)} шт., по времени записи {min(g) - t_first:.2f}…"
          f"{max(g) - t_first:.2f} с от начала bag") if g else "; GNSS нет"
    print(f"[fixture_to_bag] {a.out}: {len(rows)} сообщений, {span:.1f} с записи{gi}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
