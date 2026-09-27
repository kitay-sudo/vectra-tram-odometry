#!/usr/bin/env python3
"""bag_gnss_filter - копия rosbag2 с урезанным GNSS (проверка ноды через ROS).

Все сообщения копируются как есть (сырые байты, те же моменты записи и
метки); у топиков /sensing/gnss/* остаются только сообщения первых S секунд
по времени записи от первого сообщения GNSS master fix - как, по ответу
организаторов, в проверочных bag («GNSS гарантирован в первые секунды», потом
топики молчат). --gnss-first 0 - без GNSS вовсе.

    source /opt/ros/humble/setup.bash
    python3 tools/bag_gnss_filter.py /data/30618_e9a34502 out/bags/30618_e9a34502_gnss3 \\
        --gnss-first 3

Нужен rosbag2_py (есть в любом ROS 2 Humble). Выходной каталог перезаписывается.
"""

import argparse
import os
import shutil
import sys

GNSS_PREFIX = "/sensing/gnss/"
MASTER_FIX = "/sensing/gnss/master/fix"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("src", help="каталог исходного bag")
    ap.add_argument("dst", help="каталог нового bag (перезаписывается)")
    ap.add_argument("--gnss-first", type=float, default=3.0,
                    help="секунд GNSS от первого master fix по времени записи (0 - без GNSS)")
    a = ap.parse_args()
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.src, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = reader.get_all_topics_and_types()
    if os.path.exists(a.dst):
        shutil.rmtree(a.dst)
    os.makedirs(os.path.dirname(os.path.abspath(a.dst)), exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.dst, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for t in topics:
        writer.create_topic(t)
    t_first = None
    kept = dropped = total = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        total += 1
        if topic.startswith(GNSS_PREFIX):
            if topic == MASTER_FIX and t_first is None:
                t_first = t_ns
            limit = (t_first if t_first is not None else t_ns) + int(a.gnss_first * 1e9)
            if a.gnss_first <= 0 or t_ns > limit:
                dropped += 1
                continue
        writer.write(topic, data, t_ns)
        kept += 1
    del writer
    print(f"{a.src} -> {a.dst}: сообщений {total}, записано {kept}, "
          f"GNSS отброшено {dropped} (оставлены первые {a.gnss_first:g} с)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
