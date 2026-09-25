#!/usr/bin/env python3
"""Фикстура для test/test_e2e_real.py: начало реального прогона в компактном npz.

Читает bag из data/<bag_id> (rosbags, типы tram_vehicle_msgs берутся из
ros2_ws/src/tram_vehicle_msgs/msg) и сохраняет первые --seconds секунд по
времени записи: входы ноды, GNSS fix и vel обеих антенн (эталон скорости — master/vel,
контрольный — rover/vel).
Формат массивов как в analysis/bagio.py: первая колонка tb — время записи в
bag, вторая th — header.stamp, дальше значения.

    python3 tools/make_e2e_fixture.py 30618_b95ca60a --seconds 180 \
        --out ros2_ws/src/tram_state_estimator/test/data/e2e_30618_b95ca60a_180s.npz

Прогон берётся только из holdout_scored (tools/split.json): фикстура — кусок
отложенной записи. Файл должен быть < 1 МБ и лежать вне out/, analysis/cache
и *.db3 (они в .gitignore).
"""

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MSGS = ROOT / "ros2_ws" / "src" / "tram_vehicle_msgs" / "msg"

TOPICS = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
    "/vehicle/driver_position_cmd": "cmd",
    "/sensing/gnss/master/fix": "mfix",
    "/sensing/gnss/rover/fix": "rfix",
    "/sensing/gnss/master/vel": "mvel",
    "/sensing/gnss/rover/vel": "rvel",
}
COLS = {
    "front": "tb th velocity_kmh",
    "rear": "tb th velocity_kmh",
    "cmd": "tb th position",
    "mfix": "tb th lat lon alt status",
    "rfix": "tb th lat lon alt status",
    "mvel": "tb th vx vy vz",
    "rvel": "tb th vx vy vz",
}


def typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for name in ("VelocitySensor", "DriverControllerCommand"):
        text = (MSGS / f"{name}.msg").read_text(encoding="utf-8")
        add.update(get_types_from_msg(text, f"tram_vehicle_msgs/msg/{name}"))
    ts.register(add)
    return ts


def extract(bag_dir, seconds):
    from rosbags.highlevel import AnyReader
    rows = {k: [] for k in TOPICS.values()}
    with AnyReader([Path(bag_dir)], default_typestore=typestore()) as reader:
        t_first = reader.start_time * 1e-9
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, t, raw in reader.messages(connections=conns):
            tb = t * 1e-9
            if tb > t_first + seconds:
                break
            m = reader.deserialize(raw, conn.msgtype)
            th = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            key = TOPICS[conn.topic]
            if key in ("front", "rear"):
                rows[key].append((tb, th, m.velocity))
            elif key == "cmd":
                rows[key].append((tb, th, m.position))
            elif key in ("mfix", "rfix"):
                rows[key].append((tb, th, m.latitude, m.longitude, m.altitude,
                                  m.status.status))
            else:
                v = m.twist.linear
                rows[key].append((tb, th, v.x, v.y, v.z))
    out = {}
    for key, r in rows.items():
        ncol = len(COLS[key].split())
        out[key] = np.array(r, dtype=np.float64).reshape(-1, ncol)
    return t_first, out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--data", default=str(ROOT / "data"))
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    split = json.loads((ROOT / "tools" / "split.json").read_text(encoding="utf-8"))
    if a.bag not in split["holdout_scored"]:
        raise SystemExit(f"{a.bag} не из holdout_scored (tools/split.json)")
    t_first, arr = extract(Path(a.data) / a.bag, a.seconds)
    meta = {
        "bag": a.bag, "split": "holdout_scored", "seconds": a.seconds,
        "record_start_s": t_first, "columns": COLS,
        "note": "первые N секунд по времени записи; tb — время записи, th — header.stamp",
        "made_by": "tools/make_e2e_fixture.py",
    }
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, meta=np.array(json.dumps(meta, ensure_ascii=False)), **arr)
    print(f"{out}: {out.stat().st_size / 1024:.0f} КБ; " +
          ", ".join(f"{k}={len(v)}" for k, v in arr.items()))


if __name__ == "__main__":
    main()
