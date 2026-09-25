"""Чтение прогонов rosbag2 без ROS и кэш в numpy.

Каждый прогон читается один раз и сохраняется в analysis/cache/<bag_id>.npz.
Дальше анализ, калибровка и оценка работают с кэшем.

Для каждого топика хранятся две метки времени: tb — время записи в bag,
th — header.stamp сообщения (с).
"""

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
MSGS = ROOT / "dataset" / "tram_vehicle_msgs" / "msg"
CACHE = Path(__file__).resolve().parent / "cache"

TOPICS = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
    "/vehicle/driver_position_cmd": "cmd",
    "/sensing/gnss/master/fix": "mfix",
    "/sensing/gnss/master/vel": "mvel",
    "/sensing/gnss/rover/fix": "rfix",
    "/sensing/gnss/rover/vel": "rvel",
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


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def extract(bag_dir: Path, ts=None):
    """Читает один прогон, возвращает словарь массивов."""
    from rosbags.highlevel import AnyReader
    ts = ts or typestore()
    rows = {k: [] for k in TOPICS.values()}
    with AnyReader([bag_dir], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, t, raw in reader.messages(connections=conns):
            key = TOPICS[conn.topic]
            m = reader.deserialize(raw, conn.msgtype)
            tb, th = t * 1e-9, _stamp(m.header)
            if key in ("front", "rear"):
                rows[key].append((tb, th, m.velocity))
            elif key == "cmd":
                rows[key].append((tb, th, m.position))
            elif key in ("mfix", "rfix"):
                c = m.position_covariance
                rows[key].append((tb, th, m.latitude, m.longitude, m.altitude,
                                  m.status.status, c[0], c[4], c[8]))
            else:
                lv = m.twist.linear
                rows[key].append((tb, th, lv.x, lv.y, lv.z))
    out = {}
    for key, r in rows.items():
        a = np.array(r, dtype=float) if r else np.zeros((0, 3))
        out[key] = a
    return out


def load(bag_id: str):
    """Массивы прогона из кэша: dict ключ -> ndarray (N, k)."""
    z = np.load(CACHE / f"{bag_id}.npz")
    return {k: z[k] for k in z.files}


def bag_ids():
    return sorted(p.name for p in DATA.iterdir() if (p / "metadata.yaml").exists())


def _one(bag_id):
    f = CACHE / f"{bag_id}.npz"
    if f.exists():
        return bag_id, "cached"
    try:
        arr = extract(DATA / bag_id)
        np.savez_compressed(f, **arr)
        return bag_id, "ok"
    except Exception as e:          # noqa: BLE001 — отчёт по прогону, не падаем
        return bag_id, f"ERROR {e!r}"


def build_cache(workers=None):
    import os
    from concurrent.futures import ProcessPoolExecutor
    CACHE.mkdir(exist_ok=True)
    ids = bag_ids()
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    bad = []
    with ProcessPoolExecutor(workers) as ex:
        for i, (bid, st) in enumerate(ex.map(_one, ids), 1):
            if st.startswith("ERROR"):
                bad.append((bid, st))
            if i % 20 == 0 or i == len(ids):
                print(f"{i}/{len(ids)}", flush=True)
    for bid, st in bad:
        print(bid, st)
    return ids, bad


if __name__ == "__main__":
    build_cache()
