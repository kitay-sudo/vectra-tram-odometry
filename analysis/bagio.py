"""Чтение прогонов rosbag2 без ROS и кэш в numpy.

Каждый прогон читается один раз и сохраняется в <кэш>/<bag_id>.npz
(по умолчанию analysis/cache; другой каталог — переменная TRAM_CACHE или
set_cache()). Дальше анализ, калибровка и оценка работают с кэшем.

Данные — data/<bag_id>/ (другой каталог — TRAM_DATA). Типы сообщений
tram_vehicle_msgs берутся из пакета в репозитории
(ros2_ws/src/tram_vehicle_msgs/msg), а не из распакованного dataset/.

Для каждого топика хранятся две метки времени: tb — время записи в bag,
th — header.stamp сообщения (с).
"""

import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("TRAM_DATA") or (ROOT / "data"))
MSGS = ROOT / "ros2_ws" / "src" / "tram_vehicle_msgs" / "msg"
CACHE = Path(os.environ.get("TRAM_CACHE") or (Path(__file__).resolve().parent / "cache"))


def set_cache(path):
    """Сменить каталог кэша (и для процессов-потомков через TRAM_CACHE)."""
    global CACHE
    CACHE = Path(path)
    os.environ["TRAM_CACHE"] = str(CACHE)


def set_data(path):
    """Сменить каталог прогонов (и для процессов-потомков через TRAM_DATA)."""
    global DATA
    DATA = Path(path)
    os.environ["TRAM_DATA"] = str(DATA)

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
        tmp = f.with_name(f.stem + ".tmp.npz")   # недописанный файл не выглядит готовым
        np.savez_compressed(tmp, **arr)
        os.replace(tmp, f)
        return bag_id, "ok"
    except Exception as e:          # noqa: BLE001 — отчёт по прогону, не падаем
        return bag_id, f"ERROR {e!r}"


def build_cache(workers=None, ids=None):
    """Строит кэш для прогонов ids (по умолчанию — всех из data/); готовые
    файлы не трогает."""
    from concurrent.futures import ProcessPoolExecutor
    CACHE.mkdir(parents=True, exist_ok=True)
    ids = bag_ids() if ids is None else list(ids)
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
