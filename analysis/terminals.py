"""Конечные и разворотные кольца (поток loops, 26.09): геометрия и следование.

Организаторы 26.09: проверочные записи начинаются и кончаются примерно у
разворотных колец Щукинской (восток) и Таллинской (запад), а в балл положения
входит ошибка в КОНЦЕ записи. Этот скрипт — разбор на обучающих прогонах:

    python3 analysis/terminals.py geom  [--set train] [--out out/loops]
        траектории base_link всех прогонов набора у обеих конечных, карта
        ОЦЕНКИ (цвет — вес), начала и концы записей; out/loops/term_*.png и
        сводка концов (где стоят вагоны, по какому пути приходят)
    python3 analysis/terminals.py replay --ids train [--map F] [--tag T]
        прогон связки (лист ОЦЕНКИ, как tools/eval.py) и выход у конечных:
        out/loops/replay_<tag>/<bag>.npz (выход, эталон) и ends.json
        (ошибка в конце, вдоль/поперёк, конечная)
    python3 analysis/terminals.py plot --tag T [--ids ...]
        картинки конца прогона: эталон, выход, карта

Координаты — MGRS от угла 37UCB непрерывно (x = E − 300 000, y = N − 6 100 000
зоны 37), как у судьи и pathgraph.
"""

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bagio  # noqa: E402

ROOT = bagio.ROOT
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
sys.path.insert(0, str(PKG))
sys.path.insert(1, str(ROOT / "tools"))
from tram_state_estimator.geodesy import utm_fwd  # noqa: E402
from tram_state_estimator.track_map import TrackMap  # noqa: E402

E0, N0 = 300000.0, 6100000.0
ZONE = 37
SPLIT = ROOT / "tools" / "split.json"
EVAL_MAP = PKG / "config" / "eval" / "track_map.npz"
OUT = ROOT / "out" / "loops"
# грубые рамки конечных (MGRS 37UCB): запад — Таллинская, восток — Щукинская
BOX = {"west": (98900.0, 99500.0, 84700.0, 85200.0),
       "east": (103250.0, 103700.0, 85700.0, 86150.0)}
# крупно (только для картинок)
ZOOM = {"west": (98930.0, 99150.0, 84880.0, 85060.0),
        "east": (103480.0, 103680.0, 85880.0, 86100.0)}


def ids_of(key):
    sp = json.loads(SPLIT.read_text(encoding="utf-8"))
    return list(sp[key]) if key in sp else [b for b in key.split(",") if b]


def mgrs(lat, lon):
    E, N = utm_fwd(np.asarray(lat, float), np.asarray(lon, float), ZONE)
    return np.asarray(E, float) - E0, np.asarray(N, float) - N0


def base_track(b):
    import build_map
    a = bagio.load(b)
    tr = build_map.track(a, "base_link")
    if not tr:
        return None
    x, y = mgrs(tr["lat"], tr["lon"])
    return dict(t=tr["t"], x=x, y=y, head=tr["head"], speed=tr["speed"])


def which_box(x, y):
    for k, (x0, x1, y0, y1) in BOX.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return k
    return None


def map_xy(path=EVAL_MAP):
    m = TrackMap.load(str(path))
    x, y = mgrs(m.lat, m.lon)
    return m, x, y


# ------------------------------------------------------------------ geom

def cmd_geom(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ids = [b for b in ids_of(a.set) if (bagio.CACHE / f"{b}.npz").exists()]
    with ProcessPoolExecutor(a.workers) as ex:
        tracks = dict(zip(ids, ex.map(base_track, ids)))
    m, mx, my = map_xy(a.map)
    rows = []
    for b, tr in tracks.items():
        if tr is None:
            continue
        for end, i in (("start", 0), ("end", -1)):
            x, y = float(tr["x"][i]), float(tr["y"][i])
            rows.append(dict(bag=b, end=end, x=x, y=y, box=which_box(x, y),
                             head=float(np.degrees(tr["head"][i]))))
    (out / f"ends_{a.set}.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    for box, (x0, x1, y0, y1) in list(BOX.items()) + [(k + "_zoom", v) for k, v in ZOOM.items()]:
        fig, ax = plt.subplots(figsize=(13, 11))
        sel = (mx >= x0) & (mx <= x1) & (my >= y0) & (my <= y1)
        sc = ax.scatter(mx[sel], my[sel], c=np.minimum(m.weight[sel], 30), s=3, cmap="viridis",
                        zorder=3)
        # направление точек карты
        hs = m.head[sel]
        q = np.flatnonzero(sel)[::6]
        ax.quiver(mx[q], my[q], np.sin(m.head[q]), np.cos(m.head[q]), angles="xy",
                  scale=60, width=0.0015, color="k", alpha=0.4, zorder=4)
        del hs
        for b, tr in tracks.items():
            if tr is None:
                continue
            k = (tr["x"] >= x0) & (tr["x"] <= x1) & (tr["y"] >= y0) & (tr["y"] <= y1)
            if not k.any():
                continue
            ax.plot(tr["x"][k], tr["y"][k], "-", lw=0.5, alpha=0.5, color="tab:gray", zorder=1)
        for r in rows:
            if r["box"] != box.split("_")[0]:
                continue
            ax.plot(r["x"], r["y"], "^" if r["end"] == "start" else "v",
                    color="tab:green" if r["end"] == "start" else "tab:red", ms=6, zorder=5)
        for (la, lo) in m.terminals:
            tx, ty = mgrs([la], [lo])
            ax.plot(tx, ty, "x", color="m", ms=12, mew=2, zorder=6)
        for s in m.stops:
            sx, sy = mgrs([s[0]], [s[1]])
            ax.plot(sx, sy, "s", mfc="none", mec="orange", ms=9, zorder=6)
        ax.set_aspect("equal")
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_title(f"{box}: карта (цвет — вес), серое — base_link прогонов {a.set}, "
                     f"▲ начало, ▼ конец, × конечная, □ остановка")
        fig.colorbar(sc, ax=ax, shrink=0.6)
        fig.savefig(out / f"term_{box}_{a.set}.png", dpi=110)
        plt.close(fig)
    cnt = {}
    for r in rows:
        cnt[(r["box"], r["end"])] = cnt.get((r["box"], r["end"]), 0) + 1
    print("концы записей по конечным:", cnt)


# ------------------------------------------------------------------ replay

def _replay_one(task):
    import eval_metrics as M
    import eval_replay as R
    b, map_path, sheet_spec, sets, tag = task
    a = bagio.load(b)
    sheet = R.resolve_sheet(sheet_spec)
    node = dict(sheet["node"])
    ov = R.parse_overrides(sets) if sets else {}
    core_ov, node_ov = R.split_overrides(ov)
    node.update(node_ov)
    p = R.make_params(sheet, core_ov)
    tm = TrackMap.load(str(map_path)) if map_path else None
    r, _ = R.make_runner(p, node, tm)
    evs = R.events(a, "3")
    (O, crash), = R.replay(evs, [r], node)
    ref = M.reference(a, "base_link")
    if not len(ref["t"]) or not len(O["T"]):
        return b, None
    rx, ry = mgrs(ref["lat"], ref["lon"])
    PV = M.published(O)
    T, X = O["T"][PV], O["XYZ"][PV]
    j, ok = M.nearest(T, ref["t"])
    ex = np.where(ok, X[j, 0], np.nan)
    ey = np.where(ok, X[j, 1], np.nan)
    ez = np.where(ok, X[j, 2], np.nan)
    d3 = np.sqrt((ex - rx) ** 2 + (ey - ry) ** 2 + (ez - ref["alt"]) ** 2)
    np.savez_compressed(OUT / f"replay_{tag}" / f"{b}.npz",
                        t=ref["t"], rx=rx, ry=ry, rz=ref["alt"], ex=ex, ey=ey, ez=ez, d3=d3)
    last = np.flatnonzero(np.isfinite(d3))
    if not len(last):
        return b, None
    k = last[-1]
    # вдоль/поперёк в конце — по курсу эталона на последней точке
    yaw = ref["yaw"][k]
    dx, dy = ex[k] - rx[k], ey[k] - ry[k]
    along = dx * math.cos(yaw) + dy * math.sin(yaw)
    cross = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return b, dict(end3d=float(d3[k]), along=float(along), cross=float(cross),
                   mean3d=float(np.nanmean(d3)), box=which_box(rx[k], ry[k]),
                   start_box=which_box(rx[0], ry[0]), crash=crash,
                   anchors=int(getattr(r.pos, "anchors", 0)))


def cmd_replay(a):
    ids = ids_of(a.ids)
    ids = [b for b in ids if (bagio.CACHE / f"{b}.npz").exists()
           and len(bagio.load(b)["mfix"]) >= 100]
    od = OUT / f"replay_{a.tag}"
    od.mkdir(parents=True, exist_ok=True)
    tasks = [(b, a.map, a.sheet, a.override, a.tag) for b in ids]
    res = {}
    with ProcessPoolExecutor(a.workers) as ex:
        for b, r in ex.map(_replay_one, tasks):
            res[b] = r
            if r:
                print(f"{b}: конец {r['end3d']:7.2f} м (вдоль {r['along']:+7.2f}, поперёк "
                      f"{r['cross']:+6.2f}), ср. {r['mean3d']:6.2f}, конечная {r['box']}",
                      flush=True)
    (od / "ends.json").write_text(json.dumps(res, indent=1, ensure_ascii=False),
                                  encoding="utf-8")
    ok = [r for r in res.values() if r]
    print(f"прогонов {len(ok)}: конец ср. {np.mean([r['end3d'] for r in ok]):.2f} м, "
          f"мед. {np.median([r['end3d'] for r in ok]):.2f}, ср. 3D (среднее по прогонам) "
          f"{np.mean([r['mean3d'] for r in ok]):.2f}")


# ------------------------------------------------------------------ plot

def cmd_plot(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    od = OUT / f"replay_{a.tag}"
    ends = json.loads((od / "ends.json").read_text(encoding="utf-8"))
    ids = ids_of(a.ids) if a.ids else sorted(
        (b for b, r in ends.items() if r and r["end3d"] > a.min_err), key=lambda b: -ends[b]["end3d"])
    m, mx, my = map_xy(a.map)
    for b in ids:
        z = np.load(od / f"{b}.npz")
        k = np.flatnonzero(np.isfinite(z["d3"]))[-1]
        cx, cy = z["rx"][k], z["ry"][k]
        R_ = a.radius
        fig, ax = plt.subplots(figsize=(10, 9))
        sel = (np.abs(mx - cx) < R_) & (np.abs(my - cy) < R_)
        ax.scatter(mx[sel], my[sel], c=np.minimum(m.weight[sel], 30), s=3, cmap="viridis")
        q = np.flatnonzero(sel)[::4]
        ax.quiver(mx[q], my[q], np.sin(m.head[q]), np.cos(m.head[q]), angles="xy",
                  scale=60, width=0.0015, color="k", alpha=0.4)
        tail = np.abs(z["rx"] - cx) < R_
        tail &= np.abs(z["ry"] - cy) < R_
        ax.plot(z["rx"][tail], z["ry"][tail], "b-", lw=1, label="эталон base_link")
        ax.plot(z["ex"][tail], z["ey"][tail], "r--", lw=1, label="выход")
        ax.plot(z["rx"][k], z["ry"][k], "bo")
        ax.plot(z["ex"][k], z["ey"][k], "ro")
        ax.set_aspect("equal")
        ax.set_xlim(cx - R_, cx + R_)
        ax.set_ylim(cy - R_, cy + R_)
        r = ends[b]
        ax.set_title(f"{b} ({a.tag}): конец {r['end3d']:.1f} м, вдоль {r['along']:+.1f}")
        ax.legend()
        fig.savefig(od / f"end_{b}.png", dpi=100)
        plt.close(fig)
        print(od / f"end_{b}.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("geom", "replay", "plot"))
    ap.add_argument("--set", default="train", help="geom: набор прогонов (ключ split.json)")
    ap.add_argument("--ids", default="", help="replay/plot: ключ split.json или список")
    ap.add_argument("--map", default=str(EVAL_MAP))
    ap.add_argument("--sheet", default="eval")
    ap.add_argument("--override", default="", help="replay: поля листа k=v,... (как --set tools/eval.py)")
    ap.add_argument("--tag", default="base")
    ap.add_argument("--min-err", type=float, default=10.0)
    ap.add_argument("--radius", type=float, default=150.0)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    {"geom": cmd_geom, "replay": cmd_replay, "plot": cmd_plot}[a.cmd](a)


if __name__ == "__main__":
    main()
