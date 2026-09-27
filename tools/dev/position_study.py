"""Аудит конвейера положения (шаг 4 PROMPT_FOR_AGENT): выставка по GNSS,
карта путей, система координат судьи.

Работает на кэше analysis/cache/*.npz (analysis/bagio.py) и исходном коде пакета
(runner.py, track_map.py, evaluate.py) БЕЗ их изменения: варианты выставки
делаются подклассами в этом файле.

Запуск (в образе vectra/tram:dev, из корня репозитория):

    docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev \
        bash -c "python3 tools/dev/position_study.py all"

Подкоманды (результаты — out/position/):
    inventory  GNSS по прогонам: антенны, статус, база, старт на ходу, задержки меток
    timing     сдвиг меток GNSS относительно меток тележек (взаимная корреляция скоростей)
    frames     чувствительность к проекции: исходный equirect / ENU WGS84 / UTM
    dups       дубликаты и перекрытия прогонов по времени
    coverage   покрытие отложенных прогонов обучающей картой, leave-one-out
    run        прогоны конвейера (варианты выставки/карты) -> out/position/runs/
    score      метрики вариантов и чувствительность к началу координат судьи
    branches   разбор прогонов с ошибкой выбора ветки на стрелке
    all        всё по порядку
"""

import csv
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tram_state_estimator"))

import bagio  # noqa: E402

OUT = ROOT / "out" / "position"
RUNS = OUT / "runs"
TRAIN_MAP = ROOT / "analysis" / "cache" / "track_map_train.npz"
KMH = 3.6
INIT_S = 3.0
TOL = 0.05
WORKERS = int(os.environ.get("WORKERS", "6"))

# ------------------------------------------------------------------ геодезия
A_WGS = 6378137.0
F_WGS = 1.0 / 298.257223563
E2 = F_WGS * (2.0 - F_WGS)


def equirect(lat, lon, alt, o):
    """Исходная проекция пакета (runner.Enu): сфера R = a, x = dλ·R·cos φ0."""
    k = math.cos(math.radians(o[0]))
    return np.c_[np.radians(np.asarray(lon) - o[1]) * A_WGS * k,
                 np.radians(np.asarray(lat) - o[0]) * A_WGS,
                 np.asarray(alt) - o[2]]


def ecef(lat, lon, alt):
    la, lo = np.radians(lat), np.radians(lon)
    n = A_WGS / np.sqrt(1.0 - E2 * np.sin(la) ** 2)
    return np.c_[(n + alt) * np.cos(la) * np.cos(lo),
                 (n + alt) * np.cos(la) * np.sin(lo),
                 (n * (1.0 - E2) + alt) * np.sin(la)]


def enu_true(lat, lon, alt, o):
    """Строгий ENU WGS84 (как pymap3d.geodetic2enu / GeographicLib LocalCartesian)."""
    p = ecef(np.asarray(lat), np.asarray(lon), np.asarray(alt)) - ecef(
        np.array([o[0]]), np.array([o[1]]), np.array([o[2]]))
    la, lo = math.radians(o[0]), math.radians(o[1])
    R = np.array([[-math.sin(lo), math.cos(lo), 0.0],
                  [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                  [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])
    return p @ R.T


def utm(lat, lon, zone=37):
    """UTM (северное полушарие), ряд Крюгера до n^3 (точность < 1 мм)."""
    n = F_WGS / (2.0 - F_WGS)
    Ab = A_WGS / (1.0 + n) * (1.0 + n ** 2 / 4.0 + n ** 4 / 64.0)
    al = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16, 13 * n ** 2 / 48 - 3 * n ** 3 / 5,
          61 * n ** 3 / 240)
    phi = np.radians(np.asarray(lat, float))
    dl = np.radians(np.asarray(lon, float) - (zone * 6 - 183))
    c = 2.0 * math.sqrt(n) / (1.0 + n)
    t = np.sinh(np.arctanh(np.sin(phi)) - c * np.arctanh(c * np.sin(phi)))
    xi = np.arctan(t / np.cos(dl))
    eta = np.arctanh(np.sin(dl) / np.sqrt(1.0 + t ** 2))
    E = xi * 0 + eta
    N = xi + 0.0
    for j, a in enumerate(al, 1):
        E = E + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
        N = N + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
    return np.c_[500000.0 + 0.9996 * Ab * E, 0.9996 * Ab * N]


# ------------------------------------------------------------------ данные
def ids_all():
    return bagio.bag_ids()


def split():
    dm = json.loads((ROOT / "analysis" / "drive_model.json").read_text(encoding="utf-8"))
    return dm["train"], dm["val"]


def wheel_speed(a, t):
    """Средняя скорость тележек, м/с (км/ч -> м/с, без meas_scale), на метках t."""
    f, r = a["front"], a["rear"]
    if len(f) < 2 or len(r) < 2:
        return np.full(len(t), np.nan)
    return 0.5 * (np.interp(t, f[:, 1], f[:, 2]) + np.interp(t, r[:, 1], r[:, 2])) / KMH


def gnss_speed(a, t, key="mvel"):
    g = a[key]
    return np.interp(t, g[:, 1], np.hypot(g[:, 2], g[:, 3]))


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, restval="")
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})


def pair_master_rover(a, tol=0.051):
    """Пары master/rover с одинаковой меткой (эпоха GNSS, шаг 0,1 с)."""
    m, r = a["mfix"], a["rfix"]
    if len(m) == 0 or len(r) == 0:
        return np.zeros(0, int), np.zeros(0, int)
    j = np.clip(np.searchsorted(r[:, 1], m[:, 1]), 1, len(r) - 1)
    j = np.where(np.abs(r[j - 1, 1] - m[:, 1]) < np.abs(r[j, 1] - m[:, 1]), j - 1, j)
    ok = np.abs(r[j, 1] - m[:, 1]) <= tol
    return np.flatnonzero(ok), j[ok]


# ------------------------------------------------------------------ inventory
def _inventory_one(b):
    a = bagio.load(b)
    m, r, f = a["mfix"], a["rfix"], a["front"]
    row = dict(bag=b, veh=b.split("_")[0])
    tw = np.r_[a["front"][:, 1], a["rear"][:, 1]] if len(f) else np.zeros(0)
    row["dur_s"] = float(tw.max() - tw.min()) if len(tw) else 0.0
    row["n_mfix"], row["n_rfix"] = len(m), len(r)
    row["n_mvel"], row["n_rvel"] = len(a["mvel"]), len(a["rvel"])
    # задержки «метка bag - header.stamp» по топикам (медиана), с
    for k in ("front", "rear", "cmd", "mfix", "rfix", "mvel", "rvel"):
        x = a[k]
        row[f"lag_{k}"] = float(np.median(x[:, 0] - x[:, 1])) if len(x) else float("nan")
    if len(m) < 10:
        row["gnss"] = False
        return row
    row["gnss"] = True
    ok = np.isfinite(m[:, 2]) & np.isfinite(m[:, 3])
    row["m_nan"] = int((~ok).sum())
    row["m_st2"] = float(np.mean(m[:, 5] == 2))
    row["m_st0"] = float(np.mean(m[:, 5] == 0))
    row["m_stneg"] = float(np.mean(m[:, 5] < 0))
    row["gnss_span_s"] = float(m[-1, 1] - m[0, 1])
    row["gnss_cov"] = float(len(m) * 0.1 / max(row["dur_s"], 1e-9))
    row["m_maxgap_s"] = float(np.max(np.diff(m[:, 1]))) if len(m) > 1 else float("nan")
    row["t_first_m_minus_wheel"] = float(m[0, 1] - tw.min()) if len(tw) else float("nan")
    t0 = min(m[0, 1], r[0, 1]) if len(r) else m[0, 1]
    row["first_is"] = "rover" if len(r) and r[0, 1] < m[0, 1] else "master"
    win = (m[:, 1] <= t0 + INIT_S)
    row["n_m_win"] = int(win.sum())
    row["n_r_win"] = int(((r[:, 1] <= t0 + INIT_S)).sum()) if len(r) else 0
    row["st_m_win"] = float(np.mean(m[win, 5] == 2)) if win.any() else float("nan")
    # база master->rover
    im, ir = pair_master_rover(a)
    if len(im):
        o = (m[0, 2], m[0, 3], m[0, 4])
        pm = equirect(m[im, 2], m[im, 3], m[im, 4], o)
        pr = equirect(r[ir, 2], r[ir, 3], r[ir, 4], o)
        d = pr - pm
        base = np.hypot(d[:, 0], d[:, 1])
        both2 = (m[im, 5] == 2) & (r[ir, 5] == 2)
        row["base_med"] = float(np.median(base))
        row["base_p05"] = float(np.percentile(base, 5))
        row["base_p95"] = float(np.percentile(base, 95))
        row["base_med_st2"] = float(np.median(base[both2])) if both2.any() else float("nan")
        row["dz_rover_master"] = float(np.median(d[:, 2]))
        # курс по базе против курса по скорости GNSS на ходу
        v = a["mvel"]
        sp = np.interp(m[im, 1], v[:, 1], np.hypot(v[:, 2], v[:, 3]))
        hv = np.arctan2(np.interp(m[im, 1], v[:, 1], v[:, 2]), np.interp(m[im, 1], v[:, 1], v[:, 3]))
        hb = np.arctan2(d[:, 0], d[:, 1])
        mv = (sp > 3.0) & both2
        if mv.sum() > 20:
            dh = np.degrees(np.angle(np.exp(1j * (hb[mv] - hv[mv]))))
            row["base_vs_vel_deg_med"] = float(np.median(dh))
            row["base_vs_vel_deg_abs95"] = float(np.percentile(np.abs(dh), 95))
            row["frac_reverse"] = float(np.mean(np.abs(dh) > 90))
        inw = m[im, 1] <= t0 + INIT_S
        row["n_pairs_win"] = int(inw.sum())
    else:
        row["n_pairs_win"] = 0
    # движение на старте (первая точка master и окно INIT_S)
    tm0 = m[0, 1]
    row["v_gnss_first"] = float(gnss_speed(a, np.array([tm0]))[0]) if len(a["mvel"]) else float("nan")
    row["v_wheel_first"] = float(wheel_speed(a, np.array([tm0]))[0])
    tt = np.arange(tm0, tm0 + INIT_S, 0.02)
    row["path_wheel_win"] = float(np.sum(np.nan_to_num(wheel_speed(a, tt))) * 0.02)
    ww = m[:, 1] <= tm0 + INIT_S
    pw = equirect(m[ww, 2], m[ww, 3], m[ww, 4], (m[0, 2], m[0, 3], m[0, 4]))
    row["disp_gnss_win"] = float(np.hypot(*(pw[-1, :2] - pw[0, :2]))) if len(pw) else float("nan")
    row["moving_start"] = bool(row["path_wheel_win"] > 0.5 or row["disp_gnss_win"] > 0.5)
    # протяжённость и путь
    P = equirect(m[ok, 2], m[ok, 3], m[ok, 4], (m[0, 2], m[0, 3], m[0, 4]))
    st = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    row["path_gnss_km"] = float(st[st < 5.0].sum() / 1000.0)
    row["ext_x_km"] = float(np.ptp(P[:, 0]) / 1000.0)
    row["ext_y_km"] = float(np.ptp(P[:, 1]) / 1000.0)
    row["max_r_km"] = float(np.max(np.hypot(P[:, 0], P[:, 1])) / 1000.0)
    row["z_range_m"] = float(np.ptp(P[:, 2]))
    row["n_jumps5m"] = int((st >= 5.0).sum())
    row["start_lat"], row["start_lon"] = float(m[0, 2]), float(m[0, 3])
    row["end_lat"], row["end_lon"] = float(m[-1, 2]), float(m[-1, 3])
    return row


def cmd_inventory():
    ids = ids_all()
    with ProcessPoolExecutor(WORKERS) as ex:
        rows = list(ex.map(_inventory_one, ids))
    # единый набор колонок
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    rows = [{k: r.get(k, "") for k in keys} for r in rows]
    write_csv(OUT / "inventory.csv", rows)
    g = [r for r in rows if r["gnss"] is True]
    ms = [r for r in g if r["moving_start"] is True]
    print(f"прогонов {len(rows)}, с GNSS {len(g)}, старт на ходу {len(ms)}")
    for r in ms:
        print(f"  старт на ходу: {r['bag']} v_gnss={r['v_gnss_first']:.2f} "
              f"v_wheel={r['v_wheel_first']:.2f} путь_колёс_3с={r['path_wheel_win']:.1f} "
              f"смещение_GNSS_3с={r['disp_gnss_win']:.1f} пар_в_окне={r['n_pairs_win']}")
    np_ = [r for r in g if r["n_pairs_win"] == 0]
    print(f"без пары master/rover в окне {INIT_S} с: {len(np_)} {[r['bag'] for r in np_]}")
    b = np.array([r["base_med_st2"] for r in g if r.get("base_med_st2") not in ("", None)], float)
    b = b[np.isfinite(b)]
    print(f"база master-rover (RTK обе), медиана по прогонам: {np.median(b):.3f} м, "
          f"разброс {b.min():.3f}..{b.max():.3f}")
    return rows


# ------------------------------------------------------------------ timing
def _timing_one(b):
    """Сдвиг tau: скорость GNSS с меткой t соответствует колёсам с меткой t + tau.
    Ищется минимум СКО |v_gnss(t)| - v_wheel(t + tau)·s по tau (s — масштаб, МНК)."""
    a = bagio.load(b)
    if len(a["mvel"]) < 500 or len(a["front"]) < 500:
        return None
    g = a["mvel"]
    t = g[:, 1]
    vg = np.hypot(g[:, 2], g[:, 3])
    sel = (t > max(a["front"][0, 1], a["rear"][0, 1]) + 5) & (t < min(a["front"][-1, 1], a["rear"][-1, 1]) - 5)
    t, vg = t[sel], vg[sel]
    if len(t) < 300:
        return None
    taus = np.arange(-2.0, 2.0001, 0.01)
    best = (np.inf, 0.0, 1.0)
    res = []
    for tau in taus:
        w = wheel_speed(a, t + tau)
        s = float(np.dot(w, vg) / max(np.dot(w, w), 1e-9))
        e = float(np.sqrt(np.mean((s * w - vg) ** 2)))
        res.append(e)
        if e < best[0]:
            best = (e, float(tau), s)
    # то же по отдельности для каждой тележки
    out = dict(bag=b, tau=best[1], rmse=best[0], scale=best[2],
               rmse_tau0=res[int(np.argmin(np.abs(taus)))])
    for k in ("front", "rear"):
        x = a[k]
        bb = (np.inf, 0.0)
        for tau in taus:
            w = np.interp(t + tau, x[:, 1], x[:, 2]) / KMH
            s = float(np.dot(w, vg) / max(np.dot(w, w), 1e-9))
            e = float(np.sqrt(np.mean((s * w - vg) ** 2)))
            if e < bb[0]:
                bb = (e, float(tau))
        out[f"tau_{k}"] = bb[1]
    # тот же сдвиг во времени записи bag (tb) — на случай, если судья берёт его
    tb = g[sel, 0]
    bb = (np.inf, 0.0)
    for tau in taus:
        w = 0.5 * (np.interp(tb + tau, a["front"][:, 0], a["front"][:, 2]) +
                   np.interp(tb + tau, a["rear"][:, 0], a["rear"][:, 2])) / KMH
        s = float(np.dot(w, vg) / max(np.dot(w, w), 1e-9))
        e = float(np.sqrt(np.mean((s * w - vg) ** 2)))
        if e < bb[0]:
            bb = (e, float(tau))
    out["tau_bagtime"] = bb[1]
    out["mean_speed"] = float(np.mean(vg))
    return out


def cmd_timing():
    with ProcessPoolExecutor(WORKERS) as ex:
        rows = [r for r in ex.map(_timing_one, ids_all()) if r is not None]
    write_csv(OUT / "timing.csv", rows)
    tau = np.array([r["tau"] for r in rows])
    print(f"прогонов {len(rows)}: tau (header-время) медиана {np.median(tau):+.3f} с, "
          f"p05 {np.percentile(tau, 5):+.3f}, p95 {np.percentile(tau, 95):+.3f}")
    for k in ("tau_front", "tau_rear", "tau_bagtime"):
        x = np.array([r[k] for r in rows])
        print(f"  {k}: медиана {np.median(x):+.3f} с, p05 {np.percentile(x, 5):+.3f}, "
              f"p95 {np.percentile(x, 95):+.3f}")
    return rows


# ------------------------------------------------------------------ frames
def _frames_one(b):
    a = bagio.load(b)
    m = a["mfix"]
    if len(m) < 100:
        return None
    ok = np.isfinite(m[:, 2]) & np.isfinite(m[:, 3])
    m = m[ok]
    o = (m[0, 2], m[0, 3], m[0, 4])
    P_eq = equirect(m[:, 2], m[:, 3], m[:, 4], o)
    P_en = enu_true(m[:, 2], m[:, 3], m[:, 4], o)
    U = utm(m[:, 2], m[:, 3])
    P_ut = np.c_[U - U[0], m[:, 4] - o[2]]
    d_eq = np.linalg.norm(P_eq - P_en, axis=1)
    d_eq2 = np.linalg.norm(P_eq[:, :2] - P_en[:, :2], axis=1)
    d_ut = np.linalg.norm(P_ut[:, :2] - P_en[:, :2], axis=1)
    d_ut_eq = np.linalg.norm(P_ut[:, :2] - P_eq[:, :2], axis=1)
    rr = np.hypot(P_en[:, 0], P_en[:, 1])
    # ориентир: «город» — фиксированное начало (центр карты build_map.py)
    return dict(bag=b, max_r_km=float(rr.max() / 1000),
                eq_vs_enu_mean=float(d_eq.mean()), eq_vs_enu_max=float(d_eq.max()),
                eq_vs_enu_xy_mean=float(d_eq2.mean()),
                eq_vs_enu_z_mean=float(np.mean(np.abs(P_eq[:, 2] - P_en[:, 2]))),
                utm_vs_enu_mean=float(d_ut.mean()), utm_vs_enu_max=float(d_ut.max()),
                utm_vs_eq_mean=float(d_ut_eq.mean()), utm_vs_eq_max=float(d_ut_eq.max()))


def cmd_frames():
    # масштаб и поворот сетки UTM в районе данных (проверка реализации)
    lat, lon = 55.81, 37.46
    d = 1e-4
    U0 = utm([lat], [lon])[0]
    Ue = utm([lat], [lon + d])[0]
    Un = utm([lat + d], [lon])[0]
    Ee = enu_true([lat], [lon + d], [0.0], (lat, lon, 0.0))[0]
    En = enu_true([lat + d], [lon], [0.0], (lat, lon, 0.0))[0]
    k_e = np.hypot(*(Ue - U0)) / np.hypot(Ee[0], Ee[1])
    k_n = np.hypot(*(Un - U0)) / np.hypot(En[0], En[1])
    gamma = math.degrees(math.atan2(Un[0] - U0[0], Un[1] - U0[1]))
    Qe = equirect([lat], [lon + d], [0.0], (lat, lon, 0.0))[0]
    Qn = equirect([lat + d], [lon], [0.0], (lat, lon, 0.0))[0]
    print(f"UTM 37N у ({lat}, {lon}): масштаб E {k_e:.6f}, N {k_n:.6f}, "
          f"сближение меридианов {gamma:+.3f}° (теория Δλ·sinφ = "
          f"{(lon - 39) * math.sin(math.radians(lat)):+.3f}°)")
    print(f"исходный equirect / строгий ENU: масштаб E {Qe[0] / Ee[0]:.6f} "
          f"({(Qe[0] / Ee[0] - 1) * 100:+.3f} %), N {Qn[1] / En[1]:.6f} "
          f"({(Qn[1] / En[1] - 1) * 100:+.3f} %)")
    with ProcessPoolExecutor(WORKERS) as ex:
        rows = [r for r in ex.map(_frames_one, ids_all()) if r is not None]
    write_csv(OUT / "frames.csv", rows)
    for k in ("eq_vs_enu_mean", "eq_vs_enu_max", "utm_vs_enu_mean", "utm_vs_enu_max"):
        x = np.array([r[k] for r in rows])
        print(f"  {k}: медиана по прогонам {np.median(x):.2f} м, макс {x.max():.2f} м")
    return dict(k_utm_e=k_e, k_utm_n=k_n, gamma_deg=gamma,
                eq_scale_e=float(Qe[0] / Ee[0]), eq_scale_n=float(Qn[1] / En[1]))


# ------------------------------------------------------------------ dups
def cmd_dups():
    ids = ids_all()
    info = []
    for b in ids:
        a = bagio.load(b)
        f = a["front"]
        th = np.r_[a["front"][:, 1], a["rear"][:, 1], a["cmd"][:, 1]]
        info.append(dict(bag=b, veh=b.split("_")[0], h=hash(f.tobytes()) if len(f) else 0,
                         t0=float(th.min()) if len(th) else np.nan,
                         t1=float(th.max()) if len(th) else np.nan, n=len(f)))
    tr, va = split()
    grp = {}
    rows = []
    for i in range(len(info)):
        for j in range(i + 1, len(info)):
            p, q = info[i], info[j]
            if p["veh"] != q["veh"] or not np.isfinite(p["t0"]) or not np.isfinite(q["t0"]):
                continue
            ov = min(p["t1"], q["t1"]) - max(p["t0"], q["t0"])
            if ov > 0:
                same = p["h"] == q["h"] and p["n"] == q["n"]
                sp = lambda b: "train" if b in tr else ("val" if b in va else "-")
                rows.append(dict(a=p["bag"], b=q["bag"], overlap_s=float(ov),
                                 dur_a=p["t1"] - p["t0"], dur_b=q["t1"] - q["t0"],
                                 identical=same, split_a=sp(p["bag"]), split_b=sp(q["bag"])))
    write_csv(OUT / "duplicates.csv", rows)
    for r in rows:
        print(f"  перекрытие {r['a']}[{r['split_a']}] ~ {r['b']}[{r['split_b']}]: "
              f"{r['overlap_s']:.0f} с из {r['dur_a']:.0f}/{r['dur_b']:.0f}, идентичны={r['identical']}")
    # группы (прогоны с перекрытием -> одна группа), для leave-one-out
    parent = {x["bag"]: x["bag"] for x in info}

    def find(x):
        while parent[x] != x:
            x = parent[x]
        return x
    for r in rows:
        parent[find(r["a"])] = find(r["b"])
    for x in info:
        grp[x["bag"]] = find(x["bag"])
    (OUT / "dup_groups.json").write_text(json.dumps(grp, indent=1), encoding="utf-8")
    print(f"пар с перекрытием по времени: {len(rows)}")
    return rows


# ------------------------------------------------------------------ coverage
LAT0, LON0 = 55.80484, 37.42050   # как в build_map.py
# центры конечных станций в городской системе (по старт/концам прогонов, route_overview.png)
TERMINALS = (np.array([-1950.0, -570.0]), np.array([2600.0, 620.0]))


def moving_track(a, vmin=1.0):
    """Точки GNSS master на ходу в городской системе build_map: x, y, курс."""
    m, v = a["mfix"], a["mvel"]
    if len(m) < 100 or len(v) < 100:
        return None
    m = m[(m[:, 5] >= 0) & np.isfinite(m[:, 2])]
    ve = np.interp(m[:, 1], v[:, 1], v[:, 2])
    vn = np.interp(m[:, 1], v[:, 1], v[:, 3])
    mv = np.hypot(ve, vn) > vmin
    P = equirect(m[:, 2], m[:, 3], m[:, 4], (LAT0, LON0, 0.0))
    jump = np.r_[False, np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])) > 5.0]
    sel = mv & ~jump
    return dict(xy=P[sel, :2], h=np.arctan2(ve[sel], vn[sel]), t=m[sel, 1],
                ds=np.r_[0.0, np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))][sel])


def covered(tree, head, xy, h, r, max_dh=math.radians(35.0)):
    """Доля точек xy, у которых есть точка карты в радиусе r с курсом ±max_dh."""
    res = tree.query_ball_point(xy, r)
    ok = np.zeros(len(xy), bool)
    for i, lst in enumerate(res):
        if lst:
            dh = np.abs(np.angle(np.exp(1j * (head[lst] - h[i]))))
            ok[i] = bool((dh <= max_dh).any())
    return ok


def cmd_coverage():
    from scipy.spatial import cKDTree
    from tram_state_estimator.track_map import TrackMap
    tr, va = split()
    tm = TrackMap.load(TRAIN_MAP)
    Pm = equirect(tm.lat, tm.lon, tm.alt, (LAT0, LON0, 0.0))[:, :2]
    tree = cKDTree(Pm)
    rows = []
    radii = (1.0, 2.0, 3.0, 5.0, 10.0)
    for b in va:
        T = moving_track(bagio.load(b))
        if T is None:
            continue
        row = dict(bag=b, n=len(T["xy"]), path_km=float(T["ds"].sum() / 1000))
        for r in radii:
            ok = covered(tree, tm.head, T["xy"], T["h"], r)
            row[f"cov_{r:g}m"] = float(np.sum(T["ds"][ok]) / max(T["ds"].sum(), 1e-9))
        ok3 = covered(tree, tm.head, T["xy"], T["h"], 3.0)
        # самый длинный участок вне карты (по пути)
        run = best = 0.0
        for o, d in zip(ok3, T["ds"]):
            run = 0.0 if o else run + d
            best = max(best, run)
        row["longest_off_3m_m"] = float(best)
        rows.append(row)
    write_csv(OUT / "coverage_val.csv", rows)
    print("отложенные прогоны против обучающей карты (доля пути на ходу в радиусе):")
    for r in rows:
        print(f"  {r['bag']}: " + " ".join(f"{k}={r[k]:.3f}" for k in r if k.startswith("cov_"))
              + f" max_off={r['longest_off_3m_m']:.0f} м")
    # leave-one-group-out по всем прогонам с GNSS: карта из остальных групп,
    # клетка карты — не менее 2 разных прогонов (как в build_map.py)
    grp = json.loads((OUT / "dup_groups.json").read_text(encoding="utf-8"))
    tracks = {}
    for b in ids_all():
        T = moving_track(bagio.load(b))
        if T is not None and T["ds"].sum() > 200:
            tracks[b] = T
    # клетки 1 м × 15° по прогону
    cells = {}
    for b, T in tracks.items():
        cx = np.floor(T["xy"][:, 0]).astype(np.int64)
        cy = np.floor(T["xy"][:, 1]).astype(np.int64)
        hb = np.floor((T["h"] + np.pi) / np.radians(15.0)).astype(np.int64) % 24
        key = (cx * 100000 + cy) * 24 + hb
        cells[b] = dict(zip(key.tolist(), zip(T["xy"][:, 0], T["xy"][:, 1], T["h"])))
    groups = sorted(set(grp[b] for b in tracks))
    by_cell = {}
    for b, cs in cells.items():
        for k in cs:
            by_cell.setdefault(k, set()).add(b)
    rows2 = []
    for b, T in tracks.items():
        g = grp[b]
        pts, hd = [], []
        for k, runs in by_cell.items():
            others = {x for x in runs if grp[x] != g}
            if len(others) >= 2:
                x0 = next(iter(others))
                p = cells[x0][k]
                pts.append(p[:2])
                hd.append(p[2])
        if not pts:
            continue
        tr_ = cKDTree(np.array(pts))
        hd = np.array(hd)
        row = dict(bag=b, split=("val" if b in va else "train"), path_km=float(T["ds"].sum() / 1000))
        for r in (3.0, 10.0):
            ok = covered(tr_, hd, T["xy"], T["h"], r)
            row[f"loo_cov_{r:g}m"] = float(np.sum(T["ds"][ok]) / T["ds"].sum())
            if r == 3.0:
                # где непокрытый путь: у конечных (300 м от центров) или на линии
                dt = np.min(np.stack([np.hypot(*(T["xy"] - c).T) for c in TERMINALS]), axis=0)
                off = ~ok
                row["loo_off_m"] = float(T["ds"][off].sum())
                row["loo_off_terminal_m"] = float(T["ds"][off & (dt < 300.0)].sum())
        rows2.append(row)
    write_csv(OUT / "coverage_loo.csv", rows2)
    c = np.array([r["loo_cov_3m"] for r in rows2])
    print(f"leave-one-group-out ({len(rows2)} прогонов, {len(groups)} групп): покрытие 3 м "
          f"медиана {np.median(c):.3f}, мин {c.min():.3f}; доля прогонов с покрытием "
          f">=0.99: {np.mean(c >= 0.99):.2f}, >=0.95: {np.mean(c >= 0.95):.2f}")
    # только прогоны с хорошим GNSS (RTK > 90 %): непокрытое не из-за шума эталона
    inv = {r["bag"]: r for r in csv.DictReader(open(OUT / "inventory.csv", encoding="utf-8"))}
    good = [r for r in rows2 if float(inv[r["bag"]]["m_st2"]) > 0.9]
    if good:
        c = np.array([r["loo_cov_3m"] for r in good])
        off = sum(r["loo_off_m"] for r in good)
        offt = sum(r["loo_off_terminal_m"] for r in good)
        print(f"  RTK-прогонов {len(good)}: покрытие 3 м медиана {np.median(c):.4f}, мин {c.min():.3f}, "
              f">=0.99: {np.mean(c >= 0.99):.2f}; непокрытый путь {off:.0f} м, из него у конечных "
              f"(<300 м) {offt:.0f} м ({100 * offt / max(off, 1):.0f} %)")
    return rows, rows2


# ------------------------------------------------------------------ run
def _load_map():
    from tram_state_estimator.track_map import TrackMap
    return TrackMap.load(TRAIN_MAP)


def cut_time(a, v_min=8.0, after=300.0):
    """Момент обрезки для имитации «запись начинается на ходу»: первая метка
    master/vel после `after` с от начала, где скорость GNSS > v_min."""
    g = a["mvel"]
    sp = np.hypot(g[:, 2], g[:, 3])
    k = np.flatnonzero((g[:, 1] > g[0, 1] + after) & (sp > v_min))
    return float(g[k[0], 1]) if len(k) else None


def _events(a, drop_rover=False, init_s=INIT_S, t_cut=None):
    """Поток событий как в analysis/evaluate.py (GNSS — первые init_s с по
    времени записи). t_cut — отбросить всё, что раньше (старт на ходу)."""
    if t_cut is not None:
        a = {k: v[v[:, 1] >= t_cut] if len(v) else v for k, v in a.items()}
    ev = []
    for i, key in enumerate(("front", "rear")):
        for tb, th, v in a[key]:
            ev.append((tb, 0, i, th, v))
    for tb, th, n in a["cmd"]:
        ev.append((tb, 1, 0, th, n))
    t_end = a["mfix"][0, 0] + init_s if len(a["mfix"]) else -1
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        if drop_rover and ant == "rover":
            continue
        for row in a[key]:
            if row[0] <= t_end:
                ev.append((row[0], 2, ant, row[1], (row[2], row[3], row[4])))
    ev.sort(key=lambda e: e[0])
    return ev


def _to_strict_enu(tmap):
    """Привязка карты в строгом ENU WGS84 (как pymap3d / GeographicLib
    LocalCartesian) вместо исходный equirect (track_map.py:48-54)."""
    from tram_state_estimator.track_map import CELL

    class MapENU(type(tmap)):
        def bind(self, enu):
            super().bind(enu)
            o = (enu.lat0, enu.lon0, enu.alt0)
            P = enu_true(self.lat, self.lon, self.alt, o)
            self._xy = P[:, :2].copy()
            self._z = P[:, 2].copy()
            cells = np.floor(self._xy / CELL).astype(np.int64)
            self._grid = {}
            for i in np.lexsort((cells[:, 1], cells[:, 0])):
                self._grid.setdefault((int(cells[i, 0]), int(cells[i, 1])), []).append(i)
            self._grid = {c: np.array(v) for c, v in self._grid.items()}
            st = self.stops
            S = enu_true(st[:, 0], st[:, 1], np.zeros(len(st)) + enu.alt0, o)
            self._stop_xy = S[:, :2].copy()

    tmap.__class__ = MapENU
    return tmap


def _strict_origin(r):
    """Начало выставки — строгий ENU (исходный runner.Enu — equirect)."""
    from tram_state_estimator import runner as rmod

    class StrictEnu(rmod.Enu):
        def fwd(self, lat, lon, alt):
            p = enu_true([lat], [lon], [alt], (self.lat0, self.lon0, self.alt0))[0]
            return float(p[0]), float(p[1]), float(p[2])

    orig = r.pos.on_fix

    def on_fix(stamp, antenna, lat, lon, alt, moved, _p=r.pos):
        if _p.enu is None and antenna == "master" and math.isfinite(lat):
            _p.enu = StrictEnu(*(_p.origin or (lat, lon, alt)))
        return orig(stamp, antenna, lat, lon, alt, moved)
    r.pos.on_fix = on_fix
    return r


def make_runner(variant, tmap):
    """Варианты конвейера. Код пакета не меняется: подклассы здесь."""
    import evaluate
    from tram_state_estimator.runner import Runner, Position
    from tram_state_estimator.estimator_core import IS, IV

    params = evaluate.tram_params()
    base = variant.replace("_cut", "")
    strict = base.startswith("enu")        # модификатор: строгий ENU WGS84
    if strict:
        _to_strict_enu(tmap)
        if base == "enu_s1":
            tmap.scale = 1.0
        base = {"enu": "partner", "enu_s1": "partner", "enu_v3b": "v3b_scale"}[base]
    if base in ("partner", "norover", "nomap"):
        r = Runner(params, track_map=(None if base == "nomap" else tmap))
        return _strict_origin(r) if strict else r
    if base in ("v3_scale", "v3b_scale"):
        # исходный код + онлайн-подстройка множителя пути по привязкам к остановкам:
        # сдвиг вдоль пути при привязке / путь с прошлой привязки -> масштаб колёс
        class PositionS(Position):
            """Масштаб = (s0·L_prior + Σ(L_i·scale_i + δ_i)) / (L_prior + ΣL_i):
            δ_i — сдвиг вдоль пути при привязке к остановке, L_i — путь колёс с
            прошлой привязки. L_prior — «априорный» путь (сжатие к s0), ±1 %."""
            L_prior, max_dev, gate_k, gate_c = 1000.0, 0.01, 0.03, 5.0

            def on_stop(self, s):
                c = self._cursor
                if c is None:
                    return
                bx, by, h = c["x"], c["y"], c["h"]
                L = s - self._s_anchor
                if self.map.anchor(c, L):
                    d = (c["x"] - bx) * math.sin(h) + (c["y"] - by) * math.cos(h)
                    if not hasattr(self, "_scale0"):
                        s0 = self.map.scale
                        self._scale0, self._num, self._den = s0, s0 * self.L_prior, self.L_prior
                    if L > 100.0 and abs(d) < self.gate_k * L + self.gate_c:
                        # нужный на отрезке множитель: (L·scale + δ) / L
                        self._num += L * self.map.scale + d
                        self._den += L
                        k = self._num / self._den
                        self.map.scale = min(max(k, self._scale0 * (1 - self.max_dev)),
                                             self._scale0 * (1 + self.max_dev))
                    self._s_anchor = s
                    self.anchors += 1

        if base == "v3b_scale":
            # осторожнее: сильнее сжатие к s0 и узкий допуск на сдвиг привязки
            PositionS.L_prior, PositionS.gate_k, PositionS.gate_c = 3000.0, 0.01, 2.0
        r = Runner(params, track_map=tmap)
        r.pos = PositionS(tmap, None)
        return _strict_origin(r) if strict else r

    class PositionV2(Position):
        """Выставка «на ходу»: начало ENU — первая точка master; курс — по парам
        master/rover одной эпохи (метка), иначе по карте в точке; якорь — ПОСЛЕДНЯЯ
        точка master окна с путём колёс на её метку (без заморозки по moved)."""

        def __init__(self, *a_, runner=None, **k):
            super().__init__(*a_, **k)
            self.runner = runner
            self.wide = False     # расширенный поиск курса по карте (v2w)
            self._m = {}          # метка -> (x, y, z, s)
            self._r = {}          # метка -> (x, y, z)
            self._pairs = []

        def _s_at(self, stamp):
            c = self.runner.core
            s = float(c.x[IS])
            t = self.runner.t if self.runner.t is not None else stamp
            return s + float(c.x[IV]) * (stamp - t)

        def on_fix(self, stamp, antenna, lat, lon, alt, moved):
            if not (math.isfinite(lat) and math.isfinite(lon)):
                return
            if self._t0 is None:
                self._t0 = stamp
            if stamp - self._t0 > self.init_window:
                return
            if self.enu is None and antenna == "master":
                from tram_state_estimator.runner import Enu
                self.enu = Enu(*(self.origin or (lat, lon, alt)))
            if self.enu is None:
                return
            key = round(stamp, 2)
            p = self.enu.fwd(lat, lon, alt)
            if antenna == "master":
                self._m[key] = (*p, self._s_at(stamp))
            else:
                self._r[key] = p
            if key in self._m and key in self._r:
                mx, my, mz, _ = self._m[key]
                rx, ry, _ = self._r[key]
                if math.hypot(rx - mx, ry - my) > 8.0:
                    self._pairs.append((rx - mx, ry - my))
            if not self._m:
                return
            # якорь — последняя точка master
            last = max(self._m)
            mx, my, mz, s_fix = self._m[last]
            first = min(self._m)
            dx, dy = mx - self._m[first][0], my - self._m[first][1]
            if self._pairs:
                P = np.array(self._pairs)
                az = math.atan2(P[:, 0].sum(), P[:, 1].sum())
            elif math.hypot(dx, dy) > 5.0:
                # rover нет, но едем: курс по смещению master за окно
                az = math.atan2(dx, dy)
            elif self.map is not None:
                az = None
            else:
                return
            standing = (max(v[3] for v in self._m.values()) - min(v[3] for v in self._m.values())) < 0.3
            if standing:
                A = np.array([v[:3] for v in self._m.values()])
                x0 = tuple(A.mean(0))
            else:
                x0 = (mx, my, mz)
            if self.map is not None:
                if self._cursor is None:
                    self.map.bind(self.enu)
                if az is None:
                    az = self._map_heading(x0)
                    if az is None:
                        return
                self._cursor = self.map.locate(x0, az)
            self.xyz0, self.az = x0, az
            self.ready = True
            self._s = 0.0
            self._s_anchor = 0.0
            self.runner.s0 = s_fix if not standing else self._s_at(stamp)
            self.runner._s0_fixed = True

        def _map_heading(self, x0):
            """Курс ближайшей точки карты (двухпутка: у каждого пути своё
            направление, ближайшая ось однозначна при точности < 1,5 м)."""
            d = np.hypot(self.map._xy[:, 0] - x0[0], self.map._xy[:, 1] - x0[1])
            i = int(np.argmin(d))
            if d[i] < 3.0:
                return float(self.map.head[i])
            if not self.wide or d[i] > 30.0:
                return None
            # стоянка на конечной вне карты (карта — только точки на ходу > 1 м/с):
            # берём самую «езженую» точку в пределах (ближайшая + 5 м)
            c = np.flatnonzero(d <= d[i] + 5.0)
            return float(self.map.head[c[int(np.argmax(self.map.weight[c]))]])

    class RunnerV2(Runner):
        def __init__(self, *a_, **k):
            super().__init__(*a_, **k)
            self._s0_fixed = False
            self.pos = PositionV2(k.get("track_map"), None, runner=self)

        def on_fix(self, stamp, antenna, lat, lon, alt):
            out = self._advance(stamp)
            self.pos.on_fix(stamp, antenna, lat, lon, alt, 0.0)
            return out

    if base in ("v2", "v2_norover", "v2w_norover"):
        r = RunnerV2(params, track_map=tmap)
        r.pos.wide = base.startswith("v2w")
        return r
    raise ValueError(variant)


def _run_one(args):
    b, variant = args
    f = RUNS / variant / f"{b}.npz"
    if f.exists():
        return b, "cached"
    a = bagio.load(b)
    if len(a["mfix"]) < 100 or len(a["mvel"]) < 50:
        return b, "no-gnss"
    tmap = _load_map()
    r = make_runner(variant, tmap)
    t_cut = cut_time(a) if variant.endswith("_cut") else None
    if variant.endswith("_cut") and t_cut is None:
        return b, "no-cut"
    ev = _events(a, drop_rover="norover" in variant, t_cut=t_cut)
    outs = []
    for tb, kind, i, th, val in ev:
        if kind == 0:
            outs += r.on_wheel(i, th, val)
        elif kind == 1:
            outs += r.on_handle(th, val)
        else:
            outs += r.on_fix(th, i, *val)
    T = np.array([o["stamp"] for o in outs])
    X = np.array([[o["x"], o["y"], o["z"]] for o in outs])
    V = np.array([o["v"] for o in outs])
    S = np.array([o["s"] for o in outs])
    ready = np.array([bool(o["pos_ready"]) for o in outs])
    f.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(f, T=T, X=X, V=V, S=S, ready=ready,
                        anchors=getattr(r.pos, "anchors", 0))
    return b, "ok"


def run_variants(variants, ids):
    jobs = [(b, v) for v in variants for b in ids]
    with ProcessPoolExecutor(WORKERS) as ex:
        for (b, v), (_, st) in zip(jobs, ex.map(_run_one, jobs)):
            if st not in ("ok", "cached"):
                print(f"  {v}/{b}: {st}")


# partner      — исходный код пакета как есть (карта train)
# nomap        — без карты: прямая вдоль начального курса
# norover      — нет rover в окне выставки (выставка не состоится)
# v2           — выставка «на ходу» (якорь — последняя точка master, курс по парам одной эпохи)
# v2_norover   — v2 без rover: курс по ближайшей точке карты (< 3 м)
# v2w_norover  — то же, поиск курса по карте до 30 м (стоянка на конечной вне карты)
# enu          — строгий ENU WGS84 в выставке и привязке карты, множитель 0,99777
# enu_s1       — строгий ENU, множитель пути 1,0
# v3_scale     — исходный код + онлайн-масштаб пути по привязкам к остановкам
# v3b_scale    — то же, осторожные настройки
# enu_v3b      — строгий ENU + v3b_scale (рекомендуемая связка)
# *_cut        — запись обрезана: старт на ходу (> 8 м/с, после 300 с)
VARIANTS = ["partner", "nomap", "norover", "v2", "v2_norover", "v2w_norover", "enu", "enu_s1",
            "v3_scale", "v3b_scale", "enu_v3b", "partner_cut", "v2_cut", "v2_norover_cut", "nomap_cut"]


def cmd_run(which="val"):
    tr, va = split()
    inv = {r["bag"]: r for r in csv.DictReader(open(OUT / "inventory.csv", encoding="utf-8"))}
    ids = va if which == "val" else [b for b in ids_all() if inv[b]["gnss"] == "True"]
    run_variants(VARIANTS, ids)


# ------------------------------------------------------------------ score
def nearest(t_out, t_ref):
    j = np.clip(np.searchsorted(t_out, t_ref), 1, len(t_out) - 1)
    j = np.where(np.abs(t_out[j - 1] - t_ref) < np.abs(t_out[j] - t_ref), j - 1, j)
    return j, np.abs(t_out[j] - t_ref) <= TOL


NATIVE = "master@m0 equirect (наше)"
STRICT = "master@m0 ENU WGS84"


def ref_variants(a, t_cut=None, full=True):
    """Эталоны судьи при разных соглашениях. Наш вывод — equirect от первой
    точки master (или строгий ENU у вариантов enu*). dict имя -> (метки, N×3)."""
    m, r = a["mfix"], a["rfix"]
    if t_cut is not None:
        m, r = m[m[:, 1] >= t_cut], r[r[:, 1] >= t_cut]
    ok = np.isfinite(m[:, 2]) & np.isfinite(m[:, 3])
    m = m[ok]
    o = (m[0, 2], m[0, 3], m[0, 4])
    P = equirect(m[:, 2], m[:, 3], m[:, 4], o)
    out = {NATIVE: (m[:, 1], P),
           STRICT: (m[:, 1], enu_true(m[:, 2], m[:, 3], m[:, 4], o))}
    if not full:
        return out
    k = math.cos(math.radians(o[0]))
    out["master@m0 сфера R=6371 км"] = (m[:, 1], np.c_[np.radians(m[:, 3] - o[1]) * 6371000.0 * k,
                                                     np.radians(m[:, 2] - o[0]) * 6371000.0,
                                                     m[:, 4] - o[2]])
    U = utm(m[:, 2], m[:, 3])
    out["master@m0 UTM 37N"] = (m[:, 1], np.c_[U - U[0], m[:, 4] - o[2]])
    out["master@среднее 1 с"] = (m[:, 1], P - P[m[:, 1] <= m[0, 1] + 1.0].mean(0))
    out["master@среднее 3 с"] = (m[:, 1], P - P[m[:, 1] <= m[0, 1] + 3.0].mean(0))
    out["master@среднее всего прогона"] = (m[:, 1], P - P.mean(0))
    if len(r):
        R_ = equirect(r[:, 2], r[:, 3], r[:, 4], o)
        out["master@r0 (начало — первая точка rover)"] = (m[:, 1], P - R_[0])
        out["rover@r0 (эталон — rover)"] = (r[:, 1], R_ - R_[0])
        mm = a["mfix"] if t_cut is None else a["mfix"][a["mfix"][:, 1] >= t_cut]
        aa = dict(mfix=mm, rfix=r)
        im, ir = pair_master_rover(aa)
        if len(im):
            mid = 0.5 * (equirect(mm[im, 2], mm[im, 3], mm[im, 4], o) + R_[ir])
            out["mid@mid0 (середина базы)"] = (mm[im, 1], mid - mid[0])
            out["mid@m0"] = (mm[im, 1], mid)
    Pc = equirect(m[:, 2], m[:, 3], m[:, 4], (LAT0, LON0, 150.0))
    out["master@город (фикс. начало)"] = (m[:, 1], Pc)
    return out


def _metrics(T, X, tr, P):
    j, ok = nearest(T, tr)
    if ok.sum() < 10:
        return None
    E = X[j[ok]] - P[ok]
    d3 = np.linalg.norm(E, axis=1)
    return dict(mean3d=float(d3.mean()), end3d=float(d3[-1]), max3d=float(d3.max()),
                p95_3d=float(np.percentile(d3, 95)),
                rmse3d=float(np.sqrt(np.mean(d3 ** 2))),
                mx=float(np.mean(np.abs(E[:, 0]))), my=float(np.mean(np.abs(E[:, 1]))),
                mz=float(np.mean(np.abs(E[:, 2]))), n=int(ok.sum()))


def _track(T, X, tr, P):
    """along/cross-track относительно касательной к эталону."""
    j, ok = nearest(T, tr)
    Pk, Xk = P[ok], X[j[ok]]
    d = np.diff(Pk[:, :2], axis=0)
    tang = np.r_[d, d[-1:]] if len(d) else np.zeros((1, 2))
    nrm = np.hypot(tang[:, 0], tang[:, 1])
    good = nrm > 0.05
    tang[good] /= nrm[good, None]
    E = Xk[:, :2] - Pk[:, :2]
    along = np.sum(E * tang, axis=1)
    cross = E[:, 0] * tang[:, 1] - E[:, 1] * tang[:, 0]
    path = float(np.sum(np.hypot(*np.diff(Pk[:, :2], axis=0).T)))
    return dict(along_mean=float(np.mean(np.abs(along[good]))) if good.any() else 0.0,
                along_rmse=float(np.sqrt(np.mean(along[good] ** 2))) if good.any() else 0.0,
                along_max=float(np.max(np.abs(along[good]))) if good.any() else 0.0,
                cross_mean=float(np.mean(np.abs(cross[good]))) if good.any() else 0.0,
                cross_p95=float(np.percentile(np.abs(cross[good]), 95)) if good.any() else 0.0,
                drift_end_pct=float(100 * np.linalg.norm(Xk[-1] - Pk[-1]) / max(path, 1.0)),
                path=path)


def score_one(b, variant, full=False):
    if not (RUNS / "partner" / f"{b}.npz").exists():
        return None
    a = bagio.load(b)
    t_cut = cut_time(a) if variant.endswith("_cut") else None
    if variant.endswith("_cut") and t_cut is None:
        return None
    refs = ref_variants(a, t_cut, full=full)
    native = STRICT if variant.startswith("enu") else NATIVE
    if variant == "static":
        # вывод «стоим в точке старта» (0, 0, 0)
        tr, P = refs[native]
        T, X, anchors = tr.copy(), np.zeros_like(P), 0
    else:
        f = RUNS / variant / f"{b}.npz"
        if not f.exists():
            return None
        z = np.load(f)
        T, X, anchors = z["T"], z["X"], int(z["anchors"])
    res = {}
    for name, (tr, P) in refs.items():
        mm = _metrics(T, X, tr, P)
        if mm is not None:
            res[name] = mm
    res["_native"] = dict(res[native], **_track(T, X, *refs[native]), anchors=anchors,
                          ref=native)
    return res


def _score_job(args):
    b, v = args
    return b, v, score_one(b, v, full=(v in ("partner", "enu_s1")))


def cmd_score(which="val"):
    tr, va = split()
    ids = va if which == "val" else sorted(p.stem for p in (RUNS / "partner").glob("*.npz"))
    ids = [b for b in ids if (RUNS / "partner" / f"{b}.npz").exists()]
    variants = VARIANTS + ["static"]
    jobs = [(b, v) for v in variants for b in ids]
    with ProcessPoolExecutor(WORKERS) as ex:
        res = list(ex.map(_score_job, jobs))
    table = {}
    for b, v, r in res:
        if r is not None:
            table.setdefault(v, {})[b] = r
    (OUT / f"score_{which}.json").write_text(json.dumps(table, indent=1, ensure_ascii=False),
                                               encoding="utf-8")
    rows = []
    for v in variants:
        per = table.get(v, {})
        bs = sorted(per)
        if not bs:
            continue
        nat = [per[b]["_native"] for b in bs]
        w = np.array([x["n"] for x in nat], float)
        row = dict(variant=v, ref=nat[0]["ref"], runs=len(bs),
                   mean3d_w=float(np.average([x["mean3d"] for x in nat], weights=w)),
                   mean3d_median=float(np.median([x["mean3d"] for x in nat])),
                   end3d_median=float(np.median([x["end3d"] for x in nat])),
                   max3d_max=float(np.max([x["max3d"] for x in nat])),
                   mx=float(np.average([x["mx"] for x in nat], weights=w)),
                   my=float(np.average([x["my"] for x in nat], weights=w)),
                   mz=float(np.average([x["mz"] for x in nat], weights=w)),
                   along_mean=float(np.average([x["along_mean"] for x in nat], weights=w)),
                   cross_mean=float(np.average([x["cross_mean"] for x in nat], weights=w)),
                   drift_end_pct_median=float(np.median([x["drift_end_pct"] for x in nat])))
        # тот же вывод против строгого ENU (что увидит судья с pymap3d/GeographicLib)
        if all(STRICT in per[b] for b in bs):
            row["mean3d_w_vs_strictENU"] = float(np.average([per[b][STRICT]["mean3d"] for b in bs], weights=w))
        rows.append(row)
    write_csv(OUT / f"summary_{which}.csv", rows)
    print(f"{'вариант':<16}{'эталон':<28}{'N':>3}{'ср3D':>8}{'мед':>7}{'конец':>7}{'макс':>8}"
          f"{'|x|':>6}{'|y|':>6}{'|z|':>6}{'along':>7}{'cross':>7}{'vsENU':>8}")
    for r in rows:
        print(f"{r['variant']:<16}{r['ref'][:27]:<28}{r['runs']:>3}{r['mean3d_w']:8.1f}"
              f"{r['mean3d_median']:7.1f}{r['end3d_median']:7.1f}{r['max3d_max']:8.0f}"
              f"{r['mx']:6.1f}{r['my']:6.1f}{r['mz']:6.2f}{r['along_mean']:7.1f}{r['cross_mean']:7.1f}"
              f"{r.get('mean3d_w_vs_strictENU', float('nan')):8.1f}")
    # чувствительность к соглашению судьи
    rows2 = []
    for v in ("partner", "enu_s1"):
        per = table.get(v, {})
        if not per:
            continue
        names = [n for n in next(iter(per.values())) if not n.startswith("_")]
        for n in names:
            bs = [b for b in per if n in per[b]]
            w = np.array([per[b][n]["n"] for b in bs], float)
            rows2.append(dict(output=v, convention=n, runs=len(bs),
                              mean3d_w=float(np.average([per[b][n]["mean3d"] for b in bs], weights=w)),
                              mean3d_median=float(np.median([per[b][n]["mean3d"] for b in bs])),
                              end3d_median=float(np.median([per[b][n]["end3d"] for b in bs])),
                              mz=float(np.average([per[b][n]["mz"] for b in bs], weights=w))))
    write_csv(OUT / f"origin_sensitivity_{which}.csv", rows2)
    print("\nчувствительность к соглашению судьи:")
    for r in rows2:
        print(f"  {r['output']:<8} {r['convention']:<42} ср.3D(взв) {r['mean3d_w']:10.1f} м  медиана "
              f"{r['mean3d_median']:10.1f}  конец(медиана) {r['end3d_median']:10.1f}  |z| {r['mz']:.2f}")
    return table


# ------------------------------------------------------------------ branches
def cmd_branches(which="val", thr_end=30.0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from tram_state_estimator.track_map import TrackMap
    tm = TrackMap.load(TRAIN_MAP)
    tr, va = split()
    ids = va if which == "val" else sorted(p.stem for p in (RUNS / "partner").glob("*.npz"))
    rows = []
    for b in ids:
        f = RUNS / "partner" / f"{b}.npz"
        if not f.exists():
            continue
        z = np.load(f)
        T, X = z["T"], z["X"]
        a = bagio.load(b)
        tref, P = ref_variants(a, full=False)[NATIVE]
        j, ok = nearest(T, tref)
        Pk, Xk, tk = P[ok], X[j[ok]], tref[ok]
        d2 = np.hypot(*(Xk[:, :2] - Pk[:, :2]).T)
        if d2[-1] < thr_end and d2.max() < 3 * thr_end:
            continue
        # момент ухода: первое время, после которого ошибка > 15 м держится >= 30 с
        bad = d2 > 15.0
        k0 = None
        for k in np.flatnonzero(bad):
            kk = np.searchsorted(tk, tk[k] + 30.0)
            if kk <= len(bad) and bad[k:kk].all():
                k0 = int(k)
                break
        s_ref = np.r_[0, np.cumsum(np.hypot(*np.diff(Pk[:, :2], axis=0).T))]
        row = dict(bag=b, end_err=float(d2[-1]), max_err=float(d2.max()),
                   mean_err=float(np.mean(d2)), path_km=float(s_ref[-1] / 1000))
        # вдоль/поперёк в точке максимума ошибки: вдоль — дрейф масштаба, поперёк — ветка
        km = int(np.argmax(d2))
        k1 = min(km + 5, len(Pk) - 1)
        k_ = max(km - 5, 0)
        tng = Pk[k1, :2] - Pk[k_, :2]
        nt = np.hypot(*tng)
        if nt > 0.5:
            tng = tng / nt
            e = Xk[km, :2] - Pk[km, :2]
            row["along_at_max"] = float(e @ tng)
            row["cross_at_max"] = float(e[0] * tng[1] - e[1] * tng[0])
        c = equirect([a["mfix"][0, 2]], [a["mfix"][0, 3]], [0.0], (LAT0, LON0, 0.0))[0]
        row["start_city_x"], row["start_city_y"] = float(c[0]), float(c[1])
        if k0 is not None:
            # где разошлись: последняя точка до k0 с ошибкой < 5 м
            kk = np.flatnonzero(d2[:k0] < 5.0)
            kd = int(kk[-1]) if len(kk) else 0
            row.update(t_div=float(tk[kd] - tk[0]), s_div_km=float(s_ref[kd] / 1000),
                       x_div=float(Pk[kd, 0]), y_div=float(Pk[kd, 1]),
                       frac_after=float(1 - s_ref[kd] / max(s_ref[-1], 1)),
                       recovered=bool(d2[-1] < 10.0))
            # ветки у точки ухода: точки карты в радиусе 30 м по курсу эталона
            tm.bind(type("E", (), dict(lat0=a["mfix"][0, 2], lon0=a["mfix"][0, 3],
                                       alt0=a["mfix"][0, 4]))())
            dd = np.hypot(tm._xy[:, 0] - Pk[kd, 0], tm._xy[:, 1] - Pk[kd, 1])
            row["map_pts_30m"] = int((dd < 30).sum())
            # веса карты на эталонной и на нашей ветке через 50 м после ухода
            kr = min(len(Pk) - 1, int(np.searchsorted(s_ref, s_ref[kd] + 60.0)))
            dr = np.hypot(tm._xy[:, 0] - Pk[kr, 0], tm._xy[:, 1] - Pk[kr, 1])
            do = np.hypot(tm._xy[:, 0] - Xk[kr, 0], tm._xy[:, 1] - Xk[kr, 1])
            row["w_ref_branch"] = float(tm.weight[dr < 3].max()) if (dr < 3).any() else 0.0
            row["w_our_branch"] = float(tm.weight[do < 3].max()) if (do < 3).any() else 0.0
            row["ref_on_map"] = bool((dr < 3).any())
        rows.append(row)
        # рисунок
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
        tm.bind(type("E", (), dict(lat0=a["mfix"][0, 2], lon0=a["mfix"][0, 3], alt0=a["mfix"][0, 4]))())
        ax[0].scatter(tm._xy[:, 0], tm._xy[:, 1], s=0.2, c="0.8", label="карта (train)")
        ax[0].plot(Pk[:, 0], Pk[:, 1], "b-", lw=1, label="GNSS master")
        ax[0].plot(Xk[:, 0], Xk[:, 1], "r-", lw=1, label="оценка (partner)")
        if k0 is not None:
            ax[0].plot(Pk[kd, 0], Pk[kd, 1], "ko", ms=8, mfc="none", label="расхождение")
            cx, cy = Pk[kd, 0], Pk[kd, 1]
            ax[1].scatter(tm._xy[:, 0], tm._xy[:, 1], s=2, c=tm.weight, cmap="viridis")
            ax[1].plot(Pk[:, 0], Pk[:, 1], "b-", lw=1.5)
            ax[1].plot(Xk[:, 0], Xk[:, 1], "r--", lw=1.5)
            ax[1].set_xlim(cx - 150, cx + 150)
            ax[1].set_ylim(cy - 150, cy + 150)
            ax[1].set_title("окрестность расхождения (цвет — вес карты)")
            ax[1].set_aspect("equal")
        ax[0].set_aspect("equal")
        ax[0].legend(fontsize=7)
        ax[0].set_title(f"{b}: ошибка конца {d2[-1]:.0f} м, макс {d2.max():.0f} м")
        fig.tight_layout()
        fig.savefig(OUT / f"branch_{b}.png", dpi=110)
        plt.close(fig)
    write_csv(OUT / f"branches_{which}.csv", rows)
    for r in rows:
        print("  " + ", ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in r.items()))
    return rows


def cmd_overview():
    """Схема маршрута: карта (train) по весу, старты/концы прогонов, остановки."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from tram_state_estimator.track_map import TrackMap
    tm = TrackMap.load(TRAIN_MAP)
    P = equirect(tm.lat, tm.lon, tm.alt, (LAT0, LON0, 0.0))
    S = equirect(tm.stops[:, 0], tm.stops[:, 1], np.zeros(len(tm.stops)), (LAT0, LON0, 0.0))
    inv = [r for r in csv.DictReader(open(OUT / "inventory.csv", encoding="utf-8")) if r["gnss"] == "True"]
    st = equirect(np.array([float(r["start_lat"]) for r in inv]), np.array([float(r["start_lon"]) for r in inv]),
                  np.zeros(len(inv)), (LAT0, LON0, 0.0))
    en = equirect(np.array([float(r["end_lat"]) for r in inv]), np.array([float(r["end_lon"]) for r in inv]),
                  np.zeros(len(inv)), (LAT0, LON0, 0.0))
    fig, ax = plt.subplots(figsize=(14, 5))
    sc = ax.scatter(P[:, 0], P[:, 1], s=0.3, c=np.log2(tm.weight), cmap="viridis")
    ax.plot(st[:, 0], st[:, 1], "g^", ms=6, label="старт прогона")
    ax.plot(en[:, 0], en[:, 1], "rv", ms=6, mfc="none", label="конец прогона")
    ax.plot(S[:, 0], S[:, 1], "k+", ms=7, label="точки остановок (train)")
    plt.colorbar(sc, ax=ax, label="log2(вес = число прогонов)")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.set_title("Карта путей (только обучающие прогоны), старты и концы прогонов с GNSS")
    fig.tight_layout()
    fig.savefig(OUT / "route_overview.png", dpi=110)
    plt.close(fig)
    # точки старта: сколько различных мест
    from scipy.cluster.hierarchy import fcluster, linkage
    for name, X in (("старт", st), ("конец", en)):
        cl = fcluster(linkage(X[:, :2], "single"), 50.0, "distance")
        u, c = np.unique(cl, return_counts=True)
        print(f"{name}: {len(u)} мест (кластеры 50 м), размеры {sorted(c.tolist(), reverse=True)}")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    arg = sys.argv[2] if len(sys.argv) > 2 else "val"
    if cmd in ("inventory", "all"):
        cmd_inventory()
    if cmd in ("timing", "all"):
        cmd_timing()
    if cmd in ("frames", "all"):
        cmd_frames()
    if cmd in ("dups", "all"):
        cmd_dups()
    if cmd in ("coverage", "all"):
        cmd_coverage()
    if cmd in ("overview", "all"):
        cmd_overview()
    if cmd in ("run", "all"):
        cmd_run(arg)
    if cmd in ("score", "all"):
        cmd_score(arg)
    if cmd in ("branches", "all"):
        cmd_branches(arg)


if __name__ == "__main__":
    main()
