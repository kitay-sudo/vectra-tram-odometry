#!/usr/bin/env python3
"""Инъекции аномалий во входной поток реального прогона (общий модуль).

Работает с массивами прогона в формате кэша analysis/bagio.py: ключи
front / rear / cmd - строки (tb, th, значение), где tb - время записи в bag,
th - header.stamp. Инъекция возвращает КОПИЮ массивов; порядок проигрывания
остаётся по tb (как ros2 bag play), меняются значения, метки th или
пропадают сообщения. GNSS аномалии входов не трогают; сценарии доступности
и сбоев GNSS - отдельно (gnss_scenario, cut_start; раздел «GNSS» ниже).

Окно аномалии выбирается детерминированно и только по входам решения
(скорость тележек и ручка), поэтому модуль годится и для bag без GNSS
(экспорт прогона для симулятора, tools/export_replay.py).

    import inject
    t0, dur = inject.choose_window(a, "both_zero")
    b, info = inject.apply(a, "both_zero", t0, dur, seed=inject.seed_for(bag, "both_zero"))

Виды (KINDS): см. таблицу ниже; описание каждого - поле "ru".
"""

import zlib

import numpy as np

KMH = 3.6
T_SKIP = 60.0          # с от начала входов: окно не раньше (выставка, трогание)
AFTER = 120.0          # с после окна должно остаться до конца прогона
NOISE_SIGMA_MS = 0.05  # м/с: база σ шума (×5) - абсолютная, от листа не зависит

# name: ru - описание; dur - длительность аномалии, с; eval - длительность
# окна «во время» для метрик, с (для мгновенных аномалий больше dur);
# where - условие выбора окна (см. choose_window).
KINDS = {
    "front_zero": dict(ru="отказ передней тележки: показания 0", dur=60.0, where="moving_start"),
    "rear_drop": dict(ru="отказ задней тележки как в 30639: 1 с нулей, затем 73,5 с нет сообщений",
                      dur=73.5, where="moving_start"),
    "both_zero": dict(ru="обе тележки показывают 0", dur=20.0, where="moving"),
    "both_stuck": dict(ru="обе тележки залипли на последнем значении", dur=20.0, where="moving"),
    "dropout": dict(ru="пропуск сообщений обеих тележек", dur=2.0, eval=5.0, where="moving"),
    "gap_all": dict(ru="пропуск всех входов (тележки и ручка)", dur=2.0, eval=5.0, where="moving"),
    "skid_brake": dict(ru="юз: обе тележки −30 % на торможении", dur=4.0, where="brake"),
    "spin_traction": dict(ru="буксование: обе тележки +30 % на разгоне", dur=4.0, where="traction"),
    "outliers": dict(ru="выбросы ×3: 1, 2 и 3 сообщения подряд (через 5 с)", dur=12.0, where="moving"),
    "noise": dict(ru="шум ×5: σ = 0,25 м/с (5 × 0,05 м/с) на обеих тележках", dur=20.0, where="moving"),
    "nan": dict(ru="одно показание передней тележки NaN", dur=0.1, eval=5.0, where="moving_start"),
    "stamp_jump": dict(ru="метка одного сообщения передней тележки +30 с", dur=0.1, eval=30.0,
                       jump=30.0, where="moving_start"),
}
ORDER = tuple(KINDS)


def seed_for(bag, kind):
    """Детерминированное зерно (hash() в Python солится - не годится)."""
    return zlib.crc32(f"{bag}|{kind}".encode("utf-8"))


def eval_window(kind):
    """Длительность окна «во время» для метрик, с."""
    k = KINDS[kind]
    return float(k.get("eval", k["dur"]))


# ------------------------------------------------------------ выбор окна

def _zoh(t_src, v_src, t_q):
    o = np.argsort(t_src, kind="stable")
    ts, vs = t_src[o], v_src[o]
    j = np.searchsorted(ts, t_q, side="right") - 1
    return np.where(j >= 0, vs[np.clip(j, 0, None)], np.nan)


def input_grid(a, step=0.1):
    """Сетка по меткам входов: t, скорость тележек (м/с, без meas_scale), ручка."""
    f, r, c = a["front"], a["rear"], a["cmd"]
    ts = [x[:, 1] for x in (f, r, c) if len(x)]
    t_a = min(float(np.min(x)) for x in ts)
    t_b = max(float(np.max(x)) for x in ts)
    t = np.arange(t_a, t_b, step)
    ws = []
    for x in (f, r):
        if len(x) >= 2:
            o = np.argsort(x[:, 1], kind="stable")
            ws.append(np.interp(t, x[o, 1], x[o, 2]))
    w = np.nanmean(np.vstack(ws), axis=0) / KMH if ws else np.zeros_like(t)
    h = np.nan_to_num(_zoh(c[:, 1], c[:, 2], t), nan=0.0) if len(c) else np.zeros_like(t)
    return t, w, h


def choose_window(a, kind, t_skip=T_SKIP, after=AFTER):
    """Начало окна t0 (header.stamp) и длительность аномалии dur, либо (None, dur).

    Первое по времени окно после t_skip от начала входов, где:
      moving_start - скорость тележек в t0 не ниже 5 м/с (длинные отказы:
                     внутри окна могут быть остановки, как в жизни);
      moving       - скорость не ниже 4 м/с всё окно;
      brake        - ручка ≤ −1 всё окно, скорость в t0 ≥ 5 и всё окно ≥ 1,5 м/с;
      traction     - ручка ≥ +1 всё окно, скорость в t0 ≥ 2 м/с.
    После окна до конца прогона должно остаться after с (кроме случая, когда
    прогон короче - тогда берётся половина).
    """
    k = KINDS[kind]
    dur = float(k["dur"])
    span = max(dur, eval_window(kind))
    t, w, h = input_grid(a)
    if len(t) < 10:
        return None, dur
    dt = t[1] - t[0]
    n = max(1, int(round(span / dt)))
    t_end = t[-1] - (after if t[-1] - t[0] > t_skip + span + after else 0.5 * (t[-1] - t[0]))
    where = k["where"]
    for i in np.flatnonzero(t >= t[0] + t_skip):
        if t[i] + span > t_end:
            break
        seg = slice(i, min(i + n + 1, len(t)))
        ws, hs = w[seg], h[seg]
        if where == "moving_start":
            ok = w[i] >= 5.0
        elif where == "moving":
            ok = ws.min() >= 4.0
        elif where == "brake":
            ok = hs.max() <= -1 and w[i] >= 5.0 and ws.min() >= 1.5
        elif where == "traction":
            ok = hs.min() >= 1 and w[i] >= 2.0
        else:
            raise ValueError(where)
        if ok:
            return float(t[i]), dur
    return None, dur


# ------------------------------------------------------------ инъекция

def apply(a, kind, t0, dur=None, seed=0, sigma_meas=NOISE_SIGMA_MS):
    """Копия массивов прогона с аномалией kind в окне [t0, t0+dur) по th.

    Возвращает (массивы, info): info - окно, число затронутых сообщений и
    описание. sigma_meas - базовая σ шума, м/с (для noise: σ = 5·sigma_meas).
    По умолчанию абсолютная 0,05 м/с (sigma_meas паспортного листа), а не
    значение текущего листа: иначе смена листа меняла бы саму аномалию.
    """
    k = KINDS[kind]
    dur = float(k["dur"] if dur is None else dur)
    t1 = t0 + dur
    b = {key: v.copy() for key, v in a.items()}
    touched = {}

    def win(x, lo=t0, hi=t1):
        return (x[:, 1] >= lo) & (x[:, 1] < hi)

    if kind == "front_zero":
        w = win(b["front"])
        b["front"][w, 2] = 0.0
        touched["front"] = int(w.sum())
    elif kind == "rear_drop":
        pre = win(b["rear"], t0 - 1.0, t0)
        b["rear"][pre, 2] = 0.0
        w = win(b["rear"])
        b["rear"] = b["rear"][~w]
        touched["rear_zero_before"] = int(pre.sum())
        touched["rear_dropped"] = int(w.sum())
    elif kind in ("both_zero", "both_stuck", "skid_brake", "spin_traction", "noise"):
        rng = np.random.default_rng(seed)
        for key in ("front", "rear"):
            x = b[key]
            w = win(x)
            if kind == "both_zero":
                x[w, 2] = 0.0
            elif kind == "both_stuck":
                before = x[x[:, 1] < t0]
                x[w, 2] = before[-1, 2] if len(before) else 0.0
            elif kind == "skid_brake":
                x[w, 2] *= 0.7
            elif kind == "spin_traction":
                x[w, 2] *= 1.3
            else:
                x[w, 2] += rng.normal(0.0, 5.0 * sigma_meas * KMH, int(w.sum()))
            touched[key] = int(w.sum())
    elif kind in ("dropout", "gap_all"):
        keys = ("front", "rear") + (("cmd",) if kind == "gap_all" else ())
        for key in keys:
            w = win(b[key])
            b[key] = b[key][~w]
            touched[key] = int(w.sum())
    elif kind == "outliers":
        # 1 сообщение передней в t0, 2 задней в t0+5, 3 передней в t0+10
        for key, dt_, cnt in (("front", 0.0, 1), ("rear", 5.0, 2), ("front", 10.0, 3)):
            x = b[key]
            idx = np.flatnonzero(x[:, 1] >= t0 + dt_)[:cnt]
            x[idx, 2] *= 3.0
            touched[f"{key}@+{dt_:g}s"] = int(len(idx))
    elif kind == "nan":
        x = b["front"]
        idx = np.flatnonzero(x[:, 1] >= t0)[:1]
        x[idx, 2] = np.nan
        touched["front"] = int(len(idx))
    elif kind == "stamp_jump":
        x = b["front"]
        idx = np.flatnonzero(x[:, 1] >= t0)[:1]
        x[idx, 1] += float(k["jump"])
        touched["front"] = int(len(idx))
    else:
        raise ValueError(f"неизвестная инъекция {kind}")
    info = dict(kind=kind, ru=k["ru"], t0=float(t0), dur=dur, t1=float(t1),
                eval_s=eval_window(kind), touched=touched)
    return b, info


# ------------------------------------------------------------ GNSS: сценарии доступности

# Сценарии подачи GNSS в связку (tools/eval.py --gnss <имя>, tools/eval_gnss.py).
# Эталон оценки не меняется: он строится по всем точкам GNSS прогона. Окна -
# по времени записи в bag (tb), как ros2 bag play; всё с зерном прогона.
GNSS_SCENARIOS = {
    "first3": "GNSS только первые 3 с от первой точки master (как в проверочных bag)",
    "sparse": "первые 3 с + пачки по 5–10 с каждые 2–3 мин",
    "bursts": "первые 3 с + короткие пачки по 1–4 с, в среднем раз в минуту",
    "nostart": "в начале GNSS нет; первая пачка через 1–3 мин, дальше как sparse "
               "(выставка по GNSS посреди прогона)",
    "midstart": "запись с середины маршрута: входы и эталон с 3–8 мин, GNSS первые 3 с "
                "после этого (старт на ходу)",
    "full": "GNSS весь прогон",
    "glitchy": "sparse + сбои GNSS: скачки 15–80 м на 1–5 эпох, метки ±1 с на 2–4 с, "
               "пачка без RTK со сдвигом 5–15 м, пачка без rover, точки (0, 0), NaN и "
               "статус −1",
    "none": "GNSS нет вовсе",
}
GNSS_FIRST_S = 3.0
M_PER_DEG = 111320.0


def _fix_rows(a):
    """Точки GNSS прогона: список [tb, антенна, th, lat, lon, alt, status]."""
    out = []
    for key, ant in (("mfix", "master"), ("rfix", "rover")):
        x = np.asarray(a.get(key, np.zeros((0, 6))), float)
        if x.ndim != 2 or x.shape[1] < 5:
            continue
        for row in x:
            st = int(row[5]) if len(row) > 5 and np.isfinite(row[5]) else 0
            out.append([float(row[0]), ant, float(row[1]), float(row[2]), float(row[3]),
                        float(row[4]), st])
    return out


def _windows(rng, t0, t1, kind):
    """Окна пачек GNSS после первых 3 с: список (начало, конец) по tb."""
    wins = []
    if kind in ("sparse", "glitchy", "nostart"):
        t = t0 + (rng.uniform(60.0, 180.0) if kind == "nostart" else rng.uniform(120.0, 180.0))
        while t < t1:
            wins.append((t, t + rng.uniform(5.0, 10.0)))
            t += rng.uniform(120.0, 180.0)
    elif kind == "bursts":
        t = t0 + max(10.0, rng.exponential(60.0))
        while t < t1:
            wins.append((t, t + rng.uniform(1.0, 4.0)))
            t += max(10.0, rng.exponential(60.0))
    return wins


def _offset(row, dE, dN):
    lat = row[3]
    row[3] = lat + dN / M_PER_DEG
    row[4] = row[4] + dE / (M_PER_DEG * np.cos(np.radians(lat)))


def gnss_scenario(a, kind, seed):
    """GNSS для подачи в связку по сценарию kind (GNSS_SCENARIOS, кроме
    midstart - для него cut_start). Возвращает (rows, info): rows - список
    [tb, антенна, th, lat, lon, alt, status] по возрастанию tb; info - окна
    и сбои (для отчёта)."""
    if kind not in GNSS_SCENARIOS or kind == "midstart":
        raise ValueError(f"сценарий GNSS {kind!r}: {sorted(GNSS_SCENARIOS)}")
    rows = _fix_rows(a)
    info = dict(kind=kind, windows=[], glitches=[])
    m = [r for r in rows if r[1] == "master"]
    if not m or kind == "none":
        return [], info
    t0 = min(r[0] for r in m)
    t1 = max(r[0] for r in rows)
    if kind == "full":
        return sorted(rows, key=lambda r: r[0]), info
    rng = np.random.default_rng(seed)
    wins = [] if kind == "nostart" else [(-np.inf, t0 + GNSS_FIRST_S)]
    burst = _windows(rng, t0, t1, kind)
    wins += burst
    info["windows"] = [(float(a_ - t0), float(b_ - t0)) for a_, b_ in burst]
    out = [list(r) for r in rows if any(lo <= r[0] <= hi for lo, hi in wins)]
    if kind == "glitchy":
        out = _glitch(out, burst, rng, info, t0)
    return sorted(out, key=lambda r: r[0]), info


def _glitch(rows, burst, rng, info, t0):
    """Сбои GNSS в пачках (не в первых 3 с): скачок, сдвиг метки, пачка без
    RTK со сдвигом, пачка без rover, мусорные точки."""
    out = rows
    for lo, hi in burst:
        inb = [r for r in out if lo <= r[0] <= hi]
        if not inb:
            continue
        epochs = sorted({round(r[2], 2) for r in inb if r[1] == "master"})
        if not epochs:
            continue
        rel = float(lo - t0)
        if rng.random() < 0.5:                  # скачок положения на 1–5 эпох
            n = int(rng.integers(1, 6))
            k = int(rng.integers(0, max(1, len(epochs) - n)))
            sel = set(epochs[k:k + n])
            d, ang = rng.uniform(15.0, 80.0), rng.uniform(0.0, 2 * np.pi)
            for r in inb:
                if round(r[2], 2) in sel:
                    _offset(r, d * np.sin(ang), d * np.cos(ang))
            info["glitches"].append(dict(t=rel, kind="jump", epochs=n, m=float(d)))
        if rng.random() < 0.3:                  # метки ±1 с на 2–4 с (положение то же)
            a_ = float(rng.uniform(lo, max(lo, hi - 2.0)))
            b_ = a_ + float(rng.uniform(2.0, 4.0))
            sh = float(rng.choice([-1.0, 1.0]))
            for r in inb:
                if a_ <= r[0] <= b_:
                    r[2] += sh
            info["glitches"].append(dict(t=rel, kind="stamp", shift=sh, s=b_ - a_))
        if rng.random() < 0.25:                 # пачка без RTK со сдвигом 5–15 м
            d, ang = rng.uniform(5.0, 15.0), rng.uniform(0.0, 2 * np.pi)
            for r in inb:
                r[6] = 0
                _offset(r, d * np.sin(ang), d * np.cos(ang))
            info["glitches"].append(dict(t=rel, kind="degraded", m=float(d)))
        if rng.random() < 0.25:                 # пачка без rover
            drop = {id(r) for r in inb if r[1] == "rover"}
            out = [r for r in out if id(r) not in drop]
            info["glitches"].append(dict(t=rel, kind="no_rover"))
        if rng.random() < 0.3:                  # мусор: (0, 0), NaN, статус −1
            tb, th = inb[0][0], inb[0][2]
            out += [[tb + 0.01, "master", th, 0.0, 0.0, 0.0, 0],
                    [tb + 0.02, "rover", th, float("nan"), inb[0][4], inb[0][5], 2],
                    [tb + 0.03, "master", th + 0.05, inb[0][3], inb[0][4], inb[0][5], -1]]
            info["glitches"].append(dict(t=rel, kind="garbage"))
    return out


def cut_start(a, seed, lo=180.0, hi=480.0):
    """Запись с середины маршрута: все топики с tb не раньше t_cut (от
    первого входа + U(lo, hi) с). Возвращает (массивы, секунды отрезано)."""
    ts = [float(a[k][0, 0]) for k in ("front", "rear", "cmd") if len(a[k])]
    t_a = min(ts)
    t_end = max(float(a[k][-1, 0]) for k in ("front", "rear", "cmd") if len(a[k]))
    cut = float(np.random.default_rng(seed).uniform(lo, hi))
    cut = min(cut, max(0.0, 0.5 * (t_end - t_a)))
    t_cut = t_a + cut
    return {k: (v[v[:, 0] >= t_cut] if len(v) else v) for k, v in a.items()}, cut


def main():
    import argparse
    import json
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / "analysis"))
    import bagio
    ap = argparse.ArgumentParser(description="Показать окна инъекций для прогона")
    ap.add_argument("bag")
    ap.add_argument("--kinds", default=",".join(ORDER))
    args = ap.parse_args()
    a = bagio.load(args.bag)
    t_first = min(float(a[k][:, 1].min()) for k in ("front", "rear", "cmd") if len(a[k]))
    rows = {}
    for kind in args.kinds.split(","):
        t0, dur = choose_window(a, kind)
        if t0 is None:
            rows[kind] = None
            continue
        _, info = apply(a, kind, t0, dur, seed=seed_for(args.bag, kind))
        info["t0_rel_s"] = round(t0 - t_first, 2)
        rows[kind] = info
    print(json.dumps(rows, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
