#!/usr/bin/env python3
"""Инъекции аномалий во входной поток реального прогона (общий модуль).

Работает с массивами прогона в формате кэша analysis/bagio.py: ключи
front / rear / cmd — строки (tb, th, значение), где tb — время записи в bag,
th — header.stamp. Инъекция возвращает КОПИЮ массивов; порядок проигрывания
остаётся по tb (как ros2 bag play), меняются значения, метки th или
пропадают сообщения. GNSS не трогается.

Окно аномалии выбирается детерминированно и только по входам решения
(скорость тележек и ручка), поэтому модуль годится и для bag без GNSS
(экспорт прогона для симулятора, tools/export_replay.py).

    import inject
    t0, dur = inject.choose_window(a, "both_zero")
    b, info = inject.apply(a, "both_zero", t0, dur, seed=inject.seed_for(bag, "both_zero"))

Виды (KINDS): см. таблицу ниже; описание каждого — поле "ru".
"""

import zlib

import numpy as np

KMH = 3.6
T_SKIP = 60.0          # с от начала входов: окно не раньше (выставка, трогание)
AFTER = 120.0          # с после окна должно остаться до конца прогона

# name: ru — описание; dur — длительность аномалии, с; eval — длительность
# окна «во время» для метрик, с (для мгновенных аномалий больше dur);
# where — условие выбора окна (см. choose_window).
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
    "noise": dict(ru="шум ×5 (σ = 5·sigma_meas) на обеих тележках", dur=20.0, where="moving"),
    "nan": dict(ru="одно показание передней тележки NaN", dur=0.1, eval=5.0, where="moving_start"),
    "stamp_jump": dict(ru="метка одного сообщения передней тележки +30 с", dur=0.1, eval=30.0,
                       jump=30.0, where="moving_start"),
}
ORDER = tuple(KINDS)


def seed_for(bag, kind):
    """Детерминированное зерно (hash() в Python солится — не годится)."""
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
      moving_start — скорость тележек в t0 не ниже 5 м/с (длинные отказы:
                     внутри окна могут быть остановки, как в жизни);
      moving       — скорость не ниже 4 м/с всё окно;
      brake        — ручка ≤ −1 всё окно, скорость в t0 ≥ 5 и всё окно ≥ 1,5 м/с;
      traction     — ручка ≥ +1 всё окно, скорость в t0 ≥ 2 м/с.
    После окна до конца прогона должно остаться after с (кроме случая, когда
    прогон короче — тогда берётся половина).
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

def apply(a, kind, t0, dur=None, seed=0, sigma_meas=0.05):
    """Копия массивов прогона с аномалией kind в окне [t0, t0+dur) по th.

    Возвращает (массивы, info): info — окно, число затронутых сообщений и
    описание. sigma_meas — шум измерения скорости оси из листа, м/с (для noise).
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
