"""Проба: годятся ли тренировочные траектории как карта для отложенных.

Для отложенного прогона: кандидаты — тренировочные траектории, проходящие
в пределах 10 м от точки старта с курсом в пределах 30°. Каждый кандидат
«проходится» на ИСТИННЫЙ путь (по GNSS) — так видна ошибка самой карты и
выбора ветки, без ошибки одометрии.
"""

import json

import numpy as np

import bagio

R = 6378137.0
LAT0, LON0 = 55.80484, 37.42050


def enu(lat, lon, alt):
    k = np.cos(np.radians(LAT0))
    return np.c_[np.radians(lon - LON0) * R * k, np.radians(lat - LAT0) * R, alt]


def traj(b):
    m = bagio.load(b)["mfix"]
    if len(m) < 100:
        return None
    P = enu(m[:, 2], m[:, 3], m[:, 4])
    # выбросы: скачок больше 5 м за отсчёт
    d = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
    keep = np.r_[True, d < 5.0]
    P, t = P[keep], m[keep, 1]
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))]
    return dict(t=t, P=P, s=s)


def heading_at(T, k, span=5.0):
    s = T["s"]
    j = np.searchsorted(s, s[k] + span)
    j = min(j, len(s) - 1)
    d = T["P"][j, :2] - T["P"][k, :2]
    return np.arctan2(d[0], d[1]) if np.hypot(*d) > 1 else np.nan


def main():
    dm = json.loads((bagio.CACHE.parent / "drive_model.json").read_text(encoding="utf-8"))
    train = {b: traj(b) for b in dm["train"]}
    train = {b: T for b, T in train.items() if T is not None and T["s"][-1] > 100}
    for b in dm["val"]:
        V = traj(b)
        if V is None or V["s"][-1] < 100:
            continue
        # точка старта — первая точка движения
        k0 = int(np.searchsorted(V["s"], 1.0))
        h0 = heading_at(V, k0)
        p0 = V["P"][k0, :2]
        cands = []
        for tb, T in train.items():
            dd = np.linalg.norm(T["P"][:, :2] - p0, axis=1)
            for k in np.flatnonzero(dd < 10.0)[::20]:
                h = heading_at(T, k)
                if np.isfinite(h) and abs(np.angle(np.exp(1j * (h - h0)))) < np.radians(30):
                    cands.append((tb, k))
                    break
        errs = []
        for tb, k in cands:
            T = train[tb]
            sv = V["s"] - V["s"][k0]
            st = T["s"][k] + sv
            ok = (sv >= 0) & (st <= T["s"][-1])
            px = np.interp(st[ok], T["s"], T["P"][:, 0])
            py = np.interp(st[ok], T["s"], T["P"][:, 1])
            pz = np.interp(st[ok], T["s"], T["P"][:, 2])
            e = np.linalg.norm(np.c_[px, py, pz] - V["P"][ok], axis=1)
            cover = ok.sum() / max((sv >= 0).sum(), 1)
            errs.append((float(np.median(e)), float(np.percentile(e, 90)), cover, tb))
        errs.sort()
        good = [x for x in errs if x[2] > 0.9]
        print(f"{b}: путь {V['s'][-1]/1000:.1f} км, кандидатов {len(cands)}, "
              f"покрывают весь прогон {len(good)}; лучший: медиана {errs[0][0] if errs else np.nan:.1f} м "
              f"p90 {errs[0][1] if errs else np.nan:.1f} м покрытие {errs[0][2] if errs else 0:.2f}; "
              f"медиана по кандидатам, покрывающим всё: "
              f"{np.median([x[0] for x in good]) if good else np.nan:.1f} м")


if __name__ == "__main__":
    main()
