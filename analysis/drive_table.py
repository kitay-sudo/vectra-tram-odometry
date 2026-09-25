"""Шаг 2а: какое ускорение даёт позиция ручки на разной скорости.

Скорость — среднее передней и задней тележек (км/ч -> м/с) там, где они
согласны; ускорение — центральная разность на сетке 10 Гц. Ручка берётся с
задержкой L; L подбирается по минимуму остаточной дисперсии.
"""

import numpy as np

import bagio

KMH = 3.6
DT = 0.1


def series(b):
    a = bagio.load(b)
    f, r, c = a["front"], a["rear"], a["cmd"]
    if len(f) < 100 or len(c) < 100:
        return None
    t = np.arange(max(f[0, 0], r[0, 0], c[0, 0]), min(f[-1, 0], r[-1, 0], c[-1, 0]), DT)
    vf = np.interp(t, f[:, 0], f[:, 2]) / KMH
    vr = np.interp(t, r[:, 0], r[:, 2]) / KMH
    ok = np.abs(vf - vr) < np.maximum(0.2, 0.03 * np.maximum(vf, vr))
    v = 0.5 * (vf + vr)
    acc = np.full_like(v, np.nan)
    acc[3:-3] = (v[6:] - v[:-6]) / (6 * DT)          # окно 0,6 с
    # пропуски данных: разность по интерполяции через дыру недостоверна
    tf = f[:, 0]
    gap = np.interp(t, tf[1:], np.diff(tf)) > 0.35
    ok &= ~gap
    return t, v, acc, ok, c


def notch_at(c, t):
    i = np.clip(np.searchsorted(c[:, 0], t, side="right") - 1, 0, len(c) - 1)
    return c[i, 2]


def main():
    data = [(b, s) for b in bagio.bag_ids() if (s := series(b)) is not None]
    print(f"прогонов: {len(data)}")

    # задержка ручки -> ускорение
    res = []
    for L in np.arange(0.0, 2.01, 0.2):
        N, A, V = [], [], []
        for _, (t, v, acc, ok, c) in data:
            n = notch_at(c, t - L)
            m = ok & np.isfinite(acc) & (v > 1.0)
            N.append(n[m]); A.append(acc[m]); V.append(v[m])
        N, A, V = map(np.concatenate, (N, A, V))
        vb = np.clip((V // 2).astype(int), 0, 9)
        key = (N.astype(int) + 15) * 10 + vb
        s = np.bincount(key, A, minlength=310)
        s2 = np.bincount(key, A * A, minlength=310)
        k = np.bincount(key, minlength=310)
        resid = float((s2 - np.where(k > 0, s * s / np.maximum(k, 1), 0)).sum() / k.sum())
        res.append((resid, L))
        print(f"  задержка {L:.1f} с: остаточная дисперсия {resid:.4f} (м/с²)²")
    L = min(res)[1]
    print(f"лучшая задержка {L:.1f} с")

    N, A, V = [], [], []
    for _, (t, v, acc, ok, c) in data:
        n = notch_at(c, t - L)
        m = ok & np.isfinite(acc)
        N.append(n[m]); A.append(acc[m]); V.append(v[m])
    N, A, V = map(np.concatenate, (N, A, V))
    edges = [0.5, 2, 4, 6, 8, 10, 12, 15]
    print("медиана ускорения (м/с²) по позиции ручки и скорости (м/с); в скобках — отсчётов/100")
    print("позиция " + " ".join(f"{edges[i]:>4.1f}-{edges[i+1]:<4.1f}" for i in range(len(edges) - 1)))
    for p in range(-15, 16):
        row = []
        for i in range(len(edges) - 1):
            m = (N == p) & (V >= edges[i]) & (V < edges[i + 1])
            row.append(f"{np.median(A[m]):+5.2f}({m.sum() // 100:3d})" if m.sum() >= 200 else "     --    ")
        print(f"{p:+3d}    " + " ".join(row))
    np.savez_compressed(bagio.CACHE.parent / "drive_samples.npz", N=N, A=A, V=V, L=L)


if __name__ == "__main__":
    main()
