"""Разбор данных: сводка по всем прогонам."""

import numpy as np

import bagio


def gaps(t, thr):
    d = np.diff(t)
    return int((d > thr).sum()), float(d.max()) if len(d) else 0.0


def gnss_speed(a):
    v = a["mvel"]
    return v[:, 1], np.hypot(v[:, 2], v[:, 3])


def best_lag(t_ref, x_ref, t, x, lags):
    """Задержка x относительно x_ref (с), по минимуму СКО на движении."""
    best = (np.inf, 0.0)
    for L in lags:
        xi = np.interp(t_ref, t - L, x)
        m = x_ref > 1.0
        if m.sum() < 100:
            return np.nan
        e = float(np.mean((xi[m] - x_ref[m]) ** 2))
        best = min(best, (e, L))
    return best[1]


def main():
    ids = bagio.bag_ids()
    S = dict(dur=[], f_rate=[], r_rate=[], c_rate=[], f_gap=[], c_gap=[],
             vmin=[], vmax=[], nan=[], neg=[], lag=[], scale_f=[], scale_r=[],
             fr_rel=[], fr_big=[], notch=np.zeros(31), stamp_off=[])
    per_bag = []
    for b in ids:
        a = bagio.load(b)
        f, r, c = a["front"], a["rear"], a["cmd"]
        if len(f) < 20:
            continue
        dur = f[-1, 0] - f[0, 0]
        S["dur"].append(dur)
        S["f_rate"].append(len(f) / max(dur, 1e-9))
        S["r_rate"].append(len(r) / max(dur, 1e-9))
        S["c_rate"].append(len(c) / max(c[-1, 0] - c[0, 0], 1e-9) if len(c) > 1 else 0)
        S["f_gap"].append(gaps(f[:, 0], 0.5)[1])
        S["c_gap"].append(gaps(c[:, 0], 0.5)[1] if len(c) > 1 else 0)
        S["vmin"].append(min(f[:, 2].min(), r[:, 2].min()))
        S["vmax"].append(max(f[:, 2].max(), r[:, 2].max()))
        S["nan"].append(int(np.isnan(f[:, 2]).sum() + np.isnan(r[:, 2]).sum()))
        S["neg"].append(float(((f[:, 2] < -0.05).sum()) / len(f)))
        S["stamp_off"].append(float(np.median(f[:, 1] - f[:, 0])))
        pos = c[:, 2].astype(int)
        S["notch"] += np.bincount(np.clip(pos + 15, 0, 30), minlength=31)

        # перед и зад на общей сетке передней тележки
        rv = np.interp(f[:, 0], r[:, 0], r[:, 2])
        mv = (f[:, 2] > 2.0) & (rv > 2.0)
        if mv.sum() > 50:
            rel = (f[mv, 2] - rv[mv]) / (0.5 * (f[mv, 2] + rv[mv]))
            S["fr_rel"].append(float(np.median(rel)))
            S["fr_big"].append(float(np.mean(np.abs(rel) > 0.05)))

        # GNSS: скорость, задержка, масштаб
        if len(a["mvel"]) > 100:
            tg, vg = gnss_speed(a)
            lag = best_lag(tg, vg, f[:, 0], f[:, 2], np.arange(-1.0, 1.01, 0.05))
            S["lag"].append(lag)
            if np.isfinite(lag):
                fi = np.interp(tg, f[:, 0] - lag, f[:, 2])
                ri = np.interp(tg, r[:, 0] - lag, r[:, 2])
                # установившееся движение: скорость GNSS почти не меняется
                acc = np.gradient(vg, tg)
                m = (vg > 3.0) & (np.abs(acc) < 0.1)
                if m.sum() > 50:
                    S["scale_f"].append(float(np.median(fi[m] / vg[m])))
                    S["scale_r"].append(float(np.median(ri[m] / vg[m])))
                    per_bag.append((b, S["scale_f"][-1], S["scale_r"][-1], lag))

    q = lambda x: np.percentile(np.array(x, dtype=float)[np.isfinite(x)], [5, 50, 95]).round(3).tolist()
    print(f"прогонов с данными: {len(S['dur'])}")
    print(f"частота front/rear/cmd, Гц (5/50/95%): {q(S['f_rate'])} {q(S['r_rate'])} {q(S['c_rate'])}")
    print(f"макс. пропуск front, с: {q(S['f_gap'])}; cmd: {q(S['c_gap'])}")
    print(f"header.stamp - время bag (front), с: {q(S['stamp_off'])}")
    print(f"скорость тележек min/max по прогонам: {q(S['vmin'])} / {q(S['vmax'])}")
    print(f"NaN всего: {sum(S['nan'])}; доля отрицательных: {q(S['neg'])}")
    print(f"перед-зад, медиана относит. разности: {q(S['fr_rel'])}; доля |разн|>5%: {q(S['fr_big'])}")
    print(f"задержка тележек относительно GNSS, с: {q(S['lag'])}")
    print(f"масштаб front/GNSS: {q(S['scale_f'])}; rear/GNSS: {q(S['scale_r'])}")
    for veh in ("30618", "30639"):
        sf = [x[1] for x in per_bag if x[0].startswith(veh)]
        sr = [x[2] for x in per_bag if x[0].startswith(veh)]
        if sf:
            print(f"  {veh}: front {q(sf)} rear {q(sr)} (прогонов {len(sf)})")
    n = S["notch"]
    print("позиции ручки, доля времени, %:")
    print("  " + " ".join(f"{p:+d}:{100 * n[p + 15] / n.sum():.1f}"
                          for p in range(-15, 16) if n[p + 15] > 0))


if __name__ == "__main__":
    main()
