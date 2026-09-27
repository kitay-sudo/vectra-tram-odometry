"""Шаг 1б: квантование, проскальзывание относительно GNSS, качество GNSS."""

import numpy as np

import bagio

KMH = 3.6


def main():
    eq, steps, dev_all, ep = [], [], [], []
    st_cnt, cov, base = {}, [], []
    for b in bagio.bag_ids():
        a = bagio.load(b)
        f, r = a["front"], a["rear"]
        if len(f) < 50:
            continue
        rv = np.interp(f[:, 0], r[:, 0], r[:, 2])
        mv = f[:, 2] > 1.0
        if mv.sum() > 50:
            eq.append(float(np.mean(np.abs(f[mv, 2] - rv[mv]) < 1e-9)))
        u = np.unique(np.round(np.abs(np.diff(f[:, 2])), 6))
        u = u[u > 0]
        if len(u):
            steps.append(float(u.min()))

        if len(a["mvel"]) < 100:
            continue
        g = a["mvel"]
        tg, vg = g[:, 0], np.hypot(g[:, 2], g[:, 3])
        fi = np.interp(tg, f[:, 0], f[:, 2]) / KMH
        ri = np.interp(tg, r[:, 0], r[:, 2]) / KMH
        c = a["cmd"]
        ni = c[np.clip(np.searchsorted(c[:, 0], tg) - 1, 0, len(c) - 1), 2]
        wheel = 0.5 * (fi + ri)
        m = vg > 1.0
        dev = wheel - vg
        dev_all.append(dev[m])
        # эпизоды: |откл| > max(0.5 м/с, 5 %) дольше 0,5 с
        bad = m & (np.abs(dev) > np.maximum(0.5, 0.05 * vg))
        if bad.any():
            idx = np.flatnonzero(bad)
            splits = np.flatnonzero(np.diff(idx) > 2) + 1
            for grp in np.split(idx, splits):
                dur = tg[grp[-1]] - tg[grp[0]]
                if dur >= 0.5:
                    ep.append((b, tg[grp[0]] - tg[0], dur, float(np.mean(dev[grp])),
                               float(np.median(ni[grp])),
                               float(np.mean(fi[grp] - ri[grp])), float(vg[grp].mean())))
        for s in a["mfix"][:, 5].astype(int):
            st_cnt[s] = st_cnt.get(s, 0) + 1
        cov.append(np.median(np.sqrt(a["mfix"][:, 6])))
        # база master-rover (м) - если приёмники разнесены, даёт курс
        mf, rf = a["mfix"], a["rfix"]
        if len(rf) > 10:
            rl = np.interp(mf[:, 0], rf[:, 0], rf[:, 2])
            ro = np.interp(mf[:, 0], rf[:, 0], rf[:, 3])
            dn = (rl - mf[:, 2]) * 111_320.0
            de = (ro - mf[:, 3]) * 111_320.0 * np.cos(np.radians(mf[:, 2]))
            base.append(float(np.median(np.hypot(dn, de))))

    q = lambda x: np.percentile(np.asarray(x, float), [5, 50, 95]).round(4).tolist()
    print(f"доля точного равенства перед=зад на ходу: {q(eq)}")
    print(f"наименьший шаг изменения скорости (км/ч): {q(steps)}")
    d = np.concatenate(dev_all)
    print(f"колёса(м/с) - GNSS на ходу: процентили 1/5/50/95/99: "
          f"{np.percentile(d, [1, 5, 50, 95, 99]).round(3).tolist()}")
    print(f"эпизоды расхождения с GNSS (>max(0.5 м/с,5%), >=0.5 с): {len(ep)}, "
          f"суммарно {sum(e[2] for e in ep):.0f} с")
    if ep:
        pos = [e for e in ep if e[3] > 0]
        neg = [e for e in ep if e[3] < 0]
        print(f"  колёса быстрее GNSS: {len(pos)} (медиана позиции ручки "
              f"{np.median([e[4] for e in pos]) if pos else float('nan')}); "
              f"медленнее: {len(neg)} (медиана позиции "
              f"{np.median([e[4] for e in neg]) if neg else float('nan')})")
        print("  самые длинные:")
        for e in sorted(ep, key=lambda e: -e[2])[:8]:
            print(f"    {e[0]} t={e[1]:7.1f} dur={e[2]:5.1f}s dev={e[3]:+.2f} "
                  f"notch={e[4]:+.0f} front-rear={e[5]:+.2f} vGNSS={e[6]:.1f}")
    print(f"GNSS master status: {st_cnt}")
    print(f"GNSS σ положения (м), медиана по прогонам: {q(cov)}")
    print(f"расстояние master-rover (м): {q(base)}")


if __name__ == "__main__":
    main()
