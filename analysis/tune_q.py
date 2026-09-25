import sys, numpy as np, evaluate as E
from dataclasses import replace
from concurrent.futures import ProcessPoolExecutor
RUNS = ["30618_3e9f4952", "30618_6cb3280a", "30618_b95ca60a", "30639_d3c43d69", "30639_3b3d9eb8"]
def one(args):
    b, qv, sm = args
    p = replace(E.tram_params(), q_v=qv, sigma_meas=sm, dt=float(sys.argv[1]))
    a, outs = E.run_bag(b, params=p)
    T = np.array([o["stamp"] for o in outs]); V = np.array([o["v"] for o in outs])
    g = a["mvel"]; j, ok = E.nearest(T, g[:, 1]); vg = np.hypot(g[:, 2], g[:, 3])
    e = V[j[ok]] - vg[ok]
    e = e[np.abs(e) < 2.0]            # без выбросов самого эталона
    f, r = a["front"], a["rear"]
    tt = g[ok, 1]
    fz = f[np.clip(np.searchsorted(f[:, 1], tt, "right") - 1, 0, None), 2]
    rz = r[np.clip(np.searchsorted(r[:, 1], tt, "right") - 1, 0, None), 2]
    en = 0.5 * (fz + rz) / 3.6 * p.meas_scale - vg[ok]; en = en[np.abs(en) < 2.0]
    return b, qv, sm, float(np.mean(np.abs(e))), float(np.mean(np.abs(en)))
if __name__ == "__main__":
    grid = [(b, qv, sm) for b in RUNS for qv, sm in ((0.3, 0.05),)]
    with ProcessPoolExecutor() as ex:
        res = list(ex.map(one, grid))
    for qv, sm in ((0.3, 0.05),):
        rr = [x for x in res if x[1] == qv and x[2] == sm]
        print(f"q_v={qv} sigma_meas={sm}: фильтр {np.mean([x[3] for x in rr]):.4f}  причинная наивная {np.mean([x[4] for x in rr]):.4f}")
