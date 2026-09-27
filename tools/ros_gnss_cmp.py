"""ros_gnss_cmp - сравнение прогонов tools/ros_e2e.sh (raw.npz) одного bag против эталона base_link
по ВСЕМ точкам GNSS из кэша: скорость (против GNSS master), положение (3D,
вдоль), и скорость одного прогона против другого по меткам.

    python3 tools/ros_gnss_cmp.py <bag> <dirA> [<dirB> ...]
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "analysis"))
import bagio                    # noqa: E402
import eval_geo as G            # noqa: E402
import eval_metrics as M        # noqa: E402

bag = sys.argv[1]
dirs = sys.argv[2:]
a = bagio.load(bag)
ref = M.reference(a, "base_link")
Er, Nr = (np.asarray(v, float) for v in G.utm_fwd(ref["lat"], ref["lon"], 37))
Zr = np.asarray(ref["alt"], float)
hd = np.unwrap(np.arctan2(np.gradient(Er), np.gradient(Nr)))
mv = np.asarray(a["mvel"], float)
tv, gv = mv[:, 1], np.hypot(mv[:, 2], mv[:, 3])
res = {}
V = {}
for d in dirs:
    z = np.load(d + "/raw.npz")
    vel, odo = z["vel"], z["odo"]
    V[d] = dict(zip(np.round(vel[:, 1], 3), vel[:, 2]))
    j, ok = M.nearest(vel[:, 1], tv)
    ev = vel[j[ok], 2] - gv[ok]
    To = odo[:, 1]
    j2, ok2 = M.nearest(To, ref["t"])
    idx = np.flatnonzero(ok2)
    o = odo[j2[ok2]]
    dx, dy, dz = o[:, 2] + 300000.0 - Er[idx], o[:, 3] + 6100000.0 - Nr[idx], o[:, 4] - Zr[idx]
    d3 = np.sqrt(dx * dx + dy * dy + dz * dz)
    al = dx * np.sin(hd[idx]) + dy * np.cos(hd[idx])
    res[d] = dict(n_vel=int(len(vel)), n_pos=int(len(odo)),
                  v_mae=float(np.mean(np.abs(ev))), v_bias=float(np.mean(ev)),
                  p3d_mean=float(np.mean(d3)), p3d_p95=float(np.percentile(d3, 95)),
                  p3d_max=float(np.max(d3)), p3d_end=float(d3[-1]),
                  along_mean=float(np.mean(np.abs(al))), along_max=float(np.max(np.abs(al))),
                  pz_mean=float(np.mean(np.abs(dz))),
                  pairs=int(len(idx)), ref=int(len(ref["t"])))
if len(dirs) > 1:
    A, B = V[dirs[0]], V[dirs[1]]
    common = sorted(set(A) & set(B))
    dv = [abs(A[t] - B[t]) for t in common]
    res["speed_vs"] = dict(common_stamps=len(common), only_a=len(set(A) - set(B)),
                           only_b=len(set(B) - set(A)), max_dv=float(max(dv)) if dv else None,
                           n_diff_gt_1e9=int(sum(1 for x in dv if x > 1e-9)))
print(json.dumps(res, indent=1, ensure_ascii=False))
