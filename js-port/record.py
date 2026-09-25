import sys, json
sys.path.insert(0, '../prototype')
import numpy as np
exec(open('../prototype/scenarios.py').read().split('if __name__')[0])
out = {}
for name, kw in SCENARIOS:
    pl = Plant(kw['track'], dt=DT, seed=1); es = Estimator()
    if kw.get('faults'): pl.faults = kw['faults']
    sand = kw.get('sand', lambda t: False); tb = kw.get('tbrake', lambda t: False)
    rec = []
    for k in range(int(kw['T_end'] / DT)):
        t = k * DT
        notch = kw['driver'](t, pl.v)
        pl.step(notch, sand=sand(t), track_brake=tb(t))
        m = pl.measure()
        if k % SUB: continue
        o = es.step(notch, m)
        rec.append([notch, [float(x) for x in m], o['v'], o['s'], int(o['mode']), o['sigma_v'], o['k_t'], o['mu'], o['d'], pl.v])
    out[name] = rec
json.dump(out, open('rec.json', 'w'))
print('ok', {k: len(v) for k, v in out.items()})
