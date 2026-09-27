# 15 сценариев prototype/scenarios.py на имитаторе, ядро с Params по умолчанию -> rec.json
# для compare.js. Запуск из js-port/ (см. check.sh).
import sys, json, hashlib
sys.path.insert(0, '../prototype')
from scenarios import SCENARIOS, Plant, Estimator, DT, SUB  # noqa: E402  (шимы prototype -> ядро пакета)
import tram_state_estimator.estimator_core as EC  # noqa: E402  то ядро, которое сейчас сверяется
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
# отпечаток ядра (LF, 12 знаков) — compare.js сверяет его с TramEst.PORT.core_sha1
out['_core_sha1'] = hashlib.sha1(open(EC.__file__, 'rb').read().replace(bytes([13, 10]), bytes([10]))).hexdigest()[:12]
json.dump(out, open('rec.json', 'w'))
print('ok', {k: len(v) for k, v in out.items() if not k.startswith('_')}, 'ядро', out['_core_sha1'])
