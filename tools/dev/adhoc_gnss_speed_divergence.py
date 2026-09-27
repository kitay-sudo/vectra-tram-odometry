# Разовая проверка контракта ноды. Запуск:
# docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev python3 /repo/tools/dev/adhoc_gnss_speed_divergence.py
import sys, numpy as np
sys.path.insert(0, '/repo/tools/dev')
import fuzz_runner as F
b = '30639_3b3d9eb8'
z = np.load(F.CACHE / f'{b}.npz')
for k in ('front', 'rear', 'cmd', 'mfix', 'mvel'):
    a = z[k]; off = a[:, 1] - a[:, 0]
    d = np.diff(a[:, 1])
    print(k, 'n', len(a), 'off med %.3f min %.2f max %.2f' % (np.median(off), off.min(), off.max()), 'max gap th %.2f' % d.max(), 'at t=%.0f' % (a[np.argmax(d), 1] - a[0, 1]), 'back', int((d < 0).sum()))
recs = {}
for g in ('window', 'full'):
    ev, _ = F.bag_events(b, g)
    r = F.make_runner(False)
    recs[g] = F.run_stream(r, ev, budget_s=900)
    s = F.summarize(recs[g])
    print(g, 'n_out', s['n_out'], 'silence', round(s['max_silence_s'], 2), 'at', round(s.get('silence_at_s', 0), 1), 'burst', s['max_burst'])
w, f = recs['window'], recs['full']
c = F.compare(f, w, None)
print('common', c['common_stamps'], 'max dv', c['max_dv'])
# where do they differ
T = w['T']; j = np.searchsorted(f['T'], T); j = np.clip(j, 0, len(f['T']) - 1)
ok = np.abs(f['T'][j] - T) < 1e-6
dv = np.abs(w['V'][ok] - f['V'][j[ok]]); Tc = T[ok]
big = dv > 0.3
if big.any():
    print('dv>0.3 from t=%.1f to %.1f s, n=%d' % (Tc[big][0] - T[0], Tc[big][-1] - T[0], big.sum()))
g = z['mvel']; tg = g[:, 1]; vg = np.hypot(g[:, 2], g[:, 3])
for name, rec in recs.items():
    e = np.interp(tg, rec['T'], rec['V']) - vg
    m = (tg > rec['T'][0] + 5)
    # error by 2-minute blocks
    blocks = ((tg[m] - tg[m][0]) // 120).astype(int)
    print(name, 'MAE by 2-min block:', [round(float(np.mean(np.abs(e[m][blocks == k]))), 3) for k in range(blocks.max() + 1)])
