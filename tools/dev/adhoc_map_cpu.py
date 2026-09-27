# Разовая проверка контракта ноды. Запуск:
# docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev python3 /repo/tools/dev/adhoc_map_cpu.py
import sys, time
sys.path.insert(0, '/repo/tools/dev')
import fuzz_runner as F
from tram_state_estimator import track_map as TM
cnt = {'reacq': 0, 'near': 0}
orig_re, orig_near = TM.TrackMap._reacquire, TM.TrackMap._near
def re(self, c): cnt['reacq'] += 1; return orig_re(self, c)
def nr(self, *a): cnt['near'] += 1; return orig_near(self, *a)
TM.TrackMap._reacquire, TM.TrackMap._near = re, nr
evs, _ = F.synth(7200.0, gnss='window')
r = F.make_runner(True)
t_next = evs[0][3] + 1200; i = 0
while i < len(evs):
    j = i
    while j < len(evs) and evs[j][3] < t_next: j += 1
    c0 = time.process_time(); st = 0; cnt.update(reacq=0, near=0)
    for tb, k, a, th, v in evs[i:j]:
        st += len(r.on_wheel(a, th, v) if k == 'w' else r.on_handle(th, v) if k == 'h' else r.on_fix(th, a, *v))
    cpu = time.process_time() - c0
    c = r.pos._cursor
    print(f"{(t_next-evs[0][3])/60:4.0f} min: {1e3*cpu/st:.3f} ms/step, _near calls/step {cnt['near']/st:.2f}, reacquire/step {cnt['reacq']/st:.2f}, on_map={c['on_map']}, off={c.get('off',0):.0f} m, snap_r={r.pos.map.snap_r}", flush=True)
    i = j; t_next += 1200
