# Разовая проверка контракта ноды. Запуск:
# docker run --rm -v E:/MY-PROJECT/TrackVector:/repo -w /repo vectra/tram:dev python3 /repo/tools/dev/adhoc_slow_callbacks.py
import sys, numpy as np
sys.path.insert(0, '/repo/tools/dev')
import fuzz_runner as F
for b in ("30618_0686195f", "30618_073f08d1"):
    for rep in range(2):
        ev, z = F.bag_events(b, 'window')
        rec = F.run_stream(F.make_runner(True), ev, budget_s=900)
        cm, tb, n, k = rec['call_ms'], rec['call_tb'], rec['call_nout'], rec['call_kind']
        idx = np.argsort(-cm)[:5]
        print(b, 'rep', rep, 'p99 %.2f max %.1f' % (np.percentile(cm, 99), cm.max()),
              [(round(float(cm[i]), 1), round(float(tb[i] - tb[0]), 2), str(k[i]), int(n[i])) for i in idx])
