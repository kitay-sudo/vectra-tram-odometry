#!/usr/bin/env python3
"""tools/eval_gnss.py - коррекция положения по GNSS в середине маршрута:
сценарии доступности GNSS, «до» (GNSS только для выставки) и «после»
(gnss_correction), модель и база «только колесо».

Сценарии (tools/inject.py, GNSS_SCENARIOS; всё с зерном прогона):
  first3   - GNSS только первые 3 с (как в проверочных bag; должно совпасть с «до»);
  sparse   - первые 3 с + пачки по 5–10 с каждые 2–3 мин;
  bursts   - первые 3 с + пачки по 1–4 с, в среднем раз в минуту;
  nostart  - в начале GNSS нет, первая пачка через 1–3 мин (выставка посреди прогона);
  midstart - запись с середины маршрута (входы и эталон с 3–8 мин), GNSS первые 3 с;
  full     - GNSS весь прогон;
  glitchy  - sparse + сбои: скачки, метки ±1 с, пачка без RTK со сдвигом, без rover, мусор.
Эталон положения - base_link по ВСЕМ точкам GNSS прогона (как в tools/eval.py).

Плечи («arms»): before - gnss_correction false; after - true (параметры ноды
по умолчанию + лист + --set); naive_before / naive_after - база «только
колесо» на той же машинерии. --arm имя:k=v;k=v добавляет вариант модели
(подбор на train: --split train).

Запуск (из корня дерева, в образе с numpy):
  python3 tools/eval_gnss.py                          # holdout_scored, все сценарии
  python3 tools/eval_gnss.py --split train --runs-limit 24 --scenarios sparse,full,glitchy \\
      --arm "floor:gnss_min_interval_s=0.5" --no-naive
Итог: <out>/summary.json, <out>/runs.json, <out>/EVAL_GNSS.md.
"""

import argparse
import json
import math
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "analysis"))

import bagio                    # noqa: E402
import eval as E                # noqa: E402
import eval_metrics as M        # noqa: E402
import eval_pathgraph as PGM    # noqa: E402
import eval_replay as R         # noqa: E402
import inject as I              # noqa: E402

SCENARIOS = ("first3", "sparse", "bursts", "nostart", "midstart", "full", "glitchy")
# Сценарии с теми же первыми 3 с GNSS, что first3: при gnss_correction false
# точки после окна отбрасываются, поэтому выход плеч «до» у них тот же, что в
# first3 (проверено: tools/eval_gnss.py --no-share-before). Плечи «до»
# считаются один раз - в first3 - и копируются.
SAME_BEFORE = ("sparse", "bursts", "full", "glitchy")
KEYS = ("v_mae", "v_bias", "p3d_mean", "p3d_end_mean", "p3d_end_median", "p3d_end_max",
        "drift_pct_3d_median", "drift_pct_3d_mean", "drift_pct_3d_max",
        "along_mean", "along_max", "along_rmse", "cross_mean", "cov2s_along", "pz_mean",
        "pg_along_mean", "pg_cross_mean", "p_pair_frac", "p_gap_steps")
RU = dict(v_mae="скорость MAE, м/с", v_bias="скорость смещение, м/с", p3d_mean="3D ср., м",
          p3d_end_mean="конец 3D ср., м", p3d_end_median="конец 3D мед., м",
          p3d_end_max="конец 3D макс, м", drift_pct_3d_median="дрейф по концу, % мед.",
          drift_pct_3d_mean="дрейф по концу, % ср.", drift_pct_3d_max="дрейф по концу, % макс",
          along_mean="вдоль ср. |ош.|, м", along_max="вдоль макс, м", along_rmse="вдоль RMSE, м",
          cross_mean="поперёк ср., м", cov2s_along="|вдоль| ≤ 2σ_s", pz_mean="|z| ср., м",
          pg_along_mean="вдоль pathgraph ср., м", pg_cross_mean="поперёк pathgraph ср., м",
          p_pair_frac="доля пар положения", p_gap_steps="шагов без положения")
POS_ARMS = ("before", "after")


def _arms(args):
    """[(имя, параметры ноды поверх листа, база ли)]."""
    arms = [("before", {"gnss_correction": False}, False),
            ("after", {"gnss_correction": True}, False)]
    if not args.no_naive:
        arms += [("naive_before", {"gnss_correction": False}, True),
                 ("naive_after", {"gnss_correction": True}, True)]
    for spec in args.arm:
        name, ov = spec.split(":", 1)
        d = R.parse_overrides(ov.replace(";", ","))
        d.setdefault("gnss_correction", True)
        arms.append((name, d, False))
    return arms


def _cp(v):
    return v if np.isscalar(v) else np.copy(v)


class _Tape:
    """Лента шагов ядра ведущей связки прогона. GNSS ядро не трогает (сетку
    двигают только тележки и ручка), поэтому у всех плеч модели одного
    прогона ядро шагает одинаково: остальные плечи берут выходы ядра с ленты
    вместо пересчёта UKF (в 3–5 раз быстрее при нескольких плечах). После
    прогона сверяется, что ведомые прочли ленту до конца и сбросов не было;
    иначе задача считается заново без ленты (--no-share-core - всегда так)."""

    def __init__(self, core):
        self.rec = []
        self.core = core
        for name in ("step", "step_open_loop"):
            orig = getattr(core, name)
            setattr(core, name, self._wrap(orig))

    def _wrap(self, orig):
        def call(*a, **k):
            o = orig(*a, **k)
            c = self.core
            self.rec.append((dict(o), c.x.copy(), bool(np.isfinite(c.P).all()),
                             _cp(c.u_filt), c.mu))
            return o
        return call


class _Follower:
    """Ядро ведомого плеча: выходы с ленты ведущего (см. _Tape)."""

    def __init__(self, tape, real):
        self.tape, self.i = tape, 0
        self.p, self.nw = real.p, real.nw
        self.x, self.P = real.x.copy(), np.zeros(1)
        self.u_filt, self.mu = _cp(real.u_filt), real.mu

    def _next(self):
        o, x, ok, u, mu = self.tape.rec[self.i]
        self.i += 1
        self.x, self.P = x.copy(), np.zeros(1) if ok else np.full(1, np.nan)
        self.u_filt, self.mu = _cp(u), mu
        return dict(o)

    def step(self, *a, **k):
        return self._next()

    def step_open_loop(self, *a, **k):
        return self._next()


def run_one(task):
    """Один прогон одного сценария: все плечи в одном проходе событий."""
    res = _run_one(task, share=task.get("share_core", True))
    if res is None:                         # лента не сошлась (сброс связки): без неё
        res = _run_one(task, share=False)
        res["info"]["share_core"] = "fallback"
    return res


def _run_one(task, share):
    """Один прогон одного сценария: все плечи в одном проходе событий."""
    t_wall = time.perf_counter()
    cfg, bag, sc = task["cfg"], task["bag"], task["scenario"]
    a = bagio.load(bag)
    info = {}
    if sc == "midstart":
        a, cut = I.cut_start(a, I.seed_for(bag, "gnss|midstart"))
        info["cut_s"] = cut
    p = R.make_params(cfg["sheet"], cfg["overrides"], bag=bag)
    runners, names = [], []
    for name, ov, naive in task["arms"]:
        node = dict(cfg["sheet"]["node"], **ov)
        tm = E._map(cfg["map_path"])
        r = R.make_naive(p, node, tm) if naive else R.make_runner(p, node, tm)[0]
        runners.append(r)
        names.append(name)
    evs = R.events(a, sc, seed=I.seed_for(bag, "gnss|" + sc))
    info["n_fix"] = sum(1 for e in evs if e[1] == 2)
    model = [r for r, (_, _, naive) in zip(runners, task["arms"]) if not naive]
    tape, followers = None, []
    if share and len(model) > 1:
        tape = _Tape(model[0].core)
        for r in model[1:]:
            r.core = _Follower(tape, r.core)
            followers.append(r)
    outs = R.replay(evs, runners, cfg["sheet"]["node"])
    if tape is not None:
        if any(r.resets or r.core_resets for r in model) or any(
                not isinstance(r.core, _Follower) or r.core.i != len(tape.rec)
                for r in followers) or model[0].core is not tape.core:
            return None
        info["share_core"] = True
    res = dict(bag=bag, scenario=sc, vehicle=bag.split("_")[0], info=info, est={})
    t_first = E._t_first(a)
    for name, r, out, (_, ov, naive) in zip(names, runners, outs, task["arms"]):
        sub = dict(t_first=t_first, est={})
        E._score(sub, dict(full=True), cfg, p, a, [r], [out])
        e = sub["est"]["model"]
        row = e.get("row", {})
        pos = getattr(r, "pos", None)
        if pos is not None and not naive:
            row.update({k: getattr(pos, k, 0) for k in (
                "n_corr", "n_corr_big", "n_corr_reloc", "n_corr_gated", "n_corr_skew",
                "n_corr_geom", "n_corr_nonrtk", "n_realign")})
            row["mult"] = float(getattr(pos, "mult", 1.0))
        row["crash"] = e.get("crash")
        res["est"][name] = row
    res["wall_s"] = time.perf_counter() - t_wall
    return res


def totals(results, ids, arm):
    rows = [results[b]["est"][arm] for b in ids if b in results and arm in results[b]["est"]]
    t = M.totals(rows)
    if t is not None:
        for k in ("n_corr", "n_corr_big", "n_corr_reloc", "n_corr_gated", "n_corr_skew"):
            if any(k in r for r in rows):
                t[k] = int(sum(r.get(k, 0) for r in rows))
        t["crashes"] = int(sum(1 for r in rows if r.get("crash")))
    return t


def fmt(x, d=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "-"
    if isinstance(x, int):
        return str(x)
    s = f"{x:.{d}f}"
    return s.replace(".", ",").replace("-", "−")


DIG = dict(v_mae=4, v_bias=4, cov2s_along=3, drift_pct_3d_median=3, drift_pct_3d_mean=3,
           drift_pct_3d_max=3, pg_cross_mean=3, p_pair_frac=4, p3d_end_max=1, along_max=1)


def render(summary):
    """Markdown: таблица на сценарий и группу прогонов."""
    L = []
    A = L.append
    meta = summary["meta"]
    A("# Коррекция по GNSS: сценарии доступности (tools/eval_gnss.py)")
    A("")
    A(f"Прогоны: {meta['split']} ({len(meta['runs'])}), лист {meta['sheet']}, карта {meta['map']}. "
      f"Код пакета sha `{meta['pkg_src_sha']}`. «до» - `gnss_correction: false` (GNSS только "
      "для выставки), «после» - `true`. Эталон - base_link по всем точкам GNSS "
      "прогона, MGRS 37UCB.")
    A("")
    for sc in meta["scenarios"]:
        A(f"## {sc}: {I.GNSS_SCENARIOS.get(sc, sc)}")
        A("")
        for g, T in summary["totals"][sc].items():
            arms = [a for a in meta["arms"] if T.get(a)]
            if not arms:
                continue
            A(f"**{g}** ({T[arms[0]]['runs']} прогонов)")
            A("")
            A("| метрика | " + " | ".join(arms) + " |")
            A("|---|" + "---|" * len(arms))
            for k in KEYS:
                if not any(k in (T[a] or {}) for a in arms):
                    continue
                A(f"| {RU.get(k, k)} | " + " | ".join(fmt((T[a] or {}).get(k), DIG.get(k, 2))
                                                     for a in arms) + " |")
            ks = [k for k in ("n_corr", "n_corr_big", "n_corr_reloc", "n_corr_gated",
                              "n_corr_skew", "crashes") if any(k in (T[a] or {}) for a in arms)]
            for k in ks:
                A(f"| {k} | " + " | ".join(fmt((T[a] or {}).get(k)) for a in arms) + " |")
            A("")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--split", default="holdout_scored", help="holdout_scored | train")
    ap.add_argument("--runs", default="", help="прогоны через запятую (вместо --split)")
    ap.add_argument("--runs-limit", type=int, default=0, help="не больше стольких прогонов с GNSS")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--sheet", default="eval")
    ap.add_argument("--map", default="eval")
    ap.add_argument("--set", default="", help="поля листа / параметры ноды для всех плеч")
    ap.add_argument("--arm", action="append", default=[],
                    help="вариант модели имя:k=v;k=v (с gnss_correction true)")
    ap.add_argument("--no-naive", action="store_true")
    ap.add_argument("--pathgraph", default="auto")
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--no-share-core", action="store_true",
                    help="каждое плечо модели со своим ядром (проверка ленты _Tape)")
    ap.add_argument("--no-share-before", action="store_true",
                    help="считать плечи «до» в каждом сценарии (проверка, что они равны first3)")
    ap.add_argument("--out", default="out/eval_gnss")
    args = ap.parse_args()
    t_start = time.perf_counter()

    def log(msg):
        print(f"[eval_gnss {time.perf_counter() - t_start:6.0f} с] {msg}", flush=True)

    split = json.loads((ROOT / "tools" / "split.json").read_text(encoding="utf-8"))
    ids = args.runs.split(",") if args.runs else list(split[args.split])
    ids = [b for b in ids if len(bagio.load(b)["mfix"]) >= 100]      # только с GNSS
    if args.runs_limit and len(ids) > args.runs_limit:
        # детерминированно: каждый k-й по имени (доли вагонов сохраняются)
        ids = sorted(ids)[::max(1, len(ids) // args.runs_limit)][:args.runs_limit]
    sheet = R.resolve_sheet(args.sheet)
    ov = R.parse_overrides(args.set)
    core_ov, node_ov = R.split_overrides(ov)
    sheet["node"].update(node_ov)
    workers = args.workers or max(1, E.effective_cpus())
    _, train = E.split_ids()
    map_path, map_label, _ = E.resolve_map(args.map, bagio.CACHE, train, workers, False, log, sheet)
    pg_spec = PGM.default_spec() if args.pathgraph == "auto" else args.pathgraph
    cfg = dict(sheet=sheet, overrides=core_ov, map_path=str(map_path) if map_path else None,
               gnss="3", frame="mgrs", runner_frame="auto", runner_grid=None,
               judge_grid=M.JUDGE_GRID, quick_s=0.0, ref_point="base_link",
               pathgraph=pg_spec if PGM.load(pg_spec) is not None else "none", drop_antenna="")
    arms = _arms(args)
    scen = [s for s in args.scenarios.split(",") if s]
    share = not args.no_share_before and "first3" in scen
    before = [a for a in arms if not a[1].get("gnss_correction", True)]

    def arms_for(s):
        if share and s in SAME_BEFORE:
            return [a for a in arms if a not in before]
        return arms
    tasks = [dict(cfg=cfg, bag=b, scenario=s, arms=arms_for(s), share_core=not args.no_share_core)
             for s in scen for b in ids]
    log(f"прогонов {len(ids)}, сценариев {len(scen)}, плеч {len(arms)}; задач {len(tasks)}, "
        f"процессов {workers}")
    res = {}
    with ProcessPoolExecutor(workers) as ex:
        for t, r in zip(tasks, ex.map(run_one, tasks)):
            res.setdefault(t["scenario"], {})[t["bag"]] = r
    if share:
        for s in scen:
            if s in SAME_BEFORE:
                for b in ids:
                    for a in before:
                        res[s][b]["est"][a[0]] = res["first3"][b]["est"][a[0]]
    log("прогоны готовы")
    groups = {"all": ids, "30618": [b for b in ids if b.startswith("30618")],
              "30639": [b for b in ids if b.startswith("30639")]}
    names = [a[0] for a in arms]
    summary = dict(meta=dict(split=args.runs and "runs" or args.split, runs=ids, scenarios=scen,
                             arms=names, arm_params={a[0]: a[1] for a in arms},
                             sheet=sheet["path"], map=map_label, set=ov,
                             before_shared_from_first3=list(SAME_BEFORE) if share else [],
                             pkg_src_sha=E.src_digest(), git=E.git_head(),
                             scenario_ru={s: I.GNSS_SCENARIOS.get(s, s) for s in scen}),
                   totals={s: {g: {a: totals(res[s], bs, a) for a in names}
                               for g, bs in groups.items() if bs} for s in scen})
    runs = {s: {b: dict(info=r["info"], est=r["est"]) for b, r in res[s].items()} for s in scen}
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(E.dumps(summary), encoding="utf-8")
    (out / "runs.json").write_text(E.dumps(runs), encoding="utf-8")
    doc = render(json.loads(E.dumps(summary))).replace("–", "-")  # диапазоны через дефис
    (out / "EVAL_GNSS.md").write_text(doc, encoding="utf-8")
    for s in scen:
        T = summary["totals"][s]["all"]
        log(s + ": " + "; ".join(
            f"{a} 3D {fmt((T[a] or {}).get('p3d_mean'))} конец {fmt((T[a] or {}).get('p3d_end_mean'))} "
            f"вдоль {fmt((T[a] or {}).get('along_mean'))}" for a in names if T.get(a)))
    log(f"готово за {time.perf_counter() - t_start:.0f} с: {out}")


if __name__ == "__main__":
    main()
