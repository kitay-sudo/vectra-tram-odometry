#!/usr/bin/env python3
"""tools/eval.py — единая оценка точности и устойчивости (WP7). Одна команда.

Что делает (всё офлайн, без ROS, в образе vectra/tram:dev):
  1. строит кэш numpy из data/ для нужных прогонов, если его нет
     (analysis/bagio.py; .msg — из ros2_ws/src/tram_vehicle_msgs/msg);
  2. строит карту путей только по обучающим прогонам (analysis/build_map.py
     train), если её нет;
  3. прогоняет отложенные прогоны tools/split.json:holdout_scored через связку
     Runner так же, как нода (tools/eval_replay.py): GNSS только первые 3 с
     (--gnss full — весь прогон, как bag жюри с полным GNSS);
  4. считает метрики ТЗ против GNSS: скорость (master — основной эталон,
     rover — дополнительный) RMSE/MAE/смещение, по фазам, ±2σ, ложные стоянки;
     положение в системе судьи MGRS (x — восток, y — север, z — высота):
     3D, вдоль/поперёк пути, дрейф % по концу прогона; квадраты 100 км;
  5. рядом — причинная база «только колесо» на той же машинерии карты;
  6. инъекции аномалий (tools/inject.py) в реальные отложенные прогоны:
     до / во время / после, восстановление, флаги;
  7. графики docs/img/eval_*.png;
  8. out/eval/*.json и docs/EVAL.md (tools/eval_report.py).

Запуск (PowerShell, из корня worktree):
  docker run --rm --cpus 4 -v ${PWD}:/repo -v E:/MY-PROJECT/TrackVector/data:/repo/data:ro `
      -w /repo vectra/tram:dev python3 tools/eval.py
  ... python3 tools/eval.py --quick --check-determinism     # CI: 2 коротких прогона, дважды
Ключи — python3 tools/eval.py --help и docs/EVAL.md, раздел «Как запустить».
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "analysis"))

import bagio                    # noqa: E402
import eval_geo as G            # noqa: E402
import eval_metrics as M        # noqa: E402
import eval_replay as R         # noqa: E402
import inject as I              # noqa: E402

SPLIT = ROOT / "tools" / "split.json"
MAPS_DIR = ROOT / "out" / "maps"         # карта оценки, если каталог кэша только для чтения
INJECT_RUNS = ("30618_3e9f4952", "30639_d3c43d69", "30618_e9a34502")
QUICK_RUNS = ("30618_e9a34502", "30639_d3c43d69")
QUICK_S = 300.0
QUICK_KINDS = ("both_zero", "dropout", "nan", "stamp_jump")
VARIANTS = ("stubs:c_creep=0.02,c_creep_drag=0.002", "zero:c_creep=0,c_creep_drag=0")
SENS_FRAMES = ("mgrs", "enu", "equirect")
BEFORE_S, AFTER_S = 30.0, 60.0
GNSS_FULL_S = 600.0                     # с записи: прогон «GNSS весь прогон» — первые 10 мин
TAIL_S = 300.0                          # с: прогон с инъекцией идёт до конца окна + TAIL_S
REC_TOL, REC_HOLD = 0.1, 3.0            # м/с, с: восстановление = |v − v_чисто| ≤ tol в течение hold


# ------------------------------------------------------------------ подготовка

def effective_cpus():
    try:
        q, p = open("/sys/fs/cgroup/cpu.max").read().split()[:2]
        if q != "max":
            return max(1, int(math.floor(int(q) / int(p))))
    except (OSError, ValueError):
        pass
    try:
        return len(os.sched_getaffinity(0))
    except AttributeError:
        return os.cpu_count() or 1


def split_ids():
    d = json.loads(SPLIT.read_text(encoding="utf-8"))
    return d["holdout_scored"], d["train"]


def ensure_cache(ids, workers, log):
    missing = [b for b in ids if not (bagio.CACHE / f"{b}.npz").exists()]
    if not missing:
        return []
    if not bagio.DATA.exists():
        sys.exit(f"нет кэша для {len(missing)} прогонов и нет данных {bagio.DATA}")
    log(f"кэш: строю {len(missing)} прогонов из {bagio.DATA} в {bagio.CACHE}")
    _, bad = bagio.build_cache(workers=workers, ids=missing)
    if bad:
        sys.exit(f"не прочитались: {bad}")
    return missing


def build_train_map(out, train_ids, workers, log):
    """Карта оценки: analysis/build_map.py train (список train — из
    analysis/drive_model.json, он совпадает с tools/split.json:train)."""
    dm_path = ROOT / "analysis" / "drive_model.json"
    dm = json.loads(dm_path.read_text(encoding="utf-8"))
    if sorted(dm["train"]) != sorted(train_ids):
        sys.exit("build_map.py берёт train из analysis/drive_model.json, а он не совпадает "
                 "с tools/split.json:train — карта оценки была бы не той")
    ensure_cache(train_ids, workers, log)
    if not (bagio.CACHE.parent / "drive_model.json").exists():   # build_map ищет его рядом с кэшем
        shutil.copy(dm_path, bagio.CACHE.parent / "drive_model.json")
    import build_map
    argv = sys.argv
    sys.argv = ["build_map.py", "train", str(out)]
    try:
        log(f"карта: строю по {len(train_ids)} обучающим прогонам -> {out}")
        build_map.main()
    finally:
        sys.argv = argv


MAP_SRC = ("analysis/build_map.py", "analysis/drive_model.json",
           "ros2_ws/src/tram_state_estimator/tram_state_estimator/track_map.py",
           "ros2_ws/src/tram_state_estimator/tram_state_estimator/runner.py",
           "ros2_ws/src/tram_state_estimator/config/tram_calibration.json")


def map_key():
    """Ключ карты оценки: хэш кода, от которого она зависит. После правок
    build_map.py / track_map.py / runner.py карта пересобирается сама."""
    h = hashlib.sha256()
    for rel in MAP_SRC:
        f = ROOT / rel
        h.update(rel.encode())
        h.update(f.read_bytes() if f.exists() else b"-")
    return h.hexdigest()[:8]


def _writable(d):
    try:
        d.mkdir(parents=True, exist_ok=True)
        t = d / ".eval_write_test"
        t.write_bytes(b"")
        t.unlink()
        return True
    except OSError:
        return False


def resolve_map(spec, cache, train_ids, workers, rebuild, log, out):
    if spec == "none":
        return None, "без карты (запасной режим map_file: \"\")", None
    if spec == "train":
        key = map_key()
        name = f"track_map_train.{key}.npz"
        # карта напарника analysis/cache/track_map_train.npz не используется: она собрана
        # на другой платформе и отличается множителем пути в 16-м знаке (числа — в 6-м)
        cands = [cache / name, MAPS_DIR / name]
        f = next((c for c in cands if c.exists()), None)
        if rebuild or f is None:
            f = (cache if _writable(cache) else MAPS_DIR) / name
            f.parent.mkdir(parents=True, exist_ok=True)
            build_train_map(f, train_ids, workers, log)
        return f, f"оценочная: только обучающие прогоны (build_map.py train, ключ кода {key})", None
    if spec == "jury":
        f = R.CFG / "track_map.npz"
        return f, "боевая config/track_map.npz (все данные)",             "УТЕЧКА: боевая карта собрана по всем прогонам, включая отложенные"
    f = Path(spec)
    return f, str(spec), None


# ------------------------------------------------------------------ рабочий процесс

def _map(path):
    """Новая карта на каждую связку: TrackMap хранит привязку к прогону."""
    if path is None:
        return None
    from tram_state_estimator.track_map import TrackMap
    return TrackMap.load(path)


def _load_run(bag, quick_s):
    a = bagio.load(bag)
    if quick_s:
        a = R.truncate(a, quick_s)
    return a


def _geo(O, r, a, cfg, zone):
    """Выход Runner'а -> (lat, lon, alt) по его системе координат."""
    fr, grid, note = R.detect_frame(cfg["sheet"]["node"], cfg["runner_frame"], O["XYZ"])
    if cfg.get("runner_grid") is not None:
        grid = cfg["runner_grid"]
    origin = R.runner_origin(r) if r is not None else None
    if origin is None:
        origin = R.first_master(a)
        if fr in ("equirect", "enu"):
            note.append("начало Runner'а не найдено, беру первую точку master")
    return R.to_geo(O["XYZ"], fr, grid, origin, zone), dict(frame=fr, grid=grid, note=note)


def _series(O, keys=("T", "V", "SV", "VALID", "SLIP", "AMB", "MODE", "STALE")):
    return {k: O[k] for k in keys}


def run_task(task):
    """Один прогон (чистый, вариант листа, GNSS весь прогон или инъекция)."""
    t_wall = time.perf_counter()
    cfg = task["cfg"]
    bag = task["bag"]
    a = _load_run(bag, cfg["quick_s"])
    if task.get("limit_s"):
        a = R.truncate(a, task["limit_s"])
    info = None
    if task.get("inject"):
        kind = task["inject"]
        t0, dur = I.choose_window(a, kind)
        if t0 is None:
            return dict(task=task["id"], skipped=f"нет окна для {kind}")
        p0 = R.make_params(cfg["sheet"], cfg["overrides"])
        a, info = I.apply(a, kind, t0, dur, seed=I.seed_for(bag, kind), sigma_meas=p0.sigma_meas)
        # дальше конца окна + TAIL_S не считаем: остаток прогона только стоит времени
        a = R.cut_stamp(a, t0 + I.eval_window(kind) + TAIL_S)
    p = R.make_params(cfg["sheet"], task.get("overrides", cfg["overrides"]))
    node = cfg["sheet"]["node"]
    mp = cfg["map_path"]
    model, unused = R.make_runner(p, node, _map(mp))
    runners = [model]
    if task.get("naive", True):
        runners.append(R.make_naive(p, node, _map(mp)))
    evs = R.events(a, task.get("gnss", cfg["gnss"]))
    outs = R.replay(evs, runners)
    wall = time.perf_counter() - t_wall
    origin = R.first_master(a)
    t_first = min(float(a[k][0, 1]) for k in ("front", "rear", "cmd") if len(a[k]))
    res = dict(task=task["id"], bag=bag, vehicle=bag.split("_")[0], inject=info, t_first=t_first,
               wall_s=wall, n_events=len(evs), node_params_unused=unused, est={})
    for name, r, (O, crash) in zip(("model", "naive"), runners, outs):
        if crash is not None:
            crash = dict(crash, after_start_s=crash["stamp"] - t_first)
        e = dict(crash=crash, n_out=int(len(O["T"])))
        if origin is None or len(O["T"]) < 2:
            e["row"] = dict(n_out=int(len(O["T"])))
            res["est"][name] = e
            continue
        frame = M.Frame(cfg["frame"], origin)
        O["GEO"], e["runner_frame"] = _geo(O, r, a, cfg, frame.zone)
        naive = name == "naive"
        if naive:
            O["has_mode"] = False         # ложная стоянка базы: v < v_standstill (как core_metrics)
        row, s_v = M.score_speed(O, a, "master", p.v_standstill, p.meas_scale)
        row_r, _ = M.score_speed(O, a, "rover", p.v_standstill, p.meas_scale)
        row.update({f"rover_{k}": v for k, v in row_r.items()
                    if k in ("v_pairs", "v_rmse", "v_mae", "v_bias", "v_max")})
        rowp, s_p = M.score_position(O, a, frame, cfg["judge_grid"], full=task.get("full", True))
        row.update(rowp)
        row["n_out"] = int(len(O["T"]))
        row["rate_hz"] = float(len(O["T"]) / max(O["T"][-1] - O["T"][0], 1e-9))
        if not naive:
            row["frac_valid"] = float(O["VALID"].mean())
            row["frac_slip"] = float(O["SLIP"].mean())
            row["frac_amb"] = float(O["AMB"].mean())
            row["anchors"] = int(getattr(getattr(r, "pos", None), "anchors", 0))
        if task.get("sens"):
            sens = {}
            for fr in SENS_FRAMES:
                if fr == cfg["frame"]:
                    continue
                rp, _ = M.score_position(O, a, M.Frame(fr, origin), full=False)
                sens[fr] = {k: rp.get(k) for k in ("p3d_mean", "p3d_max", "p3d_end")}
            e["sens"] = sens
        e["row"] = row
        if task.get("keep_samples"):
            e["samples"] = dict(v=s_v, p=s_p)
        if task.get("keep_series"):
            e["xyz"] = O["XYZ"]
            e["series"] = _series(O)
            e["series"]["tp"] = s_p["tp"] if s_p else np.zeros(0)
            e["series"]["al"] = s_p.get("al", np.zeros(0)) if s_p else np.zeros(0)
            e["series"]["d3"] = s_p["d3"] if s_p else np.zeros(0)
            if s_v is not None:
                e["series"]["tg"], e["series"]["ev"] = s_v["tg"], s_v["ev"]
                e["series"]["vg"] = s_v["vg"]
        res["est"][name] = e
    return res


# ------------------------------------------------------------------ инъекции: до / во время / после

def _win(t, lo, hi):
    return (t >= lo) & (t < hi)


def _recovery(Si, Sc, t1):
    """Сколько секунд после конца аномалии до момента, с которого выход
    совпадает с чистым прогоном (|Δv| ≤ REC_TOL) не меньше REC_HOLD с."""
    Ti, Vi = Si["T"], Si["V"]
    if not len(Ti) or not len(Sc["T"]):
        return None
    j, ok = M.nearest(Sc["T"], Ti, tol=0.026)
    good = ok & (np.abs(Vi - Sc["V"][j]) <= REC_TOL)
    after = Ti >= t1
    idx = np.flatnonzero(after)
    if not len(idx):
        return None
    start = None
    for i in idx:
        if good[i]:
            if start is None:
                start = i
            if Ti[i] - Ti[start] >= REC_HOLD:
                return max(0.0, float(Ti[start] - t1))
        else:
            start = None
    return None


def inject_metrics(res_inj, res_clean):
    info = res_inj["inject"]
    t0 = info["t0"]
    t1 = t0 + info["dur"]
    te = t0 + info["eval_s"]
    wins = dict(before=(t0 - BEFORE_S, t0), during=(t0, te), after=(te, te + AFTER_S))
    out = dict(kind=info["kind"], bag=res_inj["bag"], t0_rel_s=t0 - res_clean["t_first"],
               dur=info["dur"], eval_s=info["eval_s"], touched=info["touched"], est={})
    for name in ("model", "naive"):
        ei, ec = res_inj["est"].get(name), res_clean["est"].get(name)
        if ei is None or ec is None or "series" not in ei:
            continue
        Si, Sc = ei["series"], ec["series"]
        d = dict(crash=None)
        if ei["crash"] is not None:
            d["crash"] = dict(error=ei["crash"]["error"], after_t0_s=ei["crash"]["stamp"] - t0)
        for w, (lo, hi) in wins.items():
            m = _win(Si["tg"], lo, hi)
            vs = M.vstats(Si["ev"][m]) if "ev" in Si else None
            mp = _win(Si["tp"], lo, hi)
            al = Si["al"][mp] if len(Si["al"]) else np.zeros(0)
            mc = _win(Sc["tp"], lo, hi)
            alc = Sc["al"][mc] if len(Sc["al"]) else np.zeros(0)
            nout = int(_win(Si["T"], lo, hi).sum())
            d[w] = dict(v_mae=vs["mae"] if vs else None, v_bias=vs["bias"] if vs else None,
                        v_max=vs["max"] if vs else None, v_pairs=vs["n"] if vs else 0,
                        along_end=M._last(al), along_mean=M._nanstat(lambda x: np.mean(np.abs(x)), al),
                        along_end_clean=M._last(alc), n_out=nout,
                        n_out_clean=int(_win(Sc["T"], lo, hi).sum()))
            if name == "model" and w == "during":
                mt = _win(Si["T"], lo, hi)
                if mt.any():
                    d[w].update(frac_valid=float(Si["VALID"][mt].mean()),
                                frac_slip=float(Si["SLIP"][mt].mean()),
                                frac_amb=float(Si["AMB"][mt].mean()),
                                sigma_v_max=float(np.nanmax(Si["SV"][mt])))
                    Tm = Si["T"][mt]
                    d[w]["max_gap_s"] = float(np.max(np.diff(Tm))) if len(Tm) > 1 else None
                if "ev" in Si and m.any():
                    sv_at = Si["SV"][M.nearest(Si["T"], Si["tg"][m])[0]]
                    d[w]["cov2s"] = float(np.mean(np.abs(Si["ev"][m]) <= 2 * sv_at))
        d["recovery_s"] = _recovery(Si, Sc, t1)
        # остаточная ошибка в конце хвоста (TAIL_S после окна) против чистого прогона
        d["d_along_tail"] = d["d_3d_tail"] = None
        if ei["crash"] is None and len(Si["tp"]) and len(Sc["tp"]):
            jc, okc = M.nearest(Sc["tp"], Si["tp"][-1:], tol=0.051)
            if okc[0]:
                d["d_3d_tail"] = float(Si["d3"][-1] - Sc["d3"][jc[0]])
            fa = np.flatnonzero(np.isfinite(Si["al"]))
            if len(fa):
                k = int(fa[-1])
                jk, okk = M.nearest(Sc["tp"], Si["tp"][k:k + 1], tol=0.051)
                if okk[0]:
                    d["d_along_tail"] = float(Si["al"][k] - Sc["al"][jk[0]])
        d["tail_s"] = TAIL_S
        d["n_out"] = ei["n_out"]
        d["nan_out"] = int(np.sum(~np.isfinite(Si["V"])))
        out["est"][name] = d
    return out


# ------------------------------------------------------------------ GNSS весь прогон против 3 с

def gnss_full_compare(res, base, ids):
    """Выход с GNSS весь прогон против выхода с GNSS 3 с на общем отрезке
    (первые GNSS_FULL_S с записи): README разрешает GNSS только для выставки,
    значит выходы должны совпасть. Плюс метрики обоих на этом отрезке."""
    per, rows_f, rows_b = {}, [], []
    for b in ids:
        ef = res[("gnss_full", b)]["est"]["model"]
        eb = base[b]["est"]["model"]
        Sf, Sb = ef.get("series"), eb.get("series")
        if not Sf or not Sb or not len(Sf["T"]):
            continue
        t_hi = float(Sf["T"][-1])
        mb = Sb["T"] <= t_hi
        j, ok = M.nearest(Sf["T"], Sb["T"][mb], tol=1e-6)
        same_grid = bool(ok.all()) and int(mb.sum()) == len(Sf["T"])
        dv = np.abs(Sf["V"][j[ok]] - Sb["V"][mb][ok])
        XYZf = ef["xyz"]
        XYZb = eb["xyz"][mb][ok]
        dp = np.linalg.norm(XYZf[j[ok]] - XYZb, axis=1)
        # метрики на общем отрезке: пары скорости и положения по меткам эталона
        mg = Sb["tg"] <= t_hi
        mpp = Sb["tp"] <= t_hi
        rb = dict(v_pairs=int(mg.sum()), v_mae=float(np.mean(np.abs(Sb["ev"][mg]))) if mg.any() else None,
                  p3d_mean=float(np.nanmean(Sb["d3"][mpp])) if mpp.any() else None)
        rf = dict(v_pairs=ef["row"].get("v_pairs", 0), v_mae=ef["row"].get("v_mae"),
                  p3d_mean=ef["row"].get("p3d_mean"), p3d_end=ef["row"].get("p3d_end"))
        rows_b.append(rb)
        rows_f.append(rf)
        per[b] = dict(span_s=t_hi - float(Sf["T"][0]), same_grid=same_grid,
                      n_out_full=int(len(Sf["T"])), n_out_3s=int(mb.sum()),
                      max_dv=float(dv.max()) if len(dv) else None,
                      max_dpos=float(np.nanmax(dp)) if len(dp) else None,
                      v_mae_full=rf["v_mae"], v_mae_3s=rb["v_mae"],
                      p3d_mean_full=rf["p3d_mean"], p3d_mean_3s=rb["p3d_mean"])
    return dict(span_s=GNSS_FULL_S, runs=per, full=M.totals(rows_f), gnss3=M.totals(rows_b),
                identical_runs=int(sum(1 for r in per.values()
                                       if r["same_grid"] and (r["max_dv"] or 0) <= 1e-9
                                       and (r["max_dpos"] or 0) <= 1e-6)),
                max_dv=max((r["max_dv"] or 0 for r in per.values()), default=None),
                max_dpos=max((r["max_dpos"] or 0 for r in per.values()), default=None))


# ------------------------------------------------------------------ сборка результата

def _clean(x):
    """JSON без NaN, числа до 6 значащих цифр: одинаковые байты на двух прогонах."""
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (float, np.floating)):
        x = float(x)
        return float(f"{x:.6g}") if math.isfinite(x) else None
    if isinstance(x, np.ndarray):
        return _clean(x.tolist())
    return x


def dumps(obj):
    return json.dumps(_clean(obj), ensure_ascii=False, indent=1, allow_nan=False) + "\n"


def group_totals(results, ids, est):
    rows = [results[b]["est"][est]["row"] for b in ids
            if b in results and est in results[b]["est"] and "row" in results[b]["est"][est]]
    return M.totals(rows)


def pooled(results, ids, est):
    sv = [results[b]["est"][est]["samples"]["v"] for b in ids
          if b in results and "samples" in results[b]["est"].get(est, {})]
    sp = [results[b]["est"][est]["samples"]["p"] for b in ids
          if b in results and "samples" in results[b]["est"].get(est, {})]
    return dict(by_phase=M.by_phase(sv), ref_clean=M.clean_ref(sv), position=M.pooled_position(sp))


def src_digest():
    h = hashlib.sha256()
    for f in sorted((R.PKG / "tram_state_estimator").glob("*.py")):
        h.update(f.name.encode())
        h.update(f.read_bytes())
    return h.hexdigest()[:16]


def git_head():
    try:
        import subprocess
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=10)
        dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--",
                                "ros2_ws", "tools", "analysis"],
                               capture_output=True, text=True, timeout=10)
        h = out.stdout.strip()
        if h:
            return h + ("+dirty" if dirty.stdout.strip() else "")
    except Exception:            # noqa: BLE001
        pass
    return os.environ.get("TRAM_GIT_REV")     # в контейнере .git worktree не виден


# ------------------------------------------------------------------ главный проход

def evaluate(args, log):
    holdout, train = split_ids()
    cache = bagio.CACHE
    ids = list(QUICK_RUNS) if args.quick else (args.runs.split(",") if args.runs else list(holdout))
    workers = args.workers or max(1, effective_cpus())
    ensure_cache(ids, workers, log)
    map_path, map_label, map_leak = resolve_map(args.map, cache, train, workers, args.rebuild_map, log,
                                             ROOT / args.out)
    sheet = R.resolve_sheet(args.sheet)
    overrides = R.parse_overrides(args.set)
    cfg = dict(sheet=sheet, overrides=overrides, map_path=str(map_path) if map_path else None,
               gnss=args.gnss, frame=args.frame, runner_frame=args.runner_frame,
               runner_grid=args.runner_grid, judge_grid=args.judge_grid,
               quick_s=QUICK_S if args.quick else 0.0)
    base_p = R.make_params(sheet, overrides)
    tasks = []
    for b in ids:
        tasks.append(dict(id=("base", b), bag=b, cfg=cfg, naive=True, keep_samples=True,
                          keep_series=True, sens=True))
    # варианты листа: заглушки крипа против подогнанных / нулевых
    variants = []
    if not args.no_variants and not args.quick:
        for spec in VARIANTS:
            name, ov = spec.split(":", 1)
            ovd = dict(overrides)
            ovd.update(R.parse_overrides(ov))
            if R.make_params(sheet, ovd) == base_p:
                variants.append(dict(name=name, overrides=ov, same_as_base=True))
                continue
            variants.append(dict(name=name, overrides=ov, same_as_base=False))
            for b in ids:
                tasks.append(dict(id=("var", name, b), bag=b, cfg=cfg, overrides=ovd, naive=False,
                                  full=False))
    # GNSS весь прогон (bag жюри с полным GNSS)
    if args.gnss != "full" and not args.no_gnss_full and not args.quick:
        for b in ids:
            tasks.append(dict(id=("gnss_full", b), bag=b, cfg=cfg, gnss="full", naive=False,
                              full=False, keep_series=True, limit_s=GNSS_FULL_S))
    # инъекции
    inj_runs = [] if args.no_inject else (
        [ids[0]] if args.quick else [b for b in args.inject_runs.split(",") if b])
    kinds = list(QUICK_KINDS) if args.quick else (
        args.inject_kinds.split(",") if args.inject_kinds else list(I.ORDER))
    for b in inj_runs:
        if b not in ids:
            ensure_cache([b], workers, log)
            tasks.append(dict(id=("base", b), bag=b, cfg=cfg, naive=True, keep_series=True))
        for k in kinds:
            tasks.append(dict(id=("inj", k, b), bag=b, cfg=cfg, inject=k, naive=True,
                              keep_series=True))
    # длинные — первыми, чтобы процессы кончили вместе
    order = sorted(range(len(tasks)), key=lambda i: (tasks[i]["id"][0] != "inj", i))
    log(f"задач {len(tasks)}: прогонов {len(ids)}, вариантов {len([v for v in variants if not v['same_as_base']])}, "
        f"инъекций {len(inj_runs) * len(kinds)}; процессов {workers}")
    t0 = time.perf_counter()
    res = {}
    with ProcessPoolExecutor(workers) as ex:
        for i, r in zip(order, ex.map(run_task, [tasks[i] for i in order])):
            res[tasks[i]["id"]] = r
    wall = time.perf_counter() - t0
    log(f"прогоны готовы за {wall:.0f} с")

    base = {k[1]: v for k, v in res.items() if k[0] == "base"}
    report = dict(meta=dict(
        label=args.label, sheet={k: sheet[k] for k in ("label", "path", "kind", "sha", "leak")},
        overrides=overrides, map=dict(label=map_label, file=(Path(map_path).relative_to(ROOT).as_posix()
                                                             if map_path and Path(map_path).is_relative_to(ROOT)
                                                             else (str(map_path) if map_path else None)),
                                      sha=R.sha(map_path) if map_path else None, leak=map_leak),
        gnss=args.gnss, frame=args.frame, frame_ru=M.FRAMES_RU[args.frame],
        runner_frame=args.runner_frame, judge_grid=args.judge_grid, quick=args.quick,
        runs=ids, kinds=kinds, split=SPLIT.relative_to(ROOT).as_posix(), pkg_src_sha=src_digest(),
        tol_s=M.TOL, v_stand_gnss=M.V_STAND_GNSS, v_false_ss=M.V_FALSE_SS,
        params=dict(dt=base_p.dt, q_v=base_p.q_v, c_creep=base_p.c_creep,
                    c_creep_drag=base_p.c_creep_drag, meas_scale=base_p.meas_scale,
                    delay_drive=base_p.delay_drive, tau_drive=base_p.tau_drive),
        node_params=sheet["node"],
        node_params_unused=sorted({u for r in base.values() for u in r.get("node_params_unused", [])}),
        runner_frames=sorted({json.dumps(r["est"]["model"].get("runner_frame"), ensure_ascii=False,
                                         sort_keys=True) for r in base.values()
                              if "model" in r["est"]}),
    ))
    groups = {"all": ids, "30618": [b for b in ids if b.startswith("30618")],
              "30639": [b for b in ids if b.startswith("30639")]}
    report["totals"] = {g: {e: group_totals(base, bs, e) for e in ("model", "naive")}
                        for g, bs in groups.items() if bs}
    report["pooled"] = {e: pooled(base, ids, e) for e in ("model", "naive")}
    report["crashes"] = {b: {e: base[b]["est"][e]["crash"] for e in base[b]["est"]
                             if base[b]["est"][e]["crash"]} for b in ids
                         if any(base[b]["est"][e]["crash"] for e in base[b]["est"])}
    sens = {}
    for fr in SENS_FRAMES:
        if fr == args.frame:
            continue
        rows = []
        for b in ids:
            s = base[b]["est"]["model"].get("sens", {}).get(fr)
            if s:
                rows.append(dict(s, v_pairs=base[b]["est"]["model"]["row"]["v_pairs"]))
        sens[fr] = M.totals(rows)
        rows_n = []
        for b in ids:
            s = base[b]["est"]["naive"].get("sens", {}).get(fr)
            if s:
                rows_n.append(dict(s, v_pairs=base[b]["est"]["naive"]["row"]["v_pairs"]))
        sens[fr + "_naive"] = M.totals(rows_n)
    report["frame_sensitivity"] = sens
    var_out = []
    for v in variants:
        if v["same_as_base"]:
            var_out.append(dict(v, totals=None))
            continue
        rows = [res[("var", v["name"], b)]["est"]["model"]["row"] for b in ids]
        var_out.append(dict(v, totals=M.totals(rows)))
    report["variants"] = var_out
    if any(k[0] == "gnss_full" for k in res):
        report["gnss_full"] = gnss_full_compare(res, base, ids)

    def _cr(c):
        return None if c is None else {k: v for k, v in c.items() if k != "stamp"}
    report["crashes"] = {b: {e: _cr(c) for e, c in cs.items()} for b, cs in report["crashes"].items()}
    runs = {b:{e: dict(base[b]["est"][e]["row"], crash=_cr(base[b]["est"][e]["crash"]))
                for e in base[b]["est"] if "row" in base[b]["est"][e]} for b in sorted(base)}
    inj = []
    for k, r in sorted(((k, r) for k, r in res.items() if k[0] == "inj"),
                       key=lambda kv: (inj_runs.index(kv[0][2]), kinds.index(kv[0][1]))):
        if r.get("skipped"):
            inj.append(dict(kind=k[1], bag=k[2], skipped=r["skipped"]))
            continue
        inj.append(inject_metrics(r, base[k[2]]))
    timing = dict(wall_total_s=round(wall, 1), workers=workers,
                  per_task={"|".join(map(str, k)): round(r.get("wall_s", 0.0), 2)
                            for k, r in res.items()},
                  events_per_s=round(sum(r.get("n_events", 0) for r in res.values())
                                     / max(sum(r.get("wall_s", 0.0) for r in res.values()), 1e-9)),
                  git=git_head())
    return dict(summary=report, runs=runs, inject=inj, kinds=kinds), timing, base, res


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet", default="eval",
                    help="eval (по умолчанию: config/eval/tram_eval.yaml | tram.yaml | "
                         "tram_calibration.json, иначе json с пометкой об утечке) | jury | json | путь")
    ap.add_argument("--set", default="", help="переопределить поля листа: k=v,k=v")
    ap.add_argument("--map", default="train", help="train (по умолчанию) | jury | none | путь .npz")
    ap.add_argument("--rebuild-map", action="store_true", help="пересобрать карту train")
    ap.add_argument("--gnss", default="3", help="секунд GNSS в связку (3) или full")
    ap.add_argument("--frame", default="mgrs", choices=M.FRAMES,
                    help="система эталона для ошибок положения (mgrs — как у судьи)")
    ap.add_argument("--runner-frame", default="auto", choices=R.RUNNER_FRAMES,
                    help="система выхода Runner'а: auto — по параметру листа projection, "
                         "иначе equirect (код до правок), с проверкой по величине")
    ap.add_argument("--runner-grid", default=None,
                    help="квадрат MGRS выхода Runner'а (\"\" — перенос по точке); "
                         "по умолчанию — параметр листа mgrs_grid")
    ap.add_argument("--judge-grid", default="",
                    help="соглашение судьи на границе квадратов: \"\" — перенос по точке, "
                         "или код квадрата (37UDB) — непрерывно от него")
    ap.add_argument("--runs", default="", help="прогоны через запятую (по умолчанию holdout_scored)")
    ap.add_argument("--inject-runs", default=",".join(INJECT_RUNS))
    ap.add_argument("--inject-kinds", default="", help="виды инъекций (по умолчанию все)")
    ap.add_argument("--no-inject", action="store_true")
    ap.add_argument("--no-variants", action="store_true")
    ap.add_argument("--no-gnss-full", action="store_true")
    ap.add_argument("--quick", action="store_true",
                    help=f"CI: прогоны {','.join(QUICK_RUNS)}, первые {QUICK_S:.0f} с, "
                         f"инъекции {','.join(QUICK_KINDS)}")
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--cache", default="", help="каталог кэша (по умолчанию analysis/cache)")
    ap.add_argument("--data", default="", help="каталог прогонов (по умолчанию data/)")
    ap.add_argument("--out", default="out/eval", help="каталог JSON")
    ap.add_argument("--label", default="", help="подпись версии в EVAL.md (например «до правок»)")
    ap.add_argument("--no-doc", action="store_true", help="не писать docs/EVAL.md и графики")
    ap.add_argument("--doc", default="docs/EVAL.md")
    ap.add_argument("--probe-glob", default="out/realtime/**/summary.json,out/ros_e2e/*/summary.json",
                    help="сводки tools/ros_probe.py для раздела «Реальное время»")
    ap.add_argument("--render-only", action="store_true",
                    help="только пересобрать docs/EVAL.md и графики из готовых out/eval/*.json "
                         "и plotdata.npz (без прогонов)")
    ap.add_argument("--check-determinism", action="store_true",
                    help="прогнать всё второй раз и сравнить JSON побайтно")
    args = ap.parse_args()

    if args.cache:
        bagio.set_cache(Path(args.cache).resolve())
    if args.data:
        bagio.set_data(Path(args.data).resolve())
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    t_start = time.perf_counter()

    def log(msg):
        print(f"[eval {time.perf_counter() - t_start:6.0f} с] {msg}", flush=True)

    if args.render_only:
        import eval_report
        result = dict(summary=json.loads((out / "summary.json").read_text(encoding="utf-8")),
                      runs=json.loads((out / "runs.json").read_text(encoding="utf-8")),
                      inject=json.loads((out / "inject.json").read_text(encoding="utf-8")))
        result["kinds"] = result["summary"]["meta"].get("kinds") or list(
            dict.fromkeys(x["kind"] for x in result["inject"]))
        timing = json.loads((out / "timing.json").read_text(encoding="utf-8"))
        pd = out / "plotdata.npz"
        if pd.exists():
            base_p, res_p = eval_report.load_plotdata(pd)
            pics = eval_report.draw(result, base_p, res_p, ROOT)
        else:
            pics = eval_report.existing_pics(ROOT)
        (ROOT / args.doc).write_text(eval_report.render(result, timing, args, pics, ROOT),
                                     encoding="utf-8")
        log(f"пересобран {args.doc} из {out.relative_to(ROOT)}")
        return
    result, timing, base, res = evaluate(args, log)
    files = {"summary.json": dumps(result["summary"]), "runs.json": dumps(result["runs"]),
             "inject.json": dumps(result["inject"])}
    for name, text in files.items():
        (out / name).write_text(text, encoding="utf-8")
    digests = {n: hashlib.sha256(t.encode("utf-8")).hexdigest() for n, t in files.items()}
    check = None
    if args.check_determinism:
        log("проверка детерминизма: второй проход")
        r2, _, _, _ = evaluate(args, log)
        d2 = {"summary.json": dumps(r2["summary"]), "runs.json": dumps(r2["runs"]),
              "inject.json": dumps(r2["inject"])}
        same = {n: hashlib.sha256(t.encode("utf-8")).hexdigest() == digests[n] for n, t in d2.items()}
        check = dict(identical=all(same.values()), files=same)
        log(f"детерминизм: {'JSON совпали побайтно' if check['identical'] else 'РАЗЛИЧИЯ ' + str(same)}")
    timing["wall_all_s"] = round(time.perf_counter() - t_start, 1)
    timing["sha256"] = digests
    if check is not None:
        timing["determinism"] = check
    (out / "timing.json").write_text(dumps(timing), encoding="utf-8")
    s = result["summary"]["totals"]["all"]
    m, n = s["model"], s["naive"]
    log(f"модель: v MAE {m['v_mae']:.4f} RMSE {m['v_rmse']:.4f} смещение {m['v_bias']:+.4f} "
        f"±2σ {m.get('cov2s_v', float('nan')):.3f}; 3D ср. {m['p3d_mean']:.2f} м; "
        f"along RMSE {m.get('along_rmse', float('nan')):.2f} м")
    log(f"база:   v MAE {n['v_mae']:.4f} RMSE {n['v_rmse']:.4f} смещение {n['v_bias']:+.4f}; "
        f"3D ср. {n['p3d_mean']:.2f} м")
    if not args.no_doc:
        import eval_report
        # документ — из тех же округлённых JSON, что и --render-only: байты совпадают
        shown = dict(summary=json.loads(files["summary.json"]), runs=json.loads(files["runs.json"]),
                     inject=json.loads(files["inject.json"]), kinds=result["kinds"])
        eval_report.write(shown, timing, base, res, args, ROOT, out)
        log(f"записано {args.doc} и docs/img/eval_*.png")
    log(f"готово за {time.perf_counter() - t_start:.0f} с; JSON в {out.relative_to(ROOT)}")
    if check is not None and not check["identical"]:
        sys.exit(2)


if __name__ == "__main__":
    main()
