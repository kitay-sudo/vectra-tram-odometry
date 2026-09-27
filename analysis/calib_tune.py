"""Шаг 2г: параметры фильтра, подобранные прогоном связки на подгоночных прогонах.

Таблица привода, масштаб и шум датчиков — физика, их даёт calib_drive.py и
calib_sheet.py. Здесь подбираются гиперпараметры фильтра, которые нельзя
получить регрессией: шум процесса q_v, крип (c_creep, c_creep_drag — заглушки
против нуля; регрессия calib_sheet.py даёт крип около нуля), адаптация
масштабов привода и неопределённость крипа sigma_creep.

Критерий — средняя |ошибка| скорости (MAE) против |v| GNSS master по всем
парам подгоночных прогонов; из точек в пределах 1 % от лучшей — наименьший
q_v (строже к отказам). Адаптация и sigma_creep меняются, только если это
улучшает MAE больше чем на 0,5 %.

    python3 analysis/calib_tune.py eval    # только split train (train_unique)
    python3 analysis/calib_tune.py jury    # гиперпараметры берутся из
                                           # calib_tuned_eval.json (выбраны на
                                           # train), без повторного перебора

Отложенные прогоны (holdout) в листе оценки не участвуют ни в подгонке, ни в
выборе. Для листа жюри гиперпараметры, выбранные на train, переносятся как
есть: непрерывные величины (таблица, масштаб, шумы, σ) calib_sheet.py и
calib_sigma.py подгоняют по всем данным.

Выход: analysis/calib_tuned_<eval|jury>.json — {"params": {...}, "grid": [...]}.
calib_sheet.py подмешивает "params" в лист.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import calib_eval as CE                                         # noqa: E402

SHEET = {"eval": CE.PKG / "config" / "eval" / "tram_calibration.json",
         "jury": CE.PKG / "config" / "tram_calibration.json"}
IDS = {"eval": "train_unique", "jury": "all_unique"}
STUB = dict(c_creep=0.02, c_creep_drag=0.002)
ZERO = dict(c_creep=0.0, c_creep_drag=0.0)


TUNED_KEYS = ("q_v", "c_creep", "c_creep_drag", "adapt_on", "sigma_creep")
# Решения по приёмке, а не по перебору. Окно адаптации считается по реальному времени
# (реальное время: окно 3,07 с, k_изм 1,00 ± 0,29, ворота 74 % окон на train),
# но выигрыша нет: train MAE 0,0395 с адаптацией против 0,0396 без неё,
# holdout_scored 0,0433 против 0,0428, инъекции «нули» и «пропуск» 20 с — без
# выигрыша по пути. Правило приёмки — «если вредит, выключить параметром и
# описать», поэтому адаптация выключена: adapt_on = false.
DECISIONS = {"adapt_on": False}


def grid_first():
    g = [dict(STUB, q_v=q) for q in (0.05, 0.3)]
    g += [dict(ZERO, q_v=q) for q in (0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1.2)]
    return g


def base_digest(path):
    """Отпечаток листа без перебираемых полей и выходной σ: строки перебора
    из кэша годны, только пока остальной лист (таблица, масштаб, шумы) тот же."""
    import hashlib
    d = json.loads(Path(path).read_text(encoding="utf-8"))["params"]
    keep = {k: v for k, v in d.items()
            if k not in TUNED_KEYS and not k.startswith(("sv_", "ss_"))}
    return hashlib.sha256(json.dumps(keep, sort_keys=True).encode()).hexdigest()[:16]


def pick(rows):
    """Лучший по MAE; из точек в пределах 1 % от лучшей — с наименьшим q_v:
    меньший шум процесса — больше веса модели, строже проверка показаний
    при отказах (буксование, выбросы), а по MAE разница в пределах шума."""
    best = min(r["mae"] for r in rows)
    near = [r for r in rows if r["mae"] <= best * 1.01]
    return min(near, key=lambda r: (r["set"]["q_v"], r["mae"]))


def row_of(setp, s):
    a = s["all"]
    return dict(set=setp, runs=s["runs"], mae=a["mae"], bias=a["bias"], rmse=a["rmse"],
                cov2s=s["cov2s"], naive_mae=s["naive"]["mae"],
                rover_mae=s.get("rover", {}).get("model", {}).get("mae"),
                by_phase={k: dict(mae=v["mae"], bias=v["bias"], cov2s=v["cov2s"])
                          for k, v in s["by_phase"].items()},
                adapt=s.get("adapt"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("which", choices=("eval", "jury"))
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    out = HERE / f"calib_tuned_{args.which}.json"
    if args.which == "jury":
        ev = json.loads((HERE / "calib_tuned_eval.json").read_text(encoding="utf-8"))
        keep = {k: ev["params"][k] for k in ev["params"] if not k.startswith(("sv_", "ss_"))}
        prev = json.loads(out.read_text(encoding="utf-8")) if out.exists() else {}
        params = {k: v for k, v in prev.get("params", {}).items()
                  if k.startswith(("sv_", "ss_"))}
        params.update(keep)
        out.write_text(json.dumps(dict(
            params=params, source="гиперпараметры фильтра выбраны на split train "
            "(calib_tuned_eval.json) и перенесены в лист жюри без перебора",
            grid=ev["grid"], notes=ev.get("notes")), ensure_ascii=False, indent=1),
            encoding="utf-8")
        print("записано:", out, params)
        return

    ids = CE.split_ids(IDS[args.which])
    rows = []
    # кэш строк перебора: повторный запуск (например, с расширенной сеткой)
    # не пересчитывает уже посчитанные точки при том же остальном листе
    digest = base_digest(SHEET[args.which])
    cache_f = CE.OUT / f"tune_cache_{args.which}.json"
    cache = json.loads(cache_f.read_text(encoding="utf-8")) if cache_f.exists() else {}

    def run(setp):
        t0 = time.time()
        key = digest + "|" + json.dumps(setp, sort_keys=True)
        if key in cache:
            r = cache[key]
            print(f"{json.dumps(setp)} (кэш): MAE {r['mae']:.4f} bias {r['bias']:+.4f}", flush=True)
        else:
            p = CE.load_sheet(SHEET[args.which], setp)
            _, s = CE.speed_eval(p, ids, args.workers)
            r = row_of(setp, s)
            print(CE.fmt_speed(json.dumps(setp), s), f"({time.time() - t0:.0f} с)", flush=True)
            cache[key] = r
            CE.OUT.mkdir(parents=True, exist_ok=True)
            cache_f.write_text(json.dumps(cache, ensure_ascii=False, indent=1), encoding="utf-8")
        rows.append(r)
        return r

    for setp in grid_first():
        run(setp)
    zero_rows = [r for r in rows if r["set"]["c_creep"] == 0.0]
    best = pick(zero_rows)
    stub_best = min((r for r in rows if r["set"]["c_creep"] > 0), key=lambda r: r["mae"])
    # крип: ноль против заглушки при своём лучшем q_v
    creep = best if best["mae"] <= stub_best["mae"] else stub_best
    base = dict(creep["set"])
    # адаптация масштабов привода и неопределённость крипа — поверх лучшего
    r_ad = run(dict(base, adapt_on=False))
    r_sc = run(dict(base, sigma_creep=0.0))
    final = dict(base)
    if r_ad["mae"] < creep["mae"] * 0.995:
        final["adapt_on"] = False
    if r_sc["mae"] < creep["mae"] * 0.995:
        final["sigma_creep"] = 0.0
    final.update(DECISIONS)
    prev = json.loads(out.read_text(encoding="utf-8")) if out.exists() else {}
    params = {k: v for k, v in prev.get("params", {}).items() if k.startswith(("sv_", "ss_"))}
    params.update(final)
    notes = (f"выбор по MAE на {len(ids)} прогонах {IDS[args.which]} (из них с GNSS "
             f"{rows[0]['runs']}): крип {'ноль' if base['c_creep'] == 0 else 'заглушка'}, "
             f"q_v {base['q_v']}; адаптация по перебору: "
             f"{'выключить' if r_ad['mae'] < creep['mae'] * 0.995 else 'оставить'}"
             f" (MAE {r_ad['mae']:.4f} без неё против {creep['mae']:.4f}); sigma_creep "
             f"{'0' if final.get('sigma_creep') == 0.0 else 'как в Params'} "
             f"(MAE {r_sc['mae']:.4f} при 0); решения по приёмке: {DECISIONS}")
    out.write_text(json.dumps(dict(params=params, grid=rows, notes=notes, base_digest=digest),
                              ensure_ascii=False, indent=1), encoding="utf-8")
    print(notes)
    print("записано:", out, params)


if __name__ == "__main__":
    main()
