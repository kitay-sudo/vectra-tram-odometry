"""docs/EVAL.md и графики docs/img/eval_*.png по результатам tools/eval.py.

Всё, что в документе, берётся из out/eval/*.json этого же прогона (плюс
сводки tools/ros_probe.py для раздела «Реальное время», если они есть).
"""

import glob
import json
import math
from pathlib import Path

import numpy as np

import eval_metrics as M
import inject as I

# палитра (dataviz, проверена validate_palette.js: CVD ΔE 9,2, норм. ΔE 27,6)
C_MODEL = "#2a78d6"
C_NAIVE = "#eb6834"
C_REF = "#52514e"
C_GRID = "#e4e3df"
C_WASH = "#f0efec"
C_TEXT = "#0b0b0b"
C_TEXT2 = "#52514e"
SURFACE = "#fcfcfb"

PLOT_RUN = "30618_e9a34502"
SUB15_JSON = dict(v_mae=0.0489, v_bias=0.0130, cov2s_v=0.756, p3d_mean=5.04,
                  naive_v_mae=0.0483, naive_p3d_mean=3.93)
# отпечаток кода пакета (tools/eval.src_digest), на котором считал аудит sub15:
# main 56933cc / e74471f — runner.py, estimator_core.py, track_map.py до правок
AUDIT_PKG_SHA = "5f634e9a5a63564d"
KIND_SHORT = dict(front_zero="отказ передней (0)", rear_drop="отказ задней (30639)",
                  both_zero="обе = 0, 20 с", both_stuck="обе залипли, 20 с",
                  dropout="пропуск 2 с", gap_all="пропуск всех входов 2 с",
                  skid_brake="юз −30 %", spin_traction="буксование +30 %",
                  outliers="выбросы ×3", noise="шум ×5", nan="NaN", stamp_jump="скачок метки +30 с")


# ------------------------------------------------------------------ форматирование

def f(x, nd=2, sign=False):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    s = f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"
    return s.replace(".", ",").replace("-", "−")


def pct(x, nd=1):
    return "—" if x is None or not math.isfinite(x) else f(100.0 * x, nd) + " %"


def g(d, *keys, default=None):
    for k in keys:
        if d is None:
            return default
        d = d.get(k) if isinstance(d, dict) else None
    return default if d is None else d


def table(head, rows):
    def cell(c):
        return str(c).replace("|", "\\|")         # «|Δv|» не должен ломать таблицу
    out = ["| " + " | ".join(cell(h) for h in head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(cell(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


# ------------------------------------------------------------------ графики

def _style(plt):
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": C_GRID, "axes.labelcolor": C_TEXT2, "xtick.color": C_TEXT2,
        "ytick.color": C_TEXT2, "text.color": C_TEXT, "axes.grid": True, "grid.color": C_GRID,
        "grid.linewidth": 0.8, "grid.linestyle": "-", "axes.spines.top": False,
        "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10,
        "axes.titleweight": "semibold", "legend.frameon": False, "lines.linewidth": 1.4,
        "lines.solid_capstyle": "round", "svg.hashsalt": "eval", "path.simplify": True,
    })


def _save(fig, path):
    fig.savefig(path, dpi=110, metadata={"Software": None})


def plot_speed(base, img, bag):
    import matplotlib.pyplot as plt
    r = base.get(bag)
    if not r or "series" not in r["est"].get("model", {}):
        return None
    sm, sn = r["est"]["model"]["series"], r["est"]["naive"]["series"]
    t0 = r["t_first"]
    tm = (sm["tg"] - t0) / 60.0
    tn = (sn["tg"] - t0) / 60.0
    svm = sm["SV"][M.nearest(sm["T"], sm["tg"])[0]]
    fig, ax = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True,
                           gridspec_kw=dict(height_ratios=[1, 1.4]))
    ax[0].plot(tm, sm["vg"], color=C_REF, lw=1.2, label="GNSS master")
    ax[0].set_ylabel("скорость, м/с")
    ax[0].set_title(f"{bag}: скорость по GNSS и ошибка оценки (выход − GNSS)", loc="left")
    ax[1].fill_between(tm, -2 * svm, 2 * svm, color=C_MODEL, alpha=0.10, lw=0, label="модель ±2σ")
    ax[1].plot(tn, sn["ev"], color=C_NAIVE, lw=1.0, label="база «только колесо»")
    ax[1].plot(tm, sm["ev"], color=C_MODEL, lw=1.0, label="модель")
    lim = max(0.3, float(np.nanpercentile(np.abs(np.r_[sm["ev"], sn["ev"]]), 99.8)) * 1.2)
    ax[1].set_ylim(-lim, lim)
    ax[1].set_ylabel("ошибка, м/с")
    ax[1].set_xlabel("время от начала записи, мин")
    ax[1].legend(loc="upper right", ncol=3)
    fig.tight_layout()
    p = img / "eval_speed_error.png"
    _save(fig, p)
    plt.close(fig)
    return p.name


def _med(x, w=51):
    """Скользящая медиана по окну w (NaN пропускаются)."""
    if len(x) < w:
        return x.copy()
    from numpy.lib.stride_tricks import sliding_window_view
    h = w // 2
    pad = np.r_[np.full(h, np.nan), x, np.full(h, np.nan)]
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            return np.nanmedian(sliding_window_view(pad, w), axis=1)


def plot_position(base, ids, img):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(10, 6.2), sharex=True, sharey="row")
    for col, (est, color, title) in enumerate((("model", C_MODEL, "модель"),
                                               ("naive", C_NAIVE, "база «только колесо»"))):
        for b in ids:
            s = base[b]["est"].get(est, {}).get("samples", {}).get("p")
            if not s or "sref" not in s:
                continue
            x = np.asarray(s["sref"], float) / 1000.0
            # скользящая медиана 5 с (51 фикс): скачки самого эталона GNSS иначе
            # рисуют «иглы» на весь график; числа в таблицах — без сглаживания
            al, d3 = _med(np.asarray(s["al"], float)), _med(np.asarray(s["d3"], float))
            back = np.r_[False, np.diff(x) < -0.001]      # скачок эталона назад: разрыв линии
            x = x.copy()
            x[back] = np.nan
            ax[0, col].plot(x, al, color=color, lw=0.9, alpha=0.75)
            ax[1, col].plot(x, d3, color=color, lw=0.9, alpha=0.75)
        ax[0, col].set_title(f"{title}, {len(ids)} прогонов", loc="left")
        ax[1, col].set_xlabel("путь по эталону GNSS, км")
    ax[0, 0].set_ylabel("вдоль пути, м (+ — впереди)")
    ax[1, 0].set_ylabel("3D, м")
    fig.suptitle("Ошибка положения по пути (MGRS; скользящая медиана 5 с)", x=0.01, ha="left",
                 fontsize=10, fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = img / "eval_position_error.png"
    _save(fig, p)
    plt.close(fig)
    return p.name


def plot_phases(pooled, img):
    import matplotlib.pyplot as plt
    ph = [k for k in M.PHASES if k in pooled["model"]["by_phase"]]
    if not ph:
        return None
    x = np.arange(len(ph))
    w = 0.34
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    for k, (key, title) in enumerate((("bias", "смещение, м/с"), ("mae", "MAE, м/с"))):
        for j, (est, color, lab) in enumerate((("model", C_MODEL, "модель"),
                                               ("naive", C_NAIVE, "база"))):
            vals = [pooled[est]["by_phase"].get(p, {}).get(key, np.nan) for p in ph]
            ax[k].bar(x + (j - 0.5) * w, vals, width=w - 0.03, color=color, label=lab)
        ax[k].axhline(0, color=C_TEXT2, lw=0.8)
        ax[k].grid(axis="x", visible=False)
        ax[k].set_xticks(x)
        ax[k].set_xticklabels([M.PHASES_RU[p] for p in ph])
        ax[k].set_title(title, loc="left")
    ax[0].legend(loc="upper left")
    fig.tight_layout()
    p = img / "eval_phase.png"
    _save(fig, p)
    plt.close(fig)
    return p.name


def plot_inject(res, base, bag, kinds, img):
    import matplotlib.pyplot as plt
    cells = [(k, res.get(("inj", k, bag))) for k in kinds]
    cells = [(k, r) for k, r in cells if r and not r.get("skipped")]
    if not cells or bag not in base:
        return []
    names = []
    for what, fname, ylab in (("v", "eval_inject_speed.png", "скорость, м/с"),
                              ("al", "eval_inject_along.png", "ошибка вдоль пути, м")):
        n = len(cells)
        nc = 4
        nr = int(math.ceil(n / nc))
        fig, axs = plt.subplots(nr, nc, figsize=(12, 2.6 * nr), squeeze=False)
        for ax in axs.flat[n:]:
            ax.set_visible(False)
        for ax, (kind, r) in zip(axs.flat, cells):
            info = r["inject"]
            t0, te = info["t0"], info["t0"] + I.eval_window(kind)
            lo, hi = t0 - 30.0, te + 60.0
            ax.axvspan(0, te - t0, color=C_WASH, lw=0)
            for est, color in (("naive", C_NAIVE), ("model", C_MODEL)):
                S = r["est"][est]["series"]
                if what == "v":
                    m = (S["T"] >= lo) & (S["T"] < hi)
                    ax.plot(S["T"][m] - t0, S["V"][m], color=color, lw=1.2)
                else:
                    m = (S["tp"] >= lo) & (S["tp"] < hi)
                    ax.plot(S["tp"][m] - t0, S["al"][m], color=color, lw=1.2)
            if what == "v":
                S = r["est"]["model"]["series"]
                m = (S["tg"] >= lo) & (S["tg"] < hi)
                ax.plot(S["tg"][m] - t0, S["vg"][m], color=C_REF, lw=1.0, ls="-", alpha=0.9)
            crash = r["est"]["model"]["crash"]
            ttl = KIND_SHORT.get(kind, kind) + (" — модель упала" if crash else "")
            ax.set_title(ttl, loc="left", fontsize=9)
            ax.set_xlim(lo - t0, hi - t0)
        for ax in axs[:, 0]:
            ax.set_ylabel(ylab)
        for ax in axs[-1, :]:
            ax.set_xlabel("с от начала аномалии")
        from matplotlib.lines import Line2D
        hs = [Line2D([], [], color=C_MODEL, lw=2), Line2D([], [], color=C_NAIVE, lw=2)]
        labs = ["модель", "база «только колесо»"]
        if what == "v":
            hs.append(Line2D([], [], color=C_REF, lw=2))
            labs.append("GNSS master")
        fig.legend(hs, labs, loc="upper right", ncol=3)
        fig.suptitle(f"Инъекции в {bag}; серая полоса — окно аномалии", x=0.01, ha="left",
                     fontsize=10, fontweight="semibold")
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        _save(fig, img / fname)
        plt.close(fig)
        names.append(fname)
    return names


# ------------------------------------------------------------------ реальное время

def probe_rows(root, patterns):
    rows = []
    for pat in patterns.split(","):
        for p in sorted(glob.glob(str(root / pat), recursive=True)):
            try:
                d = json.loads(Path(p).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            o = g(d, "outputs", "velocity") or {}
            L = d.get("latency", {}) or {}
            P = d.get("node_process", {}) or {}
            i2o = L.get("in2out_vehicle_ms") or {}
            cpu = P.get("cpu_pct_1core") or {}
            rss = P.get("rss_mb_first_last_max")
            rows.append([Path(p).parent.name, o.get("count", "—"), f(o.get("rate_stamp_hz"), 1),
                         f(i2o.get("p50"), 1), f(i2o.get("p99"), 1), f(i2o.get("max"), 1),
                         f(cpu.get("mean"), 1), f(cpu.get("max"), 1),
                         f(rss[-1], 0) if isinstance(rss, list) and rss else "—",
                         f(P.get("rss_slope_mb_per_min_after_30s"), 2)])
    return rows


# ------------------------------------------------------------------ сводка инъекций

def inject_summary(inj, kinds):
    """По видам: среднее по прогонам MAE во время, наибольшая |Δ вдоль| через
    300 с, падения, флаги модели."""
    out = []
    for k in kinds:
        xs = [x for x in inj if x["kind"] == k and not x.get("skipped")]
        if not xs:
            continue
        row = dict(kind=k, n=len(xs))
        for est in ("model", "naive"):
            es = [x["est"].get(est) or {} for x in xs]
            mae = [g(e, "during", "v_mae") for e in es if g(e, "during", "v_mae") is not None]
            tail = [abs(e["d_along_tail"]) for e in es if e.get("d_along_tail") is not None]
            row[est] = dict(mae=float(np.mean(mae)) if mae else None,
                            tail=float(max(tail)) if tail else None,
                            crash=sum(1 for e in es if e.get("crash")))
        em = [x["est"].get("model") or {} for x in xs]
        fv = [g(e, "during", "frac_valid") for e in em if g(e, "during", "frac_valid") is not None]
        fa = [g(e, "during", "frac_amb") for e in em if g(e, "during", "frac_amb") is not None]
        fs = [g(e, "during", "frac_slip") for e in em if g(e, "during", "frac_slip") is not None]
        cv = [g(e, "during", "cov2s") for e in em if g(e, "during", "cov2s") is not None]
        row["flag"] = dict(valid=min(fv) if fv else None, amb=max(fa) if fa else None,
                           slip=max(fs) if fs else None, cov2s=float(np.mean(cv)) if cv else None)
        out.append(row)
    return out


def verdict(r):
    """Короткий вывод по виду инъекции (для таблицы и «Главного»)."""
    m, n, fl = r["model"], r["naive"], r["flag"]
    if m["crash"]:
        return f"модель падает ({m['crash']} из {r['n']})"
    flagged = (fl["valid"] is not None and fl["valid"] < 0.99) or (fl["amb"] or 0) > 0.05 \
        or (fl["slip"] or 0) > 0.05
    worse = m["mae"] is not None and n["mae"] is not None and m["mae"] > 1.5 * n["mae"] + 0.02
    better = m["mae"] is not None and n["mae"] is not None and n["mae"] > 1.5 * m["mae"] + 0.02
    big = m["mae"] is not None and m["mae"] > 0.3
    parts = []
    if big and not flagged:
        parts.append("не замечено: ошибка без флага")
    elif flagged:
        parts.append("флаг есть")
    if better:
        parts.append("модель лучше базы")
    elif worse:
        parts.append("модель хуже базы")
    if (m["tail"] or 0) > 20:
        parts.append(f"остаток {f(m['tail'], 0)} м")
    return "; ".join(parts) or "в пределах нормы"


OK_VERDICTS = ("в пределах нормы", "флаг есть", "флаг есть; модель лучше базы", "модель лучше базы")


# ------------------------------------------------------------------ документ

def img_dir(root, args):
    """Каталог графиков — img/ рядом с документом (docs/img для docs/EVAL.md)."""
    return (root / args.doc).parent / "img"


def existing_pics(root, img=None):
    img = img or root / "docs" / "img"
    names = dict(speed="eval_speed_error.png", pos="eval_position_error.png", phase="eval_phase.png")
    out = {k: (v if (img / v).exists() else None) for k, v in names.items()}
    out["inj"] = [n for n in ("eval_inject_speed.png", "eval_inject_along.png") if (img / n).exists()]
    return out


SERIES_KEYS = ("T", "V", "SV", "tg", "ev", "vg", "tp", "al", "d3")
SAMPLE_KEYS = ("sref", "al", "d3")


def save_plotdata(path, result, base, res):
    """Минимум рядов для графиков: --render-only перерисует их без прогона."""
    ids = result["summary"]["meta"]["runs"]
    arr = {}
    for b, r in base.items():
        arr[f"t_first|{b}"] = np.array(r["t_first"])
        for est, e in r["est"].items():
            for k in SERIES_KEYS:
                if k in e.get("series", {}):
                    arr[f"series|{b}|{est}|{k}"] = np.asarray(e["series"][k])
            sp = (e.get("samples") or {}).get("p") or {}
            for k in SAMPLE_KEYS:
                if k in sp:
                    arr[f"samples|{b}|{est}|{k}"] = np.asarray(sp[k])
    for key, r in res.items():
        if key[0] != "inj" or r.get("skipped"):
            continue
        _, kind, b = key
        arr[f"inj_t0|{kind}|{b}"] = np.array(r["inject"]["t0"])
        for est, e in r["est"].items():
            arr[f"inj_crash|{kind}|{b}|{est}"] = np.array(e.get("crash") is not None)
            for k in SERIES_KEYS:
                if k in e.get("series", {}):
                    arr[f"inj|{kind}|{b}|{est}|{k}"] = np.asarray(e["series"][k])
    arr["_runs"] = np.array(ids)
    np.savez_compressed(path, **arr)


def load_plotdata(path):
    """Обратно в структуры base / res, как их видят функции графиков."""
    z = np.load(path)
    base, res = {}, {}
    for key in z.files:
        parts = key.split("|")
        if parts[0] == "t_first":
            base.setdefault(parts[1], {"est": {}})["t_first"] = float(z[key])
        elif parts[0] in ("series", "samples"):
            _, b, est, k = parts
            e = base.setdefault(b, {"est": {}})["est"].setdefault(est, {})
            if parts[0] == "series":
                e.setdefault("series", {})[k] = z[key]
            else:
                e.setdefault("samples", {}).setdefault("p", {})[k] = z[key]
        elif parts[0] == "inj_t0":
            _, kind, b = parts
            res.setdefault(("inj", kind, b), {"est": {}})["inject"] = dict(t0=float(z[key]))
        elif parts[0] == "inj_crash":
            _, kind, b, est = parts
            e = res.setdefault(("inj", kind, b), {"est": {}})["est"].setdefault(est, {})
            e["crash"] = {"error": "crash"} if bool(z[key]) else None
        elif parts[0] == "inj":
            _, kind, b, est, k = parts
            e = res.setdefault(("inj", kind, b), {"est": {}})["est"].setdefault(est, {})
            e.setdefault("series", {})[k] = z[key]
    return base, res


def draw(result, base, res, root, img=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    S = result["summary"]
    ids = S["meta"]["runs"]
    img = img or root / "docs" / "img"
    img.mkdir(parents=True, exist_ok=True)
    pics = {}
    pics["speed"] = plot_speed(base, img, PLOT_RUN if PLOT_RUN in base else ids[0])
    pics["pos"] = plot_position(base, ids, img)
    pics["phase"] = plot_phases(S["pooled"], img)
    inj_bag = next((x["bag"] for x in result["inject"] if not x.get("skipped")), None)
    pics["inj"] = plot_inject(res, base, inj_bag, result["kinds"], img) if inj_bag else []
    return pics


def write(result, timing, base, res, args, root, out=None):
    if out is not None:
        save_plotdata(out / "plotdata.npz", result, base, res)
    pics = draw(result, base, res, root, img_dir(root, args))
    doc = render(result, timing, args, pics, root)
    (root / args.doc).write_text(doc, encoding="utf-8")


def _tot(S, grp, est):
    return g(S, "totals", grp, est) or {}


def wp5_threshold(zt):
    """Порог WP5 на 15 чистых: порог TODO остаётся, если вариант без крипа его
    держит; иначе — ближайший сверху, но смещение не мягче +0,005 (TODO)."""
    if not zt or zt.get("v_mae") is None:
        return "—"
    mae, bias, p3 = zt["v_mae"], abs(zt.get("v_bias") or 0.0), zt.get("p3d_mean") or 0.0
    if mae <= 0.045 and bias <= 0.005 and p3 <= 4.1:
        return (f"порог TODO сохраняется: MAE ≤ 0,045; смещение ≤ +0,005; 3D ≤ 4,1 м (запас "
                f"{f(0.045 - mae, 4)} / {f(0.005 - bias, 4)} / {f(4.1 - p3, 2)})")
    return (f"MAE ≤ {f(max(0.045, math.ceil(mae * 1000 + 0.5) / 1000), 3)}; смещение ≤ +0,005 (как в "
            f"TODO; сейчас {f(bias, 4)} — не выполнено)" if bias > 0.005 else
            f"MAE ≤ {f(max(0.045, math.ceil(mae * 1000 + 0.5) / 1000), 3)}; смещение ≤ +0,005") +         f"; 3D ≤ {f(max(4.1, math.ceil(p3 * 10 + 1) / 10), 1)} м"


def run_mean(result, ids, est, key):
    """Среднее поля key по прогонам (None, если его нет ни у одного)."""
    x = [result["runs"][b][est][key] for b in ids
         if est in result["runs"].get(b, {}) and result["runs"][b][est].get(key) is not None]
    return float(np.mean(x)) if x else None


def completeness(t, crashes):
    """Строка «пары, NaN, падения» для итога t (totals)."""
    n_cr = sum(1 for cs in crashes.values() if cs.get("model"))
    return (f"пар скорости {pct(t.get('v_pair_frac'), 2)} меток GNSS vel (NaN в выходе: "
            f"{t.get('v_out_nan', 0)}), пар положения {pct(t.get('p_pair_frac'), 2)} меток GNSS fix "
            f"(строк выхода без опубликованного положения: {t.get('p_out_invalid', 0)}, положение NaN: "
            f"{t.get('p_nan', 0)}); падений связки: {n_cr}"
            + (f"; прогонов без выхода: {t.get('runs_empty')}" if t.get("runs_empty") else ""))


def render(result, timing, args, pics, root):
    S = result["summary"]
    meta = S["meta"]
    ids = meta["runs"]
    L = []
    A = L.append
    label = meta.get("label") or "без подписи"
    A("# Оценка точности и устойчивости (tools/eval.py)")
    A("")
    A(f"> **Версия чисел: {label}.** Документ целиком сгенерирован `tools/eval.py` "
      f"(не править руками). Код пакета: sha `{meta['pkg_src_sha']}`"
      f"{', git ' + timing['git'] if timing.get('git') else ''}. "
      f"Лист: {meta['sheet']['label']} (sha `{meta['sheet']['sha']}`). "
      f"Карта: {meta['map']['label']}"
      f"{' (sha `' + meta['map']['sha'] + '`)' if meta['map']['sha'] else ''}. "
      f"GNSS в связку: {'весь прогон' if meta['gnss'] == 'full' else 'первые ' + meta['gnss'] + ' с'}. "
      f"Прогонов: {len(ids)}{' (--quick, первые 300 с)' if meta['quick'] else ''}.")
    for leak in (meta["sheet"].get("leak"), meta["map"].get("leak")):
        if leak:
            A(f">\n> **{leak}.** Эти числа не отчётные, пока нет оценочного листа.")
    if meta["sheet"].get("kind") == "json":
        A(">\n> Лист json — не то, что читает нода: боевая нода берёт `config/tram.yaml`"
          + (" (аудит sub15 на этом коде: на yaml MAE 0,0518 и ±2σ 66,7 %, на json 0,0489 и 75,6 %)."
             if meta.get("pkg_src_sha") == AUDIT_PKG_SHA else "."))
    unused = meta.get("node_params_unused") or []
    if unused:
        A(f">\n> **ВНИМАНИЕ: параметры ноды не дошли до Runner: {', '.join(unused)}.** Связка "
          "`tools/eval_replay.py` отстала от `tram_node.py` — числа могут не совпадать с тем, что "
          "выдаёт нода. Дописать `make_runner` и перезапустить.")
    A("")
    tm0, tn0 = _tot(S, "all", "model"), _tot(S, "all", "naive")
    A("## Главное")
    A("")
    A(f"* **Скорость** ({len(ids)} отложенных, против GNSS master): модель MAE "
      f"{f(tm0.get('v_mae'), 4)} м/с, смещение {f(tm0.get('v_bias'), 4, True)}, ±2σ "
      f"{pct(tm0.get('cov2s_v'))}; база «только колесо» MAE {f(tn0.get('v_mae'), 4)}, смещение "
      f"{f(tn0.get('v_bias'), 4, True)}.")
    A(f"* **Положение** (MGRS, после перевода выхода): модель 3D ср. {f(tm0.get('p3d_mean'))} м, "
      f"вдоль RMSE {f(tm0.get('along_rmse'))} м, дрейф 3D по концу медиана "
      f"{f(tm0.get('drift_pct_3d_median'), 3)} % (макс {f(tm0.get('drift_pct_3d_max'), 2)} %); база 3D "
      f"{f(tn0.get('p3d_mean'))} м.")
    A("* **Полнота выхода** (модель): " + completeness(tm0, S.get("crashes") or {}) + ".")
    zv = next((v for v in S.get("variants", []) if v["name"] == "zero" and v.get("totals")), None)
    if zv:
        A(f"* **Без заглушек крипа** (c_creep = c_creep_drag = 0): MAE {f(zv['totals'].get('v_mae'), 4)}, "
          f"смещение {f(zv['totals'].get('v_bias'), 4, True)}, 3D {f(zv['totals'].get('p3d_mean'))} м.")
    if (tm0.get("judge_raw_3d_mean") or 0) > 1000:
        A(f"* **Система судьи:** выход Runner сейчас не в MGRS судьи (код до правок — equirect от "
          f"начала): без перевода средняя 3D у судьи ≈ {f(tm0.get('judge_raw_3d_mean') / 1000, 0)} км.")
    bg = meta.get("boundary_grid") or M.BOUNDARY_GRID
    if "bx_wrap_grid_3d_mean" in tm0:
        A(f"* **Граница квадратов MGRS** (E = 400 км, запад в 37U CB): средняя 3D модели при сочетании "
          f"«наш выход × соглашение судьи» — перенос × перенос {f(tm0.get('bx_wrap_wrap_3d_mean'))} м, "
          f"{bg} × {bg} {f(tm0.get('bx_grid_grid_3d_mean'))} м, **перенос × {bg} "
          f"{f(tm0.get('bx_wrap_grid_3d_mean'), 0)} м, {bg} × перенос "
          f"{f(tm0.get('bx_grid_wrap_3d_mean'), 0)} м** (пар с ошибкой > 1 км: "
          f"{tm0.get('bx_wrap_grid_km', 0)} и {tm0.get('bx_grid_wrap_km', 0)} из {tm0.get('p_pairs', 0)}). "
          "Несовпадение соглашений стоит ~100 км всей западной части пути; при совпадении "
          "добавляются только пары у самой границы, где оценка и эталон по разные стороны "
          f"(перенос × перенос: {tm0.get('sq_mismatch', 0)}; раздел 3.2).")
    gf0 = S.get("gnss_full")
    if gf0:
        A(f"* **GNSS весь прогон:** выход совпал с режимом «GNSS 3 с» в {gf0.get('identical_runs')} из "
          f"{len(gf0.get('runs', {}))} прогонов; 3D ср. {f(g(gf0, 'full', 'p3d_mean'), 1)} м против "
          f"{f(g(gf0, 'gnss3', 'p3d_mean'), 1)} м (первые {f(gf0['span_s'] / 60, 0)} мин).")
    summ0 = inject_summary(result["inject"], result["kinds"])
    if summ0:
        bad = [f"{r['kind']} — {verdict(r)}" for r in summ0 if verdict(r) not in OK_VERDICTS]
        A("* **Инъекции:** " + ("; ".join(bad) if bad else "все виды в пределах нормы") + ".")
    A("")
    A("## 1. Как запустить")
    A("")
    A("Одна команда в Docker (PowerShell, из корня репозитория; кэш строится из `data/` сам):")
    A("")
    A("```powershell")
    A("docker run --rm --cpus 2 -v ${PWD}:/repo -v <каталог с bag>:/repo/data:ro -w /repo `")
    A("    vectra/tram:dev python3 tools/eval.py --label \"<подпись версии>\"")
    A("```")
    A("")
    A("Основные ключи (полный список — `python3 tools/eval.py --help`):")
    A("")
    A(table(["ключ", "по умолчанию", "что делает"], [
        ["`--sheet`", "`eval`", "лист: `eval` — `config/eval/tram_eval.yaml`, `tram.yaml` или "
         "`tram_calibration.json` (только train); если их нет — `config/tram_calibration.json` с пометкой "
         "об утечке; `jury` — боевой `config/tram.yaml` (не для отчёта); `json`; путь"],
        ["`--set k=v,...`", "—", "переопределить поля листа: поля `Params` — ядру, остальное — "
         "параметрам ноды (`--set mgrs_grid=37UDB`, `--set nomap_mode=line`)"],
        ["`--map`", "`train`", "`train` — карта только по обучающим (`analysis/build_map.py`, строится "
         "сама); `eval` — `config/eval/track_map.npz` пакета; `jury` — боевая; `none` — без карты; путь"],
        ["`--gnss`", "`3`", "секунд GNSS в связку от первой записи master; `full` — весь прогон"],
        ["`--frame`", "`mgrs`", "система эталона: `mgrs` (судья), `enu`, `equirect`, `utm`"],
        ["`--runner-frame`", "`auto`", "система выхода Runner: `auto` — параметр ноды `projection` "
         "(объявление в `tram_node.py`, поверх — лист), иначе `equirect` (код до правок), с проверкой "
         "по величине; `mgrs`, `utm`, `enu`, `equirect`"],
        ["`--runner-grid`", "параметр `mgrs_grid`", "как **читать** выход Runner в MGRS (`\"\"` — перенос "
         "по точке, `37UDB`). Runner не настраивает — для этого `--set mgrs_grid=37UDB`"],
        ["`--judge-grid`", "`\"\"`", "соглашение судьи на границе квадратов для «взгляда судьи»; "
         "код квадрата — и для матрицы соглашений (иначе `37UDB`)"],
        ["`--variants`", "выкл.", "варианты листа: заглушки крипа / нули (+15 прогонов модели; замер "
         "26.09, `--cpus 2`, машина почти без соседей: 6,4 мин без ключа, 8,5 мин с ним)"],
        ["`--gnss-full-runs`", "каждый 3-й", "прогоны проверки «GNSS весь прогон» (первые 5 мин): список "
         "или `all`"],
        ["`--quick`", "—", "CI: 2 прогона по 300 с, 4 инъекции, без вариантов и GNSS-full; документ и "
         "графики — в `<out>/EVAL.md`, `docs/` не трогает"],
        ["`--check-determinism`", "—", "второй проход и побайтное сравнение JSON (код выхода 2 при расхождении)"],
        ["`--render-only`", "—", "пересобрать документ и графики из `out/eval/*.json` и `plotdata.npz`"],
        ["`--cache`, `--data`, `--out`", "`analysis/cache`, `data`, `out/eval`", "каталоги"],
        ["`--no-inject`, `--no-gnss-full`, `--no-doc`", "—", "пропустить разделы"],
    ]))
    A("")
    A("Кэш и карта. Кэш прогонов (`analysis/cache/<bag>.npz`) строится из `data/` для нужных "
      "прогонов, если его нет. Карта оценки `track_map_train.<ключ>.npz` строится "
      "`analysis/build_map.py` только по `tools/split.json:train` (98 прогонов): после WP10 — "
      "`--set train --split tools/split.json --calib <лист оценки>` (множитель пути — с `meas_scale` "
      "оцениваемого листа), до правок — `build_map.py train`. Ключ — хэш `build_map.py`, "
      "`drive_model.json`, `split.json`, `track_map.py`, `runner.py`, `geodesy.py`, "
      "`tram_calibration.json` (после WP10 и листа), поэтому после правок карта пересобирается сама "
      "(в каталог кэша или `out/maps/`, если кэш только для чтения; нужен кэш 98 train, ~2 мин).")
    A("")
    A("После слияний потоков: `python3 tools/eval.py --label \"после слияний\"`. Лист `eval` "
      "подхватится из `config/eval/`; параметры ноды (`projection`, `mgrs_grid`, …) — из объявлений "
      "`tram_node.py` и листа. Проверки: (1) в шапке нет предупреждения о параметрах, не дошедших "
      "до Runner; (2) «взгляд судьи» в разделе 3.2 равен ошибке в MGRS при своём соглашении (нет "
      "~100 км); (3) раздел 5: выход с GNSS весь прогон совпадает с GNSS 3 с во всех прогонах.")
    A("")
    A(f"Результаты: `{args.out}/summary.json` (итоги), `runs.json` (по прогонам), `inject.json` "
      "(инъекции), `timing.json` (время, sha256 JSON, git, строки реального времени; не "
      "детерминирован). Тесты инструментов: `python3 -m pytest tools/eval_selftest.py -q`.")
    A("")
    wall = timing.get("wall_all_s")
    A(f"Этот прогон: {f(wall / 60 if wall else None, 1)} мин без графиков и документа (они ~0,5 мин), "
      f"процессов {timing.get('workers')}{', с --variants' if meta.get('variants_on') else ''} (время "
      "зависит от загрузки машины: 26.09 при соседних контейнерах, где наш получал ~1,3 ядра, тот же "
      "прогон без --variants шёл 18,0 мин вместо 6,4).")
    shas = timing.get("sha256") or {}
    if shas:
        A("sha256 JSON этого прогона: " + ", ".join(f"`{k}` {v[:12]}" for k, v in shas.items()) + ".")
    A("Воспроизводимость: два полных прогона дают побайтно одинаковые `summary.json`, `runs.json`, "
      "`inject.json` (`--check-determinism` проверяет это сам); чистый `git clone` с пустым кэшем "
      "строит кэш и карту из `data/` и даёт тот же sha256 `runs.json` (проверено 25.09 на коде до "
      "правок). Карта напарника `analysis/cache/track_map_train.npz` (собрана на Windows) отличается "
      "от пересобранной множителем пути в 16-м знаке, поэтому не используется.")
    det = timing.get("determinism")
    if det:
        A(f"Детерминизм (`--check-determinism`): JSON двух проходов "
          f"{'совпали побайтно' if det['identical'] else 'РАЗЛИЧАЮТСЯ: ' + str(det['files'])}.")
    A("")

    # ---------------- методика
    A("## 2. Методика")
    A("")
    A(f"* **Прогоны.** `tools/split.json:holdout_scored` — {len(ids)} чистых отложенных записей "
      "(без дублей и копий в обучении). Лист и карта для отчёта — только по `train`; боевые "
      "(по всем данным, уходят жюри) для отчёта не используются.")
    gl = meta.get("glue") or {}
    A("* **Связка.** Запись проигрывается в порядке записи в bag через `Runner` пакета так же, как "
      "в ноде (`tools/eval_replay.py`: объявления параметров `tram_node.py`, поверх — лист → `Params` "
      "+ параметры ноды → `Runner(...)`). GNSS в связку — только первые 3 с записи от первой точки "
      "master (как в проверочных bag); `--gnss full` — весь прогон. Статус NavSatFix в `on_fix`: "
      f"{'передаётся' if gl.get('fix_status') else 'нет (Runner его не принимает)'}; сортировка "
      "стартового всплеска (StartSorter, WP24): "
      + (f"да, окно {f(gl.get('start_sort_s'), 2)} с по времени записи" if gl.get("start_sort_s") is not None
         else "нет в этом коде") + ". Пульс ноды (WP16) не эмулируется: он публикует те же узлы "
      "сетки с теми же значениями, что связка выдаёт при следующем сообщении (прогноз на копии тем "
      "же кодом), кроме ≤ 2 с после последнего входа записи. Исключение в связке считается падением "
      "ноды: дальше выходов нет (нода после WP4 ловит исключения в колбэках и живёт, но Runner их "
      "бросать не должен — это дефект в любом случае).")
    A("* **Пары.** Выход ↔ эталон по ближайшей метке `header.stamp` в пределах 0,05 с (README, 5.1). "
      "Скорость публикуется на каждом шаге; положение — только при `pos_valid` (нода после WP10 не "
      "публикует `/result/position` без якоря GNSS или у края квадрата) и конечных x, y, z: фикс "
      "сопоставляется с ближайшим **опубликованным** положением, без него — непарный. Доли пар, NaN "
      "и падения — в «Главном» и разделе 3.")
    A("* **Эталон скорости.** Официального эталона нет: источников четыре (2 тележки, 2 GNSS). "
      "Основной — |v| GNSS master по (x, y); дополнительный — rover. Фаза: стоянка, если |v| GNSS "
      "< 0,2 м/с, иначе по ручке: > 0 тяга, < 0 торможение, 0 выбег. Ложная стоянка: режим "
      "STANDSTILL (у базы — v < 0,3 м/с) при |v| GNSS > 0,5 м/с.")
    A("* **Эталон положения — система судьи.** Организаторы (25.09): плоские координаты **MGRS**; "
      "по REP-103 x — восток, y — север, z — высота NavSatFix (абсолютная). Антенна эталона — "
      "master (имена антенн — «антенна 1/2», это наше допущение). Ошибки считаются в непрерывных "
      f"координатах UTM зоны {37} (зона начала выставки); это то же, что MGRS внутри одного квадрата.")
    A("* **Граница квадратов 100 км.** Линия пересекает E = 400 км: запад (~1,2 км) в **37U CB**, "
      "остальное в **37U DB**. Соглашение судьи на границе **неизвестно** до получения их карты: "
      "(а) «перенос по точке» — координаты внутри квадрата, где лежит точка (Autoware "
      "gnss_poser, lanelet2 MGRSProjector; x скачет на 100 км), или (б) непрерывно от одного "
      "квадрата (`37UDB`, запад — отрицательный x). Поэтому отдельно считается «несовпадение "
      "квадрата»: сколько пар при переносе по точке попали бы в другой квадрат, чем эталон (каждая "
      "такая пара у судьи — ошибка ~100 км); **матрица 2×2** «наше соглашение × соглашение судьи» "
      f"(перенос по точке / непрерывно от `{meta.get('boundary_grid') or M.BOUNDARY_GRID}`) по "
      "непрерывной оценке; и «взгляд судьи» — сырые x, y, z выхода Runner против эталона в "
      "соглашении `--judge-grid`.")
    A("* **Перевод выхода.** Если выход Runner не в MGRS (код до правок — equirect от своей точки "
      "начала), x, y, z переводятся обратно в широту/долготу через его же начало и формулу, затем в "
      "UTM. Так ошибка перевода равна нулю, а «взгляд судьи» показывает, что увидел бы судья без "
      "перевода.")
    A("* **Вдоль/поперёк пути.** Проекция на ломаную эталона (медиана по 5 фиксам, шаг 1 м, окно "
      "±1 км, то же направление движения; алгоритм `core_metrics.py`). Дрейф — ошибка **в конце "
      "прогона**, отнесённая к длине пути (PDF, стр. 6): 3D и вдоль пути.")
    A("* **Итоги.** Средние взвешены числом пар скорости прогона (как `critic_offline.py sub15`): "
      "для MAE, смещения и RMSE это пул всех пар; максимумы — по всем прогонам; дрейф — среднее, "
      "медиана и максимум по прогонам. Фазы — пулом всех пар.")
    A("* **База «только колесо»** — причинная: среднее последних показаний тележек (не старше "
      "1 с) × `meas_scale` / 3,6, путь — интеграл на той же сетке 50 мс, положение — **та же** "
      "машинерия `Runner`/`Position`/карты, та же выставка и привязка к остановкам. Неконечные "
      "показания база пропускает.")
    A("* **Что утекает.** " + (
        meta["sheet"]["leak"] + ". " if meta["sheet"].get("leak") else
        "Лист — оценочный (только train). ") + (
        meta["map"]["leak"] + ". " if meta["map"].get("leak") else
        "Карта — только по train. ") + (
        "Множитель пути карты (`calibrate_scale`) считается с `meas_scale` оцениваемого листа."
        if meta.get("build_map_cli") == "argparse" else
        "Множитель пути карты (`calibrate_scale`) считается с `meas_scale` из "
        "`tram_calibration.json` (все данные) — утечка порядка 0,01 %."))
    A("* **Инъекции** — в поток входов реальных записей; шум задан абсолютно (σ = 0,25 м/с), от "
      "листа не зависит. Связка с аномалией — копия чистой связки, снятая за 35 с до аномалии "
      "(начало потока то же; проверяется, иначе прогон с нуля).")
    A("")

    # ---------------- итоги
    tm, tn = _tot(S, "all", "model"), _tot(S, "all", "naive")
    A("## 3. Итог на отложенных")
    A("")
    A("### 3.1. Скорость")
    A("")
    rows = []
    for name, t in (("модель", tm), ("база «только колесо»", tn)):
        rows.append([name, f(t.get("v_rmse"), 4), f(t.get("v_mae"), 4), f(t.get("v_bias"), 4, True),
                     f(t.get("v_max"), 2), pct(t.get("cov2s_v")) if "cov2s_v" in t else "—",
                     pct(t.get("false_ss_rate"), 3), f(t.get("v_pairs"), 0) if t.get("v_pairs") else "—",
                     pct(t.get("v_pair_frac"), 2), t.get("v_out_nan", 0)])
    A(table(["оценка", "RMSE, м/с", "MAE, м/с", "смещение, м/с", "макс, м/с", "±2σ", "ложные стоянки",
             "пар", "доля пар", "NaN в выходе"], rows))
    A("")
    A("Доля пар — от меток GNSS master vel; непарные — метки без выхода в пределах 0,05 с (до "
      "первого выхода, пропуски, падение). Падения связки на чистых прогонах: " +
      (", ".join(f"{b} ({e}: {c.get('error', '')[:60]})" for b, cs in (S.get("crashes") or {}).items()
                 for e, c in cs.items() if c) or "нет") + ".")
    A("")
    rv = []
    for name, est in (("модель", "model"), ("база", "naive")):
        rr = [result["runs"][b][est] for b in ids if est in result["runs"].get(b, {})]
        rows_r = [dict(v_pairs=r.get("rover_v_pairs", 0), v_mae=r.get("rover_v_mae"),
                       v_rmse=r.get("rover_v_rmse"), v_bias=r.get("rover_v_bias"))
                  for r in rr if r.get("rover_v_pairs")]
        t = M.totals(rows_r) or {}
        rv.append([name, f(t.get("v_rmse"), 4), f(t.get("v_mae"), 4), f(t.get("v_bias"), 4, True)])
    A("Против GNSS **rover** (дополнительный эталон, итог взвешен парами rover):")
    A("")
    A(table(["оценка", "RMSE, м/с", "MAE, м/с", "смещение, м/с"], rv))
    A("")
    A("По фазам (пулом всех пар master):")
    A("")
    rows = []
    for p in M.PHASES:
        dm = g(S, "pooled", "model", "by_phase", p)
        dn = g(S, "pooled", "naive", "by_phase", p)
        if not dm:
            continue
        rows.append([M.PHASES_RU[p], pct(dm["share"], 0), f(dm.get("mae"), 4), f(dm.get("bias"), 4, True),
                     pct(dm.get("cov2s")), f(dn.get("mae") if dn else None, 4),
                     f(dn.get("bias") if dn else None, 4, True)])
    A(table(["фаза", "доля", "модель MAE", "модель смещение", "модель ±2σ", "база MAE",
             "база смещение"], rows))
    A("")
    rc = g(S, "pooled", "model", "ref_clean") or {}
    rcn = g(S, "pooled", "naive", "ref_clean") or {}
    A(f"Без выбросов самого эталона (GNSS против обеих согласных тележек > 1 м/с, "
      f"{rc.get('glitch_samples', 0)} пар): модель MAE {f(rc.get('mae'), 4)}, RMSE "
      f"{f(rc.get('rmse'), 4)}; база MAE {f(rcn.get('mae'), 4)}, RMSE {f(rcn.get('rmse'), 4)}.")
    A("")
    if pics.get("speed"):
        A(f"![Ошибка скорости](img/{pics['speed']})")
        A("")
    if pics.get("phase"):
        A(f"![Смещение и MAE по фазам](img/{pics['phase']})")
        A("")
    A("### 3.2. Положение (MGRS; ошибки в непрерывных координатах)")
    A("")
    rows = []
    for name, t in (("модель", tm), ("база «только колесо»", tn)):
        rows.append([name, f(t.get("p3d_mean")), f(t.get("p3d_rmse")), f(t.get("p3d_max"), 1),
                     f(t.get("along_mean")), f(t.get("along_rmse")), f(t.get("along_max"), 1),
                     f(t.get("cross_mean")), f(t.get("cross_max"), 1),
                     f(t.get("drift_pct_3d_median"), 3) + " / " + f(t.get("drift_pct_3d_max"), 3),
                     f(t.get("drift_pct_along_median"), 3) + " / " + f(t.get("drift_pct_along_max"), 3)])
    A(table(["оценка", "3D ср.", "3D RMSE", "3D макс", "вдоль ср. абс.", "вдоль RMSE", "вдоль макс",
             "поперёк ср.", "поперёк макс", "дрейф 3D, % мед./макс", "дрейф вдоль, % мед./макс"], rows))
    A("")
    A(f"Метры, кроме дрейфа. Высота: средняя |Δz| модели {f(tm.get('pz_mean'))} м, в плане "
      f"(2D) {f(tm.get('p2d_mean'))} м. Покрытие |вдоль| ≤ 2σ_s: {pct(tm.get('cov2s_along'))}. "
      f"Пар без проекции (оценка дальше 60 м от пути): {tm.get('along_undef', 0)}.")
    A("")
    rows = []
    for name, est, t in (("модель", "model", tm), ("база «только колесо»", "naive", tn)):
        rows.append([name, t.get("p_ref", "—"), t.get("p_pairs", "—"), pct(t.get("p_pair_frac"), 2),
                     t.get("p_out", "—"), t.get("p_out_invalid", 0), t.get("p_nan", 0),
                     f"{f(run_mean(result, ids, est, 'rate_hz'), 2)} / "
                     f"{f(run_mean(result, ids, est, 'rate_pos_hz'), 2)}"])
    A("Полнота положения (фикс ↔ опубликованное положение в пределах 0,05 с):")
    A("")
    A(table(["оценка", "меток fix", "пар", "доля пар", "строк выхода", "без положения (pos_valid)",
             "положение NaN", "частота выхода / положения, Гц (ср. по прогонам)"], rows))
    A("")
    bg = meta.get("boundary_grid") or M.BOUNDARY_GRID
    A(f"**Квадраты MGRS.** Пар, где оценка при «переносе по точке» попала бы в другой квадрат "
      f"100 км, чем эталон: модель {tm.get('sq_mismatch', 0)} из {tm.get('p_pairs', 0)}, база "
      f"{tn.get('sq_mismatch', 0)}. Соглашение судьи на границе неизвестно, поэтому — матрица "
      "«наш выход × соглашение судьи» (средняя 3D модели, м; в скобках — пар с ошибкой > 1 км):")
    A("")
    lab = {"wrap": "перенос по точке", "grid": f"непрерывно от {bg}"}
    rows = []
    for o in ("wrap", "grid"):
        rows.append([f"наш выход: {lab[o]}"] + [
            f"{f(tm.get(f'bx_{o}_{j}_3d_mean'), 1 if (tm.get(f'bx_{o}_{j}_3d_mean') or 0) < 1000 else 0)} "
            f"({tm.get(f'bx_{o}_{j}_km', 0)})" for j in ("wrap", "grid")])
    A(table(["", f"судья: {lab['wrap']}", f"судья: {lab['grid']}"], rows))
    A("")
    A("Совпали соглашения — ошибка почти непрерывная (при переносе по точке добавляются только пары "
      "у самой границы, где оценка и эталон по разные стороны E = 400 км); не совпали — ~100 км у "
      "всей западной части пути (37U CB, ~1,2 км линии). Выбор соглашения выхода — параметр ноды "
      "`mgrs_grid` (`\"\"` / `37UDB`); до ответа организаторов это главный риск по положению.")
    A("")
    A("«Взгляд судьи» (сырые x, y, z выхода Runner против эталона MGRS "
      f"{'с переносом по точке' if not meta['judge_grid'] else 'от ' + meta['judge_grid']}): "
      f"средняя 3D **{f(tm.get('judge_raw_3d_mean'), 1)} м**, максимум {f(tm.get('judge_raw_3d_max'), 1)} м, "
      f"пар с ошибкой > 1 км: {tm.get('judge_raw_km', 0)}"
      + (" — выход Runner не в MGRS судьи (код до правок — equirect от начала): без перевода судья "
         "увидел бы ошибку ~100 км. Это закрывает поток «положение» (выход MGRS)."
         if (tm.get("judge_raw_3d_mean") or 0) > 1000 else "."))
    A("")
    A("Чувствительность к системе эталона (тот же выход, другой эталон; 3D ср. / конец ср.):")
    A("")
    rows = [["MGRS (UTM)", f(tm.get("p3d_mean")) + " / " + f(tm.get("p3d_end_mean")),
             f(tn.get("p3d_mean")) + " / " + f(tn.get("p3d_end_mean"))]]
    for fr, lab in (("enu", "строгий ENU"), ("equirect", "equirect напарника")):
        a_, b_ = g(S, "frame_sensitivity", fr) or {}, g(S, "frame_sensitivity", fr + "_naive") or {}
        if a_:
            rows.append([lab, f(a_.get("p3d_mean")) + " / " + f(a_.get("p3d_end_mean")),
                         f(b_.get("p3d_mean")) + " / " + f(b_.get("p3d_end_mean"))])
    A(table(["эталон", "модель, м", "база, м"], rows))
    A("")
    if pics.get("pos"):
        A(f"![Ошибка положения по пути](img/{pics['pos']})")
        A("")
    A("### 3.3. По вагонам")
    A("")
    rows = []
    for grp in ("30618", "30639"):
        for name, est in (("модель", "model"), ("база", "naive")):
            t = _tot(S, grp, est)
            if not t:
                continue
            rows.append([grp, name, t.get("runs"), f(t.get("v_mae"), 4), f(t.get("v_bias"), 4, True),
                         f(t.get("p3d_mean")), f(t.get("along_rmse")), f(t.get("drift_pct_3d_median"), 3)])
    A(table(["вагон", "оценка", "прогонов", "MAE, м/с", "смещение", "3D ср., м", "вдоль RMSE, м",
             "дрейф 3D, % мед."], rows))
    A("")

    # ---------------- по прогонам
    A("### 3.4. По прогонам")
    A("")
    rows = []
    for b in ids:
        rm, rn = result["runs"][b].get("model", {}), result["runs"][b].get("naive", {})
        rows.append([b, f(rm.get("path_m", 0) / 1000.0, 2), rm.get("v_pairs", "—"), f(rm.get("v_mae"), 4),
                     f(rn.get("v_mae"), 4), f(rm.get("v_bias"), 4, True), pct(rm.get("cov2s_v"), 0),
                     f(rm.get("p3d_mean")), f(rn.get("p3d_mean")), f(rm.get("p3d_end")),
                     f(rm.get("along_end"), 1, True), f(rm.get("drift_pct_3d"), 3),
                     rm.get("sq_mismatch", "—"), "упала" if rm.get("crash") else ""])
    A(table(["прогон", "путь, км", "пар v", "MAE", "MAE база", "смещение", "±2σ", "3D ср.",
             "3D база", "3D конец", "вдоль конец", "дрейф 3D, %", "квадрат ≠", "падение"], rows))
    A("")

    # ---------------- варианты листа
    A("## 4. Заглушки крипа против подогнанных / нулевых")
    A("")
    rows = [["лист как есть", f"c_creep {f(meta['params']['c_creep'], 4)}, c_creep_drag "
             f"{f(meta['params']['c_creep_drag'], 4)}", f(tm.get("v_mae"), 4), f(tm.get("v_bias"), 4, True),
             pct(tm.get("cov2s_v")), f(tm.get("p3d_mean"))]]
    for v in S.get("variants", []):
        if v.get("same_as_base"):
            rows.append([v["name"], v["overrides"], "= лист", "", "", ""])
            continue
        t = v.get("totals") or {}
        rows.append([v["name"], v["overrides"], f(t.get("v_mae"), 4), f(t.get("v_bias"), 4, True),
                     pct(t.get("cov2s_v")), f(t.get("p3d_mean"))])
    if len(rows) == 1:
        rows.append(["—", "не считалось (включается ключом --variants; в --quick не считается)",
                     "", "", "", ""])
    A(table(["вариант", "поля", "MAE, м/с", "смещение", "±2σ", "3D ср., м"], rows))
    A("")

    # ---------------- GNSS весь прогон
    A("## 5. GNSS весь прогон (bag жюри с полным GNSS)")
    A("")
    gf = S.get("gnss_full")
    if gf:
        a_, b_ = gf.get("full") or {}, gf.get("gnss3") or {}
        n = len(gf.get("runs", {}))
        A(f"Первые {f(gf['span_s'] / 60, 0)} мин записи {n} прогонов "
          f"({', '.join(sorted(gf.get('runs', {})))}; `--gnss-full-runs all` — все), только модель "
          "(база идёт через тот же Runner). README разрешает GNSS только для начальной выставки, "
          "значит выход с GNSS весь прогон должен совпасть с выходом при GNSS 3 с: те же метки сетки, "
          "те же скорость, положение и признак публикации положения.")
        A("")
        A(table(["GNSS в связку", "MAE, м/с", "3D ср., м"], [
            ["весь прогон", f(a_.get("v_mae"), 4), f(a_.get("p3d_mean"), 1)],
            ["первые 3 с", f(b_.get("v_mae"), 4), f(b_.get("p3d_mean"), 1)]]))
        A("")
        A(f"Совпали (та же сетка и pos_valid, |Δv| ≤ 1e-9, |Δxyz| ≤ 1e-6): **{gf.get('identical_runs')} из {n}** "
          f"прогонов; наибольшее |Δv| {f(gf.get('max_dv'), 3)} м/с, наибольшее |Δ положения| "
          f"{f(gf.get('max_dpos'), 1)} м. Если не совпали — GNSS после окна влияет на выход "
          "(дефект C2, WP1).")
        A("")
        rows = [[b, "да" if r["same_grid"] else f"нет ({r['n_out_full']} / {r['n_out_3s']})",
                 f(r.get("max_dv"), 3), f(r.get("max_dpos"), 1), f(r.get("p3d_mean_full"), 1),
                 f(r.get("p3d_mean_3s"), 1)] for b, r in gf.get("runs", {}).items()]
        A(table(["прогон", "та же сетка", "макс |Δv|, м/с", "макс |Δ положения|, м", "3D ср. (весь)",
                 "3D ср. (3 с)"], rows))
    else:
        A("Не считалось (`--gnss full` уже основной режим, `--no-gnss-full` или `--quick`).")
    A("")

    # ---------------- инъекции
    A("## 6. Инъекции аномалий")
    A("")
    inj = [x for x in result["inject"] if not x.get("skipped")]
    bags = sorted({x["bag"] for x in result["inject"]})
    A(f"Модуль `tools/inject.py` меняет входной поток реальной отложенной записи ({', '.join(bags)}); "
      "GNSS не трогается. Окно — первое после 60 с от начала, где выполнено условие вида (скорость, "
      "ручка), и до конца остаётся ≥ 120 с. Окна метрик: «до» — 30 с перед аномалией, «во время» — "
      "окно оценки вида, «после» — 60 с после него. Восстановление — через сколько секунд после "
      f"конца аномалии скорость совпадает с чистым прогоном (|Δv| ≤ {f(0.1, 1)} м/с не меньше 3 с). "
      "Прогон с инъекцией идёт до конца окна + 300 с; «Δ вдоль через 300 с» — остаточная "
      "ошибка вдоль пути в этот момент минус ошибка чистого прогона (снимает ли её привязка к "
      "остановке).")
    A("")
    A(table(["вид", "описание", "длит., с"],
            [[k, I.KINDS[k]["ru"], f(I.KINDS[k]["dur"], 1)] for k in result["kinds"]]))
    A("")
    summ = inject_summary(result["inject"], result["kinds"])
    if summ:
        A("**Сводка по видам** (среднее MAE во время аномалии по прогонам; наибольший |Δ вдоль| через "
          "300 с; флаги модели во время: наименьшая доля valid, наибольшие доли ambiguous и slip; "
          "±2σ — доля пар, где ошибка внутри ±2σ):")
        A("")
        rows = []
        for r in summ:
            m, n, fl = r["model"], r["naive"], r["flag"]
            rows.append([r["kind"], r["n"], f(m["mae"], 3), f(n["mae"], 3), f(m["tail"], 1), f(n["tail"], 1),
                         f"{pct(fl['valid'], 0)} / {pct(fl['amb'], 0)} / {pct(fl['slip'], 0)}",
                         pct(fl["cov2s"], 0), verdict(r)])
        A(table(["вид", "прогонов", "MAE модели", "MAE базы", "|Δ вдоль| модели, м", "|Δ вдоль| базы, м",
                 "valid / amb / slip", "±2σ", "вывод"], rows))
        A("")
        A("Подробно по прогонам:")
        A("")
    rows = []
    for x in inj:
        em, en = x["est"].get("model", {}), x["est"].get("naive", {})
        for name, e in (("модель", em), ("база", en)):
            if not e:
                continue
            crash = e.get("crash")
            dur = e.get("during", {})
            flags = ""
            if name == "модель" and dur.get("frac_valid") is not None:
                flags = (f"valid {pct(dur.get('frac_valid'), 0)}, amb {pct(dur.get('frac_amb'), 0)}, "
                         f"slip {pct(dur.get('frac_slip'), 0)}, σv≤{f(dur.get('sigma_v_max'), 1)}")
            rows.append([
                f"{x['kind']} · {x['bag'][-8:]}" if name == "модель" else "", name,
                f(g(e, "before", "v_mae"), 3), f(g(e, "during", "v_mae"), 3), f(g(e, "after", "v_mae"), 3),
                f(g(e, "during", "along_end"), 1, True) + " (" + f(g(e, "during", "along_end_clean"), 1, True) + ")",
                f(g(e, "after", "along_end"), 1, True),
                f(e.get("d_along_tail"), 1, True),
                f(e.get("recovery_s"), 1) if e.get("recovery_s") is not None else ("—" if crash else "нет"),
                ("**упала** " + f(crash.get("after_t0_s"), 2) + " с: " + crash["error"][:60]) if crash else flags,
                pct(dur.get("cov2s"), 0) if name == "модель" and dur.get("cov2s") is not None else "",
            ])
    A(table(["вид · прогон", "оценка", "MAE до", "MAE во время", "MAE после",
             "вдоль, конец окна (чисто)", "вдоль, после", "Δ вдоль через 300 с, м", "восст., с",
             "флаги / падение", "±2σ во время"], rows))
    for x in result["inject"]:
        if x.get("skipped"):
            A(f"\n* {x['kind']} · {x['bag']}: {x['skipped']}")
    A("")
    for p in pics.get("inj", []):
        A(f"![Инъекции](img/{p})")
        A("")

    # ---------------- реальное время
    A("## 7. Реальное время")
    A("")
    # строки, собранные при прогоне (timing.json), — иначе --render-only не воспроизвёл бы
    # документ, когда файлов сводок уже нет; старый timing.json без них — из файлов
    pr = timing["realtime"] if timing.get("realtime") is not None else probe_rows(root, args.probe_glob)
    if pr:
        A("Сводки `tools/ros_probe.py` (найдены по `--probe-glob`): частота по меткам, задержка "
          "in2out — от приёма входа `/vehicle/*` до приёма первого выхода, который его учёл "
          "(стенные часы пробы, включая DDS), CPU — % одного ядра, RSS — МБ.")
        A("")
        A(table(["замер", "выходов", "Гц", "in2out p50, мс", "p99", "макс", "CPU ср., %", "CPU макс",
                 "RSS, МБ", "рост RSS, МБ/мин"], pr))
        if any(str(r[0]).startswith("audit25_") for r in pr):
            A("")
            A("Строки `audit25_*` — замеры аудита ROS2_E2E 25.09 на коде до правок, bag ~4,5 мин, "
              "соседние контейнеры: предварительно. Отчётный замер (полный bag ≥ 20 мин, "
              "`--cpus 2 --memory 512m`, машина без соседей) делает поток infra (WP8).")
    else:
        A("**Заглушка.** Сводок `tools/ros_probe.py` не найдено (`--probe-glob "
          f"{args.probe_glob}`). Замер делает поток infra (`tools/measure_realtime.sh`, WP8): "
          "полный bag ≥ 20 мин, `--cpus 2 --memory 512m`, машина без соседей. После замера "
          "перезапустить `tools/eval.py` — таблица подставится сюда.")
    A("")
    A(f"Офлайн-скорость связки в этом прогоне (не замер ноды): {timing.get('events_per_s', '—')} "
      "входных сообщений в секунду на процесс вместе с базой — предварительно, при соседних "
      "контейнерах.")
    A("")

    # ---------------- сверка
    A("## 8. Сверка с аудитом (critic_offline.py sub15)")
    A("")
    same_code = meta.get("pkg_src_sha") == AUDIT_PKG_SHA
    same_setup = (same_code and meta["sheet"]["path"].endswith("config/tram_calibration.json")
                  and not meta["overrides"] and meta["map"]["label"].startswith("оценочная")
                  and meta["gnss"] == "3" and not meta["quick"] and len(ids) == 15)
    eq = g(S, "frame_sensitivity", "equirect") or {}
    eqn = g(S, "frame_sensitivity", "equirect_naive") or {}
    rows = [
        ["MAE модели, м/с", f(SUB15_JSON["v_mae"], 4), f(tm.get("v_mae"), 4)],
        ["смещение модели, м/с", f(SUB15_JSON["v_bias"], 4, True), f(tm.get("v_bias"), 4, True)],
        ["±2σ модели", f(SUB15_JSON["cov2s_v"], 3), f(tm.get("cov2s_v"), 3)],
        ["3D ср. модели (equirect), м", f(SUB15_JSON["p3d_mean"]), f(eq.get("p3d_mean"))],
        ["MAE базы, м/с", f(SUB15_JSON["naive_v_mae"], 4), f(tn.get("v_mae"), 4)],
        ["3D ср. базы (equirect), м", f(SUB15_JSON["naive_p3d_mean"]), f(eqn.get("p3d_mean"))],
    ]
    A(("Настройка совпадает с аудитом (код пакета тот же — sha `" + AUDIT_PKG_SHA + "`, json-лист, "
       "train-карта, GNSS 3 с, 15 прогонов, эталон equirect для 3D): числа должны совпасть до "
       "округления." if same_setup else
       "**Настройка отличается от аудита** (" + ("код пакета другой: sha `" + str(meta.get("pkg_src_sha"))
                                                 + "` против `" + AUDIT_PKG_SHA + "` у аудита; "
                                                 if not same_code else "") +
       "лист, карта или набор прогонов) — расхождение ожидаемо, таблица для справки."))
    A("")
    A(table(["метрика", "sub15 (аудит 25.09, json_train)", "этот прогон"], rows))
    A("")
    A("Откуда расхождения, если они есть: (1) аудит считал 3D, путь и вдоль/поперёк в equirect "
      "напарника, а здесь основная система — MGRS/UTM: длина пути в UTM на ~0,2 % больше "
      "(equirect короче по востоку на 0,23 %), средняя 3D по прогону меняется до ~0,2 %; для "
      "сверки выше взята та же equirect; (2) аудит хранил ряды в float32, поэтому доля «±2σ» на "
      "границе (стоянка, σ_v на полу) может отличаться на ~0,1 п. п.; (3) sub15 печатает 3 "
      "значащие цифры; (4) «вдоль, конец» там, где оценка в конце прогона на соседнем пути "
      "(30618_0652866c: 3D 131 м, поперёк ~55–60 м), зависит от выбора основания перпендикуляра "
      "среди почти равноудалённых (правило «ближайший ± 5 м»): последний фикс с проекцией тот же, "
      "но в equirect −35,13 м (как в аудите), в MGRS −34,08 м — UTM против equirect отличается "
      "масштабом (0,23 %) и поворотом на сближение меридианов (~1,3°). Скорость по прогонам "
      "совпадает с `out/core/runs_val_json_train.csv` аудита до 5-й значащей цифры (проверено 25.09 "
      "на коде до правок).")
    A("")

    # ---------------- пороги
    A("## 9. Пороги приёмок, пересчитанные на 15 чистых (WP7 (г))")
    A("")
    zero = next((v for v in S.get("variants", []) if v["name"] == "zero"), None)
    no_creep = not meta["params"].get("c_creep") and not meta["params"].get("c_creep_drag")
    zt = (zero or {}).get("totals") or (tm if no_creep or (zero or {}).get("same_as_base") else {})
    rn = result["runs"]
    ends = {b: g(rn, b, "model", "p3d_end") for b in ("30618_0652866c", "30618_49fe4c54", "30618_8158f0b0")}
    others = [g(rn, b, "model", "p3d_end") for b in ids if b not in ends]
    r92 = g(rn, "30639_92226df0", "model", "p3d_mean")
    rows = [
        ["WP5 крип", "MAE ≤ 0,045; смещение ≤ +0,005; 3D ≤ 4,1 м (17 прогонов, equirect)",
         f"без крипа: MAE {f(zt.get('v_mae'), 4)}, смещение {f(zt.get('v_bias'), 4, True)}, "
         f"3D {f(zt.get('p3d_mean'))} м (MGRS)" if zt else "не считалось (нужен --variants)",
         wp5_threshold(zt)],
        ["WP11 конечные", "конец у 0652866c / 49fe4c54 / 8158f0b0 < 40 м (было 131 / 55 / 54)",
         " / ".join(f(v, 1) for v in ends.values()) + " м (MGRS)",
         "< 40 м у трёх; остальные не хуже > 0,5 м (сейчас макс конца у остальных "
         f"{f(max([x for x in others if x is not None], default=None), 1)} м)"],
        ["WP13 масштаб колёс", "3D ≤ 3,9 м; 92226df0 ≤ 6 м; 30618 не хуже > 1 м",
         f"3D ср. {f(tm.get('p3d_mean'))} м; 92226df0 {f(r92)} м (MGRS)",
         (f"−15 % к 3D на момент приёмки (так выведен порог 3,9 из 4,65): сейчас "
          f"{f(math.floor(tm.get('p3d_mean', 0) * 0.85 * 10) / 10, 1)} м"
          + (f", после WP5 (без крипа, {f(zt.get('p3d_mean'))} м) — "
             f"{f(math.floor(zt['p3d_mean'] * 0.85 * 10) / 10, 1)} м" if zt and zt.get("p3d_mean") else "")
          + "; 92226df0 ≤ 6 м; ни один 30618 не хуже > 1 м") if tm.get("p3d_mean") else "—"],
    ]
    A(table(["пакет", "порог в TODO (на 17)", "база на 15 чистых", "предлагаемый порог"], rows))
    A("")
    A("Пороги предлагаются от чисел этого прогона; если лист или карта меняются, пересчитать."
      + (" **Здесь лист с утечкой (json по всем данным): на оценочном листе пороги пересчитать; "
         "порог смещения WP5 +0,005 не ослаблять.**" if meta["sheet"].get("leak") else ""))
    A("")
    A("## 10. Открытые вопросы")
    A("")
    A("* Соглашение судьи на границе квадратов 100 км (перенос по точке или непрерывно от "
      "квадрата) — ждём карту организаторов. Инструмент считает оба: `--judge-grid \"\"` / `37UDB`.")
    A("* Антенна эталона (master или rover) и tf между антеннами — обещаны позже; сейчас master.")
    A("* Эталон скорости: официального нет; показываем master (основной) и rover.")
    A("")
    return "\n".join(L) + "\n"
