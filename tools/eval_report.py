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
# итоги версии «до» для раздела 0 (закоммичены: out/ не в git)
BASELINE = "docs/data/eval_before/summary.json"
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


def gnss_label(meta):
    """Подача GNSS в связку и коррекция по GNSS (параметр ноды gnss_correction)."""
    gn = str(meta.get("gnss", "3"))
    try:
        base = f"первые {float(gn):g} с"
    except ValueError:
        base = "весь прогон" if gn == "full" else f"сценарий {gn} (tools/inject.py)"
    corr = (meta.get("node_params") or {}).get("gnss_correction")
    if corr is None:
        return base
    return base + ("; коррекция по GNSS после окна включена" if corr else
                   "; коррекция по GNSS выключена (только выставка)")


def gnss_corr(meta):
    return bool((meta.get("node_params") or {}).get("gnss_correction"))


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

def probe_files(root, patterns):
    """Сводки проб по шаблонам через запятую, без повторов, в порядке шаблонов."""
    seen = []
    for pat in patterns.split(","):
        for p in sorted(glob.glob(str(root / pat), recursive=True)):
            p = Path(p).resolve()
            if p not in seen:
                seen.append(p)
    return seen


def probe_rows(root, patterns):
    rows = []
    for p in probe_files(root, patterns):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        o = g(d, "outputs", "velocity") or {}
        L = d.get("latency", {}) or {}
        P = d.get("node_process", {}) or {}
        i2o = L.get("in2out_vehicle_ms") or {}
        cpu = P.get("cpu_pct_1core") or {}
        rss = P.get("rss_mb_first_last_max")
        rows.append([p.parent.name, o.get("count", "—"), f(o.get("rate_stamp_hz"), 1),
                     f(i2o.get("p50"), 1), f(i2o.get("p99"), 1), f(i2o.get("max"), 1),
                     f(cpu.get("mean"), 1), f(cpu.get("max"), 1),
                     f(rss[-1], 0) if isinstance(rss, list) and rss else "—",
                     f(P.get("rss_slope_mb_per_min_after_30s"), 2)])
    return rows


def probe_notes(root, patterns):
    """Примечания к строкам таблицы: note.txt рядом со сводкой пробы."""
    notes = []
    for p in probe_files(root, patterns):
        n = p.parent / "note.txt"
        if n.is_file():
            text = " ".join(n.read_text(encoding="utf-8").split())
            if text:
                notes.append([p.parent.name, text])
    return notes


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


def load_baseline(root, args):
    """Итоги прежней версии для таблицы «до / после» (--baseline; по
    умолчанию docs/data/eval_before/summary.json, если есть)."""
    spec = getattr(args, "baseline", None)
    if spec in ("", "none"):
        return None, None
    path = Path(spec) if spec else Path(root) / BASELINE
    if not path.is_absolute():
        path = Path(root) / path
    if not path.exists():
        return None, None
    import json
    B = json.loads(path.read_text(encoding="utf-8"))
    if getattr(args, "baseline_label", ""):
        B["meta"]["label"] = args.baseline_label
    try:
        rel = path.relative_to(root).as_posix()
    except ValueError:
        rel = str(path)
    return B, rel


def before_after(S, B, rel):
    """Таблица «до / после»: те же 15 прогонов, те же метрики."""
    L = []
    A = L.append
    mb, ma = _tot(B, "all", "model"), _tot(S, "all", "model")
    nb, na = _tot(B, "all", "naive"), _tot(S, "all", "naive")
    bm = B["meta"]
    A("## 0. До и после")
    A("")
    same = sorted(bm.get("runs", [])) == sorted(S["meta"]["runs"])
    A(f"«До» — `{rel}`: {bm.get('label') or 'без подписи'}, код пакета sha "
      f"`{bm.get('pkg_src_sha')}`, лист {bm['sheet']['label']}, карта {bm['map']['label']}"
      + (f"; **{bm['sheet']['leak']}**" if bm["sheet"].get("leak") else "")
      + ". «После» — этот прогон (шапка документа). Прогоны "
      + ("те же." if same else "**другие** — сравнение неполное."))
    A("")
    # эталон положения и соглашение судьи у «до» могут быть другими (у старых версий —
    # антенна master и перенос по точке) — тогда строки положения сравнивают разное
    rb, ra = bm.get("ref_point") or "master", S["meta"].get("ref_point") or "master"
    jb, ja = bm.get("judge_grid") or "", S["meta"].get("judge_grid") or ""
    if (rb, jb) != (ra, ja):
        def _ref(r, j):
            return (("base_link по tf антенн" if r == "base_link" else "антенна master")
                    + ", соглашение судьи " + (f"от {j} непрерывно" if j else "перенос по точке"))
        A(f"**Эталон положения разный:** «до» — {_ref(rb, jb)}; «после» — {_ref(ra, ja)}. "
          "Строки скорости сравнимы как есть; строки положения — нет (точка вагона и система "
          "другие): для сравнения положения в системе судьи см. docs/data/eval_before_frame "
          "(прежние умолчания против эталона судьи).")
        A("")
    rows = []

    def row(name, key, nd=4, sign=False, pc=False, src="tot"):
        def val(T):
            x = T.get(key)
            return pct(x) if pc else f(x, nd, sign)
        rows.append([name, val(mb), val(ma), val(nb), val(na)])

    row("скорость MAE, м/с", "v_mae")
    row("скорость RMSE, м/с", "v_rmse")
    row("скорость смещение, м/с", "v_bias", sign=True)
    row("скорость макс |ошибка|, м/с", "v_max", 3)
    row("±2σ скорости", "cov2s_v", pc=True)
    for p in M.PHASES:
        db, da = g(B, "pooled", "model", "by_phase", p), g(S, "pooled", "model", "by_phase", p)
        eb, ea = g(B, "pooled", "naive", "by_phase", p), g(S, "pooled", "naive", "by_phase", p)
        if not (db or da):
            continue
        for key, nm, nd, sg, pc in (("mae", "MAE", 4, False, False), ("bias", "смещение", 4, True, False),
                                    ("cov2s", "±2σ", 1, False, True)):
            def v(d):
                x = (d or {}).get(key)
                return pct(x) if pc else f(x, nd, sg)
            rows.append([f"— {M.PHASES_RU[p]}: {nm}", v(db), v(da),
                         "—" if pc else v(eb), "—" if pc else v(ea)])
    row("положение ср. 3D, м", "p3d_mean", 2)
    row("положение 3D RMSE, м", "p3d_rmse", 2)
    row("положение 3D макс, м", "p3d_max", 1)
    row("конец 3D ср., м", "p3d_end_mean", 1)
    row("конец 3D медиана, м", "p3d_end_median", 2)
    row("конец 3D макс, м", "p3d_end_max", 1)
    row("дрейф 3D по концу, % медиана", "drift_pct_3d_median", 3)
    row("дрейф 3D по концу, % ср.", "drift_pct_3d_mean", 3)
    row("дрейф 3D по концу, % макс", "drift_pct_3d_max", 3)
    row("вдоль пути ср. |ошибка|, м", "along_mean", 2)
    row("вдоль пути RMSE, м", "along_rmse", 2)
    row("вдоль пути макс, м", "along_max", 1)
    row("поперёк ср., м", "cross_mean", 2)
    row("поперёк макс, м", "cross_max", 1)
    row("|вдоль| ≤ 2σ_s", "cov2s_along", pc=True)
    row("пар в чужом 100-км квадрате (перенос)", "sq_mismatch", 0)
    row("«взгляд судьи»: сырые x, y, z против MGRS судьи, ср. 3D, м", "judge_raw_3d_mean", 2)
    row("шагов без положения после первого опубликованного", "p_gap_steps", 0)
    row("|z| ср., м", "pz_mean", 2)
    row("поперёк pathgraph ср., м", "pg_cross_mean", 3)
    row("вдоль pathgraph ср. |ошибка|, м", "pg_along_mean", 2)
    A(table(["метрика (15 holdout_scored)", "модель до", "модель после", "база до", "база после"], rows))
    A("")
    L += vehicle_before_after(S, B, "30618")
    gb, ga = B.get("gnss_full") or {}, S.get("gnss_full") or {}
    if gb or ga:
        A(f"GNSS весь прогон (первые 5 мин): выход совпал с «GNSS 3 с» до — "
          f"{gb.get('identical_runs', '—')} из {len(gb.get('runs', {}))}, после — "
          f"{ga.get('identical_runs', '—')} из {len(ga.get('runs', {}))}"
          + (f"; сетка и скорость совпали после — {ga.get('same_speed_runs', '—')} из "
             f"{len(ga.get('runs', {}))} (с коррекцией по GNSS положение и должно "
             "отличаться)" if gnss_corr(S.get("meta", {})) else "") + ".")
        A("")
    A("База «только колесо» — тот же Runner (выставка, карта, привязки) со средним свежих показаний "
      "тележек вместо ядра; «до» и «после» у неё различаются только кодом связки и картой.")
    A("")
    return L


VEH_KEYS = (("скорость MAE, м/с", "v_mae", 4, False, False),
            ("скорость RMSE, м/с", "v_rmse", 4, False, False),
            ("скорость смещение, м/с", "v_bias", 4, True, False),
            ("±2σ скорости", "cov2s_v", 1, False, True),
            ("положение ср. 3D, м", "p3d_mean", 2, False, False),
            ("положение 3D RMSE, м", "p3d_rmse", 2, False, False),
            ("положение 3D макс, м", "p3d_max", 1, False, False),
            ("конец 3D ср., м", "p3d_end_mean", 2, False, False),
            ("конец 3D медиана, м", "p3d_end_median", 2, False, False),
            ("конец 3D макс, м", "p3d_end_max", 1, False, False),
            ("дрейф 3D по концу, % медиана", "drift_pct_3d_median", 3, False, False),
            ("дрейф 3D по концу, % макс", "drift_pct_3d_max", 3, False, False),
            ("вдоль пути RMSE, м", "along_rmse", 2, False, False),
            ("поперёк ср., м", "cross_mean", 2, False, False),
            ("|вдоль| ≤ 2σ_s", "cov2s_along", 1, False, True),
            ("|z| ср., м", "pz_mean", 2, False, False),
            ("поперёк pathgraph ср., м", "pg_cross_mean", 3, False, False))


def vehicle_before_after(S, B, veh):
    """«До / после» по одному вагону (жюри проверяет только 30618)."""
    mb, ma = _tot(B, veh, "model"), _tot(S, veh, "model")
    if not (mb or ma):
        return []
    L = []
    A = L.append
    A(f"**Только вагон {veh}** ({ma.get('runs', mb.get('runs', '—'))} прогонов"
      + ("; проверка жюри — только на нём, организаторы 26.09" if veh == "30618" else "") + "):")
    A("")
    rows = []
    for name, key, nd, sg, pc in VEH_KEYS:
        if mb.get(key) is None and ma.get(key) is None:
            continue
        rows.append([name] + [pct(T.get(key)) if pc else f(T.get(key), nd, sg) for T in (mb, ma)])
    A(table(["метрика", "модель до", "модель после"], rows))
    A("")
    return L


GNSS_SC_ORDER = ("first3", "sparse", "bursts", "nostart", "midstart", "full", "glitchy")


def load_gnss_scenarios(root, spec):
    """Итоги tools/eval_gnss.py (summary.json или каталог с ним) -> (dict, путь) | (None, None)."""
    if not spec or spec == "none":
        return None, None
    path = Path(spec)
    if not path.is_absolute():
        path = Path(root) / path
    if path.is_dir():
        path = path / "summary.json"
    if not path.exists():
        return None, None
    import json
    try:
        rel = path.relative_to(root).as_posix()
    except ValueError:
        rel = str(path)
    return json.loads(path.read_text(encoding="utf-8")), rel


def gnss_scenarios(root, args):
    """Раздел 5.1: сценарии доступности GNSS (tools/eval_gnss.py): «main» —
    версия без коррекции по GNSS, «выкл.» / «вкл.» — этот код с
    gnss_correction false / true. -> (строки, (итоги, итоги main) | None)."""
    Sa, ra = load_gnss_scenarios(root, getattr(args, "gnss_scenarios", ""))
    if Sa is None:
        return [], None
    Sb, rb = load_gnss_scenarios(root, getattr(args, "gnss_scenarios_before", ""))
    L = []
    A = L.append
    ma = Sa["meta"]
    scen = ([s for s in GNSS_SC_ORDER if s in ma["scenarios"]]
            + [s for s in ma["scenarios"] if s not in GNSS_SC_ORDER])
    A("### 5.1. Сценарии доступности GNSS (`tools/eval_gnss.py`)")
    A("")
    A(f"Те же {len(ma['runs'])} отложенных, лист и карта ОЦЕНКИ; GNSS подаётся по сценарию "
      "(`tools/inject.py`, зерно — от прогона); эталон — base_link по всем точкам GNSS прогона. "
      f"«вкл.» — этот код по умолчанию (`gnss_correction: true`; `{ra}`, код пакета sha "
      f"`{ma.get('pkg_src_sha')}`), «выкл.» — он же с `gnss_correction: false` (GNSS только для "
      "выставки)"
      + (f", «main» — версия до коррекции по GNSS (`{rb}`, sha `{Sb['meta'].get('pkg_src_sha')}`; "
         "коррекции там нет)" if Sb is not None else "") + ".")
    A("")
    ru = ma.get("scenario_ru") or {}
    A(table(["сценарий", "что подаётся"], [[f"`{s}`", ru.get(s, s)] for s in scen]))
    A("")
    for grp, title in (("30618", "Вагон 30618 (проверка жюри — только он)"), ("all", "Все 15")):
        T = {s: g(Sa, "totals", s, grp) or {} for s in scen}
        if not any(T.values()):
            continue
        Tb = {s: (g(Sb, "totals", s, grp) or {}) if Sb is not None else {} for s in scen}
        n = next((x.get("after", {}).get("runs") for x in T.values() if x.get("after")), "—")
        A(f"**{title}** ({n} прогонов):")
        A("")
        rows = []
        for s in scen:
            b0 = Tb[s].get("before") or {}
            b1 = T[s].get("before") or {}
            a1 = T[s].get("after") or {}
            rows.append([f"`{s}`",
                         f(b0.get("p3d_mean"), 2), f(b1.get("p3d_mean"), 2), f(a1.get("p3d_mean"), 2),
                         f(b0.get("p3d_end_mean"), 2), f(b1.get("p3d_end_mean"), 2),
                         f(a1.get("p3d_end_mean"), 2), f(a1.get("p3d_end_max"), 1),
                         f(b0.get("v_mae"), 4), f(a1.get("v_mae"), 4),
                         str(a1.get("n_corr", "—"))])
        A(table(["сценарий", "3D ср. main", "3D ср. выкл.", "3D ср. вкл.", "конец ср. main",
                 "конец ср. выкл.", "конец ср. вкл.", "конец макс вкл.", "MAE скорости main",
                 "MAE скорости вкл.", "поправок GNSS"], rows))
        A("")
    # скорость: в сценариях с теми же первыми 3 с GNSS она та же, что в first3
    same = []
    for s in ("sparse", "bursts", "full", "glitchy"):
        if s in scen and "first3" in scen:
            a = g(Sa, "totals", s, "all", "after", "v_mae")
            b = g(Sa, "totals", "first3", "all", "after", "v_mae")
            same.append((s, a is not None and a == b))
    if same:
        A("Скорость: в сценариях с теми же первыми 3 с GNSS (" + ", ".join(f"`{s}`" for s, _ in same)
          + ") MAE «вкл.» " + ("равна MAE `first3` во всех" if all(v for _, v in same) else
                               "**отличается** от `first3` в " + ", ".join(s for s, v in same if not v))
          + ": GNSS после окна двигает только положение. В `nostart` и `midstart` выставка другая "
          "(позже или с середины записи), поэтому другой и онлайн-масштаб колёс по остановкам: он "
          "начинается с выставки.")
        A("")
    return L, (Sa, Sb)


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
      f"GNSS в связку: {gnss_label(meta)}. "
      f"Прогонов: {len(ids)}{' (--quick, первые 300 с)' if meta['quick'] else ''}.")
    for leak in (meta["sheet"].get("leak"), meta["map"].get("leak")):
        if leak:
            A(f">\n> **{leak}.** Эти числа не отчётные, пока нет оценочного листа.")
    if meta["sheet"].get("kind") == "json":
        A(">\n> Лист json — не то, что читает нода: боевая нода берёт `config/tram.yaml`.")
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
    rp = meta.get("ref_point") or "master"
    A(f"* **Положение** (MGRS от {meta.get('judge_grid') or 'квадрата точки'}, эталон — "
      f"{'base_link по tf антенн' if rp == 'base_link' else 'антенна master'}): модель 3D ср. "
      f"{f(tm0.get('p3d_mean'))} м, конец ср. {f(tm0.get('p3d_end_mean'), 1)} м, "
      f"вдоль RMSE {f(tm0.get('along_rmse'))} м, дрейф 3D по концу медиана "
      f"{f(tm0.get('drift_pct_3d_median'), 3)} % (макс {f(tm0.get('drift_pct_3d_max'), 2)} %); база 3D "
      f"{f(tn0.get('p3d_mean'))} м."
      + (f" Поперёк pathgraph организаторов ср. {f(tm0.get('pg_cross_mean'))} м (95 % — "
         f"{f(tm0.get('pg_cross_p95'))} м), вдоль pathgraph ср. |ошибка| "
         f"{f(tm0.get('pg_along_mean'))} м." if tm0.get("pg_cross_mean") is not None else ""))
    A("* **Полнота выхода** (модель): " + completeness(tm0, S.get("crashes") or {}) + ".")
    zv = next((v for v in S.get("variants", []) if v["name"] == "zero" and v.get("totals")), None)
    if zv:
        A(f"* **Без заглушек крипа** (c_creep = c_creep_drag = 0): MAE {f(zv['totals'].get('v_mae'), 4)}, "
          f"смещение {f(zv['totals'].get('v_bias'), 4, True)}, 3D {f(zv['totals'].get('p3d_mean'))} м.")
    if (tm0.get("judge_raw_3d_mean") or 0) > 1000:
        A(f"* **Система судьи:** выход Runner сейчас не в MGRS судьи (старая версия пакета — "
          f"equirect от начала): без перевода средняя 3D у судьи ≈ {f(tm0.get('judge_raw_3d_mean') / 1000, 0)} км.")
    bg = meta.get("boundary_grid") or M.BOUNDARY_GRID
    if meta.get("judge_grid"):
        A(f"* **Система судьи** (pathgraph организаторов): MGRS от угла {meta['judge_grid']} "
          f"непрерывно, точка {rp}. Сырые x, y, z выхода против эталона в этой системе: средняя 3D "
          f"{f(tm0.get('judge_raw_3d_mean'), 2)} м, пар с ошибкой > 1 км: {tm0.get('judge_raw_km', 0)}; "
          f"шагов без опубликованного положения после первого опубликованного: "
          f"{tm0.get('p_gap_steps', '—')}."
          + (f" Эталон base_link лежит на pathgraph: медиана |поперёк| "
             f"{f(tm0.get('pg_ref_lat_med'), 2)} м, высота эталона − pathgraph медиана "
             f"{f(tm0.get('pg_ref_dz_med'), 2, True)} м (средние по прогонам)."
             if tm0.get("pg_ref_lat_med") is not None else ""))
    elif "bx_wrap_grid_3d_mean" in tm0:
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
        if gnss_corr(meta):
            A(f"* **GNSS весь прогон (коррекция по GNSS):** сетка и скорость совпали с режимом "
              f"«GNSS 3 с» в {gf0.get('same_speed_runs')} из {len(gf0.get('runs', {}))} прогонов; "
              f"3D ср. {f(g(gf0, 'full', 'p3d_mean'), 2)} м против "
              f"{f(g(gf0, 'gnss3', 'p3d_mean'), 2)} м (первые {f(gf0['span_s'] / 60, 0)} мин).")
        else:
            A(f"* **GNSS весь прогон:** выход совпал с режимом «GNSS 3 с» в {gf0.get('identical_runs')} из "
              f"{len(gf0.get('runs', {}))} прогонов; 3D ср. {f(g(gf0, 'full', 'p3d_mean'), 1)} м против "
              f"{f(g(gf0, 'gnss3', 'p3d_mean'), 1)} м (первые {f(gf0['span_s'] / 60, 0)} мин).")
    gsd = gnss_scenarios(root, args)[1]
    if gsd is not None:
        ga = {s: g(gsd[0], "totals", s, "30618") or {} for s in ("first3", "sparse", "full")}
        if all(ga.values()):
            def _p(s, arm):
                return f(g(ga[s], arm, "p3d_mean"), 2)
            A("* **GNSS посреди маршрута** (раздел 5.1, вагон 30618, 3D ср. без коррекции → с ней): "
              f"GNSS 3 с {_p('first3', 'before')} → {_p('first3', 'after')} м, редкие пачки "
              f"{_p('sparse', 'before')} → {_p('sparse', 'after')} м, весь прогон "
              f"{_p('full', 'before')} → {_p('full', 'after')} м; скорость от GNSS после окна не "
              "зависит.")
    summ0 = inject_summary(result["inject"], result["kinds"])
    if summ0:
        bad = [f"{r['kind']} — {verdict(r)}" for r in summ0 if verdict(r) not in OK_VERDICTS]
        A("* **Инъекции:** " + ("; ".join(bad) if bad else "все виды в пределах нормы") + ".")
    A("")
    B, rel = load_baseline(root, args)
    if B is not None:
        L += before_after(S, B, rel)
    A("## 1. Как запустить")
    A("")
    A("Одна команда (Docker Compose, из корня репозитория; `DATA_DIR` в `.env` — каталог с bag; "
      "кэш строится из `data/` сам; ключи после `eval` уходят в `tools/eval.py`):")
    A("")
    A("```bash")
    A("docker compose run --rm eval --label \"<подпись версии>\"")
    A("```")
    A("")
    A("То же без Compose (PowerShell; образ из этого дерева — `docker compose build` "
      "или `docker build -f docker/Dockerfile -t vectra/tram:compose .`):")
    A("")
    A("```powershell")
    A("docker run --rm --cpus 2 -v ${PWD}:/repo -v <каталог с bag>:/repo/data:ro -w /repo `")
    A("    vectra/tram:compose python3 tools/eval.py --label \"<подпись версии>\"")
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
        ["`--map`", "`eval`", "`eval` — `config/eval/track_map.npz` пакета (карта ОЦЕНКИ: только train, "
         "масштаб колёс листа оценки; при другом `meas_scale` листа — предупреждение); `train` — карта "
         "только по обучающим с масштабом оцениваемого листа (`analysis/build_map.py`, строится сама); "
         "`jury` — боевая; `none` — без карты; путь"],
        ["`--baseline`", "`docs/data/eval_before/summary.json`", "итоги прежней версии для раздела 0 "
         "«До и после»; `none` — без раздела"],
        ["`--baseline-label`", "—", "подпись прежней версии в разделе 0 (иначе — из её `summary.json`)"],
        ["`--gnss`", "`3`", "секунд GNSS в связку от первой записи master; `full` — весь прогон; "
         "сценарии доступности `sparse`, `bursts`, `nostart`, `midstart`, `glitchy`, `none` "
         "(`tools/inject.py`; до/после коррекции по GNSS по всем сценариям — `tools/eval_gnss.py`)"],
        ["`--gnss-scenarios`, `--gnss-scenarios-before`", "—", "итоги `tools/eval_gnss.py` "
         "(`summary.json` или каталог) этой и прежней версии — раздел 5.1 «Сценарии доступности "
         "GNSS»; без ключа раздела нет"],
        ["`--frame`", "`mgrs`", "система эталона: `mgrs` (судья), `enu`, `equirect`, `utm`"],
        ["`--runner-frame`", "`auto`", "система выхода Runner: `auto` — параметр ноды `projection` "
         "(объявление в `tram_node.py`, поверх — лист), иначе `equirect` (старая версия пакета), с проверкой "
         "по величине; `mgrs`, `utm`, `enu`, `equirect`"],
        ["`--runner-grid`", "параметр `mgrs_grid`", "как **читать** выход Runner в MGRS (`\"\"` — перенос "
         "по точке, `37UDB`). Runner не настраивает — для этого `--set mgrs_grid=37UDB`"],
        ["`--judge-grid`", "`37UCB`", "соглашение судьи: от угла квадрата непрерывно (как pathgraph "
         "организаторов); `\"\"` — перенос по точке; код квадрата — и для матрицы соглашений"],
        ["`--ref-point`", "`base_link`", "точка эталона положения: `base_link` (по tf антенн, как у "
         "судьи) | `master` (антенна, прежний эталон — для сравнения)"],
        ["`--pathgraph`", "`auto`", "pathgraph организаторов для поперечной ошибки и пути вдоль него: "
         "`auto` — `_incoming/pathgraph`, если есть (в git его нет); `none`; путь"],
        ["`--variants`", "выкл.", "варианты листа: заглушки крипа / нули (+15 прогонов модели; при "
         "`--cpus 2` на свободной машине: 6,4 мин без ключа, 8,5 мин с ним)"],
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
      "`analysis/build_map.py` только по `tools/split.json:train` (98 прогонов): "
      "`--set train --split tools/split.json --calib <лист оценки>` (множитель пути — с `meas_scale` "
      "оцениваемого листа). Ключ — хэш `build_map.py`, `drive_model.json`, `split.json`, "
      "`track_map.py`, `runner.py`, `geodesy.py`, `tram_calibration.json` и листа, поэтому после "
      "правок кода карта пересобирается сама (в каталог кэша или `out/maps/`, если кэш только для "
      "чтения; нужен кэш 98 train, ~2 мин). По умолчанию (`--map eval`) берётся готовая оценочная "
      "карта пакета `config/eval/track_map.npz`.")
    A("")
    A("После правок кода: `python3 tools/eval.py --label \"<подпись версии>\"`. Лист `eval` "
      "подхватится из `config/eval/`; параметры ноды (`projection`, `mgrs_grid`, …) — из объявлений "
      "`tram_node.py` и листа. Проверки: (1) в шапке нет предупреждения о параметрах, не дошедших "
      "до Runner; (2) «взгляд судьи» в разделе 3.2 равен ошибке в MGRS при своём соглашении (нет "
      "~100 км); (3) раздел 5: сетка и скорость с GNSS весь прогон совпадают с GNSS 3 с во всех "
      "прогонах.")
    A("")
    A(f"Результаты: `{args.out}/summary.json` (итоги), `runs.json` (по прогонам), `inject.json` "
      "(инъекции), `timing.json` (время, sha256 JSON, git, строки реального времени; не "
      "детерминирован). Тесты инструментов: `python3 -m pytest tools/eval_selftest.py -q`.")
    A("")
    wall = timing.get("wall_all_s")
    A(f"Этот прогон: {f(wall / 60 if wall else None, 1)} мин без графиков и документа (они ~0,5 мин), "
      f"процессов {timing.get('workers')}{', с --variants' if meta.get('variants_on') else ''} (время "
      "зависит от загрузки машины: при соседних контейнерах, где наш получал ~1,3 ядра, тот же "
      "прогон без --variants шёл 18,0 мин вместо 6,4).")
    shas = timing.get("sha256") or {}
    if shas:
        A("sha256 JSON этого прогона: " + ", ".join(f"`{k}` {v[:12]}" for k, v in shas.items()) + ".")
    A("Воспроизводимость: два полных прогона дают побайтно одинаковые `summary.json`, `runs.json`, "
      "`inject.json` (`--check-determinism` проверяет это сам); чистый `git clone` с пустым кэшем "
      "строит кэш и карту из `data/` и даёт тот же sha256 `runs.json`. Старая карта "
      "`analysis/cache/track_map_train.npz` (собрана на Windows) отличается от пересобранной "
      "множителем пути в 16-м знаке, поэтому не используется.")
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
      "стартового всплеска (StartSorter): "
      + (f"да, окно {f(gl.get('start_sort_s'), 2)} с по времени записи" if gl.get("start_sort_s") is not None
         else "нет в этом коде") + ". Пульс ноды не эмулируется: он публикует те же узлы "
      "сетки с теми же значениями, что связка выдаёт при следующем сообщении (прогноз на копии тем "
      "же кодом), кроме ≤ 2 с после последнего входа записи. Исключение в связке считается падением "
      "ноды: дальше выходов нет (нода ловит исключения в колбэках и живёт, но Runner их "
      "бросать не должен — это дефект в любом случае).")
    A("* **Пары.** Выход ↔ эталон по ближайшей метке `header.stamp` в пределах 0,05 с (README, 5.1). "
      "Скорость публикуется на каждом шаге; положение — только при `pos_valid` (нода не "
      "публикует `/result/position` без якоря GNSS или у края квадрата) и конечных x, y, z: фикс "
      "сопоставляется с ближайшим **опубликованным** положением, без него — непарный. Доли пар, NaN "
      "и падения — в «Главном» и разделе 3.")
    A("* **Эталон скорости.** Официального эталона нет: источников четыре (2 тележки, 2 GNSS). "
      "Основной — |v| GNSS master по (x, y); дополнительный — rover. Фаза: стоянка, если |v| GNSS "
      "< 0,2 м/с, иначе по ручке: > 0 тяга, < 0 торможение, 0 выбег. Ложная стоянка: режим "
      "STANDSTILL (у базы — v < 0,3 м/с) при |v| GNSS > 0,5 м/с.")
    if (meta.get("ref_point") or "master") == "base_link":
        A("* **Эталон положения — система судьи.** Организаторы (25.09–26.09): плоские координаты "
          "**MGRS** от угла квадрата **37UCB непрерывно** (так записан их pathgraph: x = E − 300 000, "
          "y = N − 6 100 000, через E = 400 км без скачка); по REP-103 x — восток, y — север, z — "
          "высота. Точка — **base_link** (ось поворота передней тележки на уровне рельса; tf антенн: "
          "master (−9,873; 0; 3,0), rover (2,563; 0; 3,0)). Эталон по GNSS: пара master+rover одной "
          "эпохи (±0,05 с, база 5–25 м) — base_link = master + 9,873/12,436 · (rover − master), z по "
          "той же доле между высотами антенн минус 3,0; без пары — master + 9,873 м по курсу "
          "траектории, z − 3,0. Ошибки считаются в непрерывных координатах UTM зоны 37 — это та же "
          "система с точностью до сдвига.")
    else:
        A("* **Эталон положения.** Антенна master (прежний эталон, `--ref-point master`), плоские "
          "MGRS; ошибки в непрерывных координатах UTM зоны 37.")
    A("* **Pathgraph организаторов** (если есть `_incoming/pathgraph`): для каждой пары выбирается "
      "путь своего направления (курс эталона ±90°), на него проецируются эталон и выход: "
      "поперёк — расстояние выхода до оси пути, вдоль — разность дуговых координат выхода и "
      "эталона. Пары, где эталон дальше 10 м от пути или за концом pathgraph (записи длиннее его: "
      "конечные, развороты), не входят; их доля — «вне pathgraph».")
    A("* **Граница квадратов 100 км.** Линия пересекает E = 400 км: запад (~1,2 км) в **37U CB**, "
      "остальное в **37U DB**. Соглашение судьи известно (pathgraph: от 37UCB непрерывно, "
      "`--judge-grid 37UCB`); ниже для справки прежний разбор. До pathgraph оно было неизвестно: "
      "(а) «перенос по точке» — координаты внутри квадрата, где лежит точка (Autoware "
      "gnss_poser, lanelet2 MGRSProjector; x скачет на 100 км), или (б) непрерывно от одного "
      "квадрата (`37UDB`, запад — отрицательный x). Поэтому отдельно считается «несовпадение "
      "квадрата»: сколько пар при переносе по точке попали бы в другой квадрат, чем эталон (каждая "
      "такая пара у судьи — ошибка ~100 км); **матрица 2×2** «наше соглашение × соглашение судьи» "
      f"(перенос по точке / непрерывно от `{meta.get('boundary_grid') or M.BOUNDARY_GRID}`) по "
      "непрерывной оценке; и «взгляд судьи» — сырые x, y, z выхода Runner против эталона в "
      "соглашении `--judge-grid`.")
    A("* **Перевод выхода.** Если выход Runner не в MGRS (старая версия пакета — equirect от своей точки "
      "начала), x, y, z переводятся обратно в широту/долготу через его же начало и формулу, затем в "
      "UTM. Так ошибка перевода равна нулю, а «взгляд судьи» показывает, что увидел бы судья без "
      "перевода.")
    A("* **Вдоль/поперёк пути.** Проекция на ломаную эталона (медиана по 5 фиксам, шаг 1 м, окно "
      "±1 км, то же направление движения; алгоритм `tools/core_metrics.py`). Дрейф — ошибка **в конце "
      "прогона**, отнесённая к длине пути (PDF, стр. 6): 3D и вдоль пути.")
    A("* **Итоги.** Средние взвешены числом пар скорости прогона: "
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
    A(f"### 3.2. Положение (MGRS от {meta.get('judge_grid') or 'квадрата точки'}, точка "
      f"{meta.get('ref_point') or 'master'}; ошибки в непрерывных координатах)")
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
                     t.get("p_out", "—"), t.get("p_out_invalid", 0), t.get("p_gap_steps", "—"),
                     t.get("p_nan", 0),
                     f"{f(run_mean(result, ids, est, 'rate_hz'), 2)} / "
                     f"{f(run_mean(result, ids, est, 'rate_pos_hz'), 2)}"])
    A("Полнота положения (фикс ↔ опубликованное положение в пределах 0,05 с):")
    A("")
    A(table(["оценка", "меток эталона", "пар", "доля пар", "строк выхода", "без положения (pos_valid)",
             "из них после первого опубликованного", "положение NaN",
             "частота выхода / положения, Гц (ср. по прогонам)"], rows))
    A("")
    if tm.get("pg_cross_mean") is not None:
        pgm = meta.get("pathgraph") or {}
        A(f"**По pathgraph организаторов** (`{pgm.get('src', '')}`, путей {pgm.get('paths', '—')}, "
          f"длина {', '.join(f(x, 0) for x in pgm.get('length_m', []))} м):")
        A("")
        rows = []
        for name, t in (("модель", tm), ("база «только колесо»", tn)):
            rows.append([name, pct(t.get("pg_frac"), 1), t.get("pg_pairs", "—"),
                         f(t.get("pg_cross_mean"), 3), f(t.get("pg_cross_p95")), f(t.get("pg_cross_max"), 1),
                         f(t.get("pg_along_mean")), f(t.get("pg_along_rmse")),
                         f(t.get("pg_along_bias"), 2, True), f(t.get("p3d_on_pg")),
                         f(t.get("p3d_off_pg"))])
        A(table(["оценка", "эталон на pathgraph", "пар", "поперёк ср.", "поперёк 95 %", "поперёк макс",
                 "вдоль ср. абс.", "вдоль RMSE", "вдоль смещение", "3D ср. на pathgraph",
                 "3D ср. за его концами"], rows))
        A("")
        A(f"Сам эталон base_link (GNSS) против pathgraph: медиана |поперёк| "
          f"{f(tm.get('pg_ref_lat_med'), 3)} м (со знаком {f(tm.get('pg_ref_lat_signed_med'), 3, True)}, "
          f"95 % — {f(tm.get('pg_ref_lat_p95'), 2)} м), высота эталона − z pathgraph медиана "
          f"{f(tm.get('pg_ref_dz_med'), 2, True)} м (средние медиан по прогонам). Это проверка, "
          "что pathgraph — ось пути точки base_link на уровне рельса.")
        A("")
    bg = meta.get("boundary_grid") or M.BOUNDARY_GRID
    A(f"**Квадраты MGRS.** Пар, где оценка при «переносе по точке» попала бы в другой квадрат "
      f"100 км, чем эталон: модель {tm.get('sq_mismatch', 0)} из {tm.get('p_pairs', 0)}, база "
      f"{tn.get('sq_mismatch', 0)}. Справочно — матрица «наш выход × соглашение судьи» (средняя "
      "3D модели, м; в скобках — пар с ошибкой > 1 км; соглашение судьи с 26.09 — от 37UCB):")
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
      "части пути по другую сторону границы. Соглашение выхода — параметр ноды `mgrs_grid` "
      "(по умолчанию `37UCB`, как pathgraph; `\"\"` — перенос по точке).")
    A("")
    A("«Взгляд судьи» (сырые x, y, z выхода Runner против эталона MGRS "
      f"{'с переносом по точке' if not meta['judge_grid'] else 'от ' + meta['judge_grid']}): "
      f"средняя 3D **{f(tm.get('judge_raw_3d_mean'), 1)} м**, максимум {f(tm.get('judge_raw_3d_max'), 1)} м, "
      f"пар с ошибкой > 1 км: {tm.get('judge_raw_km', 0)}"
      + (" — выход Runner не в MGRS судьи (старая версия пакета — equirect от начала): без "
         "перевода судья увидел бы ошибку ~100 км."
         if (tm.get("judge_raw_3d_mean") or 0) > 1000 else "."))
    A("")
    A("Чувствительность к системе эталона (тот же выход, другой эталон; 3D ср. / конец ср.):")
    A("")
    rows = [["MGRS (UTM)", f(tm.get("p3d_mean")) + " / " + f(tm.get("p3d_end_mean")),
             f(tn.get("p3d_mean")) + " / " + f(tn.get("p3d_end_mean"))]]
    for fr, lab in (("enu", "строгий ENU"), ("equirect", "прежний equirect")):
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
                     f(rm.get("pg_cross_mean"), 2), rm.get("p_gap_steps", "—"),
                     "упала" if rm.get("crash") else ""])
    A(table(["прогон", "путь, км", "пар v", "MAE", "MAE база", "смещение", "±2σ", "3D ср.",
             "3D база", "3D конец", "вдоль конец", "дрейф 3D, %", "поперёк pathgraph",
             "шагов без положения", "падение"], rows))
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
          "(база идёт через тот же Runner). "
          + ("С коррекцией по GNSS (`gnss_correction: true`, организаторы 26.09 18:05) положение "
             "после окна идёт за GNSS, а метки сетки, скорость и признак публикации положения "
             "должны совпасть с выходом при GNSS 3 с: GNSS не двигает сетку и не влияет на скорость."
             if gnss_corr(meta) else
             "Без коррекции (`gnss_correction: false`) GNSS только для начальной выставки, "
             "значит выход с GNSS весь прогон должен совпасть с выходом при GNSS 3 с: те же метки "
             "сетки, те же скорость, положение и признак публикации положения."))
        A("")
        A(table(["GNSS в связку", "MAE, м/с", "3D ср., м"], [
            ["весь прогон", f(a_.get("v_mae"), 4), f(a_.get("p3d_mean"), 1)],
            ["первые 3 с", f(b_.get("v_mae"), 4), f(b_.get("p3d_mean"), 1)]]))
        A("")
        if gnss_corr(meta):
            A(f"Сетка, pos_valid и скорость совпали (|Δv| ≤ 1e-9): **{gf.get('same_speed_runs')} из {n}** "
              f"прогонов; наибольшее |Δv| {f(gf.get('max_dv'), 3)} м/с; наибольшее |Δ положения| "
              f"{f(gf.get('max_dpos'), 1)} м — это поправки по GNSS. Если сетка или скорость не "
              "совпали — GNSS влияет на ядро (это был бы дефект).")
        else:
            A(f"Совпали (та же сетка и pos_valid, |Δv| ≤ 1e-9, |Δxyz| ≤ 1e-6): **{gf.get('identical_runs')} из {n}** "
              f"прогонов; наибольшее |Δv| {f(gf.get('max_dv'), 3)} м/с, наибольшее |Δ положения| "
              f"{f(gf.get('max_dpos'), 1)} м. Если не совпали — GNSS после окна влияет на выход "
              "(это был бы дефект).")
        A("")
        rows = [[b, "да" if r["same_grid"] else f"нет ({r['n_out_full']} / {r['n_out_3s']})",
                 f(r.get("max_dv"), 3), f(r.get("max_dpos"), 1), f(r.get("p3d_mean_full"), 1),
                 f(r.get("p3d_mean_3s"), 1)] for b, r in gf.get("runs", {}).items()]
        A(table(["прогон", "та же сетка", "макс |Δv|, м/с", "макс |Δ положения|, м", "3D ср. (весь)",
                 "3D ср. (3 с)"], rows))
    else:
        A("Не считалось (`--gnss full` уже основной режим, `--no-gnss-full` или `--quick`).")
    A("")
    L += gnss_scenarios(root, args)[0]

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
        A("")
        notes = (timing["realtime_notes"] if timing.get("realtime_notes") is not None
                 else probe_notes(root, args.probe_glob))
        for name, text in notes:
            A(f"* `{name}`: {text}")
        if notes:
            A("")
        A("Отчётный замер — `tools/measure_realtime.sh` (полный bag ≥ 20 мин, "
          "`--cpus 2 --memory 512m`, машина без соседей); порядок и таблица критериев ТЗ — "
          "`docs/JURY.md` §6.")
    else:
        A("Сводок `tools/ros_probe.py` не найдено (`--probe-glob "
          f"{args.probe_glob}`). Замер — `tools/measure_realtime.sh`: полный bag ≥ 20 мин, "
          "`--cpus 2 --memory 512m`, машина без соседей. После замера перезапустить "
          "`tools/eval.py` — таблица подставится сюда.")
    A("")
    A(f"Офлайн-скорость связки в этом прогоне (не замер ноды): {timing.get('events_per_s', '—')} "
      "входных сообщений в секунду на процесс вместе с базой — предварительно, при соседних "
      "контейнерах.")
    A("")
    A("## 8. Открытые вопросы")
    A("")
    A("* Соглашение судьи на границе квадратов снято pathgraph организаторов: от 37UCB "
      "непрерывно; точка — base_link по tf антенн. Как именно судья строит свой эталон base_link "
      "(по двум антеннам или по одной с курсом), не сказано; на прямой способы совпадают, "
      "расходиться могут только на кривых.")
    A("* Эталон скорости: официального нет; показываем master (основной) и rover.")
    A("")
    return "\n".join(L) + "\n"
