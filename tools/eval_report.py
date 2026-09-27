"""docs/EVAL.md и графики docs/img/eval_*.png по результатам tools/eval.py.

Всё, что в документе, берётся из out/eval/*.json этого же прогона (плюс
сводки tools/ros_probe.py для раздела «Реальное время», если они есть).
"""

import glob
import json
import math
import os
import re
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
# итоги другой версии для раздела «До и после» (--baseline): по умолчанию раздела нет
BASELINE = "none"
KIND_SHORT = dict(front_zero="отказ передней (0)", rear_drop="отказ задней (30639)",
                  both_zero="обе = 0, 20 с", both_stuck="обе залипли, 20 с",
                  dropout="пропуск 2 с", gap_all="пропуск всех входов 2 с",
                  skid_brake="юз −30 %", spin_traction="буксование +30 %",
                  outliers="выбросы ×3", noise="шум ×5", nan="NaN", stamp_jump="скачок метки +30 с")


# ------------------------------------------------------------------ форматирование

def f(x, nd=2, sign=False):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "-"
    s = f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"
    return s.replace(".", ",").replace("-", "−")


def plural_ru(n, one, few, many):
    """Форма слова при числе n: 1 сообщение, 3 сообщения, 11 сообщений."""
    n = abs(int(n))
    if n % 10 == 1 and n % 100 != 11:
        return one
    return few if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14 else many


def pct(x, nd=1):
    return "-" if x is None or not math.isfinite(x) else f(100.0 * x, nd) + " %"


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


def table(head, rows, align=None):
    """Таблица Markdown. align - по букве на столбец: l - влево, r - вправо; по
    умолчанию первый столбец влево, остальные (числа) вправо."""
    align = align or "l" + "r" * (len(head) - 1)

    def cell(c):
        return str(c).replace("|", "\\|")         # «|Δv|» не должен ломать таблицу
    out = ["| " + " | ".join(cell(h) for h in head) + " |",
           "|" + "|".join(":---" if a == "l" else "---:" for a in align) + "|"]
    out += ["| " + " | ".join(cell(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


WIDTH = 100                                     # перенос прозы в исходнике документа, знаков
# слово, с которого не может начинаться строка абзаца: Markdown прочтёт его как
# маркер списка, заголовка, цитаты или подчёркивание заголовка
NO_LINE_START = re.compile(r"^([-+*>=#]+|\d+[.)])$")
NBSP = chr(0xA0)                                # склейка слов на время переноса
# пробел, на котором строку не рвут: между числом и единицей, внутри «10 259» и после
# знака минус перед числом
NO_BREAK = re.compile(r"(?<=\d) (?=(?:м/с|мин|мс|м|с|км|%|Гц|МБ)(?![\w/])|\d{3}(?!\d))|(?<=−) (?=\d)")


def wrap(text, first="", rest=None, width=WIDTH):
    """Абзац с переносом по словам примерно на width знаков; first и rest - начало
    первой и остальных строк («- » и «  » у пункта списка, «> » у цитаты)."""
    rest = first if rest is None else rest
    lines, cur = [], ""
    words = NO_BREAK.sub(NBSP, text.replace(NBSP, " ")).split(" ")
    for w in (x.replace(NBSP, " ") for x in words if x):
        pre = rest if lines else first
        if cur and len(pre) + len(cur) + 1 + len(w) > width and not NO_LINE_START.match(w):
            lines.append(pre + cur)
            cur = w
        else:
            cur = f"{cur} {w}" if cur else w
    lines.append((rest if lines else first) + cur)
    return "\n".join(lines)


def item(label, text):
    """Пункт списка «- **Ярлык.** текст»."""
    return wrap(f"**{label}.** {text}", "- ", "  ")


def slug(title):
    """Якорь заголовка по правилам GitHub: строчные буквы, без знаков, пробел - дефис."""
    return re.sub(r"[^\w\- ]", "", title.strip().lower()).replace(" ", "-")


def toc(lines):
    """Оглавление столбиком по заголовкам ## и ### (повторный якорь - с номером)."""
    out, seen, fence = ["**Разделы**", ""], {}, False
    for line in lines:
        if line.startswith("```"):
            fence = not fence
        m = None if fence else re.match(r"^(#{1,3}) (.+)$", line)
        if not m:
            continue
        s = slug(m.group(2))
        n = seen.get(s, 0)
        seen[s] = n + 1
        if len(m.group(1)) > 1:
            out.append(("  " if len(m.group(1)) == 3 else "")
                       + f"- [{m.group(2)}](#{s}{'-' + str(n) if n else ''})")
    return out


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
            # рисуют «иглы» на весь график; числа в таблицах - без сглаживания
            al, d3 = _med(np.asarray(s["al"], float)), _med(np.asarray(s["d3"], float))
            back = np.r_[False, np.diff(x) < -0.001]      # скачок эталона назад: разрыв линии
            x = x.copy()
            x[back] = np.nan
            ax[0, col].plot(x, al, color=color, lw=0.9, alpha=0.75)
            ax[1, col].plot(x, d3, color=color, lw=0.9, alpha=0.75)
        ax[0, col].set_title(f"{title}, {len(ids)} прогонов", loc="left")
        ax[1, col].set_xlabel("путь по эталону GNSS, км")
    ax[0, 0].set_ylabel("вдоль пути, м (+ - впереди)")
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
            ttl = KIND_SHORT.get(kind, kind) + (" - модель упала" if crash else "")
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
        fig.suptitle(f"Инъекции в {bag}; серая полоса - окно аномалии", x=0.01, ha="left",
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
        rows.append([p.parent.name, o.get("count", "-"), f(o.get("rate_stamp_hz"), 1),
                     f(i2o.get("p50"), 1), f(i2o.get("p99"), 1), f(i2o.get("max"), 1),
                     f(cpu.get("mean"), 1), f(cpu.get("max"), 1),
                     f(rss[-1], 0) if isinstance(rss, list) and rss else "-",
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
    """Каталог графиков - img/ рядом с документом (docs/img для docs/EVAL.md)."""
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
    # диапазоны в документе через дефис: описания сценариев в JSON пишутся с коротким тире
    (root / args.doc).write_text(doc.replace("–", "-"), encoding="utf-8")


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
    """Итоги другой версии для таблицы «до / после» (--baseline summary.json;
    по умолчанию BASELINE - без таблицы)."""
    spec = getattr(args, "baseline", None) or BASELINE
    if spec in ("", "none"):
        return None, None
    path = Path(spec)
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
    """Раздел 0: те же метрики у другой версии (--baseline) и у этой на тех же прогонах."""
    L = []
    A = L.append
    mb, ma = _tot(B, "all", "model"), _tot(S, "all", "model")
    nb, na = _tot(B, "all", "naive"), _tot(S, "all", "naive")
    bm, am = B["meta"], S["meta"]
    A("## 0. До и после")
    A("")
    same = sorted(bm.get("runs", [])) == sorted(am["runs"])
    same_sm = (bm["sheet"]["label"] == am["sheet"]["label"]
               and bm["map"]["label"] == am["map"]["label"])
    # тот же файл пакета у другой версии кода обычно другой: сравнивать и sha
    same_sha = (bm["sheet"].get("sha") == am["sheet"].get("sha")
                and bm["map"].get("sha") == am["map"].get("sha"))
    blab = bm.get("label") or "без подписи"
    A(item("До", f"{blab[:1].upper()}{blab[1:]}. Итоги - `{rel}`, код пакета sha "
                 f"`{bm.get('pkg_src_sha')}`"
                 + ("" if same_sm else f", лист {bm['sheet']['label']}, карта {bm['map']['label']}")
                 + (f"; **{bm['sheet']['leak']}**" if bm["sheet"].get("leak") else "") + "."))
    A(item("После", "Этот прогон, версия в шапке документа. "
           + ("Прогоны, лист и карта те же." if same and same_sm and same_sha else
              "Прогоны те же, лист и карта - оценочные файлы пакета своей версии."
              if same and same_sm else
              "Прогоны те же, лист или карта другие." if same else
              "Прогоны **другие**: сравнение неполное.")))
    A("")
    # эталон положения и соглашение судьи у «до» могут быть другими (у старых версий -
    # антенна master и перенос по точке) - тогда строки положения сравнивают разное
    rb, ra = bm.get("ref_point") or "master", am.get("ref_point") or "master"
    jb, ja = bm.get("judge_grid") or "", am.get("judge_grid") or ""
    if (rb, jb) != (ra, ja):
        def _ref(r, j):
            return (("base_link по tf антенн" if r == "base_link" else "антенна master")
                    + ", соглашение судьи " + (f"от {j} непрерывно" if j else "перенос по точке"))
        A(wrap(f"**Эталон положения разный:** «до» - {_ref(rb, jb)}; «после» - {_ref(ra, ja)}. "
               "Строки скорости сравнимы как есть, строки положения - нет: точка вагона и система "
               "другие."))
        A("")
    rows = []

    def row(name, key, nd=4, sign=False, pc=False):
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
        for key, nm, nd, sg, pc in (("mae", "MAE, м/с", 4, False, False),
                                    ("bias", "смещение, м/с", 4, True, False),
                                    ("cov2s", "±2σ", 1, False, True)):
            def v(d):
                x = (d or {}).get(key)
                return pct(x) if pc else f(x, nd, sg)
            rows.append([f"{M.PHASES_RU[p]}: {nm}", v(db), v(da),
                         "-" if pc else v(eb), "-" if pc else v(ea)])
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
    row("|z| ср., м", "pz_mean", 2)
    row("поперёк pathgraph ср., м", "pg_cross_mean", 3)
    row("вдоль pathgraph ср. |ошибка|, м", "pg_along_mean", 2)
    A(table(["Метрика", "Модель до", "Модель после", "База до", "База после"], rows))
    A("")
    L += vehicle_before_after(S, B, "30618")
    gb, ga = B.get("gnss_full") or {}, S.get("gnss_full") or {}
    if gb or ga:
        nb_, na_ = len(gb.get("runs", {})), len(ga.get("runs", {}))
        A(wrap(f"GNSS весь прогон, первые 5 мин: выход совпал с «GNSS 3 с» до - в "
               f"{gb.get('identical_runs', '-')} из {nb_}, после - в "
               f"{ga.get('identical_runs', '-')} из {na_}."
               + (f" Сетка и скорость после совпали в {ga.get('same_speed_runs', '-')} из {na_}; "
                  "положение с коррекцией по GNSS и должно отличаться." if gnss_corr(am) else "")))
        A("")
    A(wrap("База «только колесо» - тот же Runner (выставка, карта, привязки) со средним свежих "
           "показаний тележек вместо ядра; у «до» и «после» она различается только кодом связки "
           "и картой."))
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
    A(wrap(f"**Только вагон {veh}** (прогонов: {ma.get('runs', mb.get('runs', '-'))}"
           + ("; по ответу организаторов жюри проверяет только его" if veh == "30618" else "")
           + "):"))
    A("")
    rows = []
    for name, key, nd, sg, pc in VEH_KEYS:
        if mb.get(key) is None and ma.get(key) is None:
            continue
        rows.append([name] + [pct(T.get(key)) if pc else f(T.get(key), nd, sg) for T in (mb, ma)])
    A(table(["Метрика", "Модель до", "Модель после"], rows))
    A("")
    return L


def vehicle_detail(S, veh):
    """Итоги одного вагона подробно: модель и база «только колесо» (жюри проверяет только 30618)."""
    tm, tn = _tot(S, veh, "model"), _tot(S, veh, "naive")
    if not tm:
        return []
    L = []
    A = L.append
    A(wrap(f"**Только вагон {veh}** (прогонов: {tm.get('runs', '-')}"
           + ("; по ответу организаторов жюри проверяет только его" if veh == "30618" else "")
           + "):"))
    A("")
    rows = []
    for name, key, nd, sg, pc in VEH_KEYS:
        if tm.get(key) is None and tn.get(key) is None:
            continue
        rows.append([name] + [pct(T.get(key)) if pc else f(T.get(key), nd, sg) for T in (tm, tn)])
    A(table(["Метрика", "Модель", "База «только колесо»"], rows))
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
    """Раздел 5.1: сценарии доступности GNSS (tools/eval_gnss.py): «выкл.» /
    «вкл.» - этот код с gnss_correction false / true, «до» - другая версия
    (--gnss-scenarios-before), если задана. -> (строки, (итоги, итоги «до») | None)."""
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
    A(wrap(f"Те же {len(ma['runs'])} отложенных, лист и карта оценки. GNSS подаётся по сценарию "
           "(`tools/inject.py`, зерно - от прогона), эталон - base_link по всем точкам GNSS "
           "прогона. Версии в столбцах:"))
    A("")
    A(item("Вкл", f"Этот код по умолчанию, `gnss_correction: true`; итоги `{ra}`, код пакета sha "
                  f"`{ma.get('pkg_src_sha')}`."))
    A(item("Выкл", "Тот же код с `gnss_correction: false`: GNSS только для выставки."))
    if Sb is not None:
        A(item("До", f"Версия без коррекции по GNSS; итоги `{rb}`, код пакета sha "
                     f"`{Sb['meta'].get('pkg_src_sha')}`."))
    A("")
    ru = ma.get("scenario_ru") or {}
    A(table(["Сценарий", "Что подаётся"], [[f"`{s}`", ru.get(s, s)] for s in scen], "ll"))
    A("")
    for grp, title in (("30618", "Вагон 30618"), ("all", "Все отложенные")):
        T = {s: g(Sa, "totals", s, grp) or {} for s in scen}
        if not any(T.values()):
            continue
        Tb = {s: (g(Sb, "totals", s, grp) or {}) if Sb is not None else {} for s in scen}
        n = next((x.get("after", {}).get("runs") for x in T.values() if x.get("after")), "-")
        A(wrap(f"**{title}** (прогонов: {n}"
               + ("; по ответу организаторов жюри проверяет только его" if grp == "30618" else "")
               + "):"))
        A("")
        rows = []
        for s in scen:
            b0 = Tb[s].get("before") or {}
            b1 = T[s].get("before") or {}
            a1 = T[s].get("after") or {}
            # столбец «до» - только когда есть итоги другой версии (--gnss-scenarios-before)
            pos, vel = ((b0, b1, a1), (b0, a1)) if Sb is not None else ((b1, a1), (a1,))
            rows.append([f"`{s}`",
                         " / ".join(f(x.get("p3d_mean"), 2) for x in pos),
                         " / ".join(f(x.get("p3d_end_mean"), 2) for x in pos),
                         f(a1.get("p3d_end_max"), 1),
                         " / ".join(f(x.get("v_mae"), 4) for x in vel),
                         str(a1.get("n_corr", "-"))])
        arms, varms = ("до / выкл. / вкл.", "до / вкл.") if Sb is not None else ("выкл. / вкл.", "вкл.")
        A(table(["Сценарий", f"3D ср., м: {arms}", f"Конец ср., м: {arms}",
                 "Конец макс, м: вкл.", f"MAE скорости, м/с: {varms}", "Поправок GNSS: вкл."], rows))
        A("")
    # скорость: в сценариях с теми же первыми 3 с GNSS она та же, что в first3
    same = []
    for s in ("sparse", "bursts", "full", "glitchy"):
        if s in scen and "first3" in scen:
            a = g(Sa, "totals", s, "all", "after", "v_mae")
            b = g(Sa, "totals", "first3", "all", "after", "v_mae")
            same.append((s, a is not None and a == b))
    if same:
        A(wrap("Скорость: в сценариях с теми же первыми 3 с GNSS ("
               + ", ".join(f"`{s}`" for s, _ in same) + ") MAE «вкл.» "
               + ("равна MAE `first3` во всех" if all(v for _, v in same) else
                  "**отличается** от `first3` в " + ", ".join(s for s, v in same if not v))
               + ": GNSS после окна двигает только положение. В `nostart` и `midstart` выставка "
               "позже, поэтому другой и онлайн-масштаб колёс, который начинается с выставки."))
        A("")
    return L, (Sa, Sb)


def shq(s):
    """Строка в двойных кавычках для bash."""
    return '"' + re.sub(r'([\\"$`])', r"\\\1", s) + '"'


def bash_block(parts, width=WIDTH):
    """Блок bash из частей команды: перенос «\\» с отступом продолжения примерно на width знаков."""
    lines, cur = [], parts[0]
    for p in parts[1:]:
        if len(cur) + len(p) + 3 > width:
            lines.append(cur + " \\")
            cur = "    " + p
        else:
            cur += " " + p
    return ["```bash"] + lines + [cur, "```"]


def doc_keys(args, meta, rel):
    """Ключи tools/eval.py, от которых зависит вид документа (версия для сравнения, сценарии
    GNSS, каталог JSON), и отдельно подпись версии для сравнения: одни и те же для расчёта и
    пересборки."""
    keys = ["--quick"] if meta.get("quick") else []
    if rel:
        keys.append(f"--baseline {rel}")
    for name, opt in (("gnss_scenarios", "--gnss-scenarios"),
                      ("gnss_scenarios_before", "--gnss-scenarios-before")):
        if getattr(args, name, ""):
            keys.append(f"{opt} {getattr(args, name)}")
    if getattr(args, "out", "out/eval") != "out/eval":
        keys.append(f"--out {args.out}")
    blab = getattr(args, "baseline_label", "")
    return keys, (["--baseline-label " + shq(blab)] if blab and rel else [])


TOC_MARK = "\0toc"


def render(result, timing, args, pics, root):
    S = result["summary"]
    meta = S["meta"]
    ids = meta["runs"]
    L = []
    A = L.append
    label = meta.get("label") or "без подписи"
    docdir = (Path(root) / args.doc).parent

    def link(path):
        return Path(os.path.relpath(Path(root) / path, docdir)).as_posix()

    A("# Оценка точности и устойчивости (tools/eval.py)")
    A("")
    A(wrap("Точность скорости и положения, устойчивость к аномалиям входов и работа в реальном "
           "времени на отложенных записях, которых не было в обучении. Эталон - GNSS. Числа "
           "считает `tools/eval.py` (реальное время - замер `tools/ros_probe.py`), документ "
           "целиком пишет `tools/eval_report.py`, руками его не правят."))
    A("")
    A(wrap(f"Модель и параметры - [MODEL.md]({link('MODEL.md')}), запуск и проверка пакета - "
           f"[JURY.md]({link('docs/JURY.md')}), соответствие ТЗ - "
           f"[TZ_COMPLIANCE.md]({link('docs/TZ_COMPLIANCE.md')})."))
    A("")
    A(wrap(f"**Версия чисел: {label}.** Код пакета: sha `{meta['pkg_src_sha']}`"
           f"{', git ' + timing['git'] if timing.get('git') else ''}. "
           f"Лист: {meta['sheet']['label']} (sha `{meta['sheet']['sha']}`). "
           f"Карта: {meta['map']['label']}"
           f"{' (sha `' + meta['map']['sha'] + '`)' if meta['map']['sha'] else ''}. "
           f"GNSS в связку: {gnss_label(meta)}. "
           f"Прогонов: {len(ids)}{' (--quick, первые 300 с)' if meta['quick'] else ''}.", "> "))
    for leak in (meta["sheet"].get("leak"), meta["map"].get("leak")):
        if leak:
            A(">")
            A(wrap(f"**{leak}.** Эти числа не отчётные, пока нет оценочного листа.", "> "))
    if meta["sheet"].get("kind") == "json":
        A(">")
        A(wrap("Лист json - не то, что читает нода: боевая нода берёт `config/tram.yaml`.", "> "))
    unused = meta.get("node_params_unused") or []
    if unused:
        A(">")
        A(wrap(f"**Внимание: параметры ноды не дошли до Runner: {', '.join(unused)}.** Связка "
               "`tools/eval_replay.py` отстала от `tram_node.py`, числа могут не совпадать с "
               "выходом ноды. Дописать `make_runner` и перезапустить.", "> "))
    A("")
    A(TOC_MARK)
    A("")

    # ---------------- главное
    tm0, tn0 = _tot(S, "all", "model"), _tot(S, "all", "naive")
    rp = meta.get("ref_point") or "master"
    A("## Главное")
    A("")
    A(item("Скорость", f"{len(ids)} отложенных, эталон GNSS master: модель MAE "
                       f"{f(tm0.get('v_mae'), 4)} м/с, смещение {f(tm0.get('v_bias'), 4, True)}, ±2σ "
                       f"{pct(tm0.get('cov2s_v'))}; база «только колесо» MAE "
                       f"{f(tn0.get('v_mae'), 4)}, смещение {f(tn0.get('v_bias'), 4, True)}."))
    A(item("Положение", f"MGRS от {meta.get('judge_grid') or 'квадрата точки'}, эталон - "
                        f"{'base_link по tf антенн' if rp == 'base_link' else 'антенна master'}: "
                        f"модель 3D ср. {f(tm0.get('p3d_mean'))} м, конец ср. "
                        f"{f(tm0.get('p3d_end_mean'), 1)} м, вдоль RMSE {f(tm0.get('along_rmse'))} м, "
                        f"дрейф 3D по концу медиана {f(tm0.get('drift_pct_3d_median'), 3)} % (макс "
                        f"{f(tm0.get('drift_pct_3d_max'), 2)} %); база 3D {f(tn0.get('p3d_mean'))} м."
           + (f" Поперёк pathgraph организаторов ср. {f(tm0.get('pg_cross_mean'))} м (95 % - "
              f"{f(tm0.get('pg_cross_p95'))} м), вдоль pathgraph ср. |ошибка| "
              f"{f(tm0.get('pg_along_mean'))} м." if tm0.get("pg_cross_mean") is not None else "")))
    A(item("Полнота выхода", "Модель: " + completeness(tm0, S.get("crashes") or {}) + "."))
    zv = next((v for v in S.get("variants", []) if v["name"] == "zero" and v.get("totals")), None)
    if zv:
        A(item("Без заглушек крипа", f"c_creep = c_creep_drag = 0: MAE "
                                    f"{f(zv['totals'].get('v_mae'), 4)}, смещение "
                                    f"{f(zv['totals'].get('v_bias'), 4, True)}, 3D "
                                    f"{f(zv['totals'].get('p3d_mean'))} м."))
    if (tm0.get("judge_raw_3d_mean") or 0) > 1000:
        A(item("Система судьи", "Выход Runner не в MGRS судьи (старая версия пакета - equirect от "
                                "начала): без перевода средняя 3D у судьи ≈ "
                                f"{f(tm0.get('judge_raw_3d_mean') / 1000, 0)} км."))
    bg = meta.get("boundary_grid") or M.BOUNDARY_GRID
    if meta.get("judge_grid"):
        A(item("Система судьи", f"Сырые x, y, z выхода против эталона MGRS от {meta['judge_grid']} "
                                f"непрерывно, точка {rp}: средняя 3D {f(tm0.get('judge_raw_3d_mean'), 2)} м, "
                                f"пар с ошибкой > 1 км: {tm0.get('judge_raw_km', 0)}; шагов без "
                                "положения после первого опубликованного: "
                                f"{tm0.get('p_gap_steps', '-')}."
               + (f" Эталон base_link лежит на pathgraph: медиана |поперёк| "
                  f"{f(tm0.get('pg_ref_lat_med'), 2)} м, высота эталона − pathgraph медиана "
                  f"{f(tm0.get('pg_ref_dz_med'), 2, True)} м (средние по прогонам)."
                  if tm0.get("pg_ref_lat_med") is not None else "")))
    elif "bx_wrap_grid_3d_mean" in tm0:
        A(item("Граница квадратов MGRS", "E = 400 км, запад в 37U CB. Средняя 3D модели при "
                                         "сочетании «наш выход × соглашение судьи»: перенос × "
                                         f"перенос {f(tm0.get('bx_wrap_wrap_3d_mean'))} м, {bg} × "
                                         f"{bg} {f(tm0.get('bx_grid_grid_3d_mean'))} м, **перенос × "
                                         f"{bg} {f(tm0.get('bx_wrap_grid_3d_mean'), 0)} м, {bg} × "
                                         f"перенос {f(tm0.get('bx_grid_wrap_3d_mean'), 0)} м** (пар "
                                         f"с ошибкой > 1 км: {tm0.get('bx_wrap_grid_km', 0)} и "
                                         f"{tm0.get('bx_grid_wrap_km', 0)} из "
                                         f"{tm0.get('p_pairs', 0)}). Несовпадение соглашений стоит "
                                         "около 100 км всей западной части пути; при совпадении "
                                         "добавляются только пары у самой границы (перенос × "
                                         f"перенос: {tm0.get('sq_mismatch', 0)}; раздел 3.2)."))
    gf0 = S.get("gnss_full")
    if gf0:
        if gnss_corr(meta):
            A(item("GNSS весь прогон, коррекция по GNSS", "Сетка и скорость совпали с режимом "
                   f"«GNSS 3 с» в {gf0.get('same_speed_runs')} из {len(gf0.get('runs', {}))} "
                   f"прогонов; 3D ср. {f(g(gf0, 'full', 'p3d_mean'), 2)} м против "
                   f"{f(g(gf0, 'gnss3', 'p3d_mean'), 2)} м (первые {f(gf0['span_s'] / 60, 0)} мин)."))
        else:
            A(item("GNSS весь прогон", f"Выход совпал с режимом «GNSS 3 с» в "
                   f"{gf0.get('identical_runs')} из {len(gf0.get('runs', {}))} прогонов; 3D ср. "
                   f"{f(g(gf0, 'full', 'p3d_mean'), 1)} м против {f(g(gf0, 'gnss3', 'p3d_mean'), 1)} м "
                   f"(первые {f(gf0['span_s'] / 60, 0)} мин)."))
    gsd = gnss_scenarios(root, args)[1]
    if gsd is not None:
        ga = {s: g(gsd[0], "totals", s, "30618") or {} for s in ("first3", "sparse", "full")}
        if all(ga.values()):
            def _p(s, arm):
                return f(g(ga[s], arm, "p3d_mean"), 2)
            A(item("GNSS посреди маршрута", "Вагон 30618, 3D ср. без коррекции → с ней (раздел "
                   f"5.1): GNSS 3 с {_p('first3', 'before')} → {_p('first3', 'after')} м, редкие "
                   f"пачки {_p('sparse', 'before')} → {_p('sparse', 'after')} м, весь прогон "
                   f"{_p('full', 'before')} → {_p('full', 'after')} м. Скорость от GNSS после "
                   "окна не зависит."))
    summ0 = inject_summary(result["inject"], result["kinds"])
    if summ0:
        bad = [f"{r['kind']} - {verdict(r).replace('; ', ', ')}" for r in summ0
               if verdict(r) not in OK_VERDICTS]
        A(item("Инъекции", (("Требуют внимания (раздел 6): " + "; ".join(bad)) if bad else
                            "Все виды в пределах нормы (раздел 6)") + "."))
    pr0 = timing["realtime"] if timing.get("realtime") is not None else probe_rows(root, args.probe_glob)
    if pr0:
        A(item("Реальное время", "Замеры ноды (раздел 7): " + "; ".join(
            f"`{r[0]}` - {r[2]} Гц, in2out p50 {r[3]} мс, p99 {r[4]} мс, макс {r[5]} мс, CPU ср. "
            f"{r[6]} %, RSS {r[8]} МБ" for r in pr0) + "."))
    A("")
    B, rel = load_baseline(root, args)
    if B is not None:
        L += before_after(S, B, rel)

    # ---------------- как запустить
    A("## 1. Как запустить")
    A("")
    A(wrap("Команды выполняются из корня репозитория. Каталог с bag задаёт `DATA_DIR` в `.env`, "
           "кэш прогонов строится из `data/` сам, ключи после `eval` уходят в `tools/eval.py`."))
    A("")
    keys, blabel = doc_keys(args, meta, rel)
    A("Посчитать оценку с ключами этого документа и записать его.")
    A("")
    L += bash_block(["docker compose run --rm eval"]
                    + (["--variants"] if meta.get("variants_on") else []) + keys
                    + ["--label " + shq(meta.get("label") or "<подпись версии>")] + blabel)
    A("")
    A("Собрать образ `vectra/tram:compose` для запуска без Compose.")
    A("")
    A("```bash")
    A("docker compose build")
    A("```")
    A("")
    A("Посчитать оценку без Compose в PowerShell.")
    A("")
    A("```powershell")
    A("docker run --rm --cpus 2 -v ${PWD}:/repo -v <каталог с bag>:/repo/data:ro -w /repo `")
    A("    vectra/tram:compose python3 tools/eval.py --label \"<подпись версии>\"")
    A("```")
    A("")
    A("Основные опции `tools/eval.py`, полный список - `python3 tools/eval.py --help`:")
    A("")
    A(table(["Опция", "Что делает"], [
        ["`--sheet`", "лист: `eval` - оценочный из `config/eval/`, только train (по умолчанию); "
         "`jury` - боевой `config/tram.yaml`; `json`; путь"],
        ["`--set k=v,...`", "переопределить поля листа и параметры ноды: `--set mgrs_grid=37UDB`"],
        ["`--map`", "карта: `eval` - оценочная `config/eval/track_map.npz` (по умолчанию); `train` - "
         "собрать по train; `jury`; `none`; путь"],
        ["`--baseline`, `--baseline-label`", "итоги другой версии (`summary.json`) и её подпись: "
         "раздел 0 «До и после» перед разделом 1; по умолчанию `none` - без раздела"],
        ["`--gnss`", "секунд GNSS в связку, по умолчанию `3`; `full` - весь прогон; или сценарий "
         "доступности `tools/inject.py`"],
        ["`--gnss-scenarios`", "итоги `tools/eval_gnss.py` для раздела 5.1; "
         "`--gnss-scenarios-before` - итоги другой версии для столбца «до»"],
        ["`--frame`", "система эталона: `mgrs` (по умолчанию, как у судьи), `enu`, `equirect`, `utm`"],
        ["`--judge-grid`", "соглашение судьи: `37UCB` (по умолчанию) - от угла квадрата непрерывно; "
         "`\"\"` - перенос по точке"],
        ["`--ref-point`", "точка эталона: `base_link` (по умолчанию, по tf антенн) или `master` "
         "(антенна)"],
        ["`--pathgraph`", "pathgraph организаторов: `auto` - `_incoming/pathgraph`, если есть (в git "
         "его нет); `none`; путь"],
        ["`--variants`", "варианты листа для раздела 4: +15 прогонов модели"],
        ["`--gnss-full-runs`", "прогоны проверки «GNSS весь прогон», по умолчанию каждый 3-й; "
         "список или `all`"],
        ["`--quick`", "короткая проверка: 2 прогона по 300 с, 4 инъекции; документ - в "
         "`<out>/EVAL.md`, `docs/` не трогает"],
        ["`--check-determinism`", "второй проход и побайтное сравнение JSON; при расхождении код "
         "выхода 2"],
        ["`--render-only`", "пересобрать документ и графики из `out/eval/*.json` и `plotdata.npz` "
         "без прогонов"],
        ["`--cache`, `--data`, `--out`", "каталоги, по умолчанию `analysis/cache`, `data`, `out/eval`"],
        ["`--no-inject`, `--no-gnss-full`, `--no-doc`", "не считать инъекции или «GNSS весь "
         "прогон», не писать документ"],
    ], "ll"))
    A("")
    A("Пересобрать этот документ и графики из готовых JSON без прогонов.")
    A("")
    L += bash_block(["docker compose run --rm eval --render-only"] + keys + blabel)
    A("")
    A("Прогнать тесты инструментов оценки.")
    A("")
    A("```bash")
    A("python3 -m pytest tools/eval_selftest.py -q")
    A("```")
    A("")
    wall = timing.get("wall_all_s")
    A(item("Время", f"Этот прогон: {f(wall / 60 if wall else None, 1)} мин без графиков и документа "
                    f"(они около 0,5 мин), процессов {timing.get('workers')}"
                    f"{', с `--variants`' if meta.get('variants_on') else ''}. На `--cpus 2` на "
                    "свободной машине: 6,4 мин "
                    "без `--variants`, 8,5 мин с ним; при соседних контейнерах (около 1,3 ядра) - "
                    "18,0 мин без `--variants`."))
    A(item("Кэш и карта", "Кэш `analysis/cache/<bag>.npz` строится из `data/`, если его нет. Карта по "
                          "умолчанию - оценочная карта пакета `config/eval/track_map.npz`; с `--map "
                          "train` её строит `analysis/build_map.py` по 98 прогонам `train` (около "
                          "2 мин)."))
    A(item("Проверки после прогона", "В шапке нет предупреждения о параметрах ноды. «Взгляд судьи» "
                                     "в разделе 3.2 равен ошибке в MGRS. В разделе 5 сетка и скорость "
                                     "с GNSS весь прогон совпадают с GNSS 3 с во всех прогонах."))
    A(item("Воспроизводимость", "Два полных прогона дают побайтно одинаковые `summary.json`, "
                                "`runs.json` и `inject.json` (проверяет `--check-determinism`). "
                                "Чистый клон с пустым кэшем даёт тот же sha256 `runs.json`."))
    A("")
    A("Результаты прогона:")
    A("")
    A(table(["Файл", "Что внутри"], [
        [f"`{args.out}/summary.json`", "итоги"],
        [f"`{args.out}/runs.json`", "метрики по прогонам"],
        [f"`{args.out}/inject.json`", "инъекции аномалий"],
        [f"`{args.out}/timing.json`", "время, sha256 JSON, git, строки реального времени; не "
                                      "детерминирован"],
    ], "ll"))
    A("")
    shas = timing.get("sha256") or {}
    if shas:
        A(wrap("Начало sha256 JSON этого прогона: "
               + ", ".join(f"`{k}` {v[:12]}" for k, v in shas.items()) + "."))
    det = timing.get("determinism")
    if det:
        A(wrap("Детерминизм (`--check-determinism`): JSON двух проходов "
               + ("совпали побайтно." if det["identical"] else f"РАЗЛИЧАЮТСЯ: {det['files']}.")))
    A("")

    # ---------------- методика
    A("## 2. Методика")
    A("")
    A(item("Прогоны", f"`tools/split.json:holdout_scored` - {len(ids)} чистых отложенных записей "
                      "без дублей и копий в обучении."))
    gl = meta.get("glue") or {}
    A(item("Связка", "Запись проигрывается в порядке bag через `Runner` пакета, как в ноде "
                     "(`tools/eval_replay.py`). GNSS в связку - первые 3 с от первой точки master, как "
                     "в проверочных bag. Статус NavSatFix в `on_fix`: "
           + ("передаётся" if gl.get("fix_status") else "нет (Runner его не принимает)")
           + "; стартовый всплеск "
           + (f"сортируется в окне {f(gl.get('start_sort_s'), 2)} с"
              if gl.get("start_sort_s") is not None else "не сортируется")
           + ". Пульс ноды не эмулируется: он публикует те же узлы сетки с теми же значениями, "
             "кроме ≤ 2 с после последнего входа. Исключение в связке считается падением ноды."))
    A(item("Пары", "Выход и эталон сопоставляются по ближайшей метке `header.stamp` в пределах "
                   "0,05 с (`docs/DATASET_README.md`, 5.1). Положение учитывается только "
                   "опубликованное (`pos_valid`, конечные x, y, z)."))
    A(item("Эталон скорости", "Основной - |v| GNSS master по (x, y), дополнительный - rover. Фаза: "
                              "стоянка, если |v| GNSS < 0,2 м/с, иначе по ручке: > 0 тяга, < 0 "
                              "торможение, 0 выбег. Ложная стоянка - режим STANDSTILL (у базы "
                              "v < 0,3 м/с) при |v| GNSS > 0,5 м/с."))
    if (meta.get("ref_point") or "master") == "base_link":
        A(item("Эталон положения", "MGRS от угла квадрата 37UCB непрерывно, как pathgraph "
                                   "организаторов: x = E − 300 000, y = N − 6 100 000, z - высота. "
                                   "Точка - base_link, ось поворота передней тележки на уровне рельса. "
                                   "По паре антенн одной эпохи (±0,05 с, база 5–25 м) base_link = "
                                   "master + 9,873/12,436 · (rover − master), z - по той же доле между "
                                   "высотами антенн минус 3,0; без пары - "
                                   "master + 9,873 м по курсу. tf антенн: master (−9,873; 0; 3,0), "
                                   "rover (2,563; 0; 3,0). Ошибки считаются в непрерывных UTM зоны 37: "
                                   "та же система со сдвигом."))
    else:
        A(item("Эталон положения", "Антенна master (прежний эталон, `--ref-point master`), плоские "
                                   "MGRS; ошибки в непрерывных координатах UTM зоны 37."))
    A(item("Pathgraph организаторов", "Эталон и выход проецируются на путь своего направления (курс "
                                      "эталона ±90°): поперёк - расстояние выхода до оси, вдоль - "
                                      "разность дуговых координат. Пары, где эталон дальше 10 м от "
                                      "пути или за концом pathgraph, не входят."))
    A(item("Граница квадратов 100 км", "Линия пересекает E = 400 км: запад (около 1,2 км) в 37U CB, "
                                       "остальное в 37U DB. Для справки считается и «перенос по "
                                       "точке», где x скачет на 100 км: несовпадение квадрата, матрица "
                                       "2×2 «наше соглашение × соглашение судьи» (перенос по точке / "
                                       "непрерывно от "
                                       f"`{meta.get('boundary_grid') or M.BOUNDARY_GRID}`) и «взгляд "
                                       "судьи» - сырые x, y, z выхода против эталона."))
    A(item("Вдоль и поперёк пути", "Проекция на ломаную эталона: медиана по 5 фиксам, шаг 1 м, окно "
                                   "±1 км, то же направление движения (`tools/core_metrics.py`). "
                                   "Дрейф - ошибка в конце прогона, отнесённая к длине пути (ТЗ, "
                                   "стр. 6)."))
    A(item("Итоги", "MAE, смещение и RMSE - по всем парам всех прогонов вместе; максимумы - по всем "
                    "прогонам; дрейф - среднее, медиана и максимум по прогонам."))
    A(item("База «только колесо»", "Среднее последних показаний тележек (не старше 1 с) × "
                                   "`meas_scale` / 3,6, путь - интеграл на той же сетке 50 мс; "
                                   "положение - та же машинерия `Runner`, карты и привязки к "
                                   "остановкам."))
    A(item("Что утекает", (meta["sheet"]["leak"] + ". " if meta["sheet"].get("leak") else
                           "Лист - оценочный, только train. ")
           + (meta["map"]["leak"] + ". " if meta["map"].get("leak") else "Карта - только по train. ")
           + ("Множитель пути карты (`calibrate_scale`) считается с `meas_scale` оцениваемого листа."
              if meta.get("build_map_cli") == "argparse" else
              "Множитель пути карты (`calibrate_scale`) считается с `meas_scale` из "
              "`tram_calibration.json` (все данные) - утечка порядка 0,01 %.")))
    A(item("Инъекции", "Шум задан абсолютно (σ = 0,25 м/с), от листа не зависит. Связка с аномалией "
                       "- копия чистой связки, снятая за 35 с до аномалии."))
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
                     f(t.get("v_max"), 2), pct(t.get("cov2s_v")) if "cov2s_v" in t else "-",
                     pct(t.get("false_ss_rate"), 3), f(t.get("v_pairs"), 0) if t.get("v_pairs") else "-",
                     pct(t.get("v_pair_frac"), 2), t.get("v_out_nan", 0)])
    A(table(["Оценка", "RMSE, м/с", "MAE, м/с", "Смещение, м/с", "Макс, м/с", "±2σ", "Ложные стоянки",
             "Пар", "Доля пар", "NaN в выходе"], rows))
    A("")
    A(wrap("Доля пар - от меток GNSS master vel. Падения связки на чистых прогонах: " +
           (", ".join(f"{b} ({e}: {c.get('error', '')[:60]})" for b, cs in (S.get("crashes") or {}).items()
                      for e, c in cs.items() if c) or "нет") + "."))
    A("")
    rv = []
    for name, est in (("модель", "model"), ("база", "naive")):
        rr = [result["runs"][b][est] for b in ids if est in result["runs"].get(b, {})]
        rows_r = [dict(v_pairs=r.get("rover_v_pairs", 0), v_mae=r.get("rover_v_mae"),
                       v_rmse=r.get("rover_v_rmse"), v_bias=r.get("rover_v_bias"))
                  for r in rr if r.get("rover_v_pairs")]
        t = M.totals(rows_r) or {}
        rv.append([name, f(t.get("v_rmse"), 4), f(t.get("v_mae"), 4), f(t.get("v_bias"), 4, True)])
    A("Против GNSS rover - дополнительного эталона (итог взвешен парами rover):")
    A("")
    A(table(["Оценка", "RMSE, м/с", "MAE, м/с", "Смещение, м/с"], rv))
    A("")
    A("По фазам движения (все пары master вместе):")
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
    A(table(["Фаза", "Доля", "Модель MAE", "Модель смещение", "Модель ±2σ", "База MAE",
             "База смещение"], rows))
    A("")
    rc = g(S, "pooled", "model", "ref_clean") or {}
    rcn = g(S, "pooled", "naive", "ref_clean") or {}
    A(wrap(f"Без выбросов самого эталона (GNSS против обеих согласных тележек > 1 м/с, "
           f"{rc.get('glitch_samples', 0)} пар): модель MAE {f(rc.get('mae'), 4)}, RMSE "
           f"{f(rc.get('rmse'), 4)}; база MAE {f(rcn.get('mae'), 4)}, RMSE {f(rcn.get('rmse'), 4)}."))
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
    A("Ошибка 3D и дрейф (метры, дрейф - % пути):")
    A("")
    rows = []
    for name, t in (("модель", tm), ("база «только колесо»", tn)):
        rows.append([name, f(t.get("p3d_mean")), f(t.get("p3d_rmse")), f(t.get("p3d_max"), 1),
                     f(t.get("drift_pct_3d_median"), 3) + " / " + f(t.get("drift_pct_3d_max"), 3),
                     f(t.get("drift_pct_along_median"), 3) + " / " + f(t.get("drift_pct_along_max"), 3)])
    A(table(["Оценка", "3D ср.", "3D RMSE", "3D макс", "Дрейф 3D, % мед. / макс",
             "Дрейф вдоль, % мед. / макс"], rows))
    A("")
    A("Вдоль и поперёк пути, м:")
    A("")
    rows = []
    for name, t in (("модель", tm), ("база «только колесо»", tn)):
        rows.append([name, f(t.get("along_mean")), f(t.get("along_rmse")), f(t.get("along_max"), 1),
                     f(t.get("cross_mean")), f(t.get("cross_max"), 1)])
    A(table(["Оценка", "Вдоль ср. абс.", "Вдоль RMSE", "Вдоль макс", "Поперёк ср.", "Поперёк макс"],
            rows))
    A("")
    A(wrap(f"Высота: средняя |Δz| модели {f(tm.get('pz_mean'))} м, в плане (2D) "
           f"{f(tm.get('p2d_mean'))} м. Покрытие |вдоль| ≤ 2σ_s: {pct(tm.get('cov2s_along'))}. Пар "
           f"без проекции (оценка дальше 60 м от пути): {tm.get('along_undef', 0)}."))
    A("")
    A("Полнота положения: фикс сопоставляется с опубликованным положением в пределах 0,05 с.")
    A("")

    def rate(est):
        return (f"{f(run_mean(result, ids, est, 'rate_hz'), 2)} / "
                f"{f(run_mean(result, ids, est, 'rate_pos_hz'), 2)}")
    A(table(["Показатель", "Модель", "База «только колесо»"], [
        ["меток эталона", tm.get("p_ref", "-"), tn.get("p_ref", "-")],
        ["пар", tm.get("p_pairs", "-"), tn.get("p_pairs", "-")],
        ["доля пар", pct(tm.get("p_pair_frac"), 2), pct(tn.get("p_pair_frac"), 2)],
        ["строк выхода", tm.get("p_out", "-"), tn.get("p_out", "-")],
        ["без положения (pos_valid)", tm.get("p_out_invalid", 0), tn.get("p_out_invalid", 0)],
        ["из них после первого опубликованного", tm.get("p_gap_steps", "-"), tn.get("p_gap_steps", "-")],
        ["положение NaN", tm.get("p_nan", 0), tn.get("p_nan", 0)],
        ["частота выхода / положения, Гц (ср. по прогонам)", rate("model"), rate("naive")],
    ]))
    A("")
    if tm.get("pg_cross_mean") is not None:
        pgm = meta.get("pathgraph") or {}
        lens = [f(x, 0) for x in pgm.get("length_m", [])]
        lens = " и ".join([", ".join(lens[:-1]), lens[-1]] if len(lens) > 1 else lens) or "-"
        A(wrap(f"**По pathgraph организаторов** (`{pgm.get('src', '')}`, путей {pgm.get('paths', '-')} "
               f"длиной {lens} м):"))
        A("")
        A(table(["Показатель", "Модель", "База «только колесо»"], [
            ["эталон на pathgraph", pct(tm.get("pg_frac"), 1), pct(tn.get("pg_frac"), 1)],
            ["пар", tm.get("pg_pairs", "-"), tn.get("pg_pairs", "-")],
            ["поперёк ср., м", f(tm.get("pg_cross_mean"), 3), f(tn.get("pg_cross_mean"), 3)],
            ["поперёк 95 %, м", f(tm.get("pg_cross_p95")), f(tn.get("pg_cross_p95"))],
            ["поперёк макс, м", f(tm.get("pg_cross_max"), 1), f(tn.get("pg_cross_max"), 1)],
            ["вдоль ср. абс., м", f(tm.get("pg_along_mean")), f(tn.get("pg_along_mean"))],
            ["вдоль RMSE, м", f(tm.get("pg_along_rmse")), f(tn.get("pg_along_rmse"))],
            ["вдоль смещение, м", f(tm.get("pg_along_bias"), 2, True), f(tn.get("pg_along_bias"), 2, True)],
            ["3D ср. на pathgraph, м", f(tm.get("p3d_on_pg")), f(tn.get("p3d_on_pg"))],
            ["3D ср. за его концами, м", f(tm.get("p3d_off_pg")), f(tn.get("p3d_off_pg"))],
        ]))
        A("")
        A(wrap(f"Сам эталон base_link (GNSS) против pathgraph: медиана |поперёк| "
               f"{f(tm.get('pg_ref_lat_med'), 3)} м (со знаком {f(tm.get('pg_ref_lat_signed_med'), 3, True)}, "
               f"95 % - {f(tm.get('pg_ref_lat_p95'), 2)} м), высота эталона − z pathgraph медиана "
               f"{f(tm.get('pg_ref_dz_med'), 2, True)} м (средние медиан по прогонам). Значит, "
               "pathgraph - ось пути точки base_link на уровне рельса."))
        A("")
    bg = meta.get("boundary_grid") or M.BOUNDARY_GRID
    A(wrap(f"**Квадраты MGRS.** Пар, где оценка при «переносе по точке» попала бы в другой квадрат "
           f"100 км, чем эталон: модель {tm.get('sq_mismatch', 0)} из {tm.get('p_pairs', 0)}, база "
           f"{tn.get('sq_mismatch', 0)}. Справочно - матрица «наш выход × соглашение судьи» (средняя "
           "3D модели, м; в скобках - пар с ошибкой > 1 км):"))
    A("")
    lab = {"wrap": "перенос по точке", "grid": f"непрерывно от {bg}"}
    rows = []
    for o in ("wrap", "grid"):
        rows.append([f"наш выход: {lab[o]}"] + [
            f"{f(tm.get(f'bx_{o}_{j}_3d_mean'), 1 if (tm.get(f'bx_{o}_{j}_3d_mean') or 0) < 1000 else 0)} "
            f"({tm.get(f'bx_{o}_{j}_km', 0)})" for j in ("wrap", "grid")])
    A(table(["", f"Судья: {lab['wrap']}", f"Судья: {lab['grid']}"], rows))
    A("")
    A(wrap("При несовпадении соглашений часть пути по другую сторону E = 400 км получает ошибку около "
           "100 км. Соглашение выхода задаёт параметр ноды `mgrs_grid`: по умолчанию `37UCB`, как "
           "pathgraph; `\"\"` - перенос по точке."))
    A("")
    A(wrap("«Взгляд судьи» (сырые x, y, z выхода Runner против эталона MGRS "
           f"{'с переносом по точке' if not meta['judge_grid'] else 'от ' + meta['judge_grid']}): "
           f"средняя 3D **{f(tm.get('judge_raw_3d_mean'), 1)} м**, максимум "
           f"{f(tm.get('judge_raw_3d_max'), 1)} м, пар с ошибкой > 1 км: {tm.get('judge_raw_km', 0)}"
           + (" - выход Runner не в MGRS судьи (старая версия пакета - equirect от начала): без "
              "перевода судья увидел бы ошибку около 100 км."
              if (tm.get("judge_raw_3d_mean") or 0) > 1000 else ".")))
    A("")
    A("Чувствительность к системе эталона (тот же выход, другой эталон; 3D ср. / конец ср.):")
    A("")
    rows = [["MGRS (UTM)", f(tm.get("p3d_mean")) + " / " + f(tm.get("p3d_end_mean")),
             f(tn.get("p3d_mean")) + " / " + f(tn.get("p3d_end_mean"))]]
    for fr, lab in (("enu", "строгий ENU"), ("equirect", "плоский equirect")):
        a_, b_ = g(S, "frame_sensitivity", fr) or {}, g(S, "frame_sensitivity", fr + "_naive") or {}
        if a_:
            rows.append([lab, f(a_.get("p3d_mean")) + " / " + f(a_.get("p3d_end_mean")),
                         f(b_.get("p3d_mean")) + " / " + f(b_.get("p3d_end_mean"))])
    A(table(["Эталон", "Модель, м", "База, м"], rows))
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
    A(table(["Вагон", "Оценка", "Прогонов", "MAE, м/с", "Смещение, м/с", "3D ср., м", "Вдоль RMSE, м",
             "Дрейф 3D, % мед."], rows, "llrrrrrr"))
    A("")
    L += vehicle_detail(S, "30618")

    # ---------------- по прогонам
    A("### 3.4. По прогонам")
    A("")
    A("Скорость, м/с:")
    A("")
    rows = []
    for b in ids:
        rm, rn = result["runs"][b].get("model", {}), result["runs"][b].get("naive", {})
        rows.append([b, f(rm.get("path_m", 0) / 1000.0, 2), rm.get("v_pairs", "-"), f(rm.get("v_mae"), 4),
                     f(rn.get("v_mae"), 4), f(rm.get("v_bias"), 4, True), pct(rm.get("cov2s_v"), 0)])
    A(table(["Прогон", "Путь, км", "Пар v", "MAE", "MAE база", "Смещение", "±2σ"], rows))
    A("")
    A("Положение, м (дрейф - % пути):")
    A("")
    crash_col = any(result["runs"][b].get("model", {}).get("crash") for b in ids)
    rows = []
    for b in ids:
        rm, rn = result["runs"][b].get("model", {}), result["runs"][b].get("naive", {})
        rows.append([b, f(rm.get("p3d_mean")), f(rn.get("p3d_mean")), f(rm.get("p3d_end")),
                     f(rm.get("along_end"), 1, True), f(rm.get("drift_pct_3d"), 3),
                     f(rm.get("pg_cross_mean"), 2), rm.get("p_gap_steps", "-")]
                    + (["упала" if rm.get("crash") else ""] if crash_col else []))
    A(table(["Прогон", "3D ср.", "3D база", "3D конец", "Вдоль конец", "Дрейф 3D, %",
             "Поперёк pathgraph", "Шагов без положения"] + (["Падение"] if crash_col else []), rows))
    A("")

    # ---------------- варианты листа
    A("## 4. Заглушки крипа против подогнанных / нулевых")
    A("")
    A("Тот же расчёт с другими значениями полей крипа в листе (ключ `--variants`).")
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
        rows.append(["-", "не считалось (включается ключом --variants; в --quick не считается)",
                     "", "", "", ""])
    A(table(["Вариант", "Поля", "MAE, м/с", "Смещение, м/с", "±2σ", "3D ср., м"], rows, "llrrrr"))
    A("")

    # ---------------- GNSS весь прогон
    A("## 5. GNSS весь прогон (bag жюри с полным GNSS)")
    A("")
    gf = S.get("gnss_full")
    if gf:
        a_, b_ = gf.get("full") or {}, gf.get("gnss3") or {}
        n = len(gf.get("runs", {}))
        A(wrap(f"Первые {f(gf['span_s'] / 60, 0)} мин записи {n} прогонов "
               f"({', '.join(sorted(gf.get('runs', {})))}; `--gnss-full-runs all` - все), только "
               "модель. "
               + ("С коррекцией по GNSS (`gnss_correction: true`, по ответу организаторов) положение "
                  "после окна следует за GNSS, а сетка, скорость и признак публикации положения "
                  "должны совпасть с режимом «GNSS 3 с»."
                  if gnss_corr(meta) else
                  "Без коррекции (`gnss_correction: false`) GNSS нужен только для выставки, и выход "
                  "должен совпасть с режимом «GNSS 3 с» целиком.")))
        A("")
        A(table(["GNSS в связку", "MAE, м/с", "3D ср., м"], [
            ["весь прогон", f(a_.get("v_mae"), 4), f(a_.get("p3d_mean"), 1)],
            ["первые 3 с", f(b_.get("v_mae"), 4), f(b_.get("p3d_mean"), 1)]]))
        A("")
        if gnss_corr(meta):
            A(wrap(f"Сетка, pos_valid и скорость совпали (|Δv| ≤ 1e-9) в **{gf.get('same_speed_runs')} "
                   f"из {n}** прогонов, наибольшее |Δv| {f(gf.get('max_dv'), 3)} м/с; наибольшее "
                   f"|Δ положения| {f(gf.get('max_dpos'), 1)} м - поправки по GNSS."))
        else:
            A(wrap(f"Совпали (та же сетка и pos_valid, |Δv| ≤ 1e-9, |Δxyz| ≤ 1e-6) "
                   f"**{gf.get('identical_runs')} из {n}** прогонов; наибольшее |Δv| "
                   f"{f(gf.get('max_dv'), 3)} м/с, наибольшее |Δ положения| {f(gf.get('max_dpos'), 1)} м."))
        A("")
        rows = [[b, "да" if r["same_grid"] else f"нет ({r['n_out_full']} / {r['n_out_3s']})",
                 f(r.get("max_dv"), 3), f(r.get("max_dpos"), 1), f(r.get("p3d_mean_full"), 1),
                 f(r.get("p3d_mean_3s"), 1)] for b, r in gf.get("runs", {}).items()]
        A(table(["Прогон", "Та же сетка", "Макс |Δv|, м/с", "Макс |Δ положения|, м", "3D ср. (весь), м",
                 "3D ср. (3 с), м"], rows, "llrrrr"))
    else:
        A("Не считалось (`--gnss full` уже основной режим, `--no-gnss-full` или `--quick`).")
    A("")
    L += gnss_scenarios(root, args)[0]

    # ---------------- инъекции
    A("## 6. Инъекции аномалий")
    A("")
    inj = [x for x in result["inject"] if not x.get("skipped")]
    bags = sorted({x["bag"] for x in result["inject"]})
    A(item("Что меняется", f"`tools/inject.py` меняет входной поток реальной отложенной записи "
                           f"({', '.join(bags)}); GNSS не трогается."))
    A(item("Окна", "Аномалия - в первом окне после 60 с, где выполнено условие вида (скорость, "
                   "ручка) и до конца записи остаётся ≥ 120 с. Метрики: «до» - 30 с перед "
                   "аномалией, «во время» - окно оценки вида, «после» - 60 с после него."))
    A(item("Восстановление", "Через сколько секунд после конца аномалии скорость совпадает с чистым "
                             f"прогоном (|Δv| ≤ {f(0.1, 1)} м/с не меньше 3 с)."))
    A(item("Остаток пути", "«Δ вдоль через 300 с» - ошибка вдоль пути через 300 с после окна минус "
                           "ошибка чистого прогона: снимает ли её привязка к остановке."))
    A("")
    A(table(["Вид", "Описание", "Длит., с"],
            [[k, I.KINDS[k]["ru"], f(I.KINDS[k]["dur"], 1)] for k in result["kinds"]], "llr"))
    A("")
    summ = inject_summary(result["inject"], result["kinds"])
    if summ:
        A(wrap("**Сводка по видам.** MAE - среднее по прогонам во время аномалии, м/с; |Δ вдоль| - "
               "наибольший через 300 с; флаги модели во время аномалии - наименьшая доля valid, "
               "наибольшие доли ambiguous и slip; ±2σ - доля пар, где ошибка внутри ±2σ."))
        A("")
        rows = []
        for r in summ:
            m, n, fl = r["model"], r["naive"], r["flag"]
            rows.append([r["kind"], r["n"], f(m["mae"], 3), f(n["mae"], 3), f(m["tail"], 1), f(n["tail"], 1),
                         f"{pct(fl['valid'], 0)} / {pct(fl['amb'], 0)} / {pct(fl['slip'], 0)}",
                         pct(fl["cov2s"], 0), verdict(r)])
        # заголовок и порядок столбцов читает simulator/test/sandbox_report.js (столбец
        # «на реальных записях» в docs/SANDBOX.md): без правки скрипта их не менять
        A(table(["вид", "прогонов", "MAE модели", "MAE базы", "|Δ вдоль| модели, м", "|Δ вдоль| базы, м",
                 "valid / amb / slip", "±2σ", "вывод"], rows, "lrrrrrrrl"))
        A("")
    rows = []
    for x in inj:
        em, en = x["est"].get("model") or {}, x["est"].get("naive") or {}
        crash = em.get("crash")
        dur = em.get("during", {})
        flags = ""
        if dur.get("frac_valid") is not None:
            flags = (f"valid {pct(dur.get('frac_valid'), 0)}, amb {pct(dur.get('frac_amb'), 0)}, "
                     f"slip {pct(dur.get('frac_slip'), 0)}, σv≤{f(dur.get('sigma_v_max'), 1)}")
        rows.append([
            f"{x['kind']} · {x['bag'][-8:]}",
            " / ".join(f(g(em, w, "v_mae"), 3) for w in ("before", "during", "after")),
            "упала" if en.get("crash") else f(g(en, "during", "v_mae"), 3),
            f(g(em, "during", "along_end"), 1, True) + " ("
            + f(g(em, "during", "along_end_clean"), 1, True) + ") / " + f(g(em, "after", "along_end"), 1, True),
            f(em.get("d_along_tail"), 1, True) + " / " + f(en.get("d_along_tail"), 1, True),
            f(em.get("recovery_s"), 1) if em.get("recovery_s") is not None else ("-" if crash else "нет"),
            ("**упала** " + f(crash.get("after_t0_s"), 2) + " с: " + crash["error"][:60]) if crash else flags,
            pct(dur.get("cov2s"), 0) if dur.get("cov2s") is not None else "",
        ])
    if rows:
        A("<details>")
        A("<summary>Подробно по прогонам</summary>")
        A("")
        A(table(["Вид · прогон", "MAE модели до / во время / после, м/с", "MAE базы во время, м/с",
                 "Вдоль модели: конец окна (чисто) / после, м", "Δ вдоль через 300 с: модель / база, м",
                 "Восст. модели, с", "Флаги модели", "±2σ"], rows, "lrrrrrlr"))
        A("")
        A("</details>")
        A("")
    skipped = [x for x in result["inject"] if x.get("skipped")]
    for x in skipped:
        A(item(f"{x['kind']} · {x['bag']}", x["skipped"]))
    if skipped:
        A("")
    for p in pics.get("inj", []):
        A(f"![Инъекции](img/{p})")
        A("")

    # ---------------- реальное время
    A("## 7. Реальное время")
    A("")
    # строки, собранные при прогоне (timing.json), - иначе --render-only не воспроизвёл бы
    # документ, когда файлов сводок уже нет; старый timing.json без них - из файлов
    pr = timing["realtime"] if timing.get("realtime") is not None else probe_rows(root, args.probe_glob)
    if pr:
        A(wrap("Сводки `tools/ros_probe.py`, найденные по `--probe-glob`. Частота - по меткам выхода. "
               "Задержка in2out - от приёма входа `/vehicle/*` до приёма первого выхода, который его "
               "учёл, по стенным часам пробы, включая DDS. CPU - % одного ядра."))
        A("")
        names = ("выходов", "частота, Гц", "in2out p50, мс", "in2out p99, мс", "in2out макс, мс",
                 "CPU ср., %", "CPU макс, %", "RSS, МБ", "рост RSS, МБ/мин")
        A(table(["Показатель"] + [f"`{r[0]}`" for r in pr],
                [[nm] + [r[i + 1] for r in pr] for i, nm in enumerate(names)]))
        A("")
        notes = (timing["realtime_notes"] if timing.get("realtime_notes") is not None
                 else probe_notes(root, args.probe_glob))
        for name, text in notes:
            A(item(f"`{name}`", text[:1].upper() + text[1:]))
        if notes:
            A("")
        A(wrap("Как мерить: `tools/measure_realtime.sh`, полный bag ≥ 20 мин, "
               "`--cpus 2 --memory 512m`, машина без соседей. Порядок замера и таблица критериев ТЗ - "
               "`docs/JURY.md` §6."))
    else:
        A(wrap("Сводок `tools/ros_probe.py` не найдено (`--probe-glob "
               f"{args.probe_glob}`). Замер - `tools/measure_realtime.sh`: полный bag ≥ 20 мин, "
               "`--cpus 2 --memory 512m`, машина без соседей. После замера перезапустить "
               "`tools/eval.py` - таблица подставится сюда."))
    A("")
    eps = timing.get("events_per_s")
    eps_txt = (f"{eps} " + plural_ru(eps, "входное сообщение", "входных сообщения", "входных сообщений")
               if isinstance(eps, int) else "- входных сообщений")
    A(wrap(f"Офлайн-скорость связки в этом прогоне - не замер ноды: {eps_txt} в секунду на процесс "
           "вместе с базой, при соседних контейнерах."))
    A("")
    A("## 8. Допущения")
    A("")
    A(item("Система координат эталона", "Координаты считаются от угла квадрата 37UCB без переноса "
                                        "на границе, точка - base_link: так построены pathgraph и "
                                        "пример координат организаторов. В оценке эталон base_link "
                                        "строится по двум антеннам и tf; на прямых участках это "
                                        "совпадает с построением по одной антенне и курсу."))
    A(item("Эталон скорости", "Официального эталона скорости нет. Основной эталон в оценке - "
                              "скорость GNSS master, контрольный - rover."))
    A(item("Сдвиг публикации скорости", "Здесь оценивается скорость ядра без сдвига. Нода "
                                        "публикует её с задержкой `speed_output_delay_s` 0,08 с: "
                                        "эталон проверки организаторов "
                                        "`/localization/kinematic_state` отстаёт от колёс примерно "
                                        "на 0,1 с. Итог проверки организаторов со сдвигом - "
                                        "[JURY.md](JURY.md), раздел 1а."))
    A("")
    lines = "\n".join(L).split("\n")
    i = lines.index(TOC_MARK)
    lines[i:i + 1] = toc(lines)
    return "\n".join(lines).rstrip("\n") + "\n"
