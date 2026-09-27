"""Шаг 2б: калибровка модели привода по данным.

Модель: ускорение на ровном пути a = A(u, v), где u - позиция ручки после
чистой задержки delay и апериодического звена tau (в позициях), v - скорость.
A - билинейная таблица по сетке позиций и скоростей, подбирается методом
наименьших квадратов с регуляризацией гладкости (пустые клетки получают
значения соседей). delay и tau - перебором: подгонка на части прогонов,
СКО ускорения на остальных (внутренняя проверка, каждый пятый из подгоночных).

Какие прогоны - по фиксированному разбиению tools/split.json:

    python3 analysis/calib_drive.py eval   # только обучающие (train_unique):
                                           # лист оценки, по нему считаем числа
    python3 analysis/calib_drive.py jury   # все уникальные записи
                                           # (train_unique + holdout): лист жюри

Отложенные прогоны (holdout) в лист оценки не попадают ни в подгонку, ни в
перебор delay и tau. Дубли записей в подгонку не идут дважды.

Выход: analysis/drive_model_<eval|jury>.json - таблица и константы для
calib_sheet.py. Старый analysis/drive_model.json не трогается: из него другие
инструменты берут прежние списки train/val.
"""

import json
import sys
from pathlib import Path

import numpy as np

import bagio
from drive_table import series, DT

SPLIT = Path(__file__).resolve().parent.parent / "tools" / "split.json"
DELAYS = (0.0, 0.1, 0.2, 0.3)
TAUS = (0.2, 0.3, 0.4, 0.6)


def fit_ids(fit):
    """Прогоны подгонки листа: eval - только обучающие уникальные записи,
    jury - все уникальные записи (обучающие + отложенные)."""
    d = json.loads(SPLIT.read_text(encoding="utf-8"))
    if fit == "eval":
        return sorted(d["train_unique"])
    if fit == "jury":
        return sorted(d["train_unique"] + d["holdout"])
    raise SystemExit(f"неизвестный лист: {fit} (eval | jury)")

U_GRID = np.arange(-15, 16, dtype=float)                 # позиции ручки
V_GRID = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0])


def split(ids):
    """Внутренняя проверка для перебора delay и tau: каждый пятый прогон."""
    val = set(ids[2::5])
    return [b for b in ids if b not in val], sorted(val)


def lagged_notch(t, c, delay, tau):
    tn = c[:, 0] + delay
    i = np.clip(np.searchsorted(tn, t, side="right") - 1, 0, len(c) - 1)
    raw = np.where(t >= tn[0], c[i, 2], 0.0)
    if tau <= 0:
        return raw
    out = np.empty_like(raw)
    k = DT / tau
    x = raw[0]
    for j, r in enumerate(raw):
        x += (r - x) * min(k, 1.0)
        out[j] = x
    return out


def hat(x, grid):
    """Индексы и веса билинейной интерполяции по одной оси."""
    x = np.clip(x, grid[0], grid[-1])
    j = np.clip(np.searchsorted(grid, x, side="right") - 1, 0, len(grid) - 2)
    w = (x - grid[j]) / (grid[j + 1] - grid[j])
    return j, w


def design(u, v):
    ju, wu = hat(u, U_GRID)
    jv, wv = hat(v, V_GRID)
    nv = len(V_GRID)
    cols = [(ju * nv + jv, (1 - wu) * (1 - wv)), (ju * nv + jv + 1, (1 - wu) * wv),
            ((ju + 1) * nv + jv, wu * (1 - wv)), ((ju + 1) * nv + jv + 1, wu * wv)]
    return cols


def normal_eq(u, v, a):
    n = len(U_GRID) * len(V_GRID)
    cols = design(u, v)
    AtA = np.zeros((n, n))
    Atb = np.zeros(n)
    for ci, wi in cols:
        Atb += np.bincount(ci, wi * a, minlength=n)
        for cj, wj in cols:
            AtA += np.bincount(ci * n + cj, wi * wj, minlength=n * n).reshape(n, n)
    return AtA, Atb


def smoothness(lam_u, lam_v):
    nu, nv = len(U_GRID), len(V_GRID)
    n = nu * nv
    R = []
    for i in range(nu):
        for j in range(nv - 1):
            r = np.zeros(n); r[i * nv + j] = 1; r[i * nv + j + 1] = -1
            R.append(r * lam_v)
    for i in range(nu - 1):
        if U_GRID[i] < 0 <= U_GRID[i + 1]:
            continue                       # тяга и тормоз не сглаживаются друг в друга
        for j in range(nv):
            r = np.zeros(n); r[i * nv + j] = 1; r[(i + 1) * nv + j] = -1
            R.append(r * lam_u)
    R = np.array(R)
    return R.T @ R


def predict(W, u, v):
    return sum(W[ci] * wi for ci, wi in design(u, v))


def samples(ids, delay, tau):
    U, V, A = [], [], []
    for b in ids:
        s = series(b)
        if s is None:
            continue
        t, v, acc, ok, c = s
        u = lagged_notch(t, c, delay, tau)
        m = ok & np.isfinite(acc) & (v > 0.3)
        # Противоречивые отсчёты - ошибки ручки в данных (разгон при тормозной
        # позиции и наоборот): модель по ним не учится.
        contra = (np.abs(acc) > 0.3) & (np.abs(u) >= 1) & (np.sign(acc) != np.sign(u))
        m &= ~contra
        U.append(u[m]); V.append(v[m]); A.append(acc[m])
    return map(np.concatenate, (U, V, A))


def fit(U, V, A, lam=(0.3, 0.3)):
    AtA, Atb = normal_eq(U, V, A)
    R = smoothness(*lam) * (len(A) / 1e4)
    return np.linalg.solve(AtA + R + 1e-9 * np.eye(len(Atb)), Atb)


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "eval"
    ids = fit_ids(which)
    tr, va = split(ids)
    print(f"лист {which}: подгонка {len(ids)} прогонов; перебор delay/tau: "
          f"обучение {len(tr)}, внутренняя проверка {len(va)}")
    best, grid = None, []
    for delay in DELAYS:
        for tau in TAUS:
            W = fit(*samples(tr, delay, tau))
            Uv, Vv, Av = samples(va, delay, tau)
            e = Av - predict(W, Uv, Vv)
            rmse = float(np.sqrt(np.mean(e ** 2)))
            grid.append(dict(delay=delay, tau=tau, rmse=rmse))
            print(f"  delay {delay:.1f} tau {tau:.1f}: СКО на проверке {rmse:.4f} м/с²")
            if best is None or rmse < best[0]:
                best = (rmse, delay, tau, W)
    rmse, delay, tau, W = best
    # итог - по всем прогонам подгонки с лучшими delay, tau
    W = fit(*samples(ids, delay, tau))
    T = W.reshape(len(U_GRID), len(V_GRID))
    print(f"\nлучшие: delay {delay} с, tau {tau} с, СКО на проверке {rmse:.4f} м/с²")
    print("таблица A(позиция, v), м/с²; столбцы - v, м/с:")
    print("      " + " ".join(f"{v:6.0f}" for v in V_GRID))
    for i, uu in enumerate(U_GRID):
        print(f"{uu:+4.0f}  " + " ".join(f"{x:+6.2f}" for x in T[i]))
    out = dict(fit=which, delay=delay, tau=tau, rmse_val=rmse, grid=grid,
               u_grid=U_GRID.tolist(), v_grid=V_GRID.tolist(),
               table=T.round(4).tolist(), fit_ids=ids, tune_train=tr, tune_val=va)
    path = bagio.CACHE.parent / f"drive_model_{which}.json"
    path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("записано:", path)


if __name__ == "__main__":
    main()
