"""Режим опознавания привода при вводе в эксплуатацию.

Зачем отдельный режим. Проверка показала, что в обычной эксплуатации
характеристика привода **не идентифицируема**: скорость растёт монотонно
вместе с тягой, регрессор вырожден, и адаптация на ходу даёт либо вздорные
веса, либо смещённые. Возбуждение надо задавать намеренно.

Процедура. На закрытом участке с хорошим сцеплением выполняется серия циклов
«разгон на фиксированной позиции — выбег», позиции берутся разные, а выбег
даёт чистое измерение сопротивления движению. Записи обрабатываются пакетно.

Что здесь принципиально: процедура **отказывается** выдавать результат, если
данные не обеспечивают идентифицируемости. Молча выдать смещённые числа —
худшее, что может сделать такая утилита.
"""

import numpy as np

from .estimator_core import DEFAULT, shape_tract, shape_brake


class Insufficient(Exception):
    """Данных не хватает для идентификации. Число обусловленности в тексте."""


class DriveIdentifier:
    """Пакетная идентификация тяговой и тормозной характеристик.

    Модель та же, что в ядре: F(u, v) = w * u * форма(v), два параметра.
    Уравнение корпуса интегрируется по окну, чтобы не дифференцировать
    квантованную одометрию:

        M (v2 - v1) = integral(F_привода) dt - integral(F_сопр) dt
    """

    def __init__(self, mass, params=None, window_s=2.0):
        self.p = params if params is not None else DEFAULT
        self.mass = float(mass)
        self.n_win = int(round(window_s / self.p.dt))
        self.dt = self.p.dt
        self._reset()
        self.rows, self.targets = [], []
        self.coast = []          # пары (v, замедление) с выбега

    N_PARAM = 4          # w_тяги, w_торможения, res_A, res_B

    def _reset(self):
        self.k = 0
        self.phi = np.zeros(self.N_PARAM)
        self.v0 = None
        self.u_ref = None

    # ---------- накопление ----------

    def add(self, u, v, adhesion_ok=True):
        """Один отсчёт. `u` — нормированный момент, `v` — скорость по одометрии."""
        if not adhesion_ok or abs(u) < self.p.T_dead:
            if abs(u) < self.p.T_dead and self.v0 is not None:
                pass
            self._collect_coast(u, v)
            self._reset()
            return
        # У самого нуля и на стоянке тормоз вагон не замедляет (он стоит), а
        # форма тормоза силу предсказывает: окно с такими отсчётами портит
        # оценку торможения. Порог тот же, что запрещает адаптацию в ядре.
        if v < self.p.v_adapt_min:
            self._reset()
            return

        if self.v0 is None:
            self.v0, self.u_ref = v, u
        if abs(u - self.u_ref) > 0.05:      # позиция сменилась — окно прервано
            self._reset()
            return

        # Сопротивление движению входит в уравнение корпуса наравне с приводом:
        #   M (v2 - v1) = int(F_привода) dt - int(res_A + res_B * v) dt
        # Без этих двух столбцов невязка достигала 50 %, а оценка тяги была
        # занижена вдвое: сопротивление молча вычиталось из тяги.
        shape = shape_tract(v, self.p) if u > 0 else shape_brake(v, self.p)
        self.phi[0 if u > 0 else 1] += u * shape * self.dt
        self.phi[2] -= self.dt
        self.phi[3] -= abs(v) * self.dt
        self.k += 1
        if self.k >= self.n_win:
            self.rows.append(self.phi.copy())
            self.targets.append(self.mass * (v - self.v0))
            self._reset()
            self.v0, self.u_ref = v, u    # следующее окно начинается отсюда

    def _collect_coast(self, u, v):
        """На выбеге тяги нет: замедление целиком даёт сопротивление движению."""
        if abs(u) < self.p.T_dead and v > 1.0:
            self.coast.append(v)

    # ---------- решение ----------

    def solve(self, rel_sigma_max=0.15, min_windows=12):
        """Возвращает (w_тяги, w_торможения, отчёт) либо бросает Insufficient.

        Критерий приёмки — не глобальное число обусловленности, а
        ОТНОСИТЕЛЬНАЯ ПОГРЕШНОСТЬ КАЖДОГО параметра. Глобальное число
        обусловленности может быть плохим из-за направления, которое нас не
        интересует, и наоборот — хорошим при неопределённом нужном параметре.
        """
        if len(self.rows) < min_windows:
            raise Insufficient(
                f"окон {len(self.rows)}, нужно не меньше {min_windows}")

        A = np.array(self.rows)
        y = np.array(self.targets)
        n, p = A.shape

        w, _, rank, sv = np.linalg.lstsq(A, y, rcond=None)
        cond = float(sv.max() / max(sv.min(), 1e-12))
        r = A @ w - y
        resid = float(np.linalg.norm(r) / max(np.linalg.norm(y), 1e-9))

        # ковариация оценки: s^2 * (A^T A)^-1
        dof = max(n - rank, 1)
        s2 = float(r @ r) / dof
        try:
            cov = s2 * np.linalg.pinv(A.T @ A)
            sigma = np.sqrt(np.clip(np.diag(cov), 0.0, None))
        except np.linalg.LinAlgError:
            raise Insufficient("вырожденная нормальная матрица")

        names = ["тяга", "торможение", "res_A", "res_B"]
        rel = [float(sigma[j] / max(abs(w[j]), 1e-9)) for j in range(p)]
        report = {
            "окон": n,
            "ранг": int(rank),
            "число_обусловленности": cond,
            "относительная_невязка": resid,
            "параметры": {names[j]: (float(w[j]), rel[j]) for j in range(p)},
        }

        if rank < p:
            raise Insufficient(f"ранг {rank} при {p} параметрах: {report}")

        bad = [names[j] for j in (0, 1) if rel[j] > rel_sigma_max]
        if bad:
            raise Insufficient(
                f"погрешность параметров {bad} выше предела "
                f"{100 * rel_sigma_max:.0f} %. Нужны циклы на РАЗНЫХ позициях "
                f"ручки и в разном диапазоне скоростей: {report}")
        return float(w[0]), float(w[1]), report


def identification_profile(t):
    """Задание для водителя или автомата при опознавании.

    Разные позиции ручки и обязательный выбег между ними: без разнообразия
    возбуждения задача остаётся вырожденной.
    """
    # Позиции держатся заметно ниже предела сцепления: полная тяга требует
    # почти 100 % доступного сцепления, колёса срываются, и данные становятся
    # негодными для опознавания. Проверено — ускорение падало вдесятеро.
    cycle = [(1, 14.0), (0, 10.0), (3, 14.0), (0, 12.0), (2, 14.0), (0, 10.0),
             (-1, 10.0), (0, 8.0), (-3, 10.0), (0, 10.0)]
    total = sum(d for _, d in cycle)
    tt = t % total
    acc = 0.0
    for notch, dur in cycle:
        acc += dur
        if tt < acc:
            return notch
    return 0
