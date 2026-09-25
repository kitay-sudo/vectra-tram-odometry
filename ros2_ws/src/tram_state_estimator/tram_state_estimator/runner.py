"""Связка ядра с потоками сообщений. Без ROS: её используют и нода, и
офлайн-оценка по bag, поэтому результаты совпадают.

Время — метки сообщений (header.stamp), не часы узла. Ядро шагает на
равномерной сетке с шагом p.dt: когда приходит сообщение с меткой t, делаются
все шаги сетки до t. Каждая тележка в шаге отмечается свежей, только если её
показание пришло после прошлого шага.

GNSS используется ТОЛЬКО для начальной выставки: начало координат, высота и
курс по двум антеннам. После выставки его сообщения игнорируются.
"""

import math

import numpy as np

from .estimator_core import (Estimator, IV, ID, IS, STANDSTILL, body_force,
                             resistance)

import copy                                                     # noqa: E402
from .estimator_core import sensor_to_speed                     # noqa: E402

R_EARTH = 6378137.0


class Enu:
    """Локальная касательная плоскость: x — восток, y — север, z — вверх, м."""

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self.k = math.cos(math.radians(lat0))

    def fwd(self, lat, lon, alt):
        return (math.radians(lon - self.lon0) * R_EARTH * self.k,
                math.radians(lat - self.lat0) * R_EARTH,
                alt - self.alt0)


class Position:
    """Положение по пройденному пути.

    Без карты — по прямой вдоль начального курса (заглушка). С картой путей —
    вдоль карты от точки выставки (см. track_map.py).
    """

    def __init__(self, track_map=None, origin=None, init_window=3.0):
        self.map = track_map
        self.origin = origin            # (lat, lon, alt) или None: первая точка
        self.init_window = init_window  # с от первой точки GNSS
        self.enu = None
        self.xyz0 = (0.0, 0.0, 0.0)
        self.az = 0.0                   # курс, рад от севера по часовой
        self.ready = False
        self._t0 = None
        self._master = None
        self._acc = []                  # пары (положение master, вектор к rover)
        self._cursor = None
        self._s = 0.0
        self._s_anchor = 0.0            # путь на момент прошлой привязки
        self.anchors = 0

    def on_stop(self, s):
        """Вагон стоит дольше порога: привязка к точке остановки на карте."""
        if self._cursor is not None and self.map.anchor(
                self._cursor, s - self._s_anchor):
            self._s_anchor = s
            self.anchors += 1

    def on_fix(self, stamp, antenna, lat, lon, alt, moved):
        """Точка GNSS. Используется только в окне выставки и только пока
        вагон стоит (moved — путь с момента выставки); затем игнорируется."""
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        if self._t0 is None:
            self._t0 = stamp
        if stamp - self._t0 > self.init_window or moved > 0.5:
            return
        if self.enu is None and antenna == "master":
            self.enu = Enu(*(self.origin or (lat, lon, alt)))
        if self.enu is None:
            return
        if antenna == "master":
            self._master = self.enu.fwd(lat, lon, alt)
            return
        if self._master is None:
            return
        rx, ry, _ = self.enu.fwd(lat, lon, alt)
        mx, my, mz = self._master
        if math.hypot(rx - mx, ry - my) < 1.0:
            return
        # rover стоит впереди master по ходу: вектор между ними — курс.
        # Выставка — среднее по всем парам окна: отдельная точка без точного
        # решения ошибается на метры, и курс по базе 12 м — на десятки градусов.
        self._acc.append((mx, my, mz, rx - mx, ry - my))
        A = np.array(self._acc)
        self.xyz0 = tuple(A[:, :3].mean(0))
        self.az = math.atan2(A[:, 3].sum(), A[:, 4].sum())
        self.ready = True
        if self.map is not None:
            if self._cursor is None:
                self.map.bind(self.enu)
            self._cursor = self.map.locate(self.xyz0, self.az)
            self._s = 0.0
            self._s_anchor = 0.0

    def xyz(self, s):
        """Положение после пути s от точки выставки."""
        if self._cursor is not None:
            self.map.advance(self._cursor, s - self._s)
            self._s = s
            c = self._cursor
            return c["x"], c["y"], c["z"]
        x0, y0, z0 = self.xyz0
        return (x0 + s * math.sin(self.az), y0 + s * math.cos(self.az), z0)


def _num(x):
    """Число из поля сообщения; всё, что числом не является, — NaN."""
    try:
        return float(x)
    except (TypeError, ValueError, OverflowError):
        return math.nan


class Runner:
    """Ядро + выставка + карта на потоке сообщений (нода и офлайн-оценка).

    Защита входов и времени (устойчивость, WP3/WP4):
      * метка не число или <= 0 — сообщение отбрасывается целиком;
      * показание тележки не число или |z| > V_LIMIT·v_max_line (в единицах
        датчика) — значение отбрасывается, метка сетку двигает;
      * ручка не число — отбрасывается; вне ±HANDLE_LIMIT — ограничивается;
      * «своя» база времени: метка не раньше t − MAX_JUMP_S и не позже
        stamp_max + FWD_JUMP_S (первые SETTLE_S прогона — + MAX_JUMP_S:
        стартовый хвост буфера bag, метки вразнобой на 2–6 с). Сообщение вне
        базы — кандидат новой базы: откладывается, в t_wheel / t_notch не
        пишется. Кандидаты подтверждаются CONFIRM_N сообщениями подряд от
        CONFIRM_SRC разных входов (обе тележки — один вход, у них одни часы)
        или CONFIRM_SOLO сообщениями одного входа; сообщение своей базы
        кандидатов отбрасывает (одиночный выброс, сбой часов пары тележек).
        Подтверждённый скачок вперёд до GAP_MAX_S — провал входов в том же
        прогоне: сетка догоняет, состояние сохраняется. Иначе (назад, --loop,
        второй bag, далеко вперёд) — полный сброс reset(): ядро, выставка,
        s0, стоянка, сетка; прежняя выставка остаётся запасной, пока новый
        прогон не выставится по своему GNSS. Отложенные сообщения после
        подтверждения исполняются заново по порядку (не теряются);
      * не больше MAX_STEPS шагов сетки за вызов: остаток догоняется
        следующими вызовами;
      * неконечное состояние ядра — сброс ядра с сохранением пути.
    Сетка (WP23): узлы кратны p.dt, t = (k0 + n)·dt по целому счётчику n —
    совпадают с метками GNSS (кратны 0,1 с) и не дрейфуют.
    Возраст показаний (WP6): свежее показание тележки приводится к моменту
    шага по ускорению модели (age_comp).
    """

    MAX_JUMP_S = 10.0       # с: скачок назад без подтверждения (хвост буфера <= 6,2 с)
    FWD_JUMP_S = 1.5        # с: скачок вперёд без подтверждения (в данных <= 1,06 с)
    SETTLE_S = 10.0         # с меток от начала прогона: до них вперёд тоже MAX_JUMP_S
    GAP_MAX_S = 60.0        # с: подтверждённый скачок вперёд до — провал в том же прогоне
    CONFIRM_N = 3           # сообщений подряд в новой базе времени...
    CONFIRM_SRC = 2         # ...от стольких разных входов
    CONFIRM_SOLO = 10       # или подряд от одного входа (другие молчат)
    MAX_STEPS = 200         # шагов за вызов: 10 с при dt = 50 мс
    V_LIMIT = 1.5           # доля v_max_line: выше — показание невозможно
    HANDLE_LIMIT = 15.0     # позиций ручки по ТЗ в каждую сторону
    AGE_MAX_S = 0.3         # с: предел приведения показания к шагу
    age_comp = True         # WP6: приводить показания к моменту шага
    grid_align = True       # WP23: узлы сетки кратны dt
    _POS_KEEP = ("init_window",)    # настройки выставки, заданные после __init__

    def __new__(cls, *args, **kwargs):
        # Аргументы конструктора запоминаются для reset(): новый прогон
        # собирается тем же __init__, чем бы тот ни дополнялся.
        self = super().__new__(cls)
        self._ctor = (args, kwargs)
        self._grid_state()
        self.resets = 0             # полных сбросов (новый прогон)
        self.gaps = 0               # подтверждённых провалов входов в прогоне
        self.core_resets = 0        # сбросов ядра (неконечное состояние)
        self.rejected_stamps = 0    # сообщений с отброшенной меткой
        self.rejected_values = 0    # отброшенных значений (метка принята)
        self.skipped_steps = 0      # узлов сетки, пропущенных сверх предела
        self.n_in = 0               # принятых сообщений (метка годна)
        self.reset_reason = ""
        self.gap_reason = ""
        self._vlim = None
        self._pos_prev = None       # запасная выставка после сброса
        self._s_prev = 0.0          # и путь от неё на момент сброса
        return self

    def _grid_state(self):
        self._k0 = None             # номер первого узла сетки: t0 = k0·dt
        self._t0g = None            # первая метка (сетка без выравнивания)
        self._n = 0                 # шагов от первого узла
        self._pend = []             # кандидаты новой базы: (метка, вход, повтор)
        self._replaying = False     # исполняются отложенные сообщения
        self._budget = self.MAX_STEPS   # шагов, оставшихся на этот вызов
        self.stamp_ok = False       # принята ли метка последнего сообщения
        self.stamp_last = None      # метка последнего принятого сообщения
        self.stamp_max = -math.inf  # наибольшая принятая метка
        self._t_lo = math.inf       # наименьшая принятая метка прогона
        self._chg = None            # свежее показание отличается от прошлого
        self._s_good = 0.0          # путь ядра на последнем конечном шаге

    def __init__(self, params, track_map=None, origin=None,
                 wheel_timeout=1.0, handle_timeout=0.5, stop_dwell=8.0):
        self.p = params
        self.core = Estimator(params)
        self.nw = self.core.nw
        self.meas = np.zeros(self.nw)
        self.fresh = np.zeros(self.nw, dtype=bool)
        self.t_wheel = np.full(self.nw, -np.inf)
        self.notch = 0.0
        self.t_notch = -np.inf
        self.t = None
        self.s0 = None                  # путь ядра в момент выставки положения
        self.pos = Position(track_map, origin)
        self.stop_dwell = stop_dwell    # с стоянки до привязки к остановке
        self._dwell = 0.0
        self.wheel_timeout = wheel_timeout
        self.handle_timeout = handle_timeout
        self.last = None

    # ---------- входы: каждый возвращает список новых выходов ----------

    def on_wheel(self, i, stamp, value):
        out = self._advance(stamp, "wheel", ("on_wheel", (i, stamp, value)))
        if not self.stamp_ok:
            return out                  # метка отброшена: значение тоже
        v = _num(value)
        if not (0 <= i < self.nw and math.isfinite(v)
                and abs(v) <= self._v_limit()):
            self.rejected_values += 1   # значение отброшено, сетка идёт
            return out
        if self._chg is None:
            self._chg = np.zeros(self.nw, dtype=bool)
        self._chg[i] = v != self.meas[i]
        self.meas[i] = v
        self.fresh[i] = True
        self.t_wheel[i] = self.stamp_last
        return out

    def on_handle(self, stamp, position):
        out = self._advance(stamp, "handle", ("on_handle", (stamp, position)))
        if not self.stamp_ok:
            return out
        n = _num(position)
        if not math.isfinite(n):
            self.rejected_values += 1
            return out
        self.notch = min(self.HANDLE_LIMIT, max(-self.HANDLE_LIMIT, n))
        self.t_notch = self.stamp_last
        return out

    def on_fix(self, stamp, antenna, lat, lon, alt):
        """GNSS — только начальная выставка (см. Position.on_fix)."""
        out = self._advance(stamp)
        moved = (float(self.core.x[IS]) - self.s0) if self.s0 is not None else 0.0
        was = self.pos.ready
        self.pos.on_fix(stamp, antenna, lat, lon, alt, moved)
        if self.pos.ready and was:
            self.s0 = float(self.core.x[IS])    # уточнённая выставка — с места
        return out

    # ---------- шаги ----------

    def _advance(self, stamp, src="fix", replay=None):
        """Шаги сетки до метки сообщения; возвращает их выходы.

        src — вход (часы): "wheel" (обе тележки), "handle", "fix". replay —
        (метод, аргументы) для повторного исполнения сообщения, если оно
        окажется первым в подтверждённой новой базе времени; без него такое
        сообщение теряется. self.stamp_ok — применять ли значение сообщения:
        False, если метка отброшена или сообщение отложено (метка-выброс не
        должна попасть в t_wheel), а также если оно уже исполнено повтором."""
        self.stamp_ok = False
        stamp = _num(stamp)
        if not math.isfinite(stamp) or stamp <= 0.0:
            self.rejected_stamps += 1
            return []
        if not self._replaying:
            self._budget = self.MAX_STEPS
        if self.t is None:
            self._start(stamp)
            return []
        if not self._replaying and not self._in_base(stamp):
            return self._candidate(stamp, src, replay)
        if self._pend:
            self._drop_pending()        # база прежняя: кандидаты — выбросы
        self.stamp_ok = True
        self.stamp_last = stamp
        self.n_in += 1
        if stamp > self.stamp_max:
            self.stamp_max = stamp
        if stamp < self._t_lo:
            self._t_lo = stamp
        return self._run_to(stamp)

    def _in_base(self, stamp):
        """Метка в текущей базе времени прогона (см. docstring класса)."""
        settled = self.stamp_max - self._t_lo >= self.SETTLE_S
        fwd = self.FWD_JUMP_S if settled else self.MAX_JUMP_S
        return self.t - self.MAX_JUMP_S <= stamp <= self.stamp_max + fwd

    def _drop_pending(self):
        self.rejected_stamps += len(self._pend)
        self._pend = []

    def _candidate(self, stamp, src, replay):
        """Сообщение вне базы: откладывается; подтверждённая новая база —
        провал в прогоне (догоняем) или новый прогон (reset), затем отложенные
        сообщения исполняются по порядку прихода."""
        P = self._pend
        if P and max(abs(stamp - q[0]) for q in P) > self.MAX_JUMP_S:
            self._drop_pending()        # третья база: прежние кандидаты — выбросы
            P = self._pend
        P.append((stamp, src, replay))
        n, k = len(P), len({q[1] for q in P})
        if not ((n >= self.CONFIRM_N and k >= self.CONFIRM_SRC)
                or n >= self.CONFIRM_SOLO):
            return []
        self._pend = []
        lo = min(q[0] for q in P)
        jump = lo - self.stamp_max
        why = f"{n} сообщений подряд, входов {k}"
        if 0.0 < jump <= self.GAP_MAX_S:
            self.gaps += 1
            self.gap_reason = (f"провал входов {jump:.1f} с внутри прогона "
                               f"({why}): сетка догоняет, состояние сохранено")
        else:
            self.reset(f"разрыв меток {lo - self.stamp_max:+.1f} с ({why}): "
                       f"новый прогон")
        outs = []
        self._replaying = True
        try:
            for _, _, rp in P:
                if rp is None:
                    self.rejected_stamps += 1   # повтора нет (GNSS): потеряно
                else:
                    outs += getattr(self, rp[0])(*rp[1])
        finally:
            self._replaying = False
        self.stamp_ok = False           # это сообщение уже исполнено повтором
        return outs

    def _start(self, stamp):
        """Первый узел сетки — кратный dt не позже первой метки (WP23);
        grid_align = False — прежняя фаза: от первой метки."""
        self._k0 = (math.floor(stamp / self.p.dt + 1e-6) if self.grid_align
                    else None)
        self._t0g = stamp
        self._n = 0
        self.t = self._node(0)
        self.stamp_ok = True
        self.stamp_last = self.stamp_max = self._t_lo = stamp
        self.n_in += 1

    def _node(self, n):
        if self._k0 is None:
            return self._t0g + n * self.p.dt
        return (self._k0 + n) * self.p.dt

    def next_node(self):
        """Метка следующего узла сетки (None до первого сообщения)."""
        return None if self.t is None else self._node(self._n + 1)

    def _run_to(self, stamp):
        """Узлы сетки <= stamp, но не больше MAX_STEPS за вызов (_budget):
        остаток догоняется следующими вызовами. Узлы сверх провала GAP_MAX_S
        пропускаются (по построению не бывает: дальше — новый прогон)."""
        due = int(math.floor((stamp + 1e-6 - self._node(self._n)) / self.p.dt))
        if due <= 0:
            return []
        cap = int((self.GAP_MAX_S + self.MAX_JUMP_S) / self.p.dt) + self.MAX_STEPS
        if due > cap:
            self.skipped_steps += due - cap
            self._n += due - cap
            due = cap
        due = min(due, max(self._budget, 0))
        self._budget -= due
        outs = []
        for _ in range(due):
            self._n += 1
            self.t = self._node(self._n)
            outs.append(self._step_safe())
        return outs

    def backlog(self):
        """Есть ли узлы сетки не позже принятой метки (догоняние провала)."""
        return self.t is not None and self.next_node() <= self.stamp_max + 1e-6

    def tick(self, stamp):
        """Узлы сетки до stamp без входного сообщения. Для прогноза на копии
        fork() (пульс ноды при паузе входов); состояние копии меняется."""
        stamp = _num(stamp)
        if self.t is None or not math.isfinite(stamp):
            return []
        self._budget = self.MAX_STEPS
        return self._run_to(stamp)

    def fork(self):
        """Независимая копия связки для прогноза: ядро, выставка и курсор
        копируются, карта и лист общие (не меняются при шаге)."""
        memo = {id(self.p): self.p, id(self.core.p): self.core.p}
        for pos in (self.pos, self._pos_prev):
            tmap = getattr(pos, "map", None)
            if tmap is not None:
                memo[id(tmap)] = tmap
        return copy.deepcopy(self, memo)

    def reset(self, reason=""):
        """Новый прогон (второй bag, --loop, подтверждённый разрыв времени):
        ядро, выставка, s0, стоянка, сетка и отметки входов — заново. Карта,
        лист и настройки прежние; выставка возьмётся по GNSS нового прогона.

        Прежняя выставка остаётся запасной (_pos_prev, путь от неё _s_prev):
        пока новая не готова, положение идёт по ней — путь до сброса плюс
        путь после. Разрыв меток внутри прогона (сбой часов) тогда не теряет
        привязку: у жюри GNSS есть только в первые секунды."""
        keep = {k: getattr(self.pos, k) for k in self._POS_KEEP
                if hasattr(self.pos, k)}
        prev = self._fallback_now()
        args, kwargs = self._ctor
        self.__init__(*args, **kwargs)
        self._grid_state()
        for k, v in keep.items():
            setattr(self.pos, k, v)
        self._pos_prev, self._s_prev = prev
        self.resets += 1
        self.reset_reason = reason

    def _fallback_now(self):
        """(выставка, путь от неё) на этот момент — для запасной при сбросе."""
        s = float(self.core.x[IS])
        if not math.isfinite(s):
            s = self._s_good
        if self.pos.ready and self.s0 is not None:
            return self.pos, s - self.s0
        if self._pos_prev is not None:
            return self._pos_prev, self._s_prev + s
        return None, 0.0

    def _active_pos(self, s):
        """Выставка, по которой считается положение, и путь от неё (s — путь
        ядра). Новая выставка готова — она (запасная больше не нужна); иначе
        запасная после сброса; иначе None (положения нет)."""
        if self.pos.ready:
            if self.s0 is None:
                self.s0 = s
            if self._pos_prev is not None:
                self._pos_prev = None
                self._dwell = 0.0
            return self.pos, s - self.s0
        if self._pos_prev is not None:
            return self._pos_prev, self._s_prev + s
        return None, s

    def _core_ok(self):
        c = self.core
        return bool(np.isfinite(c.x).all() and np.isfinite(c.P).all())

    def reset_core(self, why):
        """Только ядро заново, путь сохраняется; выставка, сетка и отметки
        входов прежние (неконечное состояние, застрявшие ошибки в ноде)."""
        s = self._s_good
        self.core = Estimator(self.p)
        self.core.x[IS] = s
        self.fresh[:] = False
        self.core_resets += 1
        self.reset_reason = f"сброс ядра: {why}; путь {s:.1f} м сохранён"

    def _step_safe(self):
        """Шаг с последним рубежом: если состояние ядра стало неконечным,
        ядро пересоздаётся с сохранённым путём и шаг повторяется прогнозом."""
        try:
            o = self._step()
            if self._core_ok():
                self._s_good = float(self.core.x[IS])
                return o
            why = "неконечное состояние ядра"
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as e:
            if self._core_ok():
                raise
            why = f"неконечное состояние ядра ({type(e).__name__}: {e})"
        self.reset_core(why)
        o = self._step()
        self._s_good = float(self.core.x[IS])
        return o

    def _v_limit(self):
        """Предел модуля показания тележки в единицах датчика."""
        if self._vlim is None:
            per_unit = float(sensor_to_speed(1.0, self.p))   # м/с на единицу
            self._vlim = self.V_LIMIT * self.p.v_max_line / per_unit
        return self._vlim

    def _meas_at_step(self):
        """WP6: свежие показания тележек, приведённые к моменту шага t:
        z + a·(t − метка). a — ускорение модели на прошлом шаге. Показание,
        не изменившееся с прошлого (залипание), не трогается: иначе приведение
        маскировало бы залипший датчик от диагностики ядра."""
        if not self.age_comp or self.last is None or self._chg is None:
            return self.meas
        m = self.fresh & self._chg & (self.meas > 0.0)
        if not m.any():
            return self.meas
        age = np.clip(self.t - self.t_wheel, 0.0, self.AGE_MAX_S)
        per_unit = float(sensor_to_speed(1.0, self.p))
        z = self.meas + self.last["a"] * age / per_unit
        return np.where(m, np.maximum(z, 0.0), self.meas)

    def _step(self):
        c, t = self.core, self.t
        wheels_stale = (t - self.t_wheel.max()) > self.wheel_timeout
        handle_ok = (t - self.t_notch) <= self.handle_timeout
        if wheels_stale:
            o = c.step_open_loop(self.notch)
        else:
            o = c.step(self.notch, self._meas_at_step(),
                       fresh=self.fresh.copy(), handle_ok=handle_ok)
        self.fresh[:] = False
        s = float(c.x[IS])
        pos, ds = self._active_pos(s)   # после сброса — запасная выставка
        if pos is not None:
            x, y, z = pos.xyz(ds)
            standing = o["mode"] == STANDSTILL and o["valid"]
            self._dwell = self._dwell + self.p.dt if standing else 0.0
            if self._dwell >= self.stop_dwell > self._dwell - self.p.dt:
                pos.on_stop(ds)
                x, y, z = pos.xyz(ds)
        else:
            # выставки ещё не было (GNSS нет): относительная одометрия по x
            x, y, z = s, 0.0, 0.0
        v = float(c.x[IV])
        a = (body_force(c.u_filt, v, c.x[3], c.x[4], c.mu, c.p)
             - resistance(v, c.p)) / c.p.M_nom + float(c.x[ID])
        cur = pos._cursor if pos is not None else None
        az = cur["h"] if cur is not None else (pos.az if pos is not None else 0.0)
        o.update(stamp=t, x=x, y=y, z=z,
                 yaw=(math.pi / 2.0 - az) if pos is not None else None, a=float(a) if v > 0 or a > 0 else 0.0,
                 pos_ready=pos is not None, pos_fallback=pos is not None and pos is not self.pos,
                 handle_ok=handle_ok, wheels_stale=bool(wheels_stale))
        self.last = o
        return o


class StartSorter:
    """WP24. Первые window секунд по часам прихода сообщения копятся и
    отдаются по возрастанию метки, дальше — сквозной проход.

    Стартовый всплеск bag идёт в порядке записи, а метки в нём скачут назад
    на 1–3 с (хвост буфера записи, DATA п. 7): сетка, начатая с первого
    обработанного сообщения, теряет выходы до самой ранней метки, а окно
    выставки считается не от первой точки GNSS. Часы — любые монотонные:
    в ноде time.monotonic(), офлайн — время записи bag."""

    def __init__(self, window=0.3):
        self.window = float(window)
        self.done = not self.window > 0.0
        self._t0 = None
        self._buf = []

    def push(self, now, stamp, item):
        """Новое сообщение; возвращает список готовых к обработке."""
        if self.done:
            return [item]
        if self._t0 is None:
            self._t0 = now
        s = _num(stamp)
        self._buf.append((s if math.isfinite(s) else math.inf,
                          len(self._buf), item))
        return self.poll(now)

    def poll(self, now):
        """Окно истекло — всё накопленное по порядку меток (один раз)."""
        if self.done or self._t0 is None or now - self._t0 < self.window:
            return []
        self.done = True
        out = [it for _, _, it in sorted(self._buf, key=lambda e: e[:2])]
        self._buf = []
        return out
