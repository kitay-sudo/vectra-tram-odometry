"""Связка ядра с потоками сообщений. Без ROS: её используют и нода, и
офлайн-оценка по bag, поэтому результаты совпадают.

Время — метки сообщений (header.stamp), не часы узла. Ядро шагает на
равномерной сетке с шагом p.dt: когда приходит сообщение с меткой t, делаются
все шаги сетки до t. Каждая тележка в шаге отмечается свежей, только если её
показание пришло после прошлого шага.

GNSS используется ТОЛЬКО для начальной выставки в окне init_window секунд от
первой годной точки: начало координат, высота и курс. Сетку ядра GNSS не
двигает никогда: годная точка окна ставится в очередь и применяется, когда
сетка (её двигают только тележки и ручка) дойдёт до её метки. Точки вне окна
отбрасываются сразу. Поэтому скорость от GNSS не зависит вовсе, а положение —
только через выставку (docs/POSITION_FRAME.md).

Положение считается во внутренней непрерывной системе (UTM со сдвигом в точку
выставки, geodesy.Frame) и переводится в выходную (по умолчанию MGRS) только
при выдаче.
"""

import copy
import math

import numpy as np

from .estimator_core import (Estimator, IV, ID, IS, STANDSTILL, body_force,
                             position_sigma, resistance, sensor_to_speed)
from .geodesy import Equirect, Frame, projection_name
from .track_map import hold_mode

R_EARTH = 6378137.0

# Прежнее имя: это была плоская формула (equirect), а не строгий ENU.
# Оставлено для старых скриптов аудита; нода его не использует.
Enu = Equirect


class Position:
    """Положение по пройденному пути колёс.

    Выставка (только в окне init_window от первой годной точки GNSS):
      * якорь — среднее точек master окна, если вагон стоял (путь колёс за
        окно < 0,3 м), иначе ПОСЛЕДНЯЯ точка master окна с путём колёс на её
        метку (старт на ходу);
      * курс — по парам master→rover одной эпохи (rover впереди); без rover —
        по смещению master за окно (> 5 м), иначе по карте в точке якоря;
        иначе курса нет, и выход стоит в точке якоря;
      * нет master, есть rover — якорь по rover, сдвинутый назад по курсу на
        базу ROVER_BASE (rover впереди master); без курса — точка rover;
      * якорь и путь выставки s_ref пересчитываются только при точке, вошедшей
        в выставку; после окна GNSS не читается;
      * точка с меткой дальше MAX_SKEW от сетки ядра — сбой метки: не
        выставка (и не может открыть окно).
    Движение: с картой — курсор по карте (track_map.py) с привязкой к точкам
    остановок и онлайн-подстройкой масштаба пути (WP13); без карты — стоянка
    в якоре (nomap_mode "hold", по умолчанию: на отложенных 2,3 км против
    3,1 км) или прямая вдоль курса выставки ("line").
    """

    PAIR_TOL = 0.06              # с: master и rover одной эпохи
    BASE_MIN, BASE_MAX = 5.0, 25.0   # м: допустимая длина базы (≈12,4 м)
    # онлайн-масштаб пути (v3b_scale из tools/audit/position_study.py)
    SCALE_PRIOR_L = 3000.0       # м пути колёс: «априорный» вес исходного масштаба
    SCALE_MAX_DEV = 0.01         # ограничение ±1 %
    SCALE_GATE_K, SCALE_GATE_C = 0.01, 2.0   # принимать сдвиг |δ| < 1 %·L + 2 м
    SCALE_MIN_L = 100.0          # м: короче — отрезок не учитывается
    SCALE_INCONS = 0.012         # отрезки разошлись больше — подстройку выключить
    OFFSET_Z_MAX = 30.0          # м: больший сдвиг высоты GNSS − карта не переносить
    ROVER_BASE = 12.42           # м: rover впереди master (по данным; tf обещан)
    MAX_SKEW = 30.0              # с: метка GNSS дальше от сетки ядра — сбой метки
    ROVER_ONLY_AFTER = 1.0       # с: rover без master дольше — выставка по rover

    def __init__(self, track_map=None, origin=None, init_window=3.0,
                 projection="mgrs", mgrs_grid="", utm_zone=0, stop_dwell=8.0,
                 scale_adapt=True, nomap_mode="hold", keep_offset_xy=True,
                 keep_offset_z=True, keep_offset_max_status=-1, mgrs_guard_m=20.0,
                 terminal_hold="terminals"):
        self.map = track_map
        if track_map is not None:
            track_map.terminal_hold = hold_mode(terminal_hold)
        self.origin = origin            # (lat, lon, alt) или None: первая точка master
        self.init_window = float(init_window)
        self.projection = projection_name(projection)
        self.mgrs_grid = str(mgrs_grid or "").strip()
        self.utm_zone = int(utm_zone or 0)
        if self.projection == "mgrs" and self.mgrs_grid:
            Frame(55.8, 39.0, 0.0, "mgrs", self.mgrs_grid)   # проверка кода сразу
        self.stop_dwell = float(stop_dwell)
        self.scale_adapt = bool(scale_adapt)
        if nomap_mode not in ("line", "hold"):
            raise ValueError(f"nomap_mode {nomap_mode!r}: line | hold")
        self.nomap_mode = nomap_mode
        # сдвиг «GNSS окна − карта» в якоре: сохранять ли его в выходе, если
        # медиана NavSatFix.status точек окна ≤ keep_offset_max_status.
        # −1 (по умолчанию) — никогда: на train (68 прогонов) сдвиг не даёт
        # выигрыша (всегда 3,687 м, только без RTK 3,672, никогда 3,666),
        # у RTK — вреден (1,841 против 1,823). 1 — только без RTK (статус
        # 0/1), 2 — всегда (прежнее умолчание, выбранное по holdout). По
        # горизонтали это только поперечная часть (курсор притягивается к оси
        # пути поперёк, не дальше snap_r), по высоте — не больше OFFSET_Z_MAX
        self.keep_offset_xy = bool(keep_offset_xy)
        self.keep_offset_z = bool(keep_offset_z)
        self.keep_offset_max_status = int(keep_offset_max_status)
        self.offset = (0.0, 0.0, 0.0)
        self.window_status = None       # медиана статуса точек окна
        # MGRS с переносом по квадратам: ближе mgrs_guard_m к краю 100-км
        # квадрата положение не публикуется (pos_valid = False): выход и эталон
        # могли бы оказаться в разных квадратах (ошибка 100 км). 0 — выкл.
        # 20 м выбрано на train (68 прогонов): при 5 м 49 таких отсчётов на 7
        # прогонах, при 20 м — 0; цена — ~4,7 с без положения на пересечение
        self.mgrs_guard_m = float(mgrs_guard_m)
        self.frame = None               # geodesy.Frame: с первой точки master
        self.fixed = False              # якорь есть (положение известно)
        self.ready = False              # и курс есть (выставка полная)
        self.xyz0 = (0.0, 0.0, 0.0)     # якорь во внутренней системе
        self.xyz_internal = None        # последнее положение, внутренняя система
        self.az = 0.0                   # курс сетки, рад от севера по часовой
        self.s_ref = None               # путь колёс в момент якоря
        self.n_used = 0                 # точек GNSS, вошедших в выставку
        self.n_rejected = 0             # отброшено как негодные
        self._t0 = None
        self._q = []                    # точки окна, ждущие шага сетки
        # точки окна: (метка, широта, долгота, высота, путь колёс, статус)
        self._m = []                    # master
        self._r = []                    # rover
        self._frame_rover = False       # система пока от rover (master ещё нет)
        self._pairs = []                # база master→rover одной эпохи
        self._last_alt = None
        self._cursor = None
        self._bound = False
        self._s = 0.0                   # путь от якоря, на который сдвинут курсор
        self._s_anchor = 0.0            # путь на момент прошлой привязки
        self._dwell = 0.0
        self.anchors = 0
        self.mult = 1.0                 # онлайн-поправка масштаба пути
        self._num = self._den = None
        self._segs = []                 # множители принятых отрезков
        self.scale_log = []             # (путь, сдвиг, L, mult) — для оценки

    # ---------- GNSS ----------

    def window_open(self, stamp):
        """Окно выставки: ±init_window от первой годной точки (раньше неё —
        хвост буфера записи в начале bag)."""
        t0 = self._t0
        return t0 is None or (t0 - self.init_window <= stamp <= t0 + self.init_window)

    def accepts(self, stamp, lat, lon, status=0):
        """Годна ли точка GNSS для выставки (проверяется до шага сетки):
        конечные метка и координаты, не (0, 0), статус NavSatFix ≥ 0, окно."""
        if not (math.isfinite(stamp) and math.isfinite(lat) and math.isfinite(lon)
                and abs(lat) <= 90.0 and abs(lon) <= 180.0
                and not (abs(lat) < 1e-9 and abs(lon) < 1e-9)
                and (status is None or status >= 0)):
            self.n_rejected += 1
            return False
        return self.window_open(stamp)

    def on_fix(self, stamp, antenna, lat, lon, alt, s=None, v=0.0, t=None, status=0):
        """Точка GNSS. Негодная или вне окна — False. Годная ставится в
        очередь (s is None) и применяется на шаге сетки, который дойдёт до её
        метки (step); с s, v, t применяется сразу: путь на метку точки —
        s + v·(stamp − t)."""
        if not self.accepts(stamp, lat, lon, status):
            return False
        if self._t0 is None:
            self._t0 = stamp
        if s is None:
            if len(self._q) < 1000:
                self._q.append((stamp, antenna, lat, lon, alt, status))
            return True
        self._apply(stamp, antenna, lat, lon, alt, s, v, t, status)
        return True

    def resync(self, t):
        """Сетка ядра на t, а в выставку ещё ничего не вошло: точки с меткой
        дальше MAX_SKEW от t — сбой метки (например, 0 или 1e10 у первой
        точки, пришедшей до тележек). Они не должны открывать окно выставки:
        окно — заново от первой разумной точки."""
        if (self._t0 is not None and self.n_used == 0
                and not abs(self._t0 - t) <= self.MAX_SKEW):
            self._q = [q for q in self._q if abs(q[0] - t) <= self.MAX_SKEW]
            self._t0 = min((q[0] for q in self._q), default=None)

    def _flush(self, t, s, v):
        """Точки из очереди с меткой не позже шага t сетки."""
        due = sorted((q for q in self._q if q[0] <= t + 1e-9), key=lambda q: q[0])
        if due:
            self._q = [q for q in self._q if q[0] > t + 1e-9]
            for q in due:
                self._apply(*q[:5], s, v, t, q[5])

    def _apply(self, stamp, antenna, lat, lon, alt, s, v, t, status=0):
        if alt is None or not math.isfinite(alt):
            alt = self._alt_fallback(lat, lon)
        else:
            self._last_alt = alt
        dt = 0.0 if t is None else min(max(stamp - t, -0.5), 0.5)
        row = (stamp, lat, lon, alt, float(s) + v * dt, 0 if status is None else status)
        (self._m if antenna == "master" else self._r).append(row)
        self.n_used += 1
        self._align()
        return True

    def _make_frame(self, row, from_rover):
        o = self.origin or row[1:4]
        self.frame = Frame(*o, projection=self.projection,
                           mgrs_grid=self.mgrs_grid, utm_zone=self.utm_zone)
        self._frame_rover = from_rover and self.origin is None
        self._bound = False

    def _alt_fallback(self, lat, lon):
        """Высота вместо NaN: карта у точки, иначе последняя годная, иначе 0."""
        if self.map is not None:
            h = self.map.altitude_at(lat, lon)
            if h is not None:
                return h
        return self._last_alt if self._last_alt is not None else 0.0

    def _project(self, rows):
        """Точки окна -> массив (метка, x, y, z, путь колёс, статус)."""
        A = np.array(rows, float)
        P = self.frame.fwd_arr(A[:, 1], A[:, 2], A[:, 3])
        return np.c_[A[:, 0], P, A[:, 4], A[:, 5]]

    def _align(self):
        if self._m:
            from_rover = False
            if self.frame is None or self._frame_rover:
                self._make_frame(self._m[0], False)
        elif self._r and (max(q[0] for q in self._r) - min(q[0] for q in self._r)
                          >= self.ROVER_ONLY_AFTER):
            # master нет уже ROVER_ONLY_AFTER с: выставка по rover
            from_rover = True
            if self.frame is None:
                self._make_frame(self._r[0], True)
        else:
            return
        M = self._project(self._r if from_rover else self._m)
        pairs = np.zeros((0, 2))
        if not from_rover and self._r:
            R = self._project(self._r)
            i, j = np.nonzero(np.abs(M[:, 0][:, None] - R[:, 0][None, :]) <= self.PAIR_TOL)
            d = R[j, 1:3] - M[i, 1:3]
            b = np.hypot(d[:, 0], d[:, 1])
            pairs = d[(b >= self.BASE_MIN) & (b <= self.BASE_MAX)]
        self._pairs = pairs
        k_last = int(np.argmax(M[:, 0]))
        moving = (M[:, 4].max() - M[:, 4].min()) > 0.3
        if moving:
            anchor = tuple(M[k_last, 1:4])
        else:
            med = np.median(M[:, 1:3], axis=0)
            d = np.hypot(M[:, 1] - med[0], M[:, 2] - med[1])
            keep = d < 20.0
            if not keep.any():               # две точки или два облака дальше 40 м
                keep = d <= d.min() + 1e-6
            anchor = tuple(M[keep, 1:4].mean(0))
        if not all(math.isfinite(a) for a in anchor):
            return
        s_ref = float(M[k_last, 4])
        first = M[int(np.argmin(M[:, 0]))]
        dx, dy = M[k_last, 1] - first[1], M[k_last, 2] - first[2]
        if len(pairs):
            az = math.atan2(pairs[:, 0].sum(), pairs[:, 1].sum())
        elif math.hypot(dx, dy) > 5.0:
            az = math.atan2(dx, dy)                      # нет rover, но едем
        else:
            az = None
        if self.map is not None and not self._bound:
            self.map.bind(self.frame)
            self._bound = True
        if az is None and self.map is not None:
            az = self.map.heading_at(anchor[:2])
        if from_rover and az is not None:
            # rover впереди master на базу: якорь master — назад по курсу
            anchor = (anchor[0] - self.ROVER_BASE * math.sin(az),
                      anchor[1] - self.ROVER_BASE * math.cos(az), anchor[2])
        self.window_status = float(np.median(M[:, 5]))
        self.xyz0, self.s_ref, self.fixed = anchor, s_ref, True
        self._s = 0.0
        self._s_anchor = 0.0
        if az is None:
            self.ready = False
            self._cursor = None
            return
        self.az, self.ready = az, True
        if self.map is not None:
            self._cursor = self.map.locate(anchor, az)
            c = self._cursor
            self.offset = (0.0, 0.0, 0.0)
            if c.get("on_map") and self.window_status <= self.keep_offset_max_status:
                dz = anchor[2] - c["z"]
                self.offset = ((anchor[0] - c["x"]) if self.keep_offset_xy else 0.0,
                               (anchor[1] - c["y"]) if self.keep_offset_xy else 0.0,
                               dz if self.keep_offset_z and abs(dz) <= self.OFFSET_Z_MAX
                               else 0.0)

    # ---------- путь -> положение ----------

    def on_stop(self, ds):
        """Вагон стоит дольше порога: привязка к точке остановки на карте и
        онлайн-подстройка масштаба пути по сдвигу привязки."""
        c = self._cursor
        if c is None:
            return
        L = ds - self._s_anchor
        d = self.map.anchor(c, L)
        if d is None:
            return
        self.anchors += 1
        self._s_anchor = ds
        if self.scale_adapt:
            self._adapt_scale(L, d)
        self.scale_log.append((ds, d, L, self.mult))

    def _adapt_scale(self, L, d):
        """Масштаб = (s0·L_prior + Σ(L_i·s0·m + δ_i)) / (L_prior + ΣL_i): δ_i —
        сдвиг вдоль пути при привязке, L_i — путь колёс с прошлой привязки.
        Медленно (L_prior 3 км), не больше ±1 %; отрезки со сдвигом больше
        1 %·L + 2 м или короче 100 м не учитываются. Если множители принятых
        отрезков расходятся больше чем на 1,2 %, привязки противоречат друг
        другу: подстройка выключается, масштаб возвращается к исходному."""
        s0 = self.map.scale
        if self._num is None:
            self._num, self._den = s0 * self.SCALE_PRIOR_L, self.SCALE_PRIOR_L
        if L < self.SCALE_MIN_L or abs(d) >= self.SCALE_GATE_K * L + self.SCALE_GATE_C:
            return
        used = s0 * self.mult
        self._segs.append((L * used + d) / L / s0)
        if max(self._segs) - min(self._segs) > self.SCALE_INCONS:
            self.scale_adapt, self.mult = False, 1.0
            return
        self._num += L * used + d
        self._den += L
        m = self._num / self._den / s0
        self.mult = min(max(m, 1.0 - self.SCALE_MAX_DEV), 1.0 + self.SCALE_MAX_DEV)

    def _internal(self, ds):
        if self._cursor is not None:
            self.map.advance(self._cursor, ds - self._s, self.mult)
            self._s = ds
            c = self._cursor
            ox, oy, oz = self.offset
            return c["x"] + ox, c["y"] + oy, c["z"] + oz
        x0, y0, z0 = self.xyz0
        if not self.ready or self.nomap_mode == "hold":
            return x0, y0, z0
        L = ds * self.frame.k0
        return x0 + L * math.sin(self.az), y0 + L * math.cos(self.az), z0

    def near_square_edge(self, x, y):
        """Точка внутренней системы ближе mgrs_guard_m к краю 100-км квадрата
        (только MGRS с переносом по квадратам)."""
        g = self.mgrs_guard_m
        if g <= 0.0 or self.frame.projection != "mgrs" or self.frame.grid:
            return False
        E, N = self.frame.utm(x, y)
        e, n = E % 1e5, N % 1e5
        return min(e, 1e5 - e, n, 1e5 - n) < g

    def step(self, s, standing, dt, t=None, v=0.0):
        """Шаг сетки t: путь ядра s, скорость v -> (x, y, z, yaw) в выходной
        системе; None, пока нет якоря (ещё не было ни одной годной точки
        master) или точка у края 100-км квадрата MGRS (mgrs_guard_m). Сначала
        применяются точки GNSS из очереди с меткой ≤ t."""
        if t is not None:
            self.resync(t)
        if self._q and t is not None:
            self._flush(t, s, v)
        if not self.fixed:
            return None
        ds = s - self.s_ref
        x, y, z = self._internal(ds)
        self._dwell = self._dwell + dt if standing else 0.0
        if self._dwell >= self.stop_dwell > self._dwell - dt:
            self.on_stop(ds)
            x, y, z = self._internal(ds)
        c = self._cursor
        az = c["h"] if c is not None else (self.az if self.ready else None)
        self.xyz_internal = (x, y, z)
        if self.near_square_edge(x, y):
            return None
        X, Y, Z = self.frame.out(x, y, z)
        yaw = self.frame.out_yaw(x, y, az) if az is not None else None
        return X, Y, Z, yaw

    def xyz(self, ds):
        """Положение после пути ds от якоря в выходной системе (без привязок)."""
        return self.frame.out(*self._internal(ds))


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
        подтверждения исполняются заново по порядку прихода (не теряются);
      * не больше MAX_STEPS шагов сетки за вызов: сообщения, до метки
        которых сетка за вызов не доходит, ждут в очереди _defer и
        применяются, когда дойдёт (порядок «узлы до метки, потом значение»
        как в main); остаток догоняется следующими вызовами;
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
        self._defer = []            # ждут сетку: (метка, повтор) по порядку прихода
        self._replaying = False     # исполняются отложенные сообщения
        self._budget = self.MAX_STEPS   # шагов, оставшихся на этот вызов
        self.stamp_ok = False       # принята ли метка последнего сообщения
        self.stamp_last = None      # метка последнего принятого сообщения
        self.stamp_max = -math.inf  # наибольшая принятая метка
        self._t_lo = math.inf       # наименьшая принятая метка прогона
        self._chg = None            # свежее показание отличается от прошлого
        self._s_good = 0.0          # путь ядра на последнем конечном шаге

    def __init__(self, params, track_map=None, origin=None,
                 wheel_timeout=1.0, handle_timeout=0.5, stop_dwell=8.0,
                 **position_opts):
        """position_opts — параметры Position: init_window, projection,
        mgrs_grid, utm_zone, scale_adapt, nomap_mode, keep_offset_xy,
        keep_offset_z, keep_offset_max_status, mgrs_guard_m, terminal_hold."""
        self.p = params
        self.core = Estimator(params)
        self.nw = self.core.nw
        self.meas = np.zeros(self.nw)
        self.fresh = np.zeros(self.nw, dtype=bool)
        self.t_wheel = np.full(self.nw, -np.inf)
        self.notch = 0.0
        self.t_notch = -np.inf
        self.t = None
        self._pos_args = (track_map, origin, dict(position_opts, stop_dwell=stop_dwell))
        self.pos = self._new_position()
        self.wheel_timeout = wheel_timeout
        self.handle_timeout = handle_timeout
        self.last = None
        self._anchors = 0               # привязок к остановкам учтено в σ
        self._s_fix = -np.inf           # путь ядра при последней привязке

    def _new_position(self):
        """Новое положение с теми же картой и параметрами (и при сбросе)."""
        m, o, kw = self._pos_args
        return Position(m, o, **kw)

    @property
    def s0(self):
        """Путь ядра в момент выставки (совместимость; хранится в Position)."""
        return self.pos.s_ref

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

    def on_fix(self, stamp, antenna, lat, lon, alt, status=0):
        """GNSS — только начальная выставка. Сетку ядра GNSS не двигает:
        негодная точка или точка вне окна выставки отбрасывается, годная
        точка окна ставится в очередь Position и применяется на шаге сетки,
        дошедшем до её метки. Поэтому GNSS не влияет на скорость, а после
        окна — ни на что. Якорь и путь выставки пересчитываются только при
        точке, вошедшей в выставку. status — NavSatFix.status.status (< 0:
        нет решения). Точка с меткой дальше Position.MAX_SKEW от сетки —
        сбой метки, отбрасывается. Выходов не порождает: возвращает [].
        Поля, не являющиеся числами, считаются NaN (точка негодна)."""
        stamp, lat, lon = _num(stamp), _num(lat), _num(lon)
        alt = None if alt is None else _num(alt)
        if self.t is not None:
            if not abs(stamp - self.t) <= self.pos.MAX_SKEW:
                self.pos.n_rejected += 1
                return []
            self.pos.resync(self.t)
        if self.pos.accepts(stamp, lat, lon, status):
            self.pos.on_fix(stamp, antenna, lat, lon, alt, status=status)
        return []

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
        if self._replaying:             # из очереди: база и порядок проверены
            return self._accept(stamp)
        self._budget = self.MAX_STEPS
        if self.t is None:
            self._start(stamp)
            return []
        if not self._in_base(stamp):
            return self._candidate(stamp, src, replay)
        if self._pend:
            self._drop_pending()        # база прежняя: кандидаты — выбросы
        if self._defer or self._due(stamp) > self._budget:
            # сетка не дойдёт до метки за этот вызов: значение применится,
            # когда дойдёт (как в main: сначала все узлы до метки, потом
            # значение), не больше MAX_STEPS узлов за вызов
            self._note(stamp)
            if replay is not None:
                self._defer.append((stamp, replay))
            return self._drain()
        return self._accept(stamp)

    def _note(self, stamp):
        if stamp > self.stamp_max:
            self.stamp_max = stamp
        if stamp < self._t_lo:
            self._t_lo = stamp

    def _accept(self, stamp):
        """Метка принята: узлы сетки до неё; значение применит вызывающий."""
        if self.t is None:
            self._start(stamp)          # первый повтор после сброса
            return []
        self.stamp_ok = True
        self.stamp_last = stamp
        self.n_in += 1
        self._note(stamp)
        return self._run_to(stamp)

    def _due(self, stamp):
        """Сколько узлов сетки не позже stamp ещё не пройдено."""
        return int(math.floor((stamp + 1e-6 - self._node(self._n)) / self.p.dt))

    def _drain(self):
        """Очередь сообщений, ждущих сетку (отложенные кандидаты новой базы,
        догоняние провала): узлы до метки головы в пределах бюджета вызова,
        затем её значение (повтор метода), и так далее."""
        outs = []
        while self._defer:
            stamp, rp = self._defer[0]
            if self.t is not None:
                outs += self._run_to(stamp)
                if self._due(stamp) > 0:
                    break               # бюджет вызова исчерпан: дальше — потом
            self._defer.pop(0)
            self._replaying = True
            try:
                outs += getattr(self, rp[0])(*rp[1])
            finally:
                self._replaying = False
        self.stamp_ok = False           # значение этого сообщения — через очередь
        return outs

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
        for st, _, rp in P:             # по порядку прихода, через очередь
            if rp is None:
                self.rejected_stamps += 1   # повтора нет (GNSS): потеряно
                continue
            if self.t is not None:
                self._note(st)
            self._defer.append((st, rp))
        return self._drain()

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
        due = self._due(stamp)
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
        """Связка догоняет провал: есть сообщения, ждущие сетку, или узлы не
        позже принятой метки."""
        return bool(self._defer) or (
            self.t is not None and self.next_node() <= self.stamp_max + 1e-6)

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
        # σ положения (WP12) у запасной выставки продолжает расти от её
        # последней привязки: отсчёт пути запасной = путь прежнего ядра
        sig = (self._anchors, self._s_fix)
        args, kwargs = self._ctor
        self.__init__(*args, **kwargs)
        self._grid_state()
        for k, v in keep.items():
            setattr(self.pos, k, v)
        self._pos_prev, self._s_prev = prev
        if prev[0] is not None:
            self._anchors, self._s_fix = sig
        self.resets += 1
        self.reset_reason = reason

    def _fallback_now(self):
        """(выставка, путь ядра в её отсчёте) на этот момент — для запасной
        при сбросе. Position.step сам вычитает свой s_ref, поэтому хранится
        путь ядра прежнего прогона, а не путь от якоря."""
        s = float(self.core.x[IS])
        if not math.isfinite(s):
            s = self._s_good
        if self.pos.fixed:
            return self.pos, s
        if self._pos_prev is not None:
            return self._pos_prev, self._s_prev + s
        return None, 0.0

    def _position(self, s, standing, t, v):
        """Положение на шаге t -> ((x, y, z, yaw) | None, запасная ли).

        Новая выставка шагает всегда (её очередь GNSS применяется на своих
        метках). Пока у неё нет якоря, а после сброса есть прежняя выставка —
        положение идёт по прежней: путь ядра до сброса плюс путь после (t=None:
        очередь GNSS прежнего прогона не трогается). Как только новая
        выставка получила якорь, запасная больше не нужна."""
        pq = self.pos.step(s, standing, self.p.dt, t=t, v=v)
        if self._pos_prev is None:
            return pq, False
        if self.pos.fixed:
            # новая выставка: σ положения — от её якоря (отсчёт пути новый)
            self._pos_prev = None
            self._anchors, self._s_fix = 0, -np.inf
            return pq, False
        return self._pos_prev.step(self._s_prev + s, standing, self.p.dt), True

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
        # положение: Position (выставка, карта, привязки, выходная система);
        # после сброса, пока новый прогон не выставился, — запасная выставка
        pq, fallback = self._position(s, o["mode"] == STANDSTILL and o["valid"],
                                      t, float(c.x[IV]))
        pos = self._pos_prev if fallback else self.pos
        if pos.fixed:
            # σ положения (WP12): путь после выставки или последней привязки
            # к остановке — в отсчёте пути той выставки, по которой идёт
            # положение (у запасной — путь ядра прежнего прогона + новый)
            s_act = self._s_prev + s if fallback else s
            if pos.anchors != self._anchors:
                self._anchors, self._s_fix = pos.anchors, s_act
            ds_fix = s_act - max(pos.s_ref, self._s_fix)
            o["sigma_s"] = position_sigma(o["sigma_s"], ds_fix, c.p)
            o["ds_fix"] = ds_fix
        if pq is None:
            # якоря ещё нет (нет GNSS) или точка у края квадрата MGRS: положения
            # в выходной системе нет, pos_valid = False — нода
            # /result/position не публикует
            x, y, z, yaw = s, 0.0, 0.0, None
        else:
            x, y, z, yaw = pq
        v = float(c.x[IV])
        a = (body_force(c.u_filt, v, c.x[3], c.x[4], c.mu, c.p)
             - resistance(v, c.p)) / c.p.M_nom + float(c.x[ID])
        o.update(stamp=t, x=x, y=y, z=z, yaw=yaw,
                 a=float(a) if v > 0 or a > 0 else 0.0,
                 pos_ready=pos.ready, pos_valid=pq is not None,
                 pos_fallback=fallback,
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
