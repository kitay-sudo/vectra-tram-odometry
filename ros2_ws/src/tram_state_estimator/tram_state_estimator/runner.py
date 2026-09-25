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

import math

import numpy as np

from .estimator_core import (Estimator, IV, ID, IS, STANDSTILL, body_force,
                             resistance)
from .geodesy import Equirect, Frame, projection_name

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
      * якорь и путь выставки s_ref пересчитываются только при точке, вошедшей
        в выставку; после окна GNSS не читается.
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

    def __init__(self, track_map=None, origin=None, init_window=3.0,
                 projection="mgrs", mgrs_grid="", utm_zone=0, stop_dwell=8.0,
                 scale_adapt=True, nomap_mode="hold", keep_offset_xy=True,
                 keep_offset_z=True, mgrs_guard_m=0.0):
        self.map = track_map
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
        # сдвиг «GNSS окна − карта» в якоре сохраняется в выходе (эталон судьи —
        # тот же GNSS, у прогонов без RTK он смещён на метры). По горизонтали
        # это только поперечная часть (курсор притягивается к оси пути поперёк,
        # не дальше snap_r), по высоте — не больше OFFSET_Z_MAX
        self.keep_offset_xy = bool(keep_offset_xy)
        self.keep_offset_z = bool(keep_offset_z)
        self.offset = (0.0, 0.0, 0.0)
        # MGRS с переносом по квадратам: ближе mgrs_guard_m к краю 100-км
        # квадрата положение не публикуется (pos_valid = False): выход и эталон
        # могли бы оказаться в разных квадратах (ошибка 100 км). 0 — выкл.
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
        self._m = []                    # master: (метка, x, y, z, путь колёс)
        self._r = []                    # rover:  (метка, x, y)
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
                self._q.append((stamp, antenna, lat, lon, alt))
            return True
        self._apply(stamp, antenna, lat, lon, alt, s, v, t)
        return True

    def _flush(self, t, s, v):
        """Точки из очереди с меткой не позже шага t сетки."""
        due = sorted((q for q in self._q if q[0] <= t + 1e-9), key=lambda q: q[0])
        if due:
            self._q = [q for q in self._q if q[0] > t + 1e-9]
            for q in due:
                self._apply(*q, s, v, t)

    def _apply(self, stamp, antenna, lat, lon, alt, s, v, t):
        if alt is None or not math.isfinite(alt):
            alt = self._alt_fallback(lat, lon)
        else:
            self._last_alt = alt
        if self.frame is None:
            if antenna != "master":
                return False
            o = self.origin or (lat, lon, alt)
            self.frame = Frame(*o, projection=self.projection,
                               mgrs_grid=self.mgrs_grid, utm_zone=self.utm_zone)
        x, y, z = self.frame.fwd(lat, lon, alt)
        dt = 0.0 if t is None else min(max(stamp - t, -0.5), 0.5)
        s = float(s)
        if antenna == "master":
            self._m.append((stamp, x, y, z, s + v * dt))
            for tr, rx, ry in self._r[-5:]:
                self._pair(stamp, x, y, tr, rx, ry)
        else:
            self._r.append((stamp, x, y))
            for tm, mx, my, _, _ in self._m[-5:]:
                self._pair(tm, mx, my, stamp, x, y)
        self.n_used += 1
        self._align()
        return True

    def _pair(self, tm, mx, my, tr, rx, ry):
        if abs(tm - tr) <= self.PAIR_TOL:
            b = math.hypot(rx - mx, ry - my)
            if self.BASE_MIN <= b <= self.BASE_MAX:
                self._pairs.append((rx - mx, ry - my))

    def _alt_fallback(self, lat, lon):
        """Высота вместо NaN: карта у точки, иначе последняя годная, иначе 0."""
        if self.map is not None:
            h = self.map.altitude_at(lat, lon)
            if h is not None:
                return h
        return self._last_alt if self._last_alt is not None else 0.0

    def _align(self):
        if not self._m:
            return
        M = np.array(self._m)
        k_last = int(np.argmax(M[:, 0]))
        moving = (M[:, 4].max() - M[:, 4].min()) > 0.3
        if moving:
            anchor = tuple(M[k_last, 1:4])
        else:
            med = np.median(M[:, 1:3], axis=0)
            keep = np.hypot(M[:, 1] - med[0], M[:, 2] - med[1]) < 20.0
            anchor = tuple(M[keep, 1:4].mean(0))
        s_ref = float(M[k_last, 4])
        first = M[int(np.argmin(M[:, 0]))]
        dx, dy = M[k_last, 1] - first[1], M[k_last, 2] - first[2]
        if self._pairs:
            P = np.array(self._pairs)
            az = math.atan2(P[:, 0].sum(), P[:, 1].sum())
        elif math.hypot(dx, dy) > 5.0:
            az = math.atan2(dx, dy)                      # нет rover, но едем
        else:
            az = None
        if self.map is not None and not self._bound:
            self.map.bind(self.frame)
            self._bound = True
        if az is None and self.map is not None:
            az = self.map.heading_at(anchor[:2])
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
            if c.get("on_map"):
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


class Runner:
    def __init__(self, params, track_map=None, origin=None,
                 wheel_timeout=1.0, handle_timeout=0.5, stop_dwell=8.0,
                 **position_opts):
        """position_opts — параметры Position: init_window, projection,
        mgrs_grid, utm_zone, scale_adapt, nomap_mode, keep_offset_xy,
        keep_offset_z, mgrs_guard_m."""
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
        out = self._advance(stamp)
        self.meas[i] = value
        self.fresh[i] = True
        self.t_wheel[i] = stamp
        return out

    def on_handle(self, stamp, position):
        out = self._advance(stamp)
        self.notch = float(position)
        self.t_notch = stamp
        return out

    def on_fix(self, stamp, antenna, lat, lon, alt, status=0):
        """GNSS — только начальная выставка. Сетку ядра GNSS не двигает:
        негодная точка или точка вне окна выставки отбрасывается, годная
        точка окна ставится в очередь Position и применяется на шаге сетки,
        дошедшем до её метки. Поэтому GNSS не влияет на скорость, а после
        окна — ни на что. Якорь и путь выставки пересчитываются только при
        точке, вошедшей в выставку. status — NavSatFix.status.status (< 0:
        нет решения). Выходов не порождает: возвращает []."""
        if self.pos.accepts(stamp, lat, lon, status):
            self.pos.on_fix(stamp, antenna, lat, lon, alt, status=status)
        return []

    # ---------- шаги ----------

    def _advance(self, stamp):
        dt = self.p.dt
        if self.t is None:
            # WP23: узлы сетки кратны dt (совпадают с метками GNSS, кратными
            # 0,1 с); иначе фаза сетки зависит от того, какое сообщение пришло
            # первым (GNSS сетку больше не двигает)
            self.t = math.floor(stamp / dt) * dt
            return []
        outs = []
        while self.t + dt <= stamp + 1e-9:
            self.t += dt
            outs.append(self._step())
        return outs

    def _step(self):
        c, t = self.core, self.t
        wheels_stale = (t - self.t_wheel.max()) > self.wheel_timeout
        handle_ok = (t - self.t_notch) <= self.handle_timeout
        if wheels_stale:
            o = c.step_open_loop(self.notch)
        else:
            o = c.step(self.notch, self.meas, fresh=self.fresh.copy(),
                       handle_ok=handle_ok)
        self.fresh[:] = False
        s = float(c.x[IS])
        # положение: Position (выставка, карта, привязки, выходная система)
        pq = self.pos.step(s, o["mode"] == STANDSTILL and o["valid"], self.p.dt,
                           t=t, v=float(c.x[IV]))
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
                 pos_ready=self.pos.ready, pos_valid=pq is not None,
                 handle_ok=handle_ok, wheels_stale=bool(wheels_stale))
        self.last = o
        return o
