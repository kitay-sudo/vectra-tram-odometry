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


class Runner:
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

    def _advance(self, stamp):
        dt = self.p.dt
        if self.t is None:
            self.t = stamp
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
        if self.pos.ready:
            if self.s0 is None:
                self.s0 = s
            x, y, z = self.pos.xyz(s - self.s0)
            standing = o["mode"] == STANDSTILL and o["valid"]
            self._dwell = self._dwell + self.p.dt if standing else 0.0
            if self._dwell >= self.stop_dwell > self._dwell - self.p.dt:
                self.pos.on_stop(s - self.s0)
                x, y, z = self.pos.xyz(s - self.s0)
        else:
            # выставки ещё не было (GNSS нет): относительная одометрия по x
            x, y, z = s, 0.0, 0.0
        v = float(c.x[IV])
        a = (body_force(c.u_filt, v, c.x[3], c.x[4], c.mu, c.p)
             - resistance(v, c.p)) / c.p.M_nom + float(c.x[ID])
        cur = self.pos._cursor
        az = cur["h"] if cur is not None else self.pos.az
        o.update(stamp=t, x=x, y=y, z=z,
                 yaw=(math.pi / 2.0 - az) if self.pos.ready else None, a=float(a) if v > 0 or a > 0 else 0.0,
                 pos_ready=self.pos.ready, handle_ok=handle_ok,
                 wheels_stale=bool(wheels_stale))
        self.last = o
        return o
