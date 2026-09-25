"""Карта путей и движение по ней.

Карта — набор точек осей путей с направлением движения и высотой, собранный
офлайн из траекторий обучающих прогонов (analysis/build_map.py). Хранится в
широте/долготе/высоте и при выставке переводится в локальную систему прогона.

Движение по карте: на каждом шаге точка сдвигается на пройденный путь по
текущему курсу, затем притягивается к оси пути — к точкам карты в радиусе
snap_r с курсом, отличающимся не больше max_dh. Курс берётся из карты. Встречный
путь отсекается по курсу; на стрелке точек преобладающей ветки больше, и
взвешенное среднее уводит на неё. Вне карты — движение по последнему курсу.
"""

import math

import numpy as np

CELL = 2.0


class TrackMap:
    def __init__(self, lat, lon, alt, head, weight, scale=1.0, stops=None):
        self.lat, self.lon, self.alt = lat, lon, alt
        # точки остановок: (широта, долгота, курс, разброс вдоль пути, м)
        self.stops = np.zeros((0, 4)) if stops is None else np.asarray(stops)
        self.head = head                 # курс, рад от севера по часовой
        self.weight = weight
        self.scale = float(scale)        # путь по карте на метр пути колёс
        self.snap_r = 3.0
        self.max_dh = math.radians(35.0)
        self._xy = None

    @staticmethod
    def load(path):
        z = np.load(path)
        scale = float(z["scale"]) if "scale" in z.files else 1.0
        stops = z["stops"] if "stops" in z.files else None
        return TrackMap(z["lat"], z["lon"], z["alt"], z["head"], z["weight"],
                        scale, stops)

    def save(self, path):
        np.savez_compressed(path, lat=self.lat, lon=self.lon, alt=self.alt,
                            head=self.head, weight=self.weight,
                            scale=self.scale, stops=self.stops)

    # ---------- привязка к системе координат прогона ----------

    def bind(self, enu):
        R = 6378137.0
        k = math.cos(math.radians(enu.lat0))
        x = np.radians(self.lon - enu.lon0) * R * k
        y = np.radians(self.lat - enu.lat0) * R
        self._xy = np.c_[x, y]
        self._z = self.alt - enu.alt0
        self._dir = np.c_[np.sin(self.head), np.cos(self.head)]
        cells = np.floor(self._xy / CELL).astype(np.int64)
        order = np.lexsort((cells[:, 1], cells[:, 0]))
        self._grid = {}
        for i in order:
            self._grid.setdefault((int(cells[i, 0]), int(cells[i, 1])), []).append(i)
        self._grid = {c: np.array(v) for c, v in self._grid.items()}
        st = self.stops
        self._stop_xy = np.c_[np.radians(st[:, 1] - enu.lon0) * R * k,
                              np.radians(st[:, 0] - enu.lat0) * R]

    def _near(self, x, y, h):
        r = self.snap_r
        cx0, cx1 = int(math.floor((x - r) / CELL)), int(math.floor((x + r) / CELL))
        cy0, cy1 = int(math.floor((y - r) / CELL)), int(math.floor((y + r) / CELL))
        idx = [self._grid[(cx, cy)] for cx in range(cx0, cx1 + 1)
               for cy in range(cy0, cy1 + 1) if (cx, cy) in self._grid]
        if not idx:
            return None
        idx = np.concatenate(idx)
        d = self._xy[idx] - (x, y)
        dist = np.hypot(d[:, 0], d[:, 1])
        dh = np.abs(np.angle(np.exp(1j * (self.head[idx] - h))))
        m = (dist <= r) & (dh <= self.max_dh)
        return idx[m] if m.any() else None

    # ---------- курсор ----------

    def locate(self, xyz0, az):
        """Начальная точка курсора: положение и курс выставки."""
        c = dict(x=float(xyz0[0]), y=float(xyz0[1]), z=float(xyz0[2]),
                 h=float(az), on_map=False)
        self._snap(c)
        return c

    def _snap(self, c):
        idx = self._near(c["x"], c["y"], c["h"])
        if idx is None:
            c["on_map"] = False
            return
        w = self.weight[idx] / (1.0 + np.hypot(*(self._xy[idx] - (c["x"], c["y"])).T))
        dvec = (self._dir[idx] * w[:, None]).sum(0)
        h = math.atan2(dvec[0], dvec[1])
        cen = (self._xy[idx] * w[:, None]).sum(0) / w.sum()
        # притяжение поперёк пути к взвешенному центру; вдоль пути — свободно
        t = np.array([math.sin(h), math.cos(h)])
        n = np.array([t[1], -t[0]])
        p = np.array([c["x"], c["y"]])
        p = p - n * float((p - cen) @ n)
        c.update(x=float(p[0]), y=float(p[1]), h=h,
                 z=float((self._z[idx] * w).sum() / w.sum()), on_map=True)

    def advance(self, c, ds):
        """Сдвиг курсора на путь ds (м) вдоль пути. Шагами не длиннее 1 м.

        Путь колёс переводится в путь по карте множителем scale: карта собрана
        из усреднённых положений антенны и чуть короче пути, который мерят
        колёса (калибруется по обучающим прогонам).
        """
        ds = ds * self.scale
        n = max(1, int(math.ceil(abs(ds) / 1.0)))
        for _ in range(n):
            step = ds / n
            c["x"] += step * math.sin(c["h"])
            c["y"] += step * math.cos(c["h"])
            self._snap(c)
            if c["on_map"]:
                c["off"] = 0.0
            else:
                c["off"] = c.get("off", 0.0) + abs(step)
                self._reacquire(c)
        return c

    def anchor(self, c, since):
        """Привязка вдоль пути к известной точке остановки.

        Вызывается, когда вагон стоит дольше порога. Кандидат — точка
        остановки с тем же курсом, не дальше lat_gate поперёк пути и в окне
        вдоль пути 3σ, где σ растёт с путём since после прошлой привязки
        (ошибка вдоль пути копится от масштаба колёс). Остановка в очереди за
        другим трамваем (~30 м до точки) в окно не попадает.
        """
        if not c.get("on_map") or not len(self.stops):
            return False
        t = np.array([math.sin(c["h"]), math.cos(c["h"])])
        n = np.array([t[1], -t[0]])
        d = self._stop_xy - (c["x"], c["y"])
        along, lat = d @ t, d @ n
        dh = np.abs(np.angle(np.exp(1j * (self.stops[:, 2] - c["h"]))))
        sig = np.hypot(self.stops[:, 3], 2.0 + 0.003 * since)
        ok = (np.abs(lat) <= 4.0) & (dh <= self.max_dh) & (np.abs(along) <= 3 * sig)
        if not ok.any():
            return False
        i = np.flatnonzero(ok)[int(np.argmin(np.abs(along[ok])))]
        c["x"], c["y"] = float(self._stop_xy[i, 0]), float(self._stop_xy[i, 1])
        self._snap(c)
        return True

    def _reacquire(self, c):
        """Возврат на карту после ухода с неё: ближайшая точка пути с тем же
        курсом в радиусе, растущем с пройденным вне карты путём (курс мог
        быть неточен с самой выставки)."""
        r = min(self.snap_r + 0.2 * c["off"], 50.0)
        if r <= self.snap_r + 1.0 or int(c["off"]) % 5:
            return
        keep = self.snap_r
        self.snap_r = r
        idx = self._near(c["x"], c["y"], c["h"])
        self.snap_r = keep
        if idx is None:
            return
        d = np.hypot(*(self._xy[idx] - (c["x"], c["y"])).T)
        i = idx[int(np.argmin(d))]
        c.update(x=float(self._xy[i, 0]), y=float(self._xy[i, 1]),
                 h=float(self.head[i]), z=float(self._z[i]))
        self._snap(c)
