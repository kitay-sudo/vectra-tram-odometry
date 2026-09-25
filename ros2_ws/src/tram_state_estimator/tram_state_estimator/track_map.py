"""Карта путей и движение по ней.

Карта — набор точек осей путей с направлением движения, высотой и весом,
собранный офлайн из траекторий обучающих прогонов (analysis/build_map.py).
Хранится в широте/долготе/высоте (WGS84) с ИСТИННЫМИ курсами и при выставке
переводится во внутреннюю непрерывную систему прогона (geodesy.Frame: UTM со
сдвигом в точку выставки): координаты, курсы сетки (отличаются от истинных на
сближение меридианов, здесь ≈1,3°) и множитель пути на точку.

Карту организаторов (формат пока неизвестен) можно подключить без правки
ядра: TrackMap.from_polylines() делает из ломаных/рёбер графа (широта/долгота,
UTM или MGRS) точки того же вида; load() понимает .npz, .geojson/.json и .csv.

Движение по карте: на каждом шаге точка сдвигается на пройденный путь по
текущему курсу, затем притягивается к оси пути — к точкам карты в радиусе
snap_r с курсом, отличающимся не больше max_dh. Курс берётся из карты. Встречный
путь отсекается по курсу; на стрелке точек преобладающей ветки больше, и
взвешенное среднее уводит на неё.

Вне карты курсор идёт по последнему курсу и ищет путь снова (_reacquire):
так проходятся разрывы карты — участки, которых нет в карте (карта собрана
только по тому, где ездили обучающие прогоны). Исключение — тупик у ИЗВЕСТНОЙ
конечной (terminals: места, где начинались и кончались не меньше 3 обучающих
прогонов): там впереди карты нет потому, что дальше пути нет, и курсор стоит в
последней точке пути, а не уходит по прямой (WP11, terminal_hold). Удержание
где угодно («any») опасно: на изогнутом разрыве карты курсор встаёт до конца
прогона (на отложенных с вырезанными 300 м карты — 400 м против 23 м).
"""

import csv
import json
import math
from pathlib import Path

import numpy as np

from .geodesy import (A_WGS, E2_WGS, Frame, mgrs_inv, utm_inv)

CELL = 2.0
SCALE_FRAMES = ("utm", "true", "equirect")
HOLD_MODES = ("off", "terminals", "any")


def hold_mode(v):
    """terminal_hold: off | terminals (по умолчанию) | any; bool — совместимость
    (True — terminals, False — off)."""
    if isinstance(v, bool):
        return "terminals" if v else "off"
    m = str(v).strip().lower()
    if m in ("true", "1", "yes", "on"):
        return "terminals"
    if m in ("false", "0", "no", "none", ""):
        return "off"
    if m not in HOLD_MODES:
        raise ValueError(f"terminal_hold {v!r}: ожидается {HOLD_MODES}")
    return m


def _equirect_scale(lat, head, lat0):
    """Масштаб прежней плоской формулы (сфера a, cos φ0) вдоль head."""
    phi = np.radians(lat)
    w = np.sqrt(1.0 - E2_WGS * np.sin(phi) ** 2)
    M = A_WGS * (1.0 - E2_WGS) / w ** 3
    Nr = A_WGS / w
    ce = A_WGS * math.cos(math.radians(lat0)) / (Nr * np.cos(phi))
    cn = A_WGS / M
    return np.hypot(cn * np.cos(head), ce * np.sin(head))


class TrackMap:
    def __init__(self, lat, lon, alt, head, weight, scale=1.0, stops=None,
                 scale_frame="equirect", terminals=None):
        self.lat, self.lon, self.alt = (np.asarray(v, float).reshape(-1)
                                        for v in (lat, lon, alt))
        # точки остановок: (широта, долгота, истинный курс, разброс вдоль пути, м)
        self.stops = (np.zeros((0, 4)) if stops is None or not len(stops)
                      else np.asarray(stops, float).reshape(-1, 4))
        # известные конечные: (широта, долгота); тупик карты ближе terminal_r
        # к ним — настоящий тупик (удержание), остальные — разрыв карты
        self.terminals = (np.zeros((0, 2)) if terminals is None or not len(terminals)
                          else np.asarray(terminals, float).reshape(-1, 2))
        self.terminal_r = 150.0
        self.head = np.asarray(head, float).reshape(-1)   # истинный курс, рад
        self.weight = np.asarray(weight, float).reshape(-1)
        # путь по карте на метр пути колёс; измерен в системе scale_frame:
        #   "utm"      — во внутренней системе (карты с 26.09),
        #   "true"     — истинные метры (внешняя карта, scale = 1),
        #   "equirect" — прежняя плоская формула (карты до 26.09, 0,99777).
        self.scale = float(scale)
        if scale_frame not in SCALE_FRAMES:
            raise ValueError(f"scale_frame {scale_frame!r}: ожидается {SCALE_FRAMES}")
        self.scale_frame = scale_frame
        self.snap_r = 3.0
        self.max_dh = math.radians(35.0)
        # тупик карты: off — всегда прямо и поиск пути; terminals — стоять
        # только у известных конечных; any — стоять в любом тупике (опасно)
        self.terminal_hold = "terminals"
        self.probe_len = 60.0            # м: насколько далеко искать продолжение
        self._term_xy = np.zeros((0, 2))
        self._xy = None
        self.frame = None

    # ---------- загрузка ----------

    @staticmethod
    def load(path):
        """Карта из файла: .npz (наш формат), .geojson/.json (LineString /
        MultiLineString, lon/lat[/alt]), .csv (колонки line,lat,lon[,alt]
        или line,mgrs_e,mgrs_n с колонкой grid)."""
        path = Path(path)
        suf = path.suffix.lower()
        if suf in (".geojson", ".json"):
            return TrackMap.from_geojson(path)
        if suf == ".csv":
            return TrackMap.from_csv(path)
        z = np.load(path)
        scale = float(z["scale"]) if "scale" in z.files else 1.0
        stops = z["stops"] if "stops" in z.files else None
        frame = str(z["scale_frame"]) if "scale_frame" in z.files else "equirect"
        term = z["terminals"] if "terminals" in z.files else None
        return TrackMap(z["lat"], z["lon"], z["alt"], z["head"], z["weight"],
                        scale, stops, frame, term)

    def save(self, path, **meta):
        np.savez_compressed(path, lat=self.lat, lon=self.lon, alt=self.alt,
                            head=self.head, weight=self.weight,
                            scale=self.scale, stops=self.stops,
                            scale_frame=np.array(self.scale_frame),
                            terminals=self.terminals, **meta)

    @staticmethod
    def from_polylines(lines, crs="latlon", zone=None, grid=None, alt=None,
                       spacing=1.0, bidirectional=True, weight=1.0, scale=1.0,
                       stops=None, find_terminals=False, term_join=10.0):
        """Карта из ломаных (оси путей / рёбра графа организаторов).

        lines — список массивов точек K×2 или K×3:
          crs="latlon": (широта, долгота[, высота]);
          crs="utm":    (E, N[, высота]) зоны zone (северное полушарие);
          crs="mgrs":   (e, n[, высота]) в 100-км квадрате grid ("37UDB"),
                        либо непрерывные от его угла.
        Ломаные нарезаются через spacing м; курс — по направлению ребра.
        bidirectional — путь проходим в обе стороны (для двухпутки с
        односторонним движением передать False и ломаные по ходу движения).
        Высоты нет — берётся alt (число) или 0: тогда z выхода — из выставки.
        scale — множитель пути колёс (1 — истинные метры, scale_frame "true").
        find_terminals — отметить конечными (удержание в тупике, WP11) концы
        ломаных, у которых ближе term_join м нет точек других ломаных
        (висячие вершины графа путей). По умолчанию нет: если в чужой карте
        есть дыры, удержание на краю дыры хуже, чем пройти её по прямой.
        """
        la_all, lo_all, h_all, z_all = [], [], [], []
        ends = []                        # (широта, долгота, № ломаной)
        for ln in lines:
            P = np.asarray(ln, float)
            if P.ndim != 2 or len(P) < 2:
                continue
            if crs == "latlon":
                lat, lon = P[:, 0], P[:, 1]
            elif crs == "utm":
                lat, lon = utm_inv(P[:, 0], P[:, 1], int(zone))
            elif crs == "mgrs":
                lat, lon = mgrs_inv(grid, P[:, 0], P[:, 1])
            else:
                raise ValueError(f"crs {crs!r}: latlon | utm | mgrs")
            h = P[:, 2] if P.shape[1] > 2 else np.full(len(P), 0.0 if alt is None else alt)
            f = Frame(float(lat[0]), float(lon[0]), 0.0, "utm")
            Q = f.fwd_arr(lat, lon, h)
            seg = np.diff(Q[:, :2], axis=0)
            L = np.hypot(seg[:, 0], seg[:, 1])
            s = np.r_[0.0, np.cumsum(L)]
            if s[-1] <= 0.0:
                continue
            u = np.arange(0.0, s[-1] + 1e-9, spacing)
            x, y, zz = (np.interp(u, s, Q[:, k]) for k in range(3))
            j = np.clip(np.searchsorted(s, u, side="right") - 1, 0, len(seg) - 1)
            hg = np.arctan2(seg[j, 0], seg[j, 1])           # курс сетки
            la2, lo2 = f.geodetic(x, y)
            # курс сетки -> истинный: сближение меридианов в точке
            _, h_true_n = f.scale_heading(la2, lo2, np.zeros(len(u)))
            ht = hg - h_true_n
            k = len(ends) // 2
            ends += [(float(la2[0]), float(lo2[0]), k), (float(la2[-1]), float(lo2[-1]), k)]
            for sgn in ((0.0, math.pi) if bidirectional else (0.0,)):
                la_all.append(la2)
                lo_all.append(lo2)
                z_all.append(zz)
                h_all.append(np.angle(np.exp(1j * (ht + sgn))))
        if not la_all:
            raise ValueError("в карте нет ни одной ломаной")
        lat, lon = np.concatenate(la_all), np.concatenate(lo_all)
        term = []
        if find_terminals:
            # висячие концы: рядом нет точек ДРУГИХ ломаных
            own = np.concatenate([np.full(len(a), i // (2 if bidirectional else 1))
                                  for i, a in enumerate(la_all)])
            f = Frame(float(lat[0]), float(lon[0]), 0.0, "utm")
            XY = f.fwd_arr(lat, lon, np.zeros(len(lat)))[:, :2]
            for la_e, lo_e, k in ends:
                p = f.fwd(la_e, lo_e, 0.0)[:2]
                d = np.hypot(XY[:, 0] - p[0], XY[:, 1] - p[1])
                if not ((d <= term_join) & (own != k)).any():
                    term.append((la_e, lo_e))
        return TrackMap(lat, lon, np.concatenate(z_all), np.concatenate(h_all),
                        np.full(len(lat), float(weight)), scale, stops, "true",
                        term or None)

    @staticmethod
    def from_geojson(path, **kw):
        g = json.loads(Path(path).read_text(encoding="utf-8"))
        feats = g.get("features", [g]) if isinstance(g, dict) else g
        lines = []
        for f in feats:
            geom = f.get("geometry", f)
            t, c = geom.get("type"), geom.get("coordinates", [])
            parts = [c] if t == "LineString" else c if t == "MultiLineString" else []
            for p in parts:
                P = np.asarray(p, float)
                P[:, [0, 1]] = P[:, [1, 0]]                 # lon,lat -> lat,lon
                lines.append(P)
        return TrackMap.from_polylines(lines, "latlon", **kw)

    @staticmethod
    def from_csv(path, **kw):
        rows = list(csv.DictReader(open(path, encoding="utf-8")))
        by = {}
        for r in rows:
            by.setdefault(r.get("line", "0"), []).append(r)
        lines = []
        mgrs_mode = rows and "mgrs_e" in rows[0]
        for rs in by.values():
            if mgrs_mode:
                lines.append([(float(r["mgrs_e"]), float(r["mgrs_n"]),
                               float(r.get("alt") or 0.0)) for r in rs])
            else:
                lines.append([(float(r["lat"]), float(r["lon"]),
                               float(r.get("alt") or 0.0)) for r in rs])
        if mgrs_mode:
            return TrackMap.from_polylines(lines, "mgrs", grid=rows[0]["grid"], **kw)
        return TrackMap.from_polylines(lines, "latlon", **kw)

    # ---------- привязка к системе прогона ----------

    def bind(self, frame):
        """Перевод карты во внутреннюю систему прогона (geodesy.Frame; для
        совместимости — любой объект с lat0/lon0/alt0)."""
        if not isinstance(frame, Frame):
            frame = Frame(frame.lat0, frame.lon0, frame.alt0, "utm")
        self.frame = frame
        P = frame.fwd_arr(self.lat, self.lon, self.alt)
        self._xy = np.ascontiguousarray(P[:, :2])
        self._z = P[:, 2].copy()
        k, h = frame.scale_heading(self.lat, self.lon, self.head)
        if self.scale_frame == "utm":
            k = np.ones(len(k))
        elif self.scale_frame == "equirect":
            k = k / _equirect_scale(self.lat, self.head, frame.lat0)
        self._k = k                      # множитель пути: кадр калибровки -> UTM
        self._head = h                   # курс сетки UTM
        self._dir = np.c_[np.sin(h), np.cos(h)]
        # сетка клеток: индексы точек по клеткам (порядок как у lexsort)
        cells = np.floor(self._xy / CELL).astype(np.int64)
        order = np.lexsort((cells[:, 1], cells[:, 0]))
        cs = cells[order]
        self._grid = {}
        if len(order):
            brk = np.flatnonzero((np.diff(cs, axis=0) != 0).any(axis=1)) + 1
            a_, b_ = np.r_[0, brk], np.r_[brk, len(order)]
            keys = zip(cs[a_, 0].tolist(), cs[a_, 1].tolist())
            self._grid = {k: order[a:b] for k, a, b in zip(keys, a_.tolist(), b_.tolist())}
        te = self.terminals
        self._term_xy = (frame.fwd_arr(te[:, 0], te[:, 1], np.zeros(len(te)))[:, :2].copy()
                         if len(te) else np.zeros((0, 2)))
        st = self.stops
        if len(st):
            S = frame.fwd_arr(st[:, 0], st[:, 1], np.zeros(len(st)))
            self._stop_xy = S[:, :2].copy()
            self._stop_head = frame.scale_heading(st[:, 0], st[:, 1], st[:, 2])[1]
        else:
            self._stop_xy = np.zeros((0, 2))
            self._stop_head = np.zeros(0)

    def altitude_at(self, lat, lon, max_d=300.0):
        """Высота карты у точки (lat, lon); None — карты рядом нет."""
        k = math.cos(math.radians(lat))
        d = np.hypot((self.lat - lat) * 111320.0, (self.lon - lon) * 111320.0 * k)
        i = int(np.argmin(d))
        return float(self.alt[i]) if d[i] <= max_d else None

    def _near(self, x, y, h, r=None, max_dh=None):
        r = self.snap_r if r is None else r
        max_dh = self.max_dh if max_dh is None else max_dh
        cx0, cx1 = int(math.floor((x - r) / CELL)), int(math.floor((x + r) / CELL))
        cy0, cy1 = int(math.floor((y - r) / CELL)), int(math.floor((y + r) / CELL))
        idx = [self._grid[(cx, cy)] for cx in range(cx0, cx1 + 1)
               for cy in range(cy0, cy1 + 1) if (cx, cy) in self._grid]
        if not idx:
            return None
        idx = np.concatenate(idx)
        d = self._xy[idx] - (x, y)
        dist = np.hypot(d[:, 0], d[:, 1])
        dh = np.abs(np.angle(np.exp(1j * (self._head[idx] - h))))
        m = (dist <= r) & (dh <= max_dh)
        return idx[m] if m.any() else None

    # ---------- курсор ----------

    def locate(self, xyz0, az):
        """Начальная точка курсора: положение и курс сетки выставки."""
        c = dict(x=float(xyz0[0]), y=float(xyz0[1]), z=float(xyz0[2]),
                 h=float(az), on_map=False, k=1.0, hold=0.0, off=0.0)
        self._snap(c)
        return c

    def heading_at(self, xy, r_near=3.0, r_wide=30.0):
        """Курс пути в точке без курса выставки (нет rover): ближайшая точка
        карты ближе r_near; иначе самая «езженая» в пределах (ближайшая + 5 м),
        если ближайшая ближе r_wide (стоянка на конечной вне карты: карта — из
        точек на ходу). None — карты рядом нет."""
        d = np.hypot(self._xy[:, 0] - xy[0], self._xy[:, 1] - xy[1])
        i = int(np.argmin(d))
        if d[i] < r_near:
            return float(self._head[i])
        if d[i] > r_wide:
            return None
        c = np.flatnonzero(d <= d[i] + 5.0)
        return float(self._head[c[int(np.argmax(self.weight[c]))]])

    def _snap(self, c):
        idx = self._near(c["x"], c["y"], c["h"])
        if idx is None:
            c["on_map"] = False
            return
        w = self.weight[idx] / (1.0 + np.hypot(*(self._xy[idx] - (c["x"], c["y"])).T))
        ws = w.sum()
        dvec = (self._dir[idx] * w[:, None]).sum(0)
        h = math.atan2(dvec[0], dvec[1])
        cen = (self._xy[idx] * w[:, None]).sum(0) / ws
        # притяжение поперёк пути к взвешенному центру; вдоль пути — свободно
        t = np.array([math.sin(h), math.cos(h)])
        n = np.array([t[1], -t[0]])
        p = np.array([c["x"], c["y"]])
        p = p - n * float((p - cen) @ n)
        c.update(x=float(p[0]), y=float(p[1]), h=h,
                 z=float((self._z[idx] * w).sum() / ws),
                 k=float((self._k[idx] * w).sum() / ws), on_map=True)

    def advance(self, c, ds, mult=1.0):
        """Сдвиг курсора на путь колёс ds (м) вдоль пути, шагами не длиннее 1 м.

        Путь колёс переводится в путь по карте множителем scale (калибровка по
        обучающим прогонам) × mult (онлайн-поправка масштаба колёс, Position)
        × k точки (перевод из системы калибровки карты во внутреннюю).
        """
        ds = ds * self.scale * mult
        n = max(1, int(math.ceil(abs(ds) / 1.0)))
        for _ in range(n):
            step = ds / n * c.get("k", 1.0)
            if c.get("hold", 0.0) > 0.0:
                self._held(c, step)
                continue
            was = (c["x"], c["y"], c["z"], c["h"], c.get("on_map", False))
            c["x"] += step * math.sin(c["h"])
            c["y"] += step * math.cos(c["h"])
            self._snap(c)
            if c["on_map"]:
                c["off"] = 0.0
                continue
            if (was[4] and step > 0 and self._may_hold(was[0], was[1])
                    and self._dead_end(was[0], was[1], was[3])):
                # тупик карты у известной конечной: стоим в последней точке пути
                c.update(x=was[0], y=was[1], z=was[2], h=was[3], on_map=True)
                c["hold"] = step
                continue
            c["off"] = c.get("off", 0.0) + abs(step)
            self._reacquire(c)
        return c

    def _may_hold(self, x, y):
        """Можно ли удерживать курсор в тупике в точке (x, y): режим
        terminal_hold и близость к известной конечной."""
        m = hold_mode(self.terminal_hold)
        if m == "any":
            return True
        if m == "off" or not len(self._term_xy):
            return False
        d = np.hypot(self._term_xy[:, 0] - x, self._term_xy[:, 1] - y)
        return bool(d.min() <= self.terminal_r)

    def _dead_end(self, x, y, h, dist=None):
        """Нет ли впереди по курсу продолжения карты (точки с тем же курсом в
        расширяющемся конусе до dist м)? True — тупик."""
        L = self.probe_len if dist is None else dist
        sh, ch = math.sin(h), math.cos(h)
        d = 4.0
        while d <= L:
            r = 3.0 + 0.1 * d
            if self._near(x + d * sh, y + d * ch, h, r=r) is not None:
                return False
            d += max(2.0, 0.5 * r)
        return True

    def _held(self, c, step):
        """Курсор стоит в тупике и копит путь колёс (назад — отпускает). Если
        впереди на (накопленный путь + probe_len) нашлась карта — это был
        разрыв карты, а не тупик: курсор уходит вперёд на накопленный путь."""
        before = c["hold"]
        c["hold"] = before + step
        if c["hold"] <= 0.0:
            c["hold"] = 0.0
            return
        if int(c["hold"] / 10.0) == int(before / 10.0):
            return
        H = c["hold"]
        if not self._dead_end(c["x"], c["y"], c["h"], dist=H + self.probe_len):
            c["hold"] = 0.0
            c["x"] += H * math.sin(c["h"])
            c["y"] += H * math.cos(c["h"])
            self._snap(c)
            if not c["on_map"]:
                c["off"] = H
                self._reacquire(c, force=True)

    def anchor(self, c, since):
        """Привязка вдоль пути к известной точке остановки.

        Вызывается, когда вагон стоит дольше порога. Кандидат — точка
        остановки с тем же курсом, не дальше 4 м поперёк пути и в окне
        вдоль пути 3σ, где σ растёт с путём since после прошлой привязки
        (ошибка вдоль пути копится от масштаба колёс). Остановка в очереди за
        другим трамваем (~30 м до точки) в окно не попадает.
        Возвращает сдвиг курсора вдоль пути (м) или None.
        """
        if not c.get("on_map") or not len(self.stops):
            return None
        t = np.array([math.sin(c["h"]), math.cos(c["h"])])
        n = np.array([t[1], -t[0]])
        d = self._stop_xy - (c["x"], c["y"])
        along, lat = d @ t, d @ n
        dh = np.abs(np.angle(np.exp(1j * (self._stop_head - c["h"]))))
        sig = np.hypot(self.stops[:, 3], 2.0 + 0.003 * since)
        ok = (np.abs(lat) <= 4.0) & (dh <= self.max_dh) & (np.abs(along) <= 3 * sig)
        if not ok.any():
            return None
        i = np.flatnonzero(ok)[int(np.argmin(np.abs(along[ok])))]
        c["x"], c["y"] = float(self._stop_xy[i, 0]), float(self._stop_xy[i, 1])
        c["hold"] = 0.0
        self._snap(c)
        return float(along[i])

    def _reacquire(self, c, force=False):
        """Возврат на карту после ухода с неё: ближайшая точка пути с тем же
        курсом в радиусе, растущем с пройденным вне карты путём (курс мог
        быть неточен с самой выставки)."""
        r = min(self.snap_r + 0.2 * c["off"], 50.0)
        if not force and (r <= self.snap_r + 1.0 or int(c["off"]) % 5):
            return
        idx = self._near(c["x"], c["y"], c["h"], r=r)
        if idx is None:
            return
        d = np.hypot(*(self._xy[idx] - (c["x"], c["y"])).T)
        i = idx[int(np.argmin(d))]
        c.update(x=float(self._xy[i, 0]), y=float(self._xy[i, 1]),
                 h=float(self._head[i]), z=float(self._z[i]))
        self._snap(c)
