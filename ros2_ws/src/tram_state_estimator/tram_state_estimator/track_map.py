"""Карта путей и движение по ней.

Карта — набор точек осей путей с направлением движения, высотой и весом,
собранный офлайн из траекторий обучающих прогонов (analysis/build_map.py).
Хранится в широте/долготе/высоте (WGS84) с ИСТИННЫМИ курсами и при выставке
переводится во внутреннюю непрерывную систему прогона (geodesy.Frame: UTM со
сдвигом в точку выставки): координаты, курсы сетки (отличаются от истинных на
сближение меридианов, здесь ≈1,3°) и множитель пути на точку.

Точка вагона, по траектории которой собрана карта, — атрибут point:
"base_link" (ось передней тележки на уровне рельса; карты с 26.09 и карта
организаторов pathgraph) или "master" (антенна; карты до 26.09 без этого
поля). Position ведёт курсором эту точку и переносит её в точку выхода.

Карта организаторов pathgraph (26.09: два JSON, по одному на направление,
точки через 1 м в MGRS от квадрата 37UCB непрерывно, z — уровень рельса)
читается TrackMap.from_pathgraph(); ломаные/рёбра графа (широта/долгота, UTM
или MGRS) — from_polylines(); load() понимает .npz, pathgraph .json (в том
числе каталог или список через «;»), .geojson и .csv.

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

Ветки за тупиком у конечной (поток loops, 27.09; docs/POSITION_FRAME.md,
«Конечные и разворотные кольца»). У западной конечной облако точек ведёт
курсор на платформу высадки: туда приходят почти все записи. Платформа в
облаке кончается тупиком, и курсор стоит в нём (удержание выше). Но
изредка вагон едет дальше: по разворотному кольцу (после высадки) или
ещё до платформы уходит стрелкой на боковой путь (без остановки). Колёса
ветку не видят, зато видно, что вагон уехал за тупик дальше BR_MIN м:
на платформе так не бывает. Тогда курсор переходит на ветку этого тупика
(analysis/build_map.py terminal_branches: оси веток по обучающим
проходам, которые уехали за тупик) — на путь, пройденный от её начала,
и дальше ведётся по её оси. Какая ветка: стоял ли вагон перед тупиком
(окно window м) — у кольца стояли (высадка), у бокового пути — нет; и
сколько проходов было по каждой. Если вагон стоит у платформы и за тупик не
уезжает, ничего не меняется: удержание как прежде. Карта без веток — как
до 27.09.
"""

import csv
import json
import math
from pathlib import Path

import numpy as np

from .body import point_name
from .geodesy import (A_WGS, E2_WGS, Frame, mgrs_inv, utm_fwd, utm_inv)

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


PATHGRAPH_GRID = "37UCB"     # pathgraph: MGRS от угла 37UCB непрерывно (x > 100 км восточнее E = 400 км)


def _pathgraph_files(paths):
    if isinstance(paths, (list, tuple)):
        out = []
        for p in paths:
            out += _pathgraph_files(p)
        return out
    spec = str(paths)
    if ";" in spec:
        return _pathgraph_files([p for p in spec.split(";") if p.strip()])
    p = Path(spec.strip())
    if p.is_dir():
        return sorted(p.glob("*.json"))
    return [p]


def is_pathgraph(path):
    """Файл .json в формате pathgraph организаторов (поле points, не GeoJSON)."""
    try:
        with open(path, encoding="utf-8") as fh:
            head = fh.read(4096)
    except OSError:
        return False
    return '"points"' in head and '"type"' not in head


def read_pathgraph(paths):
    """pathgraph организаторов: {"points": [{x, y, z, tang, curv}], "paths":
    [{"point_indices": [...]}]}. x, y — MGRS от угла квадрата 37UCB
    непрерывно (x = E − 300 000, y = N − 6 100 000 зоны 37), z — высота
    уровня рельса (м, та же система высот, что NavSatFix), tang — угол
    касательной от оси x против часовой (рад, REP-103), curv — кривизна (1/м).
    Возвращает список путей: dict(name, xy K×2, z, tang, curv) в порядке
    point_indices (по ходу движения)."""
    out = []
    for f in _pathgraph_files(paths):
        d = json.loads(Path(f).read_text(encoding="utf-8"))
        P = np.array([[p["x"], p["y"], p.get("z", 0.0), p.get("tang", np.nan),
                       p.get("curv", np.nan)] for p in d["points"]], float)
        paths_ = d.get("paths") or [{"point_indices": list(range(len(P)))}]
        for k, path in enumerate(paths_):
            idx = np.asarray(path.get("point_indices", []), int)
            if len(idx) < 2:
                continue
            Q = P[idx]
            name = Path(f).stem + (f"#{k}" if len(paths_) > 1 else "")
            out.append(dict(name=name, xy=Q[:, :2].copy(), z=Q[:, 2].copy(),
                            tang=Q[:, 3].copy(), curv=Q[:, 4].copy()))
    if not out:
        raise ValueError(f"pathgraph {paths}: нет ни одного пути")
    return out


def _equirect_scale(lat, head, lat0):
    """Масштаб прежней плоской формулы (сфера a, cos φ0) вдоль head."""
    phi = np.radians(lat)
    w = np.sqrt(1.0 - E2_WGS * np.sin(phi) ** 2)
    M = A_WGS * (1.0 - E2_WGS) / w ** 3
    Nr = A_WGS / w
    ce = A_WGS * math.cos(math.radians(lat0)) / (Nr * np.cos(phi))
    cn = A_WGS / M
    return np.hypot(cn * np.cos(head), ce * np.sin(head))


BRANCH_KEYS = ("bp", "bi")      # ветки за тупиком в .npz (set_branches)


class TrackMap:
    # ветки за тупиком у конечной (docstring модуля)
    branches_on = True       # False — только удержание в тупике (как до 27.09)
    BR_MIN = 15.0            # м: курсор в тупике накопил столько пути — вагон уехал дальше
    BR_DE_R = 6.0            # м: тупик ветки не дальше этого от точки удержания
    BR_KEEP_STOPS = 8        # сколько последних стоянок помнит курсор

    def __init__(self, lat, lon, alt, head, weight, scale=1.0, stops=None,
                 scale_frame="equirect", terminals=None, point="master"):
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
        # точка вагона, по траектории которой собрана карта (z карты — её высота)
        self.point = point_name(point)
        self.snap_r = 3.0
        self.max_dh = math.radians(35.0)
        # тупик карты: off — всегда прямо и поиск пути; terminals — стоять
        # только у известных конечных; any — стоять в любом тупике (опасно)
        self.terminal_hold = "terminals"
        self.probe_len = 60.0            # м: насколько далеко искать продолжение
        self._term_xy = np.zeros((0, 2))
        self._xy = None
        self._ucache = None              # prepare(): (зона, север) -> величины
        self.frame = None
        # ветки за тупиком (set_branches; в .npz — ключи bp, bi)
        self.bp = np.zeros((0, 6))       # точки осей: ветка, s, широта, долгота, высота, истинный курс
        self.bi = np.zeros((0, 7))       # ветки: тупик (широта, долгота, истинный курс), offset,
        #                                  window, проходов, из них стояли в окне
        self._br = []

    # ---------- загрузка ----------

    @staticmethod
    def load(path):
        """Карта из файла: .npz (наш формат), pathgraph организаторов (.json с
        полем points; каталог с такими .json или список путей через «;»),
        .geojson/.json (LineString / MultiLineString, lon/lat[/alt]), .csv
        (колонки line,lat,lon[,alt] или line,mgrs_e,mgrs_n с колонкой grid)."""
        spec = str(path)
        if ";" in spec or Path(spec).is_dir():
            return TrackMap.from_pathgraph(spec)
        path = Path(path)
        suf = path.suffix.lower()
        if suf in (".geojson", ".json"):
            if is_pathgraph(path):
                return TrackMap.from_pathgraph(path)
            return TrackMap.from_geojson(path)
        if suf == ".csv":
            return TrackMap.from_csv(path)
        z = np.load(path)
        scale = float(z["scale"]) if "scale" in z.files else 1.0
        stops = z["stops"] if "stops" in z.files else None
        frame = str(z["scale_frame"]) if "scale_frame" in z.files else "equirect"
        term = z["terminals"] if "terminals" in z.files else None
        # карты до 26.09 собраны по антенне master
        point = str(z["point"]) if "point" in z.files else "master"
        m = TrackMap(z["lat"], z["lon"], z["alt"], z["head"], z["weight"],
                     scale, stops, frame, term, point)
        if all(k in z.files for k in BRANCH_KEYS):
            m.set_branches(*(z[k] for k in BRANCH_KEYS))
        m.prepare()                      # при загрузке, не в колбэке выставки
        return m

    def save(self, path, **meta):
        extra = {k: getattr(self, k) for k in BRANCH_KEYS} if len(self.bi) else {}
        np.savez_compressed(path, lat=self.lat, lon=self.lon, alt=self.alt,
                            head=self.head, weight=self.weight,
                            scale=self.scale, stops=self.stops,
                            scale_frame=np.array(self.scale_frame),
                            terminals=self.terminals, point=np.array(self.point),
                            **extra, **meta)

    def set_branches(self, bp, bi):
        """Ветки за тупиком (analysis/build_map.py terminal_branches): точки
        осей bp (M×6: № ветки, путь от начала ветки s, широта, долгота,
        высота, истинный курс; по возрастанию s внутри ветки) и ветки bi (K×7:
        тупик облака — широта, долгота, истинный курс; offset — путь от
        начала ветки до тупика по облаку; window — окно стоянки перед
        тупиком, м; число обучающих проходов; из них стояли в окне)."""
        self.bp = np.asarray(bp, float).reshape(-1, 6)
        self.bi = np.asarray(bi, float).reshape(-1, 7)
        self._ucache = None
        return self

    @staticmethod
    def from_pathgraph(paths, grid=PATHGRAPH_GRID, weight=1.0, scale=1.0, stops=None,
                       terminals=None):
        """Карта организаторов pathgraph -> TrackMap (точка base_link, z —
        уровень рельса; путь в каждом файле — по ходу движения, поэтому не
        в обе стороны). paths — файл, каталог с .json или список (строка через
        «;»). Точки идут через 1 м; курс — по направлению ребра (tang файла с
        ним совпадает до 1°, test_pathgraph). scale — множитель пути колёс в
        истинных метрах (scale_frame "true"); analysis/build_map.py калибрует
        его во внутренней системе и ставит "utm"."""
        lines = [np.c_[pg["xy"], pg["z"]] for pg in read_pathgraph(paths)]
        m = TrackMap.from_polylines(lines, crs="mgrs", grid=grid, spacing=1.0,
                                    bidirectional=False, weight=weight, scale=scale,
                                    stops=stops, point="base_link")
        if terminals is not None:
            m.terminals = np.asarray(terminals, float).reshape(-1, 2)
        return m

    @staticmethod
    def from_polylines(lines, crs="latlon", zone=None, grid=None, alt=None,
                       spacing=1.0, bidirectional=True, weight=1.0, scale=1.0,
                       stops=None, find_terminals=False, term_join=10.0,
                       point="base_link"):
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
        point — точка вагона, которую описывают ломаные: ось пути — base_link
        (по умолчанию), траектория антенны — master.
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
                        term or None, point)

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

    def prepare(self, frame=None):
        """Величины карты, зависящие только от зоны UTM (не от точки
        выставки): абсолютные E, N точек, остановок и конечных, масштаб и
        курс сетки. Считаются один раз — при загрузке карты (для зоны её
        центра) или при первой привязке; bind() в колбэке выставки тогда
        только вычитает начало (перевод 27 тыс. точек рядами Крюгера — это
        десятки мс)."""
        if frame is None:
            frame = Frame(float(np.mean(self.lat)), float(np.mean(self.lon)), 0.0, "utm")
        # смена точек/остановок/конечных после привязки — пересчёт
        key = (frame.zone, frame.north, id(self.lat), id(self.head),
               id(self.stops), id(self.terminals), id(self.bp), id(self.bi))
        if self._ucache is not None and self._ucache[0] == key:
            return self._ucache[1]
        E, N = utm_fwd(self.lat, self.lon, frame.zone, frame.north)
        k, h = frame.scale_heading(self.lat, self.lon, self.head)
        u = dict(E=E, N=N, k=k, h=h)
        st, te = self.stops, self.terminals
        if len(st):
            u["SE"], u["SN"] = utm_fwd(st[:, 0], st[:, 1], frame.zone, frame.north)
            u["Sh"] = frame.scale_heading(st[:, 0], st[:, 1], st[:, 2])[1]
        if len(te):
            u["TE"], u["TN"] = utm_fwd(te[:, 0], te[:, 1], frame.zone, frame.north)
        if len(self.bi) and len(self.bp):
            bp, bi = self.bp, self.bi
            u["BE"], u["BN"] = utm_fwd(bp[:, 2], bp[:, 3], frame.zone, frame.north)
            u["Bh"] = frame.scale_heading(bp[:, 2], bp[:, 3], bp[:, 5])[1]
            u["DE"], u["DN"] = utm_fwd(bi[:, 0], bi[:, 1], frame.zone, frame.north)
            u["Dh"] = frame.scale_heading(bi[:, 0], bi[:, 1], bi[:, 2])[1]
        self._ucache = (key, u)
        return u

    def bind(self, frame):
        """Перевод карты во внутреннюю систему прогона (geodesy.Frame; для
        совместимости — любой объект с lat0/lon0/alt0)."""
        if not isinstance(frame, Frame):
            frame = Frame(frame.lat0, frame.lon0, frame.alt0, "utm")
        self.frame = frame
        u = self.prepare(frame)
        self._xy = np.ascontiguousarray(np.c_[u["E"] - frame.E0, u["N"] - frame.N0])
        self._z = self.alt.astype(float).copy()
        k, h = u["k"], u["h"]
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
        self._term_xy = (np.c_[u["TE"] - frame.E0, u["TN"] - frame.N0]
                         if len(self.terminals) else np.zeros((0, 2)))
        if len(self.stops):
            self._stop_xy = np.c_[u["SE"] - frame.E0, u["SN"] - frame.N0]
            self._stop_head = u["Sh"]
        else:
            self._stop_xy = np.zeros((0, 2))
            self._stop_head = np.zeros(0)
        self._bind_branches(u, frame)

    def _bind_branches(self, u, frame):
        """Ветки за тупиком во внутренней системе: по ветке — s, xy, z, курс
        сетки (sin/cos для интерполяции), тупик, offset, окно, счётчики."""
        self._br = []
        if "BE" not in u:
            return
        xy = np.c_[u["BE"] - frame.E0, u["BN"] - frame.N0]
        kid = self.bp[:, 0].astype(int)
        for k in range(len(self.bi)):
            m = np.flatnonzero(kid == k)
            if len(m) < 2:
                continue
            m = m[np.argsort(self.bp[m, 1], kind="stable")]
            h = u["Bh"][m]
            de_lat, de_lon, _, off, win, n, ns = self.bi[k]
            self._br.append(dict(
                k=k, s=self.bp[m, 1], xy=xy[m], z=self.bp[m, 4], hs=np.sin(h), hc=np.cos(h),
                de=(float(u["DE"][k] - frame.E0), float(u["DN"][k] - frame.N0)),
                de_h=float(u["Dh"][k]), off=float(off), win=float(win), n=float(n),
                ns=float(ns)))

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
            if self._br:
                c["odo"] = c.get("odo", 0.0) + abs(step)   # путь курсора (окно стоянки)
            if "br" in c:
                self._br_step(c, step)            # на ветке за тупиком: по её оси
                continue
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
        if c["hold"] >= self.BR_MIN > before and self._br_switch(c):
            return                        # уехал за тупик: на ветку
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
        if self._br:
            # стоянка — свидетельство для выбора ветки за тупиком (путь курсора)
            st = c.setdefault("stops", [])
            st.append(c.get("odo", 0.0))
            del st[:-self.BR_KEEP_STOPS]
            if "br" in c:
                return None               # на ветке точек остановок нет
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

    # ---------- ветки за тупиком у конечной ----------

    def _br_switch(self, c):
        """Курсор стоит в тупике и накопил BR_MIN м пути: вагон уехал дальше.
        Ветки этого тупика (тупик ближе BR_DE_R, тот же курс) — выбор по
        числу проходов и по стоянке в окне перед тупиком: вес ветки
        (n + 0,5) · P(стоял | ветка), P = (стояли + 0,5) / (n + 1). Курсор —
        на ветку, на путь offset + накопленный. False — веток нет."""
        if not self.branches_on or not self._br:
            return False
        H = c["hold"]
        best, key = None, None
        for B in self._br:
            if (math.hypot(B["de"][0] - c["x"], B["de"][1] - c["y"]) > self.BR_DE_R
                    or abs(math.remainder(B["de_h"] - c["h"], 2 * math.pi)) > self.max_dh):
                continue
            # стоял ли в окне перед тупиком (путь курсора: тупик — odo − H)
            o_de = c.get("odo", 0.0) - H
            stood = any(o >= o_de - B["win"] for o in c.get("stops", ()))
            p = (B["ns"] + 0.5) / (B["n"] + 1.0)
            sc = math.log(B["n"] + 0.5) + math.log(p if stood else 1.0 - p)
            if key is None or sc > key:
                best, key = B, sc
        if best is None:
            return False
        c["hold"] = 0.0
        c["br"] = dict(k=int(best["k"]), s=best["off"] + H)
        self._br_place(c)
        return True

    def _br_axis(self, B, s):
        """Точка оси ветки B на пути s от её начала: x, y, z, курс сетки."""
        S = B["s"]
        x = float(np.interp(s, S, B["xy"][:, 0]))
        y = float(np.interp(s, S, B["xy"][:, 1]))
        z = float(np.interp(s, S, B["z"]))
        h = math.atan2(float(np.interp(s, S, B["hs"])), float(np.interp(s, S, B["hc"])))
        return x, y, z, h

    def _br_ref(self, k):
        return next(B for B in self._br if B["k"] == k)

    def _br_step(self, c, step):
        c["br"]["s"] += step
        self._br_place(c)

    def _br_place(self, c):
        """Курсор на ветке: точка оси. За её концом — снова облако (дальше
        по курсу конца оси); если облака там нет — тупик: удержание в конце
        оси. Назад за начало оси — облако от начала."""
        B = self._br_ref(c["br"]["k"])
        s, S = c["br"]["s"], B["s"]
        if S[0] <= s <= S[-1]:
            x, y, z, h = self._br_axis(B, s)
            c.update(x=x, y=y, z=z, h=h, on_map=True, off=0.0, hold=0.0)
            return
        end = S[-1] if s > S[-1] else S[0]
        over = s - end
        x, y, z, h = self._br_axis(B, end)
        del c["br"]
        c.update(x=x + over * math.sin(h), y=y + over * math.cos(h), z=z, h=h, hold=0.0)
        self._snap(c)
        if c["on_map"]:
            c["off"] = 0.0
            return
        if over > 0.0 and self._may_hold(x, y) and self._dead_end(x, y, h):
            c.update(x=x, y=y, z=z, h=h, on_map=True, off=0.0, hold=over)
            return
        c["off"] = abs(over)
        self._reacquire(c, force=True)
