"""Карта организаторов pathgraph в оценке (tools/eval.py): поперечная ошибка
(cross-track) и положение вдоль пути по pathgraph.

ТЗ называет поперечную ошибку «при привязке к карте pathgraph»; pathgraph
выдан организаторами (два JSON, по одному на направление, точки через 1 м,
MGRS от угла квадрата 37UCB непрерывно, z - уровень рельса, т. е. это ось
пути точки base_link). Файлы в git не входят (`_incoming/pathgraph/`,
лицензия не ясна): если их нет, метрики не считаются.

Для каждой пары «выход - эталон» (эталон - base_link по GNSS, eval_metrics.
reference) выбирается путь pathgraph своего направления: ближайший, чей
курс отличается от курса эталона меньше чем на 90° (двухпутка: пути ~3,5 м
друг от друга). На этот путь проецируются и эталон, и выход:
  * pg_cross - знаковое расстояние выхода до оси пути (+ слева по ходу);
  * pg_along - s_выхода − s_эталона вдоль пути pathgraph (+ выход впереди);
  * pg_ref_lat, pg_ref_dz - то же для самого эталона (проверка, что pathgraph
    и GNSS base_link - одна и та же точка вагона);
  * пара считается, только если эталон ближе ON_PATH м к пути и проекция не
    за концом пути (записи длиннее pathgraph: конечные, развороты).
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "tram_state_estimator"
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

DEFAULT_DIR = ROOT / "_incoming" / "pathgraph"
GRID_E0, GRID_N0 = 300000.0, 6100000.0      # угол квадрата 37UCB в UTM 37
ON_PATH = 10.0                               # м: эталон дальше от пути - вне pathgraph
COARSE, WIN = 5, 12                          # грубый поиск по каждой 5-й точке, уточнение ±12


class Pathgraph:
    """Пути pathgraph в UTM зоны 37 (абсолютные E, N)."""

    def __init__(self, paths=DEFAULT_DIR):
        from tram_state_estimator.track_map import read_pathgraph
        self.src = str(paths)
        self.paths = []
        for pg in read_pathgraph(paths):
            P = pg["xy"] + (GRID_E0, GRID_N0)
            seg = np.diff(P, axis=0)
            L = np.hypot(seg[:, 0], seg[:, 1])
            self.paths.append(dict(name=pg["name"], P=P, z=pg["z"], seg=seg, L=L,
                                   S=np.r_[0.0, np.cumsum(L)],
                                   yaw=np.arctan2(seg[:, 1], seg[:, 0])))

    @property
    def length(self):
        return [float(p["S"][-1]) for p in self.paths]

    def _project(self, p, Q):
        """Проекция точек Q (n×2, UTM) на путь p: (s, знаковое расстояние,
        расстояние, курс сегмента, z пути, за концом ли)."""
        P, seg, L, S = p["P"], p["seg"], p["L"], p["S"]
        n, K = len(Q), len(P)
        if n == 0:
            e = np.zeros(0)
            return e, e, e, e, e, np.zeros(0, bool)
        C = P[::COARSE]
        k = np.empty(n, int)
        for a in range(0, n, 2000):
            q = Q[a:a + 2000]
            d2 = ((q[:, None, :] - C[None, :, :]) ** 2).sum(-1)
            k[a:a + 2000] = np.argmin(d2, axis=1)
        idx = np.clip(k[:, None] * COARSE + np.arange(-WIN, WIN + 1)[None, :], 0, K - 1)
        d2 = ((P[idx] - Q[:, None, :]) ** 2).sum(-1)
        i = idx[np.arange(n), np.argmin(d2, axis=1)]
        best = None
        for j in (np.clip(i - 1, 0, K - 2), np.clip(i, 0, K - 2)):
            a = P[j]
            u = seg[j] / L[j][:, None]
            dq = Q - a
            t_raw = (dq * u).sum(1)
            t = np.clip(t_raw, 0.0, L[j])
            c = a + u * t[:, None]
            dist = np.hypot(Q[:, 0] - c[:, 0], Q[:, 1] - c[:, 1])
            lat = u[:, 0] * dq[:, 1] - u[:, 1] * dq[:, 0]
            beyond = ((j == 0) & (t_raw < -0.5)) | ((j == K - 2) & (t_raw > L[j] + 0.5))
            cand = (S[j] + t, lat, dist, p["yaw"][j],
                    p["z"][j] + (p["z"][j + 1] - p["z"][j]) * t / L[j], beyond)
            if best is None:
                best = cand
            else:
                m = dist < best[2]
                best = tuple(np.where(m, x, y) for x, y in zip(cand, best))
        return best

    def match(self, E, N, yaw):
        """Путь своего направления для точек (E, N) с курсом yaw (рад от оси
        x против часовой; NaN - любой): (номер пути или −1, s, знаковое
        расстояние, расстояние, z пути, за концом)."""
        Q = np.c_[np.asarray(E, float), np.asarray(N, float)]
        n = len(Q)
        out = dict(k=np.full(n, -1), s=np.full(n, np.nan), lat=np.full(n, np.nan),
                   dist=np.full(n, np.inf), z=np.full(n, np.nan), beyond=np.zeros(n, bool))
        yaw = np.asarray(yaw, float)
        for k, p in enumerate(self.paths):
            s, lat, dist, pyaw, z, beyond = self._project(p, Q)
            same = ~np.isfinite(yaw) | (np.abs(np.angle(np.exp(1j * (pyaw - yaw)))) < math.pi / 2)
            m = same & (dist < out["dist"])
            for key, v in (("s", s), ("lat", lat), ("dist", dist), ("z", z), ("beyond", beyond)):
                out[key] = np.where(m, v, out[key])
            out["k"] = np.where(m, k, out["k"])
        return out

    def on_path(self, k, E, N):
        """Проекция точек на заданные пути k (как у эталона): s, знаковое
        расстояние, за концом."""
        E, N = np.asarray(E, float), np.asarray(N, float)
        s = np.full(len(E), np.nan)
        lat = np.full(len(E), np.nan)
        beyond = np.zeros(len(E), bool)
        for kk, p in enumerate(self.paths):
            m = k == kk
            if m.any():
                s_, lat_, _, _, _, b_ = self._project(p, np.c_[E[m], N[m]])
                s[m], lat[m], beyond[m] = s_, lat_, b_
        return s, lat, beyond


_CACHE = {}


def load(spec):
    """Pathgraph по пути (каталог, файл или список через «;»); None / "none" /
    нет файлов - None. Кэшируется в процессе."""
    if spec in (None, "", "none"):
        return None
    key = str(spec)
    if key not in _CACHE:
        p = Path(key.split(";")[0])
        _CACHE[key] = Pathgraph(key) if p.exists() else None
    return _CACHE[key]


def default_spec():
    """Каталог pathgraph по умолчанию, если он есть (в git его нет)."""
    return str(DEFAULT_DIR) if DEFAULT_DIR.is_dir() and any(DEFAULT_DIR.glob("*.json")) else "none"


def score(pg, E_ref, N_ref, z_ref, yaw_ref, E_est, N_est):
    """Метрики по pathgraph для пар (эталон, выход). Возвращает (row, samples)."""
    r = pg.match(E_ref, N_ref, yaw_ref)
    ok = (r["k"] >= 0) & (r["dist"] <= ON_PATH) & ~r["beyond"]
    s_e, lat_e, beyond_e = pg.on_path(np.where(ok, r["k"], -1), E_est, N_est)
    ok_e = ok & np.isfinite(s_e) & ~beyond_e
    along = np.where(ok_e, s_e - r["s"], np.nan)
    cross = np.where(ok_e, lat_e, np.nan)
    row = dict(pg_pairs=int(ok_e.sum()), pg_frac=float(ok.mean()) if len(ok) else float("nan"))
    fin = lambda x: x[np.isfinite(x)]                 # noqa: E731
    if ok_e.any():
        ca, al = np.abs(fin(cross)), fin(along)
        row.update(pg_cross_mean=float(ca.mean()), pg_cross_p95=float(np.percentile(ca, 95)),
                   pg_cross_max=float(ca.max()), pg_along_mean=float(np.mean(np.abs(al))),
                   pg_along_rmse=float(np.sqrt(np.mean(al ** 2))),
                   pg_along_max=float(np.max(np.abs(al))), pg_along_end=float(al[-1]),
                   pg_along_bias=float(al.mean()))
    if ok.any():
        lr = r["lat"][ok]
        dz = (np.asarray(z_ref, float) - r["z"])[ok]
        row.update(pg_ref_lat_med=float(np.median(np.abs(lr))),
                   pg_ref_lat_signed_med=float(np.median(lr)),
                   pg_ref_lat_p95=float(np.percentile(np.abs(lr), 95)),
                   pg_ref_dz_med=float(np.median(dz[np.isfinite(dz)])) if np.isfinite(dz).any()
                   else float("nan"))
    return row, dict(pg_along=along, pg_cross=cross, pg_ok=ok)
