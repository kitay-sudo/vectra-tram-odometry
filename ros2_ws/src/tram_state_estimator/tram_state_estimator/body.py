"""Точки вагона в base_link: антенны GNSS и точка выхода положения.

Ответ организаторов 25.09: base_link - ось
поворота ПЕРЕДНЕЙ тележки на уровне касания колеса и рельса; антенны в нём
(м): master (−9,873; 0; 3,0), rover (2,563; 0; 3,0). Rover впереди, база
12,436 м. Эталон судьи (и карта организаторов pathgraph) - точка base_link:
x, y - ось пути у передней тележки, z - уровень рельса.

Ось x base_link проходит через обе антенны (y = 0 у обеих). Кузов жёсткий и
опирается на оси поворота тележек, которые стоят на оси пути, поэтому
base_link лежит на отрезке master→rover и на кривой:

    base_link = master + 9,873 / 12,436 · (rover − master),  z = z_антенны − 3,0.

Антенны на кривой уходят с оси пути (свесы кузова), base_link - нет. Если
пары антенн одной эпохи нет, точка переносится вдоль курса u (единичный
вектор оси x base_link на плоскости): P_to = P_from + (x_to − x_from)·u, по
высоте - на z_to − z_from.
"""

import math

MASTER_X = -9.873       # м, антенна master в base_link
ROVER_X = 2.563         # м, антенна rover в base_link
ANTENNA_Z = 3.0         # м, высота обеих антенн над уровнем рельса
POINTS = ("base_link", "master", "rover")
OUTPUT_POINTS = ("base_link", "master")


def point_name(name, allowed=POINTS):
    """Каноническое имя точки вагона; неизвестное -> ValueError."""
    key = str(name if name is not None else "base_link").strip().lower()
    key = {"": "base_link", "baselink": "base_link", "base": "base_link",
           "antenna1": "master", "antenna2": "rover"}.get(key, key)
    if key not in allowed:
        raise ValueError(f"точка вагона {name!r}: ожидается {' | '.join(allowed)}")
    return key


class Body:
    """Положение антенн в base_link (y = 0 у обеих)."""

    def __init__(self, master_x=MASTER_X, rover_x=ROVER_X, antenna_z=ANTENNA_Z):
        self.master_x = float(master_x)
        self.rover_x = float(rover_x)
        self.antenna_z = float(antenna_z)
        if not (self.rover_x - self.master_x) > 1.0:
            raise ValueError("rover должен быть впереди master (antenna_rover_x > "
                             "antenna_master_x): курс выставки - по вектору master→rover")

    @property
    def baseline(self):
        """База master→rover, м (12,436 по tf организаторов)."""
        return self.rover_x - self.master_x

    def x(self, name):
        return {"base_link": 0.0, "master": self.master_x, "rover": self.rover_x}[point_name(name)]

    def z(self, name):
        return 0.0 if point_name(name) == "base_link" else self.antenna_z

    def frac(self, to="base_link"):
        """Доля пути master→rover, на которой лежит точка to (base_link: 0,794)."""
        return (self.x(to) - self.master_x) / self.baseline

    def shift(self, xyz, az, src, dst):
        """Точка src вагона (x, y, z во внутренней системе, м) -> точка dst.
        az - курс оси x base_link (азимут сетки от севера по часовой, рад);
        None - курса нет: переносится только высота."""
        x, y, z = (float(v) for v in xyz)
        dz = self.z(dst) - self.z(src)
        if az is None:
            return x, y, z + dz
        d = self.x(dst) - self.x(src)
        return x + d * math.sin(az), y + d * math.cos(az), z + dz

    def from_pair(self, m, r, dst="base_link"):
        """Пара антенн одной эпохи (master m, rover r: x, y, z) -> точка dst.
        Работает поэлементно и для массивов numpy (последняя ось - x, y, z)."""
        f = self.frac(dst)
        dz = self.z(dst) - self.antenna_z
        out = [mi + f * (ri - mi) for mi, ri in zip(_xyz(m), _xyz(r))]
        out[2] = out[2] + dz
        return tuple(out)


def _xyz(p):
    try:
        return p[..., 0], p[..., 1], p[..., 2]         # numpy: (..., 3)
    except (TypeError, IndexError):
        return p[0], p[1], p[2]
