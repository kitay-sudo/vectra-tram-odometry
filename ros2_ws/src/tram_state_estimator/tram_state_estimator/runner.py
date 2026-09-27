"""Связка ядра с входными сообщениями. Без ROS: её используют и нода, и
офлайн-оценка по bag, поэтому результаты совпадают.

Время - метки сообщений (header.stamp), не часы узла. Ядро шагает на
равномерной сетке с шагом p.dt: когда приходит сообщение с меткой t, делаются
все шаги сетки до t. Каждая тележка в шаге отмечается свежей, только если её
показание пришло после прошлого шага.

GNSS в окне init_window секунд от первой годной точки - начальная выставка:
начало координат, высота и курс. После окна (gnss_correction, по умолчанию
включено; по ответу организаторов, «в середине маршрута могут быть ещё
сообщения, которые можно использовать для коррекции») точки GNSS
поправляют положение вдоль пути, выбор ветки карты и σ положения (Position,
раздел «коррекция по GNSS»). Сетку ядра GNSS не двигает никогда: годная точка
ставится в очередь и применяется, когда сетка (её двигают только тележки и
ручка) дойдёт до её метки; данные «из будущего» не используются. Скорость от
GNSS не зависит вовсе. gnss_correction: false - GNSS только для выставки:
точки вне окна отбрасываются сразу (docs/POSITION_FRAME.md).

Положение считается во внутренней непрерывной системе (UTM со сдвигом в точку
выставки, geodesy.Frame) и переводится в выходную (по умолчанию MGRS от
квадрата 37UCB непрерывно, как карта организаторов) только при выдаче. Точка
выхода - base_link (ось передней тележки, уровень рельса), не антенна.
"""

import copy
import math
from collections import deque

import numpy as np

from .estimator_core import (Estimator, IV, ID, IS, STANDSTILL, body_force,
                             position_sigma, resistance, sensor_to_speed)
from .body import ANTENNA_Z, MASTER_X, OUTPUT_POINTS, ROVER_X, Body, point_name
from .geodesy import Equirect, Frame, projection_name
from .track_map import hold_mode

R_EARTH = 6378137.0

# второе имя плоской формулы (equirect, не строгий ENU): им пользуются
# tools/core_metrics.py и analysis/calib_sigma.py; нода его не использует
Enu = Equirect


class Position:
    """Положение по пройденному пути колёс.

    Точка выхода - output_point: "base_link" (по умолчанию; ось поворота
    передней тележки на уровне рельса - так строит эталон судья и карта
    организаторов pathgraph) или "master" (антенна GNSS). Антенны в
    base_link - antenna_master_x, antenna_rover_x, antenna_z (body.py).
    Курсор по карте ведёт ту точку вагона, по траектории которой собрана
    карта (TrackMap.point: base_link у карт пакета и у pathgraph, master у
    старых карт без этого поля); без карты - сразу точку выхода. В выход
    точка переносится вдоль курса пути (body.Body.shift).

    Выставка (только в окне init_window от первой годной точки GNSS):
      * якорь антенны - среднее точек master окна, если вагон стоял (путь
        колёс за окно < 0,3 м), иначе ПОСЛЕДНЯЯ точка master окна с путём
        колёс на её метку (старт на ходу);
      * курс - по парам master→rover одной эпохи (rover впереди); без rover -
        по смещению master за окно (> 5 м), иначе по карте в точке якоря;
        иначе курса нет, и выход стоит в точке якоря;
      * нет master, есть rover - якорь антенны по rover;
      * якорь антенны -> ведомая точка: на ходу при паре master+rover у
        последней точки - по паре (base_link = master + 9,873/12,436 ·
        (rover − master)), иначе перенос вдоль курса; высота - минус
        antenna_z (base_link - уровень рельса); без курса - только высота;
      * якорь и путь выставки s_ref пересчитываются только при точке, вошедшей
        в выставку; после окна GNSS не читается;
      * точка с меткой дальше MAX_SKEW от сетки ядра - сбой метки: не
        выставка (и не может открыть окно).
    Движение: с картой - курсор по карте (track_map.py) с привязкой к точкам
    остановок и онлайн-подстройкой масштаба пути; без карты - стоянка
    в якоре (nomap_mode "hold", по умолчанию: на отложенных 2,3 км против
    3,1 км) или прямая вдоль курса выставки ("line").

    Коррекция по GNSS после окна (gnss_correction; docs/POSITION_FRAME.md):
      * точка годна, если конечна, не (0, 0), статус ≥ 0, и её метка не дальше
        gnss_max_skew_s от метки последнего входа (тележки, ручка) на момент
        прихода: в данных есть участки, где метки GNSS сдвинуты на ±1 с
        относительно часов тележек (проверено по скорости), - такие точки не
        берутся; точка ждёт шага сетки, ушедшего на CORR_DELAY за её метку
        (успевает прийти пара), и сравнивается с положением курсора на СВОЮ
        метку (путь ядра по истории сетки: v·Δt учтено, будущего нет);
      * эпоха: master + rover одной метки с базой CORR_PAIR_MIN…12,44 +
        CORR_BASE_TOL (кузов сочленённый: на тесных кривых база до ~10 м,
        длиннее - сбой антенны, эпоха не берётся) - base_link по tf на отрезке
        антенн; курс пары - только у «чистой» пары (база 12,44 ±
        CORR_BASE_TOL); одна антенна (второй нет) - перенос вдоль курса карты;
      * σ точки по NavSatFix.status (2 - RTK: gnss_sigma_rtk_m, 1 - SBAS,
        0 - без поправок: gnss_sigma_fix_m), по заявленной ковариации, если
        она есть (в данных пустая); статус эпохи - худший из двух антенн, а
        эпохи только с rover - не лучше статуса master (rover меряется от
        master: подвижная база, _status);
      * обновление Калмана вдоль пути: невязка ν вдоль касательной курсора,
        априорная σ - σ_s связки с ростом gnss_prior_rel на метр пути (Runner
        передаёт её в step), K = P/(P + R); курсор сдвигается по карте на K·ν,
        σ_s после - √((1−K)P); не чаще gnss_min_interval_s (ошибки соседних
        точек связаны);
      * без RTK (статус эпохи < 2) - только малые поправки (|K·ν| ≤
        gnss_jump_m, не чаще CORR_NONRTK_INTERVAL_S), окно привязки к
        остановкам они не сужают; RTK-поправка сужает его только до σ после
        поправки;
      * отбраковка: правдоподобная большая поправка (|ν| ≤ gnss_gate·√(P+R),
        но K·ν > gnss_jump_m) - только RTK, после gnss_confirm_n согласных
        эпох подряд;
        неправдоподобная (вне ворот, поперёк пути дальше max(2,5 м, 3σ),
        чистая пара против курса пути больше CORR_HEAD_TOL) - только RTK и
        только если держится gnss_persist_s с той же невязкой (скачок сразу
        после согласной эпохи - не меньше CORR_PERSIST_JUMP_S): короче -
        это сбой GNSS, он отбрасывается. Поправка больше CORR_ALONG_MAX,
        поперечная или против курса (другая ветка, встречный путь, курсор вне
        карты) - перестановка курсора в точку GNSS на путь карты с курсом
        пары, а если карты там нет - в саму точку с курсом чистой пары;
      * без выставки или без курса (старт с середины, GNSS в начале не было)
        первая годная точка после окна открывает окно выставки заново;
      * gnss_scale_adapt: отрезки между RTK-поправками (≥ CORR_SCALE_L м пути)
        подстраивают масштаб пути тем же _adapt_scale, что и привязки к
        остановкам; gnss_stop_skip_m - после поправки GNSS на столько метров
        пути привязка к остановке не делается (GNSS точнее разброса точки
        остановки).
    """

    PAIR_TOL = 0.06              # с: master и rover одной эпохи
    BASE_MIN, BASE_MAX = 5.0, 25.0   # м: допустимая длина базы (≈12,4 м)
    # онлайн-масштаб пути по привязкам к остановкам (scale_adapt,
    # docs/POSITION_FRAME.md)
    SCALE_PRIOR_L = 3000.0       # м пути колёс: «априорный» вес исходного масштаба
    SCALE_MAX_DEV = 0.01         # ограничение ±1 %
    SCALE_GATE_K, SCALE_GATE_C = 0.01, 2.0   # принимать сдвиг |δ| < 1 %·L + 2 м
    SCALE_MIN_L = 100.0          # м: короче - отрезок не учитывается
    SCALE_INCONS = 0.012         # отрезки разошлись больше - подстройку выключить
    OFFSET_Z_MAX = 30.0          # м: больший сдвиг высоты GNSS − карта не переносить
    MAX_SKEW = 30.0              # с: метка GNSS дальше от сетки ядра - сбой метки
    ROVER_ONLY_AFTER = 1.0       # с: rover без master дольше - выставка по rover
    # коррекция по GNSS после окна выставки (gnss_correction)
    CORR_DELAY = 0.1             # с: точка ждёт пару, пока сетка не уйдёт за её метку на столько
    CORR_HIST_S = 3.0            # с: история пути ядра по сетке (путь на метку точки)
    CORR_MAX_Q = 400             # точек в очереди коррекции (провал тележек: сетка стоит)
    CORR_BASE_TOL = 2.0          # м: «чистая» пара - база 12,44 ± это (курс пары)
    CORR_PAIR_MIN = 9.0          # м: пара для коррекции - база не короче (петли ~10 м)
    CORR_HEAD_TOL = math.radians(30.0)   # курс пары против курса пути
    CORR_CROSS_MIN = 2.5         # м: поперёк дальше max(этого, 3σ) - другая ветка или сбой
    CORR_CONFIRM_S = 3.0         # с: подтверждающие эпохи - не дальше друг от друга
    CORR_PERSIST_JUMP_S = 30.0   # с: а если несогласие началось скачком GNSS - столько
    CORR_JUMP_DT = 1.0           # с: скачок - сразу после согласной эпохи
    CORR_JUMP_M = 2.0            # м: и невязка изменилась больше этого + 3σ точки
    CORR_NONRTK_INTERVAL_S = 20.0    # с: поправки без RTK не чаще (ошибка держится
                                     # десятки секунд - повтор ничего не добавляет)
    CORR_MSTAT_S = 10.0          # с: эпоха только с rover - статус master не старше этого
    # окно привязки к остановке (TrackMap.anchor): σ = hypot(σ точки, SD0 + REL·путь)
    ANCHOR_SD0, ANCHOR_REL = 2.0, 0.003
    CORR_PEND_MAX = 300          # эпох в очереди подтверждения
    CORR_DRIFT_TOL = 0.02        # доля пути: рост невязки вдоль за время подтверждения
                                 # (колёса врут до ~2 % при срыве) - ещё согласие
    CORR_SINGLE_SD = 0.5         # м: добавка к σ одной антенны (курс карты × плечо антенны)
    CORR_VAR_FLOOR = 0.2 ** 2    # м²: σ² после поправки не меньше этого
    CORR_VAR_FRAC = 0.25         # и не меньше этой доли σ² точки: ошибки соседних
                                 # точек GNSS связаны во времени, повтор их не усредняет
    CORR_PRIOR_SD = 2.0          # м: априорная σ, если связка её не передала
    CORR_ALONG_MAX = 50.0        # м: больше - только перестановкой курсора в точку GNSS
    CORR_RELOC_MAX = 5000.0      # м: дальше - не поправка, а сбой (вся линия ~5 км)
    CORR_SCALE_L = 300.0         # м пути: отрезок между RTK-поправками для масштаба

    def __init__(self, track_map=None, origin=None, init_window=3.0,
                 projection="mgrs", mgrs_grid="37UCB", utm_zone=0, stop_dwell=8.0,
                 scale_adapt=True, nomap_mode="hold", keep_offset_xy=True,
                 keep_offset_z=True, keep_offset_max_status=-1, mgrs_guard_m=0.0,
                 terminal_hold="terminals", output_point="base_link",
                 antenna_master_x=MASTER_X, antenna_rover_x=ROVER_X,
                 antenna_z=ANTENNA_Z, gnss_correction=True, gnss_sigma_rtk_m=0.5,
                 gnss_sigma_sbas_m=1.5, gnss_sigma_fix_m=5.0, gnss_gate=3.0,
                 gnss_jump_m=3.0, gnss_confirm_n=3, gnss_min_interval_s=1.0,
                 gnss_max_skew_s=0.3, gnss_prior_rel=0.003, gnss_scale_adapt=False,
                 gnss_stop_skip_m=0.0, gnss_persist_s=10.0):
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
        # точка вагона на выходе и ведомая курсором (по ней собрана карта)
        self.body = Body(antenna_master_x, antenna_rover_x, antenna_z)
        self.output_point = point_name(output_point, OUTPUT_POINTS)
        self.track_point = (point_name(getattr(track_map, "point", "master"))
                            if track_map is not None else self.output_point)
        self.stop_dwell = float(stop_dwell)
        self.scale_adapt = bool(scale_adapt)
        if nomap_mode not in ("line", "hold"):
            raise ValueError(f"nomap_mode {nomap_mode!r}: line | hold")
        self.nomap_mode = nomap_mode
        # сдвиг «GNSS окна − карта» в якоре: сохранять ли его в выходе, если
        # медиана NavSatFix.status точек окна ≤ keep_offset_max_status.
        # −1 (по умолчанию) - никогда: на train (68 прогонов) сдвиг не даёт
        # выигрыша (всегда 3,687 м, только без RTK 3,672, никогда 3,666),
        # у RTK - вреден (1,841 против 1,823). 1 - только без RTK (статус
        # 0/1), 2 - всегда (выбор по holdout, от него отказались). По
        # горизонтали это только поперечная часть (курсор притягивается к оси
        # пути поперёк, не дальше snap_r), по высоте - не больше OFFSET_Z_MAX
        self.keep_offset_xy = bool(keep_offset_xy)
        self.keep_offset_z = bool(keep_offset_z)
        self.keep_offset_max_status = int(keep_offset_max_status)
        self.offset = (0.0, 0.0, 0.0)
        self.window_status = None       # медиана статуса точек окна
        # Только для MGRS с переносом по квадратам (mgrs_grid ""): ближе
        # mgrs_guard_m к краю 100-км квадрата положение не публикуется. По
        # умолчанию выключено (0): судья считает от квадрата 37UCB непрерывно
        # (карта организаторов pathgraph), и положение публикуется на каждом шаге.
        # С фиксированным квадратом (mgrs_grid "37UCB") не действует вовсе.
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
        self.scale_log = []             # (путь, сдвиг, L, mult) - для оценки
        # коррекция по GNSS после окна (см. docstring класса)
        self.gnss_correction = bool(gnss_correction)
        self.gnss_sigma = {2: float(gnss_sigma_rtk_m), 1: float(gnss_sigma_sbas_m),
                           0: float(gnss_sigma_fix_m)}
        self.gnss_gate = float(gnss_gate)
        self.gnss_jump_m = float(gnss_jump_m)
        self.gnss_confirm_n = max(1, int(gnss_confirm_n))
        self.gnss_min_interval_s = float(gnss_min_interval_s)
        self.gnss_max_skew_s = float(gnss_max_skew_s)
        self.gnss_prior_rel = float(gnss_prior_rel)
        self.gnss_scale_adapt = bool(gnss_scale_adapt)
        self.gnss_stop_skip_m = float(gnss_stop_skip_m)
        self.gnss_persist_s = float(gnss_persist_s)
        self._cq = []                   # точки после окна: ждут шага сетки
        self._hist = deque(maxlen=int(self.CORR_HIST_S / 0.01) + 2)   # (t, путь ядра)
        self._pend = []                 # эпохи вне ворот: ждут подтверждения
        self._pend_jump = False         # несогласие началось скачком GNSS
        self._ok = None                 # (метка, ν) последней эпохи, согласной с оценкой
        self._m_stat = None             # (метка, статус) последней точки master после окна
        self._seg = None                # отрезок масштаба: [путь от якоря, Σ поправок]
        self.n_corr = 0                 # принятых поправок
        self.corr_var = None            # σ² вдоль пути после последней поправки
        self.corr_s = None              # путь ядра в момент последней поправки
        self.corr_ds = None             # путь от якоря в момент последней поправки
        self.corr_stamp = -math.inf     # метка GNSS последней поправки
        self.corr_log = []              # (метка, ν вдоль, поправка, σ точки, вид) - для оценки
        self.n_corr_big = 0             # из них подтверждённых больших
        self.n_corr_reloc = 0           # перестановок курсора в точку GNSS
        self.n_corr_gated = 0           # эпох вне ворот (ждали подтверждения)
        self.n_corr_skew = 0            # точек с меткой не по часам входов
        self.n_corr_geom = 0            # эпох с негодной геометрией (база, курс)
        self.n_corr_skip = 0            # годных эпох между поправками (min_interval)
        self.n_corr_nonrtk = 0          # эпох без RTK с большой невязкой (не берутся)
        self.n_realign = 0              # окон выставки, открытых заново после окна

    # ---------- GNSS ----------

    def window_open(self, stamp):
        """Окно выставки: ±init_window от первой годной точки (раньше неё -
        хвост буфера записи в начале bag)."""
        t0 = self._t0
        return t0 is None or (t0 - self.init_window <= stamp <= t0 + self.init_window)

    @staticmethod
    def valid(stamp, lat, lon, status=0):
        """Точка GNSS годна вообще: конечные метка и координаты, не (0, 0),
        статус NavSatFix ≥ 0."""
        return bool(math.isfinite(stamp) and math.isfinite(lat) and math.isfinite(lon)
                    and abs(lat) <= 90.0 and abs(lon) <= 180.0
                    and not (abs(lat) < 1e-9 and abs(lon) < 1e-9)
                    and (status is None or status >= 0))

    def accepts(self, stamp, lat, lon, status=0):
        """Годна ли точка GNSS для выставки (проверяется до шага сетки):
        конечные метка и координаты, не (0, 0), статус NavSatFix ≥ 0, окно."""
        if not self.valid(stamp, lat, lon, status):
            self.n_rejected += 1
            return False
        return self.window_open(stamp)

    def on_fix(self, stamp, antenna, lat, lon, alt, s=None, v=0.0, t=None, status=0):
        """Точка GNSS. Негодная или вне окна - False. Годная ставится в
        очередь (s is None) и применяется на шаге сетки, который дойдёт до её
        метки (step); с s, v, t применяется сразу: путь на метку точки -
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

    def on_late_fix(self, stamp, antenna, lat, lon, alt, status=0, var=None, ref=None):
        """Годная точка GNSS вне окна выставки (только gnss_correction).

        Выставки нет или она без курса (старт с середины, в начале GNSS не
        было или была одна антенна на стоянке вне карты) - окно выставки
        открывается заново от этой точки. Иначе точка - кандидат коррекции:
        метка сверяется с меткой последнего входа ref на момент прихода
        (|Δ| ≤ gnss_max_skew_s, иначе метка не по часам тележек), точка
        ставится в очередь _cq и применяется на шаге сетки (_correct). var -
        заявленная дисперсия положения по горизонтали, м² (None - нет).
        Возвращает, принята ли точка."""
        if not self.gnss_correction:
            return False
        if not (self.fixed and self.ready):
            if self._q:
                # точки окна ещё ждут сетку (стартовый всплеск bag: метки
                # вразнобой) - выставка не закончена, окно не перезапускаем
                return False
            self._restart_window(stamp)
            return self.on_fix(stamp, antenna, lat, lon, alt, status=status)
        if ref is not None and not abs(stamp - ref) <= self.gnss_max_skew_s:
            self.n_corr_skew += 1
            return False
        if len(self._cq) >= self.CORR_MAX_Q:
            return False
        alt = alt if alt is not None and math.isfinite(alt) else None
        var = float(var) if var is not None and math.isfinite(var) and var > 0 else None
        self._cq.append((float(stamp), antenna, float(lat), float(lon), alt,
                         0 if status is None else int(status), var))
        return True

    def _restart_window(self, stamp):
        """Окно выставки заново от метки stamp (выставка по GNSS в середине
        прогона): точки прежнего окна забываются, начало системы остаётся."""
        self._t0 = stamp
        self._m, self._r = [], []
        self._q = [q for q in self._q if self.window_open(q[0])]
        self._cq, self._pend, self._ok = [], [], None
        self.n_realign += 1

    def resync(self, t):
        """Сетка ядра на t, а в выставку ещё ничего не вошло: точки с меткой
        дальше MAX_SKEW от t - сбой метки (например, 0 или 1e10 у первой
        точки, пришедшей до тележек). Они не должны открывать окно выставки:
        окно - заново от первой разумной точки."""
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
        """Высота антенны вместо NaN: карта у точки (плюс высота антенны над
        точкой карты: у карты base_link z - уровень рельса), иначе последняя
        годная, иначе 0."""
        if self.map is not None:
            h = self.map.altitude_at(lat, lon)
            if h is not None:
                return h + self.body.antenna_z - self.body.z(self.track_point)
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
        pi = pj = np.zeros(0, int)
        if not from_rover and self._r:
            R = self._project(self._r)
            i, j = np.nonzero(np.abs(M[:, 0][:, None] - R[:, 0][None, :]) <= self.PAIR_TOL)
            d = R[j, 1:3] - M[i, 1:3]
            b = np.hypot(d[:, 0], d[:, 1])
            ok = (b >= self.BASE_MIN) & (b <= self.BASE_MAX)
            pairs, pi, pj = d[ok], i[ok], j[ok]
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
            # курс по карте у антенны; карта base_link лежит дальше от антенны
            # на стоянке у конечной (карта - из точек на ходу, а антенна master
            # на 9,87 м позади base_link): радиус поиска шире на это плечо
            arm = abs(self.body.x("rover" if from_rover else "master")
                      - self.body.x(self.track_point))
            az = self.map.heading_at(anchor[:2], r_wide=30.0 + arm)
        # якорь антенны -> ведомая точка вагона (base_link у карт пакета):
        # на ходу - по паре master+rover у последней точки окна (кузов жёсткий,
        # base_link на отрезке антенн и на кривой), иначе перенос вдоль курса;
        # по высоте - минус antenna_z (base_link - уровень рельса)
        sel = np.flatnonzero(pi == k_last) if moving else np.zeros(0, int)
        if len(sel):
            q = sel[int(np.argmin(np.abs(R[pj[sel], 0] - M[k_last, 0])))]
            anchor = self.body.from_pair(M[k_last, 1:4], R[pj[q], 1:4], self.track_point)
        else:
            anchor = self.body.shift(anchor, az, "rover" if from_rover else "master",
                                     self.track_point)
        anchor = tuple(float(a) for a in anchor)
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

    # ---------- коррекция по GNSS после окна ----------

    def _path_at(self, stamp):
        """Путь ядра на метку stamp по истории сетки (линейно между узлами);
        None - метка старше истории (или истории нет)."""
        h = self._hist
        if not h or stamp < h[0][0] - 1e-9:
            return None
        if stamp >= h[-1][0]:
            return h[-1][1]
        for k in range(len(h) - 1, 0, -1):
            t0, s0 = h[k - 1]
            if t0 <= stamp:
                t1, s1 = h[k]
                return s0 + (s1 - s0) * (stamp - t0) / (t1 - t0) if t1 > t0 else s1
        return h[0][1]

    def _correct(self, t, s, ds, sigma):
        """Точки из очереди коррекции с меткой ≤ t − CORR_DELAY - по эпохам
        (master + rover одной метки), по возрастанию меток. True - положение
        изменилось."""
        cq = sorted(self._cq, key=lambda q: q[0])
        used = [False] * len(cq)
        changed = False
        for i, q in enumerate(cq):
            if used[i]:
                continue
            if q[0] > t - self.CORR_DELAY + 1e-9:
                break
            used[i] = True
            k = next((j for j in range(i + 1, len(cq)) if not used[j]
                      and (cq[j][1] == "master") != (q[1] == "master")
                      and abs(cq[j][0] - q[0]) <= self.PAIR_TOL), None)
            other = None
            if k is not None:
                used[k] = True
                other = cq[k]
            m, r = (q, other) if q[1] == "master" else (other, q)
            changed |= self._epoch(t, s, ds, sigma, m, r)
        self._cq = [q for q, u in zip(cq, used) if not u]
        return changed

    def _sd_fix(self, rows, single, status):
        """σ точки GNSS, м: заявленная ковариация, если есть, иначе по
        статусу эпохи status (_status); одна антенна - плюс CORR_SINGLE_SD
        (курс карты × плечо до base_link)."""
        var = [q[6] for q in rows if q[6] is not None]
        if var:
            sd = math.sqrt(max(max(var), 0.01))
        else:
            sd = self.gnss_sigma[min(max(status, 0), 2)]
        return math.hypot(sd, self.CORR_SINGLE_SD) if single else sd

    def _status(self, m, r, ts):
        """Статус эпохи для веса и правил: худший из master и rover. Rover
        меряется от master (подвижная база: статус 2 у rover - это решённый
        вектор базы, а не точность самой точки; в данных 30618_defd0170 и
        30618_0686195f у master весь прогон статус 0, у rover - 2, а база
        пары ровно 12,42 м): точность эпохи только с rover - по статусу
        master, последнего не старше CORR_MSTAT_S; master не было - без RTK."""
        if m is not None:
            self._m_stat = (m[0], m[5])
            return m[5] if r is None else min(m[5], r[5])
        ms = self._m_stat
        if ms is None and self._m:              # последняя точка master окна выставки
            ms = (self._m[-1][0], self._m[-1][5])
        st_m = ms[1] if ms is not None and abs(ts - ms[0]) <= self.CORR_MSTAT_S else 0
        return min(r[5], st_m)

    def _epoch(self, t, s, ds, sigma, m, r):
        """Одна эпоха GNSS (m - master, r - rover, любая может быть None):
        base_link по GNSS, невязка вдоль и поперёк пути на метку эпохи,
        ворота, подтверждение, поправка. True - положение изменилось."""
        ts = m[0] if m is not None else r[0]
        status = self._status(m, r, ts)
        s_fix = self._path_at(ts)
        if s_fix is None:
            self.n_corr_skew += 1
            return False
        fr = self.frame
        c = self._cursor
        h_ref = c["h"] if c is not None else self.az

        def xyz(q):
            z = q[4] if q[4] is not None else self._alt_fallback(q[2], q[3])
            return np.array(fr.fwd(q[2], q[3], z))

        # base_link по GNSS: пара одной эпохи с базой CORR_PAIR_MIN…12,44 +
        # CORR_BASE_TOL - кузов сочленённый: на тесных кривых база короче (до
        # ~10 м на петлях), а длиннее базы tf быть не может; курс пары h_g -
        # только у «чистой» пары (база 12,44 ± CORR_BASE_TOL)
        gp, h_g = None, None
        if m is not None and r is not None:
            M, R = xyz(m), xyz(r)
            d = R - M
            b = math.hypot(d[0], d[1])
            if not self.CORR_PAIR_MIN <= b <= self.body.baseline + self.CORR_BASE_TOL:
                # база не та: одна из антенн сбита, а какая - не знаем
                # (30618_616ec56b, 570 с: база 41 м; 30618_28538acf, 275 с:
                # 17 м) - эпоха не берётся
                self.n_corr_geom += 1
                return False
            gp = np.array(self.body.from_pair(M, R, self.track_point))
            if abs(b - self.body.baseline) <= self.CORR_BASE_TOL:
                h_g = math.atan2(d[0], d[1])
        if gp is None:
            q = m if m is not None else r
            gp = np.array(self.body.shift(xyz(q), h_ref, q[1] if q[1] == "master" else "rover",
                                          self.track_point))
        rows = [q for q in (m, r) if q is not None]
        sd = self._sd_fix(rows, len(rows) < 2, status)
        # пара смотрит не вдоль пути курсора (разворот на петле, курсор на
        # встречном пути или не на той ветке): поправка вдоль пути не имеет
        # смысла - только перестановка курсора по подтверждённой паре RTK
        flip = (h_g is not None
                and abs(math.remainder(h_g - h_ref, 2 * math.pi)) > self.CORR_HEAD_TOL)
        if flip:
            self.n_corr_geom += 1
        # невязка на метку эпохи: курсор сейчас на пути s, на метку - на s_fix
        # (без карты в режиме hold выход от пути не зависит: сдвига нет)
        hold = c is None and self.nomap_mode == "hold"
        if c is not None:
            k = self.map.scale * self.mult * c.get("k", 1.0)
            h = c["h"]
            px, py = c["x"], c["y"]
        else:
            k = 0.0 if hold else self.frame.k0
            h = self.az
            px, py, _ = self._internal(ds)
        lag = (s - s_fix) * k                   # м по карте от метки эпохи до шага
        tx, ty = math.sin(h), math.cos(h)
        dx, dy = gp[0] - px, gp[1] - py
        nu_a = dx * tx + dy * ty + lag
        nu_c = dx * ty - dy * tx
        P = (sigma if sigma is not None and math.isfinite(sigma) else self.CORR_PRIOR_SD) ** 2
        Rv = sd * sd
        K = P / (P + Rv)
        # без карты поперёк держать нечему: невязка - по модулю на плоскости
        nu = nu_a if c is not None else math.hypot(nu_a, nu_c)
        cross_lim = max(self.CORR_CROSS_MIN, 3.0 * sd) if c is not None else math.inf
        in_gate = abs(nu) <= self.gnss_gate * math.sqrt(P + Rv) and not flip
        cross_out = abs(nu_c) > cross_lim or flip
        if in_gate and abs(K * nu) <= self.gnss_jump_m and not cross_out:
            self._pend = []                 # эпоха согласна с оценкой: прежние - выбросы
            self._ok = (ts, nu_a)
            gap = self.gnss_min_interval_s if status >= 2 else self.CORR_NONRTK_INTERVAL_S
            if ts - self.corr_stamp < gap:
                self.n_corr_skip += 1
                return False
            return self._shift(ts, s, ds, K, nu_a, nu_c, (1.0 - K) * P, sd, status, "small")
        # Большая поправка - только после подтверждения:
        #   * правдоподобная (в воротах: оценка и так неуверенна - долгий путь
        #     без GNSS, срыв колёс) - gnss_confirm_n эпох подряд с согласной
        #     невязкой, затем обычное обновление K·ν;
        #   * неправдоподобная (вне ворот, поперёк пути, пара против курса
        #     пути) - только RTK и только если несогласие держится
        #     gnss_persist_s с одной и той же невязкой (у пары против курса -
        #     весь срок против курса): тогда оценка считается сбившейся и
        #     ставится по GNSS (априори ≥ ν²), а другая ветка, встречный путь
        #     или место вне карты - перестановкой курсора в точку GNSS. Без
        #     RTK бывают скачки на 20–50 м по нескольку секунд (30639) - по ним
        #     не переставляем никогда; у RTK - скачки на 10–30 м от секунд до
        #     минут (30618_49fe4c54, 30618_28538acf), поэтому срок не короткий;
        #   * скачок: несогласие появилось сразу (≤ CORR_JUMP_DT) после эпохи,
        #     согласной с оценкой, и невязка изменилась больше чем на
        #     CORR_JUMP_M + 3σ. Оценка по колёсам непрерывна - телепортом
        #     прыгнул GNSS (так бывает и при статусе 2: 30618_b95ca60a).
        #     Тогда держаться должно не меньше CORR_PERSIST_JUMP_S.
        self.n_corr_gated += 1
        if not self._pend:
            ok = self._ok
            self._pend_jump = (ok is not None and ts - ok[0] <= self.CORR_JUMP_DT
                               and abs(nu_a - ok[1]) > self.CORR_JUMP_M + 3.0 * sd)
        persist = (max(self.gnss_persist_s, self.CORR_PERSIST_JUMP_S) if self._pend_jump
                   else self.gnss_persist_s)
        keep_s = max(self.CORR_CONFIRM_S, persist)
        self._pend = [q for q in self._pend if ts - q[0] <= keep_s]
        self._pend.append((ts, nu_a, nu_c, sd, in_gate, cross_out, status, flip, s_fix,
                           float(gp[0]), float(gp[1])))
        self._pend = self._pend[-self.CORR_PEND_MAX:]
        if status < 2:
            # без RTK - только малые поправки (выше: |K·ν| ≤ gnss_jump_m, не
            # чаще CORR_NONRTK_INTERVAL_S); большой - никогда: смещение точек
            # без поправок бывает 10–16 м и держится минутами (30618_b8044aa0
            # glitchy: пачка статуса 0 со сдвигом 11 м давала поправки 4,7 +
            # 6,4 м), подтверждение несколькими эпохами его не отличает
            self.n_corr_nonrtk += 1
            return False
        last = self._pend[-self.gnss_confirm_n:]
        if len(last) < self.gnss_confirm_n or ts - last[0][0] > self.CORR_CONFIRM_S:
            return False
        tol = 1.0 + 2.0 * max(q[3] for q in last)
        if all(q[4] and q[6] >= 2 for q in last) and not self._pend_jump:
            use, inflate = last, False      # правдоподобная: короткое подтверждение (RTK)
        else:
            span = [q for q in self._pend if ts - q[0] <= persist + 1e-9]
            if not all(q[6] >= 2 for q in span):
                return False                # без RTK вне ворот - не верим
            if ts - span[0][0] < persist - self.CORR_DELAY:
                return False                # ждём, держится ли
            if any(q[7] for q in span) and not all(q[7] for q in span):
                return False                # то по курсу, то против - не ясно
            use, inflate = span, True
        if not all(q[7] for q in use):
            # согласие между эпохами: разброс невязки не больше 1 м + 2σ и
            # роста от масштаба колёс на пройденном за эти эпохи пути
            A = [q[1] for q in use]
            C = [q[2] for q in use]
            L = max(q[8] for q in use) - min(q[8] for q in use)
            tol_a = tol + self.CORR_DRIFT_TOL * L
            if (max(A) - min(A) > tol_a or max(C) - min(C) > tol) and not (
                    inflate and self._track_agrees(use, tol_a)):
                return False
        self._pend = []
        Pb = max(P, nu_a * nu_a + nu_c * nu_c) if inflate else P   # сбилась: априори ≥ ν²
        Kb = Pb / (Pb + Rv)
        var = (1.0 - Kb) * Pb
        if c is None:
            if math.hypot(nu_a, nu_c) > self.CORR_RELOC_MAX:
                return False
            return self._shift(ts, s, ds, Kb, nu_a, nu_c, var, sd, status, "big")
        if cross_out or not c.get("on_map", False) or abs(nu_a) > self.CORR_ALONG_MAX:
            # другая ветка, встречный путь, курсор вне карты или далеко:
            # курсор - в точку GNSS (только RTK, подтверждено выше) на путь
            # карты с курсом пары; нет там карты - в саму точку с курсом
            # чистой пары (тупик вне карты), дальше курсор ищет карту сам
            if status >= 2 and math.hypot(nu_a, nu_c) <= self.CORR_RELOC_MAX:
                hh = h_g if h_g is not None else h
                gx, gy = gp[0] + lag * math.sin(hh), gp[1] + lag * math.cos(hh)
                cn = self.map.locate((gx, gy, c["z"]), hh)
                if cn.get("on_map") or h_g is not None:
                    self._cursor = cn
                    self.n_corr_reloc += 1
                    return self._shift(ts, s, ds, 0.0, nu_a, nu_c, var, sd, status, "reloc")
            if cross_out or abs(nu_a) > 10.0 * self.CORR_ALONG_MAX:
                return False                # переставить нельзя, а вдоль - не то
        return self._shift(ts, s, ds, Kb, nu_a, nu_c, var, sd, status, "big")

    def _track_agrees(self, use, tol):
        """Путь точек GNSS эпох подтверждения (ломаная по base_link) равен пути
        колёс за то же время (с допуском tol): GNSS движется, как говорят
        колёса, - значит, сбилась не GNSS, а оценка (курсор стоит в тупике
        карты, ушёл на другую ветку), хотя невязка и растёт."""
        q_ = sorted(use, key=lambda q: q[0])
        g = [q_[0]]
        for q in q_[1:]:                    # точки не чаще раза в секунду: шум RTK
            if q[0] - g[-1][0] >= 1.0 or q is q_[-1]:   # не удлиняет ломаную
                g.append(q)
        Lg = sum(math.hypot(b[9] - a[9], b[10] - a[10]) for a, b in zip(g, g[1:]))
        k = self.map.scale * self.mult if self.map is not None else self.frame.k0
        Lw = (g[-1][8] - g[0][8]) * k
        return Lw > 5.0 and abs(Lg - Lw) <= tol

    def _shift(self, ts, s, ds, K, nu, nu_c, var, sd, status, kind):
        """Поправка K·ν: с картой - курсор по карте вдоль пути на K·ν (поперёк
        держит карта); без карты - якорь на K·(ν вдоль, ν поперёк). Учёт: σ²
        после, отрезок масштаба, отсчёт привязки к остановкам."""
        c = self._cursor
        delta = K * nu
        if K != 0.0:
            if c is not None:
                self.map.advance(c, delta / (self.map.scale * self.mult * c.get("k", 1.0)),
                                 self.mult)
            else:
                x0, y0, z0 = self.xyz0
                sa, ca = math.sin(self.az), math.cos(self.az)
                self.xyz0 = (x0 + K * (nu * sa + nu_c * ca), y0 + K * (nu * ca - nu_c * sa), z0)
        if kind == "big":
            self.n_corr_big += 1
        # масштаб пути по отрезкам между RTK-поправками: ошибка колёс на
        # отрезке = невязка в его конце + поправки внутри
        if self.gnss_scale_adapt and self.scale_adapt and c is not None:
            if kind == "reloc":
                self._seg = None
            elif self._seg is None:
                if status >= 2:
                    self._seg = [ds, 0.0]
            elif status >= 2 and ds - self._seg[0] >= self.CORR_SCALE_L:
                L, d = ds - self._seg[0], nu + self._seg[1]
                self._adapt_scale(L, d)
                self.scale_log.append((ds, d, L, self.mult))
                self._seg = [ds, 0.0]
            else:
                self._seg[1] += delta
        self.n_corr += 1
        # σ² после: (1 − K)·P, но не меньше доли σ² точки (и не больше P)
        floor = max(self.CORR_VAR_FLOOR, self.CORR_VAR_FRAC * sd * sd)
        self.corr_var = max(var, min(floor, var / max(1.0 - K, 1e-9)))
        # Окно привязки к остановке (TrackMap.anchor, σ = SD0 + REL·путь с
        # прошлой привязки) - главное средство против дрейфа без GNSS, и поправка
        # его сужает только по RTK и только до σ после поправки: отсчёт пути
        # привязки сдвигается на ds − L, где L - путь, за который окно
        # дорастает до этой σ (большее из σ поправки и σ самого окна после
        # того же обновления Калмана). Поправка без RTK окно не трогает:
        # смещение таких точек бывает 10–16 м, а привязка при ошибке колёс
        # ~25 м бывает на краю окна (30618_defd0170 sparse: привязок 11 → 8,
        # конец 3,3 → 15,6 м, когда окно сужала каждая поправка)
        if status >= 2:
            w2 = (self.ANCHOR_SD0 + self.ANCHOR_REL * max(ds - self._s_anchor, 0.0)) ** 2
            w2 = w2 * sd * sd / (w2 + sd * sd) if kind != "reloc" else 0.0
            sig = math.sqrt(max(self.corr_var, w2))
            L_eq = max(0.0, (sig - self.ANCHOR_SD0) / self.ANCHOR_REL)
            self._s_anchor = max(self._s_anchor, ds - L_eq)
        self.corr_s, self.corr_ds, self.corr_stamp = s, ds, ts
        if len(self.corr_log) < 20000:
            self.corr_log.append((ts, nu, delta, sd, kind))
        return True

    # ---------- путь -> положение ----------

    def on_stop(self, ds):
        """Вагон стоит дольше порога: привязка к точке остановки на карте и
        онлайн-подстройка масштаба пути по сдвигу привязки."""
        c = self._cursor
        if c is None:
            return
        if (self.gnss_stop_skip_m > 0.0 and self.corr_ds is not None
                and ds - self.corr_ds < self.gnss_stop_skip_m):
            return                      # недавняя поправка GNSS точнее точки остановки
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
        """Масштаб = (s0·L_prior + Σ(L_i·s0·m + δ_i)) / (L_prior + ΣL_i): δ_i -
        сдвиг вдоль пути при привязке, L_i - путь колёс с прошлой привязки.
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

    def step(self, s, standing, dt, t=None, v=0.0, sigma=None):
        """Шаг сетки t: путь ядра s, скорость v -> (x, y, z, yaw) в выходной
        системе; None, пока нет якоря (ещё не было ни одной годной точки
        master) или точка у края 100-км квадрата MGRS (mgrs_guard_m). Сначала
        применяются точки GNSS из очереди с меткой ≤ t (выставка), затем -
        поправки GNSS после окна с меткой ≤ t − CORR_DELAY; sigma -
        априорная σ положения вдоль пути на этом шаге (Runner)."""
        if t is not None:
            self.resync(t)
        if self._q and t is not None:
            self._flush(t, s, v)
        if not self.fixed:
            return None
        ds = s - self.s_ref
        x, y, z = self._internal(ds)
        if self.gnss_correction and t is not None:
            self._hist.append((t, s))
            if self._cq and self._correct(t, s, ds, sigma):
                x, y, z = self._internal(ds)
        self._dwell = self._dwell + dt if standing else 0.0
        if self._dwell >= self.stop_dwell > self._dwell - dt:
            self.on_stop(ds)
            x, y, z = self._internal(ds)
        c = self._cursor
        az = c["h"] if c is not None else (self.az if self.ready else None)
        x, y, z = self._out_point((x, y, z), az)
        self.xyz_internal = (x, y, z)
        if self.near_square_edge(x, y):
            return None
        X, Y, Z = self.frame.out(x, y, z)
        yaw = self.frame.out_yaw(x, y, az) if az is not None else None
        return X, Y, Z, yaw

    def _out_point(self, xyz, az):
        """Ведомая точка (по ней собрана карта) -> точка выхода вдоль курса
        пути az (карта base_link, выход master и наоборот)."""
        if self.track_point == self.output_point:
            return xyz
        return self.body.shift(xyz, az, self.track_point, self.output_point)

    def xyz(self, ds):
        """Положение после пути ds от якоря в выходной системе (без привязок)."""
        c = self._cursor
        xyz = self._internal(ds)
        az = c["h"] if c is not None else (self.az if self.ready else None)
        return self.frame.out(*self._out_point(xyz, az))


def _num(x):
    """Число из поля сообщения; всё, что числом не является, - NaN."""
    try:
        return float(x)
    except (TypeError, ValueError, OverflowError):
        return math.nan


class Runner:
    """Ядро + выставка + карта на входных сообщениях (нода и офлайн-оценка).

    Защита входов и времени (устойчивость):
      * метка не число или <= 0 - сообщение отбрасывается целиком;
      * показание тележки не число или |z| > V_LIMIT·v_max_line (в единицах
        датчика) - значение отбрасывается, метка сетку двигает;
      * ручка не число - отбрасывается; вне ±HANDLE_LIMIT - ограничивается;
      * «своя» база времени: метка не раньше t − MAX_JUMP_S и не позже
        stamp_max + FWD_JUMP_S (первые SETTLE_S прогона - + MAX_JUMP_S:
        стартовый хвост буфера bag, метки вразнобой на 2–6 с). Сообщение вне
        базы - кандидат новой базы: откладывается, в t_wheel / t_notch не
        пишется. Кандидаты подтверждаются CONFIRM_N сообщениями подряд от
        CONFIRM_SRC разных входов (обе тележки - один вход, у них одни часы)
        или CONFIRM_SOLO сообщениями одного входа; сообщение своей базы
        кандидатов отбрасывает (одиночный выброс, сбой часов пары тележек).
        Подтверждённый скачок вперёд до GAP_MAX_S - провал входов в том же
        прогоне: сетка догоняет, состояние сохраняется. Иначе (назад, --loop,
        второй bag, далеко вперёд) - полный сброс reset(): ядро, выставка,
        s0, стоянка, сетка; прежняя выставка остаётся запасной, пока новый
        прогон не выставится по своему GNSS. Отложенные сообщения после
        подтверждения исполняются заново по порядку прихода (не теряются);
      * не больше MAX_STEPS шагов сетки за вызов: сообщения, до метки
        которых сетка за вызов не доходит, ждут в очереди _defer и
        применяются, когда дойдёт (порядок «узлы до метки, потом значение»
        тот же, что без очереди); остаток догоняется следующими вызовами;
      * неконечное состояние ядра - сброс ядра с сохранением пути.
    Сетка: узлы кратны p.dt, t = (k0 + n)·dt по целому счётчику n -
    совпадают с метками GNSS (кратны 0,1 с) и не дрейфуют.
    Возраст показаний: свежее показание тележки приводится к моменту
    шага по ускорению модели (age_comp).
    """

    MAX_JUMP_S = 10.0       # с: скачок назад без подтверждения (хвост буфера <= 6,2 с)
    FWD_JUMP_S = 1.5        # с: скачок вперёд без подтверждения (в данных <= 1,06 с)
    SETTLE_S = 10.0         # с меток от начала прогона: до них вперёд тоже MAX_JUMP_S
    GAP_MAX_S = 60.0        # с: подтверждённый скачок вперёд до - провал в том же прогоне
    CONFIRM_N = 3           # сообщений подряд в новой базе времени...
    CONFIRM_SRC = 2         # ...от стольких разных входов
    CONFIRM_SOLO = 10       # или подряд от одного входа (другие молчат)
    MAX_STEPS = 200         # шагов за вызов: 10 с при dt = 50 мс
    V_LIMIT = 1.5           # доля v_max_line: выше - показание невозможно
    HANDLE_LIMIT = 15.0     # позиций ручки по ТЗ в каждую сторону
    AGE_MAX_S = 0.3         # с: предел приведения показания к шагу
    age_comp = True         # приводить показания к моменту шага
    grid_align = True       # узлы сетки кратны dt
    _POS_KEEP = ("init_window",)    # настройки выставки, заданные после __init__
    wheel_scale = None      # vehicle.OnlineWheelScale: поправка скорости (задаёт нода)

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
        """position_opts - параметры Position: init_window, projection,
        mgrs_grid, utm_zone, scale_adapt, nomap_mode, keep_offset_xy,
        keep_offset_z, keep_offset_max_status, mgrs_guard_m, terminal_hold,
        output_point, antenna_master_x, antenna_rover_x, antenna_z,
        gnss_correction и настройки коррекции gnss_* (см. Position)."""
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
        self._ncorr = 0                 # поправок GNSS учтено в σ
        self._g = None                  # (путь ядра, σ² после, σ²ядра) последней поправки
        # положение без поправок GNSS - только для онлайн-масштаба колёс
        # (wheel_scale): копия self.pos, снятая перед первой поправкой
        self._ws_pos = None

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

    def on_fix(self, stamp, antenna, lat, lon, alt, status=0, cov=None):
        """GNSS: начальная выставка в окне и (gnss_correction) коррекция
        положения после окна. Сетку ядра GNSS не двигает: негодная точка
        отбрасывается, годная ставится в очередь Position и применяется на
        шаге сетки, дошедшем до её метки. Поэтому GNSS не влияет на скорость.
        Якорь и путь выставки пересчитываются только при точке, вошедшей в
        выставку. Точка после окна - кандидат коррекции (Position.on_late_fix:
        сверка метки с меткой последнего входа, ворота, подтверждение); при
        gnss_correction false она отбрасывается (GNSS только для выставки). status -
        NavSatFix.status.status (< 0: нет решения); cov - заявленная
        дисперсия по горизонтали, м² (None или ≤ 0 - не заявлена). Точка с
        меткой дальше Position.MAX_SKEW от сетки - сбой метки, отбрасывается.
        Выходов не порождает: возвращает []. Поля, не являющиеся числами,
        считаются NaN (точка негодна)."""
        stamp, lat, lon = _num(stamp), _num(lat), _num(lon)
        alt = None if alt is None else _num(alt)
        if self.t is not None:
            if not abs(stamp - self.t) <= self.pos.MAX_SKEW:
                self.pos.n_rejected += 1
                return []
            self.pos.resync(self.t)
        if self.pos.accepts(stamp, lat, lon, status):
            self.pos.on_fix(stamp, antenna, lat, lon, alt, status=status)
        elif self.pos.gnss_correction and self.pos.valid(stamp, lat, lon, status):
            took = self.pos.on_late_fix(stamp, antenna, lat, lon, alt, status=status,
                                        var=None if cov is None else _num(cov),
                                        ref=self.stamp_max if self.t is not None else None)
            if (took and self.pos._cq and self._ws_pos is None
                    and self.wheel_scale is not None):
                self._ws_pos = self._shadow_position()
        return []

    def _shadow_position(self):
        """Копия положения без поправок GNSS - для онлайн-масштаба колёс.

        Онлайн-масштаб колёс (vehicle.OnlineWheelScale) учится по привязкам к
        остановкам: путь по карте против пути колёс. Поправка GNSS двигает
        курсор, и сдвиг привязки после неё уже не равен ошибке колёс; к тому
        же через этот масштаб GNSS попал бы в скорость. Поэтому до первой
        поправки масштаб учится по самому положению (поправок ещё не было),
        а с первой точки-кандидата коррекции - по копии положения, которая
        поправок не получает (gnss_correction false): её путь и привязки те
        же, что при GNSS только в окне выставки. Так GNSS после окна на
        скорость не влияет и с онлайн-масштабом колёс. Карта общая (курсор
        свой у каждой копии). Копия снимается один раз за прогон и только
        при GNSS после окна - у жюри его почти нет."""
        m = self.pos.map
        sh = copy.deepcopy(self.pos, {id(m): m} if m is not None else None)
        sh.gnss_correction = False
        sh._cq = []
        return sh

    # ---------- шаги ----------

    def _advance(self, stamp, src="fix", replay=None):
        """Шаги сетки до метки сообщения; возвращает их выходы.

        src - вход (часы): "wheel" (обе тележки), "handle", "fix". replay -
        (метод, аргументы) для повторного исполнения сообщения, если оно
        окажется первым в подтверждённой новой базе времени; без него такое
        сообщение теряется. self.stamp_ok - применять ли значение сообщения:
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
            self._drop_pending()        # база прежняя: кандидаты - выбросы
        if self._defer or self._due(stamp) > self._budget:
            # сетка не дойдёт до метки за этот вызов: значение применится,
            # когда дойдёт (как без очереди: сначала все узлы до метки, потом
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
                    break               # бюджет вызова исчерпан: дальше - потом
            self._defer.pop(0)
            self._replaying = True
            try:
                outs += getattr(self, rp[0])(*rp[1])
            finally:
                self._replaying = False
        self.stamp_ok = False           # значение этого сообщения - через очередь
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
        """Сообщение вне базы: откладывается; подтверждённая новая база -
        провал в прогоне (догоняем) или новый прогон (reset), затем отложенные
        сообщения исполняются по порядку прихода."""
        P = self._pend
        if P and max(abs(stamp - q[0]) for q in P) > self.MAX_JUMP_S:
            self._drop_pending()        # третья база: прежние кандидаты - выбросы
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
        """Первый узел сетки - кратный dt не позже первой метки;
        grid_align = False - сетка от самой первой метки."""
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
        пропускаются (по построению не бывает: дальше - новый прогон)."""
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
        ядро, выставка, s0, стоянка, сетка и отметки входов - заново. Карта,
        лист и настройки прежние; выставка возьмётся по GNSS нового прогона.

        Прежняя выставка остаётся запасной (_pos_prev, путь от неё _s_prev):
        пока новая не готова, положение идёт по ней - путь до сброса плюс
        путь после. Разрыв меток внутри прогона (сбой часов) тогда не теряет
        привязку: у жюри GNSS есть только в первые секунды."""
        keep = {k: getattr(self.pos, k) for k in self._POS_KEEP
                if hasattr(self.pos, k)}
        prev = self._fallback_now()
        # σ положения у запасной выставки продолжает расти от её
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
        """(выставка, путь ядра в её отсчёте) на этот момент - для запасной
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

    def _position(self, s, standing, t, v, sigma=None):
        """Положение на шаге t -> ((x, y, z, yaw) | None, запасная ли).

        Новая выставка шагает всегда (её очередь GNSS применяется на своих
        метках). Пока у неё нет якоря, а после сброса есть прежняя выставка -
        положение идёт по прежней: путь ядра до сброса плюс путь после (t=None:
        очередь GNSS прежнего прогона не трогается). Как только новая
        выставка получила якорь, запасная больше не нужна. sigma - априорная
        σ положения для поправок GNSS новой выставки."""
        pq = self.pos.step(s, standing, self.p.dt, t=t, v=v, sigma=sigma)
        if self._pos_prev is None:
            return pq, False
        if self.pos.fixed:
            # новая выставка: σ положения - от её якоря (отсчёт пути новый)
            self._pos_prev = None
            self._anchors, self._s_fix = 0, -np.inf
            return pq, False
        return self._pos_prev.step(self._s_prev + s, standing, self.p.dt), True

    def _sigma_pos(self, sf, s_act, pos, rel=None):
        """σ положения вдоль пути -> (σ, путь с последней привязки).
        Без поправок GNSS - position_sigma от выставки или последней привязки
        к остановке. Если последней была поправка GNSS: её σ²
        после обновления + прирост σ² пути ядра с тех пор + масштаб колёс на
        пройденном пути. rel - рост σ на метр пути вместо ss_rel листа
        (априори для поправок GNSS: типичный, а не с запасом на хвосты)."""
        rel = self.p.ss_rel if rel is None else rel
        g = self._g
        if g is not None and g[0] >= max(pos.s_ref, self._s_fix):
            ds = s_act - g[0]
            return (math.sqrt(g[1] + max(sf * sf - g[2], 0.0) + (rel * ds) ** 2), ds)
        ds = s_act - max(pos.s_ref, self._s_fix)
        if rel is self.p.ss_rel:
            return position_sigma(sf, ds, self.p), ds
        return math.sqrt(sf * sf + self.p.ss_map ** 2 + (rel * ds) ** 2), ds

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
        """Свежие показания тележек, приведённые к моменту шага t:
        z + a·(t − метка). a - ускорение модели на прошлом шаге. Показание,
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
        # априорная σ положения для поправок GNSS (та же формула, что у выхода)
        sig0 = (self._sigma_pos(o["sigma_s"], s, self.pos, self.pos.gnss_prior_rel)[0]
                if self.pos.gnss_correction and self.pos.fixed else None)
        # положение: Position (выставка, карта, привязки, выходная система);
        # после сброса, пока новый прогон не выставился, - запасная выставка
        standing = o["mode"] == STANDSTILL and o["valid"]
        pq, fallback = self._position(s, standing, t, float(c.x[IV]), sig0)
        if self._ws_pos is not None:
            # положение без поправок GNSS: тот же шаг, что у self.pos
            self._ws_pos.step(s, standing, self.p.dt, t=t, v=float(c.x[IV]))
        pos = self._pos_prev if fallback else self.pos
        if pos.fixed:
            # σ положения: путь после выставки, последней привязки
            # к остановке или поправки GNSS - в отсчёте пути той выставки, по
            # которой идёт положение (у запасной - путь ядра прежнего прогона
            # + новый)
            s_act = self._s_prev + s if fallback else s
            if pos.anchors != self._anchors:
                self._anchors, self._s_fix = pos.anchors, s_act
            if not fallback and pos.n_corr != self._ncorr:
                self._ncorr = pos.n_corr
                self._g = (s_act, pos.corr_var, o["sigma_s"] ** 2)
            o["sigma_s"], o["ds_fix"] = self._sigma_pos(o["sigma_s"], s_act, pos)
        if pq is None:
            # якоря ещё нет (нет GNSS) или точка у края квадрата MGRS: положения
            # в выходной системе нет, pos_valid = False - нода
            # /result/position не публикует
            x, y, z, yaw = s, 0.0, 0.0, None
        else:
            x, y, z, yaw = pq
        v = float(c.x[IV])
        a = (body_force(c.u_filt, v, c.x[3], c.x[4], c.mu, c.p)
             - resistance(v, c.p)) / c.p.M_nom + float(c.x[ID])
        if self.wheel_scale is not None:
            # онлайн-масштаб колёс по привязкам к остановкам (vehicle.py); с
            # GNSS после окна - по положению без поправок (_shadow_position)
            wp = self._ws_pos if self._ws_pos is not None else self.pos
            o["v"] = float(o["v"]) * self.wheel_scale.factor(
                wp.scale_log, getattr(wp.map, "scale", None))
        o.update(stamp=t, x=x, y=y, z=z, yaw=yaw,
                 a=float(a) if v > 0 or a > 0 else 0.0,
                 pos_ready=pos.ready, pos_valid=pq is not None,
                 pos_fallback=fallback,
                 handle_ok=handle_ok, wheels_stale=bool(wheels_stale))
        self.last = o
        return o


class StartSorter:
    """Первые window секунд по часам прихода сообщения копятся и
    отдаются по возрастанию метки, дальше - сквозной проход.

    Стартовый всплеск bag идёт в порядке записи, а метки в нём скачут назад
    на 1–3 с (хвост буфера записи, DATA п. 7): сетка, начатая с первого
    обработанного сообщения, теряет выходы до самой ранней метки, а окно
    выставки считается не от первой точки GNSS. Часы - любые монотонные:
    в ноде time.monotonic(), офлайн - время записи bag."""

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
        """Окно истекло - всё накопленное по порядку меток (один раз)."""
        if self.done or self._t0 is None or now - self._t0 < self.window:
            return []
        self.done = True
        out = [it for _, _, it in sorted(self._buf, key=lambda e: e[:2])]
        self._buf = []
        return out
