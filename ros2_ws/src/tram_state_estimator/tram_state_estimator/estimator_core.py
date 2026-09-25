"""Резервная оценка скорости и положения трамвая без GNSS.

Входы: положение ручки контроллера и частоты вращения колёс. Больше ничего.

Ядро не зависит от ROS. ВСЕ числовые константы собраны в `Params`; больше
нигде в модуле числовых констант нет (кроме 0, 1, 2 и тождеств). Из описания
`Params` генерируются params.yaml и таблица параметров в MODEL.md, поэтому лист
параметров существует в одном экземпляре. Подстановка данных ТЗ — это замена
значений в params.yaml.

Модель (подробно — MODEL.md):

    состояние   x = [s, v, d, k_t, k_b]
    вход        u — нормированный момент привода (из позиции ручки)
    измерение   z_a — приведённая скорость оси a

    ṡ = v
    v̇ = (F_кор(u, v; k, μ) − W(v)) / M + d
    ḋ, k̇ — медленный случайный дрейф, ТОЛЬКО там, где параметр наблюдаем
    F_кор = clip(F_привода, −μ·N_торм, +μ·N_привод)         предел сцепления
    z_a = v · (1 + c·F_a/N_a − c_0)                          крип по оси

Фильтр — сигма-точечный (UKF). Измерения обрабатываются по одному и проходят
проверку правдоподобия: невязка (по своей ковариации) и физический предел
ускорения колеса. Отвергнутое измерение состояние не меняет, а если отвергнуты
все — неопределённость по ускорению раздувается до физического предела, чтобы
блокировка фильтра заканчивалась за ограниченное время.
"""

from bisect import bisect_right
from collections import deque
from math import sqrt
from dataclasses import dataclass, field, fields, replace
import numpy as np

G = 9.81


# Единицы показаний датчиков. Это определения единиц, а не константы модели.
ANGULAR_UNITS = {"rad_s": 1.0, "rpm": 2.0 * np.pi / 60.0, "hz": 2.0 * np.pi}
LINEAR_UNITS = {"m_s": 1.0, "km_h": 1.0 / 3.6}
UNITS = set(ANGULAR_UNITS) | set(LINEAR_UNITS) | {"pulses_s"}


def _f(default, unit, src, doc, group):
    """Описание параметра: значение, единицы, источник, смысл, раздел листа."""
    return field(default=default,
                 metadata=dict(unit=unit, src=src, doc=doc, group=group))


@dataclass(frozen=True)
class Params:
    """Лист параметров. Значения по умолчанию — ЗАГЛУШКИ для имитатора.

    Источник (src): «паспорт» — из документации вагона, привода, датчика или
    линии; «измерение» — определяется опознаванием при вводе в эксплуатацию;
    «настройка» — параметр фильтра, не физическая величина.
    """

    # ---------------------------------------------------------- механика
    M_nom: float = _f(28000.0, "кг", "паспорт",
                      "номинальная масса вагона, средняя загрузка", "Механика")
    r_nom: float = _f(0.35, "м", "паспорт",
                      "радиус колеса при среднем износе", "Механика")
    n_axles: int = _f(4, "—", "паспорт", "число осей", "Механика")
    driven: tuple = _f((True, True, False, False), "—", "паспорт",
                       "моторные оси", "Механика")
    braked: tuple = _f((True, True, True, True), "—", "паспорт",
                       "оси с тормозом, участвующие в торможении", "Механика")
    v_max_line: float = _f(20.0, "м/с", "паспорт",
                           "максимальная скорость на линии", "Механика")

    # ------------------------------------------------------------ привод
    F_notch: float = _f(36000.0, "Н", "паспорт",
                        "тяга при полном отклонении ручки", "Привод")
    F_brake: float = _f(30000.0, "Н", "паспорт",
                        "тормозная сила при полном отклонении", "Привод")
    v_base: float = _f(8.0, "м/с", "паспорт",
                       "скорость начала ослабления поля", "Привод")
    v_ed_fade: float = _f(1.5, "м/с", "паспорт",
                          "скорость затухания электродинамического тормоза",
                          "Привод")
    brake_hold_frac: float = _f(1.0, "—", "паспорт",
                                "доля тормозной силы, которую механический "
                                "тормоз сохраняет ниже v_ed_fade: 1 — полное "
                                "замещение, 0 — тормоз гаснет вместе с ЭД",
                                "Привод")
    notch_tract: tuple = _f((0.25, 0.5, 0.75, 1.0), "—", "паспорт",
                            "доля полной тяги на позициях тяги 1…N; число "
                            "позиций = длина списка", "Привод")
    notch_brake: tuple = _f((0.25, 0.5, 0.75, 1.0), "—", "паспорт",
                            "доля полной тормозной силы на позициях торможения "
                            "1…N; число позиций = длина списка", "Привод")
    acc_u: tuple = _f((), "—", "измерение",
                      "табличный привод: сетка нормированной команды u (позиция "
                      "/ число позиций); пусто — параметрическая модель "
                      "F_notch/F_brake", "Привод")
    acc_v: tuple = _f((), "м/с", "измерение",
                      "табличный привод: сетка скорости", "Привод")
    acc_table: tuple = _f((), "м/с²", "измерение",
                          "табличный привод: ускорение на ровном пути A(u, v) "
                          "по строкам acc_u, len(acc_u)·len(acc_v) значений; "
                          "строка u = 0 — выбег, то есть сопротивление движению",
                          "Привод")
    tau_drive: float = _f(0.30, "с", "измерение",
                          "постоянная времени привода", "Привод")
    delay_drive: float = _f(0.10, "с", "измерение",
                            "чистое запаздывание привода", "Привод")
    T_dead: float = _f(0.05, "—", "настройка",
                       "мёртвая зона по нормированному моменту", "Привод")
    t_confirm: float = _f(0.15, "с", "настройка",
                          "время подтверждения режима движения", "Привод")

    # ------------------------------------------------- сопротивление W(v)
    res_A: float = _f(2400.0, "Н", "измерение",
                      "постоянная часть сопротивления", "Сопротивление")
    res_B: float = _f(55.0, "Н·с/м", "измерение",
                      "линейная часть сопротивления", "Сопротивление")
    res_C: float = _f(3.5, "Н·с²/м²", "измерение",
                      "квадратичная часть сопротивления", "Сопротивление")

    # -------------------------------------------- сцепление и проскальзывание
    mu_nominal: float = _f(0.25, "—", "паспорт",
                           "коэффициент сцепления сухого рельса",
                           "Сцепление")
    mu_min: float = _f(0.02, "—", "паспорт",
                       "худшее сцепление (листопадная плёнка, лёд)",
                       "Сцепление")
    tau_sat: float = _f(0.3, "с", "настройка",
                        "темп снижения сцепления при признаках срыва",
                        "Сцепление")
    tau_rec: float = _f(30.0, "с", "настройка",
                        "темп восстановления сцепления без признаков срыва",
                        "Сцепление")
    c_creep: float = _f(0.020, "—", "измерение",
                        "податливость по крипу: ξ = c · μ", "Сцепление")
    c_creep_drag: float = _f(0.002, "—", "измерение",
                             "крип ненагруженной оси от сопротивления вращению",
                             "Сцепление")
    sigma_creep: float = _f(0.004, "—", "настройка",
                            "неопределённость самого коэффициента крипа",
                            "Сцепление")

    # ---------------------------------------------------- пределы корпуса
    a_max_acc: float = _f(1.5, "м/с²", "паспорт",
                          "предельное ускорение корпуса", "Пределы корпуса")
    a_max_brake: float = _f(3.0, "м/с²", "паспорт",
                            "предельное замедление корпуса, с рельсовым тормозом",
                            "Пределы корпуса")
    theta_max: float = _f(0.06, "рад", "паспорт",
                          "максимальный уклон линии", "Пределы корпуса")
    d_extra: float = _f(0.30, "м/с²", "настройка",
                        "запас возмущения на отклонение массы и ветер",
                        "Пределы корпуса")
    a_free_decel: float = _f(0.75, "м/с²", "паспорт",
                             "предельное замедление БЕЗ команды тормоза: "
                             "уклон и сопротивление", "Пределы корпуса")
    lock_ratio: float = _f(0.7, "—", "настройка",
                           "показание оси ниже этой доли прогноза при "
                           "торможении = блокировка (плавное торможение, в том "
                           "числе рельсовым, отстаёт на проценты)",
                           "Пределы корпуса")
    a_slip_margin: float = _f(1.0, "м/с²", "настройка",
                              "запас над пределом ускорения до признания срыва",
                              "Пределы корпуса")

    # ------------------------------------------------------------ датчики
    meas_units: str = _f("rad_s", "—", "паспорт",
                         "единицы показаний: rad_s, rpm, hz (об/с), pulses_s "
                         "(импульсы/с, с ppr), m_s, km_h", "Датчики")
    ppr: int = _f(200, "имп/об", "паспорт",
                  "импульсов на оборот вала датчика (для pulses_s)", "Датчики")
    meas_scale: float = _f(1.0, "—", "измерение",
                           "поправочный множитель показаний (износ бандажей, "
                           "погрешность номинала); определяется по эталонной "
                           "скорости", "Датчики")
    sensor_ratio: float = _f(1.0, "—", "паспорт",
                             "передаточное число от колеса к валу датчика: 1 — "
                             "датчик на колесе; для m_s и km_h не применяется",
                             "Датчики")
    sensors_per_axle: int = _f(2, "—", "паспорт",
                               "датчиков на ось: 2 — оба борта, 1 — один",
                               "Датчики")
    sensor_axles: tuple = _f((True, True, True, True), "—", "паспорт",
                             "оси с датчиками (при датчике на валу двигателя — "
                             "только моторные)", "Датчики")
    curve_ratio_max: float = _f(0.038, "—", "паспорт",
                                "b/R_min: наибольшая относительная разница путей "
                                "бортов на кривой; учитывается при одном датчике "
                                "на ось", "Датчики")
    sigma_meas: float = _f(0.15, "м/с", "паспорт",
                           "шум измерения скорости оси, с учётом квантования "
                           "энкодера и окна измерения", "Датчики")
    stuck_n: int = _f(60, "отсчётов", "настройка",
                      "неизменных отсчётов = подозрение на залипание датчика",
                      "Датчики")
    stuck_dv: float = _f(0.5, "м/с", "настройка",
                         "залипание признаётся, только если остальные датчики "
                         "за это время изменились больше этого: на постоянной "
                         "скорости реальный энкодер может выдавать одинаковые "
                         "отсчёты", "Датчики")
    recover_tol: float = _f(0.5, "м/с", "настройка",
                            "исключённый датчик возвращается, если расходится "
                            "с остальными не больше recover_tol + recover_rel·v",
                            "Датчики")
    recover_rel: float = _f(0.1, "—", "настройка",
                            "относительная часть допуска возврата датчика",
                            "Датчики")
    t_recover: float = _f(2.0, "с", "настройка",
                          "сколько датчик должен согласоваться с остальными, "
                          "чтобы вернуться в работу", "Датчики")
    v_dead_ref: float = _f(2.0, "м/с", "настройка",
                           "скорость остальных, выше которой нулевой датчик "
                           "считается оборванным", "Датчики")
    axle_dot_alpha: float = _f(0.2, "—", "настройка",
                               "сглаживание производной скорости оси",
                               "Датчики")
    calib_gain: float = _f(0.01, "—", "настройка",
                           "темп калибровки масштаба осей на выбеге", "Датчики")
    calib_v_min: float = _f(2.0, "м/с", "настройка",
                            "минимальная скорость калибровки масштаба", "Датчики")
    scale_range: float = _f(0.10, "—", "паспорт",
                            "допустимый разброс масштаба оси (износ бандажа)",
                            "Датчики")

    # ------------------------------------------------------------- фильтр
    gate_nis: float = _f(9.0, "—", "настройка",
                         "порог нормированной невязки (9 = 3σ)", "Фильтр")
    sigma_rej_frac: float = _f(0.5, "—", "настройка",
                               "доля предела ускорения в σ возмущения при "
                               "отвержении всех измерений", "Фильтр")
    q_v: float = _f(0.05, "м/с²", "настройка",
                    "шум процесса по скорости", "Фильтр")
    q_d: float = _f(0.02, "м/с²/√с", "настройка",
                    "дрейф возмущения на выбеге", "Фильтр")
    q_k: float = _f(0.003, "1/√с", "настройка",
                    "дрейф масштаба привода под нагрузкой", "Фильтр")
    p0_s: float = _f(1.0, "м", "настройка", "начальная σ положения", "Фильтр")
    p0_v: float = _f(1.0, "м/с", "настройка", "начальная σ скорости", "Фильтр")
    p0_d: float = _f(0.30, "м/с²", "настройка",
                     "начальная σ возмущения", "Фильтр")
    p0_k: float = _f(0.10, "—", "настройка",
                     "начальная σ масштаба привода", "Фильтр")
    k_min: float = _f(0.4, "—", "настройка",
                      "нижняя граница масштаба привода", "Фильтр")
    k_max: float = _f(1.6, "—", "настройка",
                      "верхняя граница масштаба привода", "Фильтр")
    adapt_on: bool = _f(True, "—", "настройка",
                        "адаптация масштабов привода k_t, k_b по окнам "
                        "установившейся тяги и торможения; false — масштабы "
                        "остаются начальными", "Фильтр")
    t_adapt: float = _f(3.0, "с", "настройка",
                        "окно установившегося режима для адаптации масштаба "
                        "привода (по реальному времени)", "Фильтр")
    sigma_k_meas: float = _f(0.05, "—", "настройка",
                             "σ оконного измерения масштаба привода", "Фильтр")
    adapt_f_min: float = _f(0.3, "—", "настройка",
                            "минимальная средняя нагрузка привода в окне, доля "
                            "номинала", "Фильтр")
    t_sat_hold: float = _f(3.0, "с", "настройка",
                           "после признака срыва отвержение измерений "
                           "считается объяснённым срывом", "Фильтр")
    v_adapt_min: float = _f(2.0, "м/с", "настройка",
                            "ниже этой скорости параметры не адаптируются "
                            "(квантование энкодера у нуля)", "Фильтр")
    t_valid: float = _f(1.0, "с", "настройка",
                        "без принятых измерений дольше — оценка недостоверна",
                        "Фильтр")
    agree_tol: float = _f(0.3, "м/с", "настройка",
                          "оси согласны, если их показания расходятся не больше "
                          "этого; согласие всех осей перевешивает модель",
                          "Фильтр")
    agree_age: float = _f(0.3, "с", "настройка",
                          "показание другой оси годится для проверки согласия, "
                          "если оно не старше этого", "Фильтр")

    # ------------------------------------------------------------ стоянка
    v_standstill: float = _f(0.30, "м/с", "настройка",
                             "полоса неразличимости стоянки", "Стоянка")
    t_standstill: float = _f(0.30, "с", "настройка",
                             "время подтверждения стоянки", "Стоянка")

    # ------------------------------------------------------- выходная σ
    # Публикуемая неопределённость. Ковариация фильтра знает только то, что
    # заложено в модель; на реальных данных к ней добавляются возраст
    # показаний (при торможении и разгоне), ошибка масштаба колёс, шум
    # эталона на стоянке, ошибка карты и точки привязки. Поля калибруются по
    # остаткам обучающих прогонов (analysis/calib_sigma.py); значения по
    # умолчанию не меняют σ фильтра. На саму оценку не влияют.
    sv_gain: float = _f(1.0, "—", "настройка",
                        "множитель σ скорости фильтра в публикуемой σ",
                        "Выходная σ")
    sv_floor: float = _f(0.0, "м/с", "измерение",
                         "пол публикуемой σ скорости на ходу", "Выходная σ")
    sv_floor_stand: float = _f(0.0, "м/с", "измерение",
                               "пол публикуемой σ скорости на стоянке",
                               "Выходная σ")
    sv_age: float = _f(0.0, "с", "измерение",
                       "эффективный возраст показаний: к σ скорости "
                       "добавляется модуль ускорения, умноженный на sv_age",
                       "Выходная σ")
    sv_rel: float = _f(0.0, "—", "измерение",
                       "относительная ошибка масштаба колёс: к σ скорости "
                       "добавляется sv_rel·v", "Выходная σ")
    ss_map: float = _f(0.0, "м", "измерение",
                       "σ положения вдоль пути от карты и точки привязки",
                       "Выходная σ")
    ss_rel: float = _f(0.0, "—", "измерение",
                       "рост σ положения на метр пути после выставки или "
                       "привязки к остановке (масштаб колёс)", "Выходная σ")

    # ------------------------------------------------------------- служебное
    dt: float = _f(0.01, "с", "требование",
                   "шаг фильтра; в ноде задаётся частотой цикла rate_hz",
                   "Служебное")

    def __post_init__(self):
        """Приведение типов и проверка согласованности листа.

        Лист заполняется вручную по ТЗ: «30000» вместо «30000.0», «[1, 1, 0,
        0]» вместо «[true, true, false, false]». Типы приводятся здесь. Ошибки
        согласованности собираются все сразу и выдаются одним понятным
        сообщением при старте, а не падением где-то в середине расчёта.
        """
        errs = []
        for f in fields(self):
            v = getattr(self, f.name)
            try:
                if f.type is float:
                    if isinstance(v, bool):
                        raise ValueError
                    v = float(v)
                elif f.type is bool:
                    if not isinstance(v, (bool, int)) or v not in (0, 1):
                        raise ValueError
                    v = bool(v)
                elif f.type is int:
                    if isinstance(v, bool) or float(v) != int(float(v)):
                        raise ValueError
                    v = int(float(v))
                elif (f.type is tuple and f.default
                      and isinstance(f.default[0], bool)):
                    if not all(isinstance(x, (bool, int)) for x in v):
                        raise ValueError
                    v = tuple(bool(x) for x in v)
                elif f.type is tuple:
                    if any(isinstance(x, bool) for x in v):
                        raise ValueError
                    v = tuple(float(x) for x in v)
                elif f.type is str:
                    if not isinstance(v, str):
                        raise ValueError
                    v = v.strip()
            except (TypeError, ValueError):
                errs.append(f"{f.name}: ожидается {f.type.__name__}, "
                            f"получено {v!r}")
                continue
            object.__setattr__(self, f.name, v)
        if errs:
            raise ValueError("лист параметров:\n  " + "\n  ".join(errs))

        positive = ("M_nom", "r_nom", "v_max_line", "F_notch", "F_brake",
                    "v_base", "v_ed_fade", "tau_drive", "a_max_acc",
                    "a_max_brake", "sigma_meas", "t_adapt", "dt",
                    "sensor_ratio", "stuck_dv", "recover_tol", "t_recover",
                    "meas_scale")

        def grid_ok(g):
            return len(g) >= 2 and all(b > a for a, b in zip(g, g[1:]))

        def table_ok(t):
            return (len(t) >= 1 and all(0 < x <= 1 for x in t)
                    and all(b >= a for a, b in zip(t, t[1:])))
        checks = [(getattr(self, n) > 0, f"{n} должен быть > 0")
                  for n in positive]
        checks += [(getattr(self, n) >= 0, f"{n} должен быть >= 0")
                   for n in ("sv_floor", "sv_floor_stand", "sv_age", "sv_rel",
                             "ss_map", "ss_rel")]
        checks += [
            (self.sv_gain > 0, "sv_gain должен быть > 0"),
            (self.n_axles >= 1, "n_axles должен быть >= 1"),
            (len(self.driven) == self.n_axles,
             f"driven: {len(self.driven)} значений при n_axles = "
             f"{self.n_axles}"),
            (len(self.braked) == self.n_axles,
             f"braked: {len(self.braked)} значений при n_axles = "
             f"{self.n_axles}"),
            (any(self.driven), "driven: нет ни одной моторной оси"),
            (any(self.braked), "braked: нет ни одной тормозной оси"),
            (table_ok(self.notch_tract),
             "notch_tract: нужен непустой неубывающий список долей в (0, 1]"),
            (table_ok(self.notch_brake),
             "notch_brake: нужен непустой неубывающий список долей в (0, 1]"),
            (self.meas_units in UNITS,
             f"meas_units: «{self.meas_units}» не из {sorted(UNITS)}"),
            (self.ppr >= 1, "ppr должен быть >= 1"),
            (self.sensors_per_axle in (1, 2), "sensors_per_axle: 1 или 2"),
            (len(self.sensor_axles) == self.n_axles,
             f"sensor_axles: {len(self.sensor_axles)} значений при n_axles = "
             f"{self.n_axles}"),
            (any(self.sensor_axles), "sensor_axles: нет ни одной оси с датчиком"),
            (self.curve_ratio_max >= 0, "curve_ratio_max должен быть >= 0"),
            (0 <= self.brake_hold_frac <= 1, "brake_hold_frac: от 0 до 1"),
            (self.recover_rel >= 0, "recover_rel должен быть >= 0"),
            (self.delay_drive >= 0, "delay_drive должен быть >= 0"),
            (min(self.res_A, self.res_B, self.res_C) >= 0,
             "res_A, res_B, res_C должны быть >= 0"),
            (0 < self.mu_min <= self.mu_nominal,
             "нужно 0 < mu_min <= mu_nominal"),
            (self.k_min < self.k_max, "нужно k_min < k_max"),
            (not self.acc_table or (grid_ok(self.acc_u) and grid_ok(self.acc_v)
                                    and len(self.acc_table)
                                    == len(self.acc_u) * len(self.acc_v)),
             "acc_u, acc_v: возрастающие сетки не короче 2; acc_table: "
             "len(acc_u)·len(acc_v) значений"),
        ]
        bad = [msg for ok, msg in checks if not ok]
        if bad:
            raise ValueError("лист параметров не согласован:\n  "
                             + "\n  ".join(bad))

    @staticmethod
    def from_dict(d):
        """Значения из словаря (params.yaml). Неизвестный ключ — ошибка:
        опечатка в файле не должна молча оставлять заглушку."""
        names = {f.name for f in fields(Params)}
        bad = sorted(set(d) - names)
        if bad:
            raise KeyError(f"неизвестные параметры: {bad}")
        out = {}
        for k, v in d.items():
            out[k] = tuple(v) if isinstance(v, (list, tuple)) else v
        return Params(**out)

    def with_dt(self, dt):
        return replace(self, dt=float(dt))


DEFAULT = Params()
EP = DEFAULT                     # совместимость со стендом

COAST, TRACTION, BRAKE, TRANSITION, SLIP, STANDSTILL, DEGRADED = range(7)
MODE_NAMES = ["COAST", "TRACTION", "BRAKE", "TRANSITION",
              "SLIP", "STANDSTILL", "DEGRADED"]

NX = 5                           # s, v, d, k_t, k_b
IS, IV, ID, IKT, IKB = range(NX)


# ------------------------------------------------------------------ модель

def notch_to_u(n, p=DEFAULT):
    """Позиция ручки -> доля полного момента по таблицам листа.

    Число позиций тяги и торможения может различаться, зависимость может быть
    нелинейной. Дробная позиция (непрерывная ручка) интерполируется, позиция
    за пределами таблицы даёт полный момент.
    """
    if n > 0:
        tab = p.notch_tract
        return float(np.interp(n, np.arange(len(tab) + 1), (0.0,) + tab))
    if n < 0:
        tab = p.notch_brake
        return -float(np.interp(-n, np.arange(len(tab) + 1), (0.0,) + tab))
    return 0.0


def sensor_to_speed(meas, p=DEFAULT):
    """Показания датчиков -> линейная скорость обода колеса, м/с.

    Угловые единицы (rad_s, rpm, hz, pulses_s) относятся к валу датчика и
    делятся на передаточное число: при датчике на валу двигателя вал крутится
    быстрее колеса. Линейные (m_s, km_h) — уже скорость, передаточное число
    к ним не применяется.
    """
    m = np.asarray(meas, dtype=float) * p.meas_scale
    unit = p.meas_units
    if unit in LINEAR_UNITS:
        return m * LINEAR_UNITS[unit]
    k = 2.0 * np.pi / p.ppr if unit == "pulses_s" else ANGULAR_UNITS[unit]
    return m * k / p.sensor_ratio * p.r_nom


def sensor_layout(p=DEFAULT):
    """Какие элементы вектора показаний относятся к какой оси.

    Показания идут подряд по осям с датчиками, по sensors_per_axle на ось.
    """
    slots, i = [[] for _ in range(p.n_axles)], 0
    for a in range(p.n_axles):
        if p.sensor_axles[a]:
            slots[a] = list(range(i, i + p.sensors_per_axle))
            i += p.sensors_per_axle
    return slots, i


def shape_tract(v, p=DEFAULT):
    """Форма тяговой характеристики: полка, затем ослабление поля."""
    return min(1.0, p.v_base / max(abs(v), 1e-3))


def shape_brake(v, p=DEFAULT):
    """Форма тормозной характеристики.

    Электродинамический тормоз гаснет ниже v_ed_fade, механический замещает
    его на долю brake_hold_frac. Прежняя форма (сила уходит в ноль у самой
    остановки) противоречила описанию и недооценивала торможение как раз там,
    где вагон останавливается.
    """
    ed = min(1.0, abs(v) / p.v_ed_fade)
    return p.brake_hold_frac + (1.0 - p.brake_hold_frac) * ed


_TABLES = {}


def _table(p):
    """Табличный привод листа как массивы; None — параметрическая модель."""
    if not p.acc_table:
        return None
    hit = _TABLES.get(id(p))
    if hit is None or hit[0] is not p:
        U, V = np.asarray(p.acc_u), np.asarray(p.acc_v)
        T = np.asarray(p.acc_table).reshape(len(U), len(V))
        hit = _TABLES[id(p)] = (p, U, V, T)
    return hit[1:]


def _median(xs):
    """Медиана короткого списка без накладных расходов numpy."""
    xs = sorted(xs)
    n = len(xs)
    return float(xs[n // 2]) if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def _cell(grid, x):
    """Индекс ячейки и доля внутри неё; за краями — край."""
    i = bisect_right(grid, x) - 1
    i = 0 if i < 0 else (len(grid) - 2 if i > len(grid) - 2 else i)
    w = (x - grid[i]) / (grid[i + 1] - grid[i])
    return i, (0.0 if w < 0.0 else 1.0 if w > 1.0 else w)


def table_acc(u, v, p=DEFAULT):
    """Билинейная интерполяция таблицы A(u, v); v — число или массив.
    За пределами сеток значение берётся с края."""
    U, V, T = _table(p)
    if np.ndim(v) == 0:
        # скалярный путь без numpy: вызывается на каждую сигма-точку
        i, wu = _cell(p.acc_u, float(u))
        j, wv = _cell(p.acc_v, abs(float(v)))
        lo = (1.0 - wv) * T[i, j] + wv * T[i, j + 1]
        hi = (1.0 - wv) * T[i + 1, j] + wv * T[i + 1, j + 1]
        return float((1.0 - wu) * lo + wu * hi)
    i = int(np.clip(np.searchsorted(U, u, side="right") - 1, 0, len(U) - 2))
    wu = float(np.clip((u - U[i]) / (U[i + 1] - U[i]), 0.0, 1.0))
    va = np.abs(np.asarray(v, dtype=float))
    j = np.clip(np.searchsorted(V, va, side="right") - 1, 0, len(V) - 2)
    wv = np.clip((va - V[j]) / (V[j + 1] - V[j]), 0.0, 1.0)
    lo = (1.0 - wv) * T[i, j] + wv * T[i, j + 1]
    hi = (1.0 - wv) * T[i + 1, j] + wv * T[i + 1, j + 1]
    return (1.0 - wu) * lo + wu * hi


def _drive_nominal(u, v, p):
    """Сила привода при k = 1; v — число или массив.

    Табличная модель: ускорение по таблице минус строка выбега, то есть
    вклад привода сверх сопротивления движению; параметрическая — паспортные
    кривые тяги и торможения.
    """
    if _table(p) is not None:
        return p.M_nom * (table_acc(u, v, p) - table_acc(0.0, v, p))
    if np.ndim(v) == 0:
        va = abs(float(v))
        if u > 0:
            return p.F_notch * u * min(1.0, p.v_base / max(va, 1e-3))
        ed = min(1.0, va / p.v_ed_fade)
        return p.F_brake * u * (p.brake_hold_frac
                                + (1.0 - p.brake_hold_frac) * ed)
    va = np.abs(np.asarray(v, dtype=float))
    if u > 0:
        return p.F_notch * u * np.minimum(1.0, p.v_base / np.maximum(va, 1e-3))
    ed = np.minimum(1.0, va / p.v_ed_fade)
    return p.F_brake * u * (p.brake_hold_frac + (1.0 - p.brake_hold_frac) * ed)


def drive_force(u, v, k_t, k_b, p=DEFAULT):
    """Сила привода по команде. Нелинейна по скорости, линейна по k."""
    if u == 0:
        return 0.0
    return (k_t if u > 0 else k_b) * float(_drive_nominal(u, v, p))


def rated_force(traction, p=DEFAULT):
    """Полная сила привода: паспортная или наибольшая по таблице."""
    if _table(p) is None:
        return p.F_notch if traction else p.F_brake
    V = _table(p)[1]
    return float(np.max(np.abs(_drive_nominal(1.0 if traction else -1.0,
                                              V, p))))


def resistance(v, p=DEFAULT):
    """Сопротивление движению (v >= 0). В табличной модели к паспортной
    формуле добавляется строка выбега таблицы (в листе вагона res_* = 0)."""
    a = abs(v)
    W = p.res_A * (a > 0) + p.res_B * a + p.res_C * a * a
    if _table(p) is not None and a > 0:
        W += max(0.0, -p.M_nom * float(table_acc(0.0, a, p)))
    return W


def axle_load(p=DEFAULT):
    """Вертикальная нагрузка на ось, распределение равномерное."""
    return p.M_nom * G / p.n_axles


def body_force(u, v, k_t, k_b, mu, p=DEFAULT):
    """Сила, реально передаваемая корпусу: привод, ограниченный сцеплением.

    Именно здесь физика проскальзывания: при плохом сцеплении полная тяга
    не реализуется, и разомкнутый прогноз не разгоняет вагон силой, которой
    на льду быть не может.
    """
    F = drive_force(u, v, k_t, k_b, p)
    N = axle_load(p)
    if F > 0:
        return min(F, mu * N * max(sum(p.driven), 1))
    return max(F, -mu * N * max(sum(p.braked), 1))


def axle_shares(F, p=DEFAULT):
    """Доля силы, приходящаяся на каждую ось: моторные под тягой, тормозные
    под торможением."""
    if F > 0:
        sel, n = p.driven, max(sum(p.driven), 1)
    elif F < 0:
        sel, n = p.braked, max(sum(p.braked), 1)
    else:
        return np.zeros(p.n_axles)
    return np.array([1.0 / n if s else 0.0 for s in sel])


def creep(F, p=DEFAULT):
    """Относительное проскальзывание каждой оси.

    Передача силы требует крипа: под тягой колесо опережает корпус, под
    торможением отстаёт. Это предсказание и компенсирует систематическую
    ошибку одометрии: ξ_a = c·F_a/N_a − c_0.
    """
    return p.c_creep * F * axle_shares(F, p) / axle_load(p) - p.c_creep_drag


def f_process(x, u, mu, p=DEFAULT):
    """Прогноз состояния на шаг."""
    s, v, d, k_t, k_b = x
    a = (body_force(u, v, k_t, k_b, mu, p) - resistance(v, p)) / p.M_nom + d
    if v <= 0.0 and a < 0.0:
        a = 0.0                  # вагон не катится назад
    return np.array([s + v * p.dt, v + a * p.dt, d, k_t, k_b])


def h_axles(x, u, mu, p=DEFAULT):
    """Прогноз показаний всех осей.

    Крип считается по НОМИНАЛЬНОЙ силе (k = 1), а не по оцениваемой. Иначе
    измерение крипа, чувствительное к неточности самого коэффициента c,
    тянуло бы масштаб привода k к границе: проверка показала k = 1,6 при
    истинных 0,9. Масштаб привода определяется динамикой (разгоном), а крип
    служит поправкой к показаниям, а не датчиком силы.
    """
    v = x[IV]
    F = body_force(u, v, 1.0, 1.0, mu, p)
    return v * (1.0 + creep(F, p))


def h_axles_batch(pts, u, mu, p=DEFAULT):
    """То же, что h_axles, сразу для всех сигма-точек (матрица точки × оси).

    Скалярный вызов на каждую из 11 точек стоил около миллисекунды на шаг,
    когда принимаются все оси: на Python-накладные расходы. Здесь один
    проход numpy. Знак силы при k = 1 определяется знаком команды, поэтому
    распределение по осям одно на все точки.
    """
    v = pts[:, IV]
    N = axle_load(p)
    if u > 0:
        F = np.minimum(_drive_nominal(u, v, p), mu * N * max(sum(p.driven), 1))
    elif u < 0:
        F = np.maximum(_drive_nominal(u, v, p), -mu * N * max(sum(p.braked), 1))
    else:
        F = np.zeros_like(v)
    sh = axle_shares(float(np.sign(u)), p)
    xi = p.c_creep * F[:, None] * sh[None, :] / N - p.c_creep_drag
    return v[:, None] * (1.0 + xi)


def position_sigma(sigma_s, ds, p=DEFAULT):
    """Публикуемая σ положения вдоль пути: σ пути фильтра, ошибка карты и
    точки привязки, ошибка масштаба колёс на пути ds после выставки или
    последней привязки к остановке. Привязка сбрасывает накопленную ошибку, а
    σ пути фильтра этого не знает."""
    return sqrt(sigma_s * sigma_s + p.ss_map * p.ss_map + (p.ss_rel * ds) ** 2)


# ------------------------------------------------------------------ фильтр

class Estimator:
    """Сигма-точечный фильтр над моделью движения."""

    def __init__(self, params=None):
        p = self.p = params if params is not None else DEFAULT
        self.x = np.array([0.0, 0.0, 0.0, 1.0, 1.0])
        self.P = np.diag([p.p0_s ** 2, p.p0_v ** 2, p.p0_d ** 2,
                          p.p0_k ** 2, p.p0_k ** 2])
        self.mu = p.mu_nominal               # доступное сцепление

        self.mode = DEGRADED
        self.mode_cand = DEGRADED
        self.mode_timer = 0.0
        self.u_filt = 0.0
        # История ручки ограничена длиной запаздывания привода. Прежний список
        # рос без предела: 8,6 млн элементов за сутки при 100 Гц.
        nd = int(round(p.delay_drive / p.dt))
        self.notch_hist = deque([0.0] * (nd + 1), maxlen=nd + 1)

        self.slots, self.nw = sensor_layout(p)   # nw — длина вектора показаний
        self.healthy = np.ones(self.nw, dtype=bool)
        self.stuck_cnt = np.zeros(self.nw)
        self.stuck_ref = np.full(self.nw, np.nan)  # остальные при начале счёта
        self.agree_t = np.zeros(self.nw)           # согласие для возврата
        self.prev_meas = np.zeros(self.nw)
        self.axle_prev = np.zeros(p.n_axles)
        self.axle_dot = np.zeros(p.n_axles)
        self.axle_seen = np.zeros(p.n_axles, dtype=bool)
        # скачок показания оси за один интервал больше физически возможного
        # (см. step): ловится сразу, без сглаживания axle_dot
        self.axle_jump = np.zeros(p.n_axles, dtype=bool)
        self.axle_scale = np.ones(p.n_axles)

        self.initialised = False    # скорость взята из первых показаний
        self.have_meas = False      # показания колёс уже приходили
        # Время последнего нового показания каждого датчика и каждой оси:
        # датчики разных осей приходят в разные моменты и с разной частотой.
        self.t_meas_i = np.full(self.nw, -np.inf)
        self.t_axle = np.full(p.n_axles, -np.inf)
        self.amb = False            # неоднозначность «стоим или скользим»
        self.amb_v = 0.0            # скорость в начале неоднозначности
        self.s_extra = 0.0          # м, добавка к σ пути за время неоднозначности
        self.last_acc_n = 0

        self.zero_t = 0.0
        self.frozen = False         # залипли ВСЕ датчики разом (см. _frozen_all)
        self.t_zero_start = None    # время первого нулевого показания подряд
        self.adapting = False
        self.mode_pre = DEGRADED
        self.win_mode = None
        self.t_last_sat = -1e9
        self.win_t = 0.0
        self.win_t_last = 0.0       # время прошлого вызова _adapt_scale
        self.win_v0 = 0.0
        self.win_F = 0.0
        self.win_W = 0.0
        # диагностика адаптации (только счётчики и суммы, память не растёт)
        self.adapt_stats = dict(windows=0, force_ok=0, gate_ok=0,
                                k_sum=0.0, k_sq=0.0, win_sum=0.0)
        self.t_since_acc = 0.0
        self.t = 0.0
        self.sat = False
        self.n_acc = 0
        self.n_rej = 0
        self.n_override = 0         # показаний, принятых по согласию осей вопреки модели

        # Веса сигма-точек: lambda > 0 и все веса неотрицательны, иначе
        # ковариация теряет положительную определённость.
        n = NX
        alpha, beta, kappa = 1.0, 2.0, 1.0
        self.lam = alpha * alpha * (n + kappa) - n
        self.wm = np.full(2 * n + 1, 1.0 / (2.0 * (n + self.lam)))
        self.wc = self.wm.copy()
        self.wm[0] = self.lam / (n + self.lam)
        self.wc[0] = self.wm[0] + (1.0 - alpha * alpha + beta)

    # ------------------------------------------------------------ вход

    def _drive_model(self, notch):
        """Позиция ручки -> нормированный момент, с запаздыванием привода."""
        p = self.p
        self.notch_hist.append(notch)
        u_del = self.notch_hist[0]          # позиция, заданная delay_drive назад
        target = notch_to_u(u_del, p)
        self.u_filt += (target - self.u_filt) * p.dt / p.tau_drive
        return self.u_filt

    def _mode(self, u):
        p = self.p
        cand = COAST if abs(u) < p.T_dead else (TRACTION if u > 0 else BRAKE)
        if cand != self.mode_cand:
            self.mode_cand, self.mode_timer = cand, 0.0
            return TRANSITION
        self.mode_timer += p.dt
        return cand if self.mode_timer >= p.t_confirm else TRANSITION

    # ------------------------------------------------------------ датчики

    def _diagnose(self, meas, dts, fm):
        """Отказы отдельных датчиков: залипание и обрыв — ВОССТАНАВЛИВАЕМЫЕ.

        Прежде датчик исключался навсегда: одной оси, заклинившей на 0,5 с на
        пятне льда, хватало, чтобы потерять оба её датчика до конца поездки, а
        реальный энкодер на постоянной скорости, выдающий одинаковые отсчёты,
        принимался за залипший. Теперь:

          * залипание признаётся, только если остальные датчики за то же время
            заметно изменились;
          * исключённый датчик возвращается, когда снова меняется и
            согласуется с остальными или с подтверждённой оценкой корпуса в
            течение t_recover;
          * ноль считается обрывом, только если движение подтверждает и
            оценка корпуса, а не одни остальные датчики. Иначе при буксовании
            на месте (моторные колёса крутятся, вагон стоит) исключались как раз
            исправные датчики холостых осей: медиану давали буксующие.

        Общий отказ всех датчиков этим не ловится (некому сравнить) и
        отсекается проверкой правдоподобия при коррекции.

        Проверяются только датчики с новыми показаниями (fm); dts — время с их
        прошлого показания. Остальные датчики участвуют как образец своим
        последним показанием, если оно не старше t_valid.
        """
        p = self.p
        v_body = abs(float(self.x[IV]))
        body_ok = self.initialised and self.t_since_acc <= p.t_valid
        recent = (self.t - self.t_meas_i) <= p.t_valid
        for i in np.flatnonzero(fm):
            dt = dts[i]
            others = [abs(meas[j]) for j in range(self.nw)
                      if j != i and self.healthy[j] and recent[j]]
            ref = _median(others) if others else None
            if abs(meas[i] - self.prev_meas[i]) > 1e-9:
                self.stuck_cnt[i] = 0
            else:
                if self.stuck_cnt[i] == 0:
                    self.stuck_ref[i] = np.nan if ref is None else ref
                self.stuck_cnt[i] += 1

            if self.healthy[i]:
                stuck = (self.stuck_cnt[i] > p.stuck_n and ref is not None
                         and np.isfinite(self.stuck_ref[i])
                         and abs(ref - self.stuck_ref[i]) > p.stuck_dv)
                dead = (ref is not None and abs(meas[i]) < 1e-6
                        and ref > p.v_dead_ref and v_body > p.v_dead_ref)
                if stuck or dead:
                    self.healthy[i] = False
                    self.agree_t[i] = 0.0
            else:
                # «Живой» — менялся за последние stuck_n отсчётов: застывший на
                # нуле или на одном значении датчик так не вернётся.
                w = abs(meas[i])
                tol = p.recover_tol + p.recover_rel * max(ref or 0.0, v_body)
                agree = self.stuck_cnt[i] <= p.stuck_n and (
                    (ref is not None and abs(w - ref) <= tol)
                    or (body_ok and abs(w - v_body) <= tol))
                self.agree_t[i] = self.agree_t[i] + dt if agree else 0.0
                if self.agree_t[i] >= p.t_recover:
                    self.healthy[i] = True
                    self.stuck_cnt[i] = 0
        self.prev_meas[fm] = meas[fm]

    def _axle_speeds(self, meas, fm):
        """Скорость каждой оси по исправным датчикам с НОВЫМИ показаниями."""
        p = self.p
        out = np.zeros(p.n_axles)
        ok = np.zeros(p.n_axles, dtype=bool)
        for a in range(p.n_axles):
            hs = [meas[i] for i in self.slots[a] if self.healthy[i] and fm[i]]
            if hs:
                # среднее по бортам снимает разницу путей на кривой;
                # при одном датчике на ось она учитывается в _meas_var
                out[a] = float(np.mean(hs)) * self.axle_scale[a]
                ok[a] = True
        return out, ok

    def _calibrate(self, z, acc):
        """Масштаб осей выравнивается на выбеге по принятым измерениям: там
        срыв исключён, и расхождение осей относится к датчику."""
        p = self.p
        if self.mode != COAST or len(acc) < 2 or z[acc].min() < p.calib_v_min:
            return
        med = float(np.median(z[acc]))
        for a in acc:
            self.axle_scale[a] += p.calib_gain * (med / z[a] - 1.0)
        self.axle_scale = np.clip(self.axle_scale, 1.0 - p.scale_range,
                                  1.0 + p.scale_range)

    # ------------------------------------------------------------ UKF

    @staticmethod
    def _chol(A):
        """Разложение Холецкого с восстановлением: накопленная ошибка
        округления может вывести ковариацию из положительно определённых."""
        A = 0.5 * (A + A.T)
        try:
            return np.linalg.cholesky(A + 1e-12 * np.eye(A.shape[0]))
        except np.linalg.LinAlgError:
            w, V = np.linalg.eigh(A)
            w = np.clip(w, 1e-10, None)
            return np.linalg.cholesky((V * w) @ V.T + 1e-12 * np.eye(len(w)))

    def _sigma(self):
        n = NX
        S = self._chol((n + self.lam) * self.P)
        pts = np.empty((2 * n + 1, n))
        pts[0] = self.x
        for i in range(n):
            pts[1 + i] = self.x + S[:, i]
            pts[1 + n + i] = self.x - S[:, i]
        return pts

    def _predict(self, u):
        """Прогноз. Дрейф параметров включается ТОЛЬКО там, где параметр
        наблюдаем: возмущение — на выбеге (привод не действует), масштаб тяги —
        под тягой, масштаб торможения — под торможением. Иначе возмущение и
        масштаб привода поглощают друг друга (по данным они неразличимы)."""
        p = self.p
        pts = self._sigma()
        prop = np.array([f_process(pt, u, self.mu, p) for pt in pts])
        x = self.wm @ prop
        dx = prop - x
        P = (dx.T * self.wc) @ dx
        P[IV, IV] += (p.q_v * p.dt) ** 2
        if abs(u) < p.T_dead:
            P[ID, ID] += p.q_d ** 2 * p.dt
        if u > p.T_dead:
            P[IKT, IKT] += p.q_k ** 2 * p.dt
        if u < -p.T_dead:
            P[IKB, IKB] += p.q_k ** 2 * p.dt
        self.x, self.P = x, 0.5 * (P + P.T)

    def _a_limit(self, u, accelerating):
        """Предел изменения скорости корпуса, правдоподобный при текущей
        команде. Замедление до 3 м/с² возможно только при торможении; без
        команды тормоза скорость падает не быстрее уклона и сопротивления.
        Гладкое торможение рельсовым тормозом (сигнала о нём нет) фильтр
        отслеживает шаг за шагом; резкий скачок показаний отвергается."""
        p = self.p
        if accelerating:
            return p.a_max_acc
        return p.a_max_brake if u < -p.T_dead else p.a_free_decel

    def _meas_var(self, F, v):
        """Дисперсия измерения оси: шум датчика, а для нагруженных осей ещё и
        неопределённость самого коэффициента крипа."""
        p = self.p
        sh = axle_shares(F, p)
        var = p.sigma_meas ** 2 + (v * p.sigma_creep) ** 2 * (sh > 0)
        if p.sensors_per_axle == 1:
            # датчик на одном борту: на кривой он читает путь своего рельса
            var = var + (v * p.curve_ratio_max) ** 2
        return var

    def _correct(self, z, ok, u, mode, handle_ok=True):
        """Последовательная коррекция с проверкой правдоподобия.

        Оси упорядочиваются по невязке: сначала согласованное большинство,
        затем подозрительные. Так ось, читающая заведомо больше остальных
        (буксование), не тянет состояние на себя.
        """
        p = self.p
        axles = [a for a in range(p.n_axles) if ok[a]]
        self.n_acc = self.n_rej = 0
        self.sat = False
        acc, rej = [], []
        if not axles:
            return acc

        v0 = self.x[IV]
        F0 = body_force(u, v0, 1.0, 1.0, self.mu, p)
        hbar = h_axles(self.x, u, self.mu, p)
        var0 = self._meas_var(F0, v0)
        order = sorted(axles, key=lambda a: abs(z[a] - hbar[a])
                       / np.sqrt(var0[a] + self.P[IV, IV]))

        shares0 = axle_shares(F0, p)
        recent_sat = (self.t - self.t_last_sat) <= p.t_sat_hold
        # Признаки срыва оцениваются только на ходу. У самого нуля энкодер
        # даёт квантованные скачки и нули, и это выглядело бы срывом:
        # сцепление падало при каждой остановке, а разгон после неё блокировался.
        fast = v0 > p.v_adapt_min

        # Адаптация параметров только там, где они наблюдаемы. Возмущение —
        # на установившемся выбеге, масштаб тяги — на установившейся тяге,
        # масштаб торможения — на установившемся торможении, и только выше
        # порога скорости: у самого нуля энкодер даёт квантованные скачки, и
        # один переходный процесс старта загонял масштаб тяги в границу, а
        # ковариацию схлопывал (фильтр становился уверенным в неверном).
        # Масштабы привода через коррекцию по скорости НЕ обновляются вовсе:
        # там они получали смещение измерений (крип, радиус), а полезный
        # сигнал на шаге на порядок слабее. Они адаптируются отдельным
        # окном — см. _adapt_scale.
        mask = np.ones(NX)
        moving = self.x[IV] > p.v_adapt_min
        # без достоверной ручки режим неизвестен — возмущение не адаптируется
        mask[ID] = float(mode == COAST and moving and handle_ok)
        mask[IKT] = 0.0
        mask[IKB] = 0.0

        # СОГЛАСИЕ ОСЕЙ. Модель может ошибаться сама: неверная позиция ручки
        # (в данных вагон разгоняется при «тормозной» позиции), уклон, масса.
        # Прежде фильтр верил модели больше, чем всем осям сразу, и при
        # команде тормоза держал скорость на нуле, пока вагон разгонялся. Если
        # показание согласно со ВСЕМИ остальными осями (их последними
        # показаниями не старше agree_age), ни одна ось не ускоряется
        # физически невозможно, а срыв этим не объясняется, — ошибается модель:
        # неопределённость скорости расширяется до расхождения, и показание
        # принимается. Буксование части осей согласия не даёт (холостые
        # расходятся с моторными), резкий срыв ловит предел ускорения.
        spin_all = (np.abs(self.axle_dot) > np.where(
            self.axle_dot > 0, p.a_max_acc, p.a_max_brake) + p.a_slip_margin
                    ) | self.axle_jump
        recent = (self.t - self.t_axle) <= p.agree_age

        def agreed(a):
            others = [b for b in range(p.n_axles)
                      if b != a and (recent[b] or ok[b])]
            if not others or spin_all[[a] + others].any():
                return False
            zb = [z[b] if ok[b] else self.axle_prev[b] for b in others]
            return all(abs(z[a] - x) <= p.agree_tol for x in zb)

        # Сигма-точки и их образы пересчитываются только после принятого
        # измерения — отвергнутое состояние не меняет.
        pts = self._sigma()
        Zall = h_axles_batch(pts, u, self.mu, p)

        for a in order:
            # физический предел: колесо не может ускоряться быстрее корпуса
            dot = self.axle_dot[a]
            lim = p.a_max_acc if dot > 0 else p.a_max_brake
            spinning = abs(dot) > lim + p.a_slip_margin or bool(self.axle_jump[a])

            Z = Zall[:, a]
            zh = float(self.wm @ Z)
            dz = Z - zh
            Fv = body_force(u, self.x[IV], 1.0, 1.0, self.mu, p)
            R = float(self._meas_var(Fv, self.x[IV])[a])
            S = float(self.wc @ (dz * dz)) + R
            nu = float(z[a]) - zh

            nis = nu * nu / S
            # Срыв в самом разгаре: показания нагруженной оси, отклонённые в
            # сторону срыва, не принимаются, даже если рост неопределённости
            # уже расширил допуск. Иначе нули заблокированных колёс «дозревали»
            # до принятия за остановку, пока вагон ещё скользит. Как только
            # колёса схватят, показания уйдут из этой стороны и пройдут.
            lock = F0 < 0 and nu < 0 and z[a] < p.lock_ratio * hbar[a]
            in_sat_dir = ((F0 > 0 and nu > 0) or lock) \
                and shares0[a] > 0 and fast
            forced = recent_sat and in_sat_dir and nis > 0.25 * p.gate_nis
            # Пока неоднозначность не снята, нули колёс не принимаются: они
            # одинаково объясняются и остановкой, и скольжением заблокированных
            # колёс. Скорость ведёт модель с пониженным сцеплением — оценка
            # остаётся сверху, а не падает в ноль при скользящем вагоне.
            held = self.amb and z[a] < p.v_standstill

            if (nis > p.gate_nis and not (spinning or forced or held)
                    and agreed(a)):
                # модель противоречит согласным осям: расширить и принять
                self.P[IV, IV] = max(self.P[IV, IV], nu * nu)
                self.P[ID, ID] = max(self.P[ID, ID], (p.sigma_rej_frac * max(
                    self._a_limit(u, True), self._a_limit(u, False))) ** 2)
                pts = self._sigma()
                Zall = h_axles_batch(pts, u, self.mu, p)
                Z = Zall[:, a]
                zh = float(self.wm @ Z)
                dz = Z - zh
                S = float(self.wc @ (dz * dz)) + R
                nu = float(z[a]) - zh
                nis = 0.0
                self.n_override += 1

            if spinning or forced or held or nis > p.gate_nis:
                rej.append((a, nu))
                continue
            C = ((pts - self.x).T * self.wc) @ dz
            K = (C / S) * mask
            self.x = self.x + K * nu
            # Общая форма обновления, верная для любого усиления K: при
            # маскировании параметр не меняется, а ковариация остаётся
            # положительно определённой.
            self.P = (self.P - np.outer(K, C) - np.outer(C, K)
                      + np.outer(K, K) * S)
            self.P = 0.5 * (self.P + self.P.T)
            acc.append(a)
            pts = self._sigma()
            Zall = h_axles_batch(pts, u, self.mu, p)

        self.n_acc, self.n_rej = len(acc), len(rej)

        # Все измерения отвергнуты: состояние не знает, что происходит.
        # Неопределённость по ускорению раздувается до физического предела,
        # иначе фильтр остался бы заблокирован навсегда, если ошибся сам он.
        # Признак насыщения сцепления: нагруженная ось отвергнута в сторону,
        # соответствующую срыву (читает больше под тягой, меньше при торможении).
        if rej and fast:
            sh = axle_shares(F0, p)
            for a, nu in rej:
                lock = F0 < 0 and nu < 0 and z[a] < p.lock_ratio * hbar[a]
                if sh[a] > 0 and ((F0 > 0 and nu > 0) or lock):
                    self.sat = True

        # Все измерения отвергнуты и срывом это НЕ объясняется: ошибиться мог
        # сам фильтр (или отказали все датчики). Неопределённость по ускорению
        # раздувается до физического предела, чтобы блокировка закончилась за
        # ограниченное время. Если отвержение объясняется срывом (блокировка
        # колёс при торможении), раздувать нечего: модель с пониженным
        # сцеплением несёт состояние сама, а нулевые показания ожидаемы.
        if self.sat:
            self.t_last_sat = self.t
        recent_sat = (self.t - self.t_last_sat) <= p.t_sat_hold
        # Ненагруженная ось срываться не может. Если отвергнута она, ошибся
        # сам фильтр, и срывом это объяснить нельзя: без этого признак срыва на
        # моторных осях продлевал бы блокировку сам себя, пока вагон уезжает.
        idle_rejected = any(shares0[a] == 0 for a, _ in rej)
        if not acc and rej and (not recent_sat or idle_rejected):
            up = float(np.median([nu for _, nu in rej])) > 0
            self.P[ID, ID] = max(self.P[ID, ID],
                                 (p.sigma_rej_frac * self._a_limit(u, up)) ** 2)

        # НЕОДНОЗНАЧНОСТЬ «стоим или скользим». Все оси отвергнуты из-за срыва
        # на ходу: колёса не свидетельствуют о скорости корпуса, и дальше по
        # этим входам нельзя отличить скольжение от остановки. Прежде через
        # ~25 с нули принимались, и публиковалось v = 0 с почти нулевой σ и
        # valid = true, пока вагон ещё скользил. Теперь это состояние
        # держится до тех пор, пока колёса снова не покатятся (см. _finish),
        # а нули до тех пор отвергаются (held выше).
        # Только при торможении: под тягой срывающиеся колёса читают БОЛЬШЕ
        # корпуса, с остановкой это не спутать. И только из подтверждённого
        # состояния: если колёса давно не принимались (например, после
        # буксования всех осей), «блокировку» нельзя отличить от ошибки самой
        # оценки — тогда защёлка держала бы неверную скорость (+315 м на
        # имитаторе), и работает общий механизм раздувания неопределённости.
        # Второй вход — все колёса разом показали ноль при подтверждённом
        # движении: блокировка или общий отказ датчиков, остановку это не
        # подтверждает ни в одном случае. Прежде нули после раздувания
        # неопределённости принимались, и оценка падала в ноль на ходу.
        entry_ok = self.amb or self.t_since_acc <= p.t_valid
        all_zero = all(z[a] < p.v_standstill for a in axles) \
            and v0 > p.v_dead_ref
        if not acc and rej and entry_ok and ((self.sat and F0 < 0) or all_zero):
            if not self.amb:
                self.amb_v = float(v0)
            self.amb = True
            self.amb_v = max(self.amb_v, float(v0))
        return acc

    # ------------------------------------------------------------ шаг

    def step(self, notch, meas, fresh=True, handle_ok=True):
        """Один шаг фильтра.

        `meas` — показания датчиков в единицах meas_units, по sensors_per_axle
        на каждую ось из sensor_axles, подряд по осям.

        `fresh` — пришли ли показания колёс ПОСЛЕ прошлого шага. Датчики могут
        выдавать данные реже, чем идёт цикл фильтра (20 Гц при 100 Гц). Прежде
        одно и то же показание учитывалось как новое на каждом шаге, а
        производная скорости колеса скачком росла при приходе нового — это
        давало ложные «срывы». Теперь на шаге без новых данных — только
        прогноз, а производная считается по реальному интервалу.

        `handle_ok` — достоверен ли сигнал ручки. Если нет, скорость и путь
        по-прежнему корректируются по колёсам, но параметры модели не
        адаптируются: иначе их испортила бы устаревшая команда.

        `fresh` может быть и маской по датчикам: датчики разных осей приходят
        в разные моменты. Тогда в коррекции участвуют только новые показания,
        а устаревшие служат лишь образцом при диагностике.
        """
        p = self.p
        self.t += p.dt
        u = self._drive_model(notch)

        fm = np.broadcast_to(np.asarray(fresh, dtype=bool), (self.nw,)).copy()
        fresh = bool(fm.any())
        z = np.zeros(p.n_axles)
        ok = np.zeros(p.n_axles, dtype=bool)
        if fresh:
            meas = np.asarray(meas, dtype=float)
            if meas.shape != (self.nw,):
                raise ValueError(f"ожидается {self.nw} показаний датчиков, "
                                 f"получено {meas.size}")
            seen = np.isfinite(self.t_meas_i)
            # после разрыва потока интервал ограничен: иначе одно показание
            # засчитывалось бы датчику как долгое согласие при возврате
            dts = np.where(seen, np.minimum(
                self.t - np.where(seen, self.t_meas_i, 0.0), p.t_valid), p.dt)
            self.t_meas_i[fm] = self.t
            self.have_meas = True
            # всё дальше — в м/с по ободу колеса; пороги листа тоже в м/с
            meas = sensor_to_speed(meas, p)
            self._diagnose(meas, dts, fm)
            self._frozen_all(meas, u)
            z, ok = self._axle_speeds(meas, fm)
            self.axle_jump[:] = False
            for a in range(p.n_axles):
                if not ok[a]:
                    continue
                if not self.axle_seen[a]:        # первое показание оси
                    self.axle_prev[a] = z[a]
                    self.axle_seen[a] = True
                    self.t_axle[a] = self.t - p.dt
                gap = self.t - self.t_axle[a]
                raw = (z[a] - self.axle_prev[a]) / gap
                # Скачок за один интервал больше физически возможного (предел
                # ускорения с запасом плюс допуск согласия осей) — срыв или
                # отказ датчика СРАЗУ, а не только по сглаженной axle_dot:
                # сглаженная производная пересекала порог или нет в
                # зависимости от интервала между показаниями (фаза сетки,
                # 10 Гц с пропусками). Иначе обе тележки, разом упавшие с
                # 4 м/с в ноль, при интервале 0,2 с принимались «по согласию
                # осей» за остановку (инъекция both_zero, 30639_d3c43d69).
                dz = z[a] - self.axle_prev[a]
                lim_j = p.a_max_acc if dz > 0 else p.a_max_brake
                self.axle_jump[a] = abs(dz) > (lim_j + p.a_slip_margin) * gap + p.agree_tol
                self.axle_dot[a] += p.axle_dot_alpha * (raw - self.axle_dot[a])
                self.axle_prev[a] = z[a]
                self.t_axle[a] = self.t

            # Перезапуск на ходу: скорость берётся из первых показаний, а не
            # из нуля. Прежде при старте ноды на 12 м/с до выхода на точность
            # уходило ~7 с — фильтр отвергал «слишком большие» показания.
            # Срыв искажает показания в одну сторону: под тягой буксующие
            # колёса читают больше корпуса, при торможении заблокированные —
            # меньше. Поэтому под тягой берётся минимум, при торможении —
            # максимум; медиана при старте во время буксования половины осей
            # давала 25 м/с стоящему вагону.
            if not self.initialised and ok.any():
                zs = z[ok]
                v0 = float(zs.min() if notch > 0 else
                           zs.max() if notch < 0 else np.median(zs))
                self.x[IV] = v0
                self.P[IV, IV] = p.sigma_meas ** 2 + (v0 * p.sigma_creep) ** 2
                self.initialised = True

        # После стоянки σ скорости сжата до нуля, и первые показания движения
        # выглядели бы выбросом. Трогание с места — ожидаемое событие.
        if self.mode == STANDSTILL and abs(u) > p.T_dead:
            self.P[IV, IV] = max(self.P[IV, IV], p.sigma_meas ** 2)
        self._predict(u)
        mode = self._mode(u)
        self.mode_pre = mode
        if self.frozen:
            # Показания залипли все разом: о скорости они не говорят. Только
            # прогноз по ручке, неопределённость по ускорению — до предела,
            # как при пропаже показаний (step_open_loop). Обычная проверка
            # правдоподобия этого не ловит: залипшие тележки согласны друг с
            # другом, и правило «согласие осей сильнее модели» их принимало
            # (−25…−33 м пути за 20 с на реальных записях).
            lim = max(self._a_limit(u, True), self._a_limit(u, False))
            self.P[ID, ID] = max(self.P[ID, ID], (p.sigma_rej_frac * lim) ** 2)
            self.adapting = False
            self.n_acc = self.n_rej = 0
            self.sat = False
            self.last_acc_n = 0
            return self._finish(u, z, np.zeros(p.n_axles, dtype=bool), [],
                                False)
        if fresh:
            acc = self._correct(z, ok, u, mode, handle_ok)
            self.last_acc_n = len(acc)
            if handle_ok:
                self._adapt_scale(u, mode, acc)
            else:
                self.adapting = False
        else:
            acc = []
        return self._finish(u, z, ok, acc, fresh)

    def _frozen_all(self, meas, u):
        """Залипание ВСЕХ датчиков разом (обе тележки выдают одно и то же).

        Одиночное залипание ловит _diagnose: остальные датчики меняются, а
        этот нет. Когда застыли все, сравнивать не с чем. Признак: у каждого
        датчика больше stuck_n одинаковых показаний подряд (бит в бит), на
        ходу (все выше v_dead_ref) и при команде тяги или торможения, когда
        скорость обязана меняться. На реальных записях вагона на ходу больше
        4 одинаковых показаний подряд не бывает даже у одной тележки; на
        выбеге при постоянной скорости квантованный энкодер может повторяться,
        поэтому выбег признака не даёт. Признак снимается, как только любой
        датчик изменился."""
        p = self.p
        if not np.all(self.stuck_cnt > p.stuck_n):
            self.frozen = False
        elif (not self.frozen and abs(u) > p.T_dead
              and float(np.min(np.abs(meas))) > p.v_dead_ref):
            self.frozen = True

    def _adapt_scale(self, u, mode, acc):
        """Адаптация масштаба тяги и торможения по приращению скорости.

        За окно установившейся тяги (торможения) корпус меняет скорость на

            M (v2 − v1) = k · ∫F_ном dt − ∫W dt + M d T,

        откуда k выражается через измеренное приращение. Разность скоростей
        гасит постоянное смещение одометрии (крип, ошибка радиуса), поэтому
        сигнал устойчив там, где шаговая невязка бесполезна. Результат входит
        в фильтр как обычное линейное измерение параметра.
        """
        p = self.p
        active = (p.adapt_on and mode in (TRACTION, BRAKE) and len(acc) >= 2
                  and self.x[IV] > p.v_adapt_min and not self.sat)
        self.adapting = bool(active)
        if not active or mode != self.win_mode:
            self.win_mode = mode if active else None
            self.win_t = 0.0
            self.win_t_last = self.t
            self.win_v0 = float(self.x[IV])
            self.win_F = 0.0
            self.win_W = 0.0
            return

        v = float(self.x[IV])
        # Окно копит РЕАЛЬНОЕ прошедшее время: вызов идёт только на шагах с
        # новыми показаниями, а тележки приходят реже цикла фильтра (9,4 Гц
        # при 20 Гц). Прежде окно копило p.dt на вызов: «3 с» длились 6–7 с,
        # интеграл силы был вдвое меньше приращения скорости, k_изм выходил
        # около 2,3, и ворота отвергали 97–100 % окон на данных вагона.
        h = self.t - self.win_t_last
        self.win_t_last = self.t
        # Сила привода БЕЗ ограничения оценкой сцепления: после короткого срыва
        # оценка μ ещё десятки секунд остаётся низкой, урезала бы силу в окне
        # и завышала масштаб тяги. Окно и так работает только без срыва.
        self.win_F += drive_force(u, v, 1.0, 1.0, p) * h
        self.win_W += resistance(v, p) * h
        self.win_t += h
        if self.win_t < p.t_adapt:
            return

        rated = rated_force(mode == TRACTION, p)
        st = self.adapt_stats
        st["windows"] += 1
        st["win_sum"] += self.win_t
        if abs(self.win_F) >= p.adapt_f_min * rated * self.win_t:
            dv = v - self.win_v0
            k_meas = (p.M_nom * (dv - self.x[ID] * self.win_t) + self.win_W) \
                / self.win_F
            idx = IKT if mode == TRACTION else IKB
            S = self.P[idx, idx] + p.sigma_k_meas ** 2
            st["force_ok"] += 1
            st["k_sum"] += k_meas
            st["k_sq"] += k_meas * k_meas
            # Окно с неверной командой (ошибка ручки) или неучтённым уклоном
            # даёт мусорное k: такое измерение отвергается, как и показание
            # оси. Без этого масштабы на реальных данных упирались в k_max.
            if (k_meas - self.x[idx]) ** 2 <= p.gate_nis * S:
                st["gate_ok"] += 1
                K = self.P[:, idx] / S
                self.x = self.x + K * (k_meas - self.x[idx])
                self.P = self.P - np.outer(K, self.P[idx, :])
                self.P = 0.5 * (self.P + self.P.T)
        self.win_t = 0.0
        self.win_v0 = v
        self.win_F = 0.0
        self.win_W = 0.0

    def step_open_loop(self, notch):
        """Шаг без одометрии: датчики недоступны (устаревшие данные).

        Только прогноз по модели. Неопределённость по ускорению раздувается
        до физического предела: без измерений модель не знает, что происходит.
        Нули вместо показаний подавать нельзя — это выглядело бы остановкой.
        """
        p = self.p
        self.t += p.dt
        u = self._drive_model(notch)
        self._predict(u)
        self.mode_pre = self._mode(u)
        self.adapting = False
        self.n_acc = self.n_rej = 0
        self.sat = False
        lim = max(self._a_limit(u, True), self._a_limit(u, False))
        self.P[ID, ID] = max(self.P[ID, ID], (p.sigma_rej_frac * lim) ** 2)
        # Поток колёс прерван: по возвращении производная оси начинается
        # заново, иначе разность со старым показанием выглядела бы рывком.
        self.have_meas = False
        self.axle_seen[:] = False
        self.last_acc_n = 0
        return self._finish(u, np.zeros(p.n_axles),
                            np.zeros(p.n_axles, dtype=bool), [], False)

    def _sigma_v_out(self, sv, u):
        """Публикуемая σ скорости: σ фильтра (с множителем) и то, чего фильтр
        не видит. Показание тележки в момент шага уже старое на доли секунды:
        при торможении и разгоне оценка отстаёт на |a|·возраст (на данных
        +0,05 м/с на торможении). Масштаб колёс плавает по вагонам и датам
        (±0,5 %), эталон на стоянке шумит. Калибруется по остаткам обучающих
        прогонов; внутренняя ковариация фильтра не меняется."""
        p = self.p
        v = float(self.x[IV])
        a = 0.0
        if p.sv_age:                # ускорение модели — как в выходе связки
            a = ((body_force(u, v, self.x[IKT], self.x[IKB], self.mu, p)
                  - resistance(v, p)) / p.M_nom + float(self.x[ID]))
            if v <= 0.0 and a < 0.0:
                a = 0.0
        floor = p.sv_floor_stand if self.mode == STANDSTILL else p.sv_floor
        return sqrt((p.sv_gain * sv) ** 2 + floor * floor
                    + (p.sv_age * a) ** 2 + (p.sv_rel * v) ** 2)

    def _finish(self, u, z, ok, acc, fresh):
        """Общая часть шага: параметры, границы, стоянка, режим, выход.

        На шаге без новых показаний (fresh = False) решения, опирающиеся на
        показания, — стоянка, калибровка, восстановление сцепления — не
        пересматриваются, а сохраняются с прошлого нового показания.
        """
        p = self.p

        # сцепление: падает при признаках срыва, медленно восстанавливается
        if self.sat:
            self.mu += (p.mu_min - self.mu) * p.dt / p.tau_sat
        elif self.last_acc_n:
            self.mu += (p.mu_nominal - self.mu) * p.dt / p.tau_rec

        self.t_since_acc = 0.0 if acc else self.t_since_acc + p.dt

        # Неоднозначность снимается, только когда колёса снова катятся:
        # принятые показания выше полосы стоянки и срыва нет. Подтвердить
        # остановку после скольжения по этим входам нельзя, поэтому после
        # остановки на льду она снимется лишь при трогании.
        if (fresh and self.amb and acc and not self.sat
                and float(z[acc].max()) > p.v_standstill):
            self.amb = False
            self.amb_v = 0.0

        # физические границы состояния
        d_max = G * np.sin(p.theta_max) + p.d_extra
        self.x[IV] = float(np.clip(self.x[IV], 0.0, p.v_max_line))
        self.x[ID] = float(np.clip(self.x[ID], -d_max, d_max))
        self.x[IKT] = float(np.clip(self.x[IKT], p.k_min, p.k_max))
        self.x[IKB] = float(np.clip(self.x[IKB], p.k_min, p.k_max))

        # стоянка: принятые измерения на нуле, оценка тоже, неоднозначности нет
        # Время считается от первого нулевого показания, а не числом шагов:
        # показания могут приходить реже цикла фильтра.
        if fresh:
            zero = bool(acc) and float(z[acc].max()) < p.v_standstill
            if not zero:
                self.t_zero_start = None
            elif self.t_zero_start is None:
                self.t_zero_start = self.t - p.dt
        elif not self.have_meas:
            self.t_zero_start = None     # поток колёс прерван
        self.zero_t = (0.0 if self.t_zero_start is None
                       else self.t - self.t_zero_start)
        standstill = (self.zero_t >= p.t_standstill
                      and self.x[IV] < 2.0 * p.v_standstill
                      and not self.amb)
        if standstill:
            self.x[IV] = 0.0
            self.x[ID] = 0.0
            self.P[IV, IV] = min(self.P[IV, IV], 1e-4)

        if fresh:
            self._calibrate(z, acc)

        self.mode = self.mode_pre
        if self.sat or self.amb:
            self.mode = SLIP
        if standstill:
            self.mode = STANDSTILL
        valid = (self.have_meas and self.t_since_acc <= p.t_valid
                 and not self.amb and not self.frozen)
        if not self.have_meas or (not valid and not self.amb):
            self.mode = DEGRADED

        # Публикуемая σ не может быть меньше того, что известно о скорости:
        # при неоднозначности вагон может ехать с любой скоростью от 0 до
        # скорости начала срыва. Внутренняя ковариация фильтра не меняется.
        # Путь за это время известен с той же неопределённостью: добавка к σ
        # пути нарастает и остаётся — без внешней привязки ошибка пути не уходит.
        sv_filt = float(np.sqrt(max(self.P[IV, IV], 0.0)))
        sv = self._sigma_v_out(sv_filt, u)
        if self.amb:
            sv = max(sv, 0.5 * self.amb_v)
            self.s_extra += 0.5 * self.amb_v * p.dt
        ss = float(np.sqrt(max(self.P[IS, IS], 0.0) + self.s_extra ** 2))
        return dict(
            v=float(self.x[IV]), s=float(self.x[IS]),
            d=float(self.x[ID]), k_t=float(self.x[IKT]), k_b=float(self.x[IKB]),
            mu=float(self.mu),
            sigma_v=sv, sigma_s=ss, sigma_v_filt=sv_filt,
            mode=self.mode, healthy=self.healthy.copy(),
            slip=self.sat, ambiguous=self.amb, frozen=self.frozen,
            n_accepted=self.n_acc, n_rejected=self.n_rej,
            odometry_used=bool(acc), valid=valid,
        )
