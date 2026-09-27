"""Модель трамвая для проверки оценщика.

Сознательно содержит эффекты, которых в модели оценщика НЕТ:
двухмассовый привод с крутильной податливостью, падающую характеристику
сцепления, перераспределение нагрузки по осям, независимые колёса, кривые,
рельсовый тормоз, квантование энкодера, транспортную задержку, отказы.

Оценщик видит только notch и измеренные частоты вращения колёс.
"""

from collections import deque

import numpy as np

G = 9.81


class Params:
    # --- корпус ---
    M = 30000.0
    r = 0.35
    b = 0.762            # половина колеи 1524 мм
    n_axles = 4
    motored = (True, True, False, False)   # 2 моторные оси, 2 прицепные

    res_A = 2500.0       # Н
    res_B = 60.0
    res_C = 4.0

    # --- привод: двухмассовая модель на колесо ---
    J_motor = 30.0       # кг·м^2, приведённый ротор
    J_wheel = 30.0       # кг·м^2, колесо
    J_trailer = 12.0
    k_tors = 4.0e4       # Н·м/рад
    c_tors = 1.6e2       # Н·м·с/рад
    T_max = 6300.0       # Н·м на моторную ось
    v_base = 8.0
    tau_drive = 0.25
    delay_drive = 0.08
    jerk_lim = 3.0

    # --- тормоза ---
    T_mech_max = 4000.0
    v_ed_fade = 1.5
    F_track_brake = 30000.0   # Н: ~1 м/с² сверх колёсного торможения, итого ~3,5 м/с²

    # --- сцепление ---
    dv_peak_rel = 0.01   # пик при 1% крипа
    dv_peak_min = 0.05   # м/с, нижний предел для обусловленности

    B_wheel = 3.0        # Н·м·с/рад, сопротивление вращению

    # --- датчики ---
    ppr = 200            # импульсов на оборот
    t_meas = 0.10        # с, окно измерения
    sensor_noise = 0.01  # рад/с
    meas_delay = 0.02    # с
    dt_est = 0.01


P = Params()


def adhesion_mu(dv, v, mu_max):
    """Падающая характеристика: пик, затем спад (зона неустойчивости)."""
    dv0 = max(P.dv_peak_rel * abs(v), P.dv_peak_min)
    x = dv / dv0
    return 2.0 * mu_max * x / (1.0 + x * x)


class Track:
    def __init__(self, grade=lambda s: 0.0, radius=lambda s: np.inf,
                 mu=lambda s, t: 0.25):
        self.grade = grade
        self.radius = radius
        self.mu = mu


class Plant:
    def __init__(self, track: Track, dt=0.001, seed=0):
        self.tr = track
        self.dt = dt
        self.rng = np.random.default_rng(seed)

        self.nw = 2 * P.n_axles
        self.v = 0.0
        self.s = 0.0
        self.w = np.zeros(self.nw)        # колёса, рад/с
        self.wm = np.zeros(self.nw)       # роторы, рад/с
        self.phi = np.zeros(self.nw)      # закрутка вала, рад

        self.u_filt = 0.0
        # Истории - кольцевые буферы длиной в задержку: простые списки росли
        # бы без предела (8,6 млн элементов за сутки при 100 Гц).
        nd = int(P.delay_drive / dt)
        self.notch_hist = deque([0.0] * (nd + 1), maxlen=nd + 1)
        self.t = 0.0
        self._accel = 0.0

        self.scale = 1.0 + self.rng.uniform(-0.015, 0.015, self.nw)
        self.angle = np.zeros(self.nw)    # накопленный угол
        self.angle_hist = deque(maxlen=int(P.t_meas / dt) + 1)
        self.meas_hist = deque(maxlen=int(P.meas_delay / dt) + 1)
        self.faults = {}
        self._stuck = {}

    def _axle_of(self, i):
        return i // 2

    def _side(self, i):
        return 1 if (i % 2 == 0) else -1

    def _rail_speed(self, i):
        R = self.tr.radius(self.s)
        if not np.isfinite(R):
            return self.v
        return self.v * (R + self._side(i) * P.b) / R

    def _normal_load(self, i):
        base = P.M * G / self.nw
        transfer = -self._accel * 0.06 * base / G
        sign = 1.0 if self._axle_of(i) < P.n_axles / 2 else -1.0
        return max(base + sign * transfer, 0.2 * base)

    def _drive(self, notch):
        self.notch_hist.append(notch)
        u_del = self.notch_hist[0]
        target = np.clip(u_del / 4.0, -1.0, 1.0)
        # Апериодическое звено с ограничителем скорости нарастания. Прежняя
        # запись умножала шаг ограничителя на dt/tau*4 = 0.016, и привод
        # набирал момент ~20 с вместо 0,3 с: оценщик компенсировал этот дефект
        # имитатора занижением масштаба тяги.
        lag = (target - self.u_filt) * self.dt / P.tau_drive
        self.u_filt += float(np.clip(lag, -P.jerk_lim * self.dt,
                                     P.jerk_lim * self.dt))
        return self.u_filt

    def step(self, notch, sand=False, track_brake=False):
        dt = self.dt
        u = self._drive(notch)
        fade = 1.0 if abs(self.v) < P.v_base else P.v_base / abs(self.v)
        T_cmd_axle = u * P.T_max * fade
        if u < 0 and abs(self.v) < P.v_ed_fade:
            T_cmd_axle *= abs(self.v) / P.v_ed_fade      # ЭД тормоз гаснет

        F_tot = 0.0
        for i in range(self.nw):
            ax = self._axle_of(i)
            mot = P.motored[ax]
            v_rail = self._rail_speed(i)
            dv = self.w[i] * P.r - v_rail

            mu_max = self.tr.mu(self.s, self.t)
            if sand:
                mu_max = min(mu_max * 2.5, 0.30)
            F = adhesion_mu(dv, self.v, mu_max) * self._normal_load(i)
            F_tot += F

            T_sh = 0.0
            if mot:
                T_sh = P.k_tors * self.phi[i] + P.c_tors * (self.wm[i] - self.w[i])
                self.wm[i] += (T_cmd_axle / 2.0 - T_sh) / P.J_motor * dt
                self.phi[i] += (self.wm[i] - self.w[i]) * dt

            T = T_sh - P.B_wheel * self.w[i]

            if u < 0:   # механический тормоз, подхват при затухании ЭД
                blend = 1.0 if abs(self.v) < P.v_ed_fade else 0.3
                T -= np.sign(self.w[i]) * P.T_mech_max * abs(u) * blend \
                    if abs(self.w[i]) > 1e-2 else 0.0

            J = P.J_wheel if mot else P.J_trailer
            w_new = self.w[i] + (T - P.r * F) / J * dt
            if u < 0 and self.w[i] > 0 and w_new < 0:
                w_new = 0.0             # тормоз не крутит колесо назад
            self.w[i] = w_new

        F_res = P.res_A + P.res_B * abs(self.v) + P.res_C * self.v ** 2
        F_grade = P.M * G * np.sin(self.tr.grade(self.s))
        F_tb = P.F_track_brake if track_brake else 0.0
        F_drag = (F_res + F_tb) * np.sign(self.v) if abs(self.v) > 1e-3 else 0.0

        F_net = F_tot - F_grade - F_drag
        if abs(self.v) < 1e-3 and abs(F_tot - F_grade) < F_res:
            F_net = 0.0                 # трогание требует преодоления сопротивления

        self._accel = F_net / P.M
        self.v += self._accel * dt
        self.s += self.v * dt
        self.t += dt

        for i in range(self.nw):
            self.angle[i] += self.w[i] * self.scale[i] * dt
        return self._accel

    def measure(self):
        """Скорость из накопленного угла за окно t_meas с квантованием по импульсам."""
        self.angle_hist.append(self.angle.copy())
        quant = 2 * np.pi / P.ppr
        a_now = np.floor(self.angle / quant) * quant
        if len(self.angle_hist) == self.angle_hist.maxlen:
            a_old = np.floor(self.angle_hist[0] / quant) * quant
            meas = (a_now - a_old) / P.t_meas
        else:
            meas = np.zeros(self.nw)
        meas = meas + self.rng.normal(0, P.sensor_noise, self.nw)

        for i, (t0, kind) in self.faults.items():
            if self.t >= t0:
                if kind == "zero":
                    meas[i] = 0.0
                elif kind == "stuck":
                    self._stuck.setdefault(i, float(meas[i]))
                    meas[i] = self._stuck[i]
                elif kind == "noise":
                    meas[i] += self.rng.normal(0, 3.0)

        self.meas_hist.append(meas.copy())
        if len(self.meas_hist) == self.meas_hist.maxlen:
            return self.meas_hist[0]
        return np.zeros(self.nw)
