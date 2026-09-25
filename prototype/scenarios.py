"""Сценарии и измерение точности оценки скорости и положения.

Задача просит оценку скорости и положения — их и меряем.
"""

import numpy as np
from plant import Plant, Track, P
from estimator import Estimator, EP, MODE_NAMES, STANDSTILL

DT = 0.001
SUB = int(round(EP.dt / DT))


def run(track, driver, T_end, faults=None, seed=0, sand=lambda t: False,
        tbrake=lambda t: False):
    pl = Plant(track, dt=DT, seed=seed)
    es = Estimator()
    if faults:
        pl.faults = faults

    keys = ("t", "v", "vhat", "s", "shat", "mode", "sig_v", "k_t", "k_b", "d",
            "amb")
    log = {k: [] for k in keys}
    for k in range(int(T_end / DT)):
        t = k * DT
        notch = driver(t, pl.v)
        pl.step(notch, sand=sand(t), track_brake=tbrake(t))
        m = pl.measure()
        if k % SUB:
            continue
        o = es.step(notch, m)
        log["t"].append(t)
        log["v"].append(pl.v)
        log["vhat"].append(o["v"])
        log["s"].append(pl.s)
        log["shat"].append(o["s"])
        log["mode"].append(o["mode"])
        log["sig_v"].append(o["sigma_v"])
        log["k_t"].append(o["k_t"])
        log["k_b"].append(o["k_b"])
        log["d"].append(o["d"])
        log["amb"].append(o["ambiguous"])
    return {k: np.array(v) for k, v in log.items()}


def check(lg):
    e = lg["vhat"] - lg["v"]
    moving = np.abs(lg["v"]) > 0.5
    em = e[moving] if moving.any() else e
    return dict(
        v_mae=float(np.mean(np.abs(e))),
        v_p95=float(np.percentile(np.abs(e), 95)),
        v_max=float(np.max(np.abs(e))),
        v_bias=float(np.mean(em)),
        pos_err=float(lg["shat"][-1] - lg["s"][-1]),
        pos_rel=float(abs(lg["shat"][-1] - lg["s"][-1])
                      / max(abs(lg["s"][-1]), 1.0)),
        sig_med=float(np.median(lg["sig_v"])),
        k_t=float(lg["k_t"][-1]),
        false_ss=int(((lg["mode"] == STANDSTILL) & moving).sum()),
        amb=int(lg["amb"].sum()),
    )


# ---------------- профили линии ----------------

flat = Track()
downhill = Track(grade=lambda s: -0.06)


def radius_spiral(s):
    """Кривая R=20 с переходными участками по 10 м."""
    k_max = 1.0 / 20.0
    if s < 50 or s > 130:
        return np.inf
    if s < 60:
        k = k_max * (s - 50) / 10.0
    elif s < 120:
        k = k_max
    else:
        k = k_max * (130 - s) / 10.0
    return np.inf if k < 1e-6 else 1.0 / k


curve_spiral = Track(radius=radius_spiral)


def mu_ice_after(t0, mu_lo=0.03, mu_hi=0.25):
    return lambda s, t: mu_lo if t > t0 else mu_hi


# ---------------- профили водителя ----------------

def accel_cruise_brake(t, v):
    if t < 20:
        return 4
    if t < 35:
        return 0
    return -3


def hard_brake_from_speed(t, v):
    return 4 if t < 18 else -4


def chatter(t, v):
    if t < 12:
        return 4
    return 4 if int((t - 12) / 0.7) % 2 == 0 else -3


def moderate_accel(t, v):
    if t < 25:
        return 2
    if t < 38:
        return 0
    return -2


def stop_and_go(t, v):
    c = t % 45
    if c < 18:
        return 4
    if c < 28:
        return 0
    return -3


SCENARIOS = [
    ("сухо, прямая", dict(track=flat, driver=accel_cruise_brake, T_end=55)),
    ("дождь, умеренная тяга", dict(track=Track(mu=lambda s, t: 0.12),
                                   driver=moderate_accel, T_end=55)),
    ("дождь, буксование", dict(track=Track(mu=lambda s, t: 0.12),
                               driver=accel_cruise_brake, T_end=55)),
    ("наледь при торможении", dict(track=Track(mu=mu_ice_after(18)),
                                   driver=hard_brake_from_speed, T_end=60)),
    ("наледь при разгоне", dict(track=Track(mu=lambda s, t: 0.03),
                                driver=accel_cruise_brake, T_end=55)),
    ("заклинивание всех колёс", dict(track=Track(mu=mu_ice_after(18, 0.02)),
                                     driver=hard_brake_from_speed, T_end=70)),
    ("рельсовый тормоз", dict(track=flat, driver=hard_brake_from_speed,
                              T_end=40, tbrake=lambda t: t > 18)),
    ("дребезг тяга/тормоз", dict(track=flat, driver=chatter, T_end=45)),
    ("кривая R=20 с переходной", dict(track=curve_spiral,
                                      driver=moderate_accel, T_end=55)),
    ("обледенелый спуск", dict(track=Track(grade=lambda s: -0.06,
                                           mu=mu_ice_after(18, 0.02)),
                               driver=hard_brake_from_speed, T_end=70)),
    ("подъём 3%", dict(track=Track(grade=lambda s: 0.03),
                       driver=accel_cruise_brake, T_end=55)),
    ("отказ одного датчика", dict(track=flat, driver=accel_cruise_brake,
                                  T_end=55, faults={3: (10.0, "zero")})),
    ("залипание датчика", dict(track=flat, driver=accel_cruise_brake,
                               T_end=55, faults={5: (12.0, "stuck")})),
    ("отказ всех датчиков", dict(track=flat, driver=accel_cruise_brake,
                                 T_end=55,
                                 faults={i: (25.0, "zero") for i in range(8)})),
    ("маршрут: 3 перегона", dict(track=flat, driver=stop_and_go, T_end=135)),
]


def main():
    rows = []
    for name, kw in SCENARIOS:
        sub = [check(run(seed=s, **kw)) for s in range(3)]
        rows.append((name, sub))

    h = (f"{'сценарий':<26}{'|ош|,м/с':>9}{'p95':>8}{'макс':>8}"
         f"{'смещ.':>8}{'ош.пути,м':>11}{'отн.%':>8}{'k_t':>7}{'лож.ст.':>9}{'неодн.':>8}")
    print("ТОЧНОСТЬ ОЦЕНКИ СКОРОСТИ И ПОЛОЖЕНИЯ")
    print(h)
    print("-" * len(h))
    for name, sub in rows:
        print(f"{name:<26}{np.mean([s['v_mae'] for s in sub]):>9.3f}"
              f"{np.mean([s['v_p95'] for s in sub]):>8.2f}"
              f"{max(s['v_max'] for s in sub):>8.2f}"
              f"{np.mean([s['v_bias'] for s in sub]):>8.2f}"
              f"{np.mean([s['pos_err'] for s in sub]):>11.2f}"
              f"{100 * np.mean([s['pos_rel'] for s in sub]):>8.2f}"
              f"{np.mean([s['k_t'] for s in sub]):>7.2f}"
              f"{sum(s['false_ss'] for s in sub):>9}"
              f"{sum(s['amb'] for s in sub):>8}")
    print("-" * len(h))
    all_ = [s for _, sub in rows for s in sub]
    print(f"Средняя |ошибка| скорости по всем прогонам: "
          f"медиана {np.median([s['v_mae'] for s in all_]):.3f} м/с, "
          f"худший {max(s['v_mae'] for s in all_):.3f} м/с")
    print(f"Относительная ошибка пути: "
          f"медиана {100 * np.median([s['pos_rel'] for s in all_]):.2f} %, "
          f"худшая {100 * max(s['pos_rel'] for s in all_):.2f} %")
    print(f"Ложных объявлений стоянки: {sum(s['false_ss'] for s in all_)}")


if __name__ == "__main__":
    main()
