"""Лист параметров вагона по данным.

Собирает значения, отличающиеся от заготовки Params, в файл калибровки. Из
него tools/gen_params.py делает полный ROS-лист (yaml).

Два листа по фиксированному разбиению tools/split.json:

    python3 analysis/calib_sheet.py eval   # только обучающие прогоны ->
        ros2_ws/src/tram_state_estimator/config/eval/tram_calibration.json
        (лист ОЦЕНКИ: по нему считаются все наши числа на holdout_scored)
    python3 analysis/calib_sheet.py jury   # все уникальные записи ->
        ros2_ws/src/tram_state_estimator/config/tram_calibration.json
        (лист ЖЮРИ: уходит в пакет)

Источники:
  * таблица привода, delay, tau - calib_drive.py (drive_model_<лист>.json);
  * масштаб показаний - отношение скорости тележек к скорости GNSS на
    установившемся движении;
  * шум измерения - разброс «тележки − GNSS» там же;
  * крип - регрессия относительной разницы «тележки / GNSS − 1» на силу
    привода (только для отчёта; решение о c_creep - по прогону фильтра на
    обучающих, calib_tune.py);
  * параметры фильтра, подобранные прогоном связки на подгоночных прогонах
    (q_v, крип, адаптация - calib_tune.py; выходная σ - calib_sigma.py), -
    analysis/calib_tuned_<лист>.json;
  * масштаб колёс каждого вагона (блок "vehicles": 30618, 30639 по записям
    своего вагона из тех же подгоночных) - calib_vehicle.py; нода выбирает
    его параметром vehicle (docs/VEHICLES.md).

Весь пересчёт по порядку - analysis/calib_all.sh.
"""

import json
import sys

import numpy as np

import bagio
import calib_drive
import calib_vehicle

KMH = 3.6
G = 9.81
PKG = bagio.ROOT / "ros2_ws" / "src" / "tram_state_estimator"
OUT = {"jury": PKG / "config" / "tram_calibration.json",
       "eval": PKG / "config" / "eval" / "tram_calibration.json"}


def gnss_ids(ids):
    return [b for b in ids if len(bagio.load(b)["mvel"]) >= 100]


def speed_stats(ids):
    ratios, resid = [], []
    for b in ids:
        a = bagio.load(b)
        f, r, g = a["front"], a["rear"], a["mvel"]
        if len(g) < 100 or len(f) < 100:
            continue
        tg, vg = g[:, 1], np.hypot(g[:, 2], g[:, 3])
        fi = np.interp(tg, f[:, 1], f[:, 2]) / KMH
        ri = np.interp(tg, r[:, 1], r[:, 2]) / KMH
        acc = np.gradient(vg, tg)
        agree = np.abs(fi - ri) < 0.2
        m = (vg > 3.0) & (np.abs(acc) < 0.1) & agree
        if m.sum() < 50:
            continue
        w = 0.5 * (fi + ri)
        ratios.append(w[m] / vg[m])
        resid.append(w[m] - vg[m])
    ratio = np.concatenate(ratios)
    return float(np.median(ratio)), np.concatenate(resid)


def core_u(tc, pos, t, delay, tau, dt=0.05):
    """Нормированная команда u(t) так же, как в ядре: позиция ручки (метки
    сообщений) с чистым запаздыванием delay и апериодическим звеном tau на
    сетке dt; u = позиция / 15."""
    o = np.argsort(tc, kind="stable")
    tc, pos = tc[o], pos[o]
    grid = np.arange(tc[0], tc[-1], dt)
    i = np.clip(np.searchsorted(tc, grid - delay, side="right") - 1, 0, len(tc) - 1)
    raw = np.where(grid - delay >= tc[0], pos[i], 0.0) / 15.0
    out = np.empty_like(raw)
    x, k = 0.0, min(dt / tau, 1.0) if tau > 0 else 1.0
    for j, r in enumerate(raw):
        x += (r - x) * k
        out[j] = x
    return np.interp(t, grid, out)


def creep_regression(ids, W, delay, tau, scale):
    """ξ = w·scale/v_GNSS − 1 против x = ΔA/g = (A(u,v) − A(0,v))/g по
    движению (v > 3 м/с, тележки согласны, без грубых выбросов эталона).
    Модель ядра: ξ = c_creep·x − c_creep_drag."""
    X, R, PH = [], [], []
    for b in ids:
        a = bagio.load(b)
        f, r, g, c = a["front"], a["rear"], a["mvel"], a["cmd"]
        if len(g) < 100 or len(f) < 100 or len(c) < 100:
            continue
        tg, vg = g[:, 1], np.hypot(g[:, 2], g[:, 3])
        fi = np.interp(tg, f[:, 1], f[:, 2]) / KMH
        ri = np.interp(tg, r[:, 1], r[:, 2]) / KMH
        w = 0.5 * (fi + ri) * scale
        m = (vg > 3.0) & (np.abs(fi - ri) < 0.2)
        rr = w / np.maximum(vg, 1e-3) - 1.0
        m &= np.abs(rr) < 0.03
        u = core_u(c[:, 1], c[:, 2], tg, delay, tau)
        dA = calib_drive.predict(W, 15.0 * u, vg) - calib_drive.predict(W, 0.0 * u, vg)
        X.append(dA[m] / G)
        R.append(rr[m])
        PH.append(np.sign(np.round(15.0 * u[m])))
    X, R, PH = map(np.concatenate, (X, R, PH))
    A = np.vstack([np.ones_like(X), X]).T
    coef = np.linalg.lstsq(A, R, rcond=None)[0]
    for _ in range(3):                         # грубые выбросы - вон
        e = R - A @ coef
        keep = np.abs(e) < 4.0 * 1.4826 * np.median(np.abs(e - np.median(e)))
        coef = np.linalg.lstsq(A[keep], R[keep], rcond=None)[0]
    by = {}
    for name, k in (("traction", 1), ("coast", 0), ("brake", -1)):
        mk = PH == k
        by[name] = dict(n=int(mk.sum()), xi_mean=float(np.mean(R[mk])),
                        x_mean=float(np.mean(X[mk])))
    return dict(c_creep=float(coef[1]), offset=float(coef[0]), n=int(len(X)),
                n_kept=int(keep.sum()), by_phase=by)


def load_tuned(which):
    f = bagio.CACHE.parent / f"calib_tuned_{which}.json"
    if not f.exists():
        return {}, None
    d = json.loads(f.read_text(encoding="utf-8"))
    return d.get("params", {}), d


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "eval"
    ids = calib_drive.fit_ids(which)
    dm = json.loads((bagio.CACHE.parent / f"drive_model_{which}.json").read_text(encoding="utf-8"))
    assert dm["fit_ids"] == ids, "таблица привода подогнана по другим прогонам"
    U = np.array(dm["u_grid"]) / 15.0
    V = np.array(dm["v_grid"])
    T = np.array(dm["table"])
    # знак: тормозные позиции не разгоняют, тяговые не тормозят сильнее
    # выбега (в редких клетках без данных подгонка давала обратный знак)
    coast = T[U == 0][0]
    for i, u in enumerate(U):
        if u < 0:
            T[i] = np.minimum(T[i], np.minimum(coast, 0.0))
        elif u > 0:
            T[i] = np.maximum(T[i], coast)

    gids = gnss_ids(ids)
    ratio, resid = speed_stats(gids)
    scale = 1.0 / ratio
    sig = float(1.4826 * np.median(np.abs(resid - np.median(resid))))
    print(f"лист {which}: {len(ids)} прогонов, с GNSS {len(gids)}")
    print(f"масштаб показаний {scale:.5f} (тележки/GNSS = {ratio:.5f}), "
          f"σ «тележки − GNSS» (MAD) {sig:.3f} м/с")
    cr = creep_regression(gids, T.ravel(), dm["delay"], dm["tau"], scale)
    print(f"крип по данным: ξ = {cr['c_creep']:+.4f}·ΔA/g {cr['offset']:+.5f} "
          f"({cr['n_kept']} из {cr['n']} отсчётов); по фазам ручки "
          + ", ".join(f"{k}: ξ ср. {v['xi_mean']:+.5f} при ΔA/g {v['x_mean']:+.4f}"
                      for k, v in cr["by_phase"].items()))

    over = {
        # две тележки, по одному датчику скорости на каждой, скорость в км/ч
        "n_axles": 2,
        "driven": [True, True],
        "braked": [True, True],
        "sensor_axles": [True, True],
        "sensors_per_axle": 1,
        "meas_units": "km_h",
        "meas_scale": round(scale, 6),
        # показание тележки - уже скорость пути, разницы бортов нет
        "curve_ratio_max": 0.0,
        "sigma_meas": round(max(sig, 0.05), 3),
        # 15 позиций тяги и 15 торможения; u = позиция / 15
        "notch_tract": [round(k / 15.0, 6) for k in range(1, 16)],
        "notch_brake": [round(k / 15.0, 6) for k in range(1, 16)],
        "acc_u": [round(float(x), 6) for x in U],
        "acc_v": [float(x) for x in V],
        "acc_table": [round(float(x), 4) for x in T.ravel()],
        # сопротивление входит в таблицу (строка выбега)
        "res_A": 0.0, "res_B": 0.0, "res_C": 0.0,
        "delay_drive": float(dm["delay"]),
        "tau_drive": float(dm["tau"]),
        # датчики ~9,4 Гц: 20 отсчётов ≈ 2 с неизменных показаний
        "stuck_n": 20,
        # цикл ядра 20 Гц по меткам сообщений
        "dt": 0.05,
    }
    tuned, tmeta = load_tuned(which)
    over.update(tuned)
    meta = {
        "_source": f"analysis/calib_sheet.py {which}: tools/split.json, "
                   + ("только обучающие уникальные записи (train_unique)" if which == "eval"
                      else "все уникальные записи (train_unique + holdout)"),
        "_fit_split": "train_unique" if which == "eval" else "train_unique + holdout",
        "_fit_runs": len(ids),
        "_fit_runs_gnss": len(gids),
        "_fit_ids": ids,
        "_drive_rmse_val": dm["rmse_val"],
        "_speed_ratio_bogie_to_gnss": ratio,
        "_creep_regression": cr,
        "_tuned_from": (f"analysis/calib_tuned_{which}.json" if tmeta else None),
        "_tuned_notes": (tmeta or {}).get("notes"),
    }
    vehicles = calib_vehicle.vehicle_block(which)
    assert abs(vehicles["_mixed_meas_scale"] - over["meas_scale"]) < 2e-6,         "масштаб вагонов посчитан не тем способом, что общий"
    print("масштаб колёс по вагонам: " + ", ".join(
        f"{v} {vehicles[v]['meas_scale']:.6f}" for v in calib_vehicle.VEHICLES))
    out = OUT[which]
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({**meta, "params": over, "vehicles": vehicles},
                              ensure_ascii=False, indent=1), encoding="utf-8")
    print("записано:", out)


if __name__ == "__main__":
    main()
