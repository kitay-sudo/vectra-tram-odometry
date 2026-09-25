"""Шаг 2в: лист параметров вагона по данным.

Собирает значения, отличающиеся от заготовки Params, в
ros2_ws/src/tram_state_estimator/config/tram_calibration.json. Из него
tools/gen_params.py делает полный лист config/tram.yaml.

Источники:
  * таблица привода, delay, tau — calib_drive.py (drive_model.json);
  * масштаб показаний — отношение скорости тележек к скорости GNSS на
    установившемся движении, по обучающим прогонам;
  * шум измерения — разброс «тележки − GNSS» там же.
"""

import json

import numpy as np

import bagio

KMH = 3.6
PKG = bagio.ROOT / "ros2_ws" / "src" / "tram_state_estimator"


def speed_stats(ids):
    ratios, resid, dist = [], [], []
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
        dist.append(np.full(m.sum(), 1.0))
        resid.append(w[m] - vg[m])
    ratio = np.concatenate(ratios)
    return float(np.median(ratio)), np.concatenate(resid)


def main():
    dm = json.loads((bagio.CACHE.parent / "drive_model.json").read_text(encoding="utf-8"))
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

    ratio, resid = speed_stats(dm["train"])
    scale = 1.0 / ratio
    sig = float(1.4826 * np.median(np.abs(resid - np.median(resid))))
    print(f"масштаб показаний {scale:.5f} (тележки/GNSS = {ratio:.5f}), "
          f"σ «тележки − GNSS» (MAD) {sig:.3f} м/с")

    over = {
        # две тележки, по одному датчику скорости на каждой, скорость в км/ч
        "n_axles": 2,
        "driven": [True, True],
        "braked": [True, True],
        "sensor_axles": [True, True],
        "sensors_per_axle": 1,
        "meas_units": "km_h",
        "meas_scale": round(scale, 6),
        # показание тележки — уже скорость пути, разницы бортов нет
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
        # колёса точнее модели: шум процесса по скорости выше заготовки
        # (подобран на отложенных прогонах: |ош| 0,0606 -> 0,0575 м/с)
        "q_v": 0.3,
    }
    meta = {
        "_source": "analysis/calib_sheet.py по прогонам dataset",
        "_train_runs": len(dm["train"]),
        "_drive_rmse_val": dm["rmse_val"],
        "_speed_ratio_bogie_to_gnss": ratio,
    }
    out = PKG / "config" / "tram_calibration.json"
    out.write_text(json.dumps({**meta, "params": over}, ensure_ascii=False, indent=1),
                   encoding="utf-8")
    print("записано:", out)


if __name__ == "__main__":
    main()
