"""Ветки за тупиком у конечной (поток loops, 27.09): курсор стоит в тупике
платформы, пока вагон не уехал дальше BR_MIN м; тогда — на ветку этого
тупика: разворотное кольцо (стоял у платформы) или боковой путь (не
стоял). docs/POSITION_FRAME.md, «Конечные и разворотные кольца». ROS не
требуется.

Синтетика в UTM зоны 37 (как test_position.py): подход на восток и
платформа — облако точек до тупика (300 м от начала, там известная
конечная); путь отправления — облако на запад, 50 м севернее. Ветки (в
облаке их нет, как у западной конечной на обучающих прогонах):
  SW   — стрелка на 220 м перед платформой, направо (на юго-восток), 150 м;
         offset 80 м; по ней ходили, не останавливаясь у платформы;
  LOOP — продолжение от тупика: кольцо налево (R = 25 м, 180°) и 60 м на
         запад по пути отправления; offset 0; перед ним стояли у платформы.
"""

import copy
import math
import os
import sys
import tempfile

import numpy as np
import pytest

from tram_state_estimator import geodesy as g
from tram_state_estimator.runner import Runner
from tram_state_estimator.track_map import TrackMap

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_position as tp                                          # noqa: E402

E0, N0, ZONE, ALT = 399800.0, 6185000.0, 37, 150.0
DE = 300.0            # м от начала: тупик платформы
FORK = 220.0          # м от начала: стрелка бокового пути
R_LOOP = 25.0
EAST = math.pi / 2
# курсор встаёт в тупике не в последней точке облака, а на snap_r − 0,5 м
# дальше (там точки облака ещё в радиусе притяжения); offset веток в карте
# считает тот же курсор (analysis/build_map.py follow) — здесь так же
OVER = 2.5


def _line(p0, h, L, step=1.0):
    d = np.arange(0.0, L + 1e-9, step)
    return np.c_[p0[0] + d * math.sin(h), p0[1] + d * math.cos(h)]


def _arc(p0, h0, R, ang, left=True, step=1.0):
    sgn = -1.0 if left else 1.0
    cx = p0[0] + R * math.sin(h0 + sgn * math.pi / 2)
    cy = p0[1] + R * math.cos(h0 + sgn * math.pi / 2)
    a0 = math.atan2(p0[0] - cx, p0[1] - cy)
    n = max(2, int(R * ang / step) + 1)
    a = a0 + sgn * np.linspace(0.0, ang, n)
    return np.c_[cx + R * np.sin(a), cy + R * np.cos(a)]


def _cat(*parts):
    out = [parts[0]]
    for q in parts[1:]:
        out.append(q[1:] if np.allclose(q[0], out[-1][-1], atol=1e-6) else q)
    return np.vstack(out)


def _s(P):
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]


def _geometry():
    """Ломаные (E, N): платформа, путь отправления, ветки SW и LOOP (от их
    начала)."""
    plat = _line((E0, N0), EAST, DE)
    fork = plat[int(FORK)]
    sw = _cat(_arc(fork, EAST, 60.0, math.pi / 4, left=False),
              _line(_arc(fork, EAST, 60.0, math.pi / 4, left=False)[-1], EAST + math.pi / 4,
                    150.0 - 60.0 * math.pi / 4))
    loop_arc = _arc(plat[-1], EAST, R_LOOP, math.pi, left=True)
    loop = _cat(loop_arc, _line(loop_arc[-1], -EAST, 60.0))
    dep = _line((E0 + DE, N0 + 2 * R_LOOP), -EAST, 250.0)            # на запад
    return plat, dep, sw, loop


def _true_heads(P):
    la, lo = (np.asarray(x, float) for x in g.utm_inv(P[:, 0], P[:, 1], ZONE))
    hg = np.arctan2(np.gradient(P[:, 0]), np.gradient(P[:, 1]))
    fr = g.Frame(float(la[0]), float(lo[0]), 0.0, "utm")
    conv = fr.scale_heading(la, lo, np.zeros(len(la)))[1]
    return la, lo, np.angle(np.exp(1j * (hg - conv)))


def _map(branches=True, n=(1, 1), ns=(0, 1), win=90.0, sw_terminal=True):
    plat, dep, sw, loop = _geometry()
    tm = TrackMap.from_polylines([np.c_[plat, np.full(len(plat), ALT)],
                                  np.c_[dep, np.full(len(dep), ALT)]],
                                 crs="utm", zone=ZONE, bidirectional=False)
    tm.scale_frame = "utm"
    ends = [plat[-1]] + ([sw[-1]] if sw_terminal else [])
    pe, pn = np.array([p[0] for p in ends]), np.array([p[1] for p in ends])
    la, lo = (np.asarray(x, float) for x in g.utm_inv(pe, pn, ZONE))
    tm.terminals = np.c_[la, lo]
    if not branches:
        return tm
    bp, bi = [], []
    dla, dlo, dh = _true_heads(plat[-3:])
    for k, (P, off) in enumerate(((sw, DE - FORK + OVER), (loop, OVER))):
        la, lo, ht = _true_heads(P)
        bp.append(np.c_[np.full(len(P), k), _s(P), la, lo, np.full(len(P), ALT), ht])
        bi.append((dla[-1], dlo[-1], dh[-1], off, win, n[k], ns[k]))
    return tm.set_branches(np.vstack(bp), np.array(bi, float))


def _cursor(tm):
    la, lo = (float(np.asarray(x)[0]) for x in g.utm_inv(np.array([E0]), np.array([N0]), ZONE))
    fr = g.Frame(la, lo, ALT, "utm")
    tm.bind(fr)
    c = tm.locate((0.0, 0.0, ALT), fr.scale_heading(la, lo, EAST)[1])
    return c, fr


def _go(tm, c, dist, step=0.5):
    n = int(round(abs(dist) / step))
    for _ in range(n):
        tm.advance(c, math.copysign(step, dist))
    return c


def _xy(c, fr):
    return c["x"] + fr.E0, c["y"] + fr.N0


def _at(P, s):
    S = _s(P)
    return float(np.interp(s, S, P[:, 0])), float(np.interp(s, S, P[:, 1]))


def _d(c, fr, p):
    e, n = _xy(c, fr)
    return math.hypot(e - p[0], n - p[1])


# ================================================================ курсор


def test_short_overshoot_holds_as_before():
    """За тупиком меньше BR_MIN — вагон у платформы: курсор стоит в тупике."""
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE + TrackMap.BR_MIN - 2.0)
    assert "br" not in c and c["hold"] > 0.0
    assert _d(c, fr, plat[-1]) < OVER + 0.1


def test_no_stop_goes_to_side_branch():
    """Проехал платформу без стоянки и уехал за тупик — боковой путь, на
    пути от стрелки (offset + уехал за тупик)."""
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE + 40.0)
    assert c["br"]["k"] == 0
    assert _d(c, fr, _at(sw, DE - FORK + 40.0)) < 1.0
    assert abs(math.remainder(c["h"] - (EAST + math.pi / 4), 2 * math.pi)) < math.radians(10)


def test_switch_moment_is_br_min():
    tm = _map()
    c, fr = _cursor(tm)
    _go(tm, c, DE + OVER + TrackMap.BR_MIN - 1.0)
    assert "br" not in c
    _go(tm, c, 2.0)
    assert "br" in c


def test_platform_stop_goes_to_loop():
    """Стоял у платформы, потом уехал за тупик — разворотное кольцо: курсор
    идёт по кольцу, курс разворачивается."""
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE - 20.0)
    tm.anchor(c, DE - 20.0)                          # стоянка 8 с (Position.on_stop)
    _go(tm, c, 20.0 + 60.0)
    assert c["br"]["k"] == 1
    assert _d(c, fr, _at(loop, 60.0)) < 1.0
    assert abs(math.remainder(c["h"] - EAST, 2 * math.pi)) > math.radians(90)


def test_stop_at_dead_end_counts():
    """Недолёт до платформы по колёсам: курсор уже в тупике, вагон
    остановился (высадка), потом уехал — кольцо."""
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE + 6.0)
    assert c["hold"] > 0.0
    tm.anchor(c, DE + 6.0)
    _go(tm, c, 40.0)
    assert c["br"]["k"] == 1
    assert _d(c, fr, _at(loop, 46.0)) < 1.0


def test_stop_before_window_does_not_count():
    """Стоянка раньше окна (остановка за 120 м до тупика, окно 90 м) — не
    у платформы: боковой путь."""
    tm = _map()
    c, fr = _cursor(tm)
    _go(tm, c, DE - 120.0)
    tm.anchor(c, DE - 120.0)
    _go(tm, c, 120.0 + 40.0)
    assert c["br"]["k"] == 0


def test_frequency_decides_when_evidence_equal():
    """Стоянка одинаково вероятна на обеих ветках — выбор по числу проходов."""
    tm = _map(n=(1, 6), ns=(1, 6))
    c, fr = _cursor(tm)
    _go(tm, c, DE - 10.0)
    tm.anchor(c, 10.0)
    _go(tm, c, 40.0)
    assert c["br"]["k"] == 1
    tm = _map(n=(6, 1), ns=(6, 1))
    c, fr = _cursor(tm)
    _go(tm, c, DE - 10.0)
    tm.anchor(c, 10.0)
    _go(tm, c, 40.0)
    assert c["br"]["k"] == 0


def test_side_branch_end_is_dead_end_at_terminal():
    """За концом оси бокового пути облака нет, рядом конечная — курсор стоит
    в конце оси."""
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, FORK + _s(sw)[-1] + 30.0)
    assert "br" not in c and c["hold"] > 0.0
    assert _d(c, fr, sw[-1]) < 1.0


def test_loop_end_back_to_cloud():
    """Кольцо кончается на пути отправления (он в облаке): курсор снова на
    облаке и идёт по нему на запад."""
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE - 5.0)
    tm.anchor(c, DE - 5.0)
    _go(tm, c, 5.0 + _s(loop)[-1] + 50.0)
    assert "br" not in c and c["on_map"] and c.get("hold", 0.0) == 0.0
    e, n = _xy(c, fr)
    assert abs(n - (N0 + 2 * R_LOOP)) < 1.0
    assert e == pytest.approx(E0 + DE - 60.0 - 50.0, abs=2.0)
    assert abs(math.remainder(c["h"] + EAST, 2 * math.pi)) < math.radians(10)


def test_reverse_past_branch_start_back_to_cloud():
    tm = _map()
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE + 40.0)
    assert c["br"]["k"] == 0
    _go(tm, c, -(DE - FORK + 40.0) - 10.0)
    assert "br" not in c and c["on_map"]
    assert _d(c, fr, plat[int(FORK) - 10]) < 1.5


def test_map_without_branches_holds_to_the_end():
    """Карта без веток (как до 27.09): курсор стоит в тупике сколько угодно;
    состояние курсора — как прежде (ни пути курсора, ни стоянок)."""
    tm = _map(branches=False)
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE + 100.0)
    tm.anchor(c, 100.0)
    assert "br" not in c and _d(c, fr, plat[-1]) < OVER + 0.1
    assert "odo" not in c and "stops" not in c


def test_branches_off_switch():
    tm = _map()
    tm.branches_on = False
    plat, dep, sw, loop = _geometry()
    c, fr = _cursor(tm)
    _go(tm, c, DE + 60.0)
    assert "br" not in c and _d(c, fr, plat[-1]) < OVER + 0.1


def test_save_load_roundtrip():
    tm = _map()
    plat, dep, sw, loop = _geometry()
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, "m.npz")
        tm.save(f)
        tm2 = TrackMap.load(f)
        tm_old = _map(branches=False)
        tm_old.save(os.path.join(d, "old.npz"))
        assert "bp" not in np.load(os.path.join(d, "old.npz")).files
        assert len(TrackMap.load(os.path.join(d, "old.npz")).bi) == 0
    for k in ("bp", "bi"):
        assert np.array_equal(getattr(tm, k), getattr(tm2, k))
    c, fr = _cursor(tm2)
    _go(tm2, c, DE + 40.0)
    assert _d(c, fr, _at(sw, DE - FORK + 40.0)) < 1.0


def test_cursor_copy_is_deterministic():
    """Состояние ветки и стоянок копируется (Runner.fork — deepcopy)."""
    tm = _map()
    c, fr = _cursor(tm)
    _go(tm, c, DE - 10.0)
    tm.anchor(c, 10.0)
    c2 = copy.deepcopy(c)
    _go(tm, c, 60.0)
    _go(tm, c2, 60.0)
    assert (c["x"], c["y"], c["br"]) == (c2["x"], c2["y"], c2["br"])


# ================================================================ Runner целиком


class _SideRoute(tp.Route):
    """Истинный путь base_link: платформа до стрелки, потом боковой путь."""

    def __init__(self):
        plat, dep, sw, loop = _geometry()
        P = _cat(plat[:int(FORK) + 1], sw)
        self.P, self.s = P, _s(P)


def test_runner_follows_side_branch():
    """Нода целиком (Runner): выставка по GNSS в первые секунды, дальше только
    колёса; вагон без остановок уходит на боковой путь — в конце выход на нём,
    а не в тупике платформы (там было бы ~60 м)."""
    route = _SideRoute()
    v, acc, t0 = 8.0, 1.0, 3.0                       # разгон 1 м/с² до 8 м/с
    t_acc = v / acc

    def v_of_t(t):
        return min(v, max(0.0, t - t0) * acc)

    def s_of_t(t):
        x = max(0.0, t - t0)
        return 0.5 * acc * x * x if x < t_acc else 0.5 * v * t_acc + v * (x - t_acc)

    # base_link в 110 м от стрелки
    t_end = t0 + t_acc + (FORK + 110.0 - tp.BL - 0.5 * v * t_acc) / v
    res = {}
    for name, tm in (("branches", _map()), ("old", _map(branches=False))):
        r = Runner(tp._tram(), track_map=tm)
        o = [q for q in tp._feed(r, 0.0, t_end, v_of_t, route, s_of_t) if q["pos_valid"]][-1]
        E, N = tp._continuous(o, r.pos.frame)
        e, n = route.bl(s_of_t(o["stamp"]))
        res[name] = math.hypot(E - e, N - n)
    assert res["branches"] < 2.0
    assert res["old"] > 40.0
