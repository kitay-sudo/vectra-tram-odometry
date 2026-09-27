"""Детерминизм: два прогона одной и той же фикстуры дают побайтно
одинаковые выходы (sha256), в том числе в отдельном процессе с другим
PYTHONHASHSEED - нет зависимости от глобального состояния, порядка словарей
и множеств или случайных чисел.

Проверяется офлайн-связка Runner. В ROS побитового совпадения нет: начало
сетки зависит от того, какое сообщение нода обработает первым.
"""

import os
import subprocess
import sys

import e2e_replay as E

_CHILD = r"""
import sys
sys.path[:0] = [{test!r}, {pkg!r}]
import e2e_replay as E
print(E.digest(E.replay(E.load_fixture(), use_map=True, gnss="window")))
"""


def test_two_runs_identical_sha256():
    fx = E.load_fixture()
    a = E.digest(E.replay(fx, use_map=True, gnss="window"))
    b = E.digest(E.replay(E.load_fixture(), use_map=True, gnss="window"))
    assert a == b


def test_separate_process_identical_sha256():
    fx = E.load_fixture()
    a = E.digest(E.replay(fx, use_map=True, gnss="window"))
    env = dict(os.environ, PYTHONHASHSEED="12345")
    code = _CHILD.format(test=E.HERE, pkg=E.PKG)
    out = subprocess.run([sys.executable, "-c", code], env=env, check=True,
                         capture_output=True, text=True, timeout=300)
    assert out.stdout.strip().splitlines()[-1] == a


def test_map_free_is_deterministic():
    fx = E.load_fixture()
    a = E.digest(E.replay(fx, use_map=False, gnss="window"))
    b = E.digest(E.replay(fx, use_map=False, gnss="window"))
    assert a == b
