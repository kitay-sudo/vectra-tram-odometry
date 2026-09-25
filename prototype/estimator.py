"""Шим: каноническое ядро живёт в пакете ROS 2.

Единственный источник истины — ros2_ws/src/tram_state_estimator/.
Здесь только переадресация, чтобы стенд работал без сборки ROS.
"""

import os
import sys

_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "ros2_ws", "src", "tram_state_estimator")
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from tram_state_estimator.estimator_core import *  # noqa: F401,F403,E402
from tram_state_estimator.estimator_core import (  # noqa: F401,E402
    Params, DEFAULT, EP, Estimator, G, NX, IS, IV, ID, IKT, IKB,
    COAST, TRACTION, BRAKE, TRANSITION, SLIP, STANDSTILL, DEGRADED, MODE_NAMES,
)
