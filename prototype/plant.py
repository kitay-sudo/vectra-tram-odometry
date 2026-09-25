"""Шим: каноническая модель объекта живёт в пакете ROS 2."""

import os
import sys

_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "ros2_ws", "src", "tram_state_estimator")
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from tram_state_estimator.plant import *  # noqa: F401,F403
from tram_state_estimator.plant import Params, P, Track, Plant, adhesion_mu, G  # noqa: F401
