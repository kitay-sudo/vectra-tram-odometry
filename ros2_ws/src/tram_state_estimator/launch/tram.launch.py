"""Оценщик по контракту ТЗ: входы /vehicle/*, выходы /result/*.

    ros2 launch tram_state_estimator tram.launch.py
    ros2 bag play <прогон>          # в другом терминале

Параметры — config/tram.yaml (лист вагона, откалиброванный по данным).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tram_state_estimator")
    return LaunchDescription([
        Node(
            package="tram_state_estimator",
            executable="tram_estimator",
            name="tram_state_estimator",
            parameters=[os.path.join(share, "config", "tram.yaml")],
            output="screen",
        ),
    ])
