"""Оценщик по контракту ТЗ: входы /vehicle/*, выходы /result/*.

    ros2 launch tram_state_estimator tram.launch.py
    ros2 bag play <прогон>          # в другом терминале
    ros2 launch tram_state_estimator tram.launch.py params_file:=/путь/лист.yaml

Параметры — config/tram.yaml (лист вагона, откалиброванный по данным).
Нода перезапускается при падении (respawn): последний рубеж устойчивости,
битые входы и разрывы времени отсекаются в самой ноде.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tram_state_estimator")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "tram.yaml"),
            description="лист параметров вагона (yaml ROS 2)"),
        Node(
            package="tram_state_estimator",
            executable="tram_estimator",
            name="tram_state_estimator",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
