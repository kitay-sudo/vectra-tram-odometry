"""Оценщик по контракту ТЗ: входы /vehicle/*, выходы /result/*.

    ros2 launch tram_state_estimator tram.launch.py
    ros2 bag play <прогон>          # в другом терминале
    ros2 launch tram_state_estimator tram.launch.py params_file:=/путь/лист.yaml
    ros2 launch tram_state_estimator tram.launch.py vehicle:=30639

Параметры — config/tram.yaml (лист вагона, откалиброванный по данным).
vehicle — вагон (30618 | 30639 | auto, docs/VEHICLES.md); пусто (по
умолчанию) — как в листе (30618).
Нода перезапускается при падении (respawn): последний рубеж устойчивости,
битые входы и разрывы времени отсекаются в самой ноде.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _estimator(context):
    params = [LaunchConfiguration("params_file").perform(context)]
    vehicle = LaunchConfiguration("vehicle").perform(context).strip()
    if vehicle:
        params.append({"vehicle": vehicle})
    return [Node(
        package="tram_state_estimator",
        executable="tram_estimator",
        name="tram_state_estimator",
        parameters=params,
        output="screen",
        respawn=True,
        respawn_delay=1.0,
    )]


def generate_launch_description():
    share = get_package_share_directory("tram_state_estimator")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "tram.yaml"),
            description="лист параметров вагона (yaml ROS 2)"),
        DeclareLaunchArgument(
            "vehicle", default_value="",
            description="вагон: 30618 | 30639 | auto; пусто — как в листе"),
        OpaqueFunction(function=_estimator),
    ])
