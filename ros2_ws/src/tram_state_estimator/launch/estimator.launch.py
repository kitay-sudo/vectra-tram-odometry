"""Запуск оценщика. По умолчанию вместе с имитатором.

    ros2 launch tram_state_estimator estimator.launch.py
    ros2 launch tram_state_estimator estimator.launch.py simulator:=false
    ros2 launch tram_state_estimator estimator.launch.py mu:=0.03 grade:=-0.06
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tram_state_estimator")
    params = os.path.join(share, "config", "params.yaml")

    args = [
        DeclareLaunchArgument("simulator", default_value="true",
                              description="поднимать имитатор трамвая"),
        DeclareLaunchArgument("mu", default_value="0.25",
                              description="коэффициент сцепления рельса"),
        DeclareLaunchArgument("grade", default_value="0.0",
                              description="уклон пути, рад"),
    ]

    estimator = Node(
        package="tram_state_estimator",
        executable="estimator_node",
        name="tram_state_estimator",
        parameters=[params],
        output="screen",
    )

    simulator = Node(
        package="tram_state_estimator",
        executable="simulator_node",
        name="tram_simulator",
        parameters=[params, {
            "mu": LaunchConfiguration("mu"),
            "grade": LaunchConfiguration("grade"),
        }],
        condition=IfCondition(LaunchConfiguration("simulator")),
        output="screen",
    )

    return LaunchDescription(args + [estimator, simulator])
