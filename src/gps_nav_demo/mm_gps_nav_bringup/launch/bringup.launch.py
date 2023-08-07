"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch gps nav demo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  # pylint: disable=missing-function-docstring
    return LaunchDescription([
        *declare_launch_arguments(),
        launch_gps(),
    ])


def declare_launch_arguments() -> list:
    """Declare launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    ]


def launch_gps():
    """Launch gps controller"""
    pkg = FindPackageShare("mm_gps_nav_gps_controller")
    launch_file = PathJoinSubstitution([pkg, "launch", "gps.launch.py"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items()
    )
