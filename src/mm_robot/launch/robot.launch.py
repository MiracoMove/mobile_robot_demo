"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch real robot
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():  # pylint: disable=missing-function-docstring
    return LaunchDescription([
        launch_tarkbot()
    ])


def launch_tarkbot():
    """Launch tarkbot"""
    return Node(
        package="mm_robot",
        executable="tarkbot",
        output="screen",
    )