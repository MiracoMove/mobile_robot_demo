"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Start robot state publisher with simulated environment
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  # pylint: disable=missing-function-docstring
    return LaunchDescription([
        launch_robot_state_publisher(),
    ])


def launch_robot_state_publisher():
    """Launch robot state publisher"""
    # Get current ros2 package path
    pkg_path = FindPackageShare("mm_robot_description")
    # Get urdf file path
    urdf = PathJoinSubstitution([pkg_path, "urdf", "robot.urdf.xacro"])
    # Generate robot description
    robot_description = Command(["xacro ", urdf])

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ]
    )
