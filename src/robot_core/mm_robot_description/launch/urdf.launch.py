"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: For urdf testing
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  # pylint: disable=missing-function-docstring
    # Get current ros2 package path
    pkg_path = FindPackageShare("mm_robot_description")
    # Get urdf file path
    urdf = PathJoinSubstitution([pkg_path, "urdf", "robot.urdf.xacro"])
    # Get urdf launch file path
    urdf_tutorial_launch_file = PathJoinSubstitution(
        [FindPackageShare("urdf_tutorial"), "launch", "display.launch.py"])

    # Start urdf display node
    start_urdf_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_tutorial_launch_file),
        launch_arguments={"model": urdf}.items())

    return LaunchDescription([
        start_urdf_display
    ])
