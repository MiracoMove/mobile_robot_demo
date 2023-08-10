"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Bringup file for mm_robot
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    ~~~
    """
    return LaunchDescription([
        *declare_launch_arguments(),
        launch_robot_state_publisher(),
        launch_ros2_control(),
        launch_demo(),
    ])


def declare_launch_arguments() -> list:
    """Declare launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "demo_bringup_pkg",
            default_value="",
        )
    ]


def launch_robot_state_publisher():
    """Launch robot state publisher"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_pkg_path = FindPackageShare("mm_robot_description")
    urdf_file = PathJoinSubstitution([robot_pkg_path, "urdf", "robot.urdf.xacro"])
    rsp_launch_file = PathJoinSubstitution([robot_pkg_path, "launch", "rsp.launch.py"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_file),
        launch_arguments={"use_sim_time": use_sim_time, "urdf": urdf_file}.items(),
        condition=UnlessCondition(use_sim_time)
    )


def launch_ros2_control() -> list:
    """Launch ros2_control"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    demo_bringup_pkg = FindPackageShare(LaunchConfiguration("demo_bringup_pkg", default=""))
    bringup_pkg = FindPackageShare("mm_robot_bringup")
    config_file = PathJoinSubstitution([demo_bringup_pkg, "configs", "controllers.yaml"])
    ros2_control_launch_file = PathJoinSubstitution(
        [bringup_pkg, "launch", "ros2_control.launch.py"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": config_file
        }.items()
    )


def launch_demo():
    """Launch demo"""
    demo_bringup_pkg = LaunchConfiguration("demo_bringup_pkg", default="")
    pkg = FindPackageShare(demo_bringup_pkg)
    launch_file = PathJoinSubstitution([pkg, "launch", "bringup.launch.py"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items()
    )
