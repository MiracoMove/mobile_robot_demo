"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch ros2_control
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():  # pylint: disable=missing-function-docstring
    return LaunchDescription([
        *declare_launch_arguments(),
        launch_controller_manager(),
        spawn_diff_drive_controller(),
        spawn_joint_broad_controller(),
    ])


def declare_launch_arguments() -> list:
    """Declare launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=""
        )
    ]


def launch_controller_manager():
    """Launch controller manager"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    config_file = LaunchConfiguration("config_file")
    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )
    cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_file, {"robot_description": robot_description}],
        output="screen",
        condition=UnlessCondition(use_sim_time),
    )
    return TimerAction(period=3.0, actions=[cmd])


def spawn_diff_drive_controller():
    """Spawn diff drive controller"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )


def spawn_joint_broad_controller():
    """Spawn joint broad controller"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
