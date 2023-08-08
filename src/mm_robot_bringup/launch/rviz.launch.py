"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Start rviz
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  # pylint: disable=missing-function-docstring

    return LaunchDescription([
        *declare_launch_arguments(),
        launch_rviz2()
    ])


def declare_launch_arguments() -> list:
    """定义输入参数"""
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
    ]


def launch_rviz2():
    """启动 rviz2"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    bringup_pkg = FindPackageShare("mm_robot_bringup")
    rviz_config_file = PathJoinSubstitution([bringup_pkg, "configs", "rviz.rviz"])
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )
