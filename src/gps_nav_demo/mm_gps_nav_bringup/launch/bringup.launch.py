"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch gps nav demo
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  # pylint: disable=missing-function-docstring
    return LaunchDescription([
        *declare_launch_arguments(),
        launch_gps(),
        launch_imu(),
        *launch_robot_localization(),
        set_datum()
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


def launch_imu():
    """Launch imu controller"""
    pkg = FindPackageShare("mm_gps_nav_imu_controller")
    launch_file = PathJoinSubstitution([pkg, "launch", "imu.launch.py"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items()
    )


def launch_robot_localization():
    """Launch robot_localization"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    sim_pkg = FindPackageShare("mm_gps_nav_simulation")
    sim_ekf_config = PathJoinSubstitution([sim_pkg, "configs", "ekf.yaml"])

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[
                    sim_ekf_config,
                    {"use_sim_time": use_sim_time},
            ],
            remappings=[("odometry/filtered", "odometry/local")],
            condition=IfCondition(use_sim_time)
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[
                    sim_ekf_config,
                    {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("odometry/filtered", "odometry/global"),
            ],
            condition=IfCondition(use_sim_time)
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                    sim_ekf_config,
                    {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("imu", "mm_gps_nav_imu_controller/imu/fix"),
                ("gps/fix", "mm_gps_nav_gps_controller/gps/fix"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/global"),
            ],
            condition=IfCondition(use_sim_time)
        )
    ]


def set_datum():
    """Init robot position for localization"""
    set_gps_cmd = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/datum ",
                "robot_localization/srv/SetDatum ",
                '"{geo_pose: {position: {latitude: 31.3943545, longitude: 121.2294018, altitude: 18}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"',
            ]
        ],
        shell=True,
    )
    return TimerAction(period=3.0, actions=[set_gps_cmd])
