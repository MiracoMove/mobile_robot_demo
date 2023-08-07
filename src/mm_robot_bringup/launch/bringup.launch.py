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
    机器人按如下顺序启动：
    1. 启动 robot_state_publisher
    2. 如果 use_sim_time 为 true, 启动模拟环境
    3. 启动机器人硬件相关模块
    4. 启动 ros2_control 相关
    5. 启动 navigation 相关
    6. 启动模块管理器
    7. 启动 rviz2
    """

    return LaunchDescription([
        *declare_launch_arguments(),
        launch_robot_state_publisher(),
        launch_ros2_control(),
        # launch_navigation_by_condition(sim=False),
        # launch_navigation_by_condition(sim=True),
    ])


def declare_launch_arguments() -> list:
    """Declare launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
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
    """启动 ros2_control 相关"""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    bringup_pkg = FindPackageShare("mm_robot_bringup")
    # config_file = PathJoinSubstitution(
    #     [robot_pkg, f"{'simulation/' if sim else ''}configs", "controllers.yaml"])
    ros2_control_launch_file = PathJoinSubstitution(
        [bringup_pkg, "launch", "ros2_control.launch.py"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # "config_file": config_file,
        }.items()
    )


# def launch_navigation_by_condition(sim: bool = False):
#     """启动导航相关"""
#     use_sim_time = LaunchConfiguration("use_sim_time", default="false")
#     robot = LaunchConfiguration("robot")
#     robot_pkg = FindPackageShare(robot)
#     localization_config = PathJoinSubstitution(
#         [robot_pkg, f"{'simulation/' if sim else ''}configs", "ekf.yaml"])
#     nav2_config = PathJoinSubstitution(
#         [robot_pkg, f"{'simulation/' if sim else ''}configs", "nav2.yaml"])
#     twist_mux_config = PathJoinSubstitution(
#         [robot_pkg, f"{'simulation/' if sim else ''}configs", "twist_mux.yaml"])
#     bringup_pkg = FindPackageShare("fruit_bringup")
#     navigation_launch_file = PathJoinSubstitution([bringup_pkg, "launch", "navigation.launch.py"])
#     return IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(navigation_launch_file),
#         launch_arguments={
#             "use_sim_time": use_sim_time,
#             "localization_config": localization_config,
#             "nav2_config": nav2_config,
#             "twist_mux_config": twist_mux_config,
#         }.items(),
#         condition=IfCondition(use_sim_time) if sim else UnlessCondition(use_sim_time),
#     )
