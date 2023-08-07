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
        *launch_gazebo(),
        spawn_robot()
    ])


def launch_robot_state_publisher():
    """Launch robot state publisher"""
    # Get current ros2 package path
    pkg_path = FindPackageShare("mm_robot_simulation")
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
                "use_sim_time": True,
            }
        ]
    )


def launch_gazebo() -> list:
    """Launch gazebo"""
    # Get current package path
    pkg_path = FindPackageShare("mm_robot_simulation")
    # Set gazebo model path
    global_share_path = os.path.join(get_package_share_directory("mm_robot_simulation"), "../")
    home_path = os.environ["HOME"]
    os.environ["GAZEBO_MODEL_PATH"] = f"{global_share_path}/:{home_path}/.gazebo/models"

    # Get gazebo launch file path
    pkg_gazebo_ros_path = FindPackageShare("gazebo_ros")
    gz_server_launch_file = PathJoinSubstitution(
        [pkg_gazebo_ros_path, "launch", "gzserver.launch.py"])
    gz_client_launch_file = PathJoinSubstitution(
        [pkg_gazebo_ros_path, "launch", "gzclient.launch.py"])

    # Get world file path
    world = PathJoinSubstitution([pkg_path, "worlds", "empty.world"])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_server_launch_file),
            launch_arguments={"world": world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_client_launch_file)
        )
    ]


def spawn_robot():
    """Spwan robot in gazebo"""
    x_pose = "0.0"
    y_pose = "0.0"
    spawn_ppr_robot_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "ppr_robot",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.0",
        ],
        output="screen",
    )
    return TimerAction(period=3.0, actions=[spawn_ppr_robot_cmd])
