"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch imu controller
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():  # pylint: disable=missing-function-docstring
    imu_node = launch_imu()
    return LaunchDescription([
        *declare_launch_arguments(),
        imu_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=imu_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=imu_node,
                start_state='configuring', goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
        ),
    ])


def declare_launch_arguments() -> list:
    """Declare launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    ]


def launch_imu():
    """Launch imu controller"""
    return LifecycleNode(
        package="mm_gps_nav_imu_controller",
        executable="sim_imu_controller",
        name="mm_gps_nav_imu_controller",
        namespace="",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
