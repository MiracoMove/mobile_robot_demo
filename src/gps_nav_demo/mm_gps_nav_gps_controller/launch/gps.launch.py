"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch gps controller
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():  # pylint: disable=missing-function-docstring
    gps_nodes = launch_gps()

    ld = LaunchDescription()  # pylint: disable=invalid-name
    ld.add_action(*declare_launch_arguments())
    for gps_node in gps_nodes:
        ld.add_action(gps_node)
        ld.add_action(RegisterEventHandler(
            OnProcessStart(
                target_action=gps_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(gps_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ))
        ld.add_action(RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=gps_node,
                start_state='configuring', goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(gps_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
        ))
    return ld


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
    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        LifecycleNode(
            package="mm_gps_nav_gps_controller",
            executable="sim_gps_controller",
            name="mm_gps_nav_gps_controller",
            namespace="",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_sim_time)
        ),
        LifecycleNode(
            package="mm_gps_nav_gps_controller",
            executable="wit_gps_controller",
            name="mm_gps_nav_gps_controller",
            namespace="",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=UnlessCondition(use_sim_time)
        )
    ]
