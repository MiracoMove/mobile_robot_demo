"""
Author: Letian Wang (letian.wang@miracomove.com)
MiracoMove (c) 2023
Desc: Launch gps nav demo
"""
import launch
import launch.launch_description_sources
import launch_ros
import nav2_common.launch


def generate_launch_description():  # pylint: disable=missing-function-docstring
    return launch.LaunchDescription([
        *declare_launch_arguments(),
        launch_gps(),
        launch_imu(),
        *launch_robot_localization(),
        *launch_nav2(),
        set_datum(),
        launch_twist_mux()
    ])


def declare_launch_arguments() -> list:
    """Declare launch arguments"""
    return [
        launch.actions.DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        )
    ]


def launch_gps():
    """Launch gps controller"""
    pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_gps_controller")
    launch_file = launch.substitutions.PathJoinSubstitution([pkg, "launch", "gps.launch.py"])
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time"),
        }.items()
    )


def launch_imu():
    """Launch imu controller"""
    pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_imu_controller")
    launch_file = launch.substitutions.PathJoinSubstitution([pkg, "launch", "imu.launch.py"])
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time"),
        }.items()
    )


def launch_robot_localization():
    """Launch robot_localization"""
    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time", default="false")

    sim_pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_simulation")
    sim_ekf_config = launch.substitutions.PathJoinSubstitution([sim_pkg, "configs", "ekf.yaml"])

    pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_bringup")
    ekf_config = launch.substitutions.PathJoinSubstitution([pkg, "configs", "ekf.yaml"])

    nodes = []
    for (config, condition) in [(sim_ekf_config, launch.conditions.IfCondition(use_sim_time)),
                                (ekf_config, launch.conditions.UnlessCondition(use_sim_time))]:
        nodes.extend([
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("odometry/filtered", "odometry/local")],
                condition=condition
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("odometry/filtered", "odometry/global"),
                ],
                condition=condition
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("imu", "mm_gps_nav_imu_controller/imu/fix"),
                    ("gps/fix", "mm_gps_nav_gps_controller/gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                condition=condition
            )
        ])
    return nodes


def launch_nav2():
    """Launch nav2"""
    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time", default="false")

    sim_pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_simulation")
    sim_nav2_config = launch.substitutions.PathJoinSubstitution([sim_pkg, "configs", "nav2.yaml"])
    sim_nav_through_poses_bt_xml = launch.substitutions.PathJoinSubstitution(
        [sim_pkg, "configs", "nav_through_poses_bt.xml"])
    sim_nav2_params = nav2_common.launch.RewrittenYaml(
        source_file=sim_nav2_config, root_key="",
        param_rewrites={"default_nav_through_poses_bt_xml": sim_nav_through_poses_bt_xml},
        convert_types=True)

    pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_bringup")
    nav2_config = launch.substitutions.PathJoinSubstitution([pkg, "configs", "nav2.yaml"])
    nav_through_poses_bt_xml = launch.substitutions.PathJoinSubstitution(
        [pkg, "configs", "nav_through_poses_bt.xml"])
    nav2_params = nav2_common.launch.RewrittenYaml(
        source_file=nav2_config, root_key="",
        param_rewrites={"default_nav_through_poses_bt_xml": nav_through_poses_bt_xml},
        convert_types=True)

    nav2_bringup_pkg = launch_ros.substitutions.FindPackageShare("nav2_bringup")
    return [
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                launch.substitutions.PathJoinSubstitution(
                    [nav2_bringup_pkg, "launch", "navigation_launch.py"])),
            launch_arguments={"use_sim_time": use_sim_time, "params_file": sim_nav2_params,
                              "autostart": "true", }.items(),
            condition=launch.conditions.IfCondition(use_sim_time)),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                launch.substitutions.PathJoinSubstitution(
                    [nav2_bringup_pkg, "launch", "navigation_launch.py"])),
            launch_arguments={"use_sim_time": use_sim_time, "params_file": nav2_params,
                              "autostart": "true", }.items(),
            condition=launch.conditions.UnlessCondition(use_sim_time)),
    ]


def launch_twist_mux():
    """启动 twist_mux"""
    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time", default="false")
    sim_pkg = launch_ros.substitutions.FindPackageShare("mm_gps_nav_simulation")
    sim_twist_mux_config = launch.substitutions.PathJoinSubstitution(
        [sim_pkg, "configs", "twist_mux.yaml"])
    return launch_ros.actions.Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[sim_twist_mux_config, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
        condition=launch.conditions.IfCondition(use_sim_time),
    )


def set_datum():
    """Init robot position for localization"""
    set_gps_cmd = launch.actions.ExecuteProcess(
        cmd=[
            [
                launch.substitutions.FindExecutable(name="ros2"),
                " service call ",
                "/datum ",
                "robot_localization/srv/SetDatum ",
                '"{geo_pose: {position: {latitude: 31.3943545, longitude: 121.2294018, altitude: 18}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"',
            ]
        ],
        shell=True,
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("use_sim_time")),
    )
    return launch.actions.TimerAction(period=3.0, actions=[set_gps_cmd])
