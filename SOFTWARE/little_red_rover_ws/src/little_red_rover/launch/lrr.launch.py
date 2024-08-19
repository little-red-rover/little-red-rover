from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    run_sim = LaunchConfiguration("sim")

    config = [
        DeclareLaunchArgument(
            "sim", default_value="False", description="Run a simulation"
        )
    ]

    robot_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("little_red_rover"),
                    "/launch/sim.launch.py",
                ]
            ),
            condition=IfCondition(run_sim),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("little_red_rover"),
                    "/launch/hardware.launch.py",
                ]
            ),
            condition=UnlessCondition(run_sim),
        ),
    ]

    # XACRO DESCRIPTION
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("little_red_rover"),
                        "description",
                        "lrr.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    # TELEOPERATION
    teleop = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("teleop_twist_joy"),
                    "/launch/teleop-launch.py",
                ]
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        get_package_share_directory("little_red_rover"),
                        "config",
                        "joy_config.yaml",
                    ]
                ),
            }.items(),
        ),
    ]

    # VISUALIZATION
    visualization = [
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [
                    get_package_share_directory("foxglove_bridge"),
                    "/launch/foxglove_bridge_launch.xml",
                ]
            ),
            launch_arguments={"port": "8765"}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "use_sim_time": run_sim,
                    "robot_description": robot_description_content,
                }
            ],
        ),
    ]

    # ODOMETRY
    odometry = [
        Node(
            package="little_red_rover",
            executable="odometry_publisher",
            output="both",
            parameters=[
                {
                    "use_sim_time": run_sim,
                }
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        get_package_share_directory("little_red_rover"),
                        "config",
                        "ekf.yaml",
                    ]
                ),
                {
                    "use_sim_time": run_sim,
                },
            ],
        ),
    ]

    # ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
    # ros2 launch slam_toolbox online_async_launch.py
    navigation = [
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [
        #             get_package_share_directory("nav2_bringup"),
        #             "/launch/navigation_launch.py",
        #         ]
        #     ),
        #     launch_arguments={
        #         "use_sim_time": run_sim,
        #     }.items(),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("slam_toolbox"),
                    "/launch/online_async_launch.py",
                ]
            ),
            launch_arguments={
                "use_sim_time": run_sim,
                "slam_params_file": PathJoinSubstitution(
                    [
                        get_package_share_directory("little_red_rover"),
                        "config",
                        "slam_toolbox.yaml",
                    ]
                ),
            }.items(),
        ),
    ]

    return LaunchDescription(
        config + robot_launch + teleop + visualization + odometry + navigation
    )
