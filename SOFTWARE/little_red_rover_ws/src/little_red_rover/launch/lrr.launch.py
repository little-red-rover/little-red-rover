from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # DIFFERENTIAL DRIVE CONTROLLER
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("little_red_rover"),
            "config",
            "lrr_controllers.yaml",
        ]
    )

    diff_drive = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            output="both",
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
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diffbot_base_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
    ]

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

    return LaunchDescription(config + robot_launch + diff_drive + teleop)
