import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_lrr = get_package_share_directory("little_red_rover")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    gazebo_launch = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r -s --headless-rendering {os.path.join(pkg_lrr,'testing.sdf')}"
            }.items(),
            # launch_arguments={
            #     "gz_args": f"-s -r --headless-rendering sensors_demo.sdf"
            # }.items(),
        ),
        ExecuteProcess(
            cmd=["ros2", "param", "set", "/gazebo", "use_sim_time", use_sim_time]
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            # arguments=[
            #     # '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            #     # '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            # ],
            arguments=[
                "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
                "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            ],
            # output="screen",
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("rosbridge_server"), "launch"
                    ),
                    "/rosbridge_websocket_launch.xml",
                ]
            )
        ),
    ]

    return LaunchDescription(gazebo_launch)
