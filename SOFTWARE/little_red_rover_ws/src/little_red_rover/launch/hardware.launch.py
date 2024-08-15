from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = "192.168.4.1"

    hal_launch = [
        ExecuteProcess(
            cmd=["curl", "-s", "-X", "GET", robot_ip + "/set-agent-ip"],
        ),
        Node(
            package="little_red_rover",
            executable="hal",
            output="both",
        ),
    ]

    return LaunchDescription(hal_launch)
