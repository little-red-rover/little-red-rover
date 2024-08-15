from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hal_launch = [
        Node(
            package="little_red_rover",
            executable="hal",
            output="both",
        ),
    ]

    return LaunchDescription(hal_launch)
