from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = "192.168.4.1"

    # Setup communication with the microROS instance running on the robot.
    # First ping the robot so it can learn the agent's IP address.
    # Then launch the microROS agent, which relays messages sent from uROS.
    micro_ros_launch = [
        ExecuteProcess(
            cmd=["curl", "-s", "-X", "GET", robot_ip + "/set-agent-ip"],
        ),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            output="both",
            arguments=["udp4", "--port", "8001", "--ip", robot_ip],
        ),
    ]

    return LaunchDescription(micro_ros_launch)
