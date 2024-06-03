import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    # CONNECT TO ROBOT
    # Make sure connection is available to microROS.
    # If not, prompt user to run connection utility
    try:
        robot_ip = Path(
            '/little_red_rover_ws/src/robot_ip.txt').read_text().strip()
    except:
        print("Cannot read /little_red_rover_ws/src/robot_ip.txt.")
        print("Please connect to the robot's wifi hotspot, then run 'lrr_connect' in your terminal.")
        return

    # GAZEBO
    # Launch descriptions relating to gazebo.
    # Will be omitted when launching using real hardware.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_launch = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch'), '/rosbridge_websocket_launch.xml']
            )
        ),
        ExecuteProcess(
            cmd=['curl', '-s', '-X', 'GET', robot_ip + "/set-agent-ip"],
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s',
                 '--headless-rendering', '-v', '4', 'shapes.sdf'],
        ),
        ExecuteProcess(
            cmd=['gz', 'launch', '-v', '4', '/websocket.gzlaunch']
        ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=[
        #         '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        # ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        # ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=['/lidar2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        # ),
    ]

    # MICRO ROS
    # Setup communication with the microROS instance running on the robot.
    # First ping so it can save the agent's IP address.
    # Then launch the microROS agent, which relays messages sent from uROS.
    micro_ros_launch = [
        ExecuteProcess(
            cmd=['curl', '-s', '-X', 'GET', robot_ip + "/set-agent-ip"],
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '8001', '--ip', robot_ip],
        ),
    ]

    # VISUALIZATION
    # Settings / utilities relating to visualization of the robot.
    urdf_file_name = 'robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('little_red_rover'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    visualization = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc}],
            arguments=[urdf]),
    ]

    # return LaunchDescription(gazebo_launch + micro_ros_launch + visualization)
    return LaunchDescription(micro_ros_launch + visualization)

    # ExecuteProcess(
    #     cmd=['ros2', 'param', 'set', '/gazebo',
    #          'use_sim_time', use_sim_time]
    # )
