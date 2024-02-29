"""Launch Gazebo server and client with command line arguments."""
"""Spawn robot from URDF file."""
# https://gist.github.com/rfzeg/63d824f0c6ed44da639e9630a76fbc6c

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v', '4', 'visualize_lidar.sdf'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gz', 'launch', '-v', '4', '/websocket.gzlaunch']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        ),    
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        ),    
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/lidar2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        ),    
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo',
                 'use_sim_time', use_sim_time]
        )
    ])