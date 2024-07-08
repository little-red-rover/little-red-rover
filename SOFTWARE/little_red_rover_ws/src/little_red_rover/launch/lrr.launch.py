import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    run_sim = LaunchConfiguration("sim")
    
    config = [
        DeclareLaunchArgument(
            'sim',
            default_value='False',
            description='Run a simulation'
        )
    ]

    robot_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('little_red_rover'), '/sim.launch.py']
            ),
            condition=IfCondition(run_sim)
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('little_red_rover'), '/hardware.launch.py']
            ),
            condition=UnlessCondition(run_sim)
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
            parameters=[{'use_sim_time': run_sim,
                         'robot_description': robot_desc}],
            arguments=[urdf]),
    ]

    return LaunchDescription(config + robot_launch + visualization)
