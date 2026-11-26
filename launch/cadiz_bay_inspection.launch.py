"""Run the original surface and airborne waypoint missions in Cádiz Bay."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulator = Path(get_package_share_directory('asv_simulator'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulator / 'launch' / 'cadiz_bay_world.launch.py')),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'start_rviz': 'true',
                'rviz_with_headless': 'true',
            }.items(),
        ),
        *[
            Node(
                package='vessel_system', executable=executable, namespace='vessel_v2',
                parameters=[{'model': 'vessel_v2', 'use_sim_time': True}], output='screen',
            )
            for executable in ('controller_node', 'motor_mixer', 'waypoint_manager_node')
        ],
        Node(
            package='vessel_system', executable='x500wp_manager', output='screen',
            parameters=[{'use_sim_time': True, 'use_odometry': True, 'takeoff_delay': 0.6}],
        ),
    ])
