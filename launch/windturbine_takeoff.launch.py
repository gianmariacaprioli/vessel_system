"""Start the wind turbine world and fly the X500 off the vessel_v2 deck."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = Path(get_package_share_directory('asv_simulator')) / 'launch'
    world = 'ocean_world'
    vessel_x = 30.0
    aft_deck_offset = -2.5

    def include(filename, **arguments):
        return GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / filename)),
                launch_arguments={'world': world, **arguments}.items(),
            ),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        include(
            'sim.launch.py', headless=LaunchConfiguration('headless'),
            start_rviz='true', rviz_with_headless='true',
        ),
        include('spawn_vessel_v2.launch.py', x=str(vessel_x), z='0.24'),
        include('spawn_uav.launch.py', x=str(vessel_x + aft_deck_offset), z='1.35'),
        Node(
            package='vessel_system',
            executable='deck_takeoff',
            output='screen',
        ),
    ])
