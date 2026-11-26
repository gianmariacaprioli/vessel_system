"""Approach the offshore turbine, launch X500 and inspect the platform."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


START_CLEARANCE = 30.0
CENTER_FLOATER_RADIUS = 3.0
BOW_OFFSET = 13.11 / 2
PLATFORM_CENTER_X = -17.32051
ORBIT_RADIUS = 44.5
ENTRY_RADIUS = 5.5


def inspection_route():
    # Connect the westbound approach tangentially to the east side of the lap.
    center_distance = ORBIT_RADIUS + ENTRY_RADIUS
    angle = math.asin(ENTRY_RADIUS / center_distance)
    entry_x = PLATFORM_CENTER_X + center_distance * math.cos(angle)
    start_x = START_CLEARANCE + CENTER_FLOATER_RADIUS + BOW_OFFSET
    points = [(start_x, 0.0)]
    for step in range(13):
        entry_angle = -math.pi / 2 - (math.pi / 2 - angle) * step / 12
        points.append((
            entry_x + ENTRY_RADIUS * math.cos(entry_angle),
            ENTRY_RADIUS * (1 + math.sin(entry_angle)),
        ))
    for step in range(1, 145):
        orbit_angle = angle + math.tau * step / 144
        points.append((
            PLATFORM_CENTER_X + ORBIT_RADIUS * math.cos(orbit_angle),
            ORBIT_RADIUS * math.sin(orbit_angle),
        ))
    return [coordinate for point in points for coordinate in point]


def generate_launch_description():
    simulator = Path(get_package_share_directory('asv_simulator'))
    perimeter = inspection_route()
    vessel_parameters = {'model': 'vessel_v2', 'use_sim_time': True}

    def vessel_node(executable, **parameters):
        return Node(
            package='vessel_system', executable=executable, namespace='vessel_v2',
            parameters=[{**vessel_parameters, **parameters}], output='screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulator / 'launch' / 'ocean_world.launch.py')),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'vessel_clearance': str(START_CLEARANCE),
                'start_rviz': 'true',
                'rviz_with_headless': 'true',
                'include_uuv': 'false',
            }.items(),
        ),
        vessel_node(
            'surface_path_follower', path_xy=perimeter, entry_speed=0.95, entry_length=24.0,
        ),
        vessel_node('motor_mixer', preserve_surge=True),
        Node(
            package='vessel_system', executable='x500wp_manager', output='screen',
            parameters=[{
                'use_sim_time': True, 'use_odometry': True,
                'takeoff_altitude': 24.0, 'takeoff_delay': 0.6,
                'takeoff_vessel_radius': 28.0,
                'column_orbit_radius': 9.0, 'position_tolerance': 0.8,
                'max_horizontal_velocity': 2.0,
            }],
        ),
    ])
