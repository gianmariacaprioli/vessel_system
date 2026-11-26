"""Run a four-model inspection scenario at Viana do Castelo."""

import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DURIUS_XY = (0.0, 0.0)
UAV_ALTITUDE = 14.0
UUV_DEPTH = 2.5


def inspection_path(radius):
    """Five distinct vertices, with the first repeated to close the lap."""
    x, y = DURIUS_XY
    points = []
    for index in range(5):
        angle = -math.pi / 2 + index * math.tau / 5
        points.append((x + radius * math.cos(angle), y + radius * math.sin(angle)))
    return [coordinate for point in (*points, points[0]) for coordinate in point]


def generate_launch_description():
    simulator = get_package_share_directory('asv_simulator')

    def include(filename, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f'{simulator}/launch/{filename}.launch.py'),
            launch_arguments=arguments.items(),
        )

    def nautilus_node(executable, **parameters):
        return Node(
            package='vessel_system', executable=executable, namespace='nautilus_vessel',
            parameters=[{'use_sim_time': True, 'model': 'nautilus_vessel', **parameters}],
            output='screen',
        )

    # The DURIUS floater radius is 8.8 m. Even the middle of each
    # Nautilus chord is 14.5*cos(pi/5) = 11.7 m from its center.
    surface_path = inspection_path(14.5)
    outer_path = inspection_path(18.0)
    # Start the X500 lap at the southwest vertex closest to Nautilus.
    southwest = outer_path[8:10]
    air_path = southwest + outer_path[:8] + southwest
    # Along the inspection loop the port mesh is at least 4.85 m deep.
    underwater_path = [
        value for x, y in zip(outer_path[::2], outer_path[1::2])
        for value in (x, y, UUV_DEPTH)
    ]
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_gz_gui', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='vessel_viz.rviz'),
        include(
            'viana_durius_world', headless=LaunchConfiguration('headless'),
            start_gz_gui=LaunchConfiguration('start_gz_gui'),
            start_rviz='true', rviz_with_headless='true',
            rviz_config=LaunchConfiguration('rviz_config'),
        ),
        Node(
            package='vessel_system', executable='uuv_controller', output='screen',
            parameters=[{
                'use_sim_time': True, 'start_mode': 'waypoint',
                'waypoints_enu_depth': underwater_path,
                'max_forward_speed': 0.70, 'max_yaw_rate': 0.40,
                'surge_gain': 0.65, 'yaw_rate_gain': 0.30,
            }],
        ),
        nautilus_node('controller_node', approach_gain=1.0),
        nautilus_node('motor_mixer'),
        nautilus_node(
            'waypoint_manager_node', waypoints_xy=surface_path,
            acceptance_radius=1.2, max_arrival_speed=1.4,
        ),
        Node(
            package='vessel_system', executable='x500wp_manager', output='screen',
            parameters=[{
                'use_sim_time': True, 'use_odometry': True, 'waypoints_xy': air_path,
                'takeoff_altitude': UAV_ALTITUDE, 'position_tolerance': 1.0,
                'takeoff_delay': 0.6,
                'max_horizontal_velocity': 2.2,
            }],
        ),
    ])
