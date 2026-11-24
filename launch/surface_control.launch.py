"""Run independent waypoint controllers for the two surface vessels."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actions = []
    for model in ('vessel_v2', 'nautilus_vessel'):
        argument = f'control_{model}'
        actions.append(DeclareLaunchArgument(argument, default_value='true'))
        actions.append(GroupAction(
            condition=IfCondition(LaunchConfiguration(argument)),
            actions=[
                Node(package='vessel_system', executable=executable,
                     namespace=model, parameters=[{'model': model}],
                     output='screen')
                for executable in ('controller_node', 'motor_mixer')
            ],
        ))
    return LaunchDescription(actions)
