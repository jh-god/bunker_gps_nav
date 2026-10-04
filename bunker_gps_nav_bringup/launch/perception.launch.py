"""Run obstacle cloud filtering using the configured sensor topic and base frame."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    system_path = LaunchConfiguration('system_config').perform(context)
    with open(system_path, encoding='utf-8') as stream:
        system = yaml.safe_load(stream)
    config_dir = Path(get_package_share_directory('bunker_gps_nav_perception')) / 'config'

    return [
        Node(
            package='bunker_gps_nav_perception', executable='obstacle_cloud_filter_node',
            name='obstacle_cloud_filter_node', output='screen',
            parameters=[str(config_dir / 'obstacle_filter.yaml'), {
                'use_sim_time': False, 'output_frame': system['frames']['base'],
            }], remappings=[
                ('nonground', system['topics']['nonground']),
                ('obstacles', system['topics']['obstacles']),
            ]),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory('bunker_gps_nav_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config', default_value=str(bringup / 'config' / 'system.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
