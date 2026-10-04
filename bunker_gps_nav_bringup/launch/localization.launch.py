"""Run GNSS processing, datum initialization and both localization EKFs."""

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
    topics, frames = system['topics'], system['frames']
    package = 'bunker_gps_nav_localization'
    config_dir = Path(get_package_share_directory(package)) / 'config'
    gnss_config = str(config_dir / 'gnss.yaml')

    mode = system['datum']['mode']
    if mode not in ('auto', 'manual'):
        raise ValueError('datum.mode must be auto or manual')
    datum_params = {'use_sim_time': False, 'datum_mode': mode}
    if mode == 'manual':
        if 'latitude' not in system['datum'] or 'longitude' not in system['datum']:
            raise ValueError('Manual datum requires measured latitude and longitude in system.yaml')
        datum_params.update({
            'datum.latitude': float(system['datum']['latitude']),
            'datum.longitude': float(system['datum']['longitude']),
            'datum.altitude': float(system['datum'].get('altitude', 0.0)),
        })
    ekf_frames = {
        'use_sim_time': False,
        'map_frame': frames['map'],
        'odom_frame': frames['odom'],
        'base_link_frame': frames['base'],
    }

    return [
        Node(
            package=package, executable='gnss_heading_node', name='gnss_heading_node',
            output='screen', parameters=[gnss_config, {
                'use_sim_time': False,
                'heading_frame': frames['base'],
                'heading_mount_offset_deg': system['heading_mount_offset_deg'],
            }], remappings=[
                ('relposned', topics['relposned']),
                ('heading_imu', '/gnss/heading_imu'),
                ('rtk_state', '/gnss/rtk_state'),
            ]),
        Node(
            package=package, executable='gnss_midpoint_node', name='gnss_midpoint_node',
            output='screen', parameters=[gnss_config, {
                'use_sim_time': False, 'center_frame': frames['base'],
            }], remappings=[
                ('base_fix', topics['base_gps']), ('rover_fix', topics['rover_gps']),
                ('fix_center', '/gnss/fix_center'), ('rtk_state', '/gnss/rtk_state'),
            ]),
        Node(
            package=package, executable='datum_manager_node', name='datum_manager_node',
            output='screen', parameters=[datum_params], remappings=[
                ('fix_center', '/gnss/fix_center'), ('fix_navsat', '/gnss/fix_navsat'),
            ]),
        # bunker_ros2 owns odom -> base_link; the local EKF only publishes odometry.
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_local',
            output='screen', parameters=[str(config_dir / 'ekf_local.yaml'), {
                **ekf_frames, 'world_frame': frames['odom'], 'publish_tf': False,
                'odom0': topics['wheel_odom'], 'imu0': topics['imu'],
            }], remappings=[('odometry/filtered', '/odometry/local')]),
        # Only the global EKF publishes map -> odom.
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_global',
            output='screen', parameters=[str(config_dir / 'ekf_global.yaml'), {
                **ekf_frames, 'world_frame': frames['map'], 'publish_tf': True,
            }], remappings=[('odometry/filtered', '/odometry/global')]),
        Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen',
            parameters=[str(config_dir / 'navsat_transform.yaml'), {'use_sim_time': False}],
            remappings=[
                ('gps/fix', '/gnss/fix_navsat'), ('imu', '/gnss/heading_imu'),
                ('odometry/filtered', '/odometry/global'), ('odometry/gps', '/odometry/gps'),
            ]),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory('bunker_gps_nav_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config', default_value=str(bringup / 'config' / 'system.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
