"""Run Nav2, the navigation supervisor, goal bridge and optional RViz."""

from pathlib import Path
import tempfile

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
    nav2_share = Path(get_package_share_directory('bunker_gps_nav_nav2'))

    nav2_path = LaunchConfiguration('nav2_config').perform(context)
    with open(nav2_path, encoding='utf-8') as stream:
        nav2_params = yaml.safe_load(stream)
    behavior_tree = str(nav2_share / 'behavior_trees' / 'navigate_replanning.xml')
    nav2_params['bt_navigator']['ros__parameters'].update({
        'global_frame': frames['map'], 'robot_base_frame': frames['base'],
        'default_nav_to_pose_bt_xml': behavior_tree,
        # Humble also requires a through-poses tree during configuration.
        'default_nav_through_poses_bt_xml': behavior_tree,
    })
    nav2_params['behavior_server']['ros__parameters'].update({
        'global_frame': frames['odom'], 'robot_base_frame': frames['base'],
    })
    for costmap, frame in [('global_costmap', frames['map']), ('local_costmap', frames['odom'])]:
        params = nav2_params[costmap][costmap]['ros__parameters']
        params.update({
            'global_frame': frame, 'robot_base_frame': frames['base'],
            'footprint': system['footprint'], 'use_sim_time': False,
        })
        params['voxel_layer']['cloud'].update({
            'topic': topics['obstacles'], 'sensor_frame': frames['lidar'],
        })
    for name in ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']:
        nav2_params[name]['ros__parameters']['use_sim_time'] = False
    # Nested costmap nodes must receive the full parameter YAML.
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='bunker_nav2_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(nav2_params, stream)
        resolved_nav2_config = stream.name

    nav2_remappings = [
        ('cmd_vel', topics['cmd_vel_nav']), ('goal_pose', '/navigation/nav2_goal_pose'),
    ]
    nav2_files = [resolved_nav2_config, {'use_sim_time': False}]
    safety_config = str(
        Path(get_package_share_directory('bunker_gps_nav_safety')) / 'config' / 'safety.yaml')
    goal_config = str(
        Path(get_package_share_directory('bunker_gps_nav_goal')) / 'config' / 'goal.yaml')

    actions = [
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=nav2_files, remappings=nav2_remappings),
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=nav2_files, remappings=nav2_remappings),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=nav2_files, remappings=nav2_remappings),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=nav2_files, remappings=nav2_remappings),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen', parameters=[{
                'use_sim_time': False, 'autostart': True,
                'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator'],
            }]),
        Node(
            package='bunker_gps_nav_safety', executable='navigation_supervisor_node',
            name='navigation_supervisor_node', output='screen',
            parameters=[safety_config, {'use_sim_time': False}], remappings=[
                ('cmd_vel', topics['cmd_vel']), ('cmd_vel_nav', topics['cmd_vel_nav']),
                ('rtk_state', '/gnss/rtk_state'), ('heading_imu', '/gnss/heading_imu'),
                ('fix_center', '/gnss/fix_center'), ('odometry_local', '/odometry/local'),
                ('odometry_global', '/odometry/global'), ('odometry_gps', '/odometry/gps'),
                ('imu', topics['imu']), ('wheel_odom', topics['wheel_odom']),
                ('obstacles', topics['obstacles']), ('motion_allowed', '/navigation/motion_allowed'),
            ]),
        Node(
            package='bunker_gps_nav_goal', executable='gps_goal_bridge_node',
            name='gps_goal_bridge_node', output='screen', parameters=[goal_config, {
                'use_sim_time': False, 'map_frame': frames['map'], 'base_frame': frames['base'],
            }], remappings=[
                ('gps_goal', topics['gps_goal']), ('goal_pose', topics['rviz_goal']),
                ('waypoint', topics.get('rviz_waypoint', '/navigation/waypoint_input')),
                ('waypoints/start', '/navigation/waypoints/start'),
                ('waypoints/cancel', '/navigation/waypoints/cancel'),
                ('waypoints/clear', '/navigation/waypoints/clear'),
                ('waypoints/remove_last', '/navigation/waypoints/remove_last'),
                ('waypoints/status', '/navigation/waypoints/status'),
                ('waypoints/markers', '/navigation/waypoints/markers'),
                ('odometry_gps', '/odometry/gps'), ('motion_allowed', '/navigation/motion_allowed'),
            ]),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', str(nav2_share / 'rviz' / 'gps_navigation.rviz')],
            parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    bringup = Path(get_package_share_directory('bunker_gps_nav_bringup'))
    nav2_share = Path(get_package_share_directory('bunker_gps_nav_nav2'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config', default_value=str(bringup / 'config' / 'system.yaml')),
        DeclareLaunchArgument(
            'nav2_config', default_value=str(nav2_share / 'config' / 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
