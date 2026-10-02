"""Shared launch assembly; configuration is resolved without architecture-specific paths."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config(package, filename):
    return str(Path(share(package)) / 'config' / filename)


def generate(component='all'):
    def assemble(context):
        system_path = LaunchConfiguration('system_config').perform(context)
        with open(system_path, encoding='utf-8') as stream:
            cfg = yaml.safe_load(stream)
        t, f = cfg['topics'], cfg['frames']
        sim = False  # Live sensors and wall time on the robot.
        rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
        def node(package, executable, name=None, files=(), params=None, remaps=()):
            return Node(package=package, executable=executable, name=name or executable,
                        output='screen', parameters=[*files, {'use_sim_time': sim}, params or {}],
                        remappings=list(remaps))
        actions = []
        if component == 'all':
            for name, driver in cfg['drivers'].items():
                if not driver['enabled']:
                    continue
                if not driver['package'] or not driver['launch_file']:
                    raise ValueError(f'{name}: configure package and launch_file before enabling')
                path = Path(share(driver['package'])) / 'launch' / driver['launch_file']
                if not path.is_file():
                    raise ValueError(f'{name}: launch file does not exist: {path}')
                arguments = {k: str(v).lower() if isinstance(v, bool) else str(v)
                             for k, v in driver.get('arguments', {}).items()}
                actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                    launch_arguments=arguments.items()))
        if component in ('all', 'localization'):
            pkg = 'bunker_gps_nav_localization'
            gnss_file = config(pkg, 'gnss.yaml')
            mode = cfg['datum']['mode']
            if mode not in ('auto', 'manual'):
                raise ValueError('datum.mode must be auto or manual')
            datum_params = {'datum_mode': mode}
            if mode == 'manual':
                if 'latitude' not in cfg['datum'] or 'longitude' not in cfg['datum']:
                    raise ValueError('Manual datum requires measured latitude and longitude in system.yaml')
                datum_params.update({
                    'datum.latitude': float(cfg['datum']['latitude']),
                    'datum.longitude': float(cfg['datum']['longitude']),
                    'datum.altitude': float(cfg['datum'].get('altitude', 0.0))})
            actions += [
                node(pkg,'gnss_heading_node',files=[gnss_file],params={
                    'heading_frame':f['base'],'heading_mount_offset_deg':cfg['heading_mount_offset_deg']},
                    remaps=[('relposned',t['relposned']),('heading_imu','/gnss/heading_imu'),('rtk_state','/gnss/rtk_state')]),
                node(pkg,'gnss_midpoint_node',files=[gnss_file],params={'center_frame':f['base']},remaps=[
                    ('base_fix',t['base_gps']),('rover_fix',t['rover_gps']),('fix_center','/gnss/fix_center'),('rtk_state','/gnss/rtk_state')]),
                node(pkg,'datum_manager_node',params=datum_params,remaps=[('fix_center','/gnss/fix_center'),('fix_navsat','/gnss/fix_navsat')]),
            ]
            frames = {'map_frame':f['map'],'odom_frame':f['odom'],'base_link_frame':f['base']}
            actions += [
                node('robot_localization','ekf_node','ekf_local',files=[config(pkg,'ekf_local.yaml')],params={
                    **frames,'world_frame':f['odom'],'publish_tf':False,
                    'odom0':t['wheel_odom'],'imu0':t['imu']},remaps=[('odometry/filtered','/odometry/local')]),
                node('robot_localization','ekf_node','ekf_global',files=[config(pkg,'ekf_global.yaml')],params={
                    **frames,'world_frame':f['map'],'publish_tf':True},remaps=[('odometry/filtered','/odometry/global')]),
                node('robot_localization','navsat_transform_node','navsat_transform',files=[config(pkg,'navsat_transform.yaml')],remaps=[
                    ('gps/fix','/gnss/fix_navsat'),('imu','/gnss/heading_imu'),
                    ('odometry/filtered','/odometry/global'),('odometry/gps','/odometry/gps')]),
            ]
        if component in ('all', 'perception'):
            pkg='bunker_gps_nav_perception'
            actions.append(node(pkg,'obstacle_cloud_filter_node',files=[config(pkg,'obstacle_filter.yaml')],
                params={'output_frame':f['base']},remaps=[('nonground',t['nonground']),('obstacles',t['obstacles'])]))
        if component in ('all', 'navigation'):
            pkg='bunker_gps_nav_nav2'
            nav_path=LaunchConfiguration('nav2_config').perform(context)
            with open(nav_path, encoding='utf-8') as stream:
                nav_cfg=yaml.safe_load(stream)
            nav_cfg['bt_navigator']['ros__parameters'].update({
                'global_frame':f['map'],'robot_base_frame':f['base'],
                'default_nav_to_pose_bt_xml':str(Path(share(pkg))/'behavior_trees'/'navigate_replanning.xml'),
                # A second tree is required during Humble navigator configuration.
                'default_nav_through_poses_bt_xml':str(Path(share(pkg))/'behavior_trees'/'navigate_replanning.xml')})
            nav_cfg['behavior_server']['ros__parameters'].update({'global_frame':f['odom'],'robot_base_frame':f['base']})
            for costmap, frame in [('global_costmap',f['map']),('local_costmap',f['odom'])]:
                values=nav_cfg[costmap][costmap]['ros__parameters']
                values.update({'global_frame':frame,'robot_base_frame':f['base'],'footprint':cfg['footprint']})
                values['voxel_layer']['cloud'].update({'topic':t['obstacles'],'sensor_frame':f['lidar']})
                values['use_sim_time']=sim
            for name in ['planner_server','controller_server','behavior_server','bt_navigator']:
                nav_cfg[name]['ros__parameters']['use_sim_time']=sim
            # Child costmap nodes also need the full YAML; a Node parameter dict is insufficient.
            with tempfile.NamedTemporaryFile(mode='w',prefix='bunker_nav2_',suffix='.yaml',delete=False) as stream:
                yaml.safe_dump(nav_cfg,stream);resolved=stream.name
            for package, executable in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),
                                         ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
                actions.append(node(package,executable,files=[resolved],remaps=[('cmd_vel',t['cmd_vel_nav']),('goal_pose','/navigation/nav2_goal_pose')]))
            actions.append(node('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager_navigation',params={
                'autostart':True,'node_names':['planner_server','controller_server','behavior_server','bt_navigator']}))
            pkg='bunker_gps_nav_safety'
            actions.append(node(pkg,'navigation_supervisor_node',files=[config(pkg,'safety.yaml')],remaps=[
                ('cmd_vel',t['cmd_vel']),('cmd_vel_nav',t['cmd_vel_nav']),('rtk_state','/gnss/rtk_state'),
                ('heading_imu','/gnss/heading_imu'),('fix_center','/gnss/fix_center'),
                ('odometry_local','/odometry/local'),('odometry_global','/odometry/global'),('odometry_gps','/odometry/gps'),
                ('imu',t['imu']),('wheel_odom',t['wheel_odom']),('obstacles',t['obstacles']),('motion_allowed','/navigation/motion_allowed')]))
            pkg='bunker_gps_nav_goal'
            actions.append(node(pkg,'gps_goal_bridge_node',files=[config(pkg,'goal.yaml')],params={
                'map_frame':f['map'],'base_frame':f['base']},remaps=[('gps_goal',t['gps_goal']),('goal_pose',t['rviz_goal']),
                    ('odometry_gps','/odometry/gps'),('motion_allowed','/navigation/motion_allowed')]))
            if rviz:
                actions.append(Node(package='rviz2',executable='rviz2',output='screen',arguments=[
                    '-d',str(Path(share('bunker_gps_nav_nav2'))/'rviz'/'gps_navigation.rviz')],parameters=[{'use_sim_time':sim}]))
        return actions
    return LaunchDescription([
        DeclareLaunchArgument('system_config',default_value=config('bunker_gps_nav_bringup','system.yaml')),
        DeclareLaunchArgument('nav2_config',default_value=config('bunker_gps_nav_nav2','nav2_params.yaml')),
        DeclareLaunchArgument('rviz',default_value='false'),
        OpaqueFunction(function=assemble),
    ])
