"""Run localization, perception and navigation together."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = Path(get_package_share_directory('bunker_gps_nav_bringup'))
    nav2 = Path(get_package_share_directory('bunker_gps_nav_nav2'))
    system_config = LaunchConfiguration('system_config')
    nav2_config = LaunchConfiguration('nav2_config')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config', default_value=str(bringup / 'config' / 'system.yaml')),
        DeclareLaunchArgument(
            'nav2_config', default_value=str(nav2 / 'config' / 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'localization.launch.py')),
            launch_arguments={'system_config': system_config}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'perception.launch.py')),
            launch_arguments={'system_config': system_config}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'navigation.launch.py')),
            launch_arguments={
                'system_config': system_config,
                'nav2_config': nav2_config,
                'rviz': rviz,
            }.items()),
    ])
