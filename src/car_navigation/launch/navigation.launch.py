#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_port = LaunchConfiguration('base_port')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    default_map_file = os.path.expanduser(
        '~/Robot_Car/maps/my_map.yaml'
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('car_navigation'),
                'launch',
                'robot_bringup.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'base_port': base_port,
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/ttyS3',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Saved map YAML file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('car_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
        ),
        robot_bringup,
        nav2_bringup,
    ])