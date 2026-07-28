#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_port = LaunchConfiguration('base_port')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    configuration_directory = LaunchConfiguration(
        'configuration_directory'
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename'
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

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py',
            ])
        ),
        launch_arguments={
            'params_file': lidar_params_file,
        }.items(),
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '-configuration_directory',
            configuration_directory,
            '-configuration_basename',
            configuration_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
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
            'lidar_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'params',
                'X2.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=PathJoinSubstitution([
                FindPackageShare('car_navigation'),
                'config',
            ]),
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer_2d.lua',
        ),
        robot_bringup,
        lidar,
        cartographer,
        occupancy_grid,
    ])