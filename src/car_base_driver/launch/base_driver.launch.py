#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('car_base_driver'),
        'config',
        'base_driver.yaml',
    ])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyS3',
    )

    driver = Node(
        package='car_base_driver',
        executable='serial_driver',
        name='car_base_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'port': LaunchConfiguration('port')},
        ],
    )

    return LaunchDescription([
        params_arg,
        port_arg,
        driver,
    ])