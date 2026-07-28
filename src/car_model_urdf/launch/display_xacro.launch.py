#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution


def generate_launch_description():
    model = PathJoinSubstitution([
        FindPackageShare('car_model_urdf'),
        'urdf',
        'car_model.urdf.xacro',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('car_model_urdf'),
        'config',
        'display_model.rviz',
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model]),
        value_type=str,
    )

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        )
    ])