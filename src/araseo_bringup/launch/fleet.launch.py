#!/usr/bin/env python3
"""
ARASEO-DALIMI 6대 Fleet 런치 파일
사용법: ros2 launch araseo_bringup fleet.launch.py
특정 대수만: ros2 launch araseo_bringup fleet.launch.py robot_count:=2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_count_arg = DeclareLaunchArgument(
        'robot_count', default_value='6',
        description='운용할 로봇 대수 (1~6)')
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true')

    robot_launch = PathJoinSubstitution([
        FindPackageShare('araseo_bringup'),
        'launch', 'robot.launch.py'])

    actions = [robot_count_arg, use_nav2_arg]

    # 로봇 6대 런치 (robot_id 0~5)
    for rid in range(6):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    'robot_id': str(rid),
                    'use_nav2': LaunchConfiguration('use_nav2'),
                }.items()
            )
        )

    return LaunchDescription(actions)
