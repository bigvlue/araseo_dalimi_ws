#!/usr/bin/env python3
"""
ARASEO-DALIMI Localization 런치 파일
robot_id 인자를 받아 해당 로봇의 EKF + 변환 노드를 실행한다.
사용법: ros2 launch araseo_localization localization.launch.py robot_id:=0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='로봇 ID (0~5)'
    )
    robot_id = LaunchConfiguration('robot_id')

    ekf_config = PathJoinSubstitution([
        FindPackageShare('araseo_localization'),
        'config', 'ekf.yaml'
    ])

    return LaunchDescription([
        robot_id_arg,
        GroupAction([
            PushRosNamespace(['robot_', robot_id]),

            # GPS 카메라 위치 → Odometry 변환 노드
            Node(
                package='araseo_localization',
                executable='pose_to_odom',
                name='pose_to_odom',
                parameters=[{'robot_id': robot_id}],
                output='screen'
            ),

            # EKF 노드 (robot_localization 패키지)
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_config],
                remappings=[
                    ('odometry/filtered', 'ekf_pose'),
                    ('odom',              'odom'),
                    ('imu/data',          '/imu/data'),
                    ('gps_pose_odom',     'gps_pose_odom'),
                ],
                output='screen'
            ),
        ])
    ])
