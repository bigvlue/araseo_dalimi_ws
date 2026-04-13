#!/usr/bin/env python3
"""
ARASEO-DALIMI 단일 로봇 런치 파일
전원 켜짐 → 자동 캘리브레이션 → 캘리브레이션 노드 종료 시 main 노드 일괄 기동.
사용법: ros2 launch araseo_bringup robot.launch.py robot_id:=14

하드웨어 전제:
  - Pinky Pro: Dynamixel XL330 × 2 via /dev/ttyAMA4 @ 1Mbps
  - LiDAR: Slamtec (sllidar_ros2 패키지, pinky_bringup에서 기동)
  - 카메라/IR: pinky_start.sh 에서 별도 기동
  - pinky_bringup → bringup_robot.launch.xml 이 모터+IMU+LiDAR 기동
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction,
    RegisterEventHandler, IncludeLaunchDescription
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

# pinky_bringup 설치 여부 확인 (모터+IMU+LiDAR 기동 담당)
try:
    from ament_index_python.packages import get_package_share_directory
    _PINKY_BRINGUP_DIR = get_package_share_directory('pinky_bringup')
    _HAS_PINKY_BRINGUP = True
except Exception:
    _HAS_PINKY_BRINGUP = False
    _PINKY_BRINGUP_DIR = None


def generate_launch_description():

    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='0',
        description='로봇 ID (0~5)')
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Nav2 사용 여부')

    robot_id = LaunchConfiguration('robot_id')
    use_nav2 = LaunchConfiguration('use_nav2')

    lane_config    = PathJoinSubstitution([
        FindPackageShare('araseo_lane_following'),  'config', 'lane_params.yaml'])
    law_config     = PathJoinSubstitution([
        FindPackageShare('araseo_traffic_law'),     'config', 'traffic_law_params.yaml'])
    mission_config = PathJoinSubstitution([
        FindPackageShare('araseo_mission_manager'), 'config', 'mission_params.yaml'])
    ekf_config     = PathJoinSubstitution([
        FindPackageShare('araseo_localization'),    'config', 'ekf.yaml'])
    hud_config     = PathJoinSubstitution([
        FindPackageShare('araseo_hud'),             'config', 'hud_params.yaml'])

    # ── 0. Pinky 하드웨어 bringup (모터 + IMU + LiDAR) ───────
    # pinky_bringup 패키지의 bringup_robot.launch.xml 을 재사용.
    # Dynamixel(/dev/ttyAMA4 @1Mbps), sllidar, IMU 등 모두 기동.
    # 캘리브레이션 중에도 모터가 동작해야 하므로 가장 먼저 실행.
    hw_nodes = []
    if _HAS_PINKY_BRINGUP:
        hw_nodes.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('pinky_bringup'),
                    'launch', 'bringup_robot.launch.xml'
                ])
            ),
            launch_arguments={'use_sim_time': 'false'}.items()
        ))
    else:
        # pinky_bringup 미설치 시 자체 Dynamixel 드라이버로 폴백
        hw_nodes.append(Node(
            package='araseo_bringup',
            executable='dynamixel_driver_node.py',
            name='dynamixel_driver',
            parameters=[{
                'robot_id':         robot_id,
                'port':             '/dev/ttyAMA4',
                'baudrate':         1000000,
                'left_motor_id':    1,
                'right_motor_id':   2,
                'left_direction':   1,
                'right_direction':  -1,
                'wheel_radius_m':   0.032,
                'wheel_track_m':    0.090,
                'cmd_timeout_sec':  0.5,
            }],
            output='screen'
        ))

    # ── 1. 자동 캘리브레이션 노드 (네임스페이스 밖, 단독 실행) ─────
    # /pinky_{id}/gps_pos 직접 구독, /robot_{id}/cmd_vel_safe 직접 발행
    # 완료 시 exit(0) 으로 종료 → OnProcessExit 이 main 노드 기동
    calib_node = Node(
        package='araseo_localization',
        executable='auto_calibration_node.py',
        name='auto_calibration',
        parameters=[{
            'robot_id':            robot_id,
            'gps_timeout_sec':     10.0,
            'min_gps_confidence':  50,
        }],
        output='screen'
    )

    # ── 2. GPS 브릿지 (네임스페이스 밖) ──────────────────────────
    gps_node = Node(
        package='araseo_gps_camera',
        executable='gps_camera_receiver',
        name='gps_camera_receiver',
        parameters=[{
            'robot_id':        robot_id,
            'publish_hz':      5.0,
            'timeout_sec':     0.5,
            'fallback_enabled': True,
        }],
        output='screen'
    )

    # ── 3. 로봇 네임스페이스 내 main 노드들 ──────────────────────
    main_nodes = GroupAction([
        PushRosNamespace(['robot_', robot_id]),

        # 휠 오도메트리 (캘리브레이션 결과 로드 후 적분)
        Node(
            package='araseo_localization',
            executable='wheel_odom_node.py',
            name='wheel_odom',
            parameters=[{
                'robot_id':   robot_id,
                'publish_hz': 20.0,
            }],
            output='screen'
        ),

        # GPS 위치 → Odometry 변환
        Node(
            package='araseo_localization',
            executable='pose_to_odom',
            name='pose_to_odom',
            parameters=[{'robot_id': robot_id}],
            output='screen'
        ),

        # EKF 위치 추정
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

        # 차선 추종
        Node(
            package='araseo_lane_following',
            executable='lane_follower',
            name='lane_follower',
            parameters=[lane_config, {'robot_id': robot_id}],
            output='screen'
        ),

        # 교통법 준수
        Node(
            package='araseo_traffic_law',
            executable='traffic_law_manager',
            name='traffic_law_manager',
            parameters=[law_config, {'robot_id': robot_id}],
            output='screen'
        ),

        # 미션 관리
        Node(
            package='araseo_mission_manager',
            executable='mission_manager',
            name='mission_manager',
            parameters=[
                mission_config,
                {'robot_id': robot_id, 'use_nav2': use_nav2}
            ],
            output='screen'
        ),

        # HUD 오버레이
        Node(
            package='araseo_hud',
            executable='hud_overlay',
            name='hud_overlay',
            parameters=[hud_config, lane_config, {'robot_id': robot_id}],
            output='screen'
        ),
    ])

    # ── 4. 이벤트: 캘리브레이션 완료 → main 노드 기동 ─────────────
    start_main_on_calib_done = RegisterEventHandler(
        OnProcessExit(
            target_action=calib_node,
            on_exit=[gps_node, main_nodes]
        )
    )

    return LaunchDescription([
        robot_id_arg,
        use_nav2_arg,
        *hw_nodes,               # 하드웨어 bringup (모터+IMU+LiDAR)
        calib_node,              # 캘리브레이션 실행
        start_main_on_calib_done # 완료 후 나머지 기동
    ])
