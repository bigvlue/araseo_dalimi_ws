#!/usr/bin/env python3
"""
ARASEO-DALIMI BEV+ROI 캘리브레이션 런치 파일
사용법:

  # 정지 이미지로 캘리브레이션 (가장 쉬움, 권장)
  ros2 launch araseo_bringup bev_calib.launch.py mode:=image image:=/path/to/frame.jpg

  # 실시간 카메라 토픽으로 캘리브레이션
  ros2 launch araseo_bringup bev_calib.launch.py mode:=topic robot_id:=0

  # 저장된 결과 검증
  ros2 launch araseo_bringup bev_calib.launch.py mode:=verify image:=/path/to/frame.jpg

조작 방법 (캘리브레이션 창):
  Phase 1 — BEV 원근 보정
    마우스 좌클릭: 캘리브레이션 패드 꼭짓점 순서대로 클릭 (TL→TR→BR→BL)
    h: HSV 차선 오버레이 ON/OFF
    r: Phase 1 초기화
    n: Phase 2(ROI 선택)로 이동

  Phase 2 — 도로 ROI 다각형
    마우스 좌클릭: ROI 경계점 클릭 (도로 외곽선 따라 시계방향)
    Enter: ROI 확정
    s: lane_params.yaml 저장 (bev_src_pts, bev_dst_pts, roi_pts 갱신)
    b: Phase 1로 복귀
    r: Phase 2 초기화

  공통
    f: 프레임 고정/해제 (실시간 모드에서 특정 프레임 고정)
    q: 종료
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='image',
        description='캘리브레이션 모드: image | topic | verify')
    image_arg = DeclareLaunchArgument(
        'image', default_value='',
        description='정지 이미지 파일 경로 (mode=image 또는 verify 시 필수)')
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='0',
        description='로봇 ID (mode=topic 시 사용)')
    pad_w_arg = DeclareLaunchArgument(
        'pad_w', default_value='200',
        description='캘리브레이션 패드 가로 폭 mm')
    pad_h_arg = DeclareLaunchArgument(
        'pad_h', default_value='150',
        description='캘리브레이션 패드 세로 높이 mm')

    yaml_path = PathJoinSubstitution([
        FindPackageShare('araseo_lane_following'), 'config', 'lane_params.yaml'])

    mode    = LaunchConfiguration('mode')
    image   = LaunchConfiguration('image')
    robot_id = LaunchConfiguration('robot_id')
    pad_w   = LaunchConfiguration('pad_w')
    pad_h   = LaunchConfiguration('pad_h')

    calib_node = Node(
        package='araseo_lane_following',
        executable='bev_calibrator.py',
        name='bev_calibrator',
        arguments=[
            '--mode',     mode,
            '--image',    image,
            '--topic',    ['/robot_', robot_id, '/camera/image_raw'],
            '--verify',   image,
            '--yaml',     yaml_path,
            '--pad-w',    pad_w,
            '--pad-h',    pad_h,
            '--robot-id', robot_id,
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        mode_arg, image_arg, robot_id_arg, pad_w_arg, pad_h_arg,
        calib_node,
    ])
