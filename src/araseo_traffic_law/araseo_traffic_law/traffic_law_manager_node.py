#!/usr/bin/env python3
"""
ARASEO-DALIMI Traffic Law Manager Node
한국 도로교통법 준수 로직:
  - 황색 중앙선 침범 금지
  - 정지선 일시정지 (2초)
  - 교차로 우측 우선
  - 전방 로봇 추종 / 충돌 회피
  - 앞지르기 금지
lane_follower의 cmd_vel을 감시하고 법규 위반 시 보정하여 재발행.
"""
import math
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool
from araseo_dalimi_interfaces.msg import (
    LaneInfo, TrafficSign, FleetStatus, MissionStatus
)


class DrivingState(IntEnum):
    NORMAL           = 0
    APPROACHING_STOP = 1
    STOP_HOLD        = 2
    INTERSECTION     = 3
    YIELD            = 4
    FOLLOWING        = 5
    COLLISION_AVOID  = 6


class TrafficLawManagerNode(Node):

    def __init__(self):
        super().__init__('traffic_law_manager')

        # 파라미터 선언
        self.declare_parameters('', [
            ('robot_id',                    0),
            ('speed_scale_normal',          1.0),
            ('speed_scale_slowdown',        0.5),
            ('speed_scale_intersection',    0.6),
            ('stop_hold_sec',               2.0),
            ('yield_distance_mm',           500),
            ('center_line_warn_threshold',  0.65),
            ('follow_distance_mm',          400),
            ('collision_stop_mm',           200),
            ('lidar_slowdown_mm',   400),
            ('lidar_stop_mm',       100),
            ('lidar_fov_deg',        30),
            ('lidar_max_range_mm',  400),
            ('ultrasonic_stop_mm',  150),
            ('speed_scale_limit',   0.7),
        ])

        g = self.get_parameter
        self.robot_id          = g('robot_id').value
        self.scale_normal      = g('speed_scale_normal').value
        self.scale_slowdown    = g('speed_scale_slowdown').value
        self.scale_intersection= g('speed_scale_intersection').value
        self.stop_hold_sec     = g('stop_hold_sec').value
        self.yield_dist_mm     = g('yield_distance_mm').value
        self.cl_warn_thresh    = g('center_line_warn_threshold').value
        self.follow_dist_mm    = g('follow_distance_mm').value
        self.collision_mm      = g('collision_stop_mm').value
        self.lidar_slowdown_mm  = g('lidar_slowdown_mm').value
        self.lidar_stop_mm      = g('lidar_stop_mm').value
        self.lidar_fov_rad      = math.radians(g('lidar_fov_deg').value / 2.0)
        self.lidar_max_m        = g('lidar_max_range_mm').value / 1000.0
        self.ultrasonic_stop_mm = g('ultrasonic_stop_mm').value
        self.scale_limit       = g('speed_scale_limit').value
        self._speed_limited    = False

        # 상태 머신
        self._state          = DrivingState.NORMAL
        self._stop_hold_time = 0.0
        self._intersection_cleared = False

        # 최신 센서 데이터 캐시
        self._lane_info    = None
        self._latest_sign  = None
        self._fleet        = None
        self._my_x_mm      = 0
        self._my_y_mm      = 0
        self._my_theta     = 0

        # 센서 캐시
        self._lidar_min_front_m   = float('inf')   # 전방 최근접 LiDAR 거리 (m)
        self._lidar_stamp         = 0.0
        self._ultrasonic_m        = float('inf')   # 초음파 거리 (m)
        self._ultrasonic_stamp    = 0.0

        # 구독
        self.create_subscription(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel',
            self._cmd_vel_callback, 10)
        self.create_subscription(
            LaneInfo,
            f'/robot_{self.robot_id}/lane_info',
            self._lane_info_callback, 10)
        self.create_subscription(
            TrafficSign,
            f'/robot_{self.robot_id}/traffic_sign',
            self._traffic_sign_callback, 10)
        self.create_subscription(
            FleetStatus,
            '/fleet/status',
            self._fleet_callback, 10)
        self.create_subscription(
            LaserScan,
            f'/robot_{self.robot_id}/scan',
            self._lidar_callback, 10)
        self.create_subscription(
            Range,
            f'/robot_{self.robot_id}/ultrasonic',
            self._ultrasonic_callback, 10)

        # 발행
        self._safe_pub = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel_safe', 10)
        self._resume_pub = self.create_publisher(
            Bool, f'/robot_{self.robot_id}/lane_follower/resume', 10)

        self.get_logger().info(
            f'[robot_{self.robot_id}] TrafficLawManager 시작 '
            f'(상태: NORMAL)'
        )

    # ── 구독 콜백 ─────────────────────────────────────────────────

    def _lane_info_callback(self, msg: LaneInfo):
        self._lane_info = msg

    def _traffic_sign_callback(self, msg: TrafficSign):
        self._latest_sign = msg
        if msg.sign_type == 3:   # SPEED_LIMIT
            self._speed_limited = True
            self.get_logger().info(
                f'[robot_{self.robot_id}] 속도제한 표지 감지 '
                f'(신뢰도={msg.confidence_pct}%)')
        elif msg.sign_type == 0:
            self._speed_limited = False

    def _fleet_callback(self, msg: FleetStatus):
        self._fleet = msg
        # 내 위치 업데이트
        for i, rid in enumerate(msg.robot_ids):
            if rid == self.robot_id:
                self._my_x_mm  = msg.x_mm[i]
                self._my_y_mm  = msg.y_mm[i]
                self._my_theta = msg.theta_mrad[i]
                break

    def _lidar_callback(self, msg: LaserScan):
        """
        전방 ±lidar_fov_rad 섹터 내 최단 거리를 추출한다.
        lidar_max_m 초과 값은 맵 외부(벽·가구)로 간주하여 제외.
        """
        import time
        min_dist = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if (abs(angle) <= self.lidar_fov_rad
                    and msg.range_min < r < min(msg.range_max, self.lidar_max_m)):
                min_dist = min(min_dist, r)
            angle += msg.angle_increment
        self._lidar_min_front_m = min_dist
        self._lidar_stamp = time.monotonic()

    def _ultrasonic_callback(self, msg: Range):
        """US-016 초음파 거리 수신 (sensor_msgs/Range.range 단위: m)"""
        import time
        if msg.min_range < msg.range < msg.max_range:
            self._ultrasonic_m = msg.range
        else:
            self._ultrasonic_m = float('inf')
        self._ultrasonic_stamp = time.monotonic()

    def _sensor_fresh(self, stamp: float, max_age: float = 0.3) -> bool:
        """센서 데이터 신선도 확인 (max_age초 이내)"""
        import time
        return (time.monotonic() - stamp) < max_age

    def _effective_threshold(self, base_mm: float, conf: float) -> float:
        """
        신뢰도(0~100)에 따라 감지 임계값(mm)을 조정한다.
        신뢰도 낮을수록 임계값 증가 → 더 가까워야 반응 (보수적).
        최솟값 30% 보장으로 센서 완전 비활성화 방지.
        """
        conf_clamped = max(30.0, min(100.0, conf))
        return base_mm / (conf_clamped / 100.0)

    def _lidar_confidence(self) -> float:
        """
        LiDAR 신뢰도 계산 (30~95).
        신선도 지수 감쇠 × 전방 유효 포인트 밀도.
        """
        import time as _t, math as _m
        age          = _t.monotonic() - self._lidar_stamp
        staleness    = _m.exp(-age / 0.5)           # tau=0.5s
        base_conf    = 90.0
        conf         = base_conf * staleness
        return float(max(30.0, conf))

    def _ultrasonic_confidence(self) -> float:
        """
        초음파 신뢰도 계산 (30~80).
        초음파는 빔 폭이 넓고 반사각 영향을 받으므로
        기본 신뢰도를 LiDAR보다 낮게 설정.
        신선도에 따른 지수 감쇠 적용.
        """
        import time as _t, math as _m
        age       = _t.monotonic() - self._ultrasonic_stamp
        staleness = _m.exp(-age / 0.5)
        base_conf = 75.0
        conf      = base_conf * staleness
        return float(max(30.0, conf))

    # ── 메인 제어 콜백 ────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        """lane_follower의 cmd_vel을 수신해 법규 적용 후 재발행"""
        safe_cmd = Twist()
        safe_cmd.linear.x  = msg.linear.x
        safe_cmd.angular.z = msg.angular.z

        # ── 상태 전이 결정 ────────────────────────────────────────
        self._update_state()

        # ── 상태별 제어 적용 ──────────────────────────────────────
        state = self._state

        if state == DrivingState.NORMAL:
            safe_cmd = self._apply_center_line_guard(safe_cmd)
            if self._speed_limited:
                safe_cmd.linear.x *= self.scale_limit
            # 쿨다운 직후(정지선 탈출 중): 최소 전진 속도 보장
            if hasattr(self, '_stop_clear_time') and (time.monotonic() - self._stop_clear_time) < 3.0:
                if safe_cmd.linear.x < 0.08:
                    safe_cmd.linear.x = 0.08

        elif state == DrivingState.APPROACHING_STOP:
            safe_cmd.linear.x  *= self.scale_slowdown
            safe_cmd = self._apply_center_line_guard(safe_cmd)

        elif state == DrivingState.STOP_HOLD:
            safe_cmd.linear.x  = 0.0
            safe_cmd.angular.z = 0.0

        elif state == DrivingState.INTERSECTION:
            if self._should_yield():
                safe_cmd.linear.x  = 0.0
                safe_cmd.angular.z = 0.0
                self._state = DrivingState.YIELD
            else:
                # 정지선 통과 직후: lane_follower가 아직 resume 처리 전이면
                # cmd_vel=0 이므로 최소 전진 속도를 강제 부여하여 정지선 탈출
                base = msg.linear.x if msg.linear.x > 0.01 else (self.scale_normal * 0.15)
                safe_cmd.linear.x = max(base * self.scale_intersection, 0.08)

        elif state == DrivingState.YIELD:
            safe_cmd.linear.x  = 0.0
            safe_cmd.angular.z = 0.0
            # 우측 로봇이 지나가면 재개
            if not self._should_yield():
                self._state = DrivingState.INTERSECTION

        elif state == DrivingState.FOLLOWING:
            # 전방 로봇 속도에 맞춤
            front_speed = self._get_front_robot_speed()
            safe_cmd.linear.x = min(safe_cmd.linear.x, front_speed)

        elif state == DrivingState.COLLISION_AVOID:
            safe_cmd.linear.x  = 0.0
            safe_cmd.angular.z = 0.0

        self._safe_pub.publish(safe_cmd)

    # ── 상태 전이 ─────────────────────────────────────────────────

    def _update_state(self):
        now = time.monotonic()

        # 충돌 회피 우선 (최고 우선순위)
        if self._check_collision():
            if self._state != DrivingState.COLLISION_AVOID:
                self._state = DrivingState.COLLISION_AVOID
                self.get_logger().warn(
                    f'[robot_{self.robot_id}] 충돌 위험! COLLISION_AVOID')
            return
        # 충돌 해제 → NORMAL 복귀
        if self._state == DrivingState.COLLISION_AVOID:
            self._state = DrivingState.NORMAL
            self.get_logger().info(
                f'[robot_{self.robot_id}] 충돌 해제 → NORMAL')

        # STOP_HOLD 타이머
        if self._state == DrivingState.STOP_HOLD:
            elapsed = now - self._stop_hold_time
            if elapsed >= self.stop_hold_sec:
                # 최소 대기 시간 경과 후, 교차로 안전 확인
                if not self._should_yield():
                    resume_msg = Bool()
                    resume_msg.data = True
                    self._resume_pub.publish(resume_msg)
                    self._state = DrivingState.INTERSECTION
                    self.get_logger().info(
                        f'[robot_{self.robot_id}] 일시정지 완료 + 안전확인 → INTERSECTION')
                else:
                    # 최대 대기 시간(10초) 초과 시 강제 재출발
                    if elapsed >= self.stop_hold_sec * 5.0:
                        resume_msg = Bool()
                        resume_msg.data = True
                        self._resume_pub.publish(resume_msg)
                        self._state = DrivingState.INTERSECTION
                        self.get_logger().warn(
                            f'[robot_{self.robot_id}] 최대 대기 초과 → 강제 재출발')
            return

        # 정지선 감지 → APPROACHING_STOP (NORMAL 및 FOLLOWING 상태 모두에서 전이 허용)
        if (self._latest_sign is not None and
                self._latest_sign.sign_type == 1 and  # STOP_LINE
                self._state in (DrivingState.NORMAL, DrivingState.FOLLOWING)):
            # 신뢰도가 낮은 정지선 신호는 감속만, 높은 신뢰도만 완전 정지 준비
            conf = self._latest_sign.confidence_pct
            # 쿨다운: 직전 정지선 통과 후 일정 시간 내 재감지 무시
            if now - getattr(self, '_stop_clear_time', 0.0) < 5.0:
                self._latest_sign = None
                return
            if conf >= 60:
                self._state = DrivingState.APPROACHING_STOP
                self._latest_sign = None   # 소비 후 초기화
                self.get_logger().info(
                    f'[robot_{self.robot_id}] 정지선 감지 '
                    f'(신뢰도={conf}%) → APPROACHING_STOP')
            else:
                # 신뢰도 낮음: 속도만 줄이고 완전 정지 유보
                self.get_logger().debug(
                    f'[robot_{self.robot_id}] 정지선 저신뢰도 '
                    f'({conf}%) → 감속만 적용')
            return

        # 정지선 도달 판단 (속도 거의 0 + 정지선 플래그)
        if self._state == DrivingState.APPROACHING_STOP:
            if self._at_stop_line():
                self._state          = DrivingState.STOP_HOLD
                self._stop_hold_time = now
                self.get_logger().info(
                    f'[robot_{self.robot_id}] 정지선 도달 → STOP_HOLD 2초')
            return

        # 교차로 감지
        if (self._lane_info is not None and
                bool(self._lane_info.flags & 4) and  # FLAG_INTERSECTION
                self._state == DrivingState.NORMAL):
            self._state = DrivingState.INTERSECTION
            self.get_logger().info(
                f'[robot_{self.robot_id}] 교차로 진입 → INTERSECTION')
            return

        # 교차로 통과 완료
        if (self._state == DrivingState.INTERSECTION and
                self._lane_info is not None and
                not bool(self._lane_info.flags & 4)):
            self._state = DrivingState.NORMAL
            self._stop_clear_time = now   # 정지선 재감지 쿨다운 시작
            self._latest_sign = None      # 잔여 정지선 신호 초기화
            self.get_logger().info(
                f'[robot_{self.robot_id}] 교차로 통과 → NORMAL')
            return

        # 전방 로봇 추종
        if self._state == DrivingState.NORMAL:
            if self._check_front_robot():
                self._state = DrivingState.FOLLOWING
                return
        if self._state == DrivingState.FOLLOWING:
            if not self._check_front_robot():
                self._state = DrivingState.NORMAL

    # ── 황색 중앙선 보호 ──────────────────────────────────────────

    def _apply_center_line_guard(self, cmd: Twist) -> Twist:
        """
        황색 중앙선 침범 감지 시 우측 보정 (도로교통법: 중앙선 침범 금지)
        lateral_error < -threshold 이면 좌측 편향 → 우측으로 강제 보정
        """
        if self._lane_info is None:
            return cmd
        lateral = self._lane_info.lateral_error_norm / 127.0  # int8 → float
        if lateral < -self.cl_warn_thresh:
            # 좌측 편향이 심함 → 우측 조향 강제
            correction = abs(lateral) * 0.3
            cmd.angular.z += correction
            self.get_logger().debug(
                f'[robot_{self.robot_id}] 중앙선 보정: {correction:.3f} rad/s')
        return cmd

    # ── 교차로 우선순위 판단 ──────────────────────────────────────

    def _should_yield(self) -> bool:
        """
        우측 도로에서 진입하는 로봇이 있으면 양보
        (도로교통법 제26조: 우측 차량 우선)
        """
        if self._fleet is None:
            return False
        my_theta = self._my_theta * 0.001  # mrad → rad
        # 우측 방향 = 내 진행 방향 - 90도
        right_angle = my_theta - math.pi / 2.0

        for i, rid in enumerate(self._fleet.robot_ids):
            if rid == self.robot_id:
                continue
            dx = self._fleet.x_mm[i] - self._my_x_mm
            dy = self._fleet.y_mm[i] - self._my_y_mm
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > self.yield_dist_mm:
                continue
            # 해당 로봇이 우측 방향에 있는지 각도로 판단
            angle_to = math.atan2(dy, dx)
            angle_diff = abs(
                math.atan2(
                    math.sin(angle_to - right_angle),
                    math.cos(angle_to - right_angle)
                )
            )
            if angle_diff < math.pi / 4.0:  # 45도 이내
                return True
        return False

    # ── 전방 로봇 충돌 감지 ───────────────────────────────────────

    def _check_collision(self) -> bool:
        """
        3중 충돌 감지 융합 (OR 로직):
          1. FleetStatus — 다른 로봇 (전방 ±45°, collision_stop_mm 이내)
          2. RPLiDAR C1  — 정적 장애물 (전방 ±fov, lidar_stop_mm 이내)
          3. US-016 초음파 — 근거리 failsafe (ultrasonic_stop_mm 이내)
        """
        # 1. FleetStatus 기반 (기존 로직 유지)
        if self._fleet is not None:
            my_theta = self._my_theta * 0.001
            for i, rid in enumerate(self._fleet.robot_ids):
                if rid == self.robot_id:
                    continue
                dx = self._fleet.x_mm[i] - self._my_x_mm
                dy = self._fleet.y_mm[i] - self._my_y_mm
                dist = math.sqrt(dx * dx + dy * dy)
                if dist >= self.collision_mm:
                    continue
                angle_to = math.atan2(dy, dx)
                angle_diff = abs(math.atan2(
                    math.sin(angle_to - my_theta),
                    math.cos(angle_to - my_theta)))
                if angle_diff < math.pi / 4.0:
                    self.get_logger().debug(
                        f'[robot_{self.robot_id}] 충돌감지: FleetStatus '
                        f'dist={dist:.0f}mm')
                    return True

        # 2. LiDAR 기반 — 신뢰도 반영 유효 임계값 적용
        lidar_conf    = self._lidar_confidence()
        eff_lidar_mm  = self._effective_threshold(self.lidar_stop_mm, lidar_conf)
        if (self._sensor_fresh(self._lidar_stamp) and
                self._lidar_min_front_m * 1000 < eff_lidar_mm):
            self.get_logger().warn(
                f'[robot_{self.robot_id}] 충돌감지: LiDAR '
                f'{self._lidar_min_front_m*1000:.0f}mm '
                f'(유효임계={eff_lidar_mm:.0f}mm, 신뢰도={lidar_conf:.0f}%)')
            return True

        # 3. 초음파 기반 — 신뢰도 반영 유효 임계값 적용
        us_conf       = self._ultrasonic_confidence()
        eff_us_mm     = self._effective_threshold(self.ultrasonic_stop_mm, us_conf)
        if (self._sensor_fresh(self._ultrasonic_stamp) and
                self._ultrasonic_m * 1000 < eff_us_mm):
            self.get_logger().warn(
                f'[robot_{self.robot_id}] 충돌감지: 초음파 '
                f'{self._ultrasonic_m*1000:.0f}mm '
                f'(유효임계={eff_us_mm:.0f}mm, 신뢰도={us_conf:.0f}%)')
            return True

        return False

    def _check_front_robot(self) -> bool:
        """
        전방 추종 판단:
          FleetStatus 전방 로봇 OR LiDAR 감속 구간 내 장애물
        → FOLLOWING 상태로 전환해 속도 감소
        """
        my_theta = self._my_theta * 0.001

        # FleetStatus 전방 로봇
        if self._fleet is not None:
            for i, rid in enumerate(self._fleet.robot_ids):
                if rid == self.robot_id:
                    continue
                dx = self._fleet.x_mm[i] - self._my_x_mm
                dy = self._fleet.y_mm[i] - self._my_y_mm
                dist = math.sqrt(dx * dx + dy * dy)
                if dist > self.follow_dist_mm:
                    continue
                angle_to = math.atan2(dy, dx)
                angle_diff = abs(math.atan2(
                    math.sin(angle_to - my_theta),
                    math.cos(angle_to - my_theta)))
                if angle_diff < math.pi / 6.0:
                    return True

        # LiDAR 감속 구간 장애물 — 신뢰도 반영
        lidar_conf       = self._lidar_confidence()
        eff_slowdown_mm  = self._effective_threshold(
            self.lidar_slowdown_mm, lidar_conf)
        eff_stop_mm      = self._effective_threshold(
            self.lidar_stop_mm, lidar_conf)
        lidar_dist_mm    = self._lidar_min_front_m * 1000
        if (self._sensor_fresh(self._lidar_stamp) and
                eff_stop_mm <= lidar_dist_mm < eff_slowdown_mm):
            self.get_logger().debug(
                f'[robot_{self.robot_id}] LiDAR 감속구간: '
                f'{lidar_dist_mm:.0f}mm')
            return True

        return False

    def _get_front_robot_speed(self) -> float:
        """
        전방 로봇의 실제 상태를 FleetStatus에서 읽어 속도 추정.
        STATE_STOP(2) 또는 STATE_ERROR(3)이면 0 반환.
        STATE_RUNNING(1)이면 base_speed의 80% 반환.
        FleetStatus 없으면 보수적으로 0 반환.
        """
        if self._fleet is None:
            return 0.0
        my_theta = self._my_theta * 0.001
        for i, rid in enumerate(self._fleet.robot_ids):
            if rid == self.robot_id:
                continue
            dx = self._fleet.x_mm[i] - self._my_x_mm
            dy = self._fleet.y_mm[i] - self._my_y_mm
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.follow_dist_mm:
                continue
            angle_to = math.atan2(dy, dx)
            angle_diff = abs(math.atan2(
                math.sin(angle_to - my_theta),
                math.cos(angle_to - my_theta)))
            if angle_diff < math.pi / 6.0:
                state = self._fleet.states[i] if i < len(self._fleet.states) else 0
                if state in (2, 3):   # STATE_STOP, STATE_ERROR
                    return 0.0
                return self.scale_normal * 0.12 * 0.8   # STATE_RUNNING → 80%
        return self.scale_normal * 0.12

    def _at_stop_line(self) -> bool:
        """정지선 도달 판단: LaneInfo stop_line 플래그 + 근접"""
        if self._lane_info is None:
            return False
        return bool(self._lane_info.flags & 2)  # FLAG_STOP_LINE


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLawManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
