#!/usr/bin/env python3
"""
ARASEO-DALIMI Auto Calibration Node
전원 켜짐 시 자동 실행 — 직진 20cm + 복귀 + 360° 회전으로
velocity_scale / angular_scale 을 측정, ~/.ros/araseo_calibration.yaml 에 저장.
완료 후 exit(0) 으로 종료 → robot.launch.py 가 main 노드들을 이어서 기동.
"""
import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from araseo_dalimi_interfaces.msg import PinkyGps

CALIB_FILE      = os.path.expanduser('~/.ros/araseo_calibration.yaml')
STRAIGHT_VEL    = 0.10          # m/s
STRAIGHT_TIME   = 2.0           # s  →  약 20 cm
REVERSE_TIME    = 2.2           # s  (정지 지연 보정을 위해 0.2 s 더)
ROTATE_VEL      = 0.50          # rad/s
ROTATE_TIME     = 2.0 * math.pi / ROTATE_VEL   # ≈ 12.57 s  (360°)
PAUSE_TIME      = 0.8           # s  (각 페이즈 사이 정지 대기)


class S:   # 상태 상수
    WAIT_GPS = 0
    STRAIGHT = 1
    PAUSE1   = 2
    REVERSE  = 3
    PAUSE2   = 4
    ROTATE   = 5


class AutoCalibrationNode(Node):

    def __init__(self):
        super().__init__('auto_calibration')

        self.declare_parameters('', [
            ('robot_id',            0),
            ('gps_timeout_sec',    10.0),
            ('min_gps_confidence', 50),
        ])
        g = self.get_parameter
        self.robot_id     = g('robot_id').value
        self.gps_timeout  = g('gps_timeout_sec').value
        self.min_conf     = g('min_gps_confidence').value

        # 내부 상태
        self._state          = S.WAIT_GPS
        self._phase_start    = time.monotonic()
        self._latest_gps     = None
        self._gps_valid      = False

        # 직진 측정값
        self._straight_start = None   # (x_mm, y_mm)
        self._straight_end   = None

        # 회전 측정값
        self._rotate_prev_yaw  = None
        self._rotate_yaw_accum = 0.0   # 누적 회전 (deg)

        self.create_subscription(
            PinkyGps,
            f'/pinky_{self.robot_id}/gps_pos',
            self._gps_cb, 10)

        self._cmd = self.create_publisher(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel_safe',
            10)

        self.create_timer(0.05, self._loop)   # 20 Hz

        self.get_logger().info(
            f'[robot_{self.robot_id}] 자동 캘리브레이션 — GPS 대기 중...')

    # ── GPS 콜백 ──────────────────────────────────────────────────

    def _gps_cb(self, msg: PinkyGps):
        if msg.pinky_id != self.robot_id:
            return
        valid = getattr(msg, 'is_valid', True)
        conf  = getattr(msg, 'confidence', 90)
        if valid and conf >= self.min_conf:
            self._latest_gps = msg
            self._gps_valid  = True

    # ── 속도 발행 유틸 ────────────────────────────────────────────

    def _vel(self, vx: float, vyaw: float):
        t = Twist()
        t.linear.x  = float(vx)
        t.angular.z = float(vyaw)
        self._cmd.publish(t)

    def _stop(self):
        self._vel(0.0, 0.0)

    # ── 20 Hz 메인 루프 ───────────────────────────────────────────

    def _loop(self):
        now     = time.monotonic()
        elapsed = now - self._phase_start

        # WAIT_GPS ─────────────────────────────────────────────────
        if self._state == S.WAIT_GPS:
            if self._gps_valid and self._latest_gps is not None:
                self._straight_start = (self._latest_gps.x_mm,
                                        self._latest_gps.y_mm)
                self.get_logger().info(
                    f'GPS 확인 (x={self._latest_gps.x_mm:.0f}mm, '
                    f'y={self._latest_gps.y_mm:.0f}mm) — 직진 시작')
                self._state = S.STRAIGHT
                self._phase_start = now
            elif elapsed > self.gps_timeout:
                self.get_logger().warn(
                    f'GPS 신호 없음 ({self.gps_timeout}s 초과) '
                    f'— 기본값(1.0/1.0)으로 저장 후 종료')
                self._save(1.0, 1.0)
                self._finish()
            return

        # STRAIGHT ─────────────────────────────────────────────────
        if self._state == S.STRAIGHT:
            if elapsed < STRAIGHT_TIME:
                self._vel(STRAIGHT_VEL, 0.0)
            else:
                self._stop()
                if self._latest_gps is not None:
                    self._straight_end = (self._latest_gps.x_mm,
                                          self._latest_gps.y_mm)
                self._state = S.PAUSE1
                self._phase_start = now
            return

        # PAUSE1 ────────────────────────────────────────────────────
        if self._state == S.PAUSE1:
            self._stop()
            if elapsed >= PAUSE_TIME:
                self._state = S.REVERSE
                self._phase_start = now
            return

        # REVERSE ───────────────────────────────────────────────────
        if self._state == S.REVERSE:
            if elapsed < REVERSE_TIME:
                self._vel(-STRAIGHT_VEL, 0.0)
            else:
                self._stop()
                self._state = S.PAUSE2
                self._phase_start = now
            return

        # PAUSE2 ────────────────────────────────────────────────────
        if self._state == S.PAUSE2:
            self._stop()
            if elapsed >= PAUSE_TIME:
                if self._latest_gps is not None:
                    self._rotate_prev_yaw  = self._latest_gps.yaw_deg
                    self._rotate_yaw_accum = 0.0
                self.get_logger().info('360° 회전 캘리브레이션 시작')
                self._state = S.ROTATE
                self._phase_start = now
            return

        # ROTATE ────────────────────────────────────────────────────
        if self._state == S.ROTATE:
            if elapsed < ROTATE_TIME:
                self._vel(0.0, ROTATE_VEL)
                # GPS yaw 누적 (±180° 래핑 처리)
                if (self._latest_gps is not None
                        and self._rotate_prev_yaw is not None):
                    cur  = self._latest_gps.yaw_deg
                    diff = cur - self._rotate_prev_yaw
                    if diff >  180.0: diff -= 360.0
                    if diff < -180.0: diff += 360.0
                    self._rotate_yaw_accum += diff
                    self._rotate_prev_yaw   = cur
            else:
                self._stop()
                self._compute_and_finish()
            return

    # ── 계산 및 저장 ──────────────────────────────────────────────

    def _compute_and_finish(self):
        # velocity_scale 계산
        vel_scale = 1.0
        if self._straight_start and self._straight_end:
            x0, y0 = self._straight_start
            x1, y1 = self._straight_end
            d_gps = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)  # mm
            d_cmd = STRAIGHT_VEL * STRAIGHT_TIME * 1000.0          # mm
            if d_cmd > 10.0 and d_gps > 5.0:
                vel_scale = max(0.5, min(2.0, d_gps / d_cmd))
                self.get_logger().info(
                    f'직진: GPS={d_gps:.1f}mm, CMD={d_cmd:.1f}mm '
                    f'→ velocity_scale={vel_scale:.4f}')
            else:
                self.get_logger().warn('직진 이동량 부족 — velocity_scale=1.0 사용')

        # angular_scale 계산
        ang_scale = 1.0
        if abs(self._rotate_yaw_accum) > 10.0:
            actual_rad = math.radians(abs(self._rotate_yaw_accum))
            cmd_rad    = ROTATE_VEL * ROTATE_TIME   # ≈ 2π
            ang_scale  = max(0.5, min(2.0, actual_rad / cmd_rad))
            self.get_logger().info(
                f'회전: GPS={math.degrees(actual_rad):.1f}°, '
                f'CMD={math.degrees(cmd_rad):.1f}° '
                f'→ angular_scale={ang_scale:.4f}')
        else:
            self.get_logger().warn('회전 측정값 부족 — angular_scale=1.0 사용')

        self._save(vel_scale, ang_scale)
        self._finish()

    def _save(self, vel_scale: float, ang_scale: float):
        data = {
            'wheel_odom': {
                'ros__parameters': {
                    'velocity_scale': round(float(vel_scale), 6),
                    'angular_scale':  round(float(ang_scale), 6),
                }
            }
        }
        os.makedirs(os.path.dirname(CALIB_FILE), exist_ok=True)
        with open(CALIB_FILE, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(
            f'캘리브레이션 저장: {CALIB_FILE}\n'
            f'  velocity_scale = {vel_scale:.4f}\n'
            f'  angular_scale  = {ang_scale:.4f}')

    def _finish(self):
        self._stop()
        self.get_logger().info(
            '캘리브레이션 완료 — 노드 종료 (main 노드 기동 대기)')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
