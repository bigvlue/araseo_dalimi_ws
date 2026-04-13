#!/usr/bin/env python3
"""
ARASEO-DALIMI Pinky GPS Bridge Node
Host PC가 발행하는 /pinky_{id}/gps_pos (PinkyGps)를 구독해
RobotPose로 변환 후 /robot_{id}/pose 로 발행한다.
마커 미감지(is_valid=False 또는 타임아웃) 시 SOURCE_IMU_ODOM으로 폴백.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from araseo_dalimi_interfaces.msg import PinkyGps, RobotPose


class PinkyGpsBridgeNode(Node):

    SOURCE_GPS_CAMERA = 0
    SOURCE_IMU_ODOM   = 2

    def __init__(self):
        super().__init__('gps_camera_receiver')

        self.declare_parameters('', [
            ('robot_id',         0),
            ('timeout_sec',      0.5),
            ('publish_hz',       5.0),
            ('fallback_enabled', True),
            ('map_x_max_mm',     1880.0),
            ('map_y_max_mm',     1410.0),
        ])

        g = self.get_parameter
        self.robot_id    = g('robot_id').value
        self.timeout_sec = g('timeout_sec').value
        self.publish_hz  = g('publish_hz').value
        self.fallback_en = g('fallback_enabled').value

        # 내부 상태
        self._latest_gps     = None
        self._last_recv_time = 0.0
        self._odom_x_mm      = 0
        self._odom_y_mm      = 0
        self._odom_theta     = 0

        # Best-effort QoS (GPS는 최신값이 중요, 신뢰성보다 지연 최소화)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PinkyGps 구독
        self.create_subscription(
            PinkyGps,
            f'/pinky_{self.robot_id}/gps_pos',
            self._gps_callback,
            sensor_qos
        )

        # 폴백용 Odometry 구독
        if self.fallback_en:
            self.create_subscription(
                Odometry,
                f'/robot_{self.robot_id}/odom',
                self._odom_callback,
                10
            )

        # 발행
        self._pub = self.create_publisher(
            RobotPose,
            f'/robot_{self.robot_id}/pose',
            10
        )

        # 발행 타이머 (GPS 주기와 동일하게 5Hz)
        period = 1.0 / self.publish_hz
        self.create_timer(period, self._publish_pose)

        self.get_logger().info(
            f'[robot_{self.robot_id}] PinkyGPS Bridge 시작 '
            f'(구독: /pinky_{self.robot_id}/gps_pos, '
            f'{self.publish_hz}Hz, fallback={self.fallback_en})'
        )

    # ── PinkyGps 콜백 ─────────────────────────────────────────────

    def _gps_callback(self, msg: PinkyGps):
        # pinky_id 필터
        if msg.pinky_id != self.robot_id:
            return

        # is_valid 필드 지원 (없으면 True로 간주)
        valid = getattr(msg, 'is_valid', True)
        if not valid and msg.confidence == 0:
            # 마커 미인식 → 폴백 처리
            self.get_logger().debug(
                f'[robot_{self.robot_id}] GPS 마커 미인식, 폴백 사용')
            return

        self._latest_gps     = msg
        self._last_recv_time = time.monotonic()

    # ── Odometry 콜백 (폴백) ──────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        self._odom_x_mm = int(msg.pose.pose.position.x * 1000)
        self._odom_y_mm = int(msg.pose.pose.position.y * 1000)
        q   = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._odom_theta = int(yaw * 1000)  # mrad

    # ── 발행 타이머 ───────────────────────────────────────────────

    def _publish_pose(self):
        now     = time.monotonic()
        elapsed = now - self._last_recv_time

        pose = RobotPose()
        pose.stamp_ms = int((time.time() % 1e6) * 1000) & 0xFFFFFFFF
        pose.robot_id = self.robot_id

        if elapsed < self.timeout_sec and self._latest_gps is not None:
            # GPS 카메라 데이터 사용
            gps = self._latest_gps
            pose.x_mm       = int(gps.x_mm)
            pose.y_mm       = int(gps.y_mm)
            # yaw_deg → mrad 변환
            pose.theta_mrad = int(
                math.radians(gps.yaw_deg) * 1000)
            # confidence 필드 없으면 기본값 90
            conf = getattr(gps, 'confidence', 90)
            pose.confidence_pct = conf if conf > 0 else 90
            pose.source = self.SOURCE_GPS_CAMERA

        elif self.fallback_en:
            # 폴백: 오도메트리 사용
            pose.x_mm           = self._odom_x_mm
            pose.y_mm           = self._odom_y_mm
            pose.theta_mrad     = self._odom_theta
            pose.confidence_pct = 30
            pose.source         = self.SOURCE_IMU_ODOM

            if self._last_recv_time > 0:
                self.get_logger().warn(
                    f'[robot_{self.robot_id}] GPS 타임아웃 '
                    f'({elapsed:.2f}s), 오도메트리 폴백 사용',
                    throttle_duration_sec=5.0
                )
        else:
            return

        self._pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PinkyGpsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
