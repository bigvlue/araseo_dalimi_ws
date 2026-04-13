#!/usr/bin/env python3
"""
ARASEO-DALIMI Dynamixel Driver Node
XL330-M288-T (Protocol 2.0) 차동구동 드라이버
/robot_{id}/cmd_vel_safe (Twist) 수신 → 좌우 바퀴 속도 명령 전송
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from dynamixel_sdk import (
        PortHandler, PacketHandler, COMM_SUCCESS
    )
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False


# ── XL330 컨트롤 테이블 주소 ──────────────────────────────────────
ADDR_OPERATING_MODE   = 11   # 1 byte  (EEPROM)
ADDR_TORQUE_ENABLE    = 64   # 1 byte  (RAM)
ADDR_GOAL_VELOCITY    = 104  # 4 bytes (RAM, signed int32)
ADDR_PRESENT_VELOCITY = 128  # 4 bytes (RAM, signed int32)

VELOCITY_CONTROL_MODE = 1    # Operating Mode = Velocity Control
TORQUE_ON             = 1
TORQUE_OFF            = 0
PROTOCOL_VERSION      = 2.0
VELOCITY_UNIT_RPM     = 0.229   # 1 LSB = 0.229 RPM (XL330)
MAX_RAW_VEL           = 1023    # XL330 최대 속도 raw값 (≈ 234 RPM)


class DynamixelDriverNode(Node):

    def __init__(self):
        super().__init__('dynamixel_driver')

        self.declare_parameters('', [
            ('robot_id',         0),
            ('port',             '/dev/ttyAMA4'),   # Pinky Pro UART
            ('baudrate',         1000000),          # XL330 기본 1Mbps
            ('left_motor_id',    1),
            ('right_motor_id',   2),
            ('wheel_radius_m',   0.032),
            ('wheel_track_m',    0.090),
            # 모터 장착 방향: 1=정방향, -1=역방향
            # 오른쪽 바퀴가 반대로 장착된 경우 right_direction=-1
            ('left_direction',   1),
            ('right_direction',  -1),
            ('cmd_timeout_sec',  0.5),    # 이 시간 이상 명령 없으면 정지
        ])

        g = self.get_parameter
        self.robot_id       = g('robot_id').value
        self.port_name      = g('port').value
        self.baudrate       = g('baudrate').value
        self.left_id        = g('left_motor_id').value
        self.right_id       = g('right_motor_id').value
        self.wheel_r        = g('wheel_radius_m').value
        self.wheel_t        = g('wheel_track_m').value
        self.left_dir       = g('left_direction').value
        self.right_dir      = g('right_direction').value
        self.cmd_timeout    = g('cmd_timeout_sec').value

        # Dynamixel SDK 초기화
        self._dxl_ok = False
        if SDK_AVAILABLE:
            self._init_dynamixel()
        else:
            self.get_logger().error(
                'dynamixel_sdk 미설치: pip install dynamixel-sdk')

        # cmd_vel_safe 구독
        self.create_subscription(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel_safe',
            self._cmd_cb, 10)

        # 명령 타임아웃 감시 (10Hz)
        import time
        self._last_cmd_time = time.monotonic()
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'[robot_{self.robot_id}] DynamixelDriver 시작 '
            f'(port={self.port_name}, '
            f'IDs: L={self.left_id}/R={self.right_id})')

    # ── Dynamixel 초기화 ──────────────────────────────────────────

    def _init_dynamixel(self):
        try:
            self._port   = PortHandler(self.port_name)
            self._packet = PacketHandler(PROTOCOL_VERSION)

            if not self._port.openPort():
                self.get_logger().error(f'포트 열기 실패: {self.port_name}')
                return
            if not self._port.setBaudRate(self.baudrate):
                self.get_logger().error(f'보드레이트 설정 실패: {self.baudrate}')
                return
        except Exception as e:
            self.get_logger().error(
                f'Dynamixel 포트 초기화 예외: {e} — 모터 비활성 상태로 계속')
            return

        # 두 모터 속도 제어 모드 설정 + 토크 활성화
        for dxl_id in (self.left_id, self.right_id):
            # 토크 OFF → Operating Mode 변경 → 토크 ON
            self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
            self._write1(dxl_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
            self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON)
            self.get_logger().info(f'  모터 ID={dxl_id} 초기화 완료')

        self._dxl_ok = True

    def _write1(self, dxl_id, address, value):
        res, err = self._packet.write1ByteTxRx(
            self._port, dxl_id, address, value)
        if res != COMM_SUCCESS:
            self.get_logger().warn(
                f'write1 실패 ID={dxl_id} addr={address}: '
                f'{self._packet.getTxRxResult(res)}')

    def _write4(self, dxl_id, address, value):
        res, err = self._packet.write4ByteTxRx(
            self._port, dxl_id, address, value)
        if res != COMM_SUCCESS:
            self.get_logger().warn(
                f'write4 실패 ID={dxl_id} addr={address}: '
                f'{self._packet.getTxRxResult(res)}')

    # ── 속도 변환 ─────────────────────────────────────────────────

    def _twist_to_raw(self, linear_x: float, angular_z: float):
        """
        cmd_vel (m/s, rad/s) → 좌/우 바퀴 Dynamixel raw 속도값
        """
        # 차동구동 역기구학
        v_left  = linear_x - angular_z * (self.wheel_t / 2.0)
        v_right = linear_x + angular_z * (self.wheel_t / 2.0)

        # m/s → RPM
        rpm_per_mps = 60.0 / (2.0 * math.pi * self.wheel_r)
        rpm_left  = v_left  * rpm_per_mps
        rpm_right = v_right * rpm_per_mps

        # RPM → raw 속도값 (부호 포함)
        raw_left  = int(rpm_left  / VELOCITY_UNIT_RPM) * self.left_dir
        raw_right = int(rpm_right / VELOCITY_UNIT_RPM) * self.right_dir

        # 범위 클램핑
        raw_left  = max(-MAX_RAW_VEL, min(MAX_RAW_VEL, raw_left))
        raw_right = max(-MAX_RAW_VEL, min(MAX_RAW_VEL, raw_right))

        return raw_left, raw_right

    # ── 콜백 ──────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        import time
        self._last_cmd_time = time.monotonic()

        if not self._dxl_ok:
            return

        raw_l, raw_r = self._twist_to_raw(msg.linear.x, msg.angular.z)
        self._write4(self.left_id,  ADDR_GOAL_VELOCITY, raw_l & 0xFFFFFFFF)
        self._write4(self.right_id, ADDR_GOAL_VELOCITY, raw_r & 0xFFFFFFFF)

    def _watchdog(self):
        """명령 타임아웃 시 모터 정지"""
        import time
        if not self._dxl_ok:
            return
        if time.monotonic() - self._last_cmd_time > self.cmd_timeout:
            self._write4(self.left_id,  ADDR_GOAL_VELOCITY, 0)
            self._write4(self.right_id, ADDR_GOAL_VELOCITY, 0)

    # ── 소멸자 ────────────────────────────────────────────────────

    def destroy_node(self):
        if self._dxl_ok:
            self._write4(self.left_id,  ADDR_GOAL_VELOCITY, 0)
            self._write4(self.right_id, ADDR_GOAL_VELOCITY, 0)
            self._write1(self.left_id,  ADDR_TORQUE_ENABLE, TORQUE_OFF)
            self._write1(self.right_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
            self._port.closePort()
            self.get_logger().info('Dynamixel 정상 종료')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
