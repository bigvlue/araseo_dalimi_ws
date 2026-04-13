#!/usr/bin/env python3
"""
ARASEO-DALIMI Wheel Odometry Node
cmd_vel_safe 를 적분해 nav_msgs/Odometry (odom 토픽) 를 발행한다.
~/.ros/araseo_calibration.yaml 의 velocity_scale / angular_scale 을 반영한다.
"""
import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

CALIB_FILE = os.path.expanduser('~/.ros/araseo_calibration.yaml')


class WheelOdomNode(Node):

    def __init__(self):
        super().__init__('wheel_odom')

        self.declare_parameters('', [
            ('robot_id',       0),
            ('publish_hz',    20.0),
        ])
        g = self.get_parameter
        self.robot_id = g('robot_id').value

        # 캘리브레이션 파일 로드
        self._vel_scale, self._ang_scale = self._load_calibration()

        # 오도메트리 상태
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._vx    = 0.0
        self._vyaw  = 0.0
        self._last_t = time.monotonic()

        self.create_subscription(
            Twist,
            f'/robot_{self.robot_id}/cmd_vel_safe',
            self._cmd_cb, 10)

        self._pub   = self.create_publisher(Odometry, 'odom', 10)
        self._tfbr  = TransformBroadcaster(self)

        period = 1.0 / g('publish_hz').value
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f'[robot_{self.robot_id}] WheelOdom 시작 '
            f'(vel_scale={self._vel_scale:.4f}, '
            f'ang_scale={self._ang_scale:.4f})')

    def _load_calibration(self):
        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE) as f:
                    data = yaml.safe_load(f)
                p  = data.get('wheel_odom', {}).get('ros__parameters', {})
                vs = float(p.get('velocity_scale', 1.0))
                as_ = float(p.get('angular_scale',  1.0))
                vs  = max(0.5, min(2.0, vs))
                as_ = max(0.5, min(2.0, as_))
                self.get_logger().info(
                    f'캘리브레이션 로드 완료: '
                    f'vel={vs:.4f}, ang={as_:.4f}')
                return vs, as_
            except Exception as e:
                self.get_logger().warn(f'캘리브레이션 로드 실패: {e} — 기본값 사용')
        else:
            self.get_logger().warn(
                f'캘리브레이션 파일 없음 ({CALIB_FILE}) — 기본값 사용')
        return 1.0, 1.0

    def _cmd_cb(self, msg: Twist):
        self._vx   = msg.linear.x
        self._vyaw = msg.angular.z

    def _publish(self):
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now

        # 스케일 반영 적분
        vx_cal   = self._vx   * self._vel_scale
        vyaw_cal = self._vyaw * self._ang_scale

        d_theta = vyaw_cal * dt
        d_x     = vx_cal * math.cos(self._theta + d_theta / 2.0) * dt
        d_y     = vx_cal * math.sin(self._theta + d_theta / 2.0) * dt

        self._x     += d_x
        self._y     += d_y
        self._theta += d_theta

        pos_var = 0.01 + abs(vx_cal)   * 0.02
        yaw_var = 0.01 + abs(vyaw_cal) * 0.02

        ros_now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = ros_now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = f'robot_{self.robot_id}/base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        cov = [0.0] * 36
        cov[0] = pos_var; cov[7] = pos_var
        cov[14] = cov[21] = cov[28] = 9999.0
        cov[35] = yaw_var
        odom.pose.covariance = cov

        odom.twist.twist.linear.x  = vx_cal
        odom.twist.twist.angular.z = vyaw_cal
        tc = [0.0] * 36
        tc[0] = 0.01; tc[35] = 0.01
        odom.twist.covariance = tc

        self._pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp    = ros_now
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = f'robot_{self.robot_id}/base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z    = math.sin(self._theta / 2.0)
        tf.transform.rotation.w    = math.cos(self._theta / 2.0)
        self._tfbr.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
