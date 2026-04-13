#!/usr/bin/env python3
"""
RobotPose (아라서달이미 커스텀) → nav_msgs/Odometry 변환 노드
EKF 입력을 위해 GPS 카메라 위치를 표준 Odometry 포맷으로 변환한다.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from araseo_dalimi_interfaces.msg import RobotPose


class PoseToOdomNode(Node):

    def __init__(self):
        super().__init__('pose_to_odom')

        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').value

        # GPS 카메라 소스일 때만 변환 발행
        self._sub = self.create_subscription(
            RobotPose,
            f'/robot_{self.robot_id}/pose',
            self._pose_callback,
            10
        )
        self._pub = self.create_publisher(
            Odometry,
            'gps_pose_odom',
            10
        )
        self._tf_br = TransformBroadcaster(self)
        self.get_logger().info(
            f'[robot_{self.robot_id}] PoseToOdom 변환 노드 시작'
        )

    def _pose_callback(self, msg: RobotPose):
        # SOURCE_GPS_CAMERA(0) 또는 SOURCE_LIDAR(1) 만 변환
        if msg.source == 2:  # SOURCE_IMU_ODOM은 이미 odom 토픽에서 처리
            return

        now = self.get_clock().now().to_msg()
        theta = msg.theta_mrad * 0.001  # mrad → rad

        # 신뢰도 기반 공분산 계산
        conf = max(msg.confidence_pct, 1)
        pos_cov = (1.0 / conf) * 0.1   # confidence 높을수록 공분산 작게
        yaw_cov = (1.0 / conf) * 0.05

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'map'
        odom.child_frame_id  = f'robot_{self.robot_id}/base_link'

        odom.pose.pose.position.x = msg.x_mm * 0.001  # mm → m
        odom.pose.pose.position.y = msg.y_mm * 0.001
        odom.pose.pose.position.z = 0.0

        # yaw → quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        # 공분산 행렬 (6x6, row-major)
        cov = [0.0] * 36
        cov[0]  = pos_cov   # x
        cov[7]  = pos_cov   # y
        cov[14] = 9999.0    # z (2D 모드, 무시)
        cov[21] = 9999.0    # roll
        cov[28] = 9999.0    # pitch
        cov[35] = yaw_cov   # yaw
        odom.pose.covariance = cov

        self._pub.publish(odom)

        # TF 발행 (map → base_link_gps, 디버그용)
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = 'map'
        tf.child_frame_id  = f'robot_{self.robot_id}/gps'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = odom.pose.pose.orientation
        self._tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
