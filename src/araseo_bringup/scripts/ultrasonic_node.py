#!/usr/bin/env python3
"""
ARASEO-DALIMI Ultrasonic Sensor Node
I2C ADC 기반 초음파 센서 → /robot_{id}/ultrasonic (sensor_msgs/Range)
"""
import sys
sys.path.insert(0, '/home/pinky/pinkylib/sensor/pinkylib')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from ultrasonic import Ultrasonic


class UltrasonicNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_node')
        self.declare_parameters('', [
            ('robot_id', 14),
            ('publish_hz', 20.0),
            ('i2c_bus', 1),
            ('i2c_address', 0x08),
        ])
        g = self.get_parameter
        self.robot_id = g('robot_id').value
        hz = g('publish_hz').value

        self._us = Ultrasonic(
            i2c_bus=g('i2c_bus').value,
            i2c_address=g('i2c_address').value)

        self._pub = self.create_publisher(
            Range,
            f'/robot_{self.robot_id}/ultrasonic',
            10)

        self.create_timer(1.0 / hz, self._timer_cb)
        self.get_logger().info(
            f'[robot_{self.robot_id}] UltrasonicNode 시작 ({hz}Hz)')

    def _timer_cb(self):
        dist = self._us.get_dist()
        if dist is None:
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'robot_{self.robot_id}/ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = max(0.0, float(dist))
        self._pub.publish(msg)

    def destroy_node(self):
        self._us.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
