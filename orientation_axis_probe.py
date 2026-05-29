#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class OrientationAxisProbe(Node):
    def __init__(self, topic, frame_id, axis, value, duration, rate):
        super().__init__('orientation_axis_probe')
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.topic = topic
        self.frame_id = frame_id
        self.axis = axis
        self.value = float(value)
        self.duration = float(duration)
        self.period = 1.0 / float(rate)

    def make_msg(self, value):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        if self.axis == 'x':
            msg.twist.angular.x = value
        elif self.axis == 'y':
            msg.twist.angular.y = value
        elif self.axis == 'z':
            msg.twist.angular.z = value
        else:
            raise ValueError(f'bad axis: {self.axis}')
        return msg

    def run(self):
        self.get_logger().warn(
            f'Publishing pure angular {self.axis}={self.value:.3f} rad/s to {self.topic} in {self.frame_id} for {self.duration:.2f}s'
        )
        end = time.monotonic() + self.duration
        while rclpy.ok() and time.monotonic() < end:
            self.pub.publish(self.make_msg(self.value))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)

        self.get_logger().info('Sending zero twist')
        for _ in range(10):
            self.pub.publish(self.make_msg(0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)


def main():
    parser = argparse.ArgumentParser(description='Publish one pure angular axis for EE frame alignment testing.')
    parser.add_argument('--side', choices=['left', 'right'], default='left')
    parser.add_argument('--topic', default=None)
    parser.add_argument('--frame', default='base_link')
    parser.add_argument('--axis', choices=['x', 'y', 'z'], required=True)
    parser.add_argument('--value', type=float, default=0.25)
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--rate', type=float, default=50.0)
    args = parser.parse_args()

    topic = args.topic
    if topic is None:
        topic = '/left_servo/left_servo/delta_twist_cmds' if args.side == 'left' else '/right_servo/right_servo/delta_twist_cmds'

    rclpy.init()
    node = OrientationAxisProbe(topic, args.frame, args.axis, args.value, args.duration, args.rate)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
