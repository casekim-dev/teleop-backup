#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


CHANNELS = {
    "r_bend": 0,    # ID 11
    "r_spread": 5,  # ID 16
    "l_bend": 6,    # ID 21
    "l_spread": 11, # ID 26
}


class SpreadChannelTest(Node):
    def __init__(self, channel, value, rate_hz):
        super().__init__("spread_channel_test")
        self.pub = self.create_publisher(Float32MultiArray, "/igris_c/hand/targets", 10)
        self.channel = channel
        self.value = float(value)
        self.period = 1.0 / float(rate_hz)
        self.count = 0
        self.timer = self.create_timer(self.period, self.tick)
        self.get_logger().info(f"Publishing /igris_c/hand/targets channel={channel} value={value}")
        self.get_logger().info("channels: 0=R bend(ID11), 5=R spread(ID16), 6=L bend(ID21), 11=L spread(ID26)")

    def tick(self):
        data = [0.0] * 12
        if self.channel == "all_spread":
            data[5] = self.value
            data[11] = self.value
        elif self.channel == "all_thumb":
            data[0] = self.value
            data[5] = self.value
            data[6] = self.value
            data[11] = self.value
        else:
            data[CHANNELS[self.channel]] = self.value

        msg = Float32MultiArray()
        msg.data = data
        self.pub.publish(msg)
        self.count += 1
        if self.count % 20 == 1:
            self.get_logger().info(f"targets={data}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", choices=["r_bend", "r_spread", "l_bend", "l_spread", "all_spread", "all_thumb"], default="all_spread")
    parser.add_argument("--value", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = SpreadChannelTest(args.channel, args.value, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
