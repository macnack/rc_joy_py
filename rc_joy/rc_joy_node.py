#!/usr/bin/env python3

# Copyright 2024 Maciej Krupka
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
try:
    from rc_joy.rc_joy import RcJoy
except ImportError:
    from rc_joy import RcJoy
from sensor_msgs.msg import Joy


class RcJoyNode(Node):

    def __init__(self):
        super().__init__('rc_joy_node')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.rc_joy = RcJoy(self.port)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.publisher = self.create_publisher(Joy, '~/output/joy', 1)
        if not self.rc_joy.Connected():
            self.get_logger().error(
                f'Failed to connect to the RC joystick.Port path: {self.port}')
            self.destroy_node()
            rclpy.shutdown()

    def timer_callback(self):
        joystick = self.rc_joy.read()
        if joystick is not None:
            self.get_logger().info(str(joystick))
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [joystick["throttle"], joystick["steering"],
                        joystick["left_gain"], joystick["right_gain"]]
            msg.buttons = [joystick["button"]]
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RcJoyNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
