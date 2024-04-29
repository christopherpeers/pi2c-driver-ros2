#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2024 University of Leeds
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
# https://github.com/adafruit/Adafruit_Python_Extended_Bus
from adafruit_extended_bus import ExtendedI2C as I2C


class I2CDriver(Node):
    def __init__(self):
        # Set up node.
        super().__init__("pi2c")
        # Setu up I2C bus.  Device is /dev/i2c-1
        self.__i2c = I2C(1)
        # Set up subscriber
        self.imu_calibrate_server = self.create_subscription(
            Int8, "pi2c", self.callback_send_i2c_message, 10
        )
        self.get_logger().info("PI2C started...")

    def callback_send_i2c_message(self, value: Int8):
        self.__i2c.writeto(0x04, bytes([value]))

def main(args=None):
    rclpy.init(args=args)
    node = I2CDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
