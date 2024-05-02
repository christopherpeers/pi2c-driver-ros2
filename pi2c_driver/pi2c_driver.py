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

import smbus2
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

# Default on RPi is /dev/i2c-1
I2C_BUS = 1
# I2C addresses of devices.
I2C_ADDRESS_DEVICE_1 = 0x42
I2C_ADDRESS_DEVICE_2 = 0x43
# Maximum input value from ROS2.
MAX_VALUE = 15
# Maximum value to be written to device 1.
MAX_VALUE_DEVICE_1 = 8


class I2CDriver(Node):
    def __init__(self):
        # Set up node.
        super().__init__("pi2c")
        # Set up subscriber
        self.imu_calibrate_server = self.create_subscription(
            UInt8, "pi2c", self.callback_send_i2c_message, 10
        )
        info_string = "PI2C started using address 0x{:02X} on I2C bus {}".format(
            I2C_ADDRESS, I2C_BUS
        )
        self.get_logger().info(info_string)

    def callback_send_i2c_message(self, value: UInt8):
        if value.data > MAX_VALUE:
            warning_string = "Value {} exceeds maximum value of {}. Ignoring...".format(
                value.data, MAX_VALUE
            )
            self.get_logger().warn(warning_string)
        else:
            try:
                with smbus2.SMBus(I2C_BUS) as bus:
                    if (value.data < MAX_VALUE_DEVICE_1):
                        i2c_address = I2C_ADDRESS_DEVICE_1
                    else:
                        value.data -= MAX_VALUE_DEVICE_1
                        i2c_address = I2C_ADDRESS_DEVICE_2
                    bus.write_byte(i2c_address, value.data)
                    info_string = "Written {} to address 0x{:02X}".format(
                        value.data, i2c_address
                    )
                    self.get_logger().info(info_string)
            except Exception as e:
                error_string = "Failed to write value {} to I2C device: {}".format(
                    value.data, str(e)
                )
                self.get_logger().error(error_string)


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
