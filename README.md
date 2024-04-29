# ROS2 driver for a very simple I2C device

The custom I2C device has one register at an address 0x42 that can only be written to with values in the range 0 to 15.  The address can be changed by modifying the code.

## ROS topic and message

The topic used is `/pi2c`.

The message used is `std_msgs/msgs/UInt8`.  Values in the range 0 to 15 are sent to the device.  Other values are ignored.

## Installation

This driver using the Python SMBUS package to interface with the I2C on the RPi.  Install it using:

```bash
sudo apt install python3-smbus
```

Clone this repo in a ROS2 workspace and then build and run.  See below for example commands:

```bash
cd ~/ws/src
git clone https://github.com/RealRobotics/pi2c-driver-ros2
cd ~/ws
colcon build --packages-select pi2c_driver
ros2 launch pi2c_driver pi2c_driver.launch.py
```

When successful started, you should see output like this:

```text
[INFO] [launch]: All log files can be found below /home/pipebot/.ros/log/2024-04-22-20-18-56-873873-skatebot1-7672
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [pi2c_driver_exec-1]: process started with pid [7673]
[pi2c_driver_exec-1] [INFO] [1713813538.509213956] [pi2c]: PI2C started...
```

Verify that you can see the topic using:

```bash
$ ros2 topic list
/parameter_events
/pi2c
/rosout
```

## Testing

To send a command to the I2C device from the command line use this command:

```bash
$ ros2 topic pub -1 /pi2c std_msgs/UInt8 "{data: 14}"
publisher: beginning loop
publishing #1: std_msgs.msg.UInt8(data=14)
```

The driver can emit one of three messages.

```bash
# Value in range and device present
[pi2c_driver_exec-1] [INFO] [1713820461.992177218] [pi2c]: Written 14 to address 68
# Value out of range
[pi2c_driver_exec-1] [WARN] [1713820470.863957176] [pi2c]: Value 24 exceeds maximum value of 15. Ignoring...
# No I2C device
[pi2c_driver_exec-1] [ERROR] [1713813552.280648022] [pi2c]: Failed to write value 14 to I2C device: [Errno 121] Remote I/O error
```

## Acknowledgments

© 2024, University of Leeds.

The author, A. Blight, has asserted his moral rights.
