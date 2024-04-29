# ROS2 driver for a very simple I2C device

The custom I2C device has one register at address 0x00 that can only be written to with values in the range 0 to 15.

## Topic and message

The topic used is `/pi2c`.

The message used is `std_msgs/msgs/Int8`.  Values in the range 0-15 are sent to the device.  Other values are ignored.

## Testing

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

```

Verify that you can see the topic using:

```bash
ros2 topic list
```

To send a command to the I2C device from the command line use this command:

```bash
ros2 topic pub -1
```



## Acknowledgments

© 2024, University of Leeds.

The author, A. Blight, has asserted his moral rights.
