# ROS2 driver for a very simple I2C device

The custom I2C device has one register at address 0x00 that can only be written to with values in the range 0 to 15.

## Topic and message

The topic used is `/pi2c`.

The message used is `std_msgs/msgs/Int8`.  Values in the range 0-15 are sent to the device.  Other values are ignored.

## Dependencies

This code uses the Adafruit Extended Bus Library to access the I2C bus.  install using...

```bash
sudo pip3 install adafruit-extended-bus
```

## Acknowledgments

© 2024, University of Leeds.

The author, A. Blight, has asserted his moral rights.
