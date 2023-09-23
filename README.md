# ros_bno08x

A ROS2 node for communicating with the [BNO080](https://www.sparkfun.com/products/14686) or [BNO085](https://www.adafruit.com/product/4754) 9DOF IMU.

## Description

The node communicates with the BNO08x via i2c using Adafruit's Python library: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x.  The i2c address is preset to `0x4b` for the BNO080.  This is configurable for the BNO085 on `0x4a`.  The data is stored in the following:

* Accelerometer and Gyroscope (sensor_msgs/Imu): `ros_bno08x/raw`
* Magnometer (sensor_msgs/MagneticField): `ros_bno08x/mag`
* Temperature (sensor_msgs/Temperature): `ros_bno08x/temp`
* Diagnostics (diagnostic_msgs/DiagnosticStatus): `ros_bno08x/status`

## Installation Instructions

* Enable i2c
  ```
  sudo apt-get install i2c-tools
  i2cdetect -l
  ```
* Add `i2c-devl` to boot with `sudo nano /etc/modules-load.d/modules.conf`
* Connect i2c devices to Sparkfun Qwiic hat and run `i2cdetect -y 1` to identify channels
* Install Circuit Python: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi
* Install driver for BNO08x IMU: `sudo pip3 install adafruit-circuitpython-bno08x`.

## Running the Node

`ros2 launch ros_bno08x bno08x.launch.xml`
  
## Tested Setup

It should work on other versions but Python 3 is a requirement.

* Platform: Raspberry Pi 4
* OS: Ubuntu Jammy 22.04
* ROS2: Humble
* Python: 3.10.12

