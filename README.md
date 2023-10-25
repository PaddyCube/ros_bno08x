# ros_bno08x

A ROS2 node for communicating with the [BNO080](https://www.sparkfun.com/products/14686) or [BNO085](https://www.adafruit.com/product/4754) 9DOF IMU.

## Description

The node communicates with the BNO08x via UART (USB TTL converter) using Adafruit's Python library: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x. 

* Accelerometer and Gyroscope (sensor_msgs/Imu): `ros_bno08x/raw`
* Magnometer (sensor_msgs/MagneticField): `ros_bno08x/mag`
* Temperature (sensor_msgs/Temperature): `ros_bno08x/temp`
* Diagnostics (diagnostic_msgs/DiagnosticStatus): `ros_bno08x/status`

## Installation Instructions
* Install Circuit Python: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi
* Install driver for BNO08x IMU: `sudo pip3 install adafruit-circuitpython-bno08x`.
* clone and build the repository
## Running the Node

`ros2 launch ros_bno08x bno08x.launch.xml`
  
## Tested Setup

It should work on other versions but Python 3 is a requirement.

* Platform: Raspberry Pi 4 + NVidia Jetson + PC
* OS: Ubuntu Jammy 22.04
* ROS2: Humble
* Python: 3.10.12

