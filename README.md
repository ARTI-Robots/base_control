# base_control

[ROS](http://wiki.ros.org/) library and node for controlling a robot base (an automated ground vehicle).

Takes Twist and Ackermann messages and controls motors accordingly. In order to control the motors a 
plugin must be provided. For an example of such a plugin in see the implementation for the VESC
motor controller (https://github.com/ARTI-Robots/base_control_vesc).

This repository contains the following packages:

- *arti_base_control* contains the library and node for controlling a robot base.
- *arti_base_control_examples* contains two examples that show how to use arti_base_control.

## Prerequisites

- ROS Kinetic

## License

All files in this repository are distributed under the terms of the 2-clause BSD license. See `LICENSE.txt` for
details.
