# hardware_testing
A collection of scripts used to the functionality of hardware and hardware setups for Arduino and Teensy deployments.  

## Overview of Scripts
- [Digital Servo](/test_servo/test_servo.ino) 
- [L298N Motor Driver](/test_motor_driver/test_motor_driver.ino)
- [Mecanum Wheels Control](/test_mechanum_control/test_mechanum_control.ino)
- RP Lidar A1 Series
	- [Arduino Mega](/test_lidar_mega/test_lidar_mega.ino)

## Digital Servo
![](/img/hiwonder_hps-2018.jpg)
A script for controlling the position of a digital servo using commands.

```
The following commands are supported:
	- "center"     : sets servo to center angle
	- "sweep"      : test velocity control for all wheels
	- "read"       : get the current servo angle
	- angle_val    : set servo to a specific angle (integer)
	- "help"       : bring up this menu
```

## L298N Motor Driver
A simple script to that confirms that a L298N motor driver that can support to motors operates as expected. 
![](/img/l298n_motor_driver.jpg)


## Mecanum Wheel Control
This script is designed to test the speed control and movement patterns of a four-wheeled vehicle using mecanum (omni) wheels. It integrates serial based command line for testing and can be used to confirm that forward, side and turning movement can be controlled with normalized velocities ranging between -1.0 and 1.0.

```
The following commands are supported:
	- "stop"                        : sets all motors to zero
	- "vtest"                       : test velocity control for all wheels
	- "mtest"                       : test wheel config via possible movement patterns
	- linear_x linear_y angular_z   : set the movement of vehicle (floats with single space delimiter)
	- "help"                        : bring up this menu
```

## RP Lidar A1 Series
![](/img/rplidar_a1m8.jpg)
This code is heavily reliant on the arduino library developed by RoboPeak which can be found here: [https://github.com/robopeak/rplidar_arduino](https://github.com/robopeak/rplidar_arduino). This script is designed to retrieve data from a 360 degree lidar and convert this information into the ROS LaserScan message. This message is defined as follows:

#### sensor_msgs/msg/LaserScan Message
```
std_msgs/msg/Header header
float angle_min
float angle_max
float angle_increment
float time_increment
float scan_time
float range_min
float range_max
float[] ranges
float[] intensities
```