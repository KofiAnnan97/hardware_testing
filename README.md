# hardware_testing
A collection of scripts used to the functionality of hardware for Arduino and Teensy deployments. 

## Overview of Scripts
- [RP Lidar A1 Series](/test_lidar/test_lidar.ino)
- [L298N Motor driver](/test_motor_driver/test_motor_driver.ino)
- [Mecanum Wheels Control](/test_mechanum_control/)
- [Digital Servo](/test_servo/test_servo.ino) 

## RP Lidar A1 Series
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

## L298N Motor Driver
A simple script to that confirms that a L298N motor driver that can support to motors operates as expected. 

## Mecanum Wheel Control
This script is designed to test the speed control and movement patterns of a four-wheeled vehicle using mecanum (omni) wheels. It integrates serial based command line for testing and can be used to confirm that forward, side and turning movement can be controlled with normalized velocities ranging between -1.0 and 1.0.

```
The following commands are supported:
	- stop                        : sets all motors to zero
	- vtest                       : test velocity control for all wheels
	- mtest                       : test wheel config via possible movement patterns
	- linear_x linear_y angular_z : set the movement of vehicle (must be single space between values)
	- help                        : bring up this menu
```

## Digital Servo
A script for controlling the position of a digital servo using based on a single command.