# hardware_testing
A collection of scripts used to the functionality of hardware and hardware setups for Arduino and Teensy deployments.  

## Overview of Scripts
- Generic
	- [Digital Servo](/generic/test_servo/test_servo.ino) 
	- [L298N Motor Driver](/generic/test_motor_driver/test_motor_driver.ino)
	- [Mecanum Wheels Control](/generic/test_mechanum_control/test_mechanum_control.ino)
	- [RP Lidar A1 Series](/generic/test_rplidar_a1/test_rplidar_a1.ino)
	- [WT901 AHRS (Serial)](/generic/test_wt901_serial/test_wt901_serial.ino)
- Arduino Mega 
	- [Real-Time Clock DS2321](/arduino_mega/test_rtc_DS3231/test_rtc_DS3231.ino) (requires [RTClib](https://github.com/adafruit/RTClib))
- Teensy 4.X
	- [Internal RTC and Processor Temperature](/teensy_4/test_internal_rtc_and_temp/test_internal_rtc_and_temp.ino) (requires [InternalTemperature](https://github.com/LAtimes2/InternalTemperature))

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
This script uses my fork of the [RoboPeak's arduino library](https://github.com/robopeak/rplidar_arduino) which can be found here: [https://github.com/KofiAnnan97/rplidar_arduino](https://github.com/KofiAnnan97/rplidar_arduino). It is designed to retrieve data from a 360 degree lidar and convert this information into the ROS LaserScan message. This message is defined as follows:

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

## WT901 AHRS Serial
![](/img/wt901.jpg)
This code is designed to translate the accelerometer, magnetometer, and gyroscope to retrieve the orientation and velocity for the IMU ROS message. 
#### sensor_msgs/msg/Imu Message
```
std_msgs/msg/Header header
geometry_msgs/msg/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/msg/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/msg/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```
The library used can be found here: [https://github.com/WITMOTION/WitStandardModbus_WT901C485/tree/main/Arduino](https://github.com/WITMOTION/WitStandardModbus_WT901C485/tree/main/Arduino)


## Real-Time Clock DS3231
Uses a DS3231 real-time clock (RTC) to provide a reference clock even with battery backup. Serial commands have been implemented to get and view the time.
```
The following commands are supported:
	- "s" yyyy/mm/dd hh:mm:ss       : set date and time
	- "pdt"                         : print date and time
	- "pts"                         : print unix timestamp
	- "gt"                          : print temperature
	- "help" or "h"                 : bring up this menu
```