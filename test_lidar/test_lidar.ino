#include <Arduino.h>
#include <RPLidar.h>
#include <math.h>

/* Lidar Pins */
#define LIDAR_MOTOR 10
#define LIDAR_MOTOR_SPEED 255

RPLidar lidar;

float minDist = 100;
float angleAtMinDist = 0;
const int range_count = 360;

float range[range_count];

float ANGLE_MAX = 359;
float ANGLE_MIN = 0;
float ANGLE_INCREMENT = (ANGLE_MAX - ANGLE_MIN)/(range_count-1);

float DISTANCE_MIN = 0.2;
float DISTANCE_MAX = 12;

float before_scan = 0;
float after_scan = 0;
float scan_time;
float time_increment;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  lidar.begin(Serial1);

  pinMode(LIDAR_MOTOR, OUTPUT);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //Serial.println("Lidar is connected.");
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //angle value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    //byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    if(startBit){
      Serial.print("Angle: ");
      Serial.print(angle);
      Serial.print(" Distance: ");
      Serial.println(distance/1000);
      minDist = 100000;
      angleAtMinDist = 0;
    }else{
      if(distance > 0 && distance < minDist){
        minDist = distance/1000;
        angleAtMinDist = angle;
      }
    }
    
  } else {
    //Serial.println("Lidar is not connected.");
    analogWrite(LIDAR_MOTOR, 10); //stop the rplidar motor

    rplidar_response_device_info_t info;
    rplidar_response_device_health_t healthinfo;
    Serial.println(lidar.getHealth(healthinfo));    
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       before_scan = millis()/1000;
       lidar.startScan();
       analogWrite(LIDAR_MOTOR, LIDAR_MOTOR_SPEED);
       delay(1000);
       after_scan = millis()/1000;
       scan_time = after_scan - before_scan;
       time_increment = scan_time/(range_count-1);
       Serial.println(scan_time);
       Serial.println(time_increment);
    }
    //Serial.println(lidar.getDeviceInfo(info, 100));
  }
}
