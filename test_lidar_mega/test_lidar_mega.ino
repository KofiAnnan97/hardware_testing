#include <Arduino.h>
#include <RPLidar.h>

/*
  For use with Arduino Mega 2560
*/

/* Lidar Pins */
#define LIDAR_MOTOR 10
#define LIDAR_MOTOR_SPEED 255
#define lidarSerial Serial3

/* Initialize Lidar object */
RPLidar lidar; 

/* Variables */
const uint16_t range_count = 360;
unsigned long start_time = millis();

/* ROS LaserScan Message */
float ranges[range_count];
float range_min = 0.2;
float range_max = 12.0;
uint16_t angle_min = 0;
uint16_t angle_max = 359;
float angle_increment = 1.0;
float time_increment = 0;
float scan_time = 0;

void setup() {
  Serial.begin(115200);

  /* Setup lidar */
  lidarSerial.begin(115200);
  lidar.begin(lidarSerial);

  /* Output Pins */
  pinMode(LIDAR_MOTOR, OUTPUT);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;   //unit: [mm]
    float angle    = lidar.getCurrentPoint().angle;      //unit: [degrees]
    bool  startBit = lidar.getCurrentPoint().startBit;   //whether this point is belong to a new scan
    //byte  quality  = lidar.getCurrentPoint().quality;  //quality of the current measurement
    
    uint16_t degree = (uint16_t)angle;
    float distMeters = distance/1000;
    // store distance by degree and convert to meters
    if(degree >= angle_min && degree <= angle_max && distMeters >= range_min && distMeters <= range_max) {
      ranges[degree] = distMeters;
      /*Serial.print("Angle: ");
      Serial.print(degree);
      Serial.print(" Distance: ");
      Serial.println(ranges[degree]);*/
    }
    
    if(startBit){
      long end_time = millis();
      scan_time = (float)(end_time-start_time)/1000.0;
      time_increment = scan_time/(angle_max-angle_min+1);
      start_time = end_time;
      /*Serial.print("Scan time: ");
      Serial.println(scan_time);
      Serial.print("Time increment: ");
      Serial.println(time_increment, 10);*/
    }
    
  } else {
    analogWrite(LIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(LIDAR_MOTOR, LIDAR_MOTOR_SPEED);
       delay(1000);
    }
  }
}
