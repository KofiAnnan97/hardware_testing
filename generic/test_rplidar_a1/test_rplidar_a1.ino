#include <Arduino.h>
#include <RPLidar.h>
#include <limits>

/*
  For use with Arduino Mega 2560 (Serial3) and Teensy 4.X
*/

/* Lidar Setup */
#define LIDAR_MOTOR 10
#define LIDAR_MOTOR_SPEED 255
#define lidarSerial Serial3

/* Debug */
#define DEBUG 1

/* Initialize Lidar object */
RPLidar lidar; 

/* Variables */
const uint16_t range_count = 360;
unsigned long start_time = millis();

/* ROS LaserScan Message */
float ranges[range_count];
float intensities[range_count];
float range_min = 0.2;
float range_max = 12.0;
float angle_min = 0;
float angle_max = 2*M_PI;
float angle_increment = 0;
float time_increment = 0;
float scan_time = 0;

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)

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
    byte  quality  = lidar.getCurrentPoint().quality;  //quality of the current measurement
    
    uint16_t degree = (uint16_t)angle;  //convert angle to integer
    float rad = DEG2RAD(degree);
    float distMeters = distance/1000.0; //convert to meters
    
    if(rad >= angle_min && rad <= angle_max){
      if(distMeters >= range_min && distMeters <= range_max) ranges[degree] = distMeters;
      else ranges[degree] = std::numeric_limits<float>::infinity();
      intensities[degree] = (float)(quality >> 2);
      if(DEBUG == 1){
        Serial.print("Angle: ");
        Serial.print(degree);
        Serial.print(" Distance: ");
        Serial.print(ranges[degree]);
        Serial.print(" | Intensity: ");
        Serial.println(intensities[degree]);
      } 
    }
     
    if(startBit){
      scan_time = (millis()-start_time)/1000.0;
      time_increment = scan_time/(angle_max-angle_min);
      angle_increment = (angle_max-angle_min)/(float)(range_count-1);
      start_time = millis();
      
      if(DEBUG == 1){
        Serial.print("Scan time: ");
        Serial.println(scan_time);
        Serial.print("Time increment: ");
        Serial.println(time_increment, 10);
        Serial.print("Angle increment: ");
        Serial.println(angle_increment, 10);
      }
    }
   
    
  } else {
    analogWrite(LIDAR_MOTOR, 0); //stop lidar motor
    
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
