#include <Arduino.h>
#include "TeensyThreads.h"
#include <Servo.h> 
 
#define SERVO_PIN 14

#define DEBUG 0
#define SERVO_DELAY 15

/* Initiate servo object */
Servo headservo;

/* Variables */
const uint8_t centerPos = 90;  
String command;

/*
  TODOs:
    - figure out how to deal with the digital servo reading the wrong position
*/

void helpMenu(){
  Serial.println("The following commands are supported:");
  Serial.println("\t- \"center\"     : sets servo to center angle");
  Serial.println("\t- \"sweep\"      : test velocity control for all wheels");
  Serial.println("\t- \"read\"       : get the current servo angle");
  Serial.println("\t- angle_val    : set servo to a specific angle (integer)");
  Serial.println("\t- \"help\"       : bring up this menu");
}

void setup() { 
  headservo.attach(SERVO_PIN);  
  Serial.begin(115200);
  setHead(centerPos);
  helpMenu();
} 
 
uint8_t getHeadAngle() { return (uint8_t)headservo.read(); }

void setHead(uint8_t targetPos) {
  if(targetPos > 180) targetPos = 180;
  else if(targetPos < 0) targetPos = 0;

  uint8_t currPos = getHeadAngle();
  if(DEBUG == 1){
    Serial.print("Current Pos: ");
    Serial.print(currPos);
    Serial.print(" | Target Pos: ");
    Serial.println(targetPos);
  }
  if(targetPos > currPos) {
    for(uint8_t pos = currPos; pos <= targetPos; pos += 1) {                                  
      headservo.write(pos);
      delay(SERVO_DELAY);                      
    } 
  } 
  else if(targetPos < currPos) {
    for(uint8_t pos = currPos; pos >= targetPos; pos -= 1) {                                
      headservo.write(pos);
      delay(SERVO_DELAY);                     
    }
  }
  else {
    delay(SERVO_DELAY);
    Serial.println("Something went wrong.");
  }
}

void resetCenter(){
  setHead(centerPos);
  Serial.print("Set servo to ");
  Serial.print(centerPos);
  Serial.println("degrees (center position)");
}

void sweep(){
  uint8_t pos;
  for(pos = 0; pos <= 180; pos += 20) {                                 
    setHead(pos);
    Serial.print("Target Position: ");
    Serial.println(getHeadAngle());
    delay(500); 
  } 
  for(pos = 180; pos >= 0; pos -= 20) {                                 
    setHead(pos);
    Serial.print("Target Position: ");
    Serial.println(getHeadAngle());
    delay(500); 
  } 
}

void loop() { 
  // Send commands to servo
  if(Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("\nCommand: ");
    Serial.println(command);
    // Bring up help menu
    if(command.equals("help")) helpMenu();
    // Retrieve current servo angle
    else if(command.equals("read")){
      Serial.print("Current position: ");
      Serial.println(getHeadAngle());
    } 
    // Set servo angle to center position
    else if(command.equals("center")) resetCenter();
    // Go from 0 to 180 back to 0 [degrees]
    else if(command.equals("sweep")) sweep();
    // Set a servo to a desired angle
    else{
      uint8_t position = (uint8_t)command.toInt();
      setHead(position);
      Serial.print("Desired position: ");
      Serial.println(position);
    }
  } 
  delay(1000);
} 
