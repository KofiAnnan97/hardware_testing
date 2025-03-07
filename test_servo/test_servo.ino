
#include "TeensyThreads.h"
#include <Servo.h> 
 
#define SERVO_PIN 14

#define DEBUG 0

Servo headservo;

/*
  TODOs:
    - figure out how to deal with the digital servo reading the wrong position
*/


/* Variables */
const int centerPos = 90;  

String command;

void helpMenu(){
  Serial.println("The following commands are supported:");
  Serial.println("\t- \"center\"     : sets servo to center angle");
  Serial.println("\t- \"sweep\"      : test velocity control for all wheels");
  Serial.println("\t- \"read\"       : get the current servo angle");
  Serial.println("\t- angle_val      : set servo to a specific angle (integer)");
  Serial.println("\t- \"help\"       : bring up this menu");
}

void setup() { 
  headservo.attach(SERVO_PIN);  
  Serial.begin(115200);
  helpMenu();
} 
 
int getHeadAngle() { return headservo.read(); }

void resetCenter(){
  int currPos = getHeadAngle();
    if(centerPos > currPos) {
    for(int pos = currPos; pos <= centerPos; pos += 1) {                                 
      headservo.write(pos);
      delay(15);                      
    } 
  } 
  else if(centerPos < currPos) {
    for(int pos = currPos; pos >= centerPos; pos -= 1) {                                
      headservo.write(pos);
      delay(15);                     
    }
  }
}

void sweep(){
  int pos;
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

void setHead(int targetPos) {
  if(targetPos > 180) targetPos = 180;
  else if(targetPos < 0) targetPos = 0;

  int currPos = getHeadAngle();
  if(DEBUG == 1){
    Serial.print("Current Pos: ");
    Serial.print(currPos);
    Serial.print(" | Target Pos: ");
    Serial.println(targetPos);
  }
  if(targetPos > currPos) {
    for(int pos = currPos; pos <= targetPos; pos += 1) {                                  
      headservo.write(pos);
      delay(15);                      
    } 
  } 
  else if(targetPos < currPos) {
    for(int pos = currPos; pos >= targetPos; pos -= 1) {                                
      headservo.write(pos);
      delay(15);                     
    }
  }
  else {
    delay(15);
    Serial.println("Something went wrong.");
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
      int currPos = getHeadAngle();
      Serial.print("Current position: ");
      Serial.println(currPos);
    } 
    // Set servo angle to center position
    else if(command.equals("center")) resetCenter();
    // Go from 0 to 180 back to 0 [degrees]
    else if(command.equals("sweep")) sweep();
    // Set a servo to a desired angle
    else{
      int position = command.toInt();
      setHead(position);
      Serial.print("Desired position: ");
      Serial.println(position);
    }
  } 
  delay(1000);
} 
