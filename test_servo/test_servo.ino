
#include "TeensyThreads.h"
#include <Servo.h> 
 
#define SERVO_PIN 14

Servo headservo; 

int pos = 0;   
int iter_num = 0;
int last_pos = -1;

void setup() { 
  headservo.attach(SERVO_PIN);  
  Serial.begin(9600);
} 
 
int get_head_angle() { return headservo.read(); }

void reset_center(){
  int curr_pos = get_head_angle();
  int target_pos = 90;
  //int target_pos = 0;
  if(target_pos > curr_pos) {
    for(int pos = curr_pos; pos <= target_pos; pos += 1) {                                 
      headservo.write(pos);
      delay(15);                      
    } 
  } 
  else if(target_pos < curr_pos) {
    for(int pos = curr_pos; pos >= target_pos; pos -= 1) {                                
      headservo.write(pos);
      delay(15);                     
    }
  }
}

void set_head(int target_pos) {
  if(target_pos > 180) target_pos = 180;
  else if(target_pos < 0) target_pos = 0;

  int curr_pos = get_head_angle();
  Serial.print("Current Pos: ");
  Serial.print(curr_pos);
  Serial.print(" | Target Pos: ");
  Serial.println(target_pos);

  if(target_pos >= curr_pos) {
    for(int pos = curr_pos; pos <= target_pos; pos += 1) {                                  
      headservo.write(pos);
      delay(15);                      
    } 
  } 
  else if(target_pos < curr_pos) {
    for(int pos = curr_pos; pos >= target_pos; pos -= 1) {                                
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
  for(pos = 0; pos <= 180; pos += 20) {                                 
    set_head(pos);
    Serial.print("Position: ");
    Serial.println(get_head_angle());
    delay(2000); 
  } 
  for(pos = 180; pos >= 0; pos -= 20) {                                 
    set_head(pos);
    Serial.print("Position: ");
    Serial.println(get_head_angle());
    delay(2000); 
  } 
  reset_center();
  delay(5000);
} 
