#include <Arduino.h>
#include "TeensyThreads.h"

// Motor Driver 1 Pins
#define WHEEL1_EN 23
#define WHEEL2_EN 22

#define WHEEL1_IN1 2
#define WHEEL1_IN2 3
#define WHEEL2_IN1 4
#define WHEEL2_IN2 5

// Motor Driver 2 Pins
#define WHEEL3_EN 18
#define WHEEL4_EN 19

#define WHEEL3_IN1 6
#define WHEEL3_IN2 7
#define WHEEL4_IN1 8
#define WHEEL4_IN2 9

// Motor limits
#define MIN_SPEED_INPUT 0 
#define MAX_SPEED_INPUT 100
#define MIN_MOTOR_1_LIMIT 169
#define MIN_MOTOR_2_LIMIT 156
#define MIN_MOTOR_3_LIMIT 144
#define MIN_MOTOR_4_LIMIT 140
#define MAX_MOTOR_LIMIT 255

#define DEBUG 0

/* Wheel State structure */
struct wheelState{
  int in1_pin;
  int in2_pin;
  int pwm_pin;
  int MIN_PWM_LIMIT;
  int MAX_PWM_LIMIT;
};

/* Declare Wheels */
wheelState w1, w2, w3, w4;

/* Twist Values */
double linear_x = 0.0;   //speed  => [-1.0, 1.0] going from backwards to forwards
double linear_y = 0.0;   //strafe => [-1.0, 1.0] going left to right
double angular_z = 0.0;  //turn 

/* Command Values */
String command;
const String delim = " ";

void setup() {
  /* OUTPUTS */
  pinMode(WHEEL1_EN, OUTPUT);
  pinMode(WHEEL2_EN, OUTPUT);
  pinMode(WHEEL3_EN, OUTPUT);
  pinMode(WHEEL4_EN, OUTPUT);

  pinMode(WHEEL1_IN1, OUTPUT);
  pinMode(WHEEL1_IN2, OUTPUT);
  pinMode(WHEEL2_IN1, OUTPUT);
  pinMode(WHEEL2_IN2, OUTPUT);
  pinMode(WHEEL3_IN1, OUTPUT);
  pinMode(WHEEL3_IN2, OUTPUT);
  pinMode(WHEEL4_IN1, OUTPUT);
  pinMode(WHEEL4_IN2, OUTPUT);

  /* INITIAL STATE */
  digitalWrite(WHEEL1_IN1, LOW);
  digitalWrite(WHEEL1_IN2, LOW);
  digitalWrite(WHEEL2_IN1, LOW);
  digitalWrite(WHEEL2_IN2, LOW);
  digitalWrite(WHEEL3_IN1, LOW);
  digitalWrite(WHEEL3_IN2, LOW);
  digitalWrite(WHEEL4_IN1, LOW);
  digitalWrite(WHEEL4_IN2, LOW);

  /* Initialize Wheels */
  w1 = {WHEEL1_IN1, WHEEL1_IN2, WHEEL1_EN, MIN_MOTOR_1_LIMIT, MAX_MOTOR_LIMIT};
  w2 = {WHEEL2_IN1, WHEEL2_IN2, WHEEL2_EN, MIN_MOTOR_2_LIMIT, MAX_MOTOR_LIMIT};
  w3 = {WHEEL3_IN1, WHEEL3_IN2, WHEEL3_EN, MIN_MOTOR_3_LIMIT, MAX_MOTOR_LIMIT};
  w4 = {WHEEL4_IN1, WHEEL4_IN2, WHEEL4_EN, MIN_MOTOR_4_LIMIT, MAX_MOTOR_LIMIT};

  Serial.begin(115200);
}

void sendToMotor(wheelState ws, double speed_input){
  int in1_state, in2_state;
  double scaled_max = MAX_SPEED_INPUT/(double)MAX_SPEED_INPUT;
  if(abs(speed_input) < MIN_SPEED_INPUT) speed_input = MIN_SPEED_INPUT/abs(scaled_max);
  else if(abs(speed_input) > scaled_max) speed_input = speed_input/abs(scaled_max);

  if(speed_input > 0){
    in1_state = HIGH;
    in2_state = LOW;
  }
  else if(speed_input < 0){
    in1_state = LOW;
    in2_state = HIGH;
  }else{
    in1_state = LOW;
    in2_state = LOW;
  }

  int speed = map((int)abs(speed_input * 100), MIN_SPEED_INPUT, MAX_SPEED_INPUT, ws.MIN_PWM_LIMIT, ws.MAX_PWM_LIMIT);
  if(in1_state == in2_state) analogWrite(ws.pwm_pin, 0);
  else analogWrite(ws.pwm_pin, speed);
  digitalWrite(ws.in1_pin, in1_state);
  digitalWrite(ws.in2_pin, in2_state);
  
  if(DEBUG == 1){
    if(in1_state == in2_state) Serial.println("Motor Off");
    else if((in1_state == HIGH) && (in2_state == LOW)) Serial.println("Motor Forward");
    else if((in1_state == LOW) && (in2_state == HIGH)) Serial.println(("Motor Reverse"));
    Serial.print("Speed Input: ");
    Serial.println(speed_input);
    Serial.print("PWM Speed: ");
    Serial.println(speed);
  }
}

void displayWheelInputs(float* wheelInputs){
  Serial.print("   | Wheel Inputs | \nFL: ");
  Serial.print(wheelInputs[0]);
  Serial.print("\t FR: ");
  Serial.print(wheelInputs[1]);
  Serial.print("\nBL: ");
  Serial.print(wheelInputs[3]);
  Serial.print("\t BR: ");
  Serial.println(wheelInputs[2]);
}

void displayWheelPWMs(wheelState w1, wheelState w2, wheelState w3, wheelState w4){
  Serial.print("     | Wheel PWMs | \nFL: ");
  //Serial.print(analogRead(w1.pwm_pin));
  Serial.print("\t FR: ");
  //Serial.print(analogRead(w1.pwm_pin));
  Serial.print("\nBL: ");
  //Serial.print(analogRead(w1.pwm_pin));
  Serial.print("\t BR: ");
  //Serial.println(analogRead(w1.pwm_pin));
}

String getStrSegmentByDelim(String originalStr, char delim, int order){
  int delimCount = 0;
  String segment = "";
  unsigned int originalStrLen = originalStr.length();
  //Serial.println(originalStrLen);
  //char charArr[originalStrLen];
  //originalStr.toCharArray(charArr, sizeof(charArr));
  for(unsigned int i = 0; i < originalStrLen; i++){
    //Serial.print("Does char '");
    //Serial.print(originalStr[i]);
    //Serial.print("' == delim: ");
    Serial.println(originalStr.c_str()[i] == delim);
    if(order < delimCount) break;
    else if(originalStr.c_str()[i] == delim) delimCount++;
    else if(order == delimCount && originalStr.c_str()[i] != delim) segment += originalStr[i];
  }
  Serial.print("Value: ");
  Serial.println(segment);
  return segment; 
}

/*
void movementTest(){
  // Forward
  sendToMotor(w1, 1, 0, 0.5);
  sendToMotor(w2, 1, 0, 0.5);
  sendToMotor(w3, 1, 0, 0.5);
  sendToMotor(w4, 1, 0, 0.5);  
  delay(2000);

  // Backward 
  sendToMotor(w1, 0, 1, 0.5);
  sendToMotor(w2, 0, 1, 0.5);
  sendToMotor(w3, 0, 1, 0.5);
  sendToMotor(w4, 0, 1, 0.5);  
  delay(2000);

  // Strafe Left 
  sendToMotor(w1, 0, 1, 0.5);
  sendToMotor(w2, 1, 0, 0.5);
  sendToMotor(w3, 0, 1, 0.5);
  sendToMotor(w4, 1, 0, 0.5);  
  delay(2000);

  // Strafe Right 
  sendToMotor(w1, 1, 0, 0.5);
  sendToMotor(w2, 0, 1, 0.5);
  sendToMotor(w3, 1, 0, 0.5);
  sendToMotor(w4, 0, 1, 0.5);  
  delay(2000);

  // Turn Left 
  sendToMotor(w1, 0, 1, 0.5);
  sendToMotor(w2, 1, 0, 0.5);
  sendToMotor(w3, 1, 0, 0.5);
  sendToMotor(w4, 0, 1, 0.5);  
  delay(2000);

  // Turn Right 
  sendToMotor(w1, 1, 0, 0.5);
  sendToMotor(w2, 0, 1, 0.5);
  sendToMotor(w3, 0, 1, 0.5);
  sendToMotor(w4, 1, 0, 0.5);  
  delay(2000);
}

void speedTest(){
  // Speed Up
  for(double i = 0.0; i < 1.5; i += 0.1){
    sendToMotor(w1, 1, 0, i);
    sendToMotor(w2, 1, 0, i);
    sendToMotor(w3, 1, 0, i);
    sendToMotor(w4, 1, 0, i);
    delay(2000);
  }

  // Slow down
  for(double j = 1.0; j > -0.5; j -= 0.1){
    sendToMotor(w1, 1, 0, j);
    sendToMotor(w2, 1, 0, j);
    sendToMotor(w3, 1, 0, j);
    sendToMotor(w4, 1, 0, j);
    delay(2000);
  }
}*/

float* fromTwistToPWM(double speed, double strafe, double turn){
  float *wheels = new float[4];

  wheels[0] = speed + strafe - turn; 
  wheels[1] = speed - strafe - turn; 
  wheels[3] = speed - strafe + turn;
  wheels[2] = speed + strafe - turn;

  if(speed+abs(turn) > 1){
    wheels[0] /= speed + turn;
    wheels[1] /= speed + turn;
    wheels[2] /= speed + turn;
    wheels[3] /= speed + turn;
  }
  return wheels;
}

void loop() {
  // Check that motors speed up and slow down properly
  //speedTest();

  // Check that mecanum wheel configuration can turn and strafe
  //movementTest();

  // Send commands to mecanum wheels based on speed, strafe and turn
  /*if(Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Twist Command: ");
    Serial.println(command);
    if(command.equals("stop")){
      linear_x  = 0.0; 
      linear_y  = 0.0; 
      angular_z = 0.0;
    }
    else{
      linear_x  = getStrSegmentByDelim(command, ';', 0).toFloat();
      linear_y  = getStrSegmentByDelim(command, ';', 1).toFloat();
      angular_z = getStrSegmentByDelim(command, ';', 2).toFloat();
    }
  }*/
  
  // Convert revelant ROS Twist message to PWM values
  float *wheelInputs = fromTwistToPWM(linear_x, linear_y, angular_z);
  sendToMotor(w1, wheelInputs[0]);
  sendToMotor(w2, wheelInputs[1]);
  sendToMotor(w3, wheelInputs[2]);
  sendToMotor(w4, wheelInputs[3]);
  if(DEBUG == 1) displayWheelInputs(wheelInputs); 
  delay(2000);
}

