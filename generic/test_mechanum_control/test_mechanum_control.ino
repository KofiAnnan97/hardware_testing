#include <Arduino.h>

/*
This script is designed for a four wheel vehicle equipped with mecanum wheels
using a Teensy 4.0.
Configuration:

          |\=| |==========| |=/|
  Wheel 1 |=\|+|          |+|/=| Wheel 2
          |\=| |          | |=/|
               |          |
          |=/| |          | |\=|
  Wheel 4 |/=|+|          |+|=\| Wheel 3
          |=/| |==========| |\=|
*/

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
float linear_x = 0.0;   //speed  => [-1.0, 1.0] backwards to forwards
float linear_y = 0.0;   //strafe => [-1.0, 1.0] left to right
float angular_z = 0.0;  //turn   => [-1.0, 1.0] left to right

/* Command Values */
String command;
const char delimiter = ' ';

/* Control Functions */
float* fromTwistToPWM(float speed, float strafe, float turn);
void sendToMotor(wheelState ws, float speed_input);
void stopMotors();

/* Command Functions */
String getStrSegmentByDelim(String originalStr, char delim, int order);

/* Display Functions */
void helpMenu();
void displayWheelInputs(float* wheelInputs);

/* Test Functions */
void movementTest();
void speedTest();

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
  helpMenu();
  Serial.println("\nSet motors to zero.");
  stopMotors();
}

void loop() {
  // Send commands to mecanum wheels based on speed, strafe and turn
  if(Serial.available()){
    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("\nCommand: ");
    Serial.println(command);
    // Bring up the help menu
    if(command.equals("help")) helpMenu();
    // Set all motors to zero (stop vehicle) 
    else if(command.equals("stop")) stopMotors();
     // Check that motors speed up and slow down properly
    else if(command.equals("vtest")) speedTest();
    // Check that mecanum wheel configuration can turn and strafe
    else if(command.equals("mtest")) movementTest();
    else{
      linear_x  = getStrSegmentByDelim(command, delimiter, 0).toFloat();
      linear_y  = getStrSegmentByDelim(command, delimiter, 1).toFloat();
      angular_z = getStrSegmentByDelim(command, delimiter, 2).toFloat();
      if(DEBUG == 1){
        Serial.print("Speed | X: ");
        Serial.print(linear_x);
        Serial.print(" Y: ");
        Serial.print(linear_y);
        Serial.print(" Yaw: ");
        Serial.println(angular_z);
      }
      // Convert revelant ROS Twist message to PWM values
      float *wheelInputs = fromTwistToPWM(linear_x, linear_y, angular_z);
      sendToMotor(w1, wheelInputs[0]);
      sendToMotor(w2, wheelInputs[1]);
      sendToMotor(w3, wheelInputs[2]);
      sendToMotor(w4, wheelInputs[3]);
      displayWheelInputs(wheelInputs); 
    }
  }
  delay(2000);
}

float* fromTwistToPWM(float speed, float strafe, float turn){
  float *wheels = new float[4];

  wheels[0] = speed + strafe + turn; 
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

void sendToMotor(wheelState ws, float speed_input){
  int in1_state, in2_state;
  float scaled_max = MAX_SPEED_INPUT/(float)MAX_SPEED_INPUT;
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

void stopMotors(){
  float wheelInputs[] = {0.0, 0.0, 0.0, 0.0};
  sendToMotor(w1, wheelInputs[0]);
  sendToMotor(w2, wheelInputs[1]);
  sendToMotor(w3, wheelInputs[2]);
  sendToMotor(w4, wheelInputs[3]);
  displayWheelInputs(wheelInputs);
}

String getStrSegmentByDelim(String originalStr, char delim, int order){
  int delimCount = 0;
  int startIdx = 0;
  int endIdx = 0;
  for(unsigned int i = 0; i < originalStr.length(); i++){
    if(originalStr[i] == delim){
      ++delimCount;
      if(order < delimCount) break;
      startIdx = i+1;
      endIdx = i+1;
    } 
    else if(order == delimCount && originalStr[i] != delim) endIdx++;
  }
  if(startIdx < endIdx) return(originalStr.substring(startIdx, endIdx+1));
  else return "";
}

void helpMenu(){
  Serial.println("The following commands are supported:");
  Serial.println("\t- \"stop\"                        : sets all motors to zero");
  Serial.println("\t- \"vtest\"                       : test velocity control for all wheels");
  Serial.println("\t- \"mtest\"                       : test wheel config via possible movement patterns");
  Serial.println("\t- linear_x linear_y angular_z   : set the movement of vehicle (floats with single space delimiter)");
  Serial.println("\t- \"help\"                        : bring up this menu");
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

void movementTest(){
  String sequence[] = {"Forward", "Backward", "Strafe Left", "Strafe Right", "Turn Left", "Turn Right"}; 
  float wheel1TestVals[] = {0.5,-0.5,-0.5,0.5,-0.5,0.5};
  float wheel2TestVals[] = {0.5,-0.5,0.5,-0.5,0.5,-0.5};
  float wheel3TestVals[] = {0.5,-0.5,-0.5,0.5,0.5,-0.5};
  float wheel4TestVals[] = {0.5,-0.5,0.5,-0.5,-0.5,0.5};

  for(int i = 0; i < 6; i++){
    float wheelInputs[] = {wheel1TestVals[i], wheel2TestVals[i], wheel3TestVals[i], wheel4TestVals[i]};
    sendToMotor(w1, wheel1TestVals[i]);
    sendToMotor(w2, wheel2TestVals[i]);
    sendToMotor(w3, wheel3TestVals[i]);
    sendToMotor(w4, wheel4TestVals[i]);
    Serial.print("Movement: ");
    Serial.println(sequence[i]);
    displayWheelInputs(wheelInputs); 
    delay(2000);
  }
  stopMotors();
}

void speedTest(){
  // Speed Up
  for(float i = 0.0; i < 1.2; i += 0.1){
    sendToMotor(w1, i);
    sendToMotor(w2, i);
    sendToMotor(w3, i);
    sendToMotor(w4, i);
    Serial.print("Current Speed Input: ");
    Serial.println(i);
    delay(2000);
  }

  // Slow down
  for(float j = 1.0; j > 0; j -= 0.1){
    sendToMotor(w1, j);
    sendToMotor(w2, j);
    sendToMotor(w3, j);
    sendToMotor(w4, j);
    Serial.print("Current Speed Input: ");
    Serial.println(j);
    delay(2000);
  }
  stopMotors();
}