#include <Arduino.h>
#include "TeensyThreads.h"

// Motor Driver 1 Pins
const int WHEEL1_EN  = 23;
const int WHEEL1_IN1 = 2;
const int WHEEL1_IN2 = 3;
const int WHEEL2_EN  = 22;
const int WHEEL2_IN1 = 4;
const int WHEEL2_IN2 = 5;

// Motor Driver 2 Pins
const int WHEEL3_EN  = 18;
const int WHEEL3_IN1 = 6;
const int WHEEL3_IN2 = 7;
const int WHEEL4_EN  = 19;
const int WHEEL4_IN1 = 8;
const int WHEEL4_IN2 = 9;

// Motor Test Values
const int WHEEL1_OUT1 = 0;
const int WHEEL1_OUT2 = 0;
const int WHEEL2_OUT1 = 0;
const int WHEEL2_OUT2 = 0;
const int WHEEL3_OUT1 = 0;
const int WHEEL3_OUT2 = 0;
const int WHEEL4_OUT1 = 0;
const int WHEEL4_OUT2 = 0;

#define MIN_SPEED_INPUT 0 
#define MAX_SPEED_INPUT 1000
#define MIN_MOTOR_LIMIT 0
#define MAX_MOTOR_LIMIT 255

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

  Serial.begin(115200);
}

void sendToMotor(int in1_state, int in2_state, int in1, int in2, int pwm_pin, int speed_input){
  if(speed_input < MIN_SPEED_INPUT) speed_input = MIN_SPEED_INPUT;
  else if(speed_input > MAX_SPEED_INPUT) speed_input = MAX_SPEED_INPUT;

  int speed = map(speed_input, MIN_SPEED_INPUT, MAX_SPEED_INPUT, MIN_MOTOR_LIMIT, MAX_MOTOR_LIMIT);
  Serial.print("Speed Input: ");
  Serial.println(speed_input);
  Serial.print("PWM Speed: ");
  Serial.println(speed);

  if(in1_state == in2_state){                          // STOP
    Serial.println("Motor Off");
    speed = 0;
    analogWrite(pwm_pin, speed);
  }
  else if((in1_state == HIGH) && (in2_state == LOW)){  // Forward
    Serial.println("Motor Forward");
    analogWrite(pwm_pin, speed);
  }
  else if((in1_state == LOW) && (in2_state == HIGH)){  //Reverse
    Serial.println(("Motor Reverse"));
    analogWrite(pwm_pin, speed);
  }
  
  digitalWrite(in1, in1_state);
  digitalWrite(in2, in2_state);
}

void loop() {
  for(int i = MIN_SPEED_INPUT; i < MAX_SPEED_INPUT; i += 50){
    sendToMotor(HIGH, LOW, WHEEL1_IN1, WHEEL1_IN2, WHEEL1_EN, i);
    sendToMotor(HIGH, LOW, WHEEL2_IN1, WHEEL2_IN2, WHEEL2_EN, i);
    sendToMotor(HIGH, LOW, WHEEL3_IN1, WHEEL3_IN2, WHEEL3_EN, i);
    sendToMotor(HIGH, LOW, WHEEL4_IN1, WHEEL4_IN2, WHEEL4_EN, i);
    delay(2000);
  }
  for(int j = MAX_SPEED_INPUT; j > MIN_SPEED_INPUT + 50; j -= 50){
    sendToMotor(LOW, HIGH, WHEEL1_IN1, WHEEL1_IN2, WHEEL1_EN, j);
    sendToMotor(LOW, HIGH, WHEEL2_IN1, WHEEL2_IN2, WHEEL2_EN, j);
    sendToMotor(LOW, HIGH, WHEEL3_IN1, WHEEL3_IN2, WHEEL3_EN, j);
    sendToMotor(LOW, HIGH, WHEEL4_IN1, WHEEL4_IN2, WHEEL4_EN, j);
    delay(2000);
  }
  sendToMotor(LOW, LOW, WHEEL1_IN1, WHEEL1_IN2, WHEEL1_EN, 0);
  sendToMotor(LOW, LOW, WHEEL2_IN1, WHEEL2_IN2, WHEEL2_EN, 0);
  sendToMotor(LOW, LOW, WHEEL3_IN1, WHEEL3_IN2, WHEEL3_EN, 0);
  sendToMotor(LOW, LOW, WHEEL4_IN1, WHEEL4_IN2, WHEEL4_EN, 0);
  delay(2000);
}
