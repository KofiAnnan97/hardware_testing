#include <Arduino.h>
#include "TeensyThreads.h"

// Motor Driver 1 Pins
const int WHEEL1_EN  = 23;
const int WHEEL1_IN1 = 2;
const int WHEEL1_IN2 = 3;
const int WHEEL2_EN  = 22;
const int WHEEL2_IN1 = 4;
const int WHEEL2_IN2 = 5;

#define MIN_SPEED_INPUT 0 
#define MAX_SPEED_INPUT 100
#define MIN_MOTOR_LIMIT 172
#define MAX_MOTOR_LIMIT 255

void setup() {
  /* OUTPUTS */
  pinMode(WHEEL1_EN, OUTPUT);
  pinMode(WHEEL2_EN, OUTPUT);

  pinMode(WHEEL1_IN1, OUTPUT);
  pinMode(WHEEL1_IN2, OUTPUT);
  pinMode(WHEEL2_IN1, OUTPUT);
  pinMode(WHEEL2_IN2, OUTPUT);

  /* INITIAL STATE */
  digitalWrite(WHEEL1_IN1, LOW);
  digitalWrite(WHEEL1_IN2, LOW);
  digitalWrite(WHEEL2_IN1, LOW);
  digitalWrite(WHEEL2_IN2, LOW);

  Serial.begin(115200);
}

void sendToMotor(int in1_state, int in2_state, int in1, int in2, int pwm_pin, float speed_input){
  float scaled_max = MAX_SPEED_INPUT/(float)MAX_SPEED_INPUT;
  if(speed_input < MIN_SPEED_INPUT) speed_input = MIN_SPEED_INPUT;
  else if(speed_input > scaled_max) speed_input = scaled_max;

  int speed = map((int)(speed_input * 100), MIN_SPEED_INPUT, MAX_SPEED_INPUT, MIN_MOTOR_LIMIT, MAX_MOTOR_LIMIT);
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

  Serial.print("Speed Input: ");
  Serial.println(speed_input);
  Serial.print("PWM Speed: ");
  Serial.println(speed);
}

void loop() {
  for(float i = 0.0; i < 1.5; i += 0.1){
    sendToMotor(1, 0, WHEEL1_IN1, WHEEL1_IN2, WHEEL1_EN, i);
    sendToMotor(1, 0, WHEEL2_IN1, WHEEL2_IN2, WHEEL2_EN, i);
    delay(2000);
  }
  for(float j = 1.0; j > -0.5; j -= 0.1){
    sendToMotor(0, 1, WHEEL1_IN1, WHEEL1_IN2, WHEEL1_EN, j);
    sendToMotor(0, 1, WHEEL2_IN1, WHEEL2_IN2, WHEEL2_EN, j);
    delay(2000);
  }
}
