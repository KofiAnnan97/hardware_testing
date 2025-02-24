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

void sendToMotor(wheelState ws, int in1_state, int in2_state, double speed_input){
  double scaled_max = MAX_SPEED_INPUT/(double)MAX_SPEED_INPUT;
  if(speed_input < MIN_SPEED_INPUT) speed_input = MIN_SPEED_INPUT;
  else if(speed_input > scaled_max) speed_input = scaled_max;

  int speed = map((int)(speed_input * 100), MIN_SPEED_INPUT, MAX_SPEED_INPUT, ws.MIN_PWM_LIMIT, ws.MAX_PWM_LIMIT);
  if(in1_state == in2_state){                          // STOP
    Serial.println("Motor Off");
    speed = 0;
    analogWrite(ws.pwm_pin, speed);
  }
  else if((in1_state == HIGH) && (in2_state == LOW)){  // Forward
    Serial.println("Motor Forward");
    analogWrite(ws.pwm_pin, speed);
  }
  else if((in1_state == LOW) && (in2_state == HIGH)){  //Reverse
    Serial.println(("Motor Reverse"));
    analogWrite(ws.pwm_pin, speed);
  }
  digitalWrite(ws.in1_pin, in1_state);
  digitalWrite(ws.in2_pin, in2_state);

  Serial.print("Speed Input: ");
  Serial.println(speed_input);
  Serial.print("PWM Speed: ");
  Serial.println(speed);
}

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
}

/*float *from_twist_to_pwm(double power, double theta, double turn){
  float wheels[4];

  double v_sin = sin(theta - (M_PI/4));
  double v_cos = cos(theta - (M_PI/4));
  float v_max = max(fabs(sin), fabs(cos));

  float wheel_1 = power*v_cos/v_max+turn;
  float wheel_2 = power*v_sin/v_max-turn;
  float wheel_3 = power*v_cos/v_max-turn;
  float wheel_4 = power*v_sin/v_max+turn;

  if(power+abs(turn)  > 1){
    wheel_1 /= power + turn;
    wheel_2 /= power + turn;
    wheel_3 /= power + turn;
    wheel_4 /= power + turn;
  }
  wheels[0] = wheel_1;
  wheels[1] = wheel_2;
  wheels[2] = wheel_3;
  wheels[3] = wheel_4;

  return wheels;
}*/

void loop() {
  // Check that motors speed up and slow down properly
  //speedTest();
  // Check that mecanum wheel configuration can turn and strafe
  //movementTest();
  // Send commands to mecanum wheels based on power, degree and turn
  // from_twist_to_pwm(0.5, 90, 0.0);
}

