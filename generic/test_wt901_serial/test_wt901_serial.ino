#include <Arduino.h>
#include <REG.h>
#include <wit_c_sdk.h>

/*
Tested on Arduino MEGA 2560 and Teensy 4.0 (Hardware Serial ports 1, 2, and 3) using the WT901B sensor. 
Based on the sample code from WitMotion Arduino SDK found here: 
https://github.com/WITMOTION/WitStandardModbus_WT901C485/tree/main/Arduino
*/

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

#define imuSerial Serial4

#define DEBUG 1

struct Quaternion{
    float w, x, y, z;
};

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 
const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};

int i;
float fAcc[3], fGyro[3], fAngle[3];

//unsigned long startTime;

/* ROS IMU Message */
Quaternion orientation;
float *angularVelocity = fGyro;     // Units: [m/s^2]
float linearAcceleration[3];        // Units: [rad/s]

/* Sensor Functions */
static void AutoScanSensor(void);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
//static void updateAngularVelocity(float *gyro);

/* Conversion Functions */
static float ToRadians(float degree);
static float ToDegrees(float radian);
static Quaternion ToQuaternion(float roll, float pitch, float yaw);

void setup() {
  Serial.begin(115200);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  Serial.print("\r\n********************** WT901 Start ************************\r\n");
  AutoScanSensor();
  startTime = millis();
}

void loop() {
    while (imuSerial.available()){
      WitSerialDataIn(imuSerial.read());
    }
    if(s_cDataUpdate){
      for(i = 0; i < 3; i++){
        fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
        fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
        linearAcceleration[i] = ToRadians(fGyro[i]);
        fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
      }
      if(s_cDataUpdate & ACC_UPDATE){
        if(DEBUG == 1){
          Serial.print("Linear acc   | x: ");
          Serial.print(fAcc[0], 3);
          Serial.print(" y: ");
          Serial.print(fAcc[1], 3);
          Serial.print(" z: ");
          Serial.println(fAcc[2], 3);
        }
        s_cDataUpdate &= ~ACC_UPDATE;
      }
      if(s_cDataUpdate & GYRO_UPDATE){
        if(DEBUG == 1){
          Serial.print("Angular Vel  | x: ");
          Serial.print(linearAcceleration[0], 1);
          Serial.print(" y: ");
          Serial.print(linearAcceleration[1], 1);
          Serial.print(" z: ");
          Serial.println(linearAcceleration[2], 1);
          Serial.print("Gyro         | x: ");
          Serial.print(fGyro[0], 1);
          Serial.print(" y: ");
          Serial.print(fGyro[1], 1);
          Serial.print(" z: ");
          Serial.println(fGyro[2], 1);
        }
        s_cDataUpdate &= ~GYRO_UPDATE;
      }
      if(s_cDataUpdate & ANGLE_UPDATE){
        // Convert data to orientation
        float roll = ToRadians(fAngle[0]);
        float pitch = ToRadians(fAngle[1]);
        float yaw =  ToRadians(fAngle[2]);
        orientation = ToQuaternion(roll, pitch, yaw);

        if(DEBUG == 1){
          Serial.print("Angle:       | x: ");
          Serial.print(fAngle[0], 3);
          Serial.print(" y: ");
          Serial.print(fAngle[1], 3);
          Serial.print(" z: ");
          Serial.println(fAngle[2], 3);
          Serial.print("Orientation  | x: ");
          Serial.print(orientation.x);
          Serial.print(" y:");
          Serial.print(orientation.y);
          Serial.print(" z:");
          Serial.print(orientation.z);
          Serial.print(" w:");
          Serial.println(orientation.w);
        }
        s_cDataUpdate &= ~ANGLE_UPDATE;
      }
      if(s_cDataUpdate & MAG_UPDATE){
        if(DEBUG == 1){
          Serial.print("Magnetometer | x:");
          Serial.print(sReg[HX]);
          Serial.print(" y: ");
          Serial.print(sReg[HY]);
          Serial.print(" z: ");
          Serial.println(sReg[HZ]);
        }
        s_cDataUpdate &= ~MAG_UPDATE;
      }
      s_cDataUpdate = 0;
    }
}

static void AutoScanSensor(void){
  int i, iRetry;
  
  for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++){
    imuSerial.begin(c_uiBaud[i]);
    imuSerial.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do{
      WitReadReg(AX, 3);
      delay(200);
      while (imuSerial.available()){
        WitSerialDataIn(imuSerial.read());
      }
      if(s_cDataUpdate != 0){
        Serial.print(c_uiBaud[i]);
        Serial.print(" baud find sensor\r\n\r\n");
        return;
      }
      iRetry--;
    }while(iRetry);		
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum){
  int i;
    for(i = 0; i < uiRegNum; i++){
      switch(uiReg){
        case AZ:
          s_cDataUpdate |= ACC_UPDATE;
          break;
        case GZ:
          s_cDataUpdate |= GYRO_UPDATE;
          break;
        case HZ:
          s_cDataUpdate |= MAG_UPDATE;
          break;
        case Yaw:
          s_cDataUpdate |= ANGLE_UPDATE;
          break;
        default:
          s_cDataUpdate |= READ_UPDATE;
          break;
      }
      uiReg++;
    }
}

static void Delayms(uint16_t ucMs){
  delay(ucMs);
}

/*static void updateAngularVelocity(float *gyro){
  unsigned long endTime = millis();
  float dt = (endTime - startTime)/1000.0;
  startTime = endTime;
  angularVelocity[0] += gyro[0]*dt;
  angularVelocity[1] += gyro[1]*dt;
  angularVelocity[2] += gyro[2]*dt;

  if(DEBUG == 1){
    Serial.print("Agular Vel | x: ");
    Serial.print(angularVelocity[0]);
    Serial.print(" y: ");
    Serial.print(angularVelocity[1]);
    Serial.print(" z: ");
    Serial.println(angularVelocity[2]);
  }
}*/

static float ToRadians(float degree){
  return degree*(M_PI/180);
}

static float ToDegrees(float radian){
  return radian*(180/M_PI);
}

static Quaternion ToQuaternion(float roll, float pitch, float yaw){
  float cr = cos(roll/2);
  float cp = cos(pitch/2);
  float cy = cos(yaw/2);
  float sr = sin(roll/2);
  float sp = sin(pitch/2);
  float sy = sin(yaw/2);

  Quaternion q;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  return q;
}