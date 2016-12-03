#include <SPI.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <PID_v1.h>
#include <math.h>

int motor1Pin = 2;
int motor1Direction = 5;

int motor2Pin = 3;
int motor2Direction = 6;

int motor3Pin = 4;
int motor3Direction = 7;

#define LSM9DS1_M  0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6A // Would be 0x6A if SDO_AG is LOW
#define PRINT_SPEED 250

#define PI 3.141592

#define dt 0.01
unsigned long t0 = 0;
unsigned long t;
long T = 9;

LSM9DS1 imu;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

float mx = 0.0;
float my = 0.0;
float mz = 0.0;

float roll = 0.0;
float pitch = 0.0;

float mRoll = 0.0;
float mPitch = 0.0;
float mYaw = 0.0;

float m1Speed = 0.0;
float m2Speed = 0.0;
float m3Speed = 0.0;

float axSum = 0.0;
float aySum = 0.0;
float azSum = 0.0;
float gxSum = 0.0;
float gySum = 0.0;
float gzSum = 0.0;
float axCalibration = -0.1;
float ayCalibration = 0.0;
float azCalibration = 0.0;
float gxCalibration = -0.16;
float gyCalibration = -0.6;
float gzCalibration = 0.31;

float pitchSum = 0.0;
float rollSum = 0.0;
float pitchCalibration = -4.0;
float rollCalibration = 1.0;

// Complimentary Filter Constant
const float a = 0.02;

float vx = 0.0;
float vy = 0.0;
float vAngle = 0.0;
float vMag = 0.0;

float X_tilt = 0.0;
float Y_tilt = 0.0;

float X_gyro = 0.0;
float Y_gyro = 0.0;

float X = 0.0;
float Y = 0.0;

double setpointX, inX, outX;
double setpointY, inY, outY;

PID xPID(&inX, &outX, &setpointX, 70, 2, 0.1, DIRECT); // 70, 2, 0.1
PID yPID(&inY, &outY, &setpointY, 70, 2, 0.1, DIRECT); // 
unsigned long i = 0;

float calcMotorSpeed(float V_mag, float V_theta, float motorAngle, float w = 0.0, float R = 1.0) {
  float v_k = V_mag*(cos(motorAngle)*sin(V_theta) - sin(motorAngle)*cos(V_theta)) + R*w;
  int sign = (v_k < 0) ? -1 : 1;
  v_k = (abs(v_k) > 255.0) ? sign*255.0 : v_k;
  return v_k;
}

// Calculate Angle
float calcAngle(float xVal, float yVal) {
  // Conditionals required due to -pi to pi limit range of arctan calculation
  if(xVal >= 0.0 && yVal >= 0.0)
    return atan(yVal/xVal);
  else if((xVal < 0.0 && yVal >= 0.0) || (xVal < 0.0 && yVal < 0.0))
    return atan(yVal/xVal) + PI;
  else
    return atan(yVal/xVal) + 2*PI;
}

void setup() {
  
  Serial.begin(115200);
  // Initialize IMU
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
    
    pinMode(motor1Pin, OUTPUT);
    pinMode(motor2Pin, OUTPUT);
    pinMode(motor3Pin, OUTPUT);
    digitalWrite(motor1Pin, LOW);
    digitalWrite(motor2Pin, LOW);
    digitalWrite(motor3Pin, LOW);
    while (1) ;
  }

  // Initialize PID Control Parameters
  setpointX = 0.0;
  setpointY = 0.0;

  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-255, 255);
  xPID.SetSampleTime(T);

  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-255, 255);
  yPID.SetSampleTime(T);

  pinMode(motor1Direction, OUTPUT);
  pinMode(motor2Direction, OUTPUT);
  pinMode(motor3Direction, OUTPUT);
}

void loop() {
  imu.readAccel();
  ax = imu.calcAccel(imu.ax) + axCalibration;
  ay = imu.calcAccel(imu.ay) + ayCalibration;
  az = imu.calcAccel(imu.az) + azCalibration;

  imu.readGyro();
  gx = imu.calcGyro(imu.gx) + gxCalibration;
  gy = imu.calcGyro(imu.gy) + gyCalibration;
  gz = imu.calcGyro(imu.gz) + gzCalibration;

  imu.readMag();
  mx = imu.calcMag(imu.mx);
  my = imu.calcMag(imu.my);
  mz = imu.calcMag(imu.mz);

  roll = atan2(ay, az)*180.0/PI + rollCalibration;
  pitch = atan2(-ax, sqrt(ay*ay + az*az))*180.0/PI + pitchCalibration;
  
  Y_tilt = -roll;
  X_tilt = -pitch;
  
  Y_gyro = gx;
  X_gyro = gy;
  
  t = millis();
  if(t - t0 >= T) {
    t0 = t;

    vx = (1.0 - a)*(vx + X_gyro*dt) + a*X_tilt; //pitch
    vy = (1.0 - a)*(vy + Y_gyro*dt) + a*Y_tilt; //roll

    inX = vx;
    xPID.Compute();
    inY = vy;
    yPID.Compute();

    vMag = sqrt(outX*outX + outY*outY);
    vAngle = calcAngle(outX, outY);
    
    m1Speed = calcMotorSpeed(vMag, vAngle, PI/2.0);
    m2Speed = calcMotorSpeed(vMag, vAngle, PI+(PI/6.0));
    m3Speed = calcMotorSpeed(vMag, vAngle, 3.0*PI/2.0+(PI/3.0));

    if(m1Speed > 0.0)
      digitalWrite(motor1Direction, LOW);
      //m1.run(FORWARD);
    else
      digitalWrite(motor1Direction, HIGH);
      //m1.run(BACKWARD);
    
    if(m2Speed > 0.0)
      digitalWrite(motor2Direction, LOW);
      //m2.run(FORWARD);
    else
      digitalWrite(motor2Direction, HIGH);
      //m2.run(BACKWARD);
    
    if(m3Speed > 0.0)
      digitalWrite(motor3Direction, LOW);
      //m3.run(FORWARD);
    else
      digitalWrite(motor3Direction, HIGH);
      //m3.run(BACKWARD);

  analogWrite(motor1Pin, abs(m1Speed));
  analogWrite(motor2Pin, abs(m2Speed));
  analogWrite(motor3Pin, abs(m3Speed));
  }
//  delay(PRINT_SPEED);
}




