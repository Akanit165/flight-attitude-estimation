/* 
==== Prototype of Read Euler From MPU6050 ====

1. Read acceleration in body fixed frame from accelerometer
2. Read angular velocity in body fixed frame from gyroscope
3. Implement sensor fusion by using complimentary filter

Pitch and Roll ==> From gyroscope and accelerometer
Yaw ==> From gyroscope only

Next phase
- Create library
- Include bias and noise manipulation
- Apply EKF to optimize accel/gyro constant

*/

#include<Wire.h>
#include<math.h>

#define MPU_ADDR 0x68
#define PWR_MGMT 0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

float timeStep, currentTime, previousTime;
float roll_dot, pitch_dot, yaw_dot;
float roll_gyro   = 0;
float pitch_gyro  = 0;
float roll_accel  = 0;
float pitch_accel = 0;
float roll        = 0; 
float pitch       = 0; 
float yaw         = 0;
float alpha       = 0;
float g           = 9.8067;

typedef struct Vector
{
  float x, y, z;
} Vector;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  setupMPU();
}

void setupMPU() {
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT);
  Wire.write(0b00000000);
  Wire.endTransmission();

  // Set configuration of gyroscope to scale of +/-250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0b00000000);
  Wire.endTransmission();

  // Set configuration of accelerometer to +/- 2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0b00000000);
  Wire.endTransmission();

}

void getEuler() 
{
  // Get time step due to discrete system
  previousTime = currentTime;
  currentTime = millis();
  timeStep = (currentTime - previousTime)/1000; // Convert to second

  Vector gyro;

  // Call data from gyroscope
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR,6);

  while (Wire.available()<6);
  
  // Angular rate in body frame

  uint8_t gxHigh = Wire.read();
  uint8_t gxLow  = Wire.read();
  uint8_t gyHigh = Wire.read();
  uint8_t gyLow  = Wire.read();
  uint8_t gzHigh = Wire.read();
  uint8_t gzLow  = Wire.read();

  gyro.x = ((gxHigh << 8) | gxLow)/131;
  gyro.y = -(((gyHigh << 8) | gyLow)/131);
  gyro.z = -(((gzHigh << 8) | gzLow)/131);

  // Transform body frame to NED frame (Euler rate)
  roll_dot = gyro.x + sinf(roll*M_PI/180)*tanf(pitch*M_PI/180)*gyro.y + cosf(roll*M_PI/180)*tanf(pitch*M_PI/180)*gyro.z;
  pitch_dot = gyro.y*cosf(roll*M_PI/180) - sinf(roll*M_PI/180)*gyro.z;
  yaw_dot = gyro.y*sinf(roll*M_PI/180)/cosf(pitch*M_PI/180) + gyro.z*cosf(roll*M_PI/180)/cosf(pitch*M_PI/180);

  // Attitude estimation from gyroscope
  roll_gyro = roll + roll_dot * timeStep; // (roll)n+1 = (roll)n + Gyro_rate * delta_T
  pitch_gyro = pitch + pitch_dot * timeStep;
  yaw = yaw + yaw_dot * timeStep; // Must include magnetometer

  // Read data from accelerometer
  Vector accel;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR,6);

  while(Wire.available() < 6);

  uint8_t axHigh = Wire.read();
  uint8_t axLow  = Wire.read();
  uint8_t ayHigh = Wire.read();
  uint8_t ayLow  = Wire.read();
  uint8_t azHigh = Wire.read();
  uint8_t azLow  = Wire.read();

  accel.x = ((axHigh << 8) | axLow)*g/16384;
  accel.y = ((ayHigh << 8) | ayLow)*g/16384;
  accel.z = ((azHigh << 8) | azLow)*g/16384;

  roll_accel = atanf(accel.y/accel.z)*180/M_PI;
  pitch_accel = atanf(accel.x/sqrtf(pow(accel.y,2)+pow(accel.z,2)))*180/M_PI;

  // Apply sensor fusion with complimentary filter
  roll = 0.95*roll_accel + 0.05*roll_gyro;
  pitch = 0.95*pitch_accel + 0.05*pitch_gyro;

  /*
  Serial.print("ROLL (deg) = ");
  Serial.print(roll);
  Serial.print("  PITCH (deg) = ");
  Serial.print(pitch);
  Serial.print("  YAW (deg) = ");
  Serial.println(yaw);
  */
  transmitData(yaw, pitch, roll);
}

void transmitData(float yaw, float pitch, float roll)
{
  uint8_t header = 0xff;
  uint8_t dataLength = 0x0c;
  int bufferSize = 14;
  uint8_t bufferData[bufferSize];

  bufferData[0] = header;
  bufferData[1] = dataLength;

  for (int i = 0; i < sizeof(yaw); i++)
  {
    bufferData[i+2] = ((uint8_t*)&yaw)[i];
    bufferData[i+6] = ((uint8_t*)&pitch)[i];
    bufferData[i+10] = ((uint8_t*)&roll)[i];
  }
  Serial.write(bufferData, bufferSize);
}

void loop()
{
  getEuler();
  delay(25);
}

