#include "Attitude.h"
#include <Wire.h>
#include <math.h>

Attitude::Attitude(){}

void Attitude::init(void)
{
    Wire.begin();
    setupMPU();
    setupMAG();
}

void Attitude::setupMPU(void)
{
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

// Setup of QMC5883L 3-axis Compass 
void Attitude::setupMAG(void)
{
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_MODE_REG);
    // Continuous mode with +/-2 Gauss
    Wire.write(0b00001101); 
    Wire.endTransmission();
}

Vector Attitude::rawAccel(void)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR,6);

    while(Wire.available() < 6);
    
        acc_x_h = Wire.read();
        acc_x_l = Wire.read();
        acc_y_h = Wire.read();
        acc_y_l = Wire.read();
        acc_z_h = Wire.read();
        acc_z_l = Wire.read();
    

    rawAcc.x = (acc_x_h << 8)|acc_x_l;
    rawAcc.y = (acc_y_h << 8)|acc_y_l;
    rawAcc.z = (acc_z_h << 8)|acc_z_l;

    return rawAcc;
}

Vector Attitude::normAccel(void)
{
    rawAcc = rawAccel();
    
    normAcc.x = rawAcc.x*g*0.000061035;
    normAcc.y = rawAcc.y*g*0.000061035;
    normAcc.z = rawAcc.z*g*0.000061035;

    return normAcc;
}

Vector Attitude::rawGyro(void)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR,6);

    while(Wire.available() < 6);
    
        gyr_x_h = Wire.read();
        gyr_x_l = Wire.read();
        gyr_y_h = Wire.read();
        gyr_y_l = Wire.read();
        gyr_z_h = Wire.read();
        gyr_z_l = Wire.read();
    

    rawGyr.x = (gyr_x_h << 8)|gyr_x_l;
    rawGyr.y = -((gyr_y_h << 8)|gyr_y_l);
    rawGyr.z = -((gyr_z_h << 8)|gyr_z_l);

    return rawGyr;
}

Vector Attitude::normGyro(void)
{
    rawGyr = rawGyro();
    
    normGyr.x = rawGyr.x*0.00763359;
    normGyr.y = rawGyr.y*0.00763359;
    normGyr.z = rawGyr.z*0.00763359;

    return normGyr;
}

Vector Attitude::rawMagneto(void)
{
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_OUT_X_L);
    Wire.endTransmission();

    Wire.requestFrom(MAG_ADDR, 6);
    while(Wire.available() >= 6)
    {
        mag_x_l = Wire.read();
        mag_x_h = Wire.read();
        mag_y_l = Wire.read();
        mag_y_h = Wire.read();
        mag_z_l = Wire.read();
        mag_z_h = Wire.read();
    }

    rawMag.x = (mag_x_h << 8)|mag_x_l;
    rawMag.y = (mag_y_h << 8)|mag_y_l;
    rawMag.z = (mag_z_h << 8)|mag_z_l;

    return rawMag; 
}

Vector Attitude::normMagneto(void)
{
    rawMag = rawMagneto();

    mag_x = rawMag.x*0.0000833333;
    mag_y = rawMag.y*0.0000833333;
    mag_z = rawMag.z*0.0000833333;

    float calib[3];
    float raw[3] = {mag_x, mag_y, mag_z};

    // float bias[3] = {0.368478,0.132804,-0.217310};
    // float scale[3][3] = {{0.971691, -0.037843, -0.027270},
    //                      {-0.037843, 0.899053, -0.000747},
    //                      {-0.027270, -0.000747, 0.899432}};

    float bias[3] = {0.376457,0.119874,-0.165460};
    float scale[3][3] = {{0.880161, -0.029439, -0.022056},
                         {-0.029439, 0.969466, 0.014079},
                         {-0.022056, 0.014079, 0.949999}};

    for (int i = 0; i < 3; i++)
    {
        calib[i] = raw[i] - bias[i];
    }

    for (int i = 0; i < 3; i++)
    {
        calib[i] = scale[i][0]*calib[0] + scale[i][1]*calib[1] + scale[i][2]*calib[2];
    }

    normMag.x = calib[0];
    normMag.y = calib[1];
    normMag.z = calib[2];

    return normMag;
}

int Attitude::getHeading(void)
{
  normMag = normMagneto();
  yaw = atan2(normMag.y,normMag.x)*180/M_PI;
  return yaw < 0 ? 360 + yaw:yaw;
}

Vector Attitude::getEuler(void)
{
    normAcc = normAccel();
    normGyr = normGyro();

    previousTime = currentTime;
    currentTime = millis();
    timeStep = (currentTime - previousTime)/1000;

    // Transform body frame to NED frame (Euler rate)
    roll_dot = normGyr.x + sinf(roll*M_PI/180)*tanf(pitch*M_PI/180)*normGyr.y + cosf(roll*M_PI/180)*tanf(pitch*M_PI/180)*normGyr.z;
    pitch_dot = normGyr.y*cosf(roll*M_PI/180) - sinf(roll*M_PI/180)*normGyr.z;
    yaw_dot = normGyr.y*sinf(roll*M_PI/180)/cosf(pitch*M_PI/180) + normGyr.z*cosf(roll*M_PI/180)/cosf(pitch*M_PI/180);

    // Attitude estimation from gyroscope
    roll_gyro = roll + roll_dot * timeStep; // (roll)n+1 = (roll)n + Gyro_rate * delta_T
    pitch_gyro = pitch + pitch_dot * timeStep;

    roll_accel = atanf(normAcc.y/normAcc.z)*180/M_PI;
    pitch_accel = atanf(normAcc.x/sqrtf(pow(normAcc.y,2)+pow(normAcc.z,2)))*180/M_PI;

    // Apply sensor fusion with complimentary filter
    roll = 0.95*roll_accel + 0.05*roll_gyro;
    pitch = 0.95*pitch_accel + 0.05*pitch_gyro;

    Euler.x = roll;
    Euler.y = pitch;
    Euler.z = getHeading();

    return Euler;
}

void Attitude::transmitData(float x, float y, float z)
{
    uint8_t header = 0xff;
    uint8_t dataLength = 0x0c;
    int bufferSize = 14;
    uint8_t bufferData[bufferSize];

    bufferData[0] = header;
    bufferData[1] = dataLength;

    for (int i = 0; i < sizeof(x); i++)
    {
        bufferData[i+2] = ((uint8_t*)&x)[i];
        bufferData[i+6] = ((uint8_t*)&y)[i];
        bufferData[i+10] = ((uint8_t*)&z)[i];
    }
    
    Serial.write(bufferData, bufferSize);
}