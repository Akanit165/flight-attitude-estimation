/*

  Attitude.h - Library for MPU6050 Accelerometer/Gyroscope and QMC5883L Compass.
  Created by Akanit165, November 8, 2022.

*/

#ifndef Attitude_h
#define Attitude_h
#include "Arduino.h"
#include <Wire.h>

#define MPU_ADDR        (0x68)
#define PWR_MGMT        (0x6B)
#define GYRO_CONFIG     (0x1B)
#define GYRO_XOUT_H     (0x43)
#define ACCEL_CONFIG    (0x1C)
#define ACCEL_XOUT_H    (0x3B)

#define MAG_ADDR        (0x0D)
#define MAG_OUT_X_L     (0x00)
#define MAG_MODE_REG    (0x09)

typedef struct Vector
{
    float x, y, z;
} Vector;

class Attitude
{
    public:

        Attitude();
        void   init(void);
        Vector rawAccel(void);
        Vector rawGyro(void);
        Vector rawMagneto(void);
        Vector normAccel(void);
        Vector normGyro(void);
        Vector normMagneto(void);
        int    getHeading();
        Vector getEuler();
        void   transmitData(float x, float y, float z);
        

    private:

        void setupMPU(void);
        void setupMAG(void);

        Vector Euler;
        Vector rawAcc, normAcc;
        Vector rawGyr, normGyr;
        Vector rawMag, normMag;

        uint8_t acc_x_h, acc_x_l, acc_y_h, acc_y_l, acc_z_h, acc_z_l; 
        uint8_t gyr_x_h, gyr_x_l, gyr_y_h, gyr_y_l, gyr_z_h, gyr_z_l;
        uint8_t mag_x_h, mag_x_l, mag_y_h, mag_y_l, mag_z_h, mag_z_l;

        float timeStep, currentTime, previousTime;
        float roll_dot, pitch_dot, yaw_dot;
        float mag_x, mag_y, mag_z;
        float roll_gyro   = 0;
        float pitch_gyro  = 0;
        float roll_accel  = 0;
        float pitch_accel = 0;
        float roll        = 0; 
        float pitch       = 0; 
        int   yaw;
        float alpha       = 0;
        float g           = 9.8067;
};

#endif
