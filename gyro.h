#ifndef GYRO_H 
#define GYRO_H
#include "Arduino.h"
#include "common_utils.h"
#include<Wire.h>
#include<MPU6050_tockn.h>


class gyroscope{
  public:
    float AccX, AccY, AccZ; 
    float Xangle, Yangle, Zangle;
    float gyroWeight;// = 0.9996;   // AcclWeight will be 1 - gyroWeight
    float dampenWeight;// = 0.9;    // Used as a dampner parameter
    float currentMpuTime;
    MPU6050 mpu6050;
    gyroscope():mpu6050(Wire){
        AccX = 0;
        AccY = 0;
        AccZ = 0;
        Xangle = 0;
        Yangle  = 0;
        Zangle = 0;
        Wire.begin();
        mpu6050.begin();
        mpu6050.calcGyroOffsets(true);

    }
    int update_mpu();
    void print_vals();
};
#endif