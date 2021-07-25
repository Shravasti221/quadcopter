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

  public:
    void setup();  
    int update_mpu();
    void print_vals();
    int check_mpu();
    gyroscope();
};
#endif
