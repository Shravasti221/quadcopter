 
#include "Arduino.h"
#include "common_utils.h"
#include<Wire.h>
#include<MPU6050_tockn.h>
#include<Kalman.h>

class gyro{
    float AccX, AccY, AccZ; 
    float gyroWeight = 0.9996;   // AcclWeight will be 1 - gyroWeight
    float dampenWeight = 0.9;    // Used as a dampner parameter
    float currentMpuTime;
    MPU6050 mpu6050;

    gyro():mpu6050(Wire, ACCELEROMETER_COEFF, GYROSCOPE_COEFF){
        AccX = 0;
        AccY = 0;
        AccZ = 0;
        angX = 0;
        angY  = 0;
        angZ = 0;

    }

    void update_mpu(){
        mpu6050.update();
        AccX = mpu6050.getAccX();
        AccY = mpu6050.getAccY());
        AccZ = mpu6050.getAccZ());
    
        mpu6050.getGyroX();
        mpu6050.getGyroY();
        mpu6050.getGyroZ());
    
        mpu6050.getAccAngleX();
        mpu6050.getAccAngleY();
    
        mpu6050.getGyroAngleX();
        mpu6050.getGyroAngleY();
        mpu6050.getGyroAngleZ();
        
        angX = mpu6050.getAngleX();
        angY = mpu6050.getAngleY();
        angZ = mpu6050.getAngleZ();
    }
};