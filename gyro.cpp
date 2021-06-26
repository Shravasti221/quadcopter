#include "gyro.h"

class gyroscope{
  public:
    float AccX, AccY, AccZ; 
    float Xangle, Yangle, Zangle;
    float gyroWeight;// = 0.9996;   // AcclWeight will be 1 - gyroWeight
    float dampenWeight;// = 0.9;    // Used as a dampner parameter
    float currentMpuTime;
    MPU6050 mpu6050;

    int check_mpu(){
        mpu6050.update();
        AccX = mpu6050.getAccX();
        AccY = mpu6050.getAccY());
        AccZ = mpu6050.getAccZ());
    
        Xangle = mpu6050.getAngleX();
        Yangle = mpu6050.getAngleY();
        Zangle = mpu6050.getAngleZ();
    }

    void print_vals(){
        Serial.print("angleX : ");
        Serial.print(Xangle);
        Serial.print("\tangleY : ");
        Serial.print(Yangle);
        Serial.print("\tangleZ : ");
        Serial.println(Zangle);
    }
};