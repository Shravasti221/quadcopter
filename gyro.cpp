#include "gyro.h"

gyroscope::gyroscope():mpu6050(Wire){
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

int gyroscope::check_mpu(){
        mpu6050.update();
        AccX = mpu6050.getAccX();
        AccY = mpu6050.getAccY();
        AccZ = mpu6050.getAccZ();
    
        Xangle = mpu6050.getAngleX();
        Yangle = mpu6050.getAngleY();
        Zangle = mpu6050.getAngleZ();
}

void gyroscope::print_vals(){
        Serial.print("\nangleX : ");
        Serial.print(Xangle);
        Serial.print("\tangleY : ");
        Serial.print(Yangle);
        Serial.print("\tangleZ : ");
        Serial.println(Zangle);
}
