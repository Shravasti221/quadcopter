#include "gyro.h"

gyroscope::gyroscope():mpu6050(Wire, 0.1, 0.9){
        AccX = 0;
        AccY = 0;
        AccZ = 0;
        Xangle = 0;
        Yangle  = 0;
        Zangle = 0;
}

void gyroscope::setup() {
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
        Serial.print(" angleX : "); Serial.print(Xangle);
        Serial.print(" angleY : "); Serial.print(Yangle);
        // Serial.print(" angleZ : "); Serial.println(Zangle);
}
