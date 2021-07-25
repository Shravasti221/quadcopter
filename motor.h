#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"
#include <Servo.h>

#define MIN_ESC_DRIVE  1
#define MAX_ESC_DRIVE  180

#define MODEL_MAX 100
#define MODEL_MIN 20
#define MODEL_FLOAT 40

//short for minimum/maximum ESC PWM in milli seconds
#define MIN_ESC_PWM_IN_MS    900
#define MAX_ESC_PWM_IN_MS    2000
 
#define FLOAT_ESC_PWM_IN_MS  1400   
//the value for which the drone is assumed to stay flying in one place
class motor {
  private:
    unsigned int esc_drive_min;   // minimum range values for escs
    unsigned int esc_drive_max;   // maximum range values for escs
    unsigned int esc_drive_float; // value for esc where it will equally contribute to float
    unsigned int esc_drive_reg;   // Current value of esc drive 

    unsigned int model_drive;
    unsigned int model_min;
    unsigned int model_max;

  public:
    Servo ESC;
    motor();
    motor(int pin_no);
    motor(int pin_no, int min_, int max_, int float_);
    void set_pin(int pin_no);
    motor& operator ++();
    motor& operator --();
    motor& operator +=(const int val);
    motor& operator -=(const int val);
    void test(int motor_num_);
    void float_();
    void set_model_drive(int drive_);
    int get_model_drive();
    void drive();

};
#endif
