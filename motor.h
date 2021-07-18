#include "Arduino.h"
#include "Servo.h"
#include "flight_controller.h"
#include "common_utils.h"

class motor{
  private:
    unsigned int esc_drive_min;   // minimum range values for escs
    unsigned int esc_drive_max;   // maximum range values for escs
    unsigned int esc_drive_float; // value for esc where it will equally contribute to float
    unsigned int esc_drive_reg;   // Current value of esc drive 

    unsigned int model_drive;
    unsigned int model_min;
    unsigned int model_max;
    static unsigned int model_float;

  public:
    Servo ESC;
    motor();
    motor( int pin_no);
    motor(int pin_no, int min_, int max_, int float_);
    void set_pin(int pin_no);
    void operator ++();
    void operator +=(const int val);
    void operator -=(const int val);
    void test(int motor_num_);
    void float_();
    void set_drive(int drive_);
    int get_drive();
    void drive();

};
