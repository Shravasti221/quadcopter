#include "Arduino.h"
#include "Servo.h"
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

    motor(){
      esc_drive_min = MIN_ESC_DRIVE;
      esc_drive_max = MAX_ESC_DRIVE; 
      esc_drive_float = 50;

      model_drive = 0;
      model_min = MODEL_MIN;
      model_max  = 100;
    }
    motor( int pin_no){
      esc_drive_min = MIN_ESC_DRIVE;
      esc_drive_max = MAX_ESC_DRIVE;
      esc_drive_float = 50;

      model_drive = 0;
      model_min = MODEL_MIN;
      model_max  = MODEL_MAX;

      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    motor(int pin_no, int min_, int max_, int float_){
      esc_drive_min = min_;
      esc_drive_max = max_;
      esc_drive_float = float_;

      model_drive = 0;
      model_min = MODEL_MIN;
      model_max  = MODEL_MAX;

      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    void set_pin(int pin_no){
      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    void operator ++(){
      if(model_drive < model_max) 
        model_drive++;
    }
    void operator +=(const int val){
      if(model_drive + val <= model_max)
        model_drive += val;
    }
    void operator -=(const int val){
      if(model_drive - val >= model_min) 
        model_drive -= val;
    }

    void test(int motor_num_){
      indicate_glow();
       #if DEBUG
        Serial.print("Testing motor number ");  
      #endif
      
      Serial.println(motor_num_);
      ESC.write(20); //(esc_drive_float[LR_ESC_INDEX]);  //Rear Left
      delay(2000);
      indicate_off();
      ESC.write(1);
      delay(1000);
    }

    void float_(){
      model_drive = model_float;
    }


    void set_drive(int drive_){
      model_drive = drive_;
    }

    int get_drive(){
      return model_drive;
    }

    void drive(){
      esc_drive_reg = map(model_drive, model_min, model_max, esc_drive_min, esc_drive_max);
      #if _CONSTRAINED
        constrain (esc_drive_reg, LB, UB);
      #endif
      ESC.write(esc_drive_reg);
    }

};
