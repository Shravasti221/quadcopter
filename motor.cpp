#include "motor.h"
#include "common_utils.h"

motor::motor(){
  esc_drive_min = MIN_ESC_DRIVE;
  esc_drive_max = MAX_ESC_DRIVE; 
  esc_drive_float = 50;

  model_drive = 0;
  model_min = MODEL_MIN;
  model_max  = 100;
}

motor::motor( int pin_no){
  esc_drive_min = MIN_ESC_DRIVE;
  esc_drive_max = MAX_ESC_DRIVE;
  esc_drive_float = 50;

  model_drive = 0;
  model_min = MODEL_MIN;
  model_max  = MODEL_MAX;

  ESC.attach(pin_no, esc_drive_min, esc_drive_max);
}

motor::motor(int pin_no, int min_, int max_, int float_){
  esc_drive_min = min_;
  esc_drive_max = max_;
  esc_drive_float = float_;

  model_drive = 0;
  model_min = MODEL_MIN;
  model_max  = MODEL_MAX;

  ESC.attach(pin_no, esc_drive_min, esc_drive_max);
}

void motor::set_pin(int pin_no){
  ESC.attach(pin_no, esc_drive_min, esc_drive_max);
  ESC.write(0);
  delay(100);
}

motor& motor::operator ++(){
  if(model_drive < model_max) 
    model_drive++;
}

motor& motor::operator --(){
  if(model_drive > model_min) 
    model_drive--;
}

motor& motor::operator +=(const int val){
  if(model_drive + val <= model_max)
    model_drive += val;
}
motor& motor::operator -=(const int val){
  if(model_drive - val >= model_min) 
    model_drive -= val;
}

void motor::test(int motor_num_){
  indicate_glow();
  #if DEBUG
    Serial.print("Testing motor "); Serial.println(motor_num_);
  #endif
  
  ESC.write(50); //(esc_drive_float[LR_ESC_INDEX]);  //Rear Left
  delay(500);
  indicate_off();
  ESC.write(1);
  delay(250);
}

void motor::float_(){
  model_drive = MODEL_FLOAT;
}


void motor::set_model_drive(int drive_){
  model_drive = drive_;
}

int motor::get_model_drive(){
  return model_drive;
}

void motor::drive(){
  esc_drive_reg = map(model_drive, model_min, model_max, esc_drive_min, esc_drive_max);
  #if _CONSTRAINED
    constrain (esc_drive_reg, LB, UB);
  #endif
  ESC.write(esc_drive_reg);
}
