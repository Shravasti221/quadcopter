#include "flight_controller.h"

#if _CONSTRAINED
  int UB = MAX_ESC_DRIVE;
  int LB = MIN_ESC_DRIVE;
#endif
int flight_controller::mod2 = 0;
void flight_controller::calculate_flight_targets() {
  //throttle value decides mean speed
  motor_throttle = map(rc_throttle(), MIN_RC_THROTTLE, MAX_RC_THROTTLE, MODEL_MIN, MODEL_MAX);

  x_tilt = map(rc_elevator(), MIN_RC_ELEVATOR, MAX_RC_ELEVATOR, -5, 5);//forward -X
  y_tilt = map(rc_aileron(), MIN_RC_ROLL, MAX_RC_ROLL, -5, 5);//left -Y
  z_tilt = map(rc_rudder(), MIN_RC_THROTTLE, MAX_RC_THROTTLE, -50, 50);//clockwise -Z
}

void flight_controller::reset2float(){
  for(int i = 0; i<4; i++)
    motors[i].float_();
}

void flight_controller::reset2throttler(){
  for(int i = 0; i<4; i++)
    motors[i].set_drive(motor_throttle);
}
/********************************************************************/
void flight_controller::lost_control() {
  if (!is_rc_alive()) {
    // In failsafe mode, so drive to a lower and slow falling mode.
    #if DEBUG
      Serial.println("RC data has not been received going into failsafe mode");
    #endif
    for (int i = 0; i < 4; i++)
      motors[i].set_model_drive( MODEL_FLOAT - 5);

    #if !DEBUG
      indicate_off();
    #endif
    return;
  } 
  // Seems to be OK, so let's continue the normal way
  indicate_glow();
}

void flight_controller::get_gyro_values(gyroscope &gyro){
  Xangle = gyro->Xangle;
  Yangle = gyro->Yangle;
  Zangle = gyro->Zangle;
}
void flight_controller::set_model_drives(){
  int motor_avg = 0; //local variable
  for(int i = 0; i< 4; i++){
    motor_avg += motors[i].getdrive(); 
  }
  motor_avg = motor_avg <<2;

  //int threshold = 4 * (x_tilt + y_tilt + z_tilt); // i'll add this once I add z

  if(threshold>10) threshold = 10;

  //if the average speed of all the motor is too dfferent from the throttle speed reset the motor speeds to throttle speed.
  if (!( (throttle-10) <= motor_avg || motor_avg <= (throttle+10) ))
    reset2throttle();
  //if it is toppling over
  if(abs(x_tilt) > MAX_TILT_ANGLE || abs(y_tilt) > MAX_TILT_ANGLE || abs(z_tilt) > MAX_TILT_ANGLE){
    reset2float();
  }

  mod2 = (mod2 + 1)%2; //for chosing which pair of motors to control

  if(Xangle>x_tilt) //I need to bend backwards
  {
    if(mod2){
      motors[LF_ESC_INDEX]++;
      motors[RF_ESC_INDEX]++;
    }
    else{
      motors[LR_ESC_INDEX]--;
      motors[RR_ESC_INDEX]--;
    }
    
  }  
  else if(Xangle < x_tilt) //I need to bend forwards (bend forward is -x)
  {
    if(mod2){
      motors[LF_ESC_INDEX]++;
      motors[RF_ESC_INDEX]++;
    }
    else{
      motors[LR_ESC_INDEX]--;
      motors[RR_ESC_INDEX]--;
    }
  }
  
  if(Yangle>y_tilt) //I need to bend backwards
  {
    if(mod2){
      motors[LF_ESC_INDEX]++;
      motors[LR_ESC_INDEX]++;
    }
    else{
      motors[RF_ESC_INDEX]--;
      motors[RR_ESC_INDEX]--;
    }
  }  
  else if(Yangle < y_tilt) //I need to bend forwards
  {
    if(mod2){
      motors[LF_ESC_INDEX]++;
      motors[LR_ESC_INDEX]++;
    }
    else{
      motors[RF_ESC_INDEX]--;
      motors[RR_ESC_INDEX]--;
    }
  }

  /*if(Zangle>z_tilt) //I need to bend backwards
  {
    if(mod2){
      motors[LF_ESC_INDEX]++;
      motors[RF_ESC_INDEX]++;
    }
    else{
      motors[LR_ESC_INDEX]--;
      motors[RR_ESC_INDEX]--;
    }
  }  
  else if(Zangle < z_tilt) //I need to bend forwards
  {
    if(mod2){
      motors[LF_ESC_INDEX]++;
      motors[RF_ESC_INDEX]++;
    }
    else{
      motors[LR_ESC_INDEX]--;
      motors[RR_ESC_INDEX]--;
    }
  }*/    
    
}

void flight_controller::drive(){
  for(int i = 0; i<4; i++)
    motors[i].drive();
}

void flight_controller::print_vals(){
  Serial.print("Motor LF speed: ");Serial.println(motors[LF_ESC_INDEX]);
  Serial.print("\t RF speed: ");Serial.println(motors[RF_ESC_INDEX]);
  Serial.print("\t LR speed: ");Serial.println(motors[LR_ESC_INDEX]);
  Serial.print("\t LF speed: ");Serial.println(motors[RR_ESC_INDEX]);
}

void flight_controller::run_flight_controller() {
/*set a max angle above which we can't tilt the drone. 
when we move it and it starts tilting if the tilt angle 
is more than the max value then we set the drone to float 
*/
  gyro.check_mpu();
  set-model_drives();
  drive();
}
