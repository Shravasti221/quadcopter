#include "flight_controller.h"

#if _CONSTRAINED
  int UB = MAX_ESC_DRIVE;
  int LB = MIN_ESC_DRIVE;
#endif
int flight_controller::mod2 = 0;

flight_controller::flight_controller() {
   // empty constructor for now
}

void flight_controller::flight_controller_begin()
  {
    //testing motors
    motors[0].set_pin(LF_ESC_PIN);
    motors[1].set_pin(LR_ESC_PIN);
    motors[2].set_pin(RF_ESC_PIN);
    motors[3].set_pin(RR_ESC_PIN);
    #if DEBUG
      Serial.println("SETUP ESCs \n");
    #endif
    
    indicate_blink (10, 500, 500);// Blink for 5 seconds
    indicate_off();

    #if DEBUG
      Serial.print("Testing motors LF -> LR -> RF -> RR .....\n");
    #endif      

    for(int i = 0; i<4; i++)
      motors[i].test(i);
      
    #if DEBUG
      Serial.println("Motor driver setup");
    #endif
      // We do not wait here, but at the main loop to make sure there is no load delay
    //motors tested

    lastCtlLoopTime = 0;
  }

void flight_controller::get_flight_targets() {
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

void flight_controller::reset2throttle(){
  for(int i = 0; i<4; i++)
    motors[i].set_model_drive(motor_throttle);
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

void flight_controller::get_gyro_values(){
  Xangle = gyro.Xangle;
  Yangle = gyro.Yangle;
  Zangle = gyro.Zangle;
}
void flight_controller::calculate_set_model_drives(){
  int motor_avg = 0; //local variable
  for(int i = 0; i< 4; i++){
    motor_avg += motors[i].get_model_drive(); 
  }
  motor_avg = motor_avg <<2;

  //int threshold = 4 * (x_tilt + y_tilt + z_tilt); // i'll add this once I add z
  //if(threshold>10) threshold = 10;


  //if the average speed of all the motor is too dfferent from the throttle speed reset the motor speeds to throttle speed.
  if (!( (motor_throttle-10) <= motor_avg || motor_avg <= (motor_throttle+10) ))
    reset2throttle();
    
  //if it is toppling over
  if (abs(x_tilt) > MAX_TILT_ANGLE || abs(y_tilt) > MAX_TILT_ANGLE || abs(z_tilt) > MAX_TILT_ANGLE) {
    reset2float();
  }

  mod2 = (mod2 + 1)%2; //for chosing which pair of motors to control

  if(Xangle>x_tilt) //I need to bend backwards
  {
    if(mod2){
      ++motors[LF_MOTOR];
      ++motors[RF_MOTOR];
    }
    else{
      --motors[LR_MOTOR];
      --motors[RR_MOTOR];
    }
    
  }  
  else if(Xangle < x_tilt) //I need to bend forwards (bend forward is -x)
  {
    if(mod2){
      ++motors[LF_MOTOR];
      ++motors[RF_MOTOR];
    }
    else{
      --motors[LR_MOTOR];
      --motors[RR_MOTOR];
    }
  }
  
  if(Yangle>y_tilt) //I need to bend backwards
  {
    if(mod2){
      ++motors[LF_MOTOR];
      ++motors[LR_MOTOR];
    }
    else{
      --motors[RF_MOTOR];
      --motors[RR_MOTOR];
    }
  }  
  else if(Yangle < y_tilt) //I need to bend forwards
  {
    if(mod2){
      ++motors[LF_MOTOR];
      ++motors[LR_MOTOR];
    }
    else{
      --motors[RF_MOTOR];
      --motors[RR_MOTOR];
    }
  }

  /*if(Zangle>z_tilt) //I need to bend backwards
  {
    if(mod2){
      motors[LF_MOTOR]++;
      motors[RF_MOTOR]++;
    }
    else{
      motors[LR_MOTOR]--;
      motors[RR_MOTOR]--;
    }
  }  
  else if(Zangle < z_tilt) //I need to bend forwards
  {
    if(mod2){
      motors[LF_MOTOR]++;
      motors[RF_MOTOR]++;
    }
    else{
      motors[LR_MOTOR]--;
      motors[RR_MOTOR]--;
    }
  }*/    
    
}

void flight_controller::drive(){
  for(int i = 0; i<4; i++)
    motors[i].drive();
}

void flight_controller::print_vals(){
  Serial.print("Motor LF speed: ");Serial.println(motors[LF_MOTOR].get_model_drive());
  Serial.print("\t RF speed: ");Serial.println(motors[RF_MOTOR].get_model_drive());
  Serial.print("\t LR speed: ");Serial.println(motors[LR_MOTOR].get_model_drive());
  Serial.print("\t LF speed: ");Serial.println(motors[RR_MOTOR].get_model_drive());

  gyro.print_vals();
}

void flight_controller::run_flight_controller() {
/*set a max angle above which we can't tilt the drone. 
when we move it and it starts tilting if the tilt angle 
is more than the max value then we set the drone to float 
*/
  gyro.check_mpu();           //update gyro
  get_gyro_values();          //update drone tilt
  get_flight_targets(); //update from rc
  gyro.check_mpu();           //update gyro
  calculate_set_model_drives();         //set the drive values based on flight targets
  gyro.check_mpu();
#if not DEBUG
    drive();
#endif
}
