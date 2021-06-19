#include "Arduino.h"
#include "Servo.h"
#include "flight_controller.h"
#include "common_utils.h"
#include "rc_comm.h"
#include "rc_comm.cpp"

#if _CONSTRAINED
  int UB = MAX_ESC_DRIVE;
  int LB = MIN_ESC_DRIVE;
#endif

class motor{
  private:
    unsigned int esc_drive_min;   // minimum range values for escs
    unsigned int esc_drive_max;   // maximum range values for escs
    unsigned int esc_drive_float; // value for esc where it will equally contribute to float
    unsigned int esc_drive_reg;   // Current value of esc drive 

    unsigned int model_drive;
    unsigned int model_min_val;
    unsigned int model_max_val;
    static unsigned int model_float;

  public:
    Servo ESC;

    motor(){
      esc_drive_min = MIN_ESC_DRIVE;
      esc_drive_max = MAX_ESC_DRIVE; 
      esc_drive_float = 50;

      model_drive = 0;
      model_min_val = MODEL_MIN;
      model_max_val  = 100;
    }
    motor( int pin_no){
      esc_drive_min = MIN_ESC_DRIVE;
      esc_drive_max = MAX_ESC_DRIVE;
      esc_drive_float = 50;

      model_drive = 0;
      model_min_val = MODEL_MIN;
      model_max_val  = MODEL_MAX;

      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    motor(int pin_no, int min_, int max_, int float_){
      esc_drive_min = min_;
      esc_drive_max = max_;
      esc_drive_float = float_;

      model_drive = 0;
      model_min_val = MODEL_MIN;
      model_max_val  = MODEL_MAX;

      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    void set_pin(int pin_no){
      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    void operator++(){
      if(model_drive < model_max_val) 
        model_drive++;
    }
    void operator+=(const int val){
      if(model_drive + val <= model_max_val)
        model_drive += val;
    }
    void operator-=(const int val){
      if(model_drive - val >= model_min_val) 
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
      esc_drive_reg = map(model_drive, model_min_val, model_max_val, esc_drive_min, esc_drive_max);
      #if _CONSTRAINED
        constrain (esc_drive_reg, LB, UB);
      #endif
      ESC.write(esc_drive_reg);
    }

};


class flight_controller{
  // The model will map the value from 0 to 100 (kind of %)
  int   model_float_value;
  float lastCtlLoopTime = 0;
  int lb, ub; //for speed of motors
  float xv_target, yv_target, zv_target ;          // Calculated value mostly, to help driving ESCs
  float rotor_mean;
  float x_tilt, y_tilt, z_tilt;

  motor motors[4]; //the individual motors;
  /*
    motors[0] = &LF_ESC;
    motors[1] = &LR_ESC;
    motors[2] = &RF_ESC;
    motors[3] = &RR_ESC;
  */

  public:
  flight_controller()
  {
    //testing motors
    motors[0].set_pin(LF_ESC_PIN);
    motors[1].set_pin(LR_ESC_PIN);
    motors[2].set_pin(RF_ESC_PIN);
    motors[3].set_pin(RR_ESC_PIN);
    #if DEBUG
      Serial.println("SETUP ESCs \n");
    #endif
    
    indicate_blink (5, 500, 500);// Blink for 5 seconds
    indicate_off();

    #if DEBUG
      Serial.print("Testing motors LF -> LR -> RF -> RR .....\n");
    #endif      

    for(int i = 0; i<4; i++)
      motors[i].test();
      
    #if DEBUG
      Serial.println("Motor driver setup");
    #endif
      // We do not wait here, but at the main loop to make sure there is no load delay
    //motors tested

    model_float_value = 25 ;

    lastCtlLoopTime = 0;
    xv_target = 0, yv_target = 0, zv_target = 0;
  }


  void calculate_flight_targets() {
    //throttle value decides mean speed
    rotor_mean = map(rc_throttle(), MIN_RC_THROTTLE, MAX_RC_THROTTLE, MODEL_MIN, MODEL_MAX);

    x_tilt = map(rc_elevator(), MIN_RC_ELEVATOR, MAX_RC_ELEVATOR, -100, 100);//forward -X
    y_tilt = map(rc_aileron(), MIN_RC_ROLL, MAX_RC_ROLL, -100, 100);//left -Y
    z_tilt = map(rc_rudder(), MIN_RC_RUDDER, MAX_RC_THROTTLE, -50, 50);//clockwise -Z
  }

  void reset_angles(){
    for(i = 0; i<4; i++){
      motors[i].float()
    }
  }
/********************************************************************/
  void heartbeat() {
    if (!is_rc_alive()) {
      // In failsafe mode, so drive to a lower and slow falling mode.
      for (int i = 0; i < 4; i++) {
        rotor[i].set_model_drive( model_float_value);  //25
      }
      indicate_off();
      return;
    } 
    // Seems to be OK, so let's continue the normal way
    indicate_glow();
  }

  void set_drives(){
    
  }

  /******************************************************************/
  void set_esc_values() {

    // Set the drive in the motor controller, with mapping enabled
    for (int i=0; i < 4; i++)
        map_to_esc_drive(i, model_drive[i], model_min[i], model_max[i]);
  }
  
  // This function assumes the input data is already ready - RC/internal touch points for direction, MPU / Compass for stability/feedback
  
  void run_flight_controller() {
/*set a max angle above which we can't tilt the drone. 
when we move it and it starts tilting if the tilt angle 
is more than the max value then we set the drone to float 
*/
    calculate_flight_targets();
    drive();
  }
}fc;
