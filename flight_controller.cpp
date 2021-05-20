/* */
#include "Arduino.h"
#include "Servo.h"
#include "common_utils.h"
#include "motor_driver.h"

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
    motor( int pin_no){
      esc_drive_min = 1;
      esc_drive_max = 120;
      esc_drive_float = 50;

      model_drive = 0;
      model_min_val = 0;
      model_max_val  = 100;

      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
    }

    motor(int pin_no, int min_, int max_, int float_){
      esc_drive_min = min_;
      esc_drive_max = max_;
      esc_drive_float = float_;

      model_drive = 0;
      model_min_val = 0;
      model_max_val  = 100;

      ESC.attach(pin_no, esc_drive_min, esc_drive_max);
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

    //setters and getters;
    void set_to_float(){
      esc_drive_reg = esc_drive_float;
    }

    void set_drive(int drive_){
      esc_drive_reg = drive_;
    }

    void map_set_drive(int drive_, int min_drive, int max_drive){
      esc_drive_reg = map(drive, min_drive, max_drive, esc_drive_min, esc_drive_max);
    }

    unsigned int get_drive(){
      return esc_drive_reg;
    }

    void set_model_drive(int drive_){
      model_drive = drive_;
    }

    int get_model_drive(){
      return model_drive;
    }

    int set_model_drive_limits(int min_val, int max_val, int float_val){
      esc_drive_min = min_val;
      esc_drive_max = max_val;
      esc_drive_float = float_val;
    }

    int set_esc_drive_limits(int min_val, int max_val, int float_val){
      model_float = float_val;
      model_max_val = max_val;
      model_min_val = min_val;
    }

} rotor[4];


// ****************  ESC code   ******************
// create servo object to control the ESC
class motor_driver{
  private: 
    motor motors[4]; //the individual motors;
    /*
      motors[0] = &LF_ESC;
      motors[1] = &LR_ESC;
      motors[2] = &RF_ESC;
      motors[3] = &RR_ESC;
    */

  public:
    motor_driver():motors[0](LF_ESC_PIN), motors[1](LR_ESC_PIN), motors[2](RF_ESC_PIN), motors[3](RR_ESC_PIN)
    {
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
    }

    void esc_set_drive_to_float(int motor_no) {
      motors[motor_no].set_to_float();
    }

    void esc_set_drive(int motor_no, int drive) {
      motors[motor_no].set_drive(drive);
    }

    //converts drive input from RC to BDC speed for drone
    void map_to_esc_drive(int motor, int drive, int min_drive, int max_drive) {
      esc_drive_reg[motor].map_set_drive(drive, min_drive, max_drive);
    }

    void esc_drive_within_limit() {
      //SAFE MODE ENABLED
      // Make sure the values are within range from each other
      for(int i = 0; i<4; sum += motors[i].get_drive(), i++);
      unsigned int average = sum >>2
      Serial.print("The average speed of all the motors is ");
      Serial.println(average);
      unsigned int min_limit = average - 20;
      unsigned int max_limit = average + 20;
      
      if (min_limit > max_limit) min_limit = 0;
      
      // limit the speed withing a limit of average
      for (int i=0; i < 4; i++) {
        motors[i].set_drive(constrain (esc_drive_reg[i], min_limit, max_limit));
      }


    }

    void esc_drive_no_limit() {} // Normal mode driving, no clipping of PWM
};

// The model will be set to as follows
// 0 - not at all rotating
// 25 - Just enough to float
// 100 - to the maximum rotation
// This will allow us to have some headroom to slow down in one side to fall, while good enough 
// Resolution for flying.

class flight_controller{
  // The model will map the value from 0 to 100 (kind of %)
  int   model_float_value;
  float lastCtlLoopTime = 0;

  float roll_error, pitch_error, yaw_error;       // basically calculated errors, saved for derivative calculation

  float roll_target, pitch_target, yaw_target1;   // Target value based on RC, in degrees
  float x_target, y_target, z_target;
  float xv_target, yv_target, zv_target ;          // Calculated value mostly, to help driving ESCs

  public: 
  flight_controller()
  {

    model_float_value = 25 ;

    lastCtlLoopTime = 0;
    roll_error = 0, pitch_error = 0, yaw_error = 0; 
    roll_target = 0, pitch_target = 0, yaw_target1 = 0;
    rollAdj, pitchAdj, yawAdj;
    x_target = 0, y_target = 0, z_target = 0;
    xv_target = 0, yv_target = 0, zv_target = 0;
  }

  int get_model_drive(int motor_id) {
    return model_drive[motor_id];
  }

  int get_model_drive_min(int motor_id) {
    return model_min[motor_id];
  }

  int get_model_drive_max(int motor_id) {
    return model_max[motor_id];
  }


  /********** Stabilizer - crude and first try *****************/

  void calculate_flight_targets() {

    // To start with, just set to be stable
    x_target = 0.0;
    y_target = 0.0;
    z_target = 0.0;

    // These are between -50.0 to +50.0, describing front and back movement
    xv_target = map(rc_throttle(), MIN_RC_THROTTLE, MAX_RC_THROTTLE, 0, 100);
    yv_target = map(rc_throttle(), MIN_RC_THROTTLE, MAX_RC_THROTTLE, 0, 100);
    // zv_target = map(rc_elevator(), MIN_RC_ELEVATOR, MAX_RC_ELEVATOR, 10, 90);
    zv_target = map(rc_throttle(), MIN_RC_THROTTLE, MAX_RC_THROTTLE, 0, 100);
    
    // We could make this in roll/pitch/yaw too.
  }

/********************************************************************/
  void set_drive_target() {

    int mean_drive;
    int adj;

    // The crux of the flight controller - it should get the data from
    // MPU, RC controller, and determine the thrust required for each ESC regs
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

    // First version - only up and down - get the average 
    mean_drive = zv_target;
    for (int i = 0; i < 4; i++) {
      model_drive[i] = mean_drive;
    }
/*
  #if ADJUST_FROM_MPU
    // Positive error means the actual value is less than target
    // roll_error is positive - it needs to be rotated towards right, so pump up left side and pump down right
    adj = constrain((int)rollAdj, -30, 30) >> 1;
    model_drive[LF_MOTOR] = constrain(model_drive[LF_MOTOR] + adj, model_min[LF_MOTOR], model_max[LF_MOTOR]);
    model_drive[LR_MOTOR] = constrain(model_drive[LR_MOTOR] + adj, model_min[LR_MOTOR], model_max[LR_MOTOR]);
    model_drive[RF_MOTOR] = constrain(model_drive[RF_MOTOR] - adj, model_min[RF_MOTOR], model_max[RF_MOTOR]);
    model_drive[RR_MOTOR] = constrain(model_drive[RR_MOTOR] - adj, model_min[RR_MOTOR], model_max[RR_MOTOR]);  
    
    // pitch_error is positive - nose is down than what it should be, so pump up the front, and down the rear
    adj = constrain((int)pitchAdj, -30, 30) >> 1;
    model_drive[LF_MOTOR] = constrain(model_drive[LF_MOTOR] + adj, model_min[LF_MOTOR], model_max[LF_MOTOR]);
    model_drive[RF_MOTOR] = constrain(model_drive[RF_MOTOR] + adj, model_min[RF_MOTOR], model_max[RF_MOTOR]);
    model_drive[LR_MOTOR] = constrain(model_drive[LR_MOTOR] - adj, model_min[LR_MOTOR], model_max[LR_MOTOR]);
    model_drive[RR_MOTOR] = constrain(model_drive[RR_MOTOR] - adj, model_min[RR_MOTOR], model_max[RR_MOTOR]);  
  #endif  
*/
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
    set_drive_target();

    set_esc_values();
  }
}fc;
