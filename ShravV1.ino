#include"Arduino.h"
#include "flight_controller.h"
#include<Servo.h>
#include<Wire.h>
#include "common_utils.h"
#include "rc_comm.cpp"
#include "gyro.h"

#define I2C_SCL      A4
#define I2C_SCLK     A5
#define ACCELEROMETER_COEFF 0.02
#define GYROSCOPE_COEFF 0.98

#define ENABLE_ESC 0 //actually drive motors
#define ENABLE_RC 1
#define ENABLE_MPU 1 // Enables reading

flight_controller fc;

int count = 0;

void setup() {
  setup_indicator();
  indicate_blink (4, 500, 500);
  Serial.begin(9600);
  Serial.println("QUADCOPTER SETUP \n");
  #if ENABLE_ESC
    setup_ESCs();     // setup ESC connections
  #endif

  #if ENABLE_RC
    calibrate_rc();       // Setup RC connection
  #endif
  
  // Setup I2C 
  Wire.begin();
  indicate_blink(2, 250, 250);

}

void loop(){
  count++;
  fc.run_flight_controller();
  if(!(count = count%10)){
    update_rc_data();    
    RC_print_vals();
  }  
  fc.calculate_flight_targets();
  

}
