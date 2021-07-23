#include"Arduino.h"
#include<Servo.h>
#include<Wire.h>
#include "common_utils.h"
#include "rc_comm.h"
#include "gyro.h"
#include "flight_controller.h"

#define I2C_SCL      A4
#define I2C_SCLK     A5
#define ACCELEROMETER_COEFF 0.02
#define GYROSCOPE_COEFF 0.98

#define ENABLE_ESC 0 //actually drive motors
#define ENABLE_RC 1

flight_controller fc;

int count = 0;

void setup() {
  setup_indicator();
  indicate_blink (4, 500, 500);
  Serial.begin(9600);
  Serial.println("QUADCOPTER SETUP \n");

  // Setup I2C 
  Wire.begin();

  #if ENABLE_RC
    calibrate_rc();       // Setup RC connection
  #endif
  
  fc.flight_controller_begin();
  indicate_blink(2, 250, 250);

}

void loop(){
  count++;
  fc.run_flight_controller();
  if(count%10 == 0){
    update_rc_data();   
  }  
  if(!(count = count%30)){
    RC_print_vals();
    fc.print_vals();
  }
  

}
