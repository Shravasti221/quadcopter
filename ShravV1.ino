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

  Serial.begin(9600);
  // Setup I2C 
  Wire.begin();
  delay(1000);
  Serial.println("QUADCOPTER SETUP ... \n");
  setup_indicator();
  indicate_blink (4, 250, 250);

  #if ENABLE_RC
    setup_rc();       // Setup RC connection
  #endif

  indicate_blink(10, 500, 500);
  // fc.flight_controller_begin();
  indicate_blink(10, 250, 250);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
  
  count++;
  Serial.print("Loop count "); Serial.println(count);
  // indicate_blink (2, 250, 250);
  // fc.run_flight_controller();
  if (count%10 == 0){
    update_rc_data();   
  }  
  if (!(count = count%30)){
    RC_print_vals();
    // fc.print_vals();
  }
  delay(1000);

}
