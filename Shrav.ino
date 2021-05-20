#include"Arduino.h"
#include<Servo.h>
#include<Kalman.h>
#include "rc_comm.cpp"

#define I2C_SCL      A4
#define I2C_SCLK     A5
#define ACCELEROMETER_COEFF 0.02
#define GYROSCOPE_COEFF 0.98

#define ENABLE_ESC 0
#define ENABLE_RC 1
#define ENABLE_MPU 0


RC_Comms rc;

void setup() {
  setup_indicator();
  indicate_blink (4, 500, 500);
  Serial.begin(9600);
  Serial.println("QUADCOPTER SETUP \n");
  #if ENABLE_ESC
    setup_ESCs();     // setup ESC connections
  #endif

  #if ENABLE_RC
    rc.calibrate();       // Setup RC connection
  #endif
  
  // Setup I2C 
  Wire.begin();
  indicate_blink(2, 250, 250);

  #if ENABLE_MPU
    // setup MPU6050 sensor - gyro
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    indicate_blink(3, 250, 250);
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:
  static long int micros_after_last_rc_read = RC_READ_INTERVAL + 10;  // Force the rc to be read the first time it enters
  indicate_off();

  unsigned long int loop_timer = micros();
  unsigned long int loop_exit_target_time = loop_timer + TRACK_LOOP_DELAY;  // at what time do you want it to exit the loop function
  // Make sure it did not loop back
  
#if ENABLE_RC
  if (micros_after_last_rc_read > RC_READ_INTERVAL) {
    if (update_rc_data()) // RC updated so reset the counter
      micros_after_last_rc_read = 0;

    else // No update in RC though we have asked for it
      micros_after_last_rc_read += TRACK_LOOP_DELAY; 

  } 
  else {
    micros_after_last_rc_read += TRACK_LOOP_DELAY;
  }
#endif

#if ENABLE_MPU
  // Get the current reading - already filtered based on last readings prediction
  get_filtered_mpu6050();

  // instructs the gyro to calculate current values
  estimate_current_angles();
#endif

  // Based on the gyro data and RC data, calculate the model drive - does not really drives the motor 
  run_flight_controller();

#if ENABLE_ESC
  // Drive actual motors, by converting model values to actual based on the HW attached
  esc_drive_within_limit();
#endif

  // Wait for some time - better use library timeout to save power
  unsigned long int remaining_time = loop_exit_target_time - micros();
  if (remaining_time < TRACK_LOOP_DELAY) 
    delay(remaining_time);

#if DEBUG
  Serial.println("");
#endif

}
